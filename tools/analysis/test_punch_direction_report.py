"""Tests of punch_direction_report.py: synthetic recordings with known angles written with the reference encoder of
test_pose_capture_to_csv.py (stored blocks), a synthetic session log, synthetic clips, the verdict logic, and a regression on a small
excerpt of a real recording (fixtures/punch_direction_*; Male V punching from his knees).

  python -I test_punch_direction_report.py

The fixture is cut from a real run with  python -I test_punch_direction_report.py --carve LOG POSE CLIPS_DIR OUT_DIR  (see carve_fixture).
"""
import contextlib
import io
import json
import os
import pathlib
import sys
import tempfile
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import punch_direction_report as pdr  # noqa: E402
import test_pose_capture_to_csv as tp  # noqa: E402

FIXTURES = HERE / 'fixtures'
FPS = 60
START = 51.0                    # frame 0 = 23:39:51.000 (local); the header holds UTC 22:39:51
NAME = 'utpp-pose-2026-11-12-23-39-51.utpppose'

NAMES = ['Root', 'Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head',
         'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand',
         'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'RightUpLeg', 'RightLeg', 'RightFoot']
PARENT = {'Root': -1, 'Hips': 'Root', 'Spine': 'Hips', 'Spine1': 'Spine', 'Spine2': 'Spine1', 'Spine3': 'Spine2', 'Neck': 'Spine3',
          'Neck1': 'Neck', 'Head': 'Neck1', 'RightShoulder': 'Spine3', 'RightArm': 'RightShoulder', 'RightForeArm': 'RightArm',
          'RightHand': 'RightForeArm', 'LeftShoulder': 'Spine3', 'LeftArm': 'LeftShoulder', 'LeftForeArm': 'LeftArm',
          'LeftHand': 'LeftForeArm', 'LeftUpLeg': 'Hips', 'LeftLeg': 'LeftUpLeg', 'LeftFoot': 'LeftLeg', 'RightUpLeg': 'Hips',
          'RightLeg': 'RightUpLeg', 'RightFoot': 'RightLeg'}
GEOM = {'Root': (0, 0, 0), 'Hips': (0, 0, 96), 'Spine': (0, 0, 8), 'Spine1': (0, 0, 8), 'Spine2': (0, 0, 8), 'Spine3': (0, 0, 8),
        'Neck': (0, 0, 15), 'Neck1': (0, 0, 8), 'Head': (0, 0, 8),
        'RightShoulder': (6, 0, 12), 'RightArm': (12, 0, 0), 'RightForeArm': (28, 0, 0), 'RightHand': (26, 0, 0),
        'LeftShoulder': (-6, 0, 12), 'LeftArm': (-12, 0, 0), 'LeftForeArm': (-28, 0, 0), 'LeftHand': (-26, 0, 0),
        'LeftUpLeg': (-9, 0, -2), 'LeftLeg': (0, 0, -46), 'LeftFoot': (0, 0, -48),
        'RightUpLeg': (9, 0, -2), 'RightLeg': (0, 0, -46), 'RightFoot': (0, 0, -48)}
PAR_IX = [-1 if PARENT[n] == -1 else NAMES.index(PARENT[n]) for n in NAMES]


# ---------------------------------------------------------------------------------------------------------------------- synthetic geometry
def q_axis(axis, deg):
    a = np.radians(deg) / 2.0
    v = np.array(axis, float)
    v /= np.linalg.norm(v)
    return np.concatenate([v * np.sin(a), [np.cos(a)]])


def q_from_to(a, b):
    a, b = np.array(a, float), np.array(b, float)
    q = np.concatenate([np.cross(a, b), [1.0 + float(np.dot(a, b))]])
    return q / np.linalg.norm(q)


def pulse(t, centre, half=0.1):
    return max(0.0, 1.0 - abs(t - centre) / half)


def arm_locals(side, elev_deg, phi_deg):
    """(RightArm / LeftArm local quaternion, ForeArm local quaternion): the upper arm points forward at the elevation, the elbow bends by phi."""
    axis = (1, 0, 0) if side == 'Right' else (-1, 0, 0)
    d = (0, np.cos(np.radians(elev_deg)), np.sin(np.radians(elev_deg)))
    return q_from_to(axis, d), q_axis((0, 0, 1), phi_deg)


class Scene:
    """One synthetic recording: arm pose from the 'clip' (elevation clip_elev) in both stages, the trunk tilted back by alpha0 / alpha1
    degrees (about X) while the melee window is on, hips height, punches of the right arm at the given times."""

    def __init__(self, right=(2.0,), left=(), alpha0=0.0, alpha1=None, hips_z=96.0, clip_elev=5.0, window=(1.0, 5.0), seam=None,
                 pop_frame=None, frames=360):
        self.right, self.left = tuple(right), tuple(left)
        self.alpha0, self.alpha1 = alpha0, alpha0 if alpha1 is None else alpha1
        self.hips_z, self.clip_elev, self.window, self.seam, self.pop_frame, self.frames = hips_z, clip_elev, window, seam, pop_frame, frames

    def locals_at(self, i, stage):
        """name -> (local position cm, local quaternion) of frame i."""
        t = i / FPS
        out = {n: (np.array(GEOM[n], float), np.array([0, 0, 0, 1.0])) for n in NAMES}
        out['Hips'] = (np.array([0, 0, self.hips_z]), out['Hips'][1])
        inside = self.window[0] <= t <= self.window[1]
        alpha = (self.alpha0 if stage == 0 else self.alpha1) if inside else 0.0
        out['Spine'] = (out['Spine'][0], q_axis((1, 0, 0), alpha))
        for side, times in (('Right', self.right), ('Left', self.left)):
            p = max([pulse(t, c) for c in times] + [0.0])
            qa, qf = arm_locals(side, self.clip_elev, 110.0 * (1.0 - p))
            if self.seam is not None and i == self.seam:
                qa, qf = np.array([0, 0, 0, 1.0]), np.array([0, 0, 0, 1.0])
            if self.pop_frame is not None and i == self.pop_frame and side == 'Right':
                qa = q_axis((0, 1, 0), 120.0)
            out[side + 'Arm'] = (out[side + 'Arm'][0], qa)
            out[side + 'ForeArm'] = (out[side + 'ForeArm'][0], qf)
        return out

    def values(self):
        v = np.zeros((self.frames, len(NAMES), 24), dtype=np.float32)
        for i in range(self.frames):
            for st in (0, 1):
                loc = self.locals_at(i, st)
                for b, n in enumerate(NAMES):
                    v[i, b, st * 12:st * 12 + 3] = loc[n][0] / 100.0
                    v[i, b, st * 12 + 4:st * 12 + 8] = loc[n][1]
                    v[i, b, st * 12 + 8:st * 12 + 12] = 1.0
        return v


class Rec:
    """Slice-like object for tp.encode_header / tp.encode_block."""

    def __init__(self, names, parents, values, ticks, utc=(2026, 11, 12, 22, 39, 51, 0), qpc_hz=1000, arm_ticks=0):
        n = len(names)
        self.names = names
        self.game_index = list(range(n))
        self.bones = [(i, 0x1000 + i, parents[i]) for i in range(n)]
        self.qpc_hz, self.arm_ticks, self.utc = qpc_hz, arm_ticks, utc
        self.max_frames, self.rig, self.rig_count = 100000, 0xABC, n
        self.frames = values.shape[0]
        self.values = values
        self.meta = np.zeros((self.frames, 4), dtype=np.int64)
        self.meta[:, 0] = ticks
        self.meta[:, 1] = 7
        self.meta[:, 2] = 3
        self.meta[:, 3] = 11


def write_pose(path, scene):
    ticks = np.round(np.arange(scene.frames) * 1000.0 / FPS).astype(np.int64)
    rec = Rec(NAMES, PAR_IX, scene.values(), ticks)
    data = tp.encode_header(rec, 0, 0) + tp.encode_block(rec, 0, 0, scene.frames, 0) + tp.encode_trailer((scene.frames, 0))
    tp.write_file(path, data)


def write_clips(folder, elev=5.0, with_distractor=True):
    """combo_attack_01: the right punch pulse (21 frames at the game rate, impact 10) built from the same locals as the scene; block: another pose."""
    folder = pathlib.Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    clips = {'combo_attack_01': (elev, 10, True)}
    if with_distractor:
        clips['block'] = (60.0, 3, False)
    for name, (e, impact, punch) in clips.items():
        frames = 21
        scene = Scene(right=(), clip_elev=e)
        P = np.zeros((frames, len(NAMES), 2, 3))
        Q = np.zeros((frames, len(NAMES), 2, 4))
        for f in range(frames):
            loc = scene.locals_at(0, 0)
            p = max(0.0, 1.0 - abs(f - 10) / 6.0) if punch else 0.0
            qa, qf = arm_locals('Right', e, 110.0 * (1.0 - p))
            loc['RightArm'] = (loc['RightArm'][0], qa)
            loc['RightForeArm'] = (loc['RightForeArm'][0], qf)
            for b, n in enumerate(NAMES):
                P[f, b, :, :] = loc[n][0]
                Q[f, b, :, :] = loc[n][1]
        pd = pdr.PoseData(NAMES, PAR_IX, np.arange(frames, dtype=float), P, Q)
        wp, wr = pd.world(0)
        W = np.zeros((frames, len(NAMES), 4, 4))
        W[:, :, :3, :3] = wr
        W[:, :, :3, 3] = wp / 100.0
        W[:, :, 3, 3] = 1.0
        lr = Q[:, [NAMES.index(b) for b in pdr.MATCH_BONES], 0]
        np.savez(folder / ('pma_fists__%s.npc1.npz' % name), names=np.array(NAMES), W=W, names152=np.array(pdr.MATCH_BONES), lr=lr)
        (folder / ('pma_fists__%s.npc1.info.json' % name)).write_text(json.dumps({'impact_built': [impact, impact], 'side': 'Right',
                                                                                  'note': 'rear-hand cross'}), encoding='utf-8')


def write_log(path, window=(52.0, 56.0), full=(52.0, 55.0), body='man', aim_src='lock', extra=''):
    def ts(s):
        return '[2026-11-12 23:39:%06.3f] [info    ] [  1] [UTPP] UTPP ' % s
    L = [ts(51.2) + 'mm.motionDriver load start #1 body=%s why=first from=none' % body,
         ts(51.3) + 'pose capture: started file=red4ext\\logs\\' + NAME,
         ts(window[0] - 1.0) + 'MMMASK on reasons=0x2 (was 0x0) gate=0x110 covered=0x110 loco=0 upperBody=6 melee=0 exit=0.00 w=1.00 mask=0.00 full=0.00 fullWant=0 fullWhy=flag',
         ts(window[0] - 0.9) + 'MMMASK off reasons=0x0 (was 0x2) gate=0x0 covered=0x0 loco=1 upperBody=0 melee=0 exit=0.00 w=1.00 mask=1.00 full=0.00 fullWant=0 fullWhy=flag',
         ts(window[0]) + 'MMMASK on reasons=0x1 (was 0x0) gate=0x400 covered=0x400 loco=1 upperBody=0 melee=1 exit=0.00 w=1.00 mask=0.00 full=0.00 fullWant=0 fullWhy=melee',
         ts(full[0]) + 'MMMELEEFULL on why=on full=0.00 speed=0.00 loco=1',
         ts(window[0] + 0.5) + 'MELEEAIM src=cam tiltW=1.00 tgtPitch=1.0 tgtYaw=0.0 rawPitch=1.0 rawYaw=0.0 pitch=1.0 yaw=0.0 flags=0x0 maxUp=20',
         ts(53.0) + 'MELEEAIM src=%s tiltW=1.00 tgtPitch=4.0 tgtYaw=-2.0 rawPitch=4.0 rawYaw=-2.0 pitch=3.5 yaw=-1.5 flags=0x0 maxUp=20' % aim_src,
         ts(full[1]) + 'MMMELEEFULL off why=state full=1.00 speed=0.00 loco=1',
         ts(window[1]) + 'MMMASK off reasons=0x0 (was 0x1) gate=0x0 covered=0x0 loco=1 upperBody=0 melee=0 exit=0.00 w=1.00 mask=1.00 full=0.00 fullWant=0 fullWhy=state']
    pathlib.Path(path).write_text('\n'.join(L) + '\n' + extra, encoding='utf-8', newline='\n')


class Workspace:
    def __init__(self, tc):
        self.tmp = tempfile.TemporaryDirectory()
        tc.addCleanup(self.tmp.cleanup)
        self.root = pathlib.Path(self.tmp.name)
        self.game = self.root / 'game'
        (self.game / 'red4ext' / 'logs').mkdir(parents=True)
        self.log = self.root / 'utpp-2026-11-12-23-39-50.log'
        self.clips = self.root / 'clips'

    def build(self, scene, clips=True, **log_kw):
        write_pose(self.game / 'red4ext' / 'logs' / NAME, scene)
        write_log(self.log, **log_kw)
        if clips:
            write_clips(self.clips)
        return self

    def run(self, **kw):
        kw.setdefault('clips_dir', str(self.clips) if self.clips.exists() else str(self.root / 'no_such_clips'))
        return pdr.analyze(str(self.log), None, str(self.game), **kw)


# ---------------------------------------------------------------------------------------------------------------------- tests
class SyntheticTest(unittest.TestCase):
    def test_level_punch_passes(self):
        rep = Workspace(self).build(Scene(right=(2.0, 3.5))).run()
        self.assertEqual(rep['exit_code'], 0, rep.get('notes'))
        self.assertEqual(len(rep['punches']), 2)
        p = rep['punches'][0]
        self.assertEqual(p['verdict'], 'PASS')
        self.assertFalse(p['kneeling'])
        self.assertAlmostEqual(p['diff_vs_clip']['hips_above_root_cm'], 0.0, delta=0.1)
        self.assertEqual(p['side'], 'Right')
        self.assertEqual(p['clip']['name'], 'combo_attack_01')
        self.assertTrue(p['clip']['known'])
        self.assertAlmostEqual(p['game']['stage1']['elevation_deg'], 5.0, delta=0.6)
        self.assertAlmostEqual(p['clip']['pose']['elevation_deg'], 5.0, delta=0.3)
        self.assertAlmostEqual(p['elevation_diff_deg'], 0.0, delta=0.8)
        self.assertAlmostEqual(p['game']['stage1']['azimuth_deg'], 0.0, delta=0.6)
        self.assertAlmostEqual(p['game']['stage1']['hips_above_root_cm'], 96.0, delta=0.1)
        self.assertAlmostEqual(p['game']['stage1']['hips_above_feet_cm'], 96.0, delta=0.1)
        self.assertAlmostEqual(p['game']['stage1']['wrist_above_shoulder_cm'], 54 * np.sin(np.radians(5.0)), delta=0.8)
        self.assertLess(p['game']['stage1']['wrist_above_head_cm'], 0.0)
        self.assertAlmostEqual(p['game']['stage1']['trunk_lean_deg'], 0.0, delta=0.5)
        self.assertAlmostEqual(p['game']['stage1']['head_pitch_deg'], 0.0, delta=0.5)
        self.assertEqual(rep['body'], 'man')
        self.assertTrue(p['full_body_on'])
        self.assertFalse(p['in_blend'])
        self.assertEqual(p['tilt']['source'], 'lock')
        self.assertAlmostEqual(p['tilt']['pitch_deg'], 3.5)
        self.assertAlmostEqual(p['t_since_mask_on_s'], 1.0, delta=0.1)
        self.assertEqual(rep['seam_frames'], [])

    def test_raised_punch_fails(self):
        rep = Workspace(self).build(Scene(alpha0=25.0)).run()
        self.assertEqual(rep['exit_code'], 1)
        p = rep['punches'][0]
        self.assertEqual(p['verdict'], 'FAIL')
        self.assertAlmostEqual(p['game']['stage1']['elevation_deg'], 30.0, delta=1.0)
        self.assertAlmostEqual(p['elevation_diff_deg'], 25.0, delta=1.2)
        al = np.radians(25.0)       # Hips -> Spine3: 8 cm straight up, then 24 cm tilted back by alpha
        self.assertAlmostEqual(p['game']['stage1']['trunk_lean_deg'], float(np.degrees(np.arctan2(-24 * np.sin(al), 8 + 24 * np.cos(al)))), delta=0.5)
        self.assertAlmostEqual(p['game']['stage1']['head_pitch_deg'], 25.0, delta=1.0)
        self.assertTrue(any(f.startswith('elevation') for f in p['fails']))
        self.assertGreater(p['game']['stage1']['wrist_above_head_cm'], 0.0)
        self.assertTrue(any(f.startswith('wrist') for f in p['fails']))

    def test_kneeling_flagged(self):
        rep = Workspace(self).build(Scene(hips_z=36.0)).run()
        p = rep['punches'][0]
        self.assertEqual(rep['exit_code'], 1)
        self.assertAlmostEqual(p['game']['stage1']['hips_above_root_cm'], 36.0, delta=0.1)
        self.assertTrue(p['kneeling'])
        self.assertTrue(any(f.startswith('kneeling') for f in p['fails']))
        self.assertFalse(any(f.startswith('elevation') for f in p['fails']))
        rep = Workspace(self).build(Scene(hips_z=36.0)).run(lim=pdr.Limits(kneel_hips_cm=30.0))
        self.assertEqual(rep['exit_code'], 0)

    def test_clip_unknown_path(self):
        ws = Workspace(self).build(Scene(), clips=False)
        rep = ws.run()
        p = rep['punches'][0]
        self.assertFalse(p['clip']['known'])
        self.assertIn('no clips', p['clip']['reason'])
        self.assertEqual(p['verdict'], 'UNEVALUATED')
        self.assertTrue(p['straight_assumed'])
        self.assertEqual(rep['exit_code'], 2)
        self.assertTrue(any('clip comparison unavailable' in n for n in rep['notes']))
        self.assertAlmostEqual(p['game']['stage1']['elevation_deg'], 5.0, delta=0.6)    # the numbers are still reported
        # a hard fail still counts without a clip
        rep = Workspace(self).build(Scene(alpha0=25.0), clips=False).run()
        self.assertEqual(rep['exit_code'], 1)
        self.assertTrue(rep['punches'][0]['fails'][0].startswith('wrist'))
        rep = Workspace(self).build(Scene(hips_z=36.0), clips=False).run()
        self.assertEqual(rep['exit_code'], 1)

    def test_clip_that_matches_nothing_is_unknown(self):
        ws = Workspace(self).build(Scene(), clips=False)
        write_clips(ws.clips, with_distractor=True)
        for f in ws.clips.glob('*combo_attack_01*'):
            f.unlink()
        rep = ws.run(lim=pdr.Limits(max_clip_rms=1.0))
        p = rep['punches'][0]
        self.assertFalse(p['clip']['known'])
        self.assertIn('above --max-clip-rms', p['clip']['reason'])
        self.assertEqual(rep['exit_code'], 2)

    def test_blend_is_a_warning_stage_1_decides(self):
        # window starts 0.2 s before the punch: stage 1 may not be settled, but it is what the player sees
        for a0, a1, verdict in ((25.0, 0.0, 'PASS'), (0.0, 25.0, 'FAIL')):
            rep = Workspace(self).build(Scene(alpha0=a0, alpha1=a1, window=(1.8, 5.0)), window=(52.8, 56.0), full=(52.8, 55.0)).run()
            p = rep['punches'][0]
            self.assertTrue(p['in_blend'], (a0, a1))
            self.assertEqual(p['verdict'], verdict, (a0, a1))
            self.assertTrue(any('blend' in w for w in p['warnings']))
            self.assertGreater(abs(p['game']['stage0']['elevation_deg'] - p['game']['stage1']['elevation_deg']), 20.0)    # both stages are reported
        rep = Workspace(self).build(Scene(window=(1.8, 5.0)), window=(52.8, 56.0), full=(52.8, 55.0)).run()
        self.assertTrue(rep['punches'][0]['in_blend'])
        self.assertEqual(rep['punches'][0]['verdict'], 'PASS')

    def test_no_punch_and_seam(self):
        rep = Workspace(self).build(Scene(right=())).run()
        self.assertEqual(rep['punches'], [])
        self.assertEqual(rep['exit_code'], 2)
        # a one-frame T pose (both arms fully stretched) is not a punch
        rep = Workspace(self).build(Scene(right=(), seam=150)).run()
        self.assertEqual(rep['punches'], [])
        self.assertEqual(len(rep['seam_frames']), 1)
        rep = Workspace(self).build(Scene(right=(2.0,), seam=150)).run()
        self.assertEqual(len(rep['punches']), 1)

    def test_no_window_no_pose(self):
        ws = Workspace(self).build(Scene())
        ws.log.write_text('[2026-11-12 23:39:52.000] [info    ] [  1] [UTPP] UTPP MMMASK on reasons=0x2 (was 0x0) melee=0 full=0.00\n', encoding='utf-8')
        rep = ws.run()
        self.assertEqual(rep['exit_code'], 2)
        self.assertTrue(any('no melee window' in n for n in rep['notes']))
        ws = Workspace(self).build(Scene())
        (ws.game / 'red4ext' / 'logs' / NAME).unlink()
        rep = ws.run()
        self.assertEqual(rep['exit_code'], 2)
        self.assertTrue(any('no pose recording' in n for n in rep['notes']))

    def test_pop_indicator(self):
        rep = Workspace(self).build(Scene(right=(2.0,), pop_frame=200)).run()
        pop = rep['pop']['per_bone']['RightArm']
        self.assertGreater(pop['max_rotation_jump_deg'], 100.0)
        self.assertEqual(rep['pop']['largest_rotation_jump']['bone'], 'RightArm')
        self.assertEqual(pop['rotation_jump_time'][:8], '23:39:54')      # frame 200 = 3.33 s -> 23:39:54.33
        calm = Workspace(self).build(Scene(right=(2.0,))).run()['pop']['per_bone']['RightArm']
        self.assertLess(calm['max_rotation_jump_deg'], 60.0)
        self.assertIn('combo_attack_01', rep['pop']['clip_reference'])

    def test_run_folder_cli(self):
        ws = Workspace(self).build(Scene(alpha0=25.0))
        run = ws.root / 'run'
        (run / 'logs').mkdir(parents=True)
        (run / 'gametest').mkdir()
        (run / 'logs' / ws.log.name).write_bytes(ws.log.read_bytes())
        (run / 'gametest' / 'summary.json').write_text(json.dumps({'steps': [
            {'scenario': 'melee_punch', 'step': 'punches', 't0': '2026-11-12 23:39:52.500000', 't1': '2026-11-12 23:39:55.500000'}]}))
        out, sheet = ws.root / 'out' / 'r.json', ws.root / 'out' / 'r.png'
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = pdr.main([str(run), '--game-dir', str(ws.game), '--clips-dir', str(ws.clips), '--out', str(out), '--sheet', str(sheet)])
        self.assertEqual(rc, 1)
        self.assertIn('exit code 1 (fail)', buf.getvalue())
        data = json.loads(out.read_text(encoding='utf-8'))
        self.assertEqual(data['punches'][0]['step'], 'melee_punch/punches')
        self.assertNotIn('_draw', data)
        self.assertGreater(sheet.stat().st_size, 1000)
        self.assertTrue(data['limits_of_tool'])
        self.assertNotIn(b'\r\n', out.read_bytes())


class LogTest(unittest.TestCase):
    def test_windows_aim_body_pose_files(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        p = pathlib.Path(tmp.name) / 'u.log'
        L = lambda s, m: '[2026-11-12 23:50:%06.3f] [info    ] [ 1] [UTPP] UTPP %s' % (s, m)
        p.write_text('\n'.join([
            L(1.0, 'mm.motionDriver load start #1 body=woman why=first'),
            L(2.0, 'MMMASK on reasons=0x4 (was 0x0) melee=0 full=0.00'),          # not melee
            L(3.0, 'MMMASK off reasons=0x0 (was 0x4) melee=0 full=0.00'),
            L(4.0, 'MMMASK on reasons=0x1 (was 0x0) melee=1 full=0.00'),
            L(5.0, 'MMMASK on reasons=0x3 (was 0x1) melee=1 full=0.50'),           # re-on without off
            L(6.0, 'MMMASK on reasons=0x2 (was 0x3) melee=0 full=0.00'),           # melee bit dropped -> closes
            L(7.0, 'MMMASK on reasons=0x1 (was 0x2) melee=1 full=0.00'),
            L(9.0, 'MMMASK off reasons=0x0 (was 0x1) melee=0 full=0.00'),
            L(8.0, 'MMMELEEFULL on why=on full=0.00'), L(8.5, 'MMMELEEFULL off why=state full=1.00'),
            L(4.5, 'MELEEAIM src=hit tiltW=1.00 tgtPitch=2.0 tgtYaw=1.0 rawPitch=2.0 rawYaw=1.0 pitch=1.9 yaw=0.9 flags=0x0'),
            L(4.6, 'pose capture: started file=red4ext\\logs\\utpp-pose-2026-11-12-23-49-59.utpppose'),
            L(20.0, 'mm.motionDriver load start #2 body=man why=swap'), L(21.0, 'MMMASK on reasons=0x1 (was 0x0) melee=1 full=0.00')]) + '\n',
            encoding='utf-8')
        info = pdr.parse_log(str(p))
        base = pdr.stamp_epoch('2026-11-12', '23:50:00')
        self.assertEqual([(round(a - base), round(b - base)) for a, b in info.melee_windows], [(4, 6), (7, 9), (21, 21)])
        self.assertEqual([(round(a - base, 1), round(b - base, 1)) for a, b in info.full_windows], [(8.0, 8.5)])
        self.assertEqual(pdr.body_before(info, base + 4), 'woman')
        self.assertEqual(pdr.body_before(info, base + 30), 'man')
        self.assertEqual(pdr.body_before(info, base + 4, override='man'), 'man')
        a = pdr.nearest_aim(info, base + 4.6, 0.3)
        self.assertEqual((a['src'], a['pitch'] if 'pitch' in a else a['applied_pitch']), ('hit', 1.9))
        self.assertIsNone(pdr.nearest_aim(info, base + 6.0, 0.3))
        self.assertEqual(pdr.full_weight_at(info, base + 5.5), 0.5)
        files = pdr.resolve_pose_files(info, 'G', base + 4, base + 9)
        self.assertEqual([os.path.basename(f) for f in files], ['utpp-pose-2026-11-12-23-49-59.utpppose'])


class VerdictTest(unittest.TestCase):
    def punch(self, elev=5.0, clip_elev=5.0, hips=96.0, wrist_head=-10.0, clip_wrist_head=-10.0, known=True, blend=False, straight=True, reached=True):
        g = dict(elevation_deg=elev, hips_above_root_cm=hips, wrist_above_head_cm=wrist_head)
        p = dict(game=dict(stage0=dict(g), stage1=dict(g)), in_blend=blend, straight=straight, impact_reached=reached,
                 clip=dict(known=known, reason='' if known else 'no clips loaded'))
        if known:
            p['clip']['pose'] = dict(elevation_deg=clip_elev, wrist_above_head_cm=clip_wrist_head)
        return p

    def verdict(self, **kw):
        lim = kw.pop('lim', pdr.Limits())
        p = self.punch(**kw)
        pdr.judge(p, lim)
        return p

    def test_elevation_limit_is_inclusive(self):
        self.assertEqual(self.verdict(elev=13.0)['verdict'], 'PASS')
        self.assertEqual(self.verdict(elev=13.1)['verdict'], 'FAIL')
        self.assertEqual(self.verdict(elev=-3.1)['verdict'], 'FAIL')           # too low counts as well
        self.assertEqual(self.verdict(elev=20.0, lim=pdr.Limits(max_elev_diff=20.0))['verdict'], 'PASS')

    def test_kneel_threshold(self):
        self.assertEqual(self.verdict(hips=60.0)['verdict'], 'PASS')
        self.assertEqual(self.verdict(hips=59.9)['verdict'], 'FAIL')
        self.assertIn('kneeling', self.verdict(hips=36.0)['fails'][0])

    def test_wrist_rules(self):
        self.assertEqual(self.verdict(wrist_head=1.0, clip_wrist_head=-5.0)['verdict'], 'FAIL')
        self.assertEqual(self.verdict(wrist_head=1.0, clip_wrist_head=4.0)['verdict'], 'PASS')         # the clip itself is that high
        self.assertEqual(self.verdict(wrist_head=30.0, clip_wrist_head=-5.0, straight=False)['verdict'], 'PASS')   # a hook is exempt
        self.assertEqual(self.verdict(wrist_head=1.0, lim=pdr.Limits(max_wrist_above_head_cm=2.0))['verdict'], 'PASS')

    def test_unknown_clip_and_exit_codes(self):
        p = self.verdict(known=False)
        self.assertEqual(p['verdict'], 'UNEVALUATED')
        self.assertEqual(self.verdict(known=False, wrist_head=5.0)['verdict'], 'FAIL')
        self.assertEqual(pdr.overall([p], True, True), 2)
        self.assertEqual(pdr.overall([p, dict(verdict='FAIL')], True, True), 1)
        self.assertEqual(pdr.overall([dict(verdict='PASS')], True, True), 0)
        self.assertEqual(pdr.overall([], True, True), 2)
        self.assertEqual(pdr.overall([dict(verdict='PASS')], False, True), 2)
        self.assertEqual(pdr.overall([dict(verdict='PASS')], True, False), 2)

    def test_blend_does_not_change_the_verdict(self):
        for blend in (True, False):
            p = self.punch(blend=blend)
            p['game']['stage0']['elevation_deg'] = 40.0          # the engine pose is high, the written pose is fine
            pdr.judge(p, pdr.Limits())
            self.assertEqual(p['verdict'], 'PASS', blend)
            self.assertEqual(any('blend' in w for w in p['warnings']), blend)
        p = self.punch(blend=True, elev=30.0)
        pdr.judge(p, pdr.Limits())
        self.assertEqual(p['verdict'], 'FAIL')

    def test_kneeling_field_and_diff_vs_clip(self):
        p = self.verdict(hips=36.0)
        self.assertTrue(p['kneeling'])
        self.assertFalse(self.verdict(hips=96.0)['kneeling'])
        p = self.verdict(elev=30.0, clip_elev=5.0)
        self.assertAlmostEqual(p['diff_vs_clip']['elevation_deg'], 25.0)


class ClipPickTest(unittest.TestCase):
    def test_tie_prefers_punch_family(self):
        res = {n: (s, None, None) for n, s in (('safe_attack', 4.0), ('combo_attack_01', 4.3), ('combo_attack_01_hit', 4.0), ('jump_attack_hit', 4.0),
                                               ('block', 9.0))}
        chosen, alts = pdr.pick_clip(res)
        self.assertEqual(chosen, 'combo_attack_01')
        self.assertIn('safe_attack', alts)
        self.assertNotIn('block', alts)
        self.assertEqual(pdr.pick_clip({'a_hit': (1.0, 0, 0), 'b_deflect': (1.2, 0, 0)})[0], 'a_hit')    # nothing better: stays with the best
        clips = {'combo_attack_01': None, 'x_hit': None}
        self.assertEqual(pdr.clip_base('combo_attack_01_hit', clips), 'combo_attack_01')
        self.assertEqual(pdr.clip_base('x_hit', clips), 'x_hit')
        self.assertEqual(pdr.clip_base('y_hit', clips), 'y_hit')                 # base missing: no KeyError later


class RealExcerptTest(unittest.TestCase):
    """Male V punching from his knees (run 20261112-233852, 23:40:58 - 23:41:02): hips 36 cm above the root, arms 29 - 59 deg above horizontal."""

    @classmethod
    def setUpClass(cls):
        cls.dir = FIXTURES / 'punch_direction_male'
        cls.rep = None
        if (cls.dir / 'excerpt.utpppose').exists():
            cls.tmp = tempfile.TemporaryDirectory()
            game = pathlib.Path(cls.tmp.name) / 'game' / 'red4ext' / 'logs'
            game.mkdir(parents=True)
            name = 'utpp-pose-2026-11-12-23-39-51.utpppose'
            (game / name).write_bytes((cls.dir / 'excerpt.utpppose').read_bytes())
            cls.rep = pdr.analyze(str(cls.dir / 'excerpt.log'), None, str(pathlib.Path(cls.tmp.name) / 'game'), str(cls.dir / 'clips'))

    @classmethod
    def tearDownClass(cls):
        if cls.rep is not None:
            cls.tmp.cleanup()

    def setUp(self):
        if self.rep is None:
            self.skipTest('fixture folder missing')

    def test_punches_are_found_and_identified(self):
        rep = self.rep
        self.assertEqual(rep['body'], 'man')
        self.assertEqual(rep['exit_code'], 1)
        by = self.combo()
        self.assertIn(('Right', 'combo_attack_01'), by)
        self.assertIn(('Left', 'combo_attack_02'), by)
        self.assertIn(('Right', 'final_attack_03'), by)
        self.assertEqual(len(rep['punches']), 4)               # the draw tap at 23:40:58 counts as a punch as well

    def combo(self):
        """The three punches of the melee_punch step (from 23:41:00), keyed by side and clip."""
        return {(p['side'], p['clip']['name']): p for p in self.rep['punches'] if p['time'] >= '23:41:00'}

    def test_numbers_of_the_kneeling_run(self):
        by = self.combo()
        cross, jab, final = by[('Right', 'combo_attack_01')], by[('Left', 'combo_attack_02')], by[('Right', 'final_attack_03')]
        self.assertAlmostEqual(cross['game']['stage1']['elevation_deg'], 50.9, delta=1.5)
        self.assertAlmostEqual(cross['clip']['pose']['elevation_deg'], 4.6, delta=1.0)
        self.assertAlmostEqual(jab['game']['stage1']['elevation_deg'], 35.9, delta=1.5)
        self.assertAlmostEqual(jab['clip']['pose']['elevation_deg'], 0.9, delta=1.0)
        self.assertAlmostEqual(final['clip']['pose']['elevation_deg'], 20.1, delta=2.0)
        for p in (cross, jab, final):
            self.assertAlmostEqual(p['game']['stage1']['hips_above_root_cm'], 36.0, delta=1.5)
            self.assertTrue(any(f.startswith('kneeling') for f in p['fails']))
            self.assertEqual(p['verdict'], 'FAIL')
            self.assertTrue(p['full_body_on'])
        self.assertTrue(cross['straight'])
        self.assertFalse(final['straight'])
        self.assertAlmostEqual(jab['game']['stage1']['azimuth_deg'], -72.4, delta=2.0)      # the jab goes far to the left
        self.assertLess(jab['game']['stage1']['trunk_lean_deg'], -30.0)                      # leaning back

    def test_pop_indicator_on_the_real_run(self):
        pop = self.rep['pop']
        self.assertGreater(pop['largest_hand_position_jump']['max_position_jump_cm'], 5.0)
        self.assertIn('combo_attack_01', pop['clip_reference'])


# ---------------------------------------------------------------------------------------------------------------------- fixture cutter
def carve_fixture(log_path, pose_path, clips_dir, out_dir, t_lo, t_hi, clip_names=('combo_attack_01', 'combo_attack_02', 'final_attack_03')):
    """Cuts the excerpt [t_lo, t_hi] (seconds since 1970 of the local clock, see pdr.stamp_epoch) of a recording to the CHAIN bones (+ ancestors),
    the log lines of that time and the named clips reduced to the bones the tool reads."""
    import datetime
    out = pathlib.Path(out_dir)
    (out / 'clips').mkdir(parents=True, exist_ok=True)
    g = pdr.ph.load_groups([pose_path], pdr.ph.Config())[0]
    h = g['header']
    mapped = h.mapped()
    names = [h.names[i] for i in mapped]
    rig = [h.game_index[i] for i in mapped]
    r2c = {r: k for k, r in enumerate(rig)}
    par = [r2c.get(h.parents[r], -1) if r < len(h.parents) else -1 for r in rig]
    need = set()
    for n in pdr.CHAIN:
        k = names.index(n)
        while k >= 0 and k not in need:
            need.add(k)
            k = par[k]
    sel = sorted(need)
    day0 = (datetime.date.fromisoformat(g['day']) - pdr.EPOCH.date()).days * 86400.0
    vals, ticks = [], []
    for item in pdr.ph.pc.read_capture(g['files'][0][0]):
        if item[0] != 'block' or item[1] != 0:
            continue
        _, _, meta, v = item
        n = meta.shape[0]
        sec = day0 + g['files'][0][2] + (meta[:, 0] - h.arm_ticks) / float(h.qpc_hz)
        m = (sec >= t_lo) & (sec <= t_hi)
        if m.any():
            vals.append(v.reshape(n, len(mapped), 24)[m][:, sel])
            ticks.append(meta[m, 0])
    vals, ticks = np.concatenate(vals), np.concatenate(ticks)
    rec = Rec([names[k] for k in sel], [sel.index(par[k]) if par[k] >= 0 else -1 for k in sel], vals, ticks, utc=h.utc, qpc_hz=h.qpc_hz,
              arm_ticks=h.arm_ticks)
    data = tp.encode_header(rec, h.algorithm, 0) + tp.encode_block(rec, 0, 0, rec.frames, h.algorithm) + tp.encode_trailer((rec.frames, 0))
    tp.write_file(out / 'excerpt.utpppose', data)
    keep = []
    for ln in open(log_path, encoding='utf-8', errors='replace'):
        m = pdr.LINE_RE.match(ln)
        if not m:
            continue
        t = pdr.stamp_epoch(m.group(1), m.group(2))
        if any(k in ln for k in ('UTPP MMMASK', 'UTPP MMMELEEFULL', 'UTPP MELEEAIM')) and t_lo - 3.0 <= t <= t_hi + 1.0:
            keep.append(ln.rstrip('\n'))
        elif 'body=man' in ln and 'load start' in ln or 'pose capture: started' in ln:
            keep.append(ln.rstrip('\n'))
    (out / 'excerpt.log').write_text('\n'.join(keep) + '\n', encoding='utf-8', newline='\n')
    import glob
    small = ['Hips', 'Spine3', 'Head', 'RightArm', 'RightHand', 'LeftArm', 'LeftHand', 'LeftFoot', 'RightFoot']
    for f in sorted(glob.glob(os.path.join(clips_dir, '*.npz'))):
        nm = os.path.basename(f).split('__', 1)[1].split('.')[0]
        if nm not in clip_names:
            continue
        with np.load(f, allow_pickle=True) as z:
            n152 = [str(x) for x in z['names152']]
            wn = [str(x) for x in z['names']]
            np.savez_compressed(out / 'clips' / os.path.basename(f), names=np.array(small), names152=np.array(pdr.MATCH_BONES),
                                W=z['W'][:, [wn.index(b) for b in small]].astype(np.float32),
                                lr=z['lr'][:, [n152.index(b) for b in pdr.MATCH_BONES]].astype(np.float32))
        src = json.load(open(f.replace('.npz', '.info.json'), encoding='utf-8'))
        (out / 'clips' / os.path.basename(f).replace('.npz', '.info.json')).write_text(
            json.dumps({k: src[k] for k in ('impact_built', 'side', 'note') if k in src}), encoding='utf-8')


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--carve':
        # --carve LOG POSE CLIPS_DIR OUT_DIR FROM TO   (FROM / TO = 'YYYY-MM-DD HH:MM:SS.fff')
        a = sys.argv[2:]
        f = lambda s: pdr.stamp_epoch(*s.split(' '))
        carve_fixture(a[0], a[1], a[2], a[3], f(a[4]), f(a[5]))
    else:
        unittest.main()
