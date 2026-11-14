"""Tests of helper_residual_report.py on synthetic compact recordings (no game, no game data).

  python -I test_helper_residual_report.py

A small rig and graph (test_helper_graph_batch.py) stand in for the player's: recordings are written with the reference encoder of
test_pose_capture_to_csv.py, their helper bones are computed by the graph itself, so every verdict is known in advance (graph-exact helpers
pass, helpers left at the engine's value fail, no stream 1 is not verifiable, ...). The bundle files are the owned.txt written next to the
recording; the evaluator of the tool is replaced by the synthetic one for the duration of the tests.
"""
import json
import os
import pathlib
import sys
import tempfile
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import helper_graph_batch as hgb  # noqa: E402
import helper_residual_report as hrr  # noqa: E402
import test_helper_graph_batch as tg  # noqa: E402
import test_pose_capture_to_csv as tp  # noqa: E402
from export_player_helper_bundle import fnv64  # noqa: E402

NAME = 'utpp-pose-2026-11-11-09-00-00.utpppose'                 # local stamp = header UTC: offset 0
HZ = 1000                                                         # ticks per second: 1 frame per 20 ms below
OWNED = ['l_SHL_0_JNT', 'l_chest_front_A_bot_out_JNT', 'l_knee_mscl_JNT', 'l_elbow_back_mscl_JNT', 'l_wrist_A_mscl_JNT', 'r_SHL_0_JNT',
         'r_deltoid_back_top_out_JNT', 'r_butterfly_top_CRV_top_out_JNT']
NOT_OWNED = ['l_Wrist_0_JNT', 'l_thumb_side_mscl_JNT']


def matrix_to_row(m):
    """[n,4,4] -> [n,12] recording rows (translation, pad, quaternion xyzw, scale, pad)."""
    scale = np.linalg.norm(m[:, :3, :3], axis=1)
    r = m[:, :3, :3] / scale[:, None, :]
    q = hgb._quat_from_rot(r)
    row = np.zeros((m.shape[0], 12))
    row[:, 0:3] = m[:, :3, 3]
    row[:, 4:8] = q
    row[:, 8:11] = scale
    return row


class Rec:
    """Slice-like object for tp.encode_header / tp.encode_block."""

    def __init__(self, rig_count=619):
        self.names = tg.NAMES
        n = len(self.names)
        self.game_index = list(range(n))
        self.bones = [(i, fnv64(self.names[i]), tg.RIG[i][1]) for i in range(n)]
        self.qpc_hz, self.arm_ticks, self.utc = HZ, 0, (2026, 11, 11, 9, 0, 0, 0)
        self.max_frames, self.rig, self.rig_count = 100000, 0xABC, rig_count


def write_recording(path, frames=100, written_from=40, post='graph', rig_count=619, seed=11, post_core_shift=False):
    """post: 'graph' (stream 1 = helpers from the graph), 'owned_only' (as 'graph' but the helpers outside the bundle keep the engine's
    value), 'stale' (stream 1 = helpers the engine computed from the pose before the mod wrote) or None (no stream 1). The mod writes the
    core pose from frame `written_from` on. post_core_shift moves a core bone of stream 1 (the core changed after the stage was recorded)."""
    ev = tg.synthetic_evaluator()
    batch = hgb.BatchEvaluator('man', ev)
    core0 = hgb.perturbed_core_locals(ev, frames, seed, max_deg=12.0, max_cm=1.0)
    core1 = {k: v.copy() for k, v in core0.items()}
    rng = np.random.default_rng(seed + 1)
    ang = np.radians(rng.uniform(15, 30, (frames - written_from, 3)))
    extra = hgb._rot_from_quat(np.concatenate([ang / 2, np.ones((frames - written_from, 1))], axis=1))
    for name in ('LeftArm', 'RightArm'):
        core1[name][written_from:, :3, :3] = np.matmul(core1[name][written_from:, :3, :3], extra)
        core1[name][written_from:, :3, 3] += rng.uniform(0.04, 0.09, (frames - written_from, 3)) * rng.choice([-1.0, 1.0], (frames - written_from, 3))
    m0, _ = batch.run(core0, want=tg.NAMES)
    m1, _ = batch.run(core1, want=tg.NAMES)
    parent = {r[0]: r[1] for r in tg.RIG}

    def local_rows(models, cores):
        rows = np.zeros((frames, len(tg.NAMES), 12))
        for i, name in enumerate(tg.NAMES):
            if name in cores:
                loc = cores[name]
            else:
                p = parent[name]
                loc = np.matmul(np.linalg.inv(models[tg.NAMES[p]]), models[name])
            rows[:, i, :] = matrix_to_row(loc)
        return rows

    stage0 = local_rows(m0, core0)                                    # engine: helpers computed from the engine's own pose
    engine_on_mod_pose = local_rows(m0, core1)                        # mod pose for the core, helper locals still the engine's
    stage1 = engine_on_mod_pose
    values = np.zeros((frames, len(tg.NAMES), 24), np.float32)
    values[:, :, :12] = stage0
    values[:, :, 12:] = stage1
    rec = Rec(rig_count)
    rec.values = values
    rec.meta = np.zeros((frames, 4), dtype=np.int64)
    rec.meta[:, 0] = np.arange(frames) * 20                            # 20 ms per frame at 1 kHz
    rec.meta[:, 1] = 7
    rec.meta[:, 2] = 3
    rec.meta[:, 3] = 11
    streams = (0, 1) if post else (0,)
    data = tp.encode_header(rec, 0, 0, streams=streams)
    half = frames // 2
    data += tp.encode_block(rec, 0, 0, half, 0) + tp.encode_block(rec, 0, half, frames, 0)
    if post:
        if post in ('graph', 'owned_only'):
            post_rows = local_rows(m1, core1)
            if post == 'owned_only':
                for other in NOT_OWNED:
                    post_rows[:, tg.NAMES.index(other), :] = stage1[:, tg.NAMES.index(other), :]
        else:
            post_rows = stage1.copy()
        if post_core_shift:
            post_rows[:, tg.NAMES.index('LeftArm'), 0] += 0.01
        full = np.zeros((frames, len(tg.NAMES), 24), np.float32)
        full[:, :, :12] = post_rows
        pm = np.zeros((frames, 8), dtype=np.int64)
        pm[:, :4] = rec.meta
        pm[:, 0] += 1
        data += tp.encode_block(rec, 1, 0, frames, 0, values=full, meta=pm)
    data += tp.encode_trailer((frames, frames if post else 0))
    pathlib.Path(path).write_bytes(data)
    return str(path)


def make_args(bundle_dir, **kw):
    a = dict(bundle_dir=str(bundle_dir), plan=None, clock_offset=0.0, window_seconds=0.0, limit_pos_cm=0.05, limit_rot_deg=0.5,
             min_written=5, worst=3, detail='summary', jobs=1, extra_helpers=True)
    a.update(kw)
    return a


class ResidualReportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.batch = hgb.BatchEvaluator('man', tg.synthetic_evaluator())
        cls._orig = hrr.evaluator_for
        hrr.evaluator_for = lambda body: cls.batch

    @classmethod
    def tearDownClass(cls):
        hrr.evaluator_for = cls._orig

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=tp.scratch_root())
        self.addCleanup(self.tmp.cleanup)
        self.dir = pathlib.Path(self.tmp.name)
        self.bundle = self.dir / 'bundle'
        self.bundle.mkdir()
        (self.bundle / 'owned.txt').write_text('\n'.join(OWNED + ['lacking_bone_JNT']) + '\n', encoding='utf-8')

    def record(self, name=NAME, **kw):
        return write_recording(self.dir / name, **kw)

    def analyse(self, path, **kw):
        out = []
        bundle = kw.pop('bundle_dir', self.bundle)
        for g in hrr.ph.load_groups([path], hrr.ph.Config()):
            out.append(hrr.analyze_recording(make_args(bundle, **kw), g))
        self.assertEqual(len(out), 1)
        return out[0]

    # ------------------------------------------------------------------ verdicts
    def test_graph_exact_helpers_pass(self):
        r = self.analyse(self.record(post='graph'))
        self.assertEqual(r['status'], 'PASS', json.dumps(r['windows'][0].get('post', {}).get('written', {}).get('groups', {}))[:400])
        w = r['windows'][0]
        self.assertEqual((w['frames'], w['written'], w['not_written']), (100, 60, 40))
        for g, o in w['post']['written']['groups'].items():
            self.assertLess(o['max_pos_cm'], 1e-3, g)
            self.assertLess(o['max_rot_deg'], 0.01, g)                      # 16-bit quantisation noise of the recording
        groups = w['post']['written']['groups']
        self.assertEqual({g for g, o in groups.items() if o['covered']}, {'upper', 'lower', 'elbow', 'wrist_muscle'})
        self.assertEqual({g for g, o in groups.items() if not o['covered']}, {'wrist_roll', 'hand'})       # helpers outside the bundle: reported only
        self.assertEqual(r['evaluation']['core_gap_frames'], 0)
        self.assertEqual(r['evaluation']['evaluator_refused_frames'], 0)
        self.assertLess(r['evaluation']['max_core_gap_post'], 1e-3)       # the post stream holds the same core pose (quantisation noise only)

    def test_stale_helpers_fail_and_are_listed(self):
        r = self.analyse(self.record(post='stale'))
        self.assertEqual(r['status'], 'FAIL')
        w = r['windows'][0]
        self.assertEqual(w['verdict'], 'FAIL')
        self.assertIn('upper', w['failing_groups'])
        wr = w['post']['written']
        self.assertGreater(wr['groups']['upper']['max_pos_cm'], 1.0)      # the arms were rotated by 15-30 degrees
        self.assertTrue(wr['worst_frames'])
        self.assertGreaterEqual(wr['worst_frames'][0]['frame'], 40)       # worst frames come from the mod-written range
        # frames the mod did not write: the post stream equals the engine's helpers, which equal the graph
        nw = w['post']['not_written']
        self.assertEqual(nw['frames'], 40)
        for g, o in nw['groups'].items():
            self.assertLess(o['max_pos_cm'], 1e-3, g)

    def test_stale_mode_shows_what_the_recompute_repairs(self):
        r = self.analyse(self.record(post='graph'))
        w = r['windows'][0]
        st = w['stale']['written']['groups']['upper']
        self.assertGreater(st['p99_pos_cm'], 0.5)                          # engine helpers against the graph on the mod pose
        self.assertGreater(st['p99_rot_deg'], 1.0)
        control = w['stale']['not_written']['groups']
        for g, o in control.items():
            self.assertLess(o['max_pos_cm'], 1e-3, g)                      # graph == engine where the mod did not write

    def test_no_post_stream_is_not_verifiable(self):
        r = self.analyse(self.record(post=None))
        self.assertEqual(r['status'], 'NOT_VERIFIABLE')
        self.assertIn('post_helper_apply_after_commit', r['reason'])
        w = r['windows'][0]
        self.assertEqual(w['verdict'], 'NOT_VERIFIABLE')
        self.assertNotIn('post', w)
        self.assertGreater(w['stale']['written']['groups']['upper']['p99_pos_cm'], 0.5)

    def test_no_mod_write_is_reported_as_such(self):
        r = self.analyse(self.record(post='graph', written_from=100))
        self.assertEqual(r['status'], 'NO_MOD_WRITES')
        self.assertEqual(r['windows'][0]['written'], 0)

    def test_bundle_bones_the_recording_lacks_are_named(self):
        r = self.analyse(self.record(post='graph'))
        b = r['bundle']
        self.assertEqual(b['unverified']['lacking'], ['lacking_bone_JNT'])
        self.assertEqual(b['unverified']['unresolved_chain'], [])
        self.assertEqual(b['owned'], len(OWNED) + 1)
        self.assertEqual(b['recorded'], len(OWNED))
        self.assertEqual(b['extra_helpers_evaluated'], len(NOT_OWNED))
        self.assertEqual(b['owned_from'], 'owned.txt')

    def test_helpers_outside_the_bundle_are_reported_but_never_gate(self):
        r = self.analyse(self.record(post='owned_only'))
        self.assertEqual(r['status'], 'PASS')
        groups = r['windows'][0]['post']['written']['groups']
        self.assertGreater(groups['wrist_roll']['max_pos_cm'] + groups['hand']['max_pos_cm'], 1.0)      # the engine's value: not what the graph says
        self.assertIsNone(groups['hand']['ok'])
        self.assertEqual(groups['hand']['failing'], [])
        only = self.analyse(self.record(post='owned_only'), extra_helpers=False)
        self.assertEqual(set(only['windows'][0]['post']['written']['groups']), {'upper', 'lower', 'elbow', 'wrist_muscle'})
        self.assertEqual(only['status'], 'PASS')

    def test_core_changed_after_the_stage_is_counted(self):
        r = self.analyse(self.record(post='graph', post_core_shift=True))
        self.assertGreater(r['evaluation']['core_gap_frames'], 0)
        self.assertGreater(r['evaluation']['max_core_gap_post'], 0.005)
        text = '\n'.join(hrr.summary_lines([r]))
        self.assertIn('core changed after the mod', text)

    def test_owned_bones_from_an_installed_binding_file(self):
        installed = self.dir / 'installed'
        installed.mkdir()
        lines = ['UTPP_HELPER_BINDING 1', 'COUNTS 1 1 %d 0 0' % len(OWNED)] + ['OWNED %016X %d' % (fnv64(n), i) for i, n in enumerate(OWNED)] + ['END']
        (installed / 'binding.dat').write_text('\n'.join(lines) + '\n', encoding='ascii')
        (installed / 'program.ir').write_bytes(b'x')
        r = self.analyse(self.record(post='graph'), bundle_dir=str(installed))
        self.assertEqual(r['status'], 'PASS')
        self.assertEqual(r['bundle']['owned_from'], 'binding.dat')
        self.assertEqual(r['bundle']['owned'], len(OWNED))
        self.assertEqual(r['bundle']['unverified']['lacking'], [])
        self.assertEqual(len(r['bundle']['binding_file_sha256']), 64)
        (installed / 'binding.dat').write_text('OWNED %016X 1\n' % fnv64('no_such_bone_JNT'), encoding='ascii')
        with self.assertRaises(ValueError):
            hrr.read_owned(str(installed), tg.NAMES)

    def test_unsupported_rig_is_not_verifiable(self):
        r = self.analyse(self.record(post='graph', rig_count=100))
        self.assertEqual(r['status'], 'NOT_VERIFIABLE')
        self.assertIn('unsupported rig', r['reason'])

    # ------------------------------------------------------------------ windows
    def test_windows_from_a_plan_and_fixed_windows(self):
        path = self.record(post='stale')
        plan = {'runs': [{'scenario': 'calm', 'run': 1, 'steps': [{'name': 'idle', 'start': '2026-11-11 09:00:00.100000', 'end': '2026-11-11 09:00:00.700000'}]},
                         {'scenario': 'busy', 'run': 2, 'steps': [{'name': 'move', 'start': '2026-11-11 09:00:01.000000', 'end': '2026-11-11 09:00:01.900000'}]},
                         {'scenario': 'late', 'run': 1, 'steps': [{'name': 'x', 'start': '2026-11-11 09:05:00', 'end': '2026-11-11 09:05:01'}]}]}
        pp = self.dir / 'plan.json'
        pp.write_text(json.dumps(plan), encoding='utf-8')
        r = self.analyse(path, plan=str(pp))
        by = {w['label']: w for w in r['windows']}
        self.assertEqual(set(by), {'all', 'calm#1:idle', 'busy#2:move', 'late#1:x'})
        self.assertEqual(by['calm#1:idle']['verdict'], 'NO_WRITES')          # frames 5..34: before the first mod write (frame 40 = 0.8 s)
        self.assertEqual(by['calm#1:idle']['written'], 0)
        self.assertEqual(by['busy#2:move']['verdict'], 'FAIL')
        self.assertEqual(by['busy#2:move']['written'], 45)
        self.assertEqual(by['late#1:x']['verdict'], 'OUTSIDE_RECORDING')
        r2 = self.analyse(path, plan=str(pp), clock_offset=-1.0)             # log time = plan time - 1 s: the busy step moves to 0.0 .. 0.9 s
        by2 = {w['label']: w for w in r2['windows']}
        self.assertEqual(by2['busy#2:move']['written'], 5)                  # frames 40..44 only
        fx = self.analyse(path, window_seconds=0.5)
        self.assertEqual(fx['windows'][0]['label'], 'all')
        self.assertGreaterEqual(len(fx['windows']), 4)
        self.assertEqual(sum(w['frames'] for w in fx['windows'][1:]), 100)

    # ------------------------------------------------------------------ command line, files, exit codes
    def run_cli(self, path, *extra):
        out = self.dir / 'out'
        code = hrr.main([path, '--out-dir', str(out), '--bundle-dir', str(self.bundle), '--jobs', '1', *extra])
        return code, out

    def test_cli_exit_codes_and_files(self):
        code, out = self.run_cli(self.record(post='graph'))
        self.assertEqual(code, 0)
        for suffix in ('.json', '_bones.csv', '_summary.txt'):
            self.assertTrue((out / ('helper_residual' + suffix)).is_file(), suffix)
        data = json.loads((out / 'helper_residual.json').read_text(encoding='utf-8'))
        self.assertEqual(data['recordings'][0]['status'], 'PASS')
        csv_text = (out / 'helper_residual_bones.csv').read_text(encoding='utf-8')
        self.assertIn('l_SHL_0_JNT', csv_text)
        self.assertNotIn(str(self.dir), (out / 'helper_residual_summary.txt').read_text(encoding='utf-8').replace(str(out), ''))
        self.assertEqual(self.run_cli(self.record('utpp-pose-2026-11-11-09-10-00.utpppose', post='stale'))[0], 1)
        self.assertEqual(self.run_cli(self.record('utpp-pose-2026-11-11-09-20-00.utpppose', post=None))[0], 2)

    def test_limits_are_arguments(self):
        path = self.record(post='graph')
        code, out = self.run_cli(path, '--limit-pos-cm', '1e-9', '--limit-rot-deg', '1e-9')
        self.assertEqual(code, 1)                                            # quantisation noise exceeds an absurd limit: the limits are used
        code, out = self.run_cli(path, '--limit-pos-cm', '0.5', '--limit-rot-deg', '5')
        self.assertEqual(code, 0)

    def test_output_never_goes_next_to_the_recording(self):
        path = self.record(post='graph')
        before = sorted(os.listdir(self.dir))
        self.run_cli(path)
        after = sorted(os.listdir(self.dir))
        self.assertEqual([x for x in after if x not in before], ['out'])


class GroupTest(unittest.TestCase):
    def test_groups_by_name(self):
        g = hrr.helper_group
        self.assertEqual(g('l_SHL_0_JNT'), 'upper')
        self.assertEqual(g('r_latissimusDorsi_mscl_JNT'), 'upper')
        self.assertEqual(g('l_leg_back_A_mscl_JNT'), 'lower')
        self.assertEqual(g('l_calf_0_JNT'), 'lower')
        self.assertEqual(g('l_lowLeg_back_A_mscl_JNT'), 'lower_leg_back')
        self.assertEqual(g('r_elbow_bend_B_mscl_JNT'), 'elbow')
        self.assertEqual(g('l_wrist_A_mscl_JNT'), 'wrist_muscle')
        self.assertEqual(g('l_Wrist_1_JNT'), 'wrist_roll')
        self.assertEqual(g('l_ind_knuckle_top_mscl_JNT'), 'hand')
        self.assertEqual(g('l_thumb_side_mscl_JNT'), 'hand')
        self.assertEqual(g('r_sternocleidomastoid_bot_mscl_JNT'), 'sterno')

    def test_current_bundles_partition_into_40_16_6_6(self):
        for body in ('man', 'woman'):
            d = hrr.default_bundle_dir(body)
            if not os.path.isfile(os.path.join(d, 'owned.txt')):
                self.skipTest('bundle folder not present: ' + d)
            owned = hrr.read_owned(d)
            counts = {}
            for n in owned:
                counts[hrr.helper_group(n)] = counts.get(hrr.helper_group(n), 0) + 1
            self.assertEqual(counts, {'upper': 40, 'lower': 16, 'elbow': 6, 'wrist_muscle': 6}, body)


if __name__ == '__main__':
    unittest.main(verbosity=2)
