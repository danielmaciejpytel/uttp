"""Offline tests for bt_gate.py (synthetic log + body-trace fixtures built in a temp dir; one optional real-session test).

  python Source\\tools\\gametest\\test_bt_gate.py -v
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
from utpp_paths import game_dir  # noqa: E402

_GAME_LOGS = os.path.join(game_dir(required=False) or '', 'red4ext', 'logs')

import bt_gate as bg  # noqa: E402

# ------------------------------------------------------------------------------------------------ fixture builders

# index, name, parent, local translation (m) when the bone is not driven by the scenario
RIG = [
    (0, 'Root', -1, (0, 0, 0)),
    (2, 'Hips', 0, (0, 0, 1.0)),
    (4, 'Spine', 2, (0, 0, 0.1)),
    (7, 'Spine1', 4, (0, 0, 0.1)),
    (10, 'Spine2', 7, (0, 0, 0.1)),
    (13, 'Spine3', 10, (0, 0, 0.1)),
    (22, 'Head', 13, (0, 0, 0.3)),
    (30, 'RightArm', 13, (-0.2, 0, 0.2)),
    (31, 'RightForeArm', 30, (0, 0, -0.3)),
    (32, 'RightHand', 31, (0, 0, -0.25)),
    (5, 'LeftUpLeg', 2, (0.1, 0, -0.05)),
    (8, 'LeftLeg', 5, (0, 0, -0.45)),
    (11, 'LeftFoot', 8, (0, 0, -0.45)),
    (43, 'LeftToeBase', 11, (0, 0.1, -0.05)),
    (6, 'RightUpLeg', 2, (-0.1, 0, -0.05)),
    (9, 'RightLeg', 6, (0, 0, -0.45)),
    (12, 'RightFoot', 9, (0, 0, -0.45)),
    (44, 'RightToeBase', 12, (0, 0.1, -0.05)),
]
FRAME_S = 1.0 / 60.0
DAY = bg.epoch_of(bg.datetime(2026, 10, 28))
BASE = DAY + 12 * 3600          # local 12:00:00 = trace time 0
ARM_TICKS = 1_000_000


def qx(deg):
    h = math.radians(deg) / 2
    return (math.sin(h), 0.0, 0.0, math.cos(h))


def ts(t_rel):
    """Log timestamp for a time relative to the scenario origin."""
    s = bg.fmt_t(BASE + t_rel)
    return '[2026-10-28 %s]' % s


class Pose:
    """Per-frame driven values: hips position and an x rotation (deg) per bone; everything else rig default."""

    def __init__(self, hips=(0.0, 0.0, 0.5), rot=None, trans=None):
        self.hips = hips
        self.rot = rot or {}
        self.trans = trans or {}      # bone name -> local translation override (m)


def build_trace(path, pose_fn, times, dup_pose=None, dup_at=None, utc_hour=11, pose0_fn=None):
    """times: list of frame times (s, scenario origin). dup_at: frame time that gets an extra sample 0.3 ms later with dup_pose.
    pose0_fn: pose of the s0 columns (default: the same as s1)."""
    rows = []
    rows.append('# qpc_hz=10000000 arm_ticks=%d utc=2026-10-28T%02d:00:00.000Z max_frames=200 continuous=1' % (ARM_TICKS, utc_hour))
    for idx, name, parent, _ in RIG:
        rows.append('# rig_bone index=%d hash=0000000000000000 parent=%d' % (idx, parent))
    rows.append('ticks,desc,thread,rig,generation,rigCount,name,gameIndex,' +
                ','.join('s0_%d' % i for i in range(12)) + ',' + ','.join('s1_%d' % i for i in range(12)))

    def cols(pose, name, tr):
        p = pose.trans.get(name, tr)
        if name == 'Hips':
            p = pose.hips
        q = qx(pose.rot.get(name, 0.0))
        vals = [p[0], p[1], p[2], 1.0, q[0], q[1], q[2], q[3], 1, 1, 1, 1]
        return ','.join(repr(float(v)) for v in vals)

    def emit(t, pose, pose0=None):
        tk = ARM_TICKS + int(round(t * 1e7))
        for idx, name, parent, tr in RIG:
            s1 = cols(pose, name, tr)
            s0 = cols(pose0, name, tr) if pose0 is not None else s1
            rows.append('%d,1,2,3,4,185,%s,%d,%s,%s' % (tk, name, idx, s0, s1))

    for t in times:
        emit(t, pose_fn(t), pose0_fn(t) if pose0_fn else None)
        if dup_at is not None and abs(t - dup_at) < 1e-9:
            emit(t + 0.0003, dup_pose)
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(rows) + '\n')


def log_line(t, text, who='utpp_body'):
    return '%s [info    ] [ 1] [UTPP] UTPP CET [%s] t=%.2f %s' % (ts(t), who, t, text)


def build_log(path, *, mount=(1.0, 2.0), exit_=(4.0, 6.0), extra=(), with_anim=True, trace_name='trace.csv',
              tz_line=True, drop=()):
    """mount = (start, mount->ride); exit_ = (ride->exit, exit->after); extra = raw (t, text) lines."""
    L = []
    if tz_line:
        L.append('%s [info    ] [ 1] [UTPP] UTPP body trace continuous: started file=C:\\logs\\%s' % (ts(0.0), trace_name))
    if mount:
        if with_anim:
            L.append('%s [info    ] [ 1] [UTPP] UTPP CET [utpp_vehicle_anim] ride 1: utpp_vehicle_tpp=1 (bike mount, UTPP third person)' % ts(mount[0]))
            L.append('%s [info    ] [ 1] [UTPP] UTPP CET [utpp_vehicle_anim] ride 1: workspot animVariables [a,b,utpp_vehicle_tpp] (utpp_vehicle_tpp added)' % ts(mount[0] + 0.03))
        L.append(log_line(mount[0] + 0.03, 'vehicle idle -> mount (mount start, PSM vehicle=0 mounted=true)'))
        if mount[1] is not None:
            L.append(log_line(mount[1], 'vehicle mount -> ride (seated, PSM vehicle=1 mounted=true)'))
            # the camera module writes similar text a bit later; it must not move the window
            L.append(log_line(mount[1] + 0.8, 'keepTpp mount -> ride', who='utpp_vehiclecam'))
    if exit_:
        L.append(log_line(exit_[0], 'keepTpp ride -> exit', who='utpp_vehiclecam'))
        L.append(log_line(exit_[0], 'vehicle ride -> exit (exit start, PSM vehicle=4 mounted=true)'))
        if exit_[1] is not None:
            L.append(log_line(exit_[1], 'vehicle exit -> after (unmount, PSM vehicle=0 mounted=false)'))
    for t, text in extra:
        L.append('%s [info    ] [ 1] [UTPP] UTPP %s' % (ts(t), text))
    L = [l for l in L if not any(d in l for d in drop)]
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')


def clean_pose(t):
    """Smooth mount (legs ease to 40 deg until 2.0 s), seated until 4.0 s, smooth stand-up 4.0..5.5 s, standing after."""
    if t < 1.0:
        ang, hz = 0.0, 1.0
    elif t < 2.0:
        ang, hz = 40.0 * (t - 1.0), 1.0 - 0.5 * (t - 1.0)
    elif t < 4.0:
        ang, hz = 40.0, 0.5
    elif t < 5.5:
        k = (t - 4.0) / 1.5
        ang, hz = 40.0 * (1 - k), 0.5 + 0.5 * k
    else:
        ang, hz = 0.0, 1.0
    return Pose((0.0, 0.0, hz), {'LeftUpLeg': ang, 'RightUpLeg': ang, 'LeftLeg': ang / 2})


def frames(t0=0.0, t1=8.0):
    n = int(round((t1 - t0) / FRAME_S))
    return [t0 + i * FRAME_S for i in range(n + 1)]


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.log = os.path.join(self.tmp.name, 'utpp-test.log')
        self.trace = os.path.join(self.tmp.name, 'trace.csv')

    def run_gate(self, pose_fn=clean_pose, times=None, trace_kw=None, log_kw=None, **gate_kw):
        build_trace(self.trace, pose_fn, times if times is not None else frames(), **(trace_kw or {}))
        build_log(self.log, **(log_kw or {}))
        return bg.run_gate(self.log, self.trace, **gate_kw)

    @staticmethod
    def win(rep, kind):
        return [w for w in rep['windows'] if w['kind'] == kind][0]

    @staticmethod
    def checks(w):
        return {v['check'] for v in w['violations']}


def with_override(base_fn, t_from, t_to, fn):
    """Pose function that replaces base_fn in [t_from, t_to) by fn(t)."""
    return lambda t: fn(t) if t_from <= t < t_to else base_fn(t)


# ------------------------------------------------------------------------------------------------ tests

class CleanAndWindows(Base):
    def test_clean_session_passes(self):
        rep = self.run_gate()
        self.assertTrue(rep['pass'], bg.format_report(rep))
        self.assertEqual([w['kind'] for w in rep['windows']], ['mount', 'exit'])
        self.assertEqual(rep['trace_info']['duplicates_dropped'], 0)
        self.assertIn('+1 h', rep['trace_info']['tz'])

    def test_window_bounds_ignore_camera_module_lines(self):
        rep = self.run_gate()
        m = self.win(rep, 'mount')
        self.assertEqual(m['start'], '12:00:01.000')
        self.assertEqual(m['end'], '12:00:02.600')          # mount -> ride + 0.6, not the keepTpp line + 0.6
        e = self.win(rep, 'exit')
        self.assertEqual(e['start'], '12:00:04.000')
        self.assertEqual(e['end'], '12:00:06.600')

    def test_car_mount_without_anim_line_uses_idle_mount(self):
        rep = self.run_gate(log_kw={'with_anim': False})
        m = self.win(rep, 'mount')
        self.assertEqual(m['id'], 'mount 1')
        self.assertEqual(m['start'], '12:00:01.030')

    def test_incomplete_windows_fail(self):
        rep = self.run_gate(log_kw={'mount': (1.0, None), 'exit_': (4.0, None)})
        self.assertFalse(rep['pass'])
        for w in rep['windows']:
            self.assertEqual({v['check'] for v in w['violations']}, {'window_incomplete'})

    def test_tz_override_and_wrong_offset_is_uncovered(self):
        rep = self.run_gate(tz_offset_h=2.0)           # wrong on purpose: trace frames land an hour away
        self.assertFalse(rep['pass'])
        self.assertIn('trace_coverage', self.checks(self.win(rep, 'mount')))
        rep = self.run_gate(tz_offset_h=2.0, allow_uncovered=True)
        self.assertNotIn('trace_coverage', self.checks(self.win(rep, 'mount')))

    def test_tz_falls_back_to_file_name(self):
        named = os.path.join(self.tmp.name, 'utpp-bodytrace-2026-10-28-12-00-00.csv')
        build_trace(named, clean_pose, frames())
        build_log(self.log, trace_name='other.csv', tz_line=False)
        rep = bg.run_gate(self.log, named)
        self.assertIn('trace file name', rep['trace_info']['tz'])
        self.assertTrue(rep['pass'], bg.format_report(rep))

    def test_short_trace_is_not_a_silent_pass(self):
        rep = self.run_gate(times=frames(0.0, 3.0))     # exit window is not covered
        self.assertFalse(self.win(rep, 'exit')['pass'])
        self.assertIn('trace_coverage', self.checks(self.win(rep, 'exit')))


class MountChecks(Base):
    def test_leg_pop_flagged_and_boundary(self):
        for step, expect in ((20.0, True), (16.0, True), (13.0, False), (7.0, False)):
            pose = with_override(clean_pose, 1.5, 1.5 + FRAME_S / 2,
                                 lambda t, s=step: Pose((0, 0, 0.75), {'LeftUpLeg': 20.0 + s, 'RightUpLeg': 20.0, 'LeftLeg': 10}))
            rep = self.run_gate(pose)
            w = self.win(rep, 'mount')
            self.assertEqual('leg_rot' in self.checks(w), expect, step)

    def test_hips_step_limit(self):
        # a one-frame hips jump inside the mount window
        def jump(dz):
            return with_override(clean_pose, 1.5, 1.5 + FRAME_S * 0.9,
                                 lambda t: Pose((0, 0, 0.75 + dz), {'LeftUpLeg': 20.0, 'RightUpLeg': 20.0, 'LeftLeg': 10}))
        self.assertIn('hips_step', self.checks(self.win(self.run_gate(jump(0.09)), 'mount')))
        self.assertNotIn('hips_step', self.checks(self.win(self.run_gate(jump(0.06)), 'mount')))     # c9 enter clips: <= 6.4 cm

    def test_leg_acc_onset_and_spike(self):
        # legs held, then moving at 12 deg/frame from one frame to the next (an onset at full speed, under the 15 deg limit)
        def onset(t):
            a = 20.0 if t < 1.5 else 20.0 + 720.0 * (t - 1.5)
            return Pose((0, 0, 0.75), {'LeftUpLeg': min(a, 90.0), 'RightUpLeg': 20.0})
        w = self.win(self.run_gate(onset), 'mount')
        self.assertIn('leg_acc', self.checks(w))
        self.assertNotIn('leg_rot', self.checks(w))
        # the same speed reached over 0.25 s (ease-in, <= 0.7 deg/frame per frame) and stopped the same way passes
        def ease(t):
            u = max(0.0, t - 1.3)
            if u < 0.25:
                a = 1200.0 * u * u
            elif u < 0.35:
                a = 75.0 + 600.0 * (u - 0.25)
            else:
                v = min(u - 0.35, 0.25)
                a = 135.0 + 600.0 * v - 1200.0 * v * v
            return Pose((0, 0, 0.75), {'LeftUpLeg': 20.0 + a, 'RightUpLeg': 20.0})
        w = self.win(self.run_gate(ease), 'mount')
        self.assertNotIn('leg_acc', self.checks(w), w['violations'][:3])
        # a one-frame 12 deg spike and back (under the 15 deg step limit): flagged
        spike = with_override(clean_pose, 1.5, 1.5 + FRAME_S / 2,
                              lambda t: Pose((0, 0, 0.75), {'LeftUpLeg': 20.0 + 12.0, 'RightUpLeg': 20.0, 'LeftLeg': 10}))
        self.assertIn('leg_acc', self.checks(self.win(self.run_gate(spike), 'mount')))

    def test_leg_acc_merges_a_short_hitch_sample(self):
        # steady 5 deg/frame leg motion with a 30 ms + 6 ms frame pair: the 6 ms sample is merged (its true rate is the same)
        times = [t for t in frames() if not (1.49 < t < 1.52)] + [1.4833 + 0.030, 1.4833 + 0.036]
        times.sort()
        steady = with_override(clean_pose, 1.2, 1.9,
                               lambda t: Pose((0, 0, 0.75), {'LeftUpLeg': 20.0 + 300.0 * min(max(0.0, t - 1.3), 0.5),
                                                             'RightUpLeg': 20.0}))
        rep = self.run_gate(steady, times=times)
        m = self.win(rep, 'mount')
        self.assertTrue(any('short sample' in n for n in m['notes']), m['notes'])
        self.assertNotIn('leg_acc', [v['check'] for v in m['violations'] if 1.45 < v['t'] - BASE < 1.56], m['violations'])

    def _frozen_pose(self, hold_frames):
        # legs walk to 20 deg, then jump (cut) to 40 deg and hold for hold_frames frames, then move on smoothly
        t_cut = 1.3
        hold_end = t_cut + hold_frames * FRAME_S - 1e-6

        def pose(t):
            if t < t_cut:
                a = 20.0
            elif t < hold_end:
                a = 40.0
            else:
                a = 41.0 + 20.0 * (t - hold_end)
            return Pose((0, 0, 0.7), {'LeftUpLeg': a, 'RightUpLeg': a})
        return pose

    def test_frozen_frames_boundary(self):
        # 5 identical frames (the cut frame included) violate, 4 do not
        rep5 = self.run_gate(self._frozen_pose(5))
        rep4 = self.run_gate(self._frozen_pose(4))
        self.assertIn('frozen_legs', self.checks(self.win(rep5, 'mount')))
        self.assertNotIn('frozen_legs', self.checks(self.win(rep4, 'mount')))
        v = [v for v in self.win(rep5, 'mount')['violations'] if v['check'] == 'frozen_legs'][0]
        self.assertEqual(v['value'], 5)
        rep17 = self.run_gate(self._frozen_pose(17))
        self.assertEqual([v['value'] for v in self.win(rep17, 'mount')['violations'] if v['check'] == 'frozen_legs'], [17])

    def test_static_legs_before_the_cut_are_not_frozen_frames(self):
        # standing still for 0.3 s after 'ride N', then a jump to a moving pose: no frozen run counted before the cut
        def pose(t):
            if t < 1.3:
                a = 0.0
            else:
                a = 20.0 + 10.0 * (t - 1.3)
            return Pose((0, 0, 0.7), {'LeftUpLeg': a})
        rep = self.run_gate(pose)
        self.assertNotIn('frozen_legs', self.checks(self.win(rep, 'mount')))

    def test_static_seated_pose_after_mount_to_ride_is_not_frozen(self):
        def pose(t):
            return Pose((0, 0, 0.7), {'LeftUpLeg': 0.0 if t < 1.2 else 30.0 + 20.0 * min(t, 2.0)})
        rep = self.run_gate(pose)        # legs constant 2.0..2.6 s (after mount -> ride): not counted
        self.assertNotIn('frozen_legs', self.checks(self.win(rep, 'mount')))

    def test_feet_settle_only_after_clip_end_proxy(self):
        def moving(t_from):
            return lambda t: Pose((0, 0, 0.5), {'LeftUpLeg': 40.0 + 3.0 * (t - t_from) * 60, 'RightUpLeg': 40.0, 'LeftLeg': 20})
        # moving in 2.35..2.45 (after proxy 2.3): ~3 deg/frame on a 0.9 m leg = ~4.7 cm per frame
        late = with_override(clean_pose, 2.35, 2.45, moving(2.35))
        rep = self.run_gate(late)
        self.assertIn('feet_settle', self.checks(self.win(rep, 'mount')))
        # same motion before the proxy (2.1..2.2) is not a feet_settle violation
        early = with_override(clean_pose, 2.1, 2.2, moving(2.1))
        rep = self.run_gate(early)
        self.assertNotIn('feet_settle', self.checks(self.win(rep, 'mount')))
        # parameter: proxy offset 0.0 makes the early motion count
        rep = self.run_gate(early, cfg={'clip_end_offset': 0.0})
        self.assertIn('feet_settle', self.checks(self.win(rep, 'mount')))


class ExitChecks(Base):
    def _crouch(self, hips_z, knee):
        # the unmount is at 6.0 s; the collapsed pose lasts 4 frames from there
        return with_override(clean_pose, 6.0, 6.0 + 4 * FRAME_S,
                             lambda t: Pose((0, 0, hips_z), {'LeftUpLeg': knee, 'RightUpLeg': knee, 'LeftLeg': knee}))

    def test_exit_crouch_flagged(self):
        rep = self.run_gate(self._crouch(0.35, 110.0))
        w = self.win(rep, 'exit')
        self.assertTrue({'hips_step', 'bone_rot', 'hips_height'} <= self.checks(w), self.checks(w))
        self.assertAlmostEqual(w['stats']['hips_min_height_m'], 0.35, places=2)

    def test_hips_height_boundary(self):
        # height below 0.90 only inside [unmount - 0.1, unmount + 0.6] counts; 5.0..5.1 is 1.0 s before the unmount
        early = with_override(clean_pose, 4.5, 4.7, lambda t: Pose((0, 0, 0.7), {}))
        self.assertNotIn('hips_height', self.checks(self.win(self.run_gate(early), 'exit')))
        low = with_override(clean_pose, 5.95, 6.0, lambda t: Pose((0, 0, 0.89), {}))     # unmount at 6.0
        # smooth enough: clean_pose already stands at 1.0 here, drop by 11 cm exceeds nothing but the height rule
        rep = self.run_gate(low, cfg={'exit_hips_cm': 50.0})
        self.assertIn('hips_height', self.checks(self.win(rep, 'exit')))
        ok = with_override(clean_pose, 5.95, 6.0, lambda t: Pose((0, 0, 0.91), {}))
        rep = self.run_gate(ok, cfg={'exit_hips_cm': 50.0})
        self.assertNotIn('hips_height', self.checks(self.win(rep, 'exit')))

    def test_exit_bone_step_boundary(self):
        for step, expect in ((20.0, True), (14.0, False)):
            pose = with_override(clean_pose, 4.5, 4.5 + FRAME_S / 2,
                                 lambda t, s=step: Pose((0, 0, 0.5 + 0.5 * 0.5 / 1.5), {'LeftUpLeg': 40.0 * (1 - 0.5 / 1.5) + s,
                                                                                         'RightUpLeg': 40.0 * (1 - 0.5 / 1.5),
                                                                                         'LeftLeg': 20.0 * (1 - 0.5 / 1.5)}))
            rep = self.run_gate(pose)
            self.assertEqual('bone_rot' in self.checks(self.win(rep, 'exit')), expect, step)

    def test_arm_limit_vanilla_forearm_key(self):
        # vanilla exit_l RightForeArm: 19.4 deg per 1/60 s (one 30 fps key), shown in game as 25.6 in one frame: arms 30
        def arm(deg):
            return with_override(clean_pose, 4.5, 4.5 + FRAME_S / 2,
                                 lambda t: Pose(clean_pose(t).hips, dict(clean_pose(t).rot, RightForeArm=deg)))
        self.assertNotIn('bone_rot', self.checks(self.win(self.run_gate(arm(25.0)), 'exit')))
        rep = self.run_gate(arm(35.0))
        self.assertEqual({v['bone'] for v in self.win(rep, 'exit')['violations'] if v['check'] == 'bone_rot'}, {'RightForeArm'})
        # the spine keeps 15
        spine = with_override(clean_pose, 4.5, 4.5 + FRAME_S / 2,
                              lambda t: Pose(clean_pose(t).hips, dict(clean_pose(t).rot, Spine1=20.0)))
        self.assertIn('bone_rot', self.checks(self.win(self.run_gate(spine), 'exit')))

    def test_stall_then_catch_up_is_one_step(self):
        # stand-up at 3 cm/frame; one frame repeats the previous pose (the animation did not advance), the next one catches up
        # with a 6 cm step: measured over the two frame gaps (3 cm per 1/60 s), no violation
        def rise(t):
            return Pose((0, 0, 0.5 + 1.8 * max(0.0, min(t - 4.2, 0.5 / 1.8))), {})
        t_stall = min(frames(), key=lambda x: abs(x - 4.3))
        stalled = with_override(clean_pose, 4.0, 9.0, lambda t: rise(t - FRAME_S) if abs(t - t_stall) < 1e-6 else rise(t))
        rep = self.run_gate(stalled)
        e = self.win(rep, 'exit')
        self.assertNotIn('hips_step', self.checks(e), e['violations'][:3])
        self.assertTrue(any('stalled' in n for n in e['notes']), e['notes'])
        # a held pose that then jumps (not moving into the hold) is no stall: the jump stays a violation
        held = with_override(clean_pose, 4.0, 9.0, lambda t: Pose((0, 0, 0.5 if t < 4.3 else (0.56 if t < 4.5 else 1.0)), {}))
        self.assertIn('hips_step', self.checks(self.win(self.run_gate(held), 'exit')))
        # a 3.5 cm per frame stand-up (vanilla exit_r maximum 3.49) passes, a 5 cm one fails
        for rate, expect in ((0.035, False), (0.05, True)):
            fast = with_override(clean_pose, 4.0, 9.0,
                                 lambda t, r=rate: Pose((0, 0, 0.5 + r * 60.0 * max(0.0, min(t - 4.2, 0.5 / (r * 60.0)))), {}))
            self.assertEqual('hips_step' in self.checks(self.win(self.run_gate(fast), 'exit')), expect, rate)

    def test_duplicate_sample_is_ignored(self):
        # a 0.3 ms duplicate with a collapsed pose: nearer-to-predecessor member is kept, no violation
        t_dup = 6.5
        bad = Pose((0, 0, 0.17), {'LeftUpLeg': 90.0})
        times = frames()
        build_trace(self.trace, clean_pose, times, dup_pose=bad, dup_at=min(times, key=lambda x: abs(x - t_dup)))
        build_log(self.log)
        rep = bg.run_gate(self.log, self.trace)
        self.assertEqual(rep['trace_info']['duplicates_dropped'], 1)
        self.assertTrue(rep['pass'], bg.format_report(rep))
        # when the earlier member is the bad one the later (good) one is kept
        build_trace(self.trace, lambda t: bad if abs(t - t_dup) < 1e-3 else clean_pose(t), times,
                    dup_pose=clean_pose(t_dup), dup_at=min(times, key=lambda x: abs(x - t_dup)))
        rep = bg.run_gate(self.log, self.trace)
        self.assertEqual(rep['trace_info']['duplicates_dropped'], 1)
        self.assertTrue(rep['pass'], bg.format_report(rep))


class DtScale(Base):
    def test_long_frame_is_scaled_short_frame_is_not(self):
        # a 10 deg step spread over a 3-frame gap passes; the same step in one regular frame fails
        times = [t for t in frames() if not (4.4 < t < 4.5)]
        pose = with_override(clean_pose, 4.45, 9.0, lambda t: Pose(clean_pose(t).hips, dict(clean_pose(t).rot, LeftUpLeg=clean_pose(t).rot['LeftUpLeg'] + 40.0)))
        rep = self.run_gate(pose, times=times)
        self.assertNotIn('bone_rot', self.checks(self.win(rep, 'exit')))           # 40 deg over ~0.1 s gap, scaled
        rep = self.run_gate(pose, times=times, scale_on=False)
        self.assertIn('bone_rot', self.checks(self.win(rep, 'exit')))
        # short dt is not amplified: 9 ms sample with a 7 deg step stays 7 deg (< 8)
        short = [t for t in frames() if abs(t - 1.5) > 1e-6] + [1.5 - 0.009 + 0.0]
        short.sort()
        motionpose = with_override(clean_pose, 1.49, 9.0, lambda t: Pose(clean_pose(t).hips, dict(clean_pose(t).rot, LeftUpLeg=clean_pose(t).rot['LeftUpLeg'] + 7.0)))
        rep = self.run_gate(motionpose, times=short)
        self.assertNotIn('leg_rot', self.checks(self.win(rep, 'mount')))


def build_feed(path, ent_fn, t0=0.0, t1=8.0):
    """Feed file: game clock = the scenario time (the log's CET 't=' values); ent_fn(t) -> (x, y, yawDeg)."""
    rows = ['# cols t dt vx vy vz yawDeg capX capY capZ psm']
    for t in frames(t0, t1):
        x, y, yaw = ent_fn(t)
        rows.append('%.6f 0.0166 0 0 0 %.5f %.5f %.5f 0 1' % (t, yaw, x, y))
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(rows) + '\n')


def smooth(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3 - 2 * u)


class WorldSpace(Base):
    """A standing entry: the entity slides 1.0 m in 0.25 s and the root offset holds the body (model Hips -x); then the offset
    goes back over back_s seconds while the entity stays."""

    def _scenario(self, back_s):
        def ent(t):
            return (100.0 + 1.0 * smooth((t - 1.05) / 0.25), 50.0, 0.0)

        def pose(t):
            if t > 2.7:
                return clean_pose(t)
            hold = -1.0 * smooth((t - 1.05) / 0.25)
            back = 1.0 * smooth((t - 1.55) / back_s)
            return Pose((hold + back, 0.0, 0.75), {'LeftUpLeg': 20.0, 'RightUpLeg': 20.0})
        self.feed = os.path.join(self.tmp.name, 'utpp-feed.txt')
        build_feed(self.feed, ent)
        return pose

    def test_compensated_slide_is_still_in_the_world(self):
        pose = self._scenario(0.7)
        rep = self.run_gate(pose)                                    # model space: the 1.0 m hold reads as a Hips slide (10 cm/frame)
        self.assertIn('hips_step', self.checks(self.win(rep, 'mount')))
        self.assertTrue(any('model space' in n for n in self.win(rep, 'mount')['notes']))
        rep = self.run_gate(pose, feed_path=self.feed)
        m = self.win(rep, 'mount')
        self.assertEqual(m['stats']['space'], 'world')
        self.assertNotIn('hips_step', self.checks(m), m['violations'][:3])
        self.assertNotIn('foot_skate', self.checks(m))

    def test_fast_offset_return_skates_both_feet(self):
        rep = self.run_gate(self._scenario(0.15), feed_path=self.feed)   # 1.0 m in 0.15 s: up to 17 cm per frame under both feet
        m = self.win(rep, 'mount')
        self.assertIn('foot_skate', self.checks(m), m['violations'][:3])
        self.assertIn('hips_step', self.checks(m))

    def test_feed_cli_and_missing_anchor(self):
        pose = self._scenario(0.7)
        build_trace(self.trace, pose, frames())
        build_log(self.log)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = bg.main(['--log', self.log, '--trace', self.trace, '--feed', self.feed])
        self.assertEqual(rc, 0, out.getvalue())
        self.assertIn('feed=', out.getvalue())
        build_log(self.log, drop=('vehicle idle -> mount',))
        rep = bg.run_gate(self.log, self.trace, feed_path=self.feed)
        m = self.win(rep, 'mount')
        self.assertEqual(m['stats']['space'], 'model')
        self.assertTrue(any('n/a' in n for n in m['notes']))


class LogChecks(Base):
    def test_exit_log_events_inside_and_outside_window(self):
        extra = [(4.4, 'CET [utpp_body] t=4.4 head slot tpp -> empty (exit, 0.47 s into the window)'),
                 (4.5, 'CET [utpp_body] representation was deactivated by the game - activating again'),
                 (9.0, 'CET [utpp_body] t=9 head slot tpp -> empty (later)')]
        rep = self.run_gate(log_kw={'extra': extra})
        e = self.win(rep, 'exit')
        self.assertEqual([v['check'] for v in e['violations'] if v['check'] in ('head_slot_tpp_empty', 'rep_deactivated_by_game')],
                         ['head_slot_tpp_empty', 'rep_deactivated_by_game'])
        self.assertEqual(len([v for v in e['violations'] if v['check'] == 'head_slot_tpp_empty']), 1)
        self.assertEqual(self.win(rep, 'mount')['pass'], True)

    def test_exitdiag_ramp(self):
        ramp = [(6.001, 'EXITDIAG +0.00s gate=0x0 loco=1 tpp=1 hold=0(ok) motion=run w=<1 mask=0x4 exitLeft=2.00'),
                (6.251, 'EXITDIAG +0.25s gate=0x0 loco=1 tpp=1 hold=0(ok) motion=run w=1 mask=0x4 exitLeft=1.75 end')]
        rep = self.run_gate(log_kw={'extra': ramp})
        self.assertIn('exit_weight_ramp', self.checks(self.win(rep, 'exit')))
        self.assertTrue(any('n/a' in n for n in self.win(rep, 'exit')['notes']))
        noramp = [(6.001, 'EXITDIAG +0.00s gate=0x0 loco=1 w=1 mask=0x4 exitLeft=2.00'),
                  (6.251, 'EXITDIAG +0.25s gate=0x0 loco=1 w=1 mask=0x4 exitLeft=1.75 end')]
        rep = self.run_gate(log_kw={'extra': noramp})
        self.assertNotIn('exit_weight_ramp', self.checks(self.win(rep, 'exit')))

    def test_exitdiag_fields(self):
        def end(fields):
            return [(6.251, 'EXITDIAG +0.25s w=1 mask=0x4 exitLeft=1.75 %s end' % fields)]
        rep = self.run_gate(log_kw={'extra': end('exits=1 exitsHanded=1 noCapture=0')})
        self.assertEqual(self.checks(self.win(rep, 'exit')), set())
        rep = self.run_gate(log_kw={'extra': end('exits=1 exitsHanded=0 noCapture=1')})
        self.assertEqual(self.checks(self.win(rep, 'exit')), {'exitdiag_exits_handed', 'exitdiag_no_capture'})

    def test_legs_dropped_by_cap(self):
        line = (1.6, 'vehtrans mount: ride=1 holdWrites=2 offsetWrites=10 rootOnlyWrites=0 legsDroppedByCap=%d mountTpp=1')
        rep = self.run_gate(log_kw={'extra': [(line[0], line[1] % 2)]})
        self.assertIn('legs_dropped_by_cap', self.checks(self.win(rep, 'mount')))
        rep = self.run_gate(log_kw={'extra': [(line[0], line[1] % 0)]})
        self.assertNotIn('legs_dropped_by_cap', self.checks(self.win(rep, 'mount')))
        rep = self.run_gate(log_kw={'extra': [(1.6, 'vehtrans mount: ride=1 holdWrites=2 mountTpp=1')]})
        self.assertTrue(any('n/a' in n for n in self.win(rep, 'mount')['notes']))


class Cli(Base):
    def _run(self, *argv):
        out = io.StringIO()
        err = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = bg.main(list(argv))
        return rc, out.getvalue(), err.getvalue()

    def test_exit_codes_and_json(self):
        build_trace(self.trace, clean_pose, frames())
        build_log(self.log)
        rc, out, _ = self._run('--log', self.log, '--trace', self.trace)
        self.assertEqual(rc, 0, out)
        self.assertIn('RESULT: PASS', out)
        bad = with_override(clean_pose, 6.0, 6.0 + 4 * FRAME_S, lambda t: Pose((0, 0, 0.35), {'LeftLeg': 110.0}))
        build_trace(self.trace, bad, frames())
        rc, out, _ = self._run('--log', self.log, '--trace', self.trace, '--json', '-')
        self.assertEqual(rc, 1)
        data = json.loads(out)
        self.assertFalse(data['pass'])
        self.assertGreater(data['violation_count'], 0)
        self.assertEqual({w['kind'] for w in data['windows']}, {'mount', 'exit'})
        path = os.path.join(self.tmp.name, 'r.json')
        rc, out, _ = self._run('--log', self.log, '--trace', self.trace, '--json', path)
        self.assertEqual(rc, 1)
        self.assertIn('RESULT: FAIL', out)
        with open(path, encoding='utf-8') as f:
            self.assertFalse(json.load(f)['pass'])

    def test_threshold_option_and_missing_files(self):
        build_trace(self.trace, clean_pose, frames())
        build_log(self.log)
        rc, _, _ = self._run('--log', self.log, '--trace', self.trace, '--exit-hips-min-m', '1.5')
        self.assertEqual(rc, 1)
        rc, _, err = self._run('--log', os.path.join(self.tmp.name, 'nope.log'))
        self.assertEqual(rc, 2)
        self.assertIn('not found', err)

    def test_no_window_is_an_error_unless_allowed(self):
        build_log(self.log, mount=None, exit_=None)
        rc, _, err = self._run('--log', self.log)
        self.assertEqual(rc, 2)
        rc, _, _ = self._run('--log', self.log, '--allow-empty')
        self.assertEqual(rc, 0)

    def test_log_only_mode_runs_log_checks(self):
        build_log(self.log, extra=[(4.4, 'CET [utpp_body] t=4.4 head slot tpp -> empty (exit)')])
        rc, out, _ = self._run('--log', self.log)
        self.assertEqual(rc, 1)
        self.assertIn('head_slot_tpp_empty', out)


# ------------------------------------------------------------------------------------------------ real session

REAL_LOG_CANDIDATES = [
    os.path.join(HERE, '..', '..', '..', 'Research', 'logs', 'game', 'utpp-2026-10-28-19-54-19.log'),
    os.path.join(_GAME_LOGS, 'utpp-2026-10-28-19-54-19.log'),
]
REAL_TRACE_CANDIDATES = [
    os.path.join(HERE, '..', '..', '..', 'Research', 'logs', 'game', 'utpp-bodytrace-2026-10-28-19-54-46.csv'),
    os.path.join(_GAME_LOGS, 'utpp-bodytrace-2026-10-28-19-54-46.csv'),
]


def _first_existing(paths):
    for p in paths:
        if os.path.isfile(p):
            return os.path.abspath(p)
    return None


REAL_LOG = _first_existing(REAL_LOG_CANDIDATES)
REAL_TRACE = _first_existing(REAL_TRACE_CANDIDATES)


@unittest.skipUnless(REAL_LOG and REAL_TRACE, 'the 2026-10-28 19:54 session files are not available')
class Real_20261028_1954(unittest.TestCase):
    """The known defects of the build tested that evening must stay flagged (this is the red baseline of the gate)."""

    @classmethod
    def setUpClass(cls):
        cls.rep = bg.run_gate(REAL_LOG, REAL_TRACE)

    def vs(self, wid, check):
        w = [w for w in self.rep['windows'] if w['id'] == wid][0]
        return [v for v in w['violations'] if v['check'] == check]

    def test_four_windows_and_red(self):
        self.assertEqual([w['id'] for w in self.rep['windows']], ['ride 1', 'exit 1', 'ride 2', 'exit 2'])
        self.assertFalse(self.rep['pass'])
        self.assertEqual(self.rep['trace_info']['duplicates_dropped'], 1)

    def test_mount_leg_pop_and_frozen_frames(self):
        pop = [v for v in self.vs('ride 1', 'leg_rot') if v['time'] == '19:55:07.853']
        self.assertEqual(len(pop), 1)
        self.assertGreater(pop[0]['value'], 35.0)
        fr = self.vs('ride 1', 'frozen_legs')
        self.assertEqual(len(fr), 1)
        self.assertGreaterEqual(fr[0]['value'], 15)
        self.assertEqual(fr[0]['time'], '19:55:07.853')
        pop2 = [v for v in self.vs('ride 2', 'leg_rot') if v['time'].startswith('19:55:22.33')]
        self.assertTrue(pop2 and pop2[0]['value'] > 30.0)
        self.assertGreaterEqual(self.vs('ride 2', 'frozen_legs')[0]['value'], 15)

    def test_exit_flash(self):
        steps = {v['time']: v['value'] for v in self.vs('exit 1', 'hips_step')}
        self.assertGreater(steps['19:55:15.804'], 20.0)
        self.assertGreater(steps['19:55:15.902'], 20.0)
        self.assertEqual(len(self.vs('exit 1', 'head_slot_tpp_empty')), 1)
        self.assertEqual(len(self.vs('exit 1', 'rep_deactivated_by_game')), 1)
        self.assertTrue(self.vs('exit 2', 'head_slot_tpp_empty'))

    def test_mount_end_leg_swap(self):
        # D7 / M4: the left foot drops from the peg after the clip end
        self.assertTrue(self.vs('ride 1', 'feet_settle'))
        self.assertTrue(self.vs('ride 2', 'feet_settle'))

    def test_exit_crouch(self):
        steps = {v['time']: v['value'] for v in self.vs('exit 1', 'hips_step')}
        self.assertGreater(steps['19:55:17.267'], 60.0)
        low = self.vs('exit 1', 'hips_height')
        self.assertTrue(low and min(v['value'] for v in low) < 0.4)
        self.assertTrue(any(v['value'] > 100.0 for v in self.vs('exit 1', 'bone_rot')))
        self.assertEqual(len(self.vs('exit 1', 'exit_weight_ramp')), 1)
        self.assertTrue(self.vs('exit 2', 'hips_height'))
        self.assertEqual(len(self.vs('exit 2', 'exit_weight_ramp')), 1)


_C9_NAMES = ['utpp-2026-10-28-23-04-14.log', 'utpp-bodytrace-2026-10-28-23-04-46-2.csv', 'utpp-feed-2026-10-28-23-04-36.txt']
REAL_C9 = [_first_existing([os.path.join(HERE, '..', '..', '..', 'Research', 'logs', 'game', n),
                            os.path.join(_GAME_LOGS, n)]) for n in _C9_NAMES]


@unittest.skipUnless(all(REAL_C9), 'the 2026-10-28 23:05 session files are not available')
class Real_20261028_2305(unittest.TestCase):
    """c9 build: the exits pass (their stand-up and arm keys are vanilla clip motion); the mounts keep the classified defects."""

    @classmethod
    def setUpClass(cls):
        cls.rep = bg.run_gate(REAL_C9[0], REAL_C9[1], feed_path=REAL_C9[2])

    def w(self, wid):
        return [w for w in self.rep['windows'] if w['id'] == wid][0]

    def test_exits_pass(self):
        self.assertTrue(self.w('exit 1')['pass'], bg.format_report(self.rep))
        self.assertTrue(self.w('exit 2')['pass'])

    def test_ride1_leg_swap_after_the_clip(self):
        c = {v['check']: v for v in self.w('ride 1')['violations']}
        self.assertIn('feet_settle', c)
        self.assertEqual(c['leg_acc']['time'], '23:05:08.980')
        self.assertNotIn('leg_rot', c)
        self.assertNotIn('foot_skate', c)
        self.assertNotIn('hips_step', c)

    def test_ride2_skate_and_leg_spike(self):
        vs = self.w('ride 2')['violations']
        sk = [v for v in vs if v['check'] == 'foot_skate']
        self.assertTrue(sk and all('23:05:22.2' <= v['time'] <= '23:05:22.45' for v in sk), sk)
        self.assertTrue(any(v['check'] == 'leg_acc' and v['time'] == '23:05:22.309' for v in vs))


# ------------------------------------------------------------------------------------------------ ride edge (neck pop)

EDGE = 2.0            # 'vehicle mount -> ride' of build_log's default mount


def edge_pose(rot=None, trans=None):
    """clean_pose plus Head / Spine rotation or translation overrides (functions of t returning dicts)."""
    def fn(t):
        p = clean_pose(t)
        r = dict(p.rot)
        tr = {}
        if rot:
            r.update(rot(t))
        if trans:
            tr.update(trans(t))
        return Pose(p.hips, r, tr)
    return fn


def head_step_rot(t0, deg):
    """Head local rotation 0 before t0, deg from t0 on (one step of deg at the first frame at/after t0)."""
    return lambda t: {'Head': deg if t >= t0 - 1e-9 else 0.0}


def head_offset(fn):
    """Head local translation = rig default + fn(t) = (dx, dz)."""
    def f(t):
        dx, dz = fn(t)
        return {'Head': (dx, 0.0, 0.3 + dz)}
    return f


def bump(t_start, rise, peak):
    """Triangle in t: 0 at t_start, peak at t_start + rise, 0 at t_start + 2 rise."""
    def f(t):
        u = t - t_start
        if u <= 0 or u >= 2 * rise:
            return 0.0
        return peak * (u / rise if u < rise else (2 * rise - u) / rise)
    return f


class RideEdgeChecks(Base):
    RIDE = ('ride_neck_rot', 'ride_head_hips', 'ride_head_line')

    def ride_checks(self, rep):
        return {v['check'] for v in self.win(rep, 'mount')['violations'] if v['check'] in self.RIDE}

    def test_clean_session_has_no_ride_violations_and_reports_stats(self):
        rep = self.run_gate()
        self.assertEqual(self.ride_checks(rep), set())
        st = self.win(rep, 'mount')['stats']
        for k in ('ride_neck_max_deg', 'ride_head_hips_max_cm', 'ride_head_line_max_cm', 'edge_frames'):
            self.assertIn(k, st)
        self.assertGreater(st['edge_frames'], 50)

    def test_head_rotation_step_boundary(self):
        rep = self.run_gate(edge_pose(rot=head_step_rot(EDGE + 0.1, 7.9)))
        self.assertNotIn('ride_neck_rot', self.ride_checks(rep))
        rep = self.run_gate(edge_pose(rot=head_step_rot(EDGE + 0.1, 8.5)))
        w = self.win(rep, 'mount')
        v = [x for x in w['violations'] if x['check'] == 'ride_neck_rot']
        self.assertEqual(len(v), 1, v)
        self.assertEqual(v[0]['bone'], 'Head')
        self.assertAlmostEqual(v[0]['value'], 8.5, delta=0.05)
        self.assertFalse(w['pass'])

    def test_head_vs_hips_step_boundary(self):
        rep = self.run_gate(edge_pose(trans=head_offset(lambda t: (0.0, 0.045 if t >= EDGE else 0.0))))
        self.assertEqual(self.ride_checks(rep), set())
        rep = self.run_gate(edge_pose(trans=head_offset(lambda t: (0.0, 0.06 if t >= EDGE else 0.0))))
        self.assertIn('ride_head_hips', self.ride_checks(rep))

    def test_head_off_the_straight_line_boundary(self):
        # slow triangle bump (about 1.3 cm per frame): only the line deviation sees it; the peak sits at edge + 0.1 s
        rep = self.run_gate(edge_pose(trans=head_offset(lambda t: (bump(EDGE, 0.1, 0.049)(t), 0.0))))
        self.assertEqual(self.ride_checks(rep), set())
        rep = self.run_gate(edge_pose(trans=head_offset(lambda t: (bump(EDGE, 0.1, 0.051)(t), 0.0))))
        self.assertEqual(self.ride_checks(rep), {'ride_head_line'})

    def test_one_frame_pop_that_decays_trips_all_three(self):
        def rot(t):
            k = (t - EDGE) / 0.23
            on = 0.0 <= k < 1.0
            return {'Head': 30.0 * (1.0 - k) if on else 0.0, 'Spine1': 12.0 if on else 0.0}
        rep = self.run_gate(edge_pose(rot=rot, trans=head_offset(
            lambda t: (0.28 * max(0.0, 1 - (t - EDGE) / 0.23) if t >= EDGE else 0.0, 0.0))))
        self.assertEqual(self.ride_checks(rep), set(self.RIDE))

    def test_only_the_edge_window_is_checked(self):
        def pop_at(t_pop):
            return lambda t: {'Head': 40.0 if abs(t - t_pop) < 0.5 * FRAME_S else 0.0}
        for t_pop in (EDGE - 0.55, EDGE + 0.55):
            rep = self.run_gate(edge_pose(rot=pop_at(t_pop)))
            self.assertEqual(self.ride_checks(rep), set(), t_pop)
        rep = self.run_gate(edge_pose(rot=pop_at(EDGE + 0.45)))
        self.assertIn('ride_neck_rot', self.ride_checks(rep))

    def test_stage_selects_the_trace_columns(self):
        pop = edge_pose(rot=head_step_rot(EDGE + 0.1, 30.0))
        build_trace(self.trace, pop, frames(), pose0_fn=clean_pose)
        build_log(self.log)
        self.assertIn('ride_neck_rot', self.ride_checks(bg.run_gate(self.log, self.trace, stage='s1')))
        self.assertEqual(self.ride_checks(bg.run_gate(self.log, self.trace, stage='s0')), set())
        build_trace(self.trace, clean_pose, frames(), pose0_fn=pop)
        self.assertIn('ride_neck_rot', self.ride_checks(bg.run_gate(self.log, self.trace, stage='s0')))

    def test_thresholds_are_configurable(self):
        rep = self.run_gate(edge_pose(rot=head_step_rot(EDGE + 0.1, 30.0)), cfg={'ride_neck_deg': 40.0})
        self.assertNotIn('ride_neck_rot', self.ride_checks(rep))
        rep = self.run_gate(edge_pose(rot=head_step_rot(EDGE + 0.1, 5.0)), cfg={'ride_neck_deg': 4.0})
        self.assertIn('ride_neck_rot', self.ride_checks(rep))

    def test_cli_options(self):
        build_trace(self.trace, edge_pose(rot=head_step_rot(EDGE + 0.1, 30.0)), frames())
        build_log(self.log)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = bg.main(['--log', self.log, '--trace', self.trace, '--stage', 's1'])
        self.assertEqual(rc, 1)
        self.assertIn('ride_neck_rot', out.getvalue())
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = bg.main(['--log', self.log, '--trace', self.trace, '--ride-neck-deg', '40'])
        self.assertEqual(rc, 0, out.getvalue())

    def test_hole_around_the_edge_is_not_a_silent_pass(self):
        times = [t for t in frames() if not (1.8 < t < 2.45)]
        rep = self.run_gate(times=times)
        cov = [v for v in self.win(rep, 'mount')['violations'] if v['check'] == 'trace_coverage']
        self.assertTrue(any('head line n/a' in v.get('detail', '') for v in cov), cov)
        rep = self.run_gate(times=times, allow_uncovered=True)
        self.assertNotIn('trace_coverage', self.checks(self.win(rep, 'mount')))


# ------------------------------------------------------------------------------------------------ real sessions, ride edge

REAL_RIDE_SESSIONS = {
    '01:29': ('utpp-2026-10-29-01-28-54.log', 'utpp-bodytrace-2026-10-29-01-29-21.csv'),
    '00:29': ('utpp-2026-10-29-00-29-16.log', 'utpp-bodytrace-2026-10-29-00-29-45.csv'),
    '23:04': ('utpp-2026-10-28-23-04-14.log', 'utpp-bodytrace-2026-10-28-23-04-46-2.csv'),
    '19:54': ('utpp-2026-10-28-19-54-19.log', 'utpp-bodytrace-2026-10-28-19-54-46.csv'),
}
REAL_DIRS = [os.path.join(HERE, '..', '..', '..', 'Research', 'logs', 'game'),
             _GAME_LOGS]


def _real_pair(names):
    for d in REAL_DIRS:
        ps = [os.path.join(d, n) for n in names]
        if all(os.path.isfile(p) for p in ps):
            return [os.path.abspath(p) for p in ps]
    return None


class Real_RideEdge(unittest.TestCase):
    """Ride 1 of four recorded sessions carries the neck / Head pop at 'mount -> ride'; ride 2 does not (s0 and s1)."""

    def test_ride1_flagged_ride2_clean(self):
        seen = 0
        for name, files in REAL_RIDE_SESSIONS.items():
            pair = _real_pair(files)
            if pair is None:
                continue
            seen += 1
            for stage in ('s0', 's1'):
                rep = bg.run_gate(pair[0], pair[1], stage=stage)
                for wid, want in (('ride 1', True), ('ride 2', False)):
                    w = [x for x in rep['windows'] if x['id'] == wid][0]
                    got = {v['check'] for v in w['violations'] if v['check'].startswith('ride_')}
                    if want:
                        self.assertEqual(got, {'ride_neck_rot', 'ride_head_hips', 'ride_head_line'}, (name, stage, wid))
                    else:
                        self.assertEqual(got, set(), (name, stage, wid))
        if not seen:
            self.skipTest('the 2026-10-28 / 29 session files are not available')


if __name__ == '__main__':
    unittest.main()
