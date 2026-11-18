"""Tests of world_slide: a synthetic planted foot whose world slide is known, the frame-pair rejections, and the contact signal
taken from the motion database (clip time interpolation, loop wrap, time jumps, the crossfade stack after a clip change)."""
from __future__ import annotations

import datetime
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import world_slide as ws  # noqa: E402

DT = 1.0 / 60.0


def planted_session(slip_cm_s=0.0, n=240, teleport_at=None, contact=None, clip='Walk_Loop', feed_dt_scale=1.0):
    """V walks forward at 1.5 m/s while turning 45 deg/s; the left toe is planted in the world at (2, 3) and slides along +x
    at slip_cm_s. The pose of frame f is drawn at the capsule of feed row f (rows[f] = f - 1, FEED_SHIFT 1)."""
    fsec = 100.0 + np.arange(n) * DT
    yaw = 10.0 + 45.0 * (fsec - fsec[0])
    capx = 1.5 * (fsec - fsec[0])
    capy = 0.2 * np.sin(fsec)
    if teleport_at is not None:
        capx[teleport_at:] += 50.0
    world = np.stack([2.0 + slip_cm_s / 100.0 * (fsec - fsec[0]), np.full(n, 3.0)], axis=1)
    if teleport_at is not None:
        world[teleport_at:, 0] += 50.0                 # the game moved V and the planted foot with it
    a = np.radians(yaw)
    dx, dy = world[:, 0] - capx, world[:, 1] - capy
    toe_l = np.stack([np.cos(a) * dx + np.sin(a) * dy, -np.sin(a) * dx + np.cos(a) * dy, np.zeros(n)], axis=1)
    toe_r = toe_l + np.array([0.3, 0.0, 0.0])          # the right foot is in the air (contact 0)
    fdt = DT * feed_dt_scale
    feed = dict(dt=np.full(n + 1, fdt), capX=np.concatenate([capx, [capx[-1]]]), capY=np.concatenate([capy, [capy[-1]]]),
                yawDeg=np.concatenate([yaw, [yaw[-1]]]), t=np.arange(n + 1) * fdt)
    rows = np.arange(n) - 1
    cont = np.zeros((n, 2))
    cont[:, 0] = 1.0 if contact is None else contact
    state = np.full(n, ws.ST_OK, dtype=np.int8)
    clip_of = np.zeros(n, dtype=np.int64)
    ses = ws.Session(fsec, [toe_l, toe_r], feed, rows, cont, state, clip_of, [clip])
    ses.choose_shift(ws.FEED_SHIFT)
    return ses


class WorldSlideTests(unittest.TestCase):
    def test_a_perfect_plant_has_no_slide_while_the_capsule_moves_and_turns(self):
        ses = planted_session(0.0)
        smp, locks = ses.samples()
        self.assertGreater(len(smp['speed']), 200)
        self.assertLess(float(np.max(smp['speed'])), 0.01)
        self.assertEqual(len(locks), 1)
        self.assertLess(locks[0]['cm'], 0.01)
        self.assertTrue(set(smp['side'].tolist()) == {0})   # the airborne foot gives no sample

    def test_a_known_slip_is_recovered(self):
        ses = planted_session(20.0)
        smp, locks = ses.samples()
        self.assertAlmostEqual(float(np.mean(smp['speed'])), 20.0, delta=0.05)
        self.assertAlmostEqual(float(np.percentile(smp['speed'], 95)), 20.0, delta=0.05)
        L = locks[0]
        dur = (L['last'] - L['first']) * DT
        self.assertAlmostEqual(L['cm'], 20.0 * dur, delta=0.05)
        self.assertAlmostEqual(L['net_cm'], 20.0 * dur, delta=0.05)
        w = ws.window_stats(ses, smp, locks, 0, len(ses.fsec))
        self.assertAlmostEqual(w['speed_p95'], 20.0, delta=0.05)
        self.assertEqual(set(w['by_class']), {'loop'})
        self.assertIsNone(w['rejected'])

    def test_the_distance_is_divided_by_the_game_step_of_the_feed_row(self):
        ses = planted_session(20.0, feed_dt_scale=1.1)      # feed dt 1.67 ms longer than the pose step: still matched (< 3 ms)
        smp, _ = ses.samples()
        self.assertGreater(len(smp['speed']), 200)
        self.assertAlmostEqual(float(np.median(smp['speed'])), 20.0 / 1.1, delta=0.05)

    def test_the_wrong_feed_shift_shows_as_slide_and_is_scored_worse(self):
        ses = planted_session(0.0)
        scores = {int(k): v[0] for k, v in ses.shift_scores.items() if v[0] is not None}
        self.assertEqual(min(scores, key=scores.get), ws.FEED_SHIFT)
        self.assertIsNone(ses.shift_flag)
        self.assertLess(scores[ws.FEED_SHIFT], 0.01)
        self.assertGreater(scores[0], 1.0)                    # one frame of capsule motion while turning shows up as slide

    def test_a_placement_by_the_game_is_left_out(self):
        ses = planted_session(0.0, teleport_at=120)
        smp, locks = ses.samples()
        self.assertLess(float(np.max(smp['speed'])), 0.01)
        self.assertGreater(int(ses.teleport.sum()), 0)
        self.assertTrue(all(L['net_cm'] is None or L['net_cm'] < 0.01 for L in locks))

    def test_frames_without_a_feed_row_or_with_a_step_mismatch_are_rejected(self):
        ses = planted_session(20.0)
        ses.rows[100:140] = -1
        ses.aligned = ses.rows >= 0
        ses.feed['dt'][160] = DT + 0.01                     # the feed step of a row does not match the pose step
        ses.__init__(ses.fsec, ses.toe, ses.feed, ses.rows, ses.cont, ses.state, ses.clip_of, ses.clip_names)
        ses.choose_shift(ws.FEED_SHIFT)
        sp, _ = ses.speeds(0)
        self.assertTrue(np.isnan(sp[100:141]).all())
        self.assertTrue(np.isnan(sp[161]))
        w = ws.window_stats(ses, *ses.samples(), 100, 140)
        self.assertIsNotNone(w['rejected'])

    def test_a_pair_drawn_across_a_feed_gap_is_rejected(self):
        ses = planted_session(20.0)
        ses.feed['t'][150:] += 2.0                           # the feed wrote no rows for 2 s before row 150; dt stays one tick
        ses.__init__(ses.fsec, ses.toe, ses.feed, ses.rows, ses.cont, ses.state, ses.clip_of, ses.clip_names)
        ses.choose_shift(ws.FEED_SHIFT)
        sp, _ = ses.speeds(0)
        # pair k is drawn on rows (k - 1, k) and its frames are matched to rows (k - 2, k - 1): the step into row 150 lies in pairs 150, 151
        self.assertTrue(np.isnan(sp[150]) and np.isnan(sp[151]))
        self.assertTrue(np.isfinite(sp[149]) and np.isfinite(sp[152]))
        sp0, _ = ses.speeds(0, 0)                            # unshifted: only pair 151 (rows 149 -> 150) crosses the gap
        self.assertTrue(np.isfinite(sp0[150]) and np.isnan(sp0[151]))

    def test_a_window_with_little_known_contact_is_rejected(self):
        ses = planted_session(20.0)
        ses.state[100:130] = ws.ST_UNKNOWN                   # 30 of 60 frames without a known contact
        w = ws.window_stats(ses, *ses.samples(), 90, 150)
        self.assertIn('contact known', w['rejected'])
        self.assertEqual(w['contact_known_share'], 0.5)
        ses.state[100:130] = ws.ST_NOCONT                    # known: no contact (a dodge clip)
        w = ws.window_stats(ses, *ses.samples(), 90, 150)
        self.assertIsNone(w['rejected'])

    def test_contact_below_the_lock_threshold_and_short_locks_give_no_sample(self):
        c = np.zeros(240)
        c[20:29] = 1.0                                       # 9 frames: shorter than 2 * TRIM + MIN_SEG
        c[60:120] = 0.4                                      # below CONTACT_MIN
        ses = planted_session(20.0, contact=c)
        smp, locks = ses.samples()
        self.assertEqual(len(smp['speed']), 0)
        self.assertEqual(locks, [])
        smp, locks = ses.samples('runtime', ws.SHORT_TRIM, ws.SHORT_MIN_SEG)   # the short-lock rule measures it
        self.assertEqual(smp['frame'].tolist(), list(range(20 + ws.SHORT_TRIM + 1, 29 - ws.SHORT_TRIM)))
        self.assertEqual(len(locks), 1)
        self.assertAlmostEqual(float(np.mean(smp['speed'])), 20.0, delta=0.05)
        v = ws.window_stats(ses, *ses.samples(), 0, 240, ses.variants())
        self.assertEqual(v['short_locks']['locks'], 1)
        self.assertEqual(v['locks'], 0)

    def test_trimmed_lock_frames(self):
        c = np.zeros(240)
        c[50:80] = 1.0
        ses = planted_session(5.0, contact=c)
        smp, locks = ses.samples()
        self.assertEqual(int(smp['frame'].min()), 50 + ws.TRIM + 1)
        self.assertEqual(int(smp['frame'].max()), 80 - ws.TRIM - 1)


class ClassifyTests(unittest.TestCase):
    def test_clip_classes(self):
        for name, cls in (('Walk_Start_F', 'start'), ('Run_Stop_L', 'stop'), ('Run_Turn_135_L', 'pivot'), ('Walk_Reface_90', 'pivot'),
                          ('Walk_Loop_F', 'loop'), ('Jump_Land_Run', 'landing'), ('M_Neutral_Stand_Idle_Loop', 'idle'),
                          ('Crouch_Trans_In', 'other')):
            self.assertEqual(ws.clip_class(name), cls, name)


class LogTests(unittest.TestCase):
    def test_motion_dodge_and_slide_lines(self):
        lines = [
            '[2026-11-18 12:19:17.983] [info] [UTPP] UTPP MOTION t=346.116 db=16:Pivots clip=449:Run_Turn_L ct=1.441 cost=2.619 '
            'blend=0.00(inert) rot=orient start=0 air=mode0,ttl0.00,land00 trav=0:-@0.00 act=0',
            '[2026-11-18 12:19:18.051] [info] [UTPP] UTPP MOTION t=346.184 db=15:Loops clip=363:Run_Loop ct=0.000 cost=0.534 '
            'blend=0.50(live) rot=orient start=0 air=mode1,ttl0.40,land00 trav=0:-@0.00 act=0 chg slide=1/0/0',
            '[2026-11-18 12:19:17.767] [info] [UTPP] UTPP MOTIONDODGE start t=345.900 sector=B push=-145.7',
            '[2026-11-18 12:19:18.367] [info] [UTPP] UTPP MOTIONDODGE end t=346.501 blend=0.30',
            '[2026-11-18 12:19:18.367] [info] [UTPP] UTPP MOTIONPOSE t=346.501 x',
        ]
        ev = ws.parse_motion_lines(lines)
        self.assertEqual([e['kind'] for e in ev], ['motion', 'motion', 'dodge_start', 'dodge_end'])
        self.assertFalse(ev[0]['live'])
        self.assertTrue(ev[1]['live'] and ev[1]['chg'])
        self.assertEqual((ev[1]['air'], ev[1]['slide'], ev[1]['blend']), (1, 1, 0.5))
        self.assertEqual(ev[3]['blend'], 0.3)


class ContactTrackTests(unittest.TestCase):
    T0 = datetime.datetime(2026, 1, 1, 12, 0, 0)
    NAMES = ['Walk_Loop', 'Walk_Stop']

    def db(self):
        # clip 0: loop of 31 frames (1 s at 30 Hz), left contact on the first half; clip 1: one-shot of 11 frames, right contact
        c0 = np.zeros((31, 2))
        c0[:16, 0] = 1.0
        c0[30] = c0[0]
        c1 = np.zeros((11, 2))
        c1[:, 1] = 1.0
        return ws.ContactDB.from_arrays(self.NAMES, [31, 11], [True, False], [c0, c1], fps=30.0)

    def ev(self, s, t, clip, ct, blend=0.2, chg=False):
        return dict(wall=self.T0 + datetime.timedelta(seconds=s), t=t, clip_idx=self.NAMES.index(clip), clip=clip, ct=ct, blend=blend,
                    air=0, trav=0, chg=chg)

    def track(self, events, n=180):
        fsec = np.arange(n) * DT
        return ws.contact_track(self.db(), events, fsec, lambda w: (w - self.T0).total_seconds())

    def test_interpolation_and_loop_wrap(self):
        db = self.db()
        got = db.contact(0, np.array([0.0, 0.25, 0.5, 0.75, 1.25, 2.25]))
        self.assertEqual(got[:, 0].tolist(), [1.0, 1.0, 1.0, 0.0, 1.0, 1.0])
        self.assertAlmostEqual(float(db.contact(0, np.array([0.5 + 1 / 60]))[0, 0]), 0.5)   # half way between frames 15 and 16
        self.assertEqual(db.contact(1, np.array([5.0]))[0].tolist(), [0.0, 1.0])            # a one-shot is clamped at its end

    def test_heartbeat_lines_advance_the_clip_and_a_time_jump_is_marked(self):
        evs = [self.ev(0.0, 0.0, 'Walk_Loop', 0.0), self.ev(0.5, 0.5, 'Walk_Loop', 0.5),
               self.ev(1.0, 1.0, 'Walk_Loop', 0.9), self.ev(1.5, 1.5, 'Walk_Loop', 0.4)]
        # 0.5 -> 0.9 in 0.5 s = rate 0.8: fine; 0.9 -> 0.4 in 0.5 s: with the 1 s loop that is 0.5 s = rate 1.0: fine
        tr = self.track(evs)
        self.assertEqual(tr.notes['ct_jumps'], 0)
        self.assertEqual(int(tr.state[10]), ws.ST_OK)
        evs[2] = self.ev(1.0, 1.0, 'Walk_Loop', 0.52)       # 0.5 -> 0.52 in 0.5 s: rate 0.04, a jump no line shows
        tr = self.track(evs)
        self.assertGreaterEqual(tr.notes['ct_jumps'], 1)
        self.assertEqual(int(tr.state[int(0.6 / DT)]), ws.ST_JUMP)

    def test_a_clip_change_crossfades_the_contact_with_the_smoothstep_of_the_blend_stack(self):
        evs = [self.ev(0.0, 0.0, 'Walk_Loop', 0.0), self.ev(0.5, 0.5, 'Walk_Loop', 0.5),
               self.ev(1.0, 1.0, 'Walk_Stop', 0.0, blend=0.5, chg=True), self.ev(1.5, 1.5, 'Walk_Stop', 0.5)]
        tr = self.track(evs, n=150)
        f = int(round(1.25 / DT))                            # half way through the blend: loop at ct 0.25 (left 1), stop (right 1)
        self.assertEqual(int(tr.state[f]), ws.ST_BLEND)
        self.assertAlmostEqual(float(tr.cont[f, 0]), 0.5, delta=0.01)
        self.assertAlmostEqual(float(tr.cont[f, 1]), 0.5, delta=0.01)
        q = int(round(1.15 / DT))                            # 30 % in: smoothstep 0.216, not the linear 0.3
        self.assertAlmostEqual(float(tr.cont[q, 1]), 0.216, delta=0.002)
        self.assertAlmostEqual(float(tr.cont[q, 0]), 1 - 0.216, delta=0.002)
        x = 0.15 / (0.5 * ws.LEG_FADE_FACTOR)               # the toe of the drawn pose fades in 0.5 / 1.5 s
        self.assertAlmostEqual(float(tr.cont_pose[q, 1]), x * x * (3 - 2 * x), delta=0.005)
        self.assertEqual(tr.cont_cut[q].tolist(), [0.0, 1.0])  # no crossfade: the selected clip alone
        g = int(round(1.6 / DT))
        self.assertEqual(int(tr.state[g]), ws.ST_OK)
        self.assertEqual(tr.cont[g].tolist(), [0.0, 1.0])
        self.assertEqual(tr.notes['blends'], 1)

    def test_an_older_layer_keeps_fading_under_the_next_pick(self):
        evs = [self.ev(0.0, 0.0, 'Walk_Loop', 0.0), self.ev(0.5, 0.5, 'Walk_Loop', 0.5),
               self.ev(1.0, 1.0, 'Walk_Stop', 0.0, blend=0.4, chg=True), self.ev(1.2, 1.2, 'Walk_Loop', 0.0, blend=0.4, chg=True)]
        tr = self.track(evs, n=150)
        f = int(round(1.3 / DT))
        sm = lambda u: min(max(u, 0), 1) ** 2 * (3 - 2 * min(max(u, 0), 1))  # noqa: E731
        a_new = sm(0.1 / 0.4)
        a_stop = sm((0.2 + 0.1 * ws.OLDER_CLOCK ** 2) / 0.4)  # the stop layer is layer 1 now: its clock runs 1.21 x
        old = np.array([1.0, 0.0])                           # the first loop at ct 0.3: left planted
        stop = np.array([0.0, 1.0])
        new = np.array([1.0, 0.0])                           # the second loop at ct 0.1
        mix = old + (stop - old) * a_stop
        mix = mix + (new - mix) * a_new
        self.assertAlmostEqual(float(tr.cont[f, 0]), float(mix[0]), delta=0.01)
        self.assertAlmostEqual(float(tr.cont[f, 1]), float(mix[1]), delta=0.01)
        self.assertEqual(tr.notes['max_layers'], 3)
        evs[3] = self.ev(1.01, 1.01, 'Walk_Loop', 0.0, blend=0.4, chg=True)   # within 0.03 s of the head: it replaces the head
        tr = self.track(evs, n=150)
        self.assertEqual(tr.notes['replaced'], 1)
        self.assertEqual(tr.notes['max_layers'], 2)

    def test_a_short_interval_rate_is_not_kept_and_kept_rates_are_clamped(self):
        evs = [self.ev(0.0, 0.0, 'Walk_Loop', 0.0), self.ev(0.05, 0.05, 'Walk_Loop', 0.13),   # 2.6 over 3 frames
               self.ev(0.5, 0.5, 'Walk_Stop', 0.0, blend=0.0, chg=True)]
        tr = self.track(evs, n=40)
        self.assertEqual(tr.notes['short_rates'], 1)
        # 0.05 .. 0.5 s plays the loop at rate 1 (the last kept rate; none yet): ct = 0.13 + (t - 0.05); left planted until ct 0.5
        # (rate 2.6 kept: 0.78 at 0.3 s, swing; rate 1.15: 0.53 at 0.4 s, swing)
        self.assertEqual(float(tr.cont[int(round(0.30 / DT)), 0]), 1.0)
        self.assertEqual(float(tr.cont[int(round(0.40 / DT)), 0]), 1.0)
        evs = [self.ev(0.0, 0.0, 'Walk_Loop', 0.0), self.ev(0.5, 0.5, 'Walk_Loop', 0.8),
               self.ev(1.0, 1.0, 'Walk_Stop', 0.0, blend=0.5, chg=True)]
        tr = self.track(evs, n=90)
        self.assertEqual(tr.notes['rates_clamped'], 1)       # 1.6 measured, 1.15 kept
        f = int(round(0.42 / DT))                            # inside the interval the measured 1.6: ct 0.672 (left 0)
        self.assertEqual(float(tr.cont[f, 0]), 0.0)
        # after it (next line another clip) the loop plays on at the kept 1.15: frame 40 (0.6667 s) ct 0.9917 = loop frame 29.75,
        # left 0.75 (1.6 would be ct 1.0667: left 1; 1.0 would be 0.9667 = frame 29.0: left 0)
        self.assertAlmostEqual(float(tr.cont[40, 0]), 0.75, delta=0.01)

    def test_dodge_clips_and_slide_lines_have_no_contact(self):
        evs = [self.ev(0.0, 0.0, 'Walk_Loop', 0.0), self.ev(0.5, 0.5, 'Walk_Loop', 0.5)]
        evs.append(dict(kind='dodge_start', wall=self.T0 + datetime.timedelta(seconds=0.1), t=0.1, blend=0.0))
        evs.append(dict(kind='dodge_end', wall=self.T0 + datetime.timedelta(seconds=0.2), t=0.2, blend=0.1))
        tr = self.track(evs, n=60)
        self.assertEqual(tr.notes['dodge_windows'], 1)
        self.assertEqual(int(tr.state[int(round(0.25 / DT))]), ws.ST_NOCONT)
        self.assertEqual(tr.cont[int(round(0.25 / DT))].tolist(), [0.0, 0.0])
        self.assertEqual(int(tr.state[int(round(0.35 / DT))]), ws.ST_OK)
        evs = [dict(self.ev(0.0, 0.0, 'Walk_Loop', 0.0), slide=1)]
        tr = self.track(evs, n=20)
        self.assertEqual(int(tr.state[5]), ws.ST_NOCONT)
        self.assertEqual(tr.cont[5].tolist(), [0.0, 0.0])

    def test_after_a_trace_gap_only_the_trace_period_keeps_the_clip(self):
        evs = [self.ev(0.0, 0.0, 'Walk_Loop', 0.0), self.ev(2.0, 2.0, 'Walk_Loop', 0.0)]
        tr = self.track(evs, n=150)
        self.assertEqual(int(tr.state[int(round(0.45 / DT))]), ws.ST_OK)
        self.assertEqual(int(tr.state[int(round(0.6 / DT))]), ws.ST_UNKNOWN)


if __name__ == '__main__':
    unittest.main()
