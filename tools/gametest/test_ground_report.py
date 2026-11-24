"""ground_report.py: elevator rides (PLATFORM on / off + feed window) and curb crossings (isolated capsule z steps) of one run."""
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ground_report as gr  # noqa: E402

COLS = ('t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step haveTarget targetYawDeg capX capY '
        'capZ effGate holdEligible gateRaw upperBody detailed onGround distGround psmFall walk')


def feed_line(t, x, y, z, clip='walk_fwd', det='-', ground='1', vx=0.0, vy=3.0):
    return (f'{t:.4f} 0.0167 {vx} {vy} 0 0 0 0 0 1 0 0 {clip} 0 0 1.0 - 0 0 {x:.3f} {y:.3f} {z:.4f} 0 0 0 0 {det} {ground} -2 0 0')


def write_run(lines_log, feed_rows):
    d = tempfile.mkdtemp()
    os.makedirs(os.path.join(d, 'logs'))
    with open(os.path.join(d, 'logs', 'utpp-2026-11-22-14-00-00.log'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines_log) + '\n')
    with open(os.path.join(d, 'logs', 'utpp-feed-2026-11-22-14-00-00.txt'), 'w', encoding='utf-8') as f:
        f.write('# cols ' + COLS + '\n' + '\n'.join(feed_rows) + '\n')
    return d


def log(ts, text):
    return f'[2026-11-22 {ts}] [info    ] [  1] [UTPP] {text}'


def walk(t0, n, z_of, clip='walk_fwd', det_of=lambda i: '-'):
    return [feed_line(t0 + i / 60.0, 100.0, 200.0 + 0.05 * i, z_of(i), clip, det_of(i)) for i in range(n)]


class Rides(unittest.TestCase):
    def ride_log(self, conditioned=300):
        return [log('14:00:10.000', 'UTPP PLATFORM on: onGround=1 vz=1.20 detail=0'),
                log('14:00:15.000', f'UTPP PLATFORM off: 5.0 s, feeds=300 airFeeds=300 contactLost=40 vz=-0.10..2.40 conditioned={conditioned}')]

    def test_a_clean_ride_passes(self):
        d = write_run(self.ride_log(), walk(8.0, 600, lambda i: 10.0 + 0.02 * i, clip='M_Neutral_Idle'))
        logs, feeds = gr.find_files(d)
        lg = gr.read_log(logs)
        r = gr.ride_verdicts(lg['rides'], [(gr.feed_start(p), gr.read_feed(p)) for p in feeds])
        self.assertEqual(len(r), 1)
        self.assertEqual(r[0]['verdict'], 'PASS', r[0]['fails'])
        self.assertEqual((r[0]['feeds'], r[0]['contact_lost'], r[0]['conditioned']), (300, 40, 300))
        self.assertEqual(gr.main([d]), 0)

    def test_a_fall_clip_an_air_detail_or_an_unconditioned_feed_fails(self):
        rows = walk(8.0, 600, lambda i: 10.0, clip='M_Neutral_Idle', det_of=lambda i: '14' if i == 300 else '-')
        rows[310] = feed_line(8.0 + 310 / 60.0, 100.0, 215.0, 10.0, clip='M_Fall_Loop')
        d = write_run(self.ride_log(conditioned=290), rows)
        logs, feeds = gr.find_files(d)
        r = gr.ride_verdicts(gr.read_log(logs)['rides'], [(gr.feed_start(p), gr.read_feed(p)) for p in feeds])[0]
        self.assertEqual(r['verdict'], 'FAIL')
        text = ' | '.join(r['fails'])
        self.assertIn('10 platform feed(s) not conditioned', text)
        self.assertIn('M_Fall_Loop', text)
        self.assertIn('1 feed row(s) with an air detail', text)
        self.assertEqual(gr.main([d]), 1)

    def test_a_ride_across_a_log_rotation_is_read_in_time_order(self):
        # the session log rotated during the ride: "on" is in the older part .2, "off" in the newer part .1 (name order is .1, .2)
        on, off = self.ride_log()
        d = write_run([log('14:00:16.000', 'UTPP after the ride')], walk(8.0, 600, lambda i: 10.0, clip='M_Neutral_Idle'))
        for idx, line in ((2, on), (1, off)):
            with open(os.path.join(d, 'logs', f'utpp-2026-11-22-14-00-00.{idx}.log'), 'w', encoding='utf-8') as f:
                f.write(line + '\n')
        logs, feeds = gr.find_files(d)
        self.assertEqual([os.path.basename(p) for p in logs], ['utpp-2026-11-22-14-00-00.2.log', 'utpp-2026-11-22-14-00-00.1.log',
                                                               'utpp-2026-11-22-14-00-00.log'])
        r = gr.ride_verdicts(gr.read_log(logs)['rides'], [(gr.feed_start(p), gr.read_feed(p)) for p in feeds])
        self.assertEqual([x['verdict'] for x in r], ['PASS'])

    def test_rows_outside_the_ride_window_do_not_count(self):
        rows = walk(0.0, 300, lambda i: 10.0, clip='M_Jump_Start') + walk(8.0, 600, lambda i: 10.0, clip='M_Neutral_Idle')
        d = write_run(self.ride_log(), rows)
        logs, feeds = gr.find_files(d)
        r = gr.ride_verdicts(gr.read_log(logs)['rides'], [(gr.feed_start(p), gr.read_feed(p)) for p in feeds])[0]
        self.assertEqual(r['verdict'], 'PASS', r['fails'])


class Curbs(unittest.TestCase):
    def test_an_isolated_step_is_found_a_slope_and_a_jump_are_not(self):
        step = walk(0.0, 120, lambda i: 10.0 + (0.08 if i >= 60 else 0.0) + (0.07 if i >= 61 else 0.0))
        self.assertEqual([(s['frames'], s['dz_m']) for s in gr.curb_steps([r for r in map(self.row, step)])], [(2, 0.15)])
        slope = walk(0.0, 120, lambda i: 10.0 + 0.05 * i)
        self.assertEqual(gr.curb_steps([self.row(r) for r in slope]), [])
        jump = walk(0.0, 120, lambda i: 10.0 + (0.15 if i >= 60 else 0.0), det_of=lambda i: '18' if 55 <= i <= 62 else '-')
        self.assertEqual(gr.curb_steps([self.row(r) for r in jump]), [])

    def test_camera_snap_near_a_step_fails(self):
        rows = walk(10.0, 120, lambda i: 10.0 + (0.15 if i >= 60 else 0.0))
        logs = [log('14:00:10.900', 'UTPP CET [utpp_camera] camTrace t=1.0 body=0 out=0 pitch=0 mode=hold pivotZ=11.55(+1.55) blocked=0'),
                log('14:00:11.050', 'UTPP CET [utpp_camera] camTrace t=1.1 body=0 out=0 pitch=0 mode=hold pivotZ=11.62(+1.47) blocked=0'),
                log('14:00:11.100', 'UTPP CET [utpp_camera] pivot clamp: lag 1.10 m, V vz 9.0 m/s, step 0.15 m, dt 0.016 s (event 1)')]
        d = write_run(logs, rows)
        self.assertEqual(gr.main([d]), 1)
        lg = gr.read_log(gr.find_files(d)[0])
        self.assertEqual(lg['cams'][0][1], 1.55)
        self.assertEqual(len(lg['snaps']), 1)

    @staticmethod
    def row(line):
        p = line.split()
        return {'t': float(p[0]), 'x': float(p[19]), 'y': float(p[20]), 'z': float(p[21]), 'v': 3.0, 'clip': p[12],
                'det': int(p[26]) if p[26].isdigit() else None, 'ground': p[27]}


if __name__ == '__main__':
    unittest.main()
