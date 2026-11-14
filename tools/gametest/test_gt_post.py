"""Tests of gt_post (pool sizing, the process pool, contact sheet jobs and their merge, clip cutting) and of the per-cause failure
clip limit of failure_clips. Fake runners and temp folders; the one test with a real ffmpeg skips when there is none.

  python Source\\tools\\gametest\\test_gt_post.py -v
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
from datetime import datetime, timedelta
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import failure_clips as fc  # noqa: E402
import gt_post  # noqa: E402
import video_sheets  # noqa: E402


def ok(rc=0, err=''):
    return SimpleNamespace(returncode=rc, stdout='', stderr=err)


class PoolSizeTests(unittest.TestCase):
    def test_idle_cores_from_the_load(self):
        with mock.patch('os.cpu_count', return_value=32):
            self.assertAlmostEqual(gt_post.idle_cores(probe=lambda: 75.0), 8.0)
            self.assertAlmostEqual(gt_post.idle_cores(probe=lambda: 100.0), 0.0)
            self.assertAlmostEqual(gt_post.idle_cores(probe=lambda: None), 16.0)           # unknown load: half

    def test_pool_size_is_clamped(self):
        self.assertEqual(gt_post.pool_size(40, idle=8.0), 7)                                # idle - 1 reserved
        self.assertEqual(gt_post.pool_size(40, idle=31.0), gt_post.MAX_POOL)                # never more than 24
        self.assertEqual(gt_post.pool_size(40, idle=0.2), gt_post.MIN_POOL)                 # a busy machine still gets some
        self.assertEqual(gt_post.pool_size(3, idle=20.0), 3)                                # no more workers than jobs
        self.assertEqual(gt_post.pool_size(0, idle=20.0), 1)


class RunPoolTests(unittest.TestCase):
    def test_every_job_runs_failures_are_kept_and_the_order_is_the_job_order(self):
        calls = []

        def runner(argv, **kw):
            calls.append(argv[-1])
            if argv[-1] == 'bad':
                return ok(1, 'ffmpeg frame 3 failed\nlast line')
            if argv[-1] == 'boom':
                raise OSError('cannot start')
            if argv[-1] == 'slow':
                raise subprocess.TimeoutExpired(argv, 1)
            return ok()
        jobs = [(n, ['x', n]) for n in ('a', 'bad', 'boom', 'slow', 'b')]
        res = gt_post.run_pool(jobs, 3, runner)
        self.assertEqual(sorted(calls), sorted(n for n, _ in jobs))
        self.assertEqual(list(res), [n for n, _ in jobs])
        self.assertTrue(res['a']['ok'] and res['b']['ok'])
        self.assertEqual((res['bad']['ok'], res['bad']['rc'], res['bad']['error']), (False, 1, 'last line'))
        self.assertIn('cannot start', res['boom']['error'])
        self.assertIn('Timeout', res['slow']['error'])

    def test_workers_really_overlap_and_one_worker_is_sequential(self):
        barrier = threading.Barrier(3, timeout=5)
        res = gt_post.run_pool([(n, ['x']) for n in 'abc'], 3, lambda argv, **kw: barrier.wait() and ok() or ok())
        self.assertTrue(all(r['ok'] for r in res.values()))                                # three jobs met: concurrent
        threads = set()
        gt_post.run_pool([(n, ['x']) for n in 'abc'], 1, lambda argv, **kw: threads.add(threading.get_ident()) or ok())
        self.assertEqual(threads, {threading.get_ident()})

    def test_empty_and_priority_flag(self):
        self.assertEqual(gt_post.run_pool([], 4), {})
        seen = {}
        gt_post.run_pool([('a', ['x'])], 1, lambda argv, **kw: seen.update(kw) or ok())
        if os.name == 'nt':
            self.assertEqual(seen['creationflags'], gt_post.BELOW_NORMAL)                  # below-normal priority workers


def scen(name, a, b):
    return {'name': name, 'start': datetime(2026, 11, 14, 1, 0, 0) + timedelta(seconds=a),
            'end': datetime(2026, 11, 14, 1, 0, 0) + timedelta(seconds=b)}


class SheetPlanTests(unittest.TestCase):
    def plan(self, **kw):
        return gt_post.plan_sheets('v.mp4', 'u.log', '2026-11-14 01:07:10.330', 0.0, 'ff', datetime(2026, 11, 14, 1, 0, 0), 430.33,
                                   [scen('kbd_taps_r1', 10, 18)], 3.0, os.path.join('R', 'sheets'), **kw)

    @staticmethod
    def val(job, flag):
        a = job['args']
        return a[a.index(flag) + 1]

    def test_the_full_sheet_is_cut_into_one_sheet_parts_with_the_numbers_of_the_whole(self):
        jobs = self.plan()
        full = [j for j in jobs if j['sheet'] == 'full']
        self.assertEqual(len(full), 36)                                                     # 861 frames at 0.5 s, 24 per sheet
        self.assertEqual([self.val(full[0], f) for f in ('--start', '--end', '--frame-offset')], ['0.0', '11.5', '0'])
        self.assertEqual([self.val(full[1], f) for f in ('--start', '--end', '--frame-offset')], ['12.0', '23.5', '24'])
        self.assertEqual([self.val(full[-1], f) for f in ('--start', '--end', '--frame-offset')], ['420.0', '430.0', '840'])
        frames = sum(int((float(self.val(j, '--end')) - float(self.val(j, '--start'))) / 0.5) + 1 for j in full)
        self.assertEqual(frames, 861)                                                       # no frame twice, none lost
        self.assertTrue(all(j['out'].endswith(f'full.part{i + 1:03d}') for i, j in enumerate(full)))
        self.assertTrue(all('--no-keep-sheet-frames' in j['args'] for j in jobs))

    def test_windows_and_filters(self):
        jobs = self.plan()
        wins = [j for j in jobs if j['sheet'] != 'full']
        self.assertEqual([j['sheet'] for j in wins], ['kbd_taps_r1_start', 'kbd_taps_r1_end'])
        self.assertEqual((self.val(wins[0], '--start'), self.val(wins[0], '--end'), self.val(wins[0], '--rows')), ('7.0', '13.0', '5'))
        self.assertEqual([j['sheet'] for j in self.plan(only='kbd_taps_r1_end')], ['kbd_taps_r1_end'])
        self.assertTrue(all(j['part'] is not None for j in self.plan(only='full')))
        self.assertNotIn('--no-keep-sheet-frames', self.plan(keep_frames=True)[0]['args'])


class MergeAndRunSheetsTests(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='gp_')

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    @staticmethod
    def fake_child(fail=()):
        """Runner that behaves like `gt_post.py sheet ...`: writes the sheet and index.txt the real job would write."""
        def runner(argv, **kw):
            args = argv[4:]
            val = lambda f: args[args.index(f) + 1]  # noqa: E731
            out = val('--out')
            name = os.path.basename(out)
            if name in fail:
                return ok(1, f'boom {name}')
            os.makedirs(out, exist_ok=True)
            off = int(val('--frame-offset')) if '--frame-offset' in args else 0
            per = int(val('--cols')) * int(val('--rows'))
            n = int((float(val('--end')) - float(val('--start'))) / float(val('--every')) + 1e-9) + 1
            for s in range((n + per - 1) // per):
                with open(os.path.join(out, f'sheet_{s + 1 + off // per:02d}.png'), 'wb') as f:
                    f.write(b'png')
            head = 'video v\nlog l\nvideo start x\n'
            entries = [f'#{off + i + 1:04d} sheet {(off + i) // per + 1:02d}  frame' for i in range(n)]
            with open(os.path.join(out, 'index.txt'), 'w', encoding='utf-8') as f:
                f.write('\n'.join([head] + entries) + '\n')
            return ok()
        return runner

    def test_parts_are_merged_into_one_folder_with_one_continuous_index(self):
        sd = os.path.join(self.d, 'sheets')
        jobs = gt_post.plan_sheets('v.mp4', 'u.log', '2026-11-14 01:07:10.330', 0.0, 'ff', datetime(2026, 11, 14, 1, 0, 0), 60.0,
                                   [scen('kbd_taps_r1', 10, 18)], 3.0, sd)
        res, timing = gt_post.run_sheets(jobs, sd, 4, self.fake_child())
        self.assertEqual(set(res.values()), {'ok'})
        self.assertEqual(set(res), {'full', 'kbd_taps_r1_start', 'kbd_taps_r1_end'})
        self.assertEqual(sorted(os.listdir(sd)), ['full', 'kbd_taps_r1_end', 'kbd_taps_r1_start'])   # no part folder is left
        self.assertEqual(sorted(f for f in os.listdir(os.path.join(sd, 'full')) if f.endswith('.png')),
                         [f'sheet_{i:02d}.png' for i in range(1, 6)])                              # 120 frames = 5 sheets
        with open(os.path.join(sd, 'full', 'index.txt'), encoding='utf-8') as f:
            text = f.read()
        self.assertEqual(text.count('video v'), 1)                                                 # one header
        nums = [int(l[1:5]) for l in text.splitlines() if l.startswith('#')]
        self.assertEqual(nums, list(range(1, 121)))
        self.assertEqual(timing['jobs'], len(jobs))

    def test_a_failed_part_is_reported_for_its_sheet_and_the_rest_is_merged(self):
        sd = os.path.join(self.d, 'sheets')
        jobs = gt_post.plan_sheets('v.mp4', 'u.log', '2026-11-14 01:07:10.330', 0.0, 'ff', datetime(2026, 11, 14, 1, 0, 0), 60.0,
                                   [scen('kbd_taps_r1', 10, 18)], 3.0, sd)
        res, _ = gt_post.run_sheets(jobs, sd, 2, self.fake_child(fail=('full.part002', 'kbd_taps_r1_end')))
        self.assertTrue(res['full'].startswith('error:') and 'full.part002' in res['full'])
        self.assertTrue(res['kbd_taps_r1_end'].startswith('error:'))
        self.assertEqual(res['kbd_taps_r1_start'], 'ok')
        self.assertTrue(os.path.isfile(os.path.join(sd, 'full', 'sheet_01.png')))

    def test_nothing_to_do(self):
        self.assertEqual(gt_post.run_sheets([], self.d)[0], {})

    def test_clip_specs_are_written_run_and_removed(self):
        seen = []

        def runner(argv, **kw):
            with open(argv[-1], encoding='utf-8') as f:
                spec = json.load(f)
            seen.append(spec)
            return ok(1, 'no encoder') if spec['out'].endswith('02.mp4') else ok()
        items = [({'verdict': 'INVALID', 'name': f'n{i}', 'why': 'w', 'start': 1.0 * i, 'end': 2.0 * i + 1}, os.path.join(self.d, f'0{i}.mp4'))
                 for i in (1, 2)]
        res = gt_post.cut_pool('v.mp4', items, 'ffmpeg', True, 2, scratch=os.path.join(self.d, 'specs'), runner=runner)
        self.assertEqual(res[items[0][1]], None)
        self.assertEqual(res[items[1][1]], 'no encoder')
        self.assertEqual({s['item']['name'] for s in seen}, {'n1', 'n2'})
        self.assertTrue(all(s['small'] and s['video'] == 'v.mp4' for s in seen))
        self.assertFalse(os.path.exists(os.path.join(self.d, 'specs')))


class OnDemandSheetsTests(MergeAndRunSheetsTests):
    """gt_post.py sheets <run>: the form after a --fast run. The images go to Media like those of a run that made them itself."""

    def make_run(self, with_readme=True):
        self.run = os.path.join(self.d, 'Research', 'night', '20261114-010203')
        self.media = os.path.join(self.d, 'Media')
        rec_dir = os.path.join(self.media, 'mp4', '2026-11-14_night-20261114-010203')
        os.makedirs(rec_dir)
        os.makedirs(os.path.join(self.run, 'logs'))
        self.video = os.path.join(rec_dir, 'rec_x.mp4')
        with open(self.video, 'wb') as f:
            f.write(b'video')
        log = os.path.join(self.run, 'logs', 'utpp-x.log')
        with open(log, 'w') as f:
            f.write('log\n')
        t0 = datetime(2026, 11, 14, 1, 2, 5)
        with open(os.path.join(self.run, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump({'cycle': '20261114-010203', 'utpp_log': log, 'sheets': 'not made (--fast): see pending_media',
                       'recording': {'path': self.video, 'recording_original': os.path.join(self.run, 'rec_x.mp4')},
                       'clock': {'video_start': str(t0), 'video_end': str(t0 + timedelta(seconds=60)), 'duration_s': 60.0,
                                 'clock_offset_s': 0.0}}, f)
        with open(os.path.join(self.run, 'cycle.json'), 'w', encoding='utf-8') as f:
            json.dump({'scenarios': [{'name': 'kbd_taps', 'run': 1, 'start': str(t0 + timedelta(seconds=10)),
                                      'end': str(t0 + timedelta(seconds=18))}]}, f)
        if with_readme:
            os.makedirs(os.path.join(self.media, 'png'), exist_ok=True)
            with open(os.path.join(self.media, 'png', 'README.md'), 'w', encoding='utf-8') as f:
                f.write('| folder | files | MB |\n')

    def test_the_images_go_to_media_the_summary_is_updated_and_the_recording_stays(self):
        self.make_run()
        res, timing = gt_post.sheets_for_run(self.run, runner=self.fake_child(), media_root=self.media)
        self.assertEqual(set(res.values()), {'ok'})
        png = os.path.join(self.media, 'png', '2026-11-14_night-20261114-010203')
        names = sorted(os.listdir(png))
        self.assertIn('full__sheet_01.png', names)
        self.assertIn('kbd_taps_r1_start__sheet_01.png', names)
        self.assertFalse([f for f in os.listdir(os.path.join(self.run, 'sheets', 'full')) if f.endswith('.png')])   # moved, not copied
        with open(os.path.join(self.run, 'summary.json'), encoding='utf-8') as f:
            summ = json.load(f)
        self.assertEqual(summ['sheets']['full'], 'ok')                                       # the "not made" text is replaced
        self.assertEqual(len(summ['sheet_images']), len(names))
        self.assertTrue(all(os.path.isfile(i['path']) for i in summ['sheet_images']))
        self.assertEqual(timing['media']['sheet_images'], len(names))
        self.assertTrue(os.path.isfile(self.video))                                          # the recording in Media is untouched
        self.assertFalse(os.path.exists(os.path.join(self.run, 'rec_x.mp4')))
        with open(os.path.join(self.media, 'png', 'README.md'), encoding='utf-8') as f:
            self.assertIn('| 2026-11-14_night-20261114-010203 |', f.read())                  # the folder is listed once in the README

    def test_one_sheet_only_and_no_media_move(self):
        self.make_run(with_readme=False)
        res, timing = gt_post.sheets_for_run(self.run, only='kbd_taps_r1_end', runner=self.fake_child(), media_root=self.media, move=False)
        self.assertEqual(res, {'kbd_taps_r1_end': 'ok'})
        self.assertNotIn('media', timing)
        self.assertTrue(os.path.isfile(os.path.join(self.run, 'sheets', 'kbd_taps_r1_end', 'sheet_01.png')))
        self.assertFalse(os.path.exists(os.path.join(self.media, 'png')))
        with self.assertRaises(SystemExit):
            gt_post.sheets_for_run(self.run, only='nope', runner=self.fake_child(), media_root=self.media)

    def test_a_missing_recording_is_reported(self):
        self.make_run(with_readme=False)
        os.remove(self.video)
        with self.assertRaises(SystemExit):
            gt_post.sheets_for_run(self.run, runner=self.fake_child(), media_root=self.media)


# ----------------------------------------------------------------------------------------------------------- failure clips
def _dump(path, obj):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f)


def make_run(d, steps, scenarios, verdicts=None, duration=600.0):
    """A night run folder with the three files failure_clips reads. steps: [(scenario, run, step, verdict, fail, t0 s, t1 s)]."""
    base = datetime(2026, 11, 14, 1, 0, 0)
    ts = lambda s: (base + timedelta(seconds=s)).strftime('%Y-%m-%d %H:%M:%S.%f')  # noqa: E731
    video = os.path.join(d, 'rec.mp4')
    open(video, 'wb').close()
    os.makedirs(os.path.join(d, 'gametest'), exist_ok=True)
    _dump(os.path.join(d, 'summary.json'), {'recording': {'path': video}, 'clock': {'video_start': ts(0), 'duration_s': duration},
                                            'scenario_verdicts': verdicts or {}})
    _dump(os.path.join(d, 'gametest', 'summary.json'),
          {'steps': [{'scenario': s, 'run': r, 'step': st, 'verdict': v, 'fails': [f] if f else [], 't0': ts(a), 't1': ts(b)}
                     for s, r, st, v, f, a, b in steps]})
    _dump(os.path.join(d, 'cycle.json'), {'scenarios': [{'name': n, 'run': 1, 'start': ts(a), 'end': ts(b)} for n, a, b in scenarios]})


class FailureClipLimitTests(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='fc_')
        names = [f'kbd_{i}' for i in range(10)]
        steps, scenarios = [], []
        for i, n in enumerate(names):                      # 5 steps per scenario, all invalid for one cause (numbers differ)
            t = 20.0 * i
            scenarios.append((n, t, t + 15))
            for k in range(5):
                steps.append((n, 1, f's{k}', 'INVALID', f'blocked: V did not move (|v| <= 0.{i}{k} m/s < 0.3 on {k + 2} ticks)',
                              t + 3 * k, t + 3 * k + 3))
        steps.append(('kbd_3', 1, 'tail', 'FAIL', 'rate_min 0.12 < 0.2', 71.0, 74.0))            # another cause
        steps.append(('kbd_7', 1, 'tail', 'FAIL', 'rate_min 0.09 < 0.2', 151.0, 154.0))
        scenarios.append(('lonely', 300.0, 320.0))                                                # no failing step, a bad verdict
        make_run(self.d, steps, scenarios, {'lonely#1': {'verdict': 'TIMEOUT', 'fails': ['scenario exceeded 180 s']}})

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    def test_cause_ignores_the_numbers(self):
        a = fc.cause_of({'verdict': 'INVALID', 'why': 'blocked: V did not move (|v| <= 0.12 m/s < 0.3 on 5 ticks)'})
        b = fc.cause_of({'verdict': 'INVALID', 'why': 'blocked: V did not move (|v| <= 0.31 m/s < 0.3 on 2 ticks)'})
        self.assertEqual(a, b)
        self.assertNotEqual(a, fc.cause_of({'verdict': 'FAIL', 'why': 'blocked: V did not move (|v| <= 0.12 m/s < 0.3 on 5 ticks)'}))
        self.assertNotEqual(fc.cause_of({'verdict': 'FAIL', 'why': 'rate_min 0.1 < 0.2'}),
                            fc.cause_of({'verdict': 'FAIL', 'why': 'rate_max 1.9 > 1.8'}))

    def test_six_clips_per_cause_the_first_of_each_scenario_first(self):
        video, items, dropped = fc.plan_clips(self.d, per_cause=6)
        inv = [it for it in items if it['verdict'] == 'INVALID']
        self.assertEqual(len(inv), 6)
        self.assertEqual([it['name'] for it in inv], [f'kbd_{i}_r1_s0' for i in range(6)])         # first step of 6 scenarios
        self.assertEqual(len({it['scenario'] for it in inv}), 6)
        self.assertEqual(sum(len(x['dropped']) for x in dropped), 44)
        self.assertEqual(len([it for it in items if it['verdict'] == 'FAIL']), 2)                  # another cause: its own six
        lone = [it for it in items if it['name'] == 'lonely_r1']
        self.assertEqual(len(lone), 1)                                                              # a scenario without a failing step
        self.assertTrue(all(it['cause'] for it in items))

    def test_a_failing_step_and_its_scenario_never_both_get_a_clip(self):
        _, items, _ = fc.plan_clips(self.d, per_cause=0)
        self.assertEqual(len(items), 50 + 2 + 1)
        steps_of = {it['scenario'] for it in items if not it.get('whole')}
        self.assertFalse(any(it.get('whole') and it['scenario'] in steps_of for it in items))

    def test_rest_of_a_cause_follows_when_there_are_fewer_scenarios_than_clips(self):
        _, items, dropped = fc.plan_clips(self.d, per_cause=12)
        inv = [it['name'] for it in items if it['verdict'] == 'INVALID']
        self.assertEqual(len(inv), 12)
        self.assertTrue({f'kbd_{i}_r1_s0' for i in range(10)} <= set(inv))                          # every scenario's first step is in
        self.assertEqual(sum(len(x['dropped']) for x in dropped), 38)

    def test_name_filter_and_zero_means_everything(self):
        _, items, dropped = fc.plan_clips(self.d, per_cause=0, name_filter='kbd_3_')
        self.assertEqual({it['scenario'] for it in items}, {'kbd_3#1'})
        self.assertEqual(dropped, [])

    def test_make_clips_cuts_through_the_given_cutter_and_drops_failed_cuts(self):
        out = os.path.join(self.d, 'clips')
        got = {}

        def cutter(video, jobs):
            got['n'] = len(jobs)
            return {path: ('no encoder' if path.endswith('03_kbd_2_r1_s0.mp4') else None) for _, path in jobs}
        dropped = []
        items = fc.make_clips(self.d, out=out, per_cause=6, cutter=cutter, dropped_out=dropped)
        self.assertEqual(got['n'], 6 + 2 + 1)
        self.assertEqual(len(items), 8)                                                             # one cut failed
        self.assertTrue(all(os.path.basename(it['path'])[:2].isdigit() for it in items))
        self.assertEqual(sum(len(x['dropped']) for x in dropped), 44)

    def test_list_command_cuts_nothing(self):
        import io
        buf = io.StringIO()
        with mock.patch('sys.stdout', buf):
            rc = fc.main([self.d, '--list', '--per-cause', '3'])
        text = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn('kbd_0_r1_s0', text)
        self.assertIn('more of the cause', text)
        self.assertFalse(os.path.exists(os.path.join(self.d, 'clips')))


# ------------------------------------------------------------------------------------------------------------- real ffmpeg
FFMPEG = video_sheets.FFMPEG


@unittest.skipUnless(os.path.isfile(FFMPEG), 'the project ffmpeg is not installed')
class FrameOffsetSheetTests(unittest.TestCase):
    """A sheet cut into parts equals the uncut one: same index entries, same numbers, byte-identical sheet images."""

    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='fo_')
        self.video = os.path.join(self.d, 'v.mp4')
        subprocess.run([FFMPEG, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'lavfi', '-i', 'testsrc=size=320x180:rate=10',
                        '-t', '8', '-pix_fmt', 'yuv420p', self.video], check=True)
        self.log = os.path.join(self.d, 'u.log')
        with open(self.log, 'w', encoding='utf-8') as f:
            f.write('[2026-11-14 01:00:00.000] [info    ] [ 1] [UTPP] hello\n')

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    def sheet(self, out, *extra):
        video_sheets.annotated_main(['--video', self.video, '--log', self.log, '--video-end', '2026-11-14 01:00:08.000',
                                     '--clock-offset', '0', '--ffmpeg', FFMPEG, '--no-keep-sheet-frames', '--every', '0.5', '--cols',
                                     '4', '--rows', '2', '--width', '160', '--out', os.path.join(self.d, out), *extra])

    @staticmethod
    def sha(p):
        with open(p, 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()

    def test_part_equals_the_same_frames_of_the_whole(self):
        self.sheet('whole', '--start', '0', '--end', '7.5')
        self.sheet('part1', '--start', '0', '--end', '3.5')
        self.sheet('part2', '--start', '4', '--end', '7.5', '--frame-offset', '8')
        for n, p in (('sheet_01.png', 'part1'), ('sheet_02.png', 'part2')):
            self.assertEqual(self.sha(os.path.join(self.d, 'whole', n)), self.sha(os.path.join(self.d, p, n)), n)
        with open(os.path.join(self.d, 'whole', 'index.txt'), encoding='utf-8') as f:
            whole = f.read()
        with open(os.path.join(self.d, 'part2', 'index.txt'), encoding='utf-8') as f:
            p2 = f.read()
        self.assertEqual(whole[whole.index('#0009'):], p2[p2.index('#0009'):])
        self.assertTrue(os.path.isfile(os.path.join(self.d, 'part2', 'sheet_02.png')))
        self.assertFalse(os.path.isfile(os.path.join(self.d, 'part2', 'sheet_01.png')))
        with self.assertRaises(SystemExit):
            self.sheet('bad', '--start', '0', '--end', '3.5', '--frame-offset', '5')            # not a multiple of the sheet size


if __name__ == '__main__':
    unittest.main()
