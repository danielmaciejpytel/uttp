"""Offline tests for run_compare_film.py: fake run folders (no real recording needed), plus one real cut when ffmpeg exists.

  python Source\\tools\\films\\test_run_compare_film.py -v
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import run_compare_film as rcf  # noqa: E402

T0 = datetime(2026, 11, 14, 10, 0, 0)


def stamp(seconds: float) -> str:
    return (T0 + timedelta(seconds=seconds)).strftime('%Y-%m-%d %H:%M:%S.%f')


def make_run(root: str, name: str, video_offset: float, steps: dict, duration: float = 60.0, with_video: bool = True,
             video_name: str = 'rec.mp4') -> str:
    """A fake night run folder. `steps` = {(scenario, run): [(step, t0, t1)]} in seconds from T0; the recording starts at
    T0 - video_offset."""
    d = os.path.join(root, name)
    os.makedirs(os.path.join(d, 'gametest'))
    video = os.path.join(d, video_name)
    if with_video:
        with open(video, 'wb') as f:
            f.write(b'x')
    with open(os.path.join(d, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump({'recording': {'path': video, 'recording_original': video},
                   'clock': {'video_start': stamp(-video_offset), 'duration_s': duration}}, f)
    rows = [{'scenario': sc, 'run': r, 'step': st, 't0': stamp(a), 't1': stamp(b), 'verdict': 'PASS'}
            for (sc, r), lst in steps.items() for st, a, b in lst]
    with open(os.path.join(d, 'gametest', 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump({'steps': rows}, f)
    cyc = [{'name': sc, 'run': r, 'start': stamp(min(a for _, a, _ in lst)), 'end': stamp(max(b for _, _, b in lst))}
           for (sc, r), lst in steps.items()]
    with open(os.path.join(d, 'cycle.json'), 'w', encoding='utf-8') as f:
        json.dump({'scenarios': cyc}, f)
    return d


JUMP = [('settle', 5.0, 6.5), ('jump', 8.5, 10.0), ('land', 10.0, 13.0)]
SLIDE = [('sprint', 20.0, 23.0), ('slide', 23.0, 25.6)]


class FilmCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='film_test_')
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        # the left recording starts 3 s before T0, the right one 1.5 s before; the right run plays 0.2 s later
        self.left_dir = make_run(self.tmp, 'left', 3.0, {('jump_in_place', 1): JUMP, ('slide_sprint', 1): SLIDE})
        shift = [('jump_in_place', 1, [(n, a + 0.2, b + 0.2) for n, a, b in JUMP]),
                 ('slide_sprint', 1, [(n, a + 0.2, b + 0.2) for n, a, b in SLIDE])]
        self.right_dir = make_run(self.tmp, 'right', 1.5, {(n, r): lst for n, r, lst in shift})
        self.left, self.right = rcf.load_run(self.left_dir), rcf.load_run(self.right_dir)


class PlanTests(FilmCase):
    def test_one_film_per_scenario_with_the_whole_scenario(self):
        films = rcf.plan_films(self.left, self.right)
        self.assertEqual([f['name'] for f in films], ['jump_in_place_r1', 'slide_sprint_r1'])    # play order of the left run
        p = films[0]['pairs'][0]
        self.assertEqual(p['left']['start'], 3.0 + 5.0)            # step start in video seconds: (T0 + 5) - (T0 - 3)
        self.assertEqual(p['right']['start'], 1.5 + 5.2)           # each side starts at its own step start
        self.assertAlmostEqual(p['left']['end'] - p['left']['start'], 8.0)
        self.assertAlmostEqual(p['duration'], 8.0)

    def test_default_list_is_the_cycle_record_not_the_internal_parts(self):
        # a probe scenario that only gametest/summary.json lists (like wheel_probe) is not a scenario of the set
        steps = {('jump_in_place', 1): JUMP, ('wheel_probe', 1): [('probe', 40.0, 42.0)], ('slide_sprint', 1): SLIDE}
        d = make_run(self.tmp, 'probe_run', 3.0, steps, video_name='p.mp4')
        with open(os.path.join(d, 'cycle.json'), encoding='utf-8') as f:
            cyc = json.load(f)
        cyc['scenarios'] = [c for c in cyc['scenarios'] if c['name'] != 'wheel_probe']
        with open(os.path.join(d, 'cycle.json'), 'w', encoding='utf-8') as f:
            json.dump(cyc, f)
        run = rcf.load_run(d)
        self.assertEqual(rcf.common_scenarios(run, run, 1), ['jump_in_place', 'slide_sprint'])
        self.assertEqual([f['name'] for f in rcf.plan_films(run, run)], ['jump_in_place_r1', 'slide_sprint_r1'])
        # an internal part stays addressable by name
        self.assertEqual(rcf.plan_films(run, run, scenarios=['wheel_probe'])[0]['name'], 'wheel_probe_r1')

    def test_macro_phases_are_paired_by_name_not_by_play_order(self):
        def macro_run(name, weapons):
            d = make_run(self.tmp, name, 3.0, {('jump_in_place', 1): JUMP}, video_name=name + '.mp4')
            with open(os.path.join(d, 'cycle.json'), encoding='utf-8') as f:
                cyc = json.load(f)
            wins, t = [], 30.0
            for w in weapons:
                for ph in ('light_combo', 'light_combo_fast'):
                    wins.append({'weapon': w, 'phase': ph, 'start': stamp(t), 'end': stamp(t + 5.0)})
                    t += 6.0
            cyc['scenarios'].append({'name': 'melee_weapons', 'run': 1, 'start': stamp(29.0), 'end': stamp(t),
                                     'phase_windows': wins})
            with open(os.path.join(d, 'cycle.json'), 'w', encoding='utf-8') as f:
                json.dump(cyc, f)
            return rcf.load_run(d)
        left, right = macro_run('ml', ['knife', 'katana']), macro_run('mr', ['katana'])      # the right run skipped the knife
        p = rcf.plan_films(left, right, steps=['melee_weapons/katana.light_combo_fast'], film='f')[0]['pairs'][0]
        self.assertEqual(p['left']['start'], 3.0 + 30.0 + 18.0)       # katana is the second weapon on the left ...
        self.assertEqual(p['right']['start'], 3.0 + 30.0 + 6.0)       # ... the first on the right: same phase, own time
        with self.assertRaises(rcf.FilmError) as cm:
            rcf.plan_films(left, right, steps=['melee_weapons/knife.light_combo'], film='f')
        self.assertIn('no time in the right run', str(cm.exception))

    def test_steps_go_into_one_film_in_the_given_order(self):
        films = rcf.plan_films(self.left, self.right, steps=['slide_sprint/slide', 'jump_in_place/jump'], film='states')
        self.assertEqual([f['name'] for f in films], ['states'])
        pairs = films[0]['pairs']
        self.assertEqual([p['label'] for p in pairs], ['slide_sprint/slide', 'jump_in_place/jump'])
        self.assertEqual(pairs[0]['left']['start'], 3.0 + 23.0)
        self.assertAlmostEqual(pairs[0]['duration'], 2.6)
        self.assertAlmostEqual(pairs[1]['duration'], 1.5)

    def test_pre_and_post_apply_to_both_sides(self):
        p = rcf.plan_films(self.left, self.right, steps=['jump_in_place/jump'], film='f', pre=0.5, post=0.25)[0]['pairs'][0]
        self.assertEqual(p['left']['start'], 3.0 + 8.5 - 0.5)
        self.assertEqual(p['right']['start'], 1.5 + 8.7 - 0.5)
        self.assertAlmostEqual(p['duration'], 1.5 + 0.75)

    def test_a_step_without_a_time_in_one_run_is_refused(self):
        right = make_run(self.tmp, 'right2', 1.5, {('jump_in_place', 1): [s for s in JUMP if s[0] != 'jump']}, video_name='r2.mp4')
        with self.assertRaises(rcf.FilmError) as cm:
            rcf.plan_films(self.left, rcf.load_run(right), steps=['jump_in_place/jump', 'jump_in_place/land'], film='f')
        self.assertIn('jump_in_place/jump (run 1) has no time in the right run', str(cm.exception))
        self.assertNotIn('land', str(cm.exception))
        with self.assertRaises(rcf.FilmError) as cm:
            rcf.plan_films(rcf.load_run(right), self.left, scenarios=['slide_sprint'])
        self.assertIn('no time in the left run', str(cm.exception))

    def test_other_run_numbers_and_unknown_scenarios(self):
        with self.assertRaises(rcf.FilmError):
            rcf.plan_films(self.left, self.right, scenarios=['jump_in_place'], number=2)
        with self.assertRaises(rcf.FilmError):
            rcf.plan_films(self.left, self.right, steps=['jump_in_place/jump'])          # --steps needs --film
        with self.assertRaises(rcf.FilmError):
            rcf.plan_films(self.left, self.right, steps=['jump_in_place'], film='f')    # not scenario/step
        with self.assertRaises(rcf.FilmError):
            rcf.plan_films(self.left, self.right, scenarios=['nope'])

    def test_a_recording_that_does_not_cover_the_step_is_refused(self):
        # the step starts before the recording does
        early_steps = {('jump_in_place', 1): [('settle', -2.0, -1.0)]}
        early = make_run(self.tmp, 'early', 0.0, early_steps, video_name='early.mp4')
        with self.assertRaises(rcf.FilmError) as cm:
            rcf.plan_films(self.left, rcf.load_run(early), scenarios=['jump_in_place'])
        self.assertIn('starts', str(cm.exception))
        short = make_run(self.tmp, 'short', 3.0, {('jump_in_place', 1): JUMP}, duration=6.0, video_name='short.mp4')
        with self.assertRaises(rcf.FilmError) as cm:
            rcf.plan_films(self.left, rcf.load_run(short), scenarios=['jump_in_place'])
        self.assertIn('ends', str(cm.exception))
        # a recording that ends inside the step is cut at its end (the other half is held on its last frame)
        cut = make_run(self.tmp, 'cut', 3.0, {('jump_in_place', 1): JUMP}, duration=15.0, video_name='cut.mp4')
        p = rcf.plan_films(self.left, rcf.load_run(cut), steps=['jump_in_place/land'], film='f')[0]['pairs'][0]
        self.assertTrue(p['right']['cut_at_end'])
        self.assertFalse(p['left']['cut_at_end'])
        self.assertAlmostEqual(p['duration'], 3.0)

    def test_missing_recording_or_clock(self):
        none = make_run(self.tmp, 'novideo', 3.0, {('jump_in_place', 1): JUMP}, with_video=False)
        with self.assertRaises(rcf.FilmError):
            rcf.load_run(none)
        with self.assertRaises(rcf.FilmError):
            rcf.load_run(os.path.join(self.tmp, 'does_not_exist'))


class CommandTests(FilmCase):
    def film(self, steps=('jump_in_place/jump', 'slide_sprint/slide')):
        return rcf.plan_films(self.left, self.right, steps=list(steps), film='states')[0]

    def test_filter_graph(self):
        script = rcf.filter_script(self.film(), 'Game Animations', 'Ultimate Third Person Project')
        self.assertEqual(script.count('hstack=inputs=2'), 2)
        self.assertIn('concat=n=2:v=1:a=0[v]', script)
        self.assertEqual(script.count('fps=30'), 4)                                 # 30 fps on every input
        self.assertEqual(script.count('scale=960:1080:force_original_aspect_ratio=decrease:force_divisible_by=2'), 4)
        self.assertIn("text='Game Animations'", script)
        self.assertIn("text='Ultimate Third Person Project'", script)
        self.assertNotIn('setpts=PTS*', script)                                     # never a speed change: 1x
        self.assertIn('tpad=stop_mode=clone', script)
        single = rcf.filter_script(self.film(('jump_in_place/jump',)), 'a', 'b')
        self.assertIn('[p0]null[v]', single)
        self.assertNotIn('concat', single)

    def test_titles_are_plain_words(self):
        self.assertEqual(rcf.clean_title("Before: 'the' change, %x").split(), ['Before', 'the', 'change', 'x'])
        self.assertEqual(len(rcf.clean_title('x' * 200)), rcf.MAX_TITLE)
        script = rcf.filter_script(self.film(('jump_in_place/jump',)), "A;B,C'D", 'ok')
        self.assertNotIn("A;B", script)

    def test_ffmpeg_command_seeks_both_recordings_per_pair(self):
        cmd = rcf.build_command('ffmpeg', 'L.mp4', 'R.mp4', self.film(), 'graph.txt', 'out.mp4')
        pos = [i for i, c in enumerate(cmd) if c == '-i']
        self.assertEqual([cmd[i + 1] for i in pos], ['L.mp4', 'R.mp4', 'L.mp4', 'R.mp4'])
        ss = [float(cmd[i + 1]) for i, c in enumerate(cmd) if c == '-ss']
        self.assertEqual(ss, [3.0 + 8.5, 1.5 + 8.7, 3.0 + 23.0, 1.5 + 23.2])
        self.assertEqual(cmd[cmd.index('-r') + 1], '30')
        self.assertEqual(cmd[-1], 'out.mp4')
        self.assertIn('graph.txt', cmd)

    def test_render_writes_only_into_out_and_removes_the_script(self):
        out = os.path.join(self.tmp, 'out')
        seen = {}

        def runner(cmd, **kw):
            script = cmd[cmd.index('-filter_complex_script') + 1]
            with open(script, encoding='utf-8') as fh:
                seen['script'] = fh.read()
            seen['dir'] = os.path.dirname(script)
            with open(cmd[-1], 'wb') as f:
                f.write(b'mp4')
            return subprocess.CompletedProcess(cmd, 0, '', '')
        films = rcf.plan_films(self.left, self.right)
        files = rcf.render(films, self.left, self.right, out, 'L', 'R', 'ffmpeg', runner=runner)
        self.assertEqual([os.path.basename(f) for f in files], ['jump_in_place_r1.mp4', 'slide_sprint_r1.mp4'])
        self.assertEqual(sorted(os.listdir(out)), ['jump_in_place_r1.mp4', 'slide_sprint_r1.mp4'])   # no script left behind
        self.assertEqual(os.path.normcase(seen['dir']), os.path.normcase(os.path.abspath(out)))
        self.assertIn('hstack', seen['script'])
        with self.assertRaises(rcf.FilmError):                                      # an existing film is never overwritten silently
            rcf.render(films, self.left, self.right, out, 'L', 'R', 'ffmpeg', runner=runner)
        rcf.render(films, self.left, self.right, out, 'L', 'R', 'ffmpeg', overwrite=True, runner=runner)

    def test_a_failed_ffmpeg_leaves_no_file(self):
        out = os.path.join(self.tmp, 'out2')

        def runner(cmd, **kw):
            with open(cmd[-1], 'wb') as f:
                f.write(b'half')
            return subprocess.CompletedProcess(cmd, 1, '', 'boom')
        with self.assertRaises(rcf.FilmError):
            rcf.render(rcf.plan_films(self.left, self.right, scenarios=['jump_in_place']), self.left, self.right, out, 'L', 'R',
                       'ffmpeg', runner=runner)
        self.assertEqual(os.listdir(out), [])

    def test_command_line(self):
        out = os.path.join(self.tmp, 'cli')
        argv = ['--left', self.left_dir, '--right', self.right_dir, '--left-title', 'A', '--right-title', 'B', '--out', out,
                '--ffmpeg', 'ffmpeg', '--dry-run']
        self.assertEqual(rcf.main(argv), 0)
        self.assertFalse(os.path.exists(out))                                       # a dry run writes nothing
        self.assertEqual(rcf.main(argv[:-1] + ['--steps', 'jump_in_place/nope', '--film', 'x', '--dry-run']), 2)
        self.assertEqual(rcf.main(['--left', self.tmp, '--right', self.right_dir, '--left-title', 'A', '--right-title', 'B',
                                   '--out', out]), 2)


def project_ffmpeg() -> str:
    try:
        path = rcf.default_ffmpeg()
    except Exception:  # noqa: BLE001 - no gametest modules / libraries: the real cut is skipped
        return ''
    return path if os.path.isfile(path) else ''


@unittest.skipUnless(project_ffmpeg(), 'the project ffmpeg is not installed')
class RealCutTests(unittest.TestCase):
    def test_two_generated_recordings_become_one_1x_film(self):
        tmp = tempfile.mkdtemp(prefix='film_real_')
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        ff = rcf.default_ffmpeg()
        vids = []
        for i, size in enumerate(('640x360', '1280x720')):
            p = os.path.join(tmp, f'src{i}.mp4')
            subprocess.run([ff, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'lavfi', '-i',
                            f'testsrc=size={size}:rate=30:duration=6', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', p], check=True)
            vids.append(p)
        steps = {('s', 1): [('a', 1.0, 3.0), ('b', 3.0, 5.0)]}
        left = make_run(tmp, 'l', 0.0, steps, duration=6.0, video_name='x_l.mp4')
        right = make_run(tmp, 'r', 0.0, steps, duration=6.0, video_name='x_r.mp4')
        for run, src in ((left, vids[0]), (right, vids[1])):
            shutil.copyfile(src, os.path.join(run, 'x_' + os.path.basename(run) + '.mp4'))
        out = os.path.join(tmp, 'out')
        rc = rcf.main(['--left', left, '--right', right, '--left-title', 'Before', '--right-title', 'After', '--out', out,
                       '--steps', 's/a,s/b', '--film', 'film'])
        self.assertEqual(rc, 0)
        film = os.path.join(out, 'film.mp4')
        self.assertTrue(os.path.getsize(film) > 1000)
        probe = subprocess.run([ff, '-hide_banner', '-i', film], capture_output=True, text=True).stderr
        self.assertIn('1920x540', probe)                   # two 16:9 sides of 960 x 540
        self.assertIn('30 fps', probe)
        self.assertRegex(probe, r'Duration: 00:00:04\.0\d')   # two 2 s steps at 1x


if __name__ == '__main__':
    unittest.main()
