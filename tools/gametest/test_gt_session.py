"""Tests of gt_session: --body (save selection with exact restore), --graphics, the background requirements and the side-by-side summary.
Only temporary folders: no real save, settings file or state file is touched; the game is never started.

  python Source\\tools\\gametest\\test_gt_session.py -v
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
import unittest
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import graphics_profile  # noqa: E402
import gt_session as gs  # noqa: E402
import select_save  # noqa: E402

DAY = 86400 * 10 ** 9


class SavesBase(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='gs_')
        self.saves = os.path.join(self.root, 'saves')
        self.out_root = os.path.join(self.root, 'night')
        self.state = os.path.join(self.root, 'state.json')
        self.backups = os.path.join(self.root, 'backups')
        self.lines: list[str] = []
        self.running: list[int] = []
        base = 1_700_000_000 * 10 ** 9
        for i, name in enumerate(('ManualSave-106', 'ManualSave-114', 'ManualSave-115')):
            d = os.path.join(self.saves, name)
            os.makedirs(d)
            for fn in ('sav.dat', 'metadata.9.json', 'screenshot.png'):
                p = os.path.join(d, fn)
                with open(p, 'wb') as f:
                    f.write(name.encode() + fn.encode())
                os.utime(p, ns=(base, base + (i + 1) * DAY))
            os.utime(d, ns=(base, base + (i + 1) * DAY))
        self.before = self.times()

    def tearDown(self):
        shutil.rmtree(self.root, ignore_errors=True)

    def session(self, **kw):
        return gs.SaveSession(self.out_root, saves_dir=self.saves, state_file=self.state, backup_dir=self.backups,
                              running=lambda: list(self.running), out=self.lines.append, **kw)

    def times(self):
        out = {}
        for dp, _dn, fns in os.walk(self.saves):
            out[dp] = os.stat(dp).st_mtime_ns
            for fn in fns:
                out[os.path.join(dp, fn)] = os.stat(os.path.join(dp, fn)).st_mtime_ns
        return out

    def newest(self):
        return max(select_save.scan_saves(self.saves).items(), key=lambda kv: select_save.newest_ns(kv[1]))[0]


class BodyTests(unittest.TestCase):
    def test_bodies(self):
        self.assertEqual(gs.bodies_of('keep'), [])
        self.assertEqual(gs.bodies_of('female'), ['female'])
        self.assertEqual(gs.bodies_of('male'), ['male'])
        self.assertEqual(gs.bodies_of('both'), ['female', 'male'])
        with self.assertRaises(gs.SessionError):
            gs.bodies_of('child')
        self.assertEqual(gs.BODY_SAVES, {'female': 'ManualSave-115', 'male': 'ManualSave-106'})

    def test_min_idle_default_follows_background(self):
        self.assertEqual(gs.effective_min_idle(SimpleNamespace(min_idle=None, background=False)), 120.0)
        self.assertEqual(gs.effective_min_idle(SimpleNamespace(min_idle=None, background=True)), 0.0)
        self.assertEqual(gs.effective_min_idle(SimpleNamespace(min_idle=45.0, background=True)), 45.0)
        self.assertEqual(gs.effective_min_idle(SimpleNamespace(min_idle=0.0, background=False)), 0.0)


class SaveSessionTests(SavesBase):
    def test_select_makes_the_body_newest_and_restore_puts_every_time_back_exactly(self):
        self.assertEqual(self.newest(), 'ManualSave-115')
        s = self.session()
        s.select('male')
        self.assertEqual(self.newest(), 'ManualSave-106')                      # the male save is now the newest
        self.assertTrue(os.path.isfile(s.marker) and os.path.isfile(self.state))
        self.assertTrue(os.path.isdir(self.backups))                          # the backup copy exists
        res = s.restore()
        self.assertEqual(res['code'], 0)
        self.assertEqual(self.times(), self.before)                          # every file and folder time, to the nanosecond
        self.assertFalse(os.path.exists(s.marker) or os.path.exists(self.state))
        self.assertEqual(self.newest(), 'ManualSave-115')

    def test_both_bodies_one_after_the_other_each_restored(self):
        s = self.session()
        for body, save in (('female', 'ManualSave-115'), ('male', 'ManualSave-106')):
            s.select(body)
            self.assertEqual(self.newest(), save)
            s.restore()
            self.assertEqual(self.times(), self.before)

    def test_a_running_game_is_never_touched(self):
        self.running = [4321]
        s = self.session()
        with self.assertRaises(gs.SessionError) as cm:
            s.select('female')
        self.assertIn('not started by this run', str(cm.exception))
        self.assertEqual(self.times(), self.before)
        self.assertFalse(os.path.exists(s.marker) or os.path.exists(self.state))

    def test_a_foreign_selection_is_refused_and_left_alone(self):
        select_save.do_select(self.saves, 'ManualSave-114', self.state, self.backups, running=lambda: [], out=lambda m: None)
        changed = self.times()
        s = self.session()
        with self.assertRaises(gs.SessionError) as cm:
            s.select('male')
        self.assertIn('select_save.py --restore first', str(cm.exception))
        self.assertEqual(self.times(), changed)
        self.assertFalse(os.path.exists(s.marker))                          # not ours: never restored by the runner
        self.assertIsNone(self.session().restore_leftover())
        self.assertTrue(os.path.isfile(self.state))

    def test_a_leftover_selection_is_restored_first_on_the_next_start(self):
        self.session().select('male')                                       # the run died here: no restore
        self.assertNotEqual(self.times(), self.before)
        later = self.session()
        text = later.restore_leftover()
        self.assertIn('ManualSave-106', text)
        self.assertEqual(self.times(), self.before)
        self.assertIsNone(later.restore_leftover())                         # nothing left to do the second time
        self.assertTrue(any('restored first' in l for l in self.lines))

    def test_restore_waits_for_a_game_that_is_still_open(self):
        s = self.session()
        s.select('male')
        self.running = [99]
        res = s.restore()
        self.assertFalse(res['restored'])
        self.assertTrue(os.path.isfile(self.state) and os.path.isfile(s.marker))      # kept for the next start
        self.assertTrue(any('RESTORE PENDING' in l for l in self.lines))
        self.running = []
        self.assertTrue(self.session().restore()['restored'])
        self.assertEqual(self.times(), self.before)

    def test_a_save_the_game_made_meanwhile_is_left_alone_and_reported(self):
        s = self.session()
        s.select('female')
        made = os.path.join(self.saves, 'AutoSave-1')
        os.makedirs(made)
        with open(os.path.join(made, 'sav.dat'), 'wb') as f:
            f.write(b'x')
        res = s.restore()
        self.assertEqual(res['code'], select_save.EXIT_GAME_CHANGED)
        self.assertTrue(os.path.isdir(made))
        self.assertTrue(any('changed saves' in l for l in self.lines))

    def test_unknown_save_name_changes_nothing(self):
        s = self.session(saves={'female': 'ManualSave-999'})
        with self.assertRaises(gs.SessionError):
            s.select('female')
        self.assertEqual(self.times(), self.before)
        self.assertFalse(os.path.exists(s.marker) or os.path.exists(self.state))


class GraphicsTests(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='gg_')
        self.settings = os.path.join(self.d, 'UserSettings.json')
        with open(self.settings, 'w', encoding='utf-8') as f:
            json.dump({'data': [{'group_name': '/graphics/basic', 'options': [
                {'name': 'FilmGrain', 'value': True}, {'name': 'Vignette', 'value': True},
                {'name': 'MotionBlur', 'value': 'High', 'index': 3, 'values': ['Off', 'Low', 'Medium', 'High']}]},
                {'group_name': '/audio', 'options': [{'name': 'Volume', 'value': 70}]}]}, f)
        self._running = graphics_profile.game_running
        graphics_profile.game_running = lambda: False

    def tearDown(self):
        graphics_profile.game_running = self._running
        shutil.rmtree(self.d, ignore_errors=True)

    def read(self):
        with open(self.settings, encoding='utf-8') as f:
            return {o['name']: o['value'] for g in json.load(f)['data'] for o in g['options']}

    def test_low_changes_only_the_profile_options(self):
        sd = os.path.join(self.d, 'state')
        res = gs.apply_graphics('low', self.settings, sd)
        self.assertEqual(res['code'], 0)
        self.assertIn('graphics profile: low', res['status'])
        self.assertIn('put back when the session ends', res['status'])
        self.assertEqual(self.read(), {'FilmGrain': False, 'Vignette': False, 'MotionBlur': 'Off', 'Volume': 70})
        self.assertTrue(os.path.isfile(os.path.join(sd, 'state.json')))                    # the previous values are remembered
        self.assertIn('applied', gs.graphics_status(self.settings, sd))
        # a second launch of the session: nothing changes again, the remembered values still belong to the first apply
        gs.apply_graphics('low', self.settings, sd)
        with open(os.path.join(sd, 'state.json'), encoding='utf-8') as f:
            self.assertEqual(json.load(f)['previous']['/graphics/basic|FilmGrain']['value'], True)

    def test_keep_touches_nothing(self):
        before = self.read()
        res = gs.apply_graphics('keep', self.settings, os.path.join(self.d, 'state'))
        self.assertIn('not touched', res['status'])
        self.assertEqual(self.read(), before)
        self.assertFalse(os.path.exists(os.path.join(self.d, 'state')))

    def test_a_running_game_is_reported_not_changed(self):
        graphics_profile.game_running = lambda: True
        res = gs.apply_graphics('low', self.settings, os.path.join(self.d, 'state'))
        self.assertEqual(res['code'], 3)
        self.assertIn('NOT applied', res['status'])
        self.assertEqual(self.read()['FilmGrain'], True)

    def test_restore_puts_the_players_values_back_and_counts_the_options(self):
        sd = os.path.join(self.d, 'state')
        before = self.read()
        gs.apply_graphics('low', self.settings, sd)
        gs.apply_graphics('low', self.settings, sd)                        # a second session found it applied
        res = gs.restore_graphics(self.settings, sd)
        self.assertEqual(res['code'], 0)
        self.assertEqual(res['restored'], 3)
        self.assertEqual(res['status'], 'graphics profile restored: 3 options (the player\'s values are back)')
        self.assertEqual(self.read(), before)
        res2 = gs.restore_graphics(self.settings, sd)                      # nothing is applied any more: nothing changes
        self.assertEqual((res2['code'], res2['restored']), (0, 0))
        self.assertIn('nothing to restore', res2['status'])
        self.assertEqual(self.read(), before)

    def test_restore_counts_only_the_options_it_found(self):
        sd = os.path.join(self.d, 'state')
        gs.apply_graphics('low', self.settings, sd)
        with open(self.settings, encoding='utf-8') as f:
            doc = json.load(f)
        doc['data'][0]['options'] = [o for o in doc['data'][0]['options'] if o['name'] != 'Vignette']
        with open(self.settings, 'w', encoding='utf-8') as f:
            json.dump(doc, f)
        res = gs.restore_graphics(self.settings, sd)
        self.assertEqual(res['restored'], 2)

    def test_restore_writes_nothing_while_the_game_runs(self):
        sd = os.path.join(self.d, 'state')
        gs.apply_graphics('low', self.settings, sd)
        low = self.read()
        graphics_profile.game_running = lambda: True
        res = gs.restore_graphics(self.settings, sd)
        self.assertEqual(res['code'], 3)
        self.assertIn('NOT restored', res['status'])
        self.assertIn('graphics_profile.py --restore', res['status'])
        self.assertEqual(self.read(), low)
        self.assertTrue(os.path.isfile(os.path.join(sd, 'state.json')))    # still remembered for the next try


class BackgroundTests(unittest.TestCase):
    def test_requirements_name_what_is_missing(self):
        d = tempfile.mkdtemp(prefix='gb_')
        try:
            game, tc = os.path.join(d, 'game'), os.path.join(d, 'testcmd')
            args = SimpleNamespace(cmd_channel='auto')
            req = gs.background_requirements(game, tc, args, None, 'no Tools\\ffmpeg-<version> build has the gfxcapture filter')
            self.assertEqual([ok for _, ok, _ in req], [False, False, False, True])
            self.assertIn('gfxcapture', req[2][2])
            os.makedirs(tc)
            open(os.path.join(tc, 'init.lua'), 'w').close()
            os.makedirs(os.path.join(game, 'red4ext', 'plugins', 'UTPPTestHost'))
            open(os.path.join(game, 'red4ext', 'plugins', 'UTPPTestHost', 'UTPPTestHost.dll'), 'w').close()
            req = gs.background_requirements(game, tc, args, 'ffmpeg8.exe')
            self.assertTrue(all(ok for _, ok, _ in req))
            req = gs.background_requirements(game, tc, SimpleNamespace(cmd_channel='console'), 'ffmpeg8.exe')
            self.assertFalse(req[3][1])
        finally:
            shutil.rmtree(d, ignore_errors=True)


def _dump(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f)


class SideBySideTests(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='sb_')

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    def run_dir(self, name, body, verdict, head, anatomy):
        r = os.path.join(self.d, name)
        _dump(os.path.join(r, 'cycle.json'), {'result': 'ok', 'perf_status': 'feedP95us=90 budget=250',
                                              'scenarios': [{'name': 'kbd_taps', 'run': 1, 'verdict': verdict}]})
        _dump(os.path.join(r, 'summary.json'), {'harness': {'overall': 'PASS'}})
        _dump(os.path.join(r, 'gametest', 'summary.json'), {'steps': [
            {'scenario': 'kbd_taps', 'run': 1, 'step': 'a', 'verdict': 'PASS'}, {'scenario': 'kbd_taps', 'run': 1, 'step': 'b', 'verdict': 'FAIL'}]})
        _dump(os.path.join(r, 'pose_metrics.json'), {'body': body, 'rows': [
            {'label': 'kbd_taps#1', 'duration_s': 8.0, 'head_speed_p95': head, 'chest_speed_p95': 200.0, 'spikes_per_s': 0.31,
             'helper_p95_cm': 4.4, 'anatomy_share': anatomy}, {'label': 'kbd_taps#1/a', 'head_speed_p95': 1.0}]})
        return r

    def test_digest_and_table(self):
        f = gs.run_digest(self.run_dir('r1', 'woman', None, 112.0, 0.132))
        m = gs.run_digest(self.run_dir('r2', 'man', 'FAIL', 140.0, 0.05))
        self.assertEqual(f['pose_body'], 'woman')
        self.assertEqual(f['scenarios']['kbd_taps#1']['steps'], {'PASS': 1, 'FAIL': 1})
        self.assertNotIn('kbd_taps#1/a', f['scenarios'])                                  # step windows are not scenarios
        lines = gs.side_by_side_lines({'female': f, 'male': m})
        self.assertIn('female: night', lines[0])
        self.assertIn('male: night', lines[0])
        row = next(l for l in lines if l.startswith('kbd_taps#1'))
        self.assertIn('112', row)
        self.assertIn('140', row)
        self.assertIn('F1 P1', row)
        self.assertIn('13.2', row)
        self.assertTrue(any(l.startswith('female: run r1') for l in lines))
        self.assertTrue(any(l.startswith('male: run r2') and 'pose body man' in l for l in lines))

    def test_missing_files_do_not_raise(self):
        dg = gs.run_digest(os.path.join(self.d, 'nothing'))
        self.assertEqual(dg['scenarios'], {})
        self.assertTrue(gs.side_by_side_lines({'female': dg}))

    def test_pose_owner_in_the_result_line(self):
        f = gs.run_digest(self.run_dir('r1', 'woman', None, 112.0, 0.132))
        m = gs.run_digest(self.run_dir('r2', 'man', None, 140.0, 0.05))
        self.assertTrue(any(l.startswith('female: run r1') and 'pose owner unknown' in l for l in gs.side_by_side_lines({'female': f})))
        f['pose_owner'] = gs.gt_log.pose_owner_of_log(os.path.join(TESTDATA, 'pose_owner_fk.txt'))
        m['pose_owner'] = gs.gt_log.pose_owner_of_log(os.path.join(TESTDATA, 'pose_owner_fk.txt'))
        lines = gs.side_by_side_lines({'female': f, 'male': m})
        self.assertTrue(any(l.startswith('female: run r1') and 'pose owner FK' in l for l in lines))
        self.assertFalse([l for l in lines if 'POSE OWNER DIFFERS' in l])
        m['pose_owner'] = gs.gt_log.pose_owner_of_log(os.path.join(TESTDATA, 'pose_owner_mapping.txt'))
        lines = gs.side_by_side_lines({'female': f, 'male': m})
        self.assertTrue(any(l.startswith('male: run r2') and 'pose owner Mapping' in l for l in lines))
        self.assertEqual([l for l in lines if 'POSE OWNER DIFFERS' in l], ['!!! POSE OWNER DIFFERS: female FK, male Mapping - these runs '
                                                                         'are not comparable'])
        # the digest reads the owner from the run's summary.json (written by night.py)
        r = self.run_dir('r3', 'woman', None, 1.0, 0.1)
        _dump(os.path.join(r, 'summary.json'), {'pose_owner': m['pose_owner']})
        self.assertEqual(gs.run_digest(r)['pose_owner']['owner'], 'Mapping')


TESTDATA = os.path.join(HERE, 'testdata')


class PoseOwnerTests(unittest.TestCase):
    """gt_log.pose_owner on log fixtures: the load line of the switch file and earlyOwner of the FK counters lines."""
    def owner(self, name):
        return gs.gt_log.pose_owner_of_log(os.path.join(TESTDATA, name))

    def test_fixtures(self):
        po = self.owner('pose_owner_mapping.txt')
        self.assertEqual((po['owner'], po['owner_line'], po['early_owner'], po['catalog_valid'], po['helpers']),
                         ('Mapping', True, [1], 1, 116))
        self.assertEqual(gs.gt_log.pose_owner_label(po), 'Mapping')
        po = self.owner('pose_owner_fk.txt')
        self.assertEqual((po['owner'], po['owner_line'], po['early_owner']), ('FK', False, [0]))
        po = self.owner('pose_owner_unknown.txt')
        self.assertEqual((po['owner'], po['early_owner']), ('unknown', []))
        po = self.owner('pose_owner_conflict.txt')
        self.assertEqual(po['owner'], 'conflict')
        self.assertTrue(gs.gt_log.pose_owner_label(po).startswith('CONFLICT (load line says Mapping'))
        po = self.owner('pose_owner_mapping_invalid_catalog.txt')
        self.assertEqual((po['owner'], po['catalog_valid']), ('Mapping', 0))
        self.assertIn('catalogValid=0', gs.gt_log.pose_owner_label(po))
        self.assertEqual(self.owner('no_such_file.txt')['owner'], 'unknown')
        self.assertEqual(gs.gt_log.pose_owner_label(None), 'unknown')

    def test_text_rules(self):
        fk = 'UTPP FK counters (tpp): totalCalls=1 (+1) mmActive=1 earlyOwner=%d earlyApplied=0\n'
        load = 'UTPP early pose experiment: owner=Mapping catalogValid=1 helpers=116 restartRequired=1\n'
        po = gs.gt_log.pose_owner
        self.assertEqual(po(load)['owner'], 'Mapping')                      # the load line alone (no third-person sample yet)
        self.assertEqual(po(fk % 1)['owner'], 'conflict')                   # earlyOwner=1 without the load line
        self.assertEqual(po(fk % 0 + fk % 1)['owner'], 'conflict')          # mixed values
        self.assertEqual(po(load + fk % 1 + fk % 1)['owner'], 'Mapping')
        self.assertEqual(po(load.replace('Mapping', 'FK'))['owner'], 'conflict')


if __name__ == '__main__':
    unittest.main()
