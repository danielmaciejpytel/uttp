"""Offline tests for select_save.py (temp folders only; the game's real saves are never read).

  python Source\\tools\\gametest\\test_select_save.py -v
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import select_save as ss  # noqa: E402

SEC = 10 ** 9
T0 = 1_700_000_000 * SEC + 123_400          # a time with a sub-second part (the restore must be exact)


def make_save(saves_dir, name, age_s, payload=b'save-data', extra=True):
    """A save folder: sav.dat (+ metadata and screenshot) with every time `age_s` s after T0."""
    d = os.path.join(saves_dir, name)
    os.makedirs(d)
    files = {'sav.dat': payload}
    if extra:
        files.update({'metadata.9.json': b'{"name": "' + name.encode() + b'"}', 'screenshot.png': b'\x89PNG' + name.encode()})
    for fn, data in files.items():
        with open(os.path.join(d, fn), 'wb') as f:
            f.write(data)
    stamp(d, age_s)
    return d


def stamp(d, age_s):
    ns = T0 + age_s * SEC
    for fn in os.listdir(d):
        os.utime(os.path.join(d, fn), ns=(ns, ns))
    os.utime(d, ns=(ns, ns))


def tree_hash(d):
    h = hashlib.sha256()
    for fn in sorted(os.listdir(d)):
        h.update(fn.encode())
        with open(os.path.join(d, fn), 'rb') as f:
            h.update(f.read())
    return h.hexdigest()


def times(d):
    return {fn: os.stat(os.path.join(d, fn)).st_mtime_ns for fn in sorted(os.listdir(d))} | {'.': os.stat(d).st_mtime_ns}


class Base(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp()
        self.saves = os.path.join(self.root, 'Saved Games', 'game')
        os.makedirs(self.saves)
        self.backups = os.path.join(self.root, 'Backups')
        self.state = os.path.join(self.root, 'inventory', 'state.json')
        self.lines: list[str] = []
        self.pids: list[int] = []
        make_save(self.saves, 'AutoSave-1', 50)
        make_save(self.saves, 'ManualSave-100', 100)
        make_save(self.saves, 'ManualSave-106', 20)             # the male V save: older than ManualSave-100
        with open(os.path.join(self.saves, 'EnabledMods.json'), 'w') as f:    # a file that is not a save
            f.write('{}')

    def tearDown(self):
        shutil.rmtree(self.root, ignore_errors=True)

    def out(self, line):
        self.lines.append(line)

    def running(self):
        return list(self.pids)

    def argv(self, *a):
        return ['--saves-dir', self.saves, '--state-file', self.state, '--backup-dir', self.backups, *a]

    def run_main(self, *a, now_ns=T0 + 500 * SEC):
        self.lines.clear()
        return ss.main(self.argv(*a), running=self.running, out=self.out, now_ns=now_ns)

    def snap_all(self):
        return {n: times(os.path.join(self.saves, n)) for n in sorted(os.listdir(self.saves))
                if os.path.isdir(os.path.join(self.saves, n))}

    def hashes(self):
        return {n: tree_hash(os.path.join(self.saves, n)) for n in sorted(os.listdir(self.saves))
                if os.path.isdir(os.path.join(self.saves, n))}


class ListTests(Base):
    def test_lists_newest_first_and_ignores_files_and_folders_that_are_not_saves(self):
        os.makedirs(os.path.join(self.saves, 'not_a_save'))
        self.assertEqual(self.run_main(), 0)
        names = [l.split()[0] for l in self.lines]
        self.assertEqual(names, ['ManualSave-100', 'AutoSave-1', 'ManualSave-106'])
        self.assertTrue(self.lines[0].endswith('<- newest'))
        self.assertIn('3 files', self.lines[0])


class SelectTests(Base):
    def test_select_sets_only_times_backs_up_and_remembers(self):
        before_times, before_hash = self.snap_all(), self.hashes()
        self.assertEqual(self.run_main('ManualSave-106'), 0, self.lines)
        # content untouched; only the chosen save's times changed, to a time newer than every other save
        self.assertEqual(self.hashes(), before_hash)
        after = self.snap_all()
        for n in ('AutoSave-1', 'ManualSave-100'):
            self.assertEqual(after[n], before_times[n])
        chosen = after['ManualSave-106']
        self.assertEqual(set(chosen.values()), {T0 + 500 * SEC})                       # the files and the folder
        # one line per save
        self.assertEqual(len(self.lines), 3)
        self.assertEqual(sum('selected' in l for l in self.lines), 1)
        self.assertTrue(all('unchanged' in l for l in self.lines if 'selected' not in l))
        self.assertNotIn(self.root, ' '.join(self.lines))                                # no local paths in the output
        # the backup is an exact copy with the original times
        bk = os.path.join(self.backups, __import__('datetime').datetime.now().strftime('%Y-%m-%d'), 'saves', 'ManualSave-106')
        self.assertEqual(tree_hash(bk), before_hash['ManualSave-106'])
        self.assertEqual(times(bk)['sav.dat'], before_times['ManualSave-106']['sav.dat'])
        # the state holds every save's original times (names and times only)
        with open(self.state, encoding='utf-8') as f:
            st = json.load(f)
        self.assertEqual(st['chosen'], 'ManualSave-106')
        self.assertEqual(set(st['saves']), {'AutoSave-1', 'ManualSave-100', 'ManualSave-106'})
        self.assertEqual(st['saves']['ManualSave-106']['files']['sav.dat'][1], before_times['ManualSave-106']['sav.dat'])
        self.assertEqual(st['saves']['ManualSave-106']['dir_ns'], before_times['ManualSave-106']['.'])
        self.assertNotIn(self.root, json.dumps(st))

    def test_clock_behind_the_other_saves_still_makes_it_newest(self):
        self.assertEqual(self.run_main('ManualSave-106', now_ns=T0 - 10 * SEC), 0)
        newest = max(ss.newest_ns(s) for s in ss.scan_saves(self.saves).values())
        self.assertEqual(newest, ss.newest_ns(ss.scan_saves(self.saves)['ManualSave-106']))
        self.assertGreater(ss.newest_ns(ss.scan_saves(self.saves)['ManualSave-106']), T0 + 100 * SEC)

    def test_select_twice_changes_nothing_and_keeps_the_originals(self):
        self.run_main('ManualSave-106')
        with open(self.state, encoding='utf-8') as f:
            first = f.read()
        mid = self.snap_all()
        backups_before = sorted(os.listdir(self.backups))
        self.assertEqual(self.run_main('ManualSave-106', now_ns=T0 + 900 * SEC), 0)
        self.assertEqual(self.snap_all(), mid)                                           # not bumped again
        with open(self.state, encoding='utf-8') as f:
            self.assertEqual(f.read(), first)                                           # originals never overwritten
        self.assertEqual(sorted(os.listdir(self.backups)), backups_before)
        self.assertTrue(any('already selected' in l for l in self.lines))
        self.assertEqual(len(self.lines), 3)

    def test_another_save_while_one_is_selected_is_refused(self):
        self.run_main('ManualSave-106')
        mid = self.snap_all()
        self.assertEqual(self.run_main('ManualSave-100'), 1)
        self.assertIn('ManualSave-106 is still selected', self.lines[0])
        self.assertEqual(self.snap_all(), mid)

    def test_refuses_while_the_game_runs_and_changes_nothing(self):
        self.pids = [4242]
        before = self.snap_all()
        self.assertEqual(self.run_main('ManualSave-106'), 1)
        self.assertIn('game is running', self.lines[0])
        self.assertEqual(self.snap_all(), before)
        self.assertFalse(os.path.exists(self.state))
        self.assertFalse(os.path.exists(self.backups))
        self.assertEqual(self.run_main('--restore'), 1)
        self.assertIn('game is running', self.lines[0])

    def test_unknown_name_and_a_folder_without_a_save_are_refused(self):
        os.makedirs(os.path.join(self.saves, 'empty'))
        for bad in ('Nope', 'empty'):
            self.assertEqual(self.run_main(bad), 1)
            self.assertIn('no save named', self.lines[0])
        self.assertFalse(os.path.exists(self.state))

    def test_missing_saves_folder(self):
        shutil.rmtree(self.saves)
        self.assertEqual(self.run_main(), 1)
        self.assertIn('does not exist', self.lines[0])
        self.assertNotIn(self.root, self.lines[0])

    def test_identical_backup_anywhere_is_reused_a_different_one_is_not_overwritten(self):
        old = os.path.join(self.backups, '2026-01-01', 'saves', 'ManualSave-106')
        shutil.copytree(os.path.join(self.saves, 'ManualSave-106'), old)
        self.run_main('ManualSave-106')
        self.assertEqual(sorted(os.listdir(self.backups)), ['2026-01-01'])             # nothing new was copied
        self.assertIn('backup exists', ' '.join(self.lines))
        self.run_main('--restore')
        # a copy of the same name that differs: kept as it is, the new copy gets another folder
        with open(os.path.join(old, 'sav.dat'), 'ab') as f:
            f.write(b'x')
        self.run_main('ManualSave-106')
        days = sorted(os.listdir(self.backups))
        self.assertEqual(len(days), 2)
        with open(os.path.join(old, 'sav.dat'), 'rb') as f:
            self.assertTrue(f.read().endswith(b'x'))


class RestoreTests(Base):
    def test_restore_puts_every_time_back_exactly_and_forgets_the_state(self):
        before = self.snap_all()
        before_hash = self.hashes()
        self.run_main('ManualSave-106')
        self.assertNotEqual(self.snap_all(), before)
        self.assertEqual(self.run_main('--restore'), 0, self.lines)
        self.assertEqual(self.snap_all(), before)                                        # to the nanosecond, folder included
        self.assertEqual(self.hashes(), before_hash)
        self.assertFalse(os.path.exists(self.state))
        self.assertEqual(len(self.lines), 3)
        self.assertEqual(sum('restored' in l for l in self.lines), 1)
        newest = max(ss.scan_saves(self.saves).items(), key=lambda kv: ss.newest_ns(kv[1]))[0]
        self.assertEqual(newest, 'ManualSave-100')                                       # the user's own newest save again

    def test_restore_twice_and_without_state_is_a_no_op(self):
        self.assertEqual(self.run_main('--restore'), 0)
        self.assertIn('nothing to restore', self.lines[0])
        self.run_main('ManualSave-106')
        self.run_main('--restore')
        before = self.snap_all()
        self.assertEqual(self.run_main('--restore'), 0)
        self.assertEqual(self.snap_all(), before)

    def test_reports_a_save_the_game_created_and_one_it_changed(self):
        self.run_main('ManualSave-106')
        make_save(self.saves, 'ManualSave-107', 700)                                     # created meanwhile
        changed = os.path.join(self.saves, 'ManualSave-100', 'sav.dat')
        with open(changed, 'ab') as f:                                                   # another save rewritten by the game
            f.write(b'more')
        content_107, content_100 = tree_hash(os.path.join(self.saves, 'ManualSave-107')), tree_hash(os.path.dirname(changed))
        self.assertEqual(self.run_main('--restore'), ss.EXIT_GAME_CHANGED)
        text = '\n'.join(self.lines)
        self.assertIn('ManualSave-107', text)
        self.assertIn('created meanwhile', text)
        self.assertIn('ManualSave-100  ', text)
        self.assertIn('changed by the game meanwhile', text)
        self.assertEqual(sum('restored' in l for l in self.lines), 1)                    # the chosen one is still put back
        self.assertEqual(tree_hash(os.path.join(self.saves, 'ManualSave-107')), content_107)    # nothing edited or deleted
        self.assertEqual(tree_hash(os.path.dirname(changed)), content_100)
        self.assertTrue(os.path.isdir(os.path.join(self.saves, 'ManualSave-107')))

    def test_the_chosen_save_changed_by_the_game_keeps_its_time(self):
        self.run_main('ManualSave-106')
        sav = os.path.join(self.saves, 'ManualSave-106', 'sav.dat')
        with open(sav, 'ab') as f:                                                       # the game saved over it
            f.write(b'new')
        os.utime(sav, ns=(T0 + 800 * SEC, T0 + 800 * SEC))
        mid = self.snap_all()
        self.assertEqual(self.run_main('--restore'), ss.EXIT_GAME_CHANGED)
        self.assertEqual(self.snap_all(), mid)                                           # time left alone
        self.assertTrue(any('ManualSave-106' in l and 'changed by the game since it was selected' in l for l in self.lines))
        self.assertFalse(os.path.exists(self.state))
        self.assertEqual(self.run_main('--restore'), 0)

    def test_a_save_removed_meanwhile_is_reported(self):
        self.run_main('ManualSave-106')
        shutil.rmtree(os.path.join(self.saves, 'AutoSave-1'))
        self.assertEqual(self.run_main('--restore'), ss.EXIT_GAME_CHANGED)
        self.assertTrue(any('AutoSave-1' in l and 'missing now' in l for l in self.lines))
        self.assertEqual(sum('restored' in l for l in self.lines), 1)

    def test_restore_takes_no_name_and_a_bad_state_file_is_refused(self):
        self.assertEqual(self.run_main('--restore', 'ManualSave-106'), 1)
        os.makedirs(os.path.dirname(self.state))
        with open(self.state, 'w') as f:
            f.write('not json')
        before = self.snap_all()
        self.assertEqual(self.run_main('ManualSave-106'), 1)
        self.assertIn('state file', self.lines[0])
        self.assertEqual(self.run_main('--restore'), 1)
        self.assertEqual(self.snap_all(), before)
        self.assertTrue(os.path.exists(self.state))                                      # an unreadable state is never deleted


class LocationTests(unittest.TestCase):
    def test_backups_dir_follows_a_working_copy_link_to_the_real_project(self):
        root = tempfile.mkdtemp()
        try:
            real = os.path.join(root, 'real')
            os.makedirs(os.path.join(real, 'Research'))
            os.makedirs(os.path.join(real, 'Backups'))
            wt = os.path.join(root, 'wt')
            os.makedirs(wt)
            link = os.path.join(wt, 'Research')
            try:
                os.symlink(os.path.join(real, 'Research'), link, target_is_directory=True)
            except (OSError, NotImplementedError):
                self.skipTest('no symbolic links here')
            old = os.environ.get('UTPP_PROJECT_ROOT')
            os.environ['UTPP_PROJECT_ROOT'] = wt
            try:
                self.assertEqual(os.path.realpath(ss.backups_dir()), os.path.realpath(os.path.join(real, 'Backups')))
                self.assertEqual(ss.default_state_file(), os.path.join(wt, 'Research', 'inventory', 'select_save_state.json'))
            finally:
                if old is None:
                    del os.environ['UTPP_PROJECT_ROOT']
                else:
                    os.environ['UTPP_PROJECT_ROOT'] = old
        finally:
            shutil.rmtree(root, ignore_errors=True)

    def test_default_saves_dir_is_under_saved_games(self):
        old = os.environ.pop('UTPP_SAVES_DIR', None)
        try:
            d = ss.default_saves_dir()
            self.assertTrue(d.endswith(os.path.join('CD Projekt Red', 'Cyberpunk 2077')))
            os.environ['UTPP_SAVES_DIR'] = 'X'
            self.assertEqual(ss.default_saves_dir(), 'X')
        finally:
            os.environ.pop('UTPP_SAVES_DIR', None)
            if old is not None:
                os.environ['UTPP_SAVES_DIR'] = old

    def test_game_pids_returns_a_list(self):
        self.assertIsInstance(ss.game_pids(), list)


if __name__ == '__main__':
    unittest.main()
