"""Offline tests: the UTPP log rotates at 10 MB while a gametest runs (temp dir, synthetic files, no game).

  python Source\\tools\\gametest\\test_log_rotation.py -v

2026-11-01 (22:23-23:00): utpp-<start>.log was renamed to .1.log (the older .1 to .2) and a new utpp-<start>.log started.
A marker / byte offset taken before the rename must still be found / valid after it. Covered: gt_log (session_files,
LogMark, log_size, read_*_since, parse_file, newest_log), gt_input (file_size / read_since, UTPPLogLine verification),
car_cases_nav (nav --since, found, lines, newest, logmark) and bt_gate.parse_log.
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import bt_gate  # noqa: E402
import car_cases_nav as nav  # noqa: E402
import gt_input  # noqa: E402
import gt_log  # noqa: E402

BASE = 'utpp-2026-11-01-22-23-07'


def line(sec: int, text: str, minute: int = 30) -> str:
    return f'[2026-11-01 22:{minute:02d}:{sec:02d}.000] [info    ] [ 1] [UTPP] UTPP {text}\n'


def pos_line(sec: int, minute: int = 30) -> str:
    return line(sec, 'CET [night] POS p=0.00,0.00,0.00 cf=0.000,1.000 v=0.00,0.00,0.00,1.000,0.000 src=tag w=0 m=0', minute)


def write(path: str, text: str, mode: str = 'w') -> None:
    with open(path, mode, encoding='utf-8', newline='') as f:
        f.write(text)


class RotationBase(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='utpp-rot-')
        self.addCleanup(shutil.rmtree, self.dir, True)
        self.cur = os.path.join(self.dir, BASE + '.log')

    def p(self, suffix: str = '') -> str:
        return os.path.join(self.dir, BASE + suffix + '.log')

    def rotate(self):
        """The logger's rotation: .1 -> .2 (and so on), current -> .1, a new empty current file."""
        idxs = sorted((int(n[len(BASE) + 1:-4]) for n in os.listdir(self.dir)
                       if n.startswith(BASE + '.') and n[len(BASE) + 1:-4].isdigit()), reverse=True)
        for i in idxs:
            os.replace(self.p(f'.{i}'), self.p(f'.{i + 1}'))
        os.replace(self.cur, self.p('.1'))
        write(self.cur, '')


class SessionFiles(RotationBase):
    def test_order_is_content_time_not_name(self):
        write(self.p('.2'), line(1, 'a', 23) + line(2, 'b', 23))
        write(self.p('.1'), line(1, 'c', 41) + line(2, 'd', 41))
        write(self.cur, line(1, 'e', 58))
        names = [os.path.basename(f) for f in gt_log.session_files(self.cur)]
        self.assertEqual(names, [BASE + '.2.log', BASE + '.1.log', BASE + '.log'])

    def test_mid_rotation_state_second_rename_not_done_yet(self):
        # .1 -> .2 done, current -> .1 not yet: only .2 and the full current file exist
        write(self.p('.2'), line(1, 'old', 23))
        write(self.cur, line(1, 'new', 41))
        self.assertEqual([os.path.basename(f) for f in gt_log.session_files(self.cur)], [BASE + '.2.log', BASE + '.log'])

    def test_empty_new_current_file_is_last(self):
        write(self.p('.1'), line(1, 'a', 23))
        write(self.cur, '')
        self.assertEqual([os.path.basename(f) for f in gt_log.session_files(self.cur)], [BASE + '.1.log', BASE + '.log'])

    def test_other_sessions_and_names_are_not_mixed_in(self):
        write(self.cur, line(1, 'a'))
        other = os.path.join(self.dir, 'utpp-2026-11-01-10-00-00.log')
        write(other, line(1, 'b', 10))
        write(os.path.join(self.dir, 'utpp-2026-11-01-10-00-00.1.log'), line(2, 'c', 10))
        self.assertEqual(gt_log.session_files(self.cur), [self.cur])
        # an explicit rotated name is just that file; a non-utpp name too
        self.assertEqual(gt_log.session_files(self.p('.1')), [])
        write(self.p('.1'), line(1, 'x', 20))
        self.assertEqual(gt_log.session_files(self.p('.1')), [self.p('.1')])

    def test_newest_log_returns_the_current_file(self):
        write(self.p('.2'), line(1, 'a', 23))
        write(self.p('.1'), line(1, 'b', 41))
        write(self.cur, line(1, 'c', 58))
        os.utime(self.p('.2'), (1000, 1000))
        os.utime(self.cur, (1500, 1500))
        os.utime(self.p('.1'), (2000, 2000))            # the renamed file is the newest by mtime
        self.assertEqual(gt_log.newest_log(self.dir), self.cur)
        older = os.path.join(self.dir, 'utpp-2026-11-01-10-00-00.log')
        write(older, line(1, 'o', 10))
        os.utime(older, (1800, 1800))
        self.assertEqual(gt_log.newest_log(self.dir), self.cur)   # the session is ranked by its newest member


class MarkAcrossRotation(RotationBase):
    def test_marker_and_offset_survive_a_rotation_new_file_grows_past_the_old_cursor(self):
        write(self.cur, line(1, 'before-1') + line(2, 'before-2'))
        size = gt_input.file_size(self.cur)
        old_len = os.path.getsize(self.cur)
        write(self.cur, line(3, '[car] SPAWN case=x ') + line(4, 'ride 7: utpp_vehicle_tpp=1'), 'a')
        self.rotate()
        # the new current file is longer than the old cursor: a stale "offset into the new file" would skip lines
        fill = ''.join(line(s % 60, f'fill {s}', 59) for s in range(80))
        write(self.cur, fill + line(5, 'after-rotation'))
        self.assertGreater(os.path.getsize(self.cur), old_len)
        text = gt_input.read_since(self.cur, size)
        self.assertNotIn('before-1', text)
        self.assertNotIn('before-2', text)
        self.assertTrue(text.startswith('[2026-11-01 22:30:03.000]'))
        self.assertIn('[car] SPAWN case=x ', text)
        self.assertIn('ride 7: utpp_vehicle_tpp=1', text)
        self.assertIn('after-rotation', text)
        self.assertEqual(text.count('fill '), 80)

    def test_offset_taken_before_rotation_new_file_still_empty(self):
        write(self.cur, line(1, 'a') + line(2, 'b'))
        size = gt_input.file_size(self.cur)
        write(self.cur, line(3, 'c'), 'a')
        self.rotate()
        self.assertEqual(gt_input.read_since(self.cur, size).count('UTPP c'), 1)
        self.assertNotIn('UTPP b', gt_input.read_since(self.cur, size))

    def test_bare_int_offset_without_the_remembered_mark(self):
        write(self.cur, line(1, 'a') + line(2, 'b'))
        size = gt_input.file_size(self.cur)
        write(self.cur, line(3, 'c'), 'a')
        self.rotate()
        write(self.cur, line(4, 'd'))
        gt_log._MARKS.clear()                            # another process: only the integer is known
        text = gt_input.read_since(self.cur, size)
        self.assertNotIn('UTPP b', text)
        self.assertIn('UTPP c', text)
        self.assertIn('UTPP d', text)

    def test_two_rotations_and_the_marked_file_deleted(self):
        write(self.cur, line(1, 'a'))
        token = gt_log.log_mark(self.cur).token()
        self.rotate()
        write(self.cur, line(2, 'b', 41))
        self.rotate()
        write(self.cur, line(3, 'c', 58))
        self.assertEqual(len(gt_log.session_files(self.cur)), 3)
        text = gt_log.read_text_since(self.cur, token)      # the file holding the mark is now .2
        self.assertEqual([l[-1] for l in text.splitlines()], ['b', 'c'])
        os.remove(self.p('.2'))                              # the logger dropped its oldest file
        write(self.p('.9'), '')                              # (an unrelated empty sibling must not break the order)
        os.remove(self.p('.9'))
        text = gt_log.read_text_since(self.cur, token)
        self.assertEqual([l[-1] for l in text.splitlines()], ['b', 'c'])      # everything left is newer than the mark

    def test_marked_file_truncated_in_place_older_siblings_are_not_read(self):
        # a failed rename makes the logger truncate the current file: .1 / .2 survive and predate the mark
        write(self.p('.2'), line(1, 'x', 20))
        write(self.p('.1'), line(1, 'y', 25))
        write(self.cur, line(1, 'a'))
        token = gt_log.log_mark(self.cur).token()
        write(self.cur, line(2, 'b', 41))                    # truncated: new first line, new identity
        self.assertEqual([l[-1] for l in gt_log.read_text_since(self.cur, token).splitlines()], ['b'])

    def test_position_behind_the_end_reads_nothing_never_restarts_at_zero(self):
        write(self.cur, line(1, 'old-ride 3: utpp_vehicle_tpp=1'))
        self.assertEqual(gt_log.read_text_since(self.cur, 10 ** 9), '')
        self.assertEqual(gt_input.read_since(self.cur, -1), '')

    def test_wait_for_log_sees_a_line_written_after_the_rotation(self):
        write(self.cur, line(1, 'a'))
        off = gt_input.file_size(self.cur)
        write(self.cur, line(2, 'ride 29: utpp_vehicle_tpp=1 (car mount)'), 'a')
        self.rotate()
        ok = gt_input.wait_for_log(self.cur, off, lambda t: 'ride 29: utpp_vehicle_tpp=1' in t, 0.2, sleep=lambda s: None)
        self.assertTrue(ok)

    def test_parse_file_spans_the_rotation(self):
        write(self.cur, line(1, 'MMTRACE tick |v|=1.0'))
        mark = gt_log.log_mark(self.cur)
        write(self.cur, line(2, 'MMTRACE tick |v|=2.0'), 'a')
        self.rotate()
        write(self.cur, line(3, 'MMTRACE tick |v|=3.0'))
        got = gt_log.parse_file(self.cur, mark)
        self.assertEqual([l.num('|v|') for l in got], [2.0, 3.0])
        self.assertEqual([l.num('|v|') for l in gt_log.parse_file(self.cur)], [1.0, 2.0, 3.0])
        self.assertEqual([l.num('|v|') for l in gt_log.parse_file(self.cur, mark.token())], [2.0, 3.0])

    def test_token_round_trip(self):
        write(self.cur, line(1, 'a'))
        m = gt_log.log_mark(self.cur)
        m2 = gt_log.LogMark.parse(m.token())
        self.assertEqual((m2.virtual, m2.ident, m2.local), (m.virtual, m.ident, m.local))
        self.assertEqual(gt_log.LogMark.parse('123').virtual, 123)
        self.assertEqual(gt_log.LogMark.parse(7).ident, None)

    @unittest.skipUnless(os.name == 'nt', 'Windows share modes')
    def test_readers_do_not_block_the_loggers_rename(self):
        # a handle opened with plain open() (no FILE_SHARE_DELETE) makes the game's rotation rename fail and spdlog then
        # truncates the log it is writing; every reader here must keep the rename possible while its handle is open
        write(self.cur, line(1, 'a'))
        with gt_log._open_shared(self.cur) as fh:
            self.rotate()                                   # os.replace(cur, .1) with our handle open: must not raise
            fh.seek(0)
            self.assertIn(b'UTPP a', fh.read())             # the handle followed the renamed file
        write(self.cur, line(2, 'b'), 'a')
        mark = gt_log.log_mark(self.cur)
        self.assertEqual(gt_log.read_text_since(self.cur, mark), '')
        self.rotate()                                       # nothing is left open after the readers returned
        self.assertEqual(sorted(os.listdir(self.dir)), [BASE + '.1.log', BASE + '.2.log', BASE + '.log'])

    def test_snapshot_identity_size_and_data_come_from_one_handle(self):
        # the new file is longer than the old one; identity and offset must not be mixed between the two files
        write(self.cur, line(1, 'old'))
        token = gt_log.log_mark(self.cur).token()
        self.rotate()
        write(self.cur, line(1, 'new', 41) * 50)
        got = gt_log.read_text_since(self.cur, token)       # all of the new file, none of the old one
        self.assertEqual(got.count('UTPP new'), 50)
        self.assertNotIn('UTPP old', got)

    def test_bt_gate_reads_the_whole_session(self):
        write(self.cur, line(1, 'one'))
        self.rotate()
        write(self.cur, line(2, 'two'))
        self.assertEqual(len(bt_gate.parse_log(self.cur)), 2)


class NavAcrossRotation(RotationBase):
    def run_nav(self, *extra):
        out = os.path.join(self.dir, 'nav.json')
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            nav.main(['nav', '--log', self.cur, '--marker', '[car] SPAWN case=door ', '--vehicle', 'Vehicle.v_standard2_archer_hella_player',
                      '--side', 'driver', '--out-scen', out, '--max-age', '1e9', *extra])
        return json.loads(buf.getvalue().strip().splitlines()[-1])

    def test_marker_in_a_renamed_file_is_found(self):
        write(self.cur, line(1, 'noise') + line(2, 'CET [car] SPAWN case=door ok=true') + pos_line(3))
        self.rotate()
        write(self.cur, pos_line(4, 58))
        r = self.run_nav()
        self.assertNotEqual(r['state'], 'nopos', r)

    def test_since_does_not_fall_back_to_an_older_marker(self):
        # an earlier run (other car) logged the same case name; this run's spawn never logged
        write(self.cur, line(2, 'CET [car] SPAWN case=door ok=true') + pos_line(3))
        mark = gt_log.log_mark(self.cur).token()
        self.rotate()
        write(self.cur, pos_line(5, 58))
        r = self.run_nav('--since', mark)
        self.assertEqual(r['state'], 'nopos')
        self.assertIn('marker', r['why'])
        # once this run's own marker exists after the mark it is used
        write(self.cur, line(6, 'CET [car] SPAWN case=door ok=true', 58) + pos_line(7, 58), 'a')
        self.assertNotEqual(self.run_nav('--since', mark)['state'], 'nopos')

    def cli(self, *args):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            nav.main(list(args))
        return json.loads(buf.getvalue().strip().splitlines()[-1])

    def test_found_and_lines_across_the_rotation(self):
        write(self.cur, line(1, 'ride 1: utpp_vehicle_tpp=1 (old ride)'))
        token = self.cli('logmark', '--log', self.cur)['token']
        write(self.cur, line(2, 'CET [car] SPAWN case=a ok=true') + line(3, 'ride 9: utpp_vehicle_tpp=1 (car mount)')
              + line(4, 'unrelated') + line(5, 'CET [car] SPAWN case=b ok=true') + line(6, 'ride 10: utpp_vehicle_tpp=1'), 'a')
        self.rotate()
        write(self.cur, line(7, 'vehtrans mountdiag: ride=11', 58))
        rx = r'ride \d+: utpp_vehicle_tpp=1|vehtrans mountdiag: ride='
        self.assertTrue(self.cli('found', '--log', self.cur, '--since', token, '--rx', rx)['found'])
        end = self.cli('logmark', '--log', self.cur)['token']
        self.assertFalse(self.cli('found', '--log', self.cur, '--since', end, '--rx', rx)['found'])   # nothing new after the end
        # a mark taken before the rotation, a ride line only in the renamed file
        out = os.path.join(self.dir, 'a-lines.txt')
        r = self.cli('lines', '--log', self.cur, '--since', token, '--marker', '[car] SPAWN case=a ', '--rx', rx + r'|\[car\]', '--out', out)
        self.assertTrue(r['ok'])
        with open(out, encoding='utf-8') as f:
            got = f.read().splitlines()
        self.assertEqual(len(got), 2)                       # SPAWN a + ride 9; the case b part and the later lines are cut off
        self.assertIn('SPAWN case=a', got[0])
        self.assertIn('ride 9', got[1])
        # a case marker that exists only before the mark is not taken
        r = self.cli('lines', '--log', self.cur, '--since', end, '--marker', '[car] SPAWN case=a ', '--rx', rx, '--out', out)
        self.assertFalse(r['ok'])

    def test_newest_cli(self):
        write(self.p('.1'), line(1, 'a', 23))
        write(self.cur, line(1, 'b', 41))
        self.assertEqual(self.cli('newest', '--dir', self.dir)['path'], self.cur)


class LogLineVerification(unittest.TestCase):
    def test_expectation_from_the_command(self):
        e = gt_input.logline_expect("Game.UTPPLogLine('[car] NOT MOUNTED case=normal_cycle_parallel scenario=car_cycle after 2 X presses')")
        self.assertEqual(e, '[car] NOT MOUNTED case=normal_cycle_parallel scenario=car_cycle after 2 X presses')
        self.assertEqual(gt_input.logline_expect('Game.UTPPLogLine("[night] UTPP_MARK a")'), '[night] UTPP_MARK a')
        self.assertEqual(gt_input.logline_expect("Game.UTPPLogLine('[car] SPAWN case=x ok='..tostring(ok))"), '[car] SPAWN case=x ok=')
        self.assertIsNone(gt_input.logline_expect("Game.UTPPLogLine(string.format('%d', 1))"))
        self.assertIsNone(gt_input.logline_expect("Game.UTPPSetMMTrace(true)"))
        self.assertEqual(len(gt_input.logline_expect("Game.UTPPLogLine('" + 'a' * 600 + "')")), gt_input.LOGLINE_CAP)

    def test_seen_in_the_utpp_log_form(self):
        log = '[2026-11-01 22:54:40.277] [info    ] [ 59012] [UTPP] UTPP CET [car] NOT MOUNTED case=a scenario=b after 2 X presses\n'
        self.assertTrue(gt_input.logline_seen(log, '[car] NOT MOUNTED case=a scenario=b after 2 X presses'))
        self.assertFalse(gt_input.logline_seen(log, '[car] NOT MOUNTED case=other'))
        # the old check looked for 'UTPPLogLine(' and could never see this line
        self.assertFalse(gt_input.echo_seen(log, 'UTPPLogLine'))


if __name__ == '__main__':
    unittest.main()
