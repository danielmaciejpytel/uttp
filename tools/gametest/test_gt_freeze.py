"""Offline tests of the frozen-game guard (gt_freeze.py and its wiring in night.py). No input is sent, no game process is touched,
no real dump or nvidia-smi call is made; the one real dump test writes the dump of a throw-away child process of this test.

  python Source\\tools\\gametest\\test_gt_freeze.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
import types
import unittest
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_freeze as gf  # noqa: E402
import gt_input  # noqa: E402
import night  # noqa: E402
import test_night as tn  # noqa: E402
import test_night_data as tnd  # noqa: E402
from test_gametest import GAME_PID, FakeApi  # noqa: E402

GIB = 1 << 30
REAL_WRITE_DUMP = gf.write_dump
REAL_PROCESS_MEMORY = gf.process_memory
REAL_NVIDIA_SMI_REPORT = gf.nvidia_smi_report


def _real(name):
    def refuse(*a, **k):
        raise AssertionError(f'tests must not call the real {name}')
    return refuse


gf.write_dump = _real('dump writer')               # Tools() binds these at construction: a test that forgets to fake them fails
gf.process_memory = _real('process memory reader')
gf.nvidia_smi_report = _real('nvidia-smi')
night.padd_ping = lambda *a, **k: None


def fmt(dt: datetime, msg: str, tid: int = 48792) -> str:
    return f'[{dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}] [info    ] [ {tid}] [UTPP] {msg}'


POSE = 'UTPP pose capture: rows=6486100 frames=35060 dropped=0 postRows=6486100 postFrames=35060'
CAM = 'UTPP CET [utpp_camera] camTrace t=781.56 body=-176.6 out=-177.8 pitch=-3.1 mode=free'


class Rig:
    """A FreezeWatch over a temp log with a fake clock; `flow` writes game-thread lines like the mod does (10 per second)."""

    def __init__(self, tmp: str, seconds: float = 10.0, pids=(GAME_PID,)):
        self.path = os.path.join(tmp, 'session.log')
        open(self.path, 'w').close()
        self.now = datetime(2026, 11, 14, 14, 49, 0)
        self.mono = 1000.0
        self.pids = list(pids)
        self.trips: list = []
        self.reads = 0

        def read(path, off):
            self.reads += 1
            return gt_input.read_since(path, off)
        self.w = gf.FreezeWatch(seconds, lambda: self.path, pids_fn=lambda: list(self.pids), on_trip=self.trips.append,
                                now_fn=lambda: self.now, clock=lambda: self.mono, read_fn=read)
        self.w.start()
        self.last_line = None

    def write(self, msg: str = CAM, tid: int = 48792) -> None:
        with open(self.path, 'a', encoding='utf-8') as f:
            f.write(fmt(self.now, msg, tid) + '\n')
        if tid == 48792 and not msg.startswith(gf.IGNORED_PREFIXES):
            self.last_line = self.now

    def advance(self, s: float) -> None:
        self.now += timedelta(seconds=s)
        self.mono += s

    def poll(self):
        return self.w.poll()

    def flow(self, seconds: float, step: float = 0.1):
        n = int(round(seconds / step))
        ev = None
        for i in range(n):
            self.advance(step)
            self.write()
            if (i + 1) % 10 == 0:
                ev = self.poll() or ev
        return ev

    def silence(self, seconds: float, pose_lines: bool = True):
        """Whole seconds of silence of the game thread; the pose capture thread keeps writing its status line every second."""
        ev = None
        for _ in range(int(seconds)):
            self.advance(1.0)
            if pose_lines:
                self.write(POSE, tid=45480)
            ev = self.poll() or ev
        return ev


class ParseTests(unittest.TestCase):
    def test_pose_capture_lines_are_not_a_sign_of_life(self):
        t = datetime(2026, 11, 14, 14, 49, 1, 528000)
        text = '\n'.join([fmt(t, CAM), fmt(t + timedelta(seconds=5), POSE, 45480), 'garbage', '',
                          fmt(t + timedelta(seconds=6), 'UTPP MMTRACE tick clip=Idle')])
        got = gf.parse_alive_stamps(text)
        self.assertEqual([m[:14] for _, m in got], ['UTPP CET [utpp', 'UTPP MMTRACE t'])
        self.assertEqual(got[0][0], t)


class WatchTests(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.TemporaryDirectory()
        self.addCleanup(self.d.cleanup)

    def rig(self, **k) -> Rig:
        return Rig(self.d.name, **k)

    def test_a_flowing_log_never_trips(self):
        r = self.rig()
        self.assertIsNone(r.flow(30.0))
        self.assertTrue(r.w.armed)
        self.assertEqual(r.trips, [])

    def test_silence_with_only_pose_status_lines_trips_after_the_limit_with_the_last_game_line(self):
        r = self.rig()
        r.flow(9.0)
        last = r.last_line
        self.assertIsNone(r.silence(8))                      # 8 s: still within the limit
        ev = r.silence(3)                                    # 11 s since the last game-thread line
        self.assertIsNotNone(ev)
        self.assertEqual(ev['since'], gf.stamp_text(last))
        self.assertGreaterEqual(ev['silent_s'], 10.0)
        self.assertEqual(ev['pids'], [GAME_PID])
        self.assertTrue(ev['guard_message'].startswith('game not responding since '))
        self.assertIn('no game-thread log line', ev['message'])
        self.assertEqual(len(r.trips), 1)                    # the evidence step ran once
        again = r.poll()
        self.assertIs(again, ev)                             # sticky: the same evidence, no second capture
        r.advance(5.0)
        self.assertIs(r.poll(), ev)
        self.assertEqual(len(r.trips), 1)

    def test_the_guard_message_carries_no_user_stop_phrase(self):
        r = self.rig()
        r.flow(9.0)
        ev = r.silence(12)
        for phrase in ('focus lost', 'not the game', 'abort key', 'Ctrl+C'):
            self.assertNotIn(phrase, ev['guard_message'])
        self.assertFalse(night.is_user_stop(ev['guard_message']))

    def test_a_sparse_log_never_arms_the_watch(self):
        r = self.rig()
        for _ in range(12):                                  # one line every 5 s: not dense enough to prove anything
            r.advance(5.0)
            r.write()
            self.assertIsNone(r.poll())
        self.assertFalse(r.w.armed)
        self.assertIsNone(r.silence(40))
        self.assertEqual(r.trips, [])

    def test_a_short_burst_of_lines_does_not_arm_the_watch(self):
        r = self.rig()
        r.flow(5.0)                                          # dense, but not for ARM_SPAN_S: an event-driven log looks like this
        self.assertFalse(r.w.armed)
        self.assertIsNone(r.silence(40))
        self.assertEqual(r.trips, [])

    def test_switching_the_diagnostic_logs_off_disarms_until_the_flow_is_steady_again(self):
        r = self.rig()
        r.flow(9.0)
        self.assertTrue(r.w.armed)
        r.advance(0.1)
        r.write('UTPP MM feed dump off: lines=19123 dropped=0 file=red4ext\\log')
        r.write('UTPPDiagWriteReport: red4ext\\logs\\utpp-report-2026-11-13-09-17-08.txt')
        self.assertIsNone(r.silence(30))                     # the camera / motion lines stopped on purpose, the game is fine
        self.assertFalse(r.w.armed)
        self.assertEqual(r.trips, [])
        r.flow(10.0)                                         # a steady flow of any kind arms it again
        self.assertTrue(r.w.armed)
        self.assertIsNotNone(r.silence(12))

    def test_a_gap_above_the_arm_limit_restarts_the_steady_flow(self):
        r = self.rig()
        r.flow(5.0)
        r.silence(4)                                         # gap 4 s > ARM_MAX_GAP_S: the 5 s before it do not count
        r.flow(5.0)
        self.assertFalse(r.w.armed)
        r.flow(4.0)
        self.assertTrue(r.w.armed)

    def test_bytes_skipped_by_the_tail_read_do_not_inflate_the_limit(self):
        r = self.rig()
        r.flow(9.0)
        self.assertTrue(r.w.armed)
        pad = 'X' * 2000
        with open(r.path, 'a', encoding='utf-8') as f:           # a long pause between polls while the game logs far more
            for _ in range(300):                                 # than the tail read covers (300 x 2 KB > 64 KB)
                r.now += timedelta(seconds=0.1)
                f.write(fmt(r.now, CAM + ' ' + pad) + '\n')
        r.mono += 30.0
        self.assertIsNone(r.poll())
        self.assertLess(r.w.max_gap, 1.0)                        # the skipped stretch is not a lull of the game
        self.assertEqual(r.w.limit_s(), 10.0)
        self.assertIsNotNone(r.silence(12))                      # and a real stall is still found after 10 s

    def test_the_limit_follows_the_longest_gap_seen_while_armed(self):
        r = self.rig()
        r.flow(9.0)
        r.silence(6)                                         # a lull that ended
        r.advance(1.0)
        r.write()
        self.assertIsNone(r.poll())
        self.assertGreaterEqual(r.w.max_gap, 7.0)
        self.assertGreaterEqual(r.w.limit_s(), 21.0)
        self.assertIsNone(r.silence(12))                     # 12 s would trip a fresh watch, not this one
        ev = r.silence(10)
        self.assertIsNotNone(ev)

    def test_hold_suspends_and_restarts_the_clock(self):
        r = self.rig()
        r.flow(9.0)
        with r.w.hold():
            self.assertIsNone(r.silence(30))                 # a menu that paused the game on purpose
        self.assertIsNone(r.silence(8))                      # the silence clock restarted when the hold ended
        self.assertIsNotNone(r.silence(4))

    def test_no_process_no_verdict(self):
        r = self.rig(pids=())
        r.flow(9.0)
        self.assertIsNone(r.silence(60))                     # the guard's own "is not running" covers that case
        r.pids = [GAME_PID]
        self.assertIsNotNone(r.silence(1))

    def test_inactive_and_disabled_watches_do_nothing(self):
        r = self.rig()
        r.flow(9.0)
        r.w.stop()
        self.assertIsNone(r.silence(60))
        off = self.rig(seconds=0)
        self.assertFalse(off.w.enabled)
        off.w.start()
        off.flow(9.0)
        self.assertIsNone(off.silence(60))

    def test_the_log_is_read_at_most_once_per_interval(self):
        r = self.rig()
        r.flow(9.0)
        before = r.reads
        for _ in range(50):
            r.poll()                                         # the guard calls it every 50 ms: no clock movement here
        self.assertEqual(r.reads, before)
        r.advance(1.0)
        r.write()
        r.poll()
        self.assertEqual(r.reads, before + 1)

    def test_a_missing_log_is_not_an_error(self):
        r = self.rig()
        r.w.log_fn = lambda: None
        r.advance(2.0)
        self.assertIsNone(r.poll())
        r.w.log_fn = lambda: os.path.join(self.d.name, 'missing.log')
        r.advance(2.0)
        self.assertIsNone(r.poll())

    def test_a_failing_evidence_step_does_not_hide_the_freeze(self):
        r = self.rig()

        def boom(ev):
            raise RuntimeError('disk full')
        r.w.on_trip = boom
        r.flow(9.0)
        ev = r.silence(12)
        self.assertEqual(ev['capture_error'], "RuntimeError('disk full')")


class FakeTools(gf.Tools):
    """Tools whose every part is a small fake: the dump writes a few bytes, nvidia-smi answers a fixed text."""

    def __init__(self, free=100 * GIB, memory=(8 * GIB, 9 * GIB), smi='GPU 0: 3 % busy\n', fail_kinds=(), raises=False):
        self.free, self.memory, self.smi, self.fail_kinds, self.raises = free, memory, smi, set(fail_kinds), raises
        self.dumps: list = []
        self.write_dump = self._write
        self.process_memory = lambda pid: self.memory
        self.free_bytes = lambda path: self.free
        self.nvidia_smi = lambda: self.smi
        self.sleep = lambda s: None

    def _write(self, pid, path, flags):
        kind = next((k for k, v in gf.DUMP_FLAGS.items() if v == flags), '?')
        self.dumps.append((pid, os.path.basename(path), kind, flags))
        if self.raises:
            raise OSError('device not ready')
        if kind in self.fail_kinds:
            with open(path, 'wb') as f:
                f.write(b'partial')
            return False, f'MiniDumpWriteDump failed (error 0x8007012b) for {kind}'
        with open(path, 'wb') as f:
            f.write(b'MDMP' + b'\0' * 1000)
        return True, ''


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.TemporaryDirectory()
        self.addCleanup(self.d.cleanup)
        self.out = os.path.join(self.d.name, 'dumps')
        self.logs: list = []
        self.tails = {}
        for stem, text in (('mod_log', 'a\nb\nc\n'), ('script_console', 'x\ny\n'), ('test_host_log', 'h1\nh2\nh3\nh4\n')):
            p = os.path.join(self.d.name, stem + '.log')
            with open(p, 'w', encoding='utf-8') as f:
                f.write(text)
            self.tails[stem] = p

    def capture(self, tools, mode='full', run_cmd=None):
        return gf.capture_evidence(self.out, 4242, mode, tools, self.tails, log=self.logs.append, run_cmd=run_cmd)

    def test_full_dump_nvidia_smi_and_tails_land_in_the_folder(self):
        cmds = []
        tools = FakeTools()
        info = self.capture(tools, run_cmd=lambda c, **k: cmds.append(c))
        self.assertEqual(tools.dumps, [(4242, 'Cyberpunk2077-4242-full.dmp', 'full', gf.DUMP_FLAGS['full'])])
        self.assertEqual(info['dump']['kind'], 'full')
        self.assertEqual(info['dump']['bytes'], 1004)
        self.assertTrue(os.path.isfile(info['dump']['path']))
        with open(info['nvidia_smi'], encoding='utf-8') as f:
            self.assertEqual(f.read(), 'GPU 0: 3 % busy\n')
        for stem in self.tails:
            self.assertTrue(os.path.isfile(info['tails'][stem]), stem)
        with open(info['tails']['test_host_log'], encoding='utf-8') as f:
            self.assertIn('h4', f.read())
        self.assertEqual(cmds, [['compact', '/U', self.out]])          # new files of the folder are not compressed
        with open(os.path.join(self.out, 'freeze_evidence.json'), encoding='utf-8') as f:
            self.assertEqual(json.load(f)['dump']['kind'], 'full')
        self.assertTrue(any('full dump written' in m for m in self.logs), self.logs)

    def test_the_flags_ask_for_memory_handles_and_thread_info(self):
        f = gf.DUMP_FLAGS['full']
        self.assertEqual(f & 0x2, 0x2)               # MiniDumpWithFullMemory
        self.assertEqual(f & 0x4, 0x4)               # MiniDumpWithHandleData
        self.assertEqual(f & 0x1000, 0x1000)         # MiniDumpWithThreadInfo
        self.assertEqual(gf.DUMP_FLAGS['heap'] & 0x200, 0x200)       # MiniDumpWithPrivateReadWriteMemory
        self.assertEqual(gf.DUMP_FLAGS['heap'] & 0x2, 0)
        self.assertEqual(gf.DUMP_FLAGS['mini'] & 0x2, 0)

    def test_a_disk_too_small_for_the_full_dump_goes_down_the_ladder_with_a_logged_reason(self):
        tools = FakeTools(free=13 * GIB, memory=(10 * GIB, 11 * GIB))      # full needs about 12.1 GiB + 2 GiB reserve, heap 10 + 2
        info = self.capture(tools)
        self.assertEqual(info['dump']['kind'], 'heap')
        self.assertTrue(any('full dump skipped' in r for r in info['dump_skipped']), info['dump_skipped'])
        self.assertTrue(any('full dump skipped' in m for m in self.logs))

    def test_a_failing_write_falls_back_to_the_next_kind_and_removes_the_partial_file(self):
        tools = FakeTools(fail_kinds=('full',))
        info = self.capture(tools)
        self.assertEqual([d[2] for d in tools.dumps], ['full', 'heap'])
        self.assertEqual(info['dump']['kind'], 'heap')
        self.assertFalse(os.path.exists(os.path.join(self.out, 'Cyberpunk2077-4242-full.dmp')))
        self.assertTrue(any('full dump failed' in s for s in info['dump_skipped']))

    def test_a_raising_writer_is_a_failed_step_not_a_crash(self):
        info = self.capture(FakeTools(raises=True))
        self.assertIsNone(info['dump'])
        self.assertEqual(len(info['dump_skipped']), 3)
        self.assertTrue(any('NO dump was written' in m for m in self.logs))
        self.assertTrue(os.path.isfile(info['nvidia_smi']))            # the other evidence is still taken

    def test_dump_off_takes_nothing_but_the_rest(self):
        tools = FakeTools()
        info = self.capture(tools, mode='off')
        self.assertEqual(tools.dumps, [])
        self.assertIsNone(info['dump'])
        self.assertTrue(os.path.isfile(info['nvidia_smi']))

    def test_no_nvidia_smi_is_a_note(self):
        info = self.capture(FakeTools(smi=None))
        self.assertIsNone(info['nvidia_smi'])
        self.assertEqual(info['nvidia_smi_note'], 'nvidia-smi not found')

    def test_choose_dump(self):
        mem = (8 * GIB, 9 * GIB)
        self.assertEqual(gf.choose_dump('full', mem, 200 * GIB)[0], ['full', 'heap', 'mini'])
        self.assertEqual(gf.choose_dump('heap', mem, 200 * GIB)[0], ['heap', 'mini'])
        self.assertEqual(gf.choose_dump('mini', mem, 200 * GIB)[0], ['mini'])
        self.assertEqual(gf.choose_dump('full', mem, 9 * GIB)[0], ['mini'])        # heap needs 8 GiB + the 2 GiB reserve
        kinds, reasons = gf.choose_dump('full', mem, 1 * GIB)
        self.assertEqual(kinds, [])
        self.assertEqual(len(reasons), 3)
        self.assertEqual(gf.choose_dump('full', None, None)[0], ['full', 'heap', 'mini'])     # unknown: try, fall back on failure
        self.assertEqual(gf.choose_dump('off', mem, 200 * GIB)[0], [])

    def test_nvidia_smi_report_takes_two_samples_two_seconds_apart(self):
        calls, sleeps = [], []

        def run(cmd, **k):
            calls.append(cmd)
            return types.SimpleNamespace(stdout='util 4 %', stderr='')
        text = REAL_NVIDIA_SMI_REPORT(run=run, which=lambda n: 'C:/x/nvidia-smi.exe', sleep=sleeps.append)
        self.assertEqual(sleeps, [2.0])
        self.assertEqual(len(calls), 4)
        self.assertIn('-q', calls[0])
        self.assertTrue(any('encoder.stats.sessionCount' in a for a in calls[1]))
        self.assertIn('sample 1', text)
        self.assertIn('sample 2', text)

    def test_nvidia_smi_report_without_the_tool(self):
        old = os.environ.get('SystemRoot')
        os.environ['SystemRoot'] = self.d.name                           # no System32\nvidia-smi.exe there
        try:
            self.assertIsNone(REAL_NVIDIA_SMI_REPORT(run=_real('run'), which=lambda n: None, sleep=_real('sleep')))
        finally:
            if old is None:
                os.environ.pop('SystemRoot', None)
            else:
                os.environ['SystemRoot'] = old

    def test_tail_lines(self):
        p = os.path.join(self.d.name, 'big.txt')
        with open(p, 'w', encoding='utf-8') as f:
            f.write('\n'.join(f'line {i}' for i in range(1000)) + '\n')
        t = gf.tail_lines(p, 5).splitlines()
        self.assertEqual(t, [f'line {i}' for i in range(995, 1000)])
        self.assertEqual(gf.tail_lines(os.path.join(self.d.name, 'none'), 5), '')
        self.assertEqual(gf.tail_lines(None), '')


@unittest.skipUnless(sys.platform == 'win32', 'MiniDumpWriteDump is a Windows call')
class RealDumpWriterTests(unittest.TestCase):
    """The ctypes signatures cannot be checked with fakes: write real dumps of a child process this test starts."""

    def setUp(self):
        self.d = tempfile.TemporaryDirectory()
        self.addCleanup(self.d.cleanup)
        # the base interpreter itself (a venv's python.exe is only a small launcher process), holding some touched memory
        code = 'import time; b = bytearray(64 << 20); b[::4096] = b"x" * len(b[::4096]); time.sleep(120)'
        self.child = subprocess.Popen([getattr(sys, '_base_executable', sys.executable), '-I', '-c', code])
        self.addCleanup(self.child.wait)
        self.addCleanup(self.child.kill)
        time.sleep(1.5)                                          # the interpreter has touched its memory

    def test_mini_and_full_dumps_of_a_child_process(self):
        for kind in ('mini', 'heap', 'full'):
            path = os.path.join(self.d.name, f'child-{kind}.dmp')
            ok, err = REAL_WRITE_DUMP(self.child.pid, path, gf.DUMP_FLAGS[kind])
            self.assertTrue(ok, f'{kind}: {err}')
            with open(path, 'rb') as f:
                self.assertEqual(f.read(4), b'MDMP', kind)
            self.assertGreater(os.path.getsize(path), 10000, kind)
        sizes = {k: os.path.getsize(os.path.join(self.d.name, f'child-{k}.dmp')) for k in ('mini', 'heap', 'full')}
        self.assertGreater(sizes['full'], sizes['mini'])
        mem = REAL_PROCESS_MEMORY(self.child.pid)
        for kind in ('mini', 'heap', 'full'):                    # the estimate that picks the kind never undershoots
            self.assertLessEqual(sizes[kind], gf.dump_need_bytes(kind, mem), (kind, sizes[kind], mem))

    def test_a_missing_process_is_a_reason_not_an_exception(self):
        ok, err = REAL_WRITE_DUMP(0x7FFFFFF0, os.path.join(self.d.name, 'none.dmp'), gf.DUMP_FLAGS['mini'])
        self.assertFalse(ok)
        self.assertIn('OpenProcess', err)

    def test_process_memory_of_the_child(self):
        mem = REAL_PROCESS_MEMORY(self.child.pid)
        self.assertIsNotNone(mem)
        self.assertGreater(mem[0], 32 << 20)
        self.assertGreater(mem[1], 32 << 20)
        self.assertGreaterEqual(mem[2], mem[0])                   # committed readable memory includes the private memory


class FreezeNightTests(tnd.Base):
    """The runner side: the guard hook, the trip (evidence, end of the process), the cycle result and the verdict rule."""

    def setUp(self):
        super().setUp()
        self._pp, self._cet = night.process_path, night.CET_DIR
        night.process_path = lambda pid: ''
        night.CET_DIR = os.path.join(self.root, 'cet')
        os.makedirs(night.CET_DIR)
        with open(os.path.join(night.CET_DIR, 'scripting.log'), 'w', encoding='utf-8') as f:
            f.write('script line 1\nscript line 2\n')
        with open(os.path.join(self.game, 'utpptesthost-2026-11-14-01-42-45.log'), 'w', encoding='utf-8') as f:
            f.write('host line\n')
        self.t0 = datetime(2026, 11, 14, 14, 49, 0)
        self.fake_now = self.t0
        self.mono = 5000.0

    def tearDown(self):
        night.process_path, night.CET_DIR = self._pp, self._cet
        super().tearDown()

    @staticmethod
    def read(path):
        with open(path, encoding='utf-8') as f:
            return f.read()

    def night_with_game(self, *extra, launched=True):
        n = self.make(*extra)
        n.api = FakeApi()
        n.freeze_tools = FakeTools()
        n.launched_pids = {GAME_PID} if launched else set()
        self.calls: list = []
        reporter = {'left': 1}

        def runner(cmd, **k):
            self.calls.append(list(cmd))
            out = ''
            if cmd[:2] == ['taskkill', '/F'] and '/PID' in cmd:
                n.api.running = False
            if cmd[0] == 'tasklist' and reporter['left'] > 0:
                reporter['left'] -= 1
                out = '"CrashReporter.exe","9123","Console","1","12,000 K"\n'
            return types.SimpleNamespace(stdout=out, returncode=0)
        n.runner = runner
        n.current_scenario = 'flick_180_walk#1'
        n.freeze_begin()
        if n.freeze is not None:
            n.freeze.now_fn = lambda: self.fake_now
            n.freeze.clock = lambda: self.mono
        return n

    def feed(self, n, seconds=9.0):
        with open(n.utpp_log, 'a', encoding='utf-8') as f:
            for i in range(int(seconds * 10)):
                self.fake_now += timedelta(seconds=0.1)
                self.mono += 0.1
                f.write(fmt(self.fake_now, CAM) + '\n')

    def test_the_guard_hook_raises_after_ten_silent_seconds_and_the_trip_does_the_work_in_order(self):
        n = self.night_with_game()
        g = n.guard()
        g.ensure_focus()
        self.feed(n)
        g.check()                                                    # dense log: alive
        self.assertTrue(n.freeze.armed)
        self.fake_now += timedelta(seconds=11.0)
        self.mono += 11.0
        with self.assertRaises(gf.GameFrozenError) as cm:
            g.check()
        self.assertTrue(str(cm.exception).startswith('game not responding since '))
        text = self.read(os.path.join(self.run_dir, 'night.log')).splitlines()
        i_line = next(i for i, ln in enumerate(text) if '[freeze] game not responding since ' in ln)
        i_dump = next(i for i, ln in enumerate(text) if 'full dump written' in ln)
        i_end = next(i for i, ln in enumerate(text) if '[freeze] game process' in ln and 'ended' in ln)
        self.assertLess(i_line, i_dump)
        self.assertLess(i_dump, i_end)                               # the dump is taken before the process is ended
        self.assertIn('(no game-thread log line for ', text[i_line])
        self.assertTrue(any('crash reporter dialog(s) closed without sending: 1 (the game was ended after it froze)' in ln for ln in text), text)
        self.assertEqual([c for c in self.calls if c[0] == 'taskkill' and '/PID' in c], [['taskkill', '/F', '/PID', str(GAME_PID)]])
        self.assertFalse(any('/IM' in c and 'Cyberpunk2077.exe' in c for c in self.calls))   # never by image name
        self.assertIn(['taskkill', '/F', '/IM', 'CrashReporter.exe'], self.calls)             # the leftover reporter is closed
        ev = n.freeze_info
        self.assertEqual(ev['scenario'], 'flick_180_walk#1')
        self.assertEqual(ev['ended'], [GAME_PID])
        self.assertEqual(ev['crash_reporter_closed'], 1)
        dumps = os.path.join(self.run_dir, 'dumps')
        self.assertTrue(os.path.isfile(os.path.join(dumps, f'Cyberpunk2077-{GAME_PID}-full.dmp')))
        self.assertTrue(os.path.isfile(os.path.join(dumps, 'nvidia-smi.txt')))
        self.assertIn('script line 2', self.read(os.path.join(dumps, 'script_console_tail.txt')))
        self.assertIn('host line', self.read(os.path.join(dumps, 'test_host_log_tail.txt')))
        self.assertIn('camTrace', self.read(os.path.join(dumps, 'mod_log_tail.txt')))
        with open(os.path.join(self.run_dir, 'cycle.json'), encoding='utf-8') as f:
            self.assertEqual(json.load(f)['freeze']['scenario'], 'flick_180_walk#1')
        with self.assertRaises(gf.GameFrozenError):
            g.check()                                                # sticky: every later check refuses too
        with self.assertRaises(gf.GameFrozenError):
            n.check_scenario_time()
        n.freeze_end()
        g.check()                                                    # the phase is over: the quit paths are not blocked

    def test_the_background_guard_has_the_same_hook(self):
        n = self.night_with_game('--background', '--cmd-channel', 'file')
        g = n.guard()
        self.assertIs(g.extra_check.__self__, n)
        g.ensure_focus()
        self.feed(n)
        g.check()
        self.fake_now += timedelta(seconds=11.0)
        self.mono += 11.0
        with self.assertRaises(gf.GameFrozenError):
            g.check()

    def test_a_game_this_run_did_not_start_is_not_ended(self):
        n = self.night_with_game(launched=False)
        g = n.guard()
        g.ensure_focus()
        self.feed(n)
        g.check()
        self.fake_now += timedelta(seconds=11.0)
        self.mono += 11.0
        with self.assertRaises(gf.GameFrozenError):
            g.check()
        self.assertEqual([c for c in self.calls if c[0] == 'taskkill' and '/PID' in c], [])
        self.assertEqual(n.freeze_info['left_alone'], [GAME_PID])
        self.assertEqual(n.freeze_info['ended'], [])
        self.assertTrue(os.path.isfile(os.path.join(self.run_dir, 'dumps', f'Cyberpunk2077-{GAME_PID}-full.dmp')))   # reading is fine

    def test_the_hold_of_a_menu_close_keeps_the_hook_quiet(self):
        n = self.night_with_game()
        g = n.guard()
        g.ensure_focus()
        self.feed(n)
        g.check()
        with n.freeze_hold():
            self.fake_now += timedelta(seconds=60.0)
            self.mono += 60.0
            g.check()
        self.mono += 1.0
        self.fake_now += timedelta(seconds=1.0)
        g.check()

    def test_off_means_no_watch_at_all(self):
        n = self.night_with_game('--freeze-seconds', '0')
        self.assertIsNone(n.freeze)
        g = n.guard()
        g.ensure_focus()
        self.fake_now += timedelta(seconds=300.0)
        g.check()

    def test_options(self):
        a = night.build_parser().parse_args([])
        self.assertEqual((a.freeze_seconds, a.freeze_dump, a.freeze_dump_dir, a.graphics_keep), (10.0, 'full', '', False))
        a = night.build_parser().parse_args(['--freeze-seconds', '2', '--freeze-dump', 'mini', '--graphics-keep'])
        self.assertTrue(any('--freeze-seconds' in e for e in night.validate_args(a)))
        a = night.build_parser().parse_args(['--freeze-seconds', '0'])
        self.assertFalse(any('--freeze-seconds' in e for e in night.validate_args(a)))
        with self.assertRaises(SystemExit):
            night.build_parser().parse_args(['--freeze-dump', 'everything'])

    # -- the cycle --
    def cycle(self, exc, freeze=True):
        n = self.make()
        calls = []

        def rec(name, ret=None):
            def f(*a, **k):
                calls.append((name, a[0]) if a and isinstance(a[0], str) else name)
                return ret
            return f
        for name in ('check_user_idle', 'check_continue', 'stop_padd', 'apply_graphics', 'install', 'start_idler', 'stop_idler',
                     'arm', 'marker', 'perf_status', 'report_load', 'stop_recording', 'collect', 'report_budget', 'save',
                     'secure_session_files', 'restore_settings_leave', 'settings_left'):
            setattr(n, name, rec(name))
        n.ensure_closed = rec('ensure_closed', {'path': 'not_running'})
        n.launch = rec('launch', self.log)
        n.start_recording = rec('start_recording', {'path': 'x'})
        n.bg_input = rec('bg_input', {})
        n.bg_input_active = True

        def scenarios(plan):
            if freeze:
                n.freeze_info = {'message': 'game not responding since 14:49:01.528 (no game-thread log line for 10.4 s)',
                                 'since': '2026-11-14 14:49:01.528', 'scenario': 'flick_180_walk#1'}
            raise exc
        n.run_scenarios = scenarios
        return n, calls

    def test_a_cycle_whose_game_froze_is_its_own_result_not_a_failure(self):
        n, calls = self.cycle(night.CycleError('game not responding since 14:49:01.528'))
        cyc = n.run_cycle(1)
        self.assertTrue(cyc['result'].startswith('game froze: game not responding since 14:49:01.528'), cyc['result'])
        self.assertNotIn('failed', cyc['result'])
        self.assertIn(('settings_left', 'the game froze'), calls)       # the settings wait in the pending file
        self.assertIn(('secure_session_files', 'game froze'), calls)
        self.assertNotIn('bg_input', calls)                             # nothing is sent to the frozen game's test host
        self.assertFalse(n.bg_input_active)
        self.assertIn(('ensure_closed', 'quit'), calls)
        self.assertIn('collect', calls)
        self.assertIn('save', calls)

    def test_a_freeze_stays_a_freeze_when_another_exception_type_comes_out(self):
        n, calls = self.cycle(RuntimeError('macro re-raised something else'))
        cyc = n.run_cycle(1)                                            # not re-raised as a defect of the runner
        self.assertTrue(cyc['result'].startswith('game froze'), cyc['result'])
        self.assertIn('collect', calls)

    def test_without_a_freeze_everything_is_as_before(self):
        n, calls = self.cycle(night.CycleError('foreground window is not the game'), freeze=False)
        cyc = n.run_cycle(1)
        self.assertTrue(cyc['result'].startswith('failed: foreground'), cyc['result'])
        self.assertIn('bg_input', calls)
        self.assertNotIn(('settings_left', 'the game froze'), calls)
        n2, calls2 = self.cycle(RuntimeError('boom'), freeze=False)
        with self.assertRaises(RuntimeError):
            n2.run_cycle(1)

    def test_the_session_record_marks_a_frozen_cycle(self):
        frozen = {'name': 'c1', 'result': 'game froze: m', 'freeze': {'since': '2026-11-14 14:49:01.528'}}
        node = types.SimpleNamespace(run_cycle=lambda n: frozen, requested_stop=lambda: None)
        results: list = []
        stop = night.run_cycles(node, types.SimpleNamespace(cycles=1, pause_between=0), results, 'male')
        self.assertFalse(stop)
        self.assertEqual(results, [{'cycle': 'c1', 'body': 'male', 'result': 'game froze: m', 'game_froze': '2026-11-14 14:49:01.528'}])
        ok = {'name': 'c2', 'result': 'ok'}
        node.run_cycle = lambda n: ok
        results = []
        night.run_cycles(node, types.SimpleNamespace(cycles=1, pause_between=0), results, None)
        self.assertEqual(results, [{'cycle': 'c2', 'body': None, 'result': 'ok'}])

    # -- the verdict rule --
    def scenarios_of(self):
        t = self.t0
        return [{'name': 'run_arc_right', 'run': 1, 'start': t - timedelta(seconds=70), 'end': t - timedelta(seconds=40),
                 'verdict': 'PASS'},
                {'name': 'slalom_stick', 'run': 1, 'start': t - timedelta(seconds=30), 'end': t + timedelta(seconds=1.0),
                 'verdict': 'PASS'},
                {'name': 'flick_180_walk', 'run': 1, 'start': t + timedelta(seconds=1.9), 'end': t + timedelta(seconds=16),
                 'verdict': 'FAIL', 'fails': ['no position']}]

    def test_the_scenarios_that_reach_into_the_frozen_period_are_not_judged(self):
        n = self.make()
        n.cycle['scenarios'] = self.scenarios_of()
        n.freeze_info = {'since': '2026-11-14 14:49:00.000', 'scenario': 'flick_180_run#1', 'detected': '2026-11-14 14:49:11.0',
                         'silent_s': 11.0, 'message': 'm'}
        nj = n.freeze_not_judged()
        self.assertEqual(sorted(nj), ['flick_180_run#1', 'flick_180_walk#1', 'slalom_stick#1'])
        self.assertEqual([s['name'] for s in n.judged_scenarios(n.cycle['scenarios'])], ['run_arc_right'])
        runs = [{'scenario': 'run_arc_right', 'run': 1}, {'scenario': 'slalom_stick', 'run': 1},
                {'scenario': 'flick_180_run', 'run': 1, 'aborted': 'game not responding since 14:49:01.528'},
                {'scenario': 'wheel_probe', 'run': 1, 'aborted': 'game not responding since 14:49:01.528'}]
        self.assertEqual([r['scenario'] for r in n.judged_runs(runs)], ['run_arc_right'])
        self.assertEqual(n.freeze_summary()['not_judged'], sorted(nj))
        self.assertEqual(gf.key_of('flick_180_walk#1.step_a: no motion'), 'flick_180_walk#1')
        self.assertEqual(gf.key_of('flick_180_walk#1: full body'), 'flick_180_walk#1')

    def test_without_a_freeze_nothing_is_filtered(self):
        n = self.make()
        n.cycle['scenarios'] = self.scenarios_of()
        self.assertEqual(n.freeze_not_judged(), [])
        self.assertIs(n.judged_scenarios(n.cycle['scenarios']), n.cycle['scenarios'])
        runs = [{'scenario': 'a', 'run': 1}]
        self.assertIs(n.judged_runs(runs), runs)

    def test_collect_keeps_the_frozen_scenarios_out_of_the_verdicts_and_the_analysis(self):
        n = self.make('--no-media-move', '--no-sheets', '--no-pose-meter', '--no-failure-clips')
        n.cycle['scenarios'] = self.scenarios_of()
        n.cycle['result'] = 'game froze: x'
        n.freeze_info = {'since': '2026-11-14 14:49:00.000', 'scenario': 'flick_180_run#1', 'detected': '2026-11-14 14:49:11.0',
                         'silent_s': 11.0, 'message': 'm'}
        t = self.t0
        plan = {'runs': [{'scenario': 'run_arc_right', 'run': 1, 'steps': []},
                         {'scenario': 'flick_180_walk', 'run': 1, 'steps': [], 'aborted': 'game not responding since 14:49:01.528'}]}
        seen = {}

        def analyze(gdir, ra, log_override=None):
            with open(os.path.join(gdir, 'plan.json'), encoding='utf-8') as f:
                seen['runs'] = [r['scenario'] for r in json.load(f)['runs']]
            with open(os.path.join(gdir, 'summary.json'), 'w', encoding='utf-8') as f:
                json.dump({'steps': [], 'overall': 'FAIL', 'counts': {}}, f)
        orig = night.gt_run.analyze_dir
        night.gt_run.analyze_dir = analyze
        self.addCleanup(setattr, night.gt_run, 'analyze_dir', orig)
        summ = n.collect(None, plan)
        self.assertEqual(seen['runs'], ['run_arc_right'])
        self.assertEqual(list(summ['scenario_verdicts']), ['run_arc_right#1'])
        self.assertEqual(summ['mechanics_failures'], [])                  # the 'no position' of the frozen scenario is not a failure
        self.assertEqual(summ['game_froze']['not_judged'], sorted(['flick_180_run#1', 'flick_180_walk#1', 'slalom_stick#1']))
        self.assertEqual(summ['result'], 'game froze: x')


if __name__ == '__main__':
    unittest.main()
