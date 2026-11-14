"""Frozen-game guard of the night runner: detection, evidence capture and the verdict rule.

A game whose process is alive but whose game thread has stopped (for example a presentation stall on the GPU side) keeps
the runner busy for minutes: every scenario is played blind until the engine's own watchdog ends the process. This module
finds that state within about ten seconds, collects what is needed to analyse it, and lets the runner end the cycle as
"game froze" instead of as a mod or scenario failure.

Detection (FreezeWatch): the newest line of the mod log that is not a periodic status line of a helper thread
(IGNORED_PREFIXES) carries a time stamp; when the newest such stamp is older than the limit while a scenario phase is
running, the game thread is taken as frozen. File growth alone proves nothing: the pose capture thread keeps writing its
own status lines for as long as the process lives. Safety rules against a false alarm:
  - the watch is armed only after the log has been seen flowing steadily in this run (for ARM_SPAN_S seconds with no gap
    longer than ARM_MAX_GAP_S between lines), so a build or a mode that logs only on events never arms it;
  - a line that says the diagnostic logs were switched off (DIAG_OFF_PREFIXES) disarms it again until a steady flow is seen;
  - the limit is at least three times the longest gap seen while armed;
  - harness actions that can pause the game on purpose run inside hold() and restart the clock when they end;
  - the process must still exist (a missing process is the guard's own "is not running" case);
  - the answer is sticky: once tripped, every later poll reports the same evidence.

Evidence (capture_evidence): a process dump (MiniDumpWriteDump of the system dbghelp through ctypes; type full, heap or mini,
lowered by free disk space or by a failing write), two nvidia-smi samples 2 s apart, and the tails of the mod log, the script
console log and the test host log. Everything that touches the system sits behind small functions (Tools) that tests replace.
"""
from __future__ import annotations

import ctypes
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_input  # noqa: E402
from gt_win32 import IS_WINDOWS, GuardError  # noqa: E402

FREEZE_RESULT = 'game froze'          # cycle result prefix: not a mod failure, not a scenario failure
MESSAGE_HEAD = 'game not responding'  # first words of the guard error; none of the user-stop phrases of night.is_user_stop
DEFAULT_SECONDS = 10.0
ARM_SPAN_S = 8.0                      # the log must have flowed this long without a gap above ARM_MAX_GAP_S before the watch can trip
ARM_MAX_GAP_S = 3.0
GAP_FACTOR = 3.0                      # the limit is at least this many times the longest gap seen while armed
POLL_INTERVAL_S = 1.0
TAIL_BYTES = 65536                    # most a poll reads: the newest bytes of the log

LINE_RE = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\] \[\w+\s*\] \[\s*\d+\] \[UTPP\] (.*)$')
TS_FMT = '%Y-%m-%d %H:%M:%S.%f'
# status lines written by helper threads of the mod: they continue while the game thread is frozen
IGNORED_PREFIXES = ('UTPP pose capture:',)
# lines the mod writes when its diagnostic logs are switched off (by hand, by the timer or by the runner): the steady flow of
# camera / motion lines ends legitimately there, so the watch has to prove a steady flow again before it can trip
DIAG_OFF_PREFIXES = ('UTPPDiagWriteReport:', 'UTPP MM feed dump off')

# MINIDUMP_TYPE bits
MD_WITH_FULL_MEMORY = 0x2
MD_WITH_HANDLE_DATA = 0x4
MD_WITH_UNLOADED_MODULES = 0x20
MD_WITH_PROCESS_THREAD_DATA = 0x100
MD_WITH_PRIVATE_RW_MEMORY = 0x200
MD_WITH_FULL_MEMORY_INFO = 0x800
MD_WITH_THREAD_INFO = 0x1000
DUMP_FLAGS = {
    'full': MD_WITH_FULL_MEMORY | MD_WITH_HANDLE_DATA | MD_WITH_THREAD_INFO | MD_WITH_FULL_MEMORY_INFO | MD_WITH_UNLOADED_MODULES,
    'heap': MD_WITH_PRIVATE_RW_MEMORY | MD_WITH_HANDLE_DATA | MD_WITH_THREAD_INFO | MD_WITH_FULL_MEMORY_INFO
            | MD_WITH_UNLOADED_MODULES | MD_WITH_PROCESS_THREAD_DATA,
    'mini': MD_WITH_HANDLE_DATA | MD_WITH_THREAD_INFO | MD_WITH_FULL_MEMORY_INFO | MD_WITH_UNLOADED_MODULES,
}
DUMP_LADDER = ('full', 'heap', 'mini')            # each step is smaller than the one before
DUMP_MODES = DUMP_LADDER + ('off',)
GIB = 1 << 30
RESERVE_BYTES = 2 * GIB                           # free space that must remain after the dump
MINI_BYTES = 400 << 20
UNKNOWN_PRIVATE_BYTES = 16 * GIB                  # assumed size of the game's memory when it cannot be read


class GameFrozenError(GuardError):
    """The game process is alive but the game thread has stopped. Raised by the guard hook of the runner."""


# ------------------------------------------------------------------------------------------------ detection
def parse_alive_stamps(text: str) -> list[tuple[datetime, str]]:
    """[(time stamp, message)] of the mod log lines of `text` that show the game thread alive: every line with the log
    layout except the periodic status lines of helper threads."""
    out = []
    for line in (text or '').splitlines():
        m = LINE_RE.match(line)
        if not m:
            continue
        msg = m.group(2)
        if msg.startswith(IGNORED_PREFIXES):
            continue
        try:
            out.append((datetime.strptime(m.group(1), TS_FMT), msg))
        except ValueError:
            continue
    return out


def stamp_text(dt: datetime) -> str:
    return dt.strftime(TS_FMT)[:-3]


class FreezeWatch:
    """Polled from the guard hook of the runner (about every 50 ms while input or waits run); reads the new end of the mod
    log at most once per `interval` seconds. poll() returns None while the game is alive and the evidence dict once it is
    taken as frozen (the same dict on every later call)."""

    def __init__(self, seconds: float, log_fn, pids_fn=None, on_trip=None, size_fn=None, read_fn=None, now_fn=None,
                 clock=None, interval: float = POLL_INTERVAL_S, tail_bytes: int = TAIL_BYTES):
        self.seconds = float(seconds)
        self.log_fn = log_fn
        self.pids_fn = pids_fn
        self.on_trip = on_trip
        self.size_fn = size_fn or gt_input.file_size
        self.read_fn = read_fn or gt_input.read_since
        self.now_fn = now_fn or datetime.now
        self.clock = clock or time.monotonic
        self.interval = interval
        self.tail_bytes = tail_bytes
        self.active = False
        self.tripped = False
        self.evidence: dict | None = None
        self.armed = False
        self.armed_at: datetime | None = None
        self.newest: datetime | None = None
        self.newest_msg = ''
        self.max_gap = 0.0
        self.lines_seen = 0
        self.hold_depth = 0
        self.grace_from: datetime | None = None
        self._offset: int | None = None
        self._dense_from: datetime | None = None     # start of the current run of lines with no gap above ARM_MAX_GAP_S
        self._last_poll = -1e9
        self._skip_gap = False

    @property
    def enabled(self) -> bool:
        return self.seconds > 0

    def start(self) -> None:
        self.active = self.enabled
        self._offset = None
        self._last_poll = -1e9

    def stop(self) -> None:
        self.active = False

    @contextmanager
    def hold(self):
        """A harness action that may pause the game on purpose (closing a menu): nothing trips inside it, and the silence
        clock restarts when it ends."""
        self.hold_depth += 1
        try:
            yield
        finally:
            self.hold_depth -= 1
            self.grace_from = self.now_fn()
            self._skip_gap = True

    def _read(self) -> None:
        path = self.log_fn()
        if not path:
            return
        end = self.size_fn(path)
        if end is None or end < 0:
            return
        start = max(0, end - self.tail_bytes) if self._offset is None else max(self._offset, end - self.tail_bytes)
        if self._offset is not None and end <= self._offset:
            self._offset = end
            return
        if self._offset is not None and start > self._offset:
            self._skip_gap = True                # bytes were skipped (a long pause between polls): the time before the first
            #                                      line read says nothing about a lull of the game
        text = self.read_fn(path, start)
        self._offset = end
        for ts, msg in parse_alive_stamps(text):
            if self.newest is not None and ts <= self.newest:
                continue
            if msg.startswith(DIAG_OFF_PREFIXES):
                self.armed, self.max_gap, self._dense_from = False, 0.0, None
                self.newest, self.newest_msg, self._skip_gap = ts, msg, False
                self.lines_seen += 1
                continue
            gap = (ts - self.newest).total_seconds() if self.newest is not None else None
            if self.armed and gap is not None and not self._skip_gap:
                self.max_gap = max(self.max_gap, gap)
            self._skip_gap = False
            if gap is None or gap > ARM_MAX_GAP_S or self._dense_from is None:
                self._dense_from = ts
            self.newest, self.newest_msg = ts, msg
            self.lines_seen += 1
        if not self.armed and self._dense_from is not None and self.newest is not None \
                and (self.newest - self._dense_from).total_seconds() >= ARM_SPAN_S:
            self.armed, self.armed_at = True, self.now_fn()

    def limit_s(self) -> float:
        return max(self.seconds, GAP_FACTOR * self.max_gap)

    def poll(self) -> dict | None:
        if self.tripped:
            return self.evidence
        if not self.active or self.hold_depth > 0:
            return None
        now_m = self.clock()
        if now_m - self._last_poll < self.interval:
            return None
        self._last_poll = now_m
        self._read()
        if not self.armed or self.newest is None:
            return None
        base = max(self.newest, self.grace_from) if self.grace_from else self.newest
        now = self.now_fn()
        age = (now - base).total_seconds()
        if age < self.limit_s():
            return None
        pids = list(self.pids_fn() or []) if self.pids_fn else []
        if self.pids_fn and not pids:
            return None                         # no process: the guard reports "is not running" itself
        self.tripped = True
        self.evidence = {
            'since': stamp_text(self.newest), 'detected': stamp_text(now), 'silent_s': round(age, 1),
            'limit_s': round(self.limit_s(), 1), 'pids': pids, 'last_line': self.newest_msg[:160],
            'lines_seen': self.lines_seen, 'longest_gap_s': round(self.max_gap, 2),
            'text': (f'no game-thread log line for {age:.1f} s (limit {self.limit_s():.1f} s), last line '
                     f'"{self.newest_msg[:80]}", process {",".join(str(p) for p in pids) or "?"} alive')}
        self.evidence['guard_message'] = f'{MESSAGE_HEAD} since {self.newest:%H:%M:%S.%f}'[:-3]
        self.evidence['message'] = f'{self.evidence["guard_message"]} ({self.evidence["text"]})'
        if self.on_trip is not None:
            try:
                self.on_trip(self.evidence)
            except Exception as e:  # noqa: BLE001 - the evidence step must never hide the freeze itself
                self.evidence['capture_error'] = repr(e)
        return self.evidence


# ------------------------------------------------------------------------------------------------ system helpers
def system32_file(name: str) -> str:
    """`name` in the system folder (SystemRoot); the bare name when the environment does not say where that is."""
    root = os.environ.get('SystemRoot') or os.environ.get('windir')
    return os.path.join(root, 'System32', name) if root else name


def write_dump(pid: int, path: str, flags: int) -> tuple[bool, str]:
    """MiniDumpWriteDump of the system dbghelp: (True, '') or (False, reason). Handles are 64-bit: every signature is declared."""
    if not IS_WINDOWS:
        return False, 'not Windows'
    from ctypes import wintypes
    k32 = ctypes.WinDLL('kernel32', use_last_error=True)
    k32.OpenProcess.restype = ctypes.c_void_p
    k32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    k32.CreateFileW.restype = ctypes.c_void_p
    k32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD,
                                wintypes.DWORD, ctypes.c_void_p]
    k32.CloseHandle.argtypes = [ctypes.c_void_p]
    k32.CloseHandle.restype = wintypes.BOOL
    dbg = ctypes.WinDLL(system32_file('dbghelp.dll'), use_last_error=True)
    dbg.MiniDumpWriteDump.restype = wintypes.BOOL
    dbg.MiniDumpWriteDump.argtypes = [ctypes.c_void_p, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD, ctypes.c_void_p,
                                      ctypes.c_void_p, ctypes.c_void_p]
    # query + read memory + duplicate handle (the handle data of the dump)
    proc = k32.OpenProcess(0x0400 | 0x0010 | 0x0040, False, int(pid))
    if not proc:
        return False, f'OpenProcess failed (error {ctypes.get_last_error()})'
    invalid = ctypes.c_void_p(-1).value
    fh = k32.CreateFileW(path, 0x40000000, 0, None, 2, 0x80, None)      # GENERIC_WRITE, no sharing, CREATE_ALWAYS
    if not fh or fh == invalid:
        err = ctypes.get_last_error()
        k32.CloseHandle(proc)
        return False, f'CreateFileW failed (error {err})'
    try:
        ok = dbg.MiniDumpWriteDump(proc, int(pid), fh, int(flags), None, None, None)
        if not ok:
            return False, f'MiniDumpWriteDump failed (error {ctypes.get_last_error() & 0xFFFFFFFF:#x})'
        return True, ''
    finally:
        k32.CloseHandle(fh)
        k32.CloseHandle(proc)


def committed_bytes(handle) -> int:
    """Sum of the committed, readable memory regions of an open process (what MiniDumpWithFullMemory writes: private memory,
    mapped files and images). 0 when it cannot be read."""
    from ctypes import wintypes

    class MBI(ctypes.Structure):
        _fields_ = [('BaseAddress', ctypes.c_void_p), ('AllocationBase', ctypes.c_void_p), ('AllocationProtect', wintypes.DWORD),
                    ('PartitionId', wintypes.WORD), ('RegionSize', ctypes.c_size_t), ('State', wintypes.DWORD),
                    ('Protect', wintypes.DWORD), ('Type', wintypes.DWORD)]
    k32 = ctypes.WinDLL('kernel32', use_last_error=True)
    k32.VirtualQueryEx.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
    k32.VirtualQueryEx.restype = ctypes.c_size_t
    mbi = MBI()
    total, addr, top = 0, 0, 0x7FFFFFFF0000
    for _ in range(2_000_000):
        if addr >= top or not k32.VirtualQueryEx(handle, ctypes.c_void_p(addr), ctypes.byref(mbi), ctypes.sizeof(mbi)):
            break
        if mbi.State == 0x1000 and not (mbi.Protect & (0x01 | 0x100)):        # MEM_COMMIT, not PAGE_NOACCESS / PAGE_GUARD
            total += mbi.RegionSize
        nxt = (mbi.BaseAddress or 0) + mbi.RegionSize
        if nxt <= addr:
            break
        addr = nxt
    return int(total)


def process_memory(pid: int) -> tuple[int, int, int] | None:
    """(private bytes, working set bytes, committed readable bytes) of a process, None when unreadable."""
    if not IS_WINDOWS:
        return None
    try:
        from ctypes import wintypes

        class COUNTERS(ctypes.Structure):
            _fields_ = [('cb', wintypes.DWORD), ('PageFaultCount', wintypes.DWORD), ('PeakWorkingSetSize', ctypes.c_size_t),
                        ('WorkingSetSize', ctypes.c_size_t), ('QuotaPeakPagedPoolUsage', ctypes.c_size_t),
                        ('QuotaPagedPoolUsage', ctypes.c_size_t), ('QuotaPeakNonPagedPoolUsage', ctypes.c_size_t),
                        ('QuotaNonPagedPoolUsage', ctypes.c_size_t), ('PagefileUsage', ctypes.c_size_t),
                        ('PeakPagefileUsage', ctypes.c_size_t), ('PrivateUsage', ctypes.c_size_t)]
        k32 = ctypes.WinDLL('kernel32', use_last_error=True)
        k32.OpenProcess.restype = ctypes.c_void_p
        k32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        k32.CloseHandle.argtypes = [ctypes.c_void_p]
        psapi = ctypes.WinDLL('psapi', use_last_error=True)
        psapi.GetProcessMemoryInfo.argtypes = [ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD]
        psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
        h = k32.OpenProcess(0x0400 | 0x0010, False, int(pid))
        if not h:
            return None
        try:
            c = COUNTERS()
            c.cb = ctypes.sizeof(c)
            if not psapi.GetProcessMemoryInfo(h, ctypes.byref(c), c.cb):
                return None
            try:
                committed = committed_bytes(h)
            except Exception:  # noqa: BLE001
                committed = 0
            return int(c.PrivateUsage), int(c.WorkingSetSize), committed
        finally:
            k32.CloseHandle(h)
    except Exception:  # noqa: BLE001
        return None


def free_bytes(path: str) -> int | None:
    try:
        return int(shutil.disk_usage(path).free)
    except OSError:
        return None


SMI_SAMPLES = (
    ['-q', '-d', 'UTILIZATION,PERFORMANCE,CLOCK'],
    ['--query-gpu=timestamp,name,utilization.gpu,utilization.memory,memory.used,memory.total,encoder.stats.sessionCount,'
     'encoder.stats.averageFps,encoder.stats.averageLatency,clocks.sm,clocks.mem,power.draw,temperature.gpu', '--format=csv'],
)


def find_nvidia_smi(which=shutil.which) -> str | None:
    found = which('nvidia-smi')
    if found:
        return found
    cand = system32_file('nvidia-smi.exe')
    return cand if os.path.isfile(cand) else None


def nvidia_smi_report(run=subprocess.run, which=shutil.which, sleep=time.sleep, gap: float = 2.0,
                      timeout: float = 20.0) -> str | None:
    """Two nvidia-smi samples `gap` seconds apart (utilisation, performance, clocks, memory, encoder sessions) as text;
    None when the tool does not exist."""
    exe = find_nvidia_smi(which)
    if not exe:
        return None
    out: list[str] = []
    for n in (1, 2):
        out.append(f'===== sample {n} at {datetime.now().strftime(TS_FMT)[:-3]}')
        for args in SMI_SAMPLES:
            out.append('$ nvidia-smi ' + ' '.join(args))
            try:
                r = run([exe] + args, capture_output=True, text=True, timeout=timeout)
                out.append((r.stdout or '').rstrip() + (f'\n[stderr] {r.stderr.strip()}' if (r.stderr or '').strip() else ''))
            except Exception as e:  # noqa: BLE001
                out.append(f'[failed: {e!r}]')
        if n == 1:
            sleep(gap)
    return '\n'.join(out) + '\n'


class Tools:
    """What the evidence step touches outside the run folder; tests replace the attributes."""

    def __init__(self):
        self.write_dump = write_dump
        self.process_memory = process_memory
        self.free_bytes = free_bytes
        self.nvidia_smi = nvidia_smi_report
        self.sleep = time.sleep


# ------------------------------------------------------------------------------------------------ evidence
def dump_need_bytes(kind: str, memory: tuple | None) -> int:
    """Upper estimate of the size of a dump of `kind` for a process with that (private, working set[, committed readable]) memory.
    A full dump holds every committed readable region (mapped files and images too), a heap dump the private memory."""
    private = memory[0] if memory else UNKNOWN_PRIVATE_BYTES
    working = memory[1] if memory else UNKNOWN_PRIVATE_BYTES
    committed = memory[2] if memory and len(memory) > 2 else 0
    if kind == 'full':
        return int(max(committed, private, working) * 1.05) + (64 << 20)
    if kind == 'heap':
        return int(private)
    return MINI_BYTES


def choose_dump(mode: str, memory: tuple[int, int] | None, free: int | None) -> tuple[list[str], list[str]]:
    """(kinds to try in order, reasons for the steps skipped). Starts at the requested kind and goes down the ladder while
    the free space does not cover the estimated size plus RESERVE_BYTES; with the free space unknown the requested kind is
    tried first and the smaller ones follow if its write fails."""
    if mode == 'off':
        return [], ['dump switched off (--freeze-dump off)']
    ladder = list(DUMP_LADDER[DUMP_LADDER.index(mode):])
    reasons, kinds = [], []
    for kind in ladder:
        need = dump_need_bytes(kind, memory)
        if free is not None and free < need + RESERVE_BYTES:
            reasons.append(f'{kind} dump skipped: needs about {need / GIB:.1f} GiB, {free / GIB:.1f} GiB free '
                           f'(keeping {RESERVE_BYTES / GIB:.0f} GiB)')
            continue
        kinds.append(kind)
    return kinds, reasons


def newest_file(folder: str, pattern: str) -> str | None:
    """The most recently written file of `folder` matching the glob `pattern` (None = none)."""
    import glob
    best, best_t = None, -1.0
    for p in glob.glob(os.path.join(folder, pattern)):
        try:
            t = os.path.getmtime(p)
        except OSError:
            continue
        if t > best_t:
            best, best_t = p, t
    return best


def tail_lines(path: str | None, n: int = 200, max_bytes: int = 262144) -> str:
    if not path or not os.path.isfile(path):
        return ''
    try:
        with open(path, 'rb') as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            data = f.read()
    except OSError as e:
        return f'[unreadable: {e!r}]'
    return '\n'.join(data.decode('utf-8', errors='replace').splitlines()[-n:]) + '\n'


def capture_evidence(out_dir: str, pid: int, mode: str, tools: Tools, tails: dict, log=print, run_cmd=None) -> dict:
    """Dump + nvidia-smi + log tails into `out_dir` (created). `tails` = {file stem: source log path}. Returns the manifest
    (also written as freeze_evidence.json): the dump kind / path / size / seconds, why bigger kinds were not used, the
    nvidia-smi file, the tail files. Never raises for a failing step: the step is recorded and the next one goes on."""
    os.makedirs(out_dir, exist_ok=True)
    info: dict = {'dir': out_dir, 'pid': pid, 'dump': None, 'dump_skipped': [], 'nvidia_smi': None, 'tails': {}}
    if run_cmd is not None:
        try:   # new files of this folder are not compressed (a compressed write of a multi-GiB dump is too slow here)
            run_cmd(['compact', '/U', out_dir], capture_output=True, text=True)
        except Exception:  # noqa: BLE001
            pass
    smi: dict = {}

    def smi_work():
        try:
            smi['text'] = tools.nvidia_smi()
        except Exception as e:  # noqa: BLE001
            smi['error'] = repr(e)
    th = threading.Thread(target=smi_work, name='freeze-nvidia-smi', daemon=True)
    th.start()
    # the dump first: the engine's own watchdog ends the process about two minutes after the stall
    memory = None
    try:
        memory = tools.process_memory(pid)
    except Exception:  # noqa: BLE001
        memory = None
    free = tools.free_bytes(out_dir)
    kinds, reasons = choose_dump(mode, memory, free)
    info['dump_skipped'] = list(reasons)
    for r in reasons:
        log(f'[freeze] {r}')
    for kind in kinds:
        path = os.path.join(out_dir, f'Cyberpunk2077-{pid}-{kind}.dmp')
        t0 = time.monotonic()
        try:
            ok, err = tools.write_dump(pid, path, DUMP_FLAGS[kind])
        except Exception as e:  # noqa: BLE001
            ok, err = False, repr(e)
        seconds = round(time.monotonic() - t0, 1)
        if ok:
            size = os.path.getsize(path) if os.path.isfile(path) else 0
            info['dump'] = {'kind': kind, 'path': path, 'bytes': size, 'seconds': seconds, 'flags': f'0x{DUMP_FLAGS[kind]:X}'}
            log(f'[freeze] {kind} dump written: {path} ({size / GIB:.2f} GiB in {seconds} s)')
            break
        info['dump_skipped'].append(f'{kind} dump failed after {seconds} s: {err}')
        log(f'[freeze] {kind} dump failed: {err}')
        try:
            if os.path.isfile(path):
                os.remove(path)
        except OSError:
            pass
    if info['dump'] is None and mode != 'off':
        log('[freeze] NO dump was written (see the reasons above)')
    th.join(timeout=30.0)
    if smi.get('text'):
        p = os.path.join(out_dir, 'nvidia-smi.txt')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(smi['text'])
        info['nvidia_smi'] = p
        log(f'[freeze] nvidia-smi: two samples written to {p}')
    else:
        info['nvidia_smi_note'] = smi.get('error') or 'nvidia-smi not found'
        log(f'[freeze] nvidia-smi: not taken ({info["nvidia_smi_note"]})')
    for stem, src in tails.items():
        text = tail_lines(src)
        p = os.path.join(out_dir, f'{stem}_tail.txt')
        try:
            with open(p, 'w', encoding='utf-8') as f:
                f.write(f'# last lines of {os.path.basename(src) if src else "(none)"}\n{text}')
            info['tails'][stem] = p
        except OSError as e:
            info['tails'][stem] = f'not written: {e!r}'
    try:
        with open(os.path.join(out_dir, 'freeze_evidence.json'), 'w', encoding='utf-8') as f:
            json.dump(info, f, indent=1)
    except OSError:
        pass
    return info


# ------------------------------------------------------------------------------------------------ verdict rule
def key_of(entry: str) -> str:
    """'name#run' of a failure text 'name#run.step: ...' or 'name#run: ...'."""
    return entry.split(':', 1)[0].split('.', 1)[0].strip()


def not_judged_keys(scenarios: list, since: datetime, interrupted: str | None = None, margin_s: float = 1.0) -> list[str]:
    """'name#run' of every scenario whose window reaches into the frozen period (it ended at or after `since` minus the margin)
    plus the one that was running when the freeze was detected. Their steps were played against a frozen game."""
    out: list[str] = []
    for s in scenarios or []:
        end = s.get('end')
        if isinstance(end, datetime) and (end - since).total_seconds() >= -margin_s:
            out.append(f'{s["name"]}#{s["run"]}')
    if interrupted and interrupted not in out:
        out.append(interrupted)
    return out
