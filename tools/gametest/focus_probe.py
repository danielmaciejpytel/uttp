"""Focus probe: does the game accept virtual pad input while its window is not the foreground window?

Run with the game at gameplay (V on foot, open ground ahead and behind) and the utpp_core NightAuto POS lines on
(GetMod("utpp_core").NightAuto(true, 0.2)). For each variant the probe pushes the left stick of its own virtual pad for
--seconds, then measures how far V moved from the '[night] POS' lines of the mod log:

  fg               game in the foreground (baseline; the background variants are skipped if V does not move)
  bg               a small probe window (top-most, over the middle of the game) is the foreground window
  bg_cover         the probe window covers the whole game window (occluded game, never minimised)
  bg_no_throttle   bg + Windows power throttling (EcoQoS, timer resolution) switched off for the game process
  bg_priority      bg + game process priority class ABOVE_NORMAL
  bg_activate_msg  bg + posted WM_NCACTIVATE / WM_ACTIVATE / WM_ACTIVATEAPP / WM_SETFOCUS to the game window
  fg_control       game in the foreground again (control run)

Each mitigation is applied for its variant only and restored afterwards (original priority class, power throttling back
to system control, cursor clip released, game window re-focused). The stick direction alternates between variants so V
stays near the start point. Output, one line per variant:

  variant=<name> moved_m=<x> ok=<bool> pos_lines=<n> stale=<0|1|-> fg=<game|probe|other> [note=...]

pos_lines = POS lines logged during the push; stale = 1 when one of them reports the Locomotion reason kStale (the
player state machine stopped ticking). Lines, stale=0 and no movement = pad input dropped while inactive; stale=1 or no
lines = the game simulation (or its script update) is paused or throttled while inactive.

The probe never sends keyboard or mouse input. Its virtual pad is unplugged when it exits; the game may then show the
"controller disconnected" dialog (dismiss it with run.py --dismiss-dialog). If the padd daemon is running, its pad
stays plugged in (neutral) next to the probe's pad.

  python focus_probe.py [--seconds 2] [--variants fg,bg,fg_control] [--log <utpp log>] [--dry-run]
"""
from __future__ import annotations

import argparse
import ctypes
import math
import os
import re
import sys
import threading
import time
from ctypes import wintypes
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_log  # noqa: E402
import gt_night as gn  # noqa: E402
from gt_scenario import Frame  # noqa: E402
from gt_win32 import ForegroundGuard, GuardError  # noqa: E402

ALL_VARIANTS = ('fg', 'bg', 'bg_cover', 'bg_no_throttle', 'bg_priority', 'bg_activate_msg', 'fg_control')
DEFAULT_VARIANTS = ','.join(ALL_VARIANTS)
NIGHTAUTO_HINT = 'GetMod("utpp_core").NightAuto(true, 0.2)'

WM_ACTIVATE, WM_SETFOCUS, WM_CLOSE, WM_QUIT = 0x0006, 0x0007, 0x0010, 0x0012
WM_ACTIVATEAPP, WM_NCACTIVATE = 0x001C, 0x0086
WS_EX_TOPMOST, WS_EX_TOOLWINDOW = 0x00000008, 0x00000080
WS_POPUP, WS_VISIBLE, WS_CAPTION, WS_BORDER = 0x80000000, 0x10000000, 0x00C00000, 0x00800000
HWND_TOPMOST = -1
SWP_SHOWWINDOW, SWP_NOACTIVATE = 0x0040, 0x0010
PROCESS_SET_INFORMATION, PROCESS_QUERY_LIMITED_INFORMATION = 0x0200, 0x1000
ABOVE_NORMAL_PRIORITY_CLASS = 0x00008000
PROCESS_POWER_THROTTLING_INFO = 4                   # PROCESS_INFORMATION_CLASS ProcessPowerThrottling
POWER_THROTTLING_EXECUTION_SPEED, POWER_THROTTLING_IGNORE_TIMER_RESOLUTION = 0x1, 0x4


# ------------------------------------------------------------------------------------------------ log measurement
def pos_window(text: str, t0: datetime, t1: datetime) -> list:
    """POS samples [(ts, (x, y))] of the log text with t0 <= ts <= t1."""
    return [s for s in gn.pos_samples(text) if t0 <= s[0] <= t1]


FREEMOVE_RE = re.compile(r'freeMove=(\d)/0x([0-9A-Fa-f]+)')
REASON_STALE = 0x20000      # Locomotion reason kStale: no player state machine locomotion tick (src/input/LocomotionCore.hpp)


def stale_flag(text: str, t0: datetime, t1: datetime) -> str:
    """'1' if a POS line in [t0, t1] reports kStale (state machine not ticking: paused), '0' if none of the lines does,
    '-' if no line in the window carries a readable freeMove field."""
    seen = False
    for line in text.splitlines():
        if '[night] POS p=' not in line:
            continue
        m, fm = gn.LOGTS_RE.match(line), FREEMOVE_RE.search(line)
        if not m or not fm:
            continue
        ts = datetime.strptime(m.group(1)[:23], '%Y-%m-%d %H:%M:%S.%f')
        if t0 <= ts <= t1:
            seen = True
            if int(fm.group(2), 16) & REASON_STALE:
                return '1'
    return '0' if seen else '-'


def distance(p0, p1) -> float | None:
    if p0 is None or p1 is None:
        return None
    return math.hypot(p1[0] - p0[0], p1[1] - p0[1])


def nightauto_live(text: str, now: datetime, max_age: float = 1.5, min_lines: int = 2, span_s: float = 3.0) -> bool:
    """At least `min_lines` POS lines in the last `span_s` seconds and the newest one at most `max_age` s old."""
    recent = [s for s in gn.pos_samples(text) if s[0] >= now - timedelta(seconds=span_s)]
    return len(recent) >= min_lines and (now - recent[-1][0]).total_seconds() <= max_age


def verdict(name: str, moved: float | None, base: float | None, min_move: float, min_ratio: float) -> bool:
    if moved is None:
        return False
    if name.startswith('fg') or base is None:
        return moved >= min_move
    return moved >= max(min_move, min_ratio * base)


def format_line(name: str, moved: float | None, ok: bool, pos_lines: int, fg: str, note: str = '',
                stale: str = '-') -> str:
    m = 'nan' if moved is None else f'{moved:.2f}'
    return (f'variant={name} moved_m={m} ok={ok} pos_lines={pos_lines} stale={stale} fg={fg}'
            + (f' note={note}' if note else ''))


# ------------------------------------------------------------------------------------------------ Win32 (probe only)
class ProbeWin32:
    """Own user32 / kernel32 instances (prototypes independent of gt_win32)."""

    def __init__(self):
        self.user32 = ctypes.WinDLL('user32', use_last_error=True)
        self.kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
        u, k = self.user32, self.kernel32
        u.CreateWindowExW.argtypes = [wintypes.DWORD, wintypes.LPCWSTR, wintypes.LPCWSTR, wintypes.DWORD, ctypes.c_int,
                                      ctypes.c_int, ctypes.c_int, ctypes.c_int, wintypes.HWND, wintypes.HMENU,
                                      wintypes.HINSTANCE, wintypes.LPVOID]
        u.CreateWindowExW.restype = wintypes.HWND
        u.GetMessageW.argtypes = [ctypes.POINTER(wintypes.MSG), wintypes.HWND, wintypes.UINT, wintypes.UINT]
        u.TranslateMessage.argtypes = [ctypes.POINTER(wintypes.MSG)]
        u.DispatchMessageW.argtypes = [ctypes.POINTER(wintypes.MSG)]
        u.PostMessageW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
        u.PostThreadMessageW.argtypes = [wintypes.DWORD, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
        u.SetWindowPos.argtypes = [wintypes.HWND, wintypes.HWND, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                   wintypes.UINT]
        u.GetWindowRect.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.RECT)]
        u.GetForegroundWindow.restype = wintypes.HWND
        u.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
        u.GetWindowThreadProcessId.restype = wintypes.DWORD
        u.ClipCursor.argtypes = [ctypes.c_void_p]
        u.ShowWindowAsync.argtypes = [wintypes.HWND, ctypes.c_int]
        k.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        k.OpenProcess.restype = wintypes.HANDLE
        k.CloseHandle.argtypes = [wintypes.HANDLE]
        k.GetPriorityClass.argtypes = [wintypes.HANDLE]
        k.GetPriorityClass.restype = wintypes.DWORD
        k.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        k.SetProcessInformation.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
        k.GetCurrentThreadId.restype = wintypes.DWORD

    def foreground(self) -> int:
        return self.user32.GetForegroundWindow() or 0

    def rect(self, hwnd: int) -> tuple[int, int, int, int]:
        r = wintypes.RECT()
        self.user32.GetWindowRect(hwnd, ctypes.byref(r))
        return r.left, r.top, r.right, r.bottom

    def post(self, hwnd: int, msg: int, wp: int = 0, lp: int = 0) -> bool:
        return bool(self.user32.PostMessageW(hwnd, msg, wp, lp))

    def unclip_cursor(self) -> None:
        self.user32.ClipCursor(None)

    def game_thread(self, hwnd: int) -> int:
        return int(self.user32.GetWindowThreadProcessId(hwnd, None))


class POWER_THROTTLING_STATE(ctypes.Structure):
    _fields_ = [('Version', wintypes.ULONG), ('ControlMask', wintypes.ULONG), ('StateMask', wintypes.ULONG)]


class ProcessTweak:
    """Priority class / power throttling of the game process; restore() undoes what apply_*() changed."""

    def __init__(self, w: ProbeWin32, pid: int):
        self.w, self.pid, self.undo = w, pid, []

    def _open(self):
        h = self.w.kernel32.OpenProcess(PROCESS_SET_INFORMATION | PROCESS_QUERY_LIMITED_INFORMATION, False, self.pid)
        if not h:
            raise OSError(f'OpenProcess({self.pid}) failed err={ctypes.get_last_error()}')
        return h

    def _throttle(self, control: int, state: int) -> None:
        h = self._open()
        try:
            st = POWER_THROTTLING_STATE(1, control, state)
            if not self.w.kernel32.SetProcessInformation(h, PROCESS_POWER_THROTTLING_INFO, ctypes.byref(st),
                                                         ctypes.sizeof(st)):
                raise OSError(f'SetProcessInformation failed err={ctypes.get_last_error()}')
        finally:
            self.w.kernel32.CloseHandle(h)

    def apply_no_throttle(self) -> None:
        mask = POWER_THROTTLING_EXECUTION_SPEED | POWER_THROTTLING_IGNORE_TIMER_RESOLUTION
        self._throttle(mask, 0)                         # opt out: never throttled
        self.undo.append(lambda: self._throttle(0, 0))  # back to system-managed

    def apply_priority(self, cls: int = ABOVE_NORMAL_PRIORITY_CLASS) -> None:
        h = self._open()
        try:
            old = self.w.kernel32.GetPriorityClass(h)
            if not old or not self.w.kernel32.SetPriorityClass(h, cls):
                raise OSError(f'priority change failed err={ctypes.get_last_error()}')
        finally:
            self.w.kernel32.CloseHandle(h)

        def back():
            h2 = self._open()
            try:
                self.w.kernel32.SetPriorityClass(h2, old)
            finally:
                self.w.kernel32.CloseHandle(h2)
        self.undo.append(back)

    def restore(self) -> list[str]:
        errs = []
        while self.undo:
            try:
                self.undo.pop()()
            except OSError as e:
                errs.append(str(e))
        return errs


class ProbeWindow:
    """Small top-level window of this process with its own message loop thread (never ghosts), used as the foreground
    window of the background variants. Predefined STATIC class: no class registration, no Python window procedure."""

    def __init__(self, w: ProbeWin32):
        self.w = w
        self.hwnd = 0
        self.tid = 0
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, name='focus-probe-window', daemon=True)

    def _run(self) -> None:
        u = self.w.user32
        self.tid = int(self.w.kernel32.GetCurrentThreadId())
        self.hwnd = u.CreateWindowExW(WS_EX_TOPMOST | WS_EX_TOOLWINDOW, 'STATIC', 'UTPP focus probe',
                                      WS_POPUP | WS_CAPTION | WS_BORDER | WS_VISIBLE, 0, 0, 360, 120,
                                      None, None, None, None) or 0
        self._ready.set()
        if not self.hwnd:
            return
        msg = wintypes.MSG()
        while u.GetMessageW(ctypes.byref(msg), None, 0, 0) > 0:
            u.TranslateMessage(ctypes.byref(msg))
            u.DispatchMessageW(ctypes.byref(msg))

    def start(self) -> int:
        self._thread.start()
        self._ready.wait(5.0)
        if not self.hwnd:
            raise OSError(f'probe window not created err={ctypes.get_last_error()}')
        return self.hwnd

    def place(self, x: int, y: int, cx: int, cy: int) -> None:
        self.w.user32.SetWindowPos(self.hwnd, HWND_TOPMOST, int(x), int(y), max(int(cx), 120), max(int(cy), 60),
                                   SWP_SHOWWINDOW | SWP_NOACTIVATE)

    def hide(self) -> None:
        if self.hwnd:
            self.w.user32.ShowWindowAsync(self.hwnd, 0)    # SW_HIDE; place() shows it again

    def close(self) -> None:
        if not self._thread.is_alive():
            return
        if self.hwnd:
            self.w.post(self.hwnd, WM_CLOSE)
        if self.tid:
            self.w.user32.PostThreadMessageW(self.tid, WM_QUIT, 0, 0)
        self._thread.join(2.0)


# ------------------------------------------------------------------------------------------------ pad guard
class ProbeGuard(ForegroundGuard):
    """ForegroundGuard whose foreground requirement can be switched off for the background variants. With it off,
    check() still requires a running game process and aborts on Pause (ESC is left to other applications)."""

    require_foreground = True

    @property
    def background(self) -> bool:
        """Read by the pad daemon client: a background report skips the daemon's own foreground test."""
        return not self.require_foreground

    def check(self) -> None:
        if self.require_foreground:
            return super().check()
        self.checks += 1
        now = self.clock()
        if now - self._last_proc_check >= self.process_check_interval:
            self._last_proc_check = now
            alive = [p for p in self.game_pids if self.api.process_alive(p)]
            if not alive:
                raise GuardError(f'{self.exe_name} is not running')
            self.game_pids = alive
        if self.api.key_down_now(0x13):   # Pause
            raise GuardError('user abort key 0x13 pressed')


# ------------------------------------------------------------------------------------------------ live probe
class Probe:
    def __init__(self, args):
        self.args = args
        self.log_path = args.log or gt_log.newest_log(args.log_dir)
        if not self.log_path:
            raise GuardError(f'no utpp-*.log in {args.log_dir}')
        self.w = ProbeWin32()
        self.guard = ProbeGuard()
        self.pad = None
        self.win = ProbeWindow(self.w)
        self.sign = 1.0
        self.game_hwnd = 0
        self.game_pid = 0
        self.mark = gt_log.log_mark(self.log_path)
        if self.mark is None:
            raise GuardError(f'cannot read {self.log_path}')

    def open(self) -> None:
        """Game window to the foreground, then plug in the probe's virtual pad."""
        from gt_input import PadDriver, PaddClient, RemotePadDriver, padd_ping   # vgamepad is imported only here
        self.guard.ensure_focus()                   # game running, has a window, is (or becomes) the foreground
        self.game_hwnd = self.guard.game_hwnd
        self.game_pid = self.guard.game_pids[0]
        if padd_ping():
            print('[focus_probe] the padd daemon is running: its pad is used (the one the game already follows)')
            self.pad = RemotePadDriver(self.guard, PaddClient())
        else:
            self.pad = PadDriver(self.guard)
        time.sleep(1.0)                             # let the game enumerate the new pad

    # -- log --
    def tail(self) -> str:
        """Log text written since the probe started (self.mark), not the whole session."""
        return gt_log.read_session_text(self.log_path, since=self.mark)

    def fresh_pos(self, timeout: float) -> tuple | None:
        end = time.monotonic() + timeout
        while True:
            p = gn.latest_pos(self.tail(), datetime.now(), self.args.pos_max_age)
            if p is not None:
                return p['p'][:2]
            if time.monotonic() >= end:
                return None
            time.sleep(0.1)

    def fg_label(self) -> str:
        fg = self.w.foreground()
        if fg == self.game_hwnd:
            return 'game'
        return 'probe' if fg == self.win.hwnd else 'other'

    # -- focus --
    def focus_game(self) -> None:
        self.win.hide()                             # the game must not stay covered by the probe window
        self.guard.require_foreground = True
        self.guard.ensure_focus()

    def focus_probe(self, cover: bool) -> None:
        gl, gt, gr, gb = self.w.rect(self.game_hwnd)
        if cover:
            self.win.place(gl, gt, gr - gl, gb - gt)
        else:
            self.win.place((gl + gr) // 2 - 180, (gt + gb) // 2 - 60, 360, 120)
        self.guard.require_foreground = False
        for _ in range(3):
            self.guard.api.focus(self.win.hwnd)     # AttachThreadInput + SetForegroundWindow (gt_win32)
            time.sleep(0.3)
            if self.w.foreground() == self.win.hwnd:
                return
        raise GuardError('could not make the probe window the foreground window')

    # -- variants --
    def push(self) -> tuple[datetime, datetime, str, str]:
        fr = Frame(0.0, (0.0, self.sign * self.args.stick), (0.0, 0.0), 0.0, 0.0, 0)
        neutral = Frame(0.0, (0.0, 0.0), (0.0, 0.0), 0.0, 0.0, 0)
        fg0 = self.fg_label()
        t0 = datetime.now()
        try:
            end = time.monotonic() + self.args.seconds
            while time.monotonic() < end:
                self.pad.apply(fr)                  # guard check before every report
                time.sleep(1.0 / 60)
        finally:
            self.pad.release()
        t1 = datetime.now()
        fg1 = self.fg_label()
        self.pad.apply(neutral)
        self.sign = -self.sign
        return t0, t1, fg0, fg1

    def run_variant(self, name: str, base: float | None) -> tuple[float | None, bool]:
        tweak = ProcessTweak(self.w, self.game_pid)
        notes = []
        background = name.startswith('bg')
        self.focus_game()
        p0 = self.fresh_pos(3.0)
        try:
            if background:
                self.focus_probe(cover=(name == 'bg_cover'))
                if name == 'bg_no_throttle':
                    tweak.apply_no_throttle()
                elif name == 'bg_priority':
                    tweak.apply_priority()
                elif name == 'bg_activate_msg':
                    tid = self.w.game_thread(self.game_hwnd)
                    for msg, wp, lp in ((WM_NCACTIVATE, 1, 0), (WM_ACTIVATE, 1, 0), (WM_ACTIVATEAPP, 1, tid),
                                        (WM_SETFOCUS, 0, 0)):
                        if not self.w.post(self.game_hwnd, msg, wp, lp):
                            notes.append(f'post_0x{msg:X}_failed')
                time.sleep(self.args.settle)
            t0, t1, fg0, fg1 = self.push()
            time.sleep(self.args.settle)
            text = self.tail()
            during = pos_window(text, t0, t1)
            after = pos_window(text, t0, datetime.now())
            p1 = after[-1][1] if after else None
        finally:
            if name == 'bg_activate_msg':
                self.w.unclip_cursor()
            notes += tweak.restore()
            self.focus_game()
        if p1 is None:                              # nothing logged while inactive: position once the game is back
            p1 = self.fresh_pos(3.0)
            notes.append('p1_after_refocus')
        if p0 is None:
            notes.append('no_p0')
        expected = 'probe' if background else 'game'
        fg = fg0 if fg0 == fg1 else f'{fg0}>{fg1}'
        moved = distance(p0, p1)
        ok = verdict(name, moved, base, self.args.min_move, self.args.min_ratio)
        if fg0 != expected or fg1 != expected:
            notes.append('foreground_mismatch')
            ok = False
        print(format_line(name, moved, ok, len(during), fg, ','.join(notes), stale_flag(text, t0, t1)), flush=True)
        return moved, ok

    def run(self, variants: list[str]) -> int:
        time.sleep(2.5)                             # a few NightAuto periods after the start mark
        if not nightauto_live(self.tail(), datetime.now(), self.args.pos_max_age):
            print(f'[focus_probe] no fresh NightAuto POS lines in {os.path.basename(self.log_path)}; enable them '
                  f'(CET console or the test command channel): {NIGHTAUTO_HINT}')
            return 2
        base = None
        failed = 0
        try:
            self.open()
            self.win.start()
            self.win.place(0, 0, 360, 120)
            self.focus_game()
            self.pad.wake('rs')                     # switch the game's input mode to the pad (harness default)
            for name in variants:
                if name.startswith('bg') and (base is None or base < self.args.min_move):
                    print(format_line(name, None, False, 0, '-', 'skipped_baseline_failed'), flush=True)
                    failed += 1
                    continue
                moved, ok = self.run_variant(name, base)
                if name == 'fg' or (base is None and name.startswith('fg')):
                    base = moved
                    if not ok:
                        print('[focus_probe] baseline failed: V did not move with the game in the foreground')
                failed += 0 if ok else 1
        except GuardError as e:
            print(f'[focus_probe] aborted: {e}')
            return 3
        finally:
            if self.pad is not None:
                self.pad.release()
            self.win.close()
            try:
                self.focus_game()
            except GuardError:
                pass
            if self.pad is not None:
                self.pad.close()
        return 0 if failed == 0 else 1


# ------------------------------------------------------------------------------------------------ dry run
def dry_run() -> int:
    """Measurement functions on a synthetic log (no pad, no windows)."""
    t = datetime(2026, 11, 14, 12, 0, 0)

    def line(dt: float, x: float, y: float, n: int, reasons: int) -> str:
        ts = (t + timedelta(seconds=dt)).strftime('%Y-%m-%d %H:%M:%S.%f')[:23]
        return (f'[{ts}] [info] [1] [UTPP] UTPP CET [night] POS p={x:.2f},{y:.2f},10.00 cf=0.000,1.000 v=nil '
                f'src=none w=0 m=0 freeMove=0/0x{reasons:X} psmW=0 ub=0 wi=none n={n}')
    lines = [line(0.2 * i, 0.0, 0.0, i, 0x60000) for i in range(5)]                    # paused (stale)
    lines += [line(1.0 + 0.2 * i, 0.0, 0.4 * i, 5 + i, 0x40000) for i in range(11)]    # walking 2 s, 4 m
    text = '\n'.join(lines) + '\n'
    t0, t1 = t + timedelta(seconds=1.0), t + timedelta(seconds=3.0)
    during = pos_window(text, t0, t1)
    p0, p1 = (0.0, 0.0), during[-1][1]
    moved = distance(p0, p1)
    checks = [
        ('pos_window', len(during) == 11),
        ('distance', moved is not None and abs(moved - 4.0) < 1e-6),
        ('nightauto_live', nightauto_live(text, t + timedelta(seconds=3.2))),
        ('nightauto_stale', not nightauto_live(text, t + timedelta(seconds=9.0))),
        ('verdict_fg', verdict('fg', moved, None, 0.5, 0.5)),
        ('verdict_bg_ratio', not verdict('bg', 1.5, moved, 0.5, 0.5) and verdict('bg', 2.5, moved, 0.5, 0.5)),
        ('verdict_none', not verdict('bg', None, moved, 0.5, 0.5)),
        ('stale_walk', stale_flag(text, t0, t1) == '0'),
        ('stale_paused', stale_flag(text, t, t + timedelta(seconds=0.9)) == '1'),
        ('stale_none', stale_flag(text, t1 + timedelta(seconds=1), t1 + timedelta(seconds=2)) == '-'),
        ('format', format_line('bg', None, False, 0, 'probe', 'x') ==
         'variant=bg moved_m=nan ok=False pos_lines=0 stale=- fg=probe note=x'),
    ]
    print(format_line('fg', moved, True, len(during), 'game', '', stale_flag(text, t0, t1)))
    bad = [n for n, ok in checks if not ok]
    print('[focus_probe] dry-run ' + ('ok' if not bad else 'FAILED: ' + ', '.join(bad)))
    return 0 if not bad else 1


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=2.0, help='left stick push per variant (s)')
    ap.add_argument('--variants', default=DEFAULT_VARIANTS, help=f'comma list of {", ".join(ALL_VARIANTS)}')
    ap.add_argument('--stick', type=float, default=1.0, help='left stick magnitude (0..1)')
    ap.add_argument('--settle', type=float, default=1.0, help='wait after a focus change and after the push (s)')
    ap.add_argument('--min-move', type=float, default=0.5, help='fg: V must move at least this far (m)')
    ap.add_argument('--min-ratio', type=float, default=0.5, help='bg: moved >= ratio * fg distance')
    ap.add_argument('--pos-max-age', type=float, default=1.5, help='max age of a NightAuto POS line (s)')
    ap.add_argument('--log', default=None, help='UTPP log (default: newest utpp-*.log in --log-dir)')
    ap.add_argument('--log-dir', default=gt_log.DEFAULT_LOG_DIR)
    ap.add_argument('--dry-run', action='store_true', help='check the measurement on a synthetic log and exit')
    a = ap.parse_args(argv)
    a.variant_list = [v.strip() for v in a.variants.split(',') if v.strip()]
    unknown = [v for v in a.variant_list if v not in ALL_VARIANTS]
    if unknown:
        ap.error(f'unknown variants: {", ".join(unknown)}')
    if not 0.0 < a.stick <= 1.0 or a.seconds <= 0:
        ap.error('--stick must be in (0, 1] and --seconds > 0')
    return a


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.dry_run:
        return dry_run()
    if sys.platform != 'win32':
        print('[focus_probe] Windows only')
        return 2
    try:
        probe = Probe(args)
    except GuardError as e:
        print(f'[focus_probe] {e}')
        return 2
    return probe.run(args.variant_list)


if __name__ == '__main__':
    sys.exit(main())
