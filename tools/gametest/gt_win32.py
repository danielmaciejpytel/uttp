"""Win32 helpers for the UTPP in-game test harness (ctypes only, stdlib).

Safety model (see ForegroundGuard):
- every input (pad report, key, click) is preceded by a guard check: the foreground window must belong to a process
  whose image name is Cyberpunk2077.exe, and the game process must still be running;
- on any guard failure the caller releases all inputs (zero pad report, key-ups of keys it pressed) and aborts;
- focusing the game uses ShowWindow/BringWindowToTop/SetForegroundWindow with AttachThreadInput; it never injects
  keystrokes into the current (non-game) foreground app (no Alt-key trick).

Win32Api is a thin wrapper so tests can replace it with a fake object (same method names).
"""
from __future__ import annotations

import ctypes
import os
import sys
import time
from ctypes import wintypes

GAME_EXE = 'Cyberpunk2077.exe'

IS_WINDOWS = sys.platform == 'win32'

if IS_WINDOWS:
    user32 = ctypes.WinDLL('user32', use_last_error=True)
    kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
    try:
        winmm = ctypes.WinDLL('winmm')
    except OSError:  # pragma: no cover
        winmm = None

ULONG_PTR = ctypes.c_size_t

INPUT_MOUSE = 0
INPUT_KEYBOARD = 1
KEYEVENTF_KEYUP = 0x0002
KEYEVENTF_UNICODE = 0x0004
KEYEVENTF_SCANCODE = 0x0008
KEYEVENTF_EXTENDEDKEY = 0x0001
MOUSEEVENTF_LEFTDOWN = 0x0002
MOUSEEVENTF_LEFTUP = 0x0004

VK_RETURN = 0x0D
VK_CONTROL = 0x11
VK_ESCAPE = 0x1B
VK_PAUSE = 0x13
VK_OEM_3 = 0xC0  # grave / tilde on US layout
VK_A = 0x41

PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
STILL_ACTIVE = 259
TH32CS_SNAPPROCESS = 0x00000002
SW_RESTORE = 9
GA_ROOT = 2


class MOUSEINPUT(ctypes.Structure):
    _fields_ = [('dx', wintypes.LONG), ('dy', wintypes.LONG), ('mouseData', wintypes.DWORD),
                ('dwFlags', wintypes.DWORD), ('time', wintypes.DWORD), ('dwExtraInfo', ULONG_PTR)]


class KEYBDINPUT(ctypes.Structure):
    _fields_ = [('wVk', wintypes.WORD), ('wScan', wintypes.WORD), ('dwFlags', wintypes.DWORD),
                ('time', wintypes.DWORD), ('dwExtraInfo', ULONG_PTR)]


class HARDWAREINPUT(ctypes.Structure):
    _fields_ = [('uMsg', wintypes.DWORD), ('wParamL', wintypes.WORD), ('wParamH', wintypes.WORD)]


class _INPUTUNION(ctypes.Union):
    _fields_ = [('mi', MOUSEINPUT), ('ki', KEYBDINPUT), ('hi', HARDWAREINPUT)]


class INPUT(ctypes.Structure):
    _anonymous_ = ('u',)
    _fields_ = [('type', wintypes.DWORD), ('u', _INPUTUNION)]


class PROCESSENTRY32W(ctypes.Structure):
    _fields_ = [('dwSize', wintypes.DWORD), ('cntUsage', wintypes.DWORD), ('th32ProcessID', wintypes.DWORD),
                ('th32DefaultHeapID', ULONG_PTR), ('th32ModuleID', wintypes.DWORD), ('cntThreads', wintypes.DWORD),
                ('th32ParentProcessID', wintypes.DWORD), ('pcPriClassBase', wintypes.LONG),
                ('dwFlags', wintypes.DWORD), ('szExeFile', wintypes.WCHAR * 260)]


def _setup_prototypes():
    if not IS_WINDOWS:
        return
    user32.GetForegroundWindow.restype = wintypes.HWND
    user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    user32.GetWindowThreadProcessId.restype = wintypes.DWORD
    user32.SetForegroundWindow.argtypes = [wintypes.HWND]
    user32.BringWindowToTop.argtypes = [wintypes.HWND]
    user32.ShowWindow.argtypes = [wintypes.HWND, ctypes.c_int]
    user32.IsIconic.argtypes = [wintypes.HWND]
    user32.IsWindowVisible.argtypes = [wintypes.HWND]
    user32.GetWindow.argtypes = [wintypes.HWND, ctypes.c_uint]
    user32.GetWindow.restype = wintypes.HWND
    user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
    user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user32.GetWindowRect.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.RECT)]
    user32.GetClientRect.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.RECT)]
    user32.ClientToScreen.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.POINT)]
    user32.ScreenToClient.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.POINT)]
    user32.WindowFromPoint.argtypes = [wintypes.POINT]
    user32.WindowFromPoint.restype = wintypes.HWND
    user32.GetAncestor.argtypes = [wintypes.HWND, ctypes.c_uint]
    user32.GetAncestor.restype = wintypes.HWND
    user32.AttachThreadInput.argtypes = [wintypes.DWORD, wintypes.DWORD, wintypes.BOOL]
    user32.SendInput.argtypes = [wintypes.UINT, ctypes.POINTER(INPUT), ctypes.c_int]
    user32.SendInput.restype = wintypes.UINT
    user32.MapVirtualKeyW.argtypes = [wintypes.UINT, wintypes.UINT]
    user32.MapVirtualKeyW.restype = wintypes.UINT
    user32.SetCursorPos.argtypes = [ctypes.c_int, ctypes.c_int]
    user32.GetCursorPos.argtypes = [ctypes.POINTER(wintypes.POINT)]
    user32.GetAsyncKeyState.argtypes = [ctypes.c_int]
    user32.GetAsyncKeyState.restype = ctypes.c_short
    kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    kernel32.OpenProcess.restype = wintypes.HANDLE
    kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel32.QueryFullProcessImageNameW.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR,
                                                    ctypes.POINTER(wintypes.DWORD)]
    kernel32.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    kernel32.GetCurrentThreadId.restype = wintypes.DWORD
    kernel32.CreateToolhelp32Snapshot.argtypes = [wintypes.DWORD, wintypes.DWORD]
    kernel32.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
    kernel32.Process32FirstW.argtypes = [wintypes.HANDLE, ctypes.POINTER(PROCESSENTRY32W)]
    kernel32.Process32NextW.argtypes = [wintypes.HANDLE, ctypes.POINTER(PROCESSENTRY32W)]


_setup_prototypes()

EnumWindowsProc = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM) if IS_WINDOWS else None
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value


class Win32Api:
    """Real Win32 implementation. All read-only except focus(), send_*(), set_cursor()."""

    # ---- read-only queries -------------------------------------------------
    def foreground_window(self) -> int:
        return user32.GetForegroundWindow() or 0

    def window_pid(self, hwnd: int) -> int:
        pid = wintypes.DWORD(0)
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        return pid.value

    def window_thread(self, hwnd: int) -> int:
        return user32.GetWindowThreadProcessId(hwnd, None)

    def process_name(self, pid: int) -> str:
        """Base image name of a process ('' if not accessible)."""
        if not pid:
            return ''
        h = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
        if not h:
            return ''
        try:
            size = wintypes.DWORD(1024)
            buf = ctypes.create_unicode_buffer(1024)
            if not kernel32.QueryFullProcessImageNameW(h, 0, buf, ctypes.byref(size)):
                return ''
            return os.path.basename(buf.value)
        finally:
            kernel32.CloseHandle(h)

    def process_alive(self, pid: int) -> bool:
        h = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
        if not h:
            return False
        try:
            code = wintypes.DWORD(0)
            if not kernel32.GetExitCodeProcess(h, ctypes.byref(code)):
                return False
            return code.value == STILL_ACTIVE
        finally:
            kernel32.CloseHandle(h)

    def list_pids(self, exe_name: str) -> list[int]:
        snap = kernel32.CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0)
        if not snap or snap == INVALID_HANDLE_VALUE:
            return []
        out = []
        try:
            entry = PROCESSENTRY32W()
            entry.dwSize = ctypes.sizeof(PROCESSENTRY32W)
            ok = kernel32.Process32FirstW(snap, ctypes.byref(entry))
            while ok:
                if entry.szExeFile.lower() == exe_name.lower():
                    out.append(int(entry.th32ProcessID))
                ok = kernel32.Process32NextW(snap, ctypes.byref(entry))
        finally:
            kernel32.CloseHandle(snap)
        return out

    def find_main_window(self, pids: list[int]) -> int:
        """Largest visible, unowned top-level window of one of the pids (0 if none)."""
        found = []
        pidset = set(pids)

        def cb(hwnd, _lp):
            if user32.IsWindowVisible(hwnd) and not user32.GetWindow(hwnd, 4):  # GW_OWNER = 4
                if self.window_pid(hwnd) in pidset:
                    r = self.window_rect(hwnd)
                    found.append(((r[2] - r[0]) * (r[3] - r[1]), hwnd))
            return True

        user32.EnumWindows(EnumWindowsProc(cb), 0)
        if not found:
            return 0
        found.sort(reverse=True)
        return found[0][1]

    def window_rect(self, hwnd: int) -> tuple[int, int, int, int]:
        r = wintypes.RECT()
        user32.GetWindowRect(hwnd, ctypes.byref(r))
        return r.left, r.top, r.right, r.bottom

    def client_rect_screen(self, hwnd: int) -> tuple[int, int, int, int]:
        """Client area in screen coordinates (left, top, right, bottom)."""
        r = wintypes.RECT()
        user32.GetClientRect(hwnd, ctypes.byref(r))
        p = wintypes.POINT(0, 0)
        user32.ClientToScreen(hwnd, ctypes.byref(p))
        return p.x, p.y, p.x + r.right, p.y + r.bottom

    def root_window_at(self, x: int, y: int) -> int:
        h = user32.WindowFromPoint(wintypes.POINT(x, y))
        return user32.GetAncestor(h, GA_ROOT) if h else 0

    def cursor_pos(self) -> tuple[int, int]:
        p = wintypes.POINT()
        user32.GetCursorPos(ctypes.byref(p))
        return p.x, p.y

    def key_down_now(self, vk: int) -> bool:
        return bool(user32.GetAsyncKeyState(vk) & 0x8000)

    def window_title(self, hwnd: int) -> str:
        if not hwnd:
            return ''
        n = user32.GetWindowTextLengthW(hwnd)
        buf = ctypes.create_unicode_buffer(max(1, n + 1))
        user32.GetWindowTextW(hwnd, buf, n + 1)
        return buf.value

    def vk_to_scan(self, vk: int) -> int:
        return int(user32.MapVirtualKeyW(vk, 0))  # MAPVK_VK_TO_VSC

    # ---- state changing -----------------------------------------------------
    def focus(self, hwnd: int) -> None:
        """Bring hwnd to the foreground without injecting input into another application."""
        if user32.IsIconic(hwnd):
            user32.ShowWindow(hwnd, SW_RESTORE)
        fg = self.foreground_window()
        cur = kernel32.GetCurrentThreadId()
        fg_thread = self.window_thread(fg) if fg else 0
        attached = False
        if fg_thread and fg_thread != cur:
            attached = bool(user32.AttachThreadInput(cur, fg_thread, True))
        try:
            user32.BringWindowToTop(hwnd)
            user32.SetForegroundWindow(hwnd)
        finally:
            if attached:
                user32.AttachThreadInput(cur, fg_thread, False)

    # Each send_* call is one atomic SendInput batch (down + up together), so a guard check right before it
    # never leaves a key held and no partial sequence can land in another application.
    def send_scan_chord(self, scans: list[int]) -> None:
        """Press hardware scan codes in order, release in reverse (one batch, KEYEVENTF_SCANCODE).

        The game and CET react to scan codes (raw input); VK-only events were not seen by CET in the first runs.
        Codes > 0xFF are extended keys (0xE0xx).
        """
        evs = [self._scan(s, False) for s in scans] + [self._scan(s, True) for s in reversed(scans)]
        self._send(evs)

    def send_key_chord(self, vks: list[int]) -> None:
        """VK chord sent as scan codes (MapVirtualKey)."""
        self.send_scan_chord([self.vk_to_scan(vk) for vk in vks])

    def send_unicode(self, ch: str) -> None:
        evs = []
        for flags in (KEYEVENTF_UNICODE, KEYEVENTF_UNICODE | KEYEVENTF_KEYUP):
            inp = INPUT(type=INPUT_KEYBOARD)
            inp.ki = KEYBDINPUT(0, ord(ch), flags, 0, 0)
            evs.append(inp)
        self._send(evs)

    def set_cursor(self, x: int, y: int) -> None:
        user32.SetCursorPos(int(x), int(y))

    def send_left_click(self) -> None:
        evs = []
        for f in (MOUSEEVENTF_LEFTDOWN, MOUSEEVENTF_LEFTUP):
            inp = INPUT(type=INPUT_MOUSE)
            inp.mi = MOUSEINPUT(0, 0, 0, f, 0, 0)
            evs.append(inp)
        self._send(evs)

    @staticmethod
    def _scan(scan: int, up: bool) -> INPUT:
        flags = KEYEVENTF_SCANCODE | (KEYEVENTF_KEYUP if up else 0)
        if scan > 0xFF:
            flags |= KEYEVENTF_EXTENDEDKEY
        inp = INPUT(type=INPUT_KEYBOARD)
        inp.ki = KEYBDINPUT(0, scan & 0xFF, flags, 0, 0)
        return inp

    @staticmethod
    def _send(evs: list) -> None:
        arr = (INPUT * len(evs))(*evs)
        n = user32.SendInput(len(evs), arr, ctypes.sizeof(INPUT))
        if n != len(evs):
            raise OSError(f'SendInput sent {n}/{len(evs)} events (err={ctypes.get_last_error()})')

    def sleep(self, s: float) -> None:
        time.sleep(s)


class TimerResolution:
    """Context manager: 1 ms timer resolution while a scenario runs (restored on exit)."""

    def __enter__(self):
        if IS_WINDOWS and winmm is not None:
            winmm.timeBeginPeriod(1)
        return self

    def __exit__(self, *exc):
        if IS_WINDOWS and winmm is not None:
            winmm.timeEndPeriod(1)
        return False


class GuardError(RuntimeError):
    """Raised when sending input would not be safe (game not foreground / not running / user abort)."""


class ForegroundGuard:
    """Checks that the game window is the foreground window before every input.

    check() is cheap (GetForegroundWindow + cached pid->name). The process liveness check runs at most every
    `process_check_interval` seconds. ESC or Pause pressed by the user aborts (the harness never sends those keys).
    """

    def __init__(self, api=None, exe_name: str = GAME_EXE, process_check_interval: float = 1.0,
                 abort_keys=(VK_ESCAPE, VK_PAUSE), clock=time.monotonic, focus_grace: float = 0.0):
        self.api = api if api is not None else Win32Api()
        self.focus_grace = focus_grace      # >0: tolerate a short foreground loss (input held neutral meanwhile)
        self.on_violation = None            # callable releasing inputs (set by the pad driver)
        self.events: list[dict] = []
        self.exe_name = exe_name
        self.process_check_interval = process_check_interval
        self.abort_keys = tuple(abort_keys)
        self.clock = clock
        self.game_pids: list[int] = []
        self.game_hwnd = 0
        self._last_proc_check = -1e9
        self._name_cache: dict[int, str] = {}
        self.checks = 0
        self.extra_check = None             # callable run by every check() after the liveness check (may raise GuardError)

    # -- queries --
    def refresh_game(self) -> bool:
        self.game_pids = self.api.list_pids(self.exe_name)
        self.game_hwnd = self.api.find_main_window(self.game_pids) if self.game_pids else 0
        self._last_proc_check = self.clock()
        return bool(self.game_pids)

    def _name(self, pid: int) -> str:
        n = self._name_cache.get(pid)
        if n is None:
            n = self.api.process_name(pid)
            if n:
                self._name_cache[pid] = n
        return n or ''

    def foreground_is_game(self) -> bool:
        hwnd = self.api.foreground_window()
        if not hwnd:
            return False
        pid = self.api.window_pid(hwnd)
        if not pid:
            return False
        if self.game_pids and pid not in self.game_pids:
            return False
        return self._name(pid).lower() == self.exe_name.lower()

    def describe_foreground(self) -> str:
        """'hwnd=0x.. pid=.. exe=.. title=..' of the current foreground window (abort diagnostics)."""
        hwnd = self.api.foreground_window()
        if not hwnd:
            return 'hwnd=0 (no foreground window: desktop switch, lock screen, UAC or a window being destroyed)'
        pid = self.api.window_pid(hwnd)
        title_fn = getattr(self.api, 'window_title', None)
        title = title_fn(hwnd) if title_fn else ''
        return f'hwnd=0x{hwnd:X} pid={pid} exe={self._name(pid) or "?"} title={title[:60]!r}'

    def status(self) -> dict:
        """Read-only status (used by --dry-run)."""
        running = self.refresh_game()
        return {'game_running': running, 'game_pids': list(self.game_pids), 'game_hwnd': self.game_hwnd,
                'foreground_is_game': self.foreground_is_game() if running else False}

    # -- enforcement --
    def check(self) -> None:
        self.checks += 1
        now = self.clock()
        if now - self._last_proc_check >= self.process_check_interval:
            self._last_proc_check = now
            alive = [p for p in self.game_pids if self.api.process_alive(p)]
            if not alive:
                raise GuardError(f'{self.exe_name} is not running')
            self.game_pids = alive
        if self.extra_check is not None:
            self.extra_check()
        if not self.foreground_is_game():
            first = self.describe_foreground()
            if self.focus_grace > 0:
                # Input stays neutral (on_violation) while waiting; nothing is sent to the other window.
                if self.on_violation is not None:
                    self.on_violation()
                end = self.clock() + self.focus_grace
                while self.clock() < end:
                    self.api.sleep(0.02)
                    if self.foreground_is_game():
                        self.events.append({'type': 'focus_glitch', 'foreground': first})
                        return
            self.events.append({'type': 'focus_lost', 'foreground': first})
            raise GuardError(f'foreground window is not the game (focus lost: {first}) - inputs released, '
                             'run aborted')
        for vk in self.abort_keys:
            if self.api.key_down_now(vk):
                raise GuardError(f'user abort key 0x{vk:02X} pressed')

    def ensure_focus(self, attempts: int = 3, wait: float = 0.4) -> None:
        if not self.refresh_game():
            raise GuardError(f'{self.exe_name} is not running')
        if not self.game_hwnd:
            raise GuardError(f'{self.exe_name} has no visible top-level window')
        for _ in range(attempts):
            if self.foreground_is_game():
                break
            self.api.focus(self.game_hwnd)
            self.api.sleep(wait)
        if not self.foreground_is_game():
            raise GuardError('could not bring the game window to the foreground (click the game window and retry)')
        self._last_proc_check = -1e9
        self.check()

    def wait(self, seconds: float, step: float = 0.05) -> None:
        """Sleep while continuously re-checking the guard."""
        end = self.clock() + seconds
        while True:
            self.check()
            left = end - self.clock()
            if left <= 0:
                return
            self.api.sleep(min(step, left))
