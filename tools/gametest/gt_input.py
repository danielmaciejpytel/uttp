"""Guarded input drivers: virtual Xbox 360 pad (vgamepad/ViGEmBus, local or via the padd daemon) and CET overlay
keyboard/mouse helpers.

Every state-changing call runs ForegroundGuard.check() first. Releasing the pad (zero report) is always allowed:
it is the safe state. The virtual pad is a system-wide XInput device, so it is kept at neutral whenever the game is
not the foreground window.

Pad lifetime: destroying the virtual pad makes the game show a modal "controller disconnected" dialog that pauses
the game. Either keep the pad alive in the padd daemon (run.py --keep-pad, see padd.py) or, after a local pad is
destroyed, dismiss the dialog with a guarded keyboard Enter (scan 0x1C) and verify via fresh MMTRACE ticks
(the trace stops while the game is paused).
"""
from __future__ import annotations

import json
import os
import re
import socket
import sys
import time
import uuid
from datetime import datetime

import gt_log
from gt_scenario import BUTTONS, Frame
from gt_win32 import VK_PAUSE, ForegroundGuard, GuardError

PROJECT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
PYLIBS = os.path.join(PROJECT, 'Tools', 'pylibs')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utpp_paths import game_dir  # noqa: E402
CET_DIR = os.path.join(game_dir(required=False) or '', 'bin', 'x64', 'plugins', 'cyber_engine_tweaks')
PADD_HOST = '127.0.0.1'
PADD_PORT = 47321

SCAN_ENTER = 0x1C
SCAN_GRAVE = 0x29
VK_SCAN_FALLBACK = {0xC0: SCAN_GRAVE, 0x0D: SCAN_ENTER}
NEUTRAL = Frame(0.0, (0.0, 0.0), (0.0, 0.0), 0.0, 0.0, 0)


def import_vgamepad():
    """Import vgamepad from Tools\\pylibs (connects to ViGEmBus on import - never call in dry-run)."""
    if PYLIBS not in sys.path:
        sys.path.insert(0, PYLIBS)
    import vgamepad  # noqa: WPS433
    return vgamepad


def frame_to_msg(fr: Frame) -> dict:
    return {'op': 'frame', 'ls': [round(fr.ls[0], 5), round(fr.ls[1], 5)],
            'rs': [round(fr.rs[0], 5), round(fr.rs[1], 5)], 'lt': round(fr.lt, 5), 'rt': round(fr.rt, 5),
            'b': int(fr.buttons)}


def msg_to_frame(m: dict) -> Frame:
    def stick(v):
        x, y = float(v[0]), float(v[1])
        return max(-1.0, min(1.0, x)), max(-1.0, min(1.0, y))
    return Frame(0.0, stick(m.get('ls', (0, 0))), stick(m.get('rs', (0, 0))),
                 max(0.0, min(1.0, float(m.get('lt', 0)))), max(0.0, min(1.0, float(m.get('rt', 0)))),
                 int(m.get('b', 0)) & 0xFFFF)


def set_report(pad, fr: Frame) -> None:
    pad.reset()
    pad.left_joystick_float(float(fr.ls[0]), float(fr.ls[1]))
    pad.right_joystick_float(float(fr.rs[0]), float(fr.rs[1]))
    pad.left_trigger_float(float(fr.lt))
    pad.right_trigger_float(float(fr.rt))
    for bit in range(16):
        if fr.buttons & (1 << bit):
            pad.press_button(button=1 << bit)


class PadDriver:
    """Local virtual pad, destroyed by close() (the game then shows the disconnect dialog)."""

    persistent = False

    def __init__(self, guard: ForegroundGuard, pad=None):
        self.guard = guard
        if pad is None:
            pad = import_vgamepad().VX360Gamepad()
        self.pad = pad
        self.frames_sent = 0
        guard.on_violation = self.release

    def _push(self, fr: Frame) -> None:
        set_report(self.pad, fr)
        self.pad.update()

    def apply(self, fr: Frame) -> None:
        self.guard.check()          # raises GuardError -> caller releases and aborts
        self._push(fr)
        self.frames_sent += 1

    def release(self) -> None:
        try:
            self.pad.reset()
            self.pad.update()
        except Exception as e:  # pragma: no cover - device already gone
            print(f'[gametest] pad release failed: {e}', file=sys.stderr)

    def wake(self, mode: str = 'rs', hold: float = 0.15, settle: float = 0.6) -> None:
        """Nudge the pad once so the game switches its input/prompt mode to gamepad."""
        if mode == 'none':
            return
        fr = Frame(0.0, (0.0, 0.0), (0.0, 0.0), 0.0, 0.0, 0)
        if mode == 'rs':
            fr.rs = (0.35, 0.0)
        elif mode == 'ls':
            fr.ls = (0.0, 0.3)
        elif mode.startswith('button:'):
            fr.buttons = BUTTONS[mode.split(':', 1)[1].upper()]
        else:
            raise ValueError(f'unknown wake mode {mode!r}')
        self.apply(fr)
        self.guard.wait(hold)
        self.apply(NEUTRAL)
        self.guard.wait(settle)

    def close(self) -> None:
        self.release()
        self.guard.on_violation = None
        self.pad = None  # vgamepad unplugs the device in __del__


class PaddError(RuntimeError):
    pass


class PaddClient:
    """Line-delimited JSON over a localhost TCP socket to padd.py."""

    def __init__(self, host: str = PADD_HOST, port: int = PADD_PORT, timeout: float = 1.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.rfile = self.sock.makefile('r', encoding='utf-8', newline='\n')

    def call(self, msg: dict) -> dict:
        self.sock.sendall((json.dumps(msg) + '\n').encode('utf-8'))
        line = self.rfile.readline()
        if not line:
            raise PaddError('padd closed the connection')
        return json.loads(line)

    def close(self) -> None:
        try:
            self.rfile.close()
            self.sock.close()
        except OSError:
            pass


def padd_ping(host: str = PADD_HOST, port: int = PADD_PORT) -> dict | None:
    try:
        c = PaddClient(host, port, timeout=0.5)
    except OSError:
        return None
    try:
        return c.call({'op': 'ping'})
    except (OSError, ValueError, PaddError):
        return None
    finally:
        c.close()


class RemotePadDriver(PadDriver):
    """Pad owned by the padd daemon: close() only disconnects, the virtual pad stays plugged in (neutral).

    The guard runs here AND in the daemon before every report.
    """

    persistent = True

    def __init__(self, guard: ForegroundGuard, client: PaddClient):  # noqa: super-init not called on purpose
        self.guard = guard
        self.client = client
        self.frames_sent = 0
        guard.on_violation = self.release

    def _push(self, fr: Frame) -> None:
        msg = frame_to_msg(fr)
        background = getattr(self.guard, 'background', False)
        if background:
            msg['bg'] = True          # background run: the daemon may skip its foreground test for this report
        r = self.client.call(msg)
        if not r.get('ok'):
            if background and 'not running' not in str(r.get('error')):
                # reworded: the daemon's focus text would read as a user stop in night.py
                raise GuardError('padd refused a background pad report (the daemon requires the game in the '
                                 'foreground unless it honours "bg")')
            raise GuardError(f'padd refused input: {r.get("error")}')

    def release(self) -> None:
        try:
            self.client.call({'op': 'release'})
        except Exception as e:  # pragma: no cover
            print(f'[gametest] padd release failed: {e}', file=sys.stderr)

    def close(self) -> None:
        self.release()
        self.guard.on_violation = None
        self.client.close()


# ---------------------------------------------------------------------------------------------- UTPP log checks

def file_size(path: str | None) -> int:
    """End position of a log for read_since / wait_for_log (-1 = no file). A UTPP session log (utpp-<start>.log) is the
    rotation-safe virtual end of the session (rotated .N.log siblings included, see gt_log.log_size); any other file is
    its byte size."""
    if gt_log.is_session_log(path):
        return gt_log.log_size(path)
    return os.path.getsize(path) if path and os.path.isfile(path) else -1


def read_since(path: str | None, offset: int) -> str:
    if not path or offset < 0:
        return ''
    if gt_log.is_session_log(path):
        return gt_log.read_text_since(path, offset)
    if not os.path.isfile(path):
        return ''
    with open(path, 'rb') as f:
        f.seek(offset)
        return f.read().decode('utf-8', errors='replace')


def native_echo_name(cmd: str) -> str | None:
    """'Game.UTPPSetMMEnabled(true)' -> 'UTPPSetMMEnabled' (UTPP natives log '<Name>(...)' when called)."""
    m = re.match(r'\s*Game\.(UTPP\w+)\s*\(', cmd)
    return m.group(1) if m else None


LOGLINE_CAP = 479       # Native_LogLine copies at most sizeof(buf) - 1 = 479 chars and maps non-printable ones to '?'
_LOGLINE_RE = re.compile(r"""^\s*Game\.UTPPLogLine\s*\(\s*(?P<q>['"])(?P<body>[^\\]*?)(?P=q)\s*(?:\)|\.\.)""")


def logline_expect(cmd: str) -> str | None:
    """'Game.UTPPLogLine(<literal>)' -> the line text the native writes ('UTPP CET <text>'): the literal, or its prefix up
    to a '..' concatenation. UTPPLogLine does not log its own name (Main.cpp Native_LogLine), so the '<Name>(' echo check
    never sees it; None = not a plain literal (not verifiable)."""
    m = _LOGLINE_RE.match(cmd)
    if not m:
        return None
    body = ''.join(c if 32 <= ord(c) < 127 else '?' for c in m.group('body'))[:LOGLINE_CAP]
    return body or None


def logline_seen(text: str, expect: str) -> bool:
    return 'UTPP CET ' + expect in text


# Explicit per-command marker (2026-11-09, opt-in): a command that ENDS with Game.UTPPLogLine('[cetv:<id>] <text>'..) proves that
# the whole chunk was typed and executed (a command cut short by a lost key never reaches its last statement; a syntax error
# runs nothing). The id must be unique per call (a retry types the same command again). Conditional or optional UTPPLogLine
# calls inside a command are NOT markers: only this tag is verified, so older commands keep their behaviour.
CETV_PREFIX = 'cetv:'
_CETV_RE = re.compile(r"""Game\.UTPPLogLine\s*\(\s*(?P<q>['"])\[""" + re.escape(CETV_PREFIX) + r"""(?P<id>[A-Za-z0-9_.-]{4,64})\]""")


def cetv_marker(cmd: str) -> str | None:
    """Id of the '[cetv:<id>]' marker the command logs, or None (no marker: not verifiable this way)."""
    m = _CETV_RE.search(cmd)
    return m.group('id') if m else None


def cetv_seen(text: str, marker_id: str) -> bool:
    return 'UTPP CET [' + CETV_PREFIX + marker_id + ']' in text


def auto_marker_ok(cmd: str) -> bool:
    """True when a marker can be appended to a command that is not verifiable otherwise: it has no marker, no verified native /
    leading literal UTPPLogLine, and no '--' (a Lua comment would swallow the appended call, '--' in a string is skipped too)."""
    return bool(cmd.strip()) and '--' not in cmd and cetv_marker(cmd) is None and native_echo_name(cmd) is None \
        and logline_expect(cmd) is None


def with_marker(cmd: str, marker_id: str) -> str:
    return cmd.rstrip() + f" Game.UTPPLogLine('[{CETV_PREFIX}{marker_id}]')"


def echo_seen(text: str, name: str) -> bool:
    # natives log '[UTPP] <Name>(...)'; REDScript-side switches log through UTPPLogLine as '[UTPP] UTPP CET <Name>(...)'
    return re.search(r'\]\s+(?:UTPP CET\s+)?' + re.escape(name) + r'\(', text) is not None


def wait_for_log(path: str | None, offset: int, predicate, timeout: float, sleep=time.sleep,
                 clock=time.monotonic) -> bool:
    end = clock() + timeout
    while True:
        if predicate(read_since(path, offset)):
            return True
        if clock() >= end:
            return False
        sleep(0.2)


def has_trace_tick(text: str) -> bool:
    return 'MMTRACE' in text


def dismiss_disconnect_dialog(guard: ForegroundGuard, log_path: str | None, appear_wait: float = 1.5,
                              verify_timeout: float = 3.0, attempts: int = 2) -> dict:
    """After a local pad was destroyed: Enter (scan 0x1C) to close 'controller disconnected', verify by new trace.

    Returns {'sent': n, 'verified': True/False/None}; None = could not verify (trace off before the disconnect).
    """
    res = {'sent': 0, 'verified': None}
    guard.wait(appear_wait)
    pre = file_size(log_path)
    trace_was_on = has_trace_tick(read_since(log_path, max(0, pre - 20000))) if pre > 0 else False
    for _ in range(attempts):
        off = file_size(log_path)
        guard.check()
        guard.api.send_scan_chord([SCAN_ENTER])
        res['sent'] += 1
        if not trace_was_on:
            guard.wait(0.5)
            return res
        if wait_for_log(log_path, off, has_trace_tick, verify_timeout, sleep=guard.wait):
            res['verified'] = True
            return res
    res['verified'] = False
    return res


# ---------------------------------------------------------------------------------------------- CET overlay

def decode_cet_key(code: int) -> list[int]:
    """CET bindings.json packs up to 4 VKs as 16-bit fields, first key in the top bits (0xC0<<48 = grave)."""
    out = []
    for shift in (48, 32, 16, 0):
        vk = (int(code) >> shift) & 0xFFFF
        if vk:
            out.append(vk)
    return out


def read_cet_overlay_key(cet_dir: str = CET_DIR) -> list[int] | None:
    try:
        with open(os.path.join(cet_dir, 'bindings.json'), encoding='utf-8') as f:
            code = json.load(f)['cet']['overlay_key']
        return decode_cet_key(code) or None
    except (OSError, KeyError, ValueError, TypeError):
        return None


def read_cet_layout(cet_dir: str = CET_DIR) -> dict:
    """ImGui window rects from layout.ini: {name: (x, y, w, h, collapsed)} in game-backbuffer pixels."""
    out = {}
    path = os.path.join(cet_dir, 'layout.ini')
    if not os.path.isfile(path):
        return out
    cur = None
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            line = line.strip()
            m = re.match(r'^\[Window\]\[(.*)\]$', line)
            if m:
                cur = {'name': m.group(1)}
                out[m.group(1)] = cur
                continue
            if line.startswith('['):
                cur = None
                continue
            if cur is not None and '=' in line:
                k, v = line.split('=', 1)
                cur[k] = v
    rects = {}
    for name, d in out.items():
        try:
            x, y = (int(v) for v in d['Pos'].split(','))
            w, h = (int(v) for v in d['Size'].split(','))
        except (KeyError, ValueError):
            continue
        rects[name] = (x, y, w, h, d.get('Collapsed', '0') == '1')
    return rects


def cet_log_tail_marker(cet_dir: str = CET_DIR) -> int:
    return file_size(os.path.join(cet_dir, 'cyber_engine_tweaks.log'))


def cet_log_since(offset: int, cet_dir: str = CET_DIR) -> str:
    return read_since(os.path.join(cet_dir, 'cyber_engine_tweaks.log'), offset)


# Console input field as a fraction of the game client rect; measured in a 1536x864 window
# (client at 768,432 -> click at 1234,920 worked). layout.ini positions did not match the window scale.
DEFAULT_CONSOLE_INPUT_REL = (0.30, 0.565)


class CetOverlay:
    """Types commands into the CET console and clicks 'Reload all mods' - all guarded.

    Keys are sent as hardware scan codes (toggle = grave 0x29 by default, from bindings.json VK via MapVirtualKey).
    The console input click is relative to the game client rect (`input_rel`), or taken from layout.ini when
    `input_rel` is None (ImGui coordinates * `ui_scale`).
    """

    def __init__(self, guard: ForegroundGuard, overlay_vks: list[int] | None = None, cet_dir: str = CET_DIR,
                 ui_scale: float = 1.0, key_delay: float = 0.012, open_wait: float = 0.6, click_console: bool = True,
                 reload_offset_px: float | None = None, toggle_scans: list[int] | None = None,
                 input_rel: tuple[float, float] | None = DEFAULT_CONSOLE_INPUT_REL,
                 reload_rel: tuple[float, float] | None = None, enter_pause_min: float = 0.3,
                 enter_pause_per_char: float = 0.001, enter_pause_max: float = 2.0, auto_marker: bool = False):
        self.guard = guard
        self.auto_marker = auto_marker    # run_commands appends a [cetv:<id>] marker to commands that are not verifiable otherwise
        self.api = guard.api
        self.cet_dir = cet_dir
        self.overlay_vks = overlay_vks or read_cet_overlay_key(cet_dir) or [0xC0]
        self.toggle_scans = toggle_scans or [self._scan(vk) for vk in self.overlay_vks]
        self.ui_scale = ui_scale
        self.key_delay = key_delay
        self.open_wait = open_wait
        self.click_console = click_console
        self.reload_offset_px = reload_offset_px
        self.input_rel = input_rel
        self.reload_rel = reload_rel
        # pause between the last typed character and Enter: the ImGui input buffer is filled once per game frame, so Enter right
        # after the last key can overtake the tail (2026-11-09: a 583-char command arrived without its last 7 characters)
        self.enter_pause_min = enter_pause_min
        self.enter_pause_per_char = enter_pause_per_char
        self.enter_pause_max = enter_pause_max
        self.last_attempts: dict = {}     # command -> number of times run_commands typed it (last call)
        self.is_open = False

    def _scan(self, vk: int) -> int:
        fn = getattr(self.api, 'vk_to_scan', None)
        s = fn(vk) if fn else 0
        return s or VK_SCAN_FALLBACK.get(vk, 0)

    def toggle(self) -> None:
        self.guard.check()
        self.api.send_scan_chord(self.toggle_scans)
        self.is_open = not self.is_open
        self.guard.wait(self.open_wait)

    def _client_point(self, x: float, y: float) -> tuple[int, int]:
        left, top, right, bottom = self.api.client_rect_screen(self.guard.game_hwnd)
        sx, sy = left + x * self.ui_scale, top + y * self.ui_scale
        return self._check_inside(sx, sy, (left, top, right, bottom))

    @staticmethod
    def _check_inside(sx, sy, rect) -> tuple[int, int]:
        left, top, right, bottom = rect
        if not (left <= sx < right and top <= sy < bottom):
            raise GuardError(f'click point ({sx:.0f},{sy:.0f}) outside the game client area')
        return int(round(sx)), int(round(sy))

    def _click_screen(self, sx: int, sy: int) -> None:
        self.guard.check()
        root = self.api.root_window_at(sx, sy)
        if root and root != self.guard.game_hwnd and self.api.window_pid(root) not in self.guard.game_pids:
            raise GuardError('window under the click point is not the game')
        self.api.set_cursor(sx, sy)
        self.guard.check()
        self.api.send_left_click()
        self.guard.wait(0.15)

    def click(self, x: float, y: float) -> None:
        """Click at ImGui/layout coordinates (scaled by ui_scale)."""
        self._click_screen(*self._client_point(x, y))

    def click_rel(self, fx: float, fy: float) -> None:
        """Click at a fraction of the game client rect."""
        rect = self.api.client_rect_screen(self.guard.game_hwnd)
        left, top, right, bottom = rect
        self._click_screen(*self._check_inside(left + fx * (right - left), top + fy * (bottom - top), rect))

    def console_input_point(self) -> tuple[float, float] | None:
        """layout.ini based point (only used when input_rel is None)."""
        r = read_cet_layout(self.cet_dir).get('Console')
        if not r or r[4]:
            return None
        x, y, w, h, _ = r
        return x + w * 0.5, y + h - 18

    def console_click_screen(self) -> tuple[int, int] | None:
        """Screen point the console click would use (dry-run report); None if unknown."""
        rect = self.api.client_rect_screen(self.guard.game_hwnd) if self.guard.game_hwnd else None
        if rect is None:
            return None
        left, top, right, bottom = rect
        if self.input_rel is not None:
            return int(round(left + self.input_rel[0] * (right - left))), \
                int(round(top + self.input_rel[1] * (bottom - top)))
        pt = self.console_input_point()
        return None if pt is None else (int(left + pt[0] * self.ui_scale), int(top + pt[1] * self.ui_scale))

    def reload_button_point(self) -> tuple[float, float] | None:
        """'Reload all mods' is the last (rightmost) equal-width button of the CET toolbar window."""
        r = read_cet_layout(self.cet_dir).get('Cyber Engine Tweaks')
        if not r:
            return None
        x, y, w, h, _ = r
        off = self.reload_offset_px if self.reload_offset_px is not None else w / 16.0
        return x + w - off, y + h * 0.5

    def focus_input(self) -> None:
        if not self.click_console:
            return
        if self.input_rel is not None:
            self.click_rel(*self.input_rel)
            return
        pt = self.console_input_point()
        if pt:
            self.click(*pt)

    def enter_pause(self, cmd: str) -> float:
        """Seconds between the last character and Enter: proportional to the command length, within [min, max]."""
        return min(self.enter_pause_max, max(self.enter_pause_min, self.enter_pause_per_char * len(cmd)))

    def type_command(self, cmd: str) -> None:
        self.focus_input()
        for ch in cmd:
            self.guard.check()
            self.api.send_unicode(ch)
            time.sleep(self.key_delay)
        self.guard.wait(self.enter_pause(cmd))      # guarded: focus loss aborts before Enter
        self.guard.check()
        self.api.send_scan_chord([SCAN_ENTER])
        self.guard.wait(0.25)

    def _close_quietly(self) -> None:
        if self.is_open:
            try:
                self.toggle()
            except GuardError:
                pass  # focus lost: leave the overlay open rather than typing into another app

    def run_commands(self, cmds: list[str], log_path: str | None = None, verify_timeout: float = 3.0,
                     retries: int = 1) -> dict:
        """Type commands; with log_path verify UTPP natives by their '<Name>(' echo in the UTPP log.

        Verifiable: UTPP natives ('<Name>(' echo), a leading Game.UTPPLogLine('<literal>') and any command carrying a
        Game.UTPPLogLine('[cetv:<id>] ...') marker (see cetv_marker). Returns {cmd: True (seen) | False (not seen after
        retries) | None (not verifiable)}. Unverified commands are retyped up to `retries` times (console reopened);
        self.last_attempts[cmd] = how many times each command was typed. With self.auto_marker every command that is not
        verifiable otherwise gets a unique [cetv:<id>] marker appended (what is typed is then cmd + marker, the result and
        last_attempts stay keyed by the original cmd).
        """
        result: dict = {c: None for c in cmds}
        self.last_attempts = {c: 0 for c in cmds}
        wire = {c: (with_marker(c, uuid.uuid4().hex[:10]) if self.auto_marker and log_path and auto_marker_ok(c) else c)
                for c in cmds}
        pending = list(cmds)
        for attempt in range(retries + 1):
            if not pending:
                break
            off = file_size(log_path)
            self.toggle()
            try:
                for c in pending:
                    self.last_attempts[c] += 1
                    self.type_command(wire[c])
            finally:
                self._close_quietly()
            if not log_path:
                break
            names = {c: native_echo_name(c) for c in pending}
            for c, n in names.items():
                mid = cetv_marker(wire[c])
                if mid is not None:        # explicit marker: the last statement of the chunk ran
                    result[c] = wait_for_log(log_path, off, lambda t, i=mid: cetv_seen(t, i), verify_timeout,
                                             sleep=self.guard.wait)
                    continue
                if n is None:
                    continue
                if n == 'UTPPLogLine':     # no '<Name>(' echo: the log carries 'UTPP CET <message>'
                    exp = logline_expect(c)
                    if exp is None:
                        continue           # message is not a plain literal: not verifiable
                    result[c] = wait_for_log(log_path, off, lambda t, e=exp: logline_seen(t, e), verify_timeout,
                                             sleep=self.guard.wait)
                    continue
                result[c] = wait_for_log(log_path, off, lambda t, n=n: echo_seen(t, n), verify_timeout,
                                         sleep=self.guard.wait)
            pending = [c for c in pending if result[c] is False]
        return result

    def reload_all_mods(self, timeout: float = 8.0) -> bool:
        """Click 'Reload all mods' and wait for 'Reloaded all mods' in cyber_engine_tweaks.log."""
        pt = None if self.reload_rel else self.reload_button_point()
        if pt is None and not self.reload_rel:
            raise GuardError('CET toolbar position not found in layout.ini (use --cet-reload-rel)')
        marker = cet_log_tail_marker(self.cet_dir)
        self.toggle()
        ok = False
        try:
            if self.reload_rel:
                self.click_rel(*self.reload_rel)
            else:
                self.click(*pt)
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if 'Reloaded all mods' in cet_log_since(marker, self.cet_dir):
                    ok = True
                    break
                self.guard.wait(0.25)
        finally:
            self._close_quietly()
        return ok


# ---------------------------------------------------------------------------------------------- background mode

class NeedsKeyboard(RuntimeError):
    """Keyboard / mouse input refused in background mode (not a GuardError: callers skip the action instead of aborting)."""


KEYBOARD_METHODS = frozenset({'send_scan_chord', 'send_key_chord', 'send_unicode', '_send', '_scan', 'set_cursor',
                              'send_left_click', 'focus'})


class KeyboardRefusingApi:
    """Win32 api wrapper for background runs: keyboard, mouse and focus calls raise NeedsKeyboard, every query
    (process list, windows, key_down_now for the user-abort keys) passes through."""

    def __init__(self, api):
        self._api = api

    def __getattr__(self, name):
        if name in KEYBOARD_METHODS:
            def refused(*_a, **_k):
                raise NeedsKeyboard(f'needs keyboard: {name} is refused with --background')
            return refused
        return getattr(self._api, name)


class BackgroundGuard(ForegroundGuard):
    """Guard for runs with the game window in the background: the game process must be alive and the Pause key
    still stops the run (ESC is not an abort key here: it is used in other applications while the run goes on), but
    the foreground window is not checked and the game is never focused. Keyboard / mouse input
    raises NeedsKeyboard (see KeyboardRefusingApi); pad reports pass."""

    background = True

    def __init__(self, api=None, **kw):
        if api is None:
            from gt_win32 import Win32Api
            api = Win32Api()
        kw.setdefault('abort_keys', (VK_PAUSE,))
        super().__init__(api=KeyboardRefusingApi(api), **kw)

    def check(self) -> None:
        self.checks += 1
        now = self.clock()
        if now - self._last_proc_check >= self.process_check_interval:
            self._last_proc_check = now
            alive = [p for p in self.game_pids if self.api.process_alive(p)]
            if not alive:
                raise GuardError(f'{self.exe_name} is not running')
            self.game_pids = alive
        for vk in self.abort_keys:
            if self.api.key_down_now(vk):
                raise GuardError(f'user abort key 0x{vk:02X} pressed')

    def ensure_focus(self, attempts: int = 3, wait: float = 0.4) -> None:
        if not self.refresh_game():
            raise GuardError(f'{self.exe_name} is not running')
        self._last_proc_check = -1e9
        self.check()


# ---------------------------------------------------------------------------------------------- file command channel

# The test-only CET module utpp_testcmd runs "<seq> <lua chunk>" lines of its cmd.txt and logs
# "[testcmd] ACK seq=<n> ok=<true|false> <result>" into the UTPP log. The harness writes nothing else into the game folder.
TESTCMD_DIR = os.path.join(CET_DIR, 'mods', 'utpp_testcmd')
TESTCMD_FILE = os.path.join(TESTCMD_DIR, 'cmd.txt')
TESTCMD_SUFFIX = os.path.join('mods', 'utpp_testcmd', 'cmd.txt')
_TESTCMD_ACK_RE = re.compile(r'\[testcmd\] ACK seq=(\d+) ok=(true|false)(?: ([^\r\n]*))?')
_TESTCMD_READY_RE = re.compile(r'\[testcmd\] READY seq=(\d+)(?: loader=(\S+))?')


def testcmd_path_ok(path: str) -> bool:
    """True only for a path that ends with mods\\utpp_testcmd\\cmd.txt after normalisation ('..' cannot escape)."""
    if not path:
        return False
    p = os.path.normcase(os.path.normpath(os.path.abspath(path)))
    return p.endswith(os.sep + os.path.normcase(TESTCMD_SUFFIX))


def testcmd_acks(text: str) -> dict:
    """{seq: (ok, result)} of every ACK line in the text."""
    return {int(m.group(1)): (m.group(2) == 'true', (m.group(3) or '').strip()) for m in _TESTCMD_ACK_RE.finditer(text)}


def testcmd_ready(text: str) -> tuple[int, str] | None:
    """(last seq, loader) of the newest READY line, None = the module did not load in this session."""
    last = None
    for m in _TESTCMD_READY_RE.finditer(text):
        last = (int(m.group(1)), m.group(2) or 'unknown')
    return last


def parse_testcmd_lines(text: str) -> list[tuple[int, str]]:
    out = []
    for line in text.splitlines():
        m = re.match(r'^\s*(\d+)\s+(.*\S)\s*$', line)
        if m:
            out.append((int(m.group(1)), m.group(2)))
    return out


class FileCommandChannel:
    """Sends Lua commands through cmd.txt of the utpp_testcmd CET module and waits for their ACK in the UTPP log.

    Each command gets a new sequence number (ms clock, always above every number in the file and the module state); the
    file keeps the last `keep_lines` commands (the module runs only numbers above its last one, so a short history is
    safe) and is replaced atomically (temp file + os.replace, retried while the module has the file open). Without an
    ACK within `ack_timeout` the command is rewritten once under a new number (the unanswered line is removed); an ACK
    of either number counts. run_commands returns {cmd: True (ACK ok) | False (ACK ok=false or no ACK) | None (no log)}
    like CetOverlay.run_commands.
    """

    name = 'file'

    def __init__(self, cmd_path: str = TESTCMD_FILE, ack_timeout: float = 3.0, retries: int = 1, keep_lines: int = 8,
                 sleep=time.sleep, clock=time.monotonic, now_ms=None, replace=os.replace, replace_tries: int = 40):
        if not testcmd_path_ok(cmd_path):
            raise ValueError(f'command file must end with {TESTCMD_SUFFIX}: {cmd_path}')
        self.path = cmd_path
        self.ack_timeout = ack_timeout
        self.retries = retries
        self.keep_lines = max(1, keep_lines)
        self.sleep = sleep
        self.clock = clock
        self.now_ms = now_ms or (lambda: int(time.time() * 1000))
        self.replace = replace
        self.replace_tries = replace_tries
        self.last_seq = 0
        self.last_post_wall = None          # wall time right before the last os.replace (marker timing)
        self.last_attempts: dict = {}

    def installed(self) -> bool:
        return os.path.isdir(os.path.dirname(self.path))

    def read_entries(self) -> list[tuple[int, str]]:
        try:
            with open(self.path, encoding='utf-8', errors='replace') as f:
                return parse_testcmd_lines(f.read())
        except OSError:
            return []

    def state_seq(self) -> int:
        try:
            with open(os.path.join(os.path.dirname(self.path), 'state.txt'), encoding='utf-8') as f:
                m = re.search(r'\d+', f.read())
            return int(m.group(0)) if m else 0
        except OSError:
            return 0

    def next_seq(self, entries: list[tuple[int, str]]) -> int:
        top = max([self.last_seq, self.state_seq()] + [s for s, _ in entries])
        return max(self.now_ms(), top + 1)

    def _write(self, entries: list[tuple[int, str]]) -> None:
        if not testcmd_path_ok(self.path):
            raise ValueError(f'command file must end with {TESTCMD_SUFFIX}: {self.path}')
        tmp = self.path + '.tmp'
        with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
            f.write(''.join(f'{s} {c}\n' for s, c in entries))
        self.last_post_wall = datetime.now()
        for k in range(self.replace_tries):
            try:
                self.replace(tmp, self.path)
                return
            except PermissionError:          # the module is reading the file at this moment
                if k == self.replace_tries - 1:
                    raise
                self.sleep(0.025)

    def post(self, cmd: str, drop_seq: int | None = None) -> int:
        """Append one command (new sequence number); `drop_seq` removes an unanswered earlier line. Returns the number."""
        if not cmd or not cmd.strip():
            raise ValueError('empty command')
        if '\n' in cmd or '\r' in cmd:
            raise ValueError('a command must be one line')
        if not self.installed():
            raise GuardError(f'utpp_testcmd is not installed ({os.path.dirname(self.path)} missing)')
        entries = [e for e in self.read_entries() if e[0] != drop_seq]
        seq = self.next_seq(entries)
        kept = entries[len(entries) - (self.keep_lines - 1):] if self.keep_lines > 1 else []
        self._write(kept + [(seq, cmd.strip())])
        self.last_seq = seq
        return seq

    def send(self, cmd: str, log_path: str | None, timeout: float | None = None) -> dict:
        """Post `cmd` and wait for its ACK. {'seq', 'seqs', 'ok': True/False/None, 'result', 'attempts', 'sent_at'}."""
        timeout = self.ack_timeout if timeout is None else timeout
        off = file_size(log_path)
        seqs: list[int] = []
        res = {'ok': None, 'result': '', 'attempts': 0, 'seqs': seqs, 'seq': None, 'sent_at': None}
        for _ in range(self.retries + 1):
            seq = self.post(cmd, drop_seq=seqs[-1] if seqs else None)
            seqs.append(seq)
            res.update(seq=seq, attempts=len(seqs), sent_at=self.last_post_wall)
            if not log_path:
                return res
            found: dict = {}

            def acked(text, found=found):
                acks = testcmd_acks(text)
                hit = next((s for s in seqs if s in acks), None)
                if hit is not None:
                    found['ack'] = (hit, acks[hit])
                return hit is not None
            if wait_for_log(log_path, off, acked, timeout, sleep=self.sleep, clock=self.clock):
                hit, (ok, text) = found['ack']
                res.update(ok=ok, result=text, seq=hit)
                return res
        res['ok'] = False
        res['result'] = f'no ACK within {timeout:.1f} s ({len(seqs)} writes)'
        return res

    def run_commands(self, cmds: list[str], log_path: str | None = None, verify_timeout: float | None = None,
                     retries: int | None = None) -> dict:
        if retries is not None:
            self.retries = retries
        result: dict = {}
        self.last_attempts = {}
        for c in cmds:
            r = self.send(c, log_path, verify_timeout)
            self.last_attempts[c] = r['attempts']
            result[c] = r['ok']
        return result

    def clear(self) -> bool:
        """Empty the command file (end of a run); False when the module folder is missing."""
        if not self.installed():
            return False
        self._write([])
        return True
