"""Unattended overnight runner: quit -> (install) -> launch -> arm MM -> record -> scenarios -> quit -> collect.

  python Source\\tools\\gametest\\night.py --dry-run [--set loco,sprint_dash,weapon_draw_aim_holster,vehicle_mount_exit]   (also: --set aim_probe and --set melee_fists_target, not in 'all')
  python Source\\tools\\gametest\\night.py --dry-run --set melee_weapons   (melee_knife, melee_katana, melee_blunt, melee_baton, melee_bat, melee_hammer, melee_mantis; see gt_melee_weapons.py; not in 'all')
  python Source\\tools\\gametest\\night.py --go --cycles 3 --set all --db utpp_base
         [--install 0.5.8 "note" --cet-mods utpp_core,utpp_body] [--cet "Game.UTPPSetMMTurnCommit(true)"]

Without --go nothing with side effects on the game is done (plan only); --dry-run runs read-only prerequisite checks
(ffmpeg devices/encoders on a null output, pad driver, game window + capture rect, scripts, CET console point, power).

One cycle (Research\\logs\\night\\<cycle-ts>\\cycle.json records every step with wall-clock times):
 1 ensure_closed  game running -> focus it, type gt_night.QUIT_CMD into the CET console (inkISystemRequestsHandler
                  .ExitGame via GetSingleton('inkMenuScenario'); never saves), wait --quit-timeout s, then
                  `taskkill /IM`, after --kill-grace s `taskkill /F /IM`; the path used is logged.
 2 install        optional: install_cet_mods.py <mods> (game closed; rc != 0 aborts). Release content is installed
                  with Source\tools\install\install_verified_stage.ps1, not from here.
 3 launch         --launch steam (Start-Process steam://rungameid/1091500) | exe (bin\\x64\\Cyberpunk2077.exe, cwd
                  bin\\x64); wait for the process, a new utpp-*.log and a gameplay line; --launch-timeout s, one retry.
 4 arm            focus, Game.UTPPSetMMEnabled(true), Game.UTPPSetMMTrace(true), Game.UTPPLoadDatabase("<db>")
                  (echo-verified), --cet extras, wait 'speed calibration [<db>]'; if MMTRACE does not flow (game paused
                  by the "controller disconnected" dialog) -> Enter via run.dismiss() and recheck.
 5 record         ffmpeg gdigrab desktop cropped to the game client rect (physical px, re-read at record start;
                  fallback ddagrab + crop=), h264_nvenc (fallback libx264), fragmented mp4 rec_<YYYYmmdd-HHMMSS.fff>.mp4
                  + rec_<ts>.json; marker Game.UTPPLogLine("[night] UTPP_MARK ...") at start and end.
                  --capture window records the game window's own picture (gt_capture.py: Windows.Graphics.Capture
                  through an ffmpeg 8.x build, at most 1920x1080) and works while other windows cover the game;
                  --background selects it automatically unless --capture is given explicitly. If it cannot start the
                  run falls back to the desktop modes with a warning (the video then shows what covers the game).
 6 scenarios      run.py machinery (run_scenario / padd pad / CET overlay). Macros here: weapon_draw_aim_holster (draw
                  = Y tap, aim = LT, holster = Y double tap, each verified by NightPos w= or utpp_perspective lines)
                  melee_fists (fists by script RequestEquipFists, RT tap, punches standing/walking, LT block, UnequipWeapon
                  then Y double tap if needed;
                  verified by the UTPP MMMASK/MMGATE melee lines and MM weight w >= 0.9 while masked),
                  melee_fists_target (--set melee_fists_target only: the fists against a REAL opponent - a Codeware NPC
                  spawned at each phase distance by console commands, Immortal; far / near / camera-orbit / block phases; evidence =
                  MELEEAIM src=lock, V <-> target distance, target health before/after; see enemy_macro),
                  aim_probe (--set aim_probe only: perspective.aiming=tpp + utpp_probe AimProbe(true) by script, LT hold
                  4 s with 2 RT shots, AimProbe(false) + aiming=fpp restored in a finally; PASS = >= 4 aimprobe samples,
                  >= 1 shot, a summary line; the parsed result is stored as 'aimprobe' in cycle.json / summary.json),
                  and vehicle_mount_exit (Codeware spawn next to V, run to the door re-aimed from NightPos prints,
                  X / F to mount, drive, B hold to exit). After every scenario: weapon-wheel / menu recovery
                  (no movement in MMTRACE although the stick moved -> --wheel-close tap), also before the vehicle
                  macro (probe walk). Per-scenario timeout (--scenario-timeout) -> TIMEOUT verdict, the night goes on.
 7 quit           as step 1 (before the heavy collect so the game does not idle).
 8 collect        copy UTPP/red4ext/CET/utpp_* logs, harness analysis (run.analyze_dir), video sheets (whole @0.5 s,
                  per scenario start/end +-3 s @0.1 s) also from a killed recording, summary.json (night metrics).

Recording safety and measurement (gt_collect.py, gt_settings.py, gt_posemeter.py, gt_load.py):
 - the pose recording segments (utpp-pose-ring-*.utpppose or the single utpp-pose-*.utpppose), the input dump (utpp-feed-*.txt)
   and the mod log are copied from the game's log folder into <run>\\logs between the scenarios (finished ring segments), when the
   scenario phase ends or fails (stop file, abort key, exception, timeout), again before the cycle ends and once more after
   the game quit (the writer flushes its last block on close); the files and sizes are listed in cycle.json ("recordings").
 - --diag keep|off|timed|until: the runner sets the mod's diagnostic logs option at arm (previous values read first) and puts
   it back before every ExitGame; every other setting a macro switches (aim probe, fists) is restored in a finally and listed in
   cycle.json ("settings"). A change that could not be restored (game gone) is kept in Research\\logs\\night\\pending_restore.json and
   restored first thing by the NEXT launch.
 - after the quit a worker process runs the pose meter (pose_metrics.analyze per scenario / step window and 'auto'); result:
   pose_metrics.json, pose_table.txt, per-scenario "pose ..." lines in the log and summary.json. --pose-baseline <run folder>
   prints before / after per scenario; the exit code is non-zero only when a "pose_limits" block of scenarios.json is exceeded.
 - machine load over the scenario phase (CPU mean, compiler processes) next to the perf line.

Speed (gt_flow.py, gt_post.py, gt_budget.py):
 - look-ahead probe: skipped when the previous probe started within 1 m of here, along the same direction (camera and route within
   15 deg), reached far enough and came back clear; the 1.4 s "face" push stays. Forgotten after a camera turn, a blocked attempt,
   the vehicle scenario and at every cycle start. One log line names the reason.
 - the recorder is ended (q, --recorder-grace, then terminate) BEFORE the game quits; the check of the file (length, size, brightness,
   clock) is quick and the full decode runs in a background thread after the quit. ExitGame is given --quit-timeout (60 s) after its
   echo before any taskkill.
 - contact sheets (the "full" sheet cut into one-sheet parts) and failure clips run in a pool of processes sized to the idle cores
   (--post-workers, at most 24, below-normal priority); at most --clips-per-cause (6) clips of each failure cause; --fast makes
   neither and lists them in summary.json ("pending_media": names, video seconds, the commands that make them later:
   gt_post.py sheets <run>, failure_clips.py <run> --per-cause 0 [--name <text>] [--list]).
 - early stop (--early-stop N, 2): the first N walking scenarios invalid for one cause (V cannot move although the stick is
   pushed, no "UTPP MOTION" lines; the checks run.py applies afterwards) end the cycle with that cause in one line; the rest is
   listed as not played, the game is still quit and everything collected.
 - --idle-others: other build / test / tool processes run at idle priority while the cycle runs (gt_load.OthersIdler), so the
   mod's frame budget is not exceeded only because the machine is busy with other work.
 - at the end of a cycle a time budget table (per scenario: total, play, probe, retries, waits, copy, probes skipped; per phase:
   launch, arm, scenarios, recorder stop, quit, collect and its post-processing parts) goes to the log, time_budget.txt and
   summary.json.

Session modes:
 - --body female|male|both: that body's save is made the newest (select_save.py, times only, backup first) with the game closed and
   put back exactly after the run on every exit path; a game that is already running is never closed for it (the run refuses); a
   leftover selection of an earlier runner session is restored first at every start; "both" = one launch per body and one
   body_summary_<time>.txt with the two runs side by side. --pose-baseline is used only against the body it was recorded on.
 - --graphics low: the lowest quality profile before the launch (game closed); never restored automatically
   (graphics_profile.py --restore).
 - --background: needs nothing in the foreground, no keyboard and no user idle time (--min-idle defaults to 0, --cmd-channel to auto);
   window recorder, file command channel, pad plugin switched on after arming and off at the end of the cycle; only Pause stops the
   run. --dry-run --background states which requirements are met (test command module, pad plugin, window recorder).

One launch, these sets, these extra commands, diagnostics until, everything restored, fast post-processing (python = the
interpreter with numpy and psutil, run from the project root):
   (i)   keyboard-like and standard movement sets on the Female body with the pose meter and a baseline comparison:
         night.py --go --set kbd,loco --body female --diag until --cet "<commands>" --cet-final "<commands that undo them>"
                  --pose-baseline <run folder of an earlier Female run> --fast
   (ii)  fists, enemy and the four melee weapon sets on both bodies:
         night.py --go --set melee_fists,melee_fists_target,melee_weapons --body both --diag until --fast
   (iii) the background input probe (pad-only walks while the game window is behind others):
         night.py --go --background --set bg_probe --diag until --fast

Stopping: create Research\\logs\\night\\STOP (checked between scenarios/cycles), or send CTRL_BREAK / SIGTERM: the
cycle finishes cleanly (ffmpeg 'q', pad neutral, game quit per --on-stop, collect). The STOP file is renamed to
STOP.consumed-<ts>; a STOP file present at start refuses to run.

Safety: every key/click/pad report goes through ForegroundGuard (game foreground, alive). A focus loss or the user's
ESC/Pause stops the whole night WITHOUT quitting the game (somebody is at the PC). Before each cycle the user idle
time (GetLastInputInfo) must be >= --min-idle unless the last input was our own. Never run installers while the game
runs. The system is kept awake (SetThreadExecutionState) only while --go runs; power settings are not changed.

Commands: --cmd-channel console (default) types into the CET overlay; file writes cmd.txt of the test-only CET module
utpp_testcmd and waits for its "[testcmd] ACK seq=<n>" line in the UTPP log (no overlay, no keys, no clicks; the only
file the runner writes in the game folder besides the CET overlay state fix of the console mode); auto picks file when
the session log has the module's READY line. --background (file / auto only): the game may stay behind other windows;
no focusing, no focus-loss abort, keyboard / mouse input refused (scenarios that need keys are skipped as "needs
keyboard"), pad reports still sent, ESC/Pause still stop the run. The game reads the pad from behind other windows
only through the test-only host plugin UTPPTestHost: --background switches it on after arming
(Game.UTPPTestBackgroundInput(true), the cycle fails when the native is missing or refuses) and off again at the end of
the cycle and at every stop.
"""
from __future__ import annotations

import argparse
import ctypes
import glob
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_capture  # noqa: E402
import gt_log  # noqa: E402
import gt_melee_weapons as gw  # noqa: E402
import gt_night as gn  # noqa: E402
import run as gt_run  # noqa: E402
import failure_clips  # noqa: E402
import gt_budget as gb  # noqa: E402
import gt_collect as gc  # noqa: E402
import gt_flow  # noqa: E402
import gt_load  # noqa: E402
import gt_post  # noqa: E402
import gt_posemeter as gp  # noqa: E402
import gt_session  # noqa: E402
import select_save  # noqa: E402
import gt_settings as gs  # noqa: E402
import video_sheets  # noqa: E402
from gt_input import CET_DIR, SCAN_ENTER, TESTCMD_FILE, BackgroundGuard, FileCommandChannel, NeedsKeyboard, PaddError, file_size, \
    padd_ping, read_cet_overlay_key, read_since, testcmd_ready, wait_for_log  # noqa: E402
from gt_scenario import BUTTONS, Scenario, load_scenarios, parse_scenario  # noqa: E402
from gt_win32 import GAME_EXE, IS_WINDOWS, ForegroundGuard, GuardError, TimerResolution  # noqa: E402

PROJECT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
NIGHT_ROOT = os.path.join(PROJECT, 'Research', 'logs', 'night')
# mount: X where V arrived (camera along the car), one walk along the car and X again, then the scripted mount
MOUNT_SEEK_STICK = 0.45             # walking stick of the search along the car
DOOR_RADIUS_M = 0.45                # the approach ends this close to the door spot (X worked at lon -0.17, not at 0.34)
MOUNT_MAX_DIST_M = 4.0              # further from the car centre than this: no X (the approach did not arrive)
APPROACH_SIDESTEPS = 2              # steps aside around an obstacle on the way to the car
MOUNT_SPOTS_FILE = os.path.join(NIGHT_ROOT, 'mount_spots.json')   # car-frame spot where X last worked, per V body
MOUNT_PULSE_MAX_S = 2.0
MOUNT_PULSE_MPS = 1.0              # V's walk speed at stick 0.45 (scaled with the stick)
APPROACH_STUCK_M = 0.1
APPROACH_STUCK_CHUNKS = 2
# a summoned car that drove in but parked beyond --arrival-dist is used after this long still, up to this far
FAR_PARK_HOLD_S = 10.0
FAR_PARK_MAX_M = 120.0
# clear-path probe before a walking scenario (night.clear_path): stick forward this long; less movement = blocked
CLEAR_PROBE_S = 0.6
CLEAR_PROBE_STICK = 0.6
CLEAR_MIN_MOVE_M = 0.3
sys.path.insert(0, os.path.join(HERE, '..'))
from utpp_paths import game_dir  # noqa: E402

GAME_DIR = game_dir(required=False) or ''
GAME_EXE_PATH = os.path.join(GAME_DIR, 'bin', 'x64', GAME_EXE)
INSTALL_CET = os.path.join(PROJECT, 'Source', 'tools', 'install', 'install_cet_mods.py')
STEAM_URL = 'steam://rungameid/1091500'
QUIT_TIMEOUT_S = 60.0      # --quit-timeout default: a game that echoed ExitGame is given this long before the first kill
RECORDER_GRACE_S = 8.0     # --recorder-grace default: seconds a recorder gets after 'q' before it is ended (fragmented mp4 stays decodable)
BG_INPUT_ON = 'Game.UTPPTestBackgroundInput(true)'     # --background: test host plugin UTPPTestHost (test installs only)
BG_INPUT_OFF = 'Game.UTPPTestBackgroundInput(false)'
POST_HELPER_ON = 'Game.UTPPSetPostHelperStream(true)'   # --diag timed|until: the recording also gets the helper bones after the commit
POST_HELPER_ECHO = 'UTPPSetPostHelperStream(1)'
TS = '%Y-%m-%d %H:%M:%S.%f'


class StopNight(Exception):
    """Somebody is at the PC (focus lost / abort key / recent user input): stop everything, leave the game alone."""


class UserActive(StopNight):
    """Recent user input before a cycle: stop without touching the game."""
    leave_game = True


class CycleError(Exception):
    """The cycle failed; the night continues with the next cycle after a quit."""


class GracefulStop(CycleError):
    """STOP file / signal / --max-wall-hours: finish the cycle cleanly (recording, game quit per --on-stop, collect)."""


class EarlyStop(CycleError):
    """The first walking scenarios are invalid for one cause: the rest of the cycle is not played (the game is still quit,
    the files collected and the settings restored as after any failed cycle)."""


class ScenarioTimeout(Exception):
    """A scenario exceeded --scenario-timeout; the cycle continues with the next scenario."""


class _NightIo:
    """The command sender of the settings ledger: one echo-verified command through the runner's channel."""

    def __init__(self, night, pad, guard):
        self.night, self.pad, self.guard = night, pad, guard

    def run(self, cmd: str, tag: str, label: str):
        return self.night.console_verified(self.pad, self.guard, cmd, lambda t: f'[night] {tag} ok=' in t, label)


class _PlainGuard:
    """No focus rules: the file command channel needs none (used when the run stopped or the game is left running)."""

    def wait(self, seconds: float) -> None:
        time.sleep(seconds)

    def ensure_focus(self) -> None:
        return None

    def check(self) -> None:
        return None


def stamp(dt: datetime | None) -> str | None:
    return dt.strftime(TS)[:-3] if dt else None


def file_ts(dt: datetime) -> str:
    return dt.strftime('%Y%m%d-%H%M%S.') + f'{dt.microsecond // 1000:03d}'


def is_user_stop(msg: str) -> bool:
    return 'focus lost' in msg or 'not the game' in msg or 'abort key' in msg or 'Ctrl+C' in msg


def stop_file(out_root: str) -> str:
    return os.path.join(out_root, 'STOP')


# ---------------------------------------------------------------------------------------------- Win32 extras

class _LASTINPUTINFO(ctypes.Structure):
    _fields_ = [('cbSize', ctypes.c_uint), ('dwTime', ctypes.c_uint)]


def idle_seconds() -> float | None:
    """Seconds since the last user input of the session (GetLastInputInfo); synthetic SendInput counts too."""
    if not IS_WINDOWS:
        return None
    li = _LASTINPUTINFO()
    li.cbSize = ctypes.sizeof(li)
    if not ctypes.windll.user32.GetLastInputInfo(ctypes.byref(li)):
        return None
    return ((ctypes.windll.kernel32.GetTickCount() - li.dwTime) & 0xFFFFFFFF) / 1000.0


def keep_awake(on: bool) -> None:
    if IS_WINDOWS:  # ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED ; off = ES_CONTINUOUS only
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | (0x3 if on else 0))


def process_path(pid: int) -> str:
    """Full image path of a process ('' if not accessible)."""
    if not IS_WINDOWS or not pid:
        return ''
    from ctypes import wintypes
    k = ctypes.windll.kernel32
    k.OpenProcess.restype = wintypes.HANDLE
    h = k.OpenProcess(0x1000, False, pid)
    if not h:
        return ''
    try:
        size = wintypes.DWORD(1024)
        buf = ctypes.create_unicode_buffer(1024)
        return buf.value if k.QueryFullProcessImageNameW(h, 0, buf, ctypes.byref(size)) else ''
    finally:
        k.CloseHandle(h)


class _MONITORINFO(ctypes.Structure):
    from ctypes import wintypes as _w
    _fields_ = [('cbSize', _w.DWORD), ('rcMonitor', _w.RECT), ('rcWork', _w.RECT), ('dwFlags', _w.DWORD)]


def physical_rects(hwnd: int):
    """(client rect, monitor rect) in physical pixels via a per-monitor-DPI-aware thread context (ffmpeg uses
    physical pixels; this process is not DPI aware, so plain GetClientRect values may be scaled)."""
    from ctypes import wintypes
    u = ctypes.windll.user32
    set_ctx = getattr(u, 'SetThreadDpiAwarenessContext', None)
    old = None
    if set_ctx:
        set_ctx.restype = ctypes.c_void_p
        set_ctx.argtypes = [ctypes.c_void_p]
        old = set_ctx(ctypes.c_void_p(-4))  # DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_MOTION
    try:
        r = wintypes.RECT()
        u.GetClientRect(wintypes.HWND(hwnd), ctypes.byref(r))
        p = wintypes.POINT(0, 0)
        u.ClientToScreen(wintypes.HWND(hwnd), ctypes.byref(p))
        client = (p.x, p.y, p.x + r.right, p.y + r.bottom)
        u.MonitorFromWindow.restype = wintypes.HANDLE
        mon = u.MonitorFromWindow(wintypes.HWND(hwnd), 2)  # MONITOR_DEFAULTTONEAREST
        mi = _MONITORINFO()
        mi.cbSize = ctypes.sizeof(mi)
        screen = None
        if mon and u.GetMonitorInfoW(mon, ctypes.byref(mi)):
            m = mi.rcMonitor
            screen = (m.left, m.top, m.right, m.bottom)
        return client, screen
    finally:
        if set_ctx and old:
            set_ctx(ctypes.c_void_p(old))


class _BITMAPINFOHEADER(ctypes.Structure):
    _fields_ = [('biSize', ctypes.c_uint32), ('biWidth', ctypes.c_int32), ('biHeight', ctypes.c_int32),
                ('biPlanes', ctypes.c_uint16), ('biBitCount', ctypes.c_uint16), ('biCompression', ctypes.c_uint32),
                ('biSizeImage', ctypes.c_uint32), ('biXPelsPerMeter', ctypes.c_int32),
                ('biYPelsPerMeter', ctypes.c_int32), ('biClrUsed', ctypes.c_uint32), ('biClrImportant', ctypes.c_uint32)]


def grab_gray(rect, out_w: int = 160, out_h: int = 90) -> tuple[bytes, int, int]:
    """Read-only screen grab of `rect` (physical px), downscaled with HALFTONE StretchBlt, as 8-bit gray (row-major).
    Same source as gdigrab desktop (screen DC), which records the game correctly; ~10-30 ms."""
    u, g = ctypes.windll.user32, ctypes.windll.gdi32
    vp = ctypes.c_void_p
    u.GetDC.restype = vp
    u.GetDC.argtypes = [vp]
    u.ReleaseDC.argtypes = [vp, vp]
    g.CreateCompatibleDC.restype = vp
    g.CreateCompatibleDC.argtypes = [vp]
    g.CreateCompatibleBitmap.restype = vp
    g.CreateCompatibleBitmap.argtypes = [vp, ctypes.c_int, ctypes.c_int]
    g.SelectObject.restype = vp
    g.SelectObject.argtypes = [vp, vp]
    g.SetStretchBltMode.argtypes = [vp, ctypes.c_int]
    g.StretchBlt.argtypes = [vp, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, vp, ctypes.c_int,
                             ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint32]
    g.GetDIBits.argtypes = [vp, vp, ctypes.c_uint, ctypes.c_uint, vp, vp, ctypes.c_uint]
    g.DeleteObject.argtypes = [vp]
    g.DeleteDC.argtypes = [vp]
    set_ctx = getattr(u, 'SetThreadDpiAwarenessContext', None)
    old_ctx = None
    if set_ctx:
        set_ctx.restype = vp
        set_ctx.argtypes = [vp]
        old_ctx = set_ctx(vp(-4))
    hdc = mdc = bmp = old = None
    try:
        x, y, r, b = (int(v) for v in rect)
        hdc = u.GetDC(None)
        mdc = g.CreateCompatibleDC(hdc)
        bmp = g.CreateCompatibleBitmap(hdc, out_w, out_h)
        old = g.SelectObject(mdc, bmp)
        g.SetStretchBltMode(mdc, 4)  # HALFTONE
        if not g.StretchBlt(mdc, 0, 0, out_w, out_h, hdc, x, y, r - x, b - y, 0x00CC0020):  # SRCCOPY
            raise OSError('StretchBlt failed')
        bmi = _BITMAPINFOHEADER(ctypes.sizeof(_BITMAPINFOHEADER), out_w, -out_h, 1, 32, 0, 0, 0, 0, 0, 0)
        buf = ctypes.create_string_buffer(out_w * out_h * 4)
        if g.GetDIBits(mdc, bmp, 0, out_h, buf, ctypes.byref(bmi), 0) != out_h:
            raise OSError('GetDIBits failed')
        raw = buf.raw
        gray = bytes((raw[i + 2] * 299 + raw[i + 1] * 587 + raw[i] * 114) // 1000 for i in range(0, len(raw), 4))
        return gray, out_w, out_h
    finally:
        if old:
            g.SelectObject(mdc, old)
        if bmp:
            g.DeleteObject(bmp)
        if mdc:
            g.DeleteDC(mdc)
        if hdc:
            u.ReleaseDC(None, hdc)
        if set_ctx and old_ctx:
            set_ctx(vp(old_ctx))


# ---------------------------------------------------------------------------------------------- runner

def newest_motion_status(log: str | None, tail_bytes: int = 8 << 20) -> str | None:
    """Newest motion status text (from 'motion enabled=' on, carries 'feedP95us=') in the UTPP log, None = none
    logged. Only the last `tail_bytes` of the log are read."""
    if not log or not os.path.isfile(log):
        return None
    try:
        size = os.path.getsize(log)
        with open(log, 'rb') as f:
            f.seek(max(0, size - tail_bytes))
            text = f.read().decode('utf-8', errors='replace')
    except OSError:
        return None
    for ln in reversed(text.splitlines()):
        if 'feedP95us=' in ln:
            i = ln.find('motion enabled=')
            j = ln.find('[night] PERF ')
            if j >= 0:
                return ln[j + len('[night] PERF '):].strip()
            return (ln[i:] if i >= 0 else ln).strip()
    return None


class Night:
    def __init__(self, args, runner=subprocess.run):
        self.args = args
        self.runner = runner
        self.scenarios = load_scenarios(args.scenario_file)
        self.names = gn.resolve_sets(args.set)
        self.run_args = gt_run.build_parser().parse_args(
            ['--keep-pad', '--cet-input-rel', args.cet_input_rel, '--scenario-file', args.scenario_file,
             '--thresholds', args.thresholds, '--cet-no-auto-verify']       # night has its own marker / verify logic
            + (['--cet-key-scan', args.cet_key_scan] if args.cet_key_scan else []))
        self.api = None
        self.cycle = None
        self.cycle_dir = None
        self.last_own_input = None
        self.utpp_log = None
        self.deadline = time.monotonic() + args.max_wall_hours * 3600.0
        self.signal_reason = None
        self.stop_reason = None
        self.scenario_deadline = None
        self.pos_mode = None       # 'nightpos' (utpp_core 0.5.2) | 'inline' (long Lua line) once known
        self.pos_stats = {'ok': 0, 'miss': 0}
        self.cet_stats: dict = {}              # label -> [attempt that worked (0 = LOST), ...]
        self.speeds = dict(gn.SPEEDS)          # dead-reckoning speeds, learned from real POS displacement
        self.cam_rate = gn.CAMERA_RATE_DEG_S   # right-stick yaw rate, learned from cf feedback
        self.cam_sign = 1.0
        self.last_known = None
        self.blocked_end = 0.0                 # car end (-1 / +1) where V stood still; plan_path avoids it
        self.auto_pos = False                  # utpp_core 0.5.4 NightAuto POS lines in the log
        self.overlay_grab = self.grab_game     # (guard) -> (gray, w, h) | None; tests replace it
        self.overlay_stats: dict = {}
        self.channel_mode = None               # resolved --cmd-channel: 'console' | 'file' (auto: from the READY line)
        self.testcmd_path = TESTCMD_FILE       # tests point it at a scratch folder
        self.bg_input_active = False           # --background: UTPPTestHost background pad input switched on
        self.settings = gs.Ledger(os.path.join(args.out_root, 'pending_restore.json'), log=self.log)
        self.collector = None                  # gt_collect.SessionCollector of the running cycle
        self.load_sampler = None               # gt_load.LoadSampler over the scenario phase
        self.idler = None                      # gt_load.OthersIdler from the launch to the end of the cycle (--idle-others)
        self.meter_job = None                  # gt_posemeter.MeterJob (worker processes)
        self.budget = gb.Budget()              # where the wall time of the cycle went (printed at the end)
        self.probe_cache = gt_flow.ProbeCache()   # look-ahead probes of this cycle that came back clear
        self.early = gt_flow.EarlyStop(getattr(args, 'early_stop', 2))
        self.deep_check = None                 # background thread: full decode of the recording after the quit
        self.body = None                       # --body: 'female' | 'male' | None (the game's own save)
        self.graphics_settings = None          # tests point these at temporary files
        self.graphics_state_dir = None

    # -- bookkeeping --
    def log(self, msg: str) -> None:
        line = f'[{stamp(datetime.now())}] {msg}'
        print(line, flush=True)
        if self.cycle_dir:
            with open(os.path.join(self.cycle_dir, 'night.log'), 'a', encoding='utf-8') as f:
                f.write(line + '\n')

    def save(self) -> None:
        if self.cycle_dir and self.cycle is not None:
            with open(os.path.join(self.cycle_dir, 'cycle.json'), 'w', encoding='utf-8') as f:
                json.dump(gt_run.serialise_plan(self.cycle), f, indent=1, default=str)

    def step(self, name: str) -> dict:
        st = {'name': name, 'start': datetime.now(), 'end': None, 'status': 'running'}
        self.cycle['steps'].append(st)
        self.save()
        return st

    def done(self, st: dict, status: str = 'ok', **info) -> None:
        st.update(info)
        st['end'], st['status'] = datetime.now(), status
        self.save()

    def touched(self) -> None:
        self.last_own_input = time.monotonic()

    def pids(self) -> list[int]:
        """Game PIDs; a process whose full image path is readable must be GAME_EXE_PATH."""
        out = []
        for pid in self.api.list_pids(GAME_EXE):
            path = process_path(pid)
            if not path or os.path.normcase(path) == os.path.normcase(GAME_EXE_PATH):
                out.append(pid)
        return out

    def guard(self) -> ForegroundGuard:
        # clock resolved at call time (tests replace night.time with a fake clock)
        if getattr(self.args, 'background', False):
            return BackgroundGuard(api=self.api, focus_grace=self.args.focus_grace, clock=lambda: time.monotonic())
        return ForegroundGuard(api=self.api, focus_grace=self.args.focus_grace, clock=lambda: time.monotonic())

    # -- command channel (--cmd-channel console | file | auto) --
    def use_file(self) -> bool:
        """True when commands go through the utpp_testcmd file channel. auto: file once the current UTPP session log has
        the module's READY line (with a Lua loader), else console. --background never falls back to the console."""
        mode = getattr(self.args, 'cmd_channel', 'console')
        if mode != 'auto':
            return mode == 'file'
        if self.channel_mode is None:
            log = self.utpp_log or gt_log.newest_log(self.args.log_dir)
            info = testcmd_ready(read_since(log, 0)) if log else None
            self.channel_mode = 'file' if info and info[1] != 'none' else 'console'
            self.log(f'[cmd] --cmd-channel auto -> {self.channel_mode} (READY {info})')
        if self.channel_mode == 'console' and getattr(self.args, 'background', False):
            raise CycleError('--background needs the file command channel: no "[testcmd] READY" line in the UTPP log '
                             '(utpp_testcmd not installed?)')
        return self.channel_mode == 'file'

    def file_channel(self, guard) -> FileCommandChannel:
        return FileCommandChannel(self.testcmd_path, sleep=guard.wait, clock=lambda: time.monotonic())

    def file_commands(self, guard, cmds: list[str], log: str | None = None) -> dict:
        log = log or self.utpp_log or gt_log.newest_log(self.args.log_dir)
        ch = self.file_channel(guard)
        res = ch.run_commands(cmds, log_path=log)
        for c, ok in res.items():
            if ok is False:
                self.log(f'[cmd] file channel: no ACK ok for {c[:60]!r} after {ch.last_attempts.get(c)} writes')
        return res

    def bg_input(self, on: bool, why: str = '') -> dict:
        """--background: switch the background pad input of the test-only host plugin UTPPTestHost through the file
        channel. Plain time.sleep, never the guard (the switch-off must also go through while Pause is held). Switch-on
        raises CycleError unless the plugin answers "UTPPTestHost on"; switch-off only logs a failure."""
        res = {'ok': None, 'result': ''}
        if not on and not self.bg_input_active:
            return res
        log = self.utpp_log or gt_log.newest_log(self.args.log_dir)
        try:
            if not self.pids():
                raise GuardError('game is not running')
            ch = FileCommandChannel(self.testcmd_path, sleep=time.sleep, clock=time.monotonic)
            res = ch.send(BG_INPUT_ON if on else BG_INPUT_OFF, log_path=log)
        except (GuardError, OSError, ValueError) as e:
            res = {'ok': False, 'result': f'not sent: {e}'}
        text = str(res.get('result') or '')
        self.log(f'[bg-input] switch {"on" if on else "off"}{f" ({why})" if why else ""}: ok={res.get("ok")} {text[:240]}')
        if on:
            if not (res.get('ok') and text.startswith('UTPPTestHost on')):
                raise CycleError('--background: the game does not read the pad from behind other windows: '
                                 f'Game.UTPPTestBackgroundInput(true) -> {text or "no answer"} (install the test host plugin: '
                                 'build target UTPPTestHost, tools/install/stage_test_probes.py, install_verified_stage.ps1 '
                                 '-StageName test-probes)')
            self.bg_input_active = True
        else:
            self.bg_input_active = False
        return {'ok': res.get('ok'), 'result': text[:300]}

    def check_user_idle(self) -> None:
        idle = idle_seconds()
        need = gt_session.effective_min_idle(self.args)
        if idle is None or need <= 0:
            return
        own = None if self.last_own_input is None else time.monotonic() - self.last_own_input
        if idle < need and (own is None or own > idle + 5.0):
            raise UserActive(f'user input {idle:.0f} s ago (< --min-idle {need:.0f} s): somebody is '
                            'using the PC')

    def requested_stop(self) -> str | None:
        if self.stop_reason:
            return self.stop_reason
        sf = stop_file(self.args.out_root)
        if os.path.exists(sf):
            consumed = sf + '.consumed-' + datetime.now().strftime('%Y%m%d-%H%M%S')
            try:
                os.replace(sf, consumed)
            except OSError:
                pass
            self.stop_reason = f'STOP file ({consumed})'
        elif self.signal_reason:
            self.stop_reason = self.signal_reason
        elif time.monotonic() >= self.deadline:
            self.stop_reason = f'--max-wall-hours {self.args.max_wall_hours} reached'
        return self.stop_reason

    def check_continue(self) -> None:
        why = self.requested_stop()
        if why:
            raise GracefulStop(why)

    def check_scenario_time(self) -> None:
        if self.scenario_deadline is not None and time.monotonic() > self.scenario_deadline:
            raise ScenarioTimeout(f'scenario exceeded --scenario-timeout {self.args.scenario_timeout:.0f} s')

    # -- CET overlay state (cycle 8: the toggle got out of sync; an open overlay swallows game input) --
    def grab_game(self, guard):
        """(gray, w, h) of the game client area or None (no window / capture failed)."""
        try:
            hwnd = guard.game_hwnd or (guard.refresh_game() and guard.game_hwnd)
            if not hwnd:
                return None
            rect, screen = physical_rects(hwnd)
            return grab_gray(gn.clamp_rect(rect, screen))
        except Exception as e:  # noqa: BLE001
            self.log(f'[overlay] grab failed: {e!r}')
            return None

    def ensure_overlay(self, guard, expected: str = 'closed', where: str = '') -> str | None:
        """Verify the CET overlay state from one captured frame; if it differs, press the overlay key and re-check
        (max --overlay-tries). A toggle whose effect goes the wrong way (classifier false positive) is undone.
        Returns the final classified state (None = cannot tell, nothing done)."""
        if not self.args.overlay_check or self.use_file():   # file channel: the overlay is never opened or toggled
            return None
        shot = self.overlay_grab(guard)
        if shot is None:
            return None
        state, st = gn.classify_overlay(*shot)
        self.overlay_stats[state or 'unknown'] = self.overlay_stats.get(state or 'unknown', 0) + 1
        if state is None or state == expected:
            return state
        for k in range(1, self.args.overlay_tries + 1):
            ov = self.overlay(guard)
            ov.toggle()
            self.touched()
            shot2 = self.overlay_grab(guard)
            if shot2 is None:
                self.log(f'[overlay] {where}: expected {expected}, found {state} -> toggled ({k}); re-check impossible')
                return None
            state2, st2 = gn.classify_overlay(*shot2)
            eff = gn.overlay_toggle_effect(st, st2, expected)
            self.log(f'[overlay] {where}: expected {expected}, found {state} -> toggled ({k}): now {state2} '
                     f'(panel {st["panel_mean"]} -> {st2["panel_mean"]}, {eff})')
            self.overlay_stats['toggled'] = self.overlay_stats.get('toggled', 0) + 1
            if eff == 'fixed' or state2 == expected:
                return expected
            if eff == 'wrong':  # we changed the state the other way: undo and stop
                self.overlay(guard).toggle()
                self.log(f'[overlay] {where}: toggle went the wrong way -> undone (classifier unsure here)')
                self.overlay_stats['undone'] = self.overlay_stats.get('undone', 0) + 1
                return None
            state, st = state2 or state, st2
        self.log(f'[overlay] {where}: still {state} after {self.args.overlay_tries} toggles')
        return state

    # -- console helpers --
    def overlay(self, guard, fast: bool = False):
        ov = gt_run.make_overlay(self.run_args, guard)
        if fast:
            ov.open_wait = self.args.console_open_wait
        return ov

    def type_console(self, guard, cmds: list[str], verify_log: str | None = None, fast: bool = False) -> dict:
        """Open the console, type, close - with the overlay state verified before, while open and after closing.
        File channel: the commands are written to cmd.txt and confirmed by their ACK (no overlay, no keys)."""
        if self.use_file():
            return self.file_commands(guard, cmds, verify_log)
        try:
            self.ensure_overlay(guard, 'closed', 'before console')
            if verify_log:
                res = self.overlay(guard, fast).run_commands(cmds, log_path=verify_log)
            else:
                ov = self.overlay(guard, fast)
                ov.toggle()
                self.ensure_overlay(guard, 'open', 'console open')
                try:
                    for c in cmds:
                        ov.type_command(c)
                finally:
                    ov.toggle()
                res = {c: None for c in cmds}
            self.ensure_overlay(guard, 'closed', 'after console')
            return res
        finally:
            self.touched()

    # -- settings the run changes (gt_settings.Ledger) --
    def settings_io(self, pad, guard) -> _NightIo:
        return _NightIo(self, pad, guard)

    def note_cycle(self, key: str, value) -> None:
        if self.cycle is not None:
            self.cycle[key] = value
            self.save()

    def restore_settings(self, pad, guard, res: dict | None = None, why: str = 'before ExitGame') -> dict | None:
        """Puts back every setting the run changed (open ledger entries). A failure is logged, kept in the cycle record and
        never blocks the quit."""
        if not self.settings.open_entries():
            return None
        try:
            out = self.settings.restore_all(self.settings_io(pad, guard), why)
        except Exception as e:  # noqa: BLE001
            out = {'why': why, 'restored': 0, 'failed': [x['key'] for x in self.settings.open_entries()], 'error': repr(e)}
            self.log(f'[settings] restore failed: {e!r}')
        if res is not None:
            res['restored_settings'] = out
        self.note_cycle('settings_restore', out)
        return out

    def restore_settings_leave(self, why: str) -> None:
        """The run ends and the game keeps running (or the run died): restore through the file channel when there is one (it
        needs no focus and no key), else keep the changes in the pending file for the next launch."""
        if not self.settings.open_entries():
            return
        try:
            if self.use_file():
                self.restore_settings(None, _PlainGuard(), None, why)
        except Exception as e:  # noqa: BLE001
            self.log(f'[settings] restore without the console failed: {e!r}')
        if self.settings.open_entries():
            self.settings_left(why)

    def settings_left(self, why: str) -> None:
        left = self.settings.game_gone(why)
        if left:
            ents = self.settings.left()
            diag = [e for e in ents if e['key'] == gs.DIAG_KEY]
            self.note_cycle('settings_left', {'keys': left, 'why': why,
                                              'notes': [gs.left_text(e) for e in ents if not e['volatile']],
                                              'diagnostic_logs_left_on': any(e['requested'] in ('timed', 'until') for e in diag),
                                              'note': '; '.join(gs.left_text(e) for e in ents if not e['volatile']) or 'restore needed',
                                              'pending_file': self.settings.pending_path})

    def restore_pending_settings(self, guard) -> None:
        """First thing after the launch: put back what an earlier run left changed (its pending_restore.json)."""
        try:
            rec = self.settings.restore_pending(self.settings_io(None, guard))
        except Exception as e:  # noqa: BLE001
            rec = {'error': repr(e)}
            self.log(f'[settings] pending restore failed: {e!r}')
        if rec:
            self.note_cycle('pending_settings_restore', rec)

    def apply_diag(self, guard, info: dict) -> None:
        mode = getattr(self.args, 'diag', 'keep')
        if mode == 'keep':
            return
        if mode in ('timed', 'until'):
            info['post_helper_stream'] = self.request_post_helper_stream(guard)
        ent = self.settings.apply(self.settings_io(None, guard), gs.DIAG_KEY, gs.DIAG_VALUES[mode], '--diag', deps=gs.DIAG_DEPS)
        info['diag'] = {k: ent[k] for k in ('requested', 'state', 'previous', 'note')}
        if ent['state'] != 'applied':
            self.log(f'[arm] WARNING: --diag {mode} not applied ({ent["note"]}): the run has no diagnostic recording')

    def request_post_helper_stream(self, guard) -> bool | None:
        """The helper reports compare the recorded pose with the helper bones the engine holds after the mod's commit. Only a
        recording that starts after this request has that stream, so it is sent before the diagnostic logs are switched on. It
        lasts for the game session. True = granted, False = refused (the helper recompute does not write), None = no answer."""
        txt = self.console_verified(None, guard, POST_HELPER_ON, lambda t: POST_HELPER_ECHO in t, 'post_helper_stream')
        got = None if txt is None else (POST_HELPER_ECHO + ' -> 1') in txt
        if not got:
            self.log('[arm] WARNING: the recording will have no post-helper stream ('
                     + ('no answer' if got is None else 'refused: the helper recompute does not write')
                     + '): the helper residual report cannot check the engine pose')
        return got

    # -- recording safety (gt_collect) --
    def secure_session_files(self, phase: str, final: bool = False) -> dict | None:
        """Copies the pose recording segments, the input dump and the mod log of this session into <run>\\logs. Idempotent and
        cheap when nothing changed; never raises (a collection problem must not stop the run). Between scenarios only the
        finished ring segments are copied, `final` copies everything."""
        try:
            if self.collector is None:
                if not self.utpp_log or not self.cycle_dir:
                    return None
                since = gc.session_since(self.utpp_log, self.cycle['start'])
                self.collector = gc.SessionCollector(
                    self.args.log_dir, os.path.join(self.cycle_dir, 'logs'), since,
                    extra_files=lambda: gt_log.session_files(self.utpp_log) if self.utpp_log else [], log=self.log)
            out = self.collector.collect(phase, final=final)
            if phase == 'collect' and out['busy']:
                self.log(f'[collect] WARNING: files still held open after the quit, not copied: {out["busy"]}')
            if final or out['new'] or out['refreshed'] or out['lost'] or out['errors']:
                self.note_cycle('recordings', self.collector.summary())
            return out
        except Exception as e:  # noqa: BLE001
            self.log(f'[collect] {phase}: copying the session files failed: {e!r}')
            return None

    # -- machine load --
    def start_load(self) -> None:
        try:
            self.load_sampler = gt_load.LoadSampler().start()
        except Exception as e:  # noqa: BLE001
            self.load_sampler = None
            self.log(f'[load] sampler not started: {e!r}')

    def stop_load(self) -> None:
        if self.load_sampler is None:
            return
        try:
            self.note_cycle('machine_load', self.load_sampler.stop())
        except Exception as e:  # noqa: BLE001
            self.log(f'[load] sampler failed: {e!r}')
        self.load_sampler = None

    def report_load(self) -> None:
        res = (self.cycle or {}).get('machine_load')
        if res is not None:
            self.log('[load] ' + gt_load.format_load(res))

    def start_idler(self) -> None:
        """--idle-others: other build / test / tool processes run at idle priority from the launch to the end of the cycle."""
        if not getattr(self.args, 'idle_others', False):
            return
        try:
            self.idler = gt_load.OthersIdler(lambda: [os.getpid()] + list(self.pids())).start()
        except Exception as e:  # noqa: BLE001
            self.idler = None
            self.log(f'[load] --idle-others not started: {e!r}')

    def stop_idler(self) -> None:
        idler, self.idler = getattr(self, 'idler', None), None
        if idler is None:
            return
        try:
            res = idler.stop()
            self.note_cycle('idle_others', res)
            self.log('[load] --idle-others: ' + (f'{res["processes_set_idle"]} other build / test processes set to idle priority'
                                                 if res['available'] else 'psutil is not available, nothing changed'))
        except Exception as e:  # noqa: BLE001
            self.log(f'[load] --idle-others failed: {e!r}')

    # -- 1/7: quit --
    def ensure_closed(self, label: str) -> dict:
        st = self.step(f'{label}')
        res = {'path': 'not_running', 'cet_sent': False}
        pids = self.pids()
        if not pids:
            self.done(st, **res)
            return res
        if not self.api.find_main_window(pids):
            # 2026-10-28: after the user closes the game the process can stay alive without a window; there is
            # nothing to focus or type into -> force-kill at once instead of refocus attempts and a soft taskkill
            self.log(f'[quit] game process {pids} has no window (left over after closing) -> taskkill /F')
            self.runner(['taskkill', '/F', '/IM', GAME_EXE], capture_output=True, text=True)
            res['windowless'] = True
            return self._quit_wait(st, res, time.monotonic(), True, True)
        pad = None
        for attempt in range(1, self.args.quit_focus_tries + 1):
            try:
                pad = self._quit_console(label, res)
                break
            except CycleError as e:      # no command channel (--background without the file module): taskkill below
                res['cet_error'] = str(e)
                self.log(f'[quit] {e} -> taskkill')
                break
            except GuardError as e:
                # cycle 23: once ExitGame is echoed the game window disappears before the process ends -> the focus
                # guard fires although the quit worked; treat the echo as success instead of refocusing/taskkill
                if '[night] QUIT exitgame' in (self.log_tail() or ''):
                    res['cet_sent'] = True
                    self.log('[quit] ExitGame echo seen - the game is closing (focus lost is expected)')
                    break
                # cycle 11: another application took the foreground during quit -> refocus the game and retry, never stop
                res['cet_error'] = str(e)
                self.log(f'[quit] {e} -> refocus the game ({attempt}/{self.args.quit_focus_tries})')
                if attempt < self.args.quit_focus_tries:
                    time.sleep(self.args.quit_focus_gap)
            finally:
                self.touched()
        t0, killed, forced = time.monotonic(), False, False
        return self._quit_wait(st, res, t0, killed, forced)

    def _quit_console(self, label: str, res: dict):
        """Focus, leave the vehicle if mounted, verified ExitGame. Raises GuardError on focus problems."""
        pad = None
        try:
            g = self.guard()
            g.ensure_focus()
            if gn.mounted_state(None, self.log_tail()):
                try:  # the padd daemon keeps the virtual pad plugged until the game is closed: reuse it for B holds
                    pad = gt_run.open_pad(self.run_args, g)[0] if padd_ping(port=self.run_args.padd_port) else None
                except Exception as e:  # noqa: BLE001
                    self.log(f'[quit] no pad for the exit ({e!r}); keyboard only')
            # ExitGame from the console did not close a mounted game (cycle 4): leave the vehicle first
            try:
                if self.ensure_unmounted(pad, g, {'runs': []} if pad else None, f'before {label}'):
                    res['exited_vehicle'] = True
                    still = gn.mounted_state(None, self.log_tail())
                    if still:
                        res['still_mounted'] = True
            except NeedsKeyboard as e:   # --background: the exit needs keys -> ExitGame anyway (taskkill follows if needed)
                res['exit_skipped'] = str(e)
                self.log(f'[quit] vehicle exit skipped: {e}')
            self.send_final_commands(pad, g, res)
            if not res.get('still_mounted'):
                quit_echo = self.console_verified(pad, g, gn.QUIT_CMD, lambda t: '[night] QUIT exitgame' in t, 'quit',
                                                  timeout=2.0)
                res['cet_sent'] = quit_echo is not None
                if quit_echo is None:
                    self.log('[quit] ExitGame command LOST -> taskkill')
            else:
                self.log('[quit] V still mounted -> no ExitGame, straight to taskkill')
        finally:
            if pad is not None:
                pad.close()
        return None

    def final_commands(self) -> list[str]:
        """--cet-final / --cet-final-file: commands that undo what the session switched on (";;"-separated)."""
        cet = getattr(self.args, 'cet_final', '') or ''
        path = getattr(self.args, 'cet_final_file', '') or ''
        if path:
            with open(path, encoding='utf-8') as f:
                cet = (cet + ';;' if cet else '') + f.read().strip()
        return [c.strip() for c in cet.split(';;') if c.strip()]

    def send_final_commands(self, pad, guard, res: dict) -> None:
        """Sent before every ExitGame, so the game never closes with a test setting left on. A failure here is logged
        and never blocks the quit. The ledger's settings go back first (--diag, aim probe, fists)."""
        self.restore_settings(pad, guard, res)
        try:
            cmds = self.final_commands()
            if not cmds:
                return
            log = self.utpp_log or gt_log.newest_log(self.args.log_dir)
            off = file_size(log)
            self.console(pad, guard, cmds)
            guard.wait(0.5)
            res['final_commands'] = len(cmds)
            echo = [l.split('[night]', 1)[1].strip()[:120] for l in read_since(log, off).splitlines() if '[night]' in l]
            self.log(f'[quit] final commands sent: {len(cmds)}' + (f' ({"; ".join(echo)})' if echo else ''))
        except Exception as e:  # noqa: BLE001 - the quit must go on
            res['final_commands_error'] = repr(e)
            self.log(f'[quit] final commands NOT sent: {e!r}')

    def _close_crash_reporter(self, res: dict) -> None:
        """After the game ends (ExitGame crashes the game on quit at times), close the
        CD Projekt crash-reporter dialogs without sending and note it in the cycle record."""
        try:
            out = self.runner(['tasklist', '/FI', 'IMAGENAME eq CrashReporter.exe', '/FO', 'CSV', '/NH'],
                              capture_output=True, text=True)
            n = sum(1 for ln in (out.stdout or '').splitlines() if 'CrashReporter.exe' in ln)
        except Exception:
            n = 0
        if n:
            self.runner(['taskkill', '/F', '/IM', 'CrashReporter.exe'], capture_output=True, text=True)
            res['crash_reporter_closed'] = n
            self.log(f'[quit] crash reporter dialog(s) closed without sending: {n} (game crashed on exit?)')

    def _quit_wait(self, st: dict, res: dict, t0: float, killed: bool, forced: bool) -> dict:
        while True:
            act = gn.quit_action(bool(self.pids()), time.monotonic() - t0, res['cet_sent'], killed,
                                 self.args.quit_timeout, self.args.kill_grace)
            if act == 'done':
                res['path'] = 'taskkill_force' if forced else ('taskkill' if killed else 'cet_exitgame')
                self._close_crash_reporter(res)
                if self.settings.open_entries():
                    self.settings_left(f'the game ended ({res["path"]}) before the restore could be sent')
                    res['settings_left'] = [e['key'] for e in self.settings.left()]
                if getattr(self.args, 'cmd_channel', 'console') != 'console':
                    try:                 # no command line is left for the next session (the module skips them anyway)
                        FileCommandChannel(self.testcmd_path).clear()
                    except (OSError, ValueError) as e:
                        self.log(f'[quit] cmd.txt not cleared: {e!r}')
                break
            if act == 'taskkill':
                self.runner(['taskkill', '/IM', GAME_EXE], capture_output=True, text=True)
                killed = True
                self.log('[quit] CET ExitGame did not close the game -> taskkill /IM')
            elif act == 'taskkill_force' and not forced:
                self.runner(['taskkill', '/F', '/IM', GAME_EXE], capture_output=True, text=True)
                forced = True
                self.log('[quit] taskkill /IM did not close the game -> taskkill /F /IM')
            elif act == 'taskkill_force' and time.monotonic() - t0 > self.args.quit_timeout + self.args.kill_grace + 30:
                self.done(st, 'failed', **res)
                raise CycleError('game process survives taskkill /F')
            time.sleep(1.0)
        res['seconds'] = round(time.monotonic() - t0, 1)
        self.log(f'[quit] closed via {res["path"]} in {res["seconds"]} s')
        time.sleep(3.0)  # let Steam/UTPP release files
        self.done(st, **res)
        return res

    # -- 2: install --
    def install(self) -> None:
        a = self.args
        if not a.cet_mods:
            return
        st = self.step('install')
        if self.pids():
            self.done(st, 'failed', error='game is running')
            raise CycleError('install refused: game is running')
        cmds = []
        if a.cet_mods:
            cmds.append([sys.executable, INSTALL_CET] + [m.strip() for m in a.cet_mods.split(',') if m.strip()])
        outs = []
        for c in cmds:
            p = self.runner(c, capture_output=True, text=True, errors='replace')
            outs.append({'cmd': c, 'rc': p.returncode, 'stdout': p.stdout[-4000:], 'stderr': p.stderr[-4000:]})
            if p.returncode != 0:
                self.done(st, 'failed', runs=outs)
                raise CycleError(f'installer failed rc={p.returncode}: {" ".join(c[1:])}')
        self.done(st, runs=outs)

    # -- 3: launch --
    def fix_cet_overlay_state(self) -> None:
        """2026-10-12: CET remembers which overlay windows are open (persistent.json 'overlay'). With Bindings /
        Settings open next to the Console the typed commands never reach the console input (all arm commands
        and the quit were 'LOST' after the user's own session). Before a launch (game closed) leave only the
        console toggled; the original file is kept next to it as persistent.json.night-bak."""
        path = os.path.join(CET_DIR, 'persistent.json')
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
            overlay = data.get('overlay') or {}
            wrong = [k for k, v in overlay.items() if k.endswith('_toggled') and k != 'console_toggled' and v]
            if not wrong and overlay.get('console_toggled', True):
                return
            shutil.copyfile(path, path + '.night-bak')
            for k in list(overlay):
                if k.endswith('_toggled'):
                    overlay[k] = (k == 'console_toggled')
            data['overlay'] = overlay
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4)
            self.log(f'[launch] CET overlay state fixed: closed {wrong or []}, console on (backup persistent.json.night-bak)')
        except Exception as e:  # noqa: BLE001
            self.log(f'[launch] CET overlay state check skipped ({e!r})')

    def launch(self) -> str:
        if not self.pids() and getattr(self.args, 'cmd_channel', 'console') != 'file' \
                and not getattr(self.args, 'background', False):       # background: no overlay is ever opened, nothing to fix
            self.fix_cet_overlay_state()
        for attempt in (1, 2):
            st = self.step(f'launch#{attempt}')
            prev = gt_log.newest_log(self.args.log_dir)
            t_launch = datetime.now()
            if self.args.launch == 'exe':
                subprocess.Popen([GAME_EXE_PATH], cwd=os.path.dirname(GAME_EXE_PATH), close_fds=True,
                                 creationflags=0x00000008 | 0x00000200)  # DETACHED_PROCESS | NEW_PROCESS_GROUP
            else:
                self.runner(['powershell', '-NoProfile', '-Command', f'Start-Process "{STEAM_URL}"'],
                            capture_output=True, text=True)
            end = time.monotonic() + self.args.launch_timeout
            info = {'process_at': None, 'log': None, 'gameplay_at': None, 'gameplay_by': None}
            while time.monotonic() < end:
                if not info['process_at'] and self.pids():
                    info['process_at'] = datetime.now()
                if info['process_at'] and not info['log']:
                    n = gt_log.newest_log(self.args.log_dir)
                    if n and n != prev and os.path.getmtime(n) >= t_launch.timestamp() - 2:
                        info['log'] = n
                if info['log']:
                    by = gn.gameplay_seen(read_since(info['log'], 0))
                    if by:
                        info['gameplay_at'], info['gameplay_by'] = datetime.now(), by
                        break
                time.sleep(1.0)
            if info['gameplay_at']:
                self.log(f'[launch] gameplay ({info["gameplay_by"]!r}) in '
                         f'{(info["gameplay_at"] - t_launch).total_seconds():.0f} s, log {info["log"]}')
                time.sleep(self.args.gameplay_settle)
                self.done(st, **info)
                self.utpp_log = info['log']
                self.channel_mode = None     # auto: resolved again from the new session log
                if getattr(self.args, 'cmd_channel', 'console') == 'auto':
                    self.use_file()
                return info['log']
            self.done(st, 'failed', **info)
            self.log(f'[launch] attempt {attempt} timed out after {self.args.launch_timeout:.0f} s: {info}')
            self.ensure_closed(f'quit_after_failed_launch#{attempt}')
        raise CycleError('game did not reach gameplay after 2 launches')

    # -- 4: arm --
    def trace_flowing(self, guard, seconds: float = 3.0) -> bool:
        off = file_size(self.utpp_log)
        return wait_for_log(self.utpp_log, off, lambda t: 'MMTRACE' in t, seconds, sleep=guard.wait, clock=time.monotonic)

    def wait_calibration(self, g, log: str, off: int, info: dict) -> bool | None:
        """Wait for 'speed calibration [<db>]' (--calib-timeout). --db none loads no database, the line never comes:
        no wait then (None = not expected; info['calibration_seen'] stays None)."""
        db = self.args.db
        if str(db).strip().lower() == 'none':
            info['calibration_seen'] = None
            self.log('[arm] --db none: no speed calibration expected, not waiting')
            return None
        cal = wait_for_log(log, max(0, off), lambda t: gn.speed_calibrated(t, db),
                           self.args.calib_timeout, sleep=g.wait, clock=time.monotonic)
        info['calibration_seen'] = cal
        if not cal:
            self.log(f'[arm] WARNING: no "speed calibration [{db}]" within {self.args.calib_timeout:.0f} s')
        return cal

    def arm(self, log: str) -> None:
        st = self.step('arm')
        g = self.guard()
        info = {}
        try:
            g.ensure_focus()
            self.restore_pending_settings(g)          # what an earlier run left changed goes back before anything else
            off = file_size(log)
            cet = self.args.cet
            if self.args.cet_file:        # long command lists (quotes and length are fragile on a command line)
                with open(self.args.cet_file, encoding='utf-8') as f:
                    cet = (cet + ';;' if cet else '') + f.read().strip()
            extra = [c.strip() for c in cet.split(';;') if c.strip()]
            unowned = gs.scan_unowned(extra + self.final_commands())
            if unowned:
                info['unowned_settings'] = unowned
                self.note_cycle('unowned_settings', unowned)
                self.log(f'[arm] WARNING: --cet / --cet-final change settings the runner does not own and does not restore: {unowned}')
            res = self.type_console(g, gn.arm_cmds(self.args.db, extra), verify_log=log)
            info['cet'] = res
            if any(v is False for v in res.values()):
                self.done(st, 'failed', **info)
                raise CycleError(f'CET arm commands not confirmed: {res}')
            if self.args.night_auto:
                txt = self.console_verified(None, g, gn.nightauto_cmd(self.args.night_auto_period),
                                            lambda t: '[night] auto on' in t, 'nightauto')
                self.auto_pos = txt is not None
                info['night_auto'] = self.auto_pos
                self.log(f'[arm] NightAuto {"on: positions from the log, no console polls" if self.auto_pos else "LOST/unavailable (utpp_core < 0.5.4?): console polls"}')
            self.wait_calibration(g, log, off, info)
            # a modal dialog ("controller disconnected") pauses the game: MMTRACE stops -> Enter + recheck
            if not self.trace_flowing(g):
                self.log('[arm] no MMTRACE for 3 s: game paused by a dialog? -> Enter (run.dismiss)')
                try:
                    info['dialog_dismiss'] = gt_run.dismiss(g, log)
                except NeedsKeyboard as e:   # --background: Enter is keyboard input
                    info['dialog_dismiss'] = {'sent': 0, 'verified': None, 'error': str(e)}
                    self.log(f'[arm] dialog dismiss skipped: {e}')
                self.touched()
                info['trace_after_dismiss'] = self.trace_flowing(g)
            if self.args.night_auto and not self.auto_pos:
                # 2026-11-11 19:01: the first NightAuto right after loading was lost; the whole run then fell back
                # to console polls (22 of 30 lost, no positions for clear_path or the car approach) -> one retry
                # once the game is running (after calibration / the dialog check)
                txt = self.console_verified(None, g, gn.nightauto_cmd(self.args.night_auto_period),
                                            lambda t: '[night] auto on' in t, 'nightauto')
                self.auto_pos = txt is not None
                info['night_auto'] = self.auto_pos
                info['night_auto_retry'] = True
                self.log(f'[arm] NightAuto retry: {"on" if self.auto_pos else "LOST again - console polls"}')
            self.apply_diag(g, info)
            if getattr(self.args, 'background', False):
                try:
                    info['bg_input'] = self.bg_input(True, 'after arming')
                except CycleError as e:
                    self.done(st, 'failed', error=str(e), **info)
                    raise
        except GuardError as e:
            self.done(st, 'failed', error=str(e), **info)
            raise (StopNight if is_user_stop(str(e)) else CycleError)(str(e))
        self.done(st, **info)

    # -- 5: record --
    def ffmpeg_ok(self, encoder: str, ffmpeg: str | None = None) -> bool:
        p = self.runner([ffmpeg or self.args.ffmpeg, '-hide_banner', '-loglevel', 'error', '-f', 'lavfi', '-i',
                         'color=size=256x256:duration=0.2', '-c:v', encoder, '-f', 'null', '-'],
                        capture_output=True, text=True, errors='replace')
        return p.returncode == 0

    def window_ffmpeg(self) -> tuple[str | None, str]:
        """(ffmpeg build with the gfxcapture filter, reason when there is none) for --capture window."""
        return gt_capture.find_ffmpeg(getattr(self.args, 'capture_ffmpeg', '') or '', runner=self.runner)

    def encoders_for(self, ffmpeg: str) -> list[str]:
        if self.args.encoder != 'auto':
            return [self.args.encoder]
        return (['h264_nvenc'] if self.ffmpeg_ok('h264_nvenc', ffmpeg) else []) + ['libx264']

    def start_recording(self) -> dict:
        st = self.step('record_start')
        g = self.guard()
        g.refresh_game()
        rect, screen = physical_rects(g.game_hwnd) if g.game_hwnd else (None, None)
        if rect is None:
            self.done(st, 'failed', error='no game window')
            raise CycleError('no game window to record')
        background = bool(getattr(self.args, 'background', False))
        modes = gt_capture.mode_order(self.args.capture, background, bool(getattr(self.args, 'capture_explicit', False)))
        wanted = modes[0]
        why_not = ''
        for mode in modes:
            window = mode == gt_capture.MODE
            ffmpeg = self.args.ffmpeg
            if window:
                ffmpeg, why = self.window_ffmpeg()
                if not ffmpeg:
                    why_not = why_not or why
                    self.log(f'[record] {mode} capture not available: {why}')
                    continue
            for enc in self.encoders_for(ffmpeg):
                start = datetime.now()
                path = os.path.join(self.cycle_dir, f'rec_{file_ts(start)}.mp4')
                if window:
                    try:
                        cmd = gt_capture.build_cmd(ffmpeg, path, g.game_hwnd, (rect[2] - rect[0], rect[3] - rect[1]),
                                                   self.args.fps, enc)
                    except ValueError as e:       # no usable client area (a minimised window)
                        why_not = why_not or str(e)
                        self.log(f'[record] {mode}/{enc} not started: {e}')
                        break
                else:
                    cmd = gn.build_ffmpeg_cmd(ffmpeg, path, mode, rect, screen, self.args.fps, enc)
                errp = path[:-4] + '.ffmpeg.log'
                errf = open(errp, 'w', encoding='utf-8', errors='replace')
                spawn = datetime.now()
                proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE if window else subprocess.DEVNULL,
                                        stderr=errf)
                reader = gt_capture.ProgressReader(proc.stdout) if window else None
                if window:      # alive and frames reach the encoder (a window that delivers nothing is no recording)
                    for _ in range(int(gt_capture.START_TIMEOUT_S / 0.1)):
                        if proc.poll() is not None or reader.delivering():
                            break
                        time.sleep(0.1)
                    ok = proc.poll() is None and reader.delivering()
                else:
                    time.sleep(3.0)
                    ok = proc.poll() is None
                if ok:
                    if window:
                        w, h = gt_capture.expected_size(cmd)
                    else:
                        x, y, w, h = gn.even_rect(gn.clamp_rect(rect, screen))
                    rec = {'path': path, 'json': path[:-4] + '.json', 'cmd': cmd, 'mode': mode, 'encoder': enc,
                           'rect': rect, 'screen': screen, 'expected_wh': (w, h), 'spawn_wall': spawn, 'proc': proc,
                           'errf': errf, 'errp': errp, 'markers': [], 'finalized': False, 'progress': reader}
                    info = {}
                    if wanted == gt_capture.MODE and not window:
                        rec['warning'] = gt_capture.fallback_warning(mode, background, why_not or 'no frames')
                        info['warning'] = rec['warning']
                        self.log('[record] !!! WARNING: ' + rec['warning'])
                    self.write_rec_json(rec)
                    self.log(f'[record] {mode}/{enc} rect {rect} screen {screen} -> {path}')
                    self.done(st, path=path, mode=mode, encoder=enc, cmd=subprocess.list2cmdline(cmd), **info)
                    return rec
                if proc.poll() is None:
                    proc.kill()
                errf.close()
                if window:
                    why_not = why_not or f'{enc}: the window delivered no frames'
                self.log(f'[record] {mode}/{enc} failed (see {errp})')
        self.done(st, 'failed')
        raise CycleError('ffmpeg could not start in any capture mode')

    def write_rec_json(self, rec: dict) -> None:
        keep = {k: v for k, v in rec.items() if k not in ('proc', 'errf', 'progress')}
        with open(rec['json'], 'w', encoding='utf-8') as f:
            json.dump(gt_run.serialise_plan(keep), f, indent=1, default=str)

    def marker(self, rec: dict, tag: str) -> None:
        """Type the marker commands; the wall time is taken right before Enter of the UTPPLogLine command."""
        g = self.guard()
        try:
            g.ensure_focus()
            settles = [float(x) for x in self.args.console_settles.split(',') if x.strip()]
            sent, logged, tries = None, None, 0
            if self.use_file():          # file channel: wall time right before the marker line is written to cmd.txt
                settles = []
                off = file_size(self.utpp_log)
                ch = self.file_channel(g)
                cmds = gn.mark_cmds(tag)
                ch.send(cmds[0], self.utpp_log)
                r = ch.send(cmds[1], self.utpp_log)
                tries = r['attempts']
                if wait_for_log(self.utpp_log, off, lambda t: f'UTPP_MARK {tag}' in t, 3.0, sleep=g.wait,
                                clock=time.monotonic):
                    logged = gn.find_line_ts(read_since(self.utpp_log, off), f'UTPP_MARK {tag}')
                    sent = r['sent_at']
            for settle in settles:
                tries += 1
                g.wait(settle)
                self.ensure_overlay(g, 'closed', 'before marker')
                ov = self.overlay(g)
                off = file_size(self.utpp_log)
                ov.toggle()
                try:
                    for i, c in enumerate(gn.mark_cmds(tag)):
                        ov.focus_input()
                        for ch in c:
                            g.check()
                            g.api.send_unicode(ch)
                            time.sleep(ov.key_delay)
                        g.check()
                        if i == 1:
                            sent = datetime.now()
                        g.api.send_scan_chord([SCAN_ENTER])
                        g.wait(0.25)
                finally:
                    ov._close_quietly()
                    self.touched()
                self.ensure_overlay(g, 'closed', 'after marker')
                if wait_for_log(self.utpp_log, off, lambda t: f'UTPP_MARK {tag}' in t, 3.0, sleep=g.wait,
                                clock=time.monotonic):
                    logged = gn.find_line_ts(read_since(self.utpp_log, off), f'UTPP_MARK {tag}')
                    break
            self.log(f'[cet] marker ' + (f'ok after {tries} tries' if logged else f'LOST after {tries} tries'))
            rec['markers'].append({'tag': tag, 'sent': sent if logged else None, 'log': logged, 'tries': tries})
            self.log(f'[record] marker {tag}: sent {stamp(sent)} logged {stamp(logged)}')
        except GuardError as e:
            rec['markers'].append({'tag': tag, 'error': str(e)})
            if is_user_stop(str(e)):
                raise StopNight(str(e))
        self.write_rec_json(rec)

    def stop_recording(self, rec: dict, check: bool = True) -> dict:
        """Ends the recorder: 'q' on its stdin, --recorder-grace s to finish, then it is ended (a fragmented mp4 stays decodable).
        check=True (default) also runs the recording check (finalize_recording); the cycle passes check=False before the game
        quit and checks the file after it, so the game does not wait for a decode."""
        if rec.get('finalized'):
            return rec
        if not rec.get('stopped'):
            self.stop_recorder_process(rec)
        if check:
            self.check_recording(rec)
        return rec

    def stop_recorder_process(self, rec: dict) -> None:
        st = self.step('record_stop')
        proc = rec['proc']
        rec['stop_wall'] = datetime.now()
        t0 = time.monotonic()
        if proc.poll() is not None:
            rec['stop'] = f'ffmpeg had exited (rc {proc.returncode})'
        else:
            grace = float(getattr(self.args, 'recorder_grace', RECORDER_GRACE_S))
            try:
                proc.stdin.write(b'q')
                proc.stdin.flush()
                proc.wait(timeout=grace)
                rec['stop'] = 'q'
            except Exception:  # noqa: BLE001
                # 'q' was not taken within the grace: end the process (on Windows terminate() already is the hard kill)
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except Exception:  # noqa: BLE001
                    proc.kill()
                rec['stop'] = 'terminate'
        rec['stop_seconds'] = round(time.monotonic() - t0, 1)
        try:
            rec['errf'].close()
        except Exception:  # noqa: BLE001
            pass
        rec['stopped'] = True
        self.log(f'[record] recorder ended ({rec["stop"]}) {rec["stop_seconds"]} s after the stop request')
        self.done(st, 'ok', stop=rec['stop'], stop_seconds=rec['stop_seconds'])

    def check_recording(self, rec: dict, deep: bool = True) -> dict:
        """The recording check (length, frame size, brightness, clock; deep=True also the full decode). The cycle runs it
        with deep=False after the game quit and decodes the whole file in the background (start_deep_check)."""
        st = self.step('record_check')
        info = self.finalize_recording(rec, deep=deep)
        status = 'ok' if not info['problems'] else 'failed'
        if info['problems']:
            self.log('[record] !!! RECORDING PROBLEM: ' + '; '.join(info['problems']))
        self.log(f'[record] checked ({rec.get("stop")}), duration {rec.get("duration")} ({rec.get("duration_source")}), '
                 f'decode_ok {info.get("decode_ok")}, clock {rec.get("clock", {}).get("start_method")}')
        self.done(st, status, **info)
        return info

    def finalize_recording(self, rec: dict, deep: bool = True) -> dict:
        info: dict = {'stop': rec.get('stop'), 'problems': []}
        path = rec['path']
        rec['finalized'] = True
        if not os.path.isfile(path) or os.path.getsize(path) == 0:
            info['problems'].append('recording file missing/empty')
            self.write_rec_json(rec)
            return info
        stderr = open(rec['errp'], encoding='utf-8', errors='replace').read() if os.path.isfile(rec['errp']) else ''
        rec['input_start'] = gn.parse_input_start(stderr)
        reader = rec.get('progress')
        if reader is not None:      # window capture: the input banner has no wall clock, the progress stream has
            reader.join()
            rec['input_start'] = reader.input_start() or rec['input_start']
            rec['progress_info'] = reader.summary()
            info['problems'] += gt_capture.frozen_check(stderr)
        decoded = None
        if deep:
            p = self.runner([self.args.ffmpeg, '-hide_banner', '-i', path, '-map', '0:v:0', '-f', 'null', '-'],
                            capture_output=True, text=True, errors='replace', timeout=3600)
            decoded = gn.parse_decoded_seconds(p.stderr)
            info['decode_ok'] = p.returncode == 0 and decoded is not None
            info['decoded_s'] = decoded
            frame_size = gn.parse_frame_size(p.stderr)
            try:
                probed = video_sheets.probe_duration(path, self.args.ffmpeg)
            except (SystemExit, AttributeError):  # probe_duration was dropped from video_sheets (tools simplification)
                probed = None
            if not info['decode_ok']:
                info['problems'].append('decode failed: ' + p.stderr.strip()[-300:])
        else:
            # no decode now: the container header gives the size and (after a graceful stop) the length
            p = self.runner([self.args.ffmpeg, '-hide_banner', '-i', path], capture_output=True, text=True, errors='replace')
            frame_size = gn.parse_frame_size(p.stderr)
            probed = video_sheets.parse_duration(p.stderr)
            info['decode_ok'] = None          # the full decode runs in the background (start_deep_check)
        info['frame_size'] = frame_size
        info['probed_s'] = probed
        # graceful stop: container duration; otherwise the decoded length (last fragment may be cut)
        if rec.get('stop') == 'q' and probed:
            rec['duration'], rec['duration_source'] = probed, 'container'
        elif decoded:
            rec['duration'], rec['duration_source'] = decoded, 'decoded'
        elif probed:
            rec['duration'], rec['duration_source'] = probed, 'container'
        else:
            est = self.estimate_duration(rec)
            rec['duration'], rec['duration_source'] = est
        yavg = []
        if rec['duration']:
            for frac in (0.1, 0.5, 0.9):
                q = self.runner([self.args.ffmpeg, '-hide_banner', '-ss', f'{rec["duration"] * frac:.2f}', '-i', path,
                                 '-frames:v', '1', '-vf', 'signalstats,metadata=print:key=lavfi.signalstats.YAVG',
                                 '-f', 'null', '-'], capture_output=True, text=True, errors='replace')
                m = gn.YAVG_RE.findall(q.stderr)
                if m:
                    yavg.append(float(m[0]))
        info['yavg'] = yavg
        if reader is not None and rec.get('stop_wall'):
            info['problems'] += gt_capture.short_check(rec['duration'], (rec['stop_wall'] - rec['spawn_wall']).total_seconds())
        info['problems'] += gn.recording_check(frame_size, rec.get('expected_wh'), yavg, self.args.min_luma)
        if rec['duration']:
            self.set_recording_clock(rec)
        rec['check'] = info
        self.write_rec_json(rec)
        return info

    @staticmethod
    def estimate_duration(rec: dict) -> tuple:
        """(seconds, source) of a recording whose container has no length: the last encoded time of the progress stream
        (window capture), else the wall time between the start and the stop of the recorder."""
        reader = rec.get('progress')
        if reader is not None and getattr(reader, 'samples', None):
            return reader.samples[-1][1], 'progress'
        if rec.get('stop_wall') and rec.get('spawn_wall'):
            return round((rec['stop_wall'] - rec['spawn_wall']).total_seconds(), 2), 'wall time'
        return None, None

    def set_recording_clock(self, rec: dict) -> None:
        mk = [m for m in rec['markers'] if m.get('sent') and m.get('log')]
        rec['clock'] = gn.recording_clock(rec['spawn_wall'], rec.get('stop_wall') or datetime.now(),
                                          rec['duration'], rec['input_start'], mk[0]['sent'] if mk else None,
                                          mk[0]['log'] if mk else None)
        if len(mk) > 1:
            rec['clock']['marker_drift_s'] = round(
                (mk[-1]['log'] - mk[-1]['sent']).total_seconds() - (mk[0]['log'] - mk[0]['sent']).total_seconds(), 3)

    def start_deep_check(self, rec: dict) -> None:
        """Full decode of the recording in a background thread (the slow part of the check, kept away from the game's quit): the
        file must decode to its end. Sets rec['deep'] when done; finish_deep_check() joins it and puts the result into the
        record check. A length that was only estimated is replaced by the decoded one (and the clock is made again)."""
        import threading
        path = rec['path']
        if not os.path.isfile(path) or os.path.getsize(path) == 0:
            return

        def work():
            t0 = time.monotonic()
            try:
                # below-normal priority: the decode takes what the sheets, the clips and the pose meter leave
                low = {'creationflags': gt_post.BELOW_NORMAL} if os.name == 'nt' and self.runner is subprocess.run else {}
                p = self.runner([self.args.ffmpeg, '-hide_banner', '-i', path, '-map', '0:v:0', '-f', 'null', '-'],
                                capture_output=True, text=True, errors='replace', timeout=3600, **low)
                decoded = gn.parse_decoded_seconds(p.stderr)
                rec['deep'] = {'decode_ok': p.returncode == 0 and decoded is not None, 'decoded_s': decoded,
                               'seconds': round(time.monotonic() - t0, 1),
                               'error': '' if p.returncode == 0 and decoded is not None else p.stderr.strip()[-300:]}
            except Exception as e:  # noqa: BLE001
                rec['deep'] = {'decode_ok': False, 'decoded_s': None, 'seconds': round(time.monotonic() - t0, 1), 'error': repr(e)}
        self.deep_check = threading.Thread(target=work, name='deep-check', daemon=True)
        self.deep_check.start()

    def finish_deep_check(self, rec: dict | None) -> None:
        th, self.deep_check = self.deep_check, None
        if th is None or rec is None:
            return
        th.join(timeout=3600)
        deep = rec.get('deep')
        chk = rec.get('check')
        if not deep or chk is None:
            return
        chk['decode_ok'], chk['decoded_s'], chk['decode_seconds'] = deep['decode_ok'], deep['decoded_s'], deep['seconds']
        if not deep['decode_ok']:
            chk['problems'].append('decode failed: ' + deep['error'])
            self.log('[record] !!! RECORDING PROBLEM: decode failed: ' + deep['error'])
        elif rec.get('duration_source') in ('progress', 'wall time') and deep['decoded_s']:
            rec['duration'], rec['duration_source'] = deep['decoded_s'], 'decoded'
            self.set_recording_clock(rec)
        self.write_rec_json(rec)
        self.log(f'[record] full decode in the background: ok={deep["decode_ok"]} decoded {deep["decoded_s"]} s, '
                 f'{deep["seconds"]} s of work')

    # -- 6: scenarios --
    def play(self, pad, guard, name: str, plan: dict, sc=None) -> dict:
        self.check_scenario_time()
        sc = sc or self.scenarios[name]
        self.ensure_overlay(guard, 'closed', f'before {sc.name}')
        res = gt_run.run_scenario(sc, pad, guard)
        self.touched()
        run_no = 1 + sum(1 for r in plan['runs'] if r['scenario'] == sc.name)
        plan['runs'].append({'scenario': sc.name, 'run': run_no, **res})
        if res['aborted']:
            raise (StopNight if is_user_stop(res['aborted']) else CycleError)(res['aborted'])
        return res

    def poll_pos(self, guard, pad=None, timeout: float | None = None) -> dict | None:
        """One position record: utpp_core NightPos (short command) or the inline Lua line (~770 chars), each sent with
        console_verified (up to 3 attempts, settle 0.5/1.0/1.5 s). None = LOST (callers dead-reckon)."""
        self.check_scenario_time()
        wait = self.args.pos_timeout if timeout is None else timeout
        if self.auto_pos:  # utpp_core 0.5.4 NightAuto: read the newest POS line, no console at all
            end = time.monotonic() + wait + 1.0
            while True:
                pos = gn.latest_pos(self.log_tail(60000), datetime.now(), self.args.pos_max_age)
                if pos is not None:
                    self.pos_stats['ok'] += 1
                    return pos
                if time.monotonic() >= end:
                    self.pos_stats['miss'] += 1
                    self.log(f'[pos] no NightAuto POS line younger than {self.args.pos_max_age} s')
                    return None
                guard.wait(0.1)
        modes = [self.pos_mode] if self.pos_mode else ['nightpos', 'inline']
        for mode in modes:
            cmd = gn.NIGHTPOS_CMD if mode == 'nightpos' else gn.POS_FUNC_DEF + ' ' + gn.POS_CALL
            text = self.console_verified(pad, guard, cmd, lambda t: '[night] POS p=' in t, f'pos/{mode}', wait)
            if text is not None:
                if self.pos_mode != mode:
                    self.log(f'[pos] using {mode}')
                self.pos_mode = mode
                self.pos_stats['ok'] += 1
                return gn.parse_pos(text)
            self.pos_stats['miss'] += 1
        return None

    def console(self, pad, guard, cmds: list[str], settle: float | None = None) -> dict:
        """Type console commands with the pad neutral for `settle` (default --console-settle) s first.
        self.console_fast=False (retry path of console_verified) types with the slow per-character timing."""
        if pad is not None:
            pad.release()
        if not self.use_file():          # the settle is for typing into the overlay
            guard.wait(self.args.console_settle if settle is None else settle)
        return self.type_console(guard, cmds, fast=getattr(self, 'console_fast', True))

    def console_verified(self, pad, guard, cmd: str, echo, label: str, timeout: float = 3.0) -> str | None:
        """Type `cmd` until its echo appears in the UTPP log: up to len(--console-settles) attempts with increasing
        settle (default 0.5 / 1.0 / 1.5 s; cycle 2-6: commands typed into the CET console were sometimes lost).
        Returns the log text since the successful attempt, None = LOST. Logs '[cet] <label> ok after N tries'."""
        settles = [float(x) for x in self.args.console_settles.split(',') if x.strip()]
        if self.use_file():
            settles = [0.0]              # one send: the file channel retries itself, a second send would run it twice
        log = self.utpp_log or gt_log.newest_log(self.args.log_dir)
        for n, settle in enumerate(settles, start=1):
            off = file_size(log)
            # cycles 25/28 (2026-10-07): the overlay was verified open and yet the typed text never ran -> from the
            # second attempt on type slowly (fast=False), the overlay/input field drops fast keystrokes at times
            self.console_fast = (n == 1)
            try:
                self.console(pad, guard, [cmd], settle)
            finally:
                self.console_fast = True
            if wait_for_log(log, off, echo, timeout, sleep=guard.wait, clock=time.monotonic):
                self.cet_stats[label] = self.cet_stats.get(label, []) + [n]
                if n > 1 or self.args.log_console_ok:
                    self.log(f'[cet] {label} ok after {n} tries')
                return read_since(log, off)
        self.cet_stats[label] = self.cet_stats.get(label, []) + [0]
        self.log(f'[cet] {label} LOST after {len(settles)} tries ({cmd[:40]}...)')
        return None

    def hold_key(self, guard, scan: int, hold: float) -> None:
        """Guarded key hold (scan code); the key-up is always sent for a key we pressed."""
        api = guard.api
        guard.check()
        api._send([api._scan(scan, False)])
        try:
            guard.wait(hold)
        finally:
            api._send([api._scan(scan, True)])
            self.touched()

    def tap_key(self, pad, guard, scan: int, hold: float = 0.08) -> None:
        """Keyboard tap through the guard (pad neutral first). A press and a release in separate SendInput calls
        `hold` s apart, so the game sees a held frame."""
        if pad is not None:
            pad.release()
        self.ensure_overlay(guard, 'closed', f'before key 0x{scan:02X}')
        self.hold_key(guard, scan, hold)

    def press_close(self, pad, guard, plan: dict) -> str:
        """--wheel-close: pad button tap (default B) or 'key:0xNN' scan code tap."""
        spec = self.args.wheel_close
        if spec.lower().startswith('key:'):
            self.tap_key(pad, guard, int(spec.split(':', 1)[1], 0))
        else:
            b = spec.upper()
            sc = parse_scenario('wheel_close', {'steps': [{'name': 'close', 'duration': 0.6, 'ls': [[0, 0, 0]],
                                                           'rs': [[0, 0, 0]], 'buttons': [{'t': 0.1, 'button': b,
                                                                                            'hold': 0.08}],
                                                           'metrics': False}]})
            self.play(pad, guard, 'wheel_close', plan, sc)
        guard.wait(0.8)
        return spec

    def recover(self, pad, guard, plan: dict, where: str, sc=None, res=None, log_off: int | None = None,
                probe: bool = False) -> dict:
        """Weapon wheel / menu recovery, sent at most once per call.

        Suspected only when |stick| >= 0.7 was commanded for >= 1.5 s and every MMTRACE |v| in that window stayed
        < 0.3 m/s (gt_night.wheel_suspect; the cycle-2 false positive on flick_180_walk at |stick| 0.45 is excluded).
        probe=True: a 2.4 s walk (wheel_probe) first; with NightPos the position delta decides (< 0.4 m = stuck).
        """
        ev = {'where': where, 'at': datetime.now()}
        if probe:
            before = self.poll_pos(guard, pad) if self.pos_mode == 'nightpos' else None
            log_off = file_size(self.utpp_log)
            sc = self.scenarios['wheel_probe']
            res = self.play(pad, guard, 'wheel_probe', plan)
            if before:
                after = self.poll_pos(guard, pad)
                if after:
                    moved = ((after['p'][0] - before['p'][0]) ** 2 + (after['p'][1] - before['p'][1]) ** 2) ** 0.5
                    ev['moved_m'] = round(moved, 2)
                    ev['suspect'] = moved < 0.4
        if ev.get('suspect') is None and sc is not None and res is not None:
            guard.wait(1.0)  # log flush
            lines = gt_log.parse_text(read_since(self.utpp_log, log_off or 0))
            ev['suspect'] = gn.wheel_suspect(sc, res['steps'], lines)
        if ev.get('suspect'):
            ev['closed_with'] = self.press_close(pad, guard, plan)
            self.log(f'[recover] {where}: |stick|>=0.7 for >=1.5 s without movement -> wheel/menu suspected, '
                     f'sent {ev["closed_with"]} once')
        self.cycle.setdefault('recoveries', []).append(ev)
        return ev

    def crouch_toggle(self, pad, guard, want: str) -> dict:
        """Crouch / uncrouch by keyboard C (ToggleCrouch IK_C), verified by MMTRACE stance; one retry."""
        out = {'want': want, 'taps': 0, 'confirmed': False, 'start': datetime.now()}
        for _ in range(2):
            off = file_size(self.utpp_log)
            self.tap_key(pad, guard, gn.SCAN_C, self.args.crouch_key_hold)
            out['taps'] += 1
            if wait_for_log(self.utpp_log, off, lambda t: gn.stance_seen(gt_log.parse_text(t), want),
                            self.args.crouch_verify, sleep=guard.wait, clock=time.monotonic):
                out['confirmed'] = True
                break
        out['end'] = datetime.now()
        self.log(f'[crouch] {want}: {"confirmed" if out["confirmed"] else "NOT confirmed"} after {out["taps"]} C tap(s)')
        return out

    def play_with_key_crouch(self, pad, guard, name: str, plan: dict) -> tuple[dict, list]:
        """Play a scenario whose pad crouch-toggle steps (B hold) are replaced by keyboard C + MMTRACE verification.
        The merged run record keeps the original step names so run.py's analysis/thresholds apply unchanged."""
        sc = self.scenarios[name]
        toggles = gn.crouch_toggle_steps(sc)
        steps, fails, crouched = [], [], False
        part: list = []

        def flush():
            if part:
                sub = Scenario(sc.name, sc.description, list(part), [], {}, sc.rate_hz)
                self.ensure_overlay(guard, 'closed', f'before {sc.name} part')
                r = gt_run.run_scenario(sub, pad, guard)
                self.touched()
                steps.extend(r['steps'])
                part.clear()
                if r['aborted']:
                    raise (StopNight if is_user_stop(r['aborted']) else CycleError)(r['aborted'])
        for i, st in enumerate(sc.steps):
            self.check_scenario_time()
            if i in toggles:
                flush()
                want = 'standing' if crouched or 'uncrouch' in st.name else 'crouched'
                res = self.crouch_toggle(pad, guard, want)
                crouched = want == 'crouched'
                steps.append({'name': st.name, 'start': res['start'], 'end': res['end'], 'metrics': st.metrics})
                if not res['confirmed']:
                    fails.append(f'{st.name}: keyboard C did not give {want} in MMTRACE within '
                                 f'{self.args.crouch_verify:.1f} s (x2)')
            else:
                part.append(st)
        flush()
        run_no = 1 + sum(1 for r in plan['runs'] if r['scenario'] == sc.name)
        rec = {'scenario': sc.name, 'run': run_no, 'steps': steps, 'aborted': None}
        plan['runs'].append(rec)
        return rec, fails

    def spawn_vehicle(self, pad, guard, plan: dict, mode: str, ep: dict) -> None:
        pad.release()
        if mode in ('codeware', 'summon'):
            cmd = gn.SPAWN_CODEWARE_CMD if mode == 'codeware' else gn.SPAWN_SUMMON_CMD
            # the echo '[night] SPAWN <mode> ok=' proves the console took the command (cycle 3: lost command)
            text = self.console_verified(pad, guard, cmd, lambda t: f'[night] SPAWN {mode} ok=' in t, f'spawn/{mode}')
            line = next((l for l in (text or '').splitlines() if '[night] SPAWN' in l), '')
            ep['events'].append({'spawn_log': line.split('[night]', 1)[-1].strip()[:160] if line else None,
                                 'typed': (self.cet_stats.get(f'spawn/{mode}') or [0])[-1] or None})
        elif mode == 'vtap':  # CP2077 2.x: a V tap calls the currently selected vehicle
            self.tap_key(pad, guard, gn.SCAN_V, 0.1)
        elif mode == 'vhold':
            # hold V -> vehicle selector ("WEZWANIE POJAZDU" list, cycle 13); Down arrow highlights the first entry;
            # RELEASING V confirms (Enter only closed the list without calling a vehicle)
            if pad is not None:
                pad.release()
            self.ensure_overlay(guard, 'closed', 'before vhold')
            api = guard.api
            guard.check()
            api._send([api._scan(gn.SCAN_V, False)])
            try:
                guard.wait(self.args.vhold_seconds)
                guard.check()
                api.send_scan_chord([gn.SCAN_DOWN])
                guard.wait(0.4)
            finally:
                api._send([api._scan(gn.SCAN_V, True)])
                self.touched()
        elif mode == 'dpad':
            self.play(pad, guard, 'veh_call', plan)
        else:
            raise CycleError(f'unknown --vehicle-spawn mode {mode!r}')
        ep['events'].append({'spawn': mode, 'at': datetime.now()})

    def vehicle_spawn_modes(self) -> list[str]:
        """--vehicle-spawn in order. With --background the keyboard modes (vtap, vhold) are left out and the pad's own vehicle
        call (dpad) comes first: the run sends no key."""
        modes = [m.strip() for m in self.args.vehicle_spawn.split(',') if m.strip()]
        if getattr(self.args, 'background', False):
            kept = [m for m in modes if m not in ('vtap', 'vhold')]
            if 'dpad' not in kept:
                kept.insert(0, 'dpad')
            if kept != modes:
                self.log(f'[vehicle] --background: keyboard spawn modes left out, order {",".join(kept)}')
            modes = kept
        return modes

    def locate_vehicle(self, pad, guard, plan: dict, ep: dict) -> dict | None:
        """Spawn by the configured modes in order until a vehicle position is reported (entity verified)."""
        modes = self.vehicle_spawn_modes()
        if modes and modes[0] != 'codeware':
            # a Codeware car from an earlier attempt would be reported first (src=tag): remove it
            self.console(pad, guard, [gn.DESPAWN_CMD])
        for mode in modes:
            self.spawn_vehicle(pad, guard, plan, mode, ep)
            if mode in ('vtap', 'vhold', 'summon', 'dpad'):   # these spawn far away and DRIVE in: wait for a real arrival
                pos = self.wait_arrival(pad, guard, ep, mode)
                if pos is not None:
                    return pos
                continue
            if mode == 'codeware' and ep['events'][-2].get('spawn_log') is None:
                self.log('[vehicle] codeware spawn not confirmed -> next mode')
                continue
            wait = self.args.spawn_wait if mode == 'codeware' else self.args.vehicle_arrive_wait
            end = time.monotonic() + wait
            while True:
                guard.wait(min(1.0, max(0.0, end - time.monotonic())))
                pos = self.poll_pos(guard, pad)
                if pos and pos.get('src') != 'mounted' and gn.vehicle_in_range(pos, self.args.max_vehicle_dist):
                    ep['events'].append({'vehicle_found': pos.get('src'), 'by': mode,
                                         'found_dist_m': round(math.hypot(pos['v'][0] - pos['p'][0],
                                                                    pos['v'][1] - pos['p'][1]), 1)})
                    self.log(f'[vehicle] {mode}: {pos.get("src")} vehicle at {ep["events"][-1]["found_dist_m"]} m')
                    return pos
                if pos and pos.get('v'):
                    far = math.hypot(pos['v'][0] - pos['p'][0], pos['v'][1] - pos['p'][1])
                    ep['events'].append({'vehicle_ignored_m': round(far, 1), 'src': pos.get('src')})
                    self.log(f'[vehicle] {pos.get("src")} vehicle {far:.0f} m away > {self.args.max_vehicle_dist:.0f} m '
                             '-> treated as none')
                if time.monotonic() >= end:
                    break
            self.log(f'[vehicle] spawn mode {mode}: no vehicle position reported')
        return None

    def wait_arrival(self, pad, guard, ep: dict, mode: str) -> dict | None:
        """Stand still and wait (max --arrival-max s) for a REAL vehicle to drive in and park within --arrival-dist m
        (gt_night.arrival_state). A position that never changes is 'stale' (cycle 12: phantom summon position in an
        empty desert; cycle 10: the V-hold car arrived ~90 s later). Logs every 10 s."""
        if pad is not None:
            pad.release()
        t0 = time.monotonic()
        end, last_log, stale_logged = t0 + self.args.arrival_max, t0, False
        samples: list = []
        pos, st = None, {'state': 'none'}
        ident = None
        while True:
            p = self.poll_pos(guard, None)
            if p and p.get('v') and p.get('src') in ('summon', 'tag'):
                t = self.pos_time(p)
                if not samples or t > samples[-1][0]:
                    samples.append((t, tuple(p['v'][:2])))
                pos = p
                ident = p.get('src')
            if pos is not None:
                st = gn.arrival_state(samples, pos['p'][:2], self.args.arrival_dist, self.args.car_stop_speed,
                                      self.args.car_stop_hold)
                if st['state'] == 'arrived':
                    waited = time.monotonic() - t0
                    self.log(f'[vehicle] {mode}: {ident} vehicle ARRIVED after {waited:.0f} s, parked at {st["dist"]} m '
                             f'(moved {st["moved"]} m)')
                    ep['arrival'] = dict(st, mode=mode, after_s=round(waited, 1))
                    return pos
                if (st['state'] == 'stale' and getattr(self, 'left_vehicle_this_cycle', False)
                        and st.get('dist') is not None and st['dist'] <= self.args.max_vehicle_dist):
                    # V left this car earlier in the cycle (the save started in it): a parked car, not a phantom. The loco
                    # scenarios walk V away from it (50 m on 2026-11-11 15:49) and the game does not drive a car that close
                    # to the summon point, so any distance the approach accepts (--max-vehicle-dist) is used.
                    self.log(f'[vehicle] {mode}: {ident} vehicle parked at {st["dist"]} m is the one V left -> used')
                    ep['arrival'] = dict(st, mode=mode, after_s=round(time.monotonic() - t0, 1), parked_left=True)
                    return pos
                if (st['state'] == 'stale' and st.get('dist') is not None and st['dist'] <= self.args.arrival_dist):
                    # a car already parked next to V when the summon is sent (Male V save, 2026-11-12 10:19: 2.7 m,
                    # never moved, the run waited 180 s for a car that was there). The phantom positions that the
                    # 'stale' rule guards against were tens of metres away; within --arrival-dist the position is
                    # used, and a wrong one fails at the door (no X prompt) like any missing car.
                    self.log(f'[vehicle] {mode}: {ident} vehicle already parked at {st["dist"]} m -> used')
                    ep['arrival'] = dict(st, mode=mode, after_s=round(time.monotonic() - t0, 1), parked_near=True)
                    return pos
                if gn.parked_far(st, FAR_PARK_HOLD_S, FAR_PARK_MAX_M):
                    # it drove in but stopped short (Male V 2026-11-11 18:24: parked 87 m away for 60+ s and never came
                    # closer) -> walk to it; the approach time box scales with the distance
                    self.log(f'[vehicle] {mode}: {ident} vehicle drove in and parked at {st["dist"]} m, still '
                             f'{st["still"]} s -> walking to it')
                    ep['arrival'] = dict(st, mode=mode, after_s=round(time.monotonic() - t0, 1), parked_far=True)
                    return pos
                if st['state'] == 'stale' and not stale_logged:
                    stale_logged = True
                    self.log(f'[vehicle] {mode}: {ident} position at {st["dist"]} m never moved -> stale, ignored '
                             '(waiting for a vehicle that drives in)')
                    ep['events'].append({'stale_vehicle': dict(st, src=ident)})
                if st['state'] != 'stale' and stale_logged:
                    stale_logged = False
            now = time.monotonic()
            if now - last_log >= 10.0:
                last_log = now
                self.log(f'[vehicle] waiting for arrival: t={now - t0:.0f} d={st.get("dist")} moved={st.get("moved")} '
                         f'state={st.get("state")}')
            if now >= end:
                break
            guard.wait(0.5)
        self.log(f'[vehicle] {mode}: no vehicle arrived within {self.args.arrival_max:.0f} s (last state '
                 f'{st.get("state")}) -> next mode')
        ep['events'].append({'arrival_timeout': mode, 'last': st})
        return None

    def weapon_macro(self, pad, guard, plan: dict, fire: bool = False) -> dict:
        """Draw, aim, holster (fire=False) or draw, aimed shots, reload, holster (fire=True; parts weapon_fire,
        weapon_reload). Both verify draw and holster by NightPos / log."""
        res = {'checks': []}

        def verify(expect_drawn: bool, off: int) -> tuple:
            """Confirmation window >= --weapon-window s with up to 3 NightPos polls (0.5 s settle each); a later
            confirming POS counts (cycle 4: holster FAIL at +0.8 s, w=0 at +4.3 s)."""
            t_end = time.monotonic() + self.args.weapon_window
            ok, why, polls = None, 'no evidence', 0
            guard.wait(0.8)
            while polls < 3:
                pos = self.poll_pos(guard, pad)
                polls += 1
                ok, why = gn.weapon_state(pos, read_since(self.utpp_log, off), expect_drawn)
                if ok:
                    break
                left = t_end - time.monotonic()
                if polls < 3:
                    guard.wait(max(0.5, left / (3 - polls)))
            res['checks'].append({'expect': 'drawn' if expect_drawn else 'holstered', 'confirmed': ok,
                                  'evidence': why, 'polls': polls})
            return ok, why

        off = file_size(self.utpp_log)
        self.play(pad, guard, 'weapon_draw', plan)
        drawn, why_d = verify(True, off)
        if not fire:
            self.play_walking(pad, guard, plan, 'weapon_aim')
        elif drawn is not False:   # shots without a drawn weapon would be punches
            self.play(pad, guard, 'weapon_fire', plan)
            off_r = file_size(self.utpp_log)
            self.play(pad, guard, 'weapon_reload', plan)
            guard.wait(0.5)
            eps = gn.reload_episodes(read_since(self.utpp_log, off_r))
            res['checks'].append({'reload_episode_s': eps or None})   # report only: needs the reload diagnostics
        off = file_size(self.utpp_log)
        self.play(pad, guard, 'weapon_holster', plan)
        holstered, why_h = verify(False, off)
        if holstered is False:  # one more double tap
            off = file_size(self.utpp_log)
            self.play(pad, guard, 'weapon_holster', plan)
            holstered, why_h = verify(False, off)
        walked = self.play_walking(pad, guard, plan, 'weapon_walk')
        self.recover(pad, guard, plan, 'weapon_walk', self.scenarios['weapon_walk'], walked['res'], walked['off'])
        fails = []
        if drawn is not True:
            fails.append(f'draw not confirmed ({why_d})')
        if holstered is not True:
            fails.append(f'holster not confirmed ({why_h})')
        res['verdict'] = 'FAIL' if fails else 'PASS'
        res['fails'] = fails
        self.log(f'[weapon] {res["verdict"]}' + (f': {"; ".join(fails)}' if fails else ''))
        return res

    def shoulder_key_macro(self, pad, guard, taps: int = 2) -> dict:
        """Keyboard shoulder swap: tap L `taps` times, 2.5 s apart, standing; every tap must write a
        '[utpp_camera] shoulder -> left|right (key)' line and the side must alternate."""
        sides, fails = [], []
        for i in range(taps):
            off = file_size(self.utpp_log)
            self.tap_key(pad, guard, gn.SCAN_L, 0.1)
            guard.wait(2.5)
            hit = [ln for ln in read_since(self.utpp_log, off).splitlines() if 'shoulder -> ' in ln and '(key)' in ln]
            if not hit:
                fails.append(f'L tap {i + 1}: no "shoulder -> ... (key)" line')
                continue
            sides.append('left' if 'shoulder -> left' in hit[-1] else 'right')
        if len(sides) == taps and any(a == b for a, b in zip(sides, sides[1:])):
            fails.append(f'the side did not alternate: {sides}')
        self.log(f'[shoulder] keyboard L: {sides or "no swap"}')
        return {'verdict': 'FAIL' if fails else 'PASS', 'fails': fails, 'sides': sides}

    def melee_macro(self, pad, guard, plan: dict) -> dict:
        """Fists macro for the UTPP 0.6.2 upper-body mask. Cycle 21: the test profile starts with a pistol in hand (RT
        does not raise fists, LT aims), so the fists are equipped by script first (gn.FISTS_CMD =
        QuickSlotsManager.RequestEquipFists, --no-cet-fists skips it), then RT tap (melee_draw), punches standing and
        walking, LT block and the fists are put away by script (gn.FISTS_OFF_CMD = UnequipWeapon; the Y double tap
        melee_holster is the second try). Evidence = UTPP log only (gt_night.melee_state): `MMMASK on reasons=0x1` /
        `MMGATE melee=0x..`, the MM weight `w=` of MMGATE/MMMASK/MMTRACE lines while masked (must stay >= 0.9 after
        the mask blend; before 0.6.2 the gate closed and w went to 0) and `MMMASK off` after the holster."""
        res = {'checks': []}
        fails, warns = [], []
        state = {'active': False}
        weights: list = []

        def phase(name: str, off: int, done=None, window_s: float = 0.0) -> dict:
            """Read the log window since `off` (polling every 0.5 s up to window_s until done(st)); record the check."""
            carried = state['active']
            t_end = time.monotonic() + window_s
            guard.wait(0.5)  # log flush
            while True:
                text = read_since(self.utpp_log, off)
                pos = [l for l in text.splitlines() if '[night] POS' in l]
                st = gn.melee_state(pos, text, carried)
                if done is None or done(st) or time.monotonic() >= t_end:
                    break
                guard.wait(0.5)
            state['active'] = st['active_now']
            seen = bool(st['on'] or carried)
            if st['min_w'] is not None:
                weights.append(st['min_w'])
            res['checks'].append({'phase': name, 'mask_seen': seen, 'mask_on': st['on'], 'mask_off': st['off'],
                                  'active_end': st['active_now'], 'min_w': st['min_w'], 'samples': st['samples'],
                                  'bad': [list(b) for b in st['bad'][:5]], 'pos_n': st['pos_n']})
            self.log(f'[melee] {name}: mask_seen={seen} on={st["on"]} off={st["off"]} min_w={st["min_w"]} '
                     f'samples={st["samples"]} bad={len(st["bad"])}')
            if st['bad']:
                ts, w = min(st['bad'], key=lambda b: b[1])
                fails.append(f'{name}: MM weight {w:.2f} < {gn.MELEE_W_MIN} while the melee mask was active ({ts}; '
                             f'{len(st["bad"])} samples)')
            if st['mounted']:
                warns.append(f'{name}: NightPos m=1 (mounted) in the window')
            return st

        def script_cmd(name: str, cmd: str, tag: str) -> bool | None:
            """Type a CET command, echo-verified; records {'phase': name, 'echo': ..., 'ok': ...} in the checks."""
            text = self.console_verified(pad, guard, cmd, lambda t: f'[night] {tag} ok=' in t, name.lower())
            ok, info = gn.night_echo(text, tag)
            res['checks'].append({'phase': name, 'echo': info or None, 'ok': ok})
            self.log(f'[melee] {name}: ' + (info or 'echo LOST'))
            if ok is not True:
                fails.append(f'{name}: ' + (f'the command failed ({info})' if ok is False else
                                            f'no "[night] {tag} ok=" echo (console command lost)'))
            return ok

        off = file_size(self.utpp_log)
        fists_entry = None
        if self.args.cet_fists:
            script_cmd('fists_cmd', gn.FISTS_CMD, 'FISTS')
            # whatever happens below, the ledger puts the fists away before the game is quit (the finally of the run)
            fists_entry = self.settings.register('equipment.fists', 'away', 'raised', 'melee_fists', volatile=True,
                                                 restore=(gn.FISTS_OFF_CMD, 'FISTS_OFF'))
            guard.wait(1.0)
        self.play(pad, guard, 'melee_draw', plan)
        st = phase('melee_draw', off, done=lambda x: x['on'], window_s=self.args.weapon_window)
        if not st['on']:
            fails.append('melee_draw: no "MMMASK on reasons=0x1" / "MMGATE melee=0x.." within '
                         f'{self.args.weapon_window:.0f} s after the RT tap (fists not raised or mask not applied)')
        for name in ('melee_punch', 'melee_punch_walk', 'melee_block'):
            # a walking part: open ground first and one repeat when V is blocked (2026-11-12 04:12); the probes and a
            # discarded attempt stay out of the phase window (the offset belongs to the kept attempt)
            walked = self.play_walking(pad, guard, plan, name, clear=gn.commands_movement(self.scenarios[name]))
            st = phase(name, walked['off'])
            if not res['checks'][-1]['mask_seen']:
                warns.append(f'{name}: no melee mask line (nothing verified for this phase)')
        cleared = False
        if self.args.cet_fists:
            off = file_size(self.utpp_log)
            script_cmd('fists_off_cmd', gn.FISTS_OFF_CMD, 'FISTS_OFF')
            st = phase('melee_fists_off', off, done=lambda x: not x['active_now'], window_s=gn.MELEE_OFF_WINDOW_S)
            cleared = not st['active_now']
            res['checks'][-1]['mask_off_ok'] = cleared
            if not cleared:
                self.log('[melee] melee still active after FISTS_OFF -> Y double tap (melee_holster)')
        if not cleared:
            off = file_size(self.utpp_log)
            self.play(pad, guard, 'melee_holster', plan)
            st = phase('melee_holster', off, done=lambda x: not x['active_now'], window_s=gn.MELEE_OFF_WINDOW_S)
            cleared = not st['active_now']
            res['checks'][-1]['mask_off_ok'] = cleared
            if not cleared:
                fails.append(f'melee_holster: melee mask still active {gn.MELEE_OFF_WINDOW_S:.0f} s after '
                             + ('FISTS_OFF and the Y double tap' if self.args.cet_fists else 'the Y double tap')
                             + ' (no "MMMASK off" / melee bits still set)')
            elif self.args.cet_fists:
                warns.append('fists_off_cmd did not clear melee; the Y double tap did')
        if fists_entry is not None and cleared:
            self.settings.mark_restored(fists_entry)
        res['mask_seen'] = any(c.get('mask_seen') for c in res['checks'])
        res['min_w'] = min(weights) if weights else None
        res['warnings'] = warns
        res['verdict'] = 'FAIL' if fails else 'PASS'
        res['fails'] = fails
        if fails:
            self.log('[melee] FAIL: ' + '; '.join(fails))
        else:
            self.log(f'[melee] PASS: fists mask seen, min w={res["min_w"]}, mask off after holster'
                     + (f' (warnings: {"; ".join(warns)})' if warns else ''))
        return res

    def enemy_macro(self, pad, guard, plan: dict) -> dict:
        """Fist fight against a REAL opponent (`--set melee_fists_target`, not in 'all'): reach, hit direction and the
        look of a hit on a target. Needs only the pad and the command channel - no key, no mouse, no overlay - so it also
        runs with --background (the game must still take pad input then).

        Console (every command echo-verified, gt_night.enemy_*; typed at ~12 ms per character without the file channel):
        NightAuto is pointed at gn.ENEMY_TAG (every POS line then carries the target in `v=`), the fists are equipped and
        raised with NO target in the world, then every phase places a FRESH target: one command reads the old one (its
        health = the previous phase's result), deletes it and spawns the next at the phase distance along the camera
        forward (a target is never moved once it exists). The placement counts only when the next state
        read shows the target `at=1` (|V <-> target - wanted| <= gn.ENEMY_PLACE_TOL); that same read applies Immortal god
        mode and, when the attitude is Friendly or unreadable, Neutral (the game's lock skips a Friendly target) and echoes
        the attitude after it. A phase whose target is not there is NOT punched (verdict line says why).
        Phases: far 2.0 m, near 1.2 m, orbit (camera turned 60 deg first, target 1.2 m along the new forward), block
        1.5 m - each with 3 punches (melee_punch) or the LT block (melee_block). The health must drop in near and orbit;
        far and block are report only. Evidence per phase: MELEEAIM src= (lock?), V <-> target distance at placement and
        from the POS lines, health before / after, hit marker; gt_night.enemy_evaluate gives the verdict. Everything is
        undone in a finally (target deleted, god mode removed, NightAuto tag restored, fists put away)."""
        res = {'checks': [], 'phases': []}
        fails, warns = [], []
        st = {'defined': False, 'spawned': False, 'off': False, 'off_tries': 0, 'retag': False, 'fists': False,
              'active': False}
        phases: list = []
        enemy: dict = {'tried': [], 'record': None, 'spawn': None, 'off': None, 'report': None}
        args = self.args
        records = [r.strip() for r in args.enemy_record.split(',') if r.strip()]

        def say(msg: str) -> None:
            self.log('[enemy] ' + msg)

        def script_cmd(name: str, cmd: str, tag: str) -> bool | None:
            text = self.console_verified(pad, guard, cmd, lambda t: f'[night] {tag} ok=' in t, name.lower())
            ok, info = gn.night_echo(text, tag)
            res['checks'].append({'phase': name, 'echo': info or None, 'ok': ok})
            say(f'{name}: ' + (info or 'echo LOST'))
            if ok is not True:
                fails.append(f'{name}: ' + (f'the command failed ({info})' if ok is False else
                                            f'no "[night] {tag} ok=" echo (console command lost)'))
            return ok

        def enemy_call(label: str, want: float | None = None, prep: int | None = None) -> dict | None:
            """One UTPPENEMY state command. A call before the definition carries it; when a later echo says the global is
            gone (or is lost) the same command is sent once more WITH the definition (the one cheap alternative)."""
            e = None
            for define in ([True] if not st['defined'] else [False, True]):
                cmd = gn.enemy_cmd(label, want, prep, define=define)
                text = self.console_verified(pad, guard, cmd, lambda t: '[night] ENEMY ok=' in t, f'enemy/{label}')
                e = gn.parse_enemy(text)
                if e is not None and e['ok']:
                    st['defined'] = True
                    return e
                say(f'{label}: ' + ('echo LOST' if e is None else f'failed ({e["raw"][:120]})')
                    + ('' if define else ' -> sent again with the definition'))
            return e

        def window(off: int, done=None, window_s: float = 0.0):
            """UTPP log text since `off` (polled every 0.5 s up to window_s until done(ms)); the melee mask state."""
            carried = st['active']
            t_end = time.monotonic() + window_s
            guard.wait(0.5)  # log flush
            while True:
                text = read_since(self.utpp_log, off)
                ms = gn.melee_state([l for l in text.splitlines() if '[night] POS' in l], text, carried)
                if done is None or done(ms) or time.monotonic() >= t_end:
                    break
                guard.wait(0.5)
            st['active'] = ms['active_now']
            return text, ms

        def place(name: str, dist: float, prev: dict | None) -> dict:
            """Place a fresh target `dist` m ahead and wait for the state read that shows it there. -> {'echo': the
            `at=1` state echo or None, 'why': text when not placed, 'old': the state read of the previous target}.
            Candidates: the record that worked, else every --enemy-record in turn (a missing record -> the next one)."""
            out = {'echo': None, 'why': None, 'old': None}
            for rec in ([enemy['record']] if enemy['record'] else records):
                if rec not in enemy['tried']:
                    enemy['tried'].append(rec)
                for attempt in (1, 2):                       # the second only when the echo of the first was lost
                    cmd = gn.enemy_place_cmd(rec, dist, f'after_{prev["name"] if prev else "start"}',
                                             define=not st['defined'])
                    text = self.console_verified(pad, guard, cmd, lambda t: '[night] ENEMY_SPAWN ok=' in t,
                                                 f'enemy_place/{name}')
                    ok, info = gn.night_echo(text, 'ENEMY_SPAWN')
                    res['checks'].append({'phase': f'enemy_place/{name}', 'record': rec, 'echo': info or None, 'ok': ok})
                    say(f'{name}: place {rec} at {dist:.1f} m: ' + (info or 'echo LOST'))
                    if ok is None:
                        out['why'] = 'the placement command was lost (no echo)'
                        continue
                    if ok is False:
                        out['why'] = f'the spawn failed ({info})'
                        break
                    st['spawned'] = True
                    if 'target' not in led:
                        led['target'] = self.settings.register('enemy.target', 'absent', 'spawned', 'melee_fists_target', volatile=True,
                                                               restore=(gn.enemy_off_cmd(), 'ENEMY_OFF'))
                    st['defined'] = True                     # the definition runs before the spawn in the same command
                    if out['old'] is None and attempt == 1:
                        out['old'] = gn.parse_enemy(text)    # the previous target, read before it was deleted
                    end = time.monotonic() + args.enemy_spawn_wait
                    last = None
                    while True:                              # the earliest state read with at=1 ends the wait
                        e = enemy_call('placed', dist, 1)
                        last = e if e is not None and e['ok'] else last
                        if e is not None and e['ok'] and e['found'] and e['at']:
                            enemy['record'] = rec
                            out['echo'], out['why'] = e, None
                            return out
                        if time.monotonic() >= end:
                            break
                        guard.wait(gn.ENEMY_POLL_S)
                    out['why'] = (f'the target is {last["d2"]} m from V, wanted {dist:.1f} m (+-{gn.ENEMY_PLACE_TOL} m)'
                                  if last is not None and last['found'] else
                                  f'no target entity found within {args.enemy_spawn_wait:.0f} s after the spawn')
                    break                                    # the next record, if there is one
            return out

        def do_off() -> None:
            """Last state read, Immortal removed, entity deleted (idempotent: tagged entities only)."""
            st['off_tries'] += 1
            text = self.console_verified(pad, guard, gn.enemy_off_cmd(), lambda t: '[night] ENEMY_OFF ok=' in t,
                                         'enemy_off')
            ok, info = gn.night_echo(text, 'ENEMY_OFF')
            res['checks'].append({'phase': 'enemy_off', 'echo': info or None, 'ok': ok})
            if ok is True:
                st['off'] = True
                enemy['off'] = gn.parse_enemy(text)
                if led.get('target') is not None:
                    self.settings.mark_restored(led['target'])
            say('off: ' + (info or 'echo LOST'))

        def teardown() -> None:
            self.scenario_deadline = None    # after a ScenarioTimeout the cleanup must not raise it again
            if st['spawned'] and not st['off'] and st['off_tries'] < 2:
                try:
                    do_off()
                except Exception as ex:  # noqa: BLE001 - every cleanup step is tried, the original error propagates
                    say(f'cleanup: despawn error {ex!r}')
            if st['spawned'] and not st['off']:
                fails.append('enemy_off: the target may still be in the world (despawn echo lost)')
            if st['retag']:
                try:
                    self.console_verified(pad, guard, gn.nightauto_cmd(args.night_auto_period),
                                          lambda t: '[night] auto on' in t, 'nightauto_restore')
                    st['retag'] = False
                    if led.get('retag') is not None:
                        self.settings.mark_restored(led['retag'])
                except Exception as ex:  # noqa: BLE001
                    say(f'cleanup: NightAuto restore error {ex!r}')
            if st['fists']:
                try:
                    off = file_size(self.utpp_log)
                    script_cmd('fists_off_cmd', gn.FISTS_OFF_CMD, 'FISTS_OFF')
                    _, ms = window(off, done=lambda x: not x['active_now'], window_s=gn.MELEE_OFF_WINDOW_S)
                    if ms['active_now']:
                        say('melee still active after FISTS_OFF -> Y double tap (melee_holster)')
                        off = file_size(self.utpp_log)
                        self.play(pad, guard, 'melee_holster', plan)
                        _, ms = window(off, done=lambda x: not x['active_now'], window_s=gn.MELEE_OFF_WINDOW_S)
                        if ms['active_now']:
                            warns.append('melee mask still active after FISTS_OFF and the Y double tap')
                    st['fists'] = False
                    if led.get('fists') is not None:
                        self.settings.mark_restored(led['fists'])
                except Exception as ex:  # noqa: BLE001
                    say(f'cleanup: fists error {ex!r}')

        spawn_info = None
        led: dict = {}                                       # ledger entries of this macro (listed in the run record)
        try:
            if args.night_auto and self.auto_pos:   # POS lines with the target in v= (finer period for the distance)
                txt = self.console_verified(pad, guard, gn.nightauto_cmd(args.enemy_pos_period, True, gn.ENEMY_TAG),
                                            lambda t: '[night] auto on' in t, 'nightauto_enemy')
                st['retag'] = txt is not None
                if st['retag']:
                    led['retag'] = self.settings.register('night_auto.tag', 'vehicle', 'fight target', 'melee_fists_target', volatile=True)
                if txt is None:
                    warns.append('NightAuto retag LOST: no V <-> target distance in the POS lines')
            off = file_size(self.utpp_log)
            if args.cet_fists:
                script_cmd('fists_cmd', gn.FISTS_CMD, 'FISTS')
                st['fists'] = True
                led['fists'] = self.settings.register('equipment.fists', 'away', 'raised', 'melee_fists_target', volatile=True,
                                                      restore=(gn.FISTS_OFF_CMD, 'FISTS_OFF'))
                if self.auto_pos:   # the earliest POS line with the weapon in hand ends the wait
                    t_end = time.monotonic() + 2.0
                    while not any('[night] POS' in l and ' w=1 ' in l
                                  for l in read_since(self.utpp_log, off).splitlines()):
                        if time.monotonic() >= t_end:
                            warns.append('fists_cmd: no POS line with the weapon in hand within 2 s')
                            break
                        guard.wait(0.1)
                else:
                    guard.wait(1.0)
            self.play(pad, guard, 'melee_draw', plan)
            text, ms = window(off, done=lambda x: x['on'], window_s=args.weapon_window)
            res['checks'].append({'phase': 'melee_draw', 'mask_on': ms['on'], 'active_end': ms['active_now']})
            if not ms['on']:
                warns.append('melee_draw: no "MMMASK on reasons=0x1" / "MMGATE melee=0x.." after the RT tap')
            # the game's target lock looks along the camera from the body: a camera pitched down or a body turned away from
            # it misses a target that stands in front of V, so both are set before the first target is placed
            try:
                self.level_camera(pad, guard, plan, tol=gn.ENEMY_LEVEL_TOL)
                self.play(pad, guard, 'melee_face_forward', plan)
                res['checks'].append({'phase': 'face_forward', 'ok': True})
            except ScenarioTimeout:
                raise
            except Exception as ex:  # noqa: BLE001 - the phases still run, the report says the set-up was skipped
                res['checks'].append({'phase': 'face_forward', 'ok': False, 'error': repr(ex)})
                warns.append(f'camera level / body turn before the phases failed: {ex!r}')
            for name, dist, required, orbit, part in gn.ENEMY_PHASES:
                ph = {'name': name, 'dist_set': dist, 'required': required, 'orbit_deg': orbit, 'part': part,
                      'placed': False, 'why': None, 'aim': gn.meleeaim_stats(''), 'lock': 0, 'dist': None,
                      'mask_on': False, 'mask_seen': False, 'full_on': 0}
                phases.append(ph)
                if orbit:
                    pos = self.poll_pos(guard, pad)
                    if pos and pos.get('p') and pos.get('cf'):
                        side = gn.rotate_stick(*pos['cf'], orbit)
                        target = (pos['p'][0] + side[0] * 10.0, pos['p'][1] + side[1] * 10.0)
                        new = self.camera_align(pad, guard, plan, pos, target, {'events': []},
                                                tol=gn.ENEMY_ORBIT_TOL, why='enemy-orbit')
                        if new and new.get('cf'):
                            ph['orbit_done_deg'] = round(abs(gn.turned_deg(pos['cf'], new['cf'])), 1)
                        if ph.get('orbit_done_deg', 0.0) < 0.5 * orbit:
                            warns.append(f'{name}: the camera turned only {ph.get("orbit_done_deg")} deg (wanted {orbit:.0f})')
                    else:
                        warns.append(f'{name}: no position - the camera orbit was skipped')
                got = place(name, dist, phases[-2] if len(phases) > 1 else None)
                old = got['old']
                if len(phases) > 1 and phases[-2]['placed'] and old is not None and old['ok'] and old['found']:
                    phases[-2]['after'] = old               # health of the previous target, read before its deletion
                ph['place'] = got['echo']
                ph['placed'] = got['echo'] is not None
                ph['why'] = got['why']
                if not ph['placed']:
                    say(f'{name}: NOT placed - {ph["why"]}: no punches in this phase')
                    if enemy['record'] is None:
                        break                                   # no record ever worked: nothing to try again
                    continue
                e = got['echo']
                ph['before'] = e
                ph['placed_d2'] = e['d2']
                if enemy['spawn'] is None:
                    spawn_info = {'found': True, 'att': e['att_after'] if e['att_after'] is not None else e['att'],
                                  'act': e['act'], 'hp': e['hp'], 'hv': e['hv'], 'd2': e['d2'],
                                  'att_read': e['att']}
                    spawn_info['att_name'] = gn.ENEMY_ATT_NAMES.get(spawn_info['att'])
                    enemy['spawn'] = spawn_info
                    say(f'target {enemy["record"]}: att={e["att_name"]} -> {spawn_info["att_name"]} act={e["act"]} '
                        f'hp={e["hp"]}% hv={e["hv"]} d2={e["d2"]} m god={e["god"]} attset={e["attset"]}')
                if e['god'] is not True and not st.get('god_warned'):
                    st['god_warned'] = True
                    warns.append('Immortal god mode not set (' + e['raw'][:80] + '): the target can die')
                if spawn_info is not None and spawn_info.get('att') == 0:
                    say(f'{name}: the target is still Friendly towards V: no punches')
                    ph['skipped'] = 'the target is Friendly'
                    break
                say(f'{name}: placed {e["d2"]} m from V (wanted {dist:.1f}), hv={e["hv"]}')
                off = file_size(self.utpp_log)
                self.play(pad, guard, part, plan)
                text, ms = window(off)
                aim = gn.meleeaim_stats(text)
                ph.update(aim=aim, lock=aim['lock'], dist=gn.target_distance_stats(text), mask_on=ms['on'],
                          mask_seen=bool(ms['on'] or st['active']), full_on=text.count('MMMELEEFULL on'))
            if st['spawned']:
                do_off()
                last = next((p for p in reversed(phases) if p['placed']), None)
                if last is not None and 'after' not in last:
                    last['after'] = enemy['off']
            for ph in phases:
                ph['drop'] = gn.enemy_hp_drop(ph.get('before'), ph.get('after')) if ph['placed'] else None
                ph['after_d2'] = (ph.get('after') or {}).get('d2')
                d, a = ph.get('dist'), ph['aim']
                say(f'{ph["name"]}: want {ph["dist_set"]:.1f} m, placed {ph.get("placed_d2")} m, V<->target first/min/last '
                    f'{(d or {}).get("first")}/{(d or {}).get("min")}/{(d or {}).get("last")} m (n={(d or {}).get("n", 0)}), '
                    f'before hv={(ph.get("before") or {}).get("hv")} after hv={(ph.get("after") or {}).get("hv")} '
                    f'after d2={ph["after_d2"]} m, lock={ph["lock"]} src={a["src"]} lockPitch={a["lock_pitch"]} '
                    f'lockYaw={a["lock_yaw"]} (clamped) lockRawYaw={a["lock_raw_yaw"]} (body to target), '
                    f'hp drop={ph["drop"]}, mask_on={ph["mask_on"]} full_on={ph["full_on"]}'
                    + (f', orbit {ph["orbit_done_deg"]} deg' if 'orbit_done_deg' in ph else '')
                    + ('' if ph['placed'] else f', NOT placed ({ph["why"]})'))
        finally:
            teardown()
        never = None                                  # spawned, but never at the wanted distance (or never found)
        if spawn_info is None and st['spawned']:
            never = {'found': False, 'why': 'target: never placed (' + next(
                (p['why'] for p in reversed(phases) if p.get('why')), 'no phase ran') + ')'}
        ev = gn.enemy_evaluate(spawn_info if spawn_info is not None else never, phases)
        fails += ev['fails']
        warns += ev['warns']
        if args.punch_report:
            enemy['report'] = self.punch_report()
        res['enemy'] = {'record': enemy['record'], 'tried': enemy['tried'], 'spawn': enemy['spawn'], 'locks': ev['locks'],
                        'hits': ev['hits'], 'report': enemy['report'],
                        'phases': [{k: v for k, v in p.items() if k not in ('place', 'after', 'before')} for p in phases]}
        res['warnings'] = warns
        res['verdict'] = 'FAIL' if fails else 'PASS'
        res['fails'] = fails
        if fails:
            self.log('[enemy] FAIL: ' + '; '.join(fails))
        else:
            self.log(f'[enemy] PASS: target {enemy["record"]}, {ev["locks"]} src=lock line(s), hits {ev["hits"]}'
                     + (f' (warnings: {"; ".join(warns)})' if warns else ''))
        return res

    def punch_report(self) -> dict:
        """--punch-report: run the offline measuring tool (Source\\tools\\analysis\\punch_direction_report.py) on the UTPP log;
        its command line comes from gt_night.punch_report_cmd. Skipped (logged) when the script is not there."""
        script = os.path.normpath(os.path.join(HERE, gn.PUNCH_REPORT_REL))
        if not os.path.isfile(script):
            self.log(f'[enemy] --punch-report: {script} not found - skipped')
            return {'ran': False, 'skipped': 'script missing'}
        out = None
        try:
            p = self.runner(gn.punch_report_cmd(sys.executable, script, self.utpp_log), capture_output=True, text=True,
                            timeout=300)
            cdir = getattr(self, 'cycle_dir', None)
            if cdir:
                out = os.path.join(cdir, 'punch_report.txt')
                with open(out, 'w', encoding='utf-8') as f:
                    f.write((p.stdout or '') + (('\n' + p.stderr) if p.stderr else ''))
            self.log(f'[enemy] --punch-report: rc={p.returncode}' + (f', output {out}' if out else ''))
            return {'ran': True, 'rc': p.returncode, 'out': out}
        except Exception as e:  # noqa: BLE001 - a measuring tool never fails the run
            self.log(f'[enemy] --punch-report failed: {e!r}')
            return {'ran': False, 'error': repr(e)}

    def aim_probe_macro(self, pad, guard, plan: dict) -> dict:
        """utpp_probe AimProbe macro: where do the shots leave the weapon while aiming in TPP (camera / fpp camera /
        muzzle)? Console (echo-verified): `perspective.aiming=tpp` (noSave) and `AimProbe(true)`; pad: LT held 4 s with
        2 RT shots (aim_probe_hold); console: `AimProbe(false)` and `perspective.aiming=fpp` - ALWAYS (finally), also on
        a timeout / error. If NightPos shows no weapon in hand (w=0) the draw (Y tap, weapon_draw) is played first.
        Evidence = the UTPP log up to the `[night] AIMPROBE off` echo (gt_night.aimprobe_state): >= 4 samples, >= 1
        shot, a summary line = PASS; the parsed dict is returned as res['aimprobe']."""
        res = {'checks': []}
        fails: list = []

        def script_cmd(name: str, cmd: str, tag: str) -> bool | None:
            text = self.console_verified(pad, guard, cmd, lambda t: f'[night] {tag} ok=' in t, name)
            ok, info = gn.night_echo(text, tag)
            res['checks'].append({'phase': name, 'echo': info or None, 'ok': ok})
            self.log(f'[aimprobe] {name}: ' + (info or 'echo LOST'))
            if ok is not True:
                fails.append(f'{name}: ' + (f'the command failed ({info})' if ok is False else
                                            f'no "[night] {tag} ok=" echo (console command lost)'))
            return ok

        def cleanup(name: str, cmd: str, tag: str) -> bool | None:
            try:
                return script_cmd(name, cmd, tag)
            except Exception as e:  # noqa: BLE001 - every cleanup command is tried, the original error propagates
                fails.append(f'{name}: {e!r}')
                self.log(f'[aimprobe] {name}: cleanup error {e!r}')
                return None

        pos = self.poll_pos(guard, pad)
        if pos and pos.get('weapon') is False:
            self.log('[aimprobe] NightPos w=0 -> weapon_draw first')
            self.play(pad, guard, 'weapon_draw', plan)
            res['checks'].append({'phase': 'weapon_draw_first', 'weapon_before': False})
        # the aiming mode is read first and put back as it was (the old fixed 'fpp' only when the value cannot be read)
        prev = None
        try:
            prev = (self.settings.read(self.settings_io(pad, guard), ['perspective.aiming'], 'aiming_get') or {}).get('perspective.aiming')
        except Exception as e:  # noqa: BLE001
            self.log(f'[aimprobe] reading perspective.aiming failed: {e!r}')
        restore_to = prev if prev in ('tpp', 'fpp') else 'fpp'
        res['aiming_restore'] = {'previous': prev, 'restore_to': restore_to}
        if prev is None:
            self.log("[aimprobe] perspective.aiming could not be read: it is restored to 'fpp' as before")
        # a fallback value that was never read must not be written into anybody's saved settings by a later launch:
        # then the entry is volatile (not in the pending file) and carries its own restore command
        aim_entry = self.settings.register('perspective.aiming', restore_to, 'tpp', 'aim_probe', volatile=prev is None,
                                           restore=(gn.aiming_cmd(restore_to), 'AIMPROBE aiming=' + restore_to) if prev is None else None)
        off = file_size(self.utpp_log)
        try:
            script_cmd('aiming_tpp', gn.aiming_cmd('tpp'), 'AIMPROBE aiming=tpp')
            script_cmd('aimprobe_on', gn.aimprobe_cmd(True), 'AIMPROBE on')
            guard.wait(0.5)
            self.play(pad, guard, 'aim_probe_hold', plan)
            guard.wait(0.5)  # log flush
        finally:
            cleanup('aimprobe_off', gn.aimprobe_cmd(False), 'AIMPROBE off')
            if cleanup('aiming_' + restore_to, gn.aiming_cmd(restore_to), 'AIMPROBE aiming=' + restore_to) is True:
                self.settings.mark_restored(aim_entry)
        text = read_since(self.utpp_log, off)
        lines = text.splitlines()
        last_off = max((i for i, l in enumerate(lines) if '[night] AIMPROBE off ok=' in l), default=None)
        if last_off is not None:
            text = '\n'.join(lines[:last_off + 1])
        st = gn.aimprobe_state(text)
        fails += gn.aimprobe_fails(st)
        res['aimprobe'] = st
        res['verdict'] = 'FAIL' if fails else 'PASS'
        res['fails'] = fails
        sm = st['summary'] or {}
        self.log(f'[aimprobe] {res["verdict"]}: originNearest={sm.get("originNearest")} meanDCam={sm.get("meanDCam")} '
                 f'meanDFpp={sm.get("meanDFpp")} tppDuringAim={st["tppDuringAim"]} samples={st["samples"]} '
                 f'shots={len(st["shots"])}' + (f' ({"; ".join(fails)})' if fails else ''))
        return res

    def camera_align(self, pad, guard, plan: dict, known: dict, target, ep: dict, tol: float | None = None,
                     why: str = 'align') -> dict:
        """Turn the camera with short right-stick pulses until the target is within `tol` deg of the camera forward.
        Each pulse is followed by a real POS (cf feedback); the yaw rate and the turn direction adapt to it."""
        tol = self.args.align_tol if tol is None else tol
        self.probe_cache.invalidate(f'camera turned ({why})')
        for k in range(self.args.align_pulses):
            b = gn.bearing_deg(known['p'][:2], known['cf'], target)
            if abs(b) < tol:
                return known
            rx, dur = gn.camera_pulse(b, self.cam_rate, self.cam_sign)
            self.log(f'[camera] {why} k={k} bearing={b:.0f} rs=({rx:.0f},0) for {dur:.2f} s rate={self.cam_rate:.0f}')
            ep['events'].append({'camera': k, 'bearing': round(b, 1), 'rs': rx, 'dur': dur})
            sc = parse_scenario('cam_turn', {'steps': [
                {'name': 'turn', 'duration': dur + 0.25, 'interp': 'step', 'ls': [[0, 0, 0]],
                 'rs': [[0, 0, 0], [0.02, rx, 0], [0.02 + dur, 0, 0]], 'metrics': False}]})
            self.play(pad, guard, 'cam_turn', plan, sc)
            new = self.poll_pos(guard, pad)
            if not new:  # no feedback: assume the pulse turned the camera by rate * duration
                known = dict(known, cf=gn.rotate_stick(*known['cf'], rx * self.cam_sign * self.cam_rate * dur))
                continue
            turned = gn.turned_deg(known['cf'], new['cf'])
            want = 1.0 if b > 0 else -1.0
            if abs(turned) > 3.0:
                if turned * want < 0:  # the game turned the other way: flip the stick sign
                    self.cam_sign = -self.cam_sign
                    self.log('[camera] turn direction inverted -> right stick sign flipped')
                self.cam_rate = min(720.0, max(30.0, 0.5 * self.cam_rate + 0.5 * abs(turned) / dur))
            known = new if new.get('v') else dict(new, v=known.get('v'), vr=known.get('vr'))
        return known

    def camera_pitch(self) -> float | None:
        """Newest camera pitch (deg) from the utpp_camera trace lines (written only with the diagnostic logs on)."""
        return gn.latest_cam_pitch(self.log_tail(60000))

    def level_camera(self, pad, guard, plan: dict, tol: float = 12.0, pulses: int = 10) -> None:
        """A save can start with the camera looking steeply down (Male V save: -65 deg, 2026-11-12) and every frame of
        the run is then shot from above. Right-stick pulses bring the pitch within `tol` deg of level. Needs the
        camera trace in the log; without it nothing is done."""
        pitch = self.camera_pitch()
        if pitch is None:
            self.log('[camera] level: no camera trace in the log (diagnostic logs off) - skipped')
            return
        sign, rate = 1.0, 60.0   # stick up = look up, deg/s at half stick; both corrected from the feedback
        for k in range(pulses):
            if abs(pitch) <= tol:
                break
            ry, dur = gn.pitch_pulse(pitch, rate, sign)
            self.log(f'[camera] level k={k} pitch={pitch:.1f} rs=(0,{ry:.1f}) for {dur:.2f} s')
            sc = parse_scenario('cam_turn', {'steps': [
                {'name': 'level', 'duration': dur + 0.6, 'interp': 'step', 'ls': [[0, 0, 0]],
                 'rs': [[0, 0, 0], [0.02, 0, ry], [0.02 + dur, 0, 0]], 'metrics': False}]})
            self.play(pad, guard, 'cam_turn', plan, sc)
            new = self.camera_pitch()
            if new is None:
                break
            moved = new - pitch
            if abs(moved) > 2.0:
                if moved * -pitch < 0:
                    sign = -sign
                    self.log('[camera] level: pitch moved the other way -> stick sign flipped')
                rate = min(240.0, max(15.0, 0.5 * rate + 0.5 * abs(moved) / (dur * abs(ry))))
            pitch = new
        self.log(f'[camera] level: pitch {pitch:.1f} deg' + ('' if abs(pitch) <= tol else ' (NOT level)'))

    def turn_camera_90(self, pad, guard, plan: dict, pos: dict) -> None:
        """Turn the camera 90 deg to the right of its forward (the scenarios steer relative to the camera)."""
        self.probe_cache.invalidate('camera turned (clear path)')
        side = gn.rotate_stick(*pos['cf'], 90.0)
        target = (pos['p'][0] + side[0] * 10.0, pos['p'][1] + side[1] * 10.0)
        self.camera_align(pad, guard, plan, pos, target, {'events': []}, why='clear-path')

    def leave_car(self, pad, guard, plan: dict, seconds: float = 4.0) -> None:
        """After the car scenario: turn the camera away from the car and sprint off, so the walking scenarios that
        follow do not start next to it. Best effort (needs a position with the vehicle)."""
        pos = self.poll_pos(guard, pad)
        if not pos or not pos.get('p') or not pos.get('cf') or not pos.get('v') or pos.get('src') == 'mounted':
            self.log('[leave-car] no position with the car - staying')
            return
        away = (pos['p'][0] - pos['v'][0], pos['p'][1] - pos['v'][1])
        n = math.hypot(*away) or 1.0
        target = (pos['p'][0] + away[0] / n * 10.0, pos['p'][1] + away[1] / n * 10.0)
        self.camera_align(pad, guard, plan, pos, target, {'events': []}, why='leave-car')
        sc = parse_scenario('clear_probe', {'steps': [
            {'name': 'leave', 'duration': seconds + 1.5, 'interp': 'step',
             'ls': [[0, 0, 0], [0.05, 0, 1], [0.05 + seconds, 0, 0]], 'rs': [[0, 0, 0]],
             'buttons': [{'t': 0.2, 'button': 'LS', 'hold': 0.12}], 'metrics': False}]})
        self.play(pad, guard, 'clear_probe', plan, sc)
        new = self.poll_pos(guard, pad)
        if new and new.get('p'):
            self.log(f'[leave-car] {math.hypot(new["p"][0] - pos["v"][0], new["p"][1] - pos["v"][1]):.0f} m from the car')

    def extend_deadline(self, seconds: float) -> None:
        """Time spent on probes and a repeated attempt does not count against the scenario time limit."""
        if self.scenario_deadline is not None:
            self.scenario_deadline += seconds

    def clear_path(self, pad, guard, plan: dict, where: str) -> dict:
        """Before a walking scenario: the probe (see _clear_path); its time goes to the scenario's probe column."""
        t0 = time.monotonic()
        try:
            return self._clear_path(pad, guard, plan, where)
        finally:
            self.budget.add('probe', time.monotonic() - t0)

    def _clear_path(self, pad, guard, plan: dict, where: str) -> dict:
        """Make sure the ground ahead is open before a walking scenario (NightPos / NightAuto positions).

        Long route (gt_night.route_estimate: net displacement or longest straight leg > gn.ROUTE_LONG_M): the look-ahead
        walks the main leg (<= gn.LOOKAHEAD_MAX_S s) and checks that V kept moving, then walks back to the start
        (look_ahead). Short route: push the stick forward for CLEAR_PROBE_S and measure how far V moved.
        Blocked (< CLEAR_MIN_MOVE_M, Female V 17:37 walked into a bush: capsule 0.07 m/s with the stick at 0.8 for 9 s)
        -> turn the camera 90 deg and probe again, up to 3 turns (look-ahead: 3 directions). The walking scenarios
        steer relative to the camera, so the scored walk then starts into open ground. Returns the probe record (also
        kept in the cycle)."""
        out = {'where': where, 'probes': []}
        sc = self.scenarios.get(where)
        la = None
        if sc is not None:
            la = gn.lookahead_plan(gn.route_estimate(sc, self.speeds, self.cam_rate, self.cam_sign), self.speeds)
        done = la is not None and self.look_ahead(pad, guard, plan, where, la, out)
        for turn in range(0 if done else 4):
            p0 = self.poll_pos(guard, pad)
            if not p0 or not p0.get('p') or not p0.get('cf'):
                out['result'] = 'no position'
                break
            sc = parse_scenario('clear_probe', {'steps': [
                {'name': 'probe', 'duration': CLEAR_PROBE_S + 0.6, 'interp': 'step',
                 'ls': [[0, 0, 0], [0.02, 0, CLEAR_PROBE_STICK], [0.02 + CLEAR_PROBE_S, 0, 0]], 'rs': [[0, 0, 0]],
                 'metrics': False}]})
            self.play(pad, guard, 'clear_probe', plan, sc)
            p1 = self.poll_pos(guard, pad)
            if not p1 or not p1.get('p'):
                out['result'] = 'no position'
                break
            moved = math.hypot(p1['p'][0] - p0['p'][0], p1['p'][1] - p0['p'][1])
            out['probes'].append({'turn': turn, 'moved_m': round(moved, 2)})
            if moved >= CLEAR_MIN_MOVE_M:
                out['result'] = 'clear'
                break
            self.log(f'[clear] {where}: V moved {moved:.2f} m in {CLEAR_PROBE_S} s -> blocked, camera +90 deg')
            if turn == 3:
                out['result'] = 'blocked'
                break
            self.turn_camera_90(pad, guard, plan, p1)
        self.cycle.setdefault('clear_path', []).append(out)
        if out.get('result') != 'clear':
            self.log(f'[clear] {where}: {out.get("result")}')
        return out

    def look_ahead(self, pad, guard, plan: dict, where: str, la: dict, out: dict) -> bool:
        """Long-route look-ahead (clear_path): walk `la['duration']` s along the main leg of the route (camera-relative
        stick, camera not turned) and check with the NightAuto positions that V kept moving (no stall of
        gn.LOOKAHEAD_STALL_S s while the stick is pushed); then walk back to the start. Blocked -> camera +90 deg and
        probe again from where V stands, up to 3 directions. The way back reverses the stick instead of turning the
        camera 180 deg twice: every camera turn settles only within --align-tol, and that yaw error would rotate the
        whole camera-relative route; the walk back is slower, the cost is logged as 'probe N s'. Fills out['result']
        and out['probes']; the residual start offset is logged and kept in out['lookahead']. Returns True (the
        look-ahead replaces the short probe)."""
        t_in = time.monotonic()
        info = out['lookahead'] = {'heading_deg': la['heading_deg'], 'duration_s': la['duration'],
                                   'reach_m': la['reach_m'], 'walks': []}
        for direction in range(3):
            p0 = self.poll_pos(guard, pad)
            if not p0 or not p0.get('p') or not p0.get('cf'):
                out['result'] = 'no position'
                break
            if direction == 0:
                hit = self.probe_cache.lookup(p0['p'], p0['cf'], la)
                if hit:
                    return self.skip_look_ahead(pad, guard, plan, where, la, out, info, hit, t_in)
            walk = self.look_ahead_walk(pad, guard, plan, la, p0)
            info['walks'].append({**{k: v for k, v in walk.items() if k != 'p1'},
                                  'p0': [round(p0['p'][0], 2), round(p0['p'][1], 2)],
                                  'cf': [round(p0['cf'][0], 3), round(p0['cf'][1], 3)]})
            out['probes'].append({'turn': direction, 'moved_m': walk.get('moved_m'), 'stall_s': walk.get('stall_s')})
            out['result'] = walk['result']
            if walk['result'] == 'clear':
                self.probe_cache.remember(where, p0['p'], p0['cf'], la, walk.get('moved_m') or 0.0)
            if walk['result'] != 'blocked':
                break
            self.log(f'[clear] {where}: look-ahead {la["heading_deg"]:+.0f} deg blocked (moved {walk["moved_m"]:.2f} m, '
                     f'stall {walk["stall_s"]} s' + (', V left the ground: a drop on the route' if walk.get('drop') else '')
                     + ') -> camera +90 deg')
            if direction == 2:
                break
            self.turn_camera_90(pad, guard, plan, walk['p1'])
        spent = time.monotonic() - t_in
        info['probe_s'] = round(spent, 1)
        self.extend_deadline(spent)
        back = (info['walks'][-1].get('back') or {}) if info['walks'] else {}
        self.log(f'[clear] {where}: look-ahead {la["heading_deg"]:+.0f} deg, {la["reach_m"]} m ahead -> '
                 f'{out.get("result")}, probe {spent:.1f} s'
                 + (f', back at start +{back["residual_m"]:.2f} m' if back.get('residual_m') is not None else ''))
        return True

    def skip_look_ahead(self, pad, guard, plan: dict, where: str, la: dict, out: dict, info: dict, hit: dict, t_in: float) -> bool:
        """The look-ahead is not walked: an earlier probe from this spot along this direction reached far enough and came back
        clear (gt_flow.ProbeCache). The short push that turns V along the route still happens, so the scenario starts in the
        same state as after a probe."""
        info['skipped'] = hit
        out['result'] = 'clear'
        out['probes'].append({'turn': 0, 'moved_m': None, 'skipped': True})
        self.probe_cache.skipped += 1
        self.budget.count('skipped')
        self.face_forward(pad, guard, plan, la)
        spent = time.monotonic() - t_in
        info['probe_s'] = round(spent, 1)
        self.extend_deadline(spent)
        self.log(f'[clear] {where}: look-ahead {la["heading_deg"]:+.0f} deg SKIPPED - V stands {hit["dist_m"]} m from where the '
                 f'probe of {hit["where"]} started, camera {hit["cam_diff_deg"]:+.0f} deg / route {hit["route_diff_deg"]:+.0f} deg '
                 f'apart, that probe covered {hit["covered_m"]} m (needed {hit["needed_m"]} m) and came back clear; '
                 f'face push only, {spent:.1f} s')
        return True

    def face_forward(self, pad, guard, plan: dict, la: dict) -> None:
        """A short push in the route direction turns V forward again (after a walk back with the reversed stick he faces
        backwards), then he stands still."""
        sx, sy = la['stick']
        k = 0.35 / max(math.hypot(sx, sy), 0.01)        # a small stick: turn, do not travel
        fx, fy = round(sx * k, 4), round(sy * k, 4)
        face = parse_scenario('clear_probe', {'steps': [
            {'name': 'face', 'duration': 1.4, 'interp': 'step',
             'ls': [[0, 0, 0], [0.02, fx, fy], [0.32, 0, 0]], 'rs': [[0, 0, 0]], 'metrics': False}]})
        self.play(pad, guard, 'clear_probe', plan, face)

    def look_ahead_walk(self, pad, guard, plan: dict, la: dict, p0: dict) -> dict:
        """One look-ahead walk from position p0 and, when clear, the walk back. {'result': 'clear' | 'blocked' |
        'no position', 'moved_m', 'stall_s', 'p1', 'back': {...}}. Blocked = V moved < CLEAR_MIN_MOVE_M or stalled
        gn.LOOKAHEAD_STALL_S s with the stick pushed (POS samples; needs NightAuto)."""
        sx, sy = la['stick']
        dur = la['duration']
        sc = parse_scenario('clear_probe', {'steps': [
            {'name': 'probe', 'duration': dur + 0.6, 'interp': 'step',
             'ls': [[0, 0, 0], [0.02, sx, sy], [0.02 + dur, 0, 0]], 'rs': [[0, 0, 0]], 'metrics': False}]})
        off = file_size(self.utpp_log)
        res = self.play(pad, guard, 'clear_probe', plan, sc)
        p1 = self.poll_pos(guard, pad)
        if not p1 or not p1.get('p'):
            return {'result': 'no position'}
        moved = math.hypot(p1['p'][0] - p0['p'][0], p1['p'][1] - p0['p'][1])
        stall = None
        if self.auto_pos:
            rep = gn.blocked_report(gn.pos_samples(read_since(self.utpp_log, off)),
                                    gn.pushed_intervals(sc, res.get('steps', [])), min_s=gn.LOOKAHEAD_STALL_S)
            stall = rep['stall_s'] if rep['samples'] >= 3 else None
            blocked_stall = rep['blocked'] and rep['samples'] >= 3
        else:
            blocked_stall = False
        out = {'moved_m': round(moved, 2), 'stall_s': stall, 'p1': p1}
        if moved < CLEAR_MIN_MOVE_M or blocked_stall:
            out['result'] = 'blocked'
            return out
        # a route on which V leaves the ground (a ledge, a drop) does not measure ground locomotion: treated like an obstacle
        if self.utpp_log and gn.left_ground(read_since(self.utpp_log, off)):
            out['drop'] = True
            out['result'] = 'blocked'
            return out
        out['result'] = 'clear'
        out['back'] = self.walk_back(pad, guard, plan, la, p0, p1, moved / max(dur, 0.2))
        # V walked back with the stick reversed, so he now FACES backwards: the scenario would start with a 180 deg
        # turn in its first moving step (2026-11-12 12:14 / 12:28 runs: new first-step FAILs on footlock and
        # transitions). A short push in the route direction turns him forward again; then he stands still.
        self.face_forward(pad, guard, plan, la)
        return out

    def walk_back(self, pad, guard, plan: dict, la: dict, p0: dict, p1: dict, out_speed: float = 0.0) -> dict:
        """Back to the start p0 with the stick reversed (camera untouched); one correction pass towards p0 when the
        residual is above gn.LOOKAHEAD_BACK_TOL_M. Returns {'residual_m', 'passes', 'back_s'}; residual None = no
        position at the end."""
        sx, sy = la['stick']
        dist = math.hypot(p1['p'][0] - p0['p'][0], p1['p'][1] - p0['p'][1])
        stick, remaining, spent, here = (-sx, -sy), dist, 0.0, p1
        passes, residual = 0, dist
        for passes in (1, 2):
            # first pass: the speed just measured on the way out (same stick size); the table speed of the 'back'
            # class was too low (returns of 6.5 s for 2.5 s out, residual 1.5 m in the 12:14 run)
            spd = max(out_speed if passes == 1 and out_speed > 0.5 else
                      gn.speed_for(stick[0], stick[1], False, self.speeds), 0.5)
            dur = round(min(remaining / spd, 12.0 if passes == 1 else 3.0), 2)
            if dur < 0.2:
                passes -= 1
                break
            sc = parse_scenario('clear_probe', {'steps': [
                {'name': 'back', 'duration': dur + 0.6, 'interp': 'step',
                 'ls': [[0, 0, 0], [0.02, stick[0], stick[1]], [0.02 + dur, 0, 0]], 'rs': [[0, 0, 0]],
                 'metrics': False}]})
            self.play(pad, guard, 'clear_probe', plan, sc)
            spent += dur + 0.6
            here = self.poll_pos(guard, pad)
            if not here or not here.get('p'):
                residual = None
                break
            residual = math.hypot(here['p'][0] - p0['p'][0], here['p'][1] - p0['p'][1])
            if residual <= gn.LOOKAHEAD_BACK_TOL_M or passes == 2:
                break
            b = math.radians(gn.bearing_deg(here['p'][:2], here['cf'], p0['p'][:2]))
            stick = (round(math.sin(b) * gn.LOOKAHEAD_STICK, 4), round(math.cos(b) * gn.LOOKAHEAD_STICK, 4))
            remaining = residual
        return {'residual_m': None if residual is None else round(residual, 2), 'passes': passes,
                'back_s': round(spent, 1)}

    def play_attempt(self, pad, guard, plan: dict, name: str, key_crouch: bool = False) -> dict:
        """One attempt of a scenario: {'res', 'cfails', 'off' (UTPP log offset before it), 'start'}."""
        off, start = file_size(self.utpp_log), datetime.now()
        if key_crouch:
            res, cfails = self.play_with_key_crouch(pad, guard, name, plan)
        else:
            res, cfails = self.play(pad, guard, name, plan), []
        return {'res': res, 'cfails': cfails, 'off': off, 'start': start}

    def attempt_blocked(self, name: str, att: dict) -> dict | None:
        """Was V stuck while the stick was pushed during this attempt (stick >= 0.2 and < 0.3 m/s for >= 1.5 s,
        gt_night.blocked_report on the NightAuto POS lines of the attempt)? None = no (or no way to tell: without
        NightAuto positions nothing is detected and the verdict stays with run.py's MMTRACE check)."""
        sc = self.scenarios.get(name)
        steps = att['res'].get('steps')
        if not self.auto_pos or sc is None or not steps:
            return None
        iv = gn.pushed_intervals(sc, steps)
        if not iv:
            return None
        rep = gn.blocked_report(gn.pos_samples(read_since(self.utpp_log, att['off'])), iv)
        return rep if rep['blocked'] else None

    def play_walking(self, pad, guard, plan: dict, name: str, key_crouch: bool = False, clear: bool = True) -> dict:
        """A walking scenario (or macro part) with open ground first and ONE repeat when V is blocked.

        clear_path, then the attempt. When the NightAuto positions show V stuck (attempt_blocked) the attempt is not
        scored: its plan entry moves from plan['runs'] to plan['discarded_runs'] (marked 'blocked_repeat': True, so
        run.py never sees it: no verdict, no count, no failure clip, and the repeat keeps run number 1), a wheel/menu
        recovery is checked, the crouch is undone for key-crouch scenarios, clear_path turns the camera to free ground
        and the scenario is played once more. A second block keeps that attempt in plan['runs'] (run.py scores it
        INVALID when the MMTRACE check agrees, and it is clipped) and marks its entry 'blocked_live'.
        Returns {'res', 'cfails', 'off', 'start', 'discarded': detail | None}; 'off' is the log offset of the kept
        attempt, 'start' its start time."""
        t_in = time.monotonic()
        if self.on_foot_first(pad, guard, plan, name):
            self.extend_deadline(time.monotonic() - t_in)
        if clear:
            self.clear_path(pad, guard, plan, name)
        att = self.play_attempt(pad, guard, plan, name, key_crouch)
        att['discarded'] = None
        rep = self.attempt_blocked(name, att)
        if rep is None:
            return att
        detail = {'scenario': name, 'attempt': 1, 'stall_s': rep['stall_s'], 'at': rep['at'], 'start': att['start'],
                  'end': datetime.now()}
        self.probe_cache.invalidate(f'{name} was blocked')
        self.budget.count('retry_n')
        self.budget.add('retry_s', (detail['end'] - detail['start']).total_seconds())
        entry = next((r for r in reversed(plan['runs']) if r['scenario'] == name), None)
        if entry is not None:
            plan['runs'][:] = [r for r in plan['runs'] if r is not entry]
            entry['blocked_repeat'] = True
            entry['blocked'] = {k: v for k, v in detail.items() if k != 'scenario'}
            plan.setdefault('discarded_runs', []).append(entry)
        self.log(f'[blocked] {name}: V pushed the stick but stalled {rep["stall_s"]} s (< {gn.STALL_SPEED_MAX} m/s) -> '
                 'attempt discarded (not scored), camera to free ground, repeating once')
        self.cycle.setdefault('blocked_repeats', []).append(detail)
        sc = self.scenarios[name]
        with self.budget.timed('retry_s'):
            self.recover(pad, guard, plan, f'blocked {name}', sc, att['res'], att['off'])
            if key_crouch:
                stances = [s for s in (gn.stance_of(l) for l in gt_log.parse_text(read_since(self.utpp_log, att['off'])))
                           if s]
                if stances and stances[-1] == 'crouched':
                    self.crouch_toggle(pad, guard, 'standing')
        with self.budget.timed('wait'):
            guard.wait(1.0)
        self.clear_path(pad, guard, plan, name)
        self.extend_deadline(time.monotonic() - t_in)
        att2 = self.play_attempt(pad, guard, plan, name, key_crouch)
        att2['discarded'] = detail
        rep2 = self.attempt_blocked(name, att2)
        if rep2 is not None:
            self.log(f'[blocked] {name}: blocked again after the repeat (stall {rep2["stall_s"]} s) -> kept as played; '
                     'run.py scores it INVALID when the MMTRACE check agrees')
            entry2 = next((r for r in reversed(plan['runs']) if r['scenario'] == name), None)
            if entry2 is not None:
                entry2['blocked_live'] = {'stall_s': rep2['stall_s'], 'at': rep2['at']}
        return att2

    def approach(self, pad, guard, plan: dict, pos: dict, ep: dict) -> float:
        """Run to the driver door along car-frame waypoints (gt_night.plan_path: around an end of the car when V is
        on the wrong side or at an end, >= 1.4 m clearance along the side; cycle 10 stalled at corners pushing into
        the body).

        - waypoint more than --turn-first deg off the camera forward -> turn the camera first (camera_align), then run
          mostly forward (+ L3 sprint tap when far);
        - each iteration: <= --approach-chunk s of stick, then a real POS (NightAuto line); dead reckoning only
          bridges a missed POS (<= --dr-max s); the path is re-planned from every real POS;
        - stop: within 0.8 m of the door point AND car-frame lat in [0.9, 1.7]; then face the car centre;
        - time box max(25 s, dist/1.5 + 15 s). Logs '[approach] carframe lon=.. lat=..' every iteration.
        """
        door_rel = self.door_spot()
        known, est = pos, tuple(pos['p'][:2])
        door = gn.door_target(known, door_rel[1], door_rel[0])
        dist0 = math.hypot(door[0] - est[0], door[1] - est[1])
        box = gn.approach_timebox(dist0, self.args.approach_timeout)
        end = time.monotonic() + box
        self.log(f'[approach] start dist={dist0:.1f} m time box {box:.0f} s')
        dist, i, dr_s, src, aligns, last_wp, stuck_chunks, sidesteps = dist0, 0, 0.0, 'pos', 0, None, 0, 0
        while time.monotonic() < end:
            self.check_scenario_time()
            v_xy, vr = known['v'][:2], known['vr']
            lon, lat = gn.car_frame(est, v_xy, vr)
            door = gn.door_target(known, door_rel[1], door_rel[0])
            dist = math.hypot(door[0] - est[0], door[1] - est[1])
            if gn.at_door(lon, lat, door_rel, radius=DOOR_RADIUS_M):
                self.log(f'[approach] i={i} carframe lon={lon:.2f} lat={lat:.2f} at the door (dist {dist:.2f}, src={src})')
                break
            wps = gn.plan_path(lon, lat, door_rel, blocked_end=self.blocked_end)
            wp = wps[0]
            target = gn.from_car_frame(wp[0], wp[1], v_xy, vr)
            b = gn.bearing_deg(est, known['cf'], target)
            if wp != last_wp:
                ep['events'].append({'waypoints': [[round(a, 2), round(c, 2)] for a, c in wps]})
                last_wp = wp
            if abs(b) > self.args.turn_first and aligns < 3 + len(wps):
                aligns += 1
                known = self.camera_align(pad, guard, plan, dict(known, p=(est[0], est[1], known['p'][2])), target, ep,
                                          why='turn-first')
                est = tuple(known['p'][:2])
                continue
            to_wp = math.hypot(target[0] - est[0], target[1] - est[1])
            sx, sy, _ = gn.approach_stick(est, known['cf'], target, self.args.approach_mag)
            sprint = dist > self.args.sprint_dist and gn.stick_class(sx, sy) == 'forward'
            dur = min(self.args.approach_chunk, max(0.2, to_wp / max(gn.speed_for(sx, sy, sprint, self.speeds), 0.3)))
            self.log(f'[approach] i={i} carframe lon={lon:.2f} lat={lat:.2f} wp {len(wps)}=({wp[0]:.1f},{wp[1]:.1f}) '
                     f'dist={dist:.2f} bearing={b:.0f} stick=({sx:.2f},{sy:.2f}){" sprint" if sprint else ""} src={src}')
            ep['events'].append({'approach': i, 'dist_m': round(dist, 2), 'lon': round(lon, 2), 'lat': round(lat, 2),
                                 'wp': [round(wp[0], 2), round(wp[1], 2)], 'bearing': round(b, 1), 'stick': [sx, sy],
                                 'sprint': sprint, 'src': src})
            steps = {'name': 'approach', 'duration': dur + 0.15,
                     'ls': [[0, 0, 0], [0.05, sx, sy], [dur, sx, sy], [dur + 0.05, 0, 0]], 'metrics': False}
            if sprint:
                steps['buttons'] = [{'t': 0.1, 'button': 'LS', 'hold': 0.12}]
            start_xy = est
            self.play(pad, guard, 'veh_approach', plan, parse_scenario('veh_approach', {'steps': [steps]}))
            est = gn.dead_reckon(est, known['cf'], (sx, sy), dur, gn.speed_for(sx, sy, sprint, self.speeds))
            dr_s += dur
            i += 1
            new = self.poll_pos(guard, pad)
            if new and new.get('v') and new.get('src') != 'mounted':
                moved = math.hypot(new['p'][0] - start_xy[0], new['p'][1] - start_xy[1])
                stuck = src == 'pos' and dur >= 0.4 and moved < APPROACH_STUCK_M
                if src == 'pos' and dur >= 0.4 and not stuck:  # learn the speed of this stick class from real displacement
                    cls = gn.stick_class(sx, sy, sprint)
                    self.speeds[cls] = round(min(10.0, max(0.3, 0.5 * self.speeds[cls] + 0.5 * moved / dur)), 2)
                stuck_chunks = stuck_chunks + 1 if stuck else 0
                known, est, src, dr_s = new, tuple(new['p'][:2]), 'pos', 0.0
                if stuck_chunks >= APPROACH_STUCK_CHUNKS:
                    lon, lat = gn.car_frame(est, known['v'][:2], known['vr'])
                    if math.hypot(lon, lat) > MOUNT_MAX_DIST_M and sidesteps < APPROACH_SIDESTEPS:
                        # an obstacle on the way, far from the car (2026-11-12 12:54: stuck 32 m out, the approach
                        # ended and X was pressed there): step aside and go on, the path is re-planned from there
                        sidesteps += 1
                        self.log(f'[approach] stuck {math.hypot(lon, lat):.1f} m from the car -> step aside '
                                 f'({sidesteps}/{APPROACH_SIDESTEPS})')
                        ep['events'].append({'approach_sidestep': sidesteps, 'lon': round(lon, 2), 'lat': round(lat, 2)})
                        self.turn_camera_90(pad, guard, plan, known)
                        side = parse_scenario('veh_approach', {'steps': [
                            {'name': 'sidestep', 'duration': 1.8, 'ls': [[0, 0, 0], [0.05, 0, 1], [1.6, 0, 1], [1.65, 0, 0]],
                             'metrics': False}]})
                        self.play(pad, guard, 'veh_approach', plan, side)
                        new = self.poll_pos(guard, pad)
                        if new and new.get('v') and new.get('src') != 'mounted':
                            known, est = new, tuple(new['p'][:2])
                        stuck_chunks, aligns, last_wp = 0, 0, None
                        continue
                    if abs(lon) > 2.0:
                        self.blocked_end = math.copysign(1.0, lon)
                    self.log(f'[approach] stuck: V moved < {APPROACH_STUCK_M} m in {stuck_chunks} chunks at lon {lon:.2f} '
                             f'lat {lat:.2f} - ending the approach' + (f', end {self.blocked_end:+.0f} avoided next'
                                                                    if self.blocked_end else ''))
                    ep['events'].append({'approach_stuck': i, 'lon': round(lon, 2), 'lat': round(lat, 2)})
                    break
            else:
                src = 'dead-reckon (poll miss)'
                if dr_s >= self.args.dr_max:
                    self.log(f'[approach] {dr_s:.1f} s of dead reckoning -> waiting for a real POS')
                    new = self.poll_pos(guard, pad)
                    if new and new.get('v'):
                        known, est, src, dr_s = new, tuple(new['p'][:2]), 'pos', 0.0
        else:
            self.log(f'[approach] time box {box:.0f} s used up at dist {dist:.1f} m')
        ep['speeds'] = dict(self.speeds)
        ep['pos_stats'] = dict(self.pos_stats)
        ep['cet_stats'] = {k: list(v) for k, v in self.cet_stats.items()}
        if dist <= 3.0 and known.get('v'):
            known = self.camera_align(pad, guard, plan, dict(known, p=(est[0], est[1], known['p'][2])),
                                      known['v'][:2], ep, tol=self.args.face_tol, why='face-car')
        self.last_known = known
        return dist

    def door_spot(self) -> tuple:
        """Car-frame (lon, lat) to stand at for X: the spot where X last worked for this V body (MOUNT_SPOTS_FILE),
        else gn.DOOR_BY_BODY, else --door-lon / --door-lat. The body comes from the UTPP log of this run."""
        body = getattr(self, 'body', None)
        if body is None and self.utpp_log:
            body = self.body = gn.body_in_log(read_since(self.utpp_log, 0))
        spot = gn.load_mount_spots(MOUNT_SPOTS_FILE).get(body) or gn.DOOR_BY_BODY.get(body)
        return spot or (self.args.door_lon, self.args.door_lat)

    def pos_time(self, p: dict) -> float:
        """Sample time of a POS: the log timestamp of a NightAuto line, else now (monotonic)."""
        ts = p.get('ts')
        return ts.timestamp() if ts else time.monotonic()

    def wait_car_stopped(self, pad, guard, ep: dict, pos: dict) -> tuple[dict, bool]:
        """Stand still until the vehicle is STATIONARY (< --car-stop-speed m/s for --car-stop-hold s, from consecutive
        POS samples), max --car-stop-max s. Cycle 11: the summoned car was still parking while V ran to it."""
        if pad is not None:
            pad.release()
        samples: list = []
        t_start = time.monotonic()
        end = t_start + self.args.car_stop_max
        while True:
            p = self.poll_pos(guard, None)
            if p and p.get('v') and p.get('src') != 'mounted':
                t = self.pos_time(p)
                if not samples or t > samples[-1][0]:
                    samples.append((t, tuple(p['v'][:2])))
                pos = p
                still = gn.stationary_duration(samples, self.args.car_stop_speed)
                if still >= self.args.car_stop_hold:
                    d = math.hypot(pos['v'][0] - pos['p'][0], pos['v'][1] - pos['p'][1])
                    waited = time.monotonic() - t_start
                    self.log(f'[vehicle] car stopped after {waited:.1f} s at {d:.1f} m')
                    ep['car_stopped'] = {'after_s': round(waited, 1), 'dist_m': round(d, 1)}
                    return pos, True
            if time.monotonic() >= end:
                break
            guard.wait(0.5)
        self.log(f'[vehicle] car did NOT stop within {self.args.car_stop_max:.0f} s - approaching anyway')
        ep['car_stopped'] = None
        return pos, False

    def car_speed_now(self, guard, pad=None) -> tuple[float | None, dict | None]:
        """Vehicle speed from two POS samples >= 0.6 s apart (None = not measurable)."""
        a = self.poll_pos(guard, pad)
        guard.wait(0.6)
        b = self.poll_pos(guard, pad)
        if not (a and b and a.get('v') and b.get('v')):
            return None, b or a
        dt = self.pos_time(b) - self.pos_time(a)
        if dt <= 0:
            return None, b
        return math.hypot(b['v'][0] - a['v'][0], b['v'][1] - a['v'][1]) / dt, b

    def mount(self, pad, guard, plan: dict, pos: dict, ep: dict) -> datetime | None:
        """At the door, before each X: the car must be stationary (< --car-stop-speed m/s) and V's car-frame lat in
        [0.9, 1.7]; otherwise re-plan the approach from the current pose (max --mount-replans). X, wait --mount-wait s
        for 'keepTpp mount -> ride'/handover. Every X is pressed with the camera turned along the car (x-aim: the prompt needs
        the screen centre on the door, not on V); after a failed X V walks along the car (seek_door, gn.X_SEEK_LONS)
        and X is pressed again (--mount-tries presses in all); then keyboard F or the scripted mount."""
        x_tries = replans = 0
        while x_tries < self.args.mount_tries:
            speed, cur = self.car_speed_now(guard, pad)
            if cur and cur.get('v') and cur.get('src') != 'mounted':
                pos = cur
            lon, lat = gn.car_frame(pos['p'][:2], pos['v'][:2], pos['vr'])
            if cur is not None and cur.get('v') is not None and math.hypot(lon, lat) > MOUNT_MAX_DIST_M:
                self.log(f'[mount] V is {math.hypot(lon, lat):.1f} m from the car - no X from here')
                ep['events'].append({'mount_too_far': round(math.hypot(lon, lat), 1)})
                break
            moving = speed is not None and speed >= self.args.car_stop_speed
            measured = cur is not None and cur.get('v') is not None   # no fresh POS: cannot judge, press X
            if measured and (moving or not 0.9 <= lat <= 2.2) and replans < self.args.mount_replans:
                replans += 1
                why = f'car speed {speed:.2f} m/s' if moving else f'lat {lat:.2f} outside 0.9-2.2'
                self.log(f'[mount] {why} (lon {lon:.2f}) -> re-plan the approach ({replans}/{self.args.mount_replans})')
                ep['events'].append({'mount_replan': replans, 'speed': None if speed is None else round(speed, 2),
                                     'lon': round(lon, 2), 'lat': round(lat, 2)})
                if moving:
                    pos, _ = self.wait_car_stopped(pad, guard, ep, pos)
                self.approach(pad, guard, plan, pos, ep)
                pos = self.last_known or pos
                continue
            # The enter prompt needs the screen centre on the car, not on V: on 2026-11-12 10:29:54 (Male V, Quadra) the
            # prompt was up at lon -0.79 lat 1.85 with the camera looking ALONG the car, and it was gone once V stood
            # square to the door with the camera behind him (his body covered the centre). So the first X is pressed
            # from where V arrived, with the camera turned along the car; walking to the handle comes after a failed X.
            if measured:
                aim = gn.from_car_frame(lon + gn.X_AIM_AHEAD_M, gn.X_AIM_LAT_M, pos['v'][:2], pos['vr'])
                pos = self.camera_align(pad, guard, plan, pos, aim, ep, tol=self.args.face_tol, why='x-aim')
            x_tries += 1
            off = file_size(self.utpp_log)
            self.play(pad, guard, 'veh_enter', plan)
            if wait_for_log(self.utpp_log, off, gn.mount_seen, self.args.mount_wait, sleep=guard.wait,
                            clock=time.monotonic):
                ep['mounted_by'] = f'pad_x#{x_tries}'
                if measured and getattr(self, 'body', None):
                    gn.save_mount_spot(MOUNT_SPOTS_FILE, self.body, lon, lat)   # stand here first on the next run
                self.log(f'[mount] mounted by pad_x (try {x_tries}, carframe lon {lon:.2f} lat {lat:.2f})')
                return gn.mount_ts(read_since(self.utpp_log, off))
            self.log(f'[mount] X try {x_tries} at lon {lon:.2f} lat {lat:.2f}: no mount line')
            if x_tries < self.args.mount_tries:
                # search for the door along the car (gn.X_SEEK_LONS): the place of the prompt differs per car
                self.seek_door(pad, guard, plan, pos, ep, x_tries, gn.X_SEEK_LONS[(x_tries - 1) % len(gn.X_SEEK_LONS)])
        if not self.args.mount_try_f:
            self.log(f'[mount] no car? {x_tries} X presses at the door without a mount line - skipping further tries')
            ep['no_car_suspected'] = True
            return self.cet_mount(pad, guard, ep) if self.args.cet_mount else None
        off = file_size(self.utpp_log)
        self.tap_key(pad, guard, gn.SCAN_F, 0.1)
        if wait_for_log(self.utpp_log, off, gn.mount_seen, self.args.mount_wait, sleep=guard.wait,
                        clock=time.monotonic):
            ep['mounted_by'] = f'key_f#{x_tries + 1}'
            self.log('[mount] mounted by key F')
            return gn.mount_ts(read_since(self.utpp_log, off))
        self.log('[mount] no car? X and F gave no mount line')
        ep['no_car_suspected'] = True
        return self.cet_mount(pad, guard, ep) if self.args.cet_mount else None

    def fresh_pos(self, guard, pad, fallback: dict | None) -> dict | None:
        """A new POS with the vehicle (not mounted); `fallback` when none arrives (callers then cannot verify)."""
        cur = self.poll_pos(guard, pad)
        if (cur and cur.get('p') and cur.get('cf') and cur.get('v') and cur.get('vr')
                and cur.get('src') != 'mounted'):
            return cur
        return fallback

    def mount_pulse(self, pad, guard, plan: dict, cur: dict, goal, ep: dict, tag, mag: float, min_s: float) -> dict:
        """One stick pulse from `cur` (position + camera forward of a fresh POS) towards the world point `goal`."""
        sx, sy, dist = gn.approach_stick(cur['p'][:2], cur['cf'], goal, mag)
        speed = MOUNT_PULSE_MPS * mag / 0.45
        dur = min(MOUNT_PULSE_MAX_S, max(min_s, dist / speed))
        sc = parse_scenario('veh_nudge', {'steps': [
            {'name': 'nudge', 'duration': dur + 0.2, 'ls': [[0, 0, 0], [0.03, sx, sy], [dur, sx, sy],
                                                            [dur + 0.03, 0, 0]], 'metrics': False}]})
        self.play(pad, guard, 'veh_nudge', plan, sc)
        ev = {'mount_nudge': tag, 'stick': [sx, sy], 'dur': round(dur, 2), 'dist_m': round(dist, 2)}
        ep['events'].append(ev)
        return ev

    def seek_door(self, pad, guard, plan: dict, pos: dict, ep: dict, tag, goal_lon: float) -> dict:
        """Walk along the car to car-frame lon `goal_lon`, at least gn.X_SEEK_LAT_M from the centre line (off the body,
        not pressed into it), in up to 3 pulses each checked with a real POS. Returns the last known POS."""
        cur = self.fresh_pos(guard, pad, pos)
        for i in range(3):
            lon, lat = gn.car_frame(cur['p'][:2], cur['v'][:2], cur['vr'])
            goal_lat = max(lat, gn.X_SEEK_LAT_M)
            if math.hypot(lon - goal_lon, lat - goal_lat) <= gn.X_SEEK_TOL_M:
                break
            goal = gn.from_car_frame(goal_lon, goal_lat, cur['v'][:2], cur['vr'])
            ev = self.mount_pulse(pad, guard, plan, cur, goal, ep, f'{tag}:seek#{i + 1}', MOUNT_SEEK_STICK, 0.4)
            new = self.fresh_pos(guard, pad, None)
            if new is None:
                break
            ev['moved_m'] = round(math.hypot(new['p'][0] - cur['p'][0], new['p'][1] - cur['p'][1]), 2)
            nlon, nlat = gn.car_frame(new['p'][:2], new['v'][:2], new['vr'])
            self.log(f'[mount] {tag} seek lon {goal_lon:.1f} #{i + 1}: V moved {ev["moved_m"]:.2f} m -> lon {nlon:.2f} '
                     f'lat {nlat:.2f}')
            cur = new
        return cur

    def cet_mount(self, pad, guard, ep: dict) -> datetime | None:
        """LAST mount fallback (SCRIPTED): IMountingFacility.Mount from the console (gn.MOUNT_CMD, isInstant=false = enter
        animation; then gn.MOUNT_INSTANT_CMD). Mounted = the vehiclecam mount/ride/handover line or NightPos m=1 within
        --cet-mount-wait s. Records ep['mount'] = {'how': 'cet_mount', 'scripted': True, ...} on success."""
        log = self.utpp_log or gt_log.newest_log(self.args.log_dir)
        attempts = []
        for n, (name, cmd) in enumerate((('animated', gn.MOUNT_CMD), ('instant', gn.MOUNT_INSTANT_CMD)), start=1):
            at = datetime.now()
            off = file_size(log)
            self.log(f'[mount] cet_mount (SCRIPTED) try {n} {name} at {stamp(at)}')
            txt = self.console_verified(pad, guard, cmd, lambda t: '[night] MOUNT ok=' in t, 'mount')
            line = next((l for l in (txt or '').splitlines() if '[night] MOUNT' in l), '')
            info = line.split('[night]', 1)[-1].strip()[:160] if line else None
            attempts.append({'n': n, 'how': name, 'at': at, 'log': info})
            if 'MOUNT ok=true' not in line:
                if line:   # the Lua failed (no vehicle / API): the instant variant would fail the same way
                    break
                continue
            ts, end = None, time.monotonic() + self.args.cet_mount_wait
            while True:
                seen = read_since(log, off)
                if gn.mount_seen(seen):
                    ts = gn.mount_ts(seen) or datetime.now()
                    break
                pos = self.poll_pos(guard, pad)
                if pos and pos.get('mounted') is True:
                    ts = datetime.now()
                    break
                if time.monotonic() >= end:
                    break
                guard.wait(0.5)
            if ts:
                ep['mounted_by'] = f'cet_mount#{n} (scripted)'
                ep.pop('no_car_suspected', None)
                ep['mount'] = {'how': 'cet_mount', 'scripted': True, 'attempts': attempts}
                self.log(f'[mount] mounted by cet_mount try {n} ({name}, SCRIPTED)')
                return ts
            self.log(f'[mount] cet_mount try {n} ({name}): MOUNT ok=true but no mount evidence in '
                     f'{self.args.cet_mount_wait:.0f} s')
        ep['cet_mount_attempts'] = attempts
        self.log('[mount] cet_mount gave no mount')
        return None

    def log_tail(self, nbytes: int = 400000) -> str:
        log = self.utpp_log or gt_log.newest_log(self.args.log_dir)
        size = file_size(log)
        return read_since(log, max(0, size - nbytes)) if size > 0 else ''

    def camera_settled(self, guard, window: float = 1.0, max_jump: float = 0.05) -> bool | None:
        """Vehicle camera stable: the 'cam veh=' samples of the last `window` s move < max_jump m sample-to-sample.
        None = no samples (vehiclecam trace off)."""
        lines = gt_log.parse_text(self.log_tail(200000))
        cams = [l for l in lines if l.kind == 'CET' and 'cam veh=(' in l.raw]
        if not cams:
            return None
        t_end = cams[-1].ts
        recent = [l for l in cams if (t_end - l.ts).total_seconds() <= window]
        eps = gn.vehiclecam_episodes(recent, gap_s=window + 1.0)
        return bool(eps) and max(e['max_jump_m'] for e in eps) < max_jump

    def wait_seated(self, pad, guard, ep: dict | None = None) -> dict:
        """'Mounted' = SEATED (cycle 5: 'keepTpp mount -> ride' fired while V still walked around the car; she sat
        13 s later). Poll NightPos every --seat-poll s until m=1 (max --seat-timeout s); without an m= flag wait
        --seat-fallback s. Then --seat-settle s more, plus up to 3 x 1 s until the vehicle camera is stable."""
        res = {'start': datetime.now(), 'seated_at': None, 'polls': 0, 'pos': None, 'camera_stable': None}
        end = time.monotonic() + self.args.seat_timeout
        no_flag = 0
        while time.monotonic() < end:
            t0 = time.monotonic()
            pos = self.poll_pos(guard, pad)
            res['polls'] += 1
            if pos is not None:
                res['pos'] = pos
                if pos.get('mounted'):
                    res['seated_at'] = datetime.now()
                    break
                if pos.get('mounted') is None:
                    no_flag += 1
                    if no_flag >= 2:  # inline probe without m=: fixed wait instead
                        guard.wait(self.args.seat_fallback)
                        res['seated_at'], res['by'] = datetime.now(), 'fallback wait (no m= flag)'
                        break
            guard.wait(max(0.0, self.args.seat_poll - (time.monotonic() - t0)))
        if res['seated_at']:
            guard.wait(self.args.seat_settle)
            for _ in range(3):
                res['camera_stable'] = self.camera_settled(guard)
                if res['camera_stable'] is not False:
                    break
                guard.wait(1.0)
        self.log(f'[seat] ' + (f'seated after {(res["seated_at"] - res["start"]).total_seconds():.1f} s '
                                f'({res["polls"]} polls), camera stable {res["camera_stable"]}' if res['seated_at']
                                else f'NOT seated within {self.args.seat_timeout:.0f} s ({res["polls"]} polls)'))
        if ep is not None:
            ep['seat'] = {k: v for k, v in res.items() if k != 'pos'}
        return res

    def exit_vehicle(self, pad, guard, plan: dict | None, why: str) -> dict:
        """Leave the vehicle (seated V; the exit prompt is a HOLD progress bar - cycle 6: taps and 1.0 s holds were
        ignored). After --exit-pre s neutral: F hold 2.0 s -> F hold 3.0 s -> pad B hold 2.0 s -> pad B hold 3.0 s
        (pad tries only with a pad) -> F taps (--exit-taps). After each try: --exit-start-wait s for
        'keepTpp ride -> exit', then a NightPos re-check of m= (m=0 counts as exited). Then <= --exit-done-wait s for
        'exit pull: end' / 'keepTpp exit -> off'. Every try is logged with its timestamp."""
        res = {'why': why, 'path': None, 'exit_at': None, 'done_at': None, 'tries': []}
        if pad is not None:
            pad.release()
        guard.wait(self.args.exit_pre)
        tries = [('key_f_hold2', gn.SCAN_F, 2.0), ('key_f_hold3', gn.SCAN_F, 3.0)]
        if pad is not None and plan is not None:
            tries += [('pad_b_hold2', 'B', 2.0), ('pad_b_hold3', 'B', 3.0)]
        tries += [('key_f_tap', gn.SCAN_F, 0.1)] * self.args.exit_taps
        if getattr(self.args, 'background', False):   # keys are refused: pad holds, then the scripted unmount
            tries = [t for t in tries if not isinstance(t[1], int)]
        log = self.utpp_log or gt_log.newest_log(self.args.log_dir)
        off = file_size(log)
        for n, (name, key, hold) in enumerate(tries, start=1):
            at = datetime.now()
            if isinstance(key, int):
                self.tap_key(pad, guard, key, hold)
            else:
                sc = parse_scenario('veh_exit_hold', {'steps': [
                    {'name': 'exit', 'duration': hold + 0.4, 'ls': [[0, 0, 0]], 'rs': [[0, 0, 0]], 'lt': [[0, 0]],
                     'rt': [[0, 0]], 'buttons': [{'t': 0.2, 'button': key, 'hold': hold}], 'metrics': False}]})
                self.play(pad, guard, 'veh_exit_hold', plan, sc)
            res['tries'].append({'n': n, 'how': name, 'at': at})
            self.log(f'[exit] {why}: try {n} {name} at {stamp(at)}')
            wait = self.args.exit_tap_interval if name == 'key_f_tap' else self.args.exit_start_wait
            if wait_for_log(log, off, lambda t: gn.RIDE_EXIT in t or gn.exit_done(t), wait,
                            sleep=guard.wait, clock=time.monotonic):
                res['path'] = f'{name}#{n}'
                res['exit_at'] = gn.find_line_ts(read_since(log, off), gn.RIDE_EXIT)
                break
            pos = self.poll_pos(guard, pad)
            if pos and pos.get('mounted') is False:
                res['path'] = f'{name}#{n} (NightPos m=0)'
                res['exit_at'] = datetime.now()
                break
        if not res['path'] and self.args.cet_unmount:
            # LAST fallback: the game's own ExitWithTeleport sequence from the console (no exit animation)
            at = datetime.now()
            n = len(tries) + 1
            res['tries'].append({'n': n, 'how': 'cet_unmount', 'at': at})
            self.log(f'[exit] {why}: try {n} cet_unmount (SCRIPTED) at {stamp(at)}')
            txt = self.console_verified(pad, guard, gn.UNMOUNT_CMD, lambda t: '[night] UNMOUNT ok=' in t, 'unmount')
            line = next((l for l in (txt or '').splitlines() if '[night] UNMOUNT' in l), '')
            res['unmount_log'] = line.split('[night]', 1)[-1].strip()[:160] if line else None
            guard.wait(1.0)
            pos = self.poll_pos(guard, pad)
            if 'UNMOUNT ok=true' in line and (pos is None or pos.get('mounted') is not True):
                res['path'], res['exit_at'], res['scripted'] = f'cet_unmount#{n} (scripted)', at, True
        if res['path']:
            if wait_for_log(log, off, gn.exit_done, self.args.exit_done_wait, sleep=guard.wait, clock=time.monotonic):
                txt = read_since(log, off)
                res['done_at'] = next((gn.find_line_ts(txt, p) for p in gn.EXIT_DONE if p in txt), None)
        self.log(f'[exit] {why}: ' + (f'started by {res["path"]} at {stamp(res["exit_at"])}, done {stamp(res["done_at"])}'
                                       if res['path'] else f'NO exit after {len(tries)} tries'))
        self.cycle.setdefault('exits', []).append(res)
        return res

    def ensure_unmounted(self, pad, guard, plan: dict | None, why: str, pos: dict | None = None) -> bool | None:
        """Forced exit with the same procedure: when V is mounted (NightPos m=1 or vehiclecam phase mount/ride), wait
        until seated (an enter animation may still run), then exit_vehicle. Returns the state before (None =
        unknown)."""
        if pos is None and self.auto_pos:   # the newest NightAuto line costs nothing and carries the m= flag
            pos = gn.latest_pos(self.log_tail(60000), datetime.now(), self.args.pos_max_age)
        state = gn.mounted_state(pos, self.log_tail())
        if state:
            self.wait_seated(pad, guard)
            self.exit_vehicle(pad, guard, plan, why)
        return state

    def on_foot_first(self, pad, guard, plan: dict | None, name: str) -> bool:
        """A walking scenario needs V on foot: when the newest NightAuto line says m=1, leave the vehicle before the
        scenario instead of playing it seated (every attempt would only be reported as blocked). True = left one."""
        if not self.auto_pos:
            return False
        pos = gn.latest_pos(self.log_tail(60000), datetime.now(), self.args.pos_max_age)
        if not (pos and pos.get('mounted')):
            return False
        self.log(f'[scenarios] {name}: V sits in a vehicle (NightPos m=1) -> leaving it first')
        try:
            self.ensure_unmounted(pad, guard, plan, f'before {name}', pos=pos)
        except NeedsKeyboard as e:
            self.log(f'[scenarios] {name}: vehicle exit skipped: {e}')
            return False
        self.left_vehicle_this_cycle = True
        return True

    def vehicle_macro(self, pad, guard, plan: dict) -> dict:
        out = []
        self.probe_cache.invalidate('vehicle scenario')
        self.recover(pad, guard, plan, 'before vehicle_mount_exit', probe=True)
        for rep in range(1, self.args.vehicle_repeats + 1):
            self.check_continue()
            ep = {'name': f'vehicle_mount_exit#{rep}', 'start': datetime.now(), 'events': []}
            self.blocked_end = 0.0
            out.append(ep)
            if self.ensure_unmounted(pad, guard, plan, f'before attempt {rep}'):
                ep['events'].append({'forced_exit_before': True})
            pos = self.locate_vehicle(pad, guard, plan, ep)
            if not pos:
                ep['error'], ep['end'] = 'no vehicle located by any spawn mode', datetime.now()
                self.log(f'[vehicle] #{rep}: {ep["error"]}')
                continue
            pos, _ = self.wait_car_stopped(pad, guard, ep, pos)   # the summoned car parks by itself first
            ep['approach_dist_m'] = round(self.approach(pad, guard, plan, pos, ep), 2)
            last = self.last_known or pos
            if not last.get('v'):
                last = dict(last, v=pos['v'], vr=pos['vr'])
            mounted = self.mount(pad, guard, plan, last, ep)
            ep['mounted'] = mounted
            if not mounted:
                ep['error'] = (f'mount not detected (X/F x{self.args.mount_tries}, door distance '
                               f'{ep["approach_dist_m"]} m)')
                ep['end'] = datetime.now()
                self.log(f'[vehicle] #{rep}: {ep["error"]}')
                continue
            if (ep.get('mount') or {}).get('scripted'):
                ep['mount_scripted'] = True
                ep.setdefault('warnings', []).append('mount was SCRIPTED (cet_mount): no game mount input worked')
            seat = self.wait_seated(pad, guard, ep)
            if not seat['seated_at']:
                ep['error'] = f'mount started but V not seated within {self.args.seat_timeout:.0f} s (NightPos m=1)'
                self.log(f'[vehicle] #{rep}: {ep["error"]}')
            else:
                before = seat.get('pos') or {}
                self.play(pad, guard, 'veh_drive', plan)
                after = self.poll_pos(guard, pad)
                if before.get('v') and after and after.get('v'):
                    moved = math.hypot(after['v'][0] - before['v'][0], after['v'][1] - before['v'][1])
                    ep['drive_moved_m'] = round(moved, 2)
                    self.log(f'[drive] vehicle moved {moved:.2f} m' + ('' if moved > 1.0 else ' (NOT driven: <= 1 m)'))
                    if moved <= 1.0:
                        ep.setdefault('warnings', []).append(f'drive: vehicle moved only {moved:.2f} m')
                else:
                    self.log('[drive] vehicle movement not measurable (no vehicle position before/after)')
            ex = self.exit_vehicle(pad, guard, plan, f'attempt {rep}')
            ep['exit'] = dict(ex)
            ep['exit_pull_end'] = ex['done_at']
            if ex.get('scripted'):
                ep['exit_scripted'] = True
                ep.setdefault('warnings', []).append('exit was SCRIPTED (cet_unmount): no game exit input worked')
                if not ex['done_at']:
                    ep['exit_pull_end'] = ex['exit_at']   # no vehiclecam exit lines after an instant unmount
            if not ex['path']:
                ep['error'] = (ep.get('error', '') + '; ' if ep.get('error') else '') + \
                    f'exit not started (no "keepTpp ride -> exit" after {len(ex["tries"])} tries: F taps, F hold, B hold)'
            if ex['path'] or ex.get('scripted'):
                # V just left this car: the next attempt's summon finds it parked next to V (2.0 m, Male V 16:30) and the
                # game does not drive it, so wait_arrival uses it like the car V left at the start of the cycle
                self.left_vehicle_this_cycle = True
            guard.wait(3.0)
            ep['end'] = datetime.now()
            self.log(f'[vehicle] #{rep}: mounted {stamp(mounted)} ({ep.get("mounted_by")}), exit pull end '
                     f'{stamp(ep["exit_pull_end"])}')
        self.ensure_unmounted(pad, guard, plan, 'end of vehicle_mount_exit')
        self.console(pad, guard, [gn.DESPAWN_CMD])
        fails = [e.get('error') for e in out if e.get('error')] + \
            [f'{e["name"]}: exit started but no "exit pull: end" / "keepTpp exit -> off"' for e in out
             if e.get('mounted') and not e.get('error') and not e.get('exit_pull_end')]
        return {'episodes': out, 'verdict': 'FAIL' if fails else 'PASS', 'fails': fails}

    def scenario_needs_keyboard(self, name: str) -> bool:
        """Scenarios that always press keys (skipped with --background before they start; any other key press of a
        scenario is refused at the api and skips it the same way)."""
        if name == 'shoulder_key':
            return True
        sc = self.scenarios.get(name)
        return sc is not None and self.args.crouch_button == 'key' and bool(gn.crouch_toggle_steps(sc))

    def live_invalid_cause(self, res: dict, log_off: int) -> str | None:
        """The cause for which run.py will score the steps of a walking scenario INVALID (the same checks on the log slice of
        the scenario: no motion lines, or the stick pushed on every step and V never moved), None = valid or no way to tell."""
        try:
            steps = [st for st in (res.get('steps') or []) if st.get('start') and st.get('end') and st.get('metrics', True)]
            if not steps or not self.utpp_log:
                return None
            text = read_since(self.utpp_log, log_off)
            lines = gt_log.parse_text(text)
            pre = gt_log.motion_precondition(lines, steps[0]['start'], steps[-1]['end'])
            if not pre['ok']:
                return 'mm.motionDriver not active: no "UTPP MOTION" lines in the log while the scenario ran'
            pushed = blocked = 0
            for st in steps:
                m = gt_log.compute_metrics(lines, st['start'], st['end'])
                pushed += (m.get('pushed_ticks') or 0) >= gt_run.BLOCKED_MIN_TICKS
                blocked += bool(gt_run.step_blocked(m))
            if blocked and blocked == pushed:
                return 'V does not move although the stick is pushed' + (
                    ' (V is in a vehicle)' if gn.mounted_state(None, self.log_tail()) else '')
        except Exception as e:  # noqa: BLE001 - the check is an optimisation, never a reason to fail
            self.log(f'[early-stop] check failed: {e!r}')
        return None

    def stop_early(self, why: str, name: str, run: int) -> None:
        names = [(n, r) for n in self.names for r in range(1, self.args.repeat + 1)]
        rest = names[names.index((name, run)) + 1:]
        self.cycle['skipped_scenarios'] = [{'name': n, 'run': r, 'reason': why} for n, r in rest]
        self.log(f'[early-stop] {why} -> the cycle stops; {len(rest)} scenarios not played')
        raise EarlyStop(f'early stop: {why}')

    def run_scenarios(self, plan: dict) -> list[dict]:
        st = self.step('scenarios')
        records = []
        guard = self.guard()
        pad = None
        self.start_load()
        try:
            guard.ensure_focus()
            pad, fresh = gt_run.open_pad(self.run_args, guard)
            if fresh:
                guard.wait(self.args.pad_settle)
            pad.wake(self.args.wake)
            self.touched()
            # the save may start in a vehicle: on-foot scenarios need V on foot first
            self.left_vehicle_this_cycle = False
            try:
                if self.ensure_unmounted(pad, guard, plan, 'before the scenarios'):
                    self.left_vehicle_this_cycle = True
                    self.log('[scenarios] V was mounted at the start -> left the vehicle first')
            except NeedsKeyboard as e:
                self.log(f'[scenarios] vehicle exit before the scenarios skipped: {e}')
            self.level_camera(pad, guard, plan)
            with TimerResolution():
                for name in self.names:
                    for r in range(1, self.args.repeat + 1):
                        self.check_continue()
                        t_scen = time.monotonic()
                        self.budget.begin(f'{name}#{r}')
                        live = None            # a walking scenario that ran: (play result, log offset) for the early-stop check
                        with self.budget.timed('wait'):
                            guard.wait(self.args.gap)
                        t0 = datetime.now()
                        self.scen_log_off = file_size(self.utpp_log)
                        limit = self.args.scenario_timeout
                        if name in self.scenarios:
                            limit = max(limit, self.scenarios[name].duration * 2 + 30)
                        if name == 'melee_fists_target':     # ~2 min: spawn polls, 4 phases, console commands
                            limit = max(limit, 300.0)
                        if name in gw.WEAPON_SETS:
                            limit = max(limit, gw.time_limit_s(name))
                        self.scenario_deadline = time.monotonic() + limit
                        self.log(f'[scenarios] {name} run {r}')
                        rec = {'name': name, 'run': r, 'start': t0}
                        try:
                            if getattr(self.args, 'background', False) and self.scenario_needs_keyboard(name):
                                raise NeedsKeyboard(f'{name} uses keyboard input')
                            if name == 'vehicle_mount_exit':
                                rec.update(self.vehicle_macro(pad, guard, plan))
                                self.leave_car(pad, guard, plan)
                            elif name == 'weapon_draw_aim_holster' or name in gn.FIRING_MACROS:
                                rec.update(self.weapon_macro(pad, guard, plan, fire=name in gn.FIRING_MACROS))
                            elif name == 'melee_fists':
                                rec.update(self.melee_macro(pad, guard, plan))
                            elif name in gw.WEAPON_SETS:
                                gw.run_weapon_macro(self, pad, guard, plan, name, rec)   # fills rec while it runs
                            elif name == 'melee_fists_target':
                                rec.update(self.enemy_macro(pad, guard, plan))
                            elif name == 'shoulder_key':
                                rec.update(self.shoulder_key_macro(pad, guard))
                            elif name == 'aim_probe':
                                rec.update(self.aim_probe_macro(pad, guard, plan))
                            else:
                                sc = self.scenarios[name]
                                key_crouch = self.args.crouch_button == 'key' and bool(gn.crouch_toggle_steps(sc))
                                walks = bool(gn.stick_windows(sc))
                                if walks:   # open ground first, one repeat when V is blocked (play_walking)
                                    walked = self.play_walking(pad, guard, plan, name, key_crouch=key_crouch)
                                    res, cfails, log_off = walked['res'], walked['cfails'], walked['off']
                                    if walked['discarded']:   # the cycle record describes the kept attempt only
                                        rec['start'] = walked['start']
                                        rec['blocked_attempt'] = walked['discarded']
                                    live = (res, log_off)
                                elif key_crouch:
                                    res, cfails = self.play_with_key_crouch(pad, guard, name, plan)
                                else:
                                    res, cfails = self.play(pad, guard, name, plan), []
                                if cfails:
                                    rec['verdict'], rec['fails'] = 'FAIL', cfails
                                if walks:
                                    rec['recovery'] = self.recover(pad, guard, plan, name, sc, res, log_off)
                        except NeedsKeyboard as e:   # --background: keyboard / mouse input is refused
                            rec['verdict'], rec['skip_reason'] = 'SKIP', 'needs keyboard'
                            self.log(f'[scenarios] {name}: skipped, needs keyboard ({e})')
                            pad.release()
                        except ScenarioTimeout as e:
                            rec['verdict'], rec['fails'] = 'TIMEOUT', [str(e)]
                            self.log(f'[scenarios] {name}: {e}')
                            self.scenario_deadline = None
                            pad.release()
                            self.recover(pad, guard, plan, f'after timeout of {name}', probe=True)
                        finally:
                            self.scenario_deadline = None
                        rec['end'] = datetime.now()
                        records.append(rec)
                        self.cycle['scenarios'] = records
                        self.save()
                        with self.budget.timed('copy'):
                            self.secure_session_files('between scenarios')   # the ring keeps ten minutes: finished segments go now
                        self.budget.end(time.monotonic() - t_scen)
                        if live is not None and rec.get('verdict') not in ('SKIP', 'TIMEOUT') and self.early.watching:
                            why = self.early.note(f'{name}#{r}', self.live_invalid_cause(live[0], live[1]))
                            if why:
                                self.stop_early(why, name, r)
        except GuardError as e:
            self.done(st, 'failed', error=str(e))
            raise (StopNight if is_user_stop(str(e)) else CycleError)(str(e))
        except (StopNight, CycleError) as e:
            self.done(st, 'failed', error=str(e))
            raise
        finally:
            if self.budget.current:
                self.budget.end(0.0)
            if pad is not None:
                pad.close()  # padd: pad stays plugged (neutral); unplugged after the game is closed
            # every way out of the scenario phase (end, stop file, abort key, exception, timeout): the recording first,
            # before anything slow happens and before the game is quit
            self.secure_session_files('scenarios end', final=True)
            self.stop_load()
        self.done(st, count=len(records))
        return records

    # -- 8: collect --
    def post_workers(self, njobs: int) -> int:
        """Processes for the sheets and the clips: --post-workers, else the idle cores (gt_post.pool_size)."""
        n = int(getattr(self.args, 'post_workers', 0) or 0)
        return n if n > 0 else gt_post.pool_size(njobs)

    def write_summary(self, summary: dict) -> None:
        with open(os.path.join(self.cycle_dir, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(gt_run.serialise_plan(summary), f, indent=1, default=str)

    def sheet_plan(self, rec: dict, utpp_copy: str) -> list[dict]:
        ck = rec['clock']
        scs = [{'name': f'{s["name"]}_r{s["run"]}', 'start': s['start'], 'end': s.get('end')}
               for s in self.cycle.get('scenarios', [])]
        return gt_post.plan_sheets(rec['path'], utpp_copy, stamp(ck['video_end']), ck['clock_offset_s'], self.args.ffmpeg,
                                   ck['video_start'], ck['duration_s'], scs, self.args.sheet_pad,
                                   os.path.join(self.cycle_dir, 'sheets'), self.args.keep_sheet_frames)

    def start_sheets(self, jobs: list[dict]):
        """The contact sheets in a thread that drives the process pool; returns (thread, result dict filled when it ends)."""
        import threading
        out: dict = {}
        sd = os.path.join(self.cycle_dir, 'sheets')

        def work():
            try:
                res, timing = gt_post.run_sheets(jobs, sd, self.post_workers(len(jobs)))
                out.update(result=res, timing=timing)
            except Exception as e:  # noqa: BLE001
                out.update(result={'error': repr(e)}, timing=None)
        th = threading.Thread(target=work, name='sheets', daemon=True)
        th.start()
        return th, out

    def pending_media(self, rec: dict, clips: bool = True) -> dict:
        """--fast: what was not produced, with video seconds, and the commands that make it later."""
        run = os.path.abspath(self.cycle_dir)
        ck = rec['clock']
        pend: dict = {'sheets': [{'sheet': 'full', 'video_s': [0.0, round(ck['duration_s'], 1)]}], 'clips': [], 'commands': {
            'sheets': f'python -I "{os.path.join(HERE, "gt_post.py")}" sheets "{run}" [--only <sheet>]'}}
        for w in gn.sheet_windows(ck['video_start'], ck['duration_s'],
                                  [{'name': f'{s["name"]}_r{s["run"]}', 'start': s['start'], 'end': s.get('end')}
                                   for s in self.cycle.get('scenarios', [])], self.args.sheet_pad):
            pend['sheets'].append({'sheet': w['name'], 'video_s': [w['start'], w['end']]})
        if clips:
            pend['commands']['clips'] = (f'python -I "{os.path.join(HERE, "failure_clips.py")}" "{run}" --per-cause 0 '
                                         '[--name <text>]   (add --list to see them first)')
            try:
                _, items, _ = failure_clips.plan_clips(self.cycle_dir, per_cause=0)
                pend['clips'] = [{'name': it['name'], 'verdict': it['verdict'],
                                  'video_s': [round(it['start'], 1), round(it['end'], 1)], 'cause': it['cause']} for it in items]
            except (SystemExit, Exception) as e:  # noqa: BLE001
                pend['clips_error'] = repr(e)
        return pend

    def collect(self, rec: dict | None, plan: dict) -> dict:
        st = self.step('collect')
        fast = bool(getattr(self.args, 'fast', False))
        ld = os.path.join(self.cycle_dir, 'logs')
        os.makedirs(ld, exist_ok=True)
        with self.budget.phase('copy session files'):
            self.secure_session_files('collect', final=True)        # after the quit: the last pose block was flushed on close
        self.start_pose_meter(plan, ld)
        copied = []
        t_cycle = self.cycle['start'].timestamp() - 5
        srcs = gt_log.session_files(self.utpp_log) if self.utpp_log else []   # current file + rotated .N.log siblings
        srcs += [p for p in glob.glob(os.path.join(self.args.log_dir, 'red4ext-*.log')) if os.path.getmtime(p) >= t_cycle]
        srcs += [os.path.join(CET_DIR, n) for n in ('cyber_engine_tweaks.log', 'scripting.log')]
        srcs += glob.glob(os.path.join(CET_DIR, 'mods', 'utpp_*', '*.log'))
        with self.budget.phase('copy logs'):
            for s in srcs:
                if s and os.path.isfile(s):
                    rel = os.path.basename(os.path.dirname(s)) + '__' + os.path.basename(s) if 'mods' in s else \
                        os.path.basename(s)
                    shutil.copy2(s, os.path.join(ld, rel))
                    copied.append(rel)
        utpp_copy = os.path.join(ld, os.path.basename(self.utpp_log)) if self.utpp_log else None
        summary = {'cycle': self.cycle['name'], 'result': self.cycle.get('result'), 'utpp_log': utpp_copy,
                   'body': self.body, 'copied': copied,
                   'scenario_verdicts': {f'{s["name"]}#{s["run"]}': {'verdict': s.get('verdict'),
                                                                      'fails': s.get('fails'),
                                                                      **({'aimprobe': s['aimprobe']}
                                                                         if 'aimprobe' in s else {}),
                                                                      **({'enemy': s['enemy']}
                                                                         if 'enemy' in s else {})}
                                         for s in self.cycle.get('scenarios', []) if s.get('verdict')},
                   'recoveries': self.cycle.get('recoveries', [])}
        if self.cycle.get('skipped_scenarios'):
            summary['skipped_scenarios'] = self.cycle['skipped_scenarios']
        # recording: the recorder was ended before the quit; its check (length, size, clock) is quick, the full decode runs in
        # the background (also from a recorder that died with the game: the fragmented mp4 stays decodable)
        if rec:
            try:
                if not rec.get('stopped'):
                    self.stop_recording(rec, check=False)
                if not rec.get('finalized'):
                    with self.budget.phase('recording check'):
                        self.check_recording(rec, deep=False)
                    self.start_deep_check(rec)
            except Exception as e:  # noqa: BLE001
                self.log(f'[collect] recording check failed: {e!r}')
            summary['recording'] = {'path': rec['path'], 'mode': rec.get('mode'), 'stop': rec.get('stop'),
                                    'stop_seconds': rec.get('stop_seconds'), 'check': rec.get('check')}
        # contact sheets: they need the recording clock and the log copy only, so they start now and run while the rest goes on
        sheets_th, sheets_out, sheet_jobs = None, {}, None
        if rec and rec.get('clock') and utpp_copy:
            if rec.get('duration_source') in ('progress', 'wall time'):
                self.finish_deep_check(rec)          # the length was only estimated: the decoded one first
            summary['clock'] = rec['clock']
            if not self.args.no_sheets and not fast:
                sheet_jobs = self.sheet_plan(rec, utpp_copy)
                sheets_th, sheets_out = self.start_sheets(sheet_jobs)
        # harness analysis (run.py)
        harness_steps: list = []
        if plan['runs'] and utpp_copy:
            gdir = os.path.join(self.cycle_dir, 'gametest')
            os.makedirs(gdir, exist_ok=True)
            plan_s = dict(plan, log=utpp_copy, log_offset=0, created=datetime.now())
            with open(os.path.join(gdir, 'plan.json'), 'w', encoding='utf-8') as f:
                json.dump(gt_run.serialise_plan(plan_s), f, indent=1, default=str)
            if self.meter_job is not None:
                try:
                    started = self.meter_job.start_helper_reports(os.path.join(gdir, 'plan.json'))
                    if started:
                        self.log(f'[pose] helper reports started: {", ".join(started)}')
                except Exception as e:  # noqa: BLE001
                    self.log(f'[pose] helper reports not started: {e!r}')
            ra = gt_run.build_parser().parse_args(['--scenario-file', self.args.scenario_file, '--thresholds',
                                                   self.args.thresholds, '--clock-offset', '0'])
            with self.budget.phase('harness analysis'):
                try:
                    gt_run.analyze_dir(gdir, ra, log_override=utpp_copy)
                    with open(os.path.join(gdir, 'summary.json'), encoding='utf-8') as f:
                        h = json.load(f)
                    summary['harness'] = {k: h.get(k) for k in ('overall', 'counts', 'aborted_runs', 'cet_commands')}
                    harness_steps = h.get('steps', [])
                except Exception as e:  # noqa: BLE001
                    summary['harness'] = {'error': repr(e)}
        # failures at a glance: test mechanics (crouch/mount/weapon/timeouts) vs locomotion metrics
        mech, metr = gn.split_failures(harness_steps, self.cycle.get('scenarios', []))
        summary['mechanics_failures'] = mech
        summary['metric_failures'] = metr
        summary['failure_counts'] = {'mechanics': len(mech), 'metrics': len(metr)}
        summary['overlay'] = dict(self.overlay_stats)
        summary['night_auto'] = self.auto_pos
        if self.cycle.get('blocked_repeats'):   # attempts that were not scored (V stuck), see plan.json discarded_runs
            summary['blocked_repeats'] = self.cycle['blocked_repeats']
        summary['console'] = {k: {'ok_first': v.count(1), 'ok_retry': sum(1 for x in v if x > 1),
                                  'lost': v.count(0)} for k, v in self.cet_stats.items()}
        # night metrics
        if utpp_copy:
            lines = gt_log.parse_file(utpp_copy)
            scs = self.cycle.get('scenarios', [])
            a = scs[0]['start'] if scs else None
            b = scs[-1]['end'] if scs else None
            summary['night_metrics'] = gn.night_metrics(lines, a, b)
            summary['per_scenario'] = {f'{s["name"]}#{s["run"]}': gn.night_metrics(lines, s['start'], s['end'])
                                       for s in scs if s.get('end')}
        self.write_summary(summary)
        # failure clips (the run's recording is still in the run folder; the media move below comes after them)
        clip_info: dict = {}
        if fast and rec and rec.get('clock'):
            summary['pending_media'] = self.pending_media(rec, clips=not self.args.no_failure_clips)
            self.log(f'[post] --fast: {len(summary["pending_media"]["sheets"])} sheets and '
                     f'{len(summary["pending_media"]["clips"])} failure clips not made; the commands are in summary.json '
                     '(pending_media)')
        elif rec and rec.get('clock') and not self.args.no_failure_clips:
            # 1x clips of the failing steps / scenarios, into Media\mp4 (read from the summary.json written above); the
            # cuts run in the process pool, at most --clips-per-cause of each cause
            dropped: list = []

            def cutter(video, jobs):
                t0 = time.monotonic()
                w = self.post_workers(len(jobs))
                res = gt_post.cut_pool(video, jobs, self.args.ffmpeg, True, w,
                                       scratch=os.path.join(self.cycle_dir, '_tmp', 'clips'))
                clip_info.update(workers=w, jobs=len(jobs), seconds=round(time.monotonic() - t0, 1))
                return res
            with self.budget.phase('failure clips'):
                try:
                    clips = failure_clips.make_clips(self.cycle_dir, ffmpeg=self.args.ffmpeg,
                                                     per_cause=self.args.clips_per_cause, cutter=cutter, dropped_out=dropped)
                    summary['failure_clips'] = [{k: c[k] for k in ('path', 'name', 'verdict', 'why')} for c in clips]
                    for c in clips:
                        self.log(f'[clips] {c["verdict"]} {c["name"]}: {c["path"]}')
                    if dropped:
                        summary['failure_clips_not_cut'] = dropped
                        for d in dropped:
                            self.log(f'[clips] {len(d["dropped"])} more of the cause {d["cause"]!r} not cut (limit '
                                     f'{self.args.clips_per_cause} per cause; failure_clips.py <run> --per-cause 0 cuts them)')
                except (SystemExit, Exception) as e:  # noqa: BLE001
                    summary['failure_clips'] = f'error: {e!r}'
                    self.log(f'[clips] failed: {e!r}')
            if clip_info:
                self.budget.notes['failure clips'] = f'{clip_info["jobs"]} clips, {clip_info["workers"]} processes'
        # the pose meter workers have been running since the start of the collect
        with self.budget.phase('pose meter wait'):
            self.finish_pose_meter(summary)
        if sheets_th is not None:
            with self.budget.phase('sheets (wait)'):
                sheets_th.join()
            summary['sheets'] = sheets_out.get('result')
            tm = sheets_out.get('timing')
            if tm:
                summary['sheets_timing'] = tm
                self.budget.notes['sheets (wait)'] = (f'{tm["jobs"]} jobs, {tm["workers"]} processes, {tm["seconds"]} s wall, '
                                                      f'{tm["busy_s"]} s of work')
        elif fast and rec and rec.get('clock'):
            summary['sheets'] = 'not made (--fast): see pending_media'
        elif self.args.no_sheets and rec and rec.get('clock'):
            summary['sheets'] = {}
        with self.budget.phase('full decode (wait)'):
            self.finish_deep_check(rec)
        if rec and rec.get('clock'):
            summary['clock'] = rec['clock']
        summary['recordings'] = self.collector.summary() if self.collector else None
        if getattr(self.args, 'background', False):
            arm = next((x for x in self.cycle['steps'] if x['name'] == 'arm'), {})
            summary['background'] = {
                'min_idle_s': gt_session.effective_min_idle(self.args), 'command_channel': self.channel_mode or self.args.cmd_channel,
                'pad_plugin_answer': (arm.get('bg_input') or {}).get('result'), 'recorder': (rec or {}).get('mode'),
                'recorder_warning': (rec or {}).get('warning'),
                'keyboard_skips': [f'{s["name"]}#{s["run"]}' for s in self.cycle.get('scenarios', [])
                                   if s.get('skip_reason') == 'needs keyboard']}
        summary['machine_load'] = self.cycle.get('machine_load')
        summary['settings'] = {'ledger': self.settings.record(), 'left': self.cycle.get('settings_left'),
                               'pending_restored_at_launch': self.cycle.get('pending_settings_restore'),
                               'unowned': self.cycle.get('unowned_settings')}
        self.write_summary(summary)
        if not self.args.no_media_move:
            # Media rule: the recording and the sheet images go straight to Media\mp4 / Media\png (after the clips,
            # which read the recording from the summary written above). A killed cycle moves what exists; a failed
            # move is logged and never fails the cycle.
            with self.budget.phase('media move'):
                try:
                    summary['media'] = failure_clips.move_to_media(self.cycle_dir, summary, log=self.log)
                    self.cycle['media'] = summary['media']
                    self.save()
                except Exception as e:  # noqa: BLE001
                    summary['media'] = {'errors': [repr(e)]}
                    self.log(f'[media] move failed: {e!r}')
            self.write_summary(summary)
        self.done(st, copied=len(copied))
        return summary

    # -- pose meter (gt_posemeter) --
    def start_pose_meter(self, plan: dict, logs_dir: str) -> None:
        """Worker processes over the collected recording; returns at once (never delays the work that follows)."""
        self.meter_job = None
        if getattr(self.args, 'no_pose_meter', False):
            return
        files = gc.pose_files(logs_dir)
        if not files:
            self.note_cycle('pose_meter', {'status': 'no pose recording was collected (diagnostic logs off?)'})
            self.log('[pose] no pose recording in the run folder: the meter did not run (is --diag on, or the diagnostic logs on?)')
            return
        try:
            windows = gp.build_windows(self.cycle.get('scenarios'), plan.get('runs'))
            self.meter_job = gp.MeterJob(os.path.join(self.cycle_dir, '_tmp', 'pose_meter'), files, windows).start()
            self.log(f'[pose] meter started in worker processes: {len(files)} recording files, {len(windows)} windows')
        except Exception as e:  # noqa: BLE001
            self.meter_job = None
            self.note_cycle('pose_meter', {'status': f'not started: {e!r}'})
            self.log(f'[pose] meter not started: {e!r}')

    def finish_pose_meter(self, summary: dict) -> None:
        job = self.meter_job
        if job is None:
            if self.cycle.get('pose_meter'):
                summary['pose_meter'] = self.cycle['pose_meter']
            return
        self.meter_job = None
        try:
            out = job.finish(getattr(self.args, 'pose_timeout', 900.0))
            res = gp.assemble(self.cycle['name'], out, job.windows,
                              self.collector.summary() if self.collector else None, self.cycle.get('machine_load'))
            gp.write_outputs(self.cycle_dir, res)
            lines = gp.scenario_lines(res)
            for ln in lines:
                self.log('[pose] ' + ln)
            for e in res['errors']:
                self.log(f'[pose] ERROR {e}')
            rec = {'file': 'pose_metrics.json', 'table': 'pose_table.txt', 'lines': lines, 'errors': res['errors'],
                   'windows_without_result': res['windows_without_result'], 'helper_reports': res['helper_reports']}
            want = gt_session.POSE_BODY.get(self.body) if self.body else None
            if want and res.get('body') and res['body'] != want:
                msg = f'--body {self.body} asked for the {want} body but the pose recording is of the {res["body"]} body'
                self.log(f'[pose] !!! BODY MISMATCH: {msg}')
                rec['body_mismatch'] = summary['body_mismatch'] = self.cycle['body_mismatch'] = msg
            elif want and not res.get('body'):
                self.log(f'[pose] the body of this run (--body {self.body}) could not be checked: the pose recording names none')
            base = None
            base_body = gp.load_body(self.args.pose_baseline) if getattr(self.args, 'pose_baseline', '') else None
            if base_body and res.get('body') and base_body != res['body']:
                rec['baseline_skipped'] = f'baseline recorded on {base_body}, this run is {res["body"]}'
                self.log(f'[pose] baseline {self.args.pose_baseline} not used: {rec["baseline_skipped"]}')
            elif getattr(self.args, 'pose_baseline', ''):
                try:
                    base = gp.load_rows(self.args.pose_baseline)
                    text = gp.format_comparison(gp.compare_rows(res['rows'], base))
                    with open(os.path.join(self.cycle_dir, 'pose_compare.txt'), 'w', encoding='utf-8') as f:
                        f.write('\n'.join(text) + '\n')
                    for ln in text:
                        self.log('[pose] vs baseline ' + ln.strip())
                    rec['baseline'] = self.args.pose_baseline
                except Exception as e:  # noqa: BLE001
                    rec['baseline_error'] = repr(e)
                    self.log(f'[pose] baseline comparison failed: {e!r}')
            bad = gp.check_limits(res['rows'], gp.load_pose_limits(self.args.scenario_file), base)
            rec['limits_exceeded'] = bad
            for b in bad:
                self.log(f'[pose] LIMIT EXCEEDED {b["label"]} {b["metric"]} = {b["value"]} ({b["kind"]} {b["limit"]})')
            if bad:
                self.cycle['pose_limits_exceeded'] = bad
            summary['pose_meter'] = rec
            self.note_cycle('pose_meter', {k: rec[k] for k in ('file', 'table', 'errors', 'limits_exceeded')})
            if not res['errors']:
                shutil.rmtree(os.path.join(self.cycle_dir, '_tmp', 'pose_meter'), ignore_errors=True)
        except Exception as e:  # noqa: BLE001
            summary['pose_meter'] = {'error': repr(e)}
            self.log(f'[pose] meter failed: {e!r}')

    def perf_status(self, attempts: int = 3) -> None:
        """After the scenarios: the motion driver status (frames, feedP95us, budget, skips) into the cycle record - the
        in-game cost of the motion feed for this run. Each attempt: focus, overlay verified closed (gameplay), then the
        verified console command (itself several tries); up to `attempts` attempts with a short wait. If the console
        stays lost: the newest status line already in the UTPP log (cycle['perf_source'] = 'log'), or None.
        cycle['perf_status'] = the status text, cycle['perf_source'] = 'console' | 'log' | None. Best effort."""
        self.cycle['perf_status'], self.cycle['perf_source'] = None, None
        line = None
        for k in range(1, attempts + 1):
            try:
                g = self.guard()
                g.ensure_focus()   # a new guard knows the game window only after this
                self.ensure_overlay(g, 'closed', 'before perf status')
                txt = self.console_verified(None, g, gn.PERF_STATUS_CMD, lambda t: '[night] PERF ' in t, 'perf')
            except Exception as e:  # noqa: BLE001 - never fail a finished cycle on the status line
                self.log(f'[perf] attempt {k}/{attempts} failed: {e!r}')
                txt = None
                if isinstance(e, StopNight) or (isinstance(e, GuardError) and is_user_stop(str(e))):
                    break          # the user took the PC back: no more typing into the console
            line = next((ln for ln in (txt or '').splitlines() if '[night] PERF ' in ln), None)
            if line:
                break
            if k < attempts:
                try:
                    time.sleep(1.0)
                except Exception:  # noqa: BLE001
                    pass
        if line:
            self.cycle['perf_status'] = line.split('[night] PERF ', 1)[1].strip()
            self.cycle['perf_source'] = 'console'
            # the budget fields of the status (wall p95, trips, pause) come in their own line: the first one is cut at 479 characters
            end = time.monotonic() + 1.0
            tail = None
            while tail is None:
                tail = next((ln.split('[night] PERFB ', 1)[1].strip() for ln in (txt or '').splitlines() if '[night] PERFB ' in ln), None)
                if tail is not None or time.monotonic() >= end:
                    break
                time.sleep(0.2)
                txt = read_since(self.utpp_log, max(0, file_size(self.utpp_log) - 4096)) if self.utpp_log else txt
            if tail:
                self.cycle['perf_budget'] = tail
                self.cycle['perf_status'] += ' | ' + tail
        else:
            try:
                st = newest_motion_status(self.utpp_log or gt_log.newest_log(self.args.log_dir))
            except Exception as e:  # noqa: BLE001
                st = None
                self.log(f'[perf] log fallback failed: {e!r}')
            if st:
                self.cycle['perf_status'], self.cycle['perf_source'] = st, 'log'
                self.log('[perf] console lost: newest motion status line from the UTPP log (may be older than the run end)')
            else:
                self.log('[perf] console lost and no motion status line (feedP95us=) in the UTPP log')
        self.log(f'[perf] {self.cycle["perf_status"] or "status LOST"} (source {self.cycle["perf_source"]})')

    # -- cycle --
    def stop_padd(self) -> None:
        if self.pids() or not padd_ping(port=self.run_args.padd_port):
            return
        from gt_input import PaddClient
        c = PaddClient(port=self.run_args.padd_port)
        try:
            c.call({'op': 'quit'})
        finally:
            c.close()

    def run_cycle(self, n: int) -> dict:
        start = datetime.now()
        name = start.strftime('%Y%m%d-%H%M%S')
        self.cycle_dir = os.path.join(self.args.out_root, name)
        os.makedirs(self.cycle_dir, exist_ok=True)
        self.cycle = {'name': name, 'index': n, 'start': start, 'args': vars(self.args), 'steps': [],
                      'scenarios': [], 'result': None, 'body': self.body}
        self.utpp_log = None
        self.channel_mode = None
        self.collector = None
        self.meter_job = None
        self.budget = gb.Budget()
        self.probe_cache = gt_flow.ProbeCache()
        self.early = gt_flow.EarlyStop(getattr(self.args, 'early_stop', 2))
        self.deep_check = None
        self.settings.entries = []
        self.settings.cycle_name = name
        rec = None
        plan = {'runs': [], 'created': start}
        quit_game = True
        unexpected = None
        try:
            self.check_user_idle()
            self.check_continue()
            self.ensure_closed('ensure_closed')
            self.stop_padd()
            self.apply_graphics()
            self.install()
            self.start_idler()
            log = self.launch()
            self.arm(log)
            rec = self.start_recording()
            self.marker(rec, f'{name}-start')
            early = None
            try:
                self.run_scenarios(plan)
            except EarlyStop as e:
                early = e
            self.perf_status()
            self.report_load()
            self.marker(rec, f'{name}-end')
            self.cycle['result'] = 'ok' if early is None else f'failed: {early}'
        except StopNight as e:
            self.cycle['result'] = f'stopped: {e}'
            self.secure_session_files('stop', final=True)
            self.bg_input(False, 'stop')
            if rec:
                self.stop_recording(rec, check=False)
            if getattr(e, 'leave_game', False):
                self.settings_left('somebody is at the PC: nothing is typed into the game')
                self.log(f'[night] STOP: {e} (before the cycle: game left alone)')
            else:
                self.log(f'[night] STOP: {e} -> quitting the game anyway (never left running), then no more cycles')
                try:
                    self.ensure_closed('quit_after_stop')
                    self.stop_padd()
                except Exception as qe:  # noqa: BLE001
                    self.log(f'[night] quit after stop failed: {qe!r}')
            try:
                self.collect(rec, plan)  # read-only copies + analysis
            except Exception as ce:  # noqa: BLE001
                self.log(f'[night] collect failed: {ce!r}')
            self.cycle['end'] = datetime.now()
            self.save()
            raise
        except GracefulStop as e:
            self.cycle['result'] = f'graceful stop: {e}'
            self.log(f'[night] graceful stop: {e} (on-stop={self.args.on_stop})')
            quit_game = self.args.on_stop == 'quit'
            self.secure_session_files('graceful stop', final=True)
            if not quit_game:
                self.restore_settings_leave('run stopped, the game stays open')
        except (CycleError, NeedsKeyboard, PaddError) as e:
            self.cycle['result'] = f'failed: {e}'
            self.log(f'[night] cycle failed: {e}')
            self.secure_session_files('cycle failed', final=True)
        except Exception as e:  # noqa: BLE001 - a defect of the runner: the game is still quit and everything collected, then it is raised
            unexpected = e
            self.cycle['result'] = f'failed: unexpected error: {e!r}'
            self.log(f'[night] cycle failed on an unexpected error: {e!r} -> quitting the game, collecting, then raising it')
            self.secure_session_files('unexpected error', final=True)
        finally:
            if self.bg_input_active and self.cycle['result'] is None:   # any other exception leaves run_cycle here
                self.bg_input(False, 'error')
            if self.cycle['result'] is None:                              # an unexpected exception: copy and restore what we can
                self.secure_session_files('unexpected error', final=True)
                self.restore_settings_leave('unexpected error')
            self.stop_idler()
        self.bg_input(False, 'end of cycle')
        if rec:
            self.stop_recording(rec, check=False)      # the check (and the full decode) wait until the game is closed
        if quit_game:
            try:
                self.ensure_closed('quit')
                self.stop_padd()
            except (CycleError, StopNight) as e:
                self.log(f'[night] quit failed: {e}')
                self.cycle['result'] = f'{self.cycle["result"]}; quit failed: {e}'
        try:
            self.collect(rec, plan)
        except Exception as e:  # noqa: BLE001
            self.log(f'[night] collect failed: {e!r}')
        self.cycle['end'] = datetime.now()
        self.report_budget()
        self.save()
        if unexpected is not None:
            raise unexpected
        return self.cycle

    def apply_graphics(self) -> None:
        """--graphics low: the lowest quality profile, with the game closed, before the launch. Never restored here."""
        mode = getattr(self.args, 'graphics', 'keep')
        if mode != 'low':
            return
        res = gt_session.apply_graphics(mode, self.graphics_settings, self.graphics_state_dir)
        self.log(f'[graphics] {res["status"]}')
        self.note_cycle('graphics', {k: res[k] for k in ('mode', 'code', 'status')})

    def report_budget(self) -> None:
        """Time budget table of the cycle (printed, kept in cycle.json and next to summary.json as time_budget.txt)."""
        try:
            lines = self.budget.table(self.cycle['steps'])
            if not lines:
                return
            self.log('[budget] time budget of this run:')
            for ln in lines:
                self.log('[budget] ' + ln)
            self.cycle['time_budget'] = {'scenarios': self.budget.rows(), 'phases': self.budget.phase_totals(self.cycle['steps']),
                                         'post': dict(self.budget.phases), 'probes_skipped': self.probe_cache.skipped}
            with open(os.path.join(self.cycle_dir, 'time_budget.txt'), 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
            sp = os.path.join(self.cycle_dir, 'summary.json')
            if os.path.isfile(sp):
                with open(sp, encoding='utf-8') as f:
                    summ = json.load(f)
                summ['time_budget'] = self.cycle['time_budget']
                with open(sp, 'w', encoding='utf-8') as f:
                    json.dump(summ, f, indent=1, default=str)
        except Exception as e:  # noqa: BLE001 - a report problem never fails the cycle
            self.log(f'[budget] table failed: {e!r}')


# ---------------------------------------------------------------------------------------------- dry run

def night_pids_dry() -> list[int]:
    """Game processes for the dry run's read-only checks."""
    try:
        from gt_win32 import Win32Api
        return Win32Api().list_pids(GAME_EXE)
    except Exception:  # noqa: BLE001
        return []


def dry_run(args) -> int:
    print('DRY RUN - read-only checks; no input, no launch, no quit, no install, nothing written under the game folder\n')
    ok = True

    def item(label, good, detail=''):
        nonlocal ok
        ok = ok and bool(good)
        print(f'[{"OK" if good else "!!"}] {label}' + (f': {detail}' if detail else ''))

    scenarios = load_scenarios(args.scenario_file)
    names = gn.resolve_sets(args.set)
    errs = gn.validate_sets(names, scenarios)
    item('scenario set', not errs, f'{names}' + (f' ERR {errs}' if errs else ''))
    total = sum(scenarios[n].duration for n in names if n in scenarios) * args.repeat
    print(f'     pad input time ~{total:.0f} s plus macros; cycles={args.cycles}; scenario timeout '
          f'{args.scenario_timeout:.0f} s; max wall {args.max_wall_hours} h')
    print('\nPlan per cycle: ensure_closed -> ' + ('install -> ' if args.cet_mods else '') +
          'launch -> arm -> record_start -> marker -> scenarios -> marker -> record_stop -> quit -> collect')
    if args.cet_mods:
        print(f'     cet mods: {INSTALL_CET} {args.cet_mods.replace(",", " ")}')
    print(f'     launch: {args.launch} -> ' + (f'{GAME_EXE_PATH} (cwd {os.path.dirname(GAME_EXE_PATH)})'
                                                 if args.launch == 'exe' else f'Start-Process {STEAM_URL}'))
    print(f'     quit command (CET console): {gn.QUIT_CMD}')
    print(f'     recording safety: pose segments (utpp-pose-*.utpppose), input dump (utpp-feed-*.txt) and mod log from {args.log_dir} '
          'are copied into <run>\\logs between the scenarios, at the end of the scenario phase on every exit path, and after the quit')
    print('     --diag ' + args.diag + ': ' + ('the diagnostic logs option is not touched' if args.diag == 'keep' else
          f'diagnostic logs "{args.diag}" are set at arm (previous values read first) and put back before every ExitGame / on every '
          'exit path; a change that could not be put back stays in pending_restore.json and the next launch restores it first'))
    pend = os.path.join(args.out_root, 'pending_restore.json')
    print(f'     pending settings restore from an earlier run: ' + (f'PRESENT {pend} (the next launch restores it first)'
                                                                   if os.path.isfile(pend) else 'none'))
    print('     pose meter: worker processes after the quit (pose_metrics.json, pose_table.txt, "pose ..." lines)'
          + (f'; baseline {args.pose_baseline}' if args.pose_baseline else '') + ('; DISABLED (--no-pose-meter)' if args.no_pose_meter else ''))
    print(f'     machine load: CPU mean over the scenario phase and compiler processes seen (warning above '
          f'{gt_load.HIGH_CPU_PERCENT:.0f} % CPU or any compiler), next to the perf line')
    print(f'     speed: the recorder is ended {args.recorder_grace:.0f} s after "q" at the latest and BEFORE the game quits; its full '
          f'decode runs in the background after the quit; ExitGame is given {args.quit_timeout:.0f} s (when echoed) before taskkill; '
          'contact sheets (cut into one-sheet parts) and failure clips run in a pool of processes sized to the idle cores '
          f'(now ~{gt_post.pool_size(40)} of {os.cpu_count()} for 40 jobs; --post-workers {args.post_workers})')
    print('     post-processing: ' + ('--fast: NO sheets and NO failure clips (summary.json "pending_media" lists them with video '
                                     'seconds and the commands that make them later)' if args.fast else
                                     f'sheets{"" if not args.no_sheets else " OFF"}, failure clips'
                                     f'{"" if not args.no_failure_clips else " OFF"}: at most {args.clips_per_cause} per cause '
                                     '(the first failing item of each scenario first; 0 = all)'))
    print('     look-ahead probe: skipped when the previous probe started within '
          f'{gt_flow.SPOT_M:.0f} m of here along the same direction (camera and route within {gt_flow.HEADING_DEG:.0f} deg), '
          f'covered >= {gt_flow.REACH_SLACK * 100:.0f} % of the distance needed and came back clear; the 1.4 s face push stays; '
          'forgotten after a camera turn, a blocked attempt, the vehicle scenario and at every cycle start')
    print('     early stop: ' + (f'the first {args.early_stop} walking scenarios invalid for one cause (V cannot move / no motion '
                                 'lines) end the cycle with that cause in one line; the rest is listed as not played'
                                 if args.early_stop else 'off (--early-stop 0)'))
    print('     time budget: a table per scenario (total, play, probe, retries, waits, copy, probes skipped) and per phase '
          '(launch, arm, scenarios, recorder stop, quit, collect, post-processing) at the end; time_budget.txt in the run folder')
    bodies = gt_session.bodies_of(args.body)
    print('     body: ' + (', '.join(f'{b} = {gt_session.BODY_SAVES[b]}' for b in bodies) + ' (select_save.py before each launch with the '
                           'game closed, exact restore after each run on every exit path; one launch per body, one summary '
                           'body_summary_<time>.txt)' if bodies else 'keep (the game starts with its own newest save)'))
    print('     graphics: ' + ('low before the launch (graphics_profile.py --apply), never restored automatically'
                               if args.graphics == 'low' else 'keep'))
    print(f'     arm commands: {gn.arm_cmds(args.db, [c for c in args.cet.split(";;") if c.strip()])}')
    print(f'     position probe: {gn.NIGHTPOS_CMD} (needs utpp_core 0.5.2 installed; fallback inline '
          f'{len(gn.POS_FUNC_DEF) + len(gn.POS_CALL) + 1} chars)')
    print(f'     vehicle spawn modes (in order): {args.vehicle_spawn}')
    print(f'       codeware: {gn.SPAWN_CODEWARE_CMD}')
    print(f'       summon:   {gn.SPAWN_SUMMON_CMD}')
    print(f'     vehicle arrival (vhold/summon): wait <= {args.arrival_max:.0f} s for a car that moves > 1 m (>= 2 driving '
          f'samples) and parks < {args.car_stop_speed} m/s for {args.car_stop_hold} s within {args.arrival_dist:.0f} m; '
          'never-moving positions = stale; after the X presses without a mount: "no car?" (F only with --mount-try-f)')
    print(f'     vehicle parking: stand still until the car is < {args.car_stop_speed} m/s for {args.car_stop_hold} s '
          f'(max {args.car_stop_max:.0f} s); before each X: car still + lat 0.9-1.7, else re-plan (max {args.mount_replans})')
    print(f'     quit: focus loss -> refocus x{args.quit_focus_tries} ({args.quit_focus_gap} s gaps) -> taskkill; '
          'the game is never left running after a started cycle')
    print(f'     exit fallback (scripted, last): {gn.UNMOUNT_CMD if args.cet_unmount else "off"}')
    print(f'     mount fallback (scripted, after X/F): {gn.MOUNT_CMD if args.cet_mount else "off"}'
          + (f'  (2nd try: isInstant=true; evidence <= {args.cet_mount_wait:.0f} s)' if args.cet_mount else ''))
    print(f'       vtap:     tap scan 0x{gn.SCAN_V:02X} (V) 0.1 s')
    print(f'       vhold:    hold V, after {args.vhold_seconds} s Down arrow (0x{gn.SCAN_DOWN:04X}), release V (no Enter)')
    print(f'     weapon: draw Y tap, aim LT 2 s, holster Y double tap; verified by NightPos w= or utpp_perspective lines')
    print(f'     melee: fists by script ({"RequestEquipFists" if args.cet_fists else "off"}), RT tap, 3 punches, punch+walk, '
          'LT block 2 s, UnequipWeapon (then Y double tap if still active); verified by UTPP MMMASK/MMGATE melee lines '
          '(w >= 0.9 while masked, mask off <= 3 s after the holster)')
    print(f'     melee_fists_target (--set melee_fists_target only): Codeware NPC {", ".join(args.enemy_record.split(","))} '
          '(first that works), fists raised with no target, then a fresh Immortal target spawned at 2.0 / 1.2 / 1.2 (camera '
          'orbit 60 deg) / 1.5 m before 3 punches / 3 punches / 3 punches / LT block (a phase whose target is not at the '
          'wanted distance is not punched); PASS = target found (not Friendly), >= 1 '
          'MELEEAIM src=lock, health drop at 1.2 m; target deleted and god mode removed in a finally; pad and commands '
          'only (also with --background)'
          + (' ; --punch-report: tools\\analysis\\punch_direction_report.py on the log' if args.punch_report else ''))
    for n in names:
        if n in gw.WEAPON_SETS:
            print(f'     {n}: weapon macros (gt_melee_weapons.py), commands per weapon:')
            for tag in gw.WEAPON_SETS[n]:
                print('\n'.join('       ' + ln for ln in gw.dry_run_lines(tag, scenarios)))
    if 'melee_fists_target' in names:
        rec0 = args.enemy_record.split(',')[0]
        print(f'       place (Codeware, tag {gn.ENEMY_TAG}; the first one defines UTPPENEMY, {len(gn.ENEMY_DEFINE)} chars): '
              f'{gn.enemy_place_cmd(rec0, 1.2, "after_near")}')
        print(f'       placed? (state read until at=1, then god mode): {gn.enemy_cmd("placed", 1.2, 1)}')
        print(f'       despawn: {gn.enemy_off_cmd()}')
    print('     aim_probe (--set aim_probe only): perspective.aiming=tpp + utpp_probe AimProbe(true) by script, LT held 4 s with '
          '2 RT shots, AimProbe(false) + aiming=fpp always restored (finally); PASS = >= 4 aimprobe samples, >= 1 shot, '
          'a summary line (weapon_draw first if NightPos shows w=0)')
    print(f'     wheel/menu recovery: --wheel-close {args.wheel_close} once, only after |stick|>=0.7 for >=1.5 s '
          'with |v|<0.3')
    print(f'     crouch: {"keyboard C (scan 0x2E) + MMTRACE loco check" if args.crouch_button == "key" else "pad B hold"}')
    print(f'     vehicle approach: door point lon {args.door_lon} / lat {args.door_lat} m (car frame, left = driver), '
          f'car-frame waypoints around the ends (>= {gn.CLEAR_LAT} m clearance), stop within 0.8 m and lat 0.9-1.7; '
          f'camera turn first when > {args.turn_first:.0f} deg off (pulses until < {args.align_tol:.0f} deg, start rate '
          f'{gn.CAMERA_RATE_DEG_S:.0f} deg/s); chunks <= {args.approach_chunk} s each followed by a NightPos '
          f'(re-sent once); dead reckoning {gn.SPEEDS} m/s for <= {args.dr_max} s; sprint > {args.sprint_dist:.0f} m; '
          f'time box max({args.approach_timeout:.0f} s, dist/1.5+15 s); face the car; mount: X x{args.mount_tries} with '
          f'{args.mount_nudge} m nudges, then F')
    print(f'     seated = NightPos m=1 polled every {args.seat_poll:.0f} s (max {args.seat_timeout:.0f} s) + '
          f'{args.seat_settle} s + stable "cam veh="; drive RT 3 s, verify the car moved > 1 m')
    print(f'     positions: utpp_core NightAuto every {args.night_auto_period} s (enabled at arm, lines <= '
          f'{args.pos_max_age} s old), console NightPos only as fallback; overlay state checked from a captured frame '
          f'before/after every console command and before every input (check={args.overlay_check})')
    print(f'     console: every command echo-verified, attempts with settle {args.console_settles} s, '
          '"[cet] <cmd> ok after N tries" / "LOST" in night.log (NightPos, spawn, marker, quit)')
    print(f'     vehicle exit: {args.exit_pre} s neutral, F hold 2 s -> F hold 3 s -> pad B hold 2 s -> pad B hold 3 s '
          f'-> F tap x{args.exit_taps} (each: <= {args.exit_start_wait:.0f} s for "keepTpp ride -> exit", then a '
          f'NightPos m= re-check), then <= {args.exit_done_wait:.0f} s for "exit pull: end"/"exit -> off"; forced before '
          f'each attempt and before quit when mounted; vehicles > {args.max_vehicle_dist:.0f} m ignored; quit: taskkill '
          f'{args.quit_timeout:.0f} s after ExitGame')
    print(f'     stop: create {stop_file(args.out_root)} or CTRL_BREAK/SIGTERM; on-stop={args.on_stop}')
    print(f'     marker: {gn.mark_cmds("<cycle>-start")}')
    item('no STOP file present', not os.path.exists(stop_file(args.out_root)), stop_file(args.out_root))
    item('game exe', os.path.isfile(GAME_EXE_PATH), GAME_EXE_PATH)
    cw = os.path.join(GAME_DIR, 'red4ext', 'plugins', 'Codeware')
    item('Codeware (codeware spawn mode)', os.path.isdir(cw), cw)
    core = os.path.join(CET_DIR, 'mods', 'utpp_core', 'init.lua')
    inst = open(core, encoding='utf-8', errors='replace').read() if os.path.isfile(core) else ''
    print(f'[--] utpp_core NightPos installed in the game: {"yes" if "M.NightPos" in inst else "no (inline fallback)"}'
          f'; NightAuto (0.5.4): {"yes" if "M.NightAuto" in inst else "no -> install utpp_core 0.5.4 (--cet-mods utpp_core)"}')
    for p in (INSTALL_CET, os.path.join(HERE, 'video_sheets.py'), os.path.join(HERE, 'padd.py')):
        item(f'script {os.path.relpath(p, PROJECT)}', os.path.isfile(p))
    item('ffmpeg', os.path.isfile(args.ffmpeg), args.ffmpeg)
    if os.path.isfile(args.ffmpeg):
        dev = subprocess.run([args.ffmpeg, '-hide_banner', '-devices'], capture_output=True, text=True, errors='replace')
        item('ffmpeg gdigrab device (desktop capture)', 'gdigrab' in dev.stdout + dev.stderr)
        flt = subprocess.run([args.ffmpeg, '-hide_banner', '-filters'], capture_output=True, text=True, errors='replace')
        item('ffmpeg ddagrab filter (fallback)', 'ddagrab' in flt.stdout, '')
        ff8, why = gt_capture.find_ffmpeg(args.capture_ffmpeg)
        label = 'ffmpeg with the gfxcapture filter (--capture window, automatic with --background)'
        if gt_capture.resolve_mode(args.capture, args.background, args.capture_explicit) == gt_capture.MODE:
            item(label, bool(ff8), ff8 or why)
        else:
            print(f'[--] {label}: {ff8 or "not available: " + why}')
        for enc in ('h264_nvenc', 'libx264'):
            p = subprocess.run([args.ffmpeg, '-hide_banner', '-loglevel', 'error', '-f', 'lavfi', '-i',
                                'color=size=256x256:duration=0.2', '-c:v', enc, '-f', 'null', '-'],
                               capture_output=True, text=True, errors='replace')
            item(f'encoder {enc} (null output test)', p.returncode == 0, p.stderr.strip()[:120])
    if IS_WINDOWS:
        g = ForegroundGuard()
        api = g.api
        stt = g.status()
        print(f'[--] game: {stt}; image paths {[process_path(x) for x in stt["game_pids"]]}')
        print(f'[--] foreground now: {g.describe_foreground()}')
        if g.game_hwnd:
            rect, screen = physical_rects(g.game_hwnd)
            print(f'[--] game window title {api.window_title(g.game_hwnd)!r}, client rect (physical px) {rect}, '
                  f'monitor {screen}, logical {api.client_rect_screen(g.game_hwnd)}, crop '
                  f'{"yes" if gn.crop_needed(rect, screen) else "no (covers the monitor)"}')
            try:
                shot = grab_gray(gn.clamp_rect(rect, screen))
                print(f'[--] CET overlay state from a captured frame: {gn.classify_overlay(*shot)}')
            except Exception as e:  # noqa: BLE001
                print(f'[!!] overlay frame capture failed: {e!r}')
            ov = gt_run.make_overlay(gt_run.build_parser().parse_args(['--cet-input-rel', args.cet_input_rel]), g)
            print(f'[--] CET toggle scans {[hex(s) for s in ov.toggle_scans]}, console click (screen) '
                  f'{ov.console_click_screen()}')
        else:
            rect = screen = None
            print('[--] game not running: capture rect and console point are read at the first live cycle')
        mode = gt_capture.resolve_mode(args.capture, args.background, args.capture_explicit)
        if mode == gt_capture.MODE:
            ff8, why = gt_capture.find_ffmpeg(args.capture_ffmpeg)
            if ff8:
                cmd = gt_capture.build_cmd(ff8, r'<cycle>\rec_<ts>.mp4', g.game_hwnd or 1,
                                           (rect[2] - rect[0], rect[3] - rect[1]) if rect else (1920, 1080), args.fps)
            else:
                cmd = None
                print(f'[!!] window capture selected but unavailable: {why}; the run falls back to the desktop modes '
                      '(the video shows what covers the game)')
        else:
            cmd = gn.build_ffmpeg_cmd(args.ffmpeg, r'<cycle>\rec_<ts>.mp4', mode,
                                      rect or (0, 0, 1920, 1080), screen, args.fps, 'h264_nvenc')
        if cmd:
            print(f'[--] ffmpeg command ({mode})' + ('' if rect else ' (example rect 0,0,1920,1080)') + ': ' +
                  subprocess.list2cmdline(cmd))
        idle = idle_seconds()
        print(f'[--] user idle {idle:.0f} s (--min-idle {gt_session.effective_min_idle(args):.0f}'
              f'{" - default of --background" if args.background and args.min_idle is None else ""})')
    ovk = read_cet_overlay_key()
    item('CET overlay key (bindings.json)', bool(ovk), str([hex(v) for v in ovk]) if ovk else 'unknown')
    item('CET log', os.path.isfile(os.path.join(CET_DIR, 'cyber_engine_tweaks.log')))
    lg = gt_log.newest_log(args.log_dir)
    item('UTPP log dir', bool(lg), lg or args.log_dir)
    if args.cmd_channel != 'console':
        item('utpp_testcmd module installed (file channel)', os.path.isfile(os.path.join(os.path.dirname(TESTCMD_FILE),
                                                                                         'init.lua')))
        print(f'[--] newest UTPP log READY line: {testcmd_ready(read_since(lg, 0)) if lg else None}')
    print(f'[--] command channel {args.cmd_channel}, background {args.background}')
    if bodies:
        ss = gt_session.SaveSession(args.out_root, running=lambda: night_pids_dry())
        try:
            have = select_save.scan_saves(ss.saves_dir)
        except select_save.SaveError as e:
            have = {}
            item('saves folder', False, str(e))
        for b in bodies:
            item(f'save of the {b} body ({gt_session.BODY_SAVES[b]})', gt_session.BODY_SAVES[b] in have)
        left = ss.leftover()
        print(f'[--] leftover save selection of an earlier run: ' + (f'PRESENT {left} (the next start restores it first)' if left else 'none'))
        item('no foreign select_save state in place', not (ss.state_present() and not left), ss.state_file)
        item('game not running (a body is selected with the game closed; a running game is never closed for it)',
             not ss.game_running())
    if args.graphics == 'low':
        print('[--] graphics: ' + gt_session.graphics_status())
    if args.background:
        ff8, why8 = gt_capture.find_ffmpeg(args.capture_ffmpeg)
        reqs = gt_session.background_requirements(GAME_DIR, os.path.dirname(TESTCMD_FILE), args, ff8, why8)
        print('\nBackground mode (--background): the game stays behind other windows; no foreground, no keyboard, no user idle '
              f'time needed ({gt_session.effective_min_idle(args):.0f} s); only Pause stops the run')
        for label, good, detail in reqs:
            item(label, good, detail)
        lg_ready = testcmd_ready(read_since(lg, 0)) if lg else None
        print(f'[--] test command module READY line in the newest UTPP log: {lg_ready}'
              + ('' if lg_ready else ' (it appears at the start of a session; the run needs it after the launch)'))
        missing = [l for l, g, _ in reqs if not g]
        print('     background run ' + ('is possible: every requirement is met' if not missing else
                                        'is NOT possible yet, missing: ' + '; '.join(missing)))
    item('vgamepad package (Tools\\pylibs)', os.path.isdir(os.path.join(PROJECT, 'Tools', 'pylibs', 'vgamepad')))
    sc_q = subprocess.run(['sc', 'query', 'ViGEmBus'], capture_output=True, text=True, errors='replace')
    item('ViGEmBus service running', 'RUNNING' in sc_q.stdout)
    print(f'[--] padd daemon: {padd_ping() or "not running (started by the first live cycle)"}')
    tl = subprocess.run(['tasklist', '/FI', 'IMAGENAME eq steam.exe'], capture_output=True, text=True, errors='replace')
    item('steam.exe running', 'steam.exe' in tl.stdout.lower())
    rq = subprocess.run(['reg', 'query', r'HKCR\steam\Shell\Open\Command'], capture_output=True, text=True,
                        errors='replace')
    item('steam:// URL handler registered', rq.returncode == 0)
    for sub, setting, label in (('SUB_SLEEP', 'STANDBYIDLE', 'sleep after'), ('SUB_VIDEO', 'VIDEOIDLE',
                                                                              'display off after')):
        q = subprocess.run(['powercfg', '/query', 'SCHEME_CURRENT', sub, setting], capture_output=True, text=True,
                           errors='replace')
        ac = [l for l in q.stdout.splitlines() if 'AC' in l and '0x' in l]
        print(f'[--] power {label} (AC): {ac[0].split(":")[-1].strip() if ac else "?"} (s, 0x0 = never; '
              'night.py also requests ES_SYSTEM_REQUIRED|ES_DISPLAY_REQUIRED while --go runs)')
    free = shutil.disk_usage(PROJECT).free / 2 ** 30
    item('free disk for recordings (D:)', free > 20, f'{free:.0f} GiB')
    print('\nresult: ' + ('all prerequisites OK' if ok else 'some prerequisites missing (see !!)'))
    return 0 if ok else 1


# ---------------------------------------------------------------------------------------------- CLI

class _ExplicitChannel(argparse.Action):
    """--cmd-channel <mode>: store the mode and remember that it was given (--background otherwise picks auto)."""

    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, values)
        namespace.cmd_channel_explicit = True


class _ExplicitCapture(argparse.Action):
    """--capture <mode>: store the mode and remember that it was given (a given mode is never replaced by the
    background default)."""

    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, values)
        namespace.capture_explicit = True


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--go', action='store_true', help='really run (input, launch, quit, install, record)')
    p.add_argument('--launch', default='steam', choices=['steam', 'exe'],
                   help='steam = Start-Process steam://rungameid/1091500; exe = start bin\\\\x64\\\\Cyberpunk2077.exe '
                        '(cwd bin\\\\x64, Steam must run)')
    p.add_argument('--max-wall-hours', type=float, default=8.0, help='stop cleanly (game quit) after this time')
    p.add_argument('--on-stop', default='quit', choices=['quit', 'leave'],
                   help='game handling on STOP file / signal / wall limit')
    p.add_argument('--dry-run', action='store_true', help='read-only prerequisite checks and the plan')
    p.add_argument('--cycles', type=int, default=1)
    p.add_argument('--set', default='loco', help='comma list of sets/scenarios: ' + ', '.join(gn.SETS))
    p.add_argument('--repeat', type=int, default=1, help='runs of each scenario per cycle')
    p.add_argument('--db', default='utpp_base', help='UTPPLoadDatabase name')
    p.add_argument('--cet', default='', help='extra CET commands after arming, ";;"-separated')
    p.add_argument('--cet-file', default='', help='file with extra CET commands (";;"-separated), added after --cet')
    p.add_argument('--cet-final', default='', help='CET commands sent before every ExitGame (";;"-separated): undo what '
                                                   'the session switched on, e.g. the diagnostic logs')
    p.add_argument('--cet-final-file', default='', help='file with --cet-final commands, added after --cet-final')
    p.add_argument('--cet-mods', default='', help='run install_cet_mods.py with these mods (comma list)')
    p.add_argument('--capture', default='desktop', choices=list(gn.CAPTURE_MODES) + [gt_capture.MODE],
                   action=_ExplicitCapture,
                   help='desktop = gdigrab desktop cropped to the game client rect; ddagrab = Desktop Duplication + crop=; '
                        'window = the game window\'s own picture, also while other windows cover it (needs an ffmpeg '
                        '8.x build, see --capture-ffmpeg; at most 1920x1080). With --background the default switches '
                        'to window; an explicit value is kept.')
    p.add_argument('--capture-ffmpeg', default='',
                   help='ffmpeg with the gfxcapture filter for --capture window (default: the newest '
                        'Tools\\ffmpeg-<version>\\ffmpeg.exe that has it)')
    p.set_defaults(capture_explicit=False)
    p.add_argument('--encoder', default='auto', choices=['auto', 'h264_nvenc', 'libx264'])
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--min-luma', type=float, default=2.0, help='mean luma below this = black recording (failed step)')
    p.add_argument('--ffmpeg', default=video_sheets.FFMPEG)
    p.add_argument('--out-root', default=NIGHT_ROOT)
    p.add_argument('--log-dir', default=gt_log.DEFAULT_LOG_DIR)
    p.add_argument('--scenario-file', default=gt_run.DEFAULT_SCENARIOS)
    p.add_argument('--thresholds', default=gt_run.DEFAULT_THRESHOLDS)
    p.add_argument('--cet-input-rel', default='%.3f,%.3f' % gt_run.DEFAULT_CONSOLE_INPUT_REL)
    p.add_argument('--cet-key-scan', default='')
    p.add_argument('--console-open-wait', type=float, default=0.3, help='overlay open/close wait for position polls')
    p.add_argument('--console-settle', type=float, default=0.5,
                   help='pad neutral this long before a position poll opens the CET console')
    p.add_argument('--pos-timeout', type=float, default=1.0, help='max wait for a NightPos line per attempt')
    p.add_argument('--console-settles', default='0.5,1.0,1.5',
                   help='pad-neutral settle before each attempt of an echo-verified console command (also = attempts)')
    p.add_argument('--log-console-ok', action='store_true', help='also log first-try console successes')
    p.add_argument('--no-overlay-check', dest='overlay_check', action='store_false',
                   help='do not verify the CET overlay state from a captured frame')
    p.add_argument('--overlay-tries', type=int, default=3)
    p.add_argument('--no-night-auto', dest='night_auto', action='store_false',
                   help='do not enable utpp_core NightAuto (positions then come from typed NightPos commands)')
    p.add_argument('--night-auto-period', type=float, default=0.5)
    p.add_argument('--pos-max-age', type=float, default=1.5, help='max age of a NightAuto POS line (s)')
    p.add_argument('--crouch-button', default='key', choices=['key', 'pad'],
                   help='key = keyboard C tap (ToggleCrouch IK_C) verified by MMTRACE loco; pad = the scenario B hold')
    p.add_argument('--crouch-key-hold', type=float, default=0.08)
    p.add_argument('--crouch-verify', type=float, default=1.5, help='seconds to see loco=2 / loco=1 after a C tap')
    p.add_argument('--quit-timeout', type=float, default=QUIT_TIMEOUT_S,
                   help='seconds after an echoed ExitGame before taskkill (a game that is closing needs well over 20 s on a '
                        'busy machine; without the echo the kill is immediate)')
    p.add_argument('--quit-focus-tries', type=int, default=3, help='refocus attempts when quitting (then taskkill)')
    p.add_argument('--quit-focus-gap', type=float, default=2.0)
    p.add_argument('--car-stop-speed', type=float, default=0.2, help='m/s below which the vehicle is stationary')
    p.add_argument('--car-stop-hold', type=float, default=3.0, help='seconds the vehicle must stay stationary')
    p.add_argument('--car-stop-max', type=float, default=45.0, help='max wait for the vehicle to stop')
    p.add_argument('--mount-replans', type=int, default=2, help='approach re-plans at the door before X')
    p.add_argument('--kill-grace', type=float, default=20.0)
    p.add_argument('--launch-timeout', type=float, default=240.0)
    p.add_argument('--gameplay-settle', type=float, default=15.0)
    p.add_argument('--calib-timeout', type=float, default=30.0)
    p.add_argument('--focus-grace', type=float, default=0.0)
    p.add_argument('--min-idle', type=float, default=None,
                   help='required user idle time before a cycle (0 = off; default 120 s, 0 with --background)')
    p.add_argument('--wake', default='rs')
    p.add_argument('--pad-settle', type=float, default=2.0)
    p.add_argument('--gap', type=float, default=2.0)
    p.add_argument('--scenario-timeout', type=float, default=180.0,
                   help='per scenario (at least 2x its pad duration + 30 s); macros check it between actions')
    p.add_argument('--wheel-close', default='B',
                   help='closes the weapon wheel / a menu when no movement follows a stick command: pad button name '
                        f'({",".join(sorted(BUTTONS))}) or key:0xSCAN')
    p.add_argument('--vehicle-repeats', type=int, default=2)
    p.add_argument('--vehicle-spawn', default='vtap,vhold,summon,codeware',
                   help='spawn modes tried in order: vtap (tap V: call the selected vehicle), vhold (hold V, Down arrow, '
                        'release V = confirm; the car drives in, cycle 10 ~90 s), '
                        'summon (CET SpawnActivePlayerVehicle, also drives in; cycle 12: may report a phantom position), '
                        'codeware (CET Codeware entity next to V: cycle 9 - no exit offered), dpad (pad D-pad right)')
    p.add_argument('--arrival-max', type=float, default=120.0, help='max wait for a vhold/summon car to drive in')
    p.add_argument('--arrival-dist', type=float, default=15.0, help='an arrived car must park within this many m')
    p.add_argument('--mount-try-f', action='store_true', help='after the X presses also try keyboard F')
    p.add_argument('--no-cet-unmount', dest='cet_unmount', action='store_false',
                   help='do not use the scripted unmount (ExitWithTeleport sequence) as the last exit fallback')
    p.add_argument('--no-cet-mount', dest='cet_mount', action='store_false',
                   help='do not use the scripted mount (IMountingFacility.Mount from the console) after the X/F presses fail')
    p.add_argument('--no-cet-fists', dest='cet_fists', action='store_false',
                   help='melee_fists: do not equip/unequip the fists by script (RequestEquipFists / UnequipWeapon); '
                        'RT tap and the Y double tap only')
    p.add_argument('--enemy-record', default=','.join(gn.ENEMY_RECORDS),
                   help='melee_fists_target: Character record(s), comma list, the first that spawns and is found is used '
                        '(env UTPP_NIGHT_ENEMY sets the default); names of the installed game\'s TweakDB')
    p.add_argument('--enemy-spawn-wait', type=float, default=4.0,
                   help='melee_fists_target: seconds to wait for the spawned target to show up at the wanted distance '
                        '(state read at=1) per placement')
    p.add_argument('--enemy-pos-period', type=float, default=gn.ENEMY_POS_PERIOD,
                   help='melee_fists_target: NightAuto period (s) while the target exists (V <-> target distance)')
    p.add_argument('--punch-report', action='store_true',
                   help='melee_fists_target: afterwards run Source\\tools\\analysis\\punch_direction_report.py on the UTPP '
                        'log (command line: gt_night.punch_report_cmd); skipped when the script is missing')
    p.add_argument('--cet-mount-wait', type=float, default=6.0,
                   help='seconds to wait for mount evidence (vehiclecam line / NightPos m=1) after a scripted mount')
    p.add_argument('--vhold-seconds', type=float, default=0.8, help='V held this long before the Down arrow')
    p.add_argument('--spawn-wait', type=float, default=4.0, help='seconds to wait for a Codeware spawn')
    p.add_argument('--vehicle-arrive-wait', type=float, default=12.0)
    p.add_argument('--approach-mag', type=float, default=1.0, help='stick magnitude while running to the car')
    p.add_argument('--approach-chunk', type=float, default=1.5, help='max seconds of stick per approach iteration')
    p.add_argument('--approach-timeout', type=float, default=25.0,
                   help='minimum time box of the approach (actual: max(this, dist/1.5 + 15 s))')
    p.add_argument('--dr-max', type=float, default=2.0, help='max seconds of stick bridged by dead reckoning')
    p.add_argument('--turn-first', type=float, default=35.0, help='turn the camera first when the target is further '
                   'off the camera forward (deg)')
    p.add_argument('--align-tol', type=float, default=20.0, help='camera alignment tolerance (deg)')
    p.add_argument('--align-pulses', type=int, default=8, help='max right-stick pulses per alignment')
    p.add_argument('--face-tol', type=float, default=25.0, help='tolerance for facing the car before mounting (deg)')
    p.add_argument('--sprint-dist', type=float, default=15.0, help='L3 sprint tap when further than this (m)')
    p.add_argument('--door-lat', type=float, default=gn.DOOR_LAT, help='door point: m LEFT of the car centre')
    p.add_argument('--door-lon', type=float, default=gn.DOOR_LON, help='door point: m AHEAD of the car centre')
    p.add_argument('--mount-tries', type=int, default=2,
                   help='X presses before the scripted mount: where V arrived, then after a walk along the car (gn.X_SEEK_LONS)')
    p.add_argument('--mount-nudge', type=float, default=0.3, help="minimum m of the step to the door handle between X presses")
    p.add_argument('--mount-wait', type=float, default=3.0, help='seconds for the mount line after X / after F')
    p.add_argument('--exit-start-wait', type=float, default=3.0, help='seconds for "keepTpp ride -> exit" after the '
                   'first F tap and after F hold / B hold')
    p.add_argument('--exit-pre', type=float, default=1.5, help='neutral seconds before the first exit try')
    p.add_argument('--exit-taps', type=int, default=6, help='F taps after the F / B holds (last resort)')
    p.add_argument('--exit-tap-interval', type=float, default=2.5, help='seconds between F taps')
    p.add_argument('--seat-timeout', type=float, default=25.0, help='max seconds from the mount line to NightPos m=1')
    p.add_argument('--seat-poll', type=float, default=2.0, help='NightPos cadence while waiting to be seated')
    p.add_argument('--seat-settle', type=float, default=2.5, help='seconds after m=1 before driving')
    p.add_argument('--seat-fallback', type=float, default=14.0, help='fixed wait when NightPos has no m= flag')
    p.add_argument('--exit-done-wait', type=float, default=8.0, help='seconds for "exit pull: end" / "exit -> off"')
    p.add_argument('--max-vehicle-dist', type=float, default=gn.MAX_VEHICLE_DIST,
                   help='a reported vehicle farther than this from V is ignored (m)')
    p.add_argument('--weapon-window', type=float, default=6.0, help='draw/holster confirmation window (s)')
    p.add_argument('--sheet-pad', type=float, default=3.0)
    p.add_argument('--no-sheets', action='store_true')
    p.add_argument('--post-workers', type=int, default=0,
                   help='processes for the contact sheets and the failure clips after the game is closed (0 = from the cores '
                        'that are idle then, at most 24; 1 = one by one)')
    p.add_argument('--keep-sheet-frames', action=argparse.BooleanOptionalAction, default=False,
                   help='keep sheets\\<name>\\frames (per-frame images, intermediate files of video_sheets.py); '
                        'default: deleted once the sheets exist')
    p.add_argument('--no-media-move', action='store_true',
                   help='leave the recording and the sheet images in the run folder (default: move them to Media)')
    p.add_argument('--no-failure-clips', action='store_true',
                   help='do not cut 1x clips of failing steps into Media\\mp4\\<date>_niezaliczone_<run>')
    p.add_argument('--clips-per-cause', type=int, default=failure_clips.PER_CAUSE,
                   help='failure clips per cause (the failure kind with its numbers removed): the first failing item of each '
                        'scenario first, then the others in time order; 0 = every failing item (default %(default)s)')
    p.add_argument('--fast', action='store_true',
                   help='no contact sheets and no failure clips: the run record (summary.json, pending_media) lists which '
                        'sheets and clips are left, with their video seconds and the commands that make them later (gt_post.py '
                        'sheets <run> / failure_clips.py <run>)')
    p.add_argument('--recorder-grace', type=float, default=RECORDER_GRACE_S,
                   help="seconds the window / desktop recorder gets to finish after 'q' before it is ended (a fragmented mp4 "
                        'stays decodable)')
    p.add_argument('--early-stop', type=int, default=2,
                   help='stop the cycle when each of its first N walking scenarios is invalid for the same cause (V cannot '
                        'move, no motion lines ...); 0 = always play everything')
    p.add_argument('--idle-others', action='store_true',
                   help='from the launch to the end of the cycle set other build, test and tool processes (compilers, test '
                        'executables, Python tools outside this run) to idle priority, so a busy machine does not take processor '
                        'time from the game; the priority is not restored')
    p.add_argument('--pause-between', type=float, default=30.0, help='seconds between cycles')
    p.add_argument('--cmd-channel', default='console', choices=['console', 'file', 'auto'], action=_ExplicitChannel,
                   help='how commands reach the game: console = typed into the CET overlay; file = cmd.txt of the test '
                        'module utpp_testcmd, confirmed by its ACK in the UTPP log (no overlay, no keys, no clicks); '
                        'auto = file when the module logged READY in the session, else console')
    p.add_argument('--diag', default='keep', choices=['keep', 'off', 'timed', 'until'],
                   help="the mod's diagnostic logs option: keep = do not touch (default); off | timed (5 minutes) | until = set "
                        'at arm and put back (previous value read first) before every ExitGame and on every exit path; if the '
                        'game is gone first the change stays in pending_restore.json and the next launch restores it')
    p.add_argument('--pose-baseline', default='',
                   help='run folder with a pose_metrics.json: print before / after per scenario and per column; the exit code '
                        'is non-zero only when a "pose_limits" block of scenarios.json is exceeded')
    p.add_argument('--no-pose-meter', action='store_true', help='do not run the pose meter on the collected recording')
    p.add_argument('--pose-timeout', type=float, default=900.0, help='seconds the pose meter workers may take after the quit')
    p.add_argument('--background', action='store_true',
                   help='the game may stay in the background: no focusing, no abort on focus loss; keyboard / mouse '
                        'input is refused (scenarios that need keys are skipped), pad reports are still sent and the '
                        'user-abort keys stay active. Needs --cmd-channel file or auto (picked automatically when none is given); '
                        '--min-idle defaults to 0 and the window recorder is used; only Pause stops the run')
    p.set_defaults(cmd_channel_explicit=False)
    p.add_argument('--body', default='keep', choices=list(gt_session.BODY_CHOICES),
                   help='the body the game starts with: female / male = that body\'s save is made the newest one (select_save.py: '
                        'times only, with a backup) while the game is closed and put back exactly after the run on every exit '
                        'path; both = one launch per body and one summary with both side by side; keep = the game\'s own choice '
                        '(default). A game that is already running is never closed for this (the run refuses). Saves: '
                        + ', '.join(f'{b} = {n}' for b, n in gt_session.BODY_SAVES.items()))
    p.add_argument('--graphics', default='keep', choices=['keep', 'low'],
                   help='low = the lowest quality profile (graphics_profile.py) before the launch, game closed; it is NOT '
                        'restored automatically (graphics_profile.py --restore); keep = settings as they are (default)')
    return p


def finalize_args(args):
    """Defaults that depend on other options: --background without an explicit --cmd-channel uses auto (the file channel when
    the test command module is there)."""
    if getattr(args, 'background', False) and not getattr(args, 'cmd_channel_explicit', False):
        args.cmd_channel = 'auto'
    return args


def validate_args(args) -> list[str]:
    errs = []
    wc = args.wheel_close
    if wc.lower().startswith('key:'):
        try:
            int(wc.split(':', 1)[1], 0)
        except ValueError:
            errs.append(f'--wheel-close {wc!r}: bad scan code')
    elif wc.upper() not in BUTTONS:
        errs.append(f'--wheel-close {wc!r}: unknown pad button')
    for m in [m.strip() for m in args.vehicle_spawn.split(',') if m.strip()]:
        if m not in ('codeware', 'vtap', 'vhold', 'summon', 'dpad'):
            errs.append(f'--vehicle-spawn: unknown mode {m!r}')
    if getattr(args, 'background', False) and getattr(args, 'cmd_channel', 'console') == 'console':
        errs.append('--background needs --cmd-channel file or auto (console typing needs the game in the foreground)')
    pb = getattr(args, 'pose_baseline', '')
    if pb and not os.path.isfile(os.path.join(pb, 'pose_metrics.json')):
        errs.append(f'--pose-baseline {pb!r}: no pose_metrics.json in that folder')
    if getattr(args, 'clips_per_cause', 0) < 0 or getattr(args, 'early_stop', 0) < 0 or getattr(args, 'post_workers', 0) < 0:
        errs.append('--clips-per-cause, --early-stop and --post-workers must not be negative')
    return errs


def install_signal_handlers(night: Night) -> None:
    def handler(signum, _frame):
        night.signal_reason = f'signal {signal.Signals(signum).name}'
        print(f'[night] {night.signal_reason}: finishing the cycle cleanly', flush=True)
    for name in ('SIGTERM', 'SIGBREAK'):
        sig = getattr(signal, name, None)
        if sig is not None:
            signal.signal(sig, handler)


def run_cycles(night: Night, args, results: list, body: str | None) -> bool:
    """--cycles cycles of one body; appends to `results`. Returns True when the whole session must stop (stop request, somebody
    at the PC, Ctrl+C)."""
    for n in range(1, args.cycles + 1):
        try:
            c = night.run_cycle(n)
            results.append({'cycle': c['name'], 'body': body, 'result': c['result'],
                            **({'pose_limits_exceeded': c['pose_limits_exceeded']} if c.get('pose_limits_exceeded') else {}),
                            **({'body_mismatch': c['body_mismatch']} if c.get('body_mismatch') else {})})
        except StopNight:
            results.append({'cycle': night.cycle['name'], 'body': body, 'result': night.cycle['result']})
            return True
        except KeyboardInterrupt:
            results.append({'cycle': night.cycle['name'] if night.cycle else None, 'body': body, 'result': 'Ctrl+C'})
            return True
        if night.requested_stop():
            print(f'[night] stopping: {night.stop_reason}')
            return True
        if n < args.cycles:
            end = time.monotonic() + args.pause_between
            while time.monotonic() < end and not night.requested_stop():
                time.sleep(1.0)
    return False


def write_body_summary(args, results: list) -> list[str]:
    """--body both (or any run with a body): the runs side by side in one file next to the night_<ts>.json."""
    digests: dict = {}
    for r in results:
        if r.get('body') and r.get('cycle'):
            digests[r['body']] = gt_session.run_digest(os.path.join(args.out_root, r['cycle']))
            digests[r['body']]['saves'] = r.get('saves')
    if not digests:
        return []
    lines = gt_session.side_by_side_lines(digests)
    stamp_ = datetime.now().strftime('%Y%m%d-%H%M%S')
    with open(os.path.join(args.out_root, f'body_summary_{stamp_}.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    with open(os.path.join(args.out_root, f'body_summary_{stamp_}.json'), 'w', encoding='utf-8') as f:
        json.dump(digests, f, indent=1, default=str)
    return lines


def main(argv=None) -> int:
    args = finalize_args(build_parser().parse_args(argv))
    if args.dry_run:
        return dry_run(args)
    scenarios = load_scenarios(args.scenario_file)
    errs = gn.validate_sets(gn.resolve_sets(args.set), scenarios) + validate_args(args)
    if errs:
        raise SystemExit('; '.join(errs))
    if not args.go:
        print('night.py does nothing without --go (use --dry-run for the plan and checks)')
        return 2
    if not IS_WINDOWS:
        raise SystemExit('needs Windows')
    if os.path.exists(stop_file(args.out_root)):
        print(f'refusing to start: {stop_file(args.out_root)} exists (delete it first)')
        return 3
    night = Night(args)
    from gt_win32 import Win32Api
    night.api = Win32Api()
    bodies = gt_session.bodies_of(args.body)
    session = gt_session.SaveSession(args.out_root, out=lambda m: print(m, flush=True))
    try:
        session.restore_leftover()       # an earlier session that did not put the saves back: first thing at every start
    except gt_session.SessionError as e:
        print(f'[body] {e}')
        if bodies:
            return 4
        print('[body] the run goes on with the game\'s own save: the leftover selection waits for the next start')
    if bodies and night.pids():
        print('refusing to start: the game is already running (started by somebody else?): a body needs its save selected with the '
              'game closed and this run does not close a game it did not start - close it first')
        return 4
    install_signal_handlers(night)
    import atexit
    keep_awake(True)
    atexit.register(keep_awake, False)
    atexit.register(lambda: session.restore() if session.selected else None)     # every exit path puts the saves back
    results: list = []
    try:
        for body in bodies or [None]:
            saves: dict = {}
            if body:
                try:
                    saves['select'] = session.select(body)
                except gt_session.SessionError as e:
                    print(f'[body] {body}: {e}')
                    results.append({'cycle': None, 'body': body, 'result': f'not started: {e}', 'saves': {'select_error': str(e)}})
                    break
            night.body = body
            try:
                stop = run_cycles(night, args, results, body)
            finally:
                if body:
                    saves['restore'] = session.restore()
                    for r in results:
                        if r.get('body') == body:
                            r['saves'] = saves
            if stop:
                break
    finally:
        keep_awake(False)
    os.makedirs(args.out_root, exist_ok=True)
    with open(os.path.join(args.out_root, f'night_{datetime.now():%Y%m%d-%H%M%S}.json'), 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=1)
    print(json.dumps(results, indent=1))
    for ln in write_body_summary(args, results):
        print(ln)
    return 0 if results and all(r['result'] == 'ok' and not r.get('pose_limits_exceeded') for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
