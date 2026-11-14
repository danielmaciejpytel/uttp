"""Window capture for the game tests: records the game window's own picture while other windows cover it.

  python gt_capture.py record --title <regex> | --hwnd <n> --seconds 10 --out clip.mp4 [--ffmpeg <ffmpeg.exe>]
  python gt_capture.py check [--ffmpeg <ffmpeg.exe>]

The desktop capture modes (gdigrab / ddagrab) record the screen, so a window in front of the game ends up in the
video. This mode uses the Windows.Graphics.Capture source of ffmpeg 8.x ("gfxcapture" filter): the frames are the
window's own render target, the GPU frames go to h264_nvenc without a copy to the CPU, a window that is covered by
other windows is recorded correctly (a minimised window delivers no frames). The cursor, the yellow capture border
and the window frame are not part of the picture.

Needs an ffmpeg build with the gfxcapture filter (8.x); the project's default ffmpeg (7.1) has none, so this mode
uses its own binary: --capture-ffmpeg, else the newest Tools\\ffmpeg-<version>\\ffmpeg.exe that has the filter.

Clock: the lavfi input has no wall clock (the input banner shows "start: 0"). The `-progress` stream of ffmpeg is read
while it records; every update gives (arrival wall time, encoded video time). The smallest difference of the two is
the wall time of video time 0 plus the smallest pipeline delay (a frame or two): ProgressReader.input_start() returns
it in the form parse_input_start() uses for the desktop modes, so recording_clock() takes it as the video start.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from utpp_paths import project_root  # noqa: E402

MODE = 'window'
MAX_SIZE = (1920, 1080)          # larger windows are scaled down on the GPU (4K at cq 23 is ~25 Mbit/s, 1080p ~2)
FRAG_FLAGS = '+frag_keyframe+empty_moov+default_base_moof'   # a killed recording stays decodable
FROZEN_DUP_RATIO = 0.5           # more duplicated frames than this = the window stopped presenting
# tags of the video: the pixels are BT.601 limited range (what the other capture modes and the sheet tools decode)
COLOUR_TAGS = 'setparams=range=tv:colorspace=bt470bg:color_primaries=bt709:color_trc=iec61966-2-1'
START_TIMEOUT_S = 3.0
ESTIMATE_WINDOW_S = 60.0          # video seconds the start estimate and the drift check each look at
SOURCE_RATE_FACTOR = 4           # the source may deliver this many times the output rate; -r drops the surplus

_VERSION_RE = re.compile(r'ffmpeg-(\d+(?:\.\d+)*)')
_STATS_RE = re.compile(r'frame=\s*(\d+).*?dup=(\d+)\s+drop=(\d+)')
_filter_cache: dict = {}


def tool_version(path: str) -> tuple:
    """(8, 1) for ...\\Tools\\ffmpeg-8.1\\ffmpeg.exe, () for an unversioned folder."""
    parts = path.replace('\\', '/').split('/')
    m = _VERSION_RE.search(parts[-2]) if len(parts) > 1 else None
    return tuple(int(x) for x in m.group(1).split('.')) if m else ()


def candidate_ffmpegs(root: str | None = None) -> list[str]:
    """Tools\\ffmpeg-<version>\\ffmpeg.exe of the project, newest version first."""
    root = root or project_root()
    found = glob.glob(os.path.join(root, 'Tools', 'ffmpeg-*', 'ffmpeg.exe'))
    return sorted(found, key=tool_version, reverse=True)


def has_gfxcapture(ffmpeg: str, runner=subprocess.run) -> bool:
    """True when this ffmpeg has the gfxcapture filter (`ffmpeg -h filter=gfxcapture` describes it)."""
    if ffmpeg in _filter_cache:
        return _filter_cache[ffmpeg]
    try:
        p = runner([ffmpeg, '-hide_banner', '-h', 'filter=gfxcapture'], capture_output=True, text=True,
                   errors='replace', timeout=30)
        text = (p.stdout or '') + (p.stderr or '')
        ok = p.returncode == 0 and 'Unknown filter' not in text and 'gfxcapture AVOptions' in text
    except (OSError, subprocess.SubprocessError):
        ok = False
    _filter_cache[ffmpeg] = ok
    return ok


def find_ffmpeg(explicit: str = '', root: str | None = None, runner=subprocess.run) -> tuple[str | None, str]:
    """(path of an ffmpeg with gfxcapture or None, reason when None)."""
    if explicit:
        if not os.path.isfile(explicit):
            return None, f'--capture-ffmpeg {explicit} does not exist'
        if not has_gfxcapture(explicit, runner):
            return None, f'{explicit} has no gfxcapture filter (needs an ffmpeg 8.x build)'
        return explicit, ''
    cands = candidate_ffmpegs(root)
    for c in cands:
        if has_gfxcapture(c, runner):
            return c, ''
    if cands:
        return None, 'no Tools\\ffmpeg-<version> build has the gfxcapture filter'
    return None, 'no Tools\\ffmpeg-<version>\\ffmpeg.exe (ffmpeg 8.x with the gfxcapture filter) installed'


def output_size(w: int, h: int, max_size: tuple[int, int] = MAX_SIZE) -> tuple[int, int]:
    """Even frame size for a client area of w x h px: at most max_size (aspect kept), never enlarged."""
    if w < 2 or h < 2:
        raise ValueError(f'window size {w}x{h} is not recordable')
    k = min(1.0, max_size[0] / w, max_size[1] / h)
    ow, oh = int(w * k), int(h * k)
    return max(2, ow - ow % 2), max(2, oh - oh % 2)


def encoder_args(encoder: str, fps: int = 30) -> list[str]:
    """Encoder options for the gfxcapture frames. h264_nvenc takes the Direct3D 11 frames as they are (no -pix_fmt:
    a conversion would fail); libx264 needs them in system memory, see build_cmd. No B-frames: the video time 0 is
    then the first sample of the file."""
    gop = ['-g', str(fps * 2)]
    if encoder == 'h264_nvenc':
        return ['-c:v', 'h264_nvenc', '-preset', 'p4', '-rc', 'vbr', '-cq', '23', '-b:v', '0', '-bf', '0'] + gop
    if encoder == 'libx264':
        return ['-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '23', '-pix_fmt', 'yuv420p'] + gop
    raise ValueError(f'unknown encoder {encoder!r}')


def build_cmd(ffmpeg: str, out: str, hwnd: int, client_wh: tuple[int, int], fps: int = 30,
              encoder: str = 'h264_nvenc', max_size: tuple[int, int] = MAX_SIZE) -> list[str]:
    """Recording command of one window (fragmented mp4; stop gracefully with 'q' on stdin).

    The window is selected by its handle (a title is a regular expression and the game title has parentheses).
    The size is set explicitly with resize_mode=scale, so the whole client area is scaled on the GPU to the output
    size and the frame size stays fixed when the window is resized during the recording. The source rate limit is a
    multiple of the output rate: a limit equal to it makes the source skip frames at irregular intervals, which shows
    up as repeated frames in the output.
    -progress goes to stdout (see ProgressReader), the log to stderr."""
    if not hwnd:
        raise ValueError('no window handle')
    ow, oh = output_size(client_wh[0], client_wh[1], max_size)
    src = (f'gfxcapture=hwnd={int(hwnd)}:capture_cursor=0:display_border=0:capture_border=0:'
           f'max_framerate={fps * SOURCE_RATE_FACTOR}:width={ow}:height={oh}:resize_mode=scale')
    chain = [src, COLOUR_TAGS] if encoder == 'h264_nvenc' else [src, 'hwdownload', 'format=bgra']
    cmd = [ffmpeg, '-hide_banner', '-nostats', '-loglevel', 'info', '-progress', 'pipe:1', '-y',
           '-f', 'lavfi', '-i', ','.join(chain)]
    return cmd + encoder_args(encoder, fps) + ['-r', str(fps), '-movflags', FRAG_FLAGS, out]


def expected_size(cmd: list[str]) -> tuple[int, int] | None:
    """(w, h) of the frames a build_cmd() command produces."""
    m = re.search(r'width=(\d+):height=(\d+)', ' '.join(cmd))
    return (int(m.group(1)), int(m.group(2))) if m else None


# ---------------------------------------------------------------------------------------------- mode choice

def resolve_mode(requested: str, background: bool, explicit: bool = False) -> str:
    """--capture value -> the mode tried first. The default (desktop) records the screen, which in a background run
    shows whatever covers the game, so --background picks the window capture unless a mode was chosen explicitly."""
    if background and requested == 'desktop' and not explicit:
        return MODE
    return requested


def mode_order(requested: str, background: bool, explicit: bool = False, desktop_modes=('desktop', 'ddagrab')) -> list[str]:
    """Modes in the order they are tried. Window capture is only part of the chain when it was asked for or picked
    for a background run; the desktop modes stay the fallback."""
    first = resolve_mode(requested, background, explicit)
    rest = ([MODE] if first == MODE else []) + list(desktop_modes)
    return [first] + [m for m in rest if m != first]


def fallback_warning(mode: str, background: bool, reason: str = '') -> str:
    """Text for a recording that ended up in a desktop mode although the window capture was wanted."""
    why = f' ({reason})' if reason else ''
    if background:
        return (f'window capture unavailable{why}: recording with {mode}, which shows whatever is in front of the '
                f'game; do not use this recording to judge the picture')
    return f'window capture unavailable{why}: recording with {mode}'


# ---------------------------------------------------------------------------------------------- clock and health

class ProgressReader:
    """Reads ffmpeg `-progress pipe:1` and keeps (arrival wall time, encoded video time) pairs."""

    def __init__(self, stream=None, clock=time.time):
        self.clock = clock
        self.samples: list[tuple[float, float]] = []
        self.frames = 0
        self.ended = False
        self._lock = threading.RLock()
        self._thread = None
        if stream is not None:
            self._thread = threading.Thread(target=self._run, args=(stream,), daemon=True)
            self._thread.start()

    def _run(self, stream) -> None:
        try:
            for raw in iter(stream.readline, b''):
                self.feed(raw.decode('utf-8', 'replace') if isinstance(raw, bytes) else raw)
        except (OSError, ValueError):
            pass

    def feed(self, line: str, now: float | None = None) -> None:
        now = self.clock() if now is None else now
        key, _, val = line.strip().partition('=')
        with self._lock:
            if key == 'out_time_us':
                try:
                    us = int(val)
                except ValueError:
                    return
                if us >= 0:
                    self.samples.append((now, us / 1e6))
            elif key == 'frame':
                try:
                    self.frames = int(val)
                except ValueError:
                    pass
            elif key == 'progress' and val == 'end':
                self.ended = True

    def join(self, timeout: float = 5.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def input_start(self, min_samples: int = 2) -> tuple[str, float] | None:
        """('gfxcapture', wall clock epoch seconds of video time 0), or None with too few updates. Only the first
        ESTIMATE_WINDOW_S of the video are used: the frame times and the wall clock may drift apart over a long
        recording (see summary())."""
        with self._lock:
            pairs = [(t, v) for t, v in self.samples if v <= ESTIMATE_WINDOW_S]
        if len(pairs) < min_samples:
            return None
        return ('gfxcapture', min(t - v for t, v in pairs))

    def drift_s(self) -> float | None:
        """Start estimate of the last ESTIMATE_WINDOW_S of the video minus the one of the first: how far the video
        clock moved against the wall clock (None for a recording that is not longer than the window)."""
        with self._lock:
            pairs = list(self.samples)
        if len(pairs) < 4 or pairs[-1][1] <= ESTIMATE_WINDOW_S:
            return None
        first = [t - v for t, v in pairs if v <= ESTIMATE_WINDOW_S]
        last = [t - v for t, v in pairs if v >= pairs[-1][1] - ESTIMATE_WINDOW_S]
        return round(min(last) - min(first), 3) if first and last else None

    def delivering(self) -> bool:
        """True once frames reach the encoder (the video time advances)."""
        with self._lock:
            return self.frames > 0 and any(v > 0 for _, v in self.samples)

    def summary(self) -> dict:
        with self._lock:
            return {'updates': len(self.samples), 'frames': self.frames, 'ended': self.ended, 'start_drift_s': self.drift_s()}


def parse_dup_stats(stderr: str) -> tuple[int, int, int] | None:
    """(frames, duplicated, dropped) from the last statistics line of the ffmpeg log."""
    found = _STATS_RE.findall(stderr)
    if not found:
        return None
    f, d, dr = found[-1]
    return int(f), int(d), int(dr)


def frozen_check(stderr: str, max_dup_ratio: float = FROZEN_DUP_RATIO) -> list[str]:
    """Problems of a window recording: when the captured window stops presenting (hidden, minimised, a stalled game)
    the source delivers no frames and the output repeats the last one; the picture itself still looks normal. A log
    without statistics (ffmpeg was killed) gives no verdict."""
    st = parse_dup_stats(stderr)
    if st is None:
        return []
    frames, dup, _ = st
    if frames > 0 and dup / frames > max_dup_ratio:
        return [f'window capture delivered few new frames: {dup} of {frames} are repeats '
                f'(the window stopped presenting?)']
    return []


def short_check(duration: float | None, wall_s: float, min_ratio: float = 0.9, slack_s: float = 5.0) -> list[str]:
    """Problems of a window recording that is much shorter than the time it ran: a window that stops presenting
    delivers no frames at all, so nothing is repeated either and ffmpeg may not even see the stop request."""
    if duration is None or wall_s <= 0:
        return []
    if duration < wall_s * min_ratio and wall_s - duration > slack_s:
        return [f'window capture is {duration:.1f} s long for a {wall_s:.1f} s recording (frames stopped arriving?)']
    return []


# ---------------------------------------------------------------------------------------------- command line

def _find_window(title_regex: str) -> int:
    import ctypes
    from ctypes import wintypes
    found = []
    u = ctypes.windll.user32
    proc = ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.HWND, wintypes.LPARAM)

    def cb(h, _):
        if u.IsWindowVisible(h):
            n = u.GetWindowTextLengthW(h)
            buf = ctypes.create_unicode_buffer(n + 1)
            u.GetWindowTextW(h, buf, n + 1)
            if buf.value and re.search(title_regex, buf.value):
                found.append(int(h))
        return True
    u.EnumWindows(proc(cb), 0)
    return found[0] if found else 0


def _client_size(hwnd: int) -> tuple[int, int]:
    import ctypes
    from ctypes import wintypes
    u = ctypes.windll.user32
    set_ctx = getattr(u, 'SetThreadDpiAwarenessContext', None)
    if set_ctx:
        set_ctx.restype = ctypes.c_void_p
        set_ctx.argtypes = [ctypes.c_void_p]
        set_ctx(ctypes.c_void_p(-4))
    r = wintypes.RECT()
    u.GetClientRect(wintypes.HWND(hwnd), ctypes.byref(r))
    return r.right, r.bottom


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    rec = sub.add_parser('record', help='record one window for N seconds (a manual check of the capture)')
    rec.add_argument('--title', default='', help='regular expression of the window title')
    rec.add_argument('--hwnd', type=int, default=0)
    rec.add_argument('--seconds', type=float, default=10.0)
    rec.add_argument('--out', required=True)
    rec.add_argument('--fps', type=int, default=30)
    rec.add_argument('--encoder', default='h264_nvenc', choices=['h264_nvenc', 'libx264'])
    rec.add_argument('--ffmpeg', default='')
    chk = sub.add_parser('check', help='find the ffmpeg this mode uses')
    chk.add_argument('--ffmpeg', default='')
    a = ap.parse_args(argv)
    ff, why = find_ffmpeg(a.ffmpeg)
    if a.cmd == 'check':
        print(ff or f'unavailable: {why}')
        return 0 if ff else 1
    if not ff:
        print(f'unavailable: {why}')
        return 1
    hwnd = a.hwnd or (_find_window(a.title) if a.title else 0)
    if not hwnd:
        print('window not found (--title / --hwnd)')
        return 1
    cmd = build_cmd(ff, a.out, hwnd, _client_size(hwnd), a.fps, a.encoder)
    print(subprocess.list2cmdline(cmd))
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    reader = ProgressReader(p.stdout)
    errs = []
    threading.Thread(target=lambda: errs.append(p.stderr.read()), daemon=True).start()
    spawn = time.time()
    time.sleep(a.seconds)
    try:
        p.stdin.write(b'q')
        p.stdin.flush()
        p.wait(timeout=60)
    except Exception:  # noqa: BLE001
        p.kill()
    reader.join()
    print('rc', p.returncode, reader.summary(), 'start', reader.input_start(), 'spawn', spawn)
    return p.returncode or 0


if __name__ == '__main__':
    sys.exit(main())
