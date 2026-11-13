"""Start / stop a Windows Game Bar recording of the game from the test harness (Win+Alt+R).

  python Source\\tools\\gametest\\gamebar_record.py toggle        send Win+Alt+R (game must own the foreground; same guard as run.py)
  python Source\\tools\\gametest\\gamebar_record.py latest        print the newest capture (path, size, mtime)
  python Source\\tools\\gametest\\gamebar_record.py wait-file [--since ISO] [--timeout S]   wait until a new finished capture exists

Why: the antivirus blocks scripted screen grabbing, and the Game Bar capture is the project's established way to get frames
(Game Bar file names carry the START time of the recording; pin the offset to log events before trusting timestamps).
The chord is sent through the same ForegroundGuard as the pad input: nothing is sent unless Cyberpunk2077.exe is the
foreground window (use --focus to bring it forward first, without injecting input into another application).
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from gt_win32 import ForegroundGuard, GuardError  # noqa: E402

SCAN_LWIN = 0xE05B   # extended key
SCAN_LALT = 0x38
SCAN_R = 0x13
CAPTURES = os.path.join(os.environ.get('USERPROFILE', ''), 'Videos', 'Captures')


def newest_capture():
    if not os.path.isdir(CAPTURES):
        return None
    files = [os.path.join(CAPTURES, f) for f in os.listdir(CAPTURES) if f.lower().endswith('.mp4')]
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def toggle(focus: bool) -> int:
    guard = ForegroundGuard()
    if not guard.refresh_game():
        print('ABORT: Cyberpunk2077.exe is not running')
        return 2
    if focus:
        guard.api.focus(guard.game_hwnd)
        time.sleep(0.6)
    try:
        guard.check()
    except GuardError as e:
        print(f'ABORT: {e}')
        return 2
    guard.api.send_scan_chord([SCAN_LWIN, SCAN_LALT, SCAN_R])
    print(f'sent Win+Alt+R at {datetime.now():%H:%M:%S.%f}')
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('cmd', choices=['toggle', 'latest', 'wait-file'])
    p.add_argument('--focus', action='store_true')
    p.add_argument('--since', default='')
    p.add_argument('--timeout', type=float, default=40.0)
    a = p.parse_args()
    if a.cmd == 'toggle':
        return toggle(a.focus)
    if a.cmd == 'latest':
        f = newest_capture()
        print(f if f is None else f'{f}  {os.path.getsize(f)} bytes  {datetime.fromtimestamp(os.path.getmtime(f))}')
        return 0
    since = datetime.fromisoformat(a.since).timestamp() if a.since else time.time()
    end = time.time() + a.timeout
    last = None
    while time.time() < end:
        f = newest_capture()
        if f and os.path.getmtime(f) >= since:
            size = os.path.getsize(f)
            if last == (f, size) and size > 0:
                print(f'{f}  {size} bytes  {datetime.fromtimestamp(os.path.getmtime(f))}')
                return 0
            last = (f, size)
        time.sleep(1.5)
    print('TIMEOUT: no finished capture')
    return 1


if __name__ == '__main__':
    sys.exit(main())
