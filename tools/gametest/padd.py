"""padd - persistent virtual Xbox 360 pad daemon for the UTPP game-test harness.

Keeps ONE vgamepad device plugged in between harness runs, so the game never shows its modal
"controller disconnected" dialog (which pauses the game and blocks CET/input). run.py --keep-pad starts it on demand.

  python Source\\tools\\gametest\\padd.py [--port 47321]          (normally started by run.py --keep-pad)
  python Source\\tools\\gametest\\run.py --padd-status | --padd-stop

Protocol: 127.0.0.1 TCP only, one JSON object per line, one reply per line:
  {"op":"ping"}                  -> {"ok":true,"pid":..,"frames":..,"game_pids":[..]}
  {"op":"frame","ls":[x,y],"rs":[x,y],"lt":v,"rt":v,"b":mask}
                                 -> {"ok":true} | {"ok":false,"error":"..."}  (pad released on error)
  {"op":"release"}               -> {"ok":true}
  {"op":"quit"}                  -> {"ok":true}; pad released and unplugged, daemon exits
Safety: the daemon runs its own ForegroundGuard before every non-neutral report (game process must own the
foreground window); a neutral report is always allowed. Watchdog: if a non-neutral report is not refreshed for
`--watchdog` seconds (client died / hung) or a client disconnects, the pad is released.
Note: quitting unplugs the pad -> the game shows the dialog; dismiss it with run.py --dismiss-dialog.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from gt_input import NEUTRAL, PADD_HOST, PADD_PORT, import_vgamepad, msg_to_frame, set_report  # noqa: E402
from gt_win32 import ForegroundGuard, GuardError  # noqa: E402


def is_neutral(fr) -> bool:
    return fr.ls == (0.0, 0.0) and fr.rs == (0.0, 0.0) and fr.lt == 0.0 and fr.rt == 0.0 and fr.buttons == 0


class PadServer:
    def __init__(self, pad, guard: ForegroundGuard, watchdog: float = 0.3, log=print):
        self.pad = pad
        self.guard = guard
        self.watchdog = watchdog
        self.log = log
        self.frames = 0
        self.active = False          # last report non-neutral
        self.last_frame = 0.0
        self.running = True

    def release(self) -> None:
        set_report(self.pad, NEUTRAL)
        self.pad.update()
        self.active = False

    def handle(self, msg: dict) -> dict:
        op = msg.get('op')
        if op == 'ping':
            return {'ok': True, 'pid': os.getpid(), 'frames': self.frames, 'game_pids': self.guard.game_pids}
        if op == 'release':
            self.release()
            return {'ok': True}
        if op == 'quit':
            self.release()
            self.running = False
            return {'ok': True}
        if op == 'frame':
            try:
                fr = msg_to_frame(msg)
            except (TypeError, ValueError, IndexError) as e:
                self.release()
                return {'ok': False, 'error': f'bad frame: {e}'}
            if not is_neutral(fr):
                try:
                    # A persistent daemon outlives game processes. Refresh at
                    # every neutral -> active boundary, before checking cached
                    # liveness; otherwise its first report after a restart fails
                    # on the previous session's nonempty PID cache.
                    if (not self.active or not self.guard.game_pids) and not self.guard.refresh_game():
                        raise GuardError('Cyberpunk2077.exe is not running')
                    self.guard.check()
                except GuardError as e:
                    self.release()
                    if 'not running' in str(e):
                        self.guard.game_pids = []   # game may be restarted later
                    return {'ok': False, 'error': str(e)}
            set_report(self.pad, fr)
            self.pad.update()
            self.frames += 1
            self.active = not is_neutral(fr)
            self.last_frame = time.monotonic()
            return {'ok': True}
        return {'ok': False, 'error': f'unknown op {op!r}'}

    def tick_watchdog(self) -> None:
        if self.active and time.monotonic() - self.last_frame > self.watchdog:
            self.log('[padd] watchdog: no fresh report, pad released')
            self.release()

    def serve(self, host: str, port: int) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind((host, port))       # loopback only
        srv.listen(1)
        srv.settimeout(0.1)
        self.log(f'[padd] pid {os.getpid()} listening on {host}:{port}')
        try:
            while self.running:
                try:
                    conn, _ = srv.accept()
                except socket.timeout:
                    self.tick_watchdog()
                    continue
                self._client(conn)
        finally:
            srv.close()
            self.release()

    def _client(self, conn) -> None:
        conn.settimeout(0.1)
        buf = b''
        try:
            while self.running:
                try:
                    data = conn.recv(65536)
                except socket.timeout:
                    self.tick_watchdog()
                    continue
                if not data:
                    break
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    try:
                        msg = json.loads(line.decode('utf-8'))
                        reply = self.handle(msg if isinstance(msg, dict) else {})
                    except ValueError:
                        reply = {'ok': False, 'error': 'bad json'}
                    conn.sendall((json.dumps(reply) + '\n').encode('utf-8'))
                self.tick_watchdog()
        except OSError:
            pass
        finally:
            conn.close()
            if self.active:
                self.release()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=PADD_PORT)
    ap.add_argument('--watchdog', type=float, default=0.3)
    a = ap.parse_args(argv)

    def log(msg):
        print(f'{time.strftime("%Y-%m-%d %H:%M:%S")} {msg}', flush=True)

    pad = import_vgamepad().VX360Gamepad()
    guard = ForegroundGuard()
    guard.refresh_game()
    server = PadServer(pad, guard, a.watchdog, log)
    try:
        server.serve(PADD_HOST, a.port)
    except KeyboardInterrupt:
        pass
    finally:
        server.release()
        log('[padd] exit (virtual pad unplugged)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
