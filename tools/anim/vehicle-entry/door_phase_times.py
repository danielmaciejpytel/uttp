"""Clip frames at which the door of a class / seat moves (roofpack3 evidence that no door timing helps the roof clearance of the Turbo-R drivers).

  python -I door_phase_times.py [--class sport1] [--seat driver] [--close-after S]

Per V lag of the door model (door_timing.json): the clip frame at which the door starts to move, is fully open and starts to close. The door OPENING is the game's own event (nothing in the
mod moves it); gen_door_timing.py / UTPPCarDoorCloseDeltaS delay only the CLOSE. Default close_after = vanilla + the shipped seat delta (G.seat_delta).
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as C  # noqa: E402
import gen_door_timing as G  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--class', dest='cls', default='sport1')
    ap.add_argument('--seat', default='driver')
    ap.add_argument('--close-after', type=float)
    a = ap.parse_args()
    tj = C.load_timing()
    side = 'left' if a.seat == 'driver' else 'right'
    tm = C.side_timing(tj, a.cls, side)
    ca = a.close_after if a.close_after is not None else G.vanilla_close_after(tj, a.cls, a.seat) + G.seat_delta(tj, a.cls, a.seat)
    E, od = tm['event_clip_s'], tm['open_dur_s']
    print('%s/%s: event_clip_s %.3f open_dur_s %.3f close_dur_s %.3f close_after_s %.3f (vanilla %.3f)' % (a.cls, a.seat, E, od, tm['close_dur_s'], ca, tm['vanilla_close_after_s']))
    for lag in G.required_lags(tj, a.cls):
        print('V lag %.2f s: door starts to move at clip frame %5.1f, fully open at frame %5.1f, close starts at frame %5.1f' % (lag, (E - lag) * 30, (E + od - lag) * 30, (E + ca - lag) * 30))


if __name__ == '__main__':
    main()
