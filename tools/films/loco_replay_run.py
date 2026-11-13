"""Runs the replay for the locomotion comparison films (2026-11-09).

  python -I loco_replay_run.py
Inputs (read only): the installed v2 woman bundle of the game (red4ext\\plugins\\UTPP\\v2: utpp_motion_woman.utppmm + .chooser.json,
utpp_overlays_woman.utppov, utpp_style_weights_woman.txt; SHA-256 recorded), the synthetic feeds of loco_feeds.py.
1 identity: the official Build\\UTPP\\utpp_motion_replay.exe and the dump copy (loco_replay_build.py) with --dump, both WITHOUT --parity,
  same arguments (--slide --footlock --csv): the two CSVs must be identical (the dump patch changes nothing else).
2 film run: the copy with --parity (installed walk speed / analog floor, capsule acceleration / braking of the feeds) --slide --footlock
  --dump -> replay\\<feed>.dump.txt, .csv, .log.
Writes only under Build\\work\\loco-films-20261109\\replay; replay_inputs.json = paths, SHA-256, commands, identity result.
"""
import hashlib
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import loco_feeds as LF  # noqa: E402

ROOT = LF.ROOT
WORK = os.path.join(ROOT, 'Build', 'work', 'loco-films-20261109')
REP = os.path.join(WORK, 'replay')
sys.path.insert(0, os.path.join(HERE, '..'))
from utpp_paths import game_dir  # noqa: E402
GAME_V2 = os.path.join(game_dir(), 'red4ext', 'plugins', 'UTPP', 'data')
DB = os.path.join(GAME_V2, 'utpp_motion_woman.utppmm')
CHOOSER = os.path.join(GAME_V2, 'utpp_motion_woman.chooser.json')
OV = os.path.join(GAME_V2, 'utpp_overlays_woman.utppov')
STYLE = os.path.join(GAME_V2, 'utpp_style_weights_woman.txt')
OFFICIAL = os.path.join(ROOT, 'Build', 'UTPP', 'utpp_motion_replay.exe')
COPY = os.path.join(REP, 'loco_replay_dump.exe')
FEEDS = ['loco_film1', 'loco_film3']


def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest().upper()


def run(cmd, log):
    p = subprocess.run(cmd, capture_output=True, text=True)
    open(log, 'w', encoding='utf-8').write(' '.join('"%s"' % c for c in cmd) + '\n\n' + p.stdout + '\n' + p.stderr)
    return p


def main():
    os.makedirs(REP, exist_ok=True)
    base = ['--db', DB, '--overlay', OV, '--style', STYLE, '--slide', '--footlock']
    info = dict(inputs={p: sha(p) for p in (DB, CHOOSER, OV, STYLE, OFFICIAL, COPY)}, runs={})
    for fd in FEEDS:
        feed = os.path.join(LF.OUT, fd + '.feed')
        info['inputs'][feed] = sha(feed)
        a = os.path.join(REP, fd + '.official.csv')
        b = os.path.join(REP, fd + '.copy.csv')
        pa = run([OFFICIAL] + base + ['--csv', a, feed], os.path.join(REP, fd + '.official.log'))
        pb = run([COPY] + base + ['--csv', b, '--dump', os.path.join(REP, fd + '.identity.dump.txt'), feed], os.path.join(REP, fd + '.copy.log'))
        # column 14 'us' = wall time of the search (differs between any two runs); every other column must match
        strip = lambda p: [ln.split(',')[:13] + ln.split(',')[14:] for ln in open(p, encoding='utf-8').read().splitlines()]
        same = strip(a) == strip(b)
        film_csv = os.path.join(REP, fd + '.csv')
        dump = os.path.join(REP, fd + '.dump.txt')
        cmd = [COPY] + base + ['--parity', '--opt', 'capAccel=%g' % LF.ACCEL, '--opt', 'capBrake=%g' % LF.BRAKE, '--csv', film_csv, '--dump', dump, feed]
        pc = run(cmd, os.path.join(REP, fd + '.log'))
        info['runs'][fd] = dict(identity_csv_equal=same, official_exit=pa.returncode, copy_exit=pb.returncode, film_exit=pc.returncode,
                                film_cmd=cmd, dump=dump, csv=film_csv,
                                fails=[ln for ln in pc.stdout.splitlines() if ln.startswith('FAIL') or ln.startswith('REPLAY_FAILS')][:20],
                                fltab=[ln for ln in pc.stdout.splitlines() if ln.startswith('FLTAB') or ln.startswith('FLTOT')])
        print(fd, 'identity', same, 'exits', pa.returncode, pb.returncode, pc.returncode)
        for ln in info['runs'][fd]['fails'] + info['runs'][fd]['fltab']:
            print('  ', ln)
    json.dump(info, open(os.path.join(REP, 'replay_inputs.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
