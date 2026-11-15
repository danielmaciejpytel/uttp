"""Reads the reload diagnostic record of a game session (utpp-reload-<stamp>.bin, written while the diagnostic logs are on) and prints it
as one table per reload episode, with per-frame flags.

  python -I reload_trace.py RUN [--from HH:MM:SS[.mmm]] [--to HH:MM:SS[.mmm]] [--episode N] [--flagged] [--summary]

RUN         a run folder (its logs\\ subfolder is searched too), a folder with utpp-reload-*.bin files, or one such file
--from/--to local wall-clock window (the times of the mod log)        --episode N  only episode N (1-based, in file order)
--flagged   only frames with a flag (plus the first frame of each episode)        --summary  one line per episode, no frame table

An episode is a run of records with less than 0.25 s between them. Flags per frame:
  CLAV   a written clavicle translation differs from V's own (snapshot) by more than 1 cm
  WSTEP  the contact weight, the hand-back weight (1 - upper scale), the correction weight after the guards or the placement gate
         changes by more than max(0.2, 1.5 x dt / 0.12 s) in one frame (the fastest ramp of the plugin is 0.12 s; dt is the frame
         time, at most 50 ms)
  DROP   records are missing before this one (the ring was busy or full: gap in the push numbers)
  ORDER  the push number went back (two hook calls pushed at the same time; the rows are in file order, not call order)
  MASK   the hand-back mask bits (clavicles, arms, hands, chest, head, weapon) changed since the previous frame
  ELB    the left elbow (solve output or written) turned by more than 30 deg about the shoulder in one frame
  REJ    the correction did not accept the frame (guard cut, solve reject, failed stage); REPLAY marks a written replay
Columns: t = local time; path F finish / H hold / - none / L lock lost; exit = where the correction left; w = contact weight;
hb = hand-back weight (1 - upper scale); cw = correction weight after the guards; pg = placement gate; gw = girdle weight;
amp / hw / hgw = reject hold amplitude, recorded weight, recorded girdle weight; mask letters: C clavicles, A upper arms, F forearms,
H hands, S chest, N head, W weapon (lower case = only one side); dL / dR = written clavicle minus V's own (cm); gL = game clavicle
minus V's own (cm, left); elb / elbO = left elbow turn about the shoulder since the previous frame (solve / written, deg);
gaz / eaz = change of the azimuth about the shoulder -> hand target axis of the game's elbow (the solve's guide) / of the solved elbow;
hG / hT / hO / ch = turn of the game's hand, the hand target, the written hand and V's chest since the previous frame (deg);
twist / pron / bend = the written left arm, sbend = the wrist bend of the solve before any limit (deg); jl = joint limiter: limited bones (lc lu lf rc ru rf) and the largest request (deg).
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import struct
import sys
from datetime import datetime, timedelta

MAGIC = b'UTPPRLDT'
FORMAT = 1
HEADER_BYTES = 64
RECORD_BYTES = 432

# (name, count, type) in record order after the 32-byte head; type 'f' float32, 'I' uint32. Mirrors src/mm/ReloadTrace.hpp.
LANES = [
    ('snapWeight', 1, 'f'), ('upperScale', 1, 'f'), ('lowerScale', 1, 'f'), ('contactWeight', 1, 'f'),
    ('corrIn', 1, 'f'), ('divergence', 1, 'f'), ('lambda', 1, 'f'), ('placementRaw', 1, 'f'), ('placementGate', 1, 'f'),
    ('corrFinal', 1, 'f'), ('girdleWeight', 1, 'f'),
    ('holdAmp', 1, 'f'), ('holdWeight', 1, 'f'), ('holdGirdleWeight', 1, 'f'), ('replayOwnWeight', 1, 'f'),
    ('stockShift', 1, 'f'), ('leftArmShift', 1, 'f'),
    ('clavGame', 6, 'f'), ('clavIn', 6, 'f'), ('clavOwn', 6, 'f'), ('clavOut', 6, 'f'),
    ('jointReqDeg', 6, 'f'), ('jointMaxDeg', 1, 'f'), ('jointLimited', 1, 'I'),
    ('shoulder', 3, 'f'), ('handTarget', 3, 'f'), ('handTargetRot', 4, 'f'), ('handGameRot', 4, 'f'), ('chestRot', 4, 'f'),
    ('guide', 3, 'f'),
    ('elbowSolve', 3, 'f'), ('elbowSwivel', 3, 'f'), ('elbowOut', 3, 'f'), ('handOutRot', 4, 'f'),
    ('reachClavDeg', 1, 'f'), ('swivelDeg', 1, 'f'),
    ('anat', 15, 'f'),
]
HEAD_FMT = '<qQIIBBBBb3B'  # ..., action, pushNumber (24 bits, little endian)
HEADER_FMT = '<8sIIqq8H16x'


def lane_offsets() -> dict[str, int]:
    """Byte offset of every field (the C++ test checks the same numbers)."""
    out = {'ticks': 0, 'sequence': 8, 'flags': 16, 'mask': 20, 'path': 24, 'exit': 25, 'solveReject': 26, 'overlay': 27, 'action': 28}
    off = struct.calcsize(HEAD_FMT)
    for name, n, _ in LANES:
        out[name] = off
        off += 4 * n
    out['_end'] = off
    return out


BODY_FMT = '<' + ''.join(t * n for _, n, t in LANES)
assert struct.calcsize(HEAD_FMT) + struct.calcsize(BODY_FMT) == RECORD_BYTES

PATHS = {0: '-', 1: 'F', 2: 'H', 3: 'L'}
EXITS = {0: '', 1: 'notcap', 2: 'badw', 3: 'w0', 4: 'read', 5: 'diverg', 6: 'place', 7: 'guard0', 8: 'restore', 9: 'solve', 10: 'ok'}
REJECTS = {0: '', 1: 'input', 2: 'reach', 3: 'pole', 4: 'solve', 5: 'contact', 6: 'binding'}
FLAG_BITS = {'eligible': 0, 'authored': 1, 'admitted': 2, 'config': 3, 'captured': 4, 'permitted': 5, 'reachedSolver': 6, 'written': 7,
             'replay': 8, 'replayGirdle': 9, 'holdDelta': 10, 'finalLimit': 11, 'swivel': 12, 'ownerMapping': 13, 'aiming': 14,
             'jointsValid': 15}
MASK_BITS = ['clavL', 'clavR', 'upperL', 'upperR', 'foreL', 'foreR', 'handL', 'handR', 'spine3', 'head', 'weaponR']
CLAV_FLAG_M = 0.01
WSTEP = 0.2
PUSH_NUMBER_WRAP = 1 << 24  # Record::pushNumber is 24 bits
ELBOW_DEG = 30.0


class Record(dict):
    def flag(self, name: str) -> bool:
        return bool(self['flags'] >> FLAG_BITS[name] & 1)


def pack_header(qpc_hz: int, qpc_at_open: int, local: tuple, record_bytes: int = RECORD_BYTES, fmt: int = FORMAT) -> bytes:
    """Header bytes (for tests)."""
    lt = list(local) + [0] * (8 - len(local))
    return struct.pack(HEADER_FMT, MAGIC, fmt, record_bytes, qpc_hz, qpc_at_open, *lt[:8])


def pack_record(**fields) -> bytes:
    """Record bytes from field values (missing floats NaN, integers 0); for tests."""
    head = struct.pack(HEAD_FMT, fields.get('ticks', 0), fields.get('sequence', 0), fields.get('flags', 0), fields.get('mask', 0),
                       fields.get('path', 0), fields.get('exit', 0), fields.get('solveReject', 0), fields.get('overlay', 0),
                       fields.get('action', 0), *(fields.get('pushNumber', 0) >> s & 0xFF for s in (0, 8, 16)))
    vals = []
    for name, n, t in LANES:
        v = fields.get(name)
        if v is None:
            v = [0 if t == 'I' else math.nan] * n
        elif n == 1:
            v = [v]
        v = list(v)
        if len(v) != n:
            raise ValueError(f'{name}: {n} values expected')
        vals += v
    return head + struct.pack(BODY_FMT, *vals)


def read_file(path: str) -> tuple[dict, list[Record]]:
    """Header and records of one file. A truncated last record (the game was closed while writing) is ignored."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < HEADER_BYTES:
        raise ValueError(f'{os.path.basename(path)}: shorter than the header')
    magic, fmt, rb, hz, q0, *lt = struct.unpack_from(HEADER_FMT, data, 0)
    if magic != MAGIC:
        raise ValueError(f'{os.path.basename(path)}: not a reload record file')
    if fmt != FORMAT or rb != RECORD_BYTES:
        raise ValueError(f'{os.path.basename(path)}: format {fmt} / record size {rb} B, this reader knows format {FORMAT} / {RECORD_BYTES} B')
    header = {'qpcHz': hz, 'qpcAtOpen': q0, 'local': datetime(lt[0], lt[1], lt[2], lt[3], lt[4], lt[5], lt[6] * 1000),
              'truncatedBytes': (len(data) - HEADER_BYTES) % RECORD_BYTES, 'file': os.path.basename(path)}
    recs = []
    for i in range((len(data) - HEADER_BYTES) // RECORD_BYTES):
        off = HEADER_BYTES + i * RECORD_BYTES
        h = struct.unpack_from(HEAD_FMT, data, off)
        body = struct.unpack_from(BODY_FMT, data, off + struct.calcsize(HEAD_FMT))
        r = Record(ticks=h[0], sequence=h[1], flags=h[2], mask=h[3], path=h[4], exit=h[5], solveReject=h[6], overlay=h[7], action=h[8],
                   pushNumber=h[9] | h[10] << 8 | h[11] << 16)
        k = 0
        for name, n, _ in LANES:
            r[name] = body[k] if n == 1 else list(body[k:k + n])
            k += n
        r['time'] = header['local'] + timedelta(seconds=(r['ticks'] - q0) / hz) if hz > 0 else header['local']
        recs.append(r)
    return header, recs


def find_files(target: str) -> list[str]:
    if os.path.isfile(target):
        return [target]
    files = []
    for d in (target, os.path.join(target, 'logs')):
        files += glob.glob(os.path.join(d, 'utpp-reload-*.bin'))
    return sorted(set(files))


def episodes(recs: list[Record], gap_s: float = 0.25) -> list[list[Record]]:
    out: list[list[Record]] = []
    for r in recs:
        if out and (r['time'] - out[-1][-1]['time']).total_seconds() <= gap_s and r['time'] >= out[-1][-1]['time']:
            out[-1].append(r)
        else:
            out.append([r])
    return out


def _fin(*v) -> bool:
    return all(isinstance(x, (int, float)) and math.isfinite(x) for x in v)


def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


def _norm(a) -> float:
    return math.sqrt(sum(x * x for x in a))


def _perp(v, axis):
    """v without its component along axis."""
    n = _norm(axis)
    if not n > 1e-9:
        return [math.nan] * 3
    u = [x / n for x in axis]
    d = sum(x * y for x, y in zip(v, u))
    return [x - d * y for x, y in zip(v, u)]


def _angle_vec(a, b) -> float:
    na, nb = _norm(a), _norm(b)
    if not (na > 1e-9 and nb > 1e-9) or not _fin(*a, *b):
        return math.nan
    c = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _angle_quat(a, b) -> float:
    if not _fin(*a, *b):
        return math.nan
    na, nb = _norm(a), _norm(b)
    if not (na > 1e-9 and nb > 1e-9):
        return math.nan
    d = abs(sum(x * y for x, y in zip(a, b))) / (na * nb)
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, d))))


def mask_text(m: int) -> str:
    def pair(l, r, ch):
        lb, rb = m >> MASK_BITS.index(l) & 1, m >> MASK_BITS.index(r) & 1
        return ch if lb and rb else (ch.lower() if lb or rb else '.')
    single = lambda name, ch: ch if m >> MASK_BITS.index(name) & 1 else '.'  # noqa: E731
    return (pair('clavL', 'clavR', 'C') + pair('upperL', 'upperR', 'A') + pair('foreL', 'foreR', 'F') + pair('handL', 'handR', 'H')
            + single('spine3', 'S') + single('head', 'N') + single('weaponR', 'W'))


def analyse(ep: list[Record]) -> list[dict]:
    """Per-frame derived values and flags of one episode."""
    rows = []
    prev = None
    shoulder = None
    for r in ep:
        d = {}
        if _fin(*r['shoulder']):
            shoulder = r['shoulder']
        cdev = []
        for side in (0, 1):
            out, own = r['clavOut'][3 * side:3 * side + 3], r['clavOwn'][3 * side:3 * side + 3]
            cdev.append(_norm(_sub(out, own)) if _fin(*out, *own) else math.nan)
        d['dL'], d['dR'] = cdev
        g, o = r['clavGame'][0:3], r['clavOwn'][0:3]
        d['gL'] = _norm(_sub(g, o)) if _fin(*g, *o) else math.nan
        d['hb'] = 1.0 - r['upperScale'] if _fin(r['upperScale']) else math.nan
        flags = []
        if any(_fin(x) and x > CLAV_FLAG_M for x in cdev):
            flags.append('CLAV')
        d['elb'] = d['elbO'] = d['hG'] = d['hT'] = d['hO'] = d['ch'] = d['gaz'] = d['eaz'] = math.nan
        if prev is not None:
            pr, pd = prev
            steps = []
            for key in ('contactWeight', 'corrFinal', 'placementGate'):
                if _fin(r[key], pr[key]):
                    steps.append(abs(r[key] - pr[key]))
            if _fin(d['hb'], pd['hb']):
                steps.append(abs(d['hb'] - pd['hb']))
            dt = (r['time'] - pr['time']).total_seconds()
            if any(s > max(WSTEP, 1.5 * min(dt, 0.05) / 0.12) for s in steps):
                flags.append('WSTEP')
            gap = (r['pushNumber'] - pr['pushNumber']) % PUSH_NUMBER_WRAP
            if gap >= PUSH_NUMBER_WRAP // 2:
                flags.append('ORDER')
            elif gap > 1:
                flags.append('DROP')
            if r['mask'] != pr['mask']:
                flags.append('MASK')
            if shoulder is not None:
                for key, col in (('elbowSolve', 'elb'), ('elbowOut', 'elbO')):
                    if _fin(*r[key], *pr[key]):
                        d[col] = _angle_vec(_sub(r[key], shoulder), _sub(pr[key], shoulder))
            for key, col in (('handGameRot', 'hG'), ('handTargetRot', 'hT'), ('handOutRot', 'hO'), ('chestRot', 'ch')):
                d[col] = _angle_quat(r[key], pr[key])
            # azimuth change about this frame's shoulder -> hand target axis: the game's elbow (guide) and the solved elbow
            if shoulder is not None and _fin(*r['handTarget']):
                axis = _sub(r['handTarget'], shoulder)
                for key, col in (('guide', 'gaz'), ('elbowSolve', 'eaz')):
                    if _fin(*r[key], *pr[key]):
                        d[col] = _angle_vec(_perp(_sub(r[key], shoulder), axis), _perp(_sub(pr[key], shoulder), axis))
            if any(_fin(d[c]) and d[c] > ELBOW_DEG for c in ('elb', 'elbO')):
                flags.append('ELB')
        if r['path'] == 1 and r['exit'] not in (3, 10):
            flags.append('REJ')
        if r.flag('replay'):
            flags.append('REPLAY')
        d['flags'] = flags
        rows.append(d)
        prev = (r, d)
    return rows


def _f(v, fmt='{:5.2f}', width=5) -> str:
    return fmt.format(v) if _fin(v) else '-'.rjust(width)


def frame_line(r: Record, d: dict) -> str:
    an = r['anat']
    jl = ''.join(n if r['jointLimited'] >> k & 1 else '.' for k, n in enumerate(('c', 'u', 'f', 'c', 'u', 'f')))
    req = max([x for x in r['jointReqDeg'] if _fin(x)] or [math.nan])
    ex = EXITS.get(r['exit'], str(r['exit'])) + ('/' + REJECTS.get(r['solveReject'], '?') if r['exit'] == 9 else '')
    return (f"{r['time'].strftime('%H:%M:%S.%f')[:-3]} {PATHS.get(r['path'], '?')} {ex:<13} "
            f"{_f(r['contactWeight'])} {_f(d['hb'])} {_f(r['corrFinal'])} {_f(r['placementGate'])} {_f(r['girdleWeight'])} "
            f"{_f(r['holdAmp'])} {_f(r['holdWeight'])} {_f(r['holdGirdleWeight'])} {mask_text(r['mask'])} "
            f"{_f(d['dL'] * 100, '{:5.1f}')} {_f(d['dR'] * 100, '{:5.1f}')} {_f(d['gL'] * 100, '{:5.1f}')} "
            f"{_f(d['elb'], '{:5.1f}')} {_f(d['elbO'], '{:5.1f}')} {_f(d['gaz'], '{:5.1f}')} {_f(d['eaz'], '{:5.1f}')} "
            f"{_f(d['hG'], '{:5.1f}')} {_f(d['hT'], '{:5.1f}')} "
            f"{_f(d['hO'], '{:5.1f}')} {_f(d['ch'], '{:5.1f}')} "
            f"{_f(an[12], '{:6.1f}', 6)} {_f(an[13], '{:6.1f}', 6)} {_f(an[14], '{:5.1f}')} {_f(an[5], '{:5.1f}')} "
            f"{jl} {_f(req, '{:5.1f}')}  {' '.join(d['flags'])}")


# anat rows (3 values each: upper-arm twist, pronation, wrist bend): 0 game's own, 1 solve, 2 after swivel, 3 after joint limiter, 4 written
HEADER_LINE = ('t            p exit          w     hb    cw    pg    gw    amp   hw    hgw   mask    dL    dR    gL    elb   elbO  '
               'gaz   eaz   hG    hT    hO    ch    twist   pron   bend  sbend jl     req   flags')


def episode_summary(i: int, ep: list[Record], rows: list[dict]) -> str:
    counts: dict[str, int] = {}
    for d in rows:
        for f in d['flags']:
            counts[f] = counts.get(f, 0) + 1
    exits: dict[str, int] = {}
    for r in ep:
        k = EXITS.get(r['exit'], str(r['exit'])) if r['path'] == 1 else {0: 'none', 2: 'hold', 3: 'locklost'}.get(r['path'], '?')
        exits[k] = exits.get(k, 0) + 1
    dmax = max([max(d['dL'], d['dR']) for d in rows if _fin(d['dL'], d['dR'])] or [math.nan])
    owner = 'mapping' if ep[0].flag('ownerMapping') else 'fk'
    # push numbers unwrapped (a step back of more than half the range is a reorder); missing = numbers in the span without a record
    unwrapped, u = [0], 0
    for a, b in zip(ep, ep[1:]):
        g = (b['pushNumber'] - a['pushNumber']) % PUSH_NUMBER_WRAP
        u += g - PUSH_NUMBER_WRAP if g >= PUSH_NUMBER_WRAP // 2 else g
        unwrapped.append(u)
    lost = max(0, max(unwrapped) - min(unwrapped) + 1 - len(set(unwrapped)))
    return (f"episode {i}: {ep[0]['time'].strftime('%H:%M:%S.%f')[:-3]} - {ep[-1]['time'].strftime('%H:%M:%S.%f')[:-3]} "
            f"{len(ep)} frames ({lost} dropped), overlay {ep[0]['overlay']}, owner {owner}, exits {exits}, flags {counts}, "
            f"largest clavicle difference {_f(dmax * 100, '{:.1f}', 1)} cm")


def parse_clock(text: str | None, day: datetime) -> datetime | None:
    if not text:
        return None
    for fmt in ('%H:%M:%S.%f', '%H:%M:%S'):
        try:
            t = datetime.strptime(text, fmt)
            return day.replace(hour=t.hour, minute=t.minute, second=t.second, microsecond=t.microsecond)
        except ValueError:
            pass
    raise ValueError(f'time {text!r}: HH:MM:SS[.mmm] expected')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('run')
    ap.add_argument('--from', dest='t0')
    ap.add_argument('--to', dest='t1')
    ap.add_argument('--episode', type=int)
    ap.add_argument('--flagged', action='store_true')
    ap.add_argument('--summary', action='store_true')
    a = ap.parse_args(argv)
    files = find_files(a.run)
    if not files:
        print(f'no utpp-reload-*.bin in {a.run}', file=sys.stderr)
        return 1
    n = 0
    for path in files:
        try:
            header, recs = read_file(path)
        except ValueError as e:
            print(f'ERROR {e}', file=sys.stderr)
            return 1
        print(f"{header['file']}: {len(recs)} records, opened {header['local']:%Y-%m-%d %H:%M:%S}"
              + (f", last record truncated ({header['truncatedBytes']} B ignored)" if header['truncatedBytes'] else ''))
        t0, t1 = parse_clock(a.t0, header['local']), parse_clock(a.t1, header['local'])
        recs = [r for r in recs if (t0 is None or r['time'] >= t0) and (t1 is None or r['time'] <= t1)]
        for ep in episodes(recs):
            n += 1
            if a.episode and n != a.episode:
                continue
            rows = analyse(ep)
            print(episode_summary(n, ep, rows))
            if a.summary:
                continue
            print('  ' + HEADER_LINE)
            for k, (r, d) in enumerate(zip(ep, rows)):
                if a.flagged and not d['flags'] and k:
                    continue
                print('  ' + frame_line(r, d))
    return 0


if __name__ == '__main__':
    sys.exit(main())
