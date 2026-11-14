"""Skin helper recompute coverage by core writer, read from a UTPP log.

  python -I helper_coverage_report.py LOG [--plan plan.json [--steps] [--clock-offset S]] [--window S]
                                          [--writers driver,host] [--limit 99.5] [--min-calls N] [--since T] [--until T] [--json OUT]

The DLL writes one line about once per second while "Diagnostic logs" (or mm.trace) is on:

  UTPP skin helpers coverage: calls=N w[driver=.. host=.. vehtrans=.. neck=.. vault=.. vehguard=.. none=..]
      c[driver=.. ...] o.<writer>[<outcome>=.. ...] state[src=.. w=.. body=.. fallback=.. skip=.. detail=..]

All counts are cumulative since the DLL was loaded: calls = player FK calls; w = calls in which the writer wrote core bones of the
pose; c = of those, calls with a helper commit; o.<writer> = the other outcomes of that writer's calls (hot-gate reasons ragdoll /
gate / identity / perspective / receipt / body, build, commitrej, noeval = never reached the recompute, probe = proposal only).

The report takes differences of consecutive lines (a drop of a counter = a new DLL session: the line counts from zero again) and
attributes each difference to the window that contains the time of its line. Windows: fixed windows of --window seconds from the
first line, or with --plan the scenario runs of an in-game harness plan.json (one window per scenario run; --steps: one per step).
Coverage of a writer in a window = committed / written. A window is under the limit when a gated writer (--writers, default
driver,host: the writers that run at a published weight above 0) wrote in at least --min-calls calls and its coverage is below --limit
percent. Exit code: 0 = no window under the limit, 1 = at least one window under the limit, 2 = no coverage line found / bad input.
A session log (utpp-<start>.log) is read together with its rotated siblings.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from collections import Counter
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (os.path.join(TOOLS, 'gametest'), TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

WRITERS = ('driver', 'host', 'vehtrans', 'neck', 'vault', 'vehguard', 'none')
OUTCOMES = ('ragdoll', 'gate', 'identity', 'perspective', 'receipt', 'body', 'build', 'commitrej', 'noeval', 'probe')
TS_FMT = '%Y-%m-%d %H:%M:%S.%f'
TS_RE = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]')
LINE_RE = re.compile(r'UTPP skin helpers coverage: calls=(\d+) w\[([^\]]*)\] c\[([^\]]*)\]((?: o\.\w+\[[^\]]*\])*)(?: state\[([^\]]*)\])?')
OUT_RE = re.compile(r' o\.(\w+)\[([^\]]*)\]')
KV_RE = re.compile(r'(\w+)=(\S+)')


def _ints(text: str) -> dict:
    return {k: int(v) for k, v in KV_RE.findall(text) if v.lstrip('-').isdigit()}


def parse_lines(text: str) -> list[dict]:
    """Coverage lines of a log text, in file order: {'ts', 'calls', 'w', 'c', 'o': {writer: {outcome: n}}, 'state': {..}}."""
    out = []
    for raw in text.splitlines():
        m = LINE_RE.search(raw)
        if not m:
            continue
        t = TS_RE.match(raw)
        if not t:
            continue
        ts = datetime.strptime(t.group(1)[:26], TS_FMT)
        out.append({'ts': ts, 'calls': int(m.group(1)), 'w': _ints(m.group(2)), 'c': _ints(m.group(3)),
                    'o': {w: _ints(body) for w, body in OUT_RE.findall(m.group(4) or '')},
                    'state': dict(KV_RE.findall(m.group(5) or ''))})
    return out


def _sub(cur: dict, prev: dict) -> dict:
    return {k: v - prev.get(k, 0) for k, v in cur.items()}


def deltas(lines: list[dict]) -> list[dict]:
    """Per-line differences against the previous line. The first line counts from zero (counters start with the DLL); a drop of
    the call counter starts a new session (the line itself is the difference)."""
    out, prev = [], None
    for ln in lines:
        if prev is None or ln['calls'] < prev['calls']:
            d = {'calls': ln['calls'], 'w': dict(ln['w']), 'c': dict(ln['c']),
                 'o': {w: dict(v) for w, v in ln['o'].items()}, 'reset': prev is not None}
        else:
            d = {'calls': ln['calls'] - prev['calls'], 'w': _sub(ln['w'], prev['w']), 'c': _sub(ln['c'], prev['c']),
                 'o': {w: _sub(v, prev['o'].get(w, {})) for w, v in ln['o'].items()}, 'reset': False}
        d['ts'] = ln['ts']
        d['state'] = ln['state']
        out.append(d)
        prev = ln
    return out


def _parse_ts(text: str, ref: datetime | None) -> datetime:
    for fmt in (TS_FMT, '%Y-%m-%d %H:%M:%S'):
        try:
            return datetime.strptime(text, fmt)
        except ValueError:
            pass
    if ref is None:
        raise ValueError(f'time {text!r} needs a date (no coverage line to take it from)')
    for fmt in ('%H:%M:%S.%f', '%H:%M:%S'):
        try:
            t = datetime.strptime(text, fmt)
            return ref.replace(hour=t.hour, minute=t.minute, second=t.second, microsecond=t.microsecond)
        except ValueError:
            pass
    raise ValueError(f'unreadable time {text!r}')


def fixed_windows(ds: list[dict], seconds: float) -> list[dict]:
    if not ds:
        return []
    t0, t1 = ds[0]['ts'], ds[-1]['ts']
    out, k = [], 0
    while True:
        a = t0 + timedelta(seconds=seconds * k)
        if a > t1:
            break
        b = a + timedelta(seconds=seconds)
        out.append({'label': f'{a.strftime("%H:%M:%S")}+{seconds:g}s', 't0': a, 't1': b})
        k += 1
    return out


def plan_windows(path: str, steps: bool, clock_offset_s: float) -> list[dict]:
    """Windows from an in-game harness plan.json (tools/gametest/run.py): one per scenario run, or one per step."""
    with open(path, encoding='utf-8') as f:
        plan = json.load(f)
    off = timedelta(seconds=clock_offset_s)
    out = []
    for run in plan.get('runs', []):
        st = [s for s in run.get('steps', []) if s.get('start') and s.get('end')]
        if not st:
            continue
        name = f'{run.get("scenario", "?")}#{run.get("run", 1)}'
        if steps:
            for s in st:
                out.append({'label': f'{name}/{s["name"]}', 't0': datetime.strptime(s['start'], TS_FMT) + off,
                            't1': datetime.strptime(s['end'], TS_FMT) + off})
        else:
            out.append({'label': name, 't0': min(datetime.strptime(s['start'], TS_FMT) for s in st) + off,
                        't1': max(datetime.strptime(s['end'], TS_FMT) for s in st) + off})
    return out


def _empty():
    return {'calls': 0, 'w': Counter(), 'c': Counter(), 'o': {w: Counter() for w in WRITERS}, 'lines': 0,
            'skip': Counter(), 'detail': Counter(), 'src': Counter(), 'resets': 0}


def _add(acc: dict, d: dict) -> None:
    acc['calls'] += d['calls']
    acc['lines'] += 1
    acc['resets'] += 1 if d['reset'] else 0
    acc['w'].update(d['w'])
    acc['c'].update(d['c'])
    for w, v in d['o'].items():
        acc['o'].setdefault(w, Counter()).update(v)
    for key in ('skip', 'detail', 'src'):
        if key in d['state']:
            acc[key][d['state'][key]] += 1


def coverage(acc: dict, writer: str):
    n = acc['w'].get(writer, 0)
    return None if n <= 0 else 100.0 * acc['c'].get(writer, 0) / n


def summarise(acc: dict) -> dict:
    out = {'calls': acc['calls'], 'lines': acc['lines'], 'resets': acc['resets'], 'writers': {}}
    for w in WRITERS:
        n = acc['w'].get(w, 0)
        if n <= 0 and not acc['o'].get(w):
            continue
        out['writers'][w] = {'written': n, 'committed': acc['c'].get(w, 0), 'coverage': coverage(acc, w),
                             'outcomes': {k: v for k, v in sorted(acc['o'].get(w, {}).items()) if v}}
    for key in ('skip', 'detail', 'src'):
        out[key] = dict(acc[key].most_common(3))
    return out


def report(ds: list[dict], windows: list[dict], gated: list[str], limit: float, min_calls: int) -> dict:
    total = _empty()
    per = [(_empty(), w) for w in windows]
    outside = _empty()
    for d in ds:
        _add(total, d)
        for acc, w in per:
            if w['t0'] <= d['ts'] < w['t1']:
                _add(acc, d)
                break
        else:
            _add(outside, d)
    rows, under = [], []
    for acc, w in per:
        s = summarise(acc)
        s.update({'window': w['label'], 't0': w['t0'].strftime(TS_FMT)[:23], 't1': w['t1'].strftime(TS_FMT)[:23]})
        bad = [g for g in gated if acc['w'].get(g, 0) >= max(1, min_calls) and coverage(acc, g) is not None and coverage(acc, g) < limit]
        s['under_limit'] = bad
        rows.append(s)
        if bad:
            under.append(s)
    return {'limit': limit, 'gated': gated, 'total': summarise(total), 'outside_windows': summarise(outside),
            'windows': rows, 'under_limit': [r['window'] for r in under]}


def _fmt_cov(v) -> str:
    return '   -   ' if v is None else f'{v:6.2f}%'


def print_report(rep: dict) -> None:
    t = rep['total']
    print(f'coverage = commits / calls with a core write; limit {rep["limit"]:g} % for {",".join(rep["gated"])}')
    print(f'total: {t["calls"]} player FK calls in {t["lines"]} lines (sessions restarted: {t["resets"]})')
    for w, s in t['writers'].items():
        outs = ' '.join(f'{k}={v}' for k, v in s['outcomes'].items())
        print(f'  {w:<9} written {s["written"]:>8}  committed {s["committed"]:>8}  {_fmt_cov(s["coverage"])}  {outs}')
    print('windows:')
    for r in rep['windows']:
        parts = []
        for w, s in r['writers'].items():
            if s['written'] > 0:
                parts.append(f'{w} {s["committed"]}/{s["written"]} {_fmt_cov(s["coverage"]).strip()}')
        flag = '  UNDER ' + ','.join(r['under_limit']) if r['under_limit'] else ''
        why = ''
        for g in r['under_limit']:
            outs = r['writers'][g]['outcomes']
            why += f' [{g}: ' + ' '.join(f'{k}={v}' for k, v in outs.items()) + ']'
        state = f' skip={",".join(r["skip"]) or "-"} detail={",".join(r["detail"]) or "-"}'
        print(f'  {r["window"]:<28} {r["t0"][11:]}-{r["t1"][11:]}  ' + ('; '.join(parts) or 'no calls') + state + flag + why)
    o = rep['outside_windows']
    if o['lines']:
        print(f'outside the windows: {o["lines"]} lines, {o["calls"]} calls')
    print(f'windows under the limit: {len(rep["under_limit"])}' + (f' ({", ".join(rep["under_limit"])})' if rep['under_limit'] else ''))


def read_log(path: str) -> str:
    try:
        import gt_log  # rotation-aware reader of tools/gametest (reads utpp-<start>.log with its .1/.2 siblings)
        if gt_log.is_session_log(path):
            return gt_log.read_session_text(path)
    except ImportError:
        pass
    with open(path, 'rb') as f:
        return f.read().decode('utf-8', errors='replace')


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('log')
    p.add_argument('--plan', default='', help='in-game harness plan.json: one window per scenario run')
    p.add_argument('--steps', action='store_true', help='with --plan: one window per step')
    p.add_argument('--clock-offset', type=float, default=0.0, help='seconds added to the plan times (as tools/gametest/run.py)')
    p.add_argument('--window', type=float, default=10.0, help='fixed window length in seconds (without --plan)')
    p.add_argument('--writers', default='driver,host', help='writers held to the limit (default driver,host)')
    p.add_argument('--limit', type=float, default=99.5, help='coverage limit in percent (default 99.5)')
    p.add_argument('--min-calls', type=int, default=1, help='a writer is judged in a window from this many calls (default 1)')
    p.add_argument('--since', default='', help='ignore lines before this time (YYYY-mm-dd HH:MM:SS[.f] or HH:MM:SS)')
    p.add_argument('--until', default='', help='ignore lines after this time')
    p.add_argument('--json', default='', help='write the report as JSON')
    a = p.parse_args(argv)
    try:
        text = read_log(a.log)
    except OSError as e:
        print(f'cannot read {a.log}: {e}')
        return 2
    lines = parse_lines(text)
    if not lines:
        print('no "UTPP skin helpers coverage" line in the log (Diagnostic logs / mm.trace off, or a DLL without the counters)')
        return 2
    ds = deltas(lines)
    ref = lines[0]['ts']
    try:
        if a.since:
            since = _parse_ts(a.since, ref)
            ds = [d for d in ds if d['ts'] >= since]
        if a.until:
            until = _parse_ts(a.until, ref)
            ds = [d for d in ds if d['ts'] <= until]
    except ValueError as e:
        print(e)
        return 2
    gated = [w.strip() for w in a.writers.split(',') if w.strip()]
    unknown = [w for w in gated if w not in WRITERS]
    if unknown:
        print(f'unknown writer(s): {", ".join(unknown)} (known: {", ".join(WRITERS)})')
        return 2
    try:
        windows = plan_windows(a.plan, a.steps, a.clock_offset) if a.plan else fixed_windows(ds, a.window)
    except (OSError, ValueError, KeyError) as e:
        print(f'cannot read the plan {a.plan}: {e}')
        return 2
    rep = report(ds, windows, gated, a.limit, a.min_calls)
    print_report(rep)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(rep, f, indent=1)
    return 1 if rep['under_limit'] else 0


if __name__ == '__main__':
    sys.exit(main())
