"""Acceptance gate for locomotion changes: "not worse than the baseline build", per body and per motion class.

  python gate_eval.py measure --label NAME --out NAME.json [--] <compare_runs.py arguments>
  python gate_eval.py eval NAME.json [NAME2.json ...] [--margin M.json] [--baseline BASE.json [--baseline-side after]]
                           [--json-out OUT.json]
  python gate_eval.py eval --text compare_runs_output.txt --label NAME     (table text only: no planted hip keys, lock-count weights)

measure runs compare_runs.py in this process (same arguments, its table is printed as usual) and writes a JSON with, per body and
side (before / after), every table metric plus what the table leaves out: the per-class planted sample counts of the world slide
(weights of the body total) and the planted-hip pick keys of the runtime path (start / stop PlantedHipN, Kept, OffP50Cm).

Gate, per body (each body is judged on the aggregate of its sessions):
  gated metrics (all lower = better unless marked):
    world slide per class (start, stop, pivot, loop, landing, idle, other): p50, p95 (cm/s), lock path p95 (cm);
    runtime clip changes / s; runtime foot correction p95 and max (cm);
    start / stop planted-hip picks: kept share (Kept / N, higher = better) and offset p50 (cm).
  each gated metric is 'same' (|delta| <= tolerance), 'better', 'minor' (worse by at most 2 % of the baseline) or 'worse'.
  tolerance = max(metric resolution, A/A margin of that metric when --margin is given).
  body total (lower = better) = mean of three relative changes against the baseline:
    P50 = world slide p50 averaged over the classes, weighted by the baseline planted sample count of each class;
    P95 = the same for p95, over the valid classes only;
    CC  = runtime clip changes / s.
    A class value (P50, P95) or clip change rate (CC) within its tolerance counts as unchanged. The total improves when the mean is
    below zero.
  hard limits (absolute, after side): runtime clip changes / s <= 3.0, runtime foot correction max <= 15 cm (--corr-limit; up to
  35 cm only with separate evidence of better world slide and deformation), Feed p95 <= 250 us. Deformation is not measured here.
  verdict PASS = no hard limit exceeded, no 'worse' metric, and either no 'minor' metric or a body total that improves; every
  'minor' metric of a PASS is listed as an exception with its numbers.
Rows known to be invalid offline (world slide stop p95 on both bodies, Female loop p95) are reported but left out of the decision
and of the body total; every result is also evaluated with them included and verdicts that depend on them are marked.
Replay lab rows (FLTAB, FLTOT) and Feed cost are reported, not gated (cost is gated by the hard limit only).
"""
import argparse
import contextlib
import io
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

CLASSES = ('start', 'stop', 'pivot', 'loop', 'landing', 'idle', 'other')
BODIES = ('woman', 'man')
MINOR_FRACTION = 0.02
HARD_CLIP_CHANGES = 3.0
HARD_CORR_MAX = 15.0
HARD_FEED_P95 = 250.0
# (body or None for both, metric) rows known to be invalid offline
INVALID = ((None, 'ws_stop_p95'), ('woman', 'ws_loop_p95'))

# table label -> metric key (compare_runs.py rows)
LABELS = {
    'replay: planted slide after locker, cm/s (FLTAB ALL)': 'slide_post_cms',
    'replay: planted slide p95, cm/s': 'slide_post_p95',
    'replay: foot correction p95, cm': 'corr_p95',
    'replay: foot correction max, cm (FLTOT)': 'fl_max_corr',
    'replay: pelvis |z| p95, cm (FLPEL)': 'pelvis_z_p95',
    'replay: clip changes / s': 'clip_changes_s',
    'runtime: clip changes / s': 'rt_clip_changes_s',
    'runtime: foot correction max, cm': 'rt_corr_max',
    'runtime: foot correction p95, cm': 'rt_corr_p95',
    'runtime: procedural yaw per turn, deg': 'rt_proc_turn',
    'runtime: procedural yaw share': 'rt_proc_share',
    'runtime: Feed us p50 (median of repeats)': 'rt_feed_p50',
    'runtime: Feed us p95 (median of repeats)': 'rt_feed_p95',
    'runtime: searches with the live pose history': 'rt_history',
    'runtime: stop picks': 'stop_picks',
    'runtime: stop picks, first-frame foot offset > 10 cm': 'stop_wrong',
    'runtime: stop pick foot offset p50, cm': 'stop_off_p50',
    'runtime: stop planted drift first 0.3 s mean, cm': 'stop_drift_mean',
    'runtime: stop planted drift first 0.3 s p95 (max of runs), cm': 'stop_drift_p95',
    'runtime: start picks': 'start_picks',
    'runtime: start picks, first-frame foot offset > 10 cm': 'start_wrong',
    'runtime: start pick foot offset p50, cm': 'start_off_p50',
    'runtime: start planted drift first 0.3 s mean, cm': 'start_drift_mean',
    'runtime: start planted drift first 0.3 s p95 (max of runs), cm': 'start_drift_p95',
}
for _c in CLASSES:
    LABELS['world slide %s p50, cm/s' % _c] = 'ws_%s_p50' % _c
    LABELS['world slide %s p95, cm/s' % _c] = 'ws_%s_p95' % _c
    LABELS['world slide %s locks' % _c] = 'ws_%s_locks' % _c
    LABELS['world slide %s lock path p95, cm' % _c] = 'ws_%s_path_p95' % _c


def gated_metrics():
    """[(key, higher_is_better, resolution)] of the gate."""
    out = []
    for c in CLASSES:
        out += [('ws_%s_p50' % c, False, 0.011), ('ws_%s_p95' % c, False, 0.011), ('ws_%s_path_p95' % c, False, 0.011)]
    out += [('rt_clip_changes_s', False, 0.0011), ('rt_corr_p95', False, 0.0011), ('rt_corr_max', False, 0.051)]
    for c in ('start', 'stop'):
        out += [('%sPlantedHipKeptShare' % c, True, 1e-9), ('%sPlantedHipOffP50Cm' % c, False, 0.011)]
    return out


def is_invalid(body, key):
    return any((b is None or b == body) and k == key for b, k in INVALID)


# ---------------------------------------------------------------- reading compare_runs output

ROW = re.compile(r'^(?P<label>\S.*?)\s+(?P<b>-?\d+\.\d+|-)\s+(?P<a>-?\d+\.\d+|-)\s+(?P<d>-?\d+\.\d+|-)(\s|$)')


def parse_table_text(text):
    """{body: {'before': {key: v}, 'after': {key: v}}} from the printed compare_runs.py tables."""
    out = {}
    body = None
    for line in text.splitlines():
        m = re.match(r'^== (\w+) \(', line)
        if m:
            body = m.group(1)
            out[body] = {'before': {}, 'after': {}}
            continue
        if body is None:
            continue
        r = ROW.match(line)
        if not r or r.group('label') not in LABELS:
            continue
        key = LABELS[r.group('label')]
        for side, g in (('before', 'b'), ('after', 'a')):
            if r.group(g) != '-':
                out[body][side][key] = float(r.group(g))
    for sides in out.values():
        for m in sides.values():
            for c in CLASSES:   # text only: lock counts stand in for the planted sample counts
                if 'ws_%s_locks' % c in m and 'ws_%s_n' % c not in m:
                    m['ws_%s_n' % c] = m['ws_%s_locks' % c]
    return out


def kv(line):
    return {k: v for k, v in re.findall(r'(\w+)=(\S+)', line)}


def planted_hip_from_stages(text):
    """{key: value} of the PlantedHip keys of one runtime-path output (summed counts, offset p50 kept per session)."""
    d = {}
    for line in text.splitlines():
        if line.startswith('RUNTIME') or line.startswith('PICKS'):
            d.update(kv(line))
    out = {}
    for c in ('start', 'stop'):
        n, kept, off = d.get(c + 'PlantedHipN'), d.get(c + 'PlantedHipKept'), d.get(c + 'PlantedHipOffP50Cm')
        if n is not None and kept is not None:
            out[c] = (float(n), float(kept), float(off) if off is not None else 0.0)
    return out


def aggregate_planted_hip(per_session):
    """per_session: [{c: (N, Kept, OffP50)}] -> {key: v}; offset p50 = N-weighted mean of the session p50s."""
    out = {}
    for c in ('start', 'stop'):
        rows = [s[c] for s in per_session if c in s]
        if not rows:
            continue
        n = sum(r[0] for r in rows)
        kept = sum(r[1] for r in rows)
        out[c + 'PlantedHipN'] = n
        out[c + 'PlantedHipKept'] = kept
        out[c + 'PlantedHipKeptShare'] = kept / n if n else 0.0
        out[c + 'PlantedHipOffP50Cm'] = sum(r[0] * r[2] for r in rows) / n if n else 0.0
    return out


def measure(label, cr_args):
    """Run compare_runs.py with cr_args in this process; return (exit code, result dict)."""
    import shlex
    import compare_runs as cr
    raw = []
    tables = []
    run0, ct0 = cr.run, cr.rws.class_table

    def run(cmd):
        rc, text = run0(cmd)
        raw.append((list(cmd), rc, text))
        return rc, text

    def class_table(parts):
        t = ct0(parts)
        tables.append(t)
        return t

    cr.run, cr.rws.class_table = run, class_table
    buf = io.StringIO()
    argv0 = sys.argv
    sys.argv = ['compare_runs.py'] + cr_args
    try:
        with contextlib.redirect_stdout(buf):
            rc = cr.main()
    finally:
        sys.argv = argv0
        cr.run, cr.rws.class_table = run0, ct0
    text = buf.getvalue()
    sys.stdout.write(text)
    res = parse_table_text(text)
    ap = argparse.ArgumentParser(add_help=False)
    for k in ('--before-dir', '--after-dir', '--before-stages-args', '--after-stages-args', '--runs'):
        ap.add_argument(k, nargs='+' if k == '--runs' else None, default='' if k != '--runs' else None)
    ap.add_argument('--aa', type=int, default=0)
    ap.add_argument('--world-slide', action='store_true')
    ns, _ = ap.parse_known_args(cr_args)
    bodies = []
    for spec in (ns.runs or cr.DEFAULT_RUNS):
        b = spec.split(':', 1)[0]
        if b not in bodies:
            bodies.append(b)
    sides = ['before', 'after'] + ['aa%d' % (i + 1) for i in range(ns.aa)]
    # class tables come in the order body -> side (compare_runs.py main)
    if ns.world_slide and len(tables) == len(bodies) * len(sides):
        it = iter(tables)
        for body in bodies:
            for side in sides:
                t = next(it)
                if side in ('before', 'after'):
                    m = res.setdefault(body, {'before': {}, 'after': {}})[side]
                    for c in CLASSES:
                        m['ws_%s_n' % c] = float(t[c]['n'])
                    m['ws_all_p50'], m['ws_all_p95'] = t['all']['p50'], t['all']['p95']
    # planted-hip keys from the runtime-path outputs (first repeat of every session)
    norm = lambda p: os.path.normcase(os.path.abspath(p))  # noqa: E731
    seen = {}
    for cmd, prc, ptext in raw:
        if not os.path.basename(cmd[0]).lower().startswith('utpp_replay_pose_stages') or prc != 0:
            continue
        body = cmd[cmd.index('--body') + 1]
        feed = cmd[3]
        tail = cmd[cmd.index('--body') + 2:]
        for side, d, sargs in (('before', ns.before_dir, ns.before_stages_args), ('after', ns.after_dir, ns.after_stages_args)):
            if norm(os.path.dirname(cmd[0])) == norm(d) and tail == shlex.split(sargs or '', posix=False):
                seen.setdefault((side, body, feed), planted_hip_from_stages(ptext))
    for body in bodies:
        for side in ('before', 'after'):
            per = [v for (s, b, _), v in seen.items() if s == side and b == body]
            if per:
                res.setdefault(body, {'before': {}, 'after': {}})[side].update(aggregate_planted_hip(per))
    return rc, {'label': label, 'args': cr_args, 'bodies': res}


# ---------------------------------------------------------------- gate

def weighted(m, base, key_fmt, body, include_invalid, tol=0.011):
    """Mean of key_fmt % class over the classes, weighted by the baseline planted sample counts; a class value within tol of the
    baseline counts as the baseline value. None when nothing is weighted."""
    num = den = 0.0
    for c in CLASSES:
        k = key_fmt % c
        if not include_invalid and is_invalid(body, k):
            continue
        w, v, b = base.get('ws_%s_n' % c), m.get(k), base.get(k)
        if w is None or v is None or b is None:
            continue
        num += w * (b if abs(v - b) <= tol else v)
        den += w
    return num / den if den > 0 else None


def classify(b, a, higher_better, tol):
    d = a - b
    worse = -d if higher_better else d
    if abs(d) <= tol:
        return 'same', 0.0
    rel = worse / abs(b) if abs(b) > 1e-12 else float('inf')
    if worse < 0:
        return 'better', rel
    return ('minor' if rel <= MINOR_FRACTION + 1e-12 else 'worse'), rel


def gate_body(body, before, after, margin=None, include_invalid=False, corr_limit=HARD_CORR_MAX):
    margin = margin or {}
    rows = []
    for key, hb, res in gated_metrics():
        b, a = before.get(key), after.get(key)
        if b is None or a is None:
            continue
        tol = max(res, float(margin.get(key, 0.0)))
        state, rel = classify(b, a, hb, tol)
        inv = is_invalid(body, key)
        row = dict(key=key, before=b, after=a, delta=a - b, rel=rel, state=state, invalid=inv, counted=include_invalid or not inv)
        if key.endswith('KeptShare'):   # small pick counts: one pick can move the share by tens of percent, show the counts
            k0 = key[:-len('Share')]
            n0 = key[:-len('KeptShare')] + 'N'
            if all(x in m for m in (before, after) for x in (k0, n0)):
                row['counts'] = '%d/%d -> %d/%d' % (before[k0], before[n0], after[k0], after[n0])
        rows.append(row)
    comps = {}
    for name, fmt in (('P50', 'ws_%s_p50'), ('P95', 'ws_%s_p95')):
        bv, av = weighted(before, before, fmt, body, include_invalid), weighted(after, before, fmt, body, include_invalid)
        if bv is not None and av is not None:
            comps[name] = (bv, av, 0.0)   # class tolerances applied inside weighted()
    if before.get('rt_clip_changes_s') is not None and after.get('rt_clip_changes_s') is not None:
        comps['CC'] = (before['rt_clip_changes_s'], after['rt_clip_changes_s'], max(0.0011, float(margin.get('rt_clip_changes_s', 0))))
    rels = {}
    for name, (bv, av, tol) in comps.items():
        rels[name] = 0.0 if abs(av - bv) <= tol or abs(bv) < 1e-12 else (av - bv) / abs(bv)
    # (P50 / P95: every class change beyond its resolution counts, small ones included; CC: unchanged within its tolerance)
    total = sum(rels.values()) / len(rels) if rels else None
    improves = total is not None and total < 0
    hard = []
    for key, lim in (('rt_clip_changes_s', HARD_CLIP_CHANGES), ('rt_corr_max', corr_limit), ('rt_feed_p95', HARD_FEED_P95)):
        v = after.get(key)
        if v is not None and v > lim:
            hard.append('%s %.3f > %.1f' % (key, v, lim))
    counted = [r for r in rows if r['counted']]
    worse = [r for r in counted if r['state'] == 'worse']
    minor = [r for r in counted if r['state'] == 'minor']
    ok = not hard and not worse and (not minor or improves)
    return dict(body=body, verdict='PASS' if ok else 'FAIL', hard=hard, worse=worse, minor=minor,
                exceptions=minor if ok else [], better=[r for r in counted if r['state'] == 'better'],
                invalid_rows=[r for r in rows if r['invalid']], total=total, total_parts={k: dict(before=v[0], after=v[1], rel=rels[k])
                                                                                           for k, v in comps.items()},
                improves=improves, rows=rows)


def evaluate(result, margin=None, corr_limit=HARD_CORR_MAX):
    """{body: gate} with the decision on valid rows and a flag when the verdict changes with the invalid rows included."""
    out = {}
    for body, sides in result['bodies'].items():
        mg = (margin or {}).get(body, {})
        g = gate_body(body, sides['before'], sides['after'], mg, False, corr_limit)
        gi = gate_body(body, sides['before'], sides['after'], mg, True, corr_limit)
        g['verdict_with_invalid'] = gi['verdict']
        g['depends_on_invalid'] = gi['verdict'] != g['verdict']
        out[body] = g
    return out


def fmt_row(r):
    return '%s %.3f -> %.3f (%+.3f, %+.1f %%%s)' % (r['key'], r['before'], r['after'], r['delta'],
                                                     100 * r['rel'] if r['rel'] != float('inf') else float('inf'),
                                                     ', kept ' + r['counts'] if r.get('counts') else '')


def report(label, gates):
    lines = ['## %s' % label]
    for body, g in gates.items():
        lines.append('%s: %s%s  total %s  hard: %s' % (
            body, g['verdict'], ' (with invalid rows: %s)' % g['verdict_with_invalid'] if g['depends_on_invalid'] else '',
            '%+.2f %%' % (100 * g['total']) if g['total'] is not None else '-', ', '.join(g['hard']) or 'ok'))
        for k, p in g['total_parts'].items():
            lines.append('  total %s %.3f -> %.3f (%+.2f %%)' % (k, p['before'], p['after'], 100 * p['rel']))
        for name in ('worse', 'minor', 'better'):
            for r in g[name]:
                lines.append('  %-6s %s' % (name, fmt_row(r)))
        for r in g['invalid_rows']:
            if r['state'] != 'same':
                lines.append('  invalid(not decided) %s %s' % (r['state'], fmt_row(r)))
    return '\n'.join(lines)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == 'measure':
        ap = argparse.ArgumentParser(prog='gate_eval.py measure')
        ap.add_argument('--label', required=True)
        ap.add_argument('--out', required=True)
        own, rest = ap.parse_known_args(argv[1:])
        if rest and rest[0] == '--':
            rest = rest[1:]
        rc, res = measure(own.label, rest)
        with open(own.out, 'w', encoding='utf-8') as f:
            json.dump(res, f, indent=1, sort_keys=True)
        print(report(own.label, evaluate(res)))
        return rc
    if argv and argv[0] == 'eval':
        argv = argv[1:]
    ap = argparse.ArgumentParser(prog='gate_eval.py eval')
    ap.add_argument('results', nargs='*')
    ap.add_argument('--text', nargs='*', default=[])
    ap.add_argument('--label')
    ap.add_argument('--margin')
    ap.add_argument('--corr-limit', type=float, default=HARD_CORR_MAX)
    ap.add_argument('--json-out')
    ap.add_argument('--baseline', help='result JSON whose --baseline-side replaces the before side of every input')
    ap.add_argument('--baseline-side', default='before', choices=('before', 'after'),
                    help="'after' = the after side of the baseline JSON (a build proven identical to the baseline that also prints "
                         "the planted-hip keys)")
    a = ap.parse_args(argv)
    margin = None
    if a.margin:
        with open(a.margin, encoding='utf-8') as f:
            margin = json.load(f)
    items = []
    for p in a.results:
        with open(p, encoding='utf-8') as f:
            items.append(json.load(f))
    for p in a.text:
        with open(p, encoding='utf-8', errors='replace') as f:
            items.append({'label': a.label or os.path.splitext(os.path.basename(p))[0], 'bodies': parse_table_text(f.read())})
    if a.baseline:
        # the before side of every result from one baseline measurement (a data variant measured with --root sees the variant data
        # on both sides; its after side is judged against the baseline build on the live data)
        with open(a.baseline, encoding='utf-8') as f:
            base = json.load(f)
        for it in items:
            for body, sides in it['bodies'].items():
                if body in base['bodies']:
                    sides['before'] = base['bodies'][body][a.baseline_side]
    summary = []
    for it in items:
        gates = evaluate(it, margin, a.corr_limit)
        print(report(it['label'], gates))
        summary.append(dict(label=it['label'], gates={b: {k: v for k, v in g.items() if k != 'rows'} for b, g in gates.items()}))
    # ranked table: both bodies pass first, then by the mean body total
    def score(s):
        g = s['gates']
        npass = sum(1 for v in g.values() if v['verdict'] == 'PASS')
        tot = [v['total'] for v in g.values() if v['total'] is not None]
        return (-npass, sum(tot) / len(tot) if tot else 0.0)
    print('\n| rank | variant | ' + ' | '.join('%s verdict | %s total | %s exceptions' % (b, b, b) for b in BODIES) + ' | invalid-dependent |')
    print('|---|---|' + '---|---|---|' * len(BODIES) + '---|')
    for i, s in enumerate(sorted(summary, key=score)):
        cells = []
        for b in BODIES:
            g = s['gates'].get(b)
            if not g:
                cells += ['-', '-', '-']
                continue
            cells += [g['verdict'] + (' (%s)' % ', '.join(g['hard']) if g['hard'] else ''),
                      '%+.2f %%' % (100 * g['total']) if g['total'] is not None else '-',
                      '; '.join(fmt_row(r) for r in g['exceptions']) or ('worse: ' + '; '.join(fmt_row(r) for r in g['worse'][:3])
                                                                         if g['worse'] else '-')]
        dep = ', '.join(b for b, g in s['gates'].items() if g['depends_on_invalid']) or '-'
        print('| %d | %s | %s | %s |' % (i + 1, s['label'], ' | '.join(cells), dep))
    if a.json_out:
        with open(a.json_out, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=1, sort_keys=True, default=str)
    return 0


if __name__ == '__main__':
    sys.exit(main())
