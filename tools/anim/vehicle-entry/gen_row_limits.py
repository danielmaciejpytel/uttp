"""Write row_limits.json (documented per-row exceptions of start_gate.py) from the MEASURED values of a sources file.

  python -I gen_row_limits.py --sources <sources-final-X.txt> [--out row_limits.json] [--prev <previous row_limits.json>] [--allow-raise ROW=reason ...]

Every row that fails the general start gate (a) and/or the rearward gate (e) gets an entry with the measured value plus a small slack (0.004 m slide, 0.005 m back / rearward), the first 16 hex of the
sha256 of its round-tripped GLB (a rebuilt clip loses the exemption) and the reason of its category. Rows that pass the general limits get no entry. Run with SG_NO_LIMITS=1 semantics (this tool sets it).

CAP (r10): a documented limit may not grow. Each limit (slide, back, rear.<name>) is compared with the same limit of the PREVIOUS file (--prev, default: the file that is overwritten); a limit above the
previous one + CAP_SLACK (0.01 m), or a limit the row did not have before (a NEW exemption), is refused (exit 4, the file is not written) unless --allow-raise ROW=reason names the row; the reason
is stored in the entry ('raise_reason'). A regression against the previous file is printed either way (rows added / removed / sha changed, every limit old -> new).
"""
import argparse
import json
import os
import sys

os.environ['SG_NO_LIMITS'] = '1'
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import start_gate as SG  # noqa: E402

REASONS = {
    'sport1': "Turbo-R lifting door: the start is 0.8 m FORWARD of the game's start (pk-door7: -0.4 .. -0.7 fail the door skin, -0.8 passes), so the clip moves rearward to the seat by design (root / hips excursion = that forward start); unchanged this round",
    'passenger': "PASSENGER row, previous clip kept (not part of the driver-side change): its joint rule passes; the start shift is the lever of its opening-door gate (the natural start + 0.3 s hold fails the opening phase by 3-15 cm, startfwd/grid7); the NPC-derived sit-down moves the hips rearward more than the game's clip; unchanged this round",
    'utility_driver': "start 0.35 m behind the game's start: every closer start fails the FULL approach sweep (ride30_door hold head 0.01-0.94 cm, absent in the baseline; startfwd screens); the hips slide rearward while sitting (sink slide 0.15-0.20 m vs 0.01 for the game) exactly like the previous clip (0.17-0.20) - an --hips-mono carry-forward lever was tried and did not close it (0.08-0.12 m left) and broke the body gate",
    'standard3_driver': "standard3 driver at the PREVIOUS shipped start (start-back 0.9 m with ramp): every forward start (dy >= -0.15) adds ride14 'entity' contacts the approach baseline (which starts 0.9 m back) does not have, so the start gate and the RELATIVE approach gate are mutually exclusive for the male clip; r10: the hips-path lever (--hips-path, build_entry_clip.py) closes the rearward gate of this clip, only the start position stays documented",
}


CAP_SLACK = 0.01  # m: how far a documented limit may exceed the previous entry's without --allow-raise


def flat_limits(e):
    out = {}
    for k in ('slide', 'back'):
        if k in e:
            out[k] = float(e[k])
    for k, v in (e.get('rear') or {}).items():
        out['rear.' + k] = float(v)
    return out


def cap_and_regress(new, prev, allow):
    """(refused [text], report lines). new / prev = {row: entry}; allow = {row: reason}."""
    refused, rep = [], []
    for row in sorted(set(new) | set(prev)):
        n, p = new.get(row), prev.get(row)
        if n is None:
            rep.append('%-28s REMOVED  (was sha %s, %s)' % (row, p.get('sha'), ', '.join('%s %.3f' % kv for kv in sorted(flat_limits(p).items()))))
            continue
        nl = flat_limits(n)
        if p is None:
            rep.append('%-28s NEW      sha %s: %s' % (row, n['sha'], ', '.join('%s %.3f' % kv for kv in sorted(nl.items()))))
            if row not in allow:
                refused.append('%s: NEW exemption (%s) without --allow-raise %s=<reason>' % (row, ', '.join(sorted(nl)), row))
            continue
        pl = flat_limits(p)
        parts = []
        for k in sorted(set(nl) | set(pl)):
            if k not in nl:
                parts.append('%s %.3f -> none' % (k, pl[k]))
            elif k not in pl:
                parts.append('%s none -> %.3f NEW' % (k, nl[k]))
                if row not in allow:
                    refused.append('%s: limit %s %.3f is new (the previous entry had none); --allow-raise %s=<reason>' % (row, k, nl[k], row))
            else:
                parts.append('%s %.3f -> %.3f' % (k, pl[k], nl[k]))
                if nl[k] > pl[k] + CAP_SLACK + 1e-9 and row not in allow:
                    refused.append('%s: limit %s %.3f exceeds the previous %.3f + %.3f; --allow-raise %s=<reason>' % (row, k, nl[k], pl[k], CAP_SLACK, row))
        rep.append('%-28s %s sha %s -> %s: %s' % (row, 'same ' if p.get('sha') == n['sha'] else 'REBUILT', p.get('sha'), n['sha'], '; '.join(parts)))
    return refused, rep


def category(row):
    if 'sport1' in row:
        return 'sport1'
    if 'passenger_r' in row:
        return 'passenger'
    if 'utility_driver' in row:
        return 'utility_driver'
    if 'standard3_driver' in row:
        return 'standard3_driver'
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sources', required=True)
    ap.add_argument('--out', default=os.path.join(HERE, 'row_limits.json'))
    ap.add_argument('--prev', default=None, help='previous row_limits.json for the cap and the regression print (default: --out when it exists)')
    ap.add_argument('--allow-raise', action='append', default=[], metavar='ROW=reason', help='lets ROW have a limit above the previous entry (+ %.2f m) or a new exemption; the reason is stored' % CAP_SLACK)
    a = ap.parse_args()
    allow = {}
    for s in a.allow_raise:
        if '=' not in s or not s.split('=', 1)[1].strip():
            raise SystemExit('--allow-raise needs ROW=reason with a non-empty reason: %r' % s)
        allow[s.split('=', 1)[0].strip()] = s.split('=', 1)[1].strip()
    prev_path = a.prev or a.out
    prev = json.load(open(prev_path, encoding='utf-8')) if os.path.exists(prev_path) else {}
    out = {}
    for ln in open(a.sources, encoding='utf-8').read().splitlines():
        if not ln.strip():
            continue
        n, d = ln.split('  ', 1)
        clip = os.path.join(d.strip(), 'roundtrip', n + '.anims.glb')
        r = SG.evaluate(clip, None, None, 'auto')
        row = n.replace('utpp_car_', '')
        e = {}
        if not r['start']['ok']:
            st = r['start']
            e.update(slide=round(st['slide_m'] + 0.004, 3), back=round(max(0.0, -st.get('dy_vs_reference_m', st['dy_vs_vanilla_m'])) + 0.005, 3))
        if not r['rearward']['ok']:
            rw = r['rearward']['rows']
            rear = {}
            for nm in ('root', 'Hips', 'Spine'):
                if not rw[nm]['ok']:
                    rear[nm] = round(rw[nm]['excursion_m'] + 0.005, 3)
            if not rw['sink_slide_Hips']['ok']:
                rear['sink'] = round(rw['sink_slide_Hips']['excursion_m'] + 0.005, 3)
            e['rear'] = rear
        if e:
            cat = category(row)
            if cat is None:
                print('NO CATEGORY for the failing row %s (%s): fix the clip instead of documenting it' % (row, e))
                sys.exit(3)
            e['sha'] = r['sha16']
            e['reason'] = REASONS[cat]
            if row in allow:
                e['raise_reason'] = allow[row]
            e['measured'] = dict(dy=r['start']['dy_vs_vanilla_m'], slide=r['start']['slide_m'], rear={k: v['excursion_m'] for k, v in r['rearward']['rows'].items()},
                                 game_rear={k: v['game_m'] for k, v in r['rearward']['rows'].items()})
            out[row] = e
        print('%-28s %s' % (row, 'EXEMPT ' + ','.join(k for k in ('slide', 'rear') if k in e) if e else 'meets the general limits'), flush=True)
    refused, rep = cap_and_regress(out, prev, allow)
    print('REGRESSION against %s (%d -> %d entries):' % (prev_path, len(prev), len(out)))
    print('\n'.join(rep))
    if refused:
        print('REFUSED (%d), %s not written:\n  %s' % (len(refused), a.out, '\n  '.join(refused)))
        sys.exit(4)
    json.dump(out, open(a.out, 'w', encoding='utf-8', newline='\n'), indent=1)
    print('wrote %s (%d entries)' % (a.out, len(out)))


if __name__ == '__main__':
    main()
