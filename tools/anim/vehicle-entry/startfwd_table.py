"""Per-row table of the start-forward rework (round 4): the 24 rows of a sources file ('<clip>  <run dir>' lines) with

  frame-0 root vs the game's clip start (dx / dy / slide) and vs the door-aperture rear edge, joint rule, body worst (clearance cm, capsule, gate), roof, door phases at the
  solved close (closed / opening / open / closing / closed_after), approach gate (run's approach-<clip>.txt RESULT line, 'n/a' when none), peak head-duck bend, close_after_s.

  python -I startfwd_table.py --sources <sources-final-X.txt> [--verdict-dir <dir with door-close-<clip>.json>] [--base <sources.txt>] [--txt table.txt] [--json table.json]

The door numbers come from the verdict fragment (door-close-<clip>.json: check_door_clearance.py --solve; its at_close_after record) - the verdict dir first, then the run dir; the start
position / joint rule are recomputed here from the round-tripped clip. A row whose verdict lacks the body phase is listed as NO-BODY-VERDICT. With --base the previous set's start numbers
are printed next to the new ones.
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as C  # noqa: E402
import start_gate as SG  # noqa: E402


def read_sources(p):
    out = {}
    for ln in open(p, encoding='utf-8').read().splitlines():
        if ln.strip():
            n, d = ln.split('  ', 1)
            out[n.strip()] = d.strip()
    return out


def approach_result(run, name):
    for fn in ('approach-%s.txt' % name,):
        p = os.path.join(run, fn)
        if os.path.exists(p):
            b = open(p, 'rb').read()
            t = b.decode('utf-16') if b[:2] == bytes([255, 254]) else b.decode('utf-8', 'replace')
            m = re.findall(r'RESULT approach[^\n]*', t)
            if m:
                mm = re.search(r'(\d+) starts, (\d+) FAIL \(hold / entity\) RELATIVE.*?(\d+) FAIL ABSOLUTE', m[-1])
                return ('rel %s / abs %s of %s' % (mm.group(2), mm.group(3), mm.group(1))) if mm else m[-1].strip()[:40]
    return 'n/a (not in the run)'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sources', required=True)
    ap.add_argument('--base')
    ap.add_argument('--verdict-dir', action='append', default=[])
    ap.add_argument('--decision', default=None, help='pack-decision-<TAG>.json: NEW / INHERITED and the conditional problems per row')
    ap.add_argument('--txt')
    ap.add_argument('--json')
    a = ap.parse_args()
    src = read_sources(a.sources)
    dec = json.load(open(a.decision, encoding='utf-8')) if a.decision else {}
    base = read_sources(a.base) if a.base else {}
    lines, rows = [], {}
    hdr = ('%-30s %-26s %-26s %-7s | %-30s | %-4s | %-22s %-14s | %-10s | %-52s | %-22s | %s' % (
        'row', 'frame-0 root vs game start', 'vs aperture edge (clip/game)', 'slide', 'start / joint rule', 'bend', 'body worst (cm, capsule, gate)', 'roof (cm/gate)', 'close_after',
        'door clearance cm (closed opening open closing closed_after rear)', 'approach FAILs (relative to the baseline / absolute)', ''))
    lines.append(hdr)
    for n in sorted(src):
        run = src[n]
        clip = os.path.join(run, 'roundtrip', n + '.anims.glb')
        g, cls, seat = C.parse_name(clip)
        frag = None
        for vd in list(a.verdict_dir) + [run]:
            p = os.path.join(vd, 'door-close-%s.json' % n)
            if os.path.exists(p):
                frag = json.load(open(p, encoding='utf-8'))
                break
        bt = (frag or {}).get('body_tol_cm')
        rt = (frag or {}).get('roof_tol_cm')
        sg = SG.evaluate(clip, None, None, 'auto', body_tol_cm=bt, roof_tol_cm=rt)
        st = sg['start']
        ph = ((frag or {}).get('at_close_after') or {}).get('phases', {})
        body = ph.get('body')
        roof = ph.get('roof')
        door = ' '.join('%s %.1f%s' % (p[:5], ph[p]['clearance_cm'], '' if ph[p]['ok'] else '!') for p in ('closed', 'opening', 'open', 'closing', 'closed_after', 'rear', 'jamb') if p in ph)
        bstr = 'NO-BODY-VERDICT' if not body else '%.1f %s g%.1f%s' % (body['clearance_cm'], body['capsule'], body['gate_cm'], '' if body['ok'] else '!')
        rstr = 'n/a' if not roof else '%.1f/g%.1f%s' % (roof['clearance_cm'], roof['gate_cm'], '' if roof['ok'] else '!')
        br = os.path.join(run, 'glb', n + '.build-report.json')
        bend = '-'
        if os.path.exists(br):
            hd = ((json.load(open(br)).get('duck') or {}).get('head_duck') or {})
            if hd.get('max_bend_deg') is not None:
                bend = '%.0f' % hd['max_bend_deg']
        old = ''
        if n in base and base[n] != run:
            bclip = os.path.join(base[n], 'roundtrip', n + '.anims.glb')
            try:
                bs = SG.evaluate(bclip, None, None, 'auto')['start']
                old = 'was dy %+.2f slide %.2f' % (bs['dy_vs_vanilla_m'], bs['slide_m'])
            except Exception as e:  # noqa: BLE001
                old = 'was ?(%s)' % e
        rw = sg['rearward']['rows']
        rear = 'REAR %s root %.2f/%.2f hips %.2f/%.2f spine %.2f/%.2f sink %.2f/%.2f%s' % ('ok' if sg['rearward']['ok'] else 'FAIL', rw['root']['excursion_m'], rw['root']['game_m'], rw['Hips']['excursion_m'], rw['Hips']['game_m'], rw['Spine']['excursion_m'], rw['Spine']['game_m'], rw['sink_slide_Hips']['excursion_m'], rw['sink_slide_Hips']['game_m'], ' [documented]' if any(v.get('documented') for v in rw.values()) else '')
        dd = dec.get(n.replace('utpp_car_', ''))
        origin = ('%s clip%s' % ('NEW' if dd['status'] == 'NEW' else 'PREVIOUS', (' | conditional: ' + ','.join(sorted(set(p['type'] for p in dd['conditional_accepted'])))) if dd and dd['conditional_accepted'] else '')) if dd else ''
        ln = '%-30s dx %+.2f dy %+.2f %-13s %s %+.2f / %+.2f   %5.2f   | start %s joints %s%s | %4s | %-22s %-14s | %-10s | %-52s | %-22s | %s' % (
            n.replace('utpp_car_', ''), st['dx_vs_vanilla_m'], st['dy_vs_vanilla_m'], '(root0 %+.2f,%+.2f)' % tuple(st['root0_veh'][:2]), '', st['root0_y_vs_aperture_edge_m'],
            st['vanilla_root0_y_vs_aperture_edge_m'], st['slide_m'], 'ok' if st['ok'] else 'FAIL', 'ok' if sg['joint_rule']['ok'] else 'FAIL',
            (' [doc: %s]' % st['documented_limit'][:40]) if st['documented_limit'] else '', bend, bstr, rstr, ('%.3f' % frag['close_after_s']) if frag and frag.get('close_after_s') else 'n/a',
            door, approach_result(run, n), old) + ' | ' + rear + ' | ' + origin
        lines.append(ln)
        rows[n] = dict(start=st, joint_rule_ok=sg['joint_rule']['ok'], door_phases={p: ph[p] for p in ph if p != 'by_capsule'} if ph else None, close_after_s=(frag or {}).get('close_after_s'),
                       approach=approach_result(run, n), bend_deg=bend, run=run, waivers=sg['waivers'])
        print(ln, flush=True)
    if a.txt:
        open(a.txt, 'w', encoding='utf-8').write(chr(10).join(lines) + chr(10))
    if a.json:
        json.dump(rows, open(a.json, 'w'), indent=1, default=float)


if __name__ == '__main__':
    main()
