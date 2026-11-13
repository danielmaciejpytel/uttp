"""Inventory of the source pack traversal + jump clips (offline, read-only): category, obstacle / ledge height, speed, start / end stance,
root motion per V rig, montage presence, use in source pack and in the installed traversal file.

python -I trav_inventory.py --out-dir DIR [--rt-dir DIR_WITH rt_set{2,3}_{woman,man}] [--am-dir DIR_WITH_AM_uassets] [--trav FILE]
  Source: Animations\\sourcepack\\set{2,3}_export\\meta\\<clip>.json (UE root track ends, length, curves, referenced_by).
  Root motion per rig = source root track (UE cm) x hip_ratio (woman 1.092927, man 1.031735; the retarget scales the Trajectory and
  root_z by it).  Entry speed: from the retargeted rtz clips when --rt-dir has them (first 3 frames of the Trajectory), else the
  average planar speed (distance / length) marked 'avg'.
  Writes DIR\\sourcepack_trav_jump_inventory.json and .tsv.
"""
import argparse
import glob
import json
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
HIP = {'woman': 1.092927, 'man': 1.031735}
SETS = {'set2': os.path.join(ROOT, 'Animations', 'sourcepack', 'set2_export', 'meta'),
        'set3': os.path.join(ROOT, 'Animations', 'sourcepack', 'set3_export', 'meta')}
M10 = os.path.join(ROOT, 'Animations', 'sourcepack', 'als_export', 'meta')


def category(n):
    b = re.sub(r'^M_(Neutral|Relaxed)_', '', n)
    if b.startswith('Traversal_'):
        t = b[len('Traversal_'):]
        for k in ('Catch_Hurdle', 'Catch_Mantle', 'Catch_Vault', 'Catch_Cliff', 'Climb_Start', 'Hurdle', 'Mantle', 'Vault'):
            if t.startswith(k):
                return 'traversal', k
        return 'traversal', 'other'
    if b.startswith('Jump_'):
        if 'Loop_Fall' in b:
            return 'jump', 'Loop_Fall'
        for k in ('Start', 'Land', 'Off'):
            if '_%s' % k in b:
                sub = k
                if k == 'Land':
                    sub += '_' + next((x for x in ('Roll', 'Stumble', 'Heavy', 'Light') if x in b), '?')
                if k == 'Start' and ('Across' in b or 'Cliff' in b):
                    sub += '_' + ('Across' if 'Across' in b else 'Cliff')
                return 'jump', sub
    if 'ragdoll' in n:
        return 'ragdoll', 'getup_roll' if 'getup' in n else 'poses'
    return 'other', 'other'


def speed_word(n):
    for w in ('sprint', 'run', 'walk', 'stand'):
        if re.search(r'(^|_)%s(_|$)' % w, n.lower()):
            return w
    if '_Across' in n or '_Cliff' in n:
        return 'run'
    return '-'


def direction(n):
    m = re.search(r'_(F|B|LL|LR|RL)_', n)
    return m.group(1) if m else ('F' if '_F_' in n else '-')


def height_label(n):
    m = re.search(r'_(\d)_(\d)_', n)
    if m:
        return float('%s.%s' % m.groups())
    m = re.search(r'_(low|med|high)', n)
    return m.group(1) if m else None


def stance(z_cm, other_z_cm, end):
    if abs(z_cm) < 1.0:
        return 'ground'
    if end:
        return 'on top %.2f m' % (z_cm / 100) if z_cm > 0 else 'falling %.2f m' % (z_cm / 100)
    return 'air %.2f m' % (z_cm / 100) if z_cm > 0 else 'below %.2f m' % (z_cm / 100)


def rt_entry_speed(rt_dir, rig, setn, n):
    if not rt_dir:
        return None
    p = os.path.join(rt_dir, 'rt_%s_%s' % (setn, rig), n + '.utppclip.json')
    if not os.path.isfile(p):
        return None
    c = json.load(open(p, encoding='utf-8'))
    ti = c['bone_names'].index('Trajectory')
    fr = c['frames']
    k = min(3, len(fr) - 1)
    a, b = fr[0]['bones'][ti], fr[k]['bones'][ti]
    return math.hypot(b[0] - a[0], b[1] - a[1]) * float(c['fps']) / k


def trav_montages(path):
    """sequence -> montage of the M lines of a .utpptrav."""
    out = {}
    if path and os.path.isfile(path):
        with open(path, encoding='ascii') as f:
            for ln in f:
                if ln.startswith('M '):
                    t = ln.split()
                    out[t[2]] = t[1]
    return out


def _rng(c):
    lo, hi, no_lo, no_hi = c
    return [None if no_lo else lo, None if no_hi else hi]


def chooser_rows(table_path):
    """Leaf montage -> constraints of the CHT_TraversalAnims chooser (UE cm, cm/s; MovementMode bitmask over the enum)."""
    d = json.load(open(table_path, encoding='utf-8'))
    mm = d['movement_mode_enum']
    act = {x['value']: x['name'] for x in d['action_enum']}
    tabs = d['tables']
    out = {}

    def walk(ti, inherited):
        t = tabs[ti]
        for ri, row in enumerate(t['rows']):
            cons = dict(inherited)
            for c in t['columns']:
                v = c['cells'][ri]
                if c['kind'] == 'float':
                    cons[c['input']] = _rng(v)
                elif c['kind'] == 'bool':
                    cons[c['input']] = v
                elif c['kind'] == 'multienum':
                    cons[c['input']] = [mm[i] for i in range(len(mm)) if int(v) >> i & 1]
                elif c['kind'] == 'output_enum':
                    cons['Action'] = act.get(v, v)
            r = row['result']
            if r['type'] == 'nested':
                walk(r['table'], cons)
            elif r['type'] == 'asset':
                cons['subtable'] = t['name']
                out.setdefault(r['asset'], []).append(cons)
    walk(d['root'], {})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--rt-dir')
    ap.add_argument('--am-dir', help='folder with the source pack 1.2 AM_*.uasset montage files (names only are used)')
    ap.add_argument('--trav', help='installed / candidate .utpptrav (montage sequences in the file)')
    ap.add_argument('--table', default=os.path.join(ROOT, 'Data', 'traversal', 'traversal_table.json'))
    a = ap.parse_args()
    ams = set(os.path.basename(x)[3:-7] for x in glob.glob(os.path.join(a.am_dir, '**', 'AM_*.uasset'), recursive=True)) if a.am_dir else set()
    in_trav = trav_montages(a.trav)
    chooser = chooser_rows(a.table) if a.table and os.path.isfile(a.table) else {}
    rows = []
    for setn, d in SETS.items():
        for p in sorted(glob.glob(os.path.join(d, 'M_*.json'))):
            n = os.path.basename(p)[:-5]
            cat, sub = category(n)
            if cat not in ('traversal', 'jump'):
                continue
            m = json.load(open(p, encoding='utf-8'))
            s0 = m.get('root_track_start', {}).get('translation', [0, 0, 0])
            s1 = m.get('root_track_end', {}).get('translation', [0, 0, 0])
            L = float(m.get('length_s') or 0)
            planar = math.hypot(s1[0] - s0[0], s1[1] - s0[1])
            curves = sorted(set((m.get('curves') or {}).keys()) | set((m.get('curve_keys') or {}).keys()))
            refs = [str(x).split('/')[-1].split('.')[0] for x in (m.get('referenced_by') or [])]
            am = ('AM_' + n) if ('AM_' + n) in ('AM_' + x for x in ams) else None
            if am is None:
                alt = [x for x in ams if x.startswith(n)]     # AM_M_Neutral_Traversal_Catch_Cliff_high_standF
                am = ('AM_' + alt[0]) if alt else None
            r = dict(set=setn, style=('Relaxed' if '_Relaxed_' in n else 'Neutral'), clip=n, category=cat, sub=sub, direction=direction(n),
                     speed_class=speed_word(n), height_label=height_label(n), frames=m.get('num_frames'), length_s=round(L, 3),
                     root_start_src_cm=[round(x, 1) for x in s0], root_end_src_cm=[round(x, 1) for x in s1],
                     start_stance=stance(s0[2], s1[2], False), end_stance=stance(s1[2], s0[2], True),
                     has_distance_from_ledge='distance_from_ledge' in [c.lower() for c in curves],
                     montage_in_set=am, used_by=[x for x in refs if x.startswith(('AM_', 'CHT_', 'PSD_'))][:6],
                     in_base_export=os.path.isfile(os.path.join(M10, n + '.json')),
                     in_installed_traversal=n in in_trav, installed_montage=in_trav.get(n),
                     chooser_base_export=chooser.get(in_trav.get(n) or '') or None)
            for rig, h in HIP.items():
                es = rt_entry_speed(a.rt_dir, rig, setn, n)
                r[rig] = dict(planar_m=round(planar / 100 * h, 3), dz_m=round((s1[2] - s0[2]) / 100 * h, 3), start_z_m=round(s0[2] / 100 * h, 3),
                              end_z_m=round(s1[2] / 100 * h, 3), avg_speed_ms=round(planar / 100 * h / L, 2) if L > 0 else None,
                              entry_speed_ms=(round(es, 2) if es is not None else None),
                              height_m=(round(r['height_label'] * h, 3) if isinstance(r['height_label'], float) else None))
            rows.append(r)
    os.makedirs(a.out_dir, exist_ok=True)
    jp = os.path.join(a.out_dir, 'sourcepack_trav_jump_inventory.json')
    json.dump(rows, open(jp, 'w', encoding='utf-8'), indent=1)
    cols = ['set', 'style', 'clip', 'category', 'sub', 'direction', 'speed_class', 'height_label', 'frames', 'length_s', 'start_stance', 'end_stance',
            'has_distance_from_ledge', 'montage_in_set', 'in_base_export', 'in_installed_traversal', 'chooser_base_export']
    with open(os.path.join(a.out_dir, 'sourcepack_trav_jump_inventory.tsv'), 'w', encoding='utf-8') as f:
        f.write('\t'.join(cols + ['W_planar_m', 'W_dz_m', 'W_entry_ms', 'W_height_m', 'M_planar_m', 'M_dz_m', 'M_entry_ms', 'M_height_m', 'used_by']) + '\n')
        for r in rows:
            f.write('\t'.join([json.dumps(r[c]) if isinstance(r[c], (list, dict)) else str(r[c]) for c in cols] + [str(r['woman']['planar_m']), str(r['woman']['dz_m']),
                                                           str(r['woman']['entry_speed_ms'] if r['woman']['entry_speed_ms'] is not None else 'avg %s' % r['woman']['avg_speed_ms']),
                                                           str(r['woman']['height_m']), str(r['man']['planar_m']), str(r['man']['dz_m']),
                                                           str(r['man']['entry_speed_ms'] if r['man']['entry_speed_ms'] is not None else 'avg %s' % r['man']['avg_speed_ms']),
                                                           str(r['man']['height_m']), ','.join(r['used_by'])]) + '\n')
    by = {}
    for r in rows:
        by.setdefault((r['set'], r['style'], r['category'], r['sub']), []).append(r)
    print('rows', len(rows))
    for k in sorted(by):
        print('%-5s %-8s %-10s %-14s %3d  montage %d  in1.0 %d  installed %d' % (k[0], k[1], k[2], k[3], len(by[k]), sum(1 for r in by[k] if r['montage_in_set']),
                                                                              sum(1 for r in by[k] if r['in_base_export']), sum(1 for r in by[k] if r['in_installed_traversal'])))


if __name__ == '__main__':
    main()
