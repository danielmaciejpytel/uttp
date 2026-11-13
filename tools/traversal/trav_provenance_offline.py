"""Provenance manifest of a traversal bake (travbake.py --format 2): every input file the bake reads, with SHA-256 (offline, read-only).

python -I trav_provenance_offline.py --rig woman|man --rtz-dir DIR [--clips-dir DIR] [--variant knee] --trav FILE.utpptrav --out OUT.json
  Re-runs travbake's own input resolution (travbake.traversal_montages / the montage metas / rtz-or-plain clip choice) WITHOUT baking,
  hashes the table, the rig json, the 34 montage metas, the clip actually used per montage (rtz or the plain fallback) and the tool
  files, and reads the M lines of FILE.utpptrav (root_z_from_rtz flag, frames) so a fallback to a plain clip is visible.
Imports travbake.py read-only (never edits it). Paths in the output are relative to the project root.
"""
import argparse
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'mmbake'))
import travbake as TB  # noqa: E402

TOOLS = ['Source/tools/mmbake/travbake.py', 'Source/tools/mmbake/FORMAT_TRAVERSAL.txt', 'Source/tools/retarget/retarget_batch.py',
         'Source/tools/retarget/bonemap_source_to_game_woman.json', 'Source/tools/retarget/bonemap_source_to_game_man.json',
         'Animations/retarget/cp_player_woman_skeleton.blend', 'Animations/retarget/cp_player_man_skeleton.blend']


def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


def rel(p):
    return os.path.relpath(os.path.abspath(p), ROOT).replace('\\', '/')


def finfo(p):
    st = os.stat(p)
    return dict(path=rel(p), sha256=sha(p), bytes=st.st_size, mtime=st.st_mtime)


def m_lines(trav):
    out = {}
    with open(trav, 'r', encoding='ascii') as f:
        for ln in f:
            if ln.startswith('M '):
                t = ln.split()
                # M montage sequence length rate bi_s bi_opt bo_s bo_opt auto trigger action fps frames rtz
                out[t[1]] = dict(sequence=t[2], fps=float(t[12]), frames=int(t[13]), root_z_from_rtz=int(t[14]), action=t[11])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rig', required=True, choices=sorted(TB.RIGS))
    ap.add_argument('--variant', default='knee', choices=sorted(TB.VARIANT_SUFFIX))
    ap.add_argument('--rtz-dir')
    ap.add_argument('--clips-dir')
    ap.add_argument('--trav', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rig_json, cdir, rdir = TB.RIGS[a.rig]
    cdir, rdir = cdir + TB.VARIANT_SUFFIX[a.variant], rdir + TB.VARIANT_SUFFIX[a.variant]
    cdir, rdir = a.clips_dir or cdir, a.rtz_dir or rdir
    table = os.path.join(ROOT, 'Data/traversal/traversal_table.json')
    mons = TB.traversal_montages(table)
    ml = m_lines(a.trav)
    rows, fallback = [], []
    for m in mons:
        meta = os.path.join(ROOT, 'Animations/sourcepack/als_export/meta', m + '.json')
        seq = TB.load_json(meta)['referenced_sequences'][0]
        rp = os.path.join(ROOT, rdir, seq + '.utppclip.json')
        cp = os.path.join(ROOT, cdir, seq + '.utppclip.json')
        used = rp if os.path.isfile(rp) else cp
        if used == cp:
            fallback.append(m)
        rows.append(dict(montage=m, sequence=seq, meta=finfo(meta), clip=finfo(used), clip_kind='rtz' if used == rp else 'plain-fallback',
                         trav_m_line=ml.get(m)))
    res = dict(rig=a.rig, variant=a.variant, rtz_dir=rel(os.path.join(ROOT, rdir)), clips_dir=rel(os.path.join(ROOT, cdir)),
               trav=finfo(a.trav), table=finfo(table), rig_json=finfo(os.path.join(ROOT, rig_json)),
               tools=[finfo(os.path.join(ROOT, t)) for t in TOOLS if os.path.isfile(os.path.join(ROOT, t))],
               montages=rows, plain_fallback=fallback,
               m_lines_rtz0=[k for k, v in ml.items() if v['root_z_from_rtz'] == 0], m_line_count=len(ml))
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=1)
    print('%s: %d montages, rtz clips %d, plain fallback %d, M lines with rtz 0: %d -> %s' % (
        a.rig, len(rows), len(rows) - len(fallback), len(fallback), len(res['m_lines_rtz0']), a.out))


if __name__ == '__main__':
    main()
