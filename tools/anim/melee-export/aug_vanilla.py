"""Augmented vanilla inputs for UTPP-only clip names (2026-11-10; permanent version of the earlier knife / blunt / katana aug_vanilla.py scripts).

  python -I aug_vanilla.py --vanilla <frozen vanilla dir> --input <clips dir> --suffix <s> --sets a,b --out <dir>

The export pipeline (build_melee_set / animset_subset / WolvenKit import / patch_melee_json / verify_melee_set) works per clip name of the vanilla
set.  A clip whose name the vanilla set does not have (the graph patch melee-parried-graph-20261110 plays parried_<left|right>_<attack>) would be
skipped.  For every selected set this script looks at <input>\\<set>__<clip>.<suffix>.npz: a clip name that is not in the vanilla set and matches
melee_export_lib.UTPP_CLIP_TEMPLATES (allowlist) gets a CLONE of its template clip (parried_left_combo_01 <- parried_left): same buffer, events,
duration, motion extraction, with fresh HandleId / BufferId values, appended to the set json and to both GLBs (glb, glb-nort).  The clone only
gives the clip its identity in the set; the anim data comes from the input clip, the rest is patched back to the template's values by
patch_melee_json.py and checked by verify_melee_set.py.  Written to <out>\\json, glb, glb-nort, anims (the selected sets only; sets without clones are
copied unchanged); the vanilla dir is never modified.  run_melee_export.ps1 runs this step by itself and uses <Out>\\vanilla-aug when it reports clones.

Names that are neither in the vanilla set nor allowlisted are listed (AUG_IGNORED) and stay unbuilt, as before.  An allowlisted name whose template
clip is missing in the set is an error (exit 2): the set cannot take that clip.
Exit codes: 0 clones written (or none needed; AUG_CLONES n tells), 2 template missing.
"""
import argparse
import copy
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_export_lib as L  # noqa: E402
import glbanim as G  # noqa: E402


def max_id(doc, key):
    m, st = 0, [doc]
    while st:
        v = st.pop()
        if isinstance(v, dict):
            if key in v:
                try:
                    m = max(m, int(v[key]))
                except ValueError:
                    pass
            st.extend(v.values())
        elif isinstance(v, list):
            st.extend(v)
    return m


def renumber(value, hc, bc):
    """deep copy with fresh HandleId / BufferId (internal HandleRefIds remapped; a reference to an outside handle is an error)."""
    v = copy.deepcopy(value)
    mp, refs, st = {}, [], [v]
    while st:
        x = st.pop()
        if isinstance(x, dict):
            if 'HandleId' in x:
                hc[0] += 1
                mp[x['HandleId']] = str(hc[0])
                x['HandleId'] = str(hc[0])
            if 'BufferId' in x:
                bc[0] += 1
                x['BufferId'] = str(bc[0])
            if 'HandleRefId' in x and x['HandleRefId'] != '-1':
                refs.append(x)
            st.extend(x.values())
        elif isinstance(x, list):
            st.extend(x)
    for r in refs:
        if r['HandleRefId'] not in mp:
            raise SystemExit('outside handle %s' % r['HandleRefId'])
        r['HandleRefId'] = mp[r['HandleRefId']]
    return v


def input_clips(indir, s, suffix):
    pre, post = '%s__' % s, '.%s.npz' % suffix
    return sorted(f[len(pre):-len(post)] for f in os.listdir(indir) if f.startswith(pre) and f.endswith(post))


def plan(vnames, clips):
    """-> (new [(name, template)] per suffix, left then right, ignored [name], missing [(name, template)])"""
    new, ignored, missing = [], [], []
    for c in clips:
        if c in vnames:
            continue
        t = L.template_clip(c)
        if t is None:
            ignored.append(c)
        elif t not in vnames:
            missing.append((c, t))
        else:
            new.append((c, t))
    new.sort(key=lambda nt: (nt[0].split("_", 2)[2], nt[0].split("_", 2)[1]))   # per attack suffix, left before right (as the earlier builds listed them)
    return new, ignored, missing


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--vanilla', required=True)
    ap.add_argument('--input', required=True)
    ap.add_argument('--suffix', default='v4')
    ap.add_argument('--sets', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    sets = [x for x in a.sets.split(',') if x]
    for sub in ('json', 'glb', 'glb-nort', 'anims'):
        os.makedirs(os.path.join(a.out, sub), exist_ok=True)
    total, bad = 0, []
    for s in sets:
        jp = os.path.join(a.vanilla, 'json', s + '.anims.json')
        doc = json.load(open(jp, encoding='utf-8'))
        rc = doc['Data']['RootChunk']
        by = {e['Data']['animation']['Data']['name']['$value']: e for e in rc['animations']}
        new, ignored, missing = plan(set(by), input_clips(a.input, s, a.suffix))
        bad += ['%s/%s (template %s missing)' % (s, c, t) for c, t in missing]
        if ignored:
            print('AUG_IGNORED %s %s' % (s, ignored))
        for sub, ext in (('anims', '.anims'),):
            p = os.path.join(a.vanilla, sub, s + ext)
            if os.path.exists(p):
                shutil.copy2(p, os.path.join(a.out, sub, s + ext))
        if not new:
            shutil.copy2(jp, os.path.join(a.out, 'json', s + '.anims.json'))
            for sub in ('glb', 'glb-nort'):
                shutil.copy2(os.path.join(a.vanilla, sub, s + '.anims.glb'), os.path.join(a.out, sub, s + '.anims.glb'))
            print('AUG %s none' % s)
            continue
        hc, bc = [max_id(doc, 'HandleId') + 10], [max_id(doc, 'BufferId') + 10]
        for n, t in new:
            e = renumber(by[t], hc, bc)
            e['Data']['animation']['Data']['name']['$value'] = n
            rc['animations'].append(e)
        json.dump(doc, open(os.path.join(a.out, 'json', s + '.anims.json'), 'w', encoding='utf-8'), indent=1)
        for sub in ('glb', 'glb-nort'):
            gj, binb = G.read_glb(os.path.join(a.vanilla, sub, s + '.anims.glb'))
            names = {an.get('name'): i for i, an in enumerate(gj['animations'])}
            for n, t in new:
                c = copy.deepcopy(gj['animations'][names[t]])
                c['name'] = n
                gj['animations'].append(c)
            G.write_glb(os.path.join(a.out, sub, s + '.anims.glb'), gj, binb)
        total += len(new)
        print('AUG %s %d clones: %s' % (s, len(new), [n for n, _ in new]))
    if bad:
        print('AUG_FAIL %s' % bad)
        return 2
    print('AUG_CLONES %d' % total)
    return 0


main_args = main


if __name__ == '__main__':
    sys.exit(main())
