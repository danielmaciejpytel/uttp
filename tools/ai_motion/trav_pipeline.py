"""Final stages of the traversal / jump gap clips: world npz -> FBX (Blender, mannequin_write.py) -> meta + contact curves ->
retarget to the female and male V rigs (retarget_batch.py, same flags as the current twistfix retarget: --knee-fix --keep-all-curves,
plus an --rtz variant) -> checks on the retargeted clips -> out dir with metadata.

python -I trav_pipeline.py select.json fbx|meta|retarget|check|collect
select.json = [{"name": "TMB_...", "recipe": "LH_run", "var": "L_h25_N66", "obstacle": {"h_cm": 25, "y0_cm": .., "y1_cm": ..} (optional)}]
Work dirs under Build/work/trav-mb-20261106: fbx/, meta/, rt/<rig>[_rtz]/, out/{man,woman}[/rtz]/.
"""
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trav_lib as T  # noqa: E402

W = T.WORK
RIGS = {
    'woman': dict(blend='cp_player_woman_skeleton.blend', rig=os.path.join('Extracted', 'game', 'json', 'vanilla', 'player_woman_skeleton.rig.json'),
                  bonemap='bonemap_source_to_game_woman.json'),
    'man': dict(blend='cp_player_man_skeleton.blend', rig=os.path.join('Extracted', 'game', 'json', 'vanilla-man', 'player_man_skeleton.rig.json'),
                bonemap='bonemap_source_to_game_man.json'),
}


def sh(cmd, log=None):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if log:
        open(log, 'w', encoding='utf-8').write(r.stdout + '\n' + r.stderr)
    return r


def stage_fbx(sel):
    os.makedirs(os.path.join(W, 'fbx'), exist_ok=True)
    tmpl = os.path.join(T.SOURCEPACK_FBX, 'M_Neutral_Run_Loop_F.fbx')

    def one(c):
        wn = os.path.join(W, 'asm', c['recipe'], c['var'] + '.world.npz')
        out = os.path.join(W, 'fbx', c['name'] + '.fbx')
        r = sh([T.BLENDER, '-b', '--factory-startup', '-P', os.path.join(T.HERE, 'mannequin_write.py'), '--', tmpl, wn, out], os.path.join(W, 'fbx', c['name'] + '.log'))
        return c['name'], 'WROTE' in r.stdout

    with ThreadPoolExecutor(8) as ex:
        for n, ok in ex.map(one, sel):
            print('FBX', n, 'ok' if ok else 'FAILED')


def contact_curves(c, sk):
    """contact_l / contact_r per frame of the final clip: source pack meta curves on unedited source frames, rule elsewhere."""
    from skeleton_bridge import load_bridge
    asm = os.path.join(W, 'asm', c['recipe'])
    s = np.load(os.path.join(asm, c['var'] + '.soma.npz'))
    sel = json.load(open(os.path.join(asm, c['var'] + '.sel.json')))
    t0, t1 = sel['trim']
    P = T.fk_all(sk, s['G'], s['hips'])
    ground = P[:, :, 1].min(0)
    out = {}
    for side, key in (('Left', 'contact_l'), ('Right', 'contact_r')):
        mask, _ = T.contacts(P, sk, side, ground, 0.05, 1.0)
        v = mask.astype(float)
        # unedited source pieces: source pack meta contact curves (frames a..b of the source clip)
        for tag, s0, s1, src, a, b in sel['spans']:
            if src.startswith('blend'):
                continue
            cv = T.meta_curve(src, key)
            if cv:
                for i in range(s0, s1 + 1):
                    fi = a + (i - s0)
                    if fi < len(cv):
                        v[i] = float(cv[fi])
        out[key] = [round(float(x), 4) for x in v[t0:t1 + 1]] if key else None
    # rule curves smoothed only where they came from the rule (cheap: smooth everything, source pack curves are already smooth)
    return out


def stage_meta(sel):
    sk = T.Skel(T.TP)
    os.makedirs(os.path.join(W, 'meta'), exist_ok=True)
    for c in sel:
        cur = contact_curves(c, sk)
        n = len(cur['contact_l'])
        meta = {'name': c['name'], 'num_frames': n, 'length_s': (n - 1) / 30.0, 'enable_root_motion': True, 'curves': cur, 'notifies': [],
                'note': 'MotionBricks traversal gap clip (trav_pipe.py): GASP-ALS pose seeds + generated gap; contact curves: GASP meta on source frames, height/speed rule elsewhere'}
        json.dump(meta, open(os.path.join(W, 'meta', c['name'] + '.json'), 'w'), indent=1)
        print('META', c['name'], n)


def stage_retarget(sel):
    names = ','.join(c['name'] for c in sel)
    jobs = []
    for rig, cfg in RIGS.items():
        for rtz in (False, True):
            out = os.path.join(W, 'rt', rig + ('_rtz' if rtz else ''))
            os.makedirs(out, exist_ok=True)
            cmd = [T.BLENDER, '-b', '--factory-startup', '-P', os.path.join(T.ROOT, 'Source', 'tools', 'retarget', 'retarget_batch.py'), '--',
                   '--root', T.ROOT, '--fbx-dir', os.path.join(W, 'fbx'), '--meta-dir', os.path.join(W, 'meta'), '--out-dir', out, '--no-preview',
                   '--keep-all-curves', '--knee-fix', '--clips', names,
                   '--cp-blend', os.path.join(T.ROOT, 'Animations', 'retarget', cfg['blend']), '--rig-json', os.path.join(T.ROOT, cfg['rig']),
                   '--bonemap', os.path.join(T.ROOT, 'Source', 'tools', 'retarget', cfg['bonemap'])]
            if rtz:
                cmd.append('--rtz')
            jobs.append((rig, rtz, cmd, os.path.join(W, 'rt', 'run_%s%s.log' % (rig, '_rtz' if rtz else ''))))

    def one(j):
        r = sh(j[2], j[3])
        return j[0], j[1], r.returncode, sum(1 for l in r.stdout.splitlines() if l.startswith('UTPP_RETARGET') and ' ok ' in l)

    with ThreadPoolExecutor(4) as ex:
        for rig, rtz, rc, nok in ex.map(one, jobs):
            print('RETARGET', rig, 'rtz' if rtz else 'plain', 'rc', rc, 'ok clips', nok)


if __name__ == '__main__':
    sel = json.load(open(sys.argv[1]))
    {'fbx': stage_fbx, 'meta': stage_meta, 'retarget': stage_retarget}[sys.argv[2]](sel)
