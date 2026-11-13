"""Motorcycle ENTRY clips (get-on): MotionBricks candidates for the approach span, screened offline (no game). Companion of bike_steps_gen.py (exits); reuses its glide / clearance / dynamics rules.

  python -I bike_entry_steps.py sweep <out dir> <jobs> <name=dump.npz> ... [--cs 3,4,5,6] [--ns 6,7] [--seeds 10,11,12,13] [--targets 4] [--extra "args of exit_steps_gen.py assemble"]
        bike_entry_run.sh for every clip x c x N x seed; candidates land in <out dir>/<name>/bike_c<c>_N<N>_t<t>_s<seed>.steps.npz
  python -I bike_entry_steps.py pick <run dir> <shipped enter glb> <clip enter_l|enter_r> <out glb> [--hips-w 0.15] [--max-hips-cm 8]
        every <run dir>/bike_*.steps.npz is spliced into the shipped clip (build_entry_steps.py: legs / hips / spine of frames c..g, helpers re-anchored, Root and everything outside c..g untouched),
        dumped and scored: glide cm (bike_steps_gen.slide_list rule, floors of the SHIPPED clip) + hips-w * max hips deviation (cm) + 0.5 * max(0, ground stance - shipped - 5)
        + 1000 when a gate fails: bike clearance proxy in the SEAT frame (the Root of the last frame; inward / downward / hand deviation against the shipped path next to the bike), joint
        acceleration / leg step (bike_steps_gen.dynamics), both feet airborne longer than in the shipped clip (a hop), toe below the shipped ground, hips deviation above --max-hips-cm.
        Table to stdout, <run dir>/pick.json, winner copied to <out glb>.
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)
import bike_steps_gen as BSG  # noqa: E402

PYK = sys.executable
BUILD_STEPS = os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry', 'build_entry_steps.py')
BASH = os.environ.get('UTPP_BASH') or os.path.join(os.environ.get('ProgramFiles', ''), 'Git', 'bin', 'bash.exe')


def to_seat(W):
    """express the dump in the seat frame = the Root of the LAST frame (the clip ends with the Root at the seat origin)"""
    M = np.linalg.inv(W[-1, 0])
    return np.einsum('ij,tnjk->tnik', M, W)


def heights(W, names, floors):
    """per side height above the clip's own floor of the lower of ankle / toe (the contact rule's pivot height), shape (T,)"""
    out = {}
    for s in ('Left', 'Right'):
        ja, jt = names.index(s + 'Foot'), names.index(s + 'ToeBase')
        fa, ft = floors[s]
        out[s] = np.minimum(W[:, ja, 2, 3] - fa, W[:, jt, 2, 3] - ft)
    return out


def airborne_frames(W, names, floors=None, h=0.06, win=8):
    """frames where BOTH feet are more than h above their LOCAL ground (lowest height of the lower of ankle / toe inside +-win frames: the start ground of the shipped clip lies ~9 cm above the bike-side
    ground, a global floor would call the first frames airborne) = a hop; the over-saddle foot is up, so the support foot must stay below h"""
    hh = {}
    for s in ('Left', 'Right'):
        ja, jt = names.index(s + 'Foot'), names.index(s + 'ToeBase')
        za, zt = W[:, ja, 2, 3], W[:, jt, 2, 3]
        lf = lambda z: np.array([z[max(0, f - win):f + win + 1].min() for f in range(len(z))])
        hh[s] = np.minimum(za - lf(za), zt - lf(zt))
    return int(((hh['Left'] > h) & (hh['Right'] > h)).sum())


def cmd_sweep(a):
    from concurrent.futures import ThreadPoolExecutor
    cs, ns, seeds, tg, extra = [3, 4, 5, 6], [6, 7], [10, 11, 12, 13], 4, ''
    for k in ('--cs', '--ns', '--seeds', '--targets', '--extra'):
        if k in a:
            i = a.index(k)
            v = a[i + 1]
            a = a[:i] + a[i + 2:]
            if k == '--cs':
                cs = [int(x) for x in v.split(',')]
            elif k == '--ns':
                ns = [int(x) for x in v.split(',')]
            elif k == '--seeds':
                seeds = [int(x) for x in v.split(',')]
            elif k == '--targets':
                tg = int(v)
            else:
                extra = v
    out, jobs, specs = a[0], int(a[1]), a[2:]
    todo = []
    for spec in specs:
        name, dump = spec.split('=', 1)
        d = os.path.join(out, name)
        for c in cs:
            for n in ns:
                for sd in seeds:
                    tag = 'c%d_N%d_t%d_s%d' % (c, n, tg, sd)
                    if os.path.exists(os.path.join(d, 'bike_%s.steps.npz' % tag)):
                        continue
                    todo.append([BASH, os.path.join(HERE, 'bike_entry_run.sh').replace('\\', '/'), dump.replace('\\', '/'), os.path.relpath(d, ROOT).replace(os.sep, '/'), str(c), str(n), str(tg), str(sd)] + extra.split())
    print('SWEEP %d jobs' % len(todo))

    def run(cmd):
        r = subprocess.run(cmd, capture_output=True, text=True)
        return (r.stdout.strip().splitlines() or ['no output ' + r.stderr[-200:]])[-1]
    with ThreadPoolExecutor(jobs) as ex:
        for line in ex.map(run, todo):
            print(line[:170])


def cmd_pick(a):
    hw, max_hips = 0.15, 8.0
    for k in ('--hips-w', '--max-hips-cm'):
        if k in a:
            i = a.index(k)
            v = float(a[i + 1])
            a = a[:i] + a[i + 2:]
            if k == '--hips-w':
                hw = v
            else:
                max_hips = v
    run, glb, clip, outp = a[:4]
    import game_skeleton_bridge as _CB
    par_ = _CB.load_rig()[1]
    W0, names = BSG.load('%s:%s' % (glb, clip))
    fl0 = BSG.floors_of(W0, names)
    base = BSG.glide(W0, names, None, fl0)
    base_gs = BSG.ground_stance(W0, names, W0)
    base_air = airborne_frames(W0, names, fl0)
    S0 = to_seat(W0)
    floor_toe0 = min(fl0['Left'][1], fl0['Right'][1])
    print('SHIPPED glide %.1f cm (L %.1f R %.1f) ground stance %.0f airborne2 %d' % (base['glide_total_cm'], base['Left']['glide_cm'], base['Right']['glide_cm'], base_gs, base_air))
    rows = []
    cands = sorted(f for f in os.listdir(run) if f.startswith('bike_') and f.endswith('.steps.npz'))
    os.makedirs(os.path.join(run, 'glb'), exist_ok=True)
    for f in cands:
        tag = f[:-len('.steps.npz')]
        out = os.path.join(run, 'glb', tag + '.anims.glb')
        if not (os.path.exists(out) and os.path.getmtime(out) >= os.path.getmtime(os.path.join(run, f))):
            r = subprocess.run([PYK, '-I', BUILD_STEPS, '--glb', glb, '--clip', clip, '--name', clip, '--steps', os.path.join(run, f), '--out', out,
                                '--report', os.path.join(run, 'glb', tag + '.json')], capture_output=True, text=True)
            if r.returncode:
                print('BUILD FAILED', tag, (r.stderr or r.stdout)[-300:])
                continue
        W, _ = BSG.load('%s:%s' % (out, clip))
        g = BSG.glide(W, names, None, fl0)
        hd = BSG.hips_dev(W0, W, names)
        cl = BSG.clearance(S0, to_seat(W), names)
        dyn = BSG.dynamics(W0, W, names, par_)
        gs = BSG.ground_stance(W, names, W0)
        air = airborne_frames(W, names, fl0)
        low = min(float(W[:, names.index('LeftToeBase'), 2, 3].min()), float(W[:, names.index('RightToeBase'), 2, 3].min()))
        below = max(0.0, (floor_toe0 - low) * 100)
        okg = bool(cl['ok'] and dyn['ok'] and air <= base_air and below <= 3.0 and hd <= max_hips)
        score = g['glide_total_cm'] + hw * hd + 0.5 * max(0.0, gs - base_gs - 5.0) + (0.0 if okg else 1000.0)
        rows.append(dict(tag=tag, glb=out, glide=g, hips_dev_cm=round(hd, 1), clearance=cl, dynamics=dyn, ground_stance_cm=gs, airborne2=air, toe_below_cm=round(below, 1), gates_ok=okg, score=round(score, 2)))
        print('CAND %-22s glide %6.1f (L %.1f R %.1f) gst %.0f hips %.1f clr %.1f/%.1f/%.1f air %d low %.1f %s acc %.1f/%.1f step %.1f/%.1f score %.1f' % (
            tag, g['glide_total_cm'], g['Left']['glide_cm'], g['Right']['glide_cm'], gs, hd, cl['inward_cm'], cl['down_cm'], cl['hand_cm'], air, below, 'ok' if okg else 'FAIL',
            dyn['accel_cm'], dyn['accel_vanilla_cm'], dyn['step_deg'], dyn['step_vanilla_deg'], score))
    rows.sort(key=lambda r: r['score'])
    json.dump(dict(shipped=base, shipped_airborne2=base_air, candidates=rows), open(os.path.join(run, 'pick.json'), 'w'), indent=1)
    if rows:
        import shutil
        shutil.copyfile(rows[0]['glb'], outp)
        print('PICK %s score %.1f -> %s' % (rows[0]['tag'], rows[0]['score'], outp))


def cmd_eval(a):
    """eval <shipped glb> <clip> <candidate glb> [more candidate glbs]: the gate numbers of pick for finished candidate clips (gait pass, no search)"""
    import game_skeleton_bridge as _CB
    par_ = _CB.load_rig()[1]
    glb, clip = a[0], a[1]
    W0, names = BSG.load('%s:%s' % (glb, clip))
    fl0 = BSG.floors_of(W0, names)
    base = BSG.glide(W0, names, None, fl0)
    base_gs = BSG.ground_stance(W0, names, W0)
    base_air = airborne_frames(W0, names, fl0)
    S0 = to_seat(W0)
    floor_toe0 = min(fl0['Left'][1], fl0['Right'][1])
    print('SHIPPED glide %.1f cm (L %.1f R %.1f) ground stance %.0f airborne2 %d' % (base['glide_total_cm'], base['Left']['glide_cm'], base['Right']['glide_cm'], base_gs, base_air))
    for cand in a[2:]:
        W, _ = BSG.load('%s:%s' % (cand, clip))
        g = BSG.glide(W, names, None, fl0)
        hd = BSG.hips_dev(W0, W, names)
        cl = BSG.clearance(S0, to_seat(W), names)
        dyn = BSG.dynamics(W0, W, names, par_)
        gs = BSG.ground_stance(W, names, W0)
        air = airborne_frames(W, names, fl0)
        low = min(float(W[:, names.index('LeftToeBase'), 2, 3].min()), float(W[:, names.index('RightToeBase'), 2, 3].min()))
        below = max(0.0, (floor_toe0 - low) * 100)
        okg = bool(cl['ok'] and dyn['ok'] and air <= base_air and below <= 3.0 and hd <= 0.5)
        print('EVAL %-40s glide %5.1f (L %.1f R %.1f) gst %.0f hips %.1f clr %.1f/%.1f/%.1f air %d low %.1f %s acc %.1f/%.1f step %.1f/%.1f' % (
            os.path.basename(cand)[:40], g['glide_total_cm'], g['Left']['glide_cm'], g['Right']['glide_cm'], gs, hd, cl['inward_cm'], cl['down_cm'], cl['hand_cm'], air, below, 'ok' if okg else 'FAIL',
            dyn['accel_cm'], dyn['accel_vanilla_cm'], dyn['step_deg'], dyn['step_vanilla_deg']))


if __name__ == '__main__':
    {'sweep': cmd_sweep, 'pick': cmd_pick, 'eval': cmd_eval}[sys.argv[1]](sys.argv[2:])
