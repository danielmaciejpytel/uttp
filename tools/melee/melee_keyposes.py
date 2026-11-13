"""Generate a full clip from a sequence of corrected key poses with MotionBricks (SOMA77), game <-> SOMA through game_skeleton_bridge.

  python -I melee_keyposes.py <corrected\<set>\<clip>.keys.json> [--out PREFIX] [--no-mb] [--seed N | --seeds 7,11,3] [--no-gate] [--free-arms] [--leg-ik]
Input: the output of apply_keyposes.py (<clip>.timed.npz = corrected clip on the OUTPUT timeline + <clip>.keys.json with the planned
segments). Steps (timing of each is printed):
  1 fwd     game_skeleton_bridge fwd  timed.npz -> full soma scene (G, hips) of the corrected, re-timed clip
  2 jobs    one MotionBricks gap per 'mb' segment: context = scene frames c-3..c (the first key's window, front-padded when c<3),
            target = frames g..g+3 of the next key (end-padded), gap = 4N-7 frames (N = 6..16, i.e. 17, 21, ... 57). Because the
            output spacing D of two keys is usually not 4N-7, each scene gets its own context/target at 4N-7 and the generated span
            (4N-6 frames) is afterwards RESAMPLED to D+1 frames (slerp of the global rotations, linear hips) so key k stays at its time_frame
  3 mb      segment_generator.py through `uv run` in Tools\\ai-motion\\motionbricks (ONE engine start for all segments)
  4 splice  generated spans replace the scene frames between the keys (key frames themselves stay exact); 'offset' segments keep the
            corrected + re-timed clip (offset layer of apply_keyposes)
  5 inv     game_skeleton_bridge inv with timed.npz as the template (fingers, weapon slots, helpers, bones SOMA does not drive) -> PREFIX.mb.utppclip.json
            + PREFIX.mb.dump.npz (W [T,73,4,4] game axes, dump layout without the 152-joint locals, usable by melee_sheet.py --source)
Outputs go to <out dir of keys.json>\\<clip>.mb.* unless --out is given. --no-mb: skips step 3 (all segments stay 'offset'), proves the
rest of the chain without the GPU. --leg-ik: game_skeleton_bridge inv --leg-ik (leg IK for clips with moving legs; it moved the planted feet of the static FPP legs by 3 cm, so it is OFF by default; without it frames outside the generated spans reproduce the template exactly).
Segments shorter than 17 or longer than 57 frames are never sent to the model.
"""
import json
import os
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402

AIM = os.path.join(MC.ROOT, 'Source', 'tools', 'ai_motion')
TPOSE = os.path.join(AIM, 'data', 'body_tpose_generator.npz')


def bash_path(p):
    p = os.path.abspath(p).replace('\\', '/')
    return '/' + p[0].lower() + p[2:] if len(p) > 1 and p[1] == ':' else p


def run_cp(args, rig):
    env = dict(os.environ, UTPP_RIG=rig)
    p = subprocess.run([MC.PY, '-I', os.path.join(AIM, 'game_skeleton_bridge.py')] + args, capture_output=True, text=True, env=env)
    if p.returncode:
        raise RuntimeError('game_skeleton_bridge %s failed:\n%s' % (args[0], (p.stdout + p.stderr)[-800:]))
    return p.stdout.strip().splitlines()[-1]


def run_mb(jobs_path):
    """uv run segment_generator.py in the motionbricks project, environment as Tools\\ai-motion\\env.sh."""
    sh = ('source "%s"; cd "%s" && uv run python "%s" "%s"' % (
        bash_path(os.path.join(MC.ROOT, 'Tools', 'ai-motion', 'env.sh')),
        bash_path(os.path.join(MC.ROOT, 'Tools', 'ai-motion', 'motionbricks')),
        bash_path(os.path.join(AIM, 'segment_generator.py')), bash_path(jobs_path)))
    bash = None
    for c in (os.path.join(os.environ.get('ProgramFiles', ''), 'Git', 'bin', 'bash.exe'), os.path.join(os.environ.get('ProgramFiles', ''), 'Git', 'usr', 'bin', 'bash.exe')):
        if os.path.exists(c):
            bash = c
            break
    p = subprocess.run([bash or 'bash', '-c', sh], capture_output=True, text=True)
    out = p.stdout + p.stderr
    if p.returncode or 'TRAV_MB done' not in out:
        raise RuntimeError('MotionBricks run failed:\n' + out[-1500:])
    return [l for l in out.splitlines() if 'TRAV_MB' in l][-1]


def pad_take(a, lo, hi):
    """a[lo:hi+1] with indices clamped into range (edge padding)."""
    ix = np.clip(np.arange(lo, hi + 1), 0, len(a) - 1)
    return a[ix]


def resample_span(G, hips, n_out):
    """Resample a generated span (frames [n,...]) to n_out frames: slerp of global rotations per joint, linear hips."""
    n = len(G)
    if n == n_out:
        return G, hips
    t_in, t_out = np.linspace(0, 1, n), np.linspace(0, 1, n_out)
    Go = np.empty((n_out,) + G.shape[1:])
    for j in range(G.shape[1]):
        Go[:, j] = Slerp(t_in, R.from_matrix(G[:, j]))(t_out).as_matrix()
    ho = np.stack([np.interp(t_out, t_in, hips[:, c]) for c in range(3)], 1)
    return Go, ho


def lock_arms(prefix, timed_path, rig, spans):
    """Inside the generated spans the whole shoulder girdle + arm + hand subtree (clavicle, arm, forearm, hand, fingers, weapon slots) is taken
    from the corrected + re-timed source (offset layer) and attached to the generated chest (Spine3): MotionBricks re-invents arm arcs and
    arm roll (arms flung overhead, twisted wrists, the SOMA round trip adds roll), the vanilla arm motion is what is known to be natural.
    MotionBricks keeps hips / legs / spine / neck. Rewrites <prefix>.mb.dump.npz + .mb.utppclip.json (the raw MB output is kept as .mbraw.dump.npz)."""
    z = dict(np.load(prefix + '.mb.dump.npz', allow_pickle=True))
    T = np.load(timed_path, allow_pickle=True)
    names = [str(n) for n in z['names']]
    par = [int(x) for x in z['parents']]
    ix = {n: i for i, n in enumerate(names)}
    sub = []
    for j in range(len(names)):
        k = j
        while k >= 0 and names[k] not in ('LeftShoulder', 'RightShoulder'):
            k = par[k]
        if k >= 0:
            sub.append(j)
    W, Wt = z['W'].copy(), T['W']
    s3 = ix['Spine3']
    np.savez_compressed(prefix + '.mbraw.dump.npz', **z)
    for c0, g0 in spans:
        for t in range(c0, g0 + 1):
            M = W[t, s3] @ np.linalg.inv(Wt[t, s3])
            W[t, sub] = M @ Wt[t, sub]
    z['W'] = W
    np.savez_compressed(prefix + '.mb.dump.npz', **z)
    os.environ['UTPP_RIG'] = rig
    sys.path.insert(0, AIM)
    import game_skeleton_bridge as CP
    nm, pr, ref = CP.load_rig()
    clip = CP.to_utppclip(W, nm, pr, ref, z['rest'], os.path.basename(prefix) + '_mb')
    json.dump(clip, open(prefix + '.mb.utppclip.json', 'w'), separators=(',', ':'))
    return len(sub)


def run_one(a, seed, no_mb=False):
    kj = json.load(open(a[0], encoding='utf-8'))
    out_dir = os.path.dirname(os.path.abspath(a[0]))
    prefix = a[a.index('--out') + 1] if '--out' in a else os.path.join(out_dir, kj['clip'])
    timed = kj['timed']
    rig = MC.RIG_OF_SET[kj['set']]
    tm = {}
    t0 = time.time()
    scene = prefix + '.scene.soma.npz'
    run_cp(['fwd', timed, TPOSE, scene], rig)
    S = np.load(scene)
    G, hips = S['G'].astype(np.float64), S['hips'].astype(np.float64)
    tm['fwd'] = round(time.time() - t0, 1)
    keys = {k['id']: k for k in kj['keys']}
    segs = [s for s in kj['segments'] if s['mode'] == 'mb' and not no_mb and '--no-mb' not in a]
    jobs, seg_scene = [], []
    t1 = time.time()
    for i, s in enumerate(segs):
        ka, kb = keys[s['from']], keys[s['to']]
        c0, g0, N = ka['time_frame'], kb['time_frame'], s['N']
        gap = 4 * N - 7
        ctx = pad_take(G, c0 - 3, c0)
        tgt = pad_take(G, g0, g0 + 3)
        cth, tth = pad_take(hips, c0 - 3, c0), pad_take(hips, g0, g0 + 3)
        filler = np.repeat(ctx[-1:], gap - 1, 0)
        fh = np.repeat(cth[-1:], gap - 1, 0)
        sc_path = '%s.seg%d.scene.soma.npz' % (prefix, i)
        np.savez(sc_path, G=np.concatenate([ctx, filler, tgt]), hips=np.concatenate([cth, fh, tth]), names=S['names'], k=S['k'], fps=30)
        jobs.append(dict(soma=sc_path, out='%s.seg%d' % (prefix, i), gaps=[dict(c=3, N=N, targets=4)], seeds=[seed], jitter_cm=0.0))
        seg_scene.append(sc_path)
    tm['jobs'] = round(time.time() - t1, 1)
    Gf, hf = G.copy(), hips.copy()
    if jobs:
        jp = prefix + '.jobs.json'
        json.dump(jobs, open(jp, 'w'))
        t2 = time.time()
        msg = run_mb(jp)
        tm['mb'] = round(time.time() - t2, 1)
        t3 = time.time()
        for i, s in enumerate(segs):
            ka, kb = keys[s['from']], keys[s['to']]
            c0, g0 = ka['time_frame'], kb['time_frame']
            gen = np.load('%s.seg%d.g0.s%d.gen.npz' % (prefix, i, seed))
            Gg, hg = resample_span(gen['G'].astype(np.float64), gen['hips'].astype(np.float64), g0 - c0 + 1)
            Gg[0], hg[0] = G[c0], hips[c0]       # key frames exact
            Gg[-1], hg[-1] = G[g0], hips[g0]
            Gf[c0:g0 + 1], hf[c0:g0 + 1] = Gg, hg
        tm['splice'] = round(time.time() - t3, 1)
    t4 = time.time()
    full = prefix + '.mb.soma.npz'
    np.savez(full, G=Gf, hips=hf, names=S['names'], k=S['k'], fps=30)
    n = len(Gf)
    run_cp(['inv', full, timed, TPOSE, prefix + '.mb.utppclip.json', '--frames', '0:%d' % (n - 1)] + (['--leg-ik'] if '--leg-ik' in a else []) + ['--name', kj['clip'] + '_mb',
            '--dump', prefix + '.mb.dump.npz'], rig)
    tm['inv'] = round(time.time() - t4, 1)
    if segs and '--free-arms' not in a:
        lock_arms(prefix, timed if os.path.isabs(timed) else os.path.join(MC.ROOT, timed), rig,
                  [(keys[x['from']]['time_frame'], keys[x['to']]['time_frame']) for x in segs])
    tm['total'] = round(time.time() - t0, 1)
    info = dict(set=kj['set'], clip=kj['clip'], frames=n, mb_segments=[dict(s, gen_frames=4 * s['N'] - 6) for s in segs],
                timing_s=tm, utppclip=prefix + '.mb.utppclip.json', dump=prefix + '.mb.dump.npz')
    MC.jdump(info, prefix + '.mb.info.json')
    print('MB_KEYPOSES %s/%s seed %s frames %d mb segments %d timing_s %s -> %s.mb.*' % (kj['set'], kj['clip'], seed, n, len(segs), tm, prefix))
    return kj, prefix


def main(a):
    """Anatomy gate on the result (anatomy_gate.py, absolute features vs the retimed vanilla; the delta budgets do not apply inside generated
    spans): seeds are tried in order (--seeds 7,11,3,..., default --seed N or 7); the first output that passes is kept. When none passes
    the 'offset' version (corrected + re-timed source, no generated span) replaces it, and that must pass too (it did before: apply_keyposes gates it).
    --no-gate keeps the first output unchecked."""
    import anatomy_gate as AG
    seeds = [int(x) for x in a[a.index('--seeds') + 1].split(',')] if '--seeds' in a else [int(a[a.index('--seed') + 1]) if '--seed' in a else 7]
    gate_on = '--no-gate' not in a
    last = None
    for seed in seeds:
        kj, prefix = run_one(a, seed)
        if not gate_on:
            return
        res = AG.gate_files(prefix + '.mb.dump.npz', kj['set'], kj['clip'], a[0], deltas=True)
        print('  anatomy gate seed %d: %s (%d violations, margin %.1f) %s' % (seed, 'PASS' if res['passed'] else 'FAIL', res['n_violations'], res['margin_min'],
                                                                         '' if res['passed'] else [(e['feature'], e['frame'], e['value']) for e in res['violations'][:3]]))
        info = json.load(open(prefix + '.mb.info.json'))
        info['gate'] = dict(passed=res['passed'], seed=seed, margin_min=res['margin_min'], violations=res['violations'][:10], mode='mb')
        MC.jdump(info, prefix + '.mb.info.json')
        if res['passed']:
            return
    print('  anatomy gate: no MotionBricks seed passed -> offset layer (no generated span)')
    kj, prefix = run_one(a, seeds[-1], no_mb=True)
    res = AG.gate_files(prefix + '.mb.dump.npz', kj['set'], kj['clip'], a[0], deltas=False)
    info = json.load(open(prefix + '.mb.info.json'))
    info['gate'] = dict(passed=res['passed'], seed=None, margin_min=res['margin_min'], violations=res['violations'][:10], mode='offset_fallback')
    MC.jdump(info, prefix + '.mb.info.json')
    print('  fallback gate: %s' % ('PASS' if res['passed'] else 'FAIL'))


if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(__doc__)
    else:
        main(sys.argv[1:])
