"""Checks of the MotionBricks stage (melee_regenerate.py).  python -I test_melee_regenerate.py [--rig man] [--no-gpu]
CPU: resample keeps endpoints / length, key planning rules (incl. the 30-frame split), ease weights, local arm lock keeps the hand-to-chest transforms (1e-6) and
the feet, key frames exact, --no-mb == V3 bit for bit; fix round E: leg identity span (F1 / F9: build_span with chain = V3 locals reproduces V3's legs, so the
keys are continuous), magnitude acceptance (F3: a deeper violation on a frame that already fails is rejected), Hips frame (F4: the MotionBricks hips delta
reaches the clip unrotated in a clip with a trajectory yaw); round F: per-unit tolerance on pairs violated in V3, clean pairs strict, new failing
frames rejected; screens: strike intent (turned body rejected) and arm-through-torso (forearm swept into the chest rejected), V3 passes both.
GPU (skipped with --no-gpu): one MotionBricks segment, generated span length 4N-6 and finite values.
The rig is fixed once per process (UTPP_RIG / --rig); temp files under melee_regenerate.MB4\\_tmp (mb4 / mb4_man)."""
import copy
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_regenerate as M  # noqa: E402
import melee_common as MC  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

PRE = 'pwa_' if M.RIG == 'woman' else 'pma_'
S0 = PRE + 'katana'
C0 = 'combo_attack_01'
S_DEF = 'default_melee_weapon' if M.RIG == 'woman' else 'pma_default_melee_weapon'


def test_resample():
    rng = np.random.default_rng(1)
    G = R.random(10 * 5, random_state=2).as_matrix().reshape(10, 5, 3, 3)
    h = rng.normal(size=(10, 3))
    for n_out in (6, 10, 14):
        Go, ho = M.MK.resample_span(G, h, n_out)
        assert len(Go) == n_out and len(ho) == n_out
        assert np.abs(Go[0] - G[0]).max() < 1e-9 and np.abs(Go[-1] - G[-1]).max() < 1e-9
        assert np.abs(ho[0] - h[0]).max() < 1e-12 and np.abs(ho[-1] - h[-1]).max() < 1e-12
    print('PASS resample endpoints / length')


def test_keys():
    B = M.RS.Body()
    al = M.GB.allowed()
    d, W3, gv, intent, _ = M.make_v3(B, al, S0, C0)
    T = len(W3)
    keys, src = M.pick_keys(B, W3, {'startup': [0, 3], 'active': [3, 11], 'recover': [11, 29]}, T)
    assert keys[0] == 0 and keys[-1] == T - 1 and np.diff(keys).min() >= M.MIN_SEG and len(keys) - 1 <= M.MAX_SEGS, keys
    keys2, _ = M.pick_keys(B, W3, {}, T)
    assert keys2[0] == 0 and keys2[-1] == T - 1 and np.diff(keys2).min() >= M.MIN_SEG, keys2
    segs = M.plan_segments([0, 4, 20, 60])
    assert segs[0]['mode'] == 'v3' and segs[1]['mode'] == 'mb' and segs[1]['N'] == 6 and segs[2]['gap'] == 4 * segs[2]['N'] - 7
    # F6: the 66-frame heavy attack (no phases in the male manifest) is never one span: every gap <= MAX_LEN, >= MIN_SEG
    dh, Wh, *_ = M.make_v3(B, al, PRE + 'two_handed_blunt', 'heavy_attack_01')
    ph = M.manifest().get((PRE + 'two_handed_blunt', 'heavy_attack_01'), {}).get('phases') or {}
    kh, srch = M.pick_keys(B, Wh, ph, len(Wh), max_len=M.MAX_LEN, nudge=M.KEY_NUDGE)
    assert kh[0] == 0 and kh[-1] == len(Wh) - 1 and np.diff(kh).max() <= M.MAX_LEN and np.diff(kh).min() >= M.MIN_SEG and len(kh) >= 4, kh
    print('PASS key planning', keys, keys2, 'heavy_attack_01', kh, srch)


def test_ease():
    for n in (11, 18, 40):
        w = M.ease_weights(n, 1.0)
        assert w[0] == 0.0 and w[-1] == 0.0 and abs(w[M.EASE_FRAMES + 1] - 1.0) < 1e-12 and abs(w[n - 2 - M.EASE_FRAMES] - 1.0) < 1e-12
        assert np.all(np.diff(w[:M.EASE_FRAMES + 2]) > 0) and 0.0 < w[1] < 0.2
        assert np.allclose(M.ease_weights(n, 0.4), 0.4 * w)
    print('PASS ease weights (0 on the keys, smoothstep over %d interior frames, x beta)' % M.EASE_FRAMES)


def synth_span(B, ctx, seg):
    """A stand-in for the MotionBricks output: the V3 SOMA span with the spine chain rotated by a smooth bump (no GPU)."""
    c, g = seg['c'], seg['g']
    G, hips = ctx['G'].copy(), ctx['hips'].copy()
    Gg, hg = G[c:g + 1].copy(), hips[c:g + 1].copy()
    n = len(Gg)
    bump = np.sin(np.linspace(0, np.pi, n))
    br = ctx['br']
    for jn in ('Spine1', 'Spine2', 'Chest', 'Spine3'):
        if jn in br.snames:
            j = br.snames.index(jn)
            for t in range(n):
                Gg[t, j] = R.from_rotvec([0.0, 0.0, np.radians(9.0) * bump[t]]).as_matrix() @ Gg[t, j]
    hg[:, 0] += 0.02 * bump
    return Gg, hg


def test_lock_and_keys():
    B = M.RS.Body()
    al = M.GB.allowed()
    wd = os.path.join(M.MB4, '_tmp', 'test_melee_regenerate')
    ctx, jobs = M.prepare(B, al, S0, C0, [0], wd)
    assert jobs, 'no mb segment in %s/%s' % (S0, C0)
    seg = [s for s in ctx['segs'] if s['mode'] == 'mb'][0]
    c, g = seg['c'], seg['g']
    Gg, hg = synth_span(B, ctx, seg)
    Lmb, Winv = M.mb_chain(B, ctx['br'], ctx, seg, Gg, hg)
    ix = B.ix
    for beta in (1.0, 0.5):
        Ws, resid = M.build_span(B, ctx, seg, Lmb, beta)
        W4 = M.assemble(ctx['W3'], [(c, g, Ws)])
        W3 = ctx['W3']
        # the span really changed (the synthetic bump), key frames and everything outside are V3 bit for bit
        assert np.abs(W4[c + 1:g] - W3[c + 1:g]).max() > 1e-3
        assert np.array_equal(W4[:c + 1], W3[:c + 1]) and np.array_equal(W4[g:], W3[g:])
        # arm lock: every joint under LeftShoulder / RightShoulder keeps its V3 transform relative to Spine3 (1e-6), also the helpers under the hands
        sub = [j for j in range(len(B.names)) if any(B.names[k] in ('LeftShoulder', 'RightShoulder') for k in M_anc(B, j))]
        assert len(sub) > 30
        for t in range(c + 1, g):
            r4 = np.linalg.inv(W4[t, ix['Spine3']]) @ W4[t, sub]
            r3 = np.linalg.inv(W3[t, ix['Spine3']]) @ W3[t, sub]
            assert np.abs(r4 - r3).max() < 1e-6, (beta, t, np.abs(r4 - r3).max())
        # feet pinned: Foot / ToeBase / Heel world transforms equal V3 (leg IK reaches), min z unchanged
        for n_ in ('Foot', 'ToeBase', 'Heel'):
            for S in ('Left', 'Right'):
                j = ix[S + n_]
                assert np.abs(W4[:, j, :3, 3] - W3[:, j, :3, 3]).max() < 2e-3, (n_, S)
        assert resid < 0.5, resid
        # the hips follow the bump (beta 1: ~2 cm)
        dz = np.abs(W4[:, ix['Hips'], :3, 3] - W3[:, ix['Hips'], :3, 3]).max()
        assert dz > 0.003 * beta, dz
        # F2: eased in / out: the first / last interior frame carries only ease(1) ~ 0.16 of the delta
        d1 = np.linalg.norm(W4[c + 1, ix['Hips'], :3, 3] - W3[c + 1, ix['Hips'], :3, 3])
        assert d1 < 0.3 * dz + 1e-4, (d1, dz)
    shutil.rmtree(wd, ignore_errors=True)
    print('PASS local arm lock (1e-6), feet pinned, key frames / outside frames bit for bit, eased key frames')


def M_anc(B, j):
    out = []
    while j >= 0:
        out.append(j)
        j = B.par[j]
    return out


def test_leg_identity():
    """F1 / F9: build_span with an IDENTITY chain (V3 locals, no MotionBricks) on <pre>fists/block_deflect_right segment 0..13 must give V3's legs on every
    frame: UpLeg world rotation within 1 deg and knee (Leg joint) within 0.3 cm (before the fix: 5.49 deg / 1.17 cm, snapping back at the key)."""
    B = M.RS.Body()
    al = M.GB.allowed()
    wd = os.path.join(M.MB4, '_tmp', 'test_leg_identity')
    ctx, _ = M.prepare(B, al, PRE + 'fists', 'block_deflect_right', [0], wd, no_mb=True)
    seg = [s for s in ctx['segs'] if s['c'] == 0][0]
    c, g = seg['c'], seg['g']
    assert (c, g) == (0, 13), (c, g, ctx['keys'])
    ix = B.ix
    Ws, resid = M.build_span(B, ctx, seg, ctx['L3'][c:g + 1].copy(), 1.0)
    W3 = ctx['W3'][c:g + 1]
    up = max(float(M.rot_angle_deg(Ws[:, ix[S + 'UpLeg'], :3, :3], W3[:, ix[S + 'UpLeg'], :3, :3]).max()) for S in ('Left', 'Right'))
    kn = max(float(np.linalg.norm(Ws[:, ix[S + 'Leg'], :3, 3] - W3[:, ix[S + 'Leg'], :3, 3], axis=1).max() * 100) for S in ('Left', 'Right'))
    assert up < 1.0 and kn < 0.3 and resid < 0.05, (up, kn, resid)
    # and the assembled clip passes every continuity / join screen
    W4 = M.assemble(ctx['W3'], [(c, g, Ws)])
    gk = M.GB.frame_gate(W4, B.rg, al=al)
    pr = M.window_problems(B, ctx, seg, W4, gk)
    assert not pr, pr
    shutil.rmtree(wd, ignore_errors=True)
    print('PASS leg identity span %s 0..13: UpLeg %.3f deg, knee %.3f cm, IK resid %.4f cm, no join problem' % (PRE + 'fists/block_deflect_right', up, kn, resid))


def test_magnitude():
    """F3: a violation that deepens on a frame which already fails in V3 (frame count unchanged) is rejected; equal depth passes."""
    B = M.RS.Body()
    al = M.GB.allowed()
    wd = os.path.join(M.MB4, '_tmp', 'test_magnitude')
    ctx, _ = M.prepare(B, al, S0, C0, [0], wd, no_mb=True)
    seg = ctx['segs'][0]
    f = (seg['c'] + seg['g']) // 2
    k = 'hand_eye_cm' if 'hand_eye_cm' in ctx['gate3']['viol'] else sorted(ctx['gate3']['viol'])[0]
    g3 = copy.deepcopy(ctx['gate3'])
    g3['viol'][k] = g3['viol'][k].copy()
    g3['viol'][k][f] = 0.5
    g3['ok'] = g3['ok'].copy()
    g3['ok'][f] = False
    ctx['gate3'] = g3
    same = copy.deepcopy(g3)
    sl = slice(0, ctx['T'])
    assert not M.deeper_violations(same, g3, sl) and not M.deeper_violations(same, g3, sl, mode='strict')
    assert not [p for p in M.window_problems(B, ctx, seg, ctx['W3'], same) if p.startswith(('deeper', 'new violation', 'gate pass', 'new failing'))]
    # round F (F3', default unit_tol): a pair already violated in V3 may deepen by MAG_TOL[unit]; beyond that it is rejected
    assert M.MAG_MODE == 'unit_tol'
    tol = M.MAG_TOL[M.feature_unit(k)]
    assert M.feature_unit(k) == 'cm' and tol == 0.5, (k, M.feature_unit(k), tol)
    deep = copy.deepcopy(g3)
    deep['viol'][k][f] = 0.5 + 0.01
    pr = M.window_problems(B, ctx, seg, ctx['W3'], deep)
    assert not any(p.startswith(('deeper violation', 'new violation', 'new failing')) for p in pr), pr
    assert M.deeper_violations(deep, g3, sl) == {} and M.deeper_violations(deep, g3, sl, mode='strict') == {k: 0.01}
    over = copy.deepcopy(g3)
    over['viol'][k][f] = 0.5 + tol + 0.01
    pr = M.window_problems(B, ctx, seg, ctx['W3'], over)
    assert any(p.startswith('deeper violation') for p in pr), pr
    assert M.deeper_violations(over, g3, sl) == {k: round(tol + 0.01, 4)}
    # a pair CLEAN in V3 stays strict, even on a frame that already fails (another feature)
    k2 = [x for x in sorted(g3['viol']) if x != k and g3['viol'][x][f] <= 1e-9 and M.feature_unit(x) == 'deg'][0]
    new = copy.deepcopy(g3)
    new['viol'][k2] = new['viol'][k2].copy()
    new['viol'][k2][f] = 0.01
    assert M.deeper_violations(new, g3, sl) == {k2: 0.01}, M.deeper_violations(new, g3, sl)
    # a frame that passes in V3 must pass in V4 ('new failing frames'), whatever the depth
    w0, w1 = max(0, seg['c'] - 1), min(ctx['T'] - 1, seg['g'] + 1)
    f2 = [x for x in range(w0, w1 + 1) if g3['ok'][x] and x != f][0]
    nf = copy.deepcopy(g3)
    nf['ok'] = nf['ok'].copy()
    nf['ok'][f2] = False
    pr = M.window_problems(B, ctx, seg, ctx['W3'], nf)
    assert any(p.startswith('new failing frames [%d]' % f2) for p in pr), pr
    # the round E rule is still available: strict rejects the +0.01
    M.MAG_MODE = 'strict'
    try:
        pr = M.window_problems(B, ctx, seg, ctx['W3'], deep)
        assert any(p.startswith('deeper violation') for p in pr) and not any(p.startswith('new violation') for p in pr), pr
    finally:
        M.MAG_MODE = 'unit_tol'
    # every gate feature has a unit (no silent tolerance)
    units = {x: M.feature_unit(x) for x in g3['viol']}
    assert set(units.values()) <= set(M.MAG_TOL), units
    shutil.rmtree(wd, ignore_errors=True)
    print('PASS magnitude acceptance (unit_tol: %s +0.01 on f%d accepted, +%.2f rejected; clean pair %s +0.01 rejected; new failing frame f%d rejected; '
          'strict: +0.01 rejected; units %s)' % (k, f, tol + 0.01, k2, f2, sorted(set(units.values()))))


def _descendants(par, j0):
    out = []
    for j in range(len(par)):
        k = j
        while k >= 0 and k != j0:
            k = par[k]
        if k == j0:
            out.append(j)
    return out


def test_screens():
    """round F screens (extra_screens): V3 itself passes; the strike-intent screen rejects the body turned 90 deg about the vertical through Spine3
    (hand path off the game path); the penetration screen rejects the right forearm + hand swept step by step from their place into the chest centre."""
    B = M.RS.Body()
    al = M.GB.allowed()
    wd = os.path.join(M.MB4, '_tmp', 'test_screens')
    ctx, _ = M.prepare(B, al, PRE + 'fists', 'block_deflect_right', [0], wd, no_mb=True)
    W3 = ctx['W3']
    seg = dict(c=0, g=len(W3) - 1)                  # the screens look at the interior frames c+1..g-1 only: the whole clip as one window
    r0, m0 = M.extra_screens(B, ctx, seg, W3)
    assert not r0, (r0, m0)
    ix = B.ix
    Rz = np.eye(4)
    Rz[:3, :3] = R.from_euler('z', 90, degrees=True).as_matrix()
    Wt = W3.copy()
    for t in range(1, len(W3) - 1):
        T1, T0 = np.eye(4), np.eye(4)
        T1[:3, 3], T0[:3, 3] = W3[t, ix['Spine3'], :3, 3], -W3[t, ix['Spine3'], :3, 3]
        Wt[t] = T1 @ Rz @ T0 @ W3[t]
    ri, mi = M.extra_screens(B, ctx, seg, Wt)
    assert any(x.startswith('intent dev') for x in ri) and not any(x.startswith('penetration') for x in ri), (ri, mi)    # rigid turn: no new contact
    P = M.RPN.proxy()
    j0 = ix['RightForeArm']
    chain = _descendants(P.par, j0)
    Wp = W3.copy()
    n = len(W3) - 2
    for k, t in enumerate(range(1, len(W3) - 1)):
        mid = 0.5 * (W3[t, j0, :3, 3] + W3[t, ix['RightHand'], :3, 3])
        Wp[t, chain, :3, 3] += (k + 1) / n * (W3[t, ix['Spine2'], :3, 3] - mid)
    rp, mp = M.extra_screens(B, ctx, seg, Wp)
    assert any(x.startswith('penetration') for x in rp), (rp, mp)
    shutil.rmtree(wd, ignore_errors=True)
    print('PASS screens: V3 passes (intent over v3 max %.2f cm, pen max %d); turned body rejected (%s); forearm through the chest rejected (%s, pen max %d, thr %d)'
          % (m0['intent_over_v3_max'], m0['pen_max'], ri[0], [x for x in rp if x.startswith('penetration')][0], mp['pen_max'], P.thr))


def test_hips_frame():
    """F4: in a clip whose Trajectory joint carries a fixed yaw (default_melee_weapon / fall_attack: 48.6 deg) a synthetic MotionBricks hips bump must reach the
    clip unrotated: the hips translation delta of mb_chain equals the raw cp_inv (game world) delta; the pre-fix formulation (relative to Trajectory) is off."""
    B = M.RS.Body()
    al = M.GB.allowed()
    wd = os.path.join(M.MB4, '_tmp', 'test_hips_frame')
    ctx, jobs = M.prepare(B, al, S_DEF, 'fall_attack', [0], wd)
    ix = B.ix
    W3, L3 = ctx['W3'], ctx['L3']
    yaw = float(M.rot_angle_deg(W3[0, ix['Trajectory'], :3, :3], np.eye(3)))
    assert yaw > 30.0, yaw
    seg = [s for s in ctx['segs'] if s['mode'] == 'mb'][0]
    c, g = seg['c'], seg['g']
    Gg, hg = ctx['G'][c:g + 1].copy(), ctx['hips'][c:g + 1].copy()
    bump = np.sin(np.linspace(0, np.pi, len(Gg)))
    hg[:, 0] += 0.03 * bump
    hg[:, 1] += 0.02 * bump
    Lmb, Winv = M.mb_chain(B, ctx['br'], ctx, seg, Gg, hg)
    j = ix['Hips']
    d_raw = Winv[:, j, :3, 3] - W3[c:g + 1, j, :3, 3]                      # MotionBricks hips delta in the game world
    d_new = Lmb[:, j, :3, 3] - L3[c:g + 1, j, :3, 3]                       # delta handed to the clip (Hips parent Root = identity)
    err = float(np.linalg.norm(d_new - d_raw, axis=1).max() * 100)
    rerr = float(M.rot_angle_deg(Lmb[:, j, :3, :3], Winv[:, j, :3, :3]).max())
    # the pre-fix formulation: hips relative to the V3 Trajectory joint, endpoints fixed by the same linear offset
    old = np.linalg.inv(W3[c:g + 1, ix['Trajectory']]) @ Winv[:, j]
    w = np.linspace(0.0, 1.0, len(old))
    ta, tb = L3[c, j, :3, 3] - old[0, :3, 3], L3[g, j, :3, 3] - old[-1, :3, 3]
    d_old = old[:, :3, 3] + (1 - w)[:, None] * ta + w[:, None] * tb - L3[c:g + 1, j, :3, 3]
    err_old = float(np.linalg.norm(d_old - d_raw, axis=1).max() * 100)
    amp = float(np.linalg.norm(d_raw, axis=1).max() * 100)
    assert amp > 2.0 and err < 0.05 and rerr < 0.5 and err_old > 0.5, (amp, err, rerr, err_old)
    shutil.rmtree(wd, ignore_errors=True)
    print('PASS hips frame %s/fall_attack (trajectory yaw %.1f deg): bump %.2f cm, delta error %.4f cm (pre-fix formulation %.2f cm), hips rotation vs cp_inv %.3f deg'
          % (S_DEF, yaw, amp, err, err_old, rerr))


def test_no_mb_identity():
    out = os.path.join(M.MB4, '_tmp', 'test_nomb_out')
    shutil.rmtree(out, ignore_errors=True)
    infos = M.run_clips([(S0, C0)], [0], out, no_mb=True)
    v3 = np.load(os.path.join(out, '%s__%s.v3.npz' % (S0, C0)))['W']
    v4 = np.load(os.path.join(out, '%s__%s.v4.npz' % (S0, C0)))['W']
    assert np.array_equal(v3, v4) and infos[0]['identical_to_v3']
    # and V3 is exactly what redo_demo.py produces
    B = M.RS.Body()
    d, Wk, *_ = M.make_v3(B, M.GB.allowed(), S0, C0)
    assert np.array_equal(Wk, v3)
    shutil.rmtree(out, ignore_errors=True)
    print('PASS --no-mb == V3 bit for bit')


def test_mb_smoke():
    B = M.RS.Body()
    al = M.GB.allowed()
    wd = os.path.join(M.MB4, '_tmp', 'test_melee_regenerate_gpu')
    ctx, jobs = M.prepare(B, al, S0, C0, [0], wd)
    seg_i = [i for i, s in enumerate(ctx['segs']) if s['mode'] == 'mb'][0]
    jobs = [j for j in jobs if j['out'].endswith('seg%d' % seg_i)]
    M.run_mb_jobs(jobs, os.path.join(wd, 'jobs.json'))
    seg = ctx['segs'][seg_i]
    gen = np.load(os.path.join(wd, 'seg%d.g0.s-1.gen.npz' % seg_i))
    assert len(gen['G']) == 4 * seg['N'] - 6 and len(gen['hips']) == 4 * seg['N'] - 6, (len(gen['G']), seg['N'])
    assert np.isfinite(gen['G']).all() and np.isfinite(gen['hips']).all()
    shutil.rmtree(wd, ignore_errors=True)
    print('PASS MotionBricks smoke: span %d frames = 4N-6, finite' % (4 * seg['N'] - 6))


def run_all(no_gpu=False):
    print('RIG %s MB4 %s' % (M.RIG, M.MB4))
    test_resample()
    test_keys()
    test_ease()
    test_lock_and_keys()
    test_leg_identity()
    test_magnitude()
    test_screens()
    test_hips_frame()
    test_no_mb_identity()
    if not no_gpu:
        test_mb_smoke()
    print('ALL PASS')


if __name__ == '__main__':
    run_all('--no-gpu' in sys.argv)
