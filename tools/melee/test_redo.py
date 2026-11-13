"""Offline checks of the redo tools (base-body gate, solver, per-frame keep).  python -I test_redo.py [--rig man]   (needs base_body.py build [--rig man] once)
The rig is fixed once per process (UTPP_RIG / --rig): run the woman and the man tests as two invocations."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate_base as GB  # noqa: E402
import base_body as BB  # noqa: E402
import melee_common as MC  # noqa: E402
import redo_solve as RS  # noqa: E402


def main():
    B = RS.Body()
    al = GB.allowed()
    # 1 the base poses pass, the source pack idle passes, a vanilla melee clip fails on the shoulders in every frame
    g = GB.frame_gate(np.stack([B.Wb, B.Wi]), B.rg, steps=False, al=al)
    assert g['ok'].all(), GB.summary(g)
    g = GB.frame_gate(BB.sourcepack_W('M_Neutral_Stand_Idle_Loop'), B.rg, al=al)
    assert g['ok'].all(), GB.summary(g)
    pre = 'pwa_' if BB.RIG_NAME == 'woman' else 'pma_'      # the vanilla FPP clip of the selected body
    d = MC.load_dump(MC.clip_path(pre + 'fists', 'combo_attack_01'))
    gv = GB.frame_gate(d['W'], B.rg, al=al)
    assert not gv['ok'].any() and gv['viol']['shoulder_width_cm'].max() > 0
    # 2 solver: >= 85 % frames pass, bone lengths kept, toe bases stay, feet on the floor
    Wr, info, _ = RS.solve_refined(B, d)
    gr = GB.frame_gate(Wr, B.rg, al=al)
    assert gr['ok'].mean() >= (0.85 if BB.RIG_NAME == 'woman' else 0.90), GB.summary(gr)     # man: > 90 % (the redo has to hold on the male body as well)
    assert gr['feat']['seg_len_dev_pct'].max() < 1.0
    for S in ('Left', 'Right'):
        drift = np.ptp(Wr[:, B.ix[S + 'ToeBase'], :3, 3], axis=0)
        assert drift.max() < 0.01, drift
    assert info['leg_reach_resid_max_cm'] < 0.5, info
    # 3 per-frame keep: the upper body follows the game inside the kept range, the lower body is always the redo, ramps are continuous
    ok = np.zeros(len(Wr), bool)
    ok[15:30] = True
    Wk, w = RS.blend_keep(B, d['W'], Wr, ok, margin=3)
    lower = [B.ix[n] for n in ('Hips', 'LeftFoot', 'RightFoot', 'LeftLeg', 'RightLeg')]
    assert np.abs(Wk[:, lower] - Wr[:, lower]).max() < 1e-6
    assert w[0] == 0 and w[-1] == 0 and abs(w[22] - 1) < 1e-9 and (np.abs(np.diff(w)) <= 1 / 3 + 1e-9).all(), w
    j = B.ix['Spine3']
    assert np.abs(B.locals_of(Wk[22])[j] - B.locals_of(d['W'][22])[j]).max() < 1e-6        # kept frame = the game's upper-body locals
    if BB.RIG_NAME == 'man':
        # male base: the shipped Masculine idle + Pistol2H ready stance (overlay of the male file), male rig, redo folder separate from the female one
        assert BB.IDLE_POSE == 'Pose_Masculine_Stand_Idle' and BB.OVERLAY.name == 'utpp_overlays_man.utppov' and os.path.basename(BB.REDO) == 'redo_man', (BB.IDLE_POSE, BB.OVERLAY, BB.REDO)
        assert BB.CP.RIG_NAME == 'man' and BB.CLIP_DIR.endswith('out_sourcepack_set3_man')
    print('test_redo OK', BB.RIG_NAME, GB.summary(gr)['pass_frames'], '/', len(Wr))
    test_round_f(B, al)


def test_round_f(B, al):
    """round F: strike intent 'game_path' (M5), torso-surface clearance 'surface' (M4), keep_mask, the pose-labelled penetration proxy."""
    import redo_intent as RI
    import redo_pen as RPN
    assert RS.INTENT_MODE == 'v3b' and RS.PEN_MODE == 'axis', (RS.INTENT_MODE, RS.PEN_MODE)       # the defaults stay the v3b solver (redo / redo_man)
    pre = 'pwa_' if BB.RIG_NAME == 'woman' else 'pma_'
    d = MC.load_dump(MC.clip_path(pre + 'fists', 'combo_attack_01'))
    gv = GB.frame_gate(d['W'], B.rg, al=al)
    assert np.array_equal(RS.keep_mask(B, d['W'], gv['ok']), gv['ok'])                         # axis mode: keep mask = the gate
    ratio = RS.arm_ratio(B, d['W'])
    assert all(abs(v - 1.0) < 1e-3 for v in ratio.values()), ratio                              # same skeleton: arm-length ratio 1
    res = {}
    for mode in ('v3b', 'game_path'):
        intent = RS.plan_intent(B, d)
        intent['mode'] = mode
        tr = {}
        Wr, info, _ = RS.solve_refined(B, d, intent, trace=tr)
        Wk, _ = RS.blend_keep(B, d['W'], Wr, RS.keep_mask(B, d['W'], gv['ok']))
        res[mode] = (int(GB.frame_gate(Wk, B.rg, al=al)['ok'].sum()), RI.summary(RI.dev(B, d['W'], Wk))['worst'], tr)
    (p3, i3, t3), (pg, ig, tg) = res['v3b'], res['game_path']
    # M5: the extension push is the v3b deviation (> 10 cm), game_path follows the game hand path (worst-hand mean < 7 cm, < half of v3b), gate not lower
    assert i3['mean'] > 10.0 and ig['mean'] < 7.0 and ig['mean'] < 0.5 * i3['mean'] and pg >= p3, (i3, ig, p3, pg)
    assert np.abs(tg['ext']['Right'] - tg['raw']['Right']).max() < 1e-12                          # no arm-extension push in game_path
    # M4: on the heavy attack the surface rule has more clean frames (gate pass and no arm-through-torso frame) than the axis rule, fewer penetrating frames
    d = MC.load_dump(MC.clip_path(pre + 'two_handed_blunt', 'heavy_attack_01'))
    out = {}
    for pm in ('axis', 'surface'):
        RS.PEN_MODE = pm
        try:
            intent = RS.plan_intent(B, d)
            intent['mode'] = 'game_path'
            Wr, info, _ = RS.solve_refined(B, d, intent)
            gv = GB.frame_gate(d['W'], B.rg, al=al)
            Wk, _ = RS.blend_keep(B, d['W'], Wr, RS.keep_mask(B, d['W'], gv['ok']))
        finally:
            RS.PEN_MODE = 'axis'
        ok, g, pen = RS.clean_frames(B, Wk, al)
        out[pm] = (int(ok.sum()), int(g.sum()), int(pen.sum()))
    assert out['surface'][0] > out['axis'][0] and out['surface'][2] < out['axis'][2], out
    # the proxy: healthy base pose has no contact, the threshold is the calibrated slack
    c, pen = RPN.pen_frames(np.stack([B.Wb, B.Wi]))
    assert not pen.any() and RPN.proxy().thr == RPN.PEN_SLACK, (c, RPN.proxy().thr)
    # step 4: a closed loop clip (game frame 0 == last) keeps its seam closed with LOOP_MODE 'periodic'; 'held' (v3b default) is reported, not required
    d = MC.load_dump(MC.clip_path(pre + 'fists', 'melee_sprint_loop'))
    assert RS.LOOP_MODE == 'held' and RS.closed_loop(d['W']) and list(RS.frame_order(5)) == list(range(5))
    seam = {}
    for lm in ('held', 'periodic'):
        RS.LOOP_MODE = lm
        try:
            Wr, _, _ = RS.solve_refined(B, d)
        finally:
            RS.LOOP_MODE = 'held'
        seam[lm] = float(np.abs(Wr[-1, :, :3, 3] - Wr[0, :, :3, 3]).max() * 100)
    assert seam['periodic'] < 0.01, seam
    RS.set_loop(d)
    assert RS._PERIOD_T == 0                                       # held: no wrap
    print('test_round_f OK', BB.RIG_NAME, 'loop seam %s/melee_sprint_loop held %.2f cm -> periodic %.4f cm' % (pre + 'fists', seam['held'], seam['periodic']))
    print('test_round_f OK', BB.RIG_NAME, 'intent v3b %.1f/%.1f -> game_path %.1f/%.1f cm, pass %d -> %d; heavy clean/gate/pen axis %s surface %s'
          % (i3['mean'], i3['max'], ig['mean'], ig['max'], p3, pg, out['axis'], out['surface']))


if __name__ == '__main__':
    main()
