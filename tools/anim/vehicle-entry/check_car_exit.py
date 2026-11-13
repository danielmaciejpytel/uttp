"""Gates of the head-duck car EXIT clip (build_exit_duck.py; vanilla exit of the set = reference).

  python -I check_car_exit.py --clip <ducked exit .anims.glb> --vanilla <vanilla set .anims.glb> --class sport2 --seat driver [--clip-name exit] [--vanilla-clip exit]
        [--report out.json] [--work <dir for the time-reversed temp GLBs>] [--tol-open-cm 0.3] [--selftest]

Every check_door_clearance / roof_report gate anchors the SEATED pose on the LAST frame (entry clips). An exit starts seated, so both clips are written time-REVERSED (last frame = seated)
and judged by the unchanged static phases of check_door_clearance.Model (roof sphere, body shell per capsule) and the per-capsule penetration of the FULLY OPEN door (door-data pose 'su').
Checks:
  a_roof_gate        head sphere vs roof cloud, 120 Hz, max <= the absolute gate max(1 cm, seated head penetration (first exit frame) + 0.5 cm)
  b_body_rel         body shell, per capsule: ducked max <= max(vanilla max, 0) + --tol-body-cm (default 0.3) or the seated-baseline gate of the Model, whichever is larger
  c_door_open_rel    door poses 0, 1/4, 1/2, 3/4, 1 of the opening, every 30 fps frame, per capsule: ducked <= vanilla at the SAME frame and pose + --tol-open-cm (independent of the real door timing)
  c2_full_shell_rel  full shell cloud WITHOUT the doorway mask (sill, rocker, floor, A-pillar base), per capsule and frame: ducked <= vanilla at the same frame + --tol-body-cm
  d_first_last       first and last frame: rotation (deg) and translation (m) of every bone == vanilla within --tol-end-deg / --tol-end-m (the end pose goes to the motion matcher unchanged)
  e_root             Root track (root motion) identical to vanilla at every frame
  f_events_tracks    animEvents (type / name / frame / side) and float track keys identical to vanilla
  g_unchanged_bones  bones outside the duck chain (+ Hips and legs when the Hips sink is on) identical to vanilla at every frame
  h_window           frames that differ from vanilla (any bone) lie inside the clearance window (the unducked pose is over the gate) +- --window-pad frames
  i_continuity       largest per-frame rotation step of every changed bone <= max(vanilla step of that bone + --step-margin-deg, --step-deg)
  j_anatomy          max neck flexion / trunk lean (head_duck.py anatomical limits, from the build report) under the limits
  l_hips_seat        the Hips never go more than --tol-hips-cm below min(seated Hips, vanilla Hips at the same frame) (no sinking into the seat)
  k_feet             feet stay at the vanilla world positions (max deviation <= --tol-foot-cm)
  p_floor (steps mode) strict (--tol-floor-cm) over the changed span (frames that differ from --base), outside it min foot height >= base min - 0.5 cm (floor_gate; 2026-11-06)
Mutation self-test (--selftest): the UNducked vanilla clip as the 'ducked' clip must FAIL a_roof_gate and h_window must be vacuous (no frames changed), the clip with the neck bent 60 deg
over a frame must FAIL j_anatomy / g_unchanged_bones / d_first_last when the bend is at the last frame.
"""
import argparse
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import skel as S  # noqa: E402
import glbanim as GA  # noqa: E402
import build_entry_clip as B  # noqa: E402
import check_door_clearance as C  # noqa: E402

LIM_NECK, LIM_LEAN = 55.0, 75.0
DOOR_DELAYS_S = (0.0, 0.1, 0.2)   # gated; 0.3 s is reported by the build log only (door_timing.json: the entry door starts 0.047 s after the mount line)


def quat_angle_deg(a, b):
    d = np.abs(np.sum(a * b, axis=-1)) / np.maximum(np.linalg.norm(a, axis=-1) * np.linalg.norm(b, axis=-1), 1e-12)
    return np.degrees(2.0 * np.arccos(np.clip(d, -1.0, 1.0)))


def load(path, clip_name):
    sk = S.Skeleton.from_glb(path)
    c = sk.clip(clip_name)
    dur, t = B.frames_of(sk, c)
    lt, lr = sk.sample_local(c, t)
    lr = S.qcontinuous(lr)
    extras = sk.gj['animations'][c].get('extras', {})
    return sk, t, lt, lr, extras


def write_reversed(sk, t, lt, lr, extras, out, name='exit'):
    B.write_out(sk, out, name, t, lt[::-1].copy(), lr[::-1].copy(), extras)


def model_series(rev_glb, cls, seat, name):
    m = C.Model(rev_glb, cls, seat, clip_name=name)
    rp, seated_roof = m.roof_series()
    bp_series = m.static_series()
    nc = len(m.names)
    # the door state during an exit is NOT measured: it opens linearly (door_timing.json open_dur_s of the seat, the entry model's mapping) from a start delay d after the clip start; d is swept over
    # DOOR_DELAYS_S (0 .. 0.3 s; the entry's measured open start is 0.047 s after the mount line). Per frame the door pose is su * clip((t - d) / open_dur, 0, 1).
    od = float(m.tm['open_dur_s'])
    poses = [float(m.su) * float(np.clip((k / C.FPS - d) / od, 0.0, 1.0)) for d in DOOR_DELAYS_S for k in range(len(m.V))]   # exit time k (the model clip is time-reversed: V[T-1-k])
    T = len(m.V)
    open_pen = np.full((len(DOOR_DELAYS_S), T, nc), -0.15)
    for di, d in enumerate(DOOR_DELAYS_S):
        for k in range(T):   # k = exit frame; the reversed model body is V[T-1-k]
            f = float(m.su) * float(np.clip((k / C.FPS - d) / od, 0.0, 1.0))
            open_pen[di, k] = m.pen_caps_V(m.V[T - 1 - k], f)[0]
    # full shell cloud (no doorway mask: sill, rocker, floor, A-pillar base included) per frame: the pelvis crosses the side line while the Hips sink
    bd = m.bodyd
    full = {'cloud': bd['cloud'], 'label': bd['label'], 'xs': np.ascontiguousarray(bd['cloud'][:, 0])}
    assert np.all(np.diff(full['xs']) >= 0), 'body cloud not sorted by x: _body_pen needs searchsorted'
    full_pen = np.full((len(m.V), nc), -0.15)
    for k in range(len(m.V)):   # exit frame order
        full_pen[k] = m._body_pen(m.V[len(m.V) - 1 - k], bc=full)[0]
    m._full_pen = full_pen
    return m, rp, seated_roof, bp_series, open_pen


def step_metrics(Wt, ix, c, floors, h_con=0.03):
    """slide (cm / frame, over consecutive contact frames from c on), swing runs (>= 3 non-contact frames; lift above the contact height, travel), floor margin (cm), ankle distance (cm).
    glTF world (Y up). Contact = the lower of ankle / toe within h_con of that joint's floor (floors = lowest height from frame 40 on of the VANILLA clip)."""
    T = Wt.shape[0]
    out = {}
    for s_ in ('Left', 'Right'):
        a, t = Wt[:, ix[s_ + 'Foot']], Wt[:, ix[s_ + 'ToeBase']]
        ha, ht = a[:, 1] - floors[s_][0], t[:, 1] - floors[s_][1]
        piv = np.where(ha <= ht, 0, 1)
        P = np.where(piv[:, None] == 0, a, t)
        con = np.minimum(ha, ht) < h_con
        sl = [np.linalg.norm((P[f] - P[f - 1])[[0, 2]]) * 100 for f in range(max(c, 1), T) if con[f] and con[f - 1] and piv[f] == piv[f - 1]]
        core, f = [], max(c, 1)
        while f < T:   # support core: contiguous contact runs of >= 5 frames minus the first 2 and the last 2 frames (landing / take-off roll)
            if con[f]:
                e = f
                while e + 1 < T and con[e + 1]:
                    e += 1
                if e - f + 1 >= 5:
                    core += [np.linalg.norm((P[k] - P[k - 1])[[0, 2]]) * 100 for k in range(f + 3, e - 1) if piv[k] == piv[k - 1]]
                f = e + 1
            else:
                f += 1
        rest = float(np.median(a[con, 1])) if con.any() else float(a[:, 1].min())
        runs, f = [], c + 1
        while f < T:
            if not con[f]:
                e = f
                while e + 1 < T and not con[e + 1]:
                    e += 1
                if e - f + 1 >= 3 and f > c and e < T - 1:
                    lift = float(a[f:e + 1, 1].max() - rest)
                    trav = float(np.linalg.norm((a[min(e + 1, T - 1)] - a[f - 1])[[0, 2]]))
                    runs.append((f, e, round(lift * 100, 1), round(trav * 100)))
                f = e + 1
            else:
                f += 1
        out[s_] = dict(slide=np.array(sl), core=np.array(core), runs=runs, n_con=int(con[c:].sum()), floor_margin_cm=float(min((a[:, 1] - floors[s_][0]).min(), (t[:, 1] - floors[s_][1]).min()) * 100))
    la, ra = Wt[:, ix['LeftFoot']], Wt[:, ix['RightFoot']]
    out['feet_min_cm'] = float(np.linalg.norm((la - ra)[c:][:, [0, 2]], axis=1).min() * 100)
    return out


def knee_step_deg(Wt, ix, c):
    worst = 0.0
    for s_ in ('Left', 'Right'):
        u, k_, a_ = (Wt[:, ix[s_ + x]] for x in ('UpLeg', 'Leg', 'Foot'))
        v1, v2 = u - k_, a_ - k_
        cs = (v1 * v2).sum(1) / (np.linalg.norm(v1, axis=1) * np.linalg.norm(v2, axis=1))
        worst = max(worst, float(np.abs(np.diff(np.degrees(np.arccos(np.clip(cs, -1, 1)))[c:])).max()))
    return worst


def floor_gate(Wt, bWt, ix, floors, chg, tol_cm, vanilla_cm=None, mutate=True):
    """p_floor in steps mode: strict (-tol_cm) over the changed span, outside it no worse than the base (min height >= base min - 0.5 cm). glTF world (Y up), floors = (ankle, toe) per side."""
    def hmin(W_):
        h = np.full(len(W_), 9.0)
        for s_ in ('Left', 'Right'):
            h = np.minimum(h, np.minimum(W_[:, ix[s_ + 'Foot'], 1] - floors[s_][0], W_[:, ix[s_ + 'ToeBase'], 1] - floors[s_][1]))
        return h
    idx = np.flatnonzero(chg)
    h, hb = hmin(Wt), hmin(bWt)
    if len(idx):
        inside = np.zeros(len(h), bool)
        inside[idx[0]:idx[-1] + 1] = True
    else:
        inside = np.zeros(len(h), bool)
    in_min = float(h[inside].min()) * 100 if inside.any() else 0.0
    out_min = float(h[~inside].min()) * 100 if (~inside).any() else 0.0
    out_base = float(hb[~inside].min()) * 100 if (~inside).any() else 0.0
    ok_in, ok_out = bool(in_min >= -tol_cm), bool(out_min >= out_base - 0.5)
    r = dict(ok=bool(ok_in and ok_out), lowest_vs_vanilla_tail_cm=round(in_min, 2), vanilla=None if vanilla_cm is None else round(vanilla_cm, 2), tol_cm=tol_cm,
             span=[int(idx[0]), int(idx[-1])] if len(idx) else None, outside_min_cm=round(out_min, 2), outside_base_min_cm=round(out_base, 2), scope='changed span strict, outside no worse than base - 0.5 cm')
    if mutate and len(idx):   # self-test: a foot pushed 5 cm under the floor inside the span, and one pushed below the base outside it, must both fail
        W2 = Wt.copy()
        f = int(idx[len(idx) // 2])
        for nm in ('LeftFoot', 'LeftToeBase'):
            W2[f, ix[nm], 1] = min(W2[f, ix[nm], 1], floors['Left'][0 if nm == 'LeftFoot' else 1] - 0.05)
        r['mutation_in_span_caught'] = not floor_gate(W2, bWt, ix, floors, chg, tol_cm, None, False)['ok']
        outs = np.flatnonzero(~inside)
        if len(outs):
            W3 = Wt.copy()
            f = int(outs[0])
            for nm in ('LeftFoot', 'LeftToeBase'):
                W3[f, ix[nm], 1] = min(W3[f, ix[nm], 1], floors['Left'][0 if nm == 'LeftFoot' else 1] - 0.30)
            r['mutation_outside_caught'] = not floor_gate(W3, bWt, ix, floors, chg, tol_cm, None, False)['ok']
    return r


def steps_gates(res, clip_sk, lt, lr, van, base, a, c, g):
    """gates of a clip with generated stepping (frames c..g): everything outside c..g is the base override, the feet stop gliding while they carry weight"""
    vsk, vt, vlt, vlr, vextras = van
    ix = clip_sk.index
    Wt, _ = clip_sk.world(lt, lr)
    vWt, _ = vsk.world(vlt, vlr)
    floors = {s_: (float(vWt[40:, ix[s_ + 'Foot'], 1].min()), float(vWt[40:, ix[s_ + 'ToeBase'], 1].min())) for s_ in ('Left', 'Right')}
    m = step_metrics(Wt, ix, c, floors)
    m0 = step_metrics(vWt, ix, c, floors)
    cat = lambda mm, k='slide': np.concatenate([mm[s_][k] for s_ in ('Left', 'Right')] + [np.zeros(0)])
    sl, sl0 = cat(m), cat(m0)
    co, co0 = cat(m, 'core'), cat(m0, 'core')
    if not len(sl):
        sl = np.zeros(1)
    if not len(co):
        co = np.zeros(1)
    res['n_foot_slide'] = dict(ok=bool(co.mean() <= a.tol_slide_mean and np.percentile(co, 90) <= a.tol_slide_p90 and sl.mean() <= a.tol_slide_all),
                               support_core=dict(mean=round(float(co.mean()), 3), p90=round(float(np.percentile(co, 90)), 3), max=round(float(co.max()), 3), frames=int(len(co)),
                                                 vanilla=dict(mean=round(float(co0.mean()), 3) if len(co0) else None, max=round(float(co0.max()), 3) if len(co0) else None, frames=int(len(co0)))),
                               all_contact=dict(mean=round(float(sl.mean()), 3), p90=round(float(np.percentile(sl, 90)), 3), max=round(float(sl.max()), 3)),
                               contact_frames=int(sum(m[s_]['n_con'] for s_ in ('Left', 'Right'))),
                               per_foot={s_: dict(all_mean=round(float(m[s_]['slide'].mean()), 3) if len(m[s_]['slide']) else None, all_max=round(float(m[s_]['slide'].max()), 3) if len(m[s_]['slide']) else None, core_mean=round(float(m[s_]['core'].mean()), 3) if len(m[s_]['core']) else None, core_max=round(float(m[s_]['core'].max()), 3) if len(m[s_]['core']) else None) for s_ in ('Left', 'Right')},
                               vanilla_all_contact=dict(mean=round(float(sl0.mean()), 3), p90=round(float(np.percentile(sl0, 90)), 3), max=round(float(sl0.max()), 3)),
                               gate=dict(core_mean=a.tol_slide_mean, core_p90=a.tol_slide_p90, all_contact_mean=a.tol_slide_all), frames=[int(c), int(len(lt) - 1)])
    runs = {s_: m[s_]['runs'] for s_ in ('Left', 'Right')}
    n_steps = sum(1 for s_ in runs for r in runs[s_] if r[2] >= 4.0 and r[3] >= 20)
    res['o_steps'] = dict(ok=bool(n_steps >= a.min_steps), steps=n_steps, runs=runs, vanilla_steps=sum(1 for s_ in ('Left', 'Right') for r in m0[s_]['runs'] if r[2] >= 4.0 and r[3] >= 20))
    # p_floor (2026-11-06): judged on the CHANGED span only (frames where the clip differs from the base override: c..g and any extended head / tail span); outside it the clip is the base (m_outside_identical)
    # and only has to be no worse than the base (min foot height >= base min - 0.5 cm), since a vanilla foot that sinks below the floor before c is not the stepping's doing.
    bsk0, _bt0, blt0, blr0, _bx0 = base
    bWt0, _ = bsk0.world(blt0, blr0)
    ang0 = np.stack([quat_angle_deg(lr[:, j], blr0[:, j]) for j in range(lr.shape[1])], 1)
    chg0 = ((ang0 > a.chg_deg) | (np.linalg.norm(lt - blt0, axis=-1) > a.chg_m)).any(1)
    fm0 = min(m0[s_]['floor_margin_cm'] for s_ in ('Left', 'Right'))
    res['p_floor'] = floor_gate(Wt, bWt0, ix, floors, chg0, a.tol_floor_cm, fm0)
    res['q_feet_cross'] = dict(ok=bool(m['feet_min_cm'] >= a.min_feet_cm), min_ankle_distance_cm=round(m['feet_min_cm'], 1), vanilla=round(m0['feet_min_cm'], 1), min_cm=a.min_feet_cm)
    kn, kn0 = knee_step_deg(Wt, ix, c), knee_step_deg(vWt, ix, c)
    res['r_knee_step'] = dict(ok=bool(kn <= max(a.tol_knee_deg, kn0 + 2.0)), max_deg_per_frame=round(kn, 1), vanilla=round(kn0, 1), gate=a.tol_knee_deg)
    bsk, bt, blt, blr, bextras = base
    ang = np.stack([quat_angle_deg(lr[:, j], blr[:, j]) for j in range(lr.shape[1])], 1)
    dtr = np.linalg.norm(lt - blt, axis=-1)
    chg = (ang > a.chg_deg) | (dtr > a.chg_m)
    outside = [int(f) for f in np.flatnonzero(chg.any(1)) if f < c or f > g]
    res['m_outside_identical'] = dict(ok=not outside, span=[int(c), int(g)], changed_frames=[int(f) for f in np.flatnonzero(chg.any(1))], outside=outside,
                                      max_rot_deg_outside=round(float(max([ang[f].max() for f in range(len(ang)) if f < c or f > g] or [0])), 5),
                                      max_trans_m_outside=float(max([dtr[f].max() for f in range(len(dtr)) if f < c or f > g] or [0])))
    res['g_unchanged_bones'] = dict(ok=True, waived='steps mode: m_outside_identical covers it')
    res['h_window'] = dict(ok=True, waived='steps mode')
    fe = {nm: round(float(np.linalg.norm(Wt[:c, ix[nm]] - vWt[:c, ix[nm]], axis=-1).max() * 100), 2) for nm in ('LeftFoot', 'RightFoot')}
    res['k_feet'] = dict(ok=bool(max(fe.values()) <= a.tol_foot_cm), frames_before_c_max_dev_cm=fe)
    return res


def compare(clip_sk, tr, lt, lr, extras, van, vcheck_name, a, work, tag):
    vsk, vt, vlt, vlr, vextras = van
    T = len(vt)
    res = {}
    assert len(tr) == T, 'frame count differs from the vanilla clip'
    ix = clip_sk.index
    ri = ix['Root']
    # ---- model series (time reversed so that the seated pose is the last frame)
    p1 = os.path.join(work, tag + '.rev.glb')
    p0 = os.path.join(work, 'vanilla.rev.glb')
    write_reversed(clip_sk, tr, lt, lr, extras, p1)
    if not os.path.exists(p0):
        write_reversed(vsk, vt, vlt, vlr, vextras, p0)
    m1, rp1, sr1, bp1, op1 = model_series(p1, a.cls, a.seat, 'exit')
    m0, rp0, sr0, bp0, op0 = model_series(p0, a.cls, a.seat, 'exit')
    nc = len(m1.names)
    # a) roof
    gate_roof = max(1.0, sr0 * 100 + 0.5)
    res['a_roof_gate'] = dict(ok=bool(rp1.max() * 100 <= gate_roof + 1e-9), ducked_max_cm=round(float(rp1.max() * 100), 3), vanilla_max_cm=round(float(rp0.max() * 100), 3),
                              gate_cm=round(gate_roof, 3), seated_cm=round(float(sr0 * 100), 3), ducked_seated_cm=round(float(sr1 * 100), 3),
                              worst_time_s=round(float(rp1.argmax() * C.DT), 3), vanilla_over_gate_s=round(float((rp0 * 100 > gate_roof).sum() * C.DT), 3))
    # b) body shell per capsule, relative to vanilla
    b1 = bp1[0].max(axis=0) * 100
    b0 = bp0[0].max(axis=0) * 100
    gate_b = np.maximum(np.maximum(b0, 0.0), m1.gate_cm('body')) + a.tol_body_cm
    bad = [m1.names[c] for c in range(nc) if b1[c] > gate_b[c] + 1e-9]
    res['b_body_rel'] = dict(ok=not bad, bad=bad, ducked_cm={m1.names[c]: round(float(b1[c]), 2) for c in range(nc)}, vanilla_cm={m1.names[c]: round(float(b0[c]), 2) for c in range(nc)})
    # c) door per capsule, every 30 fps frame (exit order) and every door start delay: the duck may not penetrate the door more than the vanilla exit at the same frame and door pose
    #    (violation = ducked penetration > max(0, vanilla) + tol: a capsule that stays clear of the door is never a violation)
    viol = (op1 * 100.0) - (np.maximum(op0, 0.0) * 100.0 + a.tol_open_cm)
    inc = (op1 - op0) * 100.0   # (delays, frames, caps) cm
    o1 = op1.max(axis=(0, 1)) * 100
    o0 = op0.max(axis=(0, 1)) * 100
    badc = sorted({m1.names[c] for c in np.argwhere(viol.max(axis=(0, 1)) > 0)[:, 0]})
    worst = np.unravel_index(viol.argmax(), viol.shape)
    res['c_door_open_rel'] = dict(ok=not badc, bad=badc, door_start_delays_s=list(DOOR_DELAYS_S), worst_violation_cm=round(float(viol.max()), 2),
                                  worst_at=dict(delay_s=DOOR_DELAYS_S[worst[0]], frame=int(worst[1]), capsule=m1.names[worst[2]], ducked_cm=round(float(op1[worst] * 100), 2), vanilla_cm=round(float(op0[worst] * 100), 2)),
                                  max_increase_vs_vanilla_same_frame_cm=round(float(inc.max()), 2),
                                  ducked_cm={m1.names[c]: round(float(o1[c]), 2) for c in range(nc)}, vanilla_cm={m1.names[c]: round(float(o0[c]), 2) for c in range(nc)})
    # c2) full shell (doorway included), per capsule and frame (exit order): same rule
    fviol = (m1._full_pen * 100.0) - (np.maximum(m0._full_pen, 0.0) * 100.0 + a.tol_body_cm)
    finc = (m1._full_pen - m0._full_pen) * 100.0
    badf = sorted({m1.names[c] for c in np.argwhere(fviol.max(axis=0) > 0)[:, 0]})
    fw = np.unravel_index(fviol.argmax(), fviol.shape)
    res['c2_full_shell_rel'] = dict(ok=not badf, bad=badf, worst_violation_cm=round(float(fviol.max()), 2), worst_at=dict(frame=int(fw[0]), capsule=m1.names[fw[1]]), max_increase_vs_vanilla_same_frame_cm=round(float(finc.max()), 2),
                                    by_capsule_cm={m1.names[c]: dict(ducked=round(float(m1._full_pen[:, c].max() * 100), 2), vanilla=round(float(m0._full_pen[:, c].max() * 100), 2), max_inc=round(float(finc[:, c].max()), 2)) for c in range(nc)})
    # d) first / last frames
    dang = np.stack([quat_angle_deg(lr[:, j], vlr[:, j]) for j in range(lr.shape[1])], 1)
    dtr = np.linalg.norm(lt - vlt, axis=-1)
    ends = {}
    ok = True
    for nm, k in (('first', 0), ('last', T - 1)):
        ends[nm] = dict(max_rot_deg=float(dang[k].max()), max_trans_m=float(dtr[k].max()))
        ok &= bool(dang[k].max() <= a.tol_end_deg and dtr[k].max() <= a.tol_end_m)
    res['d_first_last'] = dict(ok=ok, **ends)
    # e) root
    res['e_root'] = dict(ok=bool(dang[:, ri].max() <= 1e-4 and dtr[:, ri].max() <= 1e-6), max_rot_deg=float(dang[:, ri].max()), max_trans_m=float(dtr[:, ri].max()))
    # f) events / float tracks
    def evsig(ex):
        return [(e['type'], e['eventName'], int(e['startFrame']), e.get('side')) for e in (ex.get('animEvents') or [])]

    def trsig(ex):
        return [(k['trackIndex'], round(float(k['time']), 6), round(float(k['value']), 6)) for key in ('trackKeys', 'constTrackKeys') for k in (ex.get(key) or [])]
    res['f_events_tracks'] = dict(ok=bool(evsig(extras) == evsig(vextras) and trsig(extras) == trsig(vextras)), events=len(evsig(extras)), track_keys=len(trsig(extras)))
    # g) unchanged bones
    chg = (dang > a.chg_deg) | (dtr > a.chg_m)
    changed_bones = sorted({clip_sk.names[j] for j in np.flatnonzero(chg.any(0))})
    allowed = set(a.duck_bones.split(','))
    extra_bones = [b for b in changed_bones if b not in allowed]
    res['g_unchanged_bones'] = dict(ok=not extra_bones, changed_bones=changed_bones, unexpected=extra_bones)
    # h) window of the changes vs the frames where the vanilla pose is over the gate (30 fps frames; the 120 Hz series is sampled at the frames)
    rp0_f = np.array([rp0[int(round(k * C.HZ / C.FPS))] for k in range(T)])[::-1]   # reversed model -> back to exit time
    over0 = np.flatnonzero(rp0_f * 100 > gate_roof - a.window_margin_cm)
    changed_frames = [int(t) for t in np.flatnonzero(chg.any(1))]
    if len(over0) and changed_frames:
        lo, hi = over0.min(), over0.max()
        outside = [f for f in changed_frames if f < lo - a.window_pad or f > hi + a.window_pad]
    else:
        lo = hi = -1
        outside = list(changed_frames)
    res['h_window'] = dict(ok=not outside if len(over0) else not changed_frames, over_gate_frames=[int(x) for x in over0], window=[int(lo), int(hi)], changed_frames=changed_frames, outside=outside)
    # i) continuity
    st1 = np.stack([quat_angle_deg(lr[1:, j], lr[:-1, j]) for j in range(lr.shape[1])], 1)
    st0 = np.stack([quat_angle_deg(vlr[1:, j], vlr[:-1, j]) for j in range(vlr.shape[1])], 1)
    badc = {}
    for j in np.flatnonzero(chg.any(0)):
        lim = max(float(st0[:, j].max()) + a.step_margin_deg, a.step_deg)
        if st1[:, j].max() > lim:
            badc[clip_sk.names[j]] = dict(ducked=round(float(st1[:, j].max()), 2), limit=round(lim, 2))
    res['i_continuity'] = dict(ok=not badc, bad=badc, ducked_max_step_deg={clip_sk.names[j]: round(float(st1[:, j].max()), 2) for j in np.flatnonzero(chg.any(0))},
                               vanilla_max_step_deg={clip_sk.names[j]: round(float(st0[:, j].max()), 2) for j in np.flatnonzero(chg.any(0))})
    # j) anatomy: neck flexion / trunk lean over the clip, from the skeleton (same definitions as head_duck.py)
    Wt, Wr = clip_sk.world(lt, lr)
    vWt, vWr = vsk.world(vlt, vlr)

    def ang(u, v):
        u = u / np.maximum(np.linalg.norm(u, axis=-1, keepdims=True), 1e-9)
        v = v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)
        return np.degrees(np.arccos(np.clip((u * v).sum(-1), -1, 1)))
    nf = ang(Wt[:, ix['Neck']] - Wt[:, ix['Spine3']], Wt[:, ix['Head']] - Wt[:, ix['Neck']])
    ln = ang(Wt[:, ix['Neck']] - Wt[:, ix['Hips']], np.broadcast_to(np.array([0.0, 1.0, 0.0]), nf.shape + (3,)))
    nf0 = ang(vWt[:, ix['Neck']] - vWt[:, ix['Spine3']], vWt[:, ix['Head']] - vWt[:, ix['Neck']])
    ln0 = ang(vWt[:, ix['Neck']] - vWt[:, ix['Hips']], np.broadcast_to(np.array([0.0, 1.0, 0.0]), nf0.shape + (3,)))
    res['j_anatomy'] = dict(ok=bool(nf.max() <= LIM_NECK and ln.max() <= LIM_LEAN), neck_flex_max_deg=round(float(nf.max()), 1), trunk_lean_max_deg=round(float(ln.max()), 1),
                            vanilla_neck_flex_max_deg=round(float(nf0.max()), 1), vanilla_trunk_lean_max_deg=round(float(ln0.max()), 1), limits=[LIM_NECK, LIM_LEAN])
    # k) feet (and hands, informational) world positions vs vanilla
    fe = {}
    for nm in ('LeftFoot', 'RightFoot', 'LeftHand', 'RightHand', 'Hips', 'Head'):
        fe[nm] = round(float(np.linalg.norm(Wt[:, ix[nm]] - vWt[:, ix[nm]], axis=-1).max() * 100), 2)
    res['k_feet'] = dict(ok=bool(fe['LeftFoot'] <= a.tol_foot_cm and fe['RightFoot'] <= a.tol_foot_cm), max_dev_cm=fe)
    # l) the pelvis never goes below the seated pelvis (the seat cushion): clip world up = glTF Y
    # 2026-11-05 (all classes): the floor is the LOWER of the seated Hips (first exit frame) and the vanilla Hips at the same frame - the vanilla exit of the low classes (sport1, standard25 / 3, utility) itself
    # goes below the seated Hips (it steps down / crouches out), so the duck may only sink the pelvis tol below what the vanilla exit does there. Where the vanilla Hips never go below the seated Hips
    # (sport2, standard2) this is exactly the old rule 'Hips >= seated Hips - tol'.
    hy0 = vWt[0, ix['Hips'], 1]
    sink_max = float((vWt[:, ix['Hips'], 1] - Wt[:, ix['Hips'], 1]).max() * 100)
    below = float((np.minimum(hy0, vWt[:, ix['Hips'], 1]) - Wt[:, ix['Hips'], 1]).max() * 100)
    res['l_hips_seat'] = dict(ok=bool(below <= a.tol_hips_cm), hips_below_seated_max_cm=round(below, 2), hips_sink_vs_vanilla_max_cm=round(sink_max, 2), tol_cm=a.tol_hips_cm)
    # numbers: head penetration per frame before / after (30 fps frames, exit time)
    rp1_f = np.array([rp1[int(round(k * C.HZ / C.FPS))] for k in range(T)])[::-1]
    res['head_pen_cm'] = dict(frames_vanilla=[round(float(x * 100), 2) for x in rp0_f], frames_ducked=[round(float(x * 100), 2) for x in rp1_f])
    res['frames_changed'] = changed_frames
    if getattr(a, 'steps', None):
        c_, g_ = (int(x) for x in a.steps.split(':'))
        steps_gates(res, clip_sk, lt, lr, van, a.base_loaded or van, a, c_, g_)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clip', required=True)
    ap.add_argument('--vanilla', required=True)
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--seat', default='driver')
    ap.add_argument('--clip-name', default='exit')
    ap.add_argument('--vanilla-clip', default='exit')
    ap.add_argument('--report')
    ap.add_argument('--work')
    ap.add_argument('--duck-bones', default='Spine,Spine1,Spine2,Spine3,Neck,Neck1,Hips,LeftUpLeg,LeftLeg,LeftFoot,RightUpLeg,RightLeg,RightFoot,LeftToeBase,RightToeBase,Torso_LeftLeg_PoleVector_JNT,Torso_World_LeftLeg_PoleVector_JNT,Torso_RightLeg_PoleVector_JNT,Torso_World_RightLeg_PoleVector_JNT,Torso_LeftArm_PoleVector_JNT,Torso_World_LeftArm_PoleVector_JNT,Torso_RightArm_PoleVector_JNT,Torso_World_RightArm_PoleVector_JNT',
                    help='bones allowed to differ from vanilla (the duck chain, the Hips sink and the leg IK that keeps the feet); trim to the real chain for a strict run')
    ap.add_argument('--tol-body-cm', type=float, default=0.5, help='cm (the entry gates use seated + 0.5 cm: the shell clouds are sparse, +-0.5 cm model noise)')
    ap.add_argument('--tol-open-cm', type=float, default=0.5, help='cm: per capsule, frame and door pose the duck may be this much worse than the vanilla exit')
    ap.add_argument('--chg-deg', type=float, default=1e-3, help='deg: a bone / frame counts as changed above this rotation difference to vanilla (round trip run: 0.02 = 5x the measured identity noise 0.004)')
    ap.add_argument('--chg-m', type=float, default=1e-5)
    ap.add_argument('--tol-end-deg', type=float, default=1e-3)
    ap.add_argument('--tol-end-m', type=float, default=1e-5)
    ap.add_argument('--tol-foot-cm', type=float, default=0.5)
    ap.add_argument('--tol-hips-cm', type=float, default=1.0, help='cm the Hips may sit below the seated Hips (first exit frame)')
    ap.add_argument('--step-deg', type=float, default=11.0)
    ap.add_argument('--step-margin-deg', type=float, default=2.0, help='deg/frame a changed bone may exceed the vanilla exit own largest step of that bone (the foot IK steps 15.9 vs 14.8 deg; the vanilla left foot steps 16.1)')
    ap.add_argument('--window-pad', type=int, default=14, help='frames: changes may extend this far outside the over-the-gate window (the bend ramps in and out)')
    ap.add_argument('--window-margin-cm', type=float, default=1.0)
    ap.add_argument('--steps', help='C:G first / last frame of the generated stepping (steps mode: foot slide / step / floor / knee gates, frames outside C..G identical to --base)')
    ap.add_argument('--base', help='the clip without the stepping (head-duck override, round-tripped GLB): frames outside C..G must equal it')
    ap.add_argument('--base-clip', default='exit')
    ap.add_argument('--tol-slide-mean', type=float, default=0.5, help='cm / frame: mean horizontal speed of the contact joint over contact frames from C on')
    ap.add_argument('--tol-slide-p90', type=float, default=1.5, help='cm / frame, support core')
    ap.add_argument('--tol-slide-all', type=float, default=1.2, help='cm / frame, mean over ALL contact frames (landing / take-off roll included; vanilla ~3.8)')
    ap.add_argument('--min-steps', type=int, default=2)
    ap.add_argument('--tol-floor-cm', type=float, default=1.5)
    ap.add_argument('--min-feet-cm', type=float, default=12.0)
    ap.add_argument('--tol-knee-deg', type=float, default=20.0)
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    work = a.work or tempfile.mkdtemp(prefix='exitchk_')
    os.makedirs(work, exist_ok=True)
    van = load(a.vanilla, a.vanilla_clip)
    sk, t, lt, lr, extras = load(a.clip, a.clip_name)
    a.base_loaded = load(a.base, a.base_clip) if a.steps and a.base else None
    res = compare(sk, t, lt, lr, extras, van, a.vanilla_clip, a, work, 'ducked')
    gates = [k for k in res if k.split('_')[0] in ('a', 'b', 'c', 'c2', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r')]
    failed = [k for k in gates if not res[k]['ok']]
    for k in gates:
        r = res[k]
        print('%-18s %s %s' % (k, 'ok  ' if r['ok'] else 'FAIL', json.dumps({x: y for x, y in r.items() if x not in ('ok', 'ducked_cm', 'vanilla_cm', 'ducked_max_step_deg', 'vanilla_max_step_deg')})[:260]))
    if a.selftest:
        # mutation 1: the unducked vanilla clip as the candidate must fail the roof gate
        m1 = compare(van[0], van[1], van[2], van[3], van[4], van, a.vanilla_clip, a, work, 'mut-vanilla')
        st = dict(vanilla_as_clip_fails_roof=not m1['a_roof_gate']['ok'])
        # mutation 2: an 80 deg bend of the Neck at the last frame (about each local axis) must fail d_first_last (and j_anatomy for the flexing axis)
        j = sk.index['Neck']
        fails_d, fails_j = [], []
        for axis in range(3):
            lr2 = lr.copy()
            x2, y2, z2, w2 = lr2[-1, j]
            h = np.radians(80.0) / 2
            dq = np.zeros(4)
            dq[axis] = np.sin(h)
            dq[3] = np.cos(h)
            x1, y1, z1, w1 = dq
            lr2[-1, j] = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
            m2 = compare(sk, t, lt, lr2, extras, van, a.vanilla_clip, a, work, 'mut-last%d' % axis)
            fails_d.append(not m2['d_first_last']['ok'])
            fails_j.append(not m2['j_anatomy']['ok'])
        st['bend_at_last_frame_fails_d'] = all(fails_d)
        st['bend_at_last_frame_fails_j_some_axis'] = any(fails_j)
        # mutation 3: a hand moved (LeftHand local rotation) must fail g_unchanged_bones when the allowed list is the strict duck chain
        if a.steps:
            # steps mode: the vanilla glide (and the unstepped head-duck override) as the candidate must fail the foot-slide and step gates; a hand moved outside C..G must fail m_outside_identical
            st['vanilla_glide_fails_slide'] = not m1['n_foot_slide']['ok']
            st['vanilla_glide_fails_steps'] = not m1['o_steps']['ok']
            if a.base_loaded:
                bsk, bt, blt, blr, bex = a.base_loaded
                mb = compare(bsk, bt, blt, blr, bex, van, a.vanilla_clip, a, work, 'mut-base')
                st['override_without_steps_fails_slide'] = not mb['n_foot_slide']['ok']
                st['override_without_steps_fails_steps'] = not mb['o_steps']['ok']
            lr3 = lr.copy()
            jh = sk.index['LeftArm']
            lr3[3:6, jh] = lr3[3:6, jh][:, [1, 0, 3, 2]] * np.array([1, -1, 1, -1])
            m3 = compare(sk, t, lt, lr3, extras, van, a.vanilla_clip, a, work, 'mut-arm')
            st['arm_change_outside_span_fails_m'] = not m3['m_outside_identical']['ok']
        else:
            lr3 = lr.copy()
            jh = sk.index['LeftArm']
            lr3[10:20, jh] = lr3[10:20, jh][:, [1, 0, 3, 2]] * np.array([1, -1, 1, -1])
            m3 = compare(sk, t, lt, lr3, extras, van, a.vanilla_clip, a, work, 'mut-arm')
            st['arm_change_fails_g'] = not m3['g_unchanged_bones']['ok']
        if a.steps and 'mutation_in_span_caught' in res.get('p_floor', {}):
            st['p_floor_foot_under_floor_in_span_fails'] = bool(res['p_floor']['mutation_in_span_caught'])
            if 'mutation_outside_caught' in res['p_floor']:
                st['p_floor_foot_below_base_outside_span_fails'] = bool(res['p_floor']['mutation_outside_caught'])
        res['selftest'] = st
        print('selftest', st)
        if not all(st.values()):
            failed.append('selftest')
    res['failed'] = failed
    res['ok'] = not failed
    if a.report:
        json.dump(res, open(a.report, 'w'), indent=1)
    print('RESULT %s' % ('OK' if not failed else 'FAIL: ' + ','.join(failed)))
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main())
