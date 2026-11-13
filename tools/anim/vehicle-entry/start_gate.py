"""Start-position gate (a), joint rule (b) and waiver audit (c) of a car entry clip (round 4 'startfwd').

  python -I start_gate.py --clip <utpp_car_p?a_<class>_<seat>.anims.glb> [--vanilla <p?a_.. glb> | auto] [--class C --seat S] [--json out.json] [--selftest]

(a) START POSITION. The engine puts the entity on the clip's frame-0 root (seat * inverse(root delta)) in a straight line within a fixed ~0.3 s and the
    clip plays from there. Vehicle frame (x right, y forward, z up). The clip start must not lie more than START_DY_LIMIT_M BEHIND the start of the game's own
    clip of the same seat and gender (y of the frame-0 root; the game's start is already at the B-pillar; where the game's start lies FORWARD of the aperture
    rear edge at hip height - utility only - the edge is the reference: ref y = min(game start y, edge y), r13e), and the slide between the two start spots must not
    exceed START_SLIDE_LIMIT_M (README 'Glide limit': a peak of (pi/2) d / T <= 3 m/s at T = 0.30 s). A row that cannot meet it carries a documented limit
    in ROW_LIMITS (per row, with the reason; printed in every report) - there is no silent exception.
(b) JOINT RULE. No hips / thigh / knee / spine / chest joint may be BEHIND the real door-aperture rear edge (the rear edge of the hole in the body shell at that
    joint's height, from the shell point cloud - not the swing arc of the door) and INBOARD of the car's side skin at the same (y, z). Evaluated at 120 Hz on the
    linearly interpolated joint tracks. The skin x is the outermost shell / glass point of the same side around (y, z) (search radius widened until points
    are found; with none found the side band maximum is used, never 'not inboard').
(c) WAIVER AUDIT. A flat waiver W of the body gate is no gate for a capsule whose radius is <= W; a waiver >= the largest capsule radius (incl. the class body
    margin), or a roof waiver >= the head sphere radius, gates nothing and FAILS. Open / opening door tolerances are audited the same way.
Mutations (--selftest): the whole body 0.3 m to the rear, the frame-0 root 0.3 m to the rear, a 0.7 m lateral start slide, a waiver equal to the largest radius -
every one must be rejected; the unmutated clip is judged normally.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as C  # noqa: E402

START_DY_LIMIT_M = 0.15       # frame-0 root at most this far behind the game's clip start (vehicle y)
START_SLIDE_LIMIT_M = 0.57    # README glide limit: slide from the game's start spot, m
RULE_JOINTS = ('Hips', 'LeftUpLeg', 'RightUpLeg', 'Spine', 'Spine1', 'Spine2', 'Spine3')  # pelvis, hips / thighs, spine / chest (knees are not part of the rule: the van's knee heights have no reliable aperture edge)
JOINT_RULE_SIZE_M = 0.05  # a joint counts when it is behind the aperture edge AND inboard of the skin by MORE than this on both counts. The game's own clips reach 0.047 (Turbo-R right thigh) / 0.041 (Bratsk), so 5 cm is their maximum rounded up
# Documented per-row exceptions (row_limits.json next to this file, written by gen_row_limits.py from the MEASURED values): every entry is tied to the sha256 of the round-tripped clip it was measured
# on (first 16 hex) - a rebuilt clip gets the general limits - and carries its reason. ROW_LIMITS: row -> (max slide m, max dy behind m, reason, sha16); REAR_LIMITS: row -> dict(sha, root, Hips, Spine, sink, reason).
ROW_LIMITS = {}
REAR_MARGIN_M = 0.03       # (e) rearward excursion of root / hips / spine may exceed the game's own clip by this much
REAR_JOINTS = ('Hips', 'Spine')  # + the root; pelvis = Hips-Spine capsule (Spine2 / chest are not part of the rule: the game's own chest leans back 0.11-0.17 m while sitting, the NPC-derived clips lean more)
REAR_MARGIN_BY = {'Spine': 0.05}  # trunk: 5 cm (lean differs between the game's clip and the NPC-derived source); root, Hips and the sink slide: REAR_MARGIN_M
# (e) documented exceptions, tied to the clip file: row -> dict(sha=<first 16 hex of the sha256 of the round-tripped GLB>, root=m, Hips=m, Spine=m, Spine2=m, sink=m, reason=str)
REAR_LIMITS = {}
_LIMITS_JSON = os.path.join(HERE, 'row_limits.json')
if os.path.exists(_LIMITS_JSON) and not os.environ.get('SG_NO_LIMITS'):
    for _row, _e in json.load(open(_LIMITS_JSON, encoding='utf-8')).items():
        if 'slide' in _e:
            ROW_LIMITS[_row] = (_e['slide'], _e['back'], _e['reason'], _e['sha'])
        if 'rear' in _e:
            REAR_LIMITS[_row] = dict(_e['rear'], sha=_e['sha'], reason=_e.get('rear_reason', _e['reason']))
APERTURE_DZ = tuple(round(0.05 * k, 2) for k in range(0, 19))  # heights above the seat bone probed for the aperture, m


APERTURE_MAX_GAP_M = 1.6  # a single front-door aperture; wider = the front and rear door openings merged (the B-pillar is not in the shell point cloud)


def aperture_rear(bodyd, side, arc_edge=None):
    """Rear edge y of the door aperture by height: ([z], [y_rear]) from the shell point cloud. A height band counts only if its largest y-gap of the
    outboard shell points contains the seat y (windows above the beltline give other gaps)."""
    j = bodyd['json']
    sh = np.asarray(j['parts']['shell']['points'], float)
    sgn = -1.0 if side == 'left' else 1.0
    seat = j['seat'][side]
    sidemax = float(np.abs(sh[sh[:, 0] * sgn > 0, 0]).max())
    zs, ys = [], []
    for dz in APERTURE_DZ:
        z = seat[2] + dz
        m = (sh[:, 0] * sgn > sidemax - 0.25) & (np.abs(sh[:, 2] - z) < 0.04)
        yy = np.sort(sh[m, 1])
        if len(yy) < 2:
            continue
        g = np.diff(yy)
        best = None
        for k in np.argsort(-g)[:6]:
            if 0.35 <= g[k] <= APERTURE_MAX_GAP_M and yy[k] <= seat[1] <= yy[k + 1]:
                best = (float(yy[k]), float(yy[k + 1]), float(g[k]))
                break
        if best:
            zs.append(z)
            ys.append(best[0])
    if not zs:  # no separate aperture in the shell (merged front / rear door openings): the closed door's rear edge from the arc table is the edge
        if arc_edge is None:
            raise RuntimeError('no door aperture found in the body shell (%s) and no arc edge' % side)
        return np.array([seat[2], seat[2] + 1.0]), np.array([arc_edge, arc_edge]), sidemax
    return np.array(zs), np.array(ys), sidemax


class Skin:
    """Outer side-skin x at (y, z) of one side from the shell + static glass."""

    def __init__(self, bodyd, side):
        j = bodyd['json']
        pts = np.concatenate([np.asarray(j['parts']['shell']['points'], float), np.asarray(j['parts']['glass']['points'], float).reshape(-1, 3)])
        sgn = -1.0 if side == 'left' else 1.0
        self.sgn = sgn
        self.pts = pts[pts[:, 0] * sgn > 0.3]
        self.maxx = float(np.abs(self.pts[:, 0]).max())

    def x(self, y, z):
        for r in (0.06, 0.12, 0.25):
            m = (np.abs(self.pts[:, 1] - y) < r) & (np.abs(self.pts[:, 2] - z) < r)
            if m.any():
                return float(np.abs(self.pts[m, 0]).max()), r
        return self.maxx, None


def arc_edge_y(model):
    ar = model.tj['arcs'][model.cls][model.side]
    return ar['hingeY'] + ar['length'] * math.sin(math.radians(ar['closedDirDeg'])) if ar.get('valid', True) else None


def joints_veh(model):
    """{joint: (T, 3)} of the rule joints in the vehicle frame (mutations of the model included)."""
    return {k: v @ C.GLTF_TO_VEH.T for k, v in model.J.items() if k in RULE_JOINTS}


def joint_rule(model, step=0.25):
    """(b): per joint the 120 Hz samples behind the aperture rear edge and inboard of the skin. Returns dict(ok, joints={name: {n, worst}}, edge info)."""
    bd = model.bodyd
    if bd is None:
        return dict(ok=True, skipped='no body data', joints={})
    side = model.side
    zs, ys, sidemax = aperture_rear(bd, side, arc_edge_y(model))
    skin = Skin(bd, side)
    sgn = skin.sgn
    out, ok = {}, True
    for name, P in joints_veh(model).items():
        T = len(P)
        n_s = int((T - 1) / step) + 1
        u = np.linspace(0, T - 1, n_s)
        i0 = np.minimum(u.astype(int), T - 2)
        f = (u - i0)[:, None]
        Q = P[i0] * (1 - f) + P[i0 + 1] * f
        bad = []
        for k, p in enumerate(Q):
            edge = float(np.interp(p[2], zs, ys))
            behind = edge - p[1]
            if behind <= JOINT_RULE_SIZE_M:
                continue
            sx, _ = skin.x(p[1], p[2])
            depth = sx - abs(p[0])  # > 0: inboard of the skin
            if sgn * p[0] > 0 and depth > JOINT_RULE_SIZE_M:
                bad.append((float(u[k]), float(p[1] - edge), float(depth)))
        out[name] = dict(n=len(bad), first_frame=(bad[0][0] if bad else None),
                         worst=(min(bad, key=lambda b: b[1]) if bad else None))
        ok &= not bad
    return dict(ok=bool(ok), joints=out, side_max=sidemax, aperture_rear_by_z=[[round(float(z), 3), round(float(y), 3)] for z, y in zip(zs, ys)])


def limit_of(key, sha):
    """ROW_LIMITS entry of a row, valid only for the clip file it was measured on (entry[3] = first 16 hex of its sha256; a rebuilt clip gets the general limits)."""
    lim = ROW_LIMITS.get(key)
    if lim and len(lim) > 3 and lim[3] and not (sha or '').upper().startswith(lim[3].upper()):
        return None
    return lim


def start_report(model, van_model, key=None, sha=None):
    """(a): frame-0 root vs the game's clip start (vehicle frame) and the aperture rear edge."""
    r0 = model.root_g[0] @ C.GLTF_TO_VEH.T
    v0 = van_model.root_g[0] @ C.GLTF_TO_VEH.T
    side = model.side
    zs, ys, _ = aperture_rear(model.bodyd, side, arc_edge_y(model)) if model.bodyd is not None else (np.array([0.0]), np.array([float('nan')]), 0.0)
    seat_z = model.bodyd['json']['seat'][side][2] if model.bodyd is not None else 0.0
    edge_hip = float(np.interp(seat_z + 0.15, zs, ys))
    dy_v = float(r0[1] - v0[1])
    dx = float(r0[0] - v0[0])
    slide = math.hypot(dx, dy_v)
    # r13e: the dy limit protects the B-pillar: the game's start is the reference because it lies AT or behind the aperture rear edge (hip height) on every class but one.
    # Where the game's start lies FORWARD of that edge (utility: +0.325 m driver / +0.12..0.15 m passenger, and the game's clip then slides its root 0.24-0.36 m to the rear),
    # the edge itself is the reference: ref y = min(game start y, aperture rear edge y). Identical to the game's start wherever that start is at or behind the edge.
    ref_y, ref_name = float(v0[1]), 'game start'
    if model.bodyd is not None and float(v0[1]) > edge_hip:
        ref_y, ref_name = edge_hip, 'aperture rear edge (game start %+.3f m forward of it)' % (float(v0[1]) - edge_hip)
    dy = float(r0[1] - ref_y)
    lim = limit_of(key, sha) if key else None
    max_slide = lim[0] if lim else START_SLIDE_LIMIT_M
    max_back = lim[1] if lim else START_DY_LIMIT_M
    ok = (dy >= -max_back - 1e-9) and (slide <= max_slide + 1e-9)
    return dict(ok=bool(ok), root0_veh=[round(float(v), 3) for v in r0], vanilla_root0_veh=[round(float(v), 3) for v in v0], dy_vs_vanilla_m=round(dy_v, 3),
                dy_vs_reference_m=round(dy, 3), reference=ref_name, reference_y=round(ref_y, 3),
                dx_vs_vanilla_m=round(dx, 3), slide_m=round(slide, 3), max_back_m=max_back, max_slide_m=max_slide,
                documented_limit=(lim[2] if lim else None), root0_y_vs_aperture_edge_m=round(float(r0[1] - edge_hip), 3),
                vanilla_root0_y_vs_aperture_edge_m=round(float(v0[1] - edge_hip), 3), aperture_edge_y_at_hip=round(edge_hip, 3))


def _y_z(model, name):
    P = (model.root_g if name == 'root' else model.J[name]) @ C.GLTF_TO_VEH.T
    return P[:, 1], P[:, 2]


def back_excursion(y):
    """Largest backward travel from the running forward-most position (car-frame y)."""
    return float(np.max(np.maximum.accumulate(y) - y))


def sink_slide(y, z):
    """Backward travel summed over the frames in which the body DESCENDS (z falls by more than 2 mm per frame): the lower body sliding to the rear while it sits down."""
    dy, dz = np.diff(y), np.diff(z)
    return float(np.sum(np.where((dz < -0.002) & (dy < 0), -dy, 0.0)))


def rearward_report(model, van_model, key=None, sha=None):
    """(e) REARWARD MOTION over the whole clip: back excursion of the root, Hips, Spine, Spine2 and the sink slide of the Hips vs the game's own clip of the row (+ REAR_MARGIN_M)."""
    rows, ok = {}, True
    lim = REAR_LIMITS.get(key)
    if lim and not (sha or '').upper().startswith(str(lim.get('sha', '')).upper()):
        lim = None
    for nm in ('root',) + REAR_JOINTS:
        y, z = _y_z(model, nm)
        yv, zv = _y_z(van_model, nm)
        e, ev = back_excursion(y), back_excursion(yv)
        cap = (lim[nm] if lim and nm in lim else ev + REAR_MARGIN_BY.get(nm, REAR_MARGIN_M))
        rows[nm] = dict(excursion_m=round(e, 3), game_m=round(ev, 3), limit_m=round(cap, 3), ok=bool(e <= cap + 1e-9), documented=bool(lim and nm in lim))
        ok &= rows[nm]['ok']
    y, z = _y_z(model, 'Hips')
    yv, zv = _y_z(van_model, 'Hips')
    s, sv = sink_slide(y, z), sink_slide(yv, zv)
    cap = lim['sink'] if lim and 'sink' in lim else sv + REAR_MARGIN_M
    rows['sink_slide_Hips'] = dict(excursion_m=round(s, 3), game_m=round(sv, 3), limit_m=round(cap, 3), ok=bool(s <= cap + 1e-9), documented=bool(lim and 'sink' in lim))
    ok &= rows['sink_slide_Hips']['ok']
    return dict(ok=bool(ok), rows=rows, documented_reason=(lim or {}).get('reason'))


CORE_CAPSULES = ('pelvis', 'hips_l', 'spine', 'chest', 'thigh_l', 'thigh_r')  # the trunk / thigh capsules: no waiver may ungate any of them


def waiver_audit(model, body_tol_cm=None, roof_tol_cm=None, open_tol_cm=None, opening_tol_cm=None, tol_cm=1.0):
    """(c): the waivers a row ships with (flat number = every capsule, dict = the named capsules). Fails when the waiver - or the EFFECTIVE gate max(waiver, seated baseline + 0.5 cm) of a capsule
    that carries a waiver - reaches the radius of a hips / pelvis / spine / chest / thigh capsule (a capsule cannot penetrate deeper than its radius: the gate is gone), when a roof waiver reaches
    the head sphere radius, or when an open / opening door waiver does. Returns dict(ok, items=[...])."""
    names, rads = list(model.names), model.rad_by_cap_cm()
    rmax = float(np.max(model.rad)) * 100.0  # largest capsule radius incl. the class body margin
    items = []
    if body_tol_cm is not None:
        base = model.static_series()[3] * 100 + 0.5  # seated baseline gate per capsule
        w = C.body_tol_vec(body_tol_cm, names, 0.0)
        applied = w > 0
        eff = np.maximum(w, base)
        bad_core = [n for n, e, r, ap in zip(names, eff, rads, applied) if ap and n in CORE_CAPSULES and e >= r - 1e-9]
        items.append(dict(phase='body', waiver_cm=body_tol_cm, largest_radius_cm=round(rmax, 2), ok=bool(not bad_core), core_capsules_ungated=bad_core,
                          ungated=[n for n, e, r, ap in zip(names, eff, rads, applied) if ap and e >= r - 1e-9]))
    if roof_tol_cm is not None:
        rr = (C.ROOF_HEAD_R_M + C.ROOF_HAIR_M) * 100.0
        items.append(dict(phase='roof', waiver_cm=roof_tol_cm, largest_radius_cm=round(rr, 2), ok=bool(roof_tol_cm < rr - 1e-9), ungated=['head'] if roof_tol_cm >= rr - 1e-9 else []))
    for ph, v, base_ in (('open', open_tol_cm, 2.0), ('opening', opening_tol_cm, tol_cm)):
        if v is not None and v > base_ + 1e-9:
            bad_core = [n for n, r in zip(names, rads) if n in CORE_CAPSULES and v >= r - 1e-9]
            items.append(dict(phase=ph, waiver_cm=v, largest_radius_cm=round(rmax, 2), ok=bool(not bad_core), core_capsules_ungated=bad_core,
                              ungated=[n for n, r in zip(names, rads) if v >= r - 1e-9]))
    return dict(ok=all(i['ok'] for i in items), items=items)


def evaluate_model(m, vm, key, body_tol_cm=None, roof_tol_cm=None, open_tol_cm=None, opening_tol_cm=None, sha=None):
    """Everything in one dict from loaded models (m = the clip, vm = the game's clip of the same seat and gender; no_rear / no_env models are enough)."""
    res = dict(row=key, sha16=(sha or '')[:16], start=start_report(m, vm, key, sha), rearward=rearward_report(m, vm, key, sha), joint_rule=joint_rule(m), vanilla_joint_rule=joint_rule(vm),
               waivers=waiver_audit(m, body_tol_cm, roof_tol_cm, open_tol_cm, opening_tol_cm))
    res['ok'] = bool(res['start']['ok'] and res['rearward']['ok'] and res['joint_rule']['ok'] and res['waivers']['ok'])
    return res


def file_sha(path):
    import hashlib
    return hashlib.sha256(open(path, 'rb').read()).hexdigest().upper()


def evaluate(clip, cls, seat, vanilla='auto', body_tol_cm=None, roof_tol_cm=None, open_tol_cm=None, opening_tol_cm=None, mut=None, exempt=True):
    """Loads the models and judges (used by the table tool, the CLI and the self-test; check_door_clearance.py passes its own models to evaluate_model)."""
    g, c, s = C.parse_name(clip)
    cls, seat = cls or c, seat or s
    if vanilla in (None, 'auto'):
        vanilla = os.path.join(os.path.dirname(os.path.abspath(C.DATA)), 'glb', 'p%sa_%s_%s.anims.glb' % (g, cls, seat))
    kw = dict(no_rear=True, no_env=True)
    kw.update(mut or {})
    m = C.Model(clip, cls, seat, **kw)
    vm = C.Model(vanilla, cls, seat, no_rear=True, no_env=True)
    return evaluate_model(m, vm, 'p%sa_%s_%s' % (g, cls, seat), body_tol_cm, roof_tol_cm, open_tol_cm, opening_tol_cm, file_sha(clip) if exempt else '')


def fmt(res):
    s = res['start']
    jr = res['joint_rule']
    bad = ', '.join('%s n=%d first f%.1f worst y-edge %+.2f in %.2f' % (k, v['n'], v['first_frame'], v['worst'][1], v['worst'][2]) for k, v in jr['joints'].items() if v['n'])
    ln = ['%s: START %s  root0 %s vs game start %s: dy %+.3f m vs %s (limit -%.2f), slide %.3f m (limit %.2f)%s; root0 y vs aperture edge %+.3f m (game start %+.3f)' % (
        res['row'], 'ok' if s['ok'] else 'FAIL', s['root0_veh'][:2], s['vanilla_root0_veh'][:2], s.get('dy_vs_reference_m', s['dy_vs_vanilla_m']), s.get('reference', 'game start'), s['max_back_m'], s['slide_m'], s['max_slide_m'],
        (' [documented: %s]' % s['documented_limit']) if s['documented_limit'] else '', s['root0_y_vs_aperture_edge_m'], s['vanilla_root0_y_vs_aperture_edge_m']),
          '    REARWARD %s: %s' % ('ok' if res['rearward']['ok'] else 'FAIL', ', '.join('%s %.3f/%.3f%s' % (k, v['excursion_m'], v['limit_m'], '' if v['ok'] else '!') for k, v in res['rearward']['rows'].items())),
          '    JOINT RULE %s%s' % ('ok' if jr['ok'] else 'FAIL', (': ' + bad) if bad else ''),
          '    WAIVERS %s%s' % ('ok' if res['waivers']['ok'] else 'FAIL', ''.join('; %s %s cm (largest radius %.1f; %d capsule(s) ungated, core %s) %s' % (i['phase'], (','.join('%s=%s' % kv for kv in i['waiver_cm'].items()) if isinstance(i['waiver_cm'], dict) else '%.1f' % i['waiver_cm']), i['largest_radius_cm'], len(i['ungated']), i.get('core_capsules_ungated', []), 'ok' if i['ok'] else 'NO GATE') for i in res['waivers']['items']))]
    return '\n'.join(ln)


def selftest(clip, cls, seat, vanilla='auto', body_tol_cm=None, roof_tol_cm=None):
    """Mutations that must be rejected; returns (ok, lines). A mutation counts only if the mutated verdict FAILS the gate it targets."""
    _g, _c, _s = C.parse_name(clip)
    cls, seat = cls or _c, seat or _s
    base = evaluate(clip, cls, seat, vanilla, body_tol_cm, roof_tol_cm)
    lines = ['selftest base: %s' % ('PASS' if base['ok'] else 'FAIL (gates judged normally)')]
    ok = True
    gen_ok = evaluate(clip, cls, seat, vanilla, body_tol_cm, roof_tol_cm, exempt=False)['start']['ok']
    if gen_ok:
        # r13: a clip that starts FORWARD of the game's start (dy > 0) needs a larger rear mutation: 0.3 m from dy +0.17 is dy -0.13, still inside the 0.15 m limit and the mutation would pass
        amp = 0.3 + max(0.0, base['start']['dy_vs_reference_m'])
        lab = '%.2f' % amp if amp > 0.3 + 1e-9 else '0.3'
        muts = (('frame-0 root %s m to the rear' % lab, dict(shift_veh=(0.0, -amp, 0.0))), ('whole body %s m to the rear' % lab, dict(shift_rear=amp)),
                ('start 0.7 m farther from the car side', dict(shift_veh=((-0.7 if seat == 'driver' else 0.7), 0.0, 0.0))))
    else:  # an exempt row (documented limits tied to this clip): the mutations must exceed the documented limits
        lines.append('selftest start gate: the unmutated clip is a documented exemption (ROW_LIMITS) - mutations judged against the documented limits')
        muts = (('frame-0 root 1.5 m to the rear', dict(shift_veh=(0.0, -1.5, 0.0))), ('start 0.7 m farther from the car side', dict(shift_veh=((-0.7 if seat == 'driver' else 0.7), 0.0, 0.0))))
    for name, mut in muts:
        r = evaluate(clip, cls, seat, vanilla, body_tol_cm, roof_tol_cm, mut=mut, exempt=not gen_ok)
        rej = not r['start']['ok']
        lines.append('selftest %-34s start gate: %s (dy %+.3f vs %s, slide %.3f)' % (name, 'REJECTED' if rej else 'NOT rejected', r['start']['dy_vs_reference_m'], r['start']['reference'], r['start']['slide_m']))
        ok &= rej
    # (e) rearward motion over the clip: a smooth whole-body mid-clip rearward bump (clip fraction 0.25..0.75) of the scale of the problem must be rejected; report the smallest size of the ladder
    first = None
    for amp in (0.10, 0.15, 0.20, 0.30):
        r = evaluate(clip, cls, seat, vanilla, body_tol_cm, roof_tol_cm, mut=dict(bump_rear=(amp, 0.25, 0.75)))
        if not r['rearward']['ok']:
            first = amp
            break
    lines.append('selftest %-34s rearward gate: %s' % ('mid-clip rearward bump 0.10..0.30 m', ('REJECTED at %.2f m (%s)' % (first, ','.join(k for k, v in r['rearward']['rows'].items() if not v['ok']))) if first else 'NOT rejected'))
    # the gate allows the game's own excursion + margin (the end sit-back of the game's clip is 0.05-0.14 m), so a mid-clip bump is only visible once it exceeds the row's headroom: the smallest
    # rejected size is reported with the headroom of the binding metric; the self-test requires rejection within 0.30 m
    rw0 = base['rearward']['rows']
    lines.append('selftest %-34s   headroom of the unmutated clip: %s' % ('', ', '.join('%s %.2f' % (k, v['limit_m'] - v['excursion_m']) for k, v in rw0.items())))
    ok &= first is not None and first <= 0.30 + 1e-9
    # joint rule: whole body to the rear (ladder, smallest size reported; the rule is a joint-centre rule with a 5 cm size, so it needs more than the capsule gate)
    first = None
    for amp in (0.10, 0.20, 0.30, 0.45, 0.60):
        r = evaluate(clip, cls, seat, vanilla, body_tol_cm, roof_tol_cm, mut=dict(shift_rear=amp))
        if not r['joint_rule']['ok']:
            first = amp
            break
    lines.append('selftest %-34s joint rule: %s' % ('whole body to the rear 0.10..0.60 m', ('REJECTED at %.2f m (%s)' % (first, ','.join('%s:%d' % (k, v['n']) for k, v in r['joint_rule']['joints'].items() if v['n']))) if first else 'NOT rejected'))
    ok &= first is not None and first <= 0.45 + 1e-9
    m = C.Model(clip, cls, seat, no_rear=True, no_env=True)
    rmax = float(np.max(m.rad)) * 100.0
    rc = [r for n, r in zip(m.names, m.rad_by_cap_cm()) if n in CORE_CAPSULES]
    rpel = m.rad_by_cap_cm()[m.names.index('pelvis')]
    for ph, kw in (('body flat = largest radius', dict(body_tol_cm=rmax)), ('body flat = smallest trunk/thigh radius', dict(body_tol_cm=min(rc))), ('body flat = largest radius - 0.5', dict(body_tol_cm=rmax - 0.5)),
                   ('roof = sphere radius', dict(roof_tol_cm=(C.ROOF_HEAD_R_M + C.ROOF_HAIR_M) * 100.0)), ('open = largest radius', dict(open_tol_cm=rmax)), ('opening = largest radius', dict(opening_tol_cm=rmax)),
                   ('body named pelvis = its radius', dict(body_tol_cm={'pelvis': rpel}))):
        w = waiver_audit(m, **kw)
        rej = not w['ok']
        lines.append('selftest waiver %-32s: %s' % (ph, 'REJECTED' if rej else 'NOT rejected'))
        ok &= rej
    for ph, kw in (('body head=11.5 only', dict(body_tol_cm={'head': 11.5})), ('body uarm_r=6.9 only', dict(body_tol_cm={'uarm_r': 6.9})), ('open 6.0 (door waiver)', dict(open_tol_cm=6.0))):
        w = waiver_audit(m, **kw)
        lines.append('selftest waiver %-32s: %s (must be ACCEPTED: it ungates no trunk / thigh capsule)' % (ph, 'accepted' if w['ok'] else 'rejected'))
        ok &= w['ok']
    return bool(ok), lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clip', required=True, nargs='+')
    ap.add_argument('--class', dest='cls')
    ap.add_argument('--seat')
    ap.add_argument('--vanilla', default='auto')
    ap.add_argument('--body-tol-cm', type=C.parse_tol, default=None, help="body waiver: a flat number (every capsule) or 'capsule=cm,capsule=cm' (per capsule, like check_door_clearance.py)")
    ap.add_argument('--roof-tol-cm', type=float, default=None)
    ap.add_argument('--json')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    allres, rc = [], 0
    for clip in a.clip:
        r = evaluate(clip, a.cls, a.seat, a.vanilla, a.body_tol_cm, a.roof_tol_cm)
        print(fmt(r))
        allres.append(r)
        rc |= 0 if r['ok'] else 1
        if a.selftest:
            ok, lines = selftest(clip, a.cls, a.seat, a.vanilla, a.body_tol_cm, a.roof_tol_cm)
            print('\n'.join(lines))
            rc |= 0 if ok else 2
    if a.json:
        json.dump(allres, open(a.json, 'w'), indent=1)
    sys.exit(rc)


if __name__ == '__main__':
    main()
