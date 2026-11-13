"""Third-person ONE-HANDED BLUNT (p?a_one_handed_blunt) and BATON (p?a_baton) clips of V from the game's NPC melee animations (npc1), 2026-11-09.

  python -I npc_blunt_build.py --weapon blunt|baton [--rig woman|man] [--only clip,clip] [--lunge-start 0] [--out DIR]
Output (OUT = Build\\work\\melee-npc-blunt-20261109 unless --out): clips\\<set>__<clip>.npc1.npz (+ .info.json, .twohand.npz), gate failures in
clips_rejected\\, chain-reverted clips (passed their own gates, but a clip of their chain did not) in clips_chain\\, build_report_<set>.json.

Method = the knife / katana builder npc_blade_build.py (imported UNCHANGED; its functions are used, a few module globals are set in this
process only), which is the fists method of npc_build.py:
  * donors: wa_ / ma_gang_onehanded_action_combat (the NPC set of one-handed clubs, axes, machetes; exact rigs), NPC locals copied by name,
    NPC WeaponRight translation and fingers (exact NPC grip); time map donor swing peak -> vanilla strike frame; guard = NPC idle_combat f0;
  * one-handed weapon: the left hand may still hold the weapon in some NPC clips (bar block: idle_block / block_left / block_right hold the
    shaft ~25 cm in front of the fist; a few attack frames).  Two-hand detection of npc_blade_common.left_rel is EXTENDED here to the shaft
    side (palm < 7.5 cm from the weapon axis, 6 cm .. weapon tip in front of WeaponRight) and the katana hilt IK of npc_blade_build (TWO) is on,
    so on those frames the left hand follows the shaft; FIST_GAP keeps its one-hand value (no katana override);
  * the katana exemption of the hand-cross gate on two-hand frames is NOT used (HAND_CROSS_2H = ()): every frame is gated strictly;
  * weapon proxy vs body (npc_blade_common.blade_pen): the right hand always excluded, the left hand on two-hand frames and on the 2 frames
    next to them while its IK weight is above 0 (fingers leaving / reaching the shaft; blade_pen_release);
  * per clip: donor candidates in order (spec 'donors' + 'alts' + 'fallback'; reactions: the side from the swing, then the other sides,
    each at the donor amplitude 1.0 / 0.85 / 0.7 / 0.55 / 0.4 - deviation from the donor's frame 0 scaled, strongest first, as the fists
    builder); the first candidate that passes every gate is kept;
  * chain closure (no npc1 <-> game pop inside a chain): attack + _hit + _deflect; heavy + its hold clips + _hit + _deflect; the block group
    (block_start / block / block_end / block_attack(+_hit/_deflect) / block_deflect_*); the two idle <-> safe transitions.  A chain
    ships only if EVERY member passed; otherwise its passing members move to clips_chain\\ (they stay the game's clips);
  * new kind 'pose_blend' (melee_idle_to_safe / melee_safe_to_idle, equip / unequip fallback): built pose A -> built pose B, smoothstep,
    feet planted (same guard stance in both poses), anatomy fixes and gates as every clip.
Weapon proxy and vanilla baton dumps: npc_blunt_common.py.
"""
import json
import os
import shutil
import sys
import time

import numpy as np


def opt(a, k, d=None):
    return a[a.index(k) + 1] if k in a else d


WEAPON = opt(sys.argv, '--weapon', 'blunt')
assert WEAPON in ('blunt', 'baton'), WEAPON
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import npc_blunt_common as UC  # noqa: E402  (registers the weapons; imports npc_build -> base_body, which consumes --rig)

BC, NB, MC = UC.BC, UC.NB, UC.MC
TIP = BC.BLADE[WEAPON][1]
_left_rel0 = BC.left_rel


def _left_rel(raw):
    rel, on, p = _left_rel0(raw)
    radial = np.linalg.norm(p[:, :2], axis=1)
    on = on | ((radial < 0.075) & (p[:, 2] > 0.06) & (p[:, 2] < TIP))
    return rel, on, p


BC.left_rel = _left_rel
import npc_blade_build as BB  # noqa: E402  (reads --weapon; SPEC there is replaced below)

BB.TWO = True                 # left hand on the shaft where the NPC holds it (hilt IK of the katana path); FIST_GAP unchanged
BB.HAND_CROSS_2H = ()         # no katana hand-cross exemption
BB.OUT = UC.OUTW
SET, A, RIG, P, IX = BB.SET, BB.ACT, BB.RIG, BB.P, BB.IX
SUFFIX = BB.SUFFIX


def d(*clips):
    return [(A, c) for c in clips]


DEFLECT = {'to_right': (A, 'deflect_rightarm_to_right'), 'to_left': (A, 'deflect_rightarm_to_left'),
           'to_down': (A, 'deflect_rightarm_to_down'), 'to_up': (A, 'deflect_rightarm_to_up')}
COMMON = {
    'heavy_attack_01': dict(kind='heavy', donors=d('melee_heavy_01_idle_to_attack', 'melee_heavy_01_miss_to_idle'),
                            alts=[d('melee_heavy_01_idle_to_attack', 'melee_heavy_01_hit_to_idle'),
                                  dict(donors=d('melee_jump_attack_01_jump_to_attack', 'melee_jump_attack_01_hit_to_idle'), calm_hips=True),
                                  d('melee_heavy_01_dash', 'melee_heavy_01_miss_to_idle'), d('melee_combo_infinite_attack_03')],
                            note='overhead chop from the hold pose'),
    'heavy_attack_02': dict(kind='heavy', donors=d('melee_heavy_02_idle_to_attack', 'melee_heavy_02_miss_to_idle'),
                            alts=[d('melee_heavy_02_idle_to_attack', 'melee_heavy_02_hit_to_idle'), d('melee_combo_01_p1_from_idle', 'melee_combo_01_p1_miss'),
                                  d('melee_heavy_02_dash', 'melee_heavy_02_miss_to_idle'), d('melee_combo_infinite_startup_02'),
                                  dict(donors=d('melee_jump_attack_01_jump_to_attack', 'melee_jump_attack_01_hit_to_idle'), calm_hips=True),
                                  d('melee_heavy_01_dash', 'melee_heavy_01_miss_to_idle'), d('melee_combo_infinite_attack_03'),
                                  d('melee_heavy_01_idle_to_attack', 'melee_heavy_01_hit_to_idle'),
                                  d('melee_heavy_01_idle_to_attack', 'melee_heavy_01_miss_to_idle')],
                            note='rising backhand power swing from the hold pose (fallbacks: the heavy_01 donors)'),
    'safe_attack': dict(kind='attack', donors=d('melee_combo_01_p1_from_idle', 'melee_combo_01_p1_miss'),
                        alts=[d('melee_combo_01_p1', 'melee_combo_01_p1_miss'), d('melee_combo_close_01_p2', 'melee_combo_close_01_p2_miss')],
                        note='backhand swing out of the lowered weapon'),
    'sprint_attack': dict(kind='attack', donors=d('melee_charge_01_sprint_to_attack', 'melee_charge_01_hit_to_idle'), lunge=True,
                          alts=[d('melee_heavy_01_sprint_to_attack', 'melee_heavy_01_hit_to_idle'),
                                d('melee_heavy_02_sprint_to_attack', 'melee_heavy_02_hit_to_idle')], note='charge swing out of the sprint, lunge'),
    'jump_attack': dict(kind='attack', donors=d('melee_jump_attack_01_jump_to_attack', 'melee_jump_attack_01_hit_to_idle'), calm_hips=True,
                        note='downward chop (airborne legs: DLL; donor jump not in the hips)'),
    'crouch_attack': dict(kind='attack', donors=d('melee_combo_infinite_startup_02'), alts=[d('melee_combo_infinite_attack_02')],
                          note='rising swing (standing guard; crouch: DLL)'),
    'block_attack': dict(kind='attack', donors=d('parry_front'), start='block',
                         alts=[d('melee_combo_close_01_p2', 'melee_combo_close_01_p2_miss'), d('melee_combo_01_p1', 'melee_combo_01_p1_miss')],
                         note='riposte out of the block'),
    'block_start': dict(kind='block_start', donors=d('idle_block'), fallback=dict(kind='pose_blend', a='melee_idlepose', b='block',
                                                                                  note='fallback: guard -> block pose')),
    'block': dict(kind='block', donors=d('idle_block')),
    'block_end': dict(kind='block_end', donors=d('idle_block'), fallback=dict(kind='pose_blend', a='block', b='melee_idlepose',
                                                                              note='fallback: block pose -> guard')),
    'block_deflect_left': dict(kind='react', start='block', donors=d('block_left'), alts=[d('block_right')], note='NPC block hit reaction, back to the guard'),
    'block_deflect_right': dict(kind='react', start='block', donors=d('block_right'), alts=[d('block_left')], note='NPC block hit reaction, back to the guard'),
}
if WEAPON == 'blunt':
    SPEC = {
        'combo_attack_01': dict(kind='attack', donors=d('melee_combo_01_p2', 'melee_combo_01_p2_miss'),
                                alts=[d('melee_combo_infinite_attack_01'), d('melee_combo_02_p1_from_idle', 'melee_combo_02_p1_miss')],
                                note='descending forehand swing'),
        'combo_attack_02': dict(kind='attack', donors=d('melee_combo_01_p1', 'melee_combo_01_p1_miss'),
                                alts=[d('melee_combo_close_01_p2', 'melee_combo_close_01_p2_miss'), d('melee_combo_01_p1_from_idle', 'melee_combo_01_p1_miss')],
                                note='rising backhand swing'),
        'combo_attack_03': dict(kind='attack', donors=d('melee_combo_02_p1', 'melee_combo_02_p1_miss'),
                                alts=[d('melee_combo_infinite_attack_01'), d('melee_combo_infinite_attack_03')], note='forehand swing'),
        'final_attack_03': dict(kind='attack', donors=d('melee_combo_infinite_attack_01'), lunge=True,
                                alts=[d('melee_combo_02_p1_from_idle', 'melee_combo_02_p1_miss'), d('melee_combo_infinite_attack_03')],
                                note='horizontal forehand power swing, lunge'),
    }
    BASES = ['combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'final_attack_03', 'heavy_attack_01', 'heavy_attack_02', 'safe_attack',
             'sprint_attack', 'jump_attack', 'crouch_attack', 'block_attack']
else:
    SPEC = {
        'combo_attack_01': dict(kind='attack', donors=d('melee_combo_infinite_attack_01'),
                                alts=[d('melee_combo_01_p2', 'melee_combo_01_p2_miss'), d('melee_combo_02_p1_from_idle', 'melee_combo_02_p1_miss')],
                                note='horizontal forehand swing'),
        'combo_attack_02': dict(kind='attack', donors=d('melee_combo_close_01_p2', 'melee_combo_close_01_p2_miss'),
                                alts=[d('melee_combo_01_p1', 'melee_combo_01_p1_miss'), d('melee_combo_02_p2', 'melee_combo_02_p2_miss')],
                                note='backhand swing'),
        'combo_attack_03': dict(kind='attack', donors=d('melee_combo_01_p2', 'melee_combo_01_p2_miss'),
                                alts=[d('melee_combo_infinite_attack_03'), d('melee_combo_02_p1', 'melee_combo_02_p1_miss')],
                                note='descending forehand swing'),
        'combo_attack_04': dict(kind='attack', donors=d('melee_combo_01_p1', 'melee_combo_01_p1_miss'),
                                alts=[d('melee_combo_02_p2', 'melee_combo_02_p2_miss'), d('melee_combo_close_01_p2', 'melee_combo_close_01_p2_miss')],
                                note='backhand swing'),
        'final_attack_03': dict(kind='attack', donors=d('melee_combo_02_p1_from_idle', 'melee_combo_02_p1_miss'), lunge=True,
                                alts=[d('melee_combo_infinite_attack_03'), d('melee_heavy_01_sprint_to_attack', 'melee_heavy_01_hit_to_idle')],
                                note='forehand power swing, lunge'),
        'final_attack_04': dict(kind='attack', donors=d('melee_combo_02_p2', 'melee_combo_02_p2_miss'), lunge=True,
                                alts=[d('melee_combo_01_p1_from_idle', 'melee_combo_01_p1_miss'), d('melee_combo_close_01_p2', 'melee_combo_close_01_p2_miss'),
                                      d('melee_combo_infinite_attack_02'), d('melee_combo_close_01_p1', 'melee_combo_close_01_p1_miss'),
                                      d('melee_combo_infinite_attack_01'), d('melee_combo_01_p2', 'melee_combo_01_p2_miss')],
                                note='backhand power swing, lunge (Male: the first three failed, m1)'),
        'final_attack_05': dict(kind='attack', donors=d('melee_combo_infinite_attack_03'), lunge=True,
                                alts=[d('melee_combo_close_01_p1', 'melee_combo_close_01_p1_miss'), d('melee_combo_infinite_attack_01')],
                                note='downward forward power swing, lunge'),
    }
    BASES = ['combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'combo_attack_04', 'final_attack_03', 'final_attack_04', 'final_attack_05',
             'heavy_attack_01', 'heavy_attack_02', 'safe_attack', 'sprint_attack', 'jump_attack', 'crouch_attack', 'block_attack']
SPEC.update(COMMON)
SPEC.update({
    'melee_idlepose': dict(kind='pose'),
    'melee_safepose': dict(kind='safepose', donors=d('ovr_idle')),
    'guard_idle_loop': dict(kind='idle_loop', donors=d('idle_combat'), frames=120),
    'melee_idle_to_safe': dict(kind='pose_blend', a='melee_idlepose', b='melee_safepose', note='guard -> weapon lowered'),
    'melee_safe_to_idle': dict(kind='pose_blend', a='melee_safepose', b='melee_idlepose', note='weapon lowered -> guard'),
    'hold_start_01': dict(kind='hold_start', of='heavy_attack_01'),
    'hold_loop_01': dict(kind='hold_loop', of='heavy_attack_01'),
    'hold_loop_02': dict(kind='hold_loop', of='heavy_attack_02'),
    'hold_start_offscreen_01': dict(kind='hold_start', of='heavy_attack_01', start='safepose'),
    'hold_start_offscreen_02': dict(kind='hold_start', of='heavy_attack_02', start='safepose'),
    'melee_equip': dict(kind='equip', donors=d('ovr_equip'), fallback=dict(kind='pose_blend', a='melee_safepose', b='melee_idlepose',
                                                                           note='fallback: weapon raised from the lowered pose to the guard')),
    'melee_unequip': dict(kind='unequip', donors=d('ovr_unequip'), fallback=dict(kind='pose_blend', a='melee_idlepose', b='melee_safepose',
                                                                                 note='fallback: weapon lowered from the guard')),
    'parried_left': dict(kind='react', start='auto', donors=[DEFLECT['to_left']], alts=[[DEFLECT['to_down']], [DEFLECT['to_right']]],
                         note='weapon arm knocked away'),
    'parried_right': dict(kind='react', start='auto', donors=[DEFLECT['to_right']], alts=[[DEFLECT['to_up']], [DEFLECT['to_left']]],
                          note='weapon arm knocked away'),
})
for _b in BASES:
    SPEC[_b + '_hit'] = dict(kind='hit', of=_b)
    SPEC[_b + '_deflect'] = dict(kind='react', start=(_b, None), donors='deflect', note='weapon arm deflected')
ATTACKS = [b for b in BASES if b != 'block_attack']
ORDER = (['guard_idle_loop', 'melee_idlepose', 'melee_safepose', 'melee_idle_to_safe', 'melee_safe_to_idle', 'block', 'block_start', 'block_end']
         + ATTACKS + ['block_attack', 'hold_start_01', 'hold_loop_01', 'hold_loop_02', 'hold_start_offscreen_01', 'hold_start_offscreen_02',
                      'melee_equip', 'melee_unequip', 'parried_left', 'parried_right', 'block_deflect_left', 'block_deflect_right']
         + [b + v for b in BASES for v in ('_hit', '_deflect')])
LOOPS = ('hold_loop_01', 'hold_loop_02', 'guard_idle_loop', 'block')
CHAINS = ([[b, b + '_hit', b + '_deflect'] for b in ATTACKS if not b.startswith('heavy')]
          + [['heavy_attack_01', 'heavy_attack_01_hit', 'heavy_attack_01_deflect', 'hold_start_01', 'hold_loop_01', 'hold_start_offscreen_01'],
             ['heavy_attack_02', 'heavy_attack_02_hit', 'heavy_attack_02_deflect', 'hold_loop_02', 'hold_start_offscreen_02'],
             ['block_start', 'block', 'block_end', 'block_attack', 'block_attack_hit', 'block_attack_deflect', 'block_deflect_left', 'block_deflect_right'],
             ['melee_idle_to_safe', 'melee_safe_to_idle']])
for _k in ('SPEC', 'BASES', 'ORDER', 'LOOPS', 'DEFLECT'):
    setattr(BB, _k, globals()[_k])
_vidx = BC.vanilla_index(SET)
ORDER = [c for c in ORDER if c == 'guard_idle_loop' or c in _vidx]       # clip names this set has
BB.ORDER = ORDER


def auto_start(name):
    """parried_*: start at the branch frame of the closest BUILT combo attack (npc_blade_build.auto_start assumes combo_attack_01..04)."""
    best = None
    for b in [x for x in BASES if x.startswith('combo_attack')]:
        if not os.path.exists(os.path.join(BB.CDIR, '%s__%s.%s.npz' % (SET, b, SUFFIX))):
            continue
        f, dcm = BB.branch_frame(name, b)
        if best is None or dcm < best[2]:
            best = (b, f, dcm)
    if best is None:
        raise FileNotFoundError('no built combo attack for ' + name)
    return best


BB.auto_start = auto_start
_other0 = BB.other_clip


def other_clip(G, name, spec, lunge_start):
    if spec['kind'] != 'pose_blend':
        return _other0(G, name, spec, lunge_start)
    T = len(BC.vanilla(SET, name)[0]['W'])
    a, b = BB.built_clip(spec['a']), BB.built_clip(spec['b'])
    i = np.arange(T, dtype=float)
    w = NB.sstep(i / (T - 1))
    c = np.zeros((T, 3))
    tracks = NB.static_tracks(G, T, c)
    lt = a['lt'][0][None] * (1 - w)[:, None, None] + b['lt'][0][None] * w[:, None, None]
    J = a['lr'].shape[1]
    lr = NB.qslerp(np.repeat(a['lr'][:1], T, 0), np.repeat(b['lr'][:1], T, 0), np.repeat(w[:, None], J, 1))
    hips = a['W'][0, IX['Hips'], :3, 3][None] * (1 - w)[:, None] + b['W'][0, IX['Hips'], :3, 3][None] * w[:, None]
    log = {}
    lt, lr = NB.finish(G, lt, lr, hips, tracks, c, log)
    relM = BB.rel_blend(np.repeat(a['rel'][:1], T, 0), np.repeat(b['rel'][:1], T, 0), w)
    w2h = float(a['w'][0]) * (1 - w) + float(b['w'][0]) * w
    BB.post_fix(lt, lr, relM, w2h, BB.lock_w(T), log)
    lt[0], lr[0], lt[-1], lr[-1] = a['lt'][0], a['lr'][0], b['lt'][0], b['lr'][0]
    info = dict(kind='pose_blend', source='%s frame 0 -> %s frame 0, smoothstep over %d frames, feet planted (guard stance)' % (spec['a'], spec['b'], T),
                note=spec.get('note'), fixes=log)
    return lt, NB.qcont(lr), c, info, relM, np.asarray(w2h, float)


BB.other_clip = other_clip


def grip_metric(W):
    """palm centre in the WeaponRight frame (the weapon hangs from WeaponRight) vs the vanilla idle grip of the set: max deviation (cm)."""
    k, p = UC.knuckle_in_wr(W)
    dv = MC.world_all(BC.vanilla(SET, 'melee_idlepose')[0])
    _, p0 = UC.knuckle_in_wr(dv[:1])
    dev = np.linalg.norm(p - p0[0][None], axis=1) * 100
    rad = np.linalg.norm(p[:, :2], axis=1) * 100            # palm centre to the weapon axis (handle line), cm
    rad0 = float(np.linalg.norm(p0[0, :2]) * 100)
    return dict(palm_in_weapon_dev_max_cm=round(float(dev.max()), 2), palm_to_axis_cm=[round(float(rad.min()), 2), round(float(rad.max()), 2)],
                palm_to_axis_vanilla_idle_cm=round(rad0, 2), palm_to_axis_dev_max_cm=round(float(np.abs(rad - rad0).max()), 2),
                palm_along_axis_cm=[round(float(p[:, 2].min() * 100), 2), round(float(p[:, 2].max() * 100), 2)],
                knuckle_axis_z_min=round(float(k[:, 2].min()), 3))


AMPS = (1.0, 0.85, 0.7, 0.55, 0.4)       # reaction donors: deviation from the donor's frame 0 scaled (as the fists builder), strongest first
AMP = [1.0]
_BSeq0 = BC.BSeq


class BSeqAmp(_BSeq0):
    """npc_blade_common.BSeq with the donor's deviation from its frame 0 scaled by AMP[0] (locals, root path); AMP 1 = unchanged."""

    def __init__(self, donors, despike=True):
        super().__init__(donors, despike)
        a = AMP[0]
        if a < 1.0:
            J = self.lr.shape[1]
            self.lr = NB.qcont(NB.qslerp(np.repeat(self.lr[:1], self.n, 0), self.lr, np.full((self.n, J), a)))
            self.lt = self.lt[:1] + (self.lt - self.lt[:1]) * a
            self.root_t = self.root_t * a
            self.W = NB.fk(self.lt, self.lr)


BC.BSeq = BSeqAmp

# weapon proxy vs body: npc_blunt_common.blade_pen_release (installed as BC.blade_pen on import)


def candidates(name, spec):
    if spec['kind'] == 'react':
        if spec.get('donors') == 'deflect':
            first = BB.resolve_donors(name, spec)[0]
            opts = [[first]] + [[v] for k, v in DEFLECT.items() if v != first]
        else:
            opts = [spec['donors']] + list(spec.get('alts', []))
        return [dict(spec, donors=o, amp=a_) for a_ in AMPS for o in opts]
    out = [spec]
    for alt in spec.get('alts', []):
        out.append(dict(spec, **alt) if isinstance(alt, dict) else dict(spec, donors=alt))
    if spec.get('fallback'):
        out.append(dict(spec['fallback']))
    return out


def build_one(G, name, spec, lunge_start):
    if spec['kind'] in ('hold_start', 'hold_loop'):
        BB.built_clip(spec['of'])
    if spec['kind'] in ('attack', 'heavy'):
        return BB.attack_clip(G, name, spec, lunge_start)
    if spec['kind'] == 'hit':
        return BB.hit_clip(G, name, spec)
    if spec['kind'] == 'react':
        AMP[0] = float(spec.get('amp', 1.0))
        try:
            out = BB.react_clip(G, name, spec)
        finally:
            AMP[0] = 1.0
        out[3]['donor_amplitude'] = float(spec.get('amp', 1.0))
        return out
    return other_clip(G, name, spec, lunge_start)


def score(g):
    return (bool(g['PASS']), g['base_gate_pass'] - g['frames'], -g['pen_frames'], -g['blade_pen_frames'])


def move(stem, src, dst):
    os.makedirs(dst, exist_ok=True)
    for ext in ('.npz', '.twohand.npz', '.info.json'):
        s = os.path.join(src, stem + ext)
        if os.path.exists(s):
            shutil.move(s, os.path.join(dst, stem + ext))


def main(a):
    t0 = time.time()
    out_dir = opt(a, '--out', UC.OUTW)
    cdir = os.path.join(out_dir, 'clips')
    rdir = os.path.join(out_dir, 'clips_rejected')
    hdir = os.path.join(out_dir, 'clips_chain')
    os.makedirs(cdir, exist_ok=True)
    BB.CDIR = cdir
    for f in (os.listdir(hdir) if os.path.isdir(hdir) else []):       # restore chain-reverted clips: chains are re-evaluated at the end
        if f.startswith(SET + '__'):
            shutil.move(os.path.join(hdir, f), os.path.join(cdir, f))
    only = opt(a, '--only').split(',') if '--only' in a else None
    lunge_start = float(opt(a, '--lunge-start', 0.0))
    G = BB.BGuard()
    print('GUARD', WEAPON, RIG, SET, 'fixes', G.log, flush=True)
    for h in ('heavy_attack_01', 'heavy_attack_02'):
        BB.HOLDS[h] = dict(lr=BB.hold_lr(G, 1.0), lr2=BB.hold_lr(G, 1.0 + NB.HOLD_PULSE), side='Right')
    rep_path = os.path.join(out_dir, 'build_report_%s.json' % SET)
    rep = json.load(open(rep_path)) if os.path.exists(rep_path) and only else {}
    for name in ORDER:
        if only and name not in only:
            continue
        spec = SPEC[name]
        t1 = time.time()
        best, tried = None, []
        stem = '%s__%s.%s' % (SET, name, SUFFIX)
        try:
            cands = candidates(name, spec)
        except FileNotFoundError as e:
            cands, err = [], e
        for k, sp in enumerate(cands):
            try:
                lt, lr, c, info, relM, w2h = build_one(G, name, sp, lunge_start)
            except FileNotFoundError as e:
                err = e
                continue
            lr = NB.qcont(lr)
            W = NB.fk(lt, lr)
            g = BB.gates_b(W, c, name in LOOPS, np.asarray(w2h, float) * np.ones(len(W)), sp['kind'], relM)
            tried.append(dict(candidate=k, kind=sp['kind'], donors=[p['clip'] for p in info.get('donors', [])] if isinstance(info.get('donors'), list) else None,
                              PASS=g['PASS'], base=[g['base_gate_pass'], g['frames']], fail=g['base_gate_fail'], pen=g['pen_frames'],
                              weapon_pen=g['blade_pen_frames'], spike=g['spike_max_cm'], slide=max(g['slide_max_cm'].values())))
            if best is None or score(g) > score(best[0]):
                best = (g, (lt, lr, c, info, relM, w2h), k)
            if g['PASS']:
                break
        if best is None:
            print('%-26s SKIP (missing input: %s)' % (name, err), flush=True)
            for dd in (cdir, rdir):
                for ext in ('.npz', '.twohand.npz', '.info.json'):
                    if os.path.exists(os.path.join(dd, stem + ext)):
                        os.remove(os.path.join(dd, stem + ext))
            rep[name] = dict(PASS=False, skipped='input clip rejected -> vanilla: %s' % os.path.basename(str(err)))
            continue
        g, (lt, lr, c, info, relM, w2h), k = best
        w2h = np.asarray(w2h, float) * np.ones(len(lt))
        meta = dict(set=SET, clip=name, rig=RIG, animation_type='Normal', additive=False, root_motion_type='None', frames=len(lt),
                    duration=(len(lt) - 1) / 30.0, events=[], source='npc_blunt_build.py ' + SUFFIX, weapon=WEAPON)
        dmp = BC.NR.make_dump(P, lt, lr, meta)
        W = NB.fk(lt, lr)
        if name != 'guard_idle_loop':
            dv, _ = BC.vanilla(SET, name)
            assert len(dv['W']) == len(lt), (name, len(dv['W']), len(lt))
        if info.get('kind') in ('attack', 'heavy'):
            info['strike_built'] = int(BC.strike_frame(W, WEAPON, 0, len(W) - 1)[0])
            info['guard_dist_f0_fN_cm'] = [round(float(np.linalg.norm(W[q, NB.BODY_IX, :3, 3] - G.W[NB.BODY_IX, :3, 3], axis=1).max() * 100), 1)
                                           for q in (0, len(W) - 1)]
        info['grip'] = 'NPC grip: WeaponRight local rotation + translation and fingers of the NPC donor frame'
        info['grip_metric'] = grip_metric(W)
        info['candidates'] = tried
        info['candidate_chosen'] = k
        info['capsule_path_cm'] = np.round(c[:, 1] * 100, 2).tolist()
        sc = BB.start_continuity(name, SPEC[name] if SPEC[name]['kind'] == info.get('kind') else dict(kind=info.get('kind')), W)
        if sc:
            info['start_continuity'] = sc
        info['gates'] = {q: v for q, v in g.items() if not q.startswith('_')}
        info['failing_frames'] = [dict(frame=int(t), worst=g['_worst'][t]) for t in np.where(~g['_ok_frames'])[0]][:40]
        BB._BUILT.pop(name, None)
        tgt_dir, other_dirs = (cdir, (rdir, hdir)) if g['PASS'] else (rdir, (cdir, hdir))
        os.makedirs(tgt_dir, exist_ok=True)
        for od in other_dirs:
            for ext in ('.npz', '.twohand.npz', '.info.json'):
                if os.path.exists(os.path.join(od, stem + ext)):
                    os.remove(os.path.join(od, stem + ext))
        base = os.path.join(tgt_dir, stem)
        BC.NR.save_dump(base + '.npz', dmp)
        np.savez_compressed(base + '.twohand.npz', rel=np.asarray(relM, np.float64), w=w2h)
        MC.jdump(info, base + '.info.json')
        rep[name] = dict(PASS=g['PASS'], **{q: v for q, v in info['gates'].items() if q != 'PASS'},
                         donors=[p['clip'] for p in info.get('donors', [])] if isinstance(info.get('donors'), list) else [],
                         impact=[info.get('impact_vanilla'), info.get('strike_built')], speed=info.get('speed_chosen'), candidate=k,
                         candidates=len(tried), grip=info['grip_metric'], seconds=round(time.time() - t1, 1))
        print('%-26s %-4s c%d/%d base %3d/%3d fail=%s pen=%d wpn=%d(%.1fcm) head=%.1f ff1=%s slide=%s spike=%.1f hilt=%.1f grip=%.1fcm rad=%.1fcm amp=%s %s %.1fs' % (
            name, 'PASS' if g['PASS'] else 'FAIL', k, len(cands), g['base_gate_pass'], g['frames'], g['base_gate_fail'], g['pen_frames'],
            g['blade_pen_frames'], g['blade_clear_min_cm'], g['head_clear_min_cm'], g['hand_hand_one_hand_min_cm'], g['slide_max_cm'],
            g['spike_max_cm'], g['hilt_err_max_cm'], info['grip_metric']['palm_in_weapon_dev_max_cm'], info['grip_metric']['palm_to_axis_dev_max_cm'], info.get('donor_amplitude', '-'),
            [x['donors'] for x in tried][-1] if tried else '', time.time() - t1), flush=True)
    # chain closure
    for ch in CHAINS:
        mem = [m for m in ch if m in ORDER]
        bad = [m for m in mem if not rep.get(m, {}).get('PASS')]
        for m in mem:
            stem = '%s__%s.%s' % (SET, m, SUFFIX)
            if rep.get(m, {}).get('PASS') and bad:
                move(stem, cdir, hdir)
                rep[m]['chain_reverted'] = 'stays the game clip: chain member(s) %s did not pass' % ','.join(bad)
            elif m in rep:
                rep[m].pop('chain_reverted', None)
    MC.jdump(rep, rep_path)
    ship = sorted(m for m, v in rep.items() if v.get('PASS') and not v.get('chain_reverted'))
    print('CHAINS: shipped %d clips; reverted %s' % (len(ship), sorted(m for m, v in rep.items() if v.get('chain_reverted'))))
    print('BUILD done %.1f s -> %s' % (time.time() - t0, cdir))


if __name__ == '__main__':
    main(sys.argv[1:])
