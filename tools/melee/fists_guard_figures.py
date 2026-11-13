"""Offline figures and continuity numbers of the fists guard (2026-11-13): hands' height through punch - pause - punch - walk - sprint.

  python -I fists_guard_figures.py --hands-ship <hands_ship_pwa.npz> --mm <mm_hands_woman.npz> --timelines <dir> --out <dir> --tag female

Run with the python 3.12 that has numpy; matplotlib is taken from Tools\\pylibs (found by walking up from this file).
Inputs
  --hands-ship  per-clip joint tracks (cm, game axes, model space) of the SHIPPED fists set, from the exported utpp_pwa_fists.anims.glb
                (npz keys clip__<name> -> [T, 11, 3], joint order LeftHand RightHand LeftForeArm RightForeArm Hips Spine3 Head LeftFoot
                RightFoot LeftArm RightArm);
  --mm          the same joints of the locomotion source clips (M_Neutral_Stand_Idle_Loop / Walk_Loop_F / Sprint_Loop_F);
  --timelines   timeline_<name>.csv written by `UTPP_GUARD_TIMELINE_DIR=<dir> utpp_melee_fullbody_test.exe`: the REAL mask / tilt / leg
                weights of the Feed wiring (MeleeGate, GuardGate, MaskTimeTracker, UpperMaskBlend) over the PSM timeline of
                TestGuardSequences (before_review, before_main, after_guard; hard_jump, hard_swap, soft_crouch optional).
Model: hand(t) = (1 - w(t)) * locomotion hand + w(t) * game hand, w = the logged upper-mask weight; game hand = the guard
(melee_idlepose) except while a hold / strike clip plays (hold_start_01 for the hold, combo_attack_NN from the strike start to its last
frame, which is the guard again); locomotion hand = idle / walk / sprint loop (hips-relative xy, absolute z). A linear blend of hand
positions stands in for the bone-local blend of the DLL (BlendIntoRecords: lerp position, nlerp rotation).
Output: <out>/fists_guard_hands_<tag>.png and .json (the numbers of the report).
"""
import argparse
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np

for _p in [Path(os.path.abspath(__file__)).parent] + list(Path(os.path.abspath(__file__)).parents):
    _lib = _p / 'Tools' / 'pylibs'
    if _lib.is_dir():
        sys.path.insert(0, str(_lib))
        break
import matplotlib  # noqa: E402

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

LH, RH = 0, 1
HIPS = 4
FPS = 30.0
STRIKE_CLIPS = ['combo_attack_01', 'combo_attack_02', 'combo_attack_03']


def load_npz(p):
    z = np.load(p)
    return {k[6:]: z[k].astype(np.float64) for k in z.files if k.startswith('clip__')}


def load_csv(p):
    rows = list(csv.DictReader(open(p, encoding='utf-8')))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def sample(clip, t, loop=False):
    """clip [T,J,3] at time t (s), linear between the 30 fps frames; the last frame is held (or looped)."""
    n = len(clip)
    f = t * FPS
    if loop:
        f = f % (n - 1)
    f = min(max(f, 0.0), n - 1.0)
    i = int(np.floor(f))
    j = min(i + 1, n - 1)
    a = f - i
    return clip[i] * (1 - a) + clip[j] * a


def strikes_of(tl):
    """[(t_hold_start, t_attack_start)] of every strike: a Hold 6 run followed by the attack state."""
    st = tl['state']
    out = []
    i = 0
    while i < len(st):
        if st[i] == 6:
            j = i
            while j < len(st) and st[j] == 6:
                j += 1
            out.append((float(tl['t'][i]), float(tl['t'][j]) if j < len(st) else float(tl['t'][-1])))
            i = j
        else:
            i += 1
    return out


def hands_track(tl, ship, mm, strikes):
    """(pos, game, loco) hand tracks [N,2,3] over the timeline."""
    t = tl['t']
    guard = ship['melee_idlepose'][0]
    hold = ship['hold_start_01']
    clips = {'idle': mm['M_Neutral_Stand_Idle_Loop'], 'walk': mm['M_Neutral_Walk_Loop_F'], 'sprint': mm['M_Neutral_Sprint_Loop_F']}

    def mmhand(clip, tt):
        s = sample(clip, tt, loop=True)
        return s[[LH, RH]] - s[HIPS] + np.array([0.0, 0.0, s[HIPS][2]])

    game = np.zeros((len(t), 2, 3))
    loco = np.zeros((len(t), 2, 3))
    kind, prev_kind, kind_t = 'idle', 'idle', -1.0
    for k, tt in enumerate(t):
        new_kind = 'sprint' if tl['detail'][k] == 4 else ('walk' if tl['stick'][k] > 0 else 'idle')
        if new_kind != kind:
            prev_kind, kind, kind_t = kind, new_kind, tt
        a = min(1.0, (tt - kind_t) / 0.3) if kind_t >= 0 else 1.0
        a = a * a * (3 - 2 * a)
        loco[k] = (1 - a) * mmhand(clips[prev_kind], tt) + a * mmhand(clips[kind], tt)
        g = guard[[LH, RH]]
        for n, (th, ts) in enumerate(strikes):
            clip = ship[STRIKE_CLIPS[n % len(STRIKE_CLIPS)]]
            if th <= tt < ts:
                g = sample(hold, tt - th)[[LH, RH]]
            elif ts <= tt < ts + (len(clip) - 1) / FPS:
                g = sample(clip, tt - ts)[[LH, RH]]
        game[k] = g
    w = tl['mask'][:, None, None]
    return (1 - w) * loco + w * game, game, loco


def resample(tl, arr, tt):
    return np.stack([np.stack([np.interp(tt, tl['t'], arr[:, h, a]) for a in range(3)], -1) for h in range(2)], 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hands-ship', required=True)
    ap.add_argument('--mm', required=True)
    ap.add_argument('--timelines', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--title', default='')
    a = ap.parse_args()
    ship, mm = load_npz(a.hands_ship), load_npz(a.mm)
    tls = {}
    for k in ('before_review', 'before_main', 'after_guard', 'hard_jump', 'hard_swap', 'soft_crouch'):
        p = os.path.join(a.timelines, 'timeline_%s.csv' % k)
        if os.path.exists(p):
            tls[k] = load_csv(p)
    strikes = strikes_of(tls['after_guard'])
    G = ship['melee_idlepose'][0][[LH, RH]]
    res = {'tag': a.tag, 'strikes_hold_start_attack_start_s': strikes}

    # ---- hard cuts between the shipped clips (hands, cm): the graph goes 0 s into an attack, so the next clip starts as a cut
    names = [n for n in ('combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'combo_attack_04', 'combo_attack_05', 'final_attack_03',
                         'final_attack_04', 'final_attack_05', 'safe_attack', 'crouch_attack', 'sprint_attack', 'jump_attack') if n in ship]
    cuts = {}
    for x in names:
        for y in names:
            cuts['%s>%s' % (x, y)] = float(np.linalg.norm(ship[x][-1][[LH, RH]] - ship[y][0][[LH, RH]], axis=-1).max())
    res['cut_end_to_start_cm'] = {'max': max(cuts.values()), 'worst': max(cuts, key=cuts.get)}
    res['guard_to_clip_start_cm'] = {n: float(np.linalg.norm(ship[n][0][[LH, RH]] - G, axis=-1).max()) for n in names + ['hold_start_01']}
    res['clip_end_to_guard_cm'] = {n: float(np.linalg.norm(ship[n][-1][[LH, RH]] - G, axis=-1).max()) for n in names + ['hold_start_01']}
    hd = np.linalg.norm(ship['hold_start_01'][:, [LH, RH]] - G, axis=-1).max(axis=1)
    res['hold_start_distance_to_guard_by_frame_cm'] = [round(float(x), 2) for x in hd]
    res['hold_to_strike_cut_at_3_frames_cm'] = float(hd[3])  # the click is released after ~0.1 s = 3 frames of hold_start_01
    # hand speed inside the real clips (cm / frame at 30 fps): the natural motion of the animation itself
    peaks = {n: float(np.linalg.norm(np.diff(ship[n][:, [LH, RH]], axis=0), axis=-1).max()) for n in names + ['hold_start_01']}
    res['clip_hand_step_cm_per_frame_30fps'] = peaks
    res['clip_hand_step_peak'] = max(peaks.values())

    # ---- trajectories
    track, gtrack, ltrack = {}, {}, {}
    for k, tl in tls.items():
        track[k], gtrack[k], ltrack[k] = hands_track(tl, ship, mm, strikes if k in ('before_review', 'before_main', 'after_guard') else [])

    def blend_step(k, t0=None, t1=None):
        """|d weight| * |game hand - locomotion hand| per 1/30 s: what the mask fade itself adds to the hands, cm / frame."""
        tl = tls[k]
        tt = np.arange(tl['t'][0], tl['t'][-1], 1.0 / FPS)
        w = np.interp(tt, tl['t'], tl['mask'])
        dist = np.linalg.norm(resample(tl, gtrack[k], tt) - resample(tl, ltrack[k], tt), axis=-1).max(axis=1)
        s = np.abs(np.diff(w)) * dist[1:]
        t_ = tt[1:]
        if t0 is None:
            return t_, s
        m = (t_ >= t0) & (t_ < t1)
        return float(s[m].max()) if m.any() else 0.0

    # ---- figure
    variants = [('before_review', 'before: guard off, old release (hold 0.3 s, fade 0.15 s)'),
                ('before_main', 'before: guard off, current main (hold 0.55 s, fade 0.35 s)'),
                ('after_guard', 'after: guard on')]
    fig, axs = plt.subplots(4, 1, figsize=(13, 11), sharex=True, gridspec_kw={'height_ratios': [1, 1, 1, 0.9]})
    for ax, (k, label) in zip(axs[:3], variants):
        tl = tls[k]
        ax.plot(tl['t'], track[k][:, LH, 2], color='tab:blue', lw=1.6, label='left hand')
        ax.plot(tl['t'], track[k][:, RH, 2], color='tab:red', lw=1.6, label='right hand')
        ax.plot(tl['t'], tl['mask'] * 40 + 70, color='0.55', lw=1.0, ls='--', label='upper-body mask weight (0 = 70 cm, 1 = 110 cm)')
        ax.axhline(float(G[LH][2]), color='tab:blue', lw=0.6, ls=':')
        ax.axhline(float(G[RH][2]), color='tab:red', lw=0.6, ls=':')
        ax.set_ylabel('hand height (cm)')
        ax.set_ylim(55, 200)
        ax.set_title(label, fontsize=10, loc='left')
    ta = tls['after_guard']
    t, det, stick = ta['t'], ta['detail'], ta['stick']
    clip_len = (len(ship['combo_attack_01']) - 1) / FPS
    walk_idx = np.where((stick > 0) & (det != 4))[0]
    spr_idx = np.where(det == 4)[0]
    w0, w1, s0, s1 = float(t[walk_idx[0]]), float(t[walk_idx[-1]]), float(t[spr_idx[0]]), float(t[spr_idx[-1]])
    for ax in axs:
        for th, ts in strikes:
            ax.axvspan(th, ts + clip_len, color='gold', alpha=0.18, lw=0)
        ax.axvspan(w0, w1, color='tab:green', alpha=0.10, lw=0)
        ax.axvspan(s0, s1, color='tab:purple', alpha=0.10, lw=0)
    axs[0].text(strikes[0][0] + 0.05, 192, 'punch', fontsize=9, va='top')
    axs[0].text(w0 + 0.05, 192, 'walk', fontsize=9, va='top')
    axs[0].text(s0 + 0.05, 192, 'sprint', fontsize=9, va='top')
    axs[0].legend(loc='lower right', fontsize=8, ncol=3)
    for k, col, lab in (('before_review', 'tab:red', 'before (old release 0.3 s / 0.15 s)'), ('before_main', 'tab:orange', 'before (main 0.55 s / 0.35 s)'),
                        ('after_guard', 'tab:blue', 'after (guard)')):
        t_, s = blend_step(k)
        axs[3].plot(t_, s, color=col, lw=1.2, label=lab)
    axs[3].axhline(res['clip_hand_step_peak'], color='k', lw=0.8, ls='--', label='fastest hand step inside the fist clips (%.0f)' % res['clip_hand_step_peak'])
    axs[3].set_ylabel('added by the mask fade\n(cm / frame @30 fps)')
    axs[3].set_xlabel('time (s)')
    axs[3].set_ylim(0, res['clip_hand_step_peak'] * 1.1)
    axs[3].legend(loc='upper right', fontsize=8)
    fig.suptitle(a.title or ("Fists guard, %s V: hands' height through punch - pause - punch - walk - sprint (offline, shipped clips, real fade code)" % a.tag),
                 fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    os.makedirs(a.out, exist_ok=True)
    png = os.path.join(a.out, 'fists_guard_hands_%s.png' % a.tag)
    fig.savefig(png, dpi=110)
    plt.close(fig)

    # ---- the peak the mask fade adds per transition (the strike clips' own motion is not part of this number)
    t_state_end = float(ta['t'][np.where(ta['state'] >= 11)[0][-1]])
    bs = {
        'after: raise out of the locomotion arms (first 0.7 s)': blend_step('after_guard', 0.0, 0.7),
        'after: walk start (guard stays on)': blend_step('after_guard', w0 - 0.2, w0 + 1.0),
        'after: sprint start (soft end, eased 0.35 s)': blend_step('after_guard', s0 - 0.1, s0 + 0.9),
        'before (main): release after the last strike (hold 0.55 s + fade 0.35 s)': blend_step('before_main', t_state_end, t_state_end + 1.5),
        'before (old review): release after the last strike (hold 0.3 s + fade 0.15 s)': blend_step('before_review', t_state_end, t_state_end + 1.2),
    }
    for extra, lab in (('hard_jump', 'after: hard end, jump (quick 0.15 s)'), ('hard_swap', 'after: hard end, UpperBody swap / holster (quick 0.15 s)'),
                       ('soft_crouch', 'after: soft end, crouch 0.3 s then stand (eased out, 0.25 s raise)')):
        if extra in tls:
            bs[lab] = blend_step(extra, 0.5, 99.0)
    res['blend_step_cm_per_frame_30fps'] = bs
    res['guard_to_locomotion_idle_hand_distance_cm'] = float(np.linalg.norm(gtrack['after_guard'][5] - ltrack['after_guard'][5], axis=-1).max())
    res['mask_after_last_strike_state_end'] = {}
    for k in ('before_review', 'before_main', 'after_guard'):
        tl = tls[k]
        m = tl['mask']
        i0 = np.where(tl['t'] >= t_state_end)[0][0]
        hi = np.where(m[i0:] < 0.99)[0]
        lo = np.where(m[i0:] < 0.01)[0]
        res['mask_after_last_strike_state_end'][k] = {
            'starts_dropping_after_s': round(float(tl['t'][i0 + hi[0]] - t_state_end), 3) if len(hi) else None,
            'back_on_locomotion_after_s': round(float(tl['t'][i0 + lo[0]] - t_state_end), 3) if len(lo) else None}
    json.dump(res, open(os.path.join(a.out, 'fists_guard_hands_%s.json' % a.tag), 'w'), indent=1)
    print('wrote', png)
    print(json.dumps({k: res[k] for k in ('hold_to_strike_cut_at_3_frames_cm', 'clip_hand_step_peak', 'cut_end_to_start_cm', 'blend_step_cm_per_frame_30fps',
                                          'guard_to_locomotion_idle_hand_distance_cm', 'mask_after_last_strike_state_end')}, indent=1))


if __name__ == '__main__':
    main()
