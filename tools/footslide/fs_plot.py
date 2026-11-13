"""PYTHONPATH=Tools/pylibs python fs_plot.py <outdir> : evidence PNGs (before/after contact-point slide per frame for chosen clips + per-clip bars).
Uses the replay-independent offline measure (fs_lib.measure / contact_point_slide)."""
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L  # noqa: E402

ROOT = L.ROOT
W = os.path.join(ROOT, 'Build/work/aimotion-db')
SRC = {'woman': os.path.join(W, 'inputs/clips'), 'man': os.path.join(ROOT, 'Animations/retarget/out_sourcepack_man_knee_twistfix')}
FIX = {'woman': os.path.join(W, 'clips-turnslide-woman'), 'man': os.path.join(W, 'clips-turnslide-man')}
out = sys.argv[1]
os.makedirs(out, exist_ok=True)

picks = [('woman', 'M_Neutral_Walk_Turn_L_180_Lfoot'), ('woman', 'M_Neutral_Crouch_Pivot_RR_LR_Lfoot'),
         ('man', 'M_Neutral_Crouch_Reface_Start_F_L_180'), ('man', 'M_Neutral_Crouch_Turn_R_180_Rfoot')]
fig, axs = plt.subplots(len(picks), 2, figsize=(13, 3.0 * len(picks)))
for r, (g, n) in enumerate(picks):
    a = L.Clip(os.path.join(SRC[g], n + '.utppclip.json'), g)
    b = L.Clip(os.path.join(FIX[g], n + '.utppclip.json'), g)
    ma, mb = L.measure(a), L.measure(b)
    side = 'Left' if max(ma['Left']['path'], 0) >= ma['Right']['path'] else 'Right'
    pa = L.planted(a, side)
    ax = axs[r, 0]
    for c, col, lab in ((a, 'tab:red', 'source'), (b, 'tab:green', 'pinned')):
        af, h, t = c.foot_points(side)
        s = L.contact_point_slide(af, h, t)
        ax.plot(np.arange(len(s)), np.where(pa[:-1] & pa[1:], s, np.nan), color=col, lw=1.6, label=lab)
    ax.set_title('%s  %s foot: planted-frame slide (cm/frame)' % (n.replace('M_Neutral_', ''), side), fontsize=9)
    ax.set_xlabel('frame (30 fps)')
    ax.legend(fontsize=8)
    ax = axs[r, 1]
    for c, col, lab in ((a, 'tab:red', 'source'), (b, 'tab:green', 'pinned')):
        af, h, t = c.foot_points(side)
        sl = L.contact_point_slide(af, h, t)
        for i, (sb, se) in enumerate(L.segments(pa[:-1] & pa[1:])):
            ax.plot(np.arange(se - sb + 1), np.cumsum(sl[sb:se + 1]), color=col, lw=1.4, label=lab if i == 0 else None)
    ax.set_title('cumulative contact-point slide within each plant (cm)', fontsize=9)
    ax.set_xlabel('frames since plant start')
    ax.legend(fontsize=8)
plt.tight_layout()
plt.savefig(os.path.join(out, 'turnslide_examples.png'), dpi=110)
plt.close()

fig, axs = plt.subplots(1, 2, figsize=(14, 6))
for ax, g in zip(axs, ('woman', 'man')):
    rows = json.load(open(os.path.join(ROOT, 'Build/work/turnslide/ver_%s.json' % g)))
    rows.sort(key=lambda r: -r[2])
    y = np.arange(len(rows))
    ax.barh(y + 0.2, [r[2] for r in rows], 0.4, color='tab:red', label='source (worst single plant, cm)')
    ax.barh(y - 0.2, [r[3] for r in rows], 0.4, color='tab:green', label='pinned')
    ax.set_yticks(y)
    ax.set_yticklabels([r[0].replace('M_Neutral_', '') for r in rows], fontsize=6)
    ax.invert_yaxis()
    ax.set_title('%s: worst single-plant contact-point drift per fixed clip' % g, fontsize=9)
    ax.legend(fontsize=8)
plt.tight_layout()
plt.savefig(os.path.join(out, 'turnslide_worst_plant_per_clip.png'), dpi=110)
plt.close()
print('ok')
