"""Weapon films 2026-11-09 - shared code (offline; run with Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I).

Right side ("Ultimate Third Person Project"): the PRODUCTION pose evaluator (src/mm/MotionPose.hpp) driven by weapons_eval.exe (built by
weapons_build_eval.cmd into Build\\work\\weapon-films-20261109\\bin) on copies of the INSTALLED mod data (game v2 folder, copied read-only
to Build\\work\\weapon-films-20261109\\data, SHA-256 in data\\hashes.txt).  The 73 overlay-skeleton bones are the first 73 joints of the
152-joint player rig (same names, same order, checked), so the evaluator locals go straight into FK + the skinning of the melee tools.
Left side ("Game Animations"): the game's own Female V player clips (pwa_w_* animsets, WolvenKit glb exports), see weapons_vanilla.py.
Writes only into Build\\work\\weapon-films-20261109 and (films / sheets) Media\\mp4|png\\2026-11-09_weapon-films.
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'melee'))
import melee_common as MC  # noqa: E402
import npc_retarget as NR  # noqa: E402

WORK = os.path.join(ROOT, 'Build', 'work', 'weapon-films-20261109')
TMP = os.path.join(WORK, '_tmp')
EXE = os.path.join(WORK, 'bin', 'weapons_eval.exe')
DATA = os.path.join(WORK, 'data')
DB = os.path.join(DATA, 'utpp_motion_woman.utppmm')
OV = os.path.join(DATA, 'utpp_overlays_woman.utppov')
STYLE = os.path.join(DATA, 'utpp_style_weights_woman.txt')
RECOIL = os.path.join(DATA, 'recoil')
MEDIA_MP4 = os.path.join(ROOT, 'Media', 'mp4', '2026-11-09_weapon-films')
MEDIA_PNG = os.path.join(ROOT, 'Media', 'png', '2026-11-09_weapon-films')
FPS = 30
SUB = 2                      # evaluator steps per output frame (60 Hz inside, 30 fps out)
DT = 1.0 / (FPS * SUB)
OVL = {'Default': 0, 'Rifle': 1, 'Pistol1H': 2, 'Pistol2H': 3}
IDLE = 'M_Neutral_Stand_Idle_Loop'
WALK = 'M_Neutral_Walk_Loop_F'
CLIPLEN = {}
_P = {}


def player():
    if 'w' not in _P:
        _P['w'] = NR.Player('woman')
    return _P['w']


def clip_lengths():
    if not CLIPLEN:
        p = subprocess.run([EXE, 'list', DB, OV, STYLE], capture_output=True, text=True)
        for ln in p.stdout.splitlines():
            if ln.startswith('CLIP '):
                _, i, n, d = ln.split()
                CLIPLEN[n] = float(d)
    return CLIPLEN


class Timeline:
    """per evaluator step rows (60 Hz); the scenario builders append segments with set(...) and shot(...)."""

    def __init__(self, overlay='Pistol2H', recoil='-', fem=(0, 0.025), target=(0.0, 25.0, 1.45)):
        self.rows = []
        self.overlay = OVL[overlay]
        self.recoil = recoil
        self.fem = fem
        self.target = target
        self.clip, self.ct = IDLE, 0.0
        self.aim = 'ready'
        self.moving = 0
        self.pending_jump = None
        self.shots = set()

    def switch_clip(self, clip, t0=0.0, blend=0.3):
        self.clip, self.ct = clip, t0
        self.pending_jump = blend

    def run(self, seconds, aim=None, moving=None):
        if aim is not None:
            self.aim = aim
        if moving is not None:
            self.moving = moving
        n = int(round(seconds / DT))
        L = clip_lengths()[self.clip]
        for _ in range(n):
            k = len(self.rows)
            jumped, blend = (1, self.pending_jump) if self.pending_jump is not None else (0, 0.0)
            self.pending_jump = None
            if k > 0 and not jumped:
                self.ct += DT
                if self.ct > L:                       # loop clips: wrap (the live stack wraps loops itself)
                    self.ct -= L
            ready = 1 if self.aim == 'ready' else 0
            want_aim = 1 if self.aim == 'aim' else 0
            shot = 1 if k in self.shots else 0
            self.rows.append([DT, self.clip, self.ct, jumped, blend, self.overlay, ready, want_aim, self.moving, self.moving, shot, self.recoil,
                              self.fem[0], self.fem[1], 1, self.target[0], self.target[1], self.target[2], 1 if k % SUB == SUB - 1 else 0])

    def now_step(self):
        return len(self.rows)

    def write(self, path):
        with open(path, 'w') as f:
            f.write('# dt clip clipTime jumped blendTime overlay wantsReady wantsToAim moving hasVelocity shot recoilFile femMode femAmount '
                    'haveAim tx ty tz emit\n')
            for r in self.rows:
                f.write(' '.join(str(x) if not isinstance(x, float) else '%.6f' % x for x in r) + '\n')


def run_eval(tl, name, shots_s=(), recoil_file=None):
    """shots_s: shot times (s from the start); rows get shot=1 at the nearest step. -> dict(lt, lr [T,73], names, par, diag, root)."""
    os.makedirs(TMP, exist_ok=True)
    for t in shots_s:
        k = int(round(t / DT))
        tl.rows[k][10] = 1
    if recoil_file:
        for r in tl.rows:
            r[11] = recoil_file
    sp = os.path.join(TMP, name + '.steps.txt')
    tl.write(sp)
    pre = os.path.join(TMP, name)
    p = subprocess.run([EXE, 'run', DB, OV, STYLE, RECOIL, sp, pre], capture_output=True, text=True)
    if p.returncode or 'RAN' not in p.stdout:
        raise RuntimeError('weapons_eval failed %d: %s %s' % (p.returncode, p.stdout[-800:], p.stderr[-800:]))
    lines = open(pre + '.bones.txt').read().split('\n')
    n = int(lines[0])
    names = [lines[1 + i].split()[0] for i in range(n)]
    par = np.array([int(lines[1 + i].split()[1]) for i in range(n)])
    raw = np.fromfile(pre + '.pose.bin', dtype='<f4').reshape(-1, n, 7).astype(float)
    root = np.fromfile(pre + '.root.bin', dtype='<f4').reshape(-1, 2, 7).astype(float)
    diag = np.genfromtxt(pre + '.diag.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')
    P = player()
    assert names == P.names[:73], 'overlay skeleton differs from the first 73 player joints'
    return dict(lt=raw[:, :, :3], lr=raw[:, :, 3:], names=names, par=par, diag=diag, root=root, log=p.stdout)


def full152(lt73, lr73):
    """evaluator locals of the 73 core bones -> 152-joint game-axes locals (extra joints = player reference locals)."""
    P = player()
    T = len(lt73)
    lt = np.repeat(P.ref['rest_lt'][None].astype(float), T, 0)
    lr = np.repeat(P.ref['rest_lr'][None].astype(float), T, 0)
    lt[:, :73] = lt73
    lr[:, :73] = lr73
    return lt, lr


def fk(lt, lr):
    return MC.fk_world(player().par, lt, NR.S.qcontinuous(lr))


def place_root(W, root_xy, root_yaw=None):
    """moves the whole pose by a world path: root_xy [T,2] (m), optional yaw [T] (rad about +Z)."""
    T = len(W)
    M = np.repeat(np.eye(4)[None], T, 0)
    if root_yaw is not None:
        c, s = np.cos(root_yaw), np.sin(root_yaw)
        M[:, 0, 0], M[:, 0, 1], M[:, 1, 0], M[:, 1, 1] = c, -s, s, c
    M[:, 0, 3], M[:, 1, 3] = root_xy[:, 0], root_xy[:, 1]
    return np.einsum('tij,tnjk->tnik', M, W)


def sanity(W, tag):
    P = player()
    ix = P.ix
    head = W[:, ix['Head'], :3, 3]
    hips = W[:, ix['Hips'], :3, 3]
    rh = W[:, ix['RightHand'], :3, 3]
    out = dict(tag=tag, head_z=[round(float(head[:, 2].min()), 3), round(float(head[:, 2].max()), 3)],
               hand_y_minus_hips=[round(float((rh[:, 1] - hips[:, 1]).min()), 3), round(float((rh[:, 1] - hips[:, 1]).max()), 3)],
               sole_min_cm=round(float(NR.sole_heights(P, W[:, :, :3, 3]).min() * 100), 1))
    print('SANITY', json.dumps(out))
    return out


def jdump(obj, path):
    MC.jdump(obj, path)
