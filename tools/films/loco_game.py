"""Left side of the locomotion comparison films ("Game Animations"): the game's own Female V player locomotion clips (2.5,
base\\animations\\player\\female\\body\\player_locomotion.anims, no UTPP correction) on the SAME capsule path and actor yaw as the right side.

  python -I loco_game.py survey            -> natural speed / direction of every loop clip (planted-foot method), printed
  (used by loco_film.py: timeline(feed_path) -> world matrices [T,73,4,4] at 60 Hz + per-frame clip labels)
The game has no on-foot third-person graph of its own: the player body plays these FPP-authored clips (the body seen in mirrors and in a
third-person camera without UTPP), driven by player_locomotion.animgraph.  That graph is NOT evaluated here; the clip choice below is a
documented reconstruction from the same input (one rule set, not tuned per scenario):
  * capsule: position and actor yaw of the feed (the game moves the player by the capsule; the clips are exported without root motion and
    play in place, Root = identity, body placed at the capsule with the actor yaw);
  * state per 60 Hz frame from the feed: slide (PSM detail 5) / crouch (loco 2, moving = speed > 0.3 m/s) / stand + sprint (loco 1 / 3,
    moving = stick held: the stop starts when the stick is released, while the capsule still brakes);
  * moving stand: direction = capsule velocity in the body frame, gait walk_slow / walk_fast / jog = the loop whose natural speed is
    closest to the capsule speed, the nearest of the 8 directional loops; sprint = fpp_sprint_0 while the velocity is within 60 deg of
    the facing (else the jog directional loop: the game has no backward / sideways sprint);
  * starts: fpp_idle_to_<walk|jog>_<0|090|180|270> (direction of the stick at the start, walk if the target speed < 2.5 m/s), played once
    at 1x, then the loop; stops: fpp_<walk|jog>_<0|090|180|270>_to_idle_<l|r> (velocity direction at the release; l / r = the variant
    whose first frame matches the current leg pose best), then fpp_idle_stand;
  * crouch: fpp_crouch_idle / fpp_crouch_walk_<0|045|090|180|270|315> (no 135 / 225 clip: nearest available); no crouch-enter or
    stand-up clip exists in the set, the stance change is the crossfade only;
  * slide: slide_start (once) -> slide_loop (loop) -> slide_end (once) after the PSM slide ends;
  * crossfade XF = 0.2 s on every change (the most common transition 'duration' values of player_locomotion.animgraph are 0.1 / 0.2 /
    0.3 s; the graph's own per-transition values are not read); a change during a running crossfade blends from the current output
    pose; a loop -> loop change (gait / direction) needs the new choice for DWELL = 0.15 s (hysteresis, no flicker between loops);
  * loops: entered at the frame whose foot positions (hips frame) are closest to the current pose (stand-in for the graph's sync), played
    at rate = capsule speed / natural speed (floor 0.3, no upper limit: favours the game side, its feet stay as planted as the clip
    allows; e.g. crouch walk 0.93 m/s natural vs 2.73 m/s capsule = 2.9x); starts / stops / slide parts at 1x.
Natural loop speed / direction: from the clip itself (no root motion in the export): median planted-foot velocity in the hips heading
frame over the frames where that foot is within 3 cm of its lowest height, negated.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import loco_common as LC  # noqa: E402

P, NR, MC = LC.P, LC.NR, LC.MC
XF = 0.2
DWELL = 9                      # frames (0.15 s at 60 Hz) a new gait / direction loop must stay chosen before the loop changes
DIR8 = ['0', '045', '090', '135', '180', '225', '270', '315']
LOOPS = {'walk_slow': ['fpp_walk_slow_' + d for d in DIR8], 'walk_fast': ['fpp_walk_fast_' + d for d in DIR8],
         'jog': ['fpp_jog_' + d for d in DIR8], 'crouch': ['fpp_crouch_walk_' + d for d in ('0', '045', '090', '180', '270', '315')],
         'sprint': ['fpp_sprint_0']}
_SK = {}
_CLIP = {}
_NAT = {}


def skel():
    if 'sk' not in _SK:
        sk = NR.skeleton(LC.GLB)
        assert list(sk.names) == P.names, 'player_locomotion glb joint order differs from the player rig'
        _SK['sk'] = sk
    return _SK['sk']


def clip(name):
    """-> dict(lt [n,152,3], lr [n,152,4] game axes at 60 Hz, Root = identity, W model [n,152,4,4], dur)."""
    if name not in _CLIP:
        lt, lr, ex, dur = NR.sample_clip(skel(), name)
        lt, lr = NR.locals_to_game(lt, lr)
        lr = NR.S.qcontinuous(lr)
        n30 = len(lt)
        tt = np.arange(0, (n30 - 1) * 2 + 1) / 2.0                 # 30 fps samples -> 60 Hz (linear / slerp between neighbours)
        i0 = np.floor(tt).astype(int)
        i1 = np.minimum(i0 + 1, n30 - 1)
        w = tt - i0
        lt6 = (1 - w)[:, None, None] * lt[i0] + w[:, None, None] * lt[i1]
        lr6 = LC.qslerp(lr[i0], lr[i1], np.repeat(w[:, None], lr.shape[1], 1))
        ri = P.ix['Root']
        lt6[:, ri] = 0.0
        lr6[:, ri] = [0, 0, 0, 1.0]
        _CLIP[name] = dict(lt=lt6, lr=lr6, W=MC.fk_world(P.par, lt6, lr6), dur=dur, n=len(lt6))
    return _CLIP[name]


def heading(W):
    """hips heading frame [T,3,3] (columns right, forward, up) from the up-legs (melee helper)."""
    return MC.heading_frame(W[:, P.ix['LeftUpLeg'], :3, 3], W[:, P.ix['RightUpLeg'], :3, 3])


def feet_rel(W):
    """foot + toe positions relative to the hips in the hips heading frame [T,12] (pose-match features)."""
    H = heading(W)
    h = W[:, P.ix['Hips'], :3, 3]
    out = []
    for n in ('LeftFoot', 'RightFoot', 'LeftToeBase', 'RightToeBase'):
        out.append(np.einsum('tji,tj->ti', H, W[:, P.ix[n], :3, 3] - h))
    return np.concatenate(out, 1)


def natural(name):
    """natural velocity (x right, y forward, m/s, model frame) of a loop clip from its planted feet."""
    if name not in _NAT:
        W = clip(name)['W']
        vs = []
        for f in ('LeftToeBase', 'RightToeBase', 'LeftFoot', 'RightFoot'):
            p = W[:, P.ix[f], :3, 3]
            low = p[:, 2] < p[:, 2].min() + 0.03
            v = np.gradient(p[:, :2], 1.0 / LC.FPS, axis=0)
            if low.sum() > 5:
                vs.append(v[low])
        v = -np.median(np.concatenate(vs), 0)
        _NAT[name] = v
    return _NAT[name]


def survey():
    for g, names in LOOPS.items():
        for n in names:
            v = natural(n)
            c = clip(n)
            print('%-22s dur %.2f s  speed %.2f m/s  dir %6.1f deg (0 = forward, +90 = right)' %
                  (n, c['dur'], np.hypot(*v), np.degrees(np.arctan2(v[0], v[1]))))


def nearest(names, ang):
    """the loop of `names` whose natural direction is closest to ang (deg, body frame)."""
    best, bd = None, 1e9
    for n in names:
        v = natural(n)
        a = np.degrees(np.arctan2(v[0], v[1]))
        dd = abs((a - ang + 180) % 360 - 180)
        if dd < bd:
            best, bd = n, dd
    return best


def dir4(ang):
    """0 / 090 / 180 / 270 label of a body-frame direction, mapped through the measured directions of the jog loops."""
    lab = nearest(['fpp_jog_0', 'fpp_jog_090', 'fpp_jog_180', 'fpp_jog_270'], ang)
    return lab.rsplit('_', 1)[1]


def best_entry(name, feat):
    """frame of clip `name` whose foot features are closest to feat [12]."""
    F = feet_rel(clip(name)['W'])
    return int(np.argmin(np.sum((F - feat[None]) ** 2, 1)))


class Track:
    def __init__(self, name, t0=0.0, rate=1.0, loop=True):
        self.name, self.t, self.rate, self.loop = name, float(t0), rate, loop

    def pose(self):
        c = clip(self.name)
        n = c['n']
        x = self.t % (n - 1) if self.loop else min(self.t, n - 1)
        i0 = int(np.floor(x))
        i1 = min(i0 + 1, n - 1) if not self.loop else (i0 + 1) % (n - 1)
        w = x - i0
        lt = (1 - w) * c['lt'][i0] + w * c['lt'][i1]
        lr = LC.qslerp(c['lr'][i0], c['lr'][i1], np.full(c['lr'].shape[1], w))
        return lt, lr

    def done(self):
        return (not self.loop) and self.t >= clip(self.name)['n'] - 1

    def step(self):
        self.t += self.rate


def read_feed(path):
    rows = []
    for ln in open(path, encoding='utf-8'):
        if ln.startswith('#') or not ln.strip():
            continue
        p = ln.split()
        rows.append([float(x) for x in p[:10]] + [int(p[10])])
    a = np.array(rows, float)
    t, dt, vx, vy, mvx, mvy, yaw, loco, psm = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 5], a[:, 6], a[:, 7], a[:, 9].astype(int), a[:, 10].astype(np.int64)
    x = np.cumsum(vx * dt)
    y = np.cumsum(vy * dt)
    return dict(t=t, dt=dt, vx=vx, vy=vy, mvx=mvx, mvy=mvy, yaw=np.radians(yaw), loco=loco, slide=(((psm >> 16) & 0xFF) - 1) == 5, x=x, y=y)


def body_dir(vx, vy, yaw):
    """direction (deg, 0 = forward, +90 = right) of a world vector in the body frame of the actor yaw (forward (-sin, cos), right (cos, sin))."""
    fx, fy = -np.sin(yaw), np.cos(yaw)
    rx, ry = np.cos(yaw), np.sin(yaw)
    return float(np.degrees(np.arctan2(vx * rx + vy * ry, vx * fx + vy * fy)))


def timeline(feed_path):
    """feed -> (W world [T,73,4,4] 60 Hz, labels [T] clip names, feed dict)."""
    fd = read_feed(feed_path)
    T = len(fd['t'])
    spd = np.hypot(fd['vx'], fd['vy'])
    stick = np.hypot(fd['mvx'], fd['mvy'])
    cur = Track('fpp_idle_stand')
    prev, fade = None, 1.0
    state = 'idle'
    out_lt = np.zeros((T, len(P.names), 3))
    out_lr = np.zeros((T, len(P.names), 4))
    labels = []
    xf_frames = XF * LC.FPS
    last_pose = None
    pend = [None, 0]
    loop_names = set(n for v in LOOPS.values() for n in v)

    def switch(name, loop, t0=None, rate=1.0):
        nonlocal cur, prev, fade
        if t0 is None:
            t0 = best_entry(name, feet_rel(MC.fk_world(P.par, last_pose[0][None], last_pose[1][None]))[0]) if loop and last_pose else 0.0
        # a change during a running crossfade blends from the current output pose (frozen), never from a half-faded track (no pop)
        outgoing = cur if prev is None or last_pose is None else Frozen(*last_pose)
        prev, cur, fade = outgoing, Track(name, t0, rate, loop), 0.0
        pend[0], pend[1] = None, 0

    def switch_loop(name, rate):
        """loop -> loop change (gait / direction) only after the new choice held DWELL frames (hysteresis of the reconstruction)."""
        if cur.name == name:
            pend[0], pend[1] = None, 0
            return
        if pend[0] == name:
            pend[1] += 1
        else:
            pend[0], pend[1] = name, 1
        if pend[1] >= DWELL or cur.name not in loop_names:
            switch(name, True, None, rate)

    for i in range(T):
        # standing ground: the stick drives start / stop (stop = stick released, while the capsule still brakes); slide / crouch use speed
        moving = stick[i] > 0.1
        ang = body_dir(fd['vx'][i], fd['vy'][i], fd['yaw'][i]) if spd[i] > 0.05 else (
            body_dir(fd['mvx'][i], fd['mvy'][i], fd['yaw'][i]) if stick[i] > 0.1 else 0.0)
        # ---- desired state
        if fd['slide'][i]:
            want = 'slide'
        elif fd['loco'][i] == 2 or (state in ('slide', 'slide_end') and fd['loco'][i] == 0):
            want = 'crouch_move' if spd[i] > 0.3 else 'crouch_idle'
        else:
            want = 'move' if moving else 'idle'
        # ---- transitions
        if want == 'slide':
            if state != 'slide':
                switch('slide_start', False, 0.0)
                state = 'slide'
            elif cur.name == 'slide_start' and cur.t >= clip('slide_start')['n'] - 1 - xf_frames:
                switch('slide_loop', True, 0.0)
        elif state == 'slide':
            switch('slide_end', False, 0.0)
            state = 'slide_end'
        elif state == 'slide_end' and not cur.done() and want in ('crouch_move', 'crouch_idle', 'move'):
            pass                                                           # slide_end plays to its end
        elif want == 'idle':
            if state in ('move', 'start'):
                g = 'walk' if spd[i] < 2.5 else 'jog'
                d4 = dir4(body_dir(fd['vx'][i], fd['vy'][i], fd['yaw'][i]) if spd[i] > 0.05 else 0.0)
                cands = ['fpp_%s_%s_to_idle_%s' % (g, d4, s) for s in ('l', 'r')]
                feat = feet_rel(MC.fk_world(P.par, last_pose[0][None], last_pose[1][None]))[0]
                best = min(cands, key=lambda c: float(np.sum((feet_rel(clip(c)['W'][:1])[0] - feat) ** 2)))
                switch(best, False, 0.0)
                state = 'stop'
            elif state == 'stop' and cur.done():
                switch('fpp_idle_stand', True, 0.0)
                state = 'idle'
            elif state not in ('idle', 'stop'):
                switch('fpp_idle_stand', True, 0.0)
                state = 'idle'
        elif want == 'move':
            sprint = fd['loco'][i] == 3 and abs(ang) < 60.0
            if state in ('idle', 'stop') and not (state == 'stop' and spd[i] > 2.0):
                tgt_speed = 3.84 if fd['loco'][i] == 3 else (1.6 if stick[i] < 0.6 else 3.84)
                g = 'walk' if tgt_speed < 2.5 else 'jog'
                switch('fpp_idle_to_%s_%s' % (g, dir4(ang)), False, 0.0)
                state = 'start'
            elif state == 'start' and not cur.done() and cur.t < clip(cur.name)['n'] - 1 - xf_frames:
                pass
            else:
                if sprint:
                    name = 'fpp_sprint_0'
                else:
                    gaits = ['walk_slow', 'walk_fast', 'jog']
                    g = min(gaits, key=lambda k: abs(np.hypot(*natural(LOOPS[k][0])) - spd[i]))
                    name = nearest(LOOPS[g], ang)
                rate = max(0.3, float(spd[i] / max(np.hypot(*natural(name)), 0.1)))
                switch_loop(name, rate)
                if cur.name in loop_names:
                    cur.rate = max(0.3, float(spd[i] / max(np.hypot(*natural(cur.name)), 0.1)))
                state = 'move'
        elif want in ('crouch_idle', 'crouch_move'):
            if want == 'crouch_idle':
                if cur.name != 'fpp_crouch_idle':
                    switch('fpp_crouch_idle', True, 0.0)
            else:
                name = nearest(LOOPS['crouch'], ang)
                rate = max(0.3, float(spd[i] / max(np.hypot(*natural(name)), 0.1)))
                switch_loop(name, rate)
                if cur.name in loop_names:
                    cur.rate = max(0.3, float(spd[i] / max(np.hypot(*natural(cur.name)), 0.1)))
            state = want
        # ---- evaluate (crossfade outgoing -> incoming)
        lt, lr = cur.pose()
        if prev is not None and fade < xf_frames:
            w = NBsstep(min(1.0, (fade + 1.0) / xf_frames))
            plt_, plr = prev.pose()
            lt = (1 - w) * plt_ + w * lt
            lr = LC.qslerp(plr, lr, np.full(lr.shape[0], w))
            prev.step()
            fade += 1.0
            if fade >= xf_frames:
                prev = None
        out_lt[i], out_lr[i] = lt, lr
        last_pose = (lt, lr)
        labels.append(cur.name)
        cur.step()
    Wm = MC.fk_world(P.par, out_lt, out_lr)
    W = LC.place(Wm[:, :73], fd['x'], fd['y'], fd['yaw'])
    return W, labels, fd


class Frozen:
    """a held output pose (outgoing side of a crossfade started during another crossfade)."""

    def __init__(self, lt, lr):
        self.lt, self.lr, self.name = lt, lr, '<pose>'

    def pose(self):
        return self.lt, self.lr

    def step(self):
        pass


def NBsstep(x):
    x = float(np.clip(x, 0.0, 1.0))
    return x * x * (3 - 2 * x)


if __name__ == '__main__':
    if sys.argv[1:] and sys.argv[1] == 'survey':
        survey()
