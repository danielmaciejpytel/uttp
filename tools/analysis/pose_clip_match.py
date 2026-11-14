"""Matches recorded in-game poses against animation clips frame by frame and scores a recorded pose with the player-animation survey metrics.

Library (import it) plus a small command line:

  python -I pose_clip_match.py match REC [--from HH:MM:SS --to HH:MM:SS] --bank DIR [DIR ...] [--stride N] [--stage 0|1] [--out FILE.json]
  python -I pose_clip_match.py csv-episodes CSV [--out FILE.json]

Recordings
  * compact recording (utpp-pose-*.utpppose): 185 catalogue bones, stream 0 holds the pose before the mod writes (stage 0) and after
    its writes (stage 1); read through pose_helper_report / pose_capture_to_csv.
  * text recording (utpp-pose-<ticks>.csv, written only with the opt-in marker): rows per bone and frame with the game pose (s0), the
    pose after the motion blend (s1) and the pose after every later write (s2), plus the action / overlay columns of the frame
    (action 2 = reload, 8 = shot, 3 = quick melee; overlay 1 rifle, 2 / 3 one / two hand pistol, 5 game owned). The text format names no
    bones: a row carries the game rig index, which this tool maps to a name with the index table of a compact recording of the same rig
    (the first 185 catalogue bones have the same index on both bodies).
Local rotations are compared as they are stored (engine local frame). Clip dumps are npz files with `names` (73 survey joints) and
`lr` [frames, 73, 4] xyzw local rotations in the same frame, written by dump_glb() or by the survey dump tool.

Match distance = mean over the compared bones of the rotation angle (deg) between the recorded local rotation and the clip local rotation;
a clip is only named when its best frame beats every other clip by `margin` degrees and stays under `max_resid` degrees (otherwise 'no match').
Read-only on every input; writes only the files given with --out / dump_glb(out_dir).
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# bones compared by default: the pose of the pelvis, the spine, the legs and the arm chains (no head / neck: look-at layers, no fingers)
BODY_BONES = ('Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot')
ARM_BONES = ('LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm', 'LeftForeArm', 'RightForeArm')
MATCH_BONES = BODY_BONES + ARM_BONES


# ------------------------------------------------------------------------------------------------------------------ math
def quat_angle_deg(a, b):
    """Rotation angle in degrees between xyzw quaternions (broadcasting), sign independent."""
    a = np.asarray(a, np.float64)
    b = np.asarray(b, np.float64)
    a = a / np.maximum(np.linalg.norm(a, axis=-1, keepdims=True), 1e-12)
    b = b / np.maximum(np.linalg.norm(b, axis=-1, keepdims=True), 1e-12)
    d = np.abs(np.sum(a * b, axis=-1))
    return np.degrees(2.0 * np.arccos(np.clip(d, -1.0, 1.0)))


# ------------------------------------------------------------------------------------------------------------------ clip bank
class ClipBank:
    """Frames of many clips as one array: q [M, B, 4] (the compared bones), with the (label, clip, first frame, count) of each clip."""

    def __init__(self, bones=MATCH_BONES):
        self.bones = tuple(bones)
        self.labels, self.clips, self.starts, self.counts, self.fps = [], [], [], [], []
        self._q = []
        self.q = np.zeros((0, len(self.bones), 4), np.float32)

    def add(self, label, clip, names, lr, fps=30.0):
        ix = {str(n): i for i, n in enumerate(names)}
        miss = [b for b in self.bones if b not in ix]
        if miss:
            raise ValueError('clip %s/%s lacks bones %s' % (label, clip, miss))
        q = np.asarray(lr, np.float32)[:, [ix[b] for b in self.bones], :]
        self.labels.append(label)
        self.clips.append(clip)
        self.starts.append(sum(len(x) for x in self._q))
        self.counts.append(len(q))
        self.fps.append(float(fps))
        self._q.append(q)

    def finish(self):
        self.q = np.concatenate(self._q, 0) if self._q else self.q
        self.starts = np.asarray(self.starts, np.int64)
        self.counts = np.asarray(self.counts, np.int64)
        return self

    def add_dump_dir(self, root, rig, name_filter=None, label_prefix='', clip_filter=None):
        """root/<rig>/<set>/<clip>.npz (survey layout) or root/<set>/<clip>.npz (flat, dump_glb layout)."""
        base = os.path.join(root, rig)
        if not os.path.isdir(base):
            base = root
        for setdir in sorted(os.listdir(base)):
            d = os.path.join(base, setdir)
            if not os.path.isdir(d) or (name_filter and not name_filter(setdir)):
                continue
            for p in sorted(glob.glob(os.path.join(d, '*.npz'))):
                if clip_filter and not clip_filter(os.path.basename(p)[:-4]):
                    continue
                z = np.load(p, allow_pickle=False)
                names = [str(x) for x in z['names']] if 'names' in z.files else None
                if names is None or 'lr' not in z.files:
                    continue
                if 'names152' in z.files:        # survey dump: locals of the 152 export bones
                    names = [str(x) for x in z['names152']]
                self.add(label_prefix + setdir, os.path.basename(p)[:-4], names, z['lr'], float(z['fps']) if 'fps' in z.files else 30.0)
        return self

    def best_per_clip(self, rec_q, chunk=24):
        """rec_q [n, B, 4] -> (dist [n, nclips] minimum over the frames of each clip, frame [n, nclips])."""
        n = len(rec_q)
        nc = len(self.counts)
        dist = np.empty((n, nc), np.float32)
        frm = np.empty((n, nc), np.int32)
        qb = self.q                                   # [M, B, 4]
        for a in range(0, n, chunk):
            r = rec_q[a:a + chunk].astype(np.float32)
            r = r / np.maximum(np.linalg.norm(r, axis=-1, keepdims=True), 1e-12)
            dot = np.abs(np.einsum('nbk,mbk->nmb', r, qb, optimize=True))                # [c, M, B]
            prox = (1.0 - dot).sum(-1)                                                      # monotonic in the angle: picks each clip's best frame
            rows = np.arange(len(r))
            for c in range(nc):
                s, k = self.starts[c], self.counts[c]
                j = prox[:, s:s + k].argmin(1)
                frm[a:a + len(r), c] = j
                d = dot[rows, s + j, :]                                                       # [c, B]
                dist[a:a + len(r), c] = np.degrees(2.0 * np.arccos(np.clip(d, 0.0, 1.0))).mean(-1)
        return dist, frm


def match_recording(rec_q, bank, margin=1.0, max_resid=8.0):
    """Per recorded frame: best clip, its frame, residual (deg), the best other-clip residual and the verdict.
    Clips of the same set with identical data (idle / tpp_idle) are reported through `ties` (clips within `margin` of the best)."""
    dist, frm = bank.best_per_clip(rec_q)
    order = np.argsort(dist, axis=1)
    out = []
    for i in range(len(rec_q)):
        b = int(order[i, 0])
        best = float(dist[i, b])
        ties = [int(c) for c in order[i, 1:8] if dist[i, c] - best <= margin]
        second = float(dist[i, order[i, 1]]) if dist.shape[1] > 1 else float('nan')
        out.append(dict(clip_index=b, set=bank.labels[b], clip=bank.clips[b], frame=int(frm[i, b]), resid_deg=round(best, 3),
                        second_resid_deg=round(second, 3), margin_deg=round(second - best, 3),
                        ties=[(bank.labels[c], bank.clips[c]) for c in ties],
                        verdict='match' if best <= max_resid else 'no match'))
    return out


# ------------------------------------------------------------------------------------------------------------------ recordings
class Recording:
    """Local pose of the 73 survey joints per frame and stage: lt [n, S, 73, 3] metres, lr [n, S, 73, 4] xyzw, sec [n] (local clock)."""

    def __init__(self, names, sec, lt, lr, info=None):
        self.names, self.sec, self.lt, self.lr, self.info = list(names), np.asarray(sec), lt, lr, info or {}

    def bone_q(self, bones, stage=0):
        ix = {n: i for i, n in enumerate(self.names)}
        return self.lr[:, stage][:, [ix[b] for b in bones], :]


def load_compact(path, t_from=None, t_to=None, names73=None, stages=(0, 1)):
    """Compact recording -> Recording of stream 0 (stage 0 = before the mod, stage 1 = after); window in local seconds since midnight."""
    import pose_helper_report as ph
    g = ph.load_groups([path], ph.Config())[0]
    h = g['header']
    mapped = h.mapped()
    names = [h.names[i] for i in mapped]
    keep = [names.index(n) for n in names73] if names73 else list(range(len(names)))
    secs, vals = [], []
    for stream, ticks, sec, gidx, v in ph.iter_blocks(g):
        if stream != 0:
            continue
        m = np.ones(len(sec), bool)
        if t_from is not None:
            m &= sec >= t_from
        if t_to is not None:
            m &= sec <= t_to
        if not m.any():
            continue
        secs.append(sec[m])
        vals.append(v[m][:, keep][:, :, list(stages), :].astype(np.float32))
    if not secs:
        raise ValueError('no frames in the window')
    sec = np.concatenate(secs)
    v = np.concatenate(vals)                                   # [n, nb, S, 12]
    lt = np.transpose(v[..., 0:3], (0, 2, 1, 3))               # [n, S, nb, 3]
    lr = np.transpose(v[..., 4:8], (0, 2, 1, 3))
    return Recording([names[i] for i in keep], sec, lt, lr, dict(rig_count=int(h.rig_count), day=g['day'], path=os.path.basename(path),
                                                                  qpc_hz=int(h.qpc_hz)))


_CSV_COLS = ['ticks', 'action', 'overlay', 'aiming', 'clip', 'weight', 'upper', 'bone', 'gameIndex']


def csv_frame_table(path, chunk_rows=3_000_000):
    """Text recording -> per frame arrays (ticks, action, overlay, aiming, clip, weight, upper) from the bone-0 rows."""
    import pandas as pd
    parts = []
    for ch in pd.read_csv(path, comment='#', usecols=_CSV_COLS, chunksize=chunk_rows, engine='c'):
        parts.append(ch[ch['bone'] == 0].drop(columns=['bone', 'gameIndex']))
    f = pd.concat(parts).drop_duplicates('ticks') if parts else None
    return f


def csv_episodes(path, action=2, gap_s=0.5):
    """Episodes of consecutive frames whose action column equals `action`: [dict(t0, t1 ticks, n, overlay, clip, qpc_hz)]."""
    f = csv_frame_table(path)
    qpc = 1e7
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        first = fh.readline()
    if 'qpc_hz=' in first:
        qpc = float(first.split('qpc_hz=')[1].split()[0])
    tk = f['ticks'].to_numpy()
    idx = np.nonzero(f['action'].to_numpy() == action)[0]
    if not len(idx):
        return [], qpc
    cuts = np.nonzero(np.diff(tk[idx]) > gap_s * qpc)[0]
    starts = np.r_[0, cuts + 1]
    ends = np.r_[cuts, len(idx) - 1]
    eps = []
    for a, b in zip(starts, ends):
        ia, ib = idx[a], idx[b]
        eps.append(dict(t0=int(tk[ia]), t1=int(tk[ib]), n=int(b - a + 1), overlay=int(f['overlay'].iloc[ia]), clip=int(f['clip'].iloc[ia]),
                        dur_s=float((tk[ib] - tk[ia]) / qpc)))
    return eps, qpc


def load_csv_windows(path, windows, index_to_name, names73, qpc_hz=1e7):
    """Rows of the text recording inside each (t0, t1) tick window -> one Recording per window (read once), stages 0 (game pose),
    1 (after the motion blend), 2 (after every later write). info carries per frame action / overlay / weight / upper."""
    import pandas as pd
    cols = ['ticks', 'action', 'overlay', 'weight', 'upper', 'gameIndex'] + ['s%d_%d' % (s, k) for s in range(3) for k in range(12)]
    parts = [[] for _ in windows]
    for ch in pd.read_csv(path, comment='#', usecols=cols, chunksize=3_000_000, engine='c'):
        tk = ch['ticks'].to_numpy()
        for i, (t0, t1) in enumerate(windows):
            m = (tk >= t0) & (tk <= t1) & (ch['gameIndex'].to_numpy() >= 0)
            if m.any():
                parts[i].append(ch[m])
    nix = {nm: i for i, nm in enumerate(names73)}
    out = []
    for p in parts:
        if not p:
            out.append(None)
            continue
        d = pd.concat(p)
        ticks, first = np.unique(d['ticks'].to_numpy(), return_index=True)
        tix = {int(t): i for i, t in enumerate(ticks)}
        n, nb = len(ticks), len(names73)
        lt = np.full((n, 3, nb, 3), np.nan, np.float32)
        lr = np.full((n, 3, nb, 4), np.nan, np.float32)
        ti = np.array([tix[int(t)] for t in d['ticks'].to_numpy()])
        bi = np.array([nix.get(index_to_name.get(int(g), ''), -1) for g in d['gameIndex'].to_numpy()])
        ok = bi >= 0
        for s in range(3):
            lt[ti[ok], s, bi[ok], :] = d[['s%d_%d' % (s, k) for k in range(3)]].to_numpy()[ok]
            lr[ti[ok], s, bi[ok], :] = d[['s%d_%d' % (s, k) for k in range(4, 8)]].to_numpy()[ok]
        info = dict(path=os.path.basename(path), ticks=ticks)
        for col in ('action', 'overlay', 'weight', 'upper'):
            info[col] = d[col].to_numpy()[first]
        out.append(Recording(names73, ticks / qpc_hz, lt, lr, info))
    return out


def load_csv_window(path, t0, t1, index_to_name, names73, qpc_hz=1e7):
    r = load_csv_windows(path, [(t0, t1)], index_to_name, names73, qpc_hz)[0]
    if r is None:
        raise ValueError('no rows in the window')
    return r


def index_name_table(compact_path):
    """game rig index -> bone name of the catalogue bones, from the header of a compact recording (identical on both bodies)."""
    import pose_helper_report as ph
    h = ph.read_header(compact_path)
    return {int(h.game_index[i]): h.names[i] for i in range(len(h.names)) if int(h.game_index[i]) >= 0}


# ------------------------------------------------------------------------------------------------------------------ clip export
def _survey_path():
    p = os.path.abspath(os.path.join(HERE, '..', 'anim', 'survey'))
    if p not in sys.path:
        sys.path.insert(0, p)


def dump_glb(glb, json_path, out_dir, rig, clip_rx=None):
    """glb of one animation set -> out_dir/<set>/<clip>.npz with names (73 survey joints), lr / lt local frames (engine axes), fps.
    Additive clips are skipped. Needs the melee / vehicle tools on sys.path (see survey_common.add_tool_paths)."""
    import re
    os.environ['UTPP_RIG'] = rig
    _survey_path()
    import survey_common as SC
    SC.add_tool_paths()
    from scipy.spatial.transform import Rotation as Rot
    import game_skeleton_bridge as CP
    import glbanim as GA
    import melee_common as MC
    import skel as S
    names73, _par, _ref = CP.load_rig()
    sk = S.Skeleton.from_glb(glb)
    js = json.load(open(json_path, encoding='utf-8')) if json_path else None
    frames_json = {}
    if js:
        for e in js['Data']['RootChunk']['animations']:
            a = e['Data']['animation']['Data']
            frames_json[a['name']['$value']] = (a['animBuffer']['Data']['numFrames'], a['duration'], a.get('animationType', ''))
    setkey = os.path.basename(glb)[:-len('.anims.glb')] if glb.endswith('.anims.glb') else os.path.basename(glb)
    od = os.path.join(out_dir, setkey)
    os.makedirs(od, exist_ok=True)
    cl = GA.clips(sk.gj)
    written = []
    for clip, ai in cl.items():
        ex = sk.gj['animations'][ai].get('extras', {})
        nf, dur, atype = frames_json.get(clip, (0, 0.0, ex.get('animationType')))
        atype = ex.get('animationType') or atype
        if atype in ('Additive', 'AdditiveWithoutFirstFrame') or (clip_rx and not re.search(clip_rx, clip)):
            continue
        ch = GA.channels(sk.gj, sk.binb, ai)
        if not nf:
            nf = max((len(d['rotation'][0]) for d in ch.values() if 'rotation' in d), default=0)
        if nf < 2:
            continue
        times = np.arange(int(nf)) / 30.0
        lt = np.repeat(sk.rest_t[None], int(nf), 0).copy()
        lr = np.repeat(sk.rest_r[None], int(nf), 0).copy()
        for b, d in ch.items():
            if b in sk.index:
                i = sk.index[b]
                if 'translation' in d:
                    lt[:, i] = GA.sample(d['translation'], times, 'translation')
                if 'rotation' in d:
                    lr[:, i] = GA.sample(d['rotation'], times, 'rotation')
        Rl = S.q2m(lr)
        L = np.tile(np.eye(4), (int(nf), len(sk.names), 1, 1))
        L[:, :, :3, :3] = Rl
        L[:, :, :3, 3] = lt
        Lg = MC.CI[None, None] @ L @ MC.C[None, None]
        have = [j for j, n in enumerate(names73) if n in sk.index]
        if len(have) < len(names73) - 4:
            return []                                          # a rig that lacks more than the hand tips: not a player-style skeleton
        ix = [sk.index[names73[j]] for j in have]
        q = np.zeros((int(nf), len(names73), 4), np.float32)
        q[..., 3] = 1.0                                        # joints a rig lacks (hand tips of the non-player rigs): identity
        t = np.zeros((int(nf), len(names73), 3), np.float32)
        q[:, have] = Rot.from_matrix(Lg[:, ix][:, :, :3, :3].reshape(-1, 3, 3)).as_quat().reshape(int(nf), len(ix), 4)
        t[:, have] = Lg[:, ix][:, :, :3, 3]
        np.savez_compressed(os.path.join(od, clip + '.npz'), names=np.array(names73), lr=q, lt=t, fps=30.0,
                            missing=np.array([n for n in names73 if n not in sk.index]))
        written.append(clip)
    return written


# ------------------------------------------------------------------------------------------------------------------ survey metrics
def survey_measure(lt, lr, rig, mode='full'):
    """Local pose [T, 73, 3] / [T, 73, 4] -> (features, per-feature exceedances, summary) with the survey's own Ctx.measure / summarize
    (thresholds unchanged). Imports the survey modules lazily (they read the rig from the command line / environment once)."""
    sys.argv = [sys.argv[0], '--rig', rig]
    _survey_path()
    import survey_metrics as SM
    ctx = SM.Ctx()
    import melee_common as MC
    W = MC.fk_world(ctx.par, np.asarray(lt, float), np.asarray(lr, float))
    f, ex = ctx.measure(W, legs=True)
    return f, ex, SM.summarize(ex, len(W), SM.glitch_mask(W, ctx.par)), ctx


# ------------------------------------------------------------------------------------------------------------------ command line
def _clock_to_sec(text):
    h, m, s = text.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    m = sub.add_parser('match')
    m.add_argument('rec')
    m.add_argument('--from', dest='t_from')
    m.add_argument('--to', dest='t_to')
    m.add_argument('--bank', nargs='+', required=True)
    m.add_argument('--rig', default='woman')
    m.add_argument('--stride', type=int, default=6)
    m.add_argument('--stage', type=int, default=0)
    m.add_argument('--bones', choices=('all', 'arms'), default='all', help="'arms' = shoulders, arms, forearms, hands (the bones a weapon action owns)")
    m.add_argument('--set-regex', help='only clip sets whose folder name matches')
    m.add_argument('--clip-regex', help='only clips whose name matches')
    m.add_argument('--out')
    e = sub.add_parser('csv-episodes')
    e.add_argument('csv')
    e.add_argument('--action', type=int, default=2)
    e.add_argument('--out')
    a = ap.parse_args(argv)
    if a.cmd == 'csv-episodes':
        eps, qpc = csv_episodes(a.csv, a.action)
        txt = json.dumps(dict(csv=os.path.basename(a.csv), qpc_hz=qpc, episodes=eps), indent=1)
        if a.out:
            with open(a.out, 'w', encoding='utf-8') as f:
                f.write(txt)
        else:
            print(txt)
        return 0
    import re
    bones = MATCH_BONES if a.bones == 'all' else ARM_BONES + ('LeftHand', 'RightHand')
    bank = ClipBank(bones)
    for d in a.bank:
        bank.add_dump_dir(d, a.rig, (lambda n: re.search(a.set_regex, n)) if a.set_regex else None,
                          clip_filter=(lambda n: re.search(a.clip_regex, n)) if a.clip_regex else None)
    bank.finish()
    rec = load_compact(a.rec, _clock_to_sec(a.t_from) if a.t_from else None, _clock_to_sec(a.t_to) if a.t_to else None, stages=(0, 1))
    q = rec.bone_q(bones, a.stage)[::a.stride]
    res = match_recording(q, bank)
    txt = json.dumps(dict(rec=a.rec, frames=len(q), results=res), indent=1)
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            f.write(txt)
    else:
        print(txt)
    return 0


if __name__ == '__main__':
    sys.exit(main())
