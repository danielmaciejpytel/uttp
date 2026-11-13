"""Dump every clip of the 12 vanilla melee animsets (WolvenKit .anims.glb) to per-clip npz (layout: melee_common.py).

  python -I melee_dump.py <set> [--clip NAME ...]      one set (all clips, or only the named ones)
  python -I melee_dump.py all [--jobs 12]              every set, one subprocess per set (UTPP_RIG is read by game_skeleton_bridge at
                                                       import time, so pma_* sets get UTPP_RIG=man in their own process)
Output: Build\\work\\melee-anim-20261107\\npz\\<set>\\<clip>.npz and npz\\<set>\\_index.json (clip -> frames, additive, ...).
Sampling: 30 fps, frame i at t = i / 30 (frame count from the clip's own key times, checked against the serialized .anims.json
numFrames). The Root track of root-motion clips has NORMALISED key times (WolvenKit export) and is rescaled to seconds here
(skel.sample_local only does that for clips longer than 1 s). Root motion of the vanilla melee sets is only a few cm (7 fists
clips carry a spline, all others none).
"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402


def dump_set(set_name, only=None):
    os.environ['UTPP_RIG'] = MC.RIG_OF_SET[set_name]
    ROOT = MC.ROOT
    sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'ai_motion'))
    sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry'))
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot
    import game_skeleton_bridge as CP
    import glbanim as GA
    import skel as S

    glb = os.path.join(MC.WORK, 'glb', set_name + '.anims.glb')
    sk = S.Skeleton.from_glb(glb)
    names73, par73, ref73 = CP.load_rig()
    rest73 = CP.rest_world(par73, ref73)
    assert sk.names[:73] == names73, 'glb joint order differs from the rig for %s' % set_name
    names152 = list(sk.names)
    par152 = np.array(sk.parent)
    jsn = json.load(open(os.path.join(MC.WORK, 'json', set_name + '.anims.json'), encoding='utf-8'))
    frames_json = {}
    for e in jsn['Data']['RootChunk']['animations']:
        a = e['Data']['animation']['Data']
        frames_json[a['name']['$value']] = (a['animBuffer']['Data']['numFrames'], a['duration'])
    rig_path = jsn['Data']['RootChunk']['rig']['DepotPath']['$value']
    assert ('player_man_skeleton' in rig_path) == (MC.RIG_OF_SET[set_name] == 'man'), (set_name, rig_path)
    cl = GA.clips(sk.gj)
    out_dir = os.path.join(MC.WORK, 'npz', set_name)
    os.makedirs(out_dir, exist_ok=True)
    index = {}
    t0 = time.time()
    for clip, ai in cl.items():
        if only and clip not in only:
            continue
        ex = sk.gj['animations'][ai].get('extras', {})
        nf_json, dur_json = frames_json[clip]
        nf = int(nf_json)
        times = np.arange(nf) / MC.FPS
        ch = GA.channels(sk.gj, sk.binb, ai)
        bone_end = max(float(tr[0][-1]) for b, d in ch.items() if b != 'Root' for tr in d.values())
        assert bone_end < (nf - 1) / MC.FPS + 2.5 / MC.FPS, (clip, bone_end, nf)  # shorter key ranges happen (clips whose tracks end early hold their last value)
        rm = ex.get('rootMotionType')
        if 'Root' in ch and rm not in (None, 'None', 'Unknown'):
            for k in list(ch['Root'].keys()):
                t, v, it = ch['Root'][k]
                if float(t[-1]) <= 1.0001:
                    ch['Root'][k] = (t * dur_json, v, it)
        lt = np.repeat(sk.rest_t[None], nf, 0).copy()
        lr = np.repeat(sk.rest_r[None], nf, 0).copy()
        for b, d in ch.items():
            if b in sk.index:
                i = sk.index[b]
                if 'translation' in d:
                    lt[:, i] = GA.sample(d['translation'], times, 'translation')
                if 'rotation' in d:
                    lr[:, i] = GA.sample(d['rotation'], times, 'rotation')
        Wt, Wr = sk.world(lt, lr)
        Wg = np.tile(np.eye(4), (nf, len(names152), 1, 1))
        g = np.tile(np.eye(4), (nf, 1, 1))
        for i in range(len(names152)):
            g[:] = np.eye(4)
            g[:, :3, :3] = Wr[:, i]
            g[:, :3, 3] = Wt[:, i]
            Wg[:, i] = MC.CI @ g @ MC.C
        # local transforms in game axes: L_game = C^-1 L C
        lt_g = np.zeros_like(lt)
        lr_g = np.zeros_like(lr)
        Rl = S.q2m(lr)
        for i in range(len(names152)):
            L = np.tile(np.eye(4), (nf, 1, 1))
            L[:, :3, :3] = Rl[:, i]
            L[:, :3, 3] = lt[:, i]
            Lg = MC.CI @ L @ MC.C
            lt_g[:, i] = Lg[:, :3, 3]
            lr_g[:, i] = Rot.from_matrix(Lg[:, :3, :3]).as_quat()
        rl = np.eye(4)[None].repeat(len(names152), 0)
        rl[:, :3, :3] = S.q2m(sk.rest_r)
        rl[:, :3, 3] = sk.rest_t
        rlg = MC.CI[None] @ rl @ MC.C[None]
        ev = [{k: e[k] for k in ('type', 'eventName', 'startFrame', 'durationInFrames', 'actionName') if e.get(k) is not None}
              for e in (ex.get('animEvents') or [])]
        meta = dict(set=set_name, clip=clip, rig=MC.RIG_OF_SET[set_name], animation_type=ex.get('animationType'),
                    additive=ex.get('animationType') in ('Additive', 'AdditiveWithoutFirstFrame'),
                    root_motion_type=rm, frames=nf, duration=float(dur_json), events=ev if ev else [],
                    glb=os.path.basename(glb), key_end_s=bone_end)
        extra_idx = list(range(73, len(names152)))
        np.savez_compressed(os.path.join(out_dir, clip + '.npz'), W=Wg[:, :73], names=np.array(names73), parents=par73,
                            rest=rest73, fps=30, W_extra=Wg[:, extra_idx].astype(np.float32),
                            names_extra=np.array([names152[i] for i in extra_idx]), lt=lt_g.astype(np.float32),
                            lr=lr_g.astype(np.float32), names152=np.array(names152), parents152=par152,
                            rest_lt=rlg[:, :3, 3], rest_lr=Rot.from_matrix(rlg[:, :3, :3]).as_quat(),
                            meta=json.dumps(meta))
        index[clip] = dict(frames=nf, additive=meta['additive'], animation_type=meta['animation_type'],
                           root_motion=rm, duration=meta['duration'])
    idx_path = os.path.join(out_dir, '_index.json')
    old = json.load(open(idx_path)) if only and os.path.exists(idx_path) else {}
    old.update(index)
    MC.jdump(old, idx_path)
    print('MELEE_DUMP %s %d clips %.1f s' % (set_name, len(index), time.time() - t0))


def main(a):
    if a[0] == 'all':
        jobs = int(a[a.index('--jobs') + 1]) if '--jobs' in a else 12
        procs = []
        for s in MC.SETS:
            while len([p for p in procs if p.poll() is None]) >= jobs:
                time.sleep(0.5)
            procs.append(subprocess.Popen([MC.PY, '-I', os.path.abspath(__file__), s]))
        bad = [p.args[-1] for p in procs if p.wait() != 0]
        print('MELEE_DUMP_ALL done, failed: %s' % bad)
        sys.exit(1 if bad else 0)
    only = a[a.index('--clip') + 1:] if '--clip' in a else None
    dump_set(a[0], only)


if __name__ == '__main__':
    main(sys.argv[1:])
