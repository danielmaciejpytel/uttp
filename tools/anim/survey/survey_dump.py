"""glb (WolvenKit export, root motion included) -> per-clip 73-joint world dumps of the survey (layout of Source\\tools\\melee\\melee_dump.py).

  python -I survey_dump.py <depot path of the set> [--rig woman|man]     one set, every non-additive clip
  python -I survey_dump.py --list <file> --jobs 6                         every set of the list, one subprocess per set
Output: Build\\work\\anim-survey-20261109\\dumps\\<rig>\\<setkey>\\<clip>.npz + _index.json (clip -> frames, additive, fpp camera, ...).
Differences to melee_dump.py (which is not edited): own paths, joints mapped BY NAME onto the 73-joint V rig (sets on other rigs are
accepted when all 73 names exist), additive clips are listed but not dumped (a delta is no pose), and the Torso_fppCamera_* evidence
(tracks present / animated) is stored per clip.  Same sampling: 30 fps, frame i at t = i / 30, frame count = serialized numFrames.
"""
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_common as SC  # noqa: E402

FPP_BONES = ['Torso_fppCamera_Control_GRP', 'Torso_fppCamera_Aim_JNT', 'Torso_fppCamera_Target_JNT', 'Torso_fppCamera_UpOffset_GRP',
             'Torso_fppCamera_Up_GRP']


def setkey(depot):
    d = depot.lower().replace('/', '\\')
    k = os.path.basename(d)[:-len('.anims')]
    if d.startswith('ep1\\'):
        k += '.ep1'
    if '\\player\\male\\' in d:
        k += '.m'
    elif '\\player\\female\\' in d:
        k += '.f'
    return k


def rig_of_set(depot, js=None):
    js = js or SC.load_set_json(depot)
    rig = js['Data']['RootChunk']['rig']['DepotPath']['$value'] if js else ''
    if 'player_man_skeleton' in rig or 'man_base' in rig and 'woman' not in rig:
        return 'man', rig
    if 'woman' in rig:
        return 'woman', rig
    b = os.path.basename(depot.lower())
    return ('man' if b.startswith('pma_') or '\\male\\' in depot.lower() else 'woman'), rig


def dump_set(depot, clip_rx=None):
    rig_name, rig_path = rig_of_set(depot)
    os.environ['UTPP_RIG'] = rig_name
    SC.add_tool_paths()
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot
    import game_skeleton_bridge as CP
    import glbanim as GA
    import melee_common as MC
    import skel as S
    assert CP.RIG_NAME == rig_name
    glb = os.path.join(SC.GLB, depot + '.glb')
    js = SC.load_set_json(depot)
    sk = S.Skeleton.from_glb(glb)
    names73, par73, ref73 = CP.load_rig()
    rest73 = CP.rest_world(par73, ref73)
    names = list(sk.names)
    missing = [n for n in names73 if n not in sk.index]
    out_dir = os.path.join(SC.DUMPS, rig_name, setkey(depot))
    os.makedirs(out_dir, exist_ok=True)
    if missing:
        SC.jdump(dict(error='rig joints missing', missing=missing, rig=rig_path), os.path.join(out_dir, '_index.json'))
        print('SURVEY_DUMP %s: %d of the 73 joints missing (rig %s)' % (depot, len(missing), rig_path))
        return
    m73 = [sk.index[n] for n in names73]
    frames_json = {}
    for e in js['Data']['RootChunk']['animations']:
        a = e['Data']['animation']['Data']
        frames_json[a['name']['$value']] = (a['animBuffer']['Data']['numFrames'], a['duration'], a.get('animationType', ''))
    cl = GA.clips(sk.gj)
    index = {}
    t0 = time.time()
    rl = np.eye(4)[None].repeat(len(names), 0)
    rl[:, :3, :3] = S.q2m(sk.rest_r)
    rl[:, :3, 3] = sk.rest_t
    rlg = MC.CI[None] @ rl @ MC.C[None]
    for clip, ai in cl.items():
        ex = sk.gj['animations'][ai].get('extras', {})
        nf_json, dur_json, atype = frames_json.get(clip, (0, 0.0, ex.get('animationType')))
        atype = ex.get('animationType') or atype
        additive = atype in ('Additive', 'AdditiveWithoutFirstFrame')
        ch = GA.channels(sk.gj, sk.binb, ai)
        fpp_tracks = sorted(b for b in ch if b in FPP_BONES)
        entry = dict(frames=int(nf_json), duration=float(dur_json), animation_type=atype, additive=additive, root_motion=ex.get('rootMotionType'),
                     fpp_cam_tracks=fpp_tracks, n_tracks=len(ch))
        if additive or int(nf_json) < 2 or (clip_rx and not re.search(clip_rx, clip)):
            entry['skipped'] = 'additive' if additive else ('short' if int(nf_json) < 2 else 'filter')
            index[clip] = entry
            continue
        nf = int(nf_json)
        times = np.arange(nf) / SC.FPS
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
        Wg = np.tile(np.eye(4), (nf, 73, 1, 1))
        for j, i in enumerate(m73):
            g = np.tile(np.eye(4), (nf, 1, 1))
            g[:, :3, :3] = Wr[:, i]
            g[:, :3, 3] = Wt[:, i]
            Wg[:, j] = MC.CI @ g @ MC.C
        fpp_anim = False
        for b in fpp_tracks:
            i = sk.index[b]
            if np.ptp(lr[:, i], axis=0).max() > 1e-3 or np.ptp(lt[:, i], axis=0).max() > 1e-3:
                fpp_anim = True
        lt_g = np.zeros_like(lt)
        lr_g = np.zeros_like(lr)
        Rl = S.q2m(lr)
        for i in range(len(names)):
            L = np.tile(np.eye(4), (nf, 1, 1))
            L[:, :3, :3] = Rl[:, i]
            L[:, :3, 3] = lt[:, i]
            Lg = MC.CI @ L @ MC.C
            lt_g[:, i] = Lg[:, :3, 3]
            lr_g[:, i] = Rot.from_matrix(Lg[:, :3, :3]).as_quat()
        entry.update(fpp_cam_animated=fpp_anim, dumped=True)
        meta = dict(set=depot, setkey=setkey(depot), clip=clip, rig=rig_name, rig_path=rig_path, **entry)
        np.savez_compressed(os.path.join(out_dir, clip + '.npz'), W=Wg.astype(np.float32), names=np.array(names73), parents=par73, rest=rest73,
                            fps=30, lt=lt_g.astype(np.float32), lr=lr_g.astype(np.float32), names152=np.array(names),
                            parents152=np.array(sk.parent), rest_lt=rlg[:, :3, 3], rest_lr=Rot.from_matrix(rlg[:, :3, :3]).as_quat(),
                            meta=json.dumps(meta))
        index[clip] = entry
    SC.jdump(dict(set=depot, rig=rig_name, rig_path=rig_path, clips=index), os.path.join(out_dir, '_index.json'))
    print('SURVEY_DUMP %s %s %d clips (%d dumped) %.1f s' % (rig_name, depot, len(index), sum(1 for v in index.values() if v.get('dumped')),
                                                          time.time() - t0))


def main(a):
    if a[0] == '--list':
        items = [l.rstrip('\r\n').split('\t') for l in open(a[1], encoding='utf-8') if l.strip()]
        jobs = int(a[a.index('--jobs') + 1]) if '--jobs' in a else 6
        procs, bad = [], []
        for it in items:
            depot, rx = it[0].strip(), (it[1].strip() if len(it) > 1 else '.')
            if not os.path.exists(os.path.join(SC.GLB, depot + '.glb')):
                print('NO GLB', depot)
                continue
            while len([p for _, p in procs if p.poll() is None]) >= jobs:
                time.sleep(0.3)
            procs.append((depot, subprocess.Popen([SC.PY, '-I', os.path.abspath(__file__), depot, '--clips', rx])))
        for depot, p in procs:
            if p.wait() != 0:
                bad.append(depot)
        print('SURVEY_DUMP_ALL done, failed: %s' % bad)
        sys.exit(1 if bad else 0)
    dump_set(a[0], a[a.index('--clips') + 1] if '--clips' in a else None)


if __name__ == '__main__':
    main(sys.argv[1:])
