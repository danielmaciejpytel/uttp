"""Hood slide with a weapon (2026-11-10) - comparison film 1x "przed" / "po" (skinned Female V, side camera) and worst-frame sheet.
Data: the recording of 2026-11-10 13:00 (utpp-bodytrace-2026-11-10-13-00-50.csv = the pose the game rig showed, all 73 core
bones; utpp-2026-11-10-13-00-01.log = release timeline). "przed" = the captured pose; "po" = the same frames through the production chain of
src/mm/VaultTorso.hpp (tests\\VaultTorsoFilmTool.cpp = utpp_vaulttorso_film_tool.exe, gate -> fade with exit hold -> limiter). The no-weapon vault
is shown on both sides unchanged (reference). The captured frames carry no car / root motion: the body plays in place, the camera follows the hips.
  python -I vaulttorso_film.py film   -> Media\\mp4\\2026-11-10_hood-slide-torso\\slizg_po_masce_przed_po_1x.mp4 (+ preview png in Media\\png)
  python -I vaulttorso_film.py sheet  -> Media\\png\\2026-11-10_hood-slide-torso\\slizg_po_masce_najgorsze_klatki_przed_po.png
"""
import json
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import weapons_common as WC  # noqa: E402
import weapons_fix_mesh as MESH  # noqa: E402
import weapons_film as WF  # noqa: E402
import weapons_fix_common as FC  # noqa: E402

ROOT = WC.ROOT
WORK = os.path.join(ROOT, 'Build', 'work', 'hood-slide-torso-20261110')
TMP = os.path.join(WORK, '_tmp')
FC.WORK = WORK
os.makedirs(TMP, exist_ok=True)
GAME_LOGS = os.path.join(ROOT, 'Research', 'logs', 'game')
TRACE = os.path.join(GAME_LOGS, 'utpp-bodytrace-2026-11-10-13-00-50.csv')
LOG = os.path.join(GAME_LOGS, 'utpp-2026-11-10-13-00-01.log')
TOOL = os.path.join(ROOT, 'Build', 'UTPP', 'utpp_vaulttorso_film_tool.exe')
MP4_DIR = os.path.join(ROOT, 'Media', 'mp4', '2026-11-10_hood-slide-torso')
PNG_DIR = os.path.join(ROOT, 'Media', 'png', '2026-11-10_hood-slide-torso')
BONES = 73
NAMES = ["Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1", "LeftLeg", "RightLeg", "Spine2",
         "LeftFoot", "RightFoot", "Spine3"]
PARENTS = [-1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10]
CAM = '3.0,0.0,0.25'     # side view from her right (+X), offset from the hips
TZ, LENS = 0.95, 36
# (label, release-on local time of the log, weapon in hand, caption)
CLIPS = [('rifle_vault_a', '13:03:31.940', 1, 'z karabinem'), ('rifle_vault_d', '13:05:12.593', 1, 'z karabinem'),
         ('rifle_vault_f', '13:05:37.888', 1, 'z karabinem'), ('unarmed_vault_c', '13:04:14.387', 0, 'bez broni (bez zmian)')]
BEFORE_S, AFTER_S = 0.35, 0.55


AJAX_DIR = os.path.join(ROOT, 'Extracted', 'game', 'weapon-mesh-ajax-20261109')


def ajax_mesh():
    """real Militech Ajax mesh (base body + standard magazine, as gripstock_common.weapon_mesh('ajax')), weapon frame, game axes."""
    import glob
    files = sorted(f for f in glob.glob(os.path.join(AJAX_DIR, '**', '*.glb'), recursive=True) if f.endswith(('base1_01.glb', 'mag_std_01.glb')))
    Vs, Fs, off = [], [], 0
    for f in files:
        js, binb = MESH.load(f)
        for m in js['meshes']:
            for p in m['primitives']:
                v = MESH.accessor(js, binb, p['attributes']['POSITION'])
                Vs.append(np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1))
                Fs.append(MESH.accessor(js, binb, p['indices']).reshape(-1, 3).astype(np.int64) + off)
                off += len(v)
    return np.concatenate(Vs), np.concatenate(Fs)


def secs(hms):
    h, m, s = hms.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def release_pairs():
    ev = []
    with open(LOG, 'r', encoding='utf8', errors='replace') as f:
        for line in f:
            if 'MMTRAV travRelease' in line:
                ev.append((secs(line[1:24].split(' ')[1]), line.split('travRelease')[1].split()[0] == 'on'))
    pairs, cur = [], None
    for t, on in ev:
        if on:
            cur = t
        elif cur is not None:
            pairs.append((cur, t))
            cur = None
    return pairs


def extract():
    """frames of every clip window from the body trace: {label: (t_rel[N], recs[N, 73, 7], on, off)}; t_rel = seconds from the release start."""
    with open(TRACE, 'r', encoding='utf8') as f:
        head = f.readline()
    m = re.search(r'arm_ticks=(\d+) utc=\d+-\d+-\d+T(\d+):(\d+):([\d.]+)Z', head)
    arm = int(m.group(1))
    arm_local = int(m.group(2)) * 3600 + int(m.group(3)) * 60 + float(m.group(4)) + 3600
    pairs = release_pairs()
    wins = []
    for label, hms, weapon, cap in CLIPS:
        on = secs(hms)
        off = [b for a, b in pairs if abs(a - on) < 0.01][0]
        wins.append((label, on, off))
    out = {w[0]: ([], []) for w in wins}
    lo, hi = min(w[1] for w in wins) - BEFORE_S - 0.1, max(w[2] for w in wins) + AFTER_S + 0.1
    cur, rec = None, {}

    def flush():
        if cur is None or len(rec) < BONES:
            return
        tl = arm_local + (cur - arm) / 1e7
        for label, on, off in wins:
            if on - BEFORE_S <= tl <= off + AFTER_S:
                out[label][0].append(tl - on)
                out[label][1].append(np.array([rec[i] for i in range(BONES)], float))

    with open(TRACE, 'r', encoding='utf8') as f:
        for line in f:
            if line[0] == '#' or line.startswith('ticks'):
                continue
            t = int(line[:line.find(',')])
            tl = arm_local + (t - arm) / 1e7
            if tl < lo:
                continue
            if tl > hi:
                break
            cc = line.split(',')
            if t != cur:
                flush()
                cur, rec = t, {}
            b = int(cc[7])
            if b < BONES:
                v = cc[20:32]  # stage s1 (the shown pose)
                rec[b] = (float(v[0]), float(v[1]), float(v[2]), float(v[4]), float(v[5]), float(v[6]), float(v[7]))
        flush()
    res = {}
    for label, on, off in wins:
        res[label] = (np.array(out[label][0]), np.array(out[label][1]), 0.0, off - on)
    return res


def run_tool(data, weapons):
    inp = os.path.join(TMP, 'tool_in.txt')
    outp = os.path.join(TMP, 'tool_out.txt')
    with open(inp, 'w') as o:
        o.write('rig 14\n')
        for i in range(14):
            o.write('bone %d %s %d\n' % (i, NAMES[i], PARENTS[i]))
        for label in data:
            t, recs, on, off = data[label]
            o.write('vault %s weapon=%d detail=9 release=1 on=%.5f off=%.5f frames=%d bones=%d\n' % (label, weapons[label], on, off, len(t), BONES))
            for k in range(len(t)):
                o.write('f %.5f ' % t[k] + ' '.join('%.8g' % x for x in recs[k].reshape(-1)) + '\n')
    p = subprocess.run([TOOL, inp, outp], capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('tool failed %d %s' % (p.returncode, p.stderr))
    res, cur = {}, None
    with open(outp) as f:
        for line in f:
            if line.startswith('vault '):
                cur = line.split()[1]
                res[cur] = []
            elif line.startswith('f ') and cur:
                v = line.split()
                res[cur].append(np.array(v[2:], float).reshape(BONES, 7))
    return {k: np.array(v) for k, v in res.items()}


def resample(t, k=30.0):
    """indices of the frames nearest to a uniform 30 fps time line (1x playback)."""
    grid = np.arange(t[0], t[-1], 1.0 / k)
    return np.array([int(np.argmin(np.abs(t - g))) for g in grid])


def world(recs):
    lt, lr = WC.full152(recs[:, :, :3], recs[:, :, 3:7])
    return WC.fk(lt, lr)


def elev(W, a, b):
    P = WC.player()
    d = W[:, P.ix[a], :3, 3] - W[:, P.ix[b], :3, 3]
    return np.degrees(np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1])))


def prepare():
    data = extract()
    weapons = {c[0]: c[2] for c in CLIPS}
    after = run_tool(data, weapons)
    clips = {}
    for label, hms, weapon, cap in CLIPS:
        t, recs, on, off = data[label]
        idx = resample(t)
        before = recs[idx]
        corr = after[label][idx]
        clips[label] = dict(t=t[idx], weapon=weapon, caption=cap, Wb=world(before), Wa=world(corr), on=on, off=off)
    return clips


def film():
    os.makedirs(MP4_DIR, exist_ok=True)
    os.makedirs(PNG_DIR, exist_ok=True)
    clips = prepare()
    V, F = ajax_mesh()
    parts = []
    for label, hms, weapon, cap in CLIPS:
        c = clips[label]
        gun = ('mesh', V, F) if weapon else [((0.0, 0.0, 0.0), (0.004, 0.004, 0.004))]
        mk = []
        for key, title in (('Wb', 'przed'), ('Wa', 'po')):
            mk.append(FC.render(c[key], 'film_%s_%s' % (label, key), '640,720', CAM, TZ, LENS, gun, labels=[cap] * len(c[key]), title=title,
                                follow=15, title_size='0.06'))
        pair = os.path.join(TMP, 'film_%s_pair.mkv' % label)
        WF.ffmpeg(['-y', '-i', mk[0], '-i', mk[1], '-filter_complex', '[0:v][1:v]hstack=inputs=2,format=yuv420p[v]', '-map', '[v]', '-r', '30',
                   '-c:v', 'libx264', '-crf', '18', '-preset', 'medium', pair])
        parts.append(pair)
    lst = os.path.join(TMP, 'film_concat.txt')
    with open(lst, 'w') as fh:
        for p in parts:
            fh.write("file '%s'\n" % p.replace('\\', '/'))
    out = os.path.join(MP4_DIR, 'slizg_po_masce_przed_po_1x.mp4')
    WF.ffmpeg(['-y', '-f', 'concat', '-safe', '0', '-i', lst, '-r', '30', '-c:v', 'libx264', '-crf', '20', '-preset', 'slow', '-pix_fmt', 'yuv420p',
               '-movflags', '+faststart', out])
    prev = os.path.join(PNG_DIR, 'slizg_po_masce_przed_po_1x_podglad.png')
    WF.ffmpeg(['-y', '-ss', '2.0', '-i', out, '-frames:v', '1', prev])
    print('MP4', out, os.path.getsize(out))
    print('PNG', prev)


def sheet():
    """worst frames on skin: per rifle clip the frame with the lowest chest (przed) and the same frame po, plus the first / last frame of the fade."""
    os.makedirs(PNG_DIR, exist_ok=True)
    clips = prepare()
    V, F = ajax_mesh()
    mk, info = [], []
    for label, hms, weapon, cap in CLIPS:
        c = clips[label]
        e0 = elev(c['Wb'], 'Spine3', 'Spine1')
        e1 = elev(c['Wa'], 'Spine3', 'Spine1')
        worst = int(np.argmin(e0))
        mid = int(np.argmax(np.abs(e1 - e0)))
        picks = list(dict.fromkeys([worst, mid, min(len(e0) - 1, worst + 4)]))
        Ws, labels = [], []
        for f in picks:
            Ws += [c['Wb'][f:f + 1], c['Wa'][f:f + 1]]
            labels += ['przed', 'po']
            info.append(dict(clip=label, frame=int(f), chest_before=round(float(e0[f]), 1), chest_after=round(float(e1[f]), 1)))
        Wc = np.concatenate(Ws)
        mk.append(FC.render(Wc, 'sheet_%s' % label, '420,460', CAM, TZ, LENS,
                            ('mesh', V, F) if weapon else [((0.0, 0.0, 0.0), (0.004, 0.004, 0.004))], labels=labels, follow=1))
    graph = []
    for i, x in enumerate(mk):
        graph.append('[%d:v]tile=6x1:padding=4:color=white[r%d]' % (i, i))
    args = ['-y']
    for x in mk:
        args += ['-i', x]
    out = os.path.join(PNG_DIR, 'slizg_po_masce_najgorsze_klatki_przed_po.png')
    WF.ffmpeg(args + ['-filter_complex', ';'.join(graph) + ';' + ''.join('[r%d]' % i for i in range(len(mk))) + 'vstack=inputs=%d[v]' % len(mk),
                      '-map', '[v]', '-frames:v', '1', out])
    print('PNG', out)
    print(json.dumps(info))


if __name__ == '__main__':
    film() if sys.argv[1] == 'film' else sheet()
