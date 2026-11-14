"""Skin displacement caused by stale helper bones: how far the body surface moves when the recorded helper bones are replaced by the
deformer graph, for the worst frames of each time window.

  python -I skin_displacement_report.py INPUT [INPUT ...] --out-dir DIR [--plan plan.json | --window-seconds N] [--frames-per-window 6]
                                        [--limit-male-cm 0.8] [--limit-female-cm 0.9] [--sheets all|flagged|none] [--jobs N]

INPUT, windows, bundle (owned.txt of --bundle-dir, default Build\\test-data\\body_helper_<body>), frame classes and exit codes are the ones of
helper_residual_report.py (read its description). Per window the N worst frames (largest position residual of a bundle bone,
mod-written frames first), plus the worst frame of every helper group outside the bundle, are skinned twice with linear blend skinning on the naked body meshes of the matching body (body, arms, head;
the meshes, weights and bindings of tools\\ai_motion\\skin_clip.py):
  before  every helper bone as recorded (stream 1 = after the runtime recompute when the recording has it, else stream 0 after the
          mod's write = what the engine would show without a recompute), core bones from the recording;
  after   every helper bone from the deformer graph evaluated on the same core pose.
Displacement = per vertex distance between the two skins, in cm. Since linear blend skinning is linear in the bone transforms, the share
of each helper group is exact: per group the vertices are moved by that group's helper bones only (all other bones unchanged). The
numbers per group: max over all vertices, p99 and mean over the vertices the group weights > 0.05, count of vertices over 0.5 and 1 cm.
A helper bone that is not in the recording (or whose ancestors are not all recorded) cannot be compared: it keeps its graph value in both
skins and is listed under `unmeasured_helper_bones`, so its group shows less than it may really move.

Groups (by bone name): upper (shoulder girdle, chest, deltoids), lower (thigh, knee, calf, buttock, groin), elbow, wrist_muscle, wrist_roll,
lower_leg_back, hand, sterno. A group is 'covered' when the bundle owns bones of it (the bundle of the current release: upper, lower, elbow,
wrist_muscle); the others are 'uncovered': reported, never limited. Limit (strict metric = the
max over the vertices, per covered group): --limit-male-cm (default 0.8) and --limit-female-cm (default 0.9). Window verdict (post source only):
PASS when every covered group stays at or under the limit on all worst frames of the window, FAIL otherwise. With the stale source
(no stream 1) nothing is proved: status NOT_VERIFIABLE, the numbers show the staleness the recompute has to remove.

Output (in --out-dir): <prefix>.json (all numbers) and PNG sheets (--sheets all = a sheet per window, flagged = windows over a limit and the whole
recording, none; at most --max-sheets per recording, the whole recording first, then the windows with the largest covered displacement): per
worst frame the body 'before' and 'after' (front / side) and the displacement as colour (0 .. --heat-max-cm); single frames are never written.
Meshes and rig are loaded by tools\\ai_motion\\skin_clip.py; in a working copy that lacks them set UTPP_PROJECT_ROOT to the project folder.
"""
import argparse
import json
import os
import pathlib
import re
import sys
import time
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (HERE, os.path.join(HERE, '..', 'ai_motion')):
    if _p not in sys.path:
        sys.path.insert(0, _p)
ROOT = pathlib.Path(__file__).resolve().parents[3]
import pose_helper_report as ph  # noqa: E402
import helper_graph_batch as hgb  # noqa: E402
import helper_residual_report as hrr  # noqa: E402
from export_player_helper_bundle import fnv64  # noqa: E402

AFFECT_WEIGHT = 0.05
DEFAULT_LIMITS_CM = {'man': 0.8, 'woman': 0.9}


# --------------------------------------------------------------------------------------------------------------------- skin assets
class SkinAssets:
    """Rig, meshes and bindings of one body (loaded once per process)."""

    def __init__(self, body):
        import skin_clip as SK
        self.SK = SK
        self.body = body
        self.names, self.par, self.ref = SK.load_rig(SK.DEFORM_RIG_BY_BODY[body])
        self.idx = {n: i for i, n in enumerate(self.names)}
        self.parts = SK.SC.load_meshes(body)
        self.binds, self.faces, off = [], [], 0
        for p in self.parts:
            b = SK.bindings(p, self.idx, p['name'].startswith('h0_'))
            self.binds.append(([n for n, _ in b], np.stack([m for _, m in b])))
            self.faces.append(p['faces'] + off)
            off += len(p['pos'])
        self.faces = np.concatenate(self.faces)
        self.n_vertices = off
        self.rest = np.concatenate([p['pos'] for p in self.parts])


_ASSETS = {}


def assets_for(body):
    if body not in _ASSETS:
        _ASSETS[body] = SkinAssets(body)
    return _ASSETS[body]


# --------------------------------------------------------------------------------------------------------------------- skinning
class Skinner:
    """Skins the meshes of one body for K frames from per-joint model matrices."""

    def __init__(self, sk, groups, joint_group):
        self.sk, self.groups = sk, groups
        self.joint_group = joint_group                       # rig joint name -> group index (helper joints only)

    def _frame_matrices(self, W, f):
        """{joint: [4,4]} -> per part skin matrices [J,4,4] for frame f."""
        C, CI = self.sk.SK.C, self.sk.SK.CI
        out = []
        for (jn, pre) in self.sk.binds:
            Wl = np.stack([C @ W[n][f] @ CI for n in jn])
            out.append(Wl @ pre)
        return out

    def positions(self, W, f):
        """Vertex positions [V,3] of frame f (glTF coordinates of the meshes, metres)."""
        verts = []
        for p, skin in zip(self.sk.parts, self._frame_matrices(W, f)):
            ph_ = np.c_[p['pos'], np.ones(len(p['pos']))]
            v = np.zeros((len(ph_), 3))
            for c in range(p['js'].shape[1]):
                v += p['ws'][:, c:c + 1] * np.einsum('vij,vj->vi', skin[p['js'][:, c]], ph_)[:, :3]
            verts.append(v)
        return np.concatenate(verts)

    def group_displacement(self, Wbefore, Wafter, f):
        """Per group: displacement [V] (cm) caused by that group's helper bones alone; plus the total of all helper bones."""
        C, CI = self.sk.SK.C, self.sk.SK.CI
        ng = len(self.groups)
        per_group, weight_in = [], []
        total = []
        for p, (jn, pre) in zip(self.sk.parts, self.sk.binds):
            dj = np.stack([(C @ Wafter[n][f] @ CI - C @ Wbefore[n][f] @ CI) if n in self.joint_group else np.zeros((4, 4)) for n in jn]) @ pre
            grp = np.array([self.joint_group.get(n, -1) for n in jn])
            ph_ = np.c_[p['pos'], np.ones(len(p['pos']))]
            dv = np.zeros((ng + 1, len(ph_), 3))
            ws_in = np.zeros((ng, len(ph_)))
            for c in range(p['js'].shape[1]):
                js = p['js'][:, c]
                vec = p['ws'][:, c:c + 1] * np.einsum('vij,vj->vi', dj[js], ph_)[:, :3]
                g = grp[js]
                for gi in range(ng):
                    sel = g == gi
                    if sel.any():
                        dv[gi][sel] += vec[sel]
                        ws_in[gi][sel] += p['ws'][sel, c]
                dv[ng] += vec * (g >= 0)[:, None]
            per_group.append(np.linalg.norm(dv, axis=2) * 100.0)
            weight_in.append(ws_in)
        return np.concatenate(per_group, axis=1), np.concatenate(weight_in, axis=1)


def _stats(disp, weight):
    aff = weight > AFFECT_WEIGHT
    return dict(max=round(float(disp.max()), 4), p99_affected=round(float(np.percentile(disp[aff], 99)), 4) if aff.any() else 0.0,
                mean_affected=round(float(disp[aff].mean()), 4) if aff.any() else 0.0, affected_vertices=int(aff.sum()),
                over_0p5_cm=int((disp > 0.5).sum()), over_1_cm=int((disp > 1.0).sum()))


# --------------------------------------------------------------------------------------------------------------------- rendering
def _vertex_normals(v, faces):
    n = np.cross(v[faces[:, 1]] - v[faces[:, 0]], v[faces[:, 2]] - v[faces[:, 0]])
    out = np.zeros_like(v)
    for k in range(3):
        np.add.at(out, faces[:, k], n)
    return out / np.maximum(np.linalg.norm(out, axis=1, keepdims=True), 1e-12)


def _heat(t):
    """0..1 -> RGB (black, red, yellow, white)."""
    t = np.clip(t, 0, 1) * 3.0
    return np.stack([np.clip(t, 0, 1), np.clip(t - 1, 0, 1), np.clip(t - 2, 0, 1)], axis=-1)


def _panel(v, faces, view, color_value, bounds, size, heat_max_cm):
    """Orthographic point-splat render of the skinned surface (glTF coordinates: x right, y up, z towards the viewer of the chest): the
    vertices plus the centroid and the edge midpoints of every triangle, so the surface closes without a rasteriser. view 'front' looks at
    the chest, 'side' at the right-hand side (+x). color_value None = grey shading, else displacement in cm [V] (colour)."""
    w, h = size
    n = _vertex_normals(v, faces)
    a_, b_, c_ = faces[:, 0], faces[:, 1], faces[:, 2]
    pts = np.concatenate([v, (v[a_] + v[b_] + v[c_]) / 3, (v[a_] + v[b_]) / 2, (v[b_] + v[c_]) / 2, (v[c_] + v[a_]) / 2])
    nrm = np.concatenate([n, n[a_] + n[b_] + n[c_], n[a_] + n[b_], n[b_] + n[c_], n[c_] + n[a_]])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    if color_value is not None:
        val = np.concatenate([color_value, (color_value[a_] + color_value[b_] + color_value[c_]) / 3, (color_value[a_] + color_value[b_]) / 2,
                              (color_value[b_] + color_value[c_]) / 2, (color_value[c_] + color_value[a_]) / 2])
    if view == 'front':
        u, up, depth, view_dir = pts[:, 0], pts[:, 1], pts[:, 2], np.array([0.0, 0.0, 1.0])
        lo_u, hi_u = bounds[0][0], bounds[1][0]
    else:
        u, up, depth, view_dir = -pts[:, 2], pts[:, 1], pts[:, 0], np.array([1.0, 0.0, 0.0])
        lo_u, hi_u = -bounds[1][2], -bounds[0][2]
    lo_v, hi_v = bounds[0][1], bounds[1][1]
    pad = 6
    scale = min((w - 2 * pad) / max(hi_u - lo_u, 1e-6), (h - 2 * pad) / max(hi_v - lo_v, 1e-6))
    px = ((u - 0.5 * (lo_u + hi_u)) * scale + w / 2).astype(int)
    py = (h / 2 - (up - 0.5 * (lo_v + hi_v)) * scale).astype(int)
    facing = nrm @ view_dir
    light = 0.40 + 0.60 * np.clip(facing, 0, 1)
    if color_value is None:
        rgb = np.stack([0.76 * light, 0.70 * light, 0.64 * light], axis=1)
    else:
        rgb = _heat(val / heat_max_cm) * (0.55 + 0.45 * light)[:, None]
    img = np.empty((h, w, 3), np.uint8)
    img[:] = (236, 235, 234)
    keep = np.nonzero(facing > -0.15)[0]
    keep = keep[np.argsort(depth[keep], kind='stable')]                    # far first: nearer points overwrite
    col = (rgb[keep] * 255).astype(np.uint8)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            x, y = px[keep] + dx, py[keep] + dy
            ok = (x >= 0) & (x < w) & (y >= 0) & (y < h)
            img[y[ok], x[ok]] = col[ok]
    return img


def render_sheet(path, title, rows, faces, heat_max_cm):
    """rows: [dict(label, before [V,3], after [V,3], disp [V] cm)]. Columns: before front, after front, displacement front, before side,
    displacement side. A sheet is composed in memory and written once as PNG (no frame files)."""
    from PIL import Image, ImageDraw, ImageFont
    cols = [('before (as recorded)', 'before', 'front', False), ('after (graph)', 'after', 'front', False), ('displacement', 'after', 'front', True),
            ('before (as recorded)', 'before', 'side', False), ('displacement', 'after', 'side', True)]
    allv = np.concatenate([r['after'] for r in rows] + [r['before'] for r in rows])
    bounds = (allv.min(axis=0) - 0.03, allv.max(axis=0) + 0.03)
    pw, ph_ = 300, 560
    head, label_h = 34, 26
    sheet = Image.new('RGB', (pw * len(cols), head + (ph_ + label_h) * len(rows)), (236, 235, 234))
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.load_default(size=13)
    except TypeError:
        font = ImageFont.load_default()
    draw.text((6, 8), '%s   |   heat scale 0 - %.1f cm (black to white)' % (title, heat_max_cm), fill=(20, 20, 20), font=font)
    for ri, row in enumerate(rows):
        y0 = head + ri * (ph_ + label_h)
        draw.text((6, y0 + 4), row['label'], fill=(20, 20, 20), font=font)
        for ci, (name, which, view, heat) in enumerate(cols):
            img = _panel(row[which], faces, view, row['disp'] if heat else None, bounds, (pw, ph_), heat_max_cm)
            sheet.paste(Image.fromarray(img), (ci * pw, y0 + label_h))
            draw.text((ci * pw + 6, y0 + label_h + 2), '%s | %s' % (name, view), fill=(60, 60, 60), font=font)
    pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
    sheet.save(path, optimize=True)


# --------------------------------------------------------------------------------------------------------------------- one recording
def _safe_name(label):
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', label)


def analyze_recording(a, group):
    t_start = time.time()
    limits_cm = a['limit_cm']
    rec = hrr.Recording(group)
    res = dict(file=os.path.basename(rec.path), body=rec.body, frames=rec.n, post_frames=getattr(rec, 'post_frames', 0), day=rec.day)
    if rec.body is None or rec.n == 0:
        res.update(status='NOT_VERIFIABLE', reason='unsupported rig or no frames')
        return res
    body = rec.body
    limit = limits_cm[body]
    bundle_dir = a['bundle_dir'] or hrr.default_bundle_dir(body)
    owned = hrr.read_owned(bundle_dir, hrr.evaluator_for(body).names)
    owned_groups = {hrr.helper_group(n) for n in owned}
    covered = [g for g in hrr.GROUP_ORDER if g in owned_groups]
    eng = hrr.Engine(rec, owned)
    if eng.missing_core or not eng.sp3_ok or not eng.bones:
        res.update(status='NOT_VERIFIABLE', reason='the recording lacks the bones the graph needs')
        return res
    arr = eng.run()
    use_post = rec.V1 is not None and rec.matched > 0 and a['source'] in ('auto', 'post')
    source = 'post' if use_post else 'stale'
    pos_all = arr['pos_post'] if use_post else arr['pos_stale']
    res.update(source=source, limit_cm=limit, covered_groups=covered, bundle_bones=len(owned),
               source_note=('helpers as recorded = stream 1 (after the runtime recompute)' if use_post else
                            'helpers as recorded = stream 0 after the mod\'s write (no runtime recompute in the recording: staleness, not a verdict)'))
    sk = assets_for(body)
    ev = eng.ev
    helper_rig = [n for n in ev.names if n not in ev.core_names]
    group_names = list(dict.fromkeys(hrr.helper_group(n) for n in helper_rig))
    group_names = [g for g in hrr.GROUP_ORDER if g in group_names]
    joint_group = {n: group_names.index(hrr.helper_group(n)) for n in helper_rig}
    skinner = Skinner(sk, group_names, joint_group)
    gated_cols = np.nonzero(eng.gated)[0]
    other_groups = {}
    for g in group_names:
        cols = [j for j, n in enumerate(eng.bone_names) if hrr.helper_group(n) == g and not eng.gated[j]]
        if cols and g not in covered:
            other_groups[g] = cols
    merged = {int(x): i for i, x in enumerate(rec.header.hashes)}
    # windows
    wins = [('all', None, None)]
    if a['plan']:
        wins += hrr.windows_from_plan(a['plan'], rec.day, a['clock_offset'])
    elif a['window_seconds']:
        wins += hrr.fixed_windows(rec.sec, a['window_seconds'])
    unmeasured = set()
    res['windows'] = []
    sheet_jobs = []
    for label, lo, hi in wins:
        in_win = np.ones(rec.n, bool) if lo is None else ((rec.sec >= lo) & (rec.sec < hi))
        usable = in_win & ~arr['bad'] & ~arr['nonfinite'] & np.isfinite(pos_all).any(axis=1)
        written = usable & arr['written']
        pool = np.nonzero(written if written.any() else usable)[0]
        w = dict(label=label, frames=int(in_win.sum()), written=int((in_win & arr['written']).sum()), candidates=int(len(pool)))
        if label != 'all' and not in_win.any():
            w['verdict'] = 'OUTSIDE_RECORDING'
            res['windows'].append(w)
            continue
        if not len(pool):
            w['verdict'] = 'NO_FRAMES'
            res['windows'].append(w)
            continue
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            score = np.nan_to_num(np.nanmax(pos_all[pool][:, gated_cols], axis=1), nan=-1.0)
            pick = list(pool[np.argsort(-score)[:a['frames_per_window']]])
            picked_for = {int(k): ['largest residual of a bundle bone'] for k in pick}
            for g, cols in other_groups.items():                         # the worst frame of every group outside the bundle too
                sc = np.nan_to_num(np.nanmax(pos_all[pool][:, cols], axis=1), nan=-1.0)
                k = int(pool[int(np.argmax(sc))])
                if sc.max() > 0:
                    picked_for.setdefault(k, []).append('largest residual of group ' + g)
                    if k not in pick:
                        pick.append(k)
        ks = np.sort(np.array(pick))
        w['gated_on'] = 'written frames' if written.any() else 'all frames (no mod write in the window)'
        vals1 = rec.V0[ks][:, :, 1, :].astype(np.float64)
        with np.errstate(all='ignore'):
            core_locals = {nm: hgb.quat_matrices(vals1[:, eng.nidx[nm], :]) for nm in eng.core_rec}
            models, bad = ev.run(core_locals, want=list(ev.names))
            if bad.any():
                w['evaluator_refused_frames'] = [int(ks[i]) for i in np.nonzero(bad)[0]]
            if use_post:
                pi = rec.post_of[ks]
                Vrec = rec.V1[pi].astype(np.float64)
            else:
                Vrec = vals1
            Wrec = eng.world(Vrec)
        anchor = np.matmul(models['Spine3'], np.linalg.inv(Wrec[eng.sp3]))               # recorded model space -> deformation rig space
        before, after = {}, {}
        for n in ev.names:
            after[n] = models[n]
            i = merged.get(fnv64(n))
            if n in joint_group and i is not None and i in Wrec and i in eng.rec_rig:
                before[n] = np.matmul(anchor, Wrec[i])
            else:
                before[n] = models[n]
                if n in joint_group:
                    unmeasured.add(n)
        frames = []
        sheet_rows = []
        for f_i, k in enumerate(ks):
            if bad[f_i]:
                continue
            disp, weight = skinner.group_displacement(before, after, f_i)
            row = dict(frame=int(k), clock=hrr.clock(rec.sec[k]), written=bool(arr['written'][k]), picked_for=picked_for[int(k)],
                       worst_helper_residual_cm=round(float(np.nanmax(pos_all[k][gated_cols])), 4),
                       groups={g: _stats(disp[gi], weight[gi]) for gi, g in enumerate(group_names)},
                       total=_stats(disp[-1], (disp[-1] > 1e-6).astype(float)))
            row['covered_max_cm'] = round(max(row['groups'][g]['max'] for g in covered if g in row['groups']), 4) if covered else 0.0
            frames.append(row)
            sheet_rows.append((row, f_i, disp[-1]))
        sheet_rows.sort(key=lambda t: -t[0]['covered_max_cm'])
        sheet_rows = sheet_rows[:a['sheet_frames']] if a['sheets'] != 'none' else []
        w['frames_checked'] = frames
        gm = {}
        for g in group_names:
            gm[g] = round(max(fr['groups'][g]['max'] for fr in frames), 4) if frames else 0.0
        w['group_max_cm'] = gm
        w['covered_max_cm'] = max((gm[g] for g in covered if g in gm), default=0.0)
        w['over_limit_groups'] = [g for g in covered if gm.get(g, 0.0) > limit]
        w['verdict'] = ('FAIL' if w['over_limit_groups'] else 'PASS') if use_post else 'NOT_VERIFIABLE'
        want_sheet = a['sheets'] == 'all' or (a['sheets'] == 'flagged' and (label == 'all' or w['over_limit_groups'] or (not use_post and w['covered_max_cm'] > limit)))
        if want_sheet and sheet_rows:
            rows = []
            for row, f_i, d_total in sheet_rows:
                rows.append(dict(label='frame %d  %s  %s  covered max %.2f cm (limit %.2f)  residual %.2f cm' % (
                    row['frame'], row['clock'], 'mod-written' if row['written'] else 'not written', row['covered_max_cm'], limit, row['worst_helper_residual_cm']),
                    before=skinner.positions(before, f_i), after=skinner.positions(after, f_i), disp=d_total))
            name = '%s_%s_%s_%s.png' % (a['prefix'], rec.body, os.path.basename(rec.path)[len('utpp-pose-'):].split('.')[0], _safe_name(label))
            title = '%s V  %s  window %s  source %s' % ('Male' if body == 'man' else 'Female', os.path.basename(rec.path), label, source)
            sheet_jobs.append((label == 'all', w['covered_max_cm'], w, name, title, rows))
        res['windows'].append(w)
    # at most --max-sheets sheets: the whole recording first, then the windows with the largest covered displacement
    sheet_jobs.sort(key=lambda j: (not j[0], -j[1]))
    for _, _, w, name, title, rows in sheet_jobs[:a['max_sheets']]:
        render_sheet(os.path.join(a['sheet_dir'] or a['out_dir'], name), title, rows, sk.faces, a['heat_max_cm'])
        w['sheet'] = name
    del sheet_jobs
    res['unmeasured_helper_bones'] = sorted(unmeasured)
    verdicts = [w['verdict'] for w in res['windows']]
    if not use_post:
        res['status'] = 'NOT_VERIFIABLE'
        res['reason'] = 'the recording has no post_helper_apply_after_commit stream; the numbers show staleness only'
    elif arr['written'].sum() == 0:
        res['status'] = 'NO_MOD_WRITES'
    elif 'FAIL' in verdicts:
        res['status'] = 'FAIL'
    elif 'PASS' in verdicts:
        res['status'] = 'PASS'
    else:
        res['status'] = 'NO_MOD_WRITES'
    res['seconds'] = round(time.time() - t_start, 1)
    return res


def _worker(task):
    a, path = task
    out = []
    for g in ph.load_groups([path], ph.Config()):
        try:
            out.append(analyze_recording(a, g))
        except Exception as e:                                       # noqa: BLE001 - one unreadable recording must not stop the others
            out.append(dict(file=os.path.basename(g['files'][0][0]), status='NOT_VERIFIABLE', reason='%s: %s' % (type(e).__name__, e)))
    return out


def summary_lines(results):
    lines = []
    for r in results:
        lines.append('%s  %s  status %s%s' % (r.get('file'), r.get('body') or '?', r.get('status'), ('  (%s)' % r['reason']) if r.get('reason') else ''))
        if 'windows' not in r:
            continue
        lines.append('  source %s; covered groups %s; limit %.2f cm (max over the vertices); %s s' % (r['source'], ', '.join(r['covered_groups']), r['limit_cm'], r.get('seconds')))
        for w in r['windows']:
            if w['verdict'] in ('OUTSIDE_RECORDING', 'NO_FRAMES'):
                lines.append('  [%s] %s' % (w['label'], w['verdict']))
                continue
            gm = w['group_max_cm']
            lines.append('  [%s] %s  frames %d (checked %d)  group max cm: %s%s' % (
                w['label'], w['verdict'], w['frames'], len(w['frames_checked']), ' '.join('%s %.2f' % (g, v) for g, v in gm.items()),
                '  sheet ' + w['sheet'] if w.get('sheet') else ''))
        if r.get('unmeasured_helper_bones'):
            lines.append('  helper bones not comparable (not recorded): %d' % len(r['unmeasured_helper_bones']))
    return lines


def run(inputs, a, log=print):
    import concurrent.futures as cf
    os.makedirs(a['out_dir'], exist_ok=True)
    tasks = [(a, x) for x in inputs]
    jobs = max(1, min(a['jobs'], len(tasks)))
    results = []
    if jobs == 1:
        for t in tasks:
            results += _worker(t)
    else:
        for var in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
            os.environ.setdefault(var, '1')
        with cf.ProcessPoolExecutor(jobs) as ex:
            for r in ex.map(_worker, tasks):
                results += r
    p = os.path.join(a['out_dir'], a['prefix'] + '.json')
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(dict(tool='skin_displacement_report.py', limits_cm=a['limit_cm'], recordings=results), f, indent=1)
    for line in summary_lines(results):
        log(line)
    log('wrote ' + p)
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('inputs', nargs='+', help='.utpppose file, folder or wildcard; each argument is one recording')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--prefix', default='skin_displacement')
    ap.add_argument('--bundle-dir')
    ap.add_argument('--plan')
    ap.add_argument('--clock-offset', type=float, default=0.0)
    ap.add_argument('--window-seconds', type=float, default=0.0)
    ap.add_argument('--frames-per-window', type=int, default=6, help='worst frames skinned per window')
    ap.add_argument('--limit-male-cm', type=float, default=DEFAULT_LIMITS_CM['man'])
    ap.add_argument('--limit-female-cm', type=float, default=DEFAULT_LIMITS_CM['woman'])
    ap.add_argument('--source', choices=['auto', 'post', 'stale'], default='auto', help='helper values "as recorded": post stream when present (auto), or force stream 0')
    ap.add_argument('--sheets', choices=['all', 'flagged', 'none'], default='all', help='PNG sheets: every window, only failing windows (and the whole recording), none')
    ap.add_argument('--sheet-frames', type=int, default=2, help='frames shown per sheet (the ones with the largest covered displacement)')
    ap.add_argument('--sheet-dir', help='folder for the PNG sheets (default: --out-dir); the JSON always goes to --out-dir')
    ap.add_argument('--max-sheets', type=int, default=60, help='sheets per recording (the whole recording first, then the largest displacement)')
    ap.add_argument('--heat-max-cm', type=float, default=2.0)
    ap.add_argument('--jobs', type=int, default=max(1, min(12, (os.cpu_count() or 2) // 2)))
    ns = ap.parse_args(argv)
    a = dict(out_dir=ns.out_dir, prefix=ns.prefix, bundle_dir=ns.bundle_dir, plan=ns.plan, clock_offset=ns.clock_offset, window_seconds=ns.window_seconds,
             frames_per_window=ns.frames_per_window, limit_cm={'man': ns.limit_male_cm, 'woman': ns.limit_female_cm}, source=ns.source, sheets=ns.sheets,
             sheet_frames=ns.sheet_frames, sheet_dir=ns.sheet_dir, max_sheets=ns.max_sheets, heat_max_cm=ns.heat_max_cm, jobs=ns.jobs)
    results = run(ns.inputs, a)
    st = [r.get('status') for r in results]
    if 'FAIL' in st:
        return 1
    if 'NOT_VERIFIABLE' in st or not st:
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
