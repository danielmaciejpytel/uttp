"""Foot ground contact + slide detection with a LOCAL stance ground (numpy only).

The ground is NOT the global minimum of the clip: for every frame the ground level is the lowest sole height among the
*stance candidates* (frames whose sole vertical speed is below vz_thr, of either foot) inside a window of +-window frames. A clip
that steps onto a ledge / ramp, a crouch with a lowered hips or a slide therefore keeps its contacts, while a foot that is
merely passing through the apex of a jump is removed by the minimum contact duration.

sole_height(heel_z, toe_z, heel_off, toe_off) -> height of the lower of the two sole points above the ground they stand on
  (offsets = joint height above the sole in the rig's reference pose, e.g. LeftHeel 0.083 m, LeftToeBase 0.018 m).
detect_contacts(sole [T] or [T,F], ...) -> bool [T] / [T,F]
slide_segments(contact [T], xy [T,2], ...) -> list of dict(start, end, drift_cm, max_speed_cm_s) for each contact run
"""
import numpy as np


def sole_height(heel_z, toe_z, heel_off, toe_off):
    return np.minimum(np.asarray(heel_z) - heel_off, np.asarray(toe_z) - toe_off)


def _runs(mask):
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            out.append((start, i - 1))
            start = None
    if start is not None:
        out.append((start, len(mask) - 1))
    return out


def local_ground(sole, cand, window):
    """sole [T,F], cand [T,F] bool -> ground [T] (nan where no candidate exists within 2*window frames)."""
    T = sole.shape[0]
    h = np.where(cand, sole, np.inf).min(axis=1)  # lowest candidate sole height per frame (over feet)
    g = np.full(T, np.nan)
    for t in range(T):
        lo, hi = max(0, t - window), min(T, t + window + 1)
        m = h[lo:hi].min()
        if np.isinf(m):  # extend the search to 2*window
            lo, hi = max(0, t - 2 * window), min(T, t + 2 * window + 1)
            m = h[lo:hi].min()
        g[t] = np.nan if np.isinf(m) else m
    return g


def detect_contacts(sole, fps=30.0, window=10, h_thr=0.04, vz_thr=0.35, min_len=3):
    """Contact mask. A foot is in contact when its sole is within h_thr of the local ground and not moving vertically faster than
    vz_thr (m/s); runs shorter than min_len frames are dropped (jump apex, foot passing the ground)."""
    s = np.asarray(sole, dtype=float)
    one = s.ndim == 1
    if one:
        s = s[:, None]
    T = s.shape[0]
    vz = np.zeros_like(s)
    if T > 2:
        vz[1:-1] = (s[2:] - s[:-2]) * fps / 2.0
        vz[0], vz[-1] = vz[1], vz[-2]
    cand = np.abs(vz) < vz_thr
    g = local_ground(s, cand, window)
    ok = ~np.isnan(g)
    near = np.zeros_like(cand)
    near[ok] = (s[ok] - g[ok, None]) < h_thr
    contact = cand & near
    for f in range(contact.shape[1]):
        for a, b in _runs(contact[:, f]):
            if b - a + 1 < min_len:
                contact[a:b + 1, f] = False
    return contact[:, 0] if one else contact


def slide_segments(contact, xy, fps=30.0, min_len=5):
    """Horizontal drift of a planted foot: per contact run the maximum distance of the foot from the run start (cm) and the peak
    horizontal speed (cm/s)."""
    xy = np.asarray(xy, dtype=float)
    out = []
    for a, b in _runs(contact):
        if b - a + 1 < min_len:
            continue
        seg = xy[a:b + 1]
        drift = np.linalg.norm(seg - seg[0], axis=1).max() * 100.0
        sp = np.linalg.norm(np.diff(seg, axis=0), axis=1).max() * fps * 100.0 if len(seg) > 1 else 0.0
        out.append(dict(start=a, end=b, drift_cm=float(drift), max_speed_cm_s=float(sp)))
    return out
