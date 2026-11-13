"""Shared helpers of the recoil-missing tools (2026-11-09).

The numeric core is NOT reimplemented here: weapon_curve() and sampler_text() are imported from
Source\\tools\\analysis\\export_nue_native_recoil_curves.py (the generator of the 0.1.0 profiles), so a new profile is computed
by exactly the code that produced the shipped ones. This module only adds file lookup, the byte-exact writer (CRLF, as
Path.write_text on Windows) and a parser/diff of the sampler format (UTPP_RECOIL 1).
"""
from __future__ import annotations

import hashlib
import math
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_ANALYSIS = _HERE.parent / 'analysis'
sys.path[:0] = [str(_ANALYSIS), str(_HERE.parent / 'recon')]
import export_nue_native_recoil_curves as core  # noqa: E402

ROOT = core.ROOT
# Settings of the shipped 0.1.0 weapon profiles (Data\recoil\profile-export-20261021-budget.json, "bounds" block).
SETTINGS = dict(samples=97, fade=0.15, reference='bind', limit_amplitude=False, start_ramp=0.04, budget=(0.08, 0.50))

GLB_231_WOMAN = [ROOT / 'Extracted/game/glb/weapon-grip-vanilla', ROOT / 'Extracted/game/glb/weapon-recoil-20261021']
GLB_231_MAN = [ROOT / 'Extracted/game/glb/weapon-recoil-male-20261106/glb', ROOT / 'Extracted/game/glb/weapon-grip-vanilla-man']
GLB_25 = ROOT / 'Extracted/game/player-anims/glb'      # 2.5 content + ep1 export of 2026-11-09 (provenance.txt)
RAW_25 = ROOT / 'Extracted/game/player-anims/raw'


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def index_glbs(roots, prefix):
    """file name -> path for every <prefix>_*.anims.glb below the roots (first hit wins, roots in order)."""
    found = {}
    for root in roots:
        if not Path(root).is_dir():
            continue
        for path in sorted(Path(root).rglob(prefix + '_*.anims.glb')):
            found.setdefault(path.name, path)
    return found


_ANIM_CACHE = {}


def animation(path: Path):
    key = str(Path(path).resolve())
    if key not in _ANIM_CACHE:
        _ANIM_CACHE.clear()  # one GLB at a time keeps the memory flat
        _ANIM_CACHE[key] = core.Animation(Path(path))
    return _ANIM_CACHE[key]


def export_clip(path: Path, clip: str, settings=None, budget_override=None):
    """(bytes exactly as the exporter writes them on Windows, rows, clip report) of one clip of one GLB."""
    s = dict(SETTINGS if settings is None else settings)
    if budget_override is not None:
        s['budget'] = budget_override
    anim = animation(path)
    rows, report = core.weapon_curve(anim, clip, s['samples'], s['fade'], s['reference'], s['limit_amplitude'],
                                     s['start_ramp'], s['budget'])
    text = core.sampler_text(rows, report['duration'], s['samples'])
    return text.replace('\n', '\r\n').encode('utf-8'), rows, report


def clips_of(path: Path):
    return animation(path).clips


def parse_sampler(data: bytes):
    """UTPP_RECOIL 1 text -> (duration, [(t, qx, qy, qz, qw, tx, ty, tz)])."""
    tokens = data.decode('utf-8').split()
    if tokens[:2] != ['UTPP_RECOIL', '1']:
        raise ValueError('not a UTPP_RECOIL 1 file')
    count, duration = int(tokens[2]), float(tokens[3])
    values = [float(x) for x in tokens[4:]]
    if len(values) != 8 * count:
        raise ValueError('row count mismatch')
    return duration, [tuple(values[i * 8:(i + 1) * 8]) for i in range(count)]


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def angle_between(qa, qb):
    """rotation angle (rad) of qa^-1 * qb."""
    inv = (-qa[0], -qa[1], -qa[2], qa[3])
    r = _qmul(inv, qb)
    return 2 * math.atan2(math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2), abs(r[3]))


def sample(rows, duration, t):
    """Linear key interpolation like authored_recoil::Clip::Sample (nlerp on the rotation, lerp on the translation)."""
    if t <= 0 or t >= duration:
        return (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0)
    lo, hi = 0, len(rows) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rows[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    w = (t - rows[lo][0]) / (rows[hi][0] - rows[lo][0])
    qa, qb = rows[lo][1:5], rows[hi][1:5]
    if sum(x * y for x, y in zip(qa, qb)) < 0:
        qb = tuple(-x for x in qb)
    q = tuple((1 - w) * x + w * y for x, y in zip(qa, qb))
    n = math.sqrt(sum(x * x for x in q))
    q = tuple(x / n for x in q)
    tr = tuple((1 - w) * x + w * y for x, y in zip(rows[lo][5:8], rows[hi][5:8]))
    return q, tr


def curve_diff(data_a: bytes, data_b: bytes, steps=400):
    """Worst translation (m) / rotation (rad) difference of two sampler files on a common time base, peaks, durations."""
    da, ra = parse_sampler(data_a)
    db, rb = parse_sampler(data_b)
    worst_t = worst_r = 0.0
    peak = [0.0, 0.0, 0.0, 0.0]
    for i in range(steps + 1):
        t = da * i / steps
        qa, ta = sample(ra, da, t)
        qb, tb = sample(rb, db, t)
        worst_t = max(worst_t, math.dist(ta, tb))
        worst_r = max(worst_r, angle_between(qa, qb))
        peak[0] = max(peak[0], math.hypot(*ta))
        peak[1] = max(peak[1], math.hypot(*tb))
        peak[2] = max(peak[2], angle_between((0, 0, 0, 1), qa))
        peak[3] = max(peak[3], angle_between((0, 0, 0, 1), qb))
    return dict(durationA=da, durationB=db, maxDiffT=worst_t, maxDiffR=worst_r, peakTA=peak[0], peakTB=peak[1],
                peakRA=peak[2], peakRB=peak[3])


def release_recoil_pins():
    """name -> (size, sha256) of the RECOIL list in Source\\tools\\release\\make_release.py (read only)."""
    text = (ROOT / 'Source/tools/release/make_release.py').read_text(encoding='utf-8')
    block = text.split('RECOIL = """\\\n', 1)[1].split('"""', 1)[0]
    pins = {}
    for line in block.strip().splitlines():
        name, size, sha = line.split()
        pins[name] = (int(size), sha)
    return pins
