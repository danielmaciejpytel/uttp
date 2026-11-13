"""Query reference dump for the ChooserMatcher parity test (utpp_mm_chooser_matcher_test --query-ref).

For N database rows of a .utppmm VERSION 2 file this writes the stored (normalized) DFEA feature row together with
the raw inputs that produce it: the root trajectory of the row's clip (x, y, z = RTZ root_z, yaw) on the 60 Hz grid
around the row's frame, and a pose history (root + Root-space position / rotation of every bone read by a pose
channel) on the half frames t <= 0. Raw inputs come from the retargeted clip through mmbake_motion.ClipData (the baker's
own grid: root_at / rz_at / fk), channels are rebuilt from the file's SCHC records. Self-check: mmbake_motion
schema_features on the rebuilt channels, normalized with the file's NRMV, must equal the stored row (the dump fails
otherwise), so the reference is what the baker stored.

Usage: python dump_query_reference.py <db.utppmm> --clip-dirs <dir1;dir2;...> --out <ref.txt>
       [--mmbake-dir <dir with mmbake_motion.py>] [--n 10] [--seed 20261013]
Clip directories are searched in order (first match of <clip name>.utppclip.json wins: RTZ directory first).
The output is source-pack-derived data: keep it outside git.
"""
import argparse
import os
import struct
import sys

import numpy as np


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", "..", "..", ".."))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("db")
    ap.add_argument("--clip-dirs", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mmbake-dir", default=os.path.join(root, "Source", "tools", "mmbake"))
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--seed", type=int, default=20261013)
    ap.add_argument("--tol", type=float, default=1e-4)
    a = ap.parse_args(argv)
    sys.path.insert(0, a.mmbake_dir)
    import mmbake_motion as m2  # noqa: E402

    b, h, secs = m2.read_v2(a.db)

    def off(name):
        return secs[name][2]

    strs = off("STRS")

    def string(o):
        if o == 0xFFFFFFFF:
            return ""
        e = b.index(b"\0", strs + o)
        return b[strs + o:e].decode("utf-8")

    nb = h["num_bones"]
    names, parents = [], []
    for i in range(nb):
        no, par, _, _ = struct.unpack_from("<IiII", b, off("BONE") + 16 * i)
        names.append(string(no))
        parents.append(par)
    clips = [struct.unpack_from("<IIIIIIQ", b, off("CLIP") + 32 * c) for c in range(h["num_clips"])]
    dbs = []
    for d in range(h["num_databases"]):
        r = struct.unpack_from("<6I5f5IQQIIfI", b, off("DBAS") + 96 * d)
        dbs.append(dict(name=string(r[0]), schema=r[1], norm=r[2], first_row=r[13], num_rows=r[14], dims=r[15],
                        feat=r[16]))
    nrmv = off("NRMV")

    def norm_vec(n, dims):
        _, nd, vo, _, _, _, _ = struct.unpack_from("<IIIIQII", b, off("NORM") + 32 * n)
        assert nd == dims
        mean = np.array(struct.unpack_from("<%df" % dims, b, nrmv + 4 * vo), float)
        scale = np.array(struct.unpack_from("<%df" % dims, b, nrmv + 4 * (vo + dims)), float)
        return mean, scale

    class Ch:
        pass

    def schema_channels(s):
        _, dims, fc, ncch, _, _, _, _ = struct.unpack_from("<4IfIfI", b, off("SCHM") + 32 * s)
        out = []
        for k in range(fc, fc + ncch):
            r = struct.unpack_from("<IIiiffIIffIIII3fII", b, off("SCHC") + 76 * k)
            c = Ch()
            c.kind, c.comp, c.game_bone, c.game_origin = r[0], r[1], r[2], r[3]
            c.t_sample, c.t_origin, c.vel_space, c.input_query = r[4], r[5], r[6], r[7]
            c.offset, c.card = r[10], r[11]
            c.local_axis = (r[14], r[15], r[16])
            c.flags = r[17]
            out.append(c)
        return out, dims

    cfg1 = m2.load_json(os.path.join(a.mmbake_dir, "config", "default.json"))
    dirs = [d for d in a.clip_dirs.split(";") if d]

    def clip_file(name):
        for d in dirs:
            p = os.path.join(d, name + ".utppclip.json")
            if os.path.isfile(p):
                return p
        return None

    rng = np.random.default_rng(a.seed)
    # candidates: random non-block rows of every database (turn / pivot / jump databases first)
    order = sorted(range(len(dbs)), key=lambda d: (0 if any(k in dbs[d]["name"] for k in ("Pivot", "Turn", "Jump", "Spin"))
                                                   else 1, rng.random()))
    cand = []
    for d in order:
        D = dbs[d]
        if D["num_rows"] == 0:
            continue
        for _ in range(3):
            r = int(rng.integers(D["num_rows"]))
            frame, _, _, entry, fl = struct.unpack_from("<IffII", b, off("DROW") + 20 * (D["first_row"] + r))
            if fl & 1:
                continue
            clip = struct.unpack_from("<I", b, off("DENT") + 32 * entry + 4)[0]
            cname = string(clips[clip][0])
            if clip_file(cname):
                cand.append((d, r, frame, clip, cname))
                break
        if len(cand) >= 4 * a.n:
            break

    cache = {}
    cases = []
    worst = 0.0
    for (d, r, frame, clip, cname) in cand:
        if len(cases) >= a.n:
            break
        D = dbs[d]
        chans, dims = schema_channels(D["schema"])
        assert dims == D["dims"]
        key = cname
        if key not in cache:
            cd = m2.ClipData(clip_file(cname), cfg1, names, parents)
            cd.loop = bool(clips[clip][3] & 1)
            cache[key] = cd
        cd = cache[key]
        hs_all = []
        for c in chans:
            hs, ho = int(round(c.t_sample * m2.HALF)), int(round(c.t_origin * m2.HALF))
            hs_all += [hs, ho, hs - 1, ho - 1]
        h_lo, h_hi = min(min(hs_all), 0), max(max(hs_all), 0)
        cd.build_grid(h_lo, h_hi)
        raw = m2.schema_features(cd, chans, dims)
        lf = frame - clips[clip][1]
        mean, scale = norm_vec(D["norm"], dims)
        stored = np.array(struct.unpack_from("<%df" % dims, b, 4 * (D["feat"] + r * dims) + off("DFEA")), float)
        mine = (raw[lf] - mean) * scale
        diff = float(np.abs(mine - stored).max())
        if diff > a.tol:
            print(f"SKIP {D['name']} row {r} {cname}: recomputed features differ from DFEA by {diff:.3g}", file=sys.stderr)
            continue
        worst = max(worst, diff)
        G = cd.grid
        g0 = 2 * lf - h_lo
        traj = []
        for hh in range(h_lo, h_hi + 1):
            g = g0 + hh
            rt = G["root"][g]
            traj.append((hh / m2.HALF, rt[0], rt[1], G["rz"][g], rt[2]))
        bones = sorted({x for c in chans if not (c.flags & 1) for x in (c.game_bone, c.game_origin) if x >= 0})
        hist = []
        for hh in range(max(h_lo, -23), 1):
            g = g0 + hh
            rt = G["root"][g]
            ph = G["ph"][g]
            row = [hh / m2.HALF, rt[0], rt[1], G["rz"][g], rt[2]]
            for bo in bones:
                row += list(G["gp"][ph, bo]) + list(G["gq"][ph, bo])
            hist.append(row)
        cases.append(dict(d=d, r=r, frame=frame, clip=clip, cname=cname, traj=traj, bones=bones, hist=hist,
                          feat=stored, diff=diff))

    if len(cases) < a.n:
        print(f"ERROR: only {len(cases)} of {a.n} rows reproduced", file=sys.stderr)
        return 1
    with open(a.out, "w", newline="\n") as f:
        f.write("utpp_query_ref 1\n")
        f.write(f"cases {len(cases)}\n")
        for c in cases:
            f.write(f"case {c['d']} {c['r']} {c['frame']} {c['clip']} {c['cname']}\n")
            f.write(f"traj {len(c['traj'])}\n")
            for s in c["traj"]:
                f.write(" ".join("%.9g" % v for v in s) + "\n")
            f.write(f"hist {len(c['hist'])} {len(c['bones'])}" + "".join(f" {x}" for x in c["bones"]) + "\n")
            for s in c["hist"]:
                f.write(" ".join("%.9g" % v for v in s) + "\n")
            f.write(f"feat {len(c['feat'])}" + "".join(" %.9g" % v for v in c["feat"]) + "\n")
    for c in cases:
        yaw = [s[4] for s in c["traj"]]
        print(f"{dbs[c['d']]['name']:40s} row {c['r']:6d} frame {c['frame']:7d} {c['cname']:50s} "
              f"selfcheck {c['diff']:.2e} yaw span {np.degrees(max(yaw) - min(yaw)):7.1f} deg "
              f"hist bones {c['bones']}")
    print(f"dump_query_reference: {len(cases)} rows, max |recomputed - stored| {worst:.3g} -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
