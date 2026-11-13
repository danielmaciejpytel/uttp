#!/usr/bin/env python3
"""Per-clip source family + style weight table sidecar for the ALS overlay style layer (port, report 5.5 / 6.4).

The runtime (src/mm/Overlay.hpp StyleFamilyData, OverlayMachine families) scales the style delta by
weight(style, family of the active clip) and crossfades on a family change.

The .utppmm v2 database only stores an ADDED clip flag (CLIP flags bit4 / CLP2 added), not the source, so this
script writes a small text sidecar from the clip map (sourcepack-clip-map.json):

  utpp-style-weights 1
  # comments
  [weights]
  <Style>/<family> = <weight 0..1>          Style: Neutral | Feminine | Masculine
  [clips]
  <clip name> = <family>                    family: source pack | other
  * = <family>                              family of an unlisted Neutral clip (ADDED flag 0)
  ! = <family>                              family of an unlisted non-Neutral clip (ADDED flag 1)

Only clips whose family differs from the unlisted default ("*" for Neutral clips, "!" for ADDED clips) are listed. With
--db the clip names and ADDED flags of a .utppmm v2 database are read (independent reader written from FORMAT.txt
"VERSION 2"); a Neutral clip with the ADDED flag or a non-Neutral clip without it is an error (exit 1), so the sidecar of a
database lists no clip (every family follows from the ADDED flag). Without --db every non-source-pack clip of the clip map is listed.

Default weights: 1 for every style and family. Override with --weight Style/family=w (repeatable).
--neutral-comments writes the neutral comment block of a release file instead of the provenance comments.

Data written next to the .utppov files (Data/overlay) is source-pack-derived metadata and stays outside git.

Usage:
  python export_style_weights.py --clip-map Research\\inventory\\sourcepack-clip-map.json
      --db Data\\utpp_motion_woman.utppmm --out Data\\overlay\\utpp_style_weights_woman.txt
"""
import argparse
import hashlib
import json
import os
import struct
import sys

STYLES = ["Neutral", "Feminine", "Masculine"]
FAMILIES = ["primary", "other"]
DEFAULT_WEIGHTS = {(s, f): 1.0 for s in STYLES for f in FAMILIES}
NEUTRAL_COMMENTS = ["# UTPP style weights.", "# Licensed animation data - do not extract or redistribute."]

HEADER_FMT = "<8sIIQQI" + "I" * 11 + "f" + "IIIIIII" + "QQ"
CF_ADDED = 1 << 4


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def clip_families(cmap):
    """name -> family from the clip map (source pack AnimSequences = primary, every other listed clip = other)."""
    fam = {}
    for c in cmap.get("clips", []):
        fam[c["name"]] = "primary"
    for c in cmap.get("female_clips", []):
        fam[c["name"]] = "other"
    for c in cmap.get("other_female_sources", []):
        fam.setdefault(c["name"], "other")
    return fam


def read_db_clips(path):
    """[(name, added)] of a .utppmm v2 file (header, section table, STRS, CLIP, CLP2)."""
    with open(path, "rb") as f:
        b = f.read()
    h = struct.unpack_from(HEADER_FMT, b, 0)
    if h[0] != b"UTPPMMDB" or h[1] != 2:
        raise ValueError(f"{path}: not a .utppmm version 2 file")
    num_clips, num_sections, table_off = h[8], h[18], h[19]
    secs = {}
    for i in range(num_sections):
        sid, cnt, esz, _, off, size = struct.unpack_from("<IIIIQQ", b, table_off + 32 * i)
        secs[struct.pack("<I", sid).decode("ascii")] = (cnt, esz, off, size)
    strs = secs["STRS"][2]
    clip_off, clp2_off = secs["CLIP"][2], secs["CLP2"][2]
    out = []
    for c in range(num_clips):
        name_off, _, _, flags, _, _, _ = struct.unpack_from("<IIIIIIQ", b, clip_off + 32 * c)
        added2 = struct.unpack_from("<IIIIffII", b, clp2_off + 32 * c)[6]
        e = b.index(b"\0", strs + name_off)
        name = b[strs + name_off:e].decode("utf-8")
        ng = bool(flags & CF_ADDED)
        if ng != bool(added2):
            raise ValueError(f"{path}: clip {name}: CLIP ADDED flag {int(ng)} != CLP2 added {added2}")
        out.append((name, ng))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--clip-map", required=True)
    ap.add_argument("--db", help=".utppmm v2 database: list exactly its added clips and check the ADDED flag")
    ap.add_argument("--out", required=True)
    ap.add_argument("--weight", action="append", default=[], help="Style/family=w override (repeatable)")
    ap.add_argument("--neutral-comments", action="store_true", help="neutral comment block (release copy) instead of provenance comments")
    a = ap.parse_args(argv)

    weights = dict(DEFAULT_WEIGHTS)
    for w in a.weight:
        key, _, val = w.partition("=")
        st, _, fa = key.strip().partition("/")
        if st not in STYLES or fa not in FAMILIES:
            print(f"ERROR: bad --weight {w}", file=sys.stderr)
            return 2
        v = float(val)
        if not 0.0 <= v <= 1.0:
            print(f"ERROR: --weight {w}: weight outside 0..1", file=sys.stderr)
            return 2
        weights[(st, fa)] = v

    with open(a.clip_map, "r", encoding="utf-8") as f:
        cmap = json.load(f)
    fam = clip_families(cmap)
    errors = []
    if a.db:
        listed = []
        for name, ng in read_db_clips(a.db):
            f_ = fam.get(name)
            if f_ is None:
                f_ = "other" if ng else "primary"
                if not ng:
                    errors.append(f"{name}: not in the clip map and ADDED 0")
            if (f_ == "primary") == ng:
                errors.append(f"{name}: family {f_} but ADDED {int(ng)}")
            if f_ != ("other" if ng else "primary"):
                listed.append((name, f_))
    else:
        listed = [(n, f_) for n, f_ in fam.items() if f_ != "primary"]
    listed.sort()
    counts = {f_: sum(1 for _, x in listed if x == f_) for f_ in FAMILIES if f_ != "primary"}

    lines = ["utpp-style-weights 1"]
    if a.neutral_comments:
        lines += NEUTRAL_COMMENTS
    else:
        lines += ["# generator: tools/mmbake/export_style_weights.py",
                  f"# clip map: {os.path.basename(a.clip_map)} sha256 {sha256(a.clip_map)}"]
        if a.db:
            lines.append(f"# database: {os.path.basename(a.db)} sha256 {sha256(a.db)}")
        lines += ["# listed clips: " + ", ".join(f"{k} {v}" for k, v in counts.items())]
    lines.append("[weights]")
    for s in STYLES:
        for f_ in FAMILIES:
            lines.append(f"{s}/{f_} = {weights[(s, f_)]:g}")
    lines += ["[clips]", "* = primary", "! = other"]
    lines += [f"{n} = {f_}" for n, f_ in listed]
    text = "\n".join(lines) + "\n"
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print(f"{a.out}: {len(listed)} added clips ({counts}); errors {len(errors)}")
    for e in errors[:20]:
        print("ERROR:", e, file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
