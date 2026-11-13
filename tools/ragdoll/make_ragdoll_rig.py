"""Build the LOCAL TEST player skeleton rigs that carry the NPC ragdoll bodies (UTPP ragdoll PoC 'R1', Cyberpunk 2077 2.5).

Why: NPCs get a physics ragdoll from the 22 `ragdollDesc` (physicsRagdollBodyInfo) + `ragdollNames` (physicsRagdollBodyNames) entries of
their rig (woman_base.rig / man_base.rig). The player rigs player_woman_skeleton.rig / player_man_skeleton.rig (152 bones) have both
lists empty, so an entRagdollComponent on the player entity has no bodies to build. This script copies the 22 bodies + names of the
vanilla NPC base rig into the vanilla player skeleton rig. The player rig has the same first bones but a few extra bones in front of the
hands/toes, so ParentAnimIndex / ChildAnimIndex are re-mapped BY BONE NAME (ParentBodyIndex is an index into the body list itself and
stays; the body order is kept).

`.rig` resources cannot be patched by ArchiveXL, so the PoC ships FULL replacement rigs at the vanilla depot paths:
  base\\characters\\entities\\player\\player_woman_skeleton.rig
  base\\characters\\entities\\player\\player_man_skeleton.rig
LOCAL TEST ONLY (a full replacement is not mod-compatible and not for release).

Sources (read-only): Extracted\\game\\rig250\\... (binary rigs of the 2.5 game; their SHA-256 match a fresh unbundle of basegame_1_engine.archive).
Steps per gender (w = woman, m = man):
  1. WolvenKit `convert serialize` of the NPC base rig and of the player rig into JSON (work dir).
  2. Control: player rig -> JSON -> (pure) deserialize, and player rig -> JSON -> Python json load/dump -> deserialize; both must
     reproduce the original binary byte for byte (SHA-256). Otherwise the pipeline is not trustworthy for a full replacement file
     and the script stops. (Python's json turns the token "-0" into int 0 and loses the sign bit of 7 quaternion components; the
     loader here keeps it as -0.0.)
  3. Source consistency (boneNames[ParentAnimIndex/ChildAnimIndex] == ragdollNames[i]) and name based re-mapping, asserted.
  4. deserialize -> stage\\base\\characters\\entities\\player\\*.rig, serialize again and verify: bone count unchanged, every field
     except ragdollDesc / ragdollNames identical to the vanilla player rig JSON, every body's parent / child NAME equals the source.
     Strip-back: the shipped rig with both lists emptied again must re-pack to the vanilla rig byte for byte.
Usage: python -I make_ragdoll_rig.py --stage <stage root> --work <empty work dir> [--rig-root <Extracted\\game\\rig250>] [--wkit-exe <exe>]
"""
import argparse
import copy
import hashlib
import json
import shutil
import struct
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # the tools folder
from utpp_paths import project_root  # noqa: E402

ROOT = Path(project_root())
WKIT_DEFAULT = ROOT / r"Tools\WolvenKit.CLI\WolvenKit.CLI.exe"
RIG_ROOT_DEFAULT = ROOT / r"Extracted\game\rig250"
PLAYER_DEPOT = Path("base") / "characters" / "entities" / "player"
NPC_DEPOT = Path("base") / "characters" / "base_entities"
GENDERS = {
    "w": {"npc": NPC_DEPOT / "woman_base" / "woman_base.rig", "player": PLAYER_DEPOT / "player_woman_skeleton.rig"},
    "m": {"npc": NPC_DEPOT / "man_base" / "man_base.rig", "player": PLAYER_DEPOT / "player_man_skeleton.rig"},
}
VOLATILE_HEADER = ("ExportedDateTime", "ArchiveFileName")
REPLACED = ("ragdollDesc", "ragdollNames")
APOSE = ("aPoseLS", "aPoseMS")


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest().upper()


def run(args):
    print(">", " ".join(str(a) for a in args))
    r = subprocess.run([str(a) for a in args], capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    if r.returncode != 0 or "[ 0: Error" in out or "Exception" in out:
        print(out.strip()[-3000:])
        raise SystemExit(f"command failed ({r.returncode})")
    return out


def serialize(wkit, rig, out_dir):
    """rig -> out_dir/<name>.rig.json"""
    out_dir.mkdir(parents=True, exist_ok=True)
    run([wkit, "convert", "serialize", rig, "-o", out_dir])
    j = out_dir / (Path(rig).name + ".json")
    if not j.is_file():
        raise SystemExit(f"serialize produced no {j}")
    return j


def deserialize(wkit, json_path, out_dir):
    """<name>.rig.json -> out_dir/<name>.rig"""
    out_dir.mkdir(parents=True, exist_ok=True)
    run([wkit, "convert", "deserialize", json_path, "-o", out_dir])
    built = out_dir / Path(json_path).name[: -len(".json")]
    if not built.is_file():
        raise SystemExit(f"deserialize produced no {built}")
    return built


def _parse_int(s):
    # WolvenKit writes a negative-zero float as the token "-0"; Python would turn it into int 0 and lose the sign bit (7 quaternion
    # components of the player rigs). Keep it as -0.0, which json.dumps writes back as "-0.0".
    return -0.0 if s == "-0" else int(s)


def load(p):
    return json.loads(Path(p).read_text(encoding="utf-8"), parse_int=_parse_int)


def dump(obj, p):
    Path(p).write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def root_chunk(doc):
    return doc["Data"]["RootChunk"]


def diffs(a, b, path="", limit=20):
    """Deep compare; returns up to `limit` difference descriptions (ints and floats compare by value like JSON consumers do)."""
    out = []

    def rec(x, y, p):
        if len(out) >= limit:
            return
        if isinstance(x, dict) and isinstance(y, dict):
            for k in sorted(set(x) | set(y)):
                if k not in x or k not in y:
                    out.append(f"{p}/{k}: only in {'b' if k not in x else 'a'}")
                else:
                    rec(x[k], y[k], f"{p}/{k}")
        elif isinstance(x, list) and isinstance(y, list):
            if len(x) != len(y):
                out.append(f"{p}: length {len(x)} != {len(y)}")
                return
            for i, (xi, yi) in enumerate(zip(x, y)):
                rec(xi, yi, f"{p}[{i}]")
        elif x != y:
            out.append(f"{p}: {x!r} != {y!r}")

    rec(a, b, path)
    return out


def same_json(a_doc, b_doc, skip_root=()):
    ra = {k: v for k, v in root_chunk(a_doc).items() if k not in skip_root}
    rb = {k: v for k, v in root_chunk(b_doc).items() if k not in skip_root}
    d = diffs(ra, rb)
    ha = {k: v for k, v in a_doc["Header"].items() if k not in VOLATILE_HEADER}
    hb = {k: v for k, v in b_doc["Header"].items() if k not in VOLATILE_HEADER}
    d += diffs(ha, hb, "/Header")
    d += diffs({k: v for k, v in a_doc["Data"].items() if k != "RootChunk"},
               {k: v for k, v in b_doc["Data"].items() if k != "RootChunk"}, "/Data")
    return d


def bone_names(rig):
    return [b["$value"] for b in rig["boneNames"]]


def is_ancestor(parents, anc, bone):
    seen = 0
    while bone >= 0 and seen < 10000:
        if bone == anc:
            return True
        bone = parents[bone]
        seen += 1
    return False


def remap_bodies(src, player, label):
    """Copy ragdollDesc/ragdollNames of the NPC base rig `src` into a copy of rig `player` data; returns (descs, names, info)."""
    sn, pn = bone_names(src), bone_names(player)
    if len(set(pn)) != len(pn):
        raise SystemExit(f"{label}: duplicate bone names in the player rig, name lookup is ambiguous")
    if len(set(sn)) != len(sn):
        raise SystemExit(f"{label}: duplicate bone names in the NPC rig, name lookup is ambiguous")
    ppos = {n: i for i, n in enumerate(pn)}
    descs, names = src["ragdollDesc"], src["ragdollNames"]
    if len(descs) != len(names) or not descs:
        raise SystemExit(f"{label}: ragdollDesc ({len(descs)}) / ragdollNames ({len(names)}) mismatch or empty")
    if player["ragdollDesc"] or player["ragdollNames"]:
        raise SystemExit(f"{label}: the player rig already has ragdoll data, refusing to overwrite")
    out_desc, out_names, changes, warnings = [], [], [], []
    spar = src["boneParentIndexes"]
    for i, (d, n) in enumerate(zip(descs, names)):
        pa, ch = d["ParentAnimIndex"], d["ChildAnimIndex"]
        pbi = d["ParentBodyIndex"]
        pname, cname = n["ParentAnimName"]["$value"], n["ChildAnimName"]["$value"]
        # source consistency: the NPC rig's own index -> name mapping must agree with its names list
        if ch < 0 or ch >= len(sn) or sn[ch] != cname:
            raise SystemExit(f"{label} body {i}: source ChildAnimIndex {ch} -> {sn[ch] if 0 <= ch < len(sn) else '?'} != {cname}")
        if pa >= 0 and (pa >= len(sn) or sn[pa] != pname):
            raise SystemExit(f"{label} body {i}: source ParentAnimIndex {pa} -> {sn[pa] if pa < len(sn) else '?'} != {pname}")
        if not (pbi == -1 or 0 <= pbi < i):
            raise SystemExit(f"{label} body {i}: ParentBodyIndex {pbi} is not -1 or an earlier body")
        if cname not in ppos or (pa >= 0 and pname not in ppos):
            raise SystemExit(f"{label} body {i}: bone {pname!r}/{cname!r} missing in the player rig")
        if pa >= 0 and not is_ancestor(spar, pa, ch):
            warnings.append(f"body {i}: {pname} is not an ancestor of {cname} in the NPC rig")
        nd = copy.deepcopy(d)
        nd["ChildAnimIndex"] = ppos[cname]
        if pa >= 0:
            nd["ParentAnimIndex"] = ppos[pname]
        if (nd["ParentAnimIndex"], nd["ChildAnimIndex"]) != (pa, ch):
            changes.append({"body": i, "parent": pname, "child": cname,
                            "parentIdx": [pa, nd["ParentAnimIndex"]], "childIdx": [ch, nd["ChildAnimIndex"]]})
        pp = player["boneParentIndexes"]
        if pa >= 0 and not is_ancestor(pp, nd["ParentAnimIndex"], nd["ChildAnimIndex"]):
            warnings.append(f"body {i}: {pname} is not an ancestor of {cname} in the player rig")
        out_desc.append(nd)
        out_names.append(copy.deepcopy(n))
    info = {
        "bodies": len(out_desc),
        "remappedBodies": changes,
        "warnings": warnings,
        "maxBoneIndexUsed": max(max(d["ChildAnimIndex"], d["ParentAnimIndex"]) for d in out_desc),
        "sourceLodStartIndices": src.get("levelOfDetailStartIndices"),
        "playerLodStartIndices": player.get("levelOfDetailStartIndices"),
        "playerBoneCount": len(pn),
        "sourceBoneCount": len(sn),
    }
    return out_desc, out_names, info


def _q(t):
    r = t["Rotation"]
    return (r["i"], r["j"], r["k"], r["r"])


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _qinv(q):
    return (-q[0], -q[1], -q[2], q[3])


def _qrot(q, v):
    return _qmul(_qmul(q, (v[0], v[1], v[2], 0.0)), _qinv(q))[:3]


def _qnorm(q):
    n = sum(c * c for c in q) ** 0.5
    q = tuple(c / n for c in q)
    return q if q[3] >= 0 else tuple(-c for c in q)


def fk_ms(local, parents):
    """Local -> model space (rotation, translation); matches the rigs' own aPoseLS -> aPoseMS to ~1e-7 (checked on both NPC rigs)."""
    ms = []
    for i, t in enumerate(local):
        q = _q(t)
        p = (t["Translation"]["X"], t["Translation"]["Y"], t["Translation"]["Z"])
        pi = parents[i]
        if pi < 0:
            ms.append((q, p))
        else:
            pq, pp = ms[pi]
            rp = _qrot(pq, p)
            ms.append((_qmul(pq, q), tuple(pp[k] + rp[k] for k in range(3))))
    return ms


def _f32(x):
    """The rig stores float32; round here so the file round trip compares equal (value of the float32, 9 significant digits)."""
    return float("%.9g" % struct.unpack("<f", struct.pack("<f", x))[0])


def _qs(q, t, w, scale):
    return {"$type": "QsTransform",
            "Rotation": {"$type": "Quaternion", "i": _f32(q[0]), "j": _f32(q[1]), "k": _f32(q[2]), "r": _f32(q[3])},
            "Scale": copy.deepcopy(scale),
            "Translation": {"$type": "Vector4", "W": w, "X": _f32(t[0]), "Y": _f32(t[1]), "Z": _f32(t[2])}}


def f32_diffs(a, b):
    """diffs() with numbers compared as float32 (WolvenKit writes the shortest float32 text, Python keeps 9 digits)."""
    def norm(x):
        if isinstance(x, dict):
            return {k: norm(v) for k, v in x.items()}
        if isinstance(x, list):
            return [norm(v) for v in x]
        if isinstance(x, float) or (isinstance(x, int) and not isinstance(x, bool)):
            return struct.unpack("<f", struct.pack("<f", x))[0]
        return x
    return diffs(norm(a), norm(b))


def synth_apose(src, player, label):
    """A-pose (aPoseLS / aPoseMS) for the player rig. The NPC base rigs carry one (arms ~45 deg down); the player rigs have none.
    The ragdoll joint limits of the copied bodies are tight around the arms (shoulder body +-6 deg, upper-arm swing about
    -40..+25 deg); without an A-pose V's arms settled in a flat T on the ground (game run 2026-11-03 22:34).
    Per bone: the NPC local rotation delta bind^-1 * aPose (by name) applied to the player's own bind rotation; translation and
    scale stay the player's bind values (the NPC A-pose differs from its bind only in rotations). Bones without an NPC
    counterpart keep the bind. aPoseMS is FK of aPoseLS (translation W = 1, as in the NPC rigs)."""
    sn, pn = bone_names(src), bone_names(player)
    sbind, sap, pbind = src["boneTransforms"], src["aPoseLS"], player["boneTransforms"]
    if len(sap) != len(sn) or len(src["aPoseMS"]) != len(sn):
        raise SystemExit(f"{label}: NPC rig A-pose size {len(sap)}/{len(src['aPoseMS'])} != {len(sn)} bones")
    if player.get("aPoseLS") or player.get("aPoseMS"):
        raise SystemExit(f"{label}: the player rig already has an A-pose, refusing to overwrite")
    spos = {n: i for i, n in enumerate(sn)}
    ls, changed = [], []
    for i, n in enumerate(pn):
        b = pbind[i]
        q = _q(b)
        j = spos.get(n)
        if j is not None:
            delta = _qmul(_qinv(_q(sbind[j])), _q(sap[j]))
            if max(abs(c) for c in (delta[0], delta[1], delta[2])) > 1e-6:
                q = _qnorm(_qmul(q, delta))
                changed.append(n)
        t = b["Translation"]
        ls.append(_qs(q, (t["X"], t["Y"], t["Z"]), t["W"], b["Scale"]))
    ms = [_qs(_qnorm(q), p, 1, pbind[i]["Scale"]) for i, (q, p) in enumerate(fk_ms(ls, player["boneParentIndexes"]))]

    def elev(pose_ms, names, a, b):
        pa, pb = pose_ms[names.index(a)][1], pose_ms[names.index(b)][1]
        d = [pb[k] - pa[k] for k in range(3)]
        return round(__import__("math").degrees(__import__("math").asin(d[2] / sum(x * x for x in d) ** 0.5)), 1)

    sms = fk_ms(sap, src["boneParentIndexes"])
    pms = fk_ms(ls, player["boneParentIndexes"])
    check = {}
    for a, b in (("LeftArm", "LeftForeArm"), ("RightArm", "RightForeArm"), ("LeftForeArm", "LeftHand"), ("LeftUpLeg", "LeftLeg")):
        check[f"{a}->{b}"] = {"npc": elev(sms, sn, a, b), "player": elev(pms, pn, a, b)}
        if abs(check[f"{a}->{b}"]["npc"] - check[f"{a}->{b}"]["player"]) > 6.0:
            raise SystemExit(f"{label}: synthesized A-pose {a}->{b} elevation {check[f'{a}->{b}']} differs from the NPC A-pose")
    print(f"{label}: A-pose synthesized, {len(changed)} bones rotated, elevations {check}")
    return ls, ms, {"rotatedBones": changed, "elevations": check}


LIMIT_FIELDS = ("SwingAnglesY", "SwingAnglesZ", "TwistAngles")


def parse_limits(specs):
    """--limit BODY:FIELD=a,b (BODY = the body's ChildAnimName) -> {body: {field: [a, b]}}."""
    out = {}
    for spec in specs or ():
        try:
            body, rest = spec.split(":", 1)
            field, vals = rest.split("=", 1)
            a, b = (float(x) for x in vals.split(","))
        except ValueError:
            raise SystemExit(f"--limit {spec!r}: expected BODY:FIELD=a,b")
        if field not in LIMIT_FIELDS:
            raise SystemExit(f"--limit {spec!r}: field must be one of {LIMIT_FIELDS}")
        out.setdefault(body, {})[field] = [int(a) if a.is_integer() else a, int(b) if b.is_integer() else b]
    return out


def apply_limits(label, descs, names, limits):
    """Overwrite the angle ranges named by parse_limits on the copied bodies; every named body must exist."""
    by_name = {n["ChildAnimName"]["$value"]: i for i, n in enumerate(names)}
    applied = []
    for body, fields in limits.items():
        if body not in by_name:
            raise SystemExit(f"{label}: --limit body {body!r} is not a ragdoll body")
        d = descs[by_name[body]]
        for field, val in fields.items():
            if not isinstance(d.get(field), dict) or "Elements" not in d[field]:
                raise SystemExit(f"{label}: body {body} has no {field} range to override")
            applied.append({"body": body, "field": field, "from": list(d[field]["Elements"]), "to": val})
            d[field]["Elements"] = list(val)
    return applied


def verify_final(label, rt_doc, orig_player_doc, src_doc, expected_desc, expected_names, apose=None, limits=None):
    rt, orig, src = root_chunk(rt_doc), root_chunk(orig_player_doc), root_chunk(src_doc)
    limits = limits or {}
    if apose is not None:
        for key, want in zip(APOSE, apose):
            d = f32_diffs(rt[key], want)
            if d:
                raise SystemExit(f"{label}: {key} after the round trip differs:\n  " + "\n  ".join(d))
    if len(rt["boneNames"]) != len(orig["boneNames"]):
        raise SystemExit(f"{label}: bone count changed {len(orig['boneNames'])} -> {len(rt['boneNames'])}")
    d = same_json(rt_doc, orig_player_doc, skip_root=REPLACED + (APOSE if apose is not None else ()))
    if d:
        raise SystemExit(f"{label}: fields other than ragdollDesc/ragdollNames(/aPose) changed:\n  " + "\n  ".join(d))
    if rt["ragdollDesc"] != expected_desc and diffs(rt["ragdollDesc"], expected_desc):
        raise SystemExit(f"{label}: ragdollDesc after the file round trip differs from what was written:\n  "
                         + "\n  ".join(diffs(rt["ragdollDesc"], expected_desc)))
    if diffs(rt["ragdollNames"], expected_names):
        raise SystemExit(f"{label}: ragdollNames after the round trip differ")
    rn, sn = bone_names(rt), bone_names(src)
    for i, (d_, n_) in enumerate(zip(rt["ragdollDesc"], rt["ragdollNames"])):
        sd = src["ragdollDesc"][i]
        pnm, cnm = n_["ParentAnimName"]["$value"], n_["ChildAnimName"]["$value"]
        s_p = sn[sd["ParentAnimIndex"]] if sd["ParentAnimIndex"] >= 0 else None
        s_c = sn[sd["ChildAnimIndex"]]
        p_p = rn[d_["ParentAnimIndex"]] if d_["ParentAnimIndex"] >= 0 else None
        p_c = rn[d_["ChildAnimIndex"]]
        if (p_p, p_c) != (s_p, s_c):
            raise SystemExit(f"{label} body {i}: player names ({p_p},{p_c}) != source names ({s_p},{s_c})")
        if s_p is not None and s_p != pnm:
            raise SystemExit(f"{label} body {i}: ragdollNames parent {pnm} != {s_p}")
        if s_c != cnm:
            raise SystemExit(f"{label} body {i}: ragdollNames child {cnm} != {s_c}")
        over = limits.get(cnm, {})
        for field, val in over.items():
            if d_[field]["Elements"] != val:
                raise SystemExit(f"{label} body {i}: {field} {d_[field]['Elements']} != override {val}")
        skip = ("ParentAnimIndex", "ChildAnimIndex") + tuple(over)
        other = {k: v for k, v in d_.items() if k not in skip}
        sother = {k: v for k, v in sd.items() if k not in skip}
        if diffs(other, sother):
            raise SystemExit(f"{label} body {i}: non-index fields differ from the source body")
    print(f"{label}: verified {len(rt['ragdollDesc'])} bodies, {len(rn)} bones, other fields identical")


def control(wkit, label, player_rig, player_json, work):
    """Pipeline control on the UNMODIFIED player rig (a full replacement file must not lose anything on the way)."""
    res = {"original": sha(player_rig)}
    c1 = work / "control-pure"
    b1 = deserialize(wkit, player_json, c1)
    res["pure_deserialize"] = sha(b1)
    c2 = work / "control-python"
    c2.mkdir(parents=True, exist_ok=True)
    doc = load(player_json)
    pj = c2 / player_json.name
    dump(doc, pj)
    b2 = deserialize(wkit, pj, c2 / "out")
    res["python_deserialize"] = sha(b2)
    res["pure_bytes_equal"] = res["pure_deserialize"] == res["original"]
    res["python_bytes_equal"] = res["python_deserialize"] == res["original"]
    print(f"{label} control: original {res['original'][:16]} pure {res['pure_deserialize'][:16]} "
          f"({res['pure_bytes_equal']}) python {res['python_deserialize'][:16]} ({res['python_bytes_equal']})")
    if not (res["pure_bytes_equal"] and res["python_bytes_equal"]):
        # Strict on purpose: a value compare in Python cannot see e.g. a lost negative zero, only the bytes can.
        raise SystemExit(f"{label}: the unmodified player rig does not come back byte-identical through the pipeline "
                         f"(pure {res['pure_bytes_equal']}, python {res['python_bytes_equal']}); blocks shipping a full rig")
    return res


def strip_back(wkit, label, rt_json, original_rig, work):
    """Byte-level proof of 'all other fields identical': empty the ragdoll lists of the shipped rig again and re-pack it; the
    result must be byte-identical to the vanilla player rig."""
    doc = load(rt_json)
    root_chunk(doc)["ragdollDesc"] = []
    root_chunk(doc)["ragdollNames"] = []
    for key in APOSE:
        root_chunk(doc)[key] = []
    d = work / "strip-back"
    j = d / Path(original_rig).name
    j = j.with_name(j.name + ".json")
    j.parent.mkdir(parents=True, exist_ok=True)
    dump(doc, j)
    built = deserialize(wkit, j, d / "out")
    got, want = sha(built), sha(original_rig)
    print(f"{label} strip-back: {got[:16]} vs vanilla {want[:16]} ({got == want})")
    if got != want:
        raise SystemExit(f"{label}: shipped rig minus ragdoll data is NOT byte-identical to the vanilla rig")
    return got


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, help="staging root; rigs go to <stage>/base/characters/entities/player/")
    ap.add_argument("--work", required=True, help="work dir for JSON / intermediates (must not exist or be empty)")
    ap.add_argument("--rig-root", default=str(RIG_ROOT_DEFAULT))
    ap.add_argument("--wkit-exe", default=str(WKIT_DEFAULT))
    ap.add_argument("--apose", action="store_true", help="also give the player rig an A-pose synthesized from the NPC rig (ragdoll joint frames)")
    ap.add_argument("--limit", action="append", metavar="BODY:FIELD=a,b",
                    help="override one angle range of a copied body (BODY = ChildAnimName, FIELD = SwingAnglesY/SwingAnglesZ/"
                         "TwistAngles), e.g. LeftForeArm:SwingAnglesY=40,80; repeatable, applied to both rigs")
    a = ap.parse_args()
    limits = parse_limits(a.limit)
    wkit, rig_root = Path(a.wkit_exe), Path(a.rig_root)
    work, stage = Path(a.work), Path(a.stage)
    if not wkit.is_file():
        raise SystemExit(f"WolvenKit CLI missing: {wkit}")
    if work.exists() and any(work.iterdir()):
        raise SystemExit(f"work dir is not empty, refusing: {work}")
    work.mkdir(parents=True, exist_ok=True)
    report = {}
    for g, spec in GENDERS.items():
        label = {"w": "woman", "m": "man"}[g]
        npc_rig, player_rig = rig_root / spec["npc"], rig_root / spec["player"]
        for p in (npc_rig, player_rig):
            if not p.is_file():
                raise SystemExit(f"source rig missing: {p}")
        dst = stage / spec["player"]
        if dst.exists():
            raise SystemExit(f"stage file exists, refusing to overwrite: {dst}")
        wd = work / label
        src_json = serialize(wkit, npc_rig, wd / "json-npc")
        player_json = serialize(wkit, player_rig, wd / "json-player")
        rep = {"npcRig": str(npc_rig), "npcSha256": sha(npc_rig), "playerRig": str(player_rig), "playerSha256": sha(player_rig),
               "depotPath": str(spec["player"]).replace("/", "\\")}
        rep["control"] = control(wkit, label, player_rig, player_json, wd)
        src_doc, player_doc = load(src_json), load(player_json)
        descs, names, info = remap_bodies(root_chunk(src_doc), root_chunk(player_doc), label)
        rep["remap"] = info
        if limits:
            rep["limits"] = apply_limits(label, descs, names, limits)
            for c in rep["limits"]:
                print(f"{label} limit {c['body']}.{c['field']}: {c['from']} -> {c['to']}")
        for w in info["warnings"]:
            print(f"{label} WARNING: {w}")
        for c in info["remappedBodies"]:
            print(f"{label} remap body {c['body']}: {c['parent']} {c['parentIdx']} / {c['child']} {c['childIdx']}")
        mod = copy.deepcopy(player_doc)
        root_chunk(mod)["ragdollDesc"] = descs
        root_chunk(mod)["ragdollNames"] = names
        apose = None
        if a.apose:
            als, ams, rep["apose"] = synth_apose(root_chunk(src_doc), root_chunk(player_doc), label)
            root_chunk(mod)["aPoseLS"], root_chunk(mod)["aPoseMS"] = als, ams
            apose = (als, ams)
        mod_json = wd / "json-modified" / (player_rig.name + ".json")
        mod_json.parent.mkdir(parents=True, exist_ok=True)
        dump(mod, mod_json)
        built = deserialize(wkit, mod_json, wd / "built")
        rt_json = serialize(wkit, built, wd / "roundtrip")
        verify_final(label, load(rt_json), player_doc, src_doc, descs, names, apose, limits)
        rep["stripBackSha256"] = strip_back(wkit, label, rt_json, player_rig, wd)
        rep["filterDataOverrides"] = sorted({d["FilterDataOverride"]["$value"] for d in descs})
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(built, dst)
        rep["output"] = str(dst)
        rep["outputSha256"] = sha(dst)
        rep["outputSize"] = dst.stat().st_size
        rep["boneCount"] = len(root_chunk(player_doc)["boneNames"])
        report[label] = rep
        print(f"{label}: wrote {dst} sha256 {rep['outputSha256']}")
    (work / "rig-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("RIG_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
