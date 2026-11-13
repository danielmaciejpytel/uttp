"""Build a full override of base\\vehicles\\common\\gameplay\\vehicle_motorcycle_workspot.workspot with the UTPP third-person bike
enter sets added to the player rigs.

Why (diag 2026-10-28 c8b): the bike seat entry/exit clips come from the vehicle's OccupantSlots
workspot (v_sportbike*.ent -> this .workspot), which carries its own per-rig animset list (workspotTree.finalAnimsets,
disableAutoAnimsetGeneraion = 1). The player entity's animation sets (where the earlier root / extension routes put the UTPP enter
sets) are not part of that lookup. Vanilla player rigs: p?a_sportbikeN 200 [sportbikeN_vehicle_seat_front_left] (first person),
tpp_p?a_sportbikeN 128 [player_tpp_head, seat], look_at 128 [kusanagi_fpp], player_default_vehicle 254 [].

Change (player_woman_skeleton.rig -> pwa, player_man_skeleton.rig -> pma; every other rig untouched): append to
animations.gameplay, per bike N in 1..3 and side l/r, an entry for utpp\\vehicle\\utpp_tpp_p?a_sportbike[N]_enter_{l,r}.anims
(bike 1 has no number in the file name) with variableNames [utpp_vehicle_tpp, sportbikeN_vehicle_seat_front_left] at priority 127,
then the same 6 entries again at priority 230 (covers both precedence directions; first person is unaffected because every new
entry needs utpp_vehicle_tpp, which utpp_vehicle_anim.reds only adds for a bike mount in UTPP third person). loadingHandles get
the same paths in the same order (1:1 with the appended entries, as the vanilla NPC rigs repeat a handle for a repeated set).
Entries and handles are copies of the rig's vanilla tpp_p?a_sportbike entry / handle with path, vars and priority replaced.
animations.hash stays as in the vanilla file; --hash0 sets it to "0" on the two player rigs only (variant for a second test).

Checks (all must pass, exit code != 0 otherwise):
  * optional --vanilla-bin: sha256 of the vanilla binary and its WolvenKit serialize == the vanilla JSON (Header ignored);
  * every new .anims exists in --anims-dir;
  * the built .workspot (WolvenKit deserialize) serialized back to JSON == the expected JSON (Header ignored);
  * removing the appended entries / handles (and restoring the hash for --hash0) from the round trip gives the vanilla JSON;
  * per player rig gameplay and loadingHandles grow 8 -> 20, NPC rigs identical.
--selftest runs mutations of the round-trip comparison (each must be rejected).

Usage: python -I make_bike_workspot.py --stage <archive staging root> --anims-dir <dir with the 12 .anims> [--hash0] [--selftest]
Output: <stage>\\base\\vehicles\\common\\gameplay\\vehicle_motorcycle_workspot.workspot, <stage>\\..\\workspot-roundtrip\\*.json
"""
import argparse
import copy
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

WKIT_DEFAULT = Path(project_root(), r"Tools\WolvenKit.CLI\WolvenKit.CLI.exe")
DIAG = Path(project_root(), r"Research\logs\diag-20261028-c8b\animset-rule")
VANILLA_JSON = DIAG / "json" / "vehicle_motorcycle_workspot.workspot.json"
VANILLA_BIN = DIAG / "raw" / "base" / "vehicles" / "common" / "gameplay" / "vehicle_motorcycle_workspot.workspot"
VANILLA_SHA256 = "f0cf46f6d82087eee7c8f27abd5e03e72e8fe1108374da3f210175a76f944876"
WS_DEPOT = Path("base") / "vehicles" / "common" / "gameplay"
WS_NAME = "vehicle_motorcycle_workspot.workspot"
WRAPPER = "utpp_vehicle_tpp"
PRIOS = (127, 230)
RIGS = {
    r"base\characters\entities\player\player_woman_skeleton.rig": "pwa",
    r"base\characters\entities\player\player_man_skeleton.rig": "pma",
}
VANILLA_COUNT = 8


EXIT_SETS = []   # opt-in (2026-11-06, --exit-sets all|1l,2r,w1r,m3r,...): (rig, bike, "xl"|"xr") of the normal get-off sets (MotionBricks stepping), registered like the enter sets; rig = "pwa" / "pma" (token prefix w / m) or None = both player rigs; empty = none (default)


def clip_path(g, bike, side):
    num = "" if bike == 1 else str(bike)
    if side in ("xl", "xr"):   # exit sets: utpp_tpp_p?a_sportbike[N]_exit_{l,r}.anims (one clip exit_l / exit_r)
        return r"utpp\vehicle\utpp_tpp_%s_sportbike%s_exit_%s.anims" % (g, num, side[1])
    return r"utpp\vehicle\utpp_tpp_%s_sportbike%s_enter_%s.anims" % (g, num, side)


def seat_var(bike):
    return "sportbike%d_vehicle_seat_front_left" % bike


def new_items(g=None):
    """(prio, bike, side) in append order for the player rig g ("pwa" / "pma"): all bikes / sides at the first priority, then the same at the second.
    g=None: only the rig-independent items (the enter sets and the exit sets registered on both rigs)."""
    items = [(p, b, s) for p in PRIOS for b in (1, 2, 3) for s in ("l", "r")]
    if EXIT_SETS:   # appended after every enter entry (the default list is unchanged); a set without a rig prefix on both player rigs, a w / m prefixed one only on that rig
        items += [(p, b, s) for p in PRIOS for (r, b, s) in EXIT_SETS if r is None or r == g]
    return items


def fa_list(doc):
    return doc["Data"]["RootChunk"]["workspotTree"]["Data"]["finalAnimsets"]


def rig_of(fa):
    return fa["rig"]["DepotPath"]["$value"]


def path_of(entry_or_handle):
    d = entry_or_handle.get("animSet", entry_or_handle)
    return d["DepotPath"]["$value"]


def cname(template, value):
    c = copy.deepcopy(template)
    c["$value"] = value
    return c


def build_expected(vanilla, hash0):
    doc = copy.deepcopy(vanilla)
    seen = set()
    for fa in fa_list(doc):
        g = RIGS.get(rig_of(fa))
        if not g:
            continue
        seen.add(g)
        gp = fa["animations"]["gameplay"]
        lh = fa["loadingHandles"]
        if len(gp) != VANILLA_COUNT or len(lh) != VANILLA_COUNT:
            raise SystemExit("unexpected vanilla %s: %d entries / %d handles" % (g, len(gp), len(lh)))
        tpp = r"base\animations\vehicle\sportbike\character\tpp_%s_sportbike.anims" % g
        tmpl_e = next((e for e in gp if path_of(e) == tpp), None)
        tmpl_h = next((h for h in lh if path_of(h) == tpp), None)
        if tmpl_e is None or tmpl_h is None:
            raise SystemExit("template %s not found in rig %s" % (tpp, g))
        var_t = tmpl_e["variableNames"][0]
        for prio, bike, side in new_items(g):
            p = clip_path(g, bike, side)
            e = copy.deepcopy(tmpl_e)
            e["animSet"]["DepotPath"]["$value"] = p
            e["priority"] = prio
            e["variableNames"] = [cname(var_t, WRAPPER), cname(var_t, seat_var(bike))]
            gp.append(e)
            h = copy.deepcopy(tmpl_h)
            h["DepotPath"]["$value"] = p
            lh.append(h)
        if hash0:
            fa["animations"]["hash"] = "0"
    if seen != set(RIGS.values()):
        raise SystemExit("player rigs missing in the vanilla workspot: %s" % sorted(set(RIGS.values()) - seen))
    return doc


def body(doc):
    return {k: v for k, v in doc.items() if k != "Header"}


def first_diff(a, b, p=""):
    if type(a) != type(b):
        return p + " (type)"
    if isinstance(a, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                return p + "/" + k + " (missing)"
            d = first_diff(a[k], b[k], p + "/" + k)
            if d:
                return d
        return None
    if isinstance(a, list):
        if len(a) != len(b):
            return p + " (len %d != %d)" % (len(a), len(b))
        for i, (x, y) in enumerate(zip(a, b)):
            d = first_diff(x, y, "%s[%d]" % (p, i))
            if d:
                return d
        return None
    return None if a == b else p + " (%r != %r)" % (a, b)


def strip_added(doc, vanilla):
    """Undo the UTPP change on a copy: drop appended entries / handles and restore the vanilla hash of the player rigs."""
    out = copy.deepcopy(doc)
    vfa = {rig_of(f): f for f in fa_list(vanilla)}
    for fa in fa_list(out):
        if rig_of(fa) in RIGS:
            fa["animations"]["gameplay"] = fa["animations"]["gameplay"][:VANILLA_COUNT]
            fa["loadingHandles"] = fa["loadingHandles"][:VANILLA_COUNT]
            fa["animations"]["hash"] = vfa[rig_of(fa)]["animations"]["hash"]
    return out


def verify(rt, expected, vanilla, hash0):
    """Return a list of errors (empty = pass)."""
    errs = []
    d = first_diff(body(expected), body(rt))
    if d:
        errs.append("round trip != expected at " + d)
    d = first_diff(body(vanilla), body(strip_added(rt, vanilla)))
    if d:
        errs.append("outside the appended entries the round trip differs from vanilla at " + d)
    vfa = {rig_of(f): f for f in fa_list(vanilla)}
    if [rig_of(f) for f in fa_list(rt)] != [rig_of(f) for f in fa_list(vanilla)]:
        errs.append("rig order / count changed")
    for fa in fa_list(rt):
        r = rig_of(fa)
        g = RIGS.get(r)
        if not g:
            if fa != vfa.get(r):
                errs.append("NPC rig changed: " + r)
            continue
        gp, lh = fa["animations"]["gameplay"], fa["loadingHandles"]
        n = VANILLA_COUNT + len(new_items(g))
        if len(gp) != n or len(lh) != n:
            errs.append("%s: %d entries / %d handles, want %d" % (g, len(gp), len(lh), n))
            continue
        for (prio, bike, side), e, h in zip(new_items(g), gp[VANILLA_COUNT:], lh[VANILLA_COUNT:]):
            want = clip_path(g, bike, side)
            vars_ = [v["$value"] for v in e["variableNames"]]
            if path_of(e) != want or path_of(h) != want or e["priority"] != prio or vars_ != [WRAPPER, seat_var(bike)]:
                errs.append("%s: bad entry %s %s prio %s vars %s" % (g, path_of(e), path_of(h), e["priority"], vars_))
        want_hash = "0" if hash0 else vfa[r]["animations"]["hash"]
        if fa["animations"]["hash"] != want_hash or (not hash0 and want_hash in ("0", 0)):
            errs.append("%s: hash %s, want %s" % (g, fa["animations"]["hash"], want_hash))
    return errs


def selftest(rt, expected, vanilla, hash0):
    def m_npc(d):
        for fa in fa_list(d):
            if rig_of(fa) not in RIGS:
                fa["animations"]["gameplay"][0]["priority"] = 2
                return

    def m_handle(d):
        fa_list(d)[0]["loadingHandles"].pop()

    def m_fpp_var(d):
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                fa["animations"]["gameplay"][VANILLA_COUNT]["variableNames"].pop(0)
                return

    def m_hash(d):
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                fa["animations"]["hash"] = "1" if hash0 else "0"
                return

    def m_vanilla_entry(d):
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                fa["animations"]["gameplay"][0]["priority"] = 127
                return

    def m_tree(d):
        d["Data"]["RootChunk"]["workspotTree"]["Data"]["inertializationDurationEnter"] = 0.25

    bad = 0
    for name, m in (("NPC rig entry", m_npc), ("missing handle", m_handle), ("entry without utpp_vehicle_tpp", m_fpp_var),
                    ("hash", m_hash), ("vanilla player entry", m_vanilla_entry), ("tree field", m_tree)):
        d = copy.deepcopy(rt)
        m(d)
        caught = bool(verify(d, expected, vanilla, hash0))
        print("selftest %-32s %s" % (name, "rejected" if caught else "NOT CAUGHT"))
        bad += not caught
    return bad


def run(args):
    print(">", " ".join(str(a) for a in args))
    r = subprocess.run([str(a) for a in args], capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    print(out.strip()[-1500:])
    if r.returncode != 0:
        raise SystemExit("command failed (%d)" % r.returncode)
    return out


def sha256(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def load(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, help="archive staging root (the workspot goes to <stage>/base/vehicles/...)")
    ap.add_argument("--anims-dir", required=True, help="dir holding the 12 utpp_tpp_p?a_sportbike*_enter_?.anims")
    ap.add_argument("--vanilla-json", default=str(VANILLA_JSON))
    ap.add_argument("--vanilla-bin", default=str(VANILLA_BIN), help="'' to skip the vanilla binary check")
    ap.add_argument("--wkit-exe", default=str(WKIT_DEFAULT))
    ap.add_argument("--hash0", action="store_true", help="animations.hash = 0 on the two player rigs")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--exit-sets", default="", help="also register utpp_tpp_p?a_sportbike[N]_exit_{l,r}.anims for these bikes / sides on both player rigs (same variables / priorities, 2 entries per set): "
                    "'all' or a list like 1l,1r,2l,3r (bike number + side; both rigs) or w1r / m2r (prefix w = pwa Female V only, m = pma Male V only); default off (empty)")
    a = ap.parse_args()
    global EXIT_SETS
    es = a.exit_sets.strip()
    if es == "all":
        es = "1l,1r,2l,2r,3l,3r"
    EXIT_SETS = []
    for t in es.split(","):
        if not t:
            continue
        rig = {"w": "pwa", "m": "pma"}.get(t[0]) if t[0] in "wm" else None
        t = t[1:] if t[0] in "wm" else t
        if len(t) != 2 or not t[0].isdigit():
            raise SystemExit("bad --exit-sets %r" % a.exit_sets)
        EXIT_SETS.append((rig, int(t[0]), "x" + t[1]))
    if len(set(EXIT_SETS)) != len(EXIT_SETS) or any(b not in (1, 2, 3) or s not in ("xl", "xr") for _, b, s in EXIT_SETS):
        raise SystemExit("bad --exit-sets %r" % a.exit_sets)
    wkit = Path(a.wkit_exe)
    if not wkit.is_file():
        raise SystemExit("WolvenKit CLI missing: %s" % wkit)
    stage = Path(a.stage)
    work = stage.parent / "workspot-work"
    rt_dir = stage.parent / "workspot-roundtrip"
    for d in (work, rt_dir):
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)
    vanilla = load(a.vanilla_json)
    if a.vanilla_bin:
        h = sha256(a.vanilla_bin)
        if h != VANILLA_SHA256:
            raise SystemExit("vanilla binary sha256 %s != %s" % (h, VANILLA_SHA256))
        vdir = work / "vanilla"
        vdir.mkdir()
        shutil.copy2(a.vanilla_bin, vdir / WS_NAME)
        run([wkit, "convert", "serialize", vdir / WS_NAME, "-o", vdir])
        d = first_diff(body(vanilla), body(load(vdir / (WS_NAME + ".json"))))
        if d:
            raise SystemExit("vanilla JSON does not match the vanilla binary at " + d)
        print("vanilla ok: sha256 %s, JSON == serialize(binary)" % h)
    missing = [clip_path(g, b, s) for g in RIGS.values() for _, b, s in new_items(g)
               if not (Path(a.anims_dir) / Path(clip_path(g, b, s)).name).is_file()]
    if missing:
        raise SystemExit("missing .anims in %s: %s" % (a.anims_dir, sorted(set(missing))))
    expected = build_expected(vanilla, a.hash0)
    src = work / "built" / (WS_NAME + ".json")
    src.parent.mkdir()
    src.write_text(json.dumps(expected, indent=2) + "\n", encoding="utf-8")
    out_dir = stage / WS_DEPOT
    out_dir.mkdir(parents=True, exist_ok=True)
    run([wkit, "convert", "deserialize", src, "-o", out_dir])
    built = out_dir / WS_NAME
    if not built.is_file() and (src.parent / WS_NAME).is_file():
        shutil.move(str(src.parent / WS_NAME), str(built))
    if not built.is_file():
        raise SystemExit("deserialize produced no " + WS_NAME)
    run([wkit, "convert", "serialize", built, "-o", rt_dir])
    rt = load(rt_dir / (WS_NAME + ".json"))
    errs = verify(rt, expected, vanilla, a.hash0)
    for e in errs:
        print("ERROR", e)
    if errs:
        return 1
    for fa in fa_list(rt):
        g = RIGS.get(rig_of(fa))
        if g:
            print("rig %s: %d entries, %d handles, hash %s" % (g, len(fa["animations"]["gameplay"]),
                                                              len(fa["loadingHandles"]), fa["animations"]["hash"]))
    print("workspot ok: %s sha256 %s (%d bytes), only the appended player-rig entries / handles%s differ from vanilla"
          % (built, sha256(built), built.stat().st_size, " and the player-rig hash" if a.hash0 else ""))
    shutil.rmtree(work)
    if a.selftest and selftest(rt, expected, vanilla, a.hash0):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
