"""Build a full override of base\\vehicles\\common\\gameplay\\vehicle_workspot.workspot (the OccupantSlots workspot of EVERY player-
drivable car) with the UTPP third-person car enter sets added to the two player rigs. Copy of make_bike_workspot.py (c8b) for cars.

Why (map 2026-11-01; bike precedent diag 2026-10-28 c8b): the seat entry clips of a
vehicle come from its OccupantSlots workspot, which carries its own per-rig animset list (workspotTree.finalAnimsets,
disableAutoAnimsetGeneraion = 1). Vanilla player rigs (28 entries each): p?a_<class>_driver / _passenger_r / _passenger_l 128
[<class>_vehicle_seat_front_left | front_right | back_left | back_right] (first person 'enter'), generic sets 129 [seat_*], look-at 100,
defaults 254 []. No third-person car entry exists. The entity-level registration of utpp_vehicle_entry.archive.xl is not part of that
lookup and never played.

Change (player_woman_skeleton.rig -> pwa, player_man_skeleton.rig -> pma; every other rig untouched): append to animations.gameplay,
per class (the clips found in --anims-dir) and seat (driver = seat_front_left, passenger_r = seat_front_right), an entry for
utpp\\vehicle\\utpp_car_p?a_<class>_<seat>.anims (clip 'enter') with variableNames [utpp_vehicle_tpp, <class>_vehicle_seat_front_left |
_right] at priority 127, then the same entries again at priority 230 (covers both precedence directions around the vanilla 128; first
person is unaffected because every new entry needs utpp_vehicle_tpp, which utpp_vehicle_anim.reds only adds for a vehicle mount in UTPP
third person; a class without UTPP clips keeps its vanilla entry because the vanilla entry's single var is still in the list - the
workspot uses an entry when ALL its variableNames are in the list, c8b / b72796e). loadingHandles get the same paths in the same order
(1:1 with the appended entries). Entries and handles are copies of the rig's vanilla p?a_<class>_<seat> entry / handle with path, vars and
priority replaced. animations.hash stays as in the vanilla file (the c8b bike override ran with the vanilla hash).

Passenger seated idle (2026-11-01): a TPP passenger has no 'tpp_idle' in any passenger set; ~0.4 s after seating the game asks for the clip
'tpp_idle' by name and falls through the workspot's animset list to the DEFAULT set base\\animations\\vehicle\\player_default_vehicle.anims
(priority 254, no variables), a fixed driver-like pose 6-14 cm off the passenger seat (measured offline against the body trace: mean
0.14 deg, Hips identical). A class with utpp_car_p?a_<class>_passenger_r_idle.anims (clip 'tpp_idle' = the vanilla passenger 'idle'
renamed, make_passenger_idle.py) gets those sets as well, the same variables and priorities as its passenger 'enter' set, so the
lookup finds the class idle first (lower priority number wins: the drivers already get their class 'tpp_idle' at 128 before the
default 254). Optional per class; if present, both genders are required.

Driver EXIT sets (2026-11-05, build_exit.ps1 / build_exit_duck.py): utpp_car_p?a_<class>_driver_exit.anims (clip 'exit' only = the vanilla exit with the head duck) get the same
variables and priorities (127 / 230) as the class driver 'enter' set, appended AFTER every entry / idle item (the entry items keep their positions). The lookup of the clip 'exit' then
finds the UTPP set first (lower priority number wins) while utpp_vehicle_tpp is in the workspot variable list; everything else of the vanilla driver set (idle, death, cool_exit ...) stays
vanilla. Optional per class and per gender (only the sets found are registered, e.g. female sport2 only); the entry sets stay mandatory for both genders.

Checks (all must pass, exit code != 0 otherwise):
  * --vanilla-bin: sha256 of the vanilla binary (2.31 and the installed 2.5 are identical) and its WolvenKit serialize == the vanilla JSON;
  * every new .anims exists in --anims-dir, both genders for every class;
  * the built .workspot (WolvenKit deserialize) serialized back to JSON == the expected JSON (Header ignored);
  * removing the appended entries / handles from the round trip gives the vanilla JSON (strip-to-vanilla);
  * per player rig gameplay and loadingHandles grow 28 -> 28 + 4 per class (+ 2 per class with passenger idle sets); the four NPC rigs
    are identical to vanilla.
--selftest runs mutations of the round-trip comparison (each must be rejected).

Usage: python -I make_car_workspot.py --stage <archive staging root> --anims-dir <dir with utpp_car_*.anims (incl. *_passenger_r_idle.anims)> [--selftest]
Output: <stage>\\base\\vehicles\\common\\gameplay\\vehicle_workspot.workspot, <stage>\\..\\workspot-roundtrip\\*.json
"""
import argparse
import copy
import hashlib
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

WKIT_DEFAULT = Path(project_root(), r"Tools\WolvenKit.CLI\WolvenKit.CLI.exe")
EX = Path(project_root(), r"Extracted\game\vehicle-entry-car")
VANILLA_JSON = EX / "json" / "vehicle_workspot.workspot.json"
VANILLA_BIN = EX / "raw" / "base" / "vehicles" / "common" / "gameplay" / "vehicle_workspot.workspot"
VANILLA_SHA256 = "6a3e7378ea47ed37f68c498155295ce041b9ad247029a7f15d32c61b67fe9266"
WS_DEPOT = Path("base") / "vehicles" / "common" / "gameplay"
WS_NAME = "vehicle_workspot.workspot"
WRAPPER = "utpp_vehicle_tpp"
PRIOS = (127, 230)
RIGS = {
    r"base\characters\entities\player\player_woman_skeleton.rig": "pwa",
    r"base\characters\entities\player\player_man_skeleton.rig": "pma",
}
VANILLA_COUNT = 28
# class -> animation folder of the vanilla sets; seat -> (vanilla clip suffix, var suffix)
CLASS_DIR = {"standard2": "standard", "standard25": "standard", "standard3": "standard", "sport1": "sport", "sport2": "sport",
             "utility": "utility"}
# clip set class -> seat var class (VehicleDataPackageUtility4 uses utility4_vehicle_seat_*, its player sets are p?a_utility_*)
VAR_CLASS = {"utility": "utility4"}
SEATS = {"driver": "front_left", "passenger_r": "front_right"}
# the seated idle of the front passenger (clip 'tpp_idle'): a pseudo seat sharing the passenger variable and template
IDLE_SEAT = "passenger_r_idle"
# the vanilla exit of the driver seat with the head duck (clip 'exit'): a pseudo seat sharing the driver variable and template, optional per gender
EXIT_SEAT = "driver_exit"
CLIP_RE = re.compile(r"^utpp_car_p([wm])a_([a-z0-9]+)_(driver|passenger_r|passenger_r_idle|driver_exit)\.anims$")


def clip_path(g, cls, seat):
    return r"utpp\vehicle\utpp_car_p%sa_%s_%s.anims" % (g, cls, seat)


def real_seat(seat):
    return "passenger_r" if seat == IDLE_SEAT else ("driver" if seat == EXIT_SEAT else seat)


def vanilla_path(g, cls, seat):
    return r"base\animations\vehicle\%s\character\p%sa_%s_%s.anims" % (CLASS_DIR[cls], g, cls, real_seat(seat))


def seat_var(cls, seat):
    return "%s_vehicle_seat_%s" % (VAR_CLASS.get(cls, cls), SEATS[real_seat(seat)])


def discover(anims_dir):
    """(classes with enter clips, classes with passenger idle sets): both genders and both seats must be present for each class;
    the passenger idle sets are optional per class, but then both genders are needed."""
    found = {}
    for f in sorted(Path(anims_dir).glob("utpp_car_*.anims")):
        m = CLIP_RE.match(f.name)
        if not m:
            raise SystemExit("unexpected file name: " + f.name)
        if m.group(3) == EXIT_SEAT:
            continue   # optional per gender: see discover_exits
        found.setdefault(m.group(2), set()).add((m.group(1), m.group(3)))
    if not found:
        raise SystemExit("no utpp_car_*.anims in " + str(anims_dir))
    idle = set()
    for cls, have in found.items():
        want = {(g, s) for g in "wm" for s in SEATS}
        want_idle = {(g, IDLE_SEAT) for g in "wm"}
        if cls not in CLASS_DIR or not (have == want or have == want | want_idle):
            raise SystemExit("class %s: have %s, want %s (+ optionally %s; known classes %s)"
                             % (cls, sorted(have), sorted(want), sorted(want_idle), sorted(CLASS_DIR)))
        if have == want | want_idle:
            idle.add(cls)
    return sorted(found), frozenset(idle)


def discover_exits(anims_dir, classes):
    """frozenset of (gender, class) with a utpp_car_p?a_<class>_driver_exit.anims; the class must have its enter sets (discover)."""
    out = set()
    for f in sorted(Path(anims_dir).glob("utpp_car_*_%s.anims" % EXIT_SEAT)):
        m = CLIP_RE.match(f.name)
        if not m or m.group(2) not in classes:
            raise SystemExit("exit set without the class entry sets / bad name: " + f.name)
        out.add((m.group(1), m.group(2)))
    return frozenset(out)


def seats_of(cls, idle):
    return list(SEATS) + ([IDLE_SEAT] if cls in idle else [])


def new_items(classes, idle=frozenset(), exits=frozenset(), g=None):
    """(prio, class, seat) in append order: all classes / seats (driver, passenger_r, then passenger_r_idle where present) at the first
    priority, then the same at the second; then (exits, gender g) the driver exit sets of that gender at both priorities."""
    items = [(p, c, s) for p in PRIOS for c in classes for s in seats_of(c, idle)]
    return items + [(p, c, EXIT_SEAT) for p in PRIOS for c in classes if (g, c) in exits]


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


def build_expected(vanilla, classes, idle=frozenset(), exits=frozenset()):
    doc = copy.deepcopy(vanilla)
    seen = set()
    for fa in fa_list(doc):
        g = RIGS.get(rig_of(fa))
        if not g:
            continue
        g = g[1]  # 'w' / 'm'
        seen.add(g)
        gp = fa["animations"]["gameplay"]
        lh = fa["loadingHandles"]
        if len(gp) != VANILLA_COUNT or len(lh) != VANILLA_COUNT:
            raise SystemExit("unexpected vanilla %s: %d entries / %d handles" % (g, len(gp), len(lh)))
        for prio, cls, seat in new_items(classes, idle, exits, g):
            vp = vanilla_path(g, cls, seat)
            tmpl_e = next((e for e in gp[:VANILLA_COUNT] if path_of(e) == vp), None)
            tmpl_h = next((h for h in lh[:VANILLA_COUNT] if path_of(h) == vp), None)
            if tmpl_e is None or tmpl_h is None:
                raise SystemExit("template %s not found in rig %s" % (vp, g))
            vt = tmpl_e["variableNames"]
            if [v["$value"] for v in vt] != [seat_var(cls, seat)]:
                raise SystemExit("template %s has vars %s, want [%s]" % (vp, [v["$value"] for v in vt], seat_var(cls, seat)))
            p = clip_path(g, cls, seat)
            e = copy.deepcopy(tmpl_e)
            e["animSet"]["DepotPath"]["$value"] = p
            e["priority"] = prio
            e["variableNames"] = [cname(vt[0], WRAPPER), cname(vt[0], seat_var(cls, seat))]
            gp.append(e)
            h = copy.deepcopy(tmpl_h)
            h["DepotPath"]["$value"] = p
            lh.append(h)
    if seen != {"w", "m"}:
        raise SystemExit("player rigs missing in the vanilla workspot: %s" % sorted({"w", "m"} - seen))
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


def strip_added(doc):
    """Undo the UTPP change on a copy: drop appended entries / handles of the player rigs."""
    out = copy.deepcopy(doc)
    for fa in fa_list(out):
        if rig_of(fa) in RIGS:
            fa["animations"]["gameplay"] = fa["animations"]["gameplay"][:VANILLA_COUNT]
            fa["loadingHandles"] = fa["loadingHandles"][:VANILLA_COUNT]
    return out


def verify(rt, expected, vanilla, classes, idle=frozenset(), exits=frozenset()):
    """Return a list of errors (empty = pass)."""
    errs = []
    d = first_diff(body(expected), body(rt))
    if d:
        errs.append("round trip != expected at " + d)
    d = first_diff(body(vanilla), body(strip_added(rt)))
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
        g = g[1]
        gp, lh = fa["animations"]["gameplay"], fa["loadingHandles"]
        n = VANILLA_COUNT + len(new_items(classes, idle, exits, g))
        if len(gp) != n or len(lh) != n:
            errs.append("%s: %d entries / %d handles, want %d" % (g, len(gp), len(lh), n))
            continue
        for (prio, cls, seat), e, h in zip(new_items(classes, idle, exits, g), gp[VANILLA_COUNT:], lh[VANILLA_COUNT:]):
            want = clip_path(g, cls, seat)
            vars_ = [v["$value"] for v in e["variableNames"]]
            if path_of(e) != want or path_of(h) != want or e["priority"] != prio or vars_ != [WRAPPER, seat_var(cls, seat)]:
                errs.append("%s: bad entry %s %s prio %s vars %s" % (g, path_of(e), path_of(h), e["priority"], vars_))
        if fa["animations"]["hash"] != vfa[r]["animations"]["hash"] or vfa[r]["animations"]["hash"] in ("0", 0):
            errs.append("%s: hash %s, want the vanilla %s" % (g, fa["animations"]["hash"], vfa[r]["animations"]["hash"]))
    return errs


def selftest(rt, expected, vanilla, classes, idle=frozenset(), exits=frozenset()):
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
                fa["animations"]["hash"] = "0"
                return

    def m_vanilla_entry(d):
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                fa["animations"]["gameplay"][7]["priority"] = 127
                return

    def m_tree(d):
        d["Data"]["RootChunk"]["workspotTree"]["Data"]["inertializationDurationEnter"] = 0.25

    def m_seat(d):  # driver entry bound to the passenger seat
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                fa["animations"]["gameplay"][VANILLA_COUNT]["variableNames"][1]["$value"] = seat_var(classes[0], "passenger_r")
                return

    def m_back_seat(d):  # a vanilla back-seat entry removed
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                del fa["animations"]["gameplay"][9]
                return

    def first_idle_entry(d):
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                for e in fa["animations"]["gameplay"][VANILLA_COUNT:]:
                    if path_of(e).endswith("_%s.anims" % IDLE_SEAT):
                        return e
        return None

    def m_idle_var(d):  # idle set bound to the wrong seat (the driver variable)
        e = first_idle_entry(d)
        e["variableNames"][1]["$value"] = seat_var(classes[0], "driver")

    def m_idle_wrapper(d):  # idle set without utpp_vehicle_tpp: it would also apply in first person
        first_idle_entry(d)["variableNames"].pop(0)

    def m_idle_prio(d):  # idle set behind the default set (254)
        first_idle_entry(d)["priority"] = 255

    def m_idle_gone(d):  # an idle entry removed (counts and handles no longer match)
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                for i, e in enumerate(fa["animations"]["gameplay"][VANILLA_COUNT:]):
                    if path_of(e).endswith("_%s.anims" % IDLE_SEAT):
                        del fa["animations"]["gameplay"][VANILLA_COUNT + i]
                        return

    def first_exit_entry(d):
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                for e in fa["animations"]["gameplay"][VANILLA_COUNT:]:
                    if path_of(e).endswith("_%s.anims" % EXIT_SEAT):
                        return e
        return None

    def m_exit_var(d):  # exit set bound to the passenger seat
        first_exit_entry(d)["variableNames"][1]["$value"] = seat_var(classes[0], "passenger_r")

    def m_exit_wrapper(d):  # exit set without utpp_vehicle_tpp: it would also apply in first person
        first_exit_entry(d)["variableNames"].pop(0)

    def m_exit_prio(d):  # exit set behind the vanilla driver set (128)
        first_exit_entry(d)["priority"] = 255

    def m_exit_gone(d):  # an exit entry removed (counts and handles no longer match)
        for fa in fa_list(d):
            if rig_of(fa) in RIGS:
                for i, e in enumerate(fa["animations"]["gameplay"][VANILLA_COUNT:]):
                    if path_of(e).endswith("_%s.anims" % EXIT_SEAT):
                        del fa["animations"]["gameplay"][VANILLA_COUNT + i]
                        return

    muts = [("NPC rig entry", m_npc), ("missing handle", m_handle), ("entry without utpp_vehicle_tpp", m_fpp_var),
            ("hash", m_hash), ("vanilla player entry", m_vanilla_entry), ("tree field", m_tree),
            ("wrong seat var", m_seat), ("vanilla back seat removed", m_back_seat)]
    if idle:
        muts += [("idle set on the driver seat", m_idle_var), ("idle set without utpp_vehicle_tpp", m_idle_wrapper),
                 ("idle set behind the default 254", m_idle_prio), ("idle entry removed", m_idle_gone)]
    if exits:
        muts += [("exit set on the passenger seat", m_exit_var), ("exit set without utpp_vehicle_tpp", m_exit_wrapper),
                 ("exit set behind the default 254", m_exit_prio), ("exit entry removed", m_exit_gone)]
    bad = 0
    for name, m in muts:
        d = copy.deepcopy(rt)
        m(d)
        caught = bool(verify(d, expected, vanilla, classes, idle, exits))
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
    ap.add_argument("--anims-dir", required=True, help="dir holding the utpp_car_p?a_<class>_<seat>.anims")
    ap.add_argument("--vanilla-json", default=str(VANILLA_JSON))
    ap.add_argument("--vanilla-bin", default=str(VANILLA_BIN), help="'' to skip the vanilla binary check")
    ap.add_argument("--wkit-exe", default=str(WKIT_DEFAULT))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
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
    classes, idle = discover(a.anims_dir)
    exits = discover_exits(a.anims_dir, classes)
    print("classes with UTPP clips:", classes, "| with passenger idle sets (tpp_idle):", sorted(idle), "| driver exit sets (gender, class):", sorted(exits))
    expected = build_expected(vanilla, classes, idle, exits)
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
    errs = verify(rt, expected, vanilla, classes, idle, exits)
    for e in errs:
        print("ERROR", e)
    if errs:
        return 1
    for fa in fa_list(rt):
        g = RIGS.get(rig_of(fa))
        if g:
            print("rig %s: %d entries, %d handles, hash %s" % (g, len(fa["animations"]["gameplay"]),
                                                              len(fa["loadingHandles"]), fa["animations"]["hash"]))
    print("workspot ok: %s sha256 %s (%d bytes), only the appended player-rig entries / handles differ from vanilla"
          % (built, sha256(built), built.stat().st_size))
    shutil.rmtree(work)
    if a.selftest and selftest(rt, expected, vanilla, classes, idle, exits):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
