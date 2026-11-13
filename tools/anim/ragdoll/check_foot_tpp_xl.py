"""Offline check of plugin/UTPP/archive/utpp_foot_tpp.archive.xl (on-foot idle set, wrapper variable utpp_foot_tpp).

1. Schema: the rules of ArchiveXL's animation config loader (shared with tools/vehicle_anims/check_vehicle_tpp_xl.py).
2. Content: two entries (female / male), each on the base player entity and its ep1 entity, component 'root', the gender's
   player_locomotion.anims, vars exactly [utpp_foot_tpp], priority below the security-camera set (199) and above the 128
   variant sets (per-weapon and variant locomotion sets) - 198.
3. Same wrapper name in the pieces that use it: scripts/utpp_body.reds (AnimWrapperWeightSetter key), cet/utpp_body/init.lua
   (Game.UTPPSetFootTpp) and tools/anim/ragdoll/disable_securitycam_entry.py (graph condition).
4. Optional --ent-json (WolvenKit serialization of the vanilla player entities, 2.5): the root component registers the
   locomotion set at 200 without variables and tpp_p?a_security_cam.anims at 199 with [player_tpp_head]; every other entry on
   the component has either a variable list or a path other than the locomotion set at a priority that does not tie 198.
--selftest runs the checks on mutated copies (each must fail).
usage: python -I check_foot_tpp_xl.py [--repo ROOT] [--ent-json FILE ...] [--selftest]   Exit 0 = all checks passed."""
import argparse
import importlib.util
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WRAPPER = "utpp_foot_tpp"
SECCAM_PRIORITY = 199
VARIANT_PRIORITY = 128
EXPECT = {
    "w": {"entities": ["base\\characters\\entities\\player\\player_wa_fpp.ent",
                       "ep1\\characters\\entities\\player\\player_wa_fpp_ep1.ent"],
          "set": "base\\animations\\player\\female\\body\\player_locomotion.anims"},
    "m": {"entities": ["base\\characters\\entities\\player\\player_ma_fpp.ent",
                       "ep1\\characters\\entities\\player\\player_ma_fpp_ep1.ent"],
          "set": "base\\animations\\player\\male\\body\\player_locomotion.anims"},
}


def load_vehicle_checker(repo):
    path = os.path.join(repo, "tools", "vehicle_anims", "check_vehicle_tpp_xl.py")
    spec = importlib.util.spec_from_file_location("check_vehicle_tpp_xl", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def check_content(entries):
    errs = []
    if len(entries) != 2:
        errs.append(f"expected 2 entries (female, male), found {len(entries)}")
    seen = set()
    for i, e in enumerate(entries):
        gender = "w" if "female" in e["set"] else "m" if "\\male\\" in e["set"] else None
        if gender is None:
            errs.append(f"entry {i}: set {e['set']} is neither the female nor the male locomotion set")
            continue
        seen.add(gender)
        want = EXPECT[gender]
        if e["set"] != want["set"]:
            errs.append(f"entry {i}: set {e['set']} != {want['set']}")
        if sorted(x.lower() for x in e["entities"]) != sorted(x.lower() for x in want["entities"]):
            errs.append(f"entry {i}: entities {e['entities']} != {want['entities']}")
        if e["component"] != "root":
            errs.append(f"entry {i}: component {e['component']!r} != 'root'")
        if e["vars"] != [WRAPPER]:
            errs.append(f"entry {i}: vars {e['vars']} != [{WRAPPER}]")
        if not VARIANT_PRIORITY < e["priority"] < SECCAM_PRIORITY:
            errs.append(f"entry {i}: priority {e['priority']} must be below the security-camera set ({SECCAM_PRIORITY}) "
                        f"and above the {VARIANT_PRIORITY} variant sets")
    if seen != {"w", "m"} and len(entries) == 2:
        errs.append(f"genders covered {sorted(seen)}")
    return errs


def check_consistency(repo, graph_script_text=None):
    errs = []
    reds = open(os.path.join(repo, "plugin", "UTPP", "scripts", "utpp_body.reds"), encoding="utf-8").read()
    if f'evt.key = n"{WRAPPER}"' not in reds or "func UTPPSetFootTpp" not in reds:
        errs.append("utpp_body.reds: no UTPPSetFootTpp with key " + WRAPPER)
    lua = open(os.path.join(repo, "plugin", "UTPP", "cet", "utpp_body", "init.lua"), encoding="utf-8").read()
    if "Game.UTPPSetFootTpp" not in lua or "SetFootTpp" not in lua:
        errs.append("utpp_body/init.lua: no call of UTPPSetFootTpp")
    g = graph_script_text
    if g is None:
        g = open(os.path.join(repo, "tools", "anim", "ragdoll", "disable_securitycam_entry.py"), encoding="utf-8").read()
    if f'WRAPPER = "{WRAPPER}"' not in g:
        errs.append("disable_securitycam_entry.py: wrapper name differs")
    return errs


def check_ents(paths):
    """Vanilla facts the entry relies on, read from WolvenKit-serialized player entities."""
    errs = []
    for p in paths:
        d = json.load(open(p, encoding="utf-8"))
        roots = [c for c in d["Data"]["RootChunk"]["components"]
                 if c.get("$type") == "entAnimatedComponent" and c["name"]["$value"] == "root"]
        if len(roots) != 1:
            errs.append(f"{os.path.basename(p)}: {len(roots)} root animated components")
            continue
        loco, seccam, rivals = [], [], []
        for e in roots[0]["animations"]["gameplay"]:
            path = e["animSet"]["DepotPath"]["$value"].lower()
            vars_ = [v["$value"] if isinstance(v, dict) else v for v in e.get("variableNames", [])]
            if path.endswith("\\body\\player_locomotion.anims"):
                loco.append((e["priority"], vars_))
            elif path.endswith("_security_cam.anims"):
                seccam.append((e["priority"], vars_))
            elif e["priority"] >= 198 and vars_:
                rivals.append((e["priority"], path, vars_))
        name = os.path.basename(p)
        if (200, []) not in loco:
            errs.append(f"{name}: no player_locomotion.anims at 200 without variables")
        if seccam != [(SECCAM_PRIORITY, ["player_tpp_head"])]:
            errs.append(f"{name}: security-camera set entries {seccam}, expected [(199, ['player_tpp_head'])]")
        if rivals:
            errs.append(f"{name}: other variable-gated entries at 198 or above: {rivals}")
    return errs


def run(xl_text, repo, ents):
    chk = load_vehicle_checker(repo)
    try:
        doc = chk.parse_yaml(xl_text)
    except chk.YamlError as ex:
        return ["yaml: " + str(ex)], []
    entries, errs = chk.check_schema(doc)
    if errs:
        return errs, entries
    return check_content(entries) + check_consistency(repo) + check_ents(ents), entries


def selftest(xl_text, repo, ents):
    muts = [
        ("priority 199 (ties the security-camera set)", lambda s: s.replace("priority: 198", "priority: 199", 1)),
        ("priority 200", lambda s: s.replace("priority: 198", "priority: 200", 1)),
        ("priority 128 (ties the variant sets)", lambda s: s.replace("priority: 198", "priority: 128", 1)),
        ("priority 300 (not uint8)", lambda s: s.replace("priority: 198", "priority: 300", 1)),
        ("vanilla variable instead of the UTPP wrapper", lambda s: s.replace("[utpp_foot_tpp]", "[player_tpp_head]", 1)),
        ("extra variable", lambda s: s.replace("[utpp_foot_tpp]", "[utpp_foot_tpp, player_tpp_head]", 1)),
        ("wrong component", lambda s: s.replace("component: root", "component: shadow", 1)),
        ("ep1 female entity dropped", lambda s: s.replace("      - ep1\\characters\\entities\\player\\player_wa_fpp_ep1.ent\n", "", 1)),
        ("female set on the male entity", lambda s: s.replace("    set: base\\animations\\player\\male\\body", "    set: base\\animations\\player\\female\\body", 1)),
        ("security-camera set registered instead", lambda s: s.replace("body\\player_locomotion.anims\n    vars", "body\\tpp_pwa_security_cam.anims\n    vars", 1)),
        ("vars not a list", lambda s: s.replace("vars: [utpp_foot_tpp]", "vars: utpp_foot_tpp", 1)),
        ("entry dropped", lambda s: s[:s.index("  - entity:", s.index("  - entity:") + 1)]),
    ]
    ok = True
    for name, f in muts:
        mutated = f(xl_text)
        if mutated == xl_text:
            print(f"SELFTEST ERROR mutation not applied: {name}")
            ok = False
            continue
        errs, _ = run(mutated, repo, [])
        print(f"mutation '{name}': {'FAIL as expected' if errs else 'PASSED (BAD)'}" + (f" - {errs[0]}" if errs else ""))
        ok &= bool(errs)
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=os.path.abspath(os.path.join(HERE, "..", "..", "..")))
    ap.add_argument("--ent-json", action="append", default=[])
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    xl = os.path.join(a.repo, "plugin", "UTPP", "archive", "utpp_foot_tpp.archive.xl")
    text = open(xl, encoding="utf-8").read()
    errs, entries = run(text, a.repo, a.ent_json)
    for e in errs:
        print("ERROR", e)
    print(f"utpp_foot_tpp.archive.xl: entries={len(entries)} registrations={sum(len(e['entities']) for e in entries)} "
          f"entity files checked={len(a.ent_json)} errors={len(errs)}")
    rc = 0 if not errs else 1
    if a.selftest:
        if selftest(text, a.repo, a.ent_json):
            print("SELFTEST OK")
        else:
            print("SELFTEST FAILED")
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
