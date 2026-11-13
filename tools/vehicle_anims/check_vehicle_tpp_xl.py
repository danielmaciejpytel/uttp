"""Offline check of plugin/UTPP/archive/utpp_vehicle_tpp.archive.xl (MOTO-2).

1. Schema: the rules of ArchiveXL's animation config loader (src/App/Extensions/Animation/Config.cpp, LoadYAML):
   'animations' is a list; each entry has 'entity' (scalar or list, non-empty) and 'set' (scalar); optional 'vars'
   (list of scalars), 'priority' (integer, stored as uint8 0..255, default 128) and 'component' (scalar).
   Parsed with a small YAML-subset reader (block maps/lists, flow lists, comments); anything else is an error.
2. Data, against the WolvenKit extraction table Research/inventory/vehicle-anims-2.31.json (vehicle_anims_check.py):
   every entity and set path exists in the game archives; the component is an entAnimatedComponent of that entity
   (ArchiveXL merges into entAnimatedComponent::InitializeAnimations by component name); the set's rig equals the
   component rig; the vanilla entity registers the same set with [player_tpp_head, <seat>] and the entry uses
   [utpp_vehicle_tpp, <same seat>]; the priority is lower than every vanilla entry of that seat variable (inferred
   precedence: lower number wins - runtime unverified); every player FPP entity gets all three TPP sets of its gender.
Seat pop diagnosis 2026-10-28: root-merged bike entries were never seen to play (every TPP ride showed the vanilla FPP enter), so the
same TPP riding sets are also registered on an own extension component next to MotorcycleAnimsets (route 'ext',
resource section of tools/anim/vehicle-entry/utpp_vehicle_bike.archive.xl, checked by check_bike_xl.py --ext-json). This file keeps the root entries
unchanged (priority 95, rule above) so the routes stay attributable in game.
--selftest runs the checks on mutated copies (each must fail) and on utpp_nue_reload.archive.xl (schema only, must pass).
Exit 0 = all checks passed.
"""
import argparse
import json
import os
import re
import sys

WRAPPER = "utpp_vehicle_tpp"


# ------------------------------------------------------------------ YAML subset
class YamlError(Exception):
    pass


def _scalar(s):
    s = s.strip()
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "'\"":
        return s[1:-1]
    if s.startswith("[") or s.startswith("{") or s.startswith("&") or s.startswith("*") or s.startswith("|") or s.startswith(">"):
        raise YamlError("unsupported scalar: " + s)
    return s


def _value(s):
    s = s.strip()
    if s.startswith("["):
        if not s.endswith("]"):
            raise YamlError("unterminated flow list: " + s)
        inner = s[1:-1].strip()
        return [] if not inner else [_scalar(x) for x in inner.split(",")]
    return _scalar(s)


def parse_yaml(text):
    lines = []
    for n, raw in enumerate(text.splitlines(), 1):
        if "\t" in raw[: len(raw) - len(raw.lstrip())]:
            raise YamlError(f"line {n}: tab indentation")
        s = raw.split(" #")[0].rstrip() if not raw.lstrip().startswith("#") else ""
        if s.strip():
            lines.append((n, len(s) - len(s.lstrip()), s.strip()))
    pos = [0]

    def block(indent):
        if pos[0] >= len(lines):
            return None
        n, ind, s = lines[pos[0]]
        if ind < indent:
            return None
        if s.startswith("- ") or s == "-":
            return seq(ind)
        return mapping(ind)

    def seq(ind):
        out = []
        while pos[0] < len(lines):
            n, i, s = lines[pos[0]]
            if i != ind or not (s.startswith("- ") or s == "-"):
                if i > ind:
                    raise YamlError(f"line {n}: bad indentation")
                break
            rest = s[1:].strip()
            if not rest:
                pos[0] += 1
                out.append(block(ind + 1))
            elif re.match(r"^[A-Za-z_][\w]*\s*:", rest):
                # "- key: value" starts a mapping whose keys sit at ind + 2
                lines[pos[0]] = (n, ind + 2, rest)
                out.append(mapping(ind + 2))
            else:
                pos[0] += 1
                out.append(_value(rest))
        return out

    def mapping(ind):
        out = {}
        while pos[0] < len(lines):
            n, i, s = lines[pos[0]]
            if i != ind:
                if i > ind:
                    raise YamlError(f"line {n}: bad indentation")
                break
            m = re.match(r"^([A-Za-z_][\w]*)\s*:\s*(.*)$", s)
            if not m:
                raise YamlError(f"line {n}: expected 'key: value', got {s!r}")
            k, v = m.group(1), m.group(2)
            if k in out:
                raise YamlError(f"line {n}: duplicate key {k}")
            pos[0] += 1
            if v:
                out[k] = _value(v)
            else:
                nxt = lines[pos[0]] if pos[0] < len(lines) else None
                if nxt and nxt[1] > ind:
                    out[k] = block(nxt[1])
                elif nxt and nxt[1] == ind and nxt[2].startswith("- "):
                    out[k] = seq(ind)
                else:
                    out[k] = None
        return out

    doc = block(0)
    if pos[0] != len(lines):
        n = lines[pos[0]][0]
        raise YamlError(f"line {n}: trailing content")
    return doc


# ------------------------------------------------------------------ checks
def check_schema(doc):
    """Mirror of ArchiveXL AnimationsConfig::LoadYAML. Returns (entries, errors)."""
    errs, entries = [], []
    if not isinstance(doc, dict) or "animations" not in doc:
        return entries, ["no 'animations' key"]
    anims = doc["animations"]
    if not isinstance(anims, list):
        return entries, ["'animations' is not a list (ArchiveXL: Bad format. Expected list of anim entries.)"]
    for i, e in enumerate(anims):
        if not isinstance(e, dict):
            errs.append(f"entry {i}: not a mapping")
            continue
        ent = e.get("entity")
        ents = [ent] if isinstance(ent, str) else [x for x in ent if isinstance(x, str)] if isinstance(ent, list) else []
        if not ents:
            errs.append(f"entry {i}: entity missing or empty (malformed, skipped by ArchiveXL)")
            continue
        st = e.get("set")
        if not isinstance(st, str) or not st:
            errs.append(f"entry {i}: set missing or not a scalar (malformed)")
            continue
        vars_ = e.get("vars", [])
        if vars_ is not None and not (isinstance(vars_, list) and all(isinstance(v, str) for v in vars_)):
            errs.append(f"entry {i}: vars is not a list of scalars")
            continue
        prio = e.get("priority", "128")
        if not re.fullmatch(r"\d+", str(prio)) or not 0 <= int(prio) <= 255:
            errs.append(f"entry {i}: priority {prio!r} is not an integer 0..255 (uint8)")
            continue
        comp = e.get("component")
        if comp is not None and not isinstance(comp, str):
            errs.append(f"entry {i}: component not a scalar")
            continue
        unknown = set(e) - {"entity", "set", "vars", "priority", "component"}
        if unknown:
            errs.append(f"entry {i}: unknown keys {sorted(unknown)} (ignored by ArchiveXL)")
        entries.append({"entities": ents, "set": st, "vars": list(vars_ or []), "priority": int(prio), "component": comp})
    return entries, errs


def check_data(entries, table):
    errs = []
    files = {k.lower(): v for k, v in table["files"].items()}
    ents = table["entities"]
    sets = table["animsets"]
    covered = {}
    for i, e in enumerate(entries):
        sp = e["set"]
        sname = sp.split("\\")[-1]
        if sp.lower() not in files:
            errs.append(f"entry {i}: set {sp} not found in the game archives (WolvenKit listing)")
            continue
        m = re.match(r"tpp_p(w|m)a_sportbike(\d?)\.anims$", sname)
        if not m:
            errs.append(f"entry {i}: {sname} is not a game TPP bike set")
            continue
        gender, num = m.group(1), m.group(2) or "1"
        seat = f"sportbike{num}_vehicle_seat_front_left"
        if e["vars"] != [WRAPPER, seat]:
            errs.append(f"entry {i}: vars {e['vars']} != [{WRAPPER}, {seat}]")
        if e["component"] != "root":
            errs.append(f"entry {i}: component {e['component']!r} != 'root'")
        set_rig = sets.get(sname[:-6], {}).get("rig")
        for ep in e["entities"]:
            if ep.lower() not in files:
                errs.append(f"entry {i}: entity {ep} not found in the game archives")
                continue
            en = ep.split("\\")[-1][:-4]
            if not re.match(rf"player_{gender}a_fpp(_ep1)?$", en):
                errs.append(f"entry {i}: {sname} (gender {gender}) registered on {en}")
            info = ents.get(en)
            if not info:
                errs.append(f"entry {i}: {en} missing from the table")
                continue
            comps = [c for c in info["components"] if c["where"] == "components" and c["name"] == e["component"]]
            if not comps or comps[0]["type"] != "entAnimatedComponent":
                errs.append(f"entry {i}: {en} has no entAnimatedComponent named {e['component']!r}")
            elif comps[0]["rig"] != set_rig:
                errs.append(f"entry {i}: rig mismatch {set_rig} vs {en}:{e['component']} {comps[0]['rig']}")
            vanilla = [x for c in info["components"] if c["where"] == "components" for x in c["vehicle_entries"]]
            same = [x for x in vanilla if x["set"].lower() == sp.lower()]
            if not any(x["vars"] == ["player_tpp_head", seat] for x in same):
                errs.append(f"entry {i}: {en} has no vanilla [{'player_tpp_head'}, {seat}] entry for {sname}")
            rivals = [x["priority"] for x in vanilla if seat in x["vars"]]
            if not rivals or not e["priority"] < min(rivals):
                errs.append(f"entry {i}: priority {e['priority']} not below the vanilla {seat} entries {sorted(set(rivals))}")
            covered.setdefault(en, set()).add(sname)
    for en in ents:
        mm = re.match(r"player_(w|m)a_fpp(_ep1)?$", en)
        if mm:
            g = mm.group(1)
            want = {f"tpp_p{g}a_sportbike.anims", f"tpp_p{g}a_sportbike2.anims", f"tpp_p{g}a_sportbike3.anims"}
            if covered.get(en, set()) != want:
                errs.append(f"coverage: {en} has {sorted(covered.get(en, set()))}, expected {sorted(want)}")
    return errs


def run(xl_text, table):
    try:
        doc = parse_yaml(xl_text)
    except YamlError as ex:
        return ["yaml: " + str(ex)], []
    entries, errs = check_schema(doc)
    if errs:
        return errs, entries
    return check_data(entries, table), entries


def selftest(xl_text, table, nue_text):
    muts = [
        ("priority 100 (ties the vanilla TPP entry)", lambda s: s.replace("priority: 95", "priority: 100", 1)),
        ("priority 300 (not uint8)", lambda s: s.replace("priority: 95", "priority: 300", 1)),
        ("wrong seat variable", lambda s: s.replace("sportbike2_vehicle_seat_front_left", "sportbike1_vehicle_seat_front_left", 1)),
        ("vanilla wrapper instead of UTPP", lambda s: s.replace("[utpp_vehicle_tpp,", "[player_tpp_head,", 1)),
        ("component MotorcycleAnimsets", lambda s: s.replace("component: root", "component: MotorcycleAnimsets", 1)),
        ("ep1 entity dropped", lambda s: s.replace("      - ep1\\characters\\entities\\player\\player_wa_fpp_ep1.ent\n", "", 1)),
        ("female set on male entity", lambda s: s.replace("tpp_pma_sportbike3.anims", "tpp_pwa_sportbike3.anims", 1)),
        ("missing set path", lambda s: s.replace("tpp_pwa_sportbike2.anims", "tpp_pwa_sportbike9.anims", 1)),
        ("vars not a list", lambda s: s.replace("vars: [utpp_vehicle_tpp, sportbike1_vehicle_seat_front_left]", "vars: utpp_vehicle_tpp", 1)),
    ]
    ok = True
    for name, f in muts:
        mutated = f(xl_text)
        if mutated == xl_text:
            print(f"SELFTEST ERROR mutation not applied: {name}")
            ok = False
            continue
        errs, _ = run(mutated, table)
        print(f"mutation '{name}': {'FAIL as expected' if errs else 'PASSED (BAD)'}"
              + (f" - {errs[0]}" if errs else ""))
        ok &= bool(errs)
    if nue_text is not None:
        try:
            ents, errs = check_schema(parse_yaml(nue_text))
        except YamlError as ex:
            ents, errs = [], [str(ex)]
        print(f"schema of utpp_nue_reload.archive.xl (known good in game): {'PASS' if not errs and len(ents) == 3 else 'FAIL'} "
              f"entries={len(ents)} {errs[:1]}")
        ok &= not errs and len(ents) == 3
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xl", required=True)
    ap.add_argument("--table", required=True)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--nue-xl")
    a = ap.parse_args()
    with open(a.xl, "r", encoding="utf-8") as f:
        text = f.read()
    with open(a.table, "r", encoding="utf-8") as f:
        table = json.load(f)
    errs, entries = run(text, table)
    for e in errs:
        print("ERROR", e)
    print(f"{os.path.basename(a.xl)}: entries={len(entries)} "
          f"registrations={sum(len(e['entities']) for e in entries)} errors={len(errs)}")
    rc = 0 if not errs else 1
    if a.selftest:
        nue = None
        if a.nue_xl:
            with open(a.nue_xl, "r", encoding="utf-8") as f:
                nue = f.read()
        if not selftest(text, table, nue):
            print("SELFTEST FAILED")
            rc = 1
        else:
            print("SELFTEST OK")
    return rc


if __name__ == "__main__":
    sys.exit(main())
