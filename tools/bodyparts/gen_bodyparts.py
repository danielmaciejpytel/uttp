"""UTPP body part table generator (Phase 5N step 3b, layer L2).

Reads vanilla Cyberpunk 2077 2.50 data (the only supported game version; default) that was extracted read-only with
WolvenKit.CLI (unbundle + convert serialize) by extract_body_parts.ps1 + serialize_mirrored.ps1 (same folder as this script):
  * Extracted\\game\\body-parts-250\\json\\{base,ep1}\\characters\\common\\player_base_bodies\\appearances\\**\\*.app.json (260 .app files)
  * Extracted\\game\\body-parts-250\\json\\{base,ep1}\\characters\\entities\\player\\player_{wa,ma}_fpp{,_ep1}.ent.json    (player entities)
  * Extracted\\game\\body-parts-250\\json\\base\\characters\\head\\p{w,m}a\\player_{w,m}a_tpp_head.ent.json                (TPP heads)
Option --src231 (legacy, comparison only) reads the older 2.31 extraction instead:
  * Extracted\\game\\vanilla-bodies\\...\\appearances\\**\\*.app.json, Extracted\\game\\json\\vanilla\\player_*.ent.json,
    Extracted\\game\\vanilla-app\\base\\characters\\head\\...
and writes the CET data table Source\\plugin\\UTPP\\cet\\utpp_body\\body_parts.lua.

Classification (per gender, key = component name + mesh/morphtarget path hash):
  * every FPP appearance file is paired with its TPP/base counterpart (PAIRS below, explicit and reviewable; quest
    items: "<name>_fpp" <-> "<name>" inside one file); the neck has no TPP counterpart (the TPP head has a neck);
  * per gender, F = all (name, hash) of FPP appearances, T = all of TPP appearances, U = all of unpaired files
    (unholstered cyberware, genitals, scars, muscularity, tattoos ...):
    F and T -> "shared" (never touched), F only -> "fppOnly" (hidden in TPP), T only -> "tppOnly" (enabled in TPP
    when the game left it disabled) if it is in every TPP appearance of that gender in a TPP file, otherwise
    "tppVariant" (underwear / doll / scar variants: never touched), U only -> "shared",
    F only but also in U -> "ambiguous" (never touched at runtime);
    1.3 exception (pimp_chrome): a part whose ONLY TPP source is a chrome set (arms/<kind>/pimp_chrome/...) while its
    FPP side has a non-chrome source is classified as if the chrome TPP evidence did not exist (fppOnly), because the
    base TPP set of that part uses another mesh (female mantisblade: photomode_upperarm_*) and the runtime classifier
    knows only name + hash, not the active appearance set. Compromise: a chrome TPP set that reuses such a mesh
    loses it in TPP (hidden), instead of the base TPP showing a doubled FPP upperarm;
  * components of the player entities -> "entity", components of the TPP head entities -> "head" (both owned by the
    game: never touched), except (1.1, 2026-10-07, standalone test 2) two kinds of FPP-only entity meshes the full TPP body
    duplicates: shadow-only proxies ("*_shadow", "*shadowmesh": the FPP body's shadow; the TPP body casts its own ->
    two shadows) -> "shadowProxy", and meshes driven by the anim parameter renderingPlaneAnimationParam = "renderPlane"
    (FPP fists, weapon grip cyberware, FPP lights: drawn on the weapon plane over the scene -> visible through the TPP
    body) -> "renderPlaneFpp". Both are disabled in TPP and restored in FPP by utpp_body (only when the game enabled them).
Hash = FNV-1a 64 of the lower-case depot path with backslashes (= ResourcePath / ResRef hash; checked against
`WolvenKit.CLI hash`). Only names, paths and hashes of vanilla resources are written (interoperability facts, no assets).

Usage (default source = game 2.50):
  python gen_bodyparts.py                 # regenerate body_parts.lua from 2.50 + print the summary
  python gen_bodyparts.py --check         # regenerate in memory, compare with the file on disk, run the fact checks
  python gen_bodyparts.py --out <file>    # other output path
  python gen_bodyparts.py --src231 [...]  # legacy 2.31 extraction (comparison only; --src250 is accepted and = default)
To recreate the 2.50 source data: powershell -File extract_body_parts.ps1 ; powershell -File serialize_mirrored.ps1
Exit code 0 = ok, 1 = check failed.
"""
import argparse
import json
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

GENERATOR_VERSION = "1.3"
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "Source").is_dir() and (p / "Research").is_dir())
# 1.3 (2026-11-09): default source = game 2.50, the only supported version (Extracted\game\body-parts-250\json, mirrored
# depot layout, made by extract_body_parts.ps1 + serialize_mirrored.ps1 next to this script). 2.31 only behind --src231.
SRC250 = ROOT / "Extracted" / "game" / "body-parts-250" / "json"
GAME_VERSION = "2.50"
EP1_APP_DIR = SRC250 / "ep1" / "characters" / "common" / "player_base_bodies" / "appearances"  # pimp_chrome Barghest sets
APP_DIR = SRC250 / "base" / "characters" / "common" / "player_base_bodies" / "appearances"
ENT_DIR = SRC250 / "base" / "characters" / "entities" / "player"
ENT_EP1_DIR = SRC250 / "ep1" / "characters" / "entities" / "player"
HEAD_DIR = SRC250 / "base" / "characters" / "head"


def use_src231():
    """legacy 2.31 extraction layout (comparison only)"""
    global EP1_APP_DIR, APP_DIR, ENT_DIR, ENT_EP1_DIR, HEAD_DIR, GAME_VERSION
    GAME_VERSION = "2.31"
    APP_DIR = ROOT / "Extracted" / "game" / "vanilla-bodies" / "base" / "characters" / "common" / "player_base_bodies" / "appearances"
    ENT_DIR = ROOT / "Extracted" / "game" / "json" / "vanilla"
    ENT_EP1_DIR = ENT_DIR  # 2.31 layout: all four player entities in one folder
    EP1_APP_DIR = None
    HEAD_DIR = ROOT / "Extracted" / "game" / "vanilla-app" / "base" / "characters" / "head"

DEFAULT_OUT =ROOT / "Source" / "plugin" / "UTPP" / "cet" / "utpp_body" / "body_parts.lua"

# (FPP .app, TPP .app or None) relative to APP_DIR, without ".app"
PAIRS = [
    ("t0_000_fpp__full", "t0_000_base__full"),
    ("t0_000_fpp__full_censored", "t0_000_base__full_censored"),
    ("l0_000_fpp__full", "l0_000_base__full"),
    ("l0_000_fpp__cs_flat", "l0_000_base__cs_flat"),
    ("arms/a0_000_fpp__full", "a0_000_base__full"),
    ("a0_000_fpp__nails", "a0_000_base__nails"),
    ("i0_000_fpp__nipple", "i0_000_base__nipple"),
] + [("t0_000_fpp__tattoo_%02d" % i, "t0_000_base__tattoo_%02d" % i) for i in range(1, 8)] + [
    ("n0_000_base_fpp__neck", None),
    ("arms/launcher/a0_000_holstered_launcher__base", "arms/launcher/tpp/a0_000_holstered_launcher__base_tpp"),
    ("arms/launcher/a0_000_holstered_launcher__cyberware_01", "arms/launcher/tpp/a0_000_holstered_launcher__cyberware_01_tpp"),
    ("arms/mantisblade/a0_000_holstered_mantisblade__base", "arms/mantisblade/tpp/a0_000_holstered_mantisblade__base_tpp"),
    ("arms/mantisblade/a0_000_holstered_mantisblade__cyberware_01", "arms/mantisblade/tpp/a0_000_holstered_mantisblade__cyberware_01_tpp"),
    ("arms/monowire_whip/a0_000_holstered_monowire_whip", "arms/monowire_whip/tpp/a0_000_holstered_monowire_whip_tpp"),
    ("arms/personal_link/a0_000_holstered_personal_link", "arms/personal_link/tpp/a0_000_holstered_personal_link_tpp"),
    ("arms/strong_arms/a0_000_holstered_strong_arms__base", "arms/strong_arms/tpp/a0_000_holstered_strong_arms__base_tpp"),
    ("arms/strong_arms/a0_000_holstered_strong_arms__cyberware_01", "arms/strong_arms/tpp/a0_000_holstered_strong_arms__cyberware_01_tpp"),
    ("arms/weapon_grip/a0_000__weapon_grip", "arms/weapon_grip/tpp/a0_000__weapon_grip_tpp"),
]
# files whose appearances come in pairs inside the same file: "<name>_fpp" (FPP) <-> "<name>" (TPP)
INTRA_FILE_PAIRS = [
    "quest_items/a0_000__mud_q101",
    "quest_items/a0_000__tattoo_casius_r",
    "quest_items/a0_000__tattoo_silverhand_r",
    "quest_items/a0_000__tattoo_yakuza_r",
]
ENTITIES = {
    "female": ["player_wa_fpp.ent.json", "player_wa_fpp_ep1.ent.json"],
    "male": ["player_ma_fpp.ent.json", "player_ma_fpp_ep1.ent.json"],
}
HEADS = {"female": "pwa/player_wa_tpp_head.ent.json", "male": "pma/player_ma_tpp_head.ent.json"}
MESH_CLASSES = ("entSkinnedMeshComponent", "entGarmentSkinnedMeshComponent", "entMorphTargetSkinnedMeshComponent",
                "entMeshComponent", "entPhysicalMeshComponent", "entSkinnedClothComponent")


def fnv1a64(path):
    h = 0xCBF29CE484222325
    for b in path.lower().replace("/", "\\").encode("utf-8"):
        h ^= b
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


CHROME = "/pimp_chrome/"


def val(x):
    if isinstance(x, dict):
        if "DepotPath" in x:
            return val(x["DepotPath"])
        if "$value" in x:
            return x["$value"]
    return x


def resource(c):
    """-> (path or None, hash string or None)"""
    for field in ("mesh", "morphResource", "resource"):
        if field in c:
            v = val(c[field])
            if v is None or v in ("", 0, "0"):
                return None, None
            v = str(v)
            if v.isdigit():
                return None, v
            return v, str(fnv1a64(v))
    return None, None


def entity_class(c):
    """class of a player entity mesh component (1.1): shadowProxy | renderPlaneFpp | entity"""
    name = (val(c.get("name")) or "").lower()
    if name.endswith("_shadow") or "shadowmesh" in name or "_shadow_" in name:
        return "shadowProxy"
    if val(c.get("renderingPlaneAnimationParam")) == "renderPlane":
        return "renderPlaneFpp"
    return "entity"


def gender_of_app(app_name, comps):
    n = app_name.lower()
    if re.search(r"(^|_)(female|pwa|wa|w)(_|$)", n) or n.startswith("w__") or "_pwa_" in n:
        return "female"
    if re.search(r"(^|_)(male|pma|ma|m)(_|$)", n) or n.startswith("m__") or "_pma_" in n:
        return "male"
    paths = " ".join((p or "") for _, p, _ in comps).lower()
    f = "player_female_average" in paths or "_wa_" in paths or "woman_average" in paths
    m = "player_man_average" in paths or "_ma_" in paths or "man_average\\" in paths
    if f and not m:
        return "female"
    if m and not f:
        return "male"
    return None


def app_json(rel):
    f = APP_DIR / (rel.replace("/", "\\") + ".app.json")
    if not f.is_file() and EP1_APP_DIR is not None:
        f = EP1_APP_DIR / (rel.replace("/", "\\") + ".app.json")
    return f


def load_app(rel):
    f = app_json(rel)
    d = json.loads(f.read_text(encoding="utf-8"))
    out = []
    for a in d["Data"]["RootChunk"].get("appearances", []):
        ad = a.get("Data") or {}
        name = val(ad.get("name"))
        cd = (ad.get("compiledData") or {}).get("Data") or {}
        comps = []
        for c in cd.get("Chunks", []) if isinstance(cd, dict) else []:
            if c.get("$type") not in MESH_CLASSES:
                continue
            path, h = resource(c)
            if h is None:
                continue
            comps.append((val(c.get("name")), path, h))
        out.append((name, comps))
    return out


def all_app_files():
    found = set()
    for base in (APP_DIR, EP1_APP_DIR):
        if base is not None and base.is_dir():
            found.update(str(p.relative_to(base)).replace("\\", "/")[:-len(".app.json")] for p in base.rglob("*.app.json"))
    return sorted(found)


def pimp_chrome_pairs():
    """1.2 (game 2.50): chrome-skin variants (arms/<kind>/pimp_chrome/<brand>/...) follow the base cyberware naming:
    holstered FPP set "<stem>" <-> TPP twin "tpp/<stem>_tpp" (a "_longsleeved" suffix moves behind "_tpp"); the FPP
    nails set "a0_000_fpp_launcher__pmc_nails" <-> "a0_000_base_launcher__pmc_nails". Unholstered sets and "_preview"
    sets stay unpaired, as in the base game. Returns (pairs, holstered sets without a TPP twin)."""
    files = set(all_app_files())
    pairs, orphans = [], []
    for rel in sorted(files):
        d, _, stem = rel.rpartition("/")
        if "/pimp_chrome/" not in "/" + d + "/" or d.endswith("/tpp"):
            continue
        if "_holstered_" in stem:
            long_ = stem.endswith("_longsleeved")
            tpp = d + "/tpp/" + (stem[:-len("_longsleeved")] if long_ else stem) + "_tpp" + ("_longsleeved" if long_ else "")
            if tpp in files:
                pairs.append((rel, tpp))
            else:
                orphans.append(rel)
        elif stem.startswith("a0_000_fpp_") and stem.endswith("_nails"):
            tpp = d + "/" + stem.replace("a0_000_fpp_", "a0_000_base_", 1)
            if tpp in files:
                pairs.append((rel, tpp))
            else:
                orphans.append(rel)
    return pairs, orphans


def generate():
    # evidence[(gender, name, hash)] = {class: set(source)}
    evidence = defaultdict(lambda: defaultdict(set))
    paths = {}
    stats = Counter()
    notes = []

    # full[(g, name, hash)] = True when the part is in EVERY TPP appearance of that gender of at least one TPP file
    # (skin-tone variants of the same body). Parts present only in some TPP appearances (underwear-on variants,
    # doll cyberware, scars) are "tppVariant": they belong to the TPP body but are never force-enabled.
    full = {}

    def by_gender(rel, side=None):
        res = defaultdict(set)
        napps, seen = Counter(), defaultdict(Counter)
        unknown = 0
        for app, comps in load_app(rel):
            g = gender_of_app(app, comps)
            if g is None:
                unknown += 1
                continue
            napps[g] += 1
            for c in set(comps):
                res[g].add(c)
                seen[g][c] += 1
        if side == "T":
            for g in seen:
                for (n, p, h), k in seen[g].items():
                    if k == napps[g]:
                        full[(g, n, h)] = True
        if unknown:
            notes.append("%s: %d appearance(s) without a gender (skipped)" % (rel, unknown))
        return res

    # Union per gender over ALL pairs: F = parts of FPP appearances, T = parts of TPP appearances, U = parts of
    # unpaired files (context unknown). A part that is FPP-only in one pair but present on the TPP side of another
    # pair is used in TPP, so it is "shared" (e.g. l0_000_pwa_base__cs_flat). Classes:
    #   F and T -> shared, F only -> fppOnly (hidden in TPP), T only -> tppOnly (shown in TPP), U only -> shared;
    #   F only but also used by an unpaired file (e.g. the FPP forearm in unholstered cyberware) -> ambiguous
    #   (hiding it could remove a visible arm in a state we cannot tell apart offline).
    sides = {"F": defaultdict(set), "T": defaultdict(set), "U": defaultdict(set)}

    def put(side, g, comp, src):
        n, p, h = comp
        sides[side][g].add((n, h))
        evidence[(g, n, h)][side].add(src)
        if p:
            paths[h] = p

    used = set()
    pimp_pairs, pimp_orphans = pimp_chrome_pairs()
    for o in pimp_orphans:
        notes.append("pimp_chrome set without a TPP twin (stays unpaired): " + o)
    stats["pimpPairs"] = len(pimp_pairs)
    for fpp, tpp in PAIRS + pimp_pairs:
        used.add(fpp)
        stats["pairs"] += 1
        for g, comps in by_gender(fpp).items():
            for c in comps:
                put("F", g, c, fpp)
        if tpp:
            used.add(tpp)
            for g, comps in by_gender(tpp, "T").items():
                for c in comps:
                    put("T", g, c, tpp)
    for rel in INTRA_FILE_PAIRS:
        used.add(rel)
        apps = load_app(rel)
        names = {a for a, _ in apps}
        for app, comps in apps:
            g = gender_of_app(app, comps)
            if g is None:
                notes.append("%s: appearance %s without a gender (skipped)" % (rel, app))
                continue
            if app.endswith("_fpp") and app[:-4] in names:
                stats["intraPairs"] += 1
                side = "F"
            elif app + "_fpp" in names:
                side = "T"
            else:
                side = "U"
            for c in comps:
                put(side, g, c, rel)
                if side == "T":
                    full[(g, c[0], c[2])] = True  # the TPP twin of one FPP appearance: the whole twin is needed
    for rel in all_app_files():
        if rel in used:
            continue
        stats["unpairedFiles"] += 1
        for app, comps in load_app(rel):
            g = gender_of_app(app, comps)
            for c in comps:
                for gg in ([g] if g else ["female", "male"]):
                    put("U", gg, c, rel)

    parts = {}
    for (g, n, h), ev in evidence.items():
        f, t, u = "F" in ev, "T" in ev, "U" in ev
        # 1.3: FPP side from a non-chrome set, TPP side only from chrome sets -> the chrome TPP evidence is ignored
        # (the base TPP set of this part uses another mesh; the runtime cannot tell the sets apart)
        if f and t and any(CHROME not in s for s in ev["F"]) and all(CHROME in s for s in ev["T"]):
            t = False
            stats["chromeTppIgnored"] += 1
            notes.append("pimp_chrome-only TPP evidence ignored: %s %s" % (g, n))
        if f and t:
            c = "shared"
        elif f:
            c = "ambiguous" if u else "fppOnly"
        elif t:
            c = "tppOnly" if full.get((g, n, h)) else "tppVariant"
        else:
            c = "shared"
        if c == "ambiguous":
            stats["ambiguousKeys"] += 1
        srcs = sorted(set().union(*ev.values()))
        parts[(g, n, h)] = (c, srcs)

    # game-owned components: player entities and TPP heads (override every other class)
    for g, files in ENTITIES.items():
        for fn in files:
            f = (ENT_EP1_DIR if fn.endswith("_ep1.ent.json") else ENT_DIR) / fn
            if not f.is_file():
                notes.append("missing entity json " + fn)
                continue
            d = json.loads(f.read_text(encoding="utf-8"))
            for c in d["Data"]["RootChunk"].get("components", []):
                c = c.get("Data", c) if isinstance(c.get("Data"), dict) else c
                if c.get("$type") not in MESH_CLASSES:
                    continue
                path, h = resource(c)
                parts[(g, val(c.get("name")), h or "0")] = (entity_class(c), [fn[:-5]])
                if path:
                    paths[h] = path
    for g, rel in HEADS.items():
        f = HEAD_DIR / rel
        d = json.loads(f.read_text(encoding="utf-8"))
        for c in d["Data"]["RootChunk"].get("components", []):
            c = c.get("Data", c) if isinstance(c.get("Data"), dict) else c
            if c.get("$type") not in MESH_CLASSES:
                continue
            path, h = resource(c)
            parts[(g, val(c.get("name")), h or "0")] = ("head", [rel.split("/")[-1][:-5]])
            if path:
                paths[h] = path
    return parts, paths, stats, notes


def lua_str(s):
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'


def render(parts, paths):
    lines = [
        "-- GENERATED by gen_bodyparts.py %s (offline body part table generator) - do not edit by hand." % GENERATOR_VERSION,
        "-- Vanilla Cyberpunk 2077 %s player body parts (names, depot paths, FNV-1a 64 path hashes; no assets)." % GAME_VERSION,
        "-- class: fppOnly = hide in TPP, tppOnly = show in TPP, shared / tppVariant / ambiguous / entity / head = never touched;",
        "-- player entity meshes: shadowProxy (FPP shadow-only proxy) / renderPlaneFpp (weapon-plane FPP mesh) = hide in TPP.",
        "-- part = { gender, class, component name, path hash (decimal string, '0' = no resource), depot path or false, sources }",
        "return {",
        "  version = 1,",
        "  generator = %s," % lua_str("gen_bodyparts.py " + GENERATOR_VERSION),
        "  game = %s," % lua_str(GAME_VERSION),
        "  hash = \"fnv1a64(lower(path), '\\\\\\\\')\",",
        "  parts = {",
    ]
    order = {"female": 0, "male": 1}
    for (g, n, h) in sorted(parts, key=lambda k: (order[k[0]], parts[k][0], k[1] or "", k[2])):
        c, srcs = parts[(g, n, h)]
        p = paths.get(h)
        lines.append("    { %s, %s, %s, %s, %s, %s }," % (lua_str(g), lua_str(c), lua_str(n), lua_str(h),
                                                         lua_str(p) if p else "false", lua_str(";".join(srcs))))
    lines += ["  },", "}", ""]
    return "\n".join(lines)


# Facts that must hold in the generated table (checked with --check).
FACTS = [
    ("female", "left_arm", "base\\characters\\common\\player_base_bodies\\player_female_average\\fpp\\a0_000_pwa_fpp__full_l.mesh", "fppOnly"),
    ("female", "right_arm", "base\\characters\\common\\player_base_bodies\\player_female_average\\fpp\\a0_000_pwa_fpp__full_r.mesh", "fppOnly"),
    ("female", "t0_000_pwa_fpp__torso", "base\\characters\\common\\player_base_bodies\\player_female_average\\fpp\\t0_000_pwa_fpp__torso.mesh", "fppOnly"),
    ("female", "t0_000_pwa_base__full", "base\\characters\\common\\player_base_bodies\\player_female_average\\t0_000_pwa_base__full.morphtarget", "tppOnly"),
    ("female", "a0_001_pwa_base_hq__full", "base\\characters\\common\\player_base_bodies\\player_female_average\\arms_hq\\a0_000_pwa_base_hq__l.mesh", "tppOnly"),
    ("female", "a0_000_pwa_fpp__nails_l", "base\\characters\\common\\player_base_bodies\\player_female_average\\fpp\\a0_000_pwa_fpp__nails_l.morphtarget", "fppOnly"),
    ("female", "n0_000_pwa_fpp__neck0160", "base\\characters\\common\\player_base_bodies\\player_female_average\\fpp\\n0_000_pwa_fpp__neck.mesh", "fppOnly"),
    ("female", "l0_000_pwa_base__cs_flat", "base\\characters\\common\\player_base_bodies\\player_female_average\\l0_000_pwa_base__cs_flat.mesh", "shared"),
    ("female", "a0_002_wa__monowire_whip_l_cableless", "base\\characters\\cyberware\\player\\a0_002__monowire_whip\\a0_002_wa__monowire_whip_l_cableless.mesh", "fppOnly"),
    ("female", "a0_002_wa__monowire_whip_l_cableless", "base\\characters\\cyberware\\player\\a0_002__monowire_whip\\a0_002_wa__monowire_whip_l_tpp.mesh", "tppOnly"),
    ("male", "t0_000_pma_base__full", "base\\characters\\common\\player_base_bodies\\player_man_average\\t0_000_pma_base__full.mesh", "shared"),
    ("male", "a0_000_ma_base__full_ag_hq1491", "base\\characters\\common\\player_base_bodies\\player_man_average\\arms_hq\\a0_000_pma_base_hq__l.mesh", "shared"),
    ("male", "n0_000_pma_fpp__neck2814", "base\\characters\\common\\player_base_bodies\\player_man_average\\fpp\\n0_000_pma_fpp__neck.mesh", "fppOnly"),
    ("male", "a0_000_pma_base__nails_l", "base\\characters\\common\\player_base_bodies\\player_man_average\\arms_hq\\nails\\a0_000_pma_base__nails_l.morphtarget", "shared"),
    ("female", "body", "base\\characters\\common\\player_base_bodies\\player_female_average\\fpp\\t0_000_pwa_fpp__torso.mesh", "entity"),
    ("male", "torso", "base\\characters\\common\\base_bodies\\man_average\\_old_to_delete\\t0_001_ma_base_hq__torso.mesh", "entity"),
    # 1.1: FPP entity meshes duplicated by the TPP body (standalone test 2: two shadows, hand cyberware through the body)
    ("female", "t0_000_pwa_base__full_shadow", "base\\characters\\common\\player_base_bodies\\player_female_average\\shadow_meshes\\t0_000_pwa_base__full_shadow.mesh", "shadowProxy"),
    ("female", "head_shadowmesh", "base\\characters\\common\\base_bodies\\woman_average\\_old_to_delete\\h0_000_wa__basehead_shadow.mesh", "shadowProxy"),
    ("male", "t0_000_pma_base__full_shadow", "base\\characters\\common\\player_base_bodies\\player_man_average\\shadow_meshes\\t0_000_pma_base__full_shadow.mesh", "shadowProxy"),
    ("female", "a0_004__weapon_grip_device", "base\\characters\\cyberware\\player\\a0_004__weapon_grip\\a0_004__weapon_grip_device.mesh", "renderPlaneFpp"),
    ("female", "a0_004__weapon_grip_decal_01", "base\\characters\\cyberware\\player\\a0_004__weapon_grip\\a0_004__weapon_grip_decal_01.mesh", "renderPlaneFpp"),
    ("female", "a0_000_nomad_base_fists", "base\\characters\\common\\player_base_bodies\\player_female_average\\arms_hq\\a0_000_pwa_base__fists.mesh", "renderPlaneFpp"),
    ("male", "a0_004__weapon_grip_device", "base\\characters\\cyberware\\player\\a0_004__weapon_grip\\a0_004__weapon_grip_device_male.mesh", "renderPlaneFpp"),
    # 1.3: the FPP holstered upperarm of the base mantisblade stays fppOnly although chrome TPP sets reuse it (base TPP uses
    # the photomode mesh); the male launcher / mantis upperarms are used by the base TPP sets themselves (shared)
    ("female", "a0_003_wa__mantisblade_holstered_upperarm_left", "base\\characters\\cyberware\\player\\a0_003__mantisblade\\entities\\meshes\\a0_003_wa__mantisblade_holstered_upperarm_left.mesh", "fppOnly"),
    ("female", "a0_003_wa__mantisblade_holstered_upperarm_right", "base\\characters\\cyberware\\player\\a0_003__mantisblade\\entities\\meshes\\a0_003_wa__mantisblade_holstered_upperarm_right.mesh", "fppOnly"),
    ("female", "a0_003_wa__mantisblade_photomode_upperarm_left", "base\\characters\\cyberware\\player\\a0_003__mantisblade\\entities\\meshes\\a0_003_wa__mantisblade_photomode_upperarm_left.mesh", "tppOnly"),
    # the TPP (photo mode) grip decal with the same component name stays tppOnly (resolved by hash)
    ("female", "a0_004__weapon_grip_decal_01", "base\\characters\\cyberware\\player\\a0_004__weapon_grip\\a0_004__weapon_grip_decal_01_wa_photomode.mesh", "tppOnly"),
]


def check(parts, paths, text, out):
    ok = True

    def fail(msg):
        nonlocal ok
        ok = False
        print("FAIL:", msg)

    for g, n, p, want in FACTS:
        got = parts.get((g, n, str(fnv1a64(p))), (None,))[0]
        print("%s fact %-6s %-40s %-9s (want %s)" % ("PASS" if got == want else "FAIL", g, n, got, want))
        if got != want:
            ok = False
    # every hash with a path must be the FNV of that path
    for h, p in paths.items():
        if str(fnv1a64(p)) != h:
            fail("hash mismatch " + p)
    # both genders present, each with fppOnly and tppOnly / shared entries
    cnt = Counter((g, c) for (g, _, _), (c, _) in parts.items())
    for g in ("female", "male"):
        for c in ("fppOnly", "shared", "entity", "head", "shadowProxy", "renderPlaneFpp"):
            if cnt[(g, c)] == 0:
                fail("no %s %s entries" % (g, c))
    if cnt[("female", "tppOnly")] == 0:
        fail("no female tppOnly entries")
    # no fppOnly entry may point into a TPP / photo-mode / hq resource, no tppOnly entry into an \fpp\ folder
    for (g, n, h), (c, _) in parts.items():
        p = (paths.get(h) or "").lower()
        if c == "fppOnly" and ("_tpp" in p or "photo_mode" in p or "photomode" in p):
            fail("fppOnly points to a TPP resource: %s %s %s" % (g, n, p))
        if c == "tppOnly" and "\\fpp\\" in p:
            fail("tppOnly points to an FPP resource: %s %s %s" % (g, n, p))
    if out.is_file():
        disk = out.read_text(encoding="utf-8")
        if disk != text:
            fail("%s differs from the regenerated table (run gen_bodyparts.py)" % out)
        else:
            print("PASS table on disk == regenerated (%d bytes)" % len(text))
    else:
        fail("missing " + str(out))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--src250", action="store_true", help="game 2.50 (default; kept for old command lines)")
    ap.add_argument("--src231", action="store_true", help="legacy: read the 2.31 extraction (comparison only)")
    a = ap.parse_args()
    if a.src231 and a.src250:
        ap.error("--src231 and --src250 exclude each other")
    if a.src231:
        use_src231()
    out = Path(a.out)
    parts, paths, stats, notes = generate()
    text = render(parts, paths)
    cnt = Counter((g, c) for (g, _, _), (c, _) in parts.items())
    print("gen_bodyparts %s: %d .app files, %d pairs (%d pimp_chrome), %d intra-file pairs, %d unpaired files, %d parts, %d ambiguous keys"
          % (GENERATOR_VERSION, len(all_app_files()), stats["pairs"], stats["pimpPairs"], stats["intraPairs"], stats["unpairedFiles"],
             len(parts), stats["ambiguousKeys"]))
    for g in ("female", "male"):
        print("  %-6s " % g + " ".join("%s=%d" % (c, cnt[(g, c)]) for c in
                                         ("fppOnly", "tppOnly", "tppVariant", "shared", "ambiguous", "entity", "head",
                                          "shadowProxy", "renderPlaneFpp")))
    for n in notes:
        print("  note:", n)
    if a.check:
        ok = check(parts, paths, text, out)
        print("CHECK", "PASS" if ok else "FAIL")
        return 0 if ok else 1
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8", newline="\n")
    print("written", out, len(text), "bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
