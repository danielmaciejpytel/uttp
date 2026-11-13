"""Generate plugin/UTPP/scripts/utpp_modsettings.reds (the UTPP page of the Mod Settings screen) from the CET settings registry.

The registry of the shipped CET modules is booted offline (tools/settings/dump_registry.lua, bundled Lua 5.1 DLL) and its
standard menu projection becomes one redscript class, UTPPModSettings, with one field per user-visible option:
  * visible = menu level "player" or "advanced" (the same options as the standard Native Settings view); developer and
    diagnostic options and keys without a section stay CET-only; so do registry options with hidden = true (no class field,
    not in the snapshot, so utpp_core never syncs or pushes them; their CET value, default and settings file are unchanged);
  * categories = the menu groups (General, Camera, Combat, Interactions, Vehicles, Advanced, Debug) in the same order; every
    visible option is in its own group (player and advanced alike), sorted like the standard view (group, rank,
    registration order);
  * the camera framing editor views (camera.edit.<field>) are the fixed camera profile's raw keys camera.p<N>.<field>;
  * field name = key with every character other than a letter or a digit replaced by "_" (utpp_core uses the same rule:
    M.ModSettingsField); the field names are also the keys of Mod Settings' user.ini, so they must stay stable;
  * types: bool -> Bool, float -> Float, int -> Int32 (min / max / step), choice -> one enum per option (values 0..n-1 in
    the registry's choice order); any other type fails;
  * labels, descriptions and defaults are the registry's.

Usage: python -I tools/settings/gen_modsettings.py           write the file
       python -I tools/settings/gen_modsettings.py --check   exit 1 when the file differs from the registry
The file is written as UTF-8 without BOM, LF line endings (redscript rejects a BOM).
"""
import argparse
import ctypes
import json
import re
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
STAGE = next(p for p in TOOLS.parents if (p / "plugin" / "UTPP" / "cet").is_dir())
ROOT = next(p for p in STAGE.parents if (p / "Source" / "plugin").is_dir())
LUA_DLL = ROOT / "Tools" / "lua-5.1.5" / "lua51.dll"
CET = STAGE / "plugin" / "UTPP" / "cet"
OUT = STAGE / "plugin" / "UTPP" / "scripts" / "utpp_modsettings.reds"
MOD_NAME = "Ultimate Third Person"  # entry in the Mod Settings list
BRIDGE = STAGE / "plugin" / "UTPP" / "scripts" / "utpp_modsettings_bridge.reds"
CLASS_NAME = "UTPPModSettings"
VISIBLE_LEVELS = ("player", "advanced")
# redscript 0.5 accepts only constant field initializers and a negative number is not one (INVALID_CONSTANT). Options with
# a negative default get the in-range value nearest to 0 as the Mod Settings class default (reviewed list, anything else
# fails). The class default is only used by Mod Settings' "Restore defaults" and before the first sync (utpp_core pushes
# the CET value then), so only "Restore defaults" shows the placeholder.
PLACEHOLDER_DEFAULTS = {"camera.height"}


def dump_registry():
    lua = ctypes.CDLL(str(LUA_DLL))
    lua.luaL_newstate.restype = ctypes.c_void_p
    lua.luaL_openlibs.argtypes = [ctypes.c_void_p]
    lua.luaL_loadfile.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lua.luaL_loadfile.restype = ctypes.c_int
    lua.lua_pcall.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lua.lua_pcall.restype = ctypes.c_int
    lua.lua_tolstring.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
    lua.lua_tolstring.restype = ctypes.c_char_p
    lua.lua_pushstring.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lua.lua_setfield.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p]
    lua.lua_getfield.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p]
    lua.lua_close.argtypes = [ctypes.c_void_p]
    globals_ = {
        "UTPP_CORE_PATH": CET / "utpp_core" / "init.lua",
        "UTPP_BODYTURN_PATH": CET / "utpp_bodyturn" / "init.lua",
        "UTPP_BODY_PATH": CET / "utpp_body" / "init.lua",
        "UTPP_MOTION_PATH": CET / "utpp_motion" / "init.lua",
        "UTPP_CAMERA_PATH": CET / "utpp_camera" / "init.lua",
        "UTPP_CAMERA_DIR": CET / "utpp_camera",
        "UTPP_PERSPECTIVE_PATH": CET / "utpp_perspective" / "init.lua",
        "UTPP_LOOTHEAD_PATH": CET / "utpp_loothead" / "init.lua",
        "UTPP_VEHICLECAM_PATH": CET / "utpp_vehiclecam" / "init.lua",
    }
    state = lua.luaL_newstate()
    if not state:
        raise RuntimeError("Lua state unavailable")
    try:
        lua.luaL_openlibs(state)
        for name, path in globals_.items():
            lua.lua_pushstring(state, str(path).encode("utf-8"))
            lua.lua_setfield(state, -10002, name.encode())
        script = TOOLS / "dump_registry.lua"
        if lua.luaL_loadfile(state, str(script).encode("utf-8")) or lua.lua_pcall(state, 0, 0, 0):
            raise RuntimeError(lua.lua_tolstring(state, -1, None).decode("utf-8", "replace"))
        lua.lua_getfield(state, -10002, b"UTPP_REGISTRY_JSON")
        raw = lua.lua_tolstring(state, -1, None)
        if raw is None:
            raise RuntimeError("dump_registry.lua returned no JSON")
        return json.loads(raw.decode("utf-8"))
    finally:
        lua.lua_close(state)


def field_name(key):
    return re.sub(r"[^A-Za-z0-9]", "_", key)


def reds_text(s, what):
    s = str(s)
    if any(ord(c) > 126 or (ord(c) < 32) for c in s):
        raise SystemExit(f"non-ASCII or control character in {what}: {s!r}")
    return s.replace("\\", "\\\\").replace('"', '\\"')


def num(x, is_float):
    if is_float:
        r = repr(float(x))
        if "e" in r or "inf" in r or "nan" in r:
            raise SystemExit(f"unsupported float literal {x!r}")
        return r
    if float(x) != int(x):
        raise SystemExit(f"non-integer value {x!r} for an int option")
    return str(int(x))


def ident(label):
    words = re.findall(r"[A-Za-z0-9]+", label)
    name = "".join(w[:1].upper() + w[1:] for w in words)
    if not name or not name[0].isalpha():
        name = "V" + name
    return name


def collect(reg):
    groups = [g[0] for g in reg["groups"] if g[0] != "menu"]
    gindex = {g[0]: i + 1 for i, g in enumerate(reg["groups"])}
    glabel = {g[0]: g[1] for g in reg["groups"]}
    defs = {s["key"]: s for s in reg["settings"]}
    items = []
    for s in reg["settings"]:
        if s.get("hidden"):
            continue  # registry flag hidden = true: no field in the class (the option keeps working in CET)
        if s["level"] not in VISIBLE_LEVELS or s["source"] == "no section" or not s.get("section"):
            continue
        cat = s["group"]
        items.append(dict(s, cat=cat, sk=(gindex.get(s["group"], gindex["advanced"]), s["rank"], s["order"])))
    profile = reg.get("cameraProfile")
    for v in reg["views"]:
        if v["level"] not in VISIBLE_LEVELS:
            continue
        m = re.fullmatch(r"camera\.edit\.(\w+)", v["id"])
        if not m or profile is None:
            raise SystemExit(f"view {v['id']}: no known registry key (only camera.edit.<field> views are mapped)")
        key = "camera." + {"sens": "framingSens"}.get(m.group(1), m.group(1))
        d = defs.get(key)
        if d is None:
            raise SystemExit(f"view {v['id']}: key {key} is not registered")
        if d["type"] != v["type"]:
            raise SystemExit(f"view {v['id']}: type {v['type']} differs from {key} ({d['type']})")
        cat = v["group"]
        items.append(dict(d, label=v["label"], desc=v["desc"], min=v["min"], max=v["max"], step=v["step"], cat=cat,
                          view=v["id"], sk=(gindex.get(v["group"], gindex["advanced"]), v["rank"], 100000 + v["index"])))
    seen = {}
    for it in items:
        if defs[it["key"]].get("hidden"):
            raise SystemExit(f"{it['key']}: hidden option reached the Mod Settings class (view of a hidden key?)")
        f = field_name(it["key"])
        if f in seen:
            raise SystemExit(f"field name collision: {it['key']} and {seen[f]} -> {f}")
        seen[f] = it["key"]
    cats = [g for g in groups if any(it["cat"] == g for it in items)]
    ordered = []
    for g in cats:
        ordered.extend(sorted((it for it in items if it["cat"] == g), key=lambda it: it["sk"]))
    return cats, glabel, gindex, ordered


def render(reg):
    cats, glabel, gindex, items = collect(reg)
    out = []
    w = out.append
    w("// UTPP - options of the Mod Settings screen (mod \"%s\"). GENERATED by tools/settings/gen_modsettings.py from the CET" % MOD_NAME)
    w("// settings registry (utpp_core %s) - do not edit; change the CET registration and run the generator." % reg["coreVersion"])
    w("// The CET registry applies every option; utpp_core syncs these values both ways through utpp_modsettings_bridge.reds.")
    w("// Without the Mod Settings framework the class is unused and the options stay in the Native Settings tab.")
    w("// Defaults equal the registry's, except options with a negative default (redscript has no negative initializer): their")
    w("// class default is 0 (marked below), which Mod Settings' \"Restore defaults\" shows; the first sync pushes the CET value.")
    w("")
    enums = []
    for it in items:
        if it["type"] == "choice":
            ename = "UTPPMS_" + field_name(it["key"])
            members = []
            for i, (cid, clabel) in enumerate(it["choices"]):
                m = ident(clabel)
                if m in [x[0] for x in members]:
                    m = m + str(i)
                members.append((m, i, cid, clabel))
            it["enum"] = (ename, members)
            enums.append((ename, members, it["key"]))
    for ename, members, key in enums:
        w("// %s: %s" % (key, ", ".join("%d = %s" % (i, cid) for _, i, cid, _ in members)))
        w("public enum %s {" % ename)
        w(",\n".join("  %s = %d" % (m, i) for m, i, _, _ in members))
        w("}")
        w("")
    w("public class %s extends IScriptable {" % CLASS_NAME)
    order = 0
    for it in items:
        order += 1
        key, t = it["key"], it["type"]
        f = field_name(key)
        w("")
        w("  // %s%s" % (key, (" (view " + it["view"] + ")") if it.get("view") else ""))
        w('  @runtimeProperty("ModSettings.mod", "%s")' % MOD_NAME)
        w('  @runtimeProperty("ModSettings.category", "%s")' % reds_text(glabel[it["cat"]], key))
        w('  @runtimeProperty("ModSettings.category.order", "%d")' % gindex[it["cat"]])
        w('  @runtimeProperty("ModSettings.order", "%d")' % order)
        w('  @runtimeProperty("ModSettings.displayName", "%s")' % reds_text(it["label"], key))
        if it.get("desc"):
            w('  @runtimeProperty("ModSettings.description", "%s")' % reds_text(it["desc"], key))
        if t == "bool":
            if not isinstance(it["default"], bool):
                raise SystemExit(f"{key}: bool default {it['default']!r}")
            w("  public let %s: Bool = %s;" % (f, "true" if it["default"] else "false"))
        elif t in ("float", "int"):
            is_float = t == "float"
            if it.get("min") is None or it.get("max") is None:
                raise SystemExit(f"{key}: {t} without min / max")
            step = it.get("step") if it.get("step") is not None else (0.1 if is_float else 1)
            w('  @runtimeProperty("ModSettings.min", "%s")' % num(it["min"], is_float))
            w('  @runtimeProperty("ModSettings.max", "%s")' % num(it["max"], is_float))
            w('  @runtimeProperty("ModSettings.step", "%s")' % num(step, is_float))
            default = it["default"]
            if default < 0:
                if key not in PLACEHOLDER_DEFAULTS:
                    raise SystemExit(f"{key}: negative default {default!r} (redscript cannot declare it; review and add the key "
                                     "to PLACEHOLDER_DEFAULTS)")
                if not (it["min"] <= 0 <= it["max"]):
                    raise SystemExit(f"{key}: placeholder default 0 is outside {it['min']}..{it['max']}")
                w("  // class default %s instead of the registry default %s (redscript: no negative initializer)"
                  % (num(0, is_float), num(default, is_float)))
                default = 0
            w("  public let %s: %s = %s;" % (f, "Float" if is_float else "Int32", num(default, is_float)))
        elif t == "choice":
            ename, members = it["enum"]
            # display names in enum order, in the list form used by the framework author's own mods (one property; the enum
            # member names stay readable as a last resort)
            w('  @runtimeProperty("ModSettings.displayValues", "%s")'
              % reds_text(", ".join('"%s"' % clabel for _, _, _, clabel in members), key))
            dm = [m for m, _, cid, _ in members if cid == it["default"]]
            if not dm:
                raise SystemExit(f"{key}: default {it['default']!r} is not a choice")
            w("  public let %s: %s = %s.%s;" % (f, ename, ename, dm[0]))
        else:
            raise SystemExit(f"{key}: unsupported type {t}")
    # values of a registered listener instance as "field=value\n" lines (read by utpp_core through
    # utpp_modsettings_bridge.reds; formats: UTPPMSBool / UTPPMSFloat / IntToString / enum value)
    w("")
    w("  public final func UTPPSnapshot() -> String {")
    w('    let s: String = "";')
    for it in items:
        f, t = field_name(it["key"]), it["type"]
        if t == "bool":
            expr = "UTPPMSBool(this.%s)" % f
        elif t == "float":
            expr = "UTPPMSFloat(this.%s)" % f
        elif t == "int":
            expr = "IntToString(this.%s)" % f
        elif t == "choice":
            expr = "IntToString(EnumInt(this.%s))" % f
        else:
            raise SystemExit(f"{it['key']}: unsupported type {t}")
        w('    s += "%s=" + %s + "\\n";' % (f, expr))
    w("    return s;")
    w("  }")
    w("}")
    w("")
    return "\n".join(out), len(items), cats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="compare instead of writing")
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()
    text, n, cats = render(dump_registry())
    data = text.encode("ascii")
    if ('return n"%s";' % MOD_NAME) not in BRIDGE.read_text(encoding="ascii"):
        print(f"MODSETTINGS_GEN=DRIFT {BRIDGE.name}: UTPPModSettingsModName() is not n\"{MOD_NAME}\"")
        return 1
    if a.check:
        cur = a.out.read_bytes() if a.out.exists() else b""
        if cur != data:
            print(f"MODSETTINGS_GEN=DRIFT {a.out} differs from the registry (run tools/settings/gen_modsettings.py)")
            return 1
        print(f"MODSETTINGS_GEN=OK {n} options in {len(cats)} categories")
        return 0
    a.out.write_bytes(data)
    print(f"MODSETTINGS_GEN=WROTE {a.out} ({n} options in {len(cats)} categories: {', '.join(cats)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
