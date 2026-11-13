"""Execute offline Lua tests using the official Lua 5.1.5 DLL (Tools/lua-5.1.5/lua51.dll, built from source)."""
import ctypes
from pathlib import Path

stage = next(p for p in Path(__file__).resolve().parents if (p / "plugin" / "UTPP" / "cet").is_dir())
root = next(p for p in stage.parents if (p / "Source" / "plugin").is_dir())
library = root / "Tools" / "lua-5.1.5" / "lua51.dll"
utpp_bodyturn_test_file = Path(__file__).with_name("test_utpp_bodyturn.lua")
utpp_bodyturn_mod = stage / "plugin" / "UTPP" / "cet" / "utpp_bodyturn" / "init.lua"
utpp_vehiclecam_test_file = Path(__file__).with_name("test_utpp_vehiclecam.lua")
utpp_vehiclecam_mod = stage / "plugin" / "UTPP" / "cet" / "utpp_vehiclecam" / "init.lua"
utpp_cet_load_test_file = Path(__file__).with_name("test_utpp_cet_load.lua")
utpp_keeptpp_test_file = Path(__file__).with_name("test_utpp_keeptpp.lua")
utpp_core_mod = stage / "plugin" / "UTPP" / "cet" / "utpp_core" / "init.lua"
utpp_core_test_file = Path(__file__).with_name("test_utpp_core.lua")
utpp_motion_mod = stage / "plugin" / "UTPP" / "cet" / "utpp_motion" / "init.lua"
utpp_motion_test_file = Path(__file__).with_name("test_utpp_motion.lua")
utpp_perspective_mod = stage / "plugin" / "UTPP" / "cet" / "utpp_perspective" / "init.lua"
utpp_body_mod = stage / "plugin" / "UTPP" / "cet" / "utpp_body" / "init.lua"
utpp_loothead_mod = stage / "plugin" / "UTPP" / "cet" / "utpp_loothead" / "init.lua"
utpp_body_data = stage / "plugin" / "UTPP" / "cet" / "utpp_body" / "body_parts.lua"
utpp_reload_probe_mod = stage / "plugin" / "UTPP" / "cet" / "utpp_reload_probe" / "init.lua"
utpp_probe_mod = stage / "plugin" / "UTPP" / "cet" / "utpp_probe" / "init.lua"
utpp_camera_dir = stage / "plugin" / "UTPP" / "cet" / "utpp_camera"

lua = ctypes.CDLL(str(library))
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
lua.lua_close.argtypes = [ctypes.c_void_p]

state = lua.luaL_newstate()
assert state, "Lua state unavailable"
try:
    lua.luaL_openlibs(state)
    lua.lua_pushstring(state, str(utpp_bodyturn_mod).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_BODYTURN_PATH")
    lua.lua_pushstring(state, str(utpp_vehiclecam_mod).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_VEHICLECAM_PATH")
    lua.lua_pushstring(state, str(utpp_core_mod).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_CORE_PATH")
    lua.lua_pushstring(state, str(utpp_motion_mod).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_MOTION_PATH")
    lua.lua_pushstring(state, str(utpp_perspective_mod).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_PERSPECTIVE_PATH")
    lua.lua_pushstring(state, str(utpp_body_mod).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_BODY_PATH")
    lua.lua_pushstring(state, str(utpp_body_data).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_BODY_DATA_PATH")
    lua.lua_pushstring(state, str(utpp_loothead_mod).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_LOOTHEAD_PATH")
    lua.lua_pushstring(state, str(utpp_reload_probe_mod).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_RELOAD_PROBE_PATH")
    lua.lua_pushstring(state, str(utpp_probe_mod).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_PROBE_PATH")
    lua.lua_pushstring(state, str(utpp_camera_dir / "init.lua").encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_CAMERA_PATH")
    lua.lua_pushstring(state, str(utpp_camera_dir).encode("utf-8"))
    lua.lua_setfield(state, -10002, b"UTPP_CAMERA_DIR")
    for path, execute in ((utpp_bodyturn_mod, False),
                          (utpp_bodyturn_test_file, True),
                          (utpp_vehiclecam_mod, False),
                          (utpp_vehiclecam_test_file, True),
                          (utpp_cet_load_test_file, True),
                          (utpp_keeptpp_test_file, True),
                          (utpp_core_mod, False),
                          (utpp_core_test_file, True),
                          (Path(__file__).with_name("test_utpp_ragdoll_getup.lua"), True),
                          (utpp_motion_mod, False),
                          (utpp_motion_test_file, True),
                          (utpp_perspective_mod, False),
                          (Path(__file__).with_name("test_utpp_perspective.lua"), True),
                          (Path(__file__).with_name("test_utpp_perspective_cet_load.lua"), True),
                          (utpp_body_mod, False),
                          (Path(__file__).with_name("test_utpp_body.lua"), True),
                          (Path(__file__).with_name("test_utpp_body_cet_load.lua"), True),
                          (utpp_loothead_mod, False),
                          (Path(__file__).with_name("test_utpp_loothead.lua"), True),
                          (Path(__file__).with_name("test_utpp_user_defaults.lua"), True),
                          (Path(__file__).with_name("test_utpp_settings_projection.lua"), True),
                          (Path(__file__).with_name("test_utpp_modsettings.lua"), True),
                          (Path(__file__).with_name("test_utpp_diagnostic_logs.lua"), True),
                          (Path(__file__).with_name("test_utpp_animation_pack.lua"), True),
                          (utpp_reload_probe_mod, False),
                          (Path(__file__).with_name("test_utpp_reload_probe.lua"), True),
                          (Path(__file__).with_name("test_utpp_reload_probe_positions.lua"), True),
                          (utpp_probe_mod, False),
                          (Path(__file__).with_name("test_utpp_probe_dual.lua"), True)):
        status = lua.luaL_loadfile(state, str(path).encode("utf-8"))
        if status:
            raise RuntimeError(f"{path}: {lua.lua_tolstring(state, -1, None).decode('utf-8', 'replace')}")
        if execute:
            status = lua.lua_pcall(state, 0, 0, 0)
            if status:
                raise RuntimeError(f"{path}: {lua.lua_tolstring(state, -1, None).decode('utf-8', 'replace')}")
        print(f"{'PASS' if execute else 'COMPILES'}: {path.name}")
finally:
    lua.lua_close(state)



# Mod Settings page (plugin/UTPP/scripts/utpp_modsettings.reds) must equal the generator output for the current registry
import importlib.util
spec = importlib.util.spec_from_file_location("gen_modsettings", stage / "tools" / "settings" / "gen_modsettings.py")
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)
text, count, _ = gen.render(gen.dump_registry())
if ('return n"%s";' % gen.MOD_NAME) not in gen.BRIDGE.read_text(encoding="ascii"):
    raise RuntimeError(f"{gen.BRIDGE.name}: UTPPModSettingsModName() differs from the generator's mod name")
if gen.OUT.read_bytes() != text.encode("ascii"):
    raise RuntimeError(f"{gen.OUT.name} differs from the settings registry: run tools/settings/gen_modsettings.py")
print(f"PASS: {gen.OUT.name} matches the settings registry ({count} options)")


# Suites that assign engine globals (registerForEvent, Game, ObserveAfter, ...) without restoring them: every suite runs in its
# own fresh Lua state so that it cannot disturb the shared-state suites above.
cet = stage / "plugin" / "UTPP" / "cet"
standalone_globals = {
    "UTPP_RELOAD_GATE_DIR": cet / "utpp_reload_gate",
    "UTPP_DEFORM_PROBE_PATH": cet / "utpp_deform_probe" / "init.lua",
    "UTPP_MOUNT_PROBE_PATH": cet / "utpp_mount_probe" / "init.lua",
    "UTPP_CAMERA_PATH": utpp_camera_dir / "init.lua",
    "UTPP_CAMERA_DIR": utpp_camera_dir,
    "UTPP_CORE_PATH": utpp_core_mod,
}
for name in ("test_utpp_reload_gate.lua", "test_utpp_reload_gate_adapter.lua", "test_utpp_deform_probe.lua",
             "test_utpp_mount_probe.lua", "test_utpp_camera_pose_write.lua", "test_utpp_headlook.lua"):
    own = lua.luaL_newstate()
    assert own, "Lua state unavailable"
    try:
        lua.luaL_openlibs(own)
        for key, value in standalone_globals.items():
            lua.lua_pushstring(own, str(value).replace("\\", "/").encode("utf-8"))
            lua.lua_setfield(own, -10002, key.encode("utf-8"))
        path = Path(__file__).with_name(name)
        status = lua.luaL_loadfile(own, str(path).encode("utf-8"))
        if not status:
            status = lua.lua_pcall(own, 0, 0, 0)
        if status:
            raise RuntimeError(f"{path}: {lua.lua_tolstring(own, -1, None).decode('utf-8', 'replace')}")
        print(f"PASS: {name}")
    finally:
        lua.lua_close(own)
