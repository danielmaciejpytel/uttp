"""Runs the utpp_camera offline Lua tests with the bundled Lua 5.1 DLL (same mechanism as run_lua_tests.py).

  python Source\\tools\\lua-tests\\test_utpp_camera_run.py

Globals for the tests: UTPP_CAMERA_MATH_PATH, UTPP_CAMERA_PATH, UTPP_CAMERA_DIR, UTPP_PROBE_PATH, UTPP_CORE_PATH,
UTPP_VEHICLECAM_PATH (test_utpp_camera_vehicle.lua: standalone vehicle path with utpp_vehiclecam), UTPP_PERSPECTIVE_PATH
(test_utpp_camera_load.lua M: unarmed zoom handoff through the real utpp_perspective).
UTPP_LUA_FIXTURE_DIR (tools/lua-tests/fixtures: rec1215_psm_detail.tsv of test_utpp_camera_stance.lua).
"""
import ctypes
import sys
from pathlib import Path

stage = next(p for p in Path(__file__).resolve().parents if (p / "plugin" / "UTPP" / "cet").is_dir())
root = next(p for p in stage.parents if (p / "Source" / "plugin").is_dir())
library = root / "Tools" / "lua-5.1.5" / "lua51.dll"
cet = stage / "plugin" / "UTPP" / "cet"
paths = {
    "UTPP_CAMERA_MATH_PATH": cet / "utpp_camera" / "camera_math.lua",
    "UTPP_CAMERA_PATH": cet / "utpp_camera" / "init.lua",
    "UTPP_CAMERA_DIR": cet / "utpp_camera",
    "UTPP_PROBE_PATH": cet / "utpp_probe" / "init.lua",
    "UTPP_CORE_PATH": cet / "utpp_core" / "init.lua",
    "UTPP_VEHICLECAM_PATH": cet / "utpp_vehiclecam" / "init.lua",
    "UTPP_PERSPECTIVE_PATH": cet / "utpp_perspective" / "init.lua",
    "UTPP_LUA_FIXTURE_DIR": Path(__file__).resolve().parent / "fixtures",  # recorded-session data (test_utpp_camera_stance.lua)
}
here = Path(__file__).resolve().parent
files = [(paths["UTPP_CAMERA_MATH_PATH"], False), (paths["UTPP_CAMERA_PATH"], False), (paths["UTPP_PROBE_PATH"], False),
         (here / "test_utpp_camera.lua", True), (here / "test_utpp_camera_stance.lua", True), (here / "test_utpp_camera_load.lua", True),
         (here / "test_utpp_profile_editor.lua", True),
         (paths["UTPP_VEHICLECAM_PATH"], False), (here / "test_utpp_camera_vehicle.lua", True),
         (here / "test_utpp_camera_target.lua", True), (here / "test_utpp_aim_point.lua", True),
         (here / "test_utpp_camera_ragdoll.lua", True),
         (here / "test_utpp_jumpout_replay.lua", True)]  # 2026-11-04 r13 jump-out replay  # 2026-11-03 ragdoll camera  # 2026-10-30 aim point feed
only = sys.argv[1:]
unknown = [n for n in only if n not in {p.name for p, execute in files if execute}]
if unknown:  # a misspelled suite name must not pass as "nothing failed"
    print(f"FAIL: unknown test suite(s): {', '.join(unknown)}")
    sys.exit(1)

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

failed = False
executed = 0
for path, execute in files:
    if only and execute and path.name not in only:
        continue
    executed += 1 if execute else 0
    state = lua.luaL_newstate()  # fresh state per file: no leaked globals between tests
    try:
        lua.luaL_openlibs(state)
        for k, v in paths.items():
            lua.lua_pushstring(state, str(v).encode("utf-8"))
            lua.lua_setfield(state, -10002, k.encode())  # LUA_GLOBALSINDEX in Lua 5.1
        status = lua.luaL_loadfile(state, str(path).encode("utf-8"))
        if status == 0 and execute:
            status = lua.lua_pcall(state, 0, 0, 0)
        if status:
            failed = True
            print(f"FAIL: {path.name}: {lua.lua_tolstring(state, -1, None).decode('utf-8', 'replace')}")
        else:
            print(f"{'PASS' if execute else 'COMPILES'}: {path.name}")
    finally:
        lua.lua_close(state)
if executed == 0:
    print("FAIL: no test suite was executed")
    failed = True
sys.exit(1 if failed else 0)
