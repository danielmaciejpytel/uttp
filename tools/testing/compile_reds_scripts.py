"""Offline redscript 0.5.32 compilation via its published C API, without dialogs.

All compiler state is confined to --work; source and game baseline stay read-only.
API: https://github.com/jac3km4/redscript/blob/v0.5.32/scc/lib/src/api.rs
"""
import argparse
import ctypes as C
from pathlib import Path
import shutil

p = argparse.ArgumentParser()
p.add_argument('--work', type=Path, required=True)
p.add_argument('--source', type=Path, action='append', required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[3]
work = a.work.resolve()
if not work.is_relative_to(root / 'Build'):
    raise SystemExit('Compiler work must remain under project Build')
cache = work / 'cache'
cache.mkdir(parents=True, exist_ok=True)
# Each invocation starts from the preserved vanilla 2.50 input, not prior compiled output.
for name in ('final.redscripts', 'final.redscripts.bk'):
    shutil.copy2(root / 'Build/work/compat-2.5/inputs/final-2.50.redscripts', cache / name)
for name in ('final.redscripts.ts', 'redscript.ts'):
    (cache / name).unlink(missing_ok=True)
lib = C.CDLL(str(root / 'Tools/redscript/compiler-0.5.32/scc_lib.dll'))
def fn(name, result, *args):
    f = getattr(lib, name)
    f.restype, f.argtypes = result, list(args)
    return f
settings = fn('scc_settings_new', C.c_void_p, C.c_char_p)(str(work).encode())
fn('scc_settings_disable_error_popup', None, C.c_void_p)(settings)
add = fn('scc_settings_add_script_path', None, C.c_void_p, C.c_char_p)
for source in dict.fromkeys(s.resolve() for s in a.source):
    add(settings, str(source).encode())
result = fn('scc_compile', C.c_void_p, C.c_void_p)(settings)
ok = bool(fn('scc_get_success', C.c_void_p, C.c_void_p)(result))
if not ok:
    error = C.create_string_buffer(65536)
    fn('scc_copy_error', C.c_size_t, C.c_void_p, C.c_void_p, C.c_size_t)(result, error, len(error))
    print(error.value.decode(errors='replace'))
fn('scc_free_result', None, C.c_void_p)(result)
print('OFFLINE_COMPILE=' + ('PASS' if ok else 'FAIL'))
raise SystemExit(0 if ok else 1)
