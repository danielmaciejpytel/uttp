"""Run a patched COPY of export_nue_native_recoil_curves.py (recoil-missing-20261109 patch check) with the import path of the original.

usage: python -I run_patched_exporter.py <patched exporter .py> <exporter arguments...>
The copy imports its sibling analysis modules (compare_nue_magazine_sources, ...) from Source\\tools\\analysis; recoil_common puts that
folder and the reconstruction folder on sys.path. _project_root() of a copy below Build\\ would stop at Build\\ (it also holds
Extracted/Research/Data folders), so ROOT is pinned to the project root; the patch does not touch _project_root(). The file itself is
not modified. Always pass --profiles-out / --profiles-report (the default output is Data\\recoil).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import recoil_common as rc  # noqa: E402  (sys.path of the original exporter)

if __name__ == '__main__':
    if '--profiles' in sys.argv and '--profiles-out' not in sys.argv:
        raise SystemExit('refusing to run without --profiles-out (the default output is Data\\recoil)')
    script = Path(sys.argv[1]).resolve()
    sys.argv = [str(script)] + sys.argv[2:]
    text = script.read_text(encoding='utf-8')
    anchor = '\nROOT = _project_root()\n'
    if text.count(anchor) != 1:
        raise SystemExit('ROOT anchor not found')
    text = text.replace(anchor, f'\nROOT = Path({str(rc.ROOT)!r})\n')
    exec(compile(text, str(script), 'exec'), {'__name__': '__main__', '__file__': str(script)})
