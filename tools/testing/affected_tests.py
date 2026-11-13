"""Test impact analysis for Build\\UTPP: which ctest tests depend on the files changed in Source.

Exact compile dependencies (no hand-written map): build.ninja gives test executable -> objects -> source, `ninja -t deps` gives every
header an object included. ctest's JSON gives test -> executable, the CMake file that registered it and its argument files.
A test is affected when its executable compiles a changed file, when its registering .cmake changed, or when an argument path
(e.g. tests/data/*.txt) is a changed file. Changes to CMakeLists.txt / labels.cmake / a shared file nothing compiles are reported.

Usage (run from anywhere; Build\\UTPP must be configured and built at least once):
  python affected_tests.py                     # changes = working tree + index vs HEAD
  python affected_tests.py --base main~3       # changes since a revision (committed + uncommitted)
  python affected_tests.py --files src/MotionDriver.cpp tests/RagdollGetUpTest.cpp
  options: --no-mut (skip mutation tests)  --list-file out.txt (ctest --tests-from-file input)  --run (build + run them with ctest -j 16)
Exit code: 0 ok, 2 = a change needs the full suite (CMakeLists.txt / labels.cmake / unmatched build input), 3 = no tests affected.
"""
import argparse, json, os, re, subprocess, sys, glob

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", ".."))                 # Source (git root)
ROOT = os.path.dirname(SRC)
UTPP = os.path.join(SRC, "plugin", "UTPP")
BUILD = os.path.join(ROOT, "Build", "UTPP")
NINJA = os.path.join(ROOT, "Tools", "ninja", "ninja.exe")
CTEST = (glob.glob(os.path.join(ROOT, "Tools", "cmake", "*", "bin", "ctest.exe")) or ["ctest"])[0]
FULL_SUITE_FILES = {"plugin/utpp/cmakelists.txt", "plugin/utpp/tests/labels.cmake"}


def norm(p):
    """Absolute normalized lower-case path."""
    return os.path.normcase(os.path.normpath(p if os.path.isabs(p) else os.path.join(BUILD, p)))


def changed_files(base, files):
    if files:
        out = []
        for f in files:
            p = f if os.path.isabs(f) else (os.path.join(UTPP, f) if os.path.exists(os.path.join(UTPP, f)) else os.path.join(SRC, f))
            out.append(norm(p))
        return out
    cmds = [["git", "-C", SRC, "diff", "--name-only", base], ["git", "-C", SRC, "ls-files", "--others", "--exclude-standard"]]
    names = set()
    for c in cmds:
        names.update(x for x in subprocess.run(c, capture_output=True, text=True).stdout.splitlines() if x.strip())
    return [norm(os.path.join(SRC, n)) for n in sorted(names)]


def build_graph():
    exe_objs, obj_src = {}, {}
    rx = re.compile(r"^build (\S+?): (\S+)((?: \S+)*)")
    with open(os.path.join(BUILD, "build.ninja"), encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = rx.match(line.replace("$:", ":").replace("$ ", "\0"))
            if not m:
                continue
            out, rule, ins = m.group(1).replace("\0", " "), m.group(2), m.group(3).split(" | ")[0].split()
            ins = [i.replace("\0", " ") for i in ins]
            if rule.startswith("CXX_COMPILER") and ins:
                obj_src[norm(out)] = norm(ins[0])
            elif rule.startswith("CXX_EXECUTABLE_LINKER"):
                exe_objs[norm(out)] = [norm(i) for i in ins if i.endswith(".obj")]
    deps = {}
    txt = subprocess.run([NINJA, "-C", BUILD, "-t", "deps"], capture_output=True, text=True).stdout
    cur = None
    for line in txt.splitlines():
        if line and not line.startswith(" "):
            cur = norm(line.split(":")[0]); deps[cur] = []
        elif line.strip() and cur:
            deps[cur].append(norm(line.strip()))
    return exe_objs, obj_src, deps


def ctest_tests():
    j = json.loads(subprocess.run([CTEST, "-N", "--show-only=json-v1"], cwd=BUILD, capture_output=True, text=True).stdout)
    files = j.get("backtraceGraph", {}).get("files", [])
    nodes = j.get("backtraceGraph", {}).get("nodes", [])
    tests = []
    for t in j["tests"]:
        cmd = t.get("command") or []
        labels = next((p["value"] for p in t.get("properties", []) if p["name"] == "LABELS"), [])
        regs = set()
        bt = t.get("backtrace")
        while bt is not None and bt < len(nodes):          # every CMake file on the add_test backtrace (include chain)
            regs.add(norm(files[nodes[bt]["file"]])); bt = nodes[bt].get("parent")
        tests.append({"name": t["name"], "exe": norm(cmd[0]) if cmd else None,
                      "args": [norm(a) for a in cmd[1:] if re.search(r"[/\\]", a)], "labels": labels, "cmake": regs})
    return tests


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default="HEAD"); ap.add_argument("--files", nargs="*")
    ap.add_argument("--no-mut", action="store_true"); ap.add_argument("--list-file"); ap.add_argument("--run", action="store_true")
    a = ap.parse_args()
    if not os.path.exists(os.path.join(BUILD, "build.ninja")):
        sys.exit("Build\\UTPP is not configured: run Build\\build-main.cmd first")
    changed = changed_files(a.base, a.files)
    rel = lambda p: os.path.relpath(p, SRC).replace("\\", "/").lower()
    full = [c for c in changed if rel(c) in FULL_SUITE_FILES]
    exe_objs, obj_src, deps = build_graph()
    obj_inputs = {o: {s} | set(deps.get(o, [])) for o, s in obj_src.items()}
    tests = ctest_tests()
    chg = set(changed)
    hit, why, used = [], {}, set()
    for t in tests:
        reasons = set()
        for o in exe_objs.get(t["exe"], []):
            r = obj_inputs.get(o, set()) & chg
            reasons |= r
        reasons |= (t["cmake"] & chg) | (set(t["args"]) & chg)
        if reasons:
            used |= reasons
            if a.no_mut and "mutation" in t["labels"]:
                continue
            hit.append(t["name"]); why[t["name"]] = sorted(rel(r) for r in reasons)
    code_like = [c for c in changed if c.startswith(norm(UTPP)) and re.search(r"\.(cpp|hpp|h|inl|cmake|txt|json|feed)$", c)]
    unmatched = [c for c in code_like if c not in used and c not in full]
    print(f"changed files: {len(changed)} (plugin/UTPP code/test inputs: {len(code_like)})")
    for c in full:
        print(f"  FULL SUITE: {rel(c)} changed (test registration / labels) -> run Build\\build-main.cmd full")
    for c in unmatched:
        print(f"  no test compiles or reads {rel(c)}")
    print(f"affected tests: {len(hit)} of {len(tests)}" + (" (mutations skipped)" if a.no_mut else ""))
    for n in hit:
        print(f"  {n}   <- {', '.join(why[n][:3])}")
    if a.list_file or a.run:
        lf = a.list_file or os.path.join(BUILD, "affected-tests.txt")
        with open(lf, "w", encoding="utf-8") as f:
            f.write("\n".join(hit) + "\n")
    if a.run and hit:
        b = subprocess.run([os.path.join(os.path.dirname(CTEST), "cmake.exe"), "--build", BUILD, "-j", "16"])
        if b.returncode:
            sys.exit(b.returncode)
        r = subprocess.run([CTEST, "-j", "16", "--output-on-failure", "--tests-from-file", lf], cwd=BUILD)
        sys.exit(r.returncode)
    sys.exit(2 if full else (3 if not hit else 0))


if __name__ == "__main__":
    main()
