"""b5 testorder repair: loco suite order independence gate (Build\\UTPP\\rebuild.cmd, after the plain loco run).

Every loco section run alone (UTPP_LOCO_ONLY*, all in parallel) must print the same [INFO] / [PASS] / [FAIL] lines as the same
section ("==== SECTION <name>" block) inside the full suite, wall-clock fields (QPC search / update times, snapshot age, no-feed
Tick timing) masked. An alone run that fails a check or exits non-zero fails the gate as well.

usage: python -I loco_order_check.py --exe <utpp_mm_loco_test.exe> --full <full suite log> --out <dir> <db.utppmm ...>
exit 0 = every section identical, 1 = a section differs / is missing / an alone run failed, 2 = usage / setup error.
"""
import argparse
import os
import re
import subprocess
import sys

ONLY = ["UTPP_LOCO_ONLY057", "UTPP_LOCO_ONLY058", "UTPP_LOCO_ONLY059", "UTPP_LOCO_ONLY060", "UTPP_LOCO_ONLY062",
        "UTPP_LOCO_ONLY063", "UTPP_LOCO_ONLYC1A", "UTPP_LOCO_ONLYC1B", "UTPP_LOCO_ONLYIDLE", "UTPP_LOCO_ONLYKNEE",
        "UTPP_LOCO_ONLYSTRAFE", "UTPP_LOCO_ONLYAIR"]
# diagnostic / mode variables that change what a run prints: cleared for the alone runs (the full log must come from a plain run)
CLEAR = ["UTPP_LOCO_C1A_ALL", "UTPP_LOCO_STRAFE_ALL", "UTPP_LOCO_AIR_ALL", "UTPP_LOCO_SECTION_STATUS"] + ONLY
KEEP = re.compile(r"^\[(INFO|PASS|FAIL)\]")
MARK = re.compile(r"^==== SECTION (\S+)")
WALL = [re.compile(p) for p in (r"(snapshot age max )[-0-9.]+", r"(\bage )[-0-9.]+", r"(strafeSUs=)[-0-9./]+", r"(airUs=)[-0-9./]+",
                                r"(zeroAfter )[-0-9.]+", r"(holdTicks )[0-9]+", r"(\bUs=)[-0-9./]+")]


def norm(line):
    for w in WALL:
        line = w.sub(r"\1#", line)
    return line


def read(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        return [norm(l.rstrip("\r\n")) for l in f]


def sections(lines):
    out = {}
    cur = None
    for l in lines:
        m = MARK.match(l)
        if m:
            cur = m.group(1)
            out.setdefault(cur, [])
            continue
        if l.startswith("RESULT "):
            cur = None
        if cur and KEEP.match(l):
            out[cur].append(l)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--full", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("dbs", nargs="+")
    a = ap.parse_args()
    if not os.path.isfile(a.exe) or not os.path.isfile(a.full):
        print("LOCO_ORDER setup error: missing exe or full log")
        return 2
    os.makedirs(a.out, exist_ok=True)
    fullSec = sections(read(a.full))
    if not fullSec:
        print("LOCO_ORDER setup error: no section markers in the full log")
        return 2
    procs = []
    for v in ONLY:
        env = {k: val for k, val in os.environ.items() if k.upper() not in CLEAR}
        env[v] = "1"
        log = os.path.join(a.out, f"loco_order_{v}.log")
        f = open(log, "wb")
        p = subprocess.Popen([a.exe] + a.dbs, stdout=f, stderr=subprocess.STDOUT, env=env)
        procs.append((v, p, f, log))
    bad = 0
    seen = set()
    for v, p, f, log in procs:
        rc = p.wait()
        f.close()
        lines = read(log)
        aSec = sections(lines)
        result = next((l for l in reversed(lines) if l.startswith("RESULT ")), "no RESULT line")
        if rc != 0 or not aSec:
            print(f"[FAIL] {v}: alone run exit {rc}, {len(aSec)} sections, {result}")
            bad += 1
            continue
        for name, blk in aSec.items():
            seen.add(name)
            fb = fullSec.get(name)
            if fb is None:
                print(f"[FAIL] {v} [{name}]: section not in the full log")
                bad += 1
                continue
            diffs = [(k, x, y) for k, (x, y) in enumerate(zip(blk, fb)) if x != y]
            if len(blk) != len(fb):
                diffs.append((-1, f"len alone {len(blk)}", f"len full {len(fb)}"))
            print(f"[{'FAIL' if diffs else 'PASS'}] {v} [{name}]: {len(blk)} lines alone vs full, {len(diffs)} differ ({result})")
            for k, x, y in diffs[:4]:
                print(f"   #{k}\n     alone: {x[:300]}\n     full : {y[:300]}")
            bad += 1 if diffs else 0
    missing = sorted(set(fullSec) - seen)
    if missing:
        print(f"[FAIL] full-suite sections without an alone run: {' '.join(missing)}")
        bad += 1
    print(f"LOCO_ORDER sections {len(seen)}, different / failed {bad}")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
