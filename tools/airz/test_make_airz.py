"""Offline tests for make_airz.py (negative paths). Writes only under Build\\test-data\\airz_test (no data in Source)."""
import contextlib
import io
import json
import os
import shutil
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_airz as m  # noqa: E402

WORK = os.path.normpath(os.path.join(HERE, "..", "..", "..", "Build", "test-data", "airz_test"))
REAL_AIRZ = os.path.splitext(m.DEF_DB)[0] + ".airz"
fails = []


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    if not cond:
        fails.append(msg)


def run(argv):
    return run_cap(argv)[0]


def run_cap(argv):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = m.main(argv + ["--log", "-"])
    return rc, buf.getvalue()


def expect(argv, rc_want, needle, msg):
    try:
        rc, out = run_cap(argv)
    except Exception as e:  # a traceback is a failure of the tool
        check(False, "%s (raised %s: %s)" % (msg, type(e).__name__, e))
        return
    check(rc == rc_want and needle in out, "%s (rc=%s, want %d, %r %s)"
          % (msg, rc, rc_want, needle, "found" if needle in out else "MISSING"))


def main():
    if os.path.isdir(WORK):
        shutil.rmtree(WORK)
    os.makedirs(WORK)
    # 1. real file verifies
    check(run(["--verify", "--out", REAL_AIRZ]) == 0, "real .airz verifies")
    blob = open(REAL_AIRZ, "rb").read()
    _, entries, _ = m.read_airz(REAL_AIRZ)
    # 2. tampered root_z value -> RTZ recheck fails
    p = os.path.join(WORK, "tamper_value.airz")
    b = bytearray(blob)
    off = 48 + 6 + len(entries[0][1].encode()) + 4
    struct.pack_into("<f", b, off, struct.unpack_from("<f", b, off)[0] + 0.5)
    open(p, "wb").write(b)
    check(run(["--verify", "--out", p]) == 1, "tampered root_z value fails verify")
    check(run(["--verify", "--out", p, "--no-rtz-recheck"]) == 0, "tampered value passes structural-only verify (expected)")
    # 3. wrong DB SHA
    p = os.path.join(WORK, "tamper_sha.airz")
    b = bytearray(blob)
    b[12] ^= 0xFF
    open(p, "wb").write(b)
    check(run(["--verify", "--out", p, "--no-rtz-recheck"]) == 1, "wrong DB SHA-256 fails verify")
    # 4. truncated / trailing bytes / bad magic
    p = os.path.join(WORK, "trunc.airz")
    open(p, "wb").write(blob[:-3])
    check(run(["--verify", "--out", p, "--no-rtz-recheck"]) == 1, "truncated file fails verify")
    p = os.path.join(WORK, "trail.airz")
    open(p, "wb").write(blob + b"\0")
    check(run(["--verify", "--out", p, "--no-rtz-recheck"]) == 1, "trailing byte fails verify")
    p = os.path.join(WORK, "magic.airz")
    open(p, "wb").write(b"UTPPAIRX" + blob[8:])
    check(run(["--verify", "--out", p, "--no-rtz-recheck"]) == 1, "bad magic fails verify")
    # 5. generator skips a clip whose root_z is off by 2 cm and one with a frame count mismatch
    rdir = os.path.join(WORK, "rtz")
    os.makedirs(rdir)
    names = [e[1] for e in entries[:3]]
    for i, n in enumerate(names):
        j = json.load(open(os.path.join(m.DEF_RTZ, n + ".utppclip.json"), encoding="utf-8"))
        if i == 0:
            j["root_z"][len(j["root_z"]) // 2] += 0.02
        elif i == 1:
            j["root_z"].pop()
        json.dump(j, open(os.path.join(rdir, n + ".utppclip.json"), "w", encoding="utf-8"))
    out = os.path.join(WORK, "gen.airz")
    check(run(["--rtz-dir", rdir, "--out", out]) == 0, "generator runs on the modified RTZ dir")
    _, ent2, _ = m.read_airz(out)
    check([e[1] for e in ent2] == names[2:], "2 cm deviation and frame-count mismatch are skipped, good clip kept")
    # 6. determinism: regenerate the real file into WORK, bytes equal
    out = os.path.join(WORK, "regen.airz")
    run(["--out", out])
    check(open(out, "rb").read() == blob, "regeneration is byte-identical to Data .airz")
    # 7. each verify-side structural check is exercised alone (structural mode, so no RTZ recheck masks it)
    db_sha = blob[12:44]
    ndb = m.MiniDb(m.DEF_DB).num_clips

    def mutant(tag, ents):
        q = os.path.join(WORK, "mut_%s.airz" % tag)
        m.write_airz(q, db_sha, ents)
        return q

    q = mutant("control", [(c, n, list(r)) for c, n, r in entries])
    expect(["--verify", "--out", q, "--no-rtz-recheck"], 0, "verify: PASS", "rewritten unchanged entries verify")
    e2 = [(c, n, list(r)) for c, n, r in entries]
    e2[0] = (e2[0][0], e2[0][1] + "_x", e2[0][2])
    expect(["--verify", "--out", mutant("name", e2), "--no-rtz-recheck"], 1, "name", "clip name != DB name fails verify")
    e2 = [(c, n, list(r)) for c, n, r in entries]
    e2[0], e2[1] = e2[1], e2[0]
    expect(["--verify", "--out", mutant("order", e2), "--no-rtz-recheck"], 1, "not strictly ascending",
           "non-ascending clipIndex fails verify")
    e2 = [(c, n, list(r)) for c, n, r in entries] + [(ndb, "Bogus", [0.0])]
    expect(["--verify", "--out", mutant("range", e2), "--no-rtz-recheck"], 1, "out of range",
           "clipIndex >= DB clip count fails verify")
    e2 = [(c, n, list(r)) for c, n, r in entries]
    e2[0] = (e2[0][0], e2[0][1], e2[0][2][:-1])
    expect(["--verify", "--out", mutant("count", e2), "--no-rtz-recheck"], 1, "!= db",
           "numFrames != DB clip frame count fails verify")
    for tag, val in (("nan", float("nan")), ("inf", float("inf"))):
        e2 = [(c, n, list(r)) for c, n, r in entries]
        e2[0][2][0] = val
        expect(["--verify", "--out", mutant(tag, e2), "--no-rtz-recheck"], 1, "non-finite root_z",
               "%s root_z fails verify" % tag)
    # 8. generator: non-finite and malformed RTZ json is skipped+logged (no traceback), --strict refuses to write
    rdir = os.path.join(WORK, "rtz_bad")
    os.makedirs(rdir)
    bad = {}
    cases = [
        ("nan_root_z", "non-finite root_z"),
        ("nan_hips", "non-finite Hips"),
        ("no_bones_key", "malformed RTZ frame"),
        ("short_hips", "malformed RTZ frame"),
        ("frame_not_dict", "malformed RTZ frame"),
        ("hips_z_string", "is not a number"),
        ("json_list", "not an object"),
        ("fps_string", "not a finite number"),
    ]
    for i, (tag, why) in enumerate(cases):
        n = entries[3 + i][1]
        j = json.load(open(os.path.join(m.DEF_RTZ, n + ".utppclip.json"), encoding="utf-8"))
        jh = j["bone_names"].index("Hips")
        k = len(j["frames"]) // 2
        if tag == "nan_root_z":
            j["root_z"][k] = float("nan")
        elif tag == "nan_hips":
            j["frames"][k]["bones"][jh][2] = float("nan")
        elif tag == "no_bones_key":
            del j["frames"][k]["bones"]
        elif tag == "short_hips":
            j["frames"][k]["bones"][jh] = j["frames"][k]["bones"][jh][:2]
        elif tag == "frame_not_dict":
            j["frames"][k] = 5
        elif tag == "hips_z_string":
            j["frames"][k]["bones"][jh][2] = "0.9"
        elif tag == "json_list":
            j = [j]
        elif tag == "fps_string":
            j["fps"] = "thirty"
        json.dump(j, open(os.path.join(rdir, n + ".utppclip.json"), "w", encoding="utf-8"))
        bad[n] = why
    good = entries[3 + len(cases)][1]
    shutil.copy(os.path.join(m.DEF_RTZ, good + ".utppclip.json"), rdir)
    out = os.path.join(WORK, "gen_bad.airz")
    rc, txt = run_cap(["--rtz-dir", rdir, "--out", out])
    check(rc == 0, "generator survives malformed/non-finite RTZ json (rc=%s)" % rc)
    for n, why in bad.items():
        line = [ln for ln in txt.splitlines() if (" %s " % n) in ln and "SKIP" in ln]
        check(len(line) == 1 and why in line[0], "%s skipped with reason %r" % (n, why))
    _, ent3, _ = m.read_airz(out)
    check([e[1] for e in ent3] == [good], "only the good clip is written")
    out_s = os.path.join(WORK, "gen_strict.airz")
    expect(["--rtz-dir", rdir, "--out", out_s, "--strict"], 1, "STRICT", "--strict returns 1 when clips are skipped")
    check(not os.path.exists(out_s), "--strict writes nothing when clips are skipped")
    expect(["--verify", "--out", REAL_AIRZ, "--rtz-dir", rdir], 1, "RTZ recheck",
           "verify RTZ recheck against malformed json fails cleanly")
    # 9. DB without a Hips bone -> rc 2 (no traceback) in generate and verify
    orig_init = m.MiniDb.__init__

    def no_hips_init(self, path):
        orig_init(self, path)
        self.bone_names = ["NotHips" if b == "Hips" else b for b in self.bone_names]
    m.MiniDb.__init__ = no_hips_init
    try:
        expect(["--out", os.path.join(WORK, "nohips.airz")], 2, "not found", "generate with no Hips bone -> rc 2")
        expect(["--verify", "--out", REAL_AIRZ], 2, "not found", "verify with no Hips bone -> rc 2")
    finally:
        m.MiniDb.__init__ = orig_init
    check(not os.path.exists(os.path.join(WORK, "nohips.airz")), "no output written without a Hips bone")
    print("RESULT: %s (%d failures)" % ("PASS" if not fails else "FAIL", len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
