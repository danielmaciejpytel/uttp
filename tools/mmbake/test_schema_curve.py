"""Unit tests of the schema compiler and SCHC writer for Curve channels and off-grid time offsets (mmbake_motion.py).

    python test_schema_curve.py            exit code 0 = every check passed
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mmbake_motion as mm  # noqa: E402

FAILS = []


def check(ok, what):
    print(("PASS " if ok else "FAIL ") + what)
    if not ok:
        FAILS.append(what)


def raises(fn):
    try:
        fn()
    except SystemExit:
        return True
    return False


class Strings:
    def __init__(self):
        self.names = {}

    def add(self, n):
        return self.names.setdefault(n, 4 * (len(self.names) + 1))


def traj(offsets):
    return {"_class": "PoseSearchFeatureChannel_Trajectory", "weight": 1.0,
            "samples": [{"offset": o, "flags": 2 | 32, "weight": 1.0, "normalization_group": "None"} for o in offsets]}


def curve(offset, query="USE_CONTINUING_POSE", name="Phase"):
    return {"_class": "PoseSearchFeatureChannel_Curve", "curve_name": name, "input_query_pose": query,
            "normalization_group": "None", "sample_time_offset": offset, "weight": 1.0}


def schema(channels):
    return {"_path": "/x/PSS_Test.PSS_Test", "channels": channels, "sample_rate": 30, "number_of_permutations": 1}


def main():
    # 1. existing schemas compile unchanged with the default arguments
    base = schema([traj([-0.5, 0.0, 0.5])])
    chans, dims = mm.compile_schema(base)
    check(dims == 6 and all(c.kind == mm.CK_POSITION for c in chans), "trajectory-only schema compiles as before")

    # 2. Curve channels: refused by default, accepted with allow_curve (offsets 0 and -1/30 s, continuing pose)
    sc = schema([traj([0.0]), curve(0.0), curve(-1.0 / 30.0)])
    check(raises(lambda: mm.compile_schema(sc)), "Curve channel refused without allow_curve")
    chans, dims = mm.compile_schema(sc, allow_curve=True)
    cur = [c for c in chans if c.kind == mm.CK_CURVE]
    check(len(cur) == 2 and dims == 4, "two Curve channels, one dimension each")
    check(all(c.card == 1 and c.comp == mm.COMP_SCALAR and c.curve == "Phase" for c in cur), "Curve cardinality / name")
    check([c.offset for c in cur] == [2, 3], "Curve data offsets follow the trajectory")
    check(all(c.flags & mm.CHF_UNITLESS and not c.flags & mm.CHF_Z_FROM_RTZ for c in cur), "Curve flags: unitless, no RTZ")
    check(abs(cur[1].t_sample + 1.0 / 30.0) < 1e-9, "Curve offset -1/30 s kept")
    check(not cur[0].can_normalize_with(cur[1]) and cur[0].can_normalize_with(cur[0]),
          "Curve channels of different offsets normalize separately (UE label rule)")
    pos = [c for c in chans if c.kind == mm.CK_POSITION][0]
    check(not pos.can_normalize_with(cur[0]), "Curve never normalizes with a position")
    check(raises(lambda: mm.compile_schema(schema([dict(curve(0.0), curve_name="None")]), allow_curve=True)),
          "Curve channel without a curve name refused")
    ch_char, _ = mm.compile_schema(schema([curve(0.0, "USE_CHARACTER_POSE")]), allow_curve=True)
    check(ch_char[0].input_query == mm.IQ_CHARACTER, "character-pose Curve channel compiles")

    # 3. SCHC record of a Curve channel
    st = Strings()
    rec = mm.schc_record(cur[1], 4.0, st, {"Speed": 0, "Phase": 3})
    check(len(rec) == 76, "SCHC record is 76 bytes")
    f = struct.unpack("<IIiiffIIffIIIIfffII", rec)
    check(f[0] == 5 and f[1] == 1 and f[2] == -1 and f[3] == -1, "kind 5, components 1, bone / origin -1")
    check(abs(f[4] + 1.0 / 30.0) < 1e-7 and f[5] == 0.0 and f[6] == 0 and f[7] == 0, "t_sample, t_origin 0, no velocity space, query 0")
    check(f[10] == 3 and f[11] == 1 and f[18] == 3, "data offset, cardinality 1, curve index in the last field")
    check(f[17] == mm.CHF_UNITLESS and f[14:17] == (0.0, 0.0, 0.0), "flags unitless only, axis 0")
    check(raises(lambda: mm.schc_record(cur[1], 4.0, st, {"Speed": 0})), "Curve of an unbaked curve refused")
    # non-curve record unchanged (last field 0)
    rp = struct.unpack("<IIiiffIIffIIIIfffII", mm.schc_record(pos, 4.0, st, {"Phase": 3}))
    check(rp[0] == mm.CK_POSITION and rp[18] == 0, "position record keeps the zero last field")

    # 4. header flags
    check(mm.header_flags(False, {"a": ([pos], 2)}) == 0 and mm.header_flags(True, {"a": ([pos], 2)}) == mm.HDR_HAS_SCALE,
          "header flags without curves unchanged")
    check(mm.header_flags(False, {"a": (chans, dims)}) == mm.HDR_CURVE_CHANNELS, "CURVE_CHANNELS flag with a Curve channel")

    # 5. off-grid offsets (reference -0.125 s and 0.33 s): refused by default, quantized on request
    og = schema([traj([-0.125, 0.0, 0.33])])
    check(raises(lambda: mm.compile_schema(og)), "off-grid trajectory offset refused by default")
    chans, _ = mm.compile_schema(og, quantize=True)
    ts = [c.t_sample for c in chans]
    check(ts[0] == float(mm.F32(-8.0 / 60.0)) and ts[2] == float(mm.F32(20.0 / 60.0)) and ts[1] == 0.0,
          "quantized to the nearest 1/60 s (ties away from zero): -0.125 -> -8/60, 0.33 -> 20/60")
    check(len(mm.compile_schema.quantized) == 2, "quantized offsets reported")
    check(mm.quantize_offset(0.5) == 0.5 and mm.quantize_offset(-1.0 / 30.0) == float(mm.F32(-2.0 / 60.0)),
          "on-grid offsets unchanged")

    print("%d FAIL" % len(FAILS))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
