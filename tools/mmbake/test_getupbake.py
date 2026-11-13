"""test_getupbake.py - independent checks of the get-up files written by getupbake.py (spec 6.3) plus unit tests of the fix-up maths.

Reads Data/getup/utpp_getup_{woman,man}.utpptrav (+ .contacts.json) with the Python port of ParseTraversalText, runs FK from the
P lines and checks, for every one of the 8 clips (woman / man x B / F / L / R), on frames 0..cut:
  floor        min_j (z_j - r_j) >= -0.005 for the clamp set + fingers (shared radii table)
  mesh floor   lowest vertex of the linear-blend-skinned player body (body + arms_hq + head, getup_mesh.py) >= -0.005 on frames
               0..cut (floor z = 0), hover (lowest vertex while Hips < 0.4 m) <= 0.06, plus skinning sanity (idle pose sits on the
               floor, the arm mesh follows the arm chain) and a "lower everything 3 cm" negative control. Skipped (reported) when the
               WolvenKit mesh exports under Build/test-data/getup-mesh are not on the machine.
  slip         planted effectors (contacts.json flags) move <= 0.01 m in XY inside every contact interval
  steps        every local joint rotation changes <= 25 deg per frame
  lengths      local bone translations of all non-Root-child bones equal the source clip's (1e-4)
  round trip   montage order GetUp_B/F/L/R, 73 POSEBONES = clip bones, SEC HandOff == (frames-1)/fps, Interrupt <= HandOff,
               rig hash == travbake.rig_info, one P line per frame (the grammar port rejects anything else)
  hips         Hips of the last frame within 0.05 m XY of Root; Hips z >= 0.9 x idle hips z
  toes         C's M3 on the toes (z < 0.06, speed < 0.25, >= 3 frames) moves <= 0.03 m; no hand planted at Interrupt; hips step <= 0.06
  trajectory   source Trajectory (the R lines) at the cut is (0, 0) within 0.02 m  (the "no root motion" decision)
  bake         planted slip is smaller with the Trajectory folded into the Hips than without (clips whose Trajectory moves)
plus unit tests (envelope smoothing, quaternion helpers, IK bone lengths, grammar negatives) and negative controls of the
checks themselves (a lowered Hips, a slid wrist, a 40 deg pop must be caught).
If the source pack data (Data/getup, Animations/retarget) is not on the machine the data checks are skipped and reported.

usage: python -I test_getupbake.py [--root <project>] [--rebake]
exit 0 = all checks passed.
"""
import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import getup_fixup as F  # noqa: E402
import getup_mesh as GM  # noqa: E402
import getupbake as B  # noqa: E402
import travbake as T  # noqa: E402

PASS = FAIL = SKIP = 0


def check(ok, msg):
    global PASS, FAIL
    if ok:
        PASS += 1
    else:
        FAIL += 1
        print("FAIL: " + msg)


FLOOR_TOL = 0.005
MESH_FLOOR_TOL = 0.005    # lowest skinned vertex >= -5 mm (the baker keeps >= +1.5 mm; the offline skin is not the engine skin)
MESH_HOVER_MAX = 0.06     # lowest vertex while the Hips are low: floating more than 6 cm would be a regression (woman B: 4.5 cm,
                          # pinned by the shared Spine3 clamp radius 0.10)
SLIP_MAX = 0.01
STEP_MAX_DEG = 25.0
LEN_TOL = 1e-4
HIPS_XY_MAX = 0.05
TRAJ_CUT_MAX = 0.02


# ---------------------------------------------------------------------------------------------------------------------
# the checks (pure functions over world arrays so that the negative controls can feed mutated data)
def floor_min(names, wp, upto):
    radii = F.clamp_radii(names)
    worst = (1e9, None, None)
    for b, r in radii.items():
        cl = wp[:upto + 1, b, 2] - r
        t = int(np.argmin(cl))
        if cl[t] < worst[0]:
            worst = (float(cl[t]), names[b], t)
    return worst


def slip_max(names, wp, flags):
    worst = 0.0
    for key, fl in flags.items():
        e = names.index(F.LIMBS[key][2])
        for a, b in B.runs_of(np.array(fl, dtype=bool)):
            seg = wp[a:b + 1, e, :2]
            worst = max(worst, float(np.linalg.norm(seg - seg.mean(0), axis=1).max()))
    return worst


def segment_dist(p, a, b):
    d = b - a
    t = np.clip(((p - a) @ d) / float(d @ d), 0.0, 1.0)
    return np.linalg.norm(p - (a + np.outer(t, d)), axis=1)


def mesh_sanity(ms, zs, root, rig, body, tag):
    """Skinning calibration on the Neutral idle pose: the sole sits on the floor (the woman idle foot is toe-up by ~4 cm) and the
    hq arm meshes stay within 0.12 m of the shoulder-arm-forearm-hand-finger chain (a wrong joint frame would swing them far off)."""
    idle = F.Clip(os.path.join(root, "Animations/retarget", "out_sourcepack_%s" % body, "M_Neutral_Stand_Idle_Loop.utppclip.json"), rig)
    wp, wq = F.fk(idle.pos[:1], idle.rot[:1], idle.par)
    v = ms.skin(idle.names, wp, wq)[0]
    check(-0.01 <= v[:, 2].min() <= 0.06, "%s mesh sanity: idle lowest vertex %.3f within [-0.01, 0.06]" % (tag, v[:, 2].min()))
    for side, key in (("Left", "hq__l"), ("Right", "hq__r")):
        sel = np.array([key in x for x in ms.src])
        chain = [side + n for n in ("Shoulder", "Arm", "ForeArm", "Hand", "HandMiddle3")]
        d = np.min([segment_dist(v[sel], wp[0, idle.idx[chain[i]]], wp[0, idle.idx[chain[i + 1]]]) for i in range(len(chain) - 1)], axis=0)
        check(d.max() <= 0.12, "%s mesh sanity: %s arm mesh max distance to the arm chain %.3f <= 0.12" % (tag, side, d.max()))


def step_max_deg(q):
    return float(np.degrees(F.qangle(q[:-1], q[1:])).max())


# ---------------------------------------------------------------------------------------------------------------------
def unit_tests():
    # envelope: never below the input, equals the reference, rate-limited
    rng = np.random.RandomState(3)
    for _ in range(20):
        x = np.abs(rng.randn(40)) * (rng.rand(40) > 0.7)
        e = F.env_smooth(x)
        check((e >= x - 1e-12).all(), "env_smooth >= input")
        check(np.allclose(e, F.env_smooth_ref(x)), "env_smooth == reference")
        r = F.env_smooth(x, 0.05)
        check((r >= x - 1e-12).all(), "env_smooth(rate) >= input")
        check((F.env_smooth(x, 0.05) >= F.env_smooth(x) - 1e-12).all(), "rate limited envelope >= plain envelope")
    # plain triangular blur lowers a peak (the reason for the max filter)
    pk = np.zeros(20)
    pk[10] = 1.0
    check(F.blur(pk)[10] < 0.5 and F.env_smooth(pk)[10] >= 1.0 - 1e-12, "blur lowers a peak, env_smooth does not")
    # quaternion helpers
    for _ in range(50):
        u, v = F.unit(rng.randn(3)), F.unit(rng.randn(3))
        q = F.rot_between(u, v)
        check(np.allclose(F.qrot(q, u), v, atol=1e-9), "rot_between maps u to v")
    q = F.rot_between(np.array([0, 0, 1.0]), np.array([0, 0, -1.0]))
    check(np.allclose(F.qrot(q, np.array([0, 0, 1.0])), [0, 0, -1.0], atol=1e-9), "rot_between antiparallel")
    # two-bone IK: lengths preserved for reachable and unreachable targets, pole preserved
    S = np.array([0.0, 0.0, 0.0])
    M0 = np.array([0.0, -0.1, -0.28])
    E0 = np.array([0.0, 0.0, -0.55])
    l1, l2 = np.linalg.norm(M0 - S), np.linalg.norm(E0 - M0)
    for tgt in ([0.1, 0.1, -0.4], [0.0, 0.0, -0.2], [0.5, 0.2, -0.9], [0.0, 0.0, -0.5499]):
        Mn, En = F.Solver.ik_mid(S, np.array(tgt), M0, l1, l2, E0, 0.0)
        check(abs(np.linalg.norm(Mn - S) - l1) < 1e-9 and abs(np.linalg.norm(En - Mn) - l2) < 1e-9,
              "IK bone lengths for target %s" % (tgt,))
    Mn, En = F.Solver.ik_mid(S, np.array([0.0, 0.0, -0.4]), M0, l1, l2, E0, 0.0)
    check(Mn[1] < 0, "IK keeps the pole direction (elbow stays behind)")
    # grammar negatives
    good = None
    try:
        good = _tiny_file()
        B.parse_trav(good)
        check(True, "tiny file parses")
    except ValueError as ex:
        check(False, "tiny file parses: %s" % ex)
    if good:
        for name, bad in (("CRLF", good.replace("\n", "\r\n")), ("no final LF", good[:-1]),
                          ("double space", good.replace("COUNT 1", "COUNT  1")),
                          ("bad header", good.replace("utpp-traversal 2", "utpp-traversal 3")),
                          ("missing P line", good.replace("P 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000\nP",
                                                          "P", 1)),
                          ("non unit quaternion", good.replace("0.000000 0.000000 0.000000 1.000000\nE", "0.000000 0.000000 0.000000 1.100000\nE")),
                          ("missing END", good.replace("END\n", "")), ("W window", good.replace("R 0", "W x\nR 0", 1))):
            try:
                B.parse_trav(bad)
                check(False, "grammar negative accepted: " + name)
            except ValueError:
                check(True, "grammar negative rejected: " + name)


def _tiny_file():
    p = "P 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000"
    lines = ["utpp-traversal 2", "RIG test 0x0000000000000001 3", "POSEBONES 1 Hips", "COUNT 1",
             "M GetUp_B seq 0.033333 1.000000 0.300000 SmoothStep 0.300000 SmoothStep 1 -1.000000 Other 30.000000 2 0",
             "SEC Default 0.000000 -", "SEC HandOff 0.033333 -", "R 0.000000 0.000000 0.000000 0.000000",
             "R 0.000000 0.000000 0.000000 0.000000", "C Distance_From_Ledge -", p, p, "E", "END"]
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------------------------------------------------
class B_clip:
    """Minimal clip-like object (idx, fps) for the helpers that take a Clip."""

    def __init__(self, names, src):
        self.idx = {n: i for i, n in enumerate(names)}
        self.fps = src.fps


def data_tests(root, fixture_dir):
    global SKIP
    for body in ("woman", "man"):
        trav = os.path.join(root, B.DATA_DIR, "utpp_getup_%s.utpptrav" % body)
        side = os.path.join(root, B.DATA_DIR, "utpp_getup_%s.contacts.json" % body)
        src0 = os.path.join(root, B.SRC_DIR % body, "M_ragdoll_getup_stand_B.utppclip.json")
        if not (os.path.isfile(trav) and os.path.isfile(side) and os.path.isfile(src0)):
            print("SKIP %s: get-up data not present (%s)" % (body, trav))
            SKIP += 1
            continue
        rig = F.Rig(root, body)
        text = open(trav, "rb").read().decode("ascii")
        parsed = B.parse_trav(text)
        sd = json.load(open(side, encoding="utf-8"))
        names = parsed["pose_bones"]
        check(parsed["rig"] == rig.rig_name and parsed["rig_hash"] == rig.hash and parsed["rig_bones"] == len(rig.names),
              "%s rig identity" % body)
        check([m["name"] for m in parsed["montages"]] == ["GetUp_B", "GetUp_F", "GetUp_L", "GetUp_R"], "%s montage order" % body)
        check(len(names) == 73, "%s 73 pose bones" % body)
        idle = B.load_idle(root, rig, body)
        ms = zs = None
        if GM.available(GM.default_mesh_root(root), body):
            ms = GM.MeshSet(GM.default_mesh_root(root), body)
            zs = GM.ZSkin(ms, names)
            mesh_sanity(ms, zs, root, rig, body, body)
        else:
            print("SKIP %s mesh floor checks: body mesh exports not present under %s" % (body, GM.default_mesh_root(root)))
            SKIP += 1
        for ci, c in enumerate(B.CLIPS):
            tag = "%s %s" % (body, c)
            m = parsed["montages"][ci]
            src = F.Clip(os.path.join(root, B.SRC_DIR % body, "M_ragdoll_getup_stand_%s.utppclip.json" % c), rig)
            check(names == src.names, tag + " POSEBONES == source clip bones (order)")
            check(m["sequence"] == src.name, tag + " sequence name")
            cut = m["frames"] - 1
            hand = B.sec_time(m, "HandOff")
            inter = B.sec_time(m, "Interrupt")
            check(hand is not None and abs(hand - cut / m["fps"]) < 1e-5, tag + " SEC HandOff == (frames-1)/fps")
            check(inter is not None and 0 <= inter <= hand, tag + " SEC Interrupt <= HandOff")
            check(abs(m["length"] - cut / m["fps"]) < 1e-5 and m["fps"] == 30.0, tag + " length / fps")
            check(sd["clips"][c]["cut"] == cut, tag + " contacts.json cut")
            wp, wq, par = B.parsed_fk(parsed, ci, rig)
            # floor
            fm = floor_min(names, wp, cut)
            check(fm[0] >= -FLOOR_TOL, "%s floor min clearance %.4f (%s f%d) >= -%.3f" % (tag, fm[0], fm[1], fm[2], FLOOR_TOL))
            if zs is not None:
                zz = zs.z(wp[:cut + 1], wq[:cut + 1])
                mz = float(zz.min())
                check(mz >= -MESH_FLOOR_TOL, "%s skinned mesh floor clearance %.4f (f%d, %s) >= -%.3f" % (
                    tag, mz, int(np.argmin(zz.min(1))), ms.region(int(np.argmin(zz[int(np.argmin(zz.min(1)))]))), MESH_FLOOR_TOL))
                low = wp[:cut + 1, names.index("Hips"), 2] < 0.4
                hov = float(zz.min(1)[low].max()) if low.any() else 0.0
                check(hov <= MESH_HOVER_MAX, "%s mesh hover %.3f <= %.2f" % (tag, hov, MESH_HOVER_MAX))
                lowz = zs.z(wp[:cut + 1], wq[:cut + 1]) - 0.03
                check(float(lowz.min()) < -MESH_FLOOR_TOL, tag + " control: lowering the mesh 3 cm fails the mesh floor check")
            # slip on the flagged contact intervals
            flags = {k: sd["clips"][c]["contacts"][k][:cut + 1] for k in F.LIMB_KEYS}
            sm = slip_max(names, wp, flags)
            check(sm <= SLIP_MAX, "%s planted slip %.4f <= %.3f" % (tag, sm, SLIP_MAX))
            check(sum(sum(v) for v in flags.values()) > 0, tag + " has planted frames")
            # joint steps
            st = step_max_deg(m["pose"][:, :, 3:7])
            check(st <= STEP_MAX_DEG, "%s max joint step %.1f deg <= %.0f" % (tag, st, STEP_MAX_DEG))
            # C's M3 definition on the toes (z < 0.06, speed < 0.25 m/s, >= 3 frames): xy extent <= 0.03
            toes = F.toe_plant_stats(B_clip(names, src), wp, cut)
            check(max(toes.values()) <= 0.03, "%s toe plant extent %.3f <= 0.03 (C's M3)" % (tag, max(toes.values())))
            # no planted hand at the Interrupt frame (an early hand-off must not drag a planted hand)
            fi = int(round(inter * m["fps"]))
            check(not (flags["LH"][fi] or flags["RH"][fi]), tag + " no hand planted at the Interrupt frame")
            # M2-style hips continuity: the source itself rises at up to ~1.8 m/s, i.e. 0.06 m per frame at 30 fps (the metric
            # tool measures at game rate where the step is smaller). The fix-up must not make it worse than the source.
            dh = np.linalg.norm(np.diff(wp[:, names.index("Hips")], axis=0), axis=1)
            pos_s, rot_s, _ = F.bake_trajectory(src)
            wp_s, _ = F.fk(pos_s[:m["frames"]], rot_s[:m["frames"]], src.par)
            dh_s = np.linalg.norm(np.diff(wp_s[:, src.idx["Hips"]], axis=0), axis=1)
            check(dh.max() <= dh_s.max() + 0.005 and dh.max() <= 0.065,
                  "%s max hips step %.4f (source %.4f) at 30 fps" % (tag, dh.max(), dh_s.max()))
            # bone lengths vs the source
            pos_src = src.pos[:m["frames"]]
            nonroot = [b for b in range(len(names)) if src.par[b] > 0]
            ld = np.abs(np.linalg.norm(m["pose"][:, nonroot, 0:3], axis=2) - np.linalg.norm(pos_src[:, nonroot], axis=2)).max()
            check(ld <= LEN_TOL, "%s bone length drift %.2e" % (tag, ld))
            # the non-Hips root-space data is source data: bones not touched by the fix-up are bit equal (spine chain)
            same = np.abs(m["pose"][:, [names.index("Spine"), names.index("Spine3"), names.index("Head")], 0:3]
                          - pos_src[:, [names.index("Spine"), names.index("Spine3"), names.index("Head")]]).max()
            check(same < 1e-5, tag + " trunk translations equal the source")
            # hips of the last frame near Root
            hx = wp[cut, names.index("Hips"), :2]
            check(float(np.linalg.norm(hx)) <= HIPS_XY_MAX, "%s hips xy at cut %.3f <= %.2f" % (tag, np.linalg.norm(hx), HIPS_XY_MAX))
            check(wp[cut, names.index("Hips"), 2] >= 0.9 * idle["wp"][idle["idx"]["Hips"], 2], tag + " hips z at cut >= 0.9 x idle")
            # Trajectory at the cut (source track kept in the R lines)
            tr = m["root"][cut]
            check(math.hypot(tr[0], tr[1]) <= TRAJ_CUT_MAX, "%s Trajectory at cut (%.3f, %.3f) within %.2f" % (tag, tr[0], tr[1], TRAJ_CUT_MAX))
            tr0 = np.array([r[:2] for r in m["root"]])
            check(np.abs(tr0 - np.array([[float(src.pos[f, src.idx['Trajectory'], 0]), float(src.pos[f, src.idx['Trajectory'], 1])]
                                         for f in range(m["frames"])])).max() < 1e-5, tag + " R lines are the source Trajectory")
            # the Trajectory bone itself is identity in the P lines (folded into the Hips)
            ti = names.index("Trajectory")
            check(np.abs(m["pose"][:, ti, 0:3]).max() == 0 and np.abs(m["pose"][:, ti, 3:7] - [0, 0, 0, 1]).max() < 1e-6,
                  tag + " Trajectory bone is identity in the P lines")
            # bake decision: slip with / without the Trajectory
            fl_ind = flags
            pos_nb, rot_nb = src.pos[:m["frames"]], src.rot[:m["frames"]]
            wp_nb, _ = F.fk(pos_nb, rot_nb, src.par)
            pos_b, rot_b, _ = F.bake_trajectory(src)
            wp_b, _ = F.fk(pos_b[:m["frames"]], rot_b[:m["frames"]], src.par)
            s_nb = slip_max(names, wp_nb, fl_ind)
            s_b = slip_max(names, wp_b, fl_ind)
            if np.abs(tr0).max() > 0.05:
                check(s_b < s_nb, "%s source planted slip: %.3f with Trajectory bake < %.3f without" % (tag, s_b, s_nb))
            # negative controls of the checks
            lo = wp.copy()
            lo[:, :, 2] -= 0.03
            check(floor_min(names, lo, cut)[0] < -FLOOR_TOL, tag + " control: lowering everything 3 cm fails the floor check")
            sl = wp.copy()
            for k, fl in flags.items():
                if any(fl):
                    e = names.index(F.LIMBS[k][2])
                    a, b = max(B.runs_of(np.array(fl, dtype=bool)), key=lambda x: x[1] - x[0])
                    sl[a:b + 1, e, 0] += np.linspace(0, 0.05, b - a + 1)
            check(slip_max(names, sl, flags) > SLIP_MAX, tag + " control: sliding a planted wrist 5 cm fails the slip check")
            qq = m["pose"][:, :, 3:7].copy()
            bidx = names.index("RightForeArm")
            qq[cut // 2:, bidx] = F.qnormalize(F.qmul(F.q_axis_angle((0, 0, 1), math.radians(40)), qq[cut // 2:, bidx]))
            check(step_max_deg(qq) > STEP_MAX_DEG, tag + " control: a 40 deg pop fails the step check")
        check(text.isascii() and chr(13) not in text, body + " file is ASCII / LF only")
        # fixtures for the C++ classifier tests
        fx = os.path.join(fixture_dir, "getup_frame0_%s.json" % body)
        if os.path.isfile(fx):
            f0 = json.load(open(fx, encoding="utf-8"))
            check(sorted(f0["clips"]) == ["B", "F", "L", "R"] and all(len(f0["clips"][c]["frame0_model"]) == 73 for c in f0["clips"]),
                  body + " fixture has 4 clips x 73 joints")
            want = {"B": (">", "front_z"), "F": ("<", "front_z"), "L": ("<", "lateral_z"), "R": (">", "lateral_z")}
            for c, (op, key) in want.items():
                v = f0["clips"][c]["features"][key]
                check((v > 0.5) if op == ">" else (v < -0.5), "%s %s fixture label feature %s=%.2f %s 0" % (body, c, key, v, op))
        else:
            print("SKIP %s fixture check (not found)" % body)


def rebake_test(root):
    """Bake into a temporary root and compare byte for byte with the files under test (determinism + staleness)."""
    import shutil
    import tempfile
    tmp = tempfile.mkdtemp(prefix="getupbake_")
    for body in ("woman", "man"):
        if not os.path.isfile(os.path.join(root, B.SRC_DIR % body, "M_ragdoll_getup_stand_B.utppclip.json")):
            print("SKIP rebake %s: sources missing" % body)
            continue
        if not GM.available(GM.default_mesh_root(root), body):
            print("SKIP rebake %s: the bake needs the body mesh exports under %s" % (body, GM.default_mesh_root(root)))
            continue
        B.bake_body(root, body, False, tmp, out_root=tmp)
        for rel in ["utpp_getup_%s.utpptrav" % body, "utpp_getup_%s.contacts.json" % body]:
            a = open(os.path.join(tmp, B.DATA_DIR, rel), "rb").read()
            b = open(os.path.join(root, B.DATA_DIR, rel), "rb").read()
            check(a == b, "rebake %s is byte-identical to the file under test" % rel)
        for c in B.CLIPS:
            rel = os.path.join(B.OUT_DIR % body, "M_ragdoll_getup_stand_%s.utppclip.json" % c)
            check(open(os.path.join(tmp, rel), "rb").read() == open(os.path.join(root, rel), "rb").read(), "rebake %s %s fixed clip identical" % (body, c))
    shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rebake", action="store_true", help="also bake into a temp folder and compare bytes (slow, ~1 min)")
    ap.add_argument("--root")
    ap.add_argument("--fixture-dir", default=os.path.join(B.repo_root(), "plugin", "UTPP", "tests", "data"))
    a = ap.parse_args()
    root = B.project_root(a.root)
    unit_tests()
    data_tests(root, a.fixture_dir)
    if a.rebake:
        rebake_test(root)
    print("PASS %d FAIL %d SKIP %d" % (PASS, FAIL, SKIP))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
