// Unit test of mm/OrientWarp.hpp (opt-in orientation warp of the v2 pose, CET mm.orientWarp). Synthetic rig (Root, Hips, Spine ..
// Spine3, shoulders, up legs) and, with a database argument, REAL run-clip poses of the v2 database:
//   utpp_orientwarp_test [<utpp_motion_woman.utppmm>]
// Covered: yaw convention of the pelvis / shoulder lines (forward kinematics), straight running / idle / OFF / every gate = exactly 0
// (pose bytes untouched), left / right circle convergence and exact mirror symmetry, the targets (pelvis on the path, shoulders
// 'into the turn'), rate limits and the >= 0.15 s fade, twist bound between pelvis and Spine3, root yaw bound, far-off poses (backward
// / lateral) left alone, spine twist distribution (shoulder line turns by tau, pelvis line, legs and all other bones bit-identical),
// NaN / invalid input, a skeleton without the required bones, and (database) the same on the real FL / FR / F / F_L_20 / F_R_20 loops
// with the offsets measured in game (actor lag 22 deg, visual root yaw -19 / +19 deg).
#define _CRT_SECURE_NO_WARNINGS 1
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "../src/mm/MotionDatabase.hpp"
#include "../src/mm/OrientWarp.hpp"

namespace ow = utpp::mm::orientwarp;

namespace
{
int g_fail = 0;
int g_checks = 0;
void Check(bool aOk, const char* aWhat, double aA = 0.0, double aB = 0.0)
{
    ++g_checks;
    if (!aOk)
    {
        ++g_fail;
        std::printf("FAIL: %s (%.4f, %.4f)\n", aWhat, aA, aB);
    }
}

constexpr float kDt = 1.0f / 60.0f;
constexpr float kPi = 3.14159265358979323846f;

// ---- synthetic rig
enum Bone { Root, Hips, Spine, Spine1, Spine2, Spine3, LShoulder, RShoulder, LUpLeg, RUpLeg, Neck, Head, NumBones };
const char* kNames[NumBones] = {"Root", "Hips", "Spine", "Spine1", "Spine2", "Spine3", "LeftShoulder", "RightShoulder", "LeftUpLeg", "RightUpLeg", "Neck", "Head"};
const int kParent[NumBones] = {-1, 0, 1, 2, 3, 4, 5, 5, 1, 1, 5, 10};

ow::Skeleton MakeSkeleton()
{
    return ow::Resolve(NumBones, [](uint32_t i) { return std::string_view(kNames[i]); }, [](uint32_t i) { return kParent[i]; });
}

void SetQ(float* aPose, int aBone, float aYawDeg)
{
    const float h = 0.5f * aYawDeg * ow::kRad;
    float* q = aPose + aBone * 7 + 3;
    q[0] = 0.0f; q[1] = 0.0f; q[2] = std::sin(h); q[3] = std::cos(h);
}
void SetP(float* aPose, int aBone, float x, float y, float z)
{
    float* p = aPose + aBone * 7;
    p[0] = x; p[1] = y; p[2] = z;
}
// pelvis yaw / spine yaw: the Hips rotated by aPelvis, the shoulder line (Spine3) by aPelvis + aTwist (twist spread over Spine .. Spine3)
void BuildPose(float* aPose, float aPelvisYaw, float aTwist)
{
    for (int b = 0; b < NumBones; ++b)
    {
        SetP(aPose, b, 0, 0, 0);
        SetQ(aPose, b, 0.0f);
    }
    SetP(aPose, Hips, 0, 0, 1.0f);
    SetQ(aPose, Hips, aPelvisYaw);
    SetP(aPose, Spine, 0, 0, 0.1f);
    SetP(aPose, Spine1, 0, 0, 0.1f);
    SetP(aPose, Spine2, 0, 0, 0.1f);
    SetP(aPose, Spine3, 0, 0, 0.1f);
    SetQ(aPose, Spine, aTwist * 0.2f);
    SetQ(aPose, Spine1, aTwist * 0.2f);
    SetQ(aPose, Spine2, aTwist * 0.2f);
    SetQ(aPose, Spine3, aTwist * 0.4f);
    SetP(aPose, LShoulder, -0.1f, 0, 0.1f);
    SetP(aPose, RShoulder, 0.1f, 0, 0.1f);
    SetP(aPose, LUpLeg, -0.1f, 0, -0.05f);
    SetP(aPose, RUpLeg, 0.1f, 0, -0.05f);
    SetP(aPose, Neck, 0, 0, 0.1f);
    SetP(aPose, Head, 0, 0, 0.1f);
}
float PelvisYaw(const ow::Skeleton& sk, const float* aPose, uint32_t n)
{
    float y = 0;
    ow::OrientWarp::LineYaw(sk, aPose, n, 0, 1, y);
    return y;
}
float ShoulderYaw(const ow::Skeleton& sk, const float* aPose, uint32_t n)
{
    float y = 0;
    ow::OrientWarp::LineYaw(sk, aPose, n, 2, 3, y);
    return y;
}

struct Run
{
    float dpMax = 0, tauMax = 0, dpStepMax = 0, tauStepMax = 0;
    float dp = 0, tau = 0, w = 0, pelvisAfter = 0, shouldersAfter = 0; // after: vs the path, with the correction applied
};

struct Scenario
{
    float rate = 170.0f;       // heading rate, deg/s (+ left)
    float speed = 3.85f;
    float pathRel = 22.0f;     // travel direction vs actor forward (+ left)
    float pelvisVsPath = -14.0f, shouldersVsPath = -33.0f;
    float sway = 15.0f;        // shoulder twist sway amplitude, deg at 1.5 Hz
    float rootYaw = 0.0f;
    float duration = 4.0f;
    bool aiming = false, weaponBusy = false, air = false, crouch = false, blocked = false;
    float gateAfter = -1.0f;   // time from which the gate flags below apply (negative: from the start)
};

Run Simulate(const Scenario& sc, const ow::Params& P, const ow::Skeleton& sk)
{
    ow::OrientWarp w;
    Run r;
    float heading = 30.0f;
    float pose[NumBones * 7], base[NumBones * 7];
    float prevDp = 0, prevTau = 0;
    for (float t = 0; t < sc.duration; t += kDt)
    {
        heading += sc.rate * kDt;
        ow::Signals s;
        s.vx = sc.speed * std::cos(heading * kPi / 180.0f);
        s.vy = sc.speed * std::sin(heading * kPi / 180.0f);
        s.yawDeg = heading - 90.0f - sc.pathRel;
        s.yawValid = true;
        const bool gates = sc.gateAfter < 0.0f || t >= sc.gateAfter;
        s.aiming = gates && sc.aiming; s.weaponBusy = gates && sc.weaponBusy; s.air = gates && sc.air; s.crouch = gates && sc.crouch;
        s.blocked = gates && sc.blocked;
        w.Step(P, s, kDt);
        // pose of the clip in the actor model space: pelvis / shoulders vs the path with the visual root yaw already inside
        const float pelvis = sc.pathRel + sc.pelvisVsPath - sc.rootYaw;
        const float twist = (sc.shouldersVsPath - sc.pelvisVsPath) + sc.sway * std::sin(2.0f * kPi * 1.5f * t);
        BuildPose(base, pelvis, twist);
        std::memcpy(pose, base, sizeof pose);
        const float dp = w.Measure(P, sk, pose, NumBones, sc.rootYaw, kDt);
        w.Apply(P, sk, pose, NumBones);
        r.dpMax = std::fmax(r.dpMax, std::fabs(dp));
        r.tauMax = std::fmax(r.tauMax, std::fabs(w.TorsoDeg()));
        r.dpStepMax = std::fmax(r.dpStepMax, std::fabs(dp - prevDp));
        r.tauStepMax = std::fmax(r.tauStepMax, std::fabs(w.TorsoDeg() - prevTau));
        prevDp = dp;
        prevTau = w.TorsoDeg();
        r.dp = dp;
        r.tau = w.TorsoDeg();
        r.w = w.Weight();
        // the final pose: pelvis line + extra root yaw, shoulders line + extra root yaw (sway removed for the comparison)
        r.pelvisAfter = ow::Wrap(PelvisYaw(sk, pose, NumBones) + sc.rootYaw + dp - sc.pathRel);
        r.shouldersAfter = ow::Wrap(ShoulderYaw(sk, pose, NumBones) + sc.rootYaw + dp - sc.pathRel);
    }
    return r;
}
} // namespace

int main(int argc, char** argv)
{
    const ow::Skeleton sk = MakeSkeleton();
    Check(sk.ok, "synthetic skeleton resolves");
    const ow::Params P;

    // 1) kinematics / convention
    {
        float pose[NumBones * 7];
        BuildPose(pose, 0.0f, 0.0f);
        Check(std::fabs(PelvisYaw(sk, pose, NumBones)) < 1e-3f && std::fabs(ShoulderYaw(sk, pose, NumBones)) < 1e-3f, "forward pose: pelvis and shoulder yaw 0");
        BuildPose(pose, 30.0f, 0.0f);
        Check(std::fabs(PelvisYaw(sk, pose, NumBones) - 30.0f) < 1e-3f && std::fabs(ShoulderYaw(sk, pose, NumBones) - 30.0f) < 1e-3f,
              "Hips +30 (left): pelvis and shoulders +30", PelvisYaw(sk, pose, NumBones), ShoulderYaw(sk, pose, NumBones));
        BuildPose(pose, 10.0f, -25.0f);
        Check(std::fabs(PelvisYaw(sk, pose, NumBones) - 10.0f) < 1e-3f && std::fabs(ShoulderYaw(sk, pose, NumBones) + 15.0f) < 1e-3f,
              "spine twist -25: shoulders pelvis-25", ShoulderYaw(sk, pose, NumBones), 0.0);
    }

    // 2) circles: convergence, targets, mirror symmetry
    {
        Scenario L;
        L.sway = 0.0f; // the sway is not mirrored by the scenario: compare the clean bias
        const Run l = Simulate(L, P, sk);
        Scenario Rr = L;
        Rr.rate = -L.rate; Rr.pathRel = -L.pathRel; Rr.pelvisVsPath = -L.pelvisVsPath; Rr.shouldersVsPath = -L.shouldersVsPath;
        const Run r = Simulate(Rr, P, sk);
        std::printf("left circle:  dp %+.2f tau %+.2f w %.2f -> pelvis %+.2f shoulders %+.2f vs path\n", static_cast<double>(l.dp), static_cast<double>(l.tau),
                    static_cast<double>(l.w), static_cast<double>(l.pelvisAfter), static_cast<double>(l.shouldersAfter));
        std::printf("right circle: dp %+.2f tau %+.2f w %.2f -> pelvis %+.2f shoulders %+.2f vs path\n", static_cast<double>(r.dp), static_cast<double>(r.tau),
                    static_cast<double>(r.w), static_cast<double>(r.pelvisAfter), static_cast<double>(r.shouldersAfter));
        Check(l.w > 0.99f && r.w > 0.99f, "full weight in a 170 deg/s circle", l.w, r.w);
        Check(l.dp > 5.0f && r.dp < -5.0f, "pelvis turns toward the path (left +, right -)", l.dp, r.dp);
        Check(std::fabs(l.pelvisAfter - P.pelvisIntoDeg) < P.deadbandDeg + 1.5f, "left: pelvis on the path (target into 5)", l.pelvisAfter, P.pelvisIntoDeg);
        Check(std::fabs(r.pelvisAfter + P.pelvisIntoDeg) < P.deadbandDeg + 1.5f, "right: pelvis on the path (target into 5)", r.pelvisAfter, -P.pelvisIntoDeg);
        Check(std::fabs(l.shouldersAfter - P.shoulderIntoDeg) < P.deadbandDeg + 4.5f, "left: shoulders into the turn (sway removed on average)", l.shouldersAfter, P.shoulderIntoDeg);
        Check(std::fabs(r.shouldersAfter + P.shoulderIntoDeg) < P.deadbandDeg + 4.5f, "right: shoulders into the turn", r.shouldersAfter, -P.shoulderIntoDeg);
        Check(std::fabs(l.dp + r.dp) < 0.05f && std::fabs(l.tau + r.tau) < 0.05f, "mirror symmetry of the correction", l.dp + r.dp, l.tau + r.tau);
        Check(std::fabs(l.dp) <= P.maxPelvisDeg + 1e-3f && std::fabs(l.tau) <= P.maxTorsoDeg + 1e-3f, "bounds hold", l.dp, l.tau);
        Check(l.dpStepMax <= P.maxPelvisRate * kDt + 1e-3f && l.tauStepMax <= P.maxTorsoRate * kDt + 1e-3f,
              "rate limits hold (no pop)", l.dpStepMax, l.tauStepMax);
    }

    // 3) straight running, idle, low speed, OFF (blocked), every gate: exactly 0 and the pose bytes untouched
    {
        auto zero = [&](const char* what, Scenario sc, bool aWeightZero = true, float aWarm = 0.0f) {
            ow::OrientWarp w;
            float pose[NumBones * 7], ref[NumBones * 7];
            bool allZero = true, same = true;
            float heading = 30.0f;
            for (float t = 0; t < 3.0f; t += kDt)
            {
                heading += sc.rate * kDt;
                ow::Signals s;
                s.vx = sc.speed * std::cos(heading * kPi / 180.0f);
                s.vy = sc.speed * std::sin(heading * kPi / 180.0f);
                s.yawDeg = heading - 90.0f - sc.pathRel;
                s.yawValid = true;
                s.aiming = sc.aiming; s.weaponBusy = sc.weaponBusy; s.air = sc.air; s.crouch = sc.crouch; s.blocked = sc.blocked;
                w.Step(P, s, kDt);
                BuildPose(ref, sc.pathRel + sc.pelvisVsPath, sc.shouldersVsPath - sc.pelvisVsPath);
                std::memcpy(pose, ref, sizeof pose);
                const float dp = w.Measure(P, sk, pose, NumBones, 0.0f, kDt);
                const float tw = w.Apply(P, sk, pose, NumBones);
                if (t < aWarm)
                    continue;
                allZero = allZero && dp == 0.0f && w.TorsoDeg() == 0.0f && tw == 0.0f && (!aWeightZero || (w.Weight() == 0.0f && w.Idle()));
                same = same && std::memcmp(pose, ref, sizeof pose) == 0;
            }
            Check(allZero, what);
            Check(same, what, 0, 1);
        };
        Scenario base;
        Scenario straight = base; straight.rate = 0.0f;
        zero("straight running: weight 0, outputs exactly 0", straight);
        Scenario gentle = base; gentle.rate = 20.0f;
        zero("turn rate below the ramp (20 deg/s): weight 0", gentle);
        Scenario slow = base; slow.speed = 0.6f;
        zero("low speed 0.6 m/s: weight 0", slow);
        Scenario idle = base; idle.speed = 0.0f; idle.rate = 0.0f;
        zero("idle: weight 0", idle);
        Scenario spin = base; spin.rate = 500.0f;
        zero("pirouette 500 deg/s (after the rate has settled): weight 0", spin, true, 1.5f);
        Scenario a = base; a.aiming = true;
        zero("aiming: weight 0", a);
        Scenario wb = base; wb.weaponBusy = true;
        zero("weapon action / drawn weapon: weight 0", wb);
        Scenario ai = base; ai.air = true;
        zero("air: weight 0", ai);
        Scenario cr = base; cr.crouch = true;
        zero("crouch / slide: weight 0", cr);
        Scenario bl = base; bl.blocked = true;
        zero("blocked (OFF, traversal, release, clip class): weight 0", bl);
        Scenario back = base; back.pelvisVsPath = 150.0f; back.shouldersVsPath = 160.0f;
        zero("far off the path (backward): correction 0", back, false);
        Scenario lateral = base; lateral.pelvisVsPath = 100.0f; lateral.shouldersVsPath = 100.0f;
        zero("far off the path (lateral loop): correction 0", lateral, false);
    }

    // 4) gates mid-circle: the correction fades out at the limited rate and ends exactly 0
    {
        for (int g = 0; g < 5; ++g)
        {
            Scenario sc;
            sc.duration = 5.0f; sc.gateAfter = 3.0f;
            sc.aiming = g == 0; sc.weaponBusy = g == 1; sc.air = g == 2; sc.crouch = g == 3; sc.blocked = g == 4;
            const Run r = Simulate(sc, P, sk);
            Check(r.dp == 0.0f && r.tau == 0.0f && r.w == 0.0f, "gate closed mid-circle: correction returns to exactly 0", r.dp, r.tau);
            Check(r.dpStepMax <= P.maxPelvisRate * kDt + 1e-3f && r.tauStepMax <= P.maxTorsoRate * kDt + 1e-3f, "gate fade: no pop", r.dpStepMax, r.tauStepMax);
        }
    }

    // 5) fade >= 0.15 s even with tiny fade parameters
    {
        ow::Params q = P;
        q.fadeIn = 0.01f; q.fadeOut = 0.01f;
        ow::OrientWarp w;
        float heading = 0;
        float firstFull = -1;
        for (float t = 0; t < 1.0f; t += kDt)
        {
            heading += 170.0f * kDt;
            ow::Signals s;
            s.vx = 3.85f * std::cos(heading * kPi / 180.0f); s.vy = 3.85f * std::sin(heading * kPi / 180.0f);
            s.yawDeg = heading - 90.0f - 22.0f; s.yawValid = true;
            w.Step(q, s, kDt);
            if (firstFull < 0 && w.Weight() >= 0.999f)
                firstFull = t;
        }
        // the heading rate itself needs ~0.1 s to settle (smoothing), the fade is on top of that; the weight must not jump
        Check(firstFull >= 0.15f - kDt, "weight needs >= 0.15 s to reach 1 (fade floor)", firstFull);
    }

    // 6) twist bound between pelvis and Spine3, root yaw bound
    {
        ow::Params q = P;
        q.shoulderIntoDeg = 80.0f; q.maxTorsoDeg = 90.0f; // an absurd demand: the twist bound alone has to hold it back
        Scenario sc;
        sc.pelvisVsPath = -5.0f; sc.shouldersVsPath = 40.0f; sc.sway = 0.0f; // twist0 = +45: only 5 deg more allowed
        const Run r = Simulate(sc, q, sk);
        const float twistAfter = ow::Wrap(r.shouldersAfter - r.pelvisAfter);
        Check(twistAfter <= q.maxTwistDeg + 0.6f && twistAfter >= 45.0f, "twist pelvis-Spine3 stops at maxTwist", twistAfter, q.maxTwistDeg);
        Scenario rb;
        rb.rootYaw = 40.0f; rb.pelvisVsPath = -40.0f; rb.shouldersVsPath = -40.0f; rb.sway = 0.0f;
        const Run b = Simulate(rb, P, sk);
        Check(b.dpMax <= P.maxRootYawDeg - 40.0f + 0.05f, "visual root yaw + extra never beyond maxRootYaw", b.dpMax, P.maxRootYawDeg - 40.0f);
        Scenario neg;
        neg.rootYaw = -19.0f; // in-game: the visual root is rotated to the outside (-19) in a left circle
        const Run n = Simulate(neg, P, sk);
        Check(n.dp > 0.0f && std::fabs(n.pelvisAfter - P.pelvisIntoDeg) < P.deadbandDeg + 1.5f, "root yaw -19 (outside): extra yaw positive, pelvis on the path", n.dp, n.pelvisAfter);
    }

    // 7) Apply: only Spine .. Spine3 change, the shoulder line turns by tau
    {
        float pose[NumBones * 7], ref[NumBones * 7];
        BuildPose(ref, 5.0f, -12.0f);
        std::memcpy(pose, ref, sizeof pose);
        // drive tau to a known value: a circle that wants a clearly nonzero twist
        Scenario sc; sc.sway = 0.0f;
        ow::OrientWarp w;
        float heading = 0;
        for (float t = 0; t < 4.0f; t += kDt)
        {
            heading += 170.0f * kDt;
            ow::Signals s;
            s.vx = 3.85f * std::cos(heading * kPi / 180.0f); s.vy = 3.85f * std::sin(heading * kPi / 180.0f);
            s.yawDeg = heading - 90.0f - 22.0f; s.yawValid = true;
            w.Step(P, s, kDt);
            BuildPose(pose, 22.0f - 14.0f, -19.0f);
            w.Measure(P, sk, pose, NumBones, 0.0f, kDt);
        }
        BuildPose(ref, 8.0f, -19.0f);
        std::memcpy(pose, ref, sizeof pose);
        const float tau = w.TorsoDeg();
        Check(std::fabs(tau) > 5.0f, "known nonzero tau", tau);
        const float applied = w.Apply(P, sk, pose, NumBones);
        Check(applied == tau, "Apply returns tau", applied, tau);
        Check(std::fabs(ow::Wrap(ShoulderYaw(sk, pose, NumBones) - ShoulderYaw(sk, ref, NumBones) - tau)) < 0.02f, "shoulder line turns by tau",
              ShoulderYaw(sk, pose, NumBones) - ShoulderYaw(sk, ref, NumBones), tau);
        Check(std::fabs(PelvisYaw(sk, pose, NumBones) - PelvisYaw(sk, ref, NumBones)) < 1e-4f, "pelvis line unchanged");
        bool others = true;
        for (int b = 0; b < NumBones; ++b)
        {
            if (b == Spine || b == Spine1 || b == Spine2 || b == Spine3 || b == LUpLeg || b == RUpLeg)
                continue; // the spine takes the twist, the up legs the counter-rotation of the extra root yaw (below)
            others = others && std::memcmp(pose + b * 7, ref + b * 7, 7 * sizeof(float)) == 0;
        }
        Check(others, "Root, Hips, shoulders, neck, head: bytes unchanged");
        // the extra root yaw (dp, added by the host to the Hips) must leave the thighs where they were: model yaw of the up legs
        const float dp = w.RootDeg();
        Check(std::fabs(dp) > 5.0f, "known nonzero dp", dp);
        auto legYaw = [&](const float* ps, int bone, float extraHips) {
            const float* qh = ps + Hips * 7 + 3;
            const float* ql = ps + bone * 7 + 3;
            const float yh = 2.0f * std::atan2(qh[2], qh[3]) * ow::kDeg + extraHips, yl = 2.0f * std::atan2(ql[2], ql[3]) * ow::kDeg;
            return ow::Wrap(yh + yl);
        };
        Check(std::fabs(ow::Wrap(legYaw(pose, LUpLeg, dp) - legYaw(ref, LUpLeg, 0.0f))) < 0.02f &&
                  std::fabs(ow::Wrap(legYaw(pose, RUpLeg, dp) - legYaw(ref, RUpLeg, 0.0f))) < 0.02f,
              "up legs keep their model direction when Hips turns by dp (legHold 1)", legYaw(pose, LUpLeg, dp), legYaw(ref, LUpLeg, 0.0f));
        {
            // Hips position (Root frame): the host rotates it by dp about the origin, the correction turns it back
            float p3[NumBones * 7];
            std::memcpy(p3, ref, sizeof p3);
            SetP(p3, Hips, 0.06f, 0.03f, 1.0f);
            w.Apply(P, sk, p3, NumBones);
            const float c = std::cos(dp * ow::kRad), sn = std::sin(dp * ow::kRad);
            const float hx = c * p3[Hips * 7] - sn * p3[Hips * 7 + 1], hy = sn * p3[Hips * 7] + c * p3[Hips * 7 + 1];
            Check(std::fabs(hx - 0.06f) < 1e-4f && std::fabs(hy - 0.03f) < 1e-4f && p3[Hips * 7 + 2] == 1.0f,
                  "Hips position stays where it was after the host rotation by dp", hx, hy);
        }
        {
            ow::Params nh = P;
            nh.legHold = 0.0f;
            float p2[NumBones * 7];
            std::memcpy(p2, ref, sizeof p2);
            w.Apply(nh, sk, p2, NumBones);
            Check(std::memcmp(p2 + LUpLeg * 7, ref + LUpLeg * 7, 7 * sizeof(float)) == 0 && std::memcmp(p2 + RUpLeg * 7, ref + RUpLeg * 7, 7 * sizeof(float)) == 0,
                  "legHold 0: up legs untouched");
        }
        bool positions = true;
        for (int b = 0; b < NumBones; ++b)
            positions = positions && std::memcmp(pose + b * 7, ref + b * 7, 3 * sizeof(float)) == 0;
        Check(positions, "all positions unchanged");
        // unit quaternions
        bool unit = true;
        for (int b = 0; b < NumBones; ++b)
        {
            const float* q = pose + b * 7 + 3;
            unit = unit && std::fabs(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1.0f) < 1e-5f;
        }
        Check(unit, "unit quaternions");
        // distribution 0.2 / 0.2 / 0.2 / 0.4 of tau: the Spine local yaw changed by 0.2 tau
        float yawSpine = 0.0f;
        {
            const float* q = pose + Spine * 7 + 3;
            yawSpine = 2.0f * std::atan2(q[2], q[3]) * ow::kDeg;
        }
        Check(std::fabs(ow::Wrap(yawSpine - (-19.0f * 0.2f + tau * P.share[0]))) < 0.05f, "Spine local yaw carries share 0.2 of tau", yawSpine, tau * P.share[0]);
    }

    // 8) NaN / invalid input / unusable skeleton
    {
        ow::OrientWarp w;
        float pose[NumBones * 7];
        BuildPose(pose, 0.0f, 0.0f);
        ow::Signals s;
        s.vx = 3.0f; s.vy = 0.0f; s.yawValid = true;
        w.Step(P, s, std::numeric_limits<float>::quiet_NaN());
        w.Step(P, s, -1.0f);
        Check(w.Weight() == 0.0f, "NaN / negative dt: nothing advances");
        Check(w.Measure(P, sk, pose, NumBones, 0.0f, std::numeric_limits<float>::quiet_NaN()) == 0.0f, "Measure NaN dt: 0");
        Check(w.Measure(P, sk, nullptr, NumBones, 0.0f, kDt) == 0.0f, "Measure null pose: 0");
        Check(w.Measure(P, sk, pose, NumBones, std::numeric_limits<float>::quiet_NaN(), kDt) == 0.0f, "Measure NaN root yaw: 0");
        pose[Spine3 * 7 + 3] = std::numeric_limits<float>::quiet_NaN();
        Check(w.Measure(P, sk, pose, NumBones, 0.0f, kDt) == 0.0f, "Measure NaN quaternion: 0");
        ow::Signals nanS;
        nanS.vx = std::numeric_limits<float>::quiet_NaN(); nanS.vy = 1.0f; nanS.yawValid = true;
        w.Step(P, nanS, kDt);
        Check(std::isfinite(w.Weight()) && std::isfinite(w.RootDeg()) && std::isfinite(w.TorsoDeg()), "NaN velocity: outputs finite");
        Check(w.Apply(P, sk, nullptr, NumBones) == 0.0f, "Apply null pose: 0");
        ow::Skeleton bad;
        Check(!bad.ok, "default skeleton is not ok");
        const ow::Skeleton noLeg = ow::Resolve(NumBones, [](uint32_t i) { return std::string_view(i == LUpLeg ? "X" : kNames[i]); }, [](uint32_t i) { return kParent[i]; });
        Check(!noLeg.ok, "skeleton without LeftUpLeg is not ok");
        Check(w.Measure(P, noLeg, pose, NumBones, 0.0f, kDt) == 0.0f, "Measure with a missing-bone skeleton: 0");
    }

    // 8b) run pelvis yaw trim (2026-10-30, CET mm.pelvisYawTrim): ApplyAngles(root +t, torso -t) + the host's root yaw t turns only
    // the pelvis; PelvisTrim weight = ground forward locomotion, 0 for every gate, idle, strafe and backward motion
    {
        auto zyaw = [](const float* aPose, int aBone) { const float* q = aPose + aBone * 7 + 3; return 2.0f * std::atan2(q[2], q[3]) * ow::kDeg; };
        float base[NumBones * 7], pose[NumBones * 7];
        BuildPose(base, -4.7f, 4.9f); // Run_Loop_F means: pelvis -4.7, shoulders +0.2
        std::memcpy(pose, base, sizeof(base));
        const float t = 5.0f;
        Check(ow::OrientWarp::ApplyAngles(P, sk, pose, NumBones, t, -t) == -t, "trim: ApplyAngles returns the torso angle");
        const float pel = ow::Wrap(PelvisYaw(sk, pose, NumBones) + t), sh = ow::Wrap(ShoulderYaw(sk, pose, NumBones) + t);
        Check(std::fabs(pel - 0.3f) < 1e-3f, "trim +5: pelvis -4.7 -> +0.3 once the host adds the root yaw", pel);
        Check(std::fabs(sh - ShoulderYaw(sk, base, NumBones)) < 1e-3f, "trim: shoulders keep their direction", sh, ShoulderYaw(sk, base, NumBones));
        for (int leg : {LUpLeg, RUpLeg})
            Check(std::fabs(ow::Wrap(zyaw(pose, Hips) + zyaw(pose, leg) + t) - ow::Wrap(zyaw(base, Hips) + zyaw(base, leg))) < 1e-3f,
                  "trim: thighs keep their model direction (feet do not turn)");
        bool othersSame = true;
        for (int b : {Root, LShoulder, RShoulder, Neck, Head})
            othersSame = othersSame && std::memcmp(pose + b * 7, base + b * 7, 7 * sizeof(float)) == 0;
        Check(othersSame && std::memcmp(pose + Hips * 7 + 3, base + Hips * 7 + 3, 4 * sizeof(float)) == 0,
              "trim: Root, clavicles, Neck, Head and the Hips rotation bit-identical (the host turns the root)");
        std::memcpy(pose, base, sizeof(base));
        Check(ow::OrientWarp::ApplyAngles(P, sk, pose, NumBones, 0.0f, 0.0f) == 0.0f && std::memcmp(pose, base, sizeof(base)) == 0,
              "trim 0: pose bytes untouched");
        Check(ow::OrientWarp::ApplyAngles(P, sk, pose, NumBones, std::numeric_limits<float>::quiet_NaN(), 1.0f) == 0.0f &&
                  std::memcmp(pose, base, sizeof(base)) == 0,
              "trim NaN: pose bytes untouched");
        Check(ow::ValidPelvisTrim(0.0f) && ow::ValidPelvisTrim(-10.0f) && ow::ValidPelvisTrim(10.0f) && !ow::ValidPelvisTrim(10.5f) &&
                  !ow::ValidPelvisTrim(std::numeric_limits<float>::quiet_NaN()),
              "ValidPelvisTrim: -10..10, NaN rejected");

        const ow::TrimParams TP;
        auto run = [&](ow::PelvisTrim& aT, const ow::Signals& aS, float aSec) {
            float maxStep = 0.0f;
            for (float tt = 0; tt < aSec; tt += kDt)
            {
                const float before = aT.Weight();
                aT.Step(TP, aS, kDt);
                maxStep = std::fmax(maxStep, std::fabs(aT.Weight() - before));
            }
            return maxStep;
        };
        ow::Signals fwd;
        fwd.vx = 0.0f; fwd.vy = 3.85f; fwd.yawDeg = 0.0f; fwd.yawValid = true; // running along the actor forward
        ow::PelvisTrim tr;
        const float step = run(tr, fwd, 1.0f);
        Check(tr.Weight() == 1.0f && tr.Deg(5.0f) == 5.0f, "trim: forward running -> weight 1, Deg(5) = 5", tr.Weight());
        Check(step <= kDt / 0.15f + 1e-5f, "trim: fade-in rate limited", step);
        Check(tr.Deg(11.0f) == 0.0f && tr.Deg(std::numeric_limits<float>::quiet_NaN()) == 0.0f, "trim: invalid value -> 0");
        const char* names[] = {"aiming", "weapon", "air", "crouch", "blocked"};
        for (int g = 0; g < 5; ++g)
        {
            ow::PelvisTrim tg;
            run(tg, fwd, 1.0f);
            ow::Signals s = fwd;
            s.aiming = g == 0; s.weaponBusy = g == 1; s.air = g == 2; s.crouch = g == 3; s.blocked = g == 4;
            run(tg, s, 0.5f);
            Check(tg.Weight() == 0.0f && tg.Deg(5.0f) == 0.0f, (std::string("trim gate ") + names[g] + ": weight 0").c_str(), tg.Weight());
        }
        struct Dir { const char* what; float vx, vy; };
        const Dir dirs[] = {{"idle", 0.0f, 0.0f}, {"slow walk 0.5 m/s", 0.0f, 0.5f}, {"strafe right", 3.0f, 0.0f}, {"strafe left", -3.0f, 0.0f},
                            {"backward", 0.0f, -3.0f}};
        for (const Dir& d : dirs)
        {
            ow::PelvisTrim td;
            run(td, fwd, 1.0f);
            ow::Signals s = fwd;
            s.vx = d.vx; s.vy = d.vy;
            run(td, s, 1.0f);
            Check(td.Weight() == 0.0f, (std::string("trim: ") + d.what + " -> weight 0").c_str(), td.Weight());
        }
        ow::Signals diag = fwd;
        diag.vx = -3.85f * std::sin(20.0f * ow::kRad); diag.vy = 3.85f * std::cos(20.0f * ow::kRad); // 20 deg left of forward
        ow::PelvisTrim tdg;
        run(tdg, diag, 1.0f);
        Check(tdg.Weight() == 1.0f, "trim: 20 deg off forward still full weight", tdg.Weight());
    }

    // 8c) body yaw offset (2026-10-30, CET mm.bodyYawOffset): weight 1 on foot in every stance / gait (idle, walk, run, strafe,
    // backward, air, crouch), 0 while aiming, during a weapon action and blocked; 0.25 s linear fades, feed dt deterministic
    {
        Check(ow::ValidBodyYawOffset(0.0f) && ow::ValidBodyYawOffset(-20.0f) && ow::ValidBodyYawOffset(20.0f) &&
                  !ow::ValidBodyYawOffset(20.5f) && !ow::ValidBodyYawOffset(-20.5f) &&
                  !ow::ValidBodyYawOffset(std::numeric_limits<float>::quiet_NaN()) &&
                  !ow::ValidBodyYawOffset(std::numeric_limits<float>::infinity()),
              "ValidBodyYawOffset: -20..20, NaN / inf rejected");
        const ow::BodyOffsetParams BP;
        auto run = [&](ow::BodyYawOffset& aB, const ow::Signals& aS, float aSec) {
            float maxStep = 0.0f;
            for (float tt = 0; tt < aSec; tt += kDt)
            {
                const float before = aB.Weight();
                aB.Step(BP, aS, kDt);
                maxStep = std::fmax(maxStep, std::fabs(aB.Weight() - before));
            }
            return maxStep;
        };
        ow::BodyYawOffset fresh;
        Check(fresh.Weight() == 0.0f && fresh.Deg(8.0f) == 0.0f, "body offset: starts at weight 0 (nothing applied)");
        struct Case { const char* what; float vx, vy; bool air, crouch, yawValid; };
        const Case on[] = {{"idle", 0.0f, 0.0f, false, false, true},       {"walk", 0.0f, 1.5f, false, false, true},
                           {"run", 0.0f, 3.85f, false, false, true},       {"sprint", 0.0f, 6.5f, false, false, true},
                           {"strafe", 3.0f, 0.0f, false, false, true},     {"backward", 0.0f, -3.0f, false, false, true},
                           {"crouch", 0.0f, 2.0f, false, true, true},      {"jump / air", 0.0f, 3.0f, true, false, true},
                           {"no actor yaw", 0.0f, 3.0f, false, false, false}};
        for (const Case& c : on)
        {
            ow::Signals s;
            s.vx = c.vx; s.vy = c.vy; s.air = c.air; s.crouch = c.crouch; s.yawValid = c.yawValid;
            ow::BodyYawOffset b;
            const float step = run(b, s, 1.0f);
            Check(b.Weight() == 1.0f && b.Deg(8.0f) == 8.0f && b.Deg(-12.5f) == -12.5f,
                  (std::string("body offset ") + c.what + ": weight 1, Deg(8) = 8").c_str(), b.Weight());
            Check(step <= kDt / 0.25f + 1e-5f, (std::string("body offset ") + c.what + ": fade-in rate limited (0.25 s)").c_str(), step);
        }
        {
            ow::Signals s;
            ow::BodyYawOffset b;
            for (int i = 0; i < 7; ++i) // 7 feeds of 1/60 s
                b.Step(BP, s, kDt);
            Check(std::fabs(b.Weight() - 7.0f * kDt / 0.25f) < 1e-4f, "body offset: half-way through the fade-in after ~0.12 s", b.Weight());
        }
        const char* names[] = {"aiming", "weapon action", "blocked"};
        for (int g = 0; g < 3; ++g)
        {
            ow::Signals s;
            s.vy = 3.85f; s.yawValid = true;
            ow::BodyYawOffset b;
            run(b, s, 1.0f);
            s.aiming = g == 0; s.weaponBusy = g == 1; s.blocked = g == 2;
            const float step = run(b, s, 0.5f);
            Check(b.Weight() == 0.0f && b.Deg(8.0f) == 0.0f, (std::string("body offset gate ") + names[g] + ": weight 0").c_str(), b.Weight());
            Check(step <= kDt / 0.25f + 1e-5f, (std::string("body offset gate ") + names[g] + ": fade-out rate limited (0.25 s)").c_str(), step);
            s.aiming = s.weaponBusy = s.blocked = false;
            run(b, s, 1.0f);
            Check(b.Weight() == 1.0f, (std::string("body offset gate ") + names[g] + " released: weight back to 1").c_str(), b.Weight());
        }
        ow::BodyYawOffset bi;
        ow::Signals s;
        run(bi, s, 1.0f);
        Check(bi.Deg(21.0f) == 0.0f && bi.Deg(std::numeric_limits<float>::quiet_NaN()) == 0.0f && bi.Deg(0.0f) == 0.0f,
              "body offset: invalid / 0 value -> 0");
        const float w0 = bi.Weight();
        bi.Step(BP, s, 0.0f);
        bi.Step(BP, s, -1.0f);
        bi.Step(BP, s, std::numeric_limits<float>::quiet_NaN());
        Check(bi.Weight() == w0, "body offset: dt <= 0 / NaN -> no step");
        bi.Reset();
        Check(bi.Weight() == 0.0f, "body offset: Reset -> weight 0 (fades in again)");
        // determinism: the same feed sequence gives the same weights bit for bit
        ow::BodyYawOffset d1, d2;
        bool same = true;
        for (int i = 0; i < 200; ++i)
        {
            ow::Signals si;
            si.aiming = (i / 23) % 3 == 1;
            const float dt = (i % 7 == 0) ? 1.0f / 30.0f : kDt;
            d1.Step(BP, si, dt);
            d2.Step(BP, si, dt);
            same = same && std::memcmp(&d1, &d2, sizeof(d1)) == 0;
        }
        Check(same, "body offset: deterministic for the same feed sequence");
    }

    // 9) real database poses
    if (argc >= 2)
    {
        using namespace utpp::mm;
        motion::MotionDatabase db;
        std::string err;
        if (!db.LoadFromFile(argv[1], motion::kRigIdentityWoman, err))
        {
            std::printf("FAIL: database load: %s\n", err.c_str());
            return 1;
        }
        const uint32_t nb = db.NumBones();
        const ow::Skeleton dsk = ow::ResolveDb(db, nb);
        Check(dsk.ok, "runtime skeleton resolves on the database");
        auto clipIndex = [&](const char* aName) -> int {
            for (uint32_t c = 0; c < db.NumClips(); ++c)
                if (db.ClipName(c) == aName)
                    return static_cast<int>(c);
            return -1;
        };
        auto dbPose = [&](uint32_t aFrame, float* aOut) {
            const motion::PoseRecord* p = db.Pose(aFrame);
            for (uint32_t i = 0; i < nb; ++i)
            {
                float* d = aOut + size_t(i) * 7;
                d[0] = p[i].px; d[1] = p[i].py; d[2] = p[i].pz;
                d[3] = p[i].qx; d[4] = p[i].qy; d[5] = p[i].qz; d[6] = p[i].qw;
            }
        };
        struct Case { const char* clip; float rate, pathRel, rootYaw; };
        // measured in game: actor lag 22 deg, visual root yaw -19 (left) / +19 (right)
        const Case cases[] = {{"M_Neutral_Run_Loop_FL", 170.0f, 22.0f, -19.0f},   {"M_Neutral_Run_Loop_F", 170.0f, 22.0f, -19.0f},
                              {"M_Neutral_Run_Loop_F_L_20", 170.0f, 22.0f, -19.0f}, {"M_Neutral_Run_Loop_FR", -170.0f, -22.0f, 19.0f},
                              {"M_Neutral_Run_Loop_F", -170.0f, -22.0f, 19.0f},   {"M_Neutral_Run_Loop_F_R_20", -170.0f, -22.0f, 19.0f}};
        std::vector<float> pose(size_t(nb) * 7), warped(size_t(nb) * 7);
        for (const Case& cs : cases)
        {
            const int c = clipIndex(cs.clip);
            Check(c >= 0, cs.clip);
            if (c < 0)
                continue;
            const auto& cl = db.Clip(static_cast<uint32_t>(c));
            // the clip's mean pelvis / shoulder yaw over the cycle (what the filters see) and the corrected result per frame phase
            double sumP = 0, sumS = 0, sumPa = 0, sumSa = 0, maxTwist = 0;
            int cnt = 0;
            ow::OrientWarp w;
            float heading = 0.0f;
            const uint32_t frames = cl.numFrames;
            double pos = 0.0;
            for (float t = 0; t < 6.0f; t += kDt, pos += static_cast<double>(db.Header().fps) * kDt)
            {
                const uint32_t k = static_cast<uint32_t>(pos) % frames;
                heading += cs.rate * kDt;
                ow::Signals s;
                s.vx = 3.85f * std::cos(heading * kPi / 180.0f); s.vy = 3.85f * std::sin(heading * kPi / 180.0f);
                s.yawDeg = heading - 90.0f - cs.pathRel; s.yawValid = true;
                w.Step(P, s, kDt);
                dbPose(cl.firstFrame + k, pose.data());
                std::memcpy(warped.data(), pose.data(), pose.size() * sizeof(float));
                const float dp = w.Measure(P, dsk, warped.data(), nb, cs.rootYaw, kDt);
                w.Apply(P, dsk, warped.data(), nb);
                if (t > 4.0f)
                {
                    float py = 0, sy = 0, pa = 0, sa = 0;
                    ow::OrientWarp::LineYaw(dsk, pose.data(), nb, 0, 1, py);
                    ow::OrientWarp::LineYaw(dsk, pose.data(), nb, 2, 3, sy);
                    ow::OrientWarp::LineYaw(dsk, warped.data(), nb, 0, 1, pa);
                    ow::OrientWarp::LineYaw(dsk, warped.data(), nb, 2, 3, sa);
                    sumP += ow::Wrap(py + cs.rootYaw - cs.pathRel);
                    sumS += ow::Wrap(sy + cs.rootYaw - cs.pathRel);
                    sumPa += ow::Wrap(pa + cs.rootYaw + dp - cs.pathRel);
                    sumSa += ow::Wrap(sa + cs.rootYaw + dp - cs.pathRel);
                    maxTwist = std::fmax(maxTwist, std::fabs(ow::Wrap(sa - pa)));
                    ++cnt;
                }
            }
            const double sg = cs.rate > 0 ? 1.0 : -1.0;
            std::printf("%-26s %s: pelvis vs path %+.1f -> %+.1f, shoulders vs path %+.1f -> %+.1f (+ left), dp %+.1f tau %+.1f, max twist %.1f\n", cs.clip,
                        sg > 0 ? "left " : "right", sumP / cnt, sumPa / cnt, sumS / cnt, sumSa / cnt, static_cast<double>(w.RootDeg()),
                        static_cast<double>(w.TorsoDeg()), maxTwist);
            if (std::fabs(sumP / cnt) < P.maxPelvisDeg) // within the reach of the bound; the plain F loop (pelvis 40 deg off) is only improved
                Check(std::fabs(sumPa / cnt - sg * P.pelvisIntoDeg) < 5.0, "real loop: pelvis on the path (into the turn target)", sumPa / cnt, sg * P.pelvisIntoDeg);
            Check(std::fabs(sumPa / cnt) < std::fabs(sumP / cnt) - 5.0 || std::fabs(sumP / cnt) < 8.0, "real loop: pelvis closer to the path than before", sumPa / cnt, sumP / cnt);
            Check(sg * (sumSa / cnt) > -3.0 && sg * (sumSa / cnt) < P.shoulderIntoDeg + 8.0, "real loop: shoulders along the path / slightly into the turn", sumSa / cnt, sg);
            Check(std::fabs(sumSa / cnt) < std::fabs(sumS / cnt) - 5.0, "real loop: shoulders clearly closer to the path than before", sumSa / cnt, sumS / cnt);
            Check(maxTwist <= P.maxTwistDeg + 25.0, "real loop: twist stays anatomical (sway included)", maxTwist, P.maxTwistDeg);
        }
    }
    std::printf(g_fail ? "OrientWarpTest FAILED (%d of %d)\n" : "OrientWarpTest OK (0 failures of %d checks)\n", g_fail ? g_fail : g_checks, g_checks);
    return g_fail ? 1 : 0;
}
