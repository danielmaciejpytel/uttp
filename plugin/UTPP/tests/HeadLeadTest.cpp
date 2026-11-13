// Unit test of mm/HeadLead.hpp (opt-in runtime head lead): sign, symmetry, bias removal, fades, gates, NaN safety, rate limits,
// untouched bones. Header only, no database.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "../src/mm/HeadLead.hpp"

using namespace utpp::mm::headlead;

namespace
{
int g_fail = 0;
void Check(bool aOk, const char* aWhat, double aA = 0.0, double aB = 0.0)
{
    if (!aOk)
    {
        ++g_fail;
        std::printf("FAIL: %s (%.4f, %.4f)\n", aWhat, aA, aB);
    }
}

const char* kNames[9] = {"Root", "Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck", "Neck1", "Head"};
constexpr float kDt = 1.0f / 60.0f;

Quat AxisZ(float aDeg)
{
    const float h = 0.5f * aDeg * kRad;
    return {0.0f, 0.0f, std::sin(h), std::cos(h)};
}
// bone-local x -> model Z (up), local y -> model X, local z -> model Y (forward): a cyclic permutation, det +1
Quat R0()
{
    // matrix columns (0,0,1), (1,0,0), (0,1,0): quaternion of the rotation by 120 deg about (1,1,1)/sqrt3
    const float s = std::sin(60.0f * kRad) / std::sqrt(3.0f), c = std::cos(60.0f * kRad);
    return {s, s, s, c};
}

// Pose with model orientations Rz(yaw[i]) * R0 and no translation; bones: Root Hips Spine Spine1 Spine2 Spine3 Neck Neck1 Head.
struct Rig
{
    float pose[9 * 7] = {};
    void Set(const float aYaw[9])
    {
        Quat parentModel;
        for (int i = 0; i < 9; ++i)
        {
            const Quat model = Mul(AxisZ(aYaw[i]), R0());
            const Quat local = i == 0 ? model : Mul(Conj(parentModel), model);
            float* q = pose + i * 7;
            q[0] = 0.1f * i; q[1] = 0.2f; q[2] = 0.3f; // positions: must never change
            q[3] = local.x; q[4] = local.y; q[5] = local.z; q[6] = local.w;
            parentModel = model;
        }
    }
    float HeadVsSpine3() const // same measurement as the production code, from independent FK
    {
        Quat run, model[9];
        for (int i = 0; i < 9; ++i)
        {
            const float* q = pose + i * 7 + 3;
            const Quat l{q[0], q[1], q[2], q[3]};
            run = i == 0 ? l : Mul(run, l);
            model[i] = run;
        }
        return Wrap(AxisYaw(model[8]) - AxisYaw(model[5]));
    }
};

Skeleton Sk()
{
    return Resolve(9, [](uint32_t i) { return std::string_view(kNames[i]); },
                   [](uint32_t i) { return i == 0 ? -1 : static_cast<int>(i) - 1; });
}

// V runs a circle of the given signed heading rate (deg/s, + = left) at 3.8 m/s
void Circle(HeadLead& aH, const Params& aP, float aRate, float aSeconds, Signals aS = {})
{
    static float heading = 0.0f;
    for (float t = 0; t < aSeconds; t += kDt)
    {
        heading += aRate * kDt;
        aS.vx = 3.8f * std::cos(heading * kRad);
        aS.vy = 3.8f * std::sin(heading * kRad);
        aH.Step(aP, aS, kDt);
    }
}

// Applies the correction to a pose whose head sits at aYawVsSpine3 (relative to the idle bias) for aSeconds, returns the result.
float Settle(HeadLead& aH, const Params& aP, const Skeleton& aSk, float aHeadVsIdle, float aSeconds, Signals aS = {}, float aRate = 0.0f,
             float* aMaxStep = nullptr)
{
    // spine3 yaw 30, neck / neck1 follow the spine, head at idle bias + requested offset: HeadVsSpine3 = restBias + offset
    float yaw[9] = {0, 0, 0, 0, 0, 30.0f, 30.0f, 30.0f, 30.0f + aP.restBiasDeg + aHeadVsIdle};
    Rig rig;
    float prev = 0.0f;
    float last = 0.0f;
    for (float t = 0; t < aSeconds; t += kDt)
    {
        if (aRate != 0.0f || aS.vx != 0.0f || aS.vy != 0.0f)
            Circle(aH, aP, aRate, kDt, aS);
        rig.Set(yaw);
        const float c = aH.Apply(aP, aSk, rig.pose, 9, kDt);
        if (aMaxStep)
            *aMaxStep = std::fmax(*aMaxStep, std::fabs(c - prev));
        prev = c;
        last = Wrap(rig.HeadVsSpine3() - aP.restBiasDeg);
    }
    return last;
}

// ---- path law (Signals.yawValid): the pose of the actor model space, AxisYaw(Head) == the direction the face looks (0 = forward, + left).
// local z -> model +X (rotation by +90 deg about Y), so AxisYaw(AxisZ(a) * R1) == a for every bone.
Quat R1()
{
    const float s = std::sin(45.0f * kRad), c = std::cos(45.0f * kRad);
    return {0.0f, s, 0.0f, c};
}
struct FaceRig
{
    float pose[9 * 7] = {};
    void Set(const float aFaceYaw[9])
    {
        Quat parentModel;
        for (int i = 0; i < 9; ++i)
        {
            const Quat model = Mul(AxisZ(aFaceYaw[i]), R1());
            const Quat local = i == 0 ? model : Mul(Conj(parentModel), model);
            float* q = pose + i * 7;
            q[0] = 0.1f * i; q[1] = 0.2f; q[2] = 0.3f;
            q[3] = local.x; q[4] = local.y; q[5] = local.z; q[6] = local.w;
            parentModel = model;
        }
    }
    float Yaw(int aBone) const
    {
        Quat run, model[9];
        for (int i = 0; i < 9; ++i)
        {
            const float* q = pose + i * 7 + 3;
            const Quat l{q[0], q[1], q[2], q[3]};
            run = i == 0 ? l : Mul(run, l);
            model[i] = run;
        }
        return AxisYaw(model[aBone]);
    }
};
// V runs a circle of the signed heading rate (+ left) while the actor yaw lags the velocity: the travel direction is aTravRel deg
// left (+) / right (-) of the actor forward (the live captures: +27..+39 in left circles, -31..-35 in right circles).
struct PathResult
{
    float headAbs = 0.0f;     // AxisYaw(Head) of the corrected pose (actor frame, 0 = forward)
    float headVsPath = 0.0f;  // headAbs - travel direction (0 = the face looks along the path, + = left of the path)
    float headVsSpine = 0.0f; // Wrap(head - Spine3 - twistBias) of the corrected pose (the quantity the twist window bounds)
    float corr = 0.0f, lead = 0.0f, deltaHead = 0.0f;
};
PathResult PathCircle(const Params& aP, float aRate, float aTravRel, float aSpine3, float aHead, float aSeconds, float aYawOffset = 0.0f)
{
    HeadLead H;
    const Skeleton skel = Sk();
    float heading = 0.0f;
    PathResult r;
    FaceRig base;
    const float face[9] = {0, 0, 0, 0, 0, aSpine3, aSpine3, aSpine3, aHead};
    base.Set(face);
    FaceRig rig;
    Signals s;
    s.yawValid = true;
    for (float t = 0; t < aSeconds; t += kDt)
    {
        heading += aRate * kDt;
        s.vx = 3.8f * std::cos(heading * kRad);
        s.vy = 3.8f * std::sin(heading * kRad);
        s.yawDeg = heading - 90.0f - aTravRel + aYawOffset;
        H.Step(aP, s, kDt);
        rig = base;
        H.Apply(aP, skel, rig.pose, 9, kDt);
    }
    r.headAbs = rig.Yaw(8);
    r.headVsPath = Wrap(r.headAbs - aTravRel); // the travel direction relative to the actor forward is aTravRel by construction
    r.headVsSpine = Wrap(rig.Yaw(8) - rig.Yaw(5) - aP.twistBiasDeg);
    r.corr = H.Corr();
    r.lead = H.Lead();
    r.deltaHead = Wrap(rig.Yaw(8) - aHead);
    return r;
}
} // namespace

int main()
{
    const Params P;
    const Skeleton sk = Sk();
    Check(sk.ok && sk.len == 9 && sk.head == 8 && sk.neck == 6, "resolve skeleton");
    {
        Skeleton bad = Resolve(3, [](uint32_t i) { return std::string_view(kNames[i]); }, [](uint32_t) { return -1; });
        Check(!bad.ok, "resolve fails without the bones");
        const char* swapped[9] = {"Root", "Hips", "Spine", "Spine1", "Spine2", "Neck", "Spine3", "Neck1", "Head"};
        Skeleton order = Resolve(9, [&](uint32_t i) { return std::string_view(swapped[i]); },
                                 [](uint32_t i) { return i == 0 ? -1 : static_cast<int>(i) - 1; });
        Check(!order.ok, "resolve fails when Neck is above Spine3");
    }

    // 1) sign: left circle (+ rate) -> positive lead, head turns LEFT (positive relative yaw); right circle mirrored
    {
        HeadLead L, R;
        Circle(L, P, +180.0f, 1.0f);
        Circle(R, P, -180.0f, 1.0f);
        Check(L.Lead() > 10.0f && L.Lead() <= P.maxLeadDeg + 1e-3f, "left circle: positive lead", L.Lead());
        Check(R.Lead() < -10.0f && R.Lead() >= -P.maxLeadDeg - 1e-3f, "right circle: negative lead", R.Lead());
        Check(std::fabs(L.Lead() + R.Lead()) < 1e-3f, "lead symmetry", L.Lead(), R.Lead());
        Check(L.Weight() > 0.99f && R.Weight() > 0.99f, "full weight in a tight circle", L.Weight(), R.Weight());
    }
    // 2) bias removal: head 35 deg turned the wrong way (right) in a left circle -> ends at the lead (left)
    {
        HeadLead H;
        Circle(H, P, 180.0f, 1.0f);
        float maxStep = 0.0f;
        const float res = Settle(H, P, sk, -35.0f, 2.0f, Signals{3.8f, 0.0f}, 180.0f, &maxStep);
        Check(std::fabs(res - H.Lead()) < P.deadbandDeg + 0.5f && res > 10.0f, "left circle: wrong-way head ends at +lead", res, H.Lead());
        HeadLead Rr;
        Circle(Rr, P, -180.0f, 1.0f);
        const float resR = Settle(Rr, P, sk, +35.0f, 2.0f, Signals{3.8f, 0.0f}, -180.0f);
        Check(std::fabs(resR - Rr.Lead()) < P.deadbandDeg + 0.5f && resR < -10.0f, "right circle: wrong-way head ends at -lead", resR, Rr.Lead());
        Check(std::fabs(res + resR) < 1.0f, "mirror symmetry of the result", res, resR);
    }
    // 3) a head already at the lead is left alone (inside the deadband), straight running = bit identical
    {
        HeadLead H;
        Circle(H, P, 180.0f, 1.0f);
        const float res = Settle(H, P, sk, H.Lead(), 1.0f, Signals{3.8f, 0.0f}, 180.0f);
        Check(std::fabs(res - H.Lead()) < 1.0f && std::fabs(H.Corr()) < 1.0f, "head at the lead stays", res, H.Corr());
        HeadLead S;
        Signals run;
        run.vx = 3.8f;
        for (int i = 0; i < 120; ++i)
            S.Step(P, run, kDt);
        float yaw[9] = {0, 0, 0, 0, 0, 30.0f, 30.0f, 30.0f, 30.0f + P.restBiasDeg - 25.0f};
        Rig a, b;
        a.Set(yaw);
        b = a;
        for (int i = 0; i < 120; ++i)
        {
            a = b;
            S.Apply(P, sk, a.pose, 9, kDt);
        }
        Check(std::memcmp(a.pose, b.pose, sizeof a.pose) == 0, "straight run: pose bit identical");
        Check(S.Weight() == 0.0f && S.Corr() == 0.0f, "straight run: weight 0");
    }
    // 4) untouched: positions, bones outside Neck / Neck1 / Head, Spine3 yaw, and the three bones share 0.25 / 0.25 / 0.5
    {
        HeadLead H;
        Circle(H, P, 180.0f, 1.0f);
        float yaw[9] = {0, 0, 0, 0, 0, 30.0f, 30.0f, 30.0f, 30.0f + P.restBiasDeg - 35.0f};
        Rig base, rig;
        base.Set(yaw);
        for (int i = 0; i < 90; ++i)
        {
            rig = base;
            Circle(H, P, 180.0f, kDt);
            H.Apply(P, sk, rig.pose, 9, kDt);
        }
        bool same = true;
        for (int b = 0; b < 9; ++b)
        {
            const bool changed = std::memcmp(rig.pose + b * 7 + 3, base.pose + b * 7 + 3, 16) != 0;
            same = same && std::memcmp(rig.pose + b * 7, base.pose + b * 7, 12) == 0;
            Check(changed == (b >= 6), "only Neck / Neck1 / Head rotations change", b);
        }
        Check(same, "positions untouched");
        // each bone is rotated by share * corr about the model Z: compare via the head yaw
        const float total = Wrap(rig.HeadVsSpine3() - base.HeadVsSpine3());
        Check(std::fabs(total - H.Corr()) < 0.05f, "total head yaw change = corr", total, H.Corr());
    }
    // 5) fades: the weight reaches 0 when aiming / weapon action / air / crouch / blocked / low speed, never faster than 0.15 s
    {
        const char* what[] = {"aiming", "weaponBusy", "air", "crouch", "blocked"};
        for (int k = 0; k < 5; ++k)
        {
            HeadLead H;
            Circle(H, P, 180.0f, 1.0f);
            Signals g;
            g.aiming = k == 0; g.weaponBusy = k == 1; g.air = k == 2; g.crouch = k == 3; g.blocked = k == 4;
            float prevW = H.Weight();
            float maxDrop = 0.0f;
            float tZero = -1.0f;
            for (float t = 0; t < 1.0f; t += kDt)
            {
                Circle(H, P, 180.0f, kDt, g);
                maxDrop = std::fmax(maxDrop, prevW - H.Weight());
                prevW = H.Weight();
                if (tZero < 0.0f && H.Weight() == 0.0f)
                    tZero = t;
            }
            Check(H.Weight() == 0.0f, what[k], H.Weight());
            Check(maxDrop <= kDt / 0.15f + 1e-4f, "fade out not faster than 0.15 s", maxDrop);
            // fade in again
            float maxRise = 0.0f;
            prevW = H.Weight();
            for (float t = 0; t < 1.0f; t += kDt)
            {
                Circle(H, P, 180.0f, kDt);
                maxRise = std::fmax(maxRise, H.Weight() - prevW);
                prevW = H.Weight();
            }
            Check(H.Weight() > 0.99f && maxRise <= kDt / 0.15f + 1e-4f, "fade in", H.Weight(), maxRise);
        }
        HeadLead H;
        Circle(H, P, 180.0f, 1.0f);
        Signals slow;
        for (float t = 0; t < 1.0f; t += kDt)
        {
            slow.vx = 0.4f; // 0.4 m/s: below the heading threshold
            H.Step(P, slow, kDt);
        }
        Check(H.Weight() == 0.0f && std::fabs(H.Lead()) < 0.5f, "low speed: weight 0, lead decays", H.Weight(), H.Lead());
        HeadLead I; // idle from the start
        for (int i = 0; i < 60; ++i)
            I.Step(P, Signals{}, kDt);
        Check(I.Weight() == 0.0f && I.Lead() == 0.0f, "idle: nothing");
    }
    // 6) rate limit of the correction and no NaN
    {
        HeadLead H;
        Circle(H, P, 180.0f, 1.0f);
        float maxStep = 0.0f;
        Settle(H, P, sk, -40.0f, 2.0f, Signals{3.8f, 0.0f}, 180.0f, &maxStep);
        Check(maxStep <= P.maxCorrRate * kDt + 1e-3f, "correction step bounded by maxCorrRate", maxStep, P.maxCorrRate * kDt);
        Check(std::fabs(H.Corr()) <= P.maxCorrDeg + 1e-3f, "correction clamped", H.Corr());
    }
    {
        HeadLead H;
        Circle(H, P, 180.0f, 1.0f);
        Signals bad;
        bad.vx = NAN; bad.vy = INFINITY;
        H.Step(P, bad, kDt);
        H.Step(P, Signals{}, NAN);
        H.Step(P, Signals{}, -1.0f);
        Check(std::isfinite(H.Lead()) && std::isfinite(H.Weight()) && std::isfinite(H.Rate()), "NaN signals are ignored");
        float yaw[9] = {0, 0, 0, 0, 0, 30.0f, 30.0f, 30.0f, 30.0f + P.restBiasDeg - 35.0f};
        Rig rig, ref;
        rig.Set(yaw);
        rig.pose[8 * 7 + 4] = NAN;
        ref = rig;
        const float c = H.Apply(P, sk, rig.pose, 9, kDt);
        Check(c == 0.0f && std::memcmp(rig.pose, ref.pose, 8 * 7 * sizeof(float)) == 0, "NaN pose: nothing written");
        Rig z;
        std::memset(z.pose, 0, sizeof z.pose); // zero quaternions
        ref = z;
        Check(H.Apply(P, sk, z.pose, 9, kDt) == 0.0f && std::memcmp(&z, &ref, sizeof z) == 0, "zero quaternion: nothing written");
        Rig r2;
        r2.Set(yaw);
        ref = r2;
        Check(H.Apply(P, sk, r2.pose, 9, NAN) == 0.0f && std::memcmp(&r2, &ref, sizeof r2) == 0, "NaN dt: nothing written");
        Check(H.Apply(P, sk, r2.pose, 5, kDt) == 0.0f, "too few bones: nothing");
        Check(H.Apply(P, Skeleton{}, r2.pose, 9, kDt) == 0.0f, "unresolved skeleton: nothing");
    }
    // 8) path law: the lead is measured against the travel direction, not against Spine3 (in-game report: left circle -> head slightly right)
    {
        Params legacy;
        legacy.pathBlend = 0.0f;
        // left circle (the captures 22-47 / 22-48: travel +27..+39 left of the actor forward, Spine3 ~6 deg right of the actor
        // forward, head 12 deg right: eyes-trav -44, i.e. the head looks 44 deg to the right of the run)
        const float trL = 39.0f, s3L = -5.7f, hdL = -12.0f;
        const PathResult newL = PathCircle(P, +180.0f, trL, s3L, hdL, 3.0f);
        const PathResult oldL = PathCircle(legacy, +180.0f, trL, s3L, hdL, 3.0f);
        Check(newL.lead > 10.0f, "path law: left circle lead positive", newL.lead);
        Check(newL.deltaHead > 20.0f, "path law: left circle turns the head LEFT in the actor frame", newL.deltaHead);
        // (the 50 deg twist window is centred on head-vs-Spine3 == 1 deg; the previous idle-centred window allowed +73 deg to the left)
        Check(newL.headVsPath > oldL.headVsPath + 5.0f, "path law: head clearly further into a left turn than the Spine3 law", newL.headVsPath, oldL.headVsPath);
        Check(newL.headVsPath > -3.0f, "path law: left circle head does not end right of the run", newL.headVsPath);
        Check(newL.headVsSpine <= P.maxTwistDeg + 0.5f, "path law: twist bound (left)", newL.headVsSpine);
        // right circle: travel 31 deg right of the actor forward, Spine3 +3.5, head +10.7 (eyes-trav +42: 42 deg LEFT of the run, uncorrected
        // by the Spine3 law because FR already holds the head right of Spine3)
        const float trR = -31.0f, s3R = 3.5f, hdR = 10.7f;
        const PathResult newR = PathCircle(P, -180.0f, trR, s3R, hdR, 3.0f);
        const PathResult oldR = PathCircle(legacy, -180.0f, trR, s3R, hdR, 3.0f);
        Check(newR.lead < -10.0f, "path law: right circle lead negative", newR.lead);
        Check(newR.deltaHead < -20.0f, "path law: right circle turns the head RIGHT in the actor frame", newR.deltaHead);
        Check(std::fabs(oldR.deltaHead) < 8.0f, "Spine3 law leaves the right circle head almost untouched", oldR.deltaHead);
        Check(newR.headVsPath < oldR.headVsPath - 20.0f, "path law: head clearly further into a right turn than the Spine3 law", newR.headVsPath, oldR.headVsPath);
        Check(newR.headVsSpine >= -P.maxTwistDeg - 0.5f, "path law: twist bound (right)", newR.headVsSpine);
        // the actor yaw is only used modulo 360
        const PathResult wr = PathCircle(P, +180.0f, trL, s3L, hdL, 3.0f, 360.0f);
        Check(std::fabs(wr.corr - newL.corr) < 0.01f, "path law: yaw 360 deg offset is irrelevant", wr.corr, newL.corr);
        // a head that already looks along the path + lead stays (deadband), a head too far into the turn is pulled back
        // (Spine3 at +20 so that the target head stays inside the 50 deg twist window)
        const PathResult at = PathCircle(P, +180.0f, trL, 20.0f, trL + 17.0f, 3.0f);
        Check(std::fabs(at.corr) < 5.0f, "path law: head at the lead stays", at.corr);
        // without a valid yaw or with pathBlend 0 the pose result is the Spine3 law, bit for bit
        {
            HeadLead A, B;
            Params pb = P;
            pb.pathBlend = 0.0f;
            const Skeleton skel = Sk();
            float yaw[9] = {0, 0, 0, 0, 0, 30.0f, 30.0f, 30.0f, 30.0f + P.restBiasDeg - 35.0f};
            Rig ra, rb, base;
            base.Set(yaw);
            float heading = 0.0f;
            bool same = true;
            for (int i = 0; i < 180; ++i)
            {
                heading += 180.0f * kDt;
                Signals sa, sb;
                sa.vx = sb.vx = 3.8f * std::cos(heading * kRad);
                sa.vy = sb.vy = 3.8f * std::sin(heading * kRad);
                sb.yawDeg = heading - 90.0f - 39.0f;
                sb.yawValid = true; // a valid yaw but pathBlend 0
                A.Step(P, sa, kDt);  // no yaw: Spine3 law
                B.Step(pb, sb, kDt); // pathBlend 0: Spine3 law
                ra = rb = base;
                A.Apply(P, skel, ra.pose, 9, kDt);
                B.Apply(pb, skel, rb.pose, 9, kDt);
                same = same && std::memcmp(ra.pose, rb.pose, sizeof ra.pose) == 0;
            }
            Check(same && A.Corr() != 0.0f, "no yaw / pathBlend 0: Spine3 law, bit identical", A.Corr(), B.Corr());
        }
        // straight running with a valid yaw: weight 0, pose bit identical
        {
            HeadLead S;
            const Skeleton skel = Sk();
            Signals run;
            run.vx = 3.8f; run.yawDeg = -90.0f + 30.0f; run.yawValid = true;
            float face[9] = {0, 0, 0, 0, 0, 5.0f, 5.0f, 5.0f, -15.0f};
            FaceRig base, a;
            base.Set(face);
            bool same = true;
            for (int i = 0; i < 120; ++i)
            {
                S.Step(P, run, kDt);
                a = base;
                S.Apply(P, skel, a.pose, 9, kDt);
                same = same && std::memcmp(a.pose, base.pose, sizeof a.pose) == 0;
            }
            Check(same && S.Weight() == 0.0f, "path law: straight run bit identical", S.Weight());
        }
        // twist window (twistBiasDeg 1.0, not the idle bias 23.39): symmetric left / right around head-vs-Spine3 == 1 deg. The run clips
        // keep the head ~1 deg off Spine3; with the window centred on the idle pose (old behaviour, twistBias == restBias) the right
        // circles were clipped at -26.6 deg and the left ones only at +73.4 deg (session 2026-10-20: right 37% of the frames, left 0%).
        {
            Params oldWin = P;
            oldWin.twistBiasDeg = oldWin.restBiasDeg; // the previous window
            // travel 39 deg off the forward in both directions, Spine3 on the actor forward, head 1 deg left of Spine3: the path law
            // asks for more than 50 deg of twist on both sides
            const PathResult wl = PathCircle(P, +180.0f, +39.0f, 0.0f, 1.0f, 3.0f);
            const PathResult wr2 = PathCircle(P, -180.0f, -39.0f, 0.0f, 1.0f, 3.0f);
            Check(wl.headVsSpine > 49.0f && wl.headVsSpine <= P.maxTwistDeg + 0.5f, "twist window: left circle saturates at +50", wl.headVsSpine);
            Check(wr2.headVsSpine < -49.0f && wr2.headVsSpine >= -P.maxTwistDeg - 0.5f, "twist window: right circle saturates at -50", wr2.headVsSpine);
            Check(std::fabs(wl.headVsSpine + wr2.headVsSpine) < 1.0f, "twist window: left / right symmetric", wl.headVsSpine, wr2.headVsSpine);
            const PathResult ol = PathCircle(oldWin, +180.0f, +39.0f, 0.0f, 1.0f, 3.0f);
            const PathResult orr = PathCircle(oldWin, -180.0f, -39.0f, 0.0f, 1.0f, 3.0f);
            // head vs Spine3 (raw): the old window ended at about +57 on the left but only at -26.6 on the right
            const float rawL = ol.headVsSpine + oldWin.twistBiasDeg, rawR = orr.headVsSpine + oldWin.twistBiasDeg;
            Check(std::fabs(rawL + rawR) > 20.0f && rawR > -P.maxTwistDeg + oldWin.restBiasDeg - 1.0f, "previous window (twistBias == restBias) was asymmetric", rawL, rawR);
            // the measurement still uses the idle bias: head at the idle pose measures 0
            HeadLead H;
            const Skeleton skel = Sk();
            FaceRig idle;
            const float face[9] = {0, 0, 0, 0, 0, 10.0f, 10.0f, 10.0f, 10.0f + P.restBiasDeg};
            idle.Set(face);
            Signals s;
            s.yawValid = true;
            s.vx = 3.8f;
            H.Step(P, s, kDt);
            FaceRig a = idle;
            H.Apply(P, skel, a.pose, 9, kDt);
            Check(std::fabs(H.Tel().measured) < 0.01f, "measurement unchanged: idle pose measures 0 (restBias)", H.Tel().measured);
            // standing pose (head at the idle bias from Spine3) in a left / right circle: the same window [-49, +51] vs Spine3
            const PathResult sl = PathCircle(P, +180.0f, 39.0f, 0.0f, P.restBiasDeg, 3.0f);
            const PathResult sr = PathCircle(P, -180.0f, -39.0f, 0.0f, P.restBiasDeg, 3.0f);
            Check(sl.headVsSpine <= P.maxTwistDeg + 0.5f && sl.headVsSpine >= -P.maxTwistDeg - 0.5f, "twist window: standing pose left", sl.headVsSpine);
            Check(sr.headVsSpine <= P.maxTwistDeg + 0.5f && sr.headVsSpine >= -P.maxTwistDeg - 0.5f, "twist window: standing pose right", sr.headVsSpine);
            // the Spine3 law (pathBlend 0) has no window: independent of twistBias, bit for bit
            Params pa = P, pb2 = P;
            pa.pathBlend = pb2.pathBlend = 0.0f;
            pb2.twistBiasDeg = 40.0f;
            const PathResult ra = PathCircle(pa, +180.0f, 39.0f, -5.7f, -12.0f, 3.0f);
            const PathResult rb2 = PathCircle(pb2, +180.0f, 39.0f, -5.7f, -12.0f, 3.0f);
            Check(ra.corr == rb2.corr && ra.headAbs == rb2.headAbs, "pathBlend 0: twistBias has no effect", ra.corr, rb2.corr);
            // configuration B (mm.lean + leanHead): the driver blocks the head lead (Signals.blocked) while V circles. Whatever the
            // twist window is, the weight stays 0 and the pose is bit identical for every twistBias and head offset.
            {
                bool same = true;
                float maxAbsCorr = 0.0f;
                for (float tb : {1.0f, 23.39f, -30.0f, 60.0f})
                    for (float off : {-60.0f, -10.0f, 0.0f, 25.0f, 80.0f})
                    {
                        Params pt = P;
                        pt.twistBiasDeg = tb;
                        HeadLead B;
                        FaceRig base2, cur;
                        const float f2[9] = {0, 0, 0, 0, 0, 10.0f, 10.0f, 10.0f, 10.0f + off};
                        base2.Set(f2);
                        float heading = 0.0f;
                        Signals sb;
                        sb.yawValid = true;
                        sb.blocked = true; // g_leanHeadOwns
                        for (int i = 0; i < 240; ++i)
                        {
                            heading += 180.0f * kDt;
                            sb.vx = 3.8f * std::cos(heading * kRad);
                            sb.vy = 3.8f * std::sin(heading * kRad);
                            sb.yawDeg = heading - 90.0f - 39.0f;
                            B.Step(pt, sb, kDt);
                            cur = base2;
                            B.Apply(pt, skel, cur.pose, 9, kDt);
                            same = same && std::memcmp(cur.pose, base2.pose, sizeof cur.pose) == 0;
                            maxAbsCorr = std::fmax(maxAbsCorr, std::fabs(B.Corr()));
                        }
                    }
                Check(same && maxAbsCorr == 0.0f, "blocked (leanHead owns): pose bit identical for any twistBias", maxAbsCorr);
            }
        }
    }
    // 7) tuning validation
    Check(ValidTuning(14.0f, 22.0f) && ValidTuning(0.0f, 0.0f) && ValidTuning(45.0f, 45.0f), "valid tuning");
    Check(!ValidTuning(NAN, 22.0f) && !ValidTuning(14.0f, NAN) && !ValidTuning(-1.0f, 22.0f) && !ValidTuning(14.0f, 46.0f) &&
              !ValidTuning(INFINITY, 1.0f),
          "invalid tuning rejected");

    std::printf(g_fail ? "HeadLeadTest FAILED (%d)\n" : "HeadLeadTest OK (%d failures)\n", g_fail);
    return g_fail ? 1 : 0;
}
