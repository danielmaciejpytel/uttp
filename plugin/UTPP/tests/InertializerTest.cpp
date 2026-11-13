// Offline test of the blend-time inertialization (src/mm/Inertializer.hpp QuinticInertializer, port 10.1).
//
//   utpp_mm_inertializer_test
//
// 1. Quintic curve (UE CalcInertialFloat): x(0) = x0, x'(0) = clamped v0, x = x' = x'' = 0 at the (possibly shortened)
//    end time, x stays in [0, x0] (no overshoot) on a grid of x0 / v0 / t1, analytic velocity = finite difference.
// 2. Pose runs at 30 / 60 / 144 fps for blend times 0.5 / 0.2 / 0.15 s (the MM node blend times), target velocity
//    compensated and UE mode:
//    - continuity at the switch: offset at t = 0 reproduces the last output exactly; output velocity at t = 0+ equals
//      the source velocity (compensated mode, approach case) / the offset speed equals the source speed (UE mode);
//    - discrete check: velocity change across the switch frame <= 15% of the jump without inertialization;
//    - convergence: output == target exactly once elapsed >= duration (per bone: foot bone duration / 1.5);
//    - no overshoot: offset along its initial direction stays in [0, x0], rotation angle in [0, angle0]; a velocity
//      pointing away from the target is clamped (UE spec), never amplified;
//    - pose switch mid-blend: continuity at the second switch (position exact, velocity within the curve's
//      acceleration * dt), convergence of the second blend, deficit rule on a third switch;
//    - frame-rate consistency of the blended trajectory (30 / 60 vs 144 fps).
// 3. Request rules (minimum of same-frame requests, ignored without a snapshot, duration 0 = cut), bones below
//    firstBone copied, quaternion sign continuity, non-finite input reset.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../src/mm/Inertializer.hpp"

using namespace utpp::mm;
using math::Quat;
using math::Vec3;

static int g_pass = 0;
static int g_fail = 0;

static void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat.c_str());
    }
}

static float Len(Vec3 v) { return std::sqrt(math::Dot(v, v)); }
static Quat QZ(float a) { return Quat{0.0f, 0.0f, std::sin(0.5f * a), std::cos(0.5f * a)}; }
// Signed angle about +Z of a rotation that is (close to) a pure Z rotation.
static float AngZ(const Quat& q) { return 2.0f * std::atan2(q.z, q.w); }
// Angle between two rotations, accurate near 0 (atan2 of the relative rotation; acos loses ~7e-4 rad in float).
static float AngleBetween(const Quat& a, const Quat& b)
{
    const Quat r = math::QuatAbs(math::QuatMul(a, math::QuatInv(b)));
    return 2.0f * std::atan2(std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z), r.w);
}
static float WrapPi(float a) { return math::WrapAngle(a); }

// ------------------------------------------------------------------------------------------------ 1. curve

static void TestCurve()
{
    const float x0s[] = {0.05f, 0.3f, 1.0f, -0.4f, 0.0f};
    const float v0s[] = {-40.0f, -6.0f, -1.0f, -0.1f, 0.0f, 0.5f, 4.0f};
    const float t1s[] = {0.5f, 0.2f, 0.15f};
    int n = 0, okStart = 0, okEnd = 0, okRange = 0, okVel = 0;
    float worstRange = 0.0f;
    for (float x0 : x0s)
        for (float v0 : v0s)
            for (float t1 : t1s)
            {
                ++n;
                QuinticCurve c;
                c.Init(x0, v0, t1);
                const float sgn = x0 < 0.0f ? -1.0f : 1.0f;
                const float vToward = (v0 * sgn) > 0.0f ? 0.0f : v0; // away from zero -> clamped
                const float ax0 = std::fabs(x0);
                const bool start = std::fabs(c.Eval(0.0f) - x0) <= 1e-6f &&
                                   (ax0 == 0.0f || std::fabs(c.Velocity(0.0f) - vToward) <= 1e-4f * (1.0f + std::fabs(v0)));
                const float tEnd = c.zero ? 0.0f : c.t1;
                const bool shortened = vToward * sgn < -kInertialSmallSpeed && 5.0f * ax0 / std::fabs(vToward) < t1;
                bool end = c.Eval(tEnd) == 0.0f && c.Eval(t1 + 1.0f) == 0.0f && c.Velocity(tEnd) == 0.0f &&
                           tEnd <= t1 + 1e-7f && (!shortened || std::fabs(tEnd - 5.0f * ax0 / std::fabs(vToward)) < 1e-6f);
                // x' and x'' -> 0 approaching t1 (triple root): |x(t1 - h)| <= K h^3
                if (!c.zero)
                {
                    const float h = 1e-2f * tEnd;
                    const float xs = std::fabs(c.Eval(tEnd - h));
                    const float vs = std::fabs(c.Velocity(tEnd - h));
                    end = end && xs <= 60.0f * (ax0 + std::fabs(vToward) * tEnd) * 1e-6f + 1e-7f &&
                          vs <= 60.0f * (ax0 / tEnd + std::fabs(vToward)) * 1e-4f + 1e-6f;
                }
                // range and analytic velocity
                bool range = true, vel = true;
                for (int i = 0; i <= 2000; ++i)
                {
                    const float t = tEnd * static_cast<float>(i) / 2000.0f;
                    const float x = c.Eval(t) * sgn; // magnitude side
                    if (x < -1e-6f * (1.0f + ax0) || x > ax0 * (1.0f + 1e-6f) + 1e-7f)
                    {
                        range = false;
                        worstRange = std::fmax(worstRange, std::fmax(-x, x - ax0));
                    }
                    if (i > 0 && i < 2000)
                    {
                        // Velocity() = d/dt Eval(): central difference of the same polynomial in double (float
                        // Horner rounding would dominate a float difference), and Eval() = the double polynomial.
                        auto xd = [&](double tt) {
                            return c.sign * (((((double(c.a) * tt + c.b) * tt + c.c) * tt + c.d) * tt + c.v0) * tt + c.x0);
                        };
                        const double h = 1e-6 * tEnd;
                        const double fd = (xd(t + h) - xd(t - h)) / (2.0 * h);
                        const double scale = ax0 + std::fabs(vToward) * tEnd;
                        if (std::fabs(fd - c.Velocity(t)) > 1e-4 * scale / tEnd + 1e-6 ||
                            std::fabs(xd(t) - c.Eval(t)) > 1e-5 * scale + 1e-7)
                            vel = false;
                    }
                }
                okStart += start;
                okEnd += end;
                okRange += range;
                okVel += vel;
                if (!(start && end && range && vel))
                    std::printf("  curve x0=%g v0=%g t1=%g: start %d end %d range %d vel %d (t1eff %g)\n", x0, v0, t1, start,
                                end, range, vel, tEnd);
            }
    std::printf("curve: %d cases; start %d, end %d, no overshoot %d (worst %.3g), velocity %d\n", n, okStart, okEnd, okRange,
                worstRange, okVel);
    Check(okStart == n, "curve: x(0) = x0, x'(0) = clamped v0");
    Check(okEnd == n, "curve: x = x' = x'' = 0 at the (shortened) end time");
    Check(okRange == n, "curve: no overshoot, x in [0, x0]");
    Check(okVel == n, "curve: Eval = polynomial, Velocity = its derivative");
    Check(InertialCurve(0.3f, -1.0f, 0.05f, 0.2f) == [] {
        QuinticCurve c;
        c.Init(0.3f, -1.0f, 0.2f);
        return c.Eval(0.05f);
    }(), "InertialCurve wrapper");
    Check(InertialCurve(NAN, 0.0f, 0.1f, 0.2f) == 0.0f && InertialCurve(0.3f, NAN, 0.1f, 0.2f) == 0.0f &&
              InertialCurve(0.3f, 0.0f, 0.1f, NAN) == 0.0f,
          "curve: non-finite input -> zero curve");
    Check(BlendProfileDurationFactor(true, 1.5f) == 1.0f / 1.5f && BlendProfileDurationFactor(true, 10.0f) == 0.1f &&
              BlendProfileDurationFactor(true, 0.0f) == 1.0f && BlendProfileDurationFactor(false, 0.5f) == 0.5f,
          "blend profile duration factors (WeightFactor 1/scale, TimeFactor scale)");
}

// ------------------------------------------------------------------------------------------------ 2. pose runs

// Linear motion of one bone: position p0 + v (t - t0), rotation about Z a0 + w (t - t0).
struct Motion
{
    Vec3 p0;
    Vec3 v;
    float a0 = 0.0f;
    float w = 0.0f;
    float t0 = 0.0f;
    BoneTransform At(float t) const { return BoneTransform{p0 + v * (t - t0), QZ(a0 + w * (t - t0))}; }
};

static constexpr uint32_t kBones = 4; // 0 root (copied), 1 pelvis (approach), 2 foot (approach, faster), 3 away

struct RunResult
{
    float posContErr = 0.0f, velContErr = 0.0f, rotContErr = 0.0f, angVelContErr = 0.0f;
    float ueVelErr = 0.0f;
    float dvRatio = 0.0f, dvRatioFoot = 0.0f;   // informational: velocity change / uninertialized jump
    float secondDiff[3] = {}, accBound[3] = {}; // max |O_k - 2 O_k-1 + O_k-2| / dt at the switch vs max |y''|
    float maxSpeed = 0.0f;                      // max |y'| of the pelvis curve
    float convergeErr = 1e9f, footConvergeErr = 1e9f;
    bool convergedAt = false, footConvergedAt = false, stillActiveBefore = false;
    float overshoot = 0.0f, rotOvershoot = 0.0f, awayGrowth = 0.0f;
    std::vector<Vec3> traj; // bone 1 output per frame after the switch
    std::vector<float> trajT;
};

static RunResult RunSwitch(float aFps, float aDuration, bool aCompensate)
{
    RunResult r;
    const float dt = 1.0f / aFps;
    const float ts = 0.5f; // first frame with the new target
    const int kSwitch = static_cast<int>(std::lround(ts * aFps));
    const int kEnd = kSwitch + static_cast<int>(std::ceil((aDuration + 0.3f) * aFps));
    // source / target motions per bone
    Motion src[kBones], dst[kBones];
    src[0] = {{0, 0, 0}, {0, 0, 0}, 0.0f, 0.0f, 0.0f};
    dst[0] = {{0, 0, 1}, {0, 0, 0}, 0.0f, 0.0f, ts};
    // pelvis: offset y = src - dst = -0.3 x, relative velocity +1.0 x (toward zero); rotation offset -0.5 rad, +1 rad/s
    src[1] = {{0, 0, 0}, {1.5f, 0, 0}, 0.0f, 2.0f, ts};
    dst[1] = {{0.3f, 0, 0}, {0.5f, 0, 0}, 0.5f, 1.0f, ts};
    // foot (duration / 1.5): offset along y with a 3D rotation offset
    src[2] = {{0, 0.2f, 0.1f}, {0, -0.8f, 0}, 0.3f, -1.0f, ts};
    dst[2] = {{0, -0.1f, 0.1f}, {0, 0.2f, 0}, -0.4f, 0.5f, ts};
    // away: relative velocity -1.5 x while the offset is -0.3 x (moving away from the target)
    src[3] = {{0, 0, 0}, {-1.0f, 0, 0}, 0.0f, 0.0f, ts};
    dst[3] = {{0.3f, 0, 0}, {0.5f, 0, 0}, 0.0f, 0.0f, ts};

    QuinticInertializer in;
    in.Init(kBones, 1);
    float factors[kBones] = {1.0f, 1.0f, BlendProfileDurationFactor(true, 1.5f), 1.0f};
    in.SetDurationFactors(factors);

    BoneTransform tgt[kBones], tgtPrev[kBones], out[kBones];
    std::vector<BoneTransform> outs; // per frame, bone 1..3 (index by frame)
    std::vector<BoneTransform> hist[kBones];
    float x0Pelvis = 0.0f, ang0Pelvis = 0.0f, x0Away = 0.0f;
    Vec3 dir0Pelvis{}, dir0Away{};
    for (int k = 0; k <= kEnd; ++k)
    {
        const float t = static_cast<float>(k) * dt;
        const bool after = k >= kSwitch;
        for (uint32_t b = 0; b < kBones; ++b)
        {
            tgt[b] = after ? dst[b].At(t) : src[b].At(t);
            tgtPrev[b] = after ? dst[b].At(t - dt) : src[b].At(t - dt);
        }
        if (k == kSwitch)
            in.Request(aDuration);
        in.Update(tgt, aCompensate ? tgtPrev : nullptr, dt, out);
        for (uint32_t b = 0; b < kBones; ++b)
            hist[b].push_back(out[b]);
        Check(out[0].p.z == tgt[0].p.z && out[0].q.w == tgt[0].q.w, "bone below firstBone copied");

        if (k == kSwitch)
        {
            // analytic continuity at the switch (offset curve origin = previous frame t_k = t - dt)
            const float tk = t - dt;
            for (uint32_t b = 1; b <= 2; ++b)
            {
                Vec3 op, lv, av;
                Quat oq;
                in.OffsetAt(b, 0.0f, op, oq, &lv, &av);
                const BoneTransform ref = aCompensate ? dst[b].At(tk) : dst[b].At(t);
                const BoneTransform sk = src[b].At(tk);
                const Vec3 pos = ref.p + op;
                const Quat rot = math::QuatMul(oq, ref.q);
                r.posContErr = std::fmax(r.posContErr, Len(pos - sk.p));
                r.rotContErr = std::fmax(r.rotContErr, AngleBetween(rot, sk.q));
                if (aCompensate)
                {
                    // output velocity at 0+ = target velocity + offset velocity (offset and its rate are colinear here)
                    r.velContErr = std::fmax(r.velContErr, Len((dst[b].v + lv) - src[b].v));
                    // angular: about the offset axis only (UE drops the perpendicular part); bone 1 is a pure Z case
                    if (b == 1)
                        r.angVelContErr = std::fmax(r.angVelContErr, std::fabs((dst[b].w + av.z) - src[b].w));
                }
                else
                {
                    // UE: offset speed = source velocity along the offset direction
                    const Vec3 dir = op * (1.0f / std::fmax(Len(op), 1e-9f));
                    r.ueVelErr = std::fmax(r.ueVelErr, std::fabs(math::Dot(lv, dir) - math::Dot(src[b].v, dir)));
                }
            }
            Vec3 op;
            Quat oq;
            in.OffsetAt(1, 0.0f, op, oq);
            x0Pelvis = Len(op);
            dir0Pelvis = op * (1.0f / x0Pelvis);
            ang0Pelvis = AngleBetween(oq, Quat{});
            in.OffsetAt(3, 0.0f, op, oq);
            x0Away = Len(op);
            dir0Away = op * (1.0f / x0Away);
            Vec3 lv;
            in.OffsetAt(3, 0.0f, op, oq, &lv);
            Check(Len(lv) == 0.0f, "away case: offset speed clamped to 0 (UE no-overshoot rule)");
            for (uint32_t b = 1; b <= 2; ++b)
            {
                const QuinticCurve& c = in.TranslationCurve(b);
                for (int i = 0; i <= 1000; ++i)
                {
                    const float tt = c.t1 * static_cast<float>(i) / 1000.0f;
                    const float h = 1e-4f;
                    r.accBound[b] = std::fmax(r.accBound[b], std::fabs((c.Velocity(tt + h) - c.Velocity(tt)) / h));
                    if (b == 1)
                        r.maxSpeed = std::fmax(r.maxSpeed, std::fabs(c.Velocity(tt)));
                }
            }
        }
        if (after)
        {
            // no overshoot: pelvis offset along its initial direction stays in [0, x0], perpendicular part 0 (compensated)
            const Vec3 off = out[1].p - tgt[1].p;
            const float along = math::Dot(off, dir0Pelvis);
            const float perp = Len(off - dir0Pelvis * along);
            r.overshoot = std::fmax(r.overshoot, std::fmax(-along, along - x0Pelvis));
            r.overshoot = std::fmax(r.overshoot, perp - 1e-5f);
            const float ang = std::fabs(WrapPi(AngZ(out[1].q) - AngZ(tgt[1].q)));
            r.rotOvershoot = std::fmax(r.rotOvershoot, ang - ang0Pelvis);
            const Vec3 offA = out[3].p - tgt[3].p;
            r.awayGrowth = std::fmax(r.awayGrowth, math::Dot(offA, dir0Away) - x0Away);
            r.awayGrowth = std::fmax(r.awayGrowth, -math::Dot(offA, dir0Away));
            r.traj.push_back(out[1].p - tgt[1].p);
            r.trajT.push_back(t);

            const float elapsed = static_cast<float>(k - kSwitch + 1) * dt; // the first output is at elapsed = dt
            const float footDur = aDuration / 1.5f;
            if (elapsed >= footDur - 1e-6f && elapsed < aDuration - 1e-6f && !r.footConvergedAt)
            {
                r.footConvergedAt = true;
                r.footConvergeErr = Len(out[2].p - tgt[2].p) + AngleBetween(out[2].q, tgt[2].q);
                r.stillActiveBefore = in.Active() && Len(out[1].p - tgt[1].p) > 0.0f;
            }
            if (elapsed >= aDuration - 1e-6f && !r.convergedAt)
            {
                r.convergedAt = true;
                r.convergeErr = 0.0f;
                for (uint32_t b = 1; b < kBones; ++b)
                    r.convergeErr =
                        std::fmax(r.convergeErr, Len(out[b].p - tgt[b].p) + AngleBetween(out[b].q, tgt[b].q));
                Check(!in.Active(), "inactive once elapsed >= duration");
            }
        }
    }
    // discrete velocity change across the switch (pelvis, foot) vs the jump without inertialization
    auto dvRatio = [&](uint32_t b) {
        const float vBefore = Len(hist[b][kSwitch - 1].p - hist[b][kSwitch - 2].p) / dt;
        (void)vBefore;
        float worst = 0.0f;
        for (int k = kSwitch; k <= kSwitch + 1; ++k)
        {
            const Vec3 va = (hist[b][k].p - hist[b][k - 1].p) * (1.0f / dt);
            const Vec3 vb = (hist[b][k - 1].p - hist[b][k - 2].p) * (1.0f / dt);
            worst = std::fmax(worst, Len(va - vb));
        }
        // without inertialization the first frame jumps by the whole offset
        const float jump = Len((dst[b].At(ts).p - src[b].At(ts - dt).p) * (1.0f / dt) - src[b].v);
        return worst / jump;
    };
    r.dvRatio = dvRatio(1);
    r.dvRatioFoot = dvRatio(2);
    for (uint32_t b = 1; b <= 2; ++b)
        for (int k = kSwitch; k <= kSwitch + 1; ++k)
            r.secondDiff[b] =
                std::fmax(r.secondDiff[b], Len(hist[b][k].p - hist[b][k - 1].p * 2.0f + hist[b][k - 2].p) / dt);
    return r;
}

// Pose switch in the middle of a running blend (1D along x): continuity at the second switch, convergence of the second
// blend, no overshoot; optionally a third switch shortly after the second one (deficit rule).
struct MidResult
{
    float posErr2 = 0.0f, velJump = 0.0f, accBound = 0.0f, dur2 = 0.0f;
    float overshoot2 = 0.0f, convergeErr2 = 1e9f;
    float dur3 = -1.0f, expectDur3 = -1.0f;
};

static MidResult RunMid(float aFps, float aDuration, bool aCompensate, bool aThirdSwitch)
{
    MidResult r;
    const float dt = 1.0f / aFps;
    const float ts = 0.5f;
    const int kSwitch = static_cast<int>(std::lround(ts * aFps));
    const int kSwitch2 = kSwitch + std::max(2, static_cast<int>(std::lround(0.4f * aDuration * aFps)));
    const int kSwitch3 = kSwitch2 + std::max(2, static_cast<int>(std::lround(0.3f * aDuration * aFps)));
    const int kEnd = aThirdSwitch ? kSwitch3 : kSwitch2 + static_cast<int>(std::ceil((aDuration + 0.1f) * aFps));
    const Motion src{{0, 0, 0}, {1.5f, 0, 0}, 0.0f, 0.0f, ts};
    const Motion dst{{0.3f, 0, 0}, {0.5f, 0, 0}, 0.0f, 0.0f, ts};
    Motion third{}; // decided at kSwitch2 from the output: 0.25 m ahead, 1 m/s slower than the output
    QuinticInertializer in;
    in.Init(2, 1);
    BoneTransform tgt[2], tgtPrev[2], out[2];
    std::vector<Vec3> outs;
    float x02 = 0.0f;
    Vec3 vTrue{}; // analytic output velocity at the last frame before the second switch: target velocity + offset rate
    for (int k = 0; k <= kEnd; ++k)
    {
        const float t = static_cast<float>(k) * dt;
        if (k == kSwitch2)
        {
            const Vec3 vMeas = (outs[k - 1] - outs[k - 2]) * (1.0f / dt);
            third = Motion{outs[k - 1] + Vec3{0.25f, 0, 0}, vMeas - Vec3{1.0f, 0, 0}, 0.0f, 0.0f, t - dt};
            // max acceleration of the first blend's curve (bound for the backward-difference velocity error)
            const QuinticCurve& c = in.TranslationCurve(1);
            for (int i = 0; i < 400; ++i)
            {
                const float tt = c.t1 * static_cast<float>(i) / 400.0f;
                const float h = 1e-4f;
                r.accBound = std::fmax(r.accBound, std::fabs((c.Velocity(tt + h) - c.Velocity(tt)) / h));
            }
        }
        const Motion& m = k < kSwitch ? src : (k < kSwitch2 ? dst : third);
        tgt[1] = m.At(t);
        tgtPrev[1] = m.At(t - dt);
        tgt[0] = tgtPrev[0] = BoneTransform{};
        if (k == kSwitch || k == kSwitch2 || (aThirdSwitch && k == kSwitch3))
            in.Request(aDuration);
        if (aThirdSwitch && k == kSwitch3)
            r.expectDur3 = std::fmax(aDuration - (in.Duration() - in.Elapsed()), 0.0f); // deficit > 0 -> applied
        in.Update(tgt, aCompensate ? tgtPrev : nullptr, dt, out);
        outs.push_back(out[1].p);
        if (k == kSwitch2 - 1)
        {
            Vec3 op, lv;
            Quat oq;
            in.OffsetAt(1, in.Elapsed(), op, oq, &lv);
            vTrue = dst.v + lv;
        }
        if (k == kSwitch2)
        {
            Vec3 op, lv;
            Quat oq;
            in.OffsetAt(1, 0.0f, op, oq, &lv);
            const BoneTransform ref = aCompensate ? third.At(t - dt) : third.At(t);
            r.posErr2 = Len(ref.p + op - outs[k - 1]);
            // output velocity right after the switch vs the true velocity right before it (the inertializer only
            // knows the backward difference of its outputs: error ~ |y''| dt / 2)
            const Vec3 vOut0 = (aCompensate ? third.v : Vec3{}) + lv;
            r.velJump = Len(vOut0 - vTrue);
            x02 = Len(op);
            r.dur2 = in.Duration();
        }
        if (aThirdSwitch && k == kSwitch3)
            r.dur3 = in.Duration();
        if (k >= kSwitch2 && !(aThirdSwitch && k >= kSwitch3))
        {
            const float off = out[1].p.x - tgt[1].p.x; // starts at -x02 (behind the target), must stay in [-x02, 0]
            r.overshoot2 = std::fmax(r.overshoot2, std::fmax(off, -off - x02 * (1.0f + 1e-5f)));
            const float elapsed2 = static_cast<float>(k - kSwitch2 + 1) * dt;
            if (elapsed2 >= aDuration - 1e-6f && r.convergeErr2 > 1e8f)
                r.convergeErr2 = std::fabs(off) + (in.Active() ? 1.0f : 0.0f);
        }
    }
    return r;
}

static void TestPoseRuns()
{
    const float fpsList[] = {30.0f, 60.0f, 144.0f};
    const float durations[] = {0.5f, 0.2f, 0.15f};
    for (int mode = 0; mode < 2; ++mode)
    {
        const bool comp = mode == 0;
        for (float dur : durations)
        {
            RunResult ref; // 144 fps for the frame-rate consistency
            RunResult res[3];
            for (int f = 0; f < 3; ++f)
                res[f] = RunSwitch(fpsList[f], dur, comp);
            ref = res[2];
            for (int f = 0; f < 3; ++f)
            {
                const RunResult& r = res[f];
                // frame-rate consistency: compare the pelvis offset at common times with 144 fps (interpolated)
                float fpsDiff = 0.0f;
                for (size_t i = 0; i < r.traj.size(); ++i)
                {
                    const float t = r.trajT[i];
                    for (size_t j = 1; j < ref.trajT.size(); ++j)
                        if (ref.trajT[j] >= t - 1e-6f)
                        {
                            const float u = (t - ref.trajT[j - 1]) / (ref.trajT[j] - ref.trajT[j - 1]);
                            const Vec3 rp = math::Lerp(ref.traj[j - 1], ref.traj[j], std::fmin(std::fmax(u, 0.0f), 1.0f));
                            fpsDiff = std::fmax(fpsDiff, Len(r.traj[i] - rp));
                            break;
                        }
                }
                std::printf("  switch %3.0f fps %.2f s %s: pos cont %.1e m, rot cont %.1e rad, vel cont %.1e m/s, angvel "
                            "cont %.1e rad/s, UE speed err %.1e, dv ratio %.3f (foot %.3f), converge %.1e (foot %.1e), "
                            "overshoot %.1e m / %.1e rad, away growth %.1e, fps diff %.4f m, vel change %.3f/%.3f m/s (curve acc*dt "
                            "%.3f/%.3f)\n",
                            fpsList[f], dur, comp ? "comp" : "UE  ", r.posContErr, r.rotContErr, r.velContErr,
                            r.angVelContErr, r.ueVelErr, r.dvRatio, r.dvRatioFoot, r.convergeErr, r.footConvergeErr,
                            r.overshoot, r.rotOvershoot, r.awayGrowth, fpsDiff, r.secondDiff[1], r.secondDiff[2],
                            r.accBound[1] / fpsList[f], r.accBound[2] / fpsList[f]);
                const std::string tag = std::to_string(static_cast<int>(fpsList[f])) + " fps " + std::to_string(dur) +
                                        (comp ? " s comp" : " s UE");
                Check(r.posContErr <= 1e-5f && r.rotContErr <= 2e-4f, "position / rotation continuous at the switch, " + tag);
                if (comp)
                    Check(r.velContErr <= 1e-3f && r.angVelContErr <= 1e-3f, "velocity continuous at the switch, " + tag);
                else
                    Check(r.ueVelErr <= 1e-3f, "UE mode: offset speed = source speed along the offset, " + tag);
                // A C1 output changes its frame velocity by at most max|y''| * dt across the switch (a velocity jump would
                // add a dt-independent term); UE mode adds the ignored target velocity on the second frame.
                const float fdt = 1.0f / fpsList[f];
                for (uint32_t b = 1; b <= 2; ++b)
                {
                    const float vTarget = comp ? 0.0f : (b == 1 ? 0.5f : 0.2f);
                    const float bound = r.accBound[b] * fdt + vTarget + 1e-3f;
                    Check(r.secondDiff[b] <= bound, "velocity change across the switch <= max|y''| dt" +
                                                        std::string(comp ? "" : " + |v target|") + ", bone " +
                                                        std::to_string(b) + ", " + tag + " (" +
                                                        std::to_string(r.secondDiff[b]) + " > " + std::to_string(bound) + ")");
                }
                Check(r.convergedAt && r.convergeErr <= 1e-6f, "converged to the target by the blend time, " + tag);
                Check(r.footConvergedAt && r.footConvergeErr <= 1e-6f && r.stillActiveBefore,
                      "foot (WeightFactor 1.5) converged by duration / 1.5 while the pelvis still blends, " + tag);
                Check(r.overshoot <= 1e-5f && r.rotOvershoot <= 1e-4f, "no overshoot (offset in [0, x0]), " + tag);
                Check(r.awayGrowth <= 1e-5f, "away velocity clamped: offset never grows or crosses, " + tag);
                // The curve starts at the previous frame (UE: first output at elapsed = dt), so trajectories of different
                // frame rates differ by O(dt): bound (2 max|y'| + |v target|) * (dt - dt_144).
                if (f < 2)
                {
                    const float bound =
                        (2.0f * ref.maxSpeed + (comp ? 0.0f : 0.5f)) * (1.0f / fpsList[f] - 1.0f / 144.0f) + 1e-4f;
                    Check(fpsDiff <= bound, "frame-rate consistency vs 144 fps within O(dt), " + tag + " (" +
                                                std::to_string(fpsDiff) + " > " + std::to_string(bound) + ")");
                }
            }
            for (int f = 0; f < 3; ++f)
            {
                const MidResult m = RunMid(fpsList[f], dur, comp, false);
                const MidResult m3 = RunMid(fpsList[f], dur, comp, true);
                std::printf("  mid-blend %3.0f fps %.2f s %s: pos %.1e m, vel jump %.3f m/s (bound acc*dt %.3f), 2nd duration "
                            "%.3f, overshoot %.1e, converge %.1e; 3rd switch duration %.4f (expected %.4f)\n",
                            fpsList[f], dur, comp ? "comp" : "UE  ", m.posErr2, m.velJump, m.accBound / fpsList[f], m.dur2,
                            m.overshoot2, m.convergeErr2, m3.dur3, m3.expectDur3);
                const std::string tag = std::to_string(static_cast<int>(fpsList[f])) + " fps " + std::to_string(dur) +
                                        (comp ? " s comp" : " s UE");
                Check(m.posErr2 <= 1e-5f, "mid-blend: position continuous at the second switch, " + tag);
                if (comp)
                    Check(m.velJump <= m.accBound / fpsList[f] + 1e-3f,
                          "mid-blend: velocity continuous within the curve acceleration * dt, " + tag);
                Check(std::fabs(m.dur2 - dur) <= 1e-6f, "mid-blend: first interruption keeps the full duration, " + tag);
                Check(m.overshoot2 <= 1e-5f, "mid-blend: second blend does not overshoot, " + tag);
                Check(m.convergeErr2 <= 1e-6f, "mid-blend: second blend converged by its blend time, " + tag);
                Check(m3.expectDur3 >= 0.0f && m3.expectDur3 < dur && std::fabs(m3.dur3 - m3.expectDur3) <= 1e-6f,
                      "mid-blend: a further interruption is shortened by the deficit (UE), " + tag);
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------ 3. rules

static void TestRules()
{
    QuinticInertializer in;
    Check(!in.Init(0, 0) && !in.Init(2, 2) && in.Init(3, 1), "Init argument checks");
    BoneTransform a[3], b[3], out[3];
    for (int i = 0; i < 3; ++i)
    {
        a[i] = BoneTransform{{0, 0, 0}, QZ(0.0f)};
        b[i] = BoneTransform{{1, 0, 0}, QZ(1.0f)};
    }
    // request before any snapshot: ignored (cut)
    in.Request(0.5f);
    in.Update(b, nullptr, 1.0f / 60.0f, out);
    Check(!in.Active() && out[1].p.x == 1.0f, "request without a snapshot is ignored");
    // same-frame requests: minimum
    in.Update(a, nullptr, 1.0f / 60.0f, out);
    in.Request(0.5f);
    in.Request(0.2f);
    in.Request(0.3f);
    in.Update(b, nullptr, 1.0f / 60.0f, out);
    Check(in.Active() && in.Duration() == 0.2f, "several requests in one frame: minimum duration");
    // duration 0 = cut
    QuinticInertializer cut;
    cut.Init(3, 1);
    cut.Update(a, nullptr, 1.0f / 60.0f, out);
    cut.Request(0.0f);
    cut.Update(b, nullptr, 1.0f / 60.0f, out);
    Check(!cut.Active() && out[1].p.x == 1.0f && out[2].q.z == b[2].q.z, "duration 0 = cut to the target");
    // quaternion sign continuity: target given with the opposite sign
    QuinticInertializer sg;
    sg.Init(3, 1);
    sg.Update(a, nullptr, 1.0f / 60.0f, out);
    BoneTransform neg[3];
    for (int i = 0; i < 3; ++i)
        neg[i] = BoneTransform{a[i].p, math::QuatNeg(QZ(0.01f))};
    sg.Update(neg, nullptr, 1.0f / 60.0f, out);
    Check(out[1].q.w > 0.0f, "output quaternion sign continuous");
    // non-finite target -> reset, false; recovers afterwards
    QuinticInertializer nf;
    nf.Init(3, 1);
    nf.Update(a, nullptr, 1.0f / 60.0f, out);
    nf.Update(a, nullptr, 1.0f / 60.0f, out);
    nf.Request(0.2f);
    BoneTransform bad[3] = {a[0], BoneTransform{{NAN, 0, 0}, QZ(0.0f)}, a[2]};
    const bool ok = nf.Update(bad, nullptr, 1.0f / 60.0f, out);
    Check(!ok && !nf.Active(), "non-finite target: Update returns false and stops the blend");
    Check(nf.Update(b, nullptr, 1.0f / 60.0f, out) && out[1].p.x == 1.0f, "recovers on the next finite frame");
    // dt 0 / NaN does not advance the blend
    QuinticInertializer z;
    z.Init(3, 1);
    z.Update(a, nullptr, 1.0f / 60.0f, out);
    z.Update(a, nullptr, 1.0f / 60.0f, out);
    z.Request(0.2f);
    z.Update(b, nullptr, 0.0f, out);
    z.Update(b, nullptr, NAN, out);
    Check(z.Active() && z.Elapsed() == 0.0f && std::fabs(out[1].p.x - 0.0f) < 1e-6f, "dt 0 / NaN: blend does not advance");
    // Reset forgets history: next request ignored until a new snapshot
    z.Reset();
    z.Request(0.2f);
    z.Update(a, nullptr, 1.0f / 60.0f, out);
    Check(!z.Active() && out[1].p.x == 0.0f, "Reset clears the history and the blend");
    // interruption deficit (UE rule) and the per-request IgnoreDeficit (air landing blend): blend 0.5 s, interrupted
    // after 1 frame by another 0.5 s request (deficit recorded, not yet paid), then a 0.2 s request one frame later pays the
    // deficit (0.5 - 1/60 > 0.2) -> duration 0 = cut; with aIgnoreDeficit the same request keeps its full 0.2 s, the deficit
    // stays recorded and the NEXT default request pays it again (the flag is per request, not sticky).
    const float dt60 = 1.0f / 60.0f;
    auto deficitRun = [&](bool aIgnore, QuinticInertializer& aIn) {
        aIn.Init(3, 1);
        aIn.Update(a, nullptr, dt60, out);
        aIn.Request(0.5f);
        aIn.Update(b, nullptr, dt60, out); // blend 1 starts
        aIn.Request(0.5f);
        aIn.Update(a, nullptr, dt60, out); // interrupted: deficit recorded
        aIn.Request(0.2f, aIgnore);
        aIn.Update(b, nullptr, dt60, out); // the "touchdown" request
    };
    QuinticInertializer dp, di;
    deficitRun(false, dp);
    Check(!dp.Active() && out[1].p.x == 1.0f, "deficit: a 0.2 s request after an interruption pays the deficit (duration 0 = cut)");
    deficitRun(true, di);
    const float diDeficit = di.Deficit();
    Check(di.Active() && di.Duration() == 0.2f && out[1].p.x < 1.0f,
          "IgnoreDeficit request: full 0.2 s blend despite the recorded deficit (no cut)");
    Check(diDeficit > 0.2f, "IgnoreDeficit request: the deficit is still recorded (" + std::to_string(diDeficit) + " s)");
    di.Request(0.2f);
    di.Update(a, nullptr, dt60, out);
    Check(di.Duration() < 0.2f - 1e-4f, "IgnoreDeficit is per request: the next default request pays the deficit again (duration " +
                                            std::to_string(di.Duration()) + " s)");
    QuinticInertializer dm;
    dm.Init(3, 1);
    dm.Update(a, nullptr, dt60, out);
    dm.Request(0.5f);
    dm.Update(b, nullptr, dt60, out);
    dm.Request(0.5f);
    dm.Update(a, nullptr, dt60, out);
    dm.Request(0.3f, true);
    dm.Request(0.2f); // same frame: minimum duration, the ignore flag of any request in the frame holds
    dm.Update(b, nullptr, dt60, out);
    Check(dm.Active() && dm.Duration() == 0.2f, "same-frame requests: minimum duration, IgnoreDeficit of either request kept");
}

int main()
{
    TestCurve();
    TestPoseRuns();
    TestRules();
    std::printf("utpp_mm_inertializer_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
