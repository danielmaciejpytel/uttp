// UTPP motion matching runtime - feet on uneven ground (stairs, slopes, curbs, rubble). Header-only, C++20, deterministic,
// allocation-free.
//
// Ground traces (one per foot, taken by the game script under the ball of the drawn foot of the previous frame) give a world hit
// height and a surface normal per foot. This stage places the feet on that ground and lets the whole leg chain adapt:
//   - planting plane per foot: the hit height follows the trace through a critically damped spring (stiffness heightStiffness) and
//     the normal through another one (normalStiffness), both in closed form of dt; the smoothed plane may stay at most
//     maxPenetration below the traced ground. The plane is held in world height, so a snap of the drawn capsule never moves a foot
//     in the world.
//   - every foot (planted or swinging) keeps the clip's height above the model ground plane, carried onto its planting plane: the
//     ball (toe base) is lifted by the plane height under it and the foot turns about the ball by the tilt of the plane, limited to
//     maxPitchDeg / maxRollDeg in the foot's heading frame (a planted toe stays where the lock put it).
//   - dead zone: deadHeight is subtracted from the height and deadAngleDeg from the tilt (continuous at the threshold); on flat
//     ground both are zero and the stage hands the pose to the body height stage untouched (bit for bit).
//   - pelvis: the PelvisPlacement range (all legs, no raise) of the ground targets minus the same range for the animated ankles, so
//     only a ground difference lowers the pelvis; a critically damped spring (pelvisStiffness), added to the body height offset.
//   - sudden capsule motion: a change of the drawn height between two updates beyond what the slope under the feet explains
//     (above suddenStep) is taken into the pelvis spring, so the body does not jump with the capsule and springs back instead.
//   - one leg solve (shared two-bone solver) per frame for the body height offset, the pelvis offset and the foot goals. A leg
//     whose knee would fold more than kneeFreeDeg over the animation gets a shorter ground correction instead of a broken knee.
// The stage fades in / out with fadeTau per foot (a missing or inconsistent trace: hit more than maxGroundDelta from the capsule).
// Model space: character frame, z up (the frame is turned about z only, so model z is world z).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "BodyHeightFilter.hpp"
#include "LegIk.hpp"
#include "MMMath.hpp"
#include "PelvisPlacement.hpp"

namespace utpp::mm::groundfeet
{

struct Config
{
    float heightStiffness = 1000.0f; // planting plane height spring
    float normalStiffness = 450.0f;  // planting plane normal spring
    float maxPenetration = 0.20f;    // m, the smoothed plane stays at most this below the traced ground
    float deadHeight = 0.005f;       // m, subtracted from the ground height under the ball
    float deadAngleDeg = 2.0f;       // deg, subtracted from the plane tilt
    float maxPitchDeg = 25.0f;       // deg, foot pitch to the surface (heading frame)
    float maxRollDeg = 15.0f;        // deg, foot roll to the surface
    float kneeFreeDeg = 60.0f;       // deg, extra knee flexion over the animation
    float minKneeDeg = 25.0f;        // deg, smallest interior knee angle
    float maxGroundDelta = 0.35f;    // m, a hit further from the capsule height is not used (car roof, crate, body)
    float fadeTau = 0.08f;           // s, per-foot fade in / out
    float maxDt = 0.25f;             // s, a longer gap restarts the stage
    float pelvisStiffness = 100.0f;  // pelvis offset spring
    float maxDrop = 0.25f;           // m, lowest pelvis offset of the stage
    float maxCompensation = 0.25f;   // m, largest pelvis offset taken from a sudden capsule move (either sign)
    float heelLift = 0.0f;           // PelvisPlacement heel share (0: the pelvis drops until the leg reaches within softReach; the
                                     // solver never lifts the heel, so a share left to it would leave the foot short of the ground)
    float reach = 0.995f;            // leg reach (fraction of thigh + shin)
    float softReach = 0.97f;         // soft reach start
    float suddenStep = 0.02f;        // m, drawn height change per update beyond the slope that counts as a capsule snap
    float bias = 0.0f;               // m, ground hit minus capsule height on flat ground (calibration)
};

inline bool ValidConfig(const Config& c)
{
    auto pos = [](float v) { return std::isfinite(v) && v > 0.0f; };
    auto nonneg = [](float v) { return std::isfinite(v) && v >= 0.0f; };
    return pos(c.heightStiffness) && pos(c.normalStiffness) && nonneg(c.maxPenetration) && nonneg(c.deadHeight) &&
           nonneg(c.deadAngleDeg) && pos(c.maxPitchDeg) && c.maxPitchDeg < 80.0f && pos(c.maxRollDeg) && c.maxRollDeg < 80.0f &&
           nonneg(c.kneeFreeDeg) && nonneg(c.minKneeDeg) && pos(c.maxGroundDelta) && pos(c.fadeTau) && pos(c.maxDt) &&
           pos(c.pelvisStiffness) && nonneg(c.maxDrop) && nonneg(c.maxCompensation) && nonneg(c.heelLift) && c.heelLift <= 1.0f &&
           std::isfinite(c.reach) && c.reach > 0.5f && c.reach <= 1.0f && std::isfinite(c.softReach) && c.softReach > 0.5f &&
           c.softReach <= c.reach && pos(c.suddenStep) && std::isfinite(c.bias) && std::fabs(c.bias) < 0.2f;
}

// One trace per foot, in the model frame of the capsule the trace was taken with (x, y: the traced point; n: the surface normal)
// plus the world height of the hit.
struct FootSample
{
    uint32_t hit = 0;  // 1 = the trace hit the ground
    float x = 0.0f, y = 0.0f;
    float z = 0.0f;    // world height of the hit
    float nx = 0.0f, ny = 0.0f, nz = 1.0f;
};
struct Samples
{
    uint32_t serial = 0;
    float yaw = 0.0f;      // rad, model -> world turn of the trace capsule (world = cap + Rz(yaw) model)
    float capZ = 0.0f;     // world height of the trace capsule
    uint32_t pad = 0;
    int64_t ticks = 0;     // clock of the trace (0 = unset)
    FootSample foot[2];
};
static_assert(std::is_trivially_copyable_v<Samples> && sizeof(Samples) % 4 == 0);

// Planting plane of a foot in the model frame of the drawn capsule (height h at (x, y), unit normal n) and its fade weight.
struct FootPlane
{
    float w = 0.0f;
    float x = 0.0f, y = 0.0f, h = 0.0f;
    float nx = 0.0f, ny = 0.0f, nz = 1.0f;
};
struct Plan
{
    FootPlane foot[2];
    float pelvis = 0.0f;  // m, ground pelvis offset (added to the body height offset)
    uint32_t active = 0;  // 1 = the plan changes the pose (some foot weight > 0 or a pelvis offset)
};
static_assert(std::is_trivially_copyable_v<Plan> && sizeof(Plan) % 4 == 0);

struct LegFrame
{
    BoneTransform ga{}, gb{}, gc{}; // UpLeg, Leg, Foot (model space)
    math::Vec3 ball{};              // toe base (the ankle without a toe)
    float thigh = 0.0f, shin = 0.0f;
};
struct PoseFrame
{
    BoneTransform par{}, pel{};
    LegFrame leg[2];
};

struct FootGoal
{
    bool active = false;
    math::Vec3 ankle{};  // ankle target
    math::Quat foot{};   // foot model rotation
    math::Vec3 ball{};   // ball after the goal (pole direction)
    float lift = 0.0f;   // m, ground height applied at the ball (after the dead zone and the weight)
    float tilt = 0.0f;   // rad, foot tilt applied
};

struct ApplyResult
{
    bool applied = false;
    int solved = 0;
    int clamped = 0;
    int kneeLimited = 0;
    float pelvis = 0.0f;      // m, total pelvis offset applied (body height + ground)
    float lift[2] = {0.0f, 0.0f};
    float tilt[2] = {0.0f, 0.0f};
    float ankleError = 0.0f;  // m, largest ankle distance from its (knee-limited) target after the solve
};

namespace detail
{
using bodyheight::detail::Load;
using bodyheight::detail::Mul;
inline float Len(math::Vec3 v) { return std::sqrt(math::Dot(v, v)); }
inline float Deg(float aDeg) { return aDeg * math::kPi / 180.0f; }

// closed-form critically damped spring of one value towards a constant goal (angular frequency w)
inline void Spring(float& x, float& v, float aGoal, float w, float dt)
{
    const float y = x - aGoal;
    const float j = v + y * w;
    const float e = std::exp(-w * dt);
    x = aGoal + (y + j * dt) * e;
    v = (v - j * (w * dt)) * e;
}
} // namespace detail

// Model-space frame of the leg chains of aPose (before any change). False for an invalid rig, another bone count or non-finite data.
inline bool Gather(const bodyheight::LegRig& aRig, const float* aPose, uint32_t aNumBones, PoseFrame& aOut)
{
    using detail::Load;
    using detail::Mul;
    if (!aRig.Valid() || !aPose || aNumBones != aRig.numBones)
        return false;
    for (int32_t i = 0; i < aRig.pathLen; ++i)
        for (int k = 0; k < 7; ++k)
            if (!std::isfinite(aPose[size_t(aRig.path[i]) * 7 + size_t(k)]))
                return false;
    BoneTransform par{};
    for (int32_t i = 0; i + 1 < aRig.pathLen; ++i)
        par = Mul(par, Load(aPose, aRig.path[i]));
    aOut.par = par;
    aOut.pel = Mul(par, Load(aPose, aRig.pelvis));
    for (int s = 0; s < 2; ++s)
    {
        LegFrame& l = aOut.leg[s];
        l.ga = Mul(aOut.pel, Load(aPose, aRig.chain[s][0]));
        l.gb = Mul(l.ga, Load(aPose, aRig.chain[s][1]));
        l.gc = Mul(l.gb, Load(aPose, aRig.chain[s][2]));
        BoneTransform t = l.gc;
        for (int32_t i = 0; i < aRig.toePathLen[s]; ++i)
            t = Mul(t, Load(aPose, aRig.toePath[s][i]));
        l.ball = t.p;
        l.thigh = detail::Len(l.gb.p - l.ga.p);
        l.shin = detail::Len(l.gc.p - l.gb.p);
        if (!math::IsFinite(l.ga.p) || !math::IsFinite(l.gc.p) || !math::IsFinite(l.ball) || !math::IsFinite(l.gc.q) ||
            !(l.thigh > 1e-3f) || !(l.shin > 1e-3f))
            return false;
    }
    return true;
}

// Foot goal of one leg on its planting plane (see the header). An inactive goal is the animated foot.
inline FootGoal Goal(const Config& c, const FootPlane& p, const LegFrame& l)
{
    FootGoal g;
    g.ankle = l.gc.p;
    g.foot = l.gc.q;
    g.ball = l.ball;
    if (!(p.w > 0.0f) || !std::isfinite(p.w) || !std::isfinite(p.h) || !std::isfinite(p.x) || !std::isfinite(p.y))
        return g;
    const float w = std::fmin(p.w, 1.0f);
    math::Vec3 n{p.nx, p.ny, p.nz};
    const float nl = detail::Len(n);
    n = (std::isfinite(nl) && nl > 1e-4f) ? n * (1.0f / nl) : math::Vec3{0.0f, 0.0f, 1.0f};
    if (!(n.z > 0.2f))
        n = math::Vec3{0.0f, 0.0f, 1.0f};
    // heading frame of the foot (ankle -> ball, horizontal)
    math::Vec3 fwd = l.ball - l.gc.p;
    fwd.z = 0.0f;
    const float fl = detail::Len(fwd);
    fwd = fl > 1e-3f ? fwd * (1.0f / fl) : math::Vec3{0.0f, 1.0f, 0.0f};
    const math::Vec3 right{fwd.y, -fwd.x, 0.0f};
    // ground height under the ball on the plane; the extrapolation from the traced point is bounded by the pitch limit
    const float dx = l.ball.x - p.x, dy = l.ball.y - p.y;
    const float dist = std::sqrt(dx * dx + dy * dy);
    float ext = -(n.x * dx + n.y * dy) / n.z;
    const float extMax = dist * std::tan(detail::Deg(c.maxPitchDeg));
    ext = std::clamp(ext, -extMax, extMax);
    float lift = p.h + ext;
    lift = (lift < 0.0f ? -1.0f : 1.0f) * std::fmax(std::fabs(lift) - c.deadHeight, 0.0f);
    lift *= w;
    // tilt: slopes along the heading and to the side, limited, then the dead angle
    const float sf = std::clamp(-math::Dot(n, fwd) / n.z, -std::tan(detail::Deg(c.maxPitchDeg)), std::tan(detail::Deg(c.maxPitchDeg)));
    const float sr = std::clamp(-math::Dot(n, right) / n.z, -std::tan(detail::Deg(c.maxRollDeg)), std::tan(detail::Deg(c.maxRollDeg)));
    const math::Vec3 nt = legik::Norm(fwd * (-sf) + right * (-sr) + math::Vec3{0.0f, 0.0f, 1.0f});
    const float a = std::acos(std::clamp(nt.z, -1.0f, 1.0f));
    const float tilt = std::fmax(a - detail::Deg(c.deadAngleDeg), 0.0f) * w;
    math::Quat r{0.0f, 0.0f, 0.0f, 1.0f};
    if (tilt > 0.0f)
    {
        const math::Vec3 axis = math::Cross(math::Vec3{0.0f, 0.0f, 1.0f}, nt);
        if (detail::Len(axis) > 1e-6f)
            r = legik::AxisAngle(legik::Norm(axis), tilt);
    }
    if (lift == 0.0f && tilt == 0.0f)
        return g;
    const math::Vec3 up{0.0f, 0.0f, lift};
    g.active = true;
    g.lift = lift;
    g.tilt = tilt;
    g.ankle = l.ball + math::QuatRotate(r, l.gc.p - l.ball) + up;
    g.foot = math::QuatNormalize(math::QuatMul(r, l.gc.q));
    g.ball = l.ball + up;
    return g;
}

// Ground pelvis target (m, <= 0): the PelvisPlacement range of the goals minus that of the animated ankles, with the hips raised by the
// body height offset aLift. Zero when no goal is active.
inline float PelvisTarget(const Config& c, const PoseFrame& f, const FootGoal (&g)[2], float aLift)
{
    if (!g[0].active && !g[1].active)
        return 0.0f;
    PelvisPlacementConfig pc;
    pc.stiffness = c.pelvisStiffness;
    pc.rebalance = 0.0f;
    pc.maxHorizontal = 0.0f;
    pc.heelLift = c.heelLift;
    pc.reach = c.softReach; // the solver is exact below the soft reach: a planted ball keeps its place
    pc.maxDrop = 1.0f;
    pc.maxRaise = 1.0f;
    PelvisLeg goal[2], anim[2];
    for (int s = 0; s < 2; ++s)
    {
        const LegFrame& l = f.leg[s];
        PelvisLeg leg;
        leg.hip = l.ga.p + math::Vec3{0.0f, 0.0f, aLift};
        leg.ankle = l.gc.p;
        leg.length = l.thigh + l.shin;
        leg.footLength = detail::Len(l.ball - l.gc.p);
        anim[s] = leg;
        anim[s].target = l.gc.p;
        goal[s] = leg;
        goal[s].target = g[s].ankle;
    }
    const float z = PelvisPlacement::TargetOffset(pc, goal).z - PelvisPlacement::TargetOffset(pc, anim).z;
    if (!std::isfinite(z))
        return 0.0f;
    return std::clamp(z, -c.maxDrop, 0.0f);
}

// Knee limit: the hip-target distance at which the knee folds kneeFreeDeg more than in the animated pose (interior angle at least
// minKneeDeg); 0 when the leg is degenerate.
inline float MinReach(const Config& c, const LegFrame& l)
{
    const float lab = l.thigh, lcb = l.shin;
    const float lac = detail::Len(l.gc.p - l.ga.p);
    const float cos0 = std::clamp((lab * lab + lcb * lcb - lac * lac) / (2.0f * lab * lcb), -1.0f, 1.0f);
    const float phi0 = std::acos(cos0);
    const float phi = std::fmax(phi0 - detail::Deg(c.kneeFreeDeg), detail::Deg(c.minKneeDeg));
    if (phi >= phi0)
        return lac; // already at the limit: no further fold
    return std::sqrt(std::fmax(lab * lab + lcb * lcb - 2.0f * lab * lcb * std::cos(phi), 0.0f));
}

// Applies the plan and the body height offset to aPose with one leg solve. Untouched when the rig / pose does not match. The caller
// runs this only for an active plan (an inactive one is the body height stage alone, bit for bit).
inline ApplyResult Apply(const Config& c, const bodyheight::LegRig& aRig, float* aPose, uint32_t aNumBones, const Plan& aPlan,
                         float aBodyOffset)
{
    using detail::Load;
    using detail::Mul;
    ApplyResult r;
    PoseFrame f;
    if (!std::isfinite(aBodyOffset) || !std::isfinite(aPlan.pelvis) || !Gather(aRig, aPose, aNumBones, f))
        return r;
    FootGoal g[2];
    for (int s = 0; s < 2; ++s)
    {
        g[s] = Goal(c, aPlan.foot[s], f.leg[s]);
        r.lift[s] = g[s].lift;
        r.tilt[s] = g[s].tilt;
    }
    if (!g[0].active && !g[1].active && aPlan.pelvis == 0.0f)
        return r; // nothing from the ground (dead zone): the pose is left to the body height stage, bit for bit
    float off = aBodyOffset + aPlan.pelvis;
    if (off > 0.0f)
    {
        // a raise only within the soft reach of each leg to its goal (the feet never lift with the pelvis)
        for (int s = 0; s < 2; ++s)
        {
            const LegFrame& l = f.leg[s];
            const math::Vec3 d = l.ga.p - g[s].ankle;
            const float lac2 = math::Dot(d, d);
            const float S = c.softReach * (l.thigh + l.shin);
            const float disc = d.z * d.z - lac2 + S * S;
            const float slack = S * S > lac2 && disc > 0.0f ? std::fmax(0.0f, -d.z + std::sqrt(disc)) : 0.0f;
            off = std::fmin(off, slack);
        }
    }
    else if (off < 0.0f)
    {
        // a drop only as far as each knee may fold over the animation with its foot where the animation put it
        for (int s = 0; s < 2; ++s)
        {
            const LegFrame& l = f.leg[s];
            const float lmin = MinReach(c, l);
            const math::Vec3 d = l.ga.p - l.gc.p;
            const float disc = d.z * d.z - math::Dot(d, d) + lmin * lmin;
            if (disc >= 0.0f)
                off = std::fmax(off, std::fmin(0.0f, -d.z + std::sqrt(disc)));
        }
    }
    r.pelvis = off;
    if (off != 0.0f)
    {
        float* pp = aPose + size_t(aRig.pelvis) * 7;
        const math::Vec3 d = math::QuatRotate(math::QuatInv(f.par.q), math::Vec3{0.0f, 0.0f, off});
        pp[0] += d.x;
        pp[1] += d.y;
        pp[2] += d.z;
    }
    const BoneTransform pel1 = Mul(f.par, Load(aPose, aRig.pelvis));
    for (int s = 0; s < 2; ++s)
    {
        const LegFrame& l = f.leg[s];
        const BoneTransform ga = Mul(pel1, Load(aPose, aRig.chain[s][0]));
        const BoneTransform gb = Mul(ga, Load(aPose, aRig.chain[s][1]));
        BoneTransform gc = Mul(gb, Load(aPose, aRig.chain[s][2]));
        gc.q = g[s].foot;
        math::Vec3 t = g[s].ankle;
        // knee limit: shorten the ground correction (towards the animated ankle) instead of folding the knee further
        const float lmin = MinReach(c, l);
        const math::Vec3 h = ga.p;
        if (g[s].active && detail::Len(t - h) < lmin)
        {
            const math::Vec3 t0 = l.gc.p, D = t - t0, e = h - t0;
            const float a = math::Dot(D, D), b = -2.0f * math::Dot(D, e), cc = math::Dot(e, e) - lmin * lmin;
            float sCut = 0.0f;
            if (a > 1e-10f && cc > 0.0f)
            {
                const float disc = b * b - 4.0f * a * cc;
                if (disc >= 0.0f)
                    sCut = std::clamp((-b - std::sqrt(disc)) / (2.0f * a), 0.0f, 1.0f);
            }
            t = t0 + D * sCut;
            ++r.kneeLimited;
        }
        const legik::TwoBoneSolution sol = legik::SolveTwoBone(pel1, ga, gb, gc, t, g[s].ball, c.reach, c.softReach, true, true);
        if (sol.clamped)
            ++r.clamped;
        if (sol.status != legik::TwoBoneStatus::kOk || !math::IsFinite(sol.ankle))
            continue;
        for (int k = 0; k < 3; ++k)
        {
            float* q = aPose + size_t(aRig.chain[s][k]) * 7 + 3;
            q[0] = sol.local[k].x;
            q[1] = sol.local[k].y;
            q[2] = sol.local[k].z;
            q[3] = sol.local[k].w;
        }
        ++r.solved;
        r.ankleError = std::fmax(r.ankleError, detail::Len(sol.ankle - t));
    }
    r.applied = true;
    return r;
}

struct StageDiag
{
    uint64_t steps = 0;
    uint64_t resets = 0;
    uint64_t sudden = 0;      // capsule snaps taken into the pelvis spring
    uint64_t rejected = 0;    // foot samples not used (no hit, inconsistent with the capsule, bad normal)
};

// Per-frame state: the planting planes, their fades and the pelvis spring.
class Stage
{
public:
    explicit Stage(const Config& aCfg = {}) : m_cfg(ValidConfig(aCfg) ? aCfg : Config{}) {}

    void SetConfig(const Config& aCfg)
    {
        if (ValidConfig(aCfg))
            m_cfg = aCfg;
    }
    const Config& Cfg() const { return m_cfg; }

    void Reset()
    {
        for (Foot& f : m_foot)
            f = Foot{};
        m_pel = m_pelV = 0.0f;
        m_havePrev = false;
        m_plan = Plan{};
    }

    // One update. aDt: s since the previous update; aSamples: the latest traces (nullptr: none); aAllowed: the stage may act; aPlatform: a
    // moving platform carries V (no ground placement); aDrawn: drawn world position of the character origin (x, y only with aHaveXY);
    // aBodyOffset: the body height offset of this frame; aRig / aPose: the pose the plan is applied to (pelvis target).
    const Plan& Step(float aDt, const Samples* aSamples, bool aAllowed, bool aPlatform, const float (&aDrawn)[3], bool aHaveXY,
                     float aBodyOffset, const bodyheight::LegRig& aRig, const float* aPose, uint32_t aNumBones)
    {
        if (!std::isfinite(aDrawn[2]) || !std::isfinite(aBodyOffset))
        {
            Reset();
            ++m_diag.resets;
            return m_plan;
        }
        bool first = !m_havePrev;
        if (!first && (!(aDt > 0.0f) || !std::isfinite(aDt)))
            return m_plan;
        if (!first && aDt > m_cfg.maxDt)
        {
            Reset();
            ++m_diag.resets;
            first = true;
        }
        const float dt = first ? 0.0f : aDt;
        ++m_diag.steps;
        const bool allowed = aAllowed && !aPlatform;
        const float wh = std::sqrt(m_cfg.heightStiffness), wn = std::sqrt(m_cfg.normalStiffness);
        const float fade = dt > 0.0f ? std::exp(-dt / m_cfg.fadeTau) : 1.0f;
        for (int s = 0; s < 2; ++s)
        {
            Foot& f = m_foot[s];
            bool valid = false;
            float g = 0.0f;
            math::Vec3 n{0.0f, 0.0f, 1.0f};
            if (allowed && aSamples)
            {
                const FootSample& fs = aSamples->foot[s];
                n = math::Vec3{fs.nx, fs.ny, fs.nz};
                const float nl = detail::Len(n);
                g = fs.z - m_cfg.bias;
                valid = fs.hit != 0 && std::isfinite(g) && std::isfinite(fs.x) && std::isfinite(fs.y) && std::isfinite(nl) && nl > 1e-3f &&
                        std::isfinite(aSamples->capZ) && std::fabs(g - aSamples->capZ) <= m_cfg.maxGroundDelta;
                if (valid)
                {
                    n = n * (1.0f / nl);
                    if (!(n.z > 0.5f)) // steeper than 60 deg (a step edge or riser under the trace): the height counts, the tilt does not
                        n = math::Vec3{0.0f, 0.0f, 1.0f};
                }
                if (!valid)
                    ++m_diag.rejected;
                else
                {
                    f.x = fs.x;
                    f.y = fs.y;
                }
            }
            if (valid)
            {
                if (!f.init)
                {
                    f.init = true;
                    f.gz = g;
                    f.gv = 0.0f;
                    f.n = n;
                    f.nv = math::Vec3{};
                }
                else if (dt > 0.0f)
                {
                    detail::Spring(f.gz, f.gv, g, wh, dt);
                    detail::Spring(f.n.x, f.nv.x, n.x, wn, dt);
                    detail::Spring(f.n.y, f.nv.y, n.y, wn, dt);
                    detail::Spring(f.n.z, f.nv.z, n.z, wn, dt);
                }
                if (f.gz < g - m_cfg.maxPenetration)
                {
                    f.gz = g - m_cfg.maxPenetration;
                    f.gv = std::fmax(f.gv, 0.0f);
                }
            }
            const float target = valid ? 1.0f : 0.0f;
            f.w = target + (f.w - target) * fade; // the first update (dt 0) keeps the weight: the stage always fades in
            if (!valid && f.w < 1e-3f)
                f.w = 0.0f;
            else if (valid && f.w > 0.999f)
                f.w = 1.0f;
            if (f.w == 0.0f && !valid)
                f.init = false;
            FootPlane& p = m_plan.foot[s];
            if (f.init && f.w > 0.0f)
            {
                const float l = detail::Len(f.n);
                const math::Vec3 nn = l > 1e-4f ? f.n * (1.0f / l) : math::Vec3{0.0f, 0.0f, 1.0f};
                p = FootPlane{f.w, f.x, f.y, f.gz - aDrawn[2], nn.x, nn.y, nn.z};
            }
            else
                p = FootPlane{};
        }
        // sudden capsule motion: the drawn height change beyond the slope under the feet goes into the pelvis spring
        const bool anyFoot = m_plan.foot[0].w > 0.0f || m_plan.foot[1].w > 0.0f;
        const float drawnZ = aDrawn[2] + aBodyOffset;
        if (!first && m_havePrev && anyFoot && allowed)
        {
            float expected = 0.0f;
            if (aHaveXY && aSamples && std::isfinite(aDrawn[0]) && std::isfinite(aDrawn[1]))
            {
                math::Vec3 n{};
                for (const FootPlane& p : m_plan.foot)
                    n = n + math::Vec3{p.nx, p.ny, p.nz} * p.w;
                const float nl = detail::Len(n);
                if (nl > 1e-4f && n.z / nl > 0.5f)
                {
                    float mx = 0.0f, my = 0.0f;
                    math::Rot2(-aSamples->yaw, aDrawn[0] - m_prev[0], aDrawn[1] - m_prev[1], mx, my); // world step -> model axes
                    expected = -(n.x * mx + n.y * my) / n.z;
                }
            }
            const float sudden = (drawnZ - m_prev[2]) - expected;
            if (std::fabs(sudden) > m_cfg.suddenStep)
            {
                m_pel = std::clamp(m_pel - sudden, -m_cfg.maxCompensation, m_cfg.maxCompensation);
                ++m_diag.sudden;
            }
        }
        // pelvis spring towards the ground target
        float target = 0.0f;
        if (allowed && anyFoot)
        {
            PoseFrame pf;
            if (Gather(aRig, aPose, aNumBones, pf))
            {
                FootGoal g[2];
                for (int s = 0; s < 2; ++s)
                    g[s] = Goal(m_cfg, m_plan.foot[s], pf.leg[s]);
                target = PelvisTarget(m_cfg, pf, g, aBodyOffset);
            }
        }
        if (dt > 0.0f)
            detail::Spring(m_pel, m_pelV, target, std::sqrt(m_cfg.pelvisStiffness), dt);
        if (!std::isfinite(m_pel) || !std::isfinite(m_pelV))
            m_pel = m_pelV = 0.0f;
        m_pel = std::clamp(m_pel, -std::fmax(m_cfg.maxDrop, m_cfg.maxCompensation), m_cfg.maxCompensation);
        if (target == 0.0f && std::fabs(m_pel) < 1e-4f && std::fabs(m_pelV) < 1e-3f)
            m_pel = m_pelV = 0.0f;
        m_plan.pelvis = m_pel;
        m_plan.active = (anyFoot || m_pel != 0.0f) ? 1u : 0u;
        m_prev[0] = aDrawn[0];
        m_prev[1] = aDrawn[1];
        m_prev[2] = drawnZ;
        m_havePrev = true;
        return m_plan;
    }

    const Plan& Current() const { return m_plan; }
    float PelvisVelocity() const { return m_pelV; }
    const StageDiag& Diagnostics() const { return m_diag; }

private:
    struct Foot
    {
        bool init = false;
        float gz = 0.0f, gv = 0.0f; // planting plane height (world) and its rate
        math::Vec3 n{0.0f, 0.0f, 1.0f}, nv{};
        float w = 0.0f;
        float x = 0.0f, y = 0.0f;   // traced point (model frame)
    };

    Config m_cfg;
    Foot m_foot[2];
    float m_pel = 0.0f, m_pelV = 0.0f;
    bool m_havePrev = false;
    float m_prev[3] = {0.0f, 0.0f, 0.0f};
    Plan m_plan;
    StageDiag m_diag;
};

} // namespace utpp::mm::groundfeet
