// UTPP motion matching runtime - feet on uneven ground (stairs, slopes, curbs, rubble). Header-only, C++20, deterministic,
// allocation-free.
//
// Ground traces (two per foot, taken by the game script under the ball and the heel of the drawn foot of the previous frame) give a
// world hit height and a surface normal per foot. This stage places the feet on that ground and lets the whole leg chain adapt:
//   - support under the sole: the ball trace sets the ground unless the heel trace hits more than heelAbove above the ball's plane
//     there (stairs down: the ball reaches past the step nose to the step below while the heel stands on the tread; a bump under the
//     heel); then the heel hit is the ground of that foot (level), so a planted foot rests on the tread it stands on and a swinging
//     one keeps its clearance above the higher step.
//   - planting plane per foot: the hit height follows the trace through a critically damped spring (stiffness heightStiffness) and
//     the normal through another one (normalStiffness), both in closed form of dt; the smoothed plane may stay at most
//     maxPenetration below the traced ground. The plane is held in world height, so a snap of the drawn capsule never moves a foot
//     in the world.
//   - a foot near the clip's floor (ball at most swingLow above the floor estimate: the lowest ball of either foot, rising at most
//     floorRise per second) keeps the clip's height above the model ground plane, carried onto its planting plane: the ball (toe
//     base) is lifted by the plane height under it and the foot turns about the ball by the tilt of the plane, limited to
//     maxPitchDeg / maxRollDeg in the foot's heading frame (a planted toe stays where the lock put it). Higher in the swing (up to
//     swingHigh) that carry fades out: an airborne foot is neither tilted nor lowered towards the ground under it and is raised only
//     as far as it needs to stay swingClearance above that ground (a step ahead), so the knee keeps the animation's fold.
//   - dead zone: deadHeight is subtracted from the height and deadAngleDeg from the tilt (continuous at the threshold); on flat
//     ground both are zero and the stage hands the pose to the body height stage untouched (bit for bit).
//   - pelvis: the PelvisPlacement range (no raise; while moving only the front leg, standing all legs) of the ground targets minus
//     the same range for the animated ankles, so only a ground difference lowers the pelvis; a critically damped spring (pelvisStiffness), added to the body height offset.
//   - sudden capsule motion: a change of the drawn height between two updates beyond what the slope under the feet or the last steady
//     vertical rate explains (above suddenStep) is taken into the pelvis spring, so the body does not jump with the capsule and
//     springs back instead.
//   - one leg solve (shared two-bone solver) per frame for the body height offset, the pelvis offset and the foot goals. A leg
//     whose knee would fold more than kneeFreeDeg (kneeFreeClimbDeg while the capsule climbs) over the animation gets a shorter ground
//     correction instead of a broken knee.
// The stage fades in / out with fadeTau per foot (a missing or inconsistent trace: hit more than maxGroundDelta above or maxGroundDrop
// below the capsule; down to maxGroundDropFollow while the drawn capsule descends steadily: a stair down, not a ledge ahead on level
// ground; up to maxGroundRiseFollow while it climbs steadily: the next steps of a stair a running foot lands on ahead of the capsule).
// Model space: character frame, z up (the frame is turned about z only, so model z is world z).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "BodyHeightFilter.hpp"
#include "LegIk.hpp"
#include "MMMath.hpp"
#include "PelvisPlacement.hpp"
#include "StairSteps.hpp"

namespace utpp::mm::groundfeet
{

struct Config
{
    float heightStiffness = 1000.0f; // planting plane height spring
    float normalStiffness = 450.0f;  // planting plane normal spring
    float maxPenetration = 0.05f;    // m, the smoothed plane stays at most this below the traced ground
    float deadHeight = 0.005f;       // m, subtracted from the ground height under the ball
    float deadAngleDeg = 2.0f;       // deg, subtracted from the plane tilt
    float maxPitchDeg = 25.0f;       // deg, foot pitch to the surface (heading frame)
    float maxRollDeg = 15.0f;        // deg, foot roll to the surface
    float kneeFreeDeg = 60.0f;       // deg, extra knee flexion over the animation
    float kneeFreeClimbDeg = 90.0f;  // deg, ... while the drawn capsule climbs steadily (a foot landing on a step ahead of the capsule)
    float minKneeDeg = 25.0f;        // deg, smallest interior knee angle
    float maxGroundDelta = 0.35f;    // m, a hit higher above the capsule height is not used (car roof, crate, body)
    float maxGroundDrop = 0.35f;     // m, a hit deeper below the capsule height is used only while the drawn capsule descends steadily
    float maxGroundDropFollow = 0.55f; // m, ... down to this depth (a step down a stair seen ahead of the descending capsule; on level
    float minDescentRate = 0.2f;     // m/s, ... ground a ledge ahead is not followed), descending = steady rate below -minDescentRate
    float maxGroundRiseFollow = 0.55f; // m, a hit up to this high above the capsule is used while the drawn capsule climbs steadily
    float minClimbRate = 0.2f;       // m/s, ... (a steady rate above minClimbRate; the script traces up to 0.6 m above the capsule)
    float swingLow = 0.02f;          // m, ball height above the floor estimate up to which the ground carry is full
    float swingHigh = 0.08f;         // m, ... from which only the clearance is kept
    float swingClearance = 0.03f;    // m, smallest height of a swinging ball above the ground under it
    float floorRise = 0.05f;         // m/s, largest rise of the floor estimate
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
    float frontLegSpeed = 0.5f;      // m/s, above it only the front leg sets the pelvis drop
    float suddenRateTau = 0.1f;      // s, how fast a snap update moves the expected vertical rate
    float bias = 0.0f;               // m, ground hit minus capsule height on flat ground (calibration)
    float heelAbove = 0.01f;         // m, a heel hit this far above the ball's plane at the heel is the support of that foot
    float flightDropFollow = 0.9f;   // m, on a stair flight going down (Samples::stair) a hit this deep below the capsule is used
    float descentBase = 0.3f;        // pelvis lowering on a flight going down: share of the rise (stairsteps::swing::DescentDrop) ...
    float descentPerTread = 0.15f;   // ... plus this share per tread of the fitted step (at most 4)
    float flightRateTau = 0.08f;     // s, on a stair flight the steady vertical rate of the drawn capsule is smoothed with this
    float landVMax = 0.5f;           // m/s, largest plane rate a landing swing hands on (StairSwings)
    float swingStart = 0.04f;        // m, ball height above the floor estimate from which a planned stair swing starts (above
                                     // swingLow: a ball wavering around swingLow in the stance does not start and land swings)
    float landAbove = 0.03f;         // m, highest a landing swing's plane may stay above the foot's own trace (StairSwings)
    float swingFadeS = 0.1f;         // s, full-scale fade of a planned swing's weight (a lost flight, the option switched off)
};

inline bool ValidConfig(const Config& c)
{
    auto pos = [](float v) { return std::isfinite(v) && v > 0.0f; };
    auto nonneg = [](float v) { return std::isfinite(v) && v >= 0.0f; };
    return pos(c.heightStiffness) && pos(c.normalStiffness) && nonneg(c.maxPenetration) && nonneg(c.deadHeight) &&
           nonneg(c.deadAngleDeg) && pos(c.maxPitchDeg) && c.maxPitchDeg < 80.0f && pos(c.maxRollDeg) && c.maxRollDeg < 80.0f &&
           nonneg(c.kneeFreeDeg) && nonneg(c.kneeFreeClimbDeg) && c.kneeFreeClimbDeg < 150.0f && nonneg(c.minKneeDeg) && pos(c.maxGroundDelta) && pos(c.maxGroundDrop) && pos(c.fadeTau) &&
           std::isfinite(c.maxGroundDropFollow) && c.maxGroundDropFollow >= c.maxGroundDrop && pos(c.minDescentRate) &&
           std::isfinite(c.maxGroundRiseFollow) && c.maxGroundRiseFollow >= c.maxGroundDelta && pos(c.minClimbRate) &&
           nonneg(c.swingLow) && std::isfinite(c.swingHigh) && c.swingHigh > c.swingLow && nonneg(c.swingClearance) &&
           pos(c.floorRise) && pos(c.maxDt) &&
           pos(c.pelvisStiffness) && nonneg(c.maxDrop) && nonneg(c.maxCompensation) && nonneg(c.heelLift) && c.heelLift <= 1.0f &&
           std::isfinite(c.reach) && c.reach > 0.5f && c.reach <= 1.0f && std::isfinite(c.softReach) && c.softReach > 0.5f &&
           c.softReach <= c.reach && pos(c.suddenStep) && pos(c.frontLegSpeed) &&
           pos(c.suddenRateTau) && std::isfinite(c.bias) && std::fabs(c.bias) < 0.2f && nonneg(c.heelAbove) &&
           std::isfinite(c.flightDropFollow) && c.flightDropFollow >= c.maxGroundDrop && nonneg(c.descentBase) &&
           nonneg(c.descentPerTread) && pos(c.flightRateTau) && nonneg(c.landVMax) && nonneg(c.landAbove) && pos(c.swingFadeS) &&
           std::isfinite(c.swingStart) && c.swingStart >= c.swingLow;
}

// One trace per foot under the ball (foot) and one under the heel (heel; hit 0 = not traced), in the model frame of the capsule the
// traces were taken with (x, y: the traced point; n: the surface normal) plus the world height of the hit.
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
    FootSample heel[2];
    stairsteps::Flight stair; // the stair flight under V (option mm.stairSteps; valid 0 = none): the swing over it (Stage)
    float capX = 0.0f, capY = 0.0f; // world position of the trace capsule (set with a stair flight only)
};
static_assert(std::is_trivially_copyable_v<Samples> && sizeof(Samples) % 4 == 0);

// Planting plane of a foot in the model frame of the drawn capsule (height h at (x, y), unit normal n), its fade weight, the share of
// the ground carry (1 = near the floor; 0 = high in the swing: clearance only) and the ball height above the floor estimate (m).
struct FootPlane
{
    float w = 0.0f;
    float x = 0.0f, y = 0.0f, h = 0.0f;
    float nx = 0.0f, ny = 0.0f, nz = 1.0f;
    float carry = 1.0f;
    float above = 0.0f;
    float planLift = 0.0f; // m, ground height at the ball of a foot swinging over a stair flight (stairsteps::swing), model
    float planW = 0.0f;    // its share of the lift [0, 1] (0 = none: the plane above alone, bit for bit)
};
struct Plan
{
    FootPlane foot[2];
    float pelvis = 0.0f;  // m, ground pelvis offset (added to the body height offset)
    uint32_t active = 0;  // 1 = the plan changes the pose (some foot weight > 0 or a pelvis offset)
    float kneeFreeDeg = 0.0f; // deg, extra knee flexion allowed for this plan (0 = Config::kneeFreeDeg)
};
static_assert(std::is_trivially_copyable_v<Plan> && sizeof(Plan) % 4 == 0);

struct LegFrame
{
    BoneTransform ga{}, gb{}, gc{}; // UpLeg, Leg, Foot (model space)
    math::Vec3 ball{};              // toe base (the ankle without a toe)
    math::Vec3 heel{};              // heel (the first bone below the foot on the way to the toe base; the ankle without one)
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
        l.heel = l.gc.p;
        for (int32_t i = 0; i < aRig.toePathLen[s]; ++i)
        {
            t = Mul(t, Load(aPose, aRig.toePath[s][i]));
            if (i == 0 && aRig.toePathLen[s] >= 2)
                l.heel = t.p;
        }
        l.ball = t.p;
        l.thigh = detail::Len(l.gb.p - l.ga.p);
        l.shin = detail::Len(l.gc.p - l.gb.p);
        if (!math::IsFinite(l.ga.p) || !math::IsFinite(l.gc.p) || !math::IsFinite(l.ball) || !math::IsFinite(l.heel) || !math::IsFinite(l.gc.q) ||
            !(l.thigh > 1e-3f) || !(l.shin > 1e-3f))
            return false;
    }
    return true;
}

// Foot goal of one leg on its planting plane (see the header). An inactive goal is the animated foot. aFollow: the vertical offset of
// the pelvis this frame (m); the airborne share of a swinging foot moves with it, so the knee keeps the animation's fold.
inline FootGoal Goal(const Config& c, const FootPlane& p, const LegFrame& l, float aFollow = 0.0f)
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
    // swing: the carry share of the lift, but never less than keeps the ball swingClearance above the ground under it
    const float carry = std::isfinite(p.carry) ? std::clamp(p.carry, 0.0f, 1.0f) : 1.0f;
    if (carry < 1.0f)
    {
        const float above = std::isfinite(p.above) ? std::fmax(p.above, 0.0f) : 0.0f;
        const float follow = std::isfinite(aFollow) ? aFollow : 0.0f;
        lift = std::fmax(carry * lift + (1.0f - carry) * follow, lift - std::fmax(above - c.swingClearance, 0.0f));
    }
    // a foot swinging over a stair flight: its planned ground (the same dead zone and weight), untilted
    const float planW = std::isfinite(p.planW) && std::isfinite(p.planLift) ? std::clamp(p.planW, 0.0f, 1.0f) : 0.0f;
    if (planW > 0.0f)
    {
        const float pl = (p.planLift < 0.0f ? -1.0f : 1.0f) * std::fmax(std::fabs(p.planLift) - c.deadHeight, 0.0f) * w;
        lift += (pl - lift) * planW;
    }
    // tilt: slopes along the heading and to the side, limited, then the dead angle
    const float sf = std::clamp(-math::Dot(n, fwd) / n.z, -std::tan(detail::Deg(c.maxPitchDeg)), std::tan(detail::Deg(c.maxPitchDeg)));
    const float sr = std::clamp(-math::Dot(n, right) / n.z, -std::tan(detail::Deg(c.maxRollDeg)), std::tan(detail::Deg(c.maxRollDeg)));
    const math::Vec3 nt = legik::Norm(fwd * (-sf) + right * (-sr) + math::Vec3{0.0f, 0.0f, 1.0f});
    const float a = std::acos(std::clamp(nt.z, -1.0f, 1.0f));
    float tilt = std::fmax(a - detail::Deg(c.deadAngleDeg), 0.0f) * w * carry;
    if (planW > 0.0f)
        tilt *= 1.0f - planW;
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
// aFrontLeg: -1 = every leg sets the pelvis; 0 / 1 = only that (front) leg does, the other keeps the animation's own need (while moving,
// the trailing leg does not pull the pelvis down a step; it reaches as far as the leg allows)
inline float PelvisTarget(const Config& c, const PoseFrame& f, const FootGoal (&g)[2], float aLift, int aFrontLeg = -1)
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
        goal[s].target = (aFrontLeg < 0 || aFrontLeg == s) ? g[s].ankle : l.gc.p;
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
inline ApplyResult Apply(const Config& aCfg, const bodyheight::LegRig& aRig, float* aPose, uint32_t aNumBones, const Plan& aPlan,
                         float aBodyOffset)
{
    using detail::Load;
    using detail::Mul;
    Config c = aCfg;
    if (std::isfinite(aPlan.kneeFreeDeg) && aPlan.kneeFreeDeg > 0.0f && aPlan.kneeFreeDeg < 150.0f)
        c.kneeFreeDeg = aPlan.kneeFreeDeg;
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
    // a swinging foot moves with the pelvis by its airborne share (never closer to the ground than its clearance)
    for (int s = 0; s < 2; ++s)
        if (off != 0.0f && aPlan.foot[s].carry < 1.0f)
        {
            g[s] = Goal(c, aPlan.foot[s], f.leg[s], off);
            r.lift[s] = g[s].lift;
            r.tilt[s] = g[s].tilt;
        }
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
    uint64_t rejected = 0;    // feet without a usable sample (no hit, inconsistent with the capsule, bad normal)
    uint64_t heel = 0;        // feet whose heel sample was the support
    uint64_t stairSwings = 0; // swings planned over a stair flight (StairSwings)
    uint64_t stairLands = 0;  // ... and their landings
    uint64_t landClamps = 0;  // ... landings whose plane the foot's trace bounded (landAbove / maxPenetration) and the last
    float landClampLast = 0.0f; //    such correction (m)
};

// Deepest hit below the capsule (m) a trace may have on a stair flight going down: the flight's drop to the traced point (its
// distance ahead of the capsule along the flight, in treads, plus one and a half rises for the tread under the capsule and the
// sphere of the capsule) and 5 cm, at most Config::flightDropFollow (a hit deeper than the flight explains, a gap in the flight, is
// not used).
inline float FlightDropLimit(const stairsteps::Flight& f, const FootSample& s, float aYaw, float aMax = 0.9f)
{
    if (!(f.tread > 1.0e-3f) || !std::isfinite(f.rise) || !std::isfinite(aYaw) || !std::isfinite(s.x) || !std::isfinite(s.y))
        return 0.0f;
    float wx = 0.0f, wy = 0.0f;
    math::Rot2(aYaw, s.x, s.y, wx, wy);
    const float ahead = std::fmax(wx * f.dirX + wy * f.dirY, 0.0f);
    return std::fmin(std::fabs(f.rise) * (ahead / f.tread + 1.5f) + 0.05f, aMax);
}

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
        m_pel = m_pelV = m_vz = 0.0f;
        m_move[0] = 0.0f;
        m_move[1] = 1.0f;
        m_havePrev = false;
        m_haveFloor = false;
        m_floor = 0.0f;
        m_haveFloorHeel = false;
        m_floorHeel = 0.0f;
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
        // a stair flight under V (option mm.stairSteps; none with the option off)
        const stairsteps::Flight* fl = allowed && aSamples && aSamples->stair.valid != 0 && aSamples->stair.weight > 0.0f &&
                                               aSamples->stair.tread > 1.0e-3f && std::isfinite(aSamples->stair.rise)
                                           ? &aSamples->stair
                                           : nullptr;
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
                // one sample: usable (hit, finite, consistent with the capsule), its height and normal (steeper than 60 deg: a step edge
                // or riser under the trace, the height counts, the tilt does not)
                auto use = [&](const FootSample& fs, float& aG, math::Vec3& aN) {
                    aN = math::Vec3{fs.nx, fs.ny, fs.nz};
                    const float nl = detail::Len(aN);
                    aG = fs.z - m_cfg.bias;
                    const float drop = aSamples->capZ - aG;
                    const bool ok = fs.hit != 0 && std::isfinite(aG) && std::isfinite(fs.x) && std::isfinite(fs.y) && std::isfinite(nl) &&
                                    nl > 1e-3f && std::isfinite(aSamples->capZ) &&
                                    (aG - aSamples->capZ <= m_cfg.maxGroundDelta ||
                                     (aG - aSamples->capZ <= m_cfg.maxGroundRiseFollow && m_vz > m_cfg.minClimbRate)) &&
                                    (drop <= m_cfg.maxGroundDrop ||
                                     (drop <= m_cfg.maxGroundDropFollow && m_vz < -m_cfg.minDescentRate) ||
                                     (fl && fl->rise < 0.0f && drop <= FlightDropLimit(*fl, fs, aSamples->yaw, m_cfg.flightDropFollow)));
                    if (ok)
                    {
                        aN = aN * (1.0f / nl);
                        if (!(aN.z > 0.5f))
                            aN = math::Vec3{0.0f, 0.0f, 1.0f};
                    }
                    return ok;
                };
                const FootSample& fb = aSamples->foot[s];
                const FootSample& fh = aSamples->heel[s];
                float gh = 0.0f;
                math::Vec3 nh{};
                valid = use(fb, g, n);
                const bool heelOk = use(fh, gh, nh);
                // the support under the sole: the heel hit when it is above the ball's plane at the heel (stairs down: the ball past the
                // step nose, the heel on the tread), level
                bool heel = heelOk && !valid;
                if (heelOk && valid)
                {
                    const float dx = fh.x - fb.x, dy = fh.y - fb.y;
                    heel = gh > g - (n.x * dx + n.y * dy) / n.z + m_cfg.heelAbove;
                }
                if (heel)
                {
                    valid = true;
                    g = gh;
                    n = math::Vec3{0.0f, 0.0f, 1.0f};
                    f.x = fh.x;
                    f.y = fh.y;
                    ++m_diag.heel;
                }
                else if (valid)
                {
                    f.x = fb.x;
                    f.y = fb.y;
                }
                if (!valid)
                    ++m_diag.rejected;
                // a stair flight: the trace of this foot in the world (its own landing, the support's tread: StairSwings)
                f.trOk = fl && valid && std::isfinite(aSamples->yaw);
                if (f.trOk)
                {
                    float wx = 0.0f, wy = 0.0f;
                    math::Rot2(aSamples->yaw, f.x, f.y, wx, wy);
                    f.trG = g;
                    f.trWX = aSamples->capX + wx;
                    f.trWY = aSamples->capY + wy;
                }
            }
            else
                f.trOk = false;
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
            // a steady vertical rate (stairs, a ramp the trace normal does not show) is expected motion too: only the part of the
            // change that neither the slope nor the last steady rate explains is a snap
            // on a stair flight the slope is the flight's (rise over tread along its direction, weighted): its climb or descent is
            // expected, only the rest is a snap
            if (fl && aHaveXY && std::isfinite(aDrawn[0]) && std::isfinite(aDrawn[1]))
            {
                const float along = (aDrawn[0] - m_prev[0]) * fl->dirX + (aDrawn[1] - m_prev[1]) * fl->dirY;
                expected = along * fl->rise / fl->tread * std::clamp(fl->weight, 0.0f, 1.0f);
            }
            const float dz = drawnZ - m_prev[2];
            const float bySlope = dz - expected, byRate = dz - m_vz * dt;
            const float sudden = std::fabs(bySlope) <= std::fabs(byRate) ? bySlope : byRate;
            if (fl && std::fabs(sudden) <= m_cfg.suddenStep)
                m_vz += (dz / dt - m_vz) * (1.0f - std::exp(-dt / m_cfg.flightRateTau)); // steady rate, smoothed (knee gate)
            else if (std::fabs(sudden) > m_cfg.suddenStep)
            {
                m_pel = std::clamp(m_pel - sudden, -m_cfg.maxCompensation, m_cfg.maxCompensation);
                ++m_diag.sudden;
                // a snap moves the rate only a little (time constant suddenRateTau), so a single snap is not expected again, while a
                // steady rate above suddenStep per update (a fast descent at a low frame rate) becomes expected within a few updates
                m_vz += (dz / dt - m_vz) * (1.0f - std::exp(-dt / m_cfg.suddenRateTau));
            }
            else
                m_vz = dz / dt;
        }
        else if (first || !m_havePrev)
            m_vz = 0.0f;
        // front leg while moving (model axes): the leg whose ball is further along the drawn motion of this update
        int front = -1;
        if (!first && m_havePrev && aHaveXY && aSamples && dt > 0.0f && std::isfinite(aDrawn[0]) && std::isfinite(aDrawn[1]))
        {
            float mx = 0.0f, my = 0.0f;
            math::Rot2(-aSamples->yaw, aDrawn[0] - m_prev[0], aDrawn[1] - m_prev[1], mx, my);
            const float d = std::sqrt(mx * mx + my * my);
            if (d > m_cfg.frontLegSpeed * dt)
                front = 0; // chosen below from the pose
            m_move[0] = mx / std::fmax(d, 1e-6f);
            m_move[1] = my / std::fmax(d, 1e-6f);
        }
        // pelvis spring towards the ground target
        float target = 0.0f;
        if (allowed && anyFoot)
        {
            PoseFrame pf;
            if (Gather(aRig, aPose, aNumBones, pf))
            {
                // floor estimate of the clip (its ground plane: the lowest ball of either foot, rising slowly) and the swing carry
                const float low = std::fmin(pf.leg[0].ball.z, pf.leg[1].ball.z);
                m_floor = (!m_haveFloor || !std::isfinite(m_floor)) ? low : std::fmin(low, m_floor + m_cfg.floorRise * dt);
                m_haveFloor = std::isfinite(m_floor);
                for (int s = 0; s < 2; ++s)
                {
                    const float above = m_haveFloor ? std::fmax(pf.leg[s].ball.z - m_floor, 0.0f) : 0.0f;
                    const float u = std::clamp((above - m_cfg.swingLow) / (m_cfg.swingHigh - m_cfg.swingLow), 0.0f, 1.0f);
                    m_plan.foot[s].carry = 1.0f - u * u * (3.0f - 2.0f * u);
                    m_plan.foot[s].above = above;
                }
                if (fl || m_foot[0].sw || m_foot[1].sw)
                    StairSwings(pf, aSamples, fl, aDrawn, aHaveXY, dt);
                else
                {
                    for (Foot& f : m_foot)
                        f.wasDown = f.sup = f.landOk = false; // off a flight: a later flight plans from fresh landings only
                    m_stepEst = 0.0f;
                }
                FootGoal g[2];
                for (int s = 0; s < 2; ++s)
                    g[s] = Goal(m_cfg, m_plan.foot[s], pf.leg[s]);
                if (front >= 0)
                {
                    const float d0 = pf.leg[0].ball.x * m_move[0] + pf.leg[0].ball.y * m_move[1];
                    const float d1 = pf.leg[1].ball.x * m_move[0] + pf.leg[1].ball.y * m_move[1];
                    front = d1 > d0 ? 1 : 0;
                }
                target = PelvisTarget(m_cfg, pf, g, aBodyOffset, front);
                if (fl)
                    target = std::fmin(target, stairsteps::swing::DescentDrop(*fl, m_cfg.descentBase, m_cfg.descentPerTread, m_cfg.maxDrop));
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
        m_plan.kneeFreeDeg = m_vz > m_cfg.minClimbRate ? m_cfg.kneeFreeClimbDeg : m_cfg.kneeFreeDeg;
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
        // swing over a stair flight (StairSwings)
        bool wasDown = false;                 // the ball was on the clip's floor in the last update
        bool sw = false;                      // a planned swing is running
        stairsteps::Flight swFl;              // the flight latched at its lift-off
        stairsteps::swing::Plan swPlan;
        float swG = 0.0f, swGv = 0.0f;        // planned ground (world) and its rate
        float swReq = 0.0f, swReqV = 0.0f;    // the swing's height requirement (world, SwingReq) followed: a lower one critically
                                              // damped, a higher one at once (and its rate)
        float swPath = 0.0f;                  // planned glide (world) and the mid-sole / ball over the clip's floor at the last step
        float swPrevS = 0.0f, swPrevAbove = 0.0f;
        float swW = 0.0f;                     // its weight (the flight's)
        bool sup = false;                     // the last planted point (world x, y: the traced point the plane height came from, else
        float supX = 0.0f, supY = 0.0f, supH = 0.0f; // the mid-sole) and plane height (support of the other foot)
        bool trOk = false;                    // a stair flight: this frame's trace of the foot (world height and point)
        float trG = 0.0f, trWX = 0.0f, trWY = 0.0f;
        bool landOk = false;                  // its last landing on the flight (world x, y of the support point)
        float landX = 0.0f, landY = 0.0f;
    };

    // The end of a planned swing (its landing, or an abort): the planting plane goes on from the planned height and a bounded rate
    // (the swing's weight of each), never deeper than maxPenetration below this frame's trace.
    void HandSwing(Foot& f, FootPlane& p, const float (&aDrawn)[3])
    {
        const float w = std::clamp(f.swW, 0.0f, 1.0f);
        if (f.init && std::isfinite(f.swG) && std::isfinite(f.swGv))
        {
            f.gz += (f.swG - f.gz) * w;
            f.gv += (std::clamp(f.swGv, -m_cfg.landVMax, m_cfg.landVMax) - f.gv) * w;
            float clamp = 0.0f;
            if (f.trOk && f.gz < f.trG - m_cfg.maxPenetration)
            {
                clamp = f.trG - m_cfg.maxPenetration - f.gz;
                f.gz = f.trG - m_cfg.maxPenetration;
                f.gv = std::fmax(f.gv, 0.0f);
            }
            else if (f.trOk && f.gz > f.trG + m_cfg.landAbove)
            {
                clamp = f.gz - f.trG - m_cfg.landAbove;
                f.gz = f.trG + m_cfg.landAbove; // never lands in the air over its own trace
                f.gv = std::fmin(f.gv, 0.0f);
            }
            if (clamp > 0.0f)
            {
                ++m_diag.landClamps;
                m_diag.landClampLast = clamp;
            }
            if (p.w > 0.0f)
                p.h = f.gz - aDrawn[2];
        }
        f.sw = false;
        f.swW = 0.0f;
    }

    // Swings over a stair flight (stairsteps::swing): at the lift-off of a foot (its ball leaves the clip's floor) the tread it lands on
    // is planned from the other foot's tread and the fitted step; while it swings its ground glides from the tread it left to that
    // tread (FootPlane::planLift / planW); at the landing the planting plane goes on from the planned height.
    void StairSwings(const PoseFrame& pf, const Samples* aSamples, const stairsteps::Flight* fl, const float (&aDrawn)[3], bool aHaveXY,
                     float dt)
    {
        namespace sw = stairsteps::swing;
        const sw::Config sc{};
        const float lowH = std::fmin(pf.leg[0].heel.z, pf.leg[1].heel.z);
        m_floorHeel = (!m_haveFloorHeel || !std::isfinite(m_floorHeel)) ? lowH : std::fmin(lowH, m_floorHeel + m_cfg.floorRise * dt);
        m_haveFloorHeel = std::isfinite(m_floorHeel);
        const bool xy = aHaveXY && aSamples && std::isfinite(aDrawn[0]) && std::isfinite(aDrawn[1]) && std::isfinite(aSamples->yaw);
        const float cy = xy ? std::cos(aSamples->yaw) : 1.0f, sy = xy ? std::sin(aSamples->yaw) : 0.0f;
        auto wx = [&](const math::Vec3& v) { return aDrawn[0] + cy * v.x - sy * v.y; };
        auto wy = [&](const math::Vec3& v) { return aDrawn[1] + sy * v.x + cy * v.y; };
        // landings first (both feet): a foot on the clip's floor ends its swing and becomes the support of the other one; a new contact
        // measures the step from the other foot's last landing along the flight
        bool landed[2] = {false, false};
        for (int s = 0; s < 2 && xy; ++s)
        {
            Foot& f = m_foot[s];
            FootPlane& p = m_plan.foot[s];
            if (!(p.above <= m_cfg.swingLow))
                continue;
            if (f.sw)
            {
                HandSwing(f, p, aDrawn); // landing: the planting plane goes on from the planned height
                ++m_diag.stairLands;
            }
            // the support of the other foot: the point its plane height was traced at (its tread), else the mid-sole
            const math::Vec3 mid = (pf.leg[s].ball + pf.leg[s].heel) * 0.5f;
            f.sup = f.init;
            f.supX = f.trOk ? f.trWX : wx(mid);
            f.supY = f.trOk ? f.trWY : wy(mid);
            f.supH = f.trOk ? f.trG : f.gz; // the traced height of its tread (a plane still settling does not carry over)
            f.swW = 0.0f;
            landed[s] = !f.wasDown;
            f.wasDown = true;
            if (landed[s])
            {
                const Foot& o = m_foot[1 - s];
                if (fl && o.landOk)
                {
                    const float st = stairsteps::swing::Along(*fl, f.supX, f.supY) - stairsteps::swing::Along(*fl, o.landX, o.landY);
                    if (st > 0.5f * fl->tread && st < 3.0f)
                        m_stepEst = st;
                }
                f.landOk = true;
                f.landX = f.supX;
                f.landY = f.supY;
            }
        }
        for (int s = 0; s < 2; ++s)
        {
            Foot& f = m_foot[s];
            FootPlane& p = m_plan.foot[s];
            const LegFrame& l = pf.leg[s];
            if (!xy)
            {
                if (f.sw)
                {
                    p.planW = f.swW;
                    p.planLift = f.swG - aDrawn[2];
                }
                continue;
            }
            const math::Vec3 mid = (l.ball + l.heel) * 0.5f;
            const float mx = wx(mid), my = wy(mid);
            if (p.above <= m_cfg.swingLow)
                continue; // landed (above)
            if (!f.sw && p.above <= m_cfg.swingStart)
                continue; // not lifted far enough yet: still down
            // the toe tip in 3D (ahead of the ball along the foot) and its own height over the clip's floor
            math::Vec3 toeDir = l.ball - l.heel;
            const float tl = detail::Len(toeDir);
            toeDir = tl > 1.0e-3f ? toeDir * (1.0f / tl) : math::Vec3{0.0f, 1.0f, 0.0f};
            const math::Vec3 toe = l.ball + toeDir * sc.toe;
            const Foot& o = m_foot[1 - s];
            if (!f.sw && f.wasDown && fl && f.init)
            {
                // the other foot supports the body when it stands now (a walk); in the flight phase of a run it does not
                const bool support = o.sup && o.wasDown;
                sw::Plan plan;
                const float sOwn = f.trOk ? sw::Along(*fl, f.trWX, f.trWY) : NAN;
                if (sw::PlanSwing(sc, *fl, sw::Along(*fl, mx, my), f.gz, sw::Along(*fl, o.supX, o.supY), o.supH, support, plan, sOwn,
                                  m_stepEst > 0.0f ? m_stepEst : NAN, f.landOk ? sw::Along(*fl, f.landX, f.landY) : NAN))
                {
                    plan.capFrom = sw::CapStart(sc, *fl, plan, sw::Along(*fl, wx(toe), wy(toe)) - sw::Along(*fl, mx, my));
                    f.sw = true;
                    f.swFl = *fl;
                    f.swPlan = plan;
                    f.swG = f.gz;
                    f.swGv = 0.0f;
                    f.swReq = f.gz;
                    f.swReqV = 0.0f;
                    f.swPath = f.gz;
                    f.swPrevS = plan.s0;
                    f.swPrevAbove = p.above;
                    f.swW = fl->weight;
                    ++m_diag.stairSwings;
                }
            }
            f.wasDown = false;
            if (!f.sw)
                continue;
            const stairsteps::Flight& F = f.swFl;
            const float sBall = sw::Along(F, wx(l.ball), wy(l.ball)), sHeel = sw::Along(F, wx(l.heel), wy(l.heel));
            const float sMid = sw::Along(F, mx, my), sToe = sw::Along(F, wx(toe), wy(toe));
            // the lower of the toe tip along the sole and the toe level with the ball (a raised toe never lowers the requirement)
            const float toeAbove = m_haveFloor ? std::fmax(std::fmin(toe.z, l.ball.z) - m_floor, -0.03f) : p.above;
            // the other foot landed: an unanchored plan (the flight phase of a run) lands one step past it
            if (landed[1 - s] && !f.swPlan.anchored)
            {
                const float step = m_stepEst > 0.0f ? m_stepEst : float(F.treads) * F.tread;
                sw::Anchor(sc, F, f.swPlan, sw::Along(F, o.supX, o.supY), o.supH, step);
            }
            if (dt > 0.0f)
                sw::Retarget(sc, F, f.swPlan, sMid, (sMid - f.swPrevS) / dt, p.above, (p.above - f.swPrevAbove) / dt);
            if (f.trOk && !sw::Reconcile(sc, F, f.swPlan, sw::Along(F, f.trWX, f.trWY), f.trG))
                sw::Converge(sc, F, f.swPlan, sw::Along(F, f.trWX, f.trWY), f.trG);
            sw::Advance(sc, f.swPlan, sMid, dt);
            if (f.swPlan.u < f.swPlan.capFrom)
                f.swPlan.capFrom = sw::CapStart(sc, F, f.swPlan, sToe - sMid); // the landing may have moved (not once the blend began)
            f.swPath = sw::PathStep(f.swPlan, f.swPath, f.swPrevS, sMid, F.rise < 0.0f);
            f.swPrevS = sMid;
            f.swPrevAbove = p.above;
            const float heelAbove = m_haveFloorHeel ? std::fmax(l.heel.z - m_floorHeel, 0.0f) : 0.0f;
            // the glide is smooth by itself; the requirement steps where a nose is passed going down or the plan is re-anchored: it is
            // followed (a lower one critically damped, a higher one at once) and joined with the glide
            const float req = sw::SwingReq(sc, F, f.swPlan, f.swPlan.u, sToe, toeAbove, sHeel, heelAbove, sBall, p.above);
            if (std::isfinite(req))
                sw::Follow(f.swReq, f.swReqV, req, sc.follow, dt, sc.followVMax);
            const float g = sw::SwingJoin(sc, f.swPath, std::isfinite(req) ? f.swReq : NAN, f.swPlan.u);
            if (!std::isfinite(g))
            {
                HandSwing(f, p, aDrawn); // abort: the plane goes on from the planned height
                continue;
            }
            f.swGv = dt > 0.0f ? (g - f.swG) / dt : 0.0f;
            f.swG = g;
            // the swing's weight follows the flight's (0 once it is lost or the option is off) at most 1 / swingFadeS per second
            const float wTarget = fl ? std::clamp(fl->weight, 0.0f, 1.0f) : 0.0f;
            const float wStep = dt > 0.0f ? dt / m_cfg.swingFadeS : 0.0f;
            f.swW = f.swW + std::clamp(wTarget - f.swW, -wStep, wStep);
            if (!(f.swW > 0.0f) && !fl)
            {
                f.sw = false;
                f.swW = 0.0f;
                continue;
            }
            p.planW = f.swW;
            p.planLift = f.swG - aDrawn[2];
        }
    }

    Config m_cfg;
    Foot m_foot[2];
    float m_pel = 0.0f, m_pelV = 0.0f;
    float m_vz = 0.0f; // steady drawn height rate (m/s)
    float m_stepEst = 0.0f; // m, last measured step on a stair flight (landing past the other foot's last landing; 0 = none)
    float m_move[2] = {0.0f, 1.0f}; // drawn motion direction of the last update (model axes)
    bool m_havePrev = false;
    float m_prev[3] = {0.0f, 0.0f, 0.0f};
    float m_floor = 0.0f;  // m, floor estimate of the clip (model z of the lowest ball, rising at most floorRise per second)
    bool m_haveFloor = false;
    float m_floorHeel = 0.0f; // m, the same for the heels (StairSwings)
    bool m_haveFloorHeel = false;
    Plan m_plan;
    StageDiag m_diag;
};

} // namespace utpp::mm::groundfeet
