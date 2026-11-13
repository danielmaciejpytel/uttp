#pragma once
// TPP posture only: shorten clavicle-to-upper-arm offsets inward while retaining both original hand contacts.
// Never edits rig assets or scales the body. Caller supplies reusable scratch; failure leaves pose untouched.
#include "GripIK.hpp"

namespace utpp::mm::femshoulder
{
// Why a pass left (part of) the requested narrowing out. ApplyContinuous only; the legacy Apply leaves it None.
enum class Reason : uint8_t { None = 0, ZeroWeight, Invalid, Crossed, Scale, Reach, Solve, Contact };
inline constexpr float kLegacyAmount = 0.020f; // m per side at weight 1 (installed 2026-10-15)
// Setting range (Mod Settings "Shoulder width (female V)" -5 .. +5 cm = 5 .. 0 cm per side; CET mm.shoulderWidth
// -> native UTPPSetMotionShoulderNarrow, cm per side). The pass below has no amount limit of its own: per frame the crossed margin
// and the reach window cap the weight for any amount, contacts stay exact (FeminineShouldersAdaptiveTest: 5 cm per side). Above the
// shipped default the socket may also drift toward the hand (DriftAllowance, ApplyContinuous), which keeps the frame-to-frame
// smoothness gates of FeminineWidthConsistencyTest over the whole range (the lateral-only pass held them up to 3.0 cm per side only:
// from 3.25 cm the narrowing snapped back at reach cliffs, 2026-11-09 amount sweep).
inline constexpr float kMaxAmountCm = 5.0f;
// Shipped default narrowing (m per side, = motionpose::kFeminineShoulderAmount). Up to it the pass is the lateral-only pass of 2026-11-02,
// bit for bit (no drift).
inline constexpr float kExactAmount = 0.025f;
// REACH DRIFT (pose-integration 2026-11-09). A lateral-only socket shift cannot keep an almost straight arm on its hand: the admissible
// shift falls with the square root of the reach slack (-c + sqrt(c^2 + R^2 - |d|^2)), so the last 1-2 % of extension wipe it out within a
// frame or two (the "snap back" of wide settings; jump starts / landings: both arms ~99 % straight, 0.95 cm left of 10 cm at 5 cm per side).
// Above kExactAmount each socket may also move up to DriftAllowance(amount) toward its hand, perpendicular to the lateral axis (the shoulder
// follows a reaching hand: protraction / elevation), just as far as the arm needs to stay inside the reach window. The admissible lateral
// shift then becomes -c + sqrt(R^2 - (|d_perp| - drift)^2): Lipschitz in the arm pose (no cliff), the full posture on every arm that is not
// both straight and pointing sideways, and the lateral narrowing (the weight) stays exact, so the rise limiter and every gate keep their
// meaning. 0 drift (amount <= kExactAmount) = the lateral-only code path, unchanged.
inline float DriftAllowance(float aAmount) { return aAmount > kExactAmount ? aAmount - kExactAmount : 0.0f; }
// cm per side -> m. A division, not cm * 0.01f: every 0.25 cm step lands on its decimal metre value, so 2.5 cm is the shipped
// default bit for bit (2.5f * 0.01f = 0.024999999f != 0.025f would move the default pose by one ulp after a setting round trip).
inline float AmountFromCm(float aCm) { return aCm / 100.0f; }
struct Result
{
    bool applied = false;
    gik::Result left{}, right{};
    float handError = 0.0f, weaponError = 0.0f;
    float requestedWeight = 0.0f, appliedWeight = 0.0f;
    // ApplyContinuous diagnostics (zero from the legacy Apply):
    Reason reason = Reason::None;
    float weight[2] = {};      // applied per-side weight (left, right), 0..requested
    float feasible[2] = {};    // per-side weight the reach / crossed limits admit before the rise limiter
    float widthBefore = 0.0f;  // upper-arm socket distance along the Spine3 lateral, before the pass (m)
    float widthAfter = 0.0f;   // same, after the pass (== widthBefore when nothing was applied)
    float elbowMove[2] = {};   // model-space displacement of each elbow (LeftForeArm, RightForeArm) caused by the pass (m)
    float elbowFlexDeg[2] = {}; // change of the interior elbow angle (upper-lower-hand) caused by the pass (deg)
    float drift[2] = {};        // model-space socket drift toward the hand per side (m, REACH DRIFT; 0 at amounts <= kExactAmount)
    bool held[2] = {};          // RETURN HOLD active on this side (amount > kExactAmount only)
};
// Upper-arm socket distance along the Spine3 lateral (m); negative = crossed, 0 = not measurable. Measurement only.
inline float SocketWidth(const ov::Skeleton& sk, const ov::Pose& pose, const gik::Rig& rig, int spine, int shoulderL,
                         int shoulderR)
{
    if (!sk.Valid() || pose.n != sk.n || !rig.Valid(sk) || spine < 0 || spine >= sk.n || shoulderL < 0 || shoulderR < 0 ||
        shoulderL >= sk.n || shoulderR >= sk.n) return 0.0f;
    ov::Vec3 lateral;
    if (!gik::Norm(pose.b[shoulderR].t - pose.b[shoulderL].t, lateral)) return 0.0f;
    ov::Quat spineRot, q; ov::Vec3 spinePos, left, right;
    gik::ModelOf(sk, pose, spine, spineRot, spinePos);
    gik::ModelOf(sk, pose, rig.left.upper, q, left);
    gik::ModelOf(sk, pose, rig.right.upper, q, right);
    const float width = math::Dot(right - left, math::QuatRotate(spineRot, lateral));
    return std::isfinite(width) ? width : 0.0f;
}
inline bool SameContact(const ov::Skeleton& sk, const ov::Pose& pose, int bone,
                        const ov::Quat& rotation, ov::Vec3 position, float& error)
{
    if (bone < 0) return true;
    ov::Quat q; ov::Vec3 p;
    gik::ModelOf(sk, pose, bone, q, p);
    error = std::fmax(error, gik::Len(p - position));
    const float dot = std::fabs(q.x * rotation.x + q.y * rotation.y + q.z * rotation.z + q.w * rotation.w);
    return error <= 1e-4f && std::isfinite(dot) && dot >= 1.0f - 1e-6f;
}
inline bool Apply(const ov::Skeleton& sk, ov::Pose& pose, ov::Pose& scratch, const gik::Rig& rig,
                  int spine, int shoulderL, int shoulderR, float weight, const gik::Params& params, Result& result,
                  bool allowAdaptive = false)
{
    result = Result{};
    const float w = ov::Clamp01(weight);
    result.requestedWeight = w;
    if (!std::isfinite(weight) || &pose == &scratch ||
        !std::isfinite(params.softStart) || !std::isfinite(params.maxReach) || !std::isfinite(params.minReach)) return false;
    if (w <= ov::kZeroWeight) return true;
    if (!sk.Valid() || pose.n != sk.n || rig.weaponLeft >= sk.n || !rig.Valid(sk) ||
        shoulderL == shoulderR || rig.left.hand == rig.right.hand ||
        spine < 0 || spine >= sk.n || shoulderL < 0 || shoulderR < 0 || shoulderL >= sk.n || shoulderR >= sk.n ||
        sk.parent[shoulderL] != spine || sk.parent[shoulderR] != spine ||
        sk.parent[rig.left.upper] != shoulderL || sk.parent[rig.right.upper] != shoulderR) return false;
    ov::Vec3 lateral;
    if (!gik::Norm(pose.b[shoulderR].t - pose.b[shoulderL].t, lateral)) return false;
    // Some native action poses cross the clavicles. Leave their sockets unchanged.
    // The clavicle roots themselves are never translated by this pass.
    ov::Quat spineRot, upperLRot, upperRRot;
    ov::Vec3 spinePos, upperLPos, upperRPos;
    gik::ModelOf(sk, pose, spine, spineRot, spinePos);
    gik::ModelOf(sk, pose, rig.left.upper, upperLRot, upperLPos);
    gik::ModelOf(sk, pose, rig.right.upper, upperRRot, upperRPos);
    const float signedWidth = math::Dot(upperRPos - upperLPos, math::QuatRotate(spineRot, lateral));
    if (!std::isfinite(signedWidth) || signedWidth <= 0.040f * w) return false;
    // ModelOf/SolveArm use rigid transforms. Fail closed instead of ignoring actual nonunit scale.
    for (int i = 0; i < sk.n; ++i)
        if (!math::IsFinite(pose.b[i].r) || !math::IsFinite(pose.b[i].t) ||
            !math::IsFinite(pose.b[i].s) || std::fabs(pose.b[i].s.x - 1.0f) > 1e-3f ||
            std::fabs(pose.b[i].s.y - 1.0f) > 1e-3f || std::fabs(pose.b[i].s.z - 1.0f) > 1e-3f) return false;
    ov::Quat handLRot, handRRot, weaponLRot, weaponRRot;
    ov::Vec3 handLPos, handRPos, weaponLPos, weaponRPos;
    gik::ModelOf(sk, pose, rig.left.hand, handLRot, handLPos);
    gik::ModelOf(sk, pose, rig.right.hand, handRRot, handRPos);
    if (rig.weaponLeft >= 0) gik::ModelOf(sk, pose, rig.weaponLeft, weaponLRot, weaponLPos);
    gik::ModelOf(sk, pose, rig.weaponRight, weaponRRot, weaponRPos);
    const auto attempt = [&](float applied) {
        Result trial; trial.requestedWeight = w;
        scratch = pose;
        // Preserve the original fully accepted path exactly: same socket
        // displacement, same SolveArm inputs/order and same contact checks.
        const ov::Vec3 shiftModel = math::QuatRotate(spineRot, lateral) * (0.020f * applied);
        ov::Quat shoulderLRot, shoulderRRot;
        ov::Vec3 shoulderLPos, shoulderRPos;
        gik::ModelOf(sk, pose, shoulderL, shoulderLRot, shoulderLPos);
        gik::ModelOf(sk, pose, shoulderR, shoulderRRot, shoulderRPos);
        scratch.b[rig.left.upper].t = scratch.b[rig.left.upper].t +
            math::QuatRotate(math::QuatInv(shoulderLRot), shiftModel);
        scratch.b[rig.right.upper].t = scratch.b[rig.right.upper].t -
            math::QuatRotate(math::QuatInv(shoulderRRot), shiftModel);
        if (!gik::SolveArm(sk, scratch, rig.left, handLPos, &handLRot, 1.0f, params, trial.left) ||
            !trial.left.ok || trial.left.clamped ||
            !gik::SolveArm(sk, scratch, rig.right, handRPos, &handRRot, 1.0f, params, trial.right) ||
            !trial.right.ok || trial.right.clamped) { result = trial; return false; }
        if (!SameContact(sk, scratch, rig.left.hand, handLRot, handLPos, trial.handError) ||
            !SameContact(sk, scratch, rig.right.hand, handRRot, handRPos, trial.handError) ||
            !SameContact(sk, scratch, rig.weaponLeft, weaponLRot, weaponLPos, trial.weaponError) ||
            !SameContact(sk, scratch, rig.weaponRight, weaponRRot, weaponRPos, trial.weaponError))
            { result = trial; return false; }
        pose = scratch; trial.applied = true; trial.appliedWeight = applied; result = trial;
        return true;
    };
    if (attempt(w)) return true;
    if (!allowAdaptive) return false; // Existing unarmed/2H callers keep prior rejection semantics.
    // Only reach clamp admits a smaller request. Other invalid/failed solves
    // retain the original atomic fail-closed behaviour.
    if ((!result.left.clamped && !result.right.clamped) ||
        !std::isfinite(params.softStart) || !std::isfinite(params.maxReach) ||
        !std::isfinite(params.minReach) || params.softStart <= 0 ||
        params.softStart > 1 || params.minReach < 0) return false;
    struct Reach { ov::Vec3 delta, direction; double minSquared, maxSquared; } reach[2];
    const ov::Vec3 oneWeight = math::QuatRotate(spineRot, lateral) * 0.020f;
    float candidates[10] = {w, 0}; int count = 2;
    const auto dotDouble=[](ov::Vec3 a,ov::Vec3 b) { return double(a.x)*b.x+double(a.y)*b.y+double(a.z)*b.z; };
    for (int side = 0; side < 2; ++side) {
        const auto& arm = side ? rig.right : rig.left;
        ov::Quat q; ov::Vec3 upper, lower, hand;
        gik::ModelOf(sk, pose, arm.upper, q, upper);
        gik::ModelOf(sk, pose, arm.lower, q, lower);
        gik::ModelOf(sk, pose, arm.hand, q, hand);
        const float l1 = gik::Len(lower-upper), l2 = gik::Len(hand-lower), full = l1+l2;
        if (!(l1>1e-4f && l2>1e-4f)) return false;
        const double minimum = std::fmax(params.minReach*full, std::fabs(l1-l2)+1e-4f*full);
        // The original solver remains authoritative. Stay below its unchanged
        // soft threshold; do not accept a clamped hand or relax physical reach.
        const double maximum = params.softStart*full;
        reach[side] = {hand-upper, oneWeight*(side ? -1.0f : 1.0f), minimum*minimum, maximum*maximum};
        const auto& r=reach[side];
        const double a=dotDouble(r.direction,r.direction), b=-2.0*dotDouble(r.delta,r.direction);
        for (double radiusSquared : {r.minSquared,r.maxSquared}) {
            const double c=dotDouble(r.delta,r.delta)-radiusSquared;
            const double discriminant=b*b-4*a*c;
            if (discriminant<0) continue;
            const double root=std::sqrt(discriminant);
            for (double value : {(-b-root)/(2*a),(-b+root)/(2*a)})
                if (value>=0 && value<=w) candidates[count++]=static_cast<float>(value);
        }
    }
    float feasible=0;
    for (int i=0;i<count;++i) {
        const float candidate=candidates[i]<w?candidates[i]*0.99999f:candidates[i]; bool allowed=true;
        for (const auto& r:reach) {
            const auto delta=r.delta-r.direction*candidate;
            const double squared=dotDouble(delta,delta);
            allowed=allowed && squared>=r.minSquared-1e-8 && squared<=r.maxSquared+1e-8;
        }
        if (allowed) feasible=std::max(feasible,candidate);
    }
    if (feasible<=ov::kZeroWeight) return false;
    // Tiny inward margin guards float FK at a quadratic boundary (at most0.2um
    // per socket), without changing any previous fully accepted request.
    return attempt(feasible*0.99999f);
}

// ---------------------------------------------------------------------------------------------------------------
// Continuous variant (2026-11-02): the same socket displacement and the same exact hand / weapon contact rules, but
// never all-or-nothing. Every limit turns into a smaller per-side weight instead of a rejected pass:
//  * crossed / narrow clavicles: the weight is capped so the sockets keep >= crossedMargin of width after the shift
//    (the legacy guard returned the whole wide pose when the UNshifted width was <= 4 cm * w),
//  * reach: each side takes the largest weight at which its arm stays inside [minReach, softStart * full] (a convex
//    quadratic per side; the legacy adaptive path solved one common weight and only for 1H / unarmed callers),
//  * recovery: opt.maxWeight (previous applied weight + rate * dt, kept by the caller) only limits a RISE; a drop
//    follows feasibility at once because the hand contact is exact.
// Mutation switch (offline tests): 1 = atomic like the legacy 2H path (any limit rejects the whole pass),
// 2 = the rise limiter is ignored, 3 = no REACH DRIFT (the lateral-only pass at every amount), 4 = no RETURN HOLD.
#ifndef UTPP_FEMSHOULDER_MUTATION
#define UTPP_FEMSHOULDER_MUTATION 0
#endif
// RETURN HOLD (dll-shoulder-brake 2026-11-09). Above kExactAmount a running arm swing (aim while running: the gun hand reaches out once per
// stride, ~0.6 s) takes the reach window down to about half the weight and gives it back a few frames later; the rise limiter returned the
// posture each time, so the shoulder "breathed" ~1.2 cm per side in step (11 drops > 5 mm in 4 s at 5 cm per side). The drop itself is
// forced (exact hand contact), the RETURN is not. A side whose reach limit comes back as a NEW dip within kHoldSeconds of its previous
// reach-limited frame (a periodic swing, not a one-off reach) holds its lowest reach-limited weight for kHoldSeconds; a further
// reach-limited frame within kHoldToleranceM (m per side) of the held level re-arms the hold, so the swing keeps one steady, slightly wider
// shoulder until it stops. Then the rise limiter returns the posture as before. One-off dips (a landing, a vault, a two-hand reach) return
// as before. Up to kExactAmount the hold is never consulted (the pass stays bit for bit).
inline constexpr float kHoldSeconds = 0.8f;
inline constexpr float kHoldToleranceM = 0.005f;
struct ReturnHold
{
    float level[2] = {1.0f, 1.0f}; // held weight per side (left, right)
    float timer[2] = {};           // s left of the hold per side; <= 0 = no hold
    float recent[2] = {};          // s left of the memory of the last reach-limited frame per side; <= 0 = none
    bool limited[2] = {};          // the previous pass was reach-limited on this side
    void Reset() { *this = ReturnHold{}; }
};
struct Options
{
    float amount = kLegacyAmount;     // m per side at weight 1
    const float* maxWeight = nullptr; // [left, right] upper bound of the applied weight (rise limiter); nullptr = none
    float crossedMargin = 0.040f;     // m of socket width the pass leaves at least
    float drift = -1.0f;              // m a socket may drift toward its hand (REACH DRIFT); < 0 = DriftAllowance(amount), 0 = lateral only
    ReturnHold* hold = nullptr;       // RETURN HOLD state of the caller (amount > kExactAmount only); nullptr = none
    float dt = 0.0f;                  // s since the previous pass (advances the hold; 0 = the same frame again)
};
namespace detail
{
// Largest s in [0, cap] with minSq <= |delta - dir * s|^2 <= maxSq; negative when no such s exists.
inline double FeasibleShift(const double delta[3], const double dir[3], double cap, double minSq, double maxSq)
{
    const auto dot = [](const double* a, const double* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    const double a = dot(dir, dir), b = -2.0 * dot(delta, dir);
    if (!(a > 0.0)) return -1.0;
    double candidates[6] = {cap, 0.0};
    int count = 2;
    for (double radiusSq : {minSq, maxSq})
    {
        const double c = dot(delta, delta) - radiusSq, disc = b * b - 4.0 * a * c;
        if (disc < 0.0) continue;
        const double root = std::sqrt(disc);
        for (double v : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)})
            if (v >= 0.0 && v <= cap) candidates[count++] = v;
    }
    double best = -1.0;
    for (int i = 0; i < count; ++i)
    {
        const double s = candidates[i] < cap ? candidates[i] * 0.99999 : candidates[i];
        const double d[3] = {delta[0] - dir[0] * s, delta[1] - dir[1] * s, delta[2] - dir[2] * s};
        const double sq = dot(d, d);
        if (sq >= minSq - 1e-8 && sq <= maxSq + 1e-8) best = std::max(best, s);
    }
    return best;
}
// REACH DRIFT variant: the upper reach bound is tested on maxDelta (the hand offset with its part perpendicular to dir shortened by the
// drift), the lower one on minDelta (the unshortened offset: no drift is used while the arm is inside the window). Largest s in [0, cap]
// with |minDelta - dir s|^2 >= minSq and |maxDelta - dir s|^2 <= maxSq; negative when none. Same candidate scheme as FeasibleShift.
inline double FeasibleShiftDrift(const double maxDelta[3], const double minDelta[3], const double dir[3], double cap, double minSq,
                                 double maxSq)
{
    const auto dot = [](const double* a, const double* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    const double a = dot(dir, dir);
    if (!(a > 0.0)) return -1.0;
    double candidates[6] = {cap, 0.0};
    int count = 2;
    const auto roots = [&](const double* delta, double radiusSq) {
        const double b = -2.0 * dot(delta, dir), c = dot(delta, delta) - radiusSq, disc = b * b - 4.0 * a * c;
        if (disc < 0.0) return;
        const double root = std::sqrt(disc);
        for (double v : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)})
            if (v >= 0.0 && v <= cap) candidates[count++] = v;
    };
    roots(minDelta, minSq);
    roots(maxDelta, maxSq);
    double best = -1.0;
    for (int i = 0; i < count; ++i)
    {
        const double s = candidates[i] < cap ? candidates[i] * 0.99999 : candidates[i];
        const double dm[3] = {minDelta[0] - dir[0] * s, minDelta[1] - dir[1] * s, minDelta[2] - dir[2] * s};
        const double dx[3] = {maxDelta[0] - dir[0] * s, maxDelta[1] - dir[1] * s, maxDelta[2] - dir[2] * s};
        if (dot(dm, dm) >= minSq - 1e-8 && dot(dx, dx) <= maxSq + 1e-8) best = std::max(best, s);
    }
    return best;
}
} // namespace detail
inline bool ApplyContinuous(const ov::Skeleton& sk, ov::Pose& pose, ov::Pose& scratch, const gik::Rig& rig,
                            int spine, int shoulderL, int shoulderR, float weight, const gik::Params& params,
                            Result& result, const Options& opt = {})
{
    result = Result{};
    const float w = ov::Clamp01(weight);
    result.requestedWeight = w;
    const auto fail = [&](Reason why) { result.reason = why; result.applied = false; return false; };
    if (!std::isfinite(weight) || &pose == &scratch || !std::isfinite(opt.amount) || !std::isfinite(opt.drift) ||
        !std::isfinite(params.softStart) || !std::isfinite(params.maxReach) || !std::isfinite(params.minReach))
        return fail(Reason::Invalid);
    if (w <= ov::kZeroWeight || opt.amount <= 0.0f) { result.reason = Reason::ZeroWeight; return true; }
#if UTPP_FEMSHOULDER_MUTATION == 3
    const float drift = 0.0f; // mutation 3: no reach drift
#else
    const float drift = opt.drift >= 0.0f ? opt.drift : DriftAllowance(opt.amount); // REACH DRIFT allowance (m), 0 = lateral only
#endif
    if (!sk.Valid() || pose.n != sk.n || rig.weaponLeft >= sk.n || !rig.Valid(sk) ||
        shoulderL == shoulderR || rig.left.hand == rig.right.hand ||
        spine < 0 || spine >= sk.n || shoulderL < 0 || shoulderR < 0 || shoulderL >= sk.n || shoulderR >= sk.n ||
        sk.parent[shoulderL] != spine || sk.parent[shoulderR] != spine ||
        sk.parent[rig.left.upper] != shoulderL || sk.parent[rig.right.upper] != shoulderR ||
        !(params.softStart > 0.0f && params.softStart <= 1.0f) || params.minReach < 0.0f)
        return fail(Reason::Invalid);
    ov::Vec3 lateral;
    if (!gik::Norm(pose.b[shoulderR].t - pose.b[shoulderL].t, lateral)) return fail(Reason::Invalid);
    for (int i = 0; i < sk.n; ++i)
        if (!math::IsFinite(pose.b[i].r) || !math::IsFinite(pose.b[i].t) ||
            !math::IsFinite(pose.b[i].s) || std::fabs(pose.b[i].s.x - 1.0f) > 1e-3f ||
            std::fabs(pose.b[i].s.y - 1.0f) > 1e-3f || std::fabs(pose.b[i].s.z - 1.0f) > 1e-3f)
            return fail(Reason::Scale);
    ov::Quat spineRot, upperLRot, upperRRot;
    ov::Vec3 spinePos, upperLPos, upperRPos;
    gik::ModelOf(sk, pose, spine, spineRot, spinePos);
    gik::ModelOf(sk, pose, rig.left.upper, upperLRot, upperLPos);
    gik::ModelOf(sk, pose, rig.right.upper, upperRRot, upperRPos);
    const ov::Vec3 lateralModel = math::QuatRotate(spineRot, lateral);
    const float signedWidth = math::Dot(upperRPos - upperLPos, lateralModel);
    if (!std::isfinite(signedWidth)) return fail(Reason::Invalid);
    result.widthBefore = result.widthAfter = signedWidth;
    // Crossed / very narrow sockets: keep the margin instead of rejecting. Both sides together move by 2 * amount * weight.
    const float widthCap = (signedWidth - opt.crossedMargin) / (2.0f * opt.amount);
    if (!(widthCap > ov::kZeroWeight)) return fail(Reason::Crossed);
    const float cap = std::fmin(w, widthCap);
    ov::Quat handLRot, handRRot, weaponLRot, weaponRRot;
    ov::Vec3 handLPos, handRPos, weaponLPos, weaponRPos;
    gik::ModelOf(sk, pose, rig.left.hand, handLRot, handLPos);
    gik::ModelOf(sk, pose, rig.right.hand, handRRot, handRPos);
    if (rig.weaponLeft >= 0) gik::ModelOf(sk, pose, rig.weaponLeft, weaponLRot, weaponLPos);
    gik::ModelOf(sk, pose, rig.weaponRight, weaponRRot, weaponRPos);
    // Reach window of each arm, from the unmodified pose (the original solver stays authoritative and is re-run below).
    float feasible[2] = {};
    const ov::Vec3 oneShift = lateralModel * opt.amount;
    // REACH DRIFT geometry per side (drift > 0 only): hand offset split along the shift direction / perpendicular to it.
    struct DriftGeo { ov::Vec3 perpUnit{}; float perpLen = 0.0f, along = 0.0f, dirLen = 0.0f, reach = 0.0f; } geo[2];
    for (int side = 0; side < 2; ++side)
    {
        const auto& arm = side ? rig.right : rig.left;
        ov::Quat q; ov::Vec3 upper, lower, hand;
        gik::ModelOf(sk, pose, arm.upper, q, upper);
        gik::ModelOf(sk, pose, arm.lower, q, lower);
        gik::ModelOf(sk, pose, arm.hand, q, hand);
        const float l1 = gik::Len(lower - upper), l2 = gik::Len(hand - lower), full = l1 + l2;
        if (!(l1 > 1e-4f && l2 > 1e-4f)) return fail(Reason::Solve);
        const double minimum = std::fmax(params.minReach * full, std::fabs(l1 - l2) + 1e-4f * full);
        const double maximum = params.softStart * full;
        const ov::Vec3 d = hand - upper, dir = oneShift * (side ? -1.0f : 1.0f);
        const double delta[3] = {d.x, d.y, d.z}, direction[3] = {dir.x, dir.y, dir.z};
        double s;
        if (drift > 0.0f)
        {
            // The part of the hand offset perpendicular to the shift may shrink by up to drift (upper bound only); the window is
            // tested slightly inside (kDriftFeasible) so that the placement below (kDriftPlace) never needs more than drift.
            constexpr double kDriftFeasible = 0.99998;
            DriftGeo& g = geo[side];
            g.dirLen = gik::Len(dir);
            const ov::Vec3 lat = dir * (1.0f / g.dirLen);
            g.along = math::Dot(d, lat);
            const ov::Vec3 perp = d - lat * g.along;
            g.perpLen = gik::Len(perp);
            g.perpUnit = g.perpLen > 1e-6f ? perp * (1.0f / g.perpLen) : ov::Vec3{0.0f, 0.0f, 0.0f};
            g.reach = static_cast<float>(maximum);
            const float keep = g.perpLen > drift ? (g.perpLen - drift) / g.perpLen : 0.0f;
            const ov::Vec3 dx = lat * g.along + perp * keep;
            const double maxDelta[3] = {dx.x, dx.y, dx.z}, tight = maximum * kDriftFeasible;
            s = detail::FeasibleShiftDrift(maxDelta, delta, direction, cap, minimum * minimum, tight * tight);
        }
        else
            s = detail::FeasibleShift(delta, direction, cap, minimum * minimum, maximum * maximum);
        feasible[side] = s > 0.0 ? static_cast<float>(s) : 0.0f;
    }
    // REACH DRIFT placement: the least move toward the hand (perpendicular to the shift) that keeps the arm inside the window at this
    // weight; 0 while the shifted socket still reaches the hand. Continuous in the weight and in the pose.
    const auto driftFor = [&](int side, float aWeight) -> ov::Vec3 {
        constexpr float kDriftPlace = 0.99999f;
        const DriftGeo& g = geo[side];
        if (!(drift > 0.0f) || !(g.perpLen > 1e-6f)) return ov::Vec3{0.0f, 0.0f, 0.0f};
        const float a = g.along - aWeight * g.dirLen, r = g.reach * kDriftPlace, rho2 = r * r - a * a;
        const float v = std::clamp(g.perpLen - (rho2 > 0.0f ? std::sqrt(rho2) : 0.0f), 0.0f, drift);
        return g.perpUnit * v;
    };
#if UTPP_FEMSHOULDER_MUTATION == 1
    if (feasible[0] < cap || feasible[1] < cap) feasible[0] = feasible[1] = 0.0f; // atomic: any limit drops the whole pass
#endif
    result.feasible[0] = feasible[0];
    result.feasible[1] = feasible[1];
    float target[2] = {feasible[0], feasible[1]};
    bool held[2] = {}; // RETURN HOLD active per side (diagnostics)
#if UTPP_FEMSHOULDER_MUTATION != 4
    if (opt.hold && opt.amount > kExactAmount) // RETURN HOLD (never consulted at or below the shipped default)
    {
        ReturnHold& h = *opt.hold;
        const float advance = std::isfinite(opt.dt) && opt.dt > 0.0f ? opt.dt : 0.0f;
        const float tolerance = kHoldToleranceM / opt.amount;
        for (int side = 0; side < 2; ++side)
        {
            h.timer[side] -= advance;
            h.recent[side] -= advance;
            const bool limited = feasible[side] < cap - 1e-6f; // reach-limited this frame
            if (limited)
            {
                if (h.timer[side] > 0.0f)
                {
                    if (feasible[side] <= h.level[side] + tolerance) // the swing comes back to the held level: keep holding
                    {
                        h.level[side] = std::fmin(h.level[side], feasible[side]);
                        h.timer[side] = kHoldSeconds;
                    }
                }
                else if (!h.limited[side] && h.recent[side] > 0.0f) // a new dip soon after the last one: periodic, start holding
                {
                    h.level[side] = feasible[side];
                    h.timer[side] = kHoldSeconds;
                }
                h.recent[side] = kHoldSeconds;
            }
            h.limited[side] = limited;
            if (h.timer[side] > 0.0f) target[side] = std::fmin(target[side], h.level[side]);
            held[side] = h.timer[side] > 0.0f;
        }
    }
#endif
#if UTPP_FEMSHOULDER_MUTATION != 2
    if (opt.maxWeight)
        for (int side = 0; side < 2; ++side)
            target[side] = std::fmin(target[side], std::fmax(0.0f, opt.maxWeight[side]));
#endif
    if (target[0] <= ov::kZeroWeight && target[1] <= ov::kZeroWeight)
        return fail(feasible[0] <= ov::kZeroWeight && feasible[1] <= ov::kZeroWeight ? Reason::Reach : Reason::ZeroWeight);
    ov::Quat shoulderLRot, shoulderRRot;
    ov::Vec3 shoulderLPos, shoulderRPos;
    gik::ModelOf(sk, pose, shoulderL, shoulderLRot, shoulderLPos);
    gik::ModelOf(sk, pose, shoulderR, shoulderRRot, shoulderRPos);
    Reason lastFail = Reason::Solve;
    Result lastTrial;
    // Contact re-check; a miss (not expected inside the reach window) shrinks both sides and tries again instead of
    // giving up the whole pass.
    for (const float shrink : {1.0f, 0.5f, 0.25f, 0.1f})
    {
        const bool useL = target[0] * shrink > ov::kZeroWeight, useR = target[1] * shrink > ov::kZeroWeight;
        const float wl = useL ? target[0] * shrink : 0.0f, wr = useR ? target[1] * shrink : 0.0f;
        if (!useL && !useR) break;
        Result trial; trial.requestedWeight = w;
        scratch = pose;
        if (drift > 0.0f)
        {
            const ov::Vec3 driftL = useL ? driftFor(0, wl) : ov::Vec3{0.0f, 0.0f, 0.0f};
            const ov::Vec3 driftR = useR ? driftFor(1, wr) : ov::Vec3{0.0f, 0.0f, 0.0f};
            scratch.b[rig.left.upper].t = scratch.b[rig.left.upper].t +
                math::QuatRotate(math::QuatInv(shoulderLRot), oneShift * wl + driftL);
            scratch.b[rig.right.upper].t = scratch.b[rig.right.upper].t +
                math::QuatRotate(math::QuatInv(shoulderRRot), driftR - oneShift * wr);
            trial.drift[0] = gik::Len(driftL);
            trial.drift[1] = gik::Len(driftR);
        }
        else
        {
            scratch.b[rig.left.upper].t = scratch.b[rig.left.upper].t +
                math::QuatRotate(math::QuatInv(shoulderLRot), oneShift * wl);
            scratch.b[rig.right.upper].t = scratch.b[rig.right.upper].t -
                math::QuatRotate(math::QuatInv(shoulderRRot), oneShift * wr);
        }
        // A side that cannot narrow at all (arm already at full reach outward) keeps its authored arm untouched: its
        // own solve would clamp against the soft reach and must not veto the other side.
        if ((useL && (!gik::SolveArm(sk, scratch, rig.left, handLPos, &handLRot, 1.0f, params, trial.left) ||
                      !trial.left.ok || trial.left.clamped)) ||
            (useR && (!gik::SolveArm(sk, scratch, rig.right, handRPos, &handRRot, 1.0f, params, trial.right) ||
                      !trial.right.ok || trial.right.clamped))) { lastFail = Reason::Solve; lastTrial = trial; continue; }
        if ((useL && !SameContact(sk, scratch, rig.left.hand, handLRot, handLPos, trial.handError)) ||
            (useR && !SameContact(sk, scratch, rig.right.hand, handRRot, handRPos, trial.handError)) ||
            (useL && !SameContact(sk, scratch, rig.weaponLeft, weaponLRot, weaponLPos, trial.weaponError)) ||
            (useR && !SameContact(sk, scratch, rig.weaponRight, weaponRRot, weaponRPos, trial.weaponError)))
        { lastFail = Reason::Contact; lastTrial = trial; continue; }
        ov::Quat q; ov::Vec3 afterL, afterR;
        gik::ModelOf(sk, scratch, rig.left.upper, q, afterL);
        gik::ModelOf(sk, scratch, rig.right.upper, q, afterR);
        for (int side = 0; side < 2; ++side) // elbow evidence: where the elbow goes and how much its bend changes
        {
            const auto& arm = side ? rig.right : rig.left;
            ov::Vec3 u0, l0, h0, u1, l1, h1;
            gik::ModelOf(sk, pose, arm.upper, q, u0); gik::ModelOf(sk, pose, arm.lower, q, l0); gik::ModelOf(sk, pose, arm.hand, q, h0);
            gik::ModelOf(sk, scratch, arm.upper, q, u1); gik::ModelOf(sk, scratch, arm.lower, q, l1); gik::ModelOf(sk, scratch, arm.hand, q, h1);
            const auto flex = [](ov::Vec3 u, ov::Vec3 l, ov::Vec3 h) {
                ov::Vec3 a, b;
                if (!gik::Norm(u - l, a) || !gik::Norm(h - l, b)) return 0.0f;
                return std::acos(std::fmax(-1.0f, std::fmin(1.0f, math::Dot(a, b)))) * (180.0f / 3.14159265358979f);
            };
            trial.elbowMove[side] = gik::Len(l1 - l0);
            trial.elbowFlexDeg[side] = flex(u1, l1, h1) - flex(u0, l0, h0);
        }
        pose = scratch;
        trial.applied = true;
        trial.appliedWeight = std::fmax(wl, wr);
        trial.reason = shrink < 1.0f ? lastFail
            : (feasible[0] < cap - 1e-6f || feasible[1] < cap - 1e-6f ? Reason::Reach : Reason::None);
        trial.weight[0] = wl; trial.weight[1] = wr;
        trial.held[0] = held[0]; trial.held[1] = held[1];
        trial.feasible[0] = feasible[0]; trial.feasible[1] = feasible[1];
        trial.widthBefore = signedWidth;
        trial.widthAfter = math::Dot(afterR - afterL, lateralModel);
        result = trial;
        return true;
    }
    result = lastTrial;
    result.requestedWeight = w;
    result.feasible[0] = feasible[0]; result.feasible[1] = feasible[1];
    result.widthBefore = result.widthAfter = signedWidth;
    return fail(lastFail);
}
} // namespace utpp::mm::femshoulder
