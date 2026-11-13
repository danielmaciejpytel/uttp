#pragma once
// Optional Pistol2H experiment: move an authored weapon pose by the native
// weapon's timed weapon-local motion. This does not change the game action.

#include "GripIK.hpp"
#include "GamePoseCapture.hpp"

#include <algorithm>
#include <cmath>

namespace utpp::mm::native_shoot
{
inline bool AdmitSamples(const integ::GamePoseSample& base, const integ::GamePoseSample& current,
                         uint64_t rig, uint64_t generation, uint32_t count,
                         int64_t eventTicks, int64_t nowTicks, int64_t maxAgeTicks)
{
    return eventTicks > 0 && maxAgeTicks > 0 && base.rig == rig && base.generation == generation &&
           base.count == count && base.ticks > 0 && base.ticks < eventTicks &&
           eventTicks - base.ticks <= maxAgeTicks && current.ticks > eventTicks &&
           current.ticks - eventTicks <= 10 * maxAgeTicks &&
           integ::FreshGamePoseSample(current, rig, generation, count, nowTicks, maxAgeTicks);
}
// Recoil is relative to the source weapon's initial frame. A rigid model-space
// delta rotates around the FPP pivot and amplifies motion on an offset TPP pose.
inline bool RelativeMotion(const ov::Xf& before, const ov::Xf& after, ov::Xf& out)
{
    if (!math::IsFinite(before.r) || !math::IsFinite(after.r) ||
        !math::IsFinite(before.t) || !math::IsFinite(after.t)) return false;
    if (std::fabs(math::QuatDot(before.r, before.r) - 1.0f) > 0.001f ||
        std::fabs(math::QuatDot(after.r, after.r) - 1.0f) > 0.001f) return false;
    const auto inverse = math::QuatInv(before.r);
    out.r = math::QuatNormalize(math::QuatMul(inverse, after.r));
    const math::Vec3 movement = after.t - before.t;
    out.t = math::QuatRotate(inverse, movement);
    const float move = std::sqrt(math::Dot(movement, movement));
    const float dot = std::fabs(math::QuatDot(before.r, after.r));
    const float turn = 2.0f * std::acos(std::clamp(dot, 0.0f, 1.0f));
    return math::IsFinite(out.r) && math::IsFinite(out.t) && std::isfinite(move) && move <= 0.25f &&
           std::isfinite(turn) && turn <= 0.70f; // <=40 degrees; reject unrelated native action poses
}

inline bool WeaponDelta(const ov::Skeleton& sk, const ov::Pose& before, const ov::Pose& after,
                        const gik::Rig& rig, ov::Xf& out)
{
    if (!rig.Valid(sk)) return false;
    ov::Xf a, b;
    gik::ModelOf(sk, before, rig.weaponRight, a.r, a.t);
    gik::ModelOf(sk, after, rig.weaponRight, b.r, b.t);
    return RelativeMotion(a, b, out);
}

// Opt-in extraction only: remove the independently sampled chest frame before
// measuring weapon-local motion. Both pairs must use the same space and time.
// This cancels locomotion shared by chest and weapon, but also chest recoil;
// arm swing is not cancelled. It is not sufficient to enable moving transfer.
inline bool ChestRelativeMotion(const ov::Xf& chestBefore, const ov::Xf& weaponBefore,
                                const ov::Xf& chestAfter, const ov::Xf& weaponAfter,
                                ov::Xf& out)
{
    const auto valid = [](const ov::Xf& x) {
        return math::IsFinite(x.r) && math::IsFinite(x.t) &&
               std::fabs(math::QuatDot(x.r, x.r) - 1.0f) <= 0.001f &&
               math::IsFinite(x.s) && std::fabs(x.s.x - 1.0f) <= 0.001f &&
               std::fabs(x.s.y - 1.0f) <= 0.001f && std::fabs(x.s.z - 1.0f) <= 0.001f;
    };
    if (!valid(chestBefore) || !valid(weaponBefore) ||
        !valid(chestAfter) || !valid(weaponAfter)) return false;
    const auto relative = [](const ov::Xf& chest, const ov::Xf& weapon) {
        const auto inverse = math::QuatInv(chest.r);
        return ov::Xf{math::QuatNormalize(math::QuatMul(inverse, weapon.r)),
                      math::QuatRotate(inverse, weapon.t - chest.t)};
    };
    ov::Xf candidate;
    if (!RelativeMotion(relative(chestBefore, weaponBefore),
                        relative(chestAfter, weaponAfter), candidate)) return false;
    out = candidate;
    return true;
}

inline ov::Xf RightHandTarget(const ov::Xf& motion, const ov::Xf& weapon, const ov::Xf& hand)
{
    const auto inverse = math::QuatInv(weapon.r);
    const auto relativeHandRot = math::QuatMul(inverse, hand.r);
    const auto relativeHandPos = math::QuatRotate(inverse, hand.t - weapon.t);
    const auto targetRot = math::QuatNormalize(math::QuatMul(weapon.r, motion.r));
    const auto targetPos = weapon.t + math::QuatRotate(weapon.r, motion.t);
    return {math::QuatNormalize(math::QuatMul(targetRot, relativeHandRot)),
            targetPos + math::QuatRotate(targetRot, relativeHandPos)};
}

// Fade the SOLVED arm rotations, not only the requested weapon delta. Even a
// zero-delta IK solve can bend a nearly straight authored Aim arm at soft reach.
// Inputs are the same frame's authored pose and its GripIK result (only these
// arm rotations may differ). Weapon/hand attachment and all local lengths stay
// unchanged. The caller must validate final two-hand contact after blending.
inline bool BlendSolvedArms(const ov::Skeleton& sk, const gik::Rig& rig,
                            const ov::Pose& base, const ov::Pose& solved,
                            float weight, ov::Pose& out)
{
    if (!sk.Valid() || !rig.Valid(sk) || !std::isfinite(weight) || weight < 0 || weight > 1)
        return false;
    if (weight == 0) { out = base; return true; }
    const int arms[] = {rig.left.upper, rig.left.lower, rig.left.hand,
                        rig.right.upper, rig.right.lower, rig.right.hand};
    for (int b : arms)
        if (!math::IsFinite(base.b[b].r) || !math::IsFinite(solved.b[b].r) ||
            std::fabs(math::QuatDot(base.b[b].r, base.b[b].r) - 1.0f) > 0.001f ||
            std::fabs(math::QuatDot(solved.b[b].r, solved.b[b].r) - 1.0f) > 0.001f)
            return false;
    if (weight == 1) { out = solved; return true; }
    ov::Pose candidate = base;
    for (int b : arms)
        candidate.b[b].r = ov::BlendXf(base.b[b], solved.b[b], weight).r;
    out = candidate; // also safe when out aliases either input
    return true;
}

// Reuse the last accepted action only for a brief capture gap in the same
// weapon/rig generation. Call Reset on action or weapon transitions.
struct DeltaHold
{
    ov::Xf last{};
    uint64_t rig = 0, generation = 0;
    int64_t lastFreshTicks = 0;
    bool valid = false;

    void Reset() { valid = false; lastFreshTicks = 0; }
    bool Resolve(bool identityOk, bool freshDelta, const ov::Xf& delta,
                 uint64_t currentRig, uint64_t currentGeneration,
                 int64_t nowTicks, int64_t maxGapTicks, ov::Xf& out, float* outWeight = nullptr)
    {
        if (outWeight) *outWeight = 0.0f;
        if (!identityOk || currentRig == 0 || currentGeneration == 0 ||
            nowTicks <= 0 || maxGapTicks <= 0)
        {
            Reset();
            return false;
        }
        if (valid && (rig != currentRig || generation != currentGeneration)) Reset();
        if (freshDelta)
        {
            last = delta;
            rig = currentRig;
            generation = currentGeneration;
            lastFreshTicks = nowTicks;
            valid = true;
        }
        if (!valid || nowTicks < lastFreshTicks || nowTicks - lastFreshTicks >= 2 * maxGapTicks)
        {
            Reset();
            return false;
        }
        out = last;
        if (outWeight)
        {
            const int64_t elapsed = nowTicks - lastFreshTicks;
            const float u = std::clamp(float(elapsed - maxGapTicks) / float(maxGapTicks), 0.0f, 1.0f);
            *outWeight = 1.0f - u * u * (3.0f - 2.0f * u);
        }
        return true;
    }
};

struct EpisodeGate
{
    bool pending = false, admitted = false;
    int64_t eventTicks = 0, acceptedTicks = 0;
    void Begin(int64_t ticks) { pending = true; admitted = false; eventTicks = ticks; acceptedTicks = 0; }
    void Reset() { pending = admitted = false; eventTicks = acceptedTicks = 0; }
    // The action event and FK sample arrive on different update paths. A pre-event
    // sample is not a rejected pose: allow a bounded wait, then fade in on admission.
    // Invalid post-event data rejects the episode; expired episodes cannot restart.
    bool Accept(bool freshDelta, bool awaitingSample, int64_t nowTicks,
                int64_t graceTicks, int64_t rampTicks, float& weight)
    {
        weight = 0.0f;
        if (nowTicks < eventTicks || eventTicks <= 0 || graceTicks <= 0 || rampTicks <= 0)
        { Reset(); return false; }
        if (pending)
        {
            if (nowTicks - eventTicks > graceTicks || (!freshDelta && !awaitingSample))
            { Reset(); return false; }
            if (!freshDelta) return false;
            admitted = true; pending = false; acceptedTicks = nowTicks;
        }
        if (admitted)
        {
            const float u = std::clamp(float(nowTicks - acceptedTicks) / float(rampTicks), 0.0f, 1.0f);
            weight = u * u * (3.0f - 2.0f * u);
        }
        return admitted;
    }
};
} // namespace utpp::mm::native_shoot
