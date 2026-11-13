#pragma once
// Melee chest base correction (2026-11-13, fists punch direction). Header-only, no RED4ext / Windows types; unit-tested offline
// (tests/MeleeBaseTest.cpp). Runtime glue: mm/MeleeBase.cpp (FK detour, after the MM blend and before the aim tilt).
//
// Problem. The fists clips are authored on their own Hips + Spine (the boxing stance leans forward 9..16 deg and the hips twist with
// each strike). In the game only Spine1 and everything above comes from the clip; Hips, Spine and the legs are the MM pose (upright)
// or, in the melee full body mode, the engine's own base pose (leaning back, narrow stance). The clip's Spine1 local rotations then sit
// on a different base and the whole upper body turns with the difference: measured on the 2026-11-12 pose recordings the striking
// arm rose +20..+50 deg above the clip (and sideways in the jab), the wrist ended 31 cm above the head.
//
// Fix. The DLL knows what the clip authored: tools/melee/fist_base_table.py bakes, for every frame of every standing fists clip that
// ships in the melee archive, the local rotations of 10 key bones (neck, head, clavicles, arms: the game plays them unchanged) and the
// Spine3 orientation in model space (the chest on the clip's own hips and spine). Per FK call:
//   1. the key bones of the game pose (read BEFORE the MM blend: while the upper-body mask blends in, the records are a mix with the MM
//      pose that matches no clip row) are matched against the table (nearest frame, interpolated with the better neighbour);
//   2. Q = chestClip(+ kChestLeadRows rows) * inverse(chestLive) (model space) is the rotation that puts the chest back to the authored
//      orientation on the hips and spine the game really shows (MM or engine, whatever they are);
//   3. Q is limited (forward bend / back bend / twist / roll, C1 soft saturation, see Limits) so the waist never folds unnaturally;
//   4. Q * weight is spread over Spine1 / Spine2 / Spine3 (a third each, about each joint, model-space axis): Spine3 and everything
//      under it (clavicles, arms, neck, head) end rotated by exactly Q * weight, the arms keep the clip's local motion.
// weight = melee fade of the snapshot x share of the upper body on the game pose (TiltWeight, as the aim tilt), times a match
// confidence gate that eases in / out, so the layer is continuous at mask fade in / out and at a lost / regained match.
// Poses that match no table row (other weapons, vanilla clips: "Game Animations" pack, sprint / crouch / slide clips, final strikes
// whose engine playback departs from the clip) are left exactly as the game and MM produced them. Runs BEFORE the melee aim tilt, so
// the tilt composes on top of the corrected chest.

#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>

#include "AimOffset.hpp"
#include "GameIntegration.hpp"
#include "MMMath.hpp"
#include "MeleeBaseData.hpp"
#include "MeleeBodyId.hpp"

namespace utpp::mm::meleebase
{
using aim::Xf;
using math::Quat;
using math::Vec3;

inline constexpr float kDegF = aim::kDegF;
inline constexpr float kRadToDeg = 57.2957795f;
inline constexpr float kConfFullDeg = 5.5f;  // arms / neck / head rms (deg) against the nearest row up to which a match counts in full
inline constexpr float kConfZeroDeg = 10.0f; // ... and from which on it is ignored (smoothstep between; measured: right rows 0.2..5, final_attack_03 on Female 10)
inline constexpr float kGateRiseHz = 40.0f;  // gate rise rate (1/s): ~25 ms time constant
inline constexpr float kGateFallHz = 12.0f;  // gate fall rate after a lost match: ~80 ms (starts after kGateHoldSeconds)
inline constexpr float kGateHoldSeconds = 0.06f; // a lost match keeps the last rotation at full strength this long (3 frames at 60 fps): the
                                             // engine's one-frame reference pose / a one-frame mismatch must not dip the chest; then it fades
inline constexpr float kRotSmoothHz = 40.0f;     // exponential smoothing of the applied rotation (time constant 25 ms, frame-rate independent)
inline constexpr float kMaxRotDegPerSecond = 480.0f; // and a rate limit on it (8 deg per frame at 60 fps)
inline constexpr float kChestLeadRows = 1.0f;    // the chest the correction aims at is the clip's chest this many table rows (1/30 s each) AHEAD of the
                                             // matched time: the applied rotation follows its target with a delay (25 ms smoothing, gate, rate limit),
                                             // so under a fast turn of the chest (the hips twist of a cross: 25-30 deg in 3 frames) it was 10-15 deg
                                             // short at the impact; aiming at where the clip's chest is going cancels the delay without touching the
                                             // smoothing / rate limit (the clip's future is known exactly: no extrapolation noise). 1 row = 33 ms is
                                             // about the delay of the 25 ms smoothing plus the gate; a longer lead follows the ramps more tightly but
                                             // also makes the correction run ahead of the step it is rate limited to (more per-frame change)
inline constexpr float kStickyRatio = 0.5f;      // another clip's row replaces the current clip's neighbouring rows only when its key distance is
                                             // below this fraction of theirs (rms ratio ~0.7)
inline constexpr float kEnableSeconds = 0.2f;    // ease in / out of the fists / standing flag (0.15 s melee fade style)
inline constexpr float kMaxDt = 0.1f;
inline constexpr float kMaxGapSeconds = 0.25f;
inline constexpr float kShare = 1.0f / 3.0f;     // of the rotation each of Spine1 / Spine2 / Spine3 turns
inline constexpr float kGuardEnterM = 0.30f;     // |engine hips z - MM hips z| above which the engine lower body is not used ...
inline constexpr float kGuardLeaveM = 0.20f;     // ... until it is back below this (hysteresis: no snap between engine and MM legs)

// ------------------------------------------------------------------------------------------------ rig

struct Rig
{
    int32_t hips = -1, spine = -1, spine1 = -1, spine2 = -1, spine3 = -1;
    int32_t key[kKeyBones];
    int16_t chain[16] = {};  // root ... Spine3 (bone indices)
    uint32_t chainLen = 0;
    uint32_t fkCount = 0;    // bones [0, fkCount) have a valid parent
    uint32_t ok = 0;         // 1 = every check passed (fail closed: 0 -> never applied)
    int16_t parents[aim::kMaxFkBones] = {};
    Rig()
    {
        for (int32_t& k : key)
            k = -1;
    }
};
static_assert(sizeof(Rig) % 4 == 0);

// Resolves Hips / Spine / Spine1..3 and the key bones by name (game CName hash). Fail closed: ok = 0 when a name is missing, an index
// used by the FK chain is >= aim::kMaxFkBones, parents are not topological, or Hips -> Spine -> Spine1 -> Spine2 -> Spine3 is not a
// chain of direct children.
inline Rig ResolveRig(std::span<const uint64_t> aHashes, std::span<const int16_t> aParents)
{
    Rig r;
    if (aHashes.empty() || aParents.size() != aHashes.size())
        return r;
    auto find = [&](std::string_view aName) -> int32_t {
        const uint64_t h = integ::CNameHash(aName);
        for (size_t i = 0; i < aHashes.size(); ++i)
            if (aHashes[i] == h)
                return static_cast<int32_t>(i);
        return -1;
    };
    r.hips = find("Hips");
    r.spine = find("Spine");
    r.spine1 = find("Spine1");
    r.spine2 = find("Spine2");
    r.spine3 = find("Spine3");
    for (int k = 0; k < kKeyBones; ++k)
    {
        r.key[k] = find(kKeyNames[k]);
        if (r.key[k] < 0)
            return Rig{};
    }
    const int32_t c[5] = {r.hips, r.spine, r.spine1, r.spine2, r.spine3};
    for (int32_t v : c)
        if (v < 0 || v >= int32_t(aim::kMaxFkBones))
            return Rig{};
    if (aParents[size_t(r.spine)] != r.hips || aParents[size_t(r.spine1)] != r.spine || aParents[size_t(r.spine2)] != r.spine1 ||
        aParents[size_t(r.spine3)] != r.spine2)
        return Rig{};
    r.fkCount = static_cast<uint32_t>(r.spine3 + 1);
    for (uint32_t i = 0; i < r.fkCount; ++i)
    {
        r.parents[i] = aParents[i];
        if (aParents[i] >= int32_t(i))
            return Rig{};
    }
    // root -> Spine3 chain
    int16_t rev[16];
    uint32_t n = 0;
    for (int32_t b = r.spine3; b >= 0 && n < 16;)
    {
        rev[n++] = static_cast<int16_t>(b);
        b = r.parents[b];
    }
    if (n == 0 || n >= 16 || r.parents[rev[n - 1]] >= 0)
        return Rig{};
    r.chainLen = n;
    for (uint32_t i = 0; i < n; ++i)
        r.chain[i] = rev[n - 1 - i];
    r.ok = 1;
    return r;
}

// Model transforms of the chain bones only (entries of aModel for other bones are not touched). False for a non-finite / degenerate
// record or a bone index >= aCount.
inline bool ChainModel(const uint8_t* aBones, uint32_t aCount, const Rig& aRig, Xf* aModel)
{
    if (!aRig.ok || !aBones || aRig.fkCount > aCount)
        return false;
    for (uint32_t i = 0; i < aRig.chainLen; ++i)
    {
        const int32_t b = aRig.chain[i];
        Xf l = aim::ReadLocal(aBones, uint32_t(b));
        if (!math::IsFinite(l.p) || !math::IsFinite(l.q))
            return false;
        const float n2 = math::QuatDot(l.q, l.q);
        if (!(n2 > 0.25f) || !(n2 < 4.0f))
            return false;
        l.q = math::QuatNormalize(l.q);
        const int32_t par = aRig.parents[b];
        aModel[b] = par < 0 ? l : aim::Mul(aModel[par], l);
    }
    return true;
}

// ------------------------------------------------------------------------------------------------ table match

inline Quat DecodeQuat(const int16_t* aP)
{
    constexpr float k = 1.0f / 32767.0f;
    return Quat{float(aP[0]) * k, float(aP[1]) * k, float(aP[2]) * k, float(aP[3]) * k};
}

struct Match
{
    bool ok = false;
    int32_t frame = -1;    // nearest table row
    int32_t neighbour = -1; // the better neighbouring row of the same clip (-1 = none)
    float t = 0.0f;        // 0 = frame ... 0.5 = halfway to the neighbour
    float rmsDeg = 1e9f;   // rms angle of the key bones against the nearest row
    Quat chest{};          // chest orientation (model space) of the clip at the interpolated time
    Quat chestLead{};      // ... kChestLeadRows rows later, inside the same clip (held at its last row): what the correction aims at
    uint32_t clip = 0;
};

// Weighted sum over the key bones of (1 - |dot|) (= angle^2 / 8 for small angles), early out above aStop.
inline float KeyDistance(const TableData& aT, uint32_t aRow, const Quat* aKey, float aStop)
{
    const int16_t* row = aT.frames + size_t(aRow) * size_t(kRowInts);
    float sum = 0.0f;
    for (int i = 0; i < kKeyBones; ++i)
    {
        const int k = kKeyOrder[i];
        const Quat q = DecodeQuat(row + size_t(k) * 4u);
        const float d = std::fabs(math::QuatDot(q, aKey[k]));
        sum += kKeyWeight[k] * (1.0f - (d > 1.0f ? 1.0f : d));
        if (sum > aStop)
            return sum;
    }
    return sum;
}

// Nearest row of aT to the key rotations aKey (kKeyBones local quaternions of the live pose, kKeyNames order; any sign); rmsDeg = rms angle
// of the first kMotionKeys keys (the clip's own motion) against the nearest row. aHint = the row chosen on the previous call (-1 = none):
// the choice is STICKY - the rows of the hint's clip within 4 of it win unless a row elsewhere has a key distance below kStickyRatio of
// theirs (no A-B-A flipping between clips whose frames look alike). Not ok for an empty table / non-finite input.
inline bool FindMatch(const TableData& aT, const Quat* aKey, int32_t aHint, Match& aOut)
{
    aOut = Match{};
    if (!aT.frames || aT.frameCount == 0)
        return false;
    Quat key[kKeyBones];
    for (int i = 0; i < kKeyBones; ++i)
    {
        if (!math::IsFinite(aKey[i]))
            return false;
        key[i] = math::QuatNormalize(aKey[i]);
    }
    float best = 1e9f;
    int32_t bi = -1;
    if (aHint >= 0 && uint32_t(aHint) < aT.frameCount)
    {
        const int32_t lo = aHint > 4 ? aHint - 4 : 0;
        const int32_t hi = uint32_t(aHint + 4) < aT.frameCount ? aHint + 4 : int32_t(aT.frameCount) - 1;
        for (int32_t i = lo; i <= hi; ++i)
        {
            if (aT.clipOfFrame[i] != aT.clipOfFrame[aHint])
                continue;
            const float d = KeyDistance(aT, uint32_t(i), key, best);
            if (d < best)
            {
                best = d;
                bi = i;
            }
        }
    }
    // the sticky clip's distance is that of the pose to its POLYLINE (the engine plays 60 fps between the 30 fps rows; a sibling clip whose rows
    // happen to sit at the half-frame offsets would otherwise win every other frame): nearest row b, better neighbour n, height of the pose
    // over the segment (b, n) by the law of cosines on the (additive, ~angle^2) distances
    float sticky = best;
    if (bi >= 0)
    {
        float dn = 1e9f;
        int32_t ni = -1;
        for (int32_t cand : {bi - 1, bi + 1})
            if (cand >= 0 && uint32_t(cand) < aT.frameCount && aT.clipOfFrame[cand] == aT.clipOfFrame[bi])
            {
                const float d = KeyDistance(aT, uint32_t(cand), key, 1e9f);
                if (d < dn)
                {
                    dn = d;
                    ni = cand;
                }
            }
        if (ni >= 0)
        {
            Quat rowN[kKeyBones];
            for (int k = 0; k < kKeyBones; ++k)
                rowN[k] = DecodeQuat(aT.frames + size_t(ni) * size_t(kRowInts) + size_t(k) * 4u);
            const float c2 = KeyDistance(aT, uint32_t(bi), rowN, 1e9f); // squared distance between the two rows
            if (c2 > 1e-9f)
            {
                const float p = (best + c2 - dn) / (2.0f * std::sqrt(c2)); // projection of the pose on the segment
                const float h2 = best - p * p;
                sticky = h2 > 0.0f ? h2 : 0.0f;
            }
        }
    }
    float bound = bi >= 0 ? sticky * kStickyRatio : 1e9f; // a row elsewhere has to beat the sticky window by this margin
    int32_t gi = -1;
    for (uint32_t i = 0; i < aT.frameCount; ++i)
    {
        const float d = KeyDistance(aT, i, key, bound);
        if (d < bound)
        {
            bound = d;
            gi = int32_t(i);
        }
    }
    if (gi >= 0)
    {
        best = bound;
        bi = gi;
    }
    if (bi < 0 || !(best < 1e8f))
        return false;
    aOut.frame = bi;
    aOut.clip = aT.clipOfFrame[bi];
    // the confidence reads the clip's own motion only (arms, neck, head): the spine locals carry the game's small constant offset
    {
        float sumArms = 0.0f;
        const int16_t* row = aT.frames + size_t(bi) * size_t(kRowInts);
        for (int k = 0; k < kMotionKeys; ++k)
        {
            const float d = std::fabs(math::QuatDot(DecodeQuat(row + size_t(k) * 4u), key[k]));
            sumArms += 1.0f - (d > 1.0f ? 1.0f : d);
        }
        aOut.rmsDeg = std::sqrt(8.0f * sumArms / float(kMotionKeys)) * kRadToDeg;
    }
    const Quat c0 = DecodeQuat(aT.frames + size_t(bi) * size_t(kRowInts) + size_t(kKeyBones) * 4u);
    aOut.chest = math::QuatNormalize(c0);
    // interpolation with the better neighbour of the same clip (the engine plays between the 30 fps rows)
    float bestN = 1e9f;
    int32_t ni = -1;
    for (int32_t cand : {bi - 1, bi + 1})
        if (cand >= 0 && uint32_t(cand) < aT.frameCount && aT.clipOfFrame[cand] == aT.clipOfFrame[bi])
        {
            const float d = KeyDistance(aT, uint32_t(cand), key, 1e9f);
            if (d < bestN)
            {
                bestN = d;
                ni = cand;
            }
        }
    if (ni >= 0)
    {
        const float d0 = std::sqrt(best), d1 = std::sqrt(bestN);
        const float t = (d0 + d1) > 1e-9f ? d0 / (d0 + d1) : 0.0f;
        const Quat c1 = DecodeQuat(aT.frames + size_t(ni) * size_t(kRowInts) + size_t(kKeyBones) * 4u);
        aOut.neighbour = ni;
        aOut.t = t;
        aOut.chest = math::QuatNlerpShortest(aOut.chest, math::QuatNormalize(c1), t);
    }
    {
        // position in rows of the matched time (t is the fraction towards the neighbour, +-1 row), plus the lead; rows of the clip only
        const float pos0 = float(bi) + (ni >= 0 ? aOut.t * float(ni - bi) : 0.0f) + kChestLeadRows;
        int32_t last = bi;
        while (uint32_t(last + 1) < aT.frameCount && aT.clipOfFrame[last + 1] == aT.clipOfFrame[bi])
            ++last;
        const float p = pos0 > float(last) ? float(last) : pos0;
        const int32_t i0 = static_cast<int32_t>(p);
        const int32_t i1 = i0 + 1 <= last ? i0 + 1 : last;
        const Quat q0 = math::QuatNormalize(DecodeQuat(aT.frames + size_t(i0) * size_t(kRowInts) + size_t(kKeyBones) * 4u));
        const Quat q1 = math::QuatNormalize(DecodeQuat(aT.frames + size_t(i1) * size_t(kRowInts) + size_t(kKeyBones) * 4u));
        aOut.chestLead = math::QuatNlerpShortest(q0, q1, p - float(i0));
    }
    aOut.ok = true;
    return true;
}

// Confidence of a match: 1 up to kConfFullDeg, 0 from kConfZeroDeg, smoothstep between.
inline float Confidence(float aRmsDeg)
{
    if (!std::isfinite(aRmsDeg))
        return 0.0f;
    const float x = aim::Clampf((aRmsDeg - kConfFullDeg) / (kConfZeroDeg - kConfFullDeg), 0.0f, 1.0f);
    const float s = x * x * (3.0f - 2.0f * x);
    return 1.0f - s;
}

// Rotation vector (axis * angle, radians) of a quaternion (shortest way) and back.
inline Vec3 ToRotationVector(const Quat& aQ)
{
    const Quat q = math::QuatNormalize(aQ);
    const float s = q.w < 0.0f ? -1.0f : 1.0f;
    const Vec3 v{q.x * s, q.y * s, q.z * s};
    const float vn = aim::Length(v);
    if (!(vn > 1e-7f))
        return Vec3{};
    const float angle = 2.0f * std::atan2(vn, q.w * s);
    return v * (angle / vn);
}

inline Quat FromRotationVector(const Vec3& aV)
{
    const float angle = aim::Length(aV);
    if (!(angle > 1e-7f) || !std::isfinite(angle))
        return Quat{};
    return aim::AxisAngle(aV * (1.0f / angle), angle);
}

// ------------------------------------------------------------------------------------------------ limits

// Natural limits of the rotation Q the chest may be turned by relative to the hips / spine the game shows. Q = Rz(yaw) Rx(pitch)
// Ry(roll) (intrinsic z-x-y, model space: +Z up, +Y forward, +X right); pitch < 0 bends the chest forward.
struct Limits
{
    float forwardDeg = 35.0f; // forward bend (pitch < 0): the clip's boxing lean is 9..16 deg of hips + the spine, so 35 covers it
    float backDeg = 20.0f;    // backward bend
    float yawDeg = 45.0f;     // twist about the vertical axis, both ways (the clip's own hips twist is lost, the chest takes it all)
    float rollDeg = 20.0f;    // side bend
    float totalDeg = 45.0f;   // the whole rotation (bend + twist + roll together): the three components alone could add up to ~60 deg
};

enum LimitFlag : uint32_t
{
    kLimForward = 1u << 0,
    kLimBack = 1u << 1,
    kLimYaw = 1u << 2,
    kLimRoll = 1u << 3,
    kLimTotal = 1u << 4,
};

inline constexpr float kKnee = 0.8f; // the limit starts to bite at this fraction of it

// Odd, C1 soft saturation: identity up to kKnee * aLimit, then it approaches aLimit asymptotically (tanh), never exceeds it.
inline float SoftLimit(float aDeg, float aLimit)
{
    if (!(aLimit > 0.0f))
        return 0.0f;
    const float a = std::fabs(aDeg);
    const float knee = kKnee * aLimit;
    if (a <= knee)
        return aDeg;
    const float room = aLimit - knee;
    const float o = knee + room * std::tanh((a - knee) / room);
    return aDeg < 0.0f ? -o : o;
}

struct Euler
{
    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f; // radians
};

// Intrinsic z-x-y decomposition of a unit quaternion (Q = Rz(yaw) Rx(pitch) Ry(roll)); pitch in [-90, 90] deg.
inline Euler ToEuler(const Quat& aQ)
{
    const Quat q = math::QuatNormalize(aQ);
    const float x = q.x, y = q.y, z = q.z, w = q.w;
    const float m21 = 2.0f * (y * z + x * w);
    const float m01 = 2.0f * (x * y - z * w);
    const float m11 = 1.0f - 2.0f * (x * x + z * z);
    const float m20 = 2.0f * (x * z - y * w);
    const float m22 = 1.0f - 2.0f * (x * x + y * y);
    Euler e;
    e.pitch = std::asin(aim::Clampf(m21, -1.0f, 1.0f));
    e.yaw = std::atan2(-m01, m11);
    e.roll = std::atan2(-m20, m22);
    return e;
}

inline Quat FromEuler(const Euler& aE)
{
    const Quat qz = aim::AxisAngle(Vec3{0.0f, 0.0f, 1.0f}, aE.yaw);
    const Quat qx = aim::AxisAngle(Vec3{1.0f, 0.0f, 0.0f}, aE.pitch);
    const Quat qy = aim::AxisAngle(Vec3{0.0f, 1.0f, 0.0f}, aE.roll);
    return math::QuatNormalize(math::QuatMul(qz, math::QuatMul(qx, qy)));
}

// Limits the rotation aQ; aFlags (optional) gets the components that were beyond their limit.
inline Quat LimitRotation(const Quat& aQ, const Limits& aL, uint32_t* aFlags = nullptr)
{
    if (aFlags)
        *aFlags = 0;
    if (!math::IsFinite(aQ))
        return Quat{};
    const Euler e = ToEuler(aQ);
    const float yawD = e.yaw / kDegF, pitchD = e.pitch / kDegF, rollD = e.roll / kDegF;
    uint32_t f = 0;
    if (pitchD < -aL.forwardDeg)
        f |= kLimForward;
    if (pitchD > aL.backDeg)
        f |= kLimBack;
    if (std::fabs(yawD) > aL.yawDeg)
        f |= kLimYaw;
    if (std::fabs(rollD) > aL.rollDeg)
        f |= kLimRoll;
    Euler o;
    o.pitch = (pitchD < 0.0f ? -SoftLimit(-pitchD, aL.forwardDeg) : SoftLimit(pitchD, aL.backDeg)) * kDegF;
    o.yaw = SoftLimit(yawD, aL.yawDeg) * kDegF;
    o.roll = SoftLimit(rollD, aL.rollDeg) * kDegF;
    Quat out = FromEuler(o);
    // total angle cap (same soft saturation): the turn keeps its axis, only its size is held
    Vec3 rv = ToRotationVector(out);
    const float angD = aim::Length(rv) / kDegF;
    if (angD > aL.totalDeg)
        f |= kLimTotal;
    if (angD > kKnee * aL.totalDeg)
        out = FromRotationVector(rv * (SoftLimit(angD, aL.totalDeg) / angD));
    if (aFlags)
        *aFlags = f;
    return out;
}

// ------------------------------------------------------------------------------------------------ gate / smoothing

// Per-call state of the layer. The limited rotation of the last match is the TARGET; the applied rotation follows it with an exponential
// smoothing (kRotSmoothHz) and a rate limit (kMaxRotDegPerSecond), frame-rate independent. The gate eases the layer in / out around a (lost)
// match: the first call after Reset snaps (the melee fade already eases the layer in); a lost match keeps the last rotation at full
// strength for kGateHoldSeconds (a one-frame mismatch, e.g. the engine's reference-pose frame, must not dip the chest) and then fades it out
// over ~80 ms; a regained match eases back in over ~25 ms.
class Gate
{
public:
    void Reset()
    {
        m_gate = 0.0f;
        m_target = Vec3{};
        m_cur = Vec3{};
        m_lost = 0.0f;
        m_fresh = true;
    }
    float Value() const { return m_gate; }
    Vec3 Current() const { return m_cur; }
    // aMatched / aConfidence / aRot: this call's match (aRot = limited rotation vector); aWeight: melee fade x game pose share x enable.
    // Returns the rotation vector to apply (smoothed rotation * gate * weight).
    Vec3 Step(bool aMatched, float aConfidence, const Vec3& aRot, float aWeight, float aDt)
    {
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? (aDt > kMaxDt ? kMaxDt : aDt) : 0.0f;
        const bool have = aMatched && std::isfinite(aConfidence) && math::IsFinite(aRot);
        const float target = have ? aim::Clampf(aConfidence, 0.0f, 1.0f) : 0.0f;
        if (have)
        {
            m_target = aRot;
            m_lost = 0.0f;
        }
        else
            m_lost += dt;
        if (m_fresh)
        {
            m_gate = target;
            m_cur = m_target;
            m_fresh = false;
        }
        else
        {
            if (have || m_lost > kGateHoldSeconds)
            {
                const float rate = target > m_gate ? kGateRiseHz : kGateFallHz;
                m_gate += (target - m_gate) * (1.0f - std::exp(-rate * dt));
                if (!(m_gate > 1e-4f) && target <= 0.0f)
                    m_gate = 0.0f;
            }
            // smoothing + rate limit of the applied rotation toward the target
            Vec3 step = (m_target - m_cur) * (1.0f - std::exp(-kRotSmoothHz * dt));
            const float maxStep = kMaxRotDegPerSecond * kDegF * dt;
            const float len = aim::Length(step);
            if (len > maxStep && len > 0.0f)
                step = step * (maxStep / len);
            m_cur = m_cur + step;
        }
        const float w = std::isfinite(aWeight) ? aim::Clampf(aWeight, 0.0f, 1.0f) : 0.0f;
        return m_cur * (m_gate * w);
    }

private:
    float m_gate = 0.0f;
    float m_lost = 0.0f;
    Vec3 m_target{};
    Vec3 m_cur{};
    bool m_fresh = true;
};

// ------------------------------------------------------------------------------------------------ apply

// Live chest (Spine3) orientation in model space.
inline bool LiveChest(const uint8_t* aBones, uint32_t aCount, const Rig& aRig, Quat& aOut)
{
    Xf model[aim::kMaxFkBones];
    if (!ChainModel(aBones, aCount, aRig, model))
        return false;
    aOut = model[aRig.spine3].q;
    return true;
}

// Rotation (model space) that turns the live chest into the authored one: aAuthored * inverse(aLive).
inline Quat ChestDelta(const Quat& aAuthored, const Quat& aLive)
{
    return math::QuatNormalize(math::QuatMul(math::QuatNormalize(aAuthored), math::QuatInv(math::QuatNormalize(aLive))));
}

// Turns Spine1 / Spine2 / Spine3 by a third of the rotation vector aRot each (model-space axis, about each joint). Everything under
// Spine3 ends up rotated by exactly aRot. Returns false (records untouched) when the rig / pose cannot be solved.
inline bool ApplyChestRotation(uint8_t* aBones, uint32_t aCount, const Rig& aRig, const Vec3& aRot)
{
    if (!aRig.ok || !aBones || aRig.fkCount > aCount || !math::IsFinite(aRot))
        return false;
    const float angle = aim::Length(aRot);
    if (!(angle > 1e-6f))
        return true; // nothing to rotate
    const Vec3 axis = aRot * (1.0f / angle);
    const int32_t joints[3] = {aRig.spine1, aRig.spine2, aRig.spine3};
    Xf model[aim::kMaxFkBones];
    for (int k = 0; k < 3; ++k)
    {
        if (!ChainModel(aBones, aCount, aRig, model))
            return false; // (rare: the previous joint is already written)
        aim::RotateAboutJoint(aBones, model, aRig.parents, uint32_t(joints[k]), aim::AxisAngle(axis, kShare * angle));
    }
    return true;
}

// Reads the key bones' local rotations from the records.
inline bool ReadKeys(const uint8_t* aBones, uint32_t aCount, const Rig& aRig, Quat* aKey)
{
    if (!aRig.ok || !aBones)
        return false;
    for (int i = 0; i < kKeyBones; ++i)
    {
        if (aRig.key[i] < 0 || uint32_t(aRig.key[i]) >= aCount)
            return false;
        aKey[i] = aim::ReadLocal(aBones, uint32_t(aRig.key[i])).q;
        if (!math::IsFinite(aKey[i]))
            return false;
    }
    return true;
}

// ------------------------------------------------------------------------------------------------ one FK call

struct Hints
{
    int32_t row = -1; // the row chosen on the previous call (sticky choice, see FindMatch)
};

struct StepInfo
{
    bool matched = false;     // a table row with confidence > 0 was found
    bool solveFailed = false; // the live chest / the rotation could not be solved
    int32_t body = -1;        // 0 Female table, 1 Male table
    uint32_t clip = 0;
    int32_t frame = -1;
    int32_t neighbour = -1;   // better neighbouring row of the same clip (interpolation partner)
    float t = 0.0f;           // 0 = frame ... 0.5 = halfway to the neighbour
    float rmsDeg = 0.0f, conf = 0.0f, gate = 0.0f;
    Euler raw, lim;           // rotation asked for / after the limits (radians)
    uint32_t flags = 0;       // LimitFlag
    Vec3 applied{};           // rotation vector actually applied (limited x gate x weight)
};

// The whole layer for one FK call on the local records (the runtime and the offline replay share it): aBody (0 Female, 1 Male: the
// armed body) picks the table; key bones -> table match ->
// chest delta -> limits -> gate / weight -> rotation spread over Spine1..3. aWeight = melee fade x game pose share (0 = nothing),
// aDt = seconds since the previous call (0 on the first). Returns false when a rotation could not be applied (records untouched).
// aEngineKeys (optional, kKeyBones quaternions in kKeyNames order, read from the records BEFORE the MM blend): the match is made on
// the pose the game plays, not on the records as they are now. While the upper-body mask blends in (0.15 s) the records are a mix of
// the MM pose and the game pose, which matches no clip row (rms 10..23 deg), so the correction could only start once the mix was
// nearly all game pose - and then climbed from zero while the first punch landed. The chest the correction acts on is still the live
// one. Without aEngineKeys (null) the records themselves are matched, as before.
inline bool CorrectChest(uint8_t* aBones, uint32_t aCount, const Rig& aRig, int32_t aBody, Hints& aHints, Gate& aGate, float aWeight, float aDt, StepInfo* aInfo = nullptr,
                         const Quat* aEngineKeys = nullptr)
{
    StepInfo info;
    bool matched = false;
    float conf = 0.0f;
    Vec3 rot{};
    Quat key[kKeyBones];
    bool haveKeys = false;
    if (aEngineKeys)
    {
        haveKeys = true;
        for (int i = 0; i < kKeyBones; ++i)
            key[i] = aEngineKeys[i];
    }
    else
        haveKeys = ReadKeys(aBones, aCount, aRig, key);
    if (haveKeys)
    {
        // ONE table: the armed body's (no scan of the other body)
        Match mt;
        const TableData& table = aBody == 1 ? MaleTable() : FemaleTable();
        const bool okT = (aBody == 0 || aBody == 1) && FindMatch(table, key, aHints.row, mt);
        if (okT)
            aHints.row = mt.frame;
        const Match& best = mt;
        if (okT && best.ok)
        {
            Quat live;
            if (LiveChest(aBones, aCount, aRig, live))
            {
                const Quat raw = ChestDelta(best.chestLead, live);
                const Quat lim = LimitRotation(raw, Limits{}, &info.flags);
                rot = ToRotationVector(lim);
                conf = Confidence(best.rmsDeg);
                matched = conf > 0.0f;
                info.raw = ToEuler(raw);
                info.lim = ToEuler(lim);
                info.rmsDeg = best.rmsDeg;
                info.body = aBody;
                info.clip = best.clip;
                info.frame = best.frame;
                info.neighbour = best.neighbour;
                info.t = best.t;
            }
            else
                info.solveFailed = true;
        }
    }
    info.matched = matched;
    info.conf = conf;
    info.applied = aGate.Step(matched, conf, rot, aWeight, aDt);
    info.gate = aGate.Value();
    bool ok = true;
    if (aim::Length(info.applied) > 1e-6f)
        ok = ApplyChestRotation(aBones, aCount, aRig, info.applied);
    if (!ok)
    {
        info.solveFailed = true;
        info.applied = Vec3{};
    }
    if (aInfo)
        *aInfo = info;
    return ok && !info.solveFailed;
}

// ------------------------------------------------------------------------------------------------ lower body guard

// The melee full body mode hands Hips / Spine / legs to the engine pose. For Male V that pose is a kneeling base (Hips at 0.36 m
// instead of 0.93 m, measured in every Male pose recording since 2026-11-12 14:13, also before the melee release change): no chest
// rotation can undo a 45 deg lean back and the legs kneel. The guard keeps the lower body on MM when the engine Hips are not where MM's
// are (dz > kGuardEnterM, back only below kGuardLeaveM: the engine base depends on the session history, so it may change or hover
// near a single threshold during one episode). Female V's engine base stands (hips 0.94..0.99 m, MM 0.93..0.99 m): unchanged.
// aPrevKeep = the previous decision of this episode (false at its start); a non-finite input keeps the previous decision.
inline bool KeepLowerOnMM(bool aPrevKeep, float aEngineHipsZ, float aMMHipsZ)
{
    if (!std::isfinite(aEngineHipsZ) || !std::isfinite(aMMHipsZ))
        return aPrevKeep;
    const float dz = std::fabs(aEngineHipsZ - aMMHipsZ);
    return aPrevKeep ? dz > kGuardLeaveM : dz > kGuardEnterM;
}
} // namespace utpp::mm::meleebase
