// Ground dodge clips (pure logic, no game types).
//
// The game's ground dodge (PSM detail 7) moves the capsule with a short impulse: the first dodge feed still carries the old
// velocity, the next one the impulse (9 - 15 m/s) that decays within about 0.2 s. This header provides the pieces to show one of
// four dodge clips of utpp_dodge_<body>.utpptrav during it (the traversal file grammar, version 2: montages Dodge_F, Dodge_B,
// Dodge_L, Dodge_R, Root-relative in-place poses); the clip's own root travel is not part of the pose (the capsule moves the
// body). The motion driver (MotionDriver.cpp) shows the clips on the standing ground.
//
//   DodgeClips    validation of the parsed file against the database bones (fail closed: ok = false, err set).
//   SampleDodge   the local pose of a clip at a clip time (frame lerp + quaternion nlerp), 7 floats per database bone.
//   ClassifySector  direction of the impulse relative to the body's facing, four sectors with hysteresis at the borders.
//   UpperBodyMask / KeepUpperBody / UpperBodyRelease  the bones above Spine3 keep the pose shown before the clip and move to
//                 the motion pose underneath over the last kUpperReleaseS of the clip.
//   DodgePlayer   per feed: arms on the dodge's first feed, picks the clip on the impulse feed, runs the clip clock at kRate,
//                 hands back at the clip end or when the body leaves the standing ground state.
//
// Yaw convention: 0 = +Y, counter-clockwise positive (a velocity (vx, vy) has yaw atan2(-vx, vy); positive = to the left).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "TraversalFile.hpp"

namespace utpp::mm::dodge
{
// Test mutations: 0 / undefined = production. 1 no hysteresis at the sector borders, 2 the clip clock counts feeds instead of
// time, 3 the clip is picked on the dodge's first feed (old velocity), 4 the loader accepts a clip with a missing pose bone.
#ifndef UTPP_DODGE_MUTATION
#define UTPP_DODGE_MUTATION 0
#endif

enum class Sector : int32_t
{
    None = -1,
    Forward = 0,
    Back = 1,
    Left = 2,
    Right = 3,
};
inline constexpr int kSectorCount = 4;
inline constexpr const char* kMontageNames[kSectorCount] = {"Dodge_F", "Dodge_B", "Dodge_L", "Dodge_R"};
inline const char* SectorName(Sector s)
{
    return s == Sector::None ? "-" : kMontageNames[static_cast<int>(s)] + 6;
}

// clip clock rate (the recovery ends about when the dodge impulse has died out); UTPP_DODGE_RATE overrides it in measurement builds only
#ifndef UTPP_DODGE_RATE
#define UTPP_DODGE_RATE 1.5f
#endif
inline constexpr float kRate = UTPP_DODGE_RATE;
// inertialized entry from the playing pose; UTPP_DODGE_BLEND_IN overrides it in measurement builds only
#ifndef UTPP_DODGE_BLEND_IN
#define UTPP_DODGE_BLEND_IN 0.20f
#endif
inline constexpr float kBlendInS = UTPP_DODGE_BLEND_IN;
inline constexpr float kBlendOutS = 0.20f;     // inertialized hand-back to the motion search
inline constexpr float kHysteresisDeg = 10.0f; // a sector border must be crossed by this much to change the sector
inline constexpr float kImpulseRiseMps = 1.5f; // the impulse feed: speed above the first feed's speed by this much
inline constexpr float kImpulseWaitS = 0.06f;  // without a rise the direction is read after this much dodge time
inline constexpr float kMinImpulseMps = 1.0f;  // slower than this on the impulse feed: no clip (no direction)
inline constexpr float kSettleS = 0.05f;       // the sector may still change (past the hysteresis) this long after the pick
inline constexpr float kMaxStepS = 0.1f;       // one feed counts for at most this much time

inline float WrapDeg(float a)
{
    if (!std::isfinite(a))
        return 0.0f;
    a = std::fmod(a + 180.0f, 360.0f);
    if (a < 0.0f)
        a += 360.0f;
    return a - 180.0f;
}

// Sector of the direction aRelDeg (velocity yaw - facing yaw, positive = left). Borders at +-45 and +-135 degrees. With a
// previous sector, the previous one is kept while aRelDeg stays inside its range widened by aHystDeg on both sides.
inline Sector ClassifySector(float aRelDeg, Sector aPrev = Sector::None, float aHystDeg = kHysteresisDeg)
{
    const float a = WrapDeg(aRelDeg);
#if UTPP_DODGE_MUTATION == 1
    aHystDeg = 0.0f;
#endif
    const auto inside = [&](Sector s, float h) {
        switch (s)
        {
        case Sector::Forward: return std::fabs(a) <= 45.0f + h;
        case Sector::Back: return std::fabs(a) >= 135.0f - h;
        case Sector::Left: return a >= 45.0f - h && a <= 135.0f + h;
        case Sector::Right: return a <= -45.0f + h && a >= -135.0f - h;
        default: return false;
        }
    };
    if (aPrev != Sector::None && inside(aPrev, std::fmax(0.0f, aHystDeg)))
        return aPrev;
    if (std::fabs(a) <= 45.0f)
        return Sector::Forward;
    if (std::fabs(a) >= 135.0f)
        return Sector::Back;
    return a > 0.0f ? Sector::Left : Sector::Right;
}

// ------------------------------------------------------------------------------------------------ data
struct DodgeClips
{
    bool ok = false;
    std::string err = "missing";
    size_t poseBones = 0;
    std::vector<int32_t> poseMap; // pose bone of every database bone
    std::array<const traversal::TravMontage*, kSectorCount> clip{};
    std::array<float, kSectorCount> length{}; // seconds of clip time
};

// Validates aFile (kept alive by the caller: the clip pointers point into it) against the database bones. aName(i) = name of
// database bone i. Every database bone must have a pose bone; all four montages must exist with >= 2 frames and finite poses.
template <class NameFn>
inline bool BuildDodgeClips(const traversal::TraversalFile& aFile, const std::string& aRigName, uint32_t aN, NameFn&& aName, DodgeClips& aOut)
{
    aOut = DodgeClips{};
    const auto fail = [&](const std::string& aWhy) {
        aOut = DodgeClips{};
        aOut.err = aWhy;
        return false;
    };
    if (aN == 0)
        return fail("bone count");
    if (aFile.version != 2 || aFile.poseBones.empty())
        return fail("no poses (version 2 expected)");
    if (aFile.rigName != aRigName)
        return fail("rig " + aFile.rigName + " != " + aRigName);
    const size_t nb = aFile.poseBones.size();
    aOut.poseBones = nb;
    aOut.poseMap.assign(aN, -1);
    for (uint32_t i = 0; i < aN; ++i)
    {
        const std::string_view nm = aName(i);
        for (size_t k = 0; k < nb; ++k)
            if (aFile.poseBones[k] == nm)
                aOut.poseMap[i] = static_cast<int32_t>(k);
#if UTPP_DODGE_MUTATION == 4
        if (aOut.poseMap[i] < 0)
            aOut.poseMap[i] = 0;
#endif
        if (aOut.poseMap[i] < 0)
            return fail("pose bone missing: " + std::string(nm));
    }
    for (int s = 0; s < kSectorCount; ++s)
    {
        const traversal::TravMontage* m = aFile.Find(kMontageNames[s]);
        if (m == nullptr)
            return fail(std::string("montage missing: ") + kMontageNames[s]);
        if (!(m->fps > 0.0) || m->frames < 2 || m->pose.size() != static_cast<size_t>(m->frames) * nb * 7)
            return fail(std::string("montage poses: ") + kMontageNames[s]);
        for (float v : m->pose)
            if (!std::isfinite(v))
                return fail(std::string("non-finite pose: ") + kMontageNames[s]);
        aOut.clip[s] = m;
        aOut.length[s] = static_cast<float>((m->frames - 1) / m->fps);
    }
    aOut.ok = true;
    aOut.err.clear();
    return true;
}

// Local pose (x y z qx qy qz qw per database bone) of clip aSector at aTime (clamped to the clip).
inline bool SampleDodge(const DodgeClips& aC, Sector aSector, double aTime, float* aOut)
{
    if (!aC.ok || aSector == Sector::None || aOut == nullptr)
        return false;
    const traversal::TravMontage& m = *aC.clip[static_cast<int>(aSector)];
    double ft = (std::isfinite(aTime) ? aTime : 0.0) * m.fps;
    ft = std::clamp(ft, 0.0, static_cast<double>(m.frames - 1));
    const int f0 = static_cast<int>(ft);
    const int f1 = f0 + 1 < m.frames ? f0 + 1 : f0;
    const float u = static_cast<float>(ft - f0);
    const float* a = &m.pose[static_cast<size_t>(f0) * aC.poseBones * 7];
    const float* c = &m.pose[static_cast<size_t>(f1) * aC.poseBones * 7];
    for (size_t i = 0; i < aC.poseMap.size(); ++i)
    {
        const size_t k = static_cast<size_t>(aC.poseMap[i]) * 7;
        float* o = aOut + i * 7;
        for (int j = 0; j < 3; ++j)
            o[j] = a[k + j] + (c[k + j] - a[k + j]) * u;
        float qb[4] = {c[k + 3], c[k + 4], c[k + 5], c[k + 6]};
        if (a[k + 3] * qb[0] + a[k + 4] * qb[1] + a[k + 5] * qb[2] + a[k + 6] * qb[3] < 0.0f)
            for (float& x : qb)
                x = -x;
        float q[4], n = 0.0f;
        for (int j = 0; j < 4; ++j)
        {
            q[j] = a[k + 3 + j] + (qb[j] - a[k + 3 + j]) * u;
            n += q[j] * q[j];
        }
        n = n > 0.0f ? 1.0f / std::sqrt(n) : 0.0f;
        for (int j = 0; j < 4; ++j)
            o[3 + j] = n > 0.0f ? q[j] * n : (j == 3 ? 1.0f : 0.0f);
    }
    return true;
}

// Upper-body mask: 1 for every database bone below aTop (its descendants, not aTop itself). aParent(i) = parent of bone i (< 0 =
// none). Empty when aTop is not a bone. The clips drive the hips, the spine up to aTop and the legs; the masked bones keep the
// pose the body had when the dodge started and move to the motion pose underneath before the hand-back (KeepUpperBody).
template <class ParentFn>
inline std::vector<uint8_t> UpperBodyMask(uint32_t aN, int32_t aTop, ParentFn&& aParent)
{
    std::vector<uint8_t> mask;
    if (aTop < 0 || static_cast<uint32_t>(aTop) >= aN)
        return mask;
    mask.assign(aN, 0);
    for (uint32_t i = 0; i < aN; ++i)
    {
        int32_t p = aParent(i);
        for (uint32_t guard = 0; p >= 0 && guard < aN; ++guard)
        {
            if (p == aTop)
            {
                mask[i] = 1;
                break;
            }
            p = aParent(static_cast<uint32_t>(p));
        }
    }
    return mask;
}

// feed seconds before the clip end over which the upper body moves from the held pose to the motion pose underneath
inline constexpr float kUpperReleaseS = 0.15f;

// Weight of the motion pose underneath for the upper body at clip time aClipTime of a clip aLength long (clip seconds):
// 0 until kUpperReleaseS (feed time) before the end, then a smoothstep to 1 at the end.
inline float UpperBodyRelease(float aClipTime, float aLength)
{
    const float span = kUpperReleaseS * kRate;
    if (!(span > 0.0f) || !std::isfinite(aClipTime))
        return 0.0f;
    const float u = std::clamp((aClipTime - (aLength - span)) / span, 0.0f, 1.0f);
    return u * u * (3.0f - 2.0f * u);
}

// Writes the masked bones (7 floats each) of aPose: aHeld, moved towards aLive by aLiveW (translation lerp, quaternion nlerp
// on the short arc). aLive null or aLiveW <= 0: aHeld. No-op when the mask is empty or aHeld / aPose is null.
inline void KeepUpperBody(const std::vector<uint8_t>& aMask, const float* aHeld, float* aPose, const float* aLive = nullptr,
                          float aLiveW = 0.0f)
{
    if (aHeld == nullptr || aPose == nullptr)
        return;
    const float w = aLive != nullptr && std::isfinite(aLiveW) ? std::clamp(aLiveW, 0.0f, 1.0f) : 0.0f;
    for (size_t i = 0; i < aMask.size(); ++i)
    {
        if (aMask[i] == 0)
            continue;
        const float* h = aHeld + i * 7;
        float* o = aPose + i * 7;
        if (w <= 0.0f)
        {
            for (size_t j = 0; j < 7; ++j)
                o[j] = h[j];
            continue;
        }
        const float* l = aLive + i * 7;
        for (size_t j = 0; j < 3; ++j)
            o[j] = h[j] + (l[j] - h[j]) * w;
        const float sgn = h[3] * l[3] + h[4] * l[4] + h[5] * l[5] + h[6] * l[6] < 0.0f ? -1.0f : 1.0f;
        float q[4], n = 0.0f;
        for (size_t j = 0; j < 4; ++j)
        {
            q[j] = h[3 + j] + (sgn * l[3 + j] - h[3 + j]) * w;
            n += q[j] * q[j];
        }
        n = n > 0.0f ? 1.0f / std::sqrt(n) : 0.0f;
        for (size_t j = 0; j < 4; ++j)
            o[3 + j] = n > 0.0f ? q[j] * n : h[3 + j];
    }
}

// ------------------------------------------------------------------------------------------------ player
struct Input
{
    float dt = 0.0f;
    bool dodgeDetail = false; // PSM detail == ground Dodge 7 on this feed
    bool standingGround = false; // on the ground, standing stance, no slide / traversal / air / fallback (the clip may play)
    float velX = 0.0f, velY = 0.0f; // capsule velocity (world, m/s)
    float facingYawDeg = 0.0f;      // the body's facing (world yaw, degrees)
    float forwardClipMaxEntryMps = 0.0f; // a forward dodge entered faster than this keeps the playing clip (the run / sprint goes on)
};

struct Frame
{
    bool active = false;   // this feed shows the dodge clip
    bool start = false;    // first feed of a clip (inertialized entry over kBlendInS)
    bool handback = false; // first feed after the clip (the motion search, inertialized over kBlendOutS)
    Sector sector = Sector::None;
    float clipTime = 0.0f;
};

class DodgePlayer
{
public:
    void Reset() { *this = DodgePlayer{}; }
    bool Active() const { return m_playing; }
    Sector CurrentSector() const { return m_sector; }
    float ClipTime() const { return m_clipTime; }
    float EntrySpeed() const { return m_entrySpeed; }

    // aLength: clip lengths (seconds of clip time) of the loaded clips.
    Frame Tick(const Input& aIn, const std::array<float, kSectorCount>& aLength)
    {
        Frame out;
        const float dt = std::isfinite(aIn.dt) ? std::clamp(aIn.dt, 0.0f, kMaxStepS) : 0.0f;
        const float vx = std::isfinite(aIn.velX) ? aIn.velX : 0.0f, vy = std::isfinite(aIn.velY) ? aIn.velY : 0.0f;
        const float speed = std::hypot(vx, vy);
        const bool edge = aIn.dodgeDetail && !m_prevDetail;
        m_prevDetail = aIn.dodgeDetail;
        if (!aIn.standingGround)
        {
            out.handback = m_playing;
            m_playing = m_armed = false;
            m_sector = Sector::None;
            return out;
        }
        if (edge)
        {
            m_armed = true;
            m_armedT = 0.0f;
            m_entrySpeed = speed;
#if UTPP_DODGE_MUTATION != 3
            if (!(speed > m_lastSpeed + kImpulseRiseMps))
            {
                m_lastSpeed = speed;
                return Continue(out, dt, aLength);
            }
            m_entrySpeed = m_lastSpeed; // the impulse is already in the first dodge feed
#endif
        }
        m_lastSpeed = speed;
        if (m_armed)
        {
            m_armedT += edge ? 0.0f : dt;
#if UTPP_DODGE_MUTATION == 3
            const bool pick = true;
#else
            const bool rise = speed > m_entrySpeed + kImpulseRiseMps;
            const bool pick = rise || m_armedT >= kImpulseWaitS || !aIn.dodgeDetail;
#endif
            if (!pick)
                return Continue(out, dt, aLength);
            m_armed = false;
            if (speed < kMinImpulseMps)
                return Continue(out, dt, aLength);
            const float rel = WrapDeg(std::atan2(-vx, vy) * 57.29578f - aIn.facingYawDeg);
            const Sector s = ClassifySector(rel, m_playing ? m_sector : Sector::None);
            if (s == Sector::Forward && m_entrySpeed >= aIn.forwardClipMaxEntryMps)
            {
                // forward from a run / sprint: the playing locomotion goes on (a clip still playing hands back)
                out.handback = m_playing;
                m_playing = false;
                m_sector = Sector::None;
                return out;
            }
            m_playing = true;
            m_sector = s;
            m_clipTime = 0.0f;
            m_sinceStart = 0.0f;
            out.active = out.start = true;
            out.sector = s;
            out.clipTime = 0.0f;
            return out;
        }
        if (m_playing && m_sinceStart < kSettleS && aIn.dodgeDetail && speed >= kMinImpulseMps)
        {
            const Sector s = ClassifySector(WrapDeg(std::atan2(-vx, vy) * 57.29578f - aIn.facingYawDeg), m_sector);
            if (s != m_sector)
            {
                m_sector = s;
                out.start = true; // a new clip: inertialized again
            }
        }
        return Continue(out, dt, aLength);
    }

private:
    Frame Continue(Frame aOut, float aDt, const std::array<float, kSectorCount>& aLength)
    {
        if (!m_playing)
            return aOut;
#if UTPP_DODGE_MUTATION == 2
        m_clipTime += (1.0f / 60.0f) * kRate;
        (void)aDt;
#else
        m_clipTime += aDt * kRate;
#endif
        m_sinceStart += aDt;
        const float len = aLength[static_cast<int>(m_sector)];
        if (!(m_clipTime < len))
        {
            m_playing = false;
            aOut.handback = true;
            aOut.active = false;
            m_sector = Sector::None;
            return aOut;
        }
        aOut.active = true;
        aOut.sector = m_sector;
        aOut.clipTime = m_clipTime;
        return aOut;
    }

    bool m_prevDetail = false, m_armed = false, m_playing = false;
    float m_armedT = 0.0f, m_entrySpeed = 0.0f, m_lastSpeed = 0.0f, m_clipTime = 0.0f, m_sinceStart = 0.0f;
    Sector m_sector = Sector::None;
};
} // namespace utpp::mm::dodge
