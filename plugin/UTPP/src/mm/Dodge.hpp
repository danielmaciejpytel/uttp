// Ground dodge clips (pure logic, no game types).
//
// The game's ground dodge (PSM detail 7) moves the capsule with a short impulse: the first dodge feed still carries the old
// velocity, the next one the impulse (9 - 15 m/s) that decays within about 0.2 s. This header provides the pieces to show one of
// four dodge clips of utpp_dodge_<body>.utpptrav during it (the traversal file grammar, version 2: montages Dodge_F, Dodge_B,
// Dodge_L, Dodge_R, Root-relative in-place poses; optional fifth montage Dodge_F_Run, the forward dodge entered from a run or
// sprint); the clip's own root travel is not part of the pose (the capsule moves the body). The motion driver (MotionDriver.cpp)
// shows the clips on the standing ground.
//
//   DodgeClips    validation of the parsed file against the database bones (fail closed: ok = false, err set).
//   SampleDodge   the local pose of a clip at a clip time (frame lerp + quaternion nlerp), 7 floats per database bone.
//   ClassifySector  direction of the impulse relative to the body's facing, four sectors with hysteresis at the borders.
//   ImpulseDirection  the impulse on a dodge feed: the capsule velocity minus the part of the entry velocity the game keeps.
//   UpperBodyMask / KeepUpperBody / UpperBodyRelease  the bones above Spine3 keep the pose shown before the clip and move to
//                 the motion pose underneath over the last kUpperReleaseS of the clip.
//   DodgePlayer   per feed: arms on the dodge's first feed, picks the clip on the impulse feed, runs the clip clock at kRate,
//                 hands back at the clip end or when the body leaves the standing ground state. A forward dodge entered faster
//                 than Input::forwardClipMaxEntryMps plays Dodge_F_Run when the file has it, else the playing locomotion goes on.
//
// Facing while a clip plays (rule, MotionHost BodyYawCommand / MotionSetFrame::dodgeClip): the body turn holds the capsule yaw, so
// the drawn facing the sector was picked against does not turn toward the input under the clip; the turn toward the movement
// direction resumes on the hand-back feed (the inertialized hand-back blend, kBlendOutS).
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
// time, 3 the clip is picked on the dodge's first feed (old velocity), 4 the loader accepts a clip with a missing pose bone,
// 5 the pick reads the capsule velocity as it is (the kept part of the entry velocity is not removed), 6 a forward dodge from a run
// never plays Dodge_F_Run.
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
    ForwardRun = 4, // not a direction: the clip of a Forward dodge entered from a run / sprint (ClassifySector never returns it)
};
inline constexpr int kSectorCount = 4; // the directions ClassifySector returns (the required montages)
inline constexpr int kClipCount = 5;   // + the optional Dodge_F_Run
inline constexpr const char* kMontageNames[kClipCount] = {"Dodge_F", "Dodge_B", "Dodge_L", "Dodge_R", "Dodge_F_Run"};
inline const char* SectorName(Sector s)
{
    const int i = static_cast<int>(s);
    return i < 0 || i >= kClipCount ? "-" : kMontageNames[i] + 6;
}
// direction of a sector (ForwardRun is a Forward dodge)
inline Sector Direction(Sector s)
{
    return s == Sector::ForwardRun ? Sector::Forward : s;
}

// clip clock rate (the recovery ends about when the dodge impulse has died out); UTPP_DODGE_RATE overrides it in measurement builds only
#ifndef UTPP_DODGE_RATE
#define UTPP_DODGE_RATE 1.5f
#endif
inline constexpr float kRate = UTPP_DODGE_RATE;
// inertialized entry from the playing pose (long enough that the held arms of a weapon stance follow the crouch at the start of
// the back clip without a jump); UTPP_DODGE_BLEND_IN overrides it in measurement builds only
#ifndef UTPP_DODGE_BLEND_IN
#define UTPP_DODGE_BLEND_IN 0.30f
#endif
inline constexpr float kBlendInS = UTPP_DODGE_BLEND_IN;
inline constexpr float kBlendOutS = 0.30f;     // inertialized hand-back to the motion search
inline constexpr float kHysteresisDeg = 10.0f; // a sector border must be crossed by this much to change the sector
inline constexpr float kImpulseRiseMps = 1.5f; // the impulse feed: speed above the first feed's speed by this much
inline constexpr float kImpulseWaitS = 0.06f;  // without a rise the direction is read after this much dodge time
inline constexpr float kMinImpulseMps = 1.0f;  // slower than this on the impulse feed: no clip (no direction)
inline constexpr float kSettleS = 0.05f;       // the sector may still change (past the hysteresis) this long after the pick
inline constexpr float kMaxStepS = 0.1f;       // one feed counts for at most this much time
// Part of the entry velocity (the last feed before the impulse) the capsule keeps under the dodge impulse. Fit of
// v = impulse + k * entry on the first impulse feeds of the recorded side and back presses (2026-11-14 sprint_dash, both bodies):
// k = 0.45 - 0.73; without removing it the first impulse feed of a side press read 24 deg (Female left: 103.4 vs push 79.0) and
// 14 deg (right: 64.6) off the push, with it 8 and 2 deg.
inline constexpr float kEntryKeep = 0.5f;

// Impulse direction on a dodge feed (world velocity minus the kept part of the entry velocity); aOutX / aOutY = that vector.
inline void ImpulseDirection(float aVelX, float aVelY, float aEntryX, float aEntryY, float& aOutX, float& aOutY)
{
#if UTPP_DODGE_MUTATION == 5
    aEntryX = aEntryY = 0.0f;
#endif
    aOutX = aVelX - kEntryKeep * aEntryX;
    aOutY = aVelY - kEntryKeep * aEntryY;
}

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
    aPrev = Direction(aPrev);
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
    std::array<const traversal::TravMontage*, kClipCount> clip{}; // [ForwardRun] null when the file has no Dodge_F_Run
    std::array<float, kClipCount> length{}; // seconds of clip time (0 = no clip)
    bool HasForwardRun() const { return ok && clip[static_cast<int>(Sector::ForwardRun)] != nullptr; }
};

// Validates aFile (kept alive by the caller: the clip pointers point into it) against the database bones. aName(i) = name of
// database bone i. Every database bone must have a pose bone; the four direction montages must exist with >= 2 frames and finite
// poses; Dodge_F_Run is optional (absent: length 0, null), but when present it must pass the same checks.
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
    for (int s = 0; s < kClipCount; ++s)
    {
        const traversal::TravMontage* m = aFile.Find(kMontageNames[s]);
        if (m == nullptr && s >= kSectorCount)
            continue; // optional clip
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
    const int ci = static_cast<int>(aSector);
    if (!aC.ok || ci < 0 || ci >= kClipCount || aC.clip[ci] == nullptr || aOut == nullptr)
        return false;
    const traversal::TravMontage& m = *aC.clip[ci];
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
// feed seconds over which the release target fades to a new clip / time of the motion pose underneath (no cut in the target)
inline constexpr float kTargetFadeS = 0.10f;

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
    float forwardClipMaxEntryMps = 0.0f; // a forward dodge entered faster than this: Dodge_F_Run when loaded, else the run goes on
};

struct Frame
{
    bool active = false;   // this feed shows the dodge clip
    bool start = false;    // first feed of a clip (inertialized entry over kBlendInS)
    bool handback = false; // first feed after the clip (the motion search, inertialized over kBlendOutS)
    Sector sector = Sector::None; // the clip shown (ForwardRun = Dodge_F_Run)
    float clipTime = 0.0f;
    // the pick of this feed (every dodge that reaches it, also a forward one without a clip): direction picked, the impulse's world
    // yaw (ImpulseDirection) and its yaw relative to the facing, the entry speed
    bool pick = false;
    Sector pickSector = Sector::None;
    float pushYawDeg = 0.0f, pushRelDeg = 0.0f, entrySpeed = 0.0f;
};

class DodgePlayer
{
public:
    void Reset() { *this = DodgePlayer{}; }
    bool Active() const { return m_playing; }
    Sector CurrentSector() const { return m_sector; }
    float ClipTime() const { return m_clipTime; }
    float EntrySpeed() const { return m_entrySpeed; }

    // aLength: clip lengths (seconds of clip time) of the loaded clips (four directions: no Dodge_F_Run).
    Frame Tick(const Input& aIn, const std::array<float, kSectorCount>& aLength)
    {
        std::array<float, kClipCount> l{};
        for (int s = 0; s < kSectorCount; ++s)
            l[s] = aLength[s];
        return Tick(aIn, l);
    }

    // aLength: clip lengths of the loaded clips; aLength[ForwardRun] <= 0 = no Dodge_F_Run.
    Frame Tick(const Input& aIn, const std::array<float, kClipCount>& aLength)
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
            m_lastVel[0] = vx;
            m_lastVel[1] = vy;
            return out;
        }
        if (edge)
        {
            m_armed = true;
            m_armedT = 0.0f;
            m_entrySpeed = speed;
            m_entryVel[0] = vx;
            m_entryVel[1] = vy;
#if UTPP_DODGE_MUTATION != 3
            if (!(speed > m_lastSpeed + kImpulseRiseMps))
            {
                m_lastSpeed = speed;
                m_lastVel[0] = vx;
                m_lastVel[1] = vy;
                return Continue(out, dt, aLength);
            }
            m_entrySpeed = m_lastSpeed; // the impulse is already in the first dodge feed
            m_entryVel[0] = m_lastVel[0];
            m_entryVel[1] = m_lastVel[1];
#endif
        }
        m_lastSpeed = speed;
        m_lastVel[0] = vx;
        m_lastVel[1] = vy;
        float ix = 0.0f, iy = 0.0f;
        ImpulseDirection(vx, vy, m_entryVel[0], m_entryVel[1], ix, iy);
        const float pushYaw = std::atan2(-ix, iy) * 57.29578f;
        const float rel = WrapDeg(pushYaw - aIn.facingYawDeg);
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
            Sector s = ClassifySector(rel, m_playing ? m_sector : Sector::None);
            out.pick = true;
            out.pickSector = s;
            out.pushYawDeg = WrapDeg(pushYaw);
            out.pushRelDeg = rel;
            out.entrySpeed = m_entrySpeed;
            if (s == Sector::Forward && m_entrySpeed >= aIn.forwardClipMaxEntryMps)
            {
#if UTPP_DODGE_MUTATION != 6
                const bool runClip = aLength[static_cast<int>(Sector::ForwardRun)] > 0.0f;
#else
                const bool runClip = false;
#endif
                if (!runClip)
                {
                    // forward from a run / sprint without Dodge_F_Run: the playing locomotion goes on (a clip still playing hands back)
                    out.handback = m_playing;
                    m_playing = false;
                    m_sector = Sector::None;
                    return out;
                }
                s = Sector::ForwardRun; // entered from the run / sprint pose, handed back to the search at speed
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
            Sector s = ClassifySector(rel, m_sector);
            if (s == Sector::Forward && m_sector == Sector::ForwardRun)
                s = Sector::ForwardRun; // still the forward dodge of the run
            else if (s == Sector::Forward && m_entrySpeed >= aIn.forwardClipMaxEntryMps && aLength[static_cast<int>(Sector::ForwardRun)] > 0.0f)
                s = Sector::ForwardRun;
            if (s != m_sector)
            {
                m_sector = s;
                out.start = true; // a new clip: inertialized again
            }
        }
        return Continue(out, dt, aLength);
    }

private:
    Frame Continue(Frame aOut, float aDt, const std::array<float, kClipCount>& aLength)
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
    float m_entryVel[2] = {0.0f, 0.0f}, m_lastVel[2] = {0.0f, 0.0f};
    Sector m_sector = Sector::None;
};
} // namespace utpp::mm::dodge
