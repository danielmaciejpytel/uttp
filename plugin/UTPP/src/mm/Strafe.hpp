// UTPP B1 strafe S1 core (2026-10-13) - aim strafe, the runtime equivalent of motion RotationMode=Strafe. Header-only,
// deterministic, no allocation per frame (ComputeStrafeProfiles allocates once per database load / derived-data recompute).
//
// 1. StrafeInput / Wanted: strafe is wanted while V aims (PSM UpperBody 6) with the upper-body mask from the game, a weapon in
//    hand (upper-body mask state Aim bit 1: CET sets states 7 with a weapon, 5 with empty hands) or cfg.unarmed, on the ground (|vz| < maxVertical, psm detail not 6 / 14..28 = air / land)
//    and the stance has facing-fixed loops covering >= 6 of 8 direction sectors (StrafeProfiles::avail).
// 2. StrafeMode: Off -> Active on the first wanted frame; Active -> Exiting when not wanted; Exiting -> Off after cfg.exitHold
//    (a 1-frame UpperBody blip does not exit); Exiting -> Active when wanted again. One-shot Entered / Exited flags + counters.
// 3. StrafeDesiredMotion: bt::MotionFrame / MotionParams -> bt::DesiredMotion with the BodyTurn.hpp conventions (character
//    frame, forward +Y, stick (MoveX, MoveY)). Velocity direction = raw stick in the body frame while Active, the CAPSULE
//    velocity direction while Exiting (critique MUST: the stick may already be camera-relative again); speed =
//    max(capsule, intentFraction * stateTargetSpeed * |stick|); facing (0,1), Direction mode, turnRate 0; stopIntent zeroes
//    the velocity with the stick released (as bt::ComputeDesiredMotion).
// 4. ComputeStrafeProfiles (one call, for RecomputeDerivedClassDataLocked AFTER ComputeStateTagMasksLocked; state tag masks
//    are inputs): per clip max |cumulative ROOT yaw|, the strafe class sets (walk / jog), the per stance x gait x |theta| bin
//    own ROOT speed table of facing-fixed loops, the 8-sector direction coverage and avail per stance.
// 5. Rate law: LoopOwnSpeedRate (clamp(capsule / own loop speed)) + StrafeRateSmoother (0.1 s halflife).
//
// Precedence vs the slow-walk gate (critique MUST): while the strafe mode is not Off (Active or Exiting) the driver bypasses
// the slow-walk gate (g_slowWalkOn map / rate floor). The strafe rate law (loop own speed, when the playing clip is a loop with
// own root speed > kOwnSpeedMin) wins; otherwise LoopRate on the strafe speed map. See StrafeBypassesSlowWalkGate. The
// turn-rate boost gate keeps "&& !strafe".
// Yaw / direction convention as BodyTurn.hpp: radians, CCW about +Z; character frame x right, y forward.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "../BaseDatabase.hpp"
#include "BodyTurn.hpp"
#include "GameIntegration.hpp"
#include "Steering.hpp"

namespace utpp::mm::strafe
{
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kDeg = kPi / 180.0f;

// Driver-side class of the strafe class sets: a clip that turns the root (>= rootTurnMaxDeg cumulative), a turn / reface /
// turn-in-place / arc / slow-turn-start clip, or a Pattern clip off the strafe gait band. Never a candidate while strafing.
inline constexpr uint8_t kClassStrafeTurn = steer::kClassStrafeTurn; // 29
static_assert(kClassStrafeTurn > steer::kClassStanceMoveToCrouch, "kClassStrafeTurn must follow the Steering.hpp classes");
static_assert(kClassStrafeTurn < 32, "class bit masks are 32-bit");

inline constexpr int kStanceStand = 0, kStanceCrouch = 1;
inline constexpr int kGaitWalk = 0, kGaitRun = 1;
inline constexpr int kNumBins = 5; // |theta| 0 / 45 / 90 / 135 / 180 deg
inline constexpr int kAimUpperBody = 6;
inline constexpr uint32_t kMaskStateAim = integ::kMaskStateAim; // upper-body mask state Aim: CET states 7 = weapon in hand, 5 = empty hands
static_assert(kMaskStateAim == (1u << 1) && (7u & kMaskStateAim) != 0 && (5u & kMaskStateAim) == 0,
              "weapon-in-hand signal = upper-body mask state Aim bit 1 (CET UTPPSetMMUpperMaskStates 7 / 5)");
inline constexpr float kOwnSpeedMin = 0.3f;        // loop own root speed below which the own-speed rate law is not used

struct StrafeConfig
{
    float exitHold = 0.15f;       // s, Exiting -> Off
    float rootTurnMaxDeg = 10.0f; // max |cumulative ROOT yaw| of a facing-fixed clip
    float rootWarpMaxDeg = 30.0f; // offset-root warp limit while strafing (driver)
    bool unarmed = false;         // strafe also without a weapon in hand
    float maxVertical = 1.0f;     // m/s, |vz| limit
    float swayKeep = 0.5f;        // S2 ChestAlign (driver / ChestAlign.hpp)
    float chestMaxDeg = 60.0f;    // S2 ChestAlign clamp
    float tableMaxYawDeg = 0.05f; // speed table / sectors: only exactly facing-fixed loops (motion 0.0 deg; non-package loops 0.1-4.2 deg)
};

inline float Finite(float v, float fallback = 0.0f) { return std::isfinite(v) ? v : fallback; }

// ------------------------------------------------------------------------------------------------ wanted

struct StrafeInput
{
    bool strafeOn = false;
    bool tpp = false;
    bool upperMaskOn = false;
    bool upperMaskOk = false;
    int32_t upperBody = 0;    // PSM UpperBody
    uint32_t maskStates = 0;  // CET mask states bit set
    float vz = 0.0f;          // m/s capsule vertical velocity
    int32_t psmDetail = 0;    // PSM locomotion detail
    int32_t stance = kStanceStand;
};

inline bool IsAirDetail(int32_t d) { return d == 6 || (d >= 14 && d <= 28); }

inline bool Wanted(const StrafeInput& in, const StrafeConfig& cfg, const bool (&avail)[2])
{
    if (!in.strafeOn || !in.tpp || !in.upperMaskOn || !in.upperMaskOk)
        return false;
    if (in.upperBody != kAimUpperBody)
        return false;
    if (!(in.maskStates & kMaskStateAim) && !cfg.unarmed)
        return false;
    if (!std::isfinite(in.vz) || !(std::fabs(in.vz) < cfg.maxVertical))
        return false;
    if (IsAirDetail(in.psmDetail))
        return false;
    if (in.stance != kStanceStand && in.stance != kStanceCrouch)
        return false;
    return avail[in.stance];
}

// ------------------------------------------------------------------------------------------------ mode

enum class StrafeState : uint8_t
{
    Off = 0,
    Active = 1,
    Exiting = 2,
};

class StrafeMode
{
public:
    StrafeState Update(bool aWanted, float aDt, const StrafeConfig& aCfg)
    {
        m_entered = m_exited = false;
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        switch (m_state)
        {
        case StrafeState::Off:
            if (aWanted)
            {
                m_state = StrafeState::Active;
                m_entered = true;
                ++m_entries;
            }
            break;
        case StrafeState::Active:
            if (!aWanted)
            {
                m_state = StrafeState::Exiting;
                m_exitTime = 0.0f;
            }
            break;
        case StrafeState::Exiting:
            if (aWanted)
            {
                m_state = StrafeState::Active;
                ++m_blips;
            }
            else
            {
                m_exitTime += dt;
                // NaN / inf / negative exitHold falls back to the 0.15 s default (a NaN compare would hold Exiting forever).
                const float hold = (std::isfinite(aCfg.exitHold) && aCfg.exitHold >= 0.0f) ? aCfg.exitHold : 0.15f;
                if (m_exitTime >= hold)
                {
                    m_state = StrafeState::Off;
                    m_exited = true;
                    ++m_exits;
                }
            }
            break;
        }
        if (m_state != StrafeState::Off)
            ++m_frames;
        return m_state;
    }
    void Reset() { *this = StrafeMode{}; }
    // UTPP-MOTION SEAM: leave at once (no exit hold; Exited edge and exit count as a normal exit); never called by v1
    void ForceOff() // UTPP-MOTION SEAM
    { // UTPP-MOTION SEAM
        m_entered = m_exited = false; // UTPP-MOTION SEAM
        if (m_state == StrafeState::Off) // UTPP-MOTION SEAM
            return; // UTPP-MOTION SEAM
        m_state = StrafeState::Off; // UTPP-MOTION SEAM
        m_exited = true; // UTPP-MOTION SEAM
        ++m_exits; // UTPP-MOTION SEAM
    } // UTPP-MOTION SEAM

    StrafeState State() const { return m_state; }
    bool On() const { return m_state != StrafeState::Off; }
    bool Entered() const { return m_entered; } // one-shot: this Update went Off -> Active
    bool Exited() const { return m_exited; }   // one-shot: this Update went Exiting -> Off
    uint64_t Entries() const { return m_entries; }
    uint64_t Exits() const { return m_exits; }
    uint64_t Blips() const { return m_blips; } // Exiting -> Active (a short gap bridged by the hold)
    uint64_t Frames() const { return m_frames; }

private:
    StrafeState m_state = StrafeState::Off;
    float m_exitTime = 0.0f;
    bool m_entered = false, m_exited = false;
    uint64_t m_entries = 0, m_exits = 0, m_blips = 0, m_frames = 0;
};

// Slow-walk gate precedence: bypassed while the strafe mode is Active or Exiting.
inline bool StrafeBypassesSlowWalkGate(StrafeState s) { return s != StrafeState::Off; }

// ------------------------------------------------------------------------------------------------ desired motion

// p.intentFraction * p.stateTargetSpeed = the strafe intent speed at full stick (driver: aim walk / jog game speed).
inline bt::DesiredMotion StrafeDesiredMotion(const bt::MotionFrame& f, const bt::MotionParams& p, StrafeState s)
{
    bt::DesiredMotion d;
    const float cx = Finite(f.charVel[0]), cy = Finite(f.charVel[1]);
    const float mx = Finite(f.move[0]), my = Finite(f.move[1]);
    const float capsule = std::sqrt(cx * cx + cy * cy);
    const float mag = std::sqrt(mx * mx + my * my);
    const float want = Finite(p.intentFraction) * Finite(p.stateTargetSpeed) * std::fmin(mag, 1.0f);
    float ix = 0.0f, iy = 0.0f;
    bool haveDir = false;
    if (s == StrafeState::Exiting && capsule > 0.05f)
    {
        ix = cx / capsule;
        iy = cy / capsule;
        haveDir = true;
    }
    else if (mag > 0.15f)
    {
        ix = mx / mag;
        iy = my / mag;
        haveDir = true;
    }
    if (haveDir && mag > 0.15f)
    {
        const float sp = std::fmax(capsule, want);
        d.vel[0] = ix * sp;
        d.vel[1] = iy * sp;
    }
    else
    {
        d.vel[0] = cx;
        d.vel[1] = cy;
    }
    if (p.stopIntent && mag <= 0.15f)
    {
        d.vel[0] = 0.0f;
        d.vel[1] = 0.0f;
    }
    d.facing[0] = 0.0f;
    d.facing[1] = 1.0f;
    d.direction = true;
    d.turnRate = 0.0f;
    return d;
}

// ------------------------------------------------------------------------------------------------ rate law

// Playback rate of a loop with own root speed aOwnSpeed at capsule speed aCapsule; < 0 = not applicable (own speed too low).
inline float LoopOwnSpeedRate(float aCapsule, float aOwnSpeed, float aRateMin, float aRateMax)
{
    if (!std::isfinite(aOwnSpeed) || !(aOwnSpeed > kOwnSpeedMin))
        return -1.0f;
    const float lo = Finite(aRateMin, 1.0f), hi = std::fmax(lo, Finite(aRateMax, 1.0f));
    const float r = Finite(aCapsule) / aOwnSpeed;
    return std::fmin(hi, std::fmax(lo, r));
}

struct StrafeRateSmoother
{
    float rate = 1.0f;
    bool valid = false;
    // Exponential smoothing with aHalflife (s); the first sample / a non-finite state snaps.
    float Update(float aTarget, float aDt, float aHalflife = 0.1f)
    {
        if (!std::isfinite(aTarget))
            return rate;
        const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        if (!valid || !std::isfinite(rate) || !(aHalflife > 0.0f))
        {
            rate = aTarget;
            valid = true;
            return rate;
        }
        const float a = 1.0f - std::exp2(-dt / aHalflife);
        rate += (aTarget - rate) * a;
        return rate;
    }
    void Reset() { valid = false; rate = 1.0f; }
};

// ------------------------------------------------------------------------------------------------ direction bins / speed table

inline int DirBin(float aAbsThetaRad)
{
    const float t = std::fabs(Finite(aAbsThetaRad));
    const int b = int(std::lround(std::fmin(t, kPi) / (45.0f * kDeg)));
    return b < 0 ? 0 : (b > 4 ? 4 : b);
}
// 8 signed sectors of 45 deg centred on 0, 45, ..., 315 (CCW from forward).
inline int DirSector(float aThetaRad)
{
    float t = bt::Wrap(Finite(aThetaRad));
    if (t < 0.0f)
        t += 2.0f * kPi;
    return int(std::lround(t / (45.0f * kDeg))) & 7;
}

// S4 jog while aiming: game speed profile over |theta| (body-relative stick direction) at the bin centres 0 / 45 / 90 / 135 / 180
// deg = F : F : side : back : back with F : side : back = 1 : 0.70 : 0.60 (the motion run loop own-speed ratios 5.46 / 3.83 / 3.28),
// linear in |theta| between the centres: every run loop then plays at the same rate F / 5.46.
inline constexpr float kJogProfile[kNumBins] = {1.0f, 1.0f, 0.70f, 0.60f, 0.60f};
// Accepted forward jog speeds (UTPPSetMMStrafeParams jogF, CET bodyturn.aimJogSpeed lower bound). Minimum 3.3 m/s: the run loops
// then play at >= 3.3 / 5.46 = 0.60 = the rate floor (below it the clips play too fast for the capsule: jog 1.65 measured a 24.9 cm
// steady foot-lock correction), and the slowest direction (back, 0.60 F = 1.98 m/s) stays faster than the vanilla aim walk 1.65.
inline constexpr float kJogMinF = 3.3f;
inline constexpr float kJogMaxF = 6.0f;
inline bool JogSpeedValid(float aJogF) { return aJogF == 0.0f || (aJogF >= kJogMinF && aJogF <= kJogMaxF); }
inline float JogProfile(float aThetaRad)
{
    const float t = std::fmin(std::fabs(Finite(aThetaRad)), kPi) / (45.0f * kDeg);
    const int i = std::min(int(t), kNumBins - 2);
    const float w = t - float(i);
    return kJogProfile[i] * (1.0f - w) + kJogProfile[i + 1] * w;
}
// Body-relative stick (x right-positive game MoveX, y forward) -> jog target game speed aJogF * JogProfile; the stick angle is
// atan2(-x, y) like the driver's strafe direction (sign irrelevant: the profile is symmetric). Stick <= 0.15: forward (aJogF).
inline float JogTargetSpeed(float aJogF, float aMoveX, float aMoveY)
{
    const float f = Finite(aJogF);
    if (!(f > 0.0f))
        return 0.0f;
    const float x = Finite(aMoveX), y = Finite(aMoveY);
    if (x * x + y * y <= 0.0225f)
        return f;
    return f * JogProfile(std::atan2(-x, y));
}

struct StrafeSpeedTable
{
    // m/s own root speed [stance][gait][bin]; 0 = no loop measured in that bin (filled from neighbours, see Fill).
    float speed[2][2][kNumBins] = {};
    uint16_t count[2][2][kNumBins] = {};

    // Linear in |theta| between bin centres.
    float Speed(int aStance, int aGait, float aThetaRad) const
    {
        if (aStance < 0 || aStance > 1 || aGait < 0 || aGait > 1)
            return 0.0f;
        const float t = std::fmin(std::fabs(Finite(aThetaRad)), kPi) / (45.0f * kDeg);
        const int i = std::min(int(t), kNumBins - 2);
        const float w = t - float(i);
        return speed[aStance][aGait][i] * (1.0f - w) + speed[aStance][aGait][i + 1] * w;
    }

    // Design Map_theta(v): game capsule speed aGameSpeed -> DB query speed in direction aThetaRad, with the game point
    // aGamePoint (game speed at full strafe gait, e.g. aim walk 1.65 m/s measured) mapped onto the DB loop speed S(theta).
    // Linear through 0. Falls back to aGameSpeed when the table has no loop there or aGamePoint is not > 0 / non-finite.
    float MapSpeed(int aStance, int aGait, float aThetaRad, float aGameSpeed, float aGamePoint) const
    {
        const float v = std::fmax(0.0f, Finite(aGameSpeed));
        const float s = Speed(aStance, aGait, aThetaRad);
        if (!(s > 0.0f) || !std::isfinite(aGamePoint) || !(aGamePoint > 0.0f))
            return v;
        return v * (s / aGamePoint);
    }
};

// ------------------------------------------------------------------------------------------------ profiles

struct StrafeProfiles
{
    std::vector<float> maxCumYawDeg;   // per clip, max over frames of |cumulative ROOT yaw|
    std::vector<uint8_t> classes[2];   // strafe class sets: [kGaitWalk] walk strafe, [kGaitRun] jog strafe (S4)
    std::vector<int8_t> loopBin;       // per clip: |theta| bin of a facing-fixed loop, -1 otherwise
    std::vector<float> loopOwnSpeed;   // per clip: mean own ROOT speed of a facing-fixed loop (m/s), 0 otherwise
    StrafeSpeedTable table;
    uint8_t sectors[2][2] = {};        // [stance][gait] bit mask of the 8 direction sectors covered by facing-fixed loops
    bool avail[2] = {false, false};    // [stance]: walk gait (crouch: its only gait) covers >= 6 sectors
    bool availGait[2][2] = {};
    // counts (diagnostics / tests)
    uint32_t facingFixedLoops = 0, patternsKept[2] = {0, 0}, patternsOffBand[2] = {0, 0}, rootTurners = 0, turnClassClips = 0;
};

inline int SectorCount(uint8_t m)
{
    int n = 0;
    for (int i = 0; i < 8; ++i)
        n += (m >> i) & 1;
    return n;
}

// Turn-class clips that never belong to a strafe query (besides any clip whose ROOT turns >= rootTurnMaxDeg). Pivot classes are
// NOT listed: motion Pivot_X_Y are facing-fixed (0.0 deg) direction changes; the turning pivots (Turn_*_180) fail the ROOT rule.
inline bool IsStrafeTurnBaseClass(uint8_t c)
{
    return (steer::IsTurnClass(c) && !steer::IsPivot(c)) || c == steer::kClassTurnInPlace || c == steer::kClassArc || c == steer::kClassSlowTurnStart ||
           c == steer::kClassTurnAgainst;
}

// |theta| bin of a loop direction tag (steer::LoopDirTagOfName: F, FL, FR, L, R, LL, LR, RL, RR, B, BL, BR); -1 = unknown.
// motion LR_offset / RR_offset root paths drift diagonally (measured 45 / 135 deg) although they are side loops, so the tag wins.
inline int DirBinOfTag(std::string_view t)
{
    if (t == "F")
        return 0;
    if (t == "FL" || t == "FR")
        return 1;
    if (t == "L" || t == "R" || t == "LL" || t == "LR" || t == "RL" || t == "RR")
        return 2;
    if (t == "BL" || t == "BR")
        return 3;
    if (t == "B")
        return 4;
    return -1;
}

// A loop name ("_Loop") of class None / Lateral.
inline bool IsLoopCandidate(std::string_view aName, uint8_t aClass)
{
    return (aClass == steer::kClassNone || aClass == steer::kClassLateral) && aName.find("_Loop") != std::string_view::npos;
}

// One call per database load / derived-data recompute. aClipClasses = driver clip classes (g_clipClasses), aStandMask /
// aCrouchMask = g_stateTagMask[kLocoStand] / [kLocoCrouch] (after ComputeStateTagMasksLocked).
inline void ComputeStrafeProfiles(const Database& aDb, std::span<const uint8_t> aClipClasses, uint64_t aStandMask, uint64_t aCrouchMask,
                                  const StrafeConfig& aCfg, StrafeProfiles& aOut)
{
    const auto clips = aDb.Clips();
    const size_t n = clips.size();
    aOut = StrafeProfiles{};
    aOut.maxCumYawDeg.assign(n, 0.0f);
    aOut.loopBin.assign(n, int8_t(-1));
    aOut.loopOwnSpeed.assign(n, 0.0f);
    for (auto& v : aOut.classes)
        v.assign(n, uint8_t(steer::kClassNone));
    const float fps = aDb.Fps() > 0.0f ? aDb.Fps() : 30.0f;
    const float turnMax = Finite(aCfg.rootTurnMaxDeg, 10.0f);
    double sum[2][2][kNumBins] = {};
    for (size_t c = 0; c < n; ++c)
    {
        const uint8_t k = c < aClipClasses.size() ? aClipClasses[c] : uint8_t(steer::kClassNone);
        const std::string_view name = aDb.ClipName(uint32_t(c));
        float acc = 0.0f, mx = 0.0f;
        double sx = 0.0, sy = 0.0, ss = 0.0;
        uint32_t m = 0;
        for (uint32_t f = 0; f < clips[c].numFrames; ++f)
        {
            const RootDelta* r = aDb.Root(clips[c].firstFrame + f);
            if (!r || !std::isfinite(r->dyaw) || !std::isfinite(r->dx) || !std::isfinite(r->dy))
                continue;
            acc += r->dyaw;
            mx = std::fmax(mx, std::fabs(acc));
            sx += r->dx;
            sy += r->dy;
            ss += std::sqrt(double(r->dx) * r->dx + double(r->dy) * r->dy);
            ++m;
        }
        const float maxDeg = mx / kDeg;
        aOut.maxCumYawDeg[c] = maxDeg;
        const bool turner = !(maxDeg < turnMax);
        const bool baseTurn = IsStrafeTurnBaseClass(k);
        aOut.rootTurners += turner ? 1u : 0u;
        aOut.turnClassClips += baseTurn ? 1u : 0u;
        for (int g = 0; g < 2; ++g)
            aOut.classes[g][c] = (turner || baseTurn) ? kClassStrafeTurn : k;
        if (k == steer::kClassPattern && !turner)
        {
            const int sc = steer::SpeedClassOfName(name); // 1 walk, 2 run, 3 sprint, 0 none (crouch)
            for (int g = 0; g < 2; ++g)
            {
                const bool keep = sc == 0 || sc == g + 1;
                if (keep)
                    ++aOut.patternsKept[g];
                else
                {
                    ++aOut.patternsOffBand[g];
                    aOut.classes[g][c] = kClassStrafeTurn;
                }
            }
        }
        // facing-fixed loops: speed table + sectors
        if (!(maxDeg <= Finite(aCfg.tableMaxYawDeg, 0.05f)) || m == 0 || !IsLoopCandidate(name, k))
            continue;
        const uint64_t tm = clips[c].tagMask;
        const int stance = (tm & aCrouchMask) ? kStanceCrouch : ((tm & aStandMask) ? kStanceStand : -1);
        if (stance < 0)
            continue;
        const int sc = steer::SpeedClassOfName(name);
        const int gait = stance == kStanceCrouch ? kGaitWalk : (sc == 1 ? kGaitWalk : (sc == 2 ? kGaitRun : -1));
        if (gait < 0)
            continue;
        const float own = float(ss / double(m)) * fps;
        if (!(own > kOwnSpeedMin))
            continue;
        const float theta = std::atan2(-float(sx), float(sy)); // CCW from forward (+Y), x right
        const int tb = DirBinOfTag(steer::LoopDirTagOfName(name));
        const int b = tb >= 0 ? tb : DirBin(std::fabs(theta));
        aOut.loopBin[c] = int8_t(b);
        aOut.loopOwnSpeed[c] = own;
        sum[stance][gait][b] += own;
        ++aOut.table.count[stance][gait][b];
        aOut.sectors[stance][gait] |= uint8_t(1u << DirSector(theta));
        ++aOut.facingFixedLoops;
    }
    for (int s = 0; s < 2; ++s)
        for (int g = 0; g < 2; ++g)
        {
            float* sp = aOut.table.speed[s][g];
            for (int b = 0; b < kNumBins; ++b)
                sp[b] = aOut.table.count[s][g][b] ? float(sum[s][g][b] / aOut.table.count[s][g][b]) : 0.0f;
            // fill empty bins from the nearest measured bin (forward first on ties)
            for (int b = 0; b < kNumBins; ++b)
                if (sp[b] == 0.0f)
                    for (int d = 1; d < kNumBins; ++d)
                    {
                        if (b - d >= 0 && aOut.table.count[s][g][b - d])
                        {
                            sp[b] = sp[b - d];
                            break;
                        }
                        if (b + d < kNumBins && aOut.table.count[s][g][b + d])
                        {
                            sp[b] = sp[b + d];
                            break;
                        }
                    }
            aOut.availGait[s][g] = SectorCount(aOut.sectors[s][g]) >= 6;
        }
    aOut.avail[kStanceStand] = aOut.availGait[kStanceStand][kGaitWalk];
    aOut.avail[kStanceCrouch] = aOut.availGait[kStanceCrouch][kGaitWalk];
}

} // namespace utpp::mm::strafe
