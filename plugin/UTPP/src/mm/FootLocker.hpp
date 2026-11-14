// UTPP motion matching runtime - contact-based foot locking with two-bone leg IK (Phase 4.5). Header-only, C++20,
// deterministic, no allocations in Update() after Init().
//
// Operates on the matcher output pose (game-local transforms, Root/Trajectory identity, so model space == character
// frame: x right, y forward, z up, metres). The character world transform is tracked by integrating the REAL capsule
// world velocity (planar x, y) over real dt and taking the measured player yaw (CCW about +Z; character -> world is
// Rot2(yaw)). No vertical tracking / ground raycast: the lock is planar, the target z is always the animated ankle z.
//
// Per foot (UpLeg -> Leg -> Foot, found by bone name, parent chain verified):
//   - contact hysteresis: on when contact > onThreshold (0.5), off when contact < offThreshold (0.3);
//   - lock point = the toe (LeftToeBase/RightToeBase, a descendant of the foot; falls back to the ankle if absent): in
//     the motion clips the toe stays planted through the whole contact while the ankle rises and moves ~10 cm forward
//     during the heel roll, so locking the ankle would fight the animation;
//   - contact rising: lock the toe world position (model-space FK of the pose + character transform). If the foot is
//     still blending out of a previous lock, the lock is placed at the currently displayed toe (FK of the blended
//     chain) and the weight is set to 1 (toe xy continuous, no re-blend);
//   - while locked: offset = locked toe (in the current character frame) - animated toe, horizontal only; if
//     |offset| > maxDrift (0.25 m; big capsule correction / teleport) the foot unlocks immediately and blends out
//     with the previous update's IK deltas (no jump to the far target); no relock until that contact ends;
//   - IK target (ankle) = animated ankle + weight * offset (the foot keeps its animated orientation, so the toe lands on
//     the lock), weight = smoothstep(s), s ramps 0->1 in blendInTime (0.08 s)
//     while locked and 1->0 in blendOutTime (0.15 s) after unlock. Blend-in is positional (target = animated ankle +
//     w * offset); the blend-out is in ROTATION space: the last IK local rotation deltas of UpLeg/Leg/Foot are faded
//     (local = nlerp(identity, delta, w) * animated), so the correction follows the animated joint motion (a frozen
//     positional offset during the swing made the knee rotate up to ~55 deg per update in tests);
//   - analytic two-bone IK in model space (mm/LegIk.hpp, shared): knee placed by the law of cosines on the pole side of the hip-target
//     line, thigh and shin rotated by minimal swings. Pole = animated knee offset from that line, with a foot-forward
//     bias that fades in only for an almost straight leg (no knee flips). Reach is limited softly from 0.97 to
//     maxExtension (0.995) of the leg length, never below the animated hip-ankle distance. The foot keeps its
//     ANIMATED orientation relative to the character (model space), so heel/toe roll of the clip is preserved; only
//     UpLeg/Leg/Foot local rotations change (translations untouched).
// Non-finite input or results reset the foot state and leave the animated pose (fail safe).
#pragma once

#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "../BaseDatabase.hpp"
#include "Inertializer.hpp"
#include "LegIk.hpp"
#include "MMMath.hpp"
#include "Steering.hpp"

namespace utpp::mm
{
struct FootLockConfig
{
    float onThreshold = 0.5f;   // contact rises above -> lock
    float offThreshold = 0.3f;  // contact falls below -> unlock (hysteresis)
    float blendInTime = 0.08f;  // s, lock start
    float blendOutTime = 0.15f; // s, after unlock
    float maxDrift = 0.25f;     // m, locked target vs animated ankle; beyond -> immediate unlock + blend out
    // m, bound of the soft lock (soft slip on, not replant mode): when the animated toe runs away from the lock faster than the slip follows
    // (a clip switch that swings the locked foot, a fast heading flip) and the offset left after the slip passes this, the lock is
    // released like a drift unlock, so the leg is never pulled further than the last solved correction
    float maxOffset = 0.12f;
    float maxExtension = 0.995f; // fraction of (thigh + shin) length the IK may reach
    float softExtension = 0.97f; // above this fraction the reach is limited softly towards maxExtension
    float maxDt = 0.25f;        // s, larger steps are clamped
    // Settling: while the capsule is (nearly) stationary the lock point slides back towards the animated toe at most at
    // settleSpeed, so a stance left by a stop/start mismatch does not stay bent for the whole idle.
    float settleSpeed = 0.12f;       // m/s (0 = off)
    float settleCapsuleSpeed = 0.3f; // m/s, capsule speed below which settling is active
};

// loco C1b (2026-10-13) plant / replant: in replant mode (SetReplant) a planted foot stays planted - no 0.5.9 soft slip, no settle -
// and when it must move it makes one short lifted step to the animated toe (motion foot locking: UnplantRadius 20 / 40 cm, unplant
// speed 100 / 250 cm/s, replant ratio; radii scaled down to stay under the 15 cm foot-correction guard of the loco tests). Per foot:
//   PLANTED -> STEP when |locked toe - animated toe| > radius (radiusStops while the playing clip is idle / stop: Stops profile),
//              or the IK reach was limited (hard clamp, or the target missed by > reachError in the soft reach zone) for >=
//              clampTime, or (tidy) the capsule is below tidyCapsuleSpeed, the offset above tidyRadius for tidyDelay and the other
//              foot is PLANTED;
//   STEP: the remaining offset closes at stepSpeed relative to the animated toe (the lock point also follows the toe's own world
//         motion, so a step ends within (d0 - end) / stepSpeed whatever the capsule / animation mismatch), the ankle target is
//         lifted by min(lift, liftFraction * d0) * sin(pi * progress) (d0 = offset at the step start); the step ends (PLANTED
//         again, relock where the toe is shown) when the offset is <= ratio * radius (clamp / tidy / handover steps: <= tidyRelock);
//   one foot at a time: no step starts while the other foot steps (the running step closes at stepSpeed x hurry while the other
//   foot waits for its own step or is reach-limited); contact end -> the normal release (blend out); the maxDrift teleport guard
//   (FootLockConfig) applies unchanged.
// Mode changes: replant mode is taken by a foot when it locks; a replant -> off switch while planted (stance change) hands the foot
// over to the 0.5.9 path at once when its offset is within the slip start, otherwise after one handover step to the animated toe
// (a running step completes first); an off -> replant switch takes over the planted foot at once (its offset is kept).
// Defaults (offline loco C1b sweep, 2026-10-13): the design's 12 / 14 cm radii (motion 20 / 40 cm scaled) made the crouch walk / pivot /
// turn foot-correction p95 worse than the 0.5.9 slip (10.1-10.5 vs 7.1-7.4 cm on 5 databases) and the idle-family switch reach
// 16.2 cm (> the 15 cm loco guard); 7 / 10 cm keep the p95 below the slip (6.4-6.6 cm) and every correction <= 14 cm. The Stops
// ratio 0.75 left 2.5 cm steps (many short steps at an idle switch): 0.5.
// Step speed is RELATIVE to the animated toe (deviation from the design's absolute bound stepSpeed x dt + 0.5 cm per update): the
// displayed toe moves in world at the animated toe's speed plus stepSpeed (x hurry) - offline peaks 5-6 m/s (8-10 cm per 60 Hz
// update) in crouch starts / stops, vs <= 3.9 m/s planted-toe slide with replant off; without hurry the waiting foot
// passes 15 cm. Standing replant (driver mask bit 1) is experimental: run starts / flicks reach 20-23 cm corrections.
struct FootReplantConfig
{
    float radius = 0.07f;          // m, unplant radius (Default profile)
    float radiusStops = 0.10f;     // m, unplant radius while the playing clip is idle / stop (Stops profile)
    float ratio = 0.2f;            // a radius step ends at offset <= ratio * radius
    float ratioStops = 0.5f;
    float stepSpeed = 1.0f;        // m/s, the step closes the offset at this speed relative to the animated toe
    float stepSpeedStops = 2.5f;
    float lift = 0.03f;            // m, maximum step lift of the ankle target
    float liftFraction = 0.3f;     // lift = min(lift, liftFraction * d0)
    float clampTime = 0.1f;        // s, IK reach limited (clamp or reachError) this long -> step
    float reachError = 0.01f;      // m, an IK target missed by more than this (soft reach limit) counts as clamped
    float tidyRadius = 0.04f;      // m
    float tidyDelay = 0.3f;        // s
    float tidyCapsuleSpeed = 0.3f; // m/s
    float tidyRelock = 0.005f;     // m, a tidy / handover step ends at this offset
    float hurry = 3.0f;            // step speed factor while the other foot waits for its own step or is reach-limited

    static bool Valid(const FootReplantConfig& c)
    {
        auto in = [](float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; };
        return in(c.radius, 0.02f, 0.15f) && in(c.radiusStops, 0.02f, 0.15f) && in(c.ratio, 0.0f, 0.95f) && in(c.ratioStops, 0.0f, 0.95f) &&
               in(c.stepSpeed, 0.2f, 10.0f) && in(c.stepSpeedStops, 0.2f, 10.0f) && in(c.lift, 0.0f, 0.1f) && in(c.liftFraction, 0.0f, 1.0f) &&
               in(c.clampTime, 0.0f, 1.0f) && in(c.reachError, 0.001f, 0.1f) && in(c.tidyRadius, 0.005f, 0.15f) && in(c.tidyDelay, 0.0f, 5.0f) &&
               in(c.tidyCapsuleSpeed, 0.0f, 5.0f) && in(c.tidyRelock, 0.0f, 0.05f) && c.tidyRelock < c.tidyRadius && in(c.hurry, 1.0f, 4.0f);
    }
};

struct FootLockFootDiag
{
    bool contact = false;    // hysteresis state
    bool locked = false;
    bool clamped = false;    // IK target out of reach this update
    float weight = 0.0f;     // applied IK weight (smoothstep)
    float offset = 0.0f;     // m, |locked toe - animated toe| while locked (0 otherwise)
    float correction = 0.0f; // m, |final ankle - animated ankle| this update
    float error = 0.0f;      // m, |final ankle - IK target| this update
    float contactIn = 0.0f;  // loco C0: contact input of this update (sampled CONT, before the hysteresis)
    bool replant = false;    // loco C1b: locked in replant mode
    bool stepping = false;   // loco C1b: replant step in progress after this update (phase S)
    bool stepped = false;    // loco C1b: a step moved the lock point this update (also its last update)
    bool hurried = false;    // loco C1b: that step closed at stepSpeed x hurry (the other foot waits)
    float lift = 0.0f;       // loco C1b: m, step lift applied to the ankle target this update (before the IK weight)
    bool stopReleased = false; // loco-stop-knees: a standing-stop swing contact released this lock this update
};

// loco C0 (diagnostics only): foot events - drift unlocks and (loco C1b) replant steps by trigger. The driver writes one
// "UTPP MMTRACE footstep" line per new event (sequence numbers keep counting across Reset).
enum FootEventReason : uint8_t
{
    kFootEventNone = 0,
    kFootEventDrift = 1,    // |locked toe - animated toe| > maxDrift: unlocked, blends out, no relock until the contact ends
    kFootEventRadius = 2,   // loco C1b: step, offset beyond the unplant radius
    kFootEventClamp = 3,    // loco C1b: step, IK reach clamped for clampTime
    kFootEventTidy = 4,     // loco C1b: step, standing capsule with a small lasting offset
    kFootEventHandover = 5, // loco C1b: step, replant switched off while planted with an offset beyond the slip start
    kFootEventCount = 6,
};
inline const char* FootEventReasonName(uint8_t aReason)
{
    switch (aReason)
    {
    case kFootEventDrift: return "drift";
    case kFootEventRadius: return "radius";
    case kFootEventClamp: return "clamp";
    case kFootEventTidy: return "tidy";
    case kFootEventHandover: return "handover";
    default: return "none";
    }
}
struct FootLockEvent
{
    uint64_t seq = 0;   // 0 = none since Init / Reset; otherwise increasing over the FootLocker's lifetime
    uint8_t reason = kFootEventNone;
    float dist = 0.0f;  // m, horizontal offset of the lock from the animated toe when the event fired
};

struct FootLockDiagnostics
{
    FootLockFootDiag foot[2];
    uint64_t updates = 0;
    uint64_t locks = 0;
    uint64_t relocks = 0;      // lock started while still blending out
    uint64_t driftUnlocks = 0; // maxDrift exceeded
    uint64_t clamps = 0;       // updates with an unreachable target (per foot)
    uint64_t nonFiniteResets = 0;
    uint64_t settles = 0; // locked-foot updates with settling applied
    uint64_t slips = 0;   // 0.5.9: locked-foot updates with the soft slip applied
    uint64_t offsetReleases = 0; // soft locks released because the offset passed FootLockConfig::maxOffset
    uint64_t footOffsetReleases[2] = {0, 0};
    float maxCorrection = 0.0f; // m, since Init/Reset
    // loco C0 (diagnostics only): per-foot counters (index kLeft / kRight; the sums equal the totals above) and last events
    uint64_t footSlips[2] = {0, 0};        // locked-foot updates with the soft slip applied
    uint64_t footDriftUnlocks[2] = {0, 0}; // maxDrift unlocks
    uint64_t footSteps[2] = {0, 0};        // replant steps started (loco C1b)
    FootLockEvent lastEvent[2];
    // loco C1b (replant mode only)
    uint64_t stepReasons[kFootEventCount] = {}; // steps started per trigger (radius / clamp / tidy / handover)
    uint64_t stepUpdates = 0;                   // foot updates in a step
    uint64_t handovers = 0;                     // feet handed over to the 0.5.9 path (replant switched off while planted)
    uint64_t simultaneousSteps = 0;             // updates with both feet stepping (one foot at a time: stays 0)
    float maxLift = 0.0f;                       // m, highest step lift since Init / Reset
    uint64_t blendOutLifts = 0;                 // foot updates where the blend-out sink bound lifted the ankle (loco-fix 2026-11-09)
    uint64_t stopSwingReleases = 0;             // standing-stop locks released as a swing contact (loco-stop-knees 2026-11-09)
};

class FootLocker
{
public:
    static constexpr int kLeft = 0;
    static constexpr int kRight = 1;
    static constexpr float kBlendOutSinkMax = 0.03f; // m, the rotation-space blend-out may lower an ankle this far below the animated one
    // m, standing stop: a lock still blending in whose animated toe moves this far relative to the planted other foot's animated toe is a
    // swing contact and is released (loco-stop-knees 2026-11-09, see UpdateFoot)
    static constexpr float kStopSwingMax = 0.05f;

    // Finds LeftUpLeg/LeftLeg/LeftFoot and RightUpLeg/RightLeg/RightFoot, verifies the parent chain and allocates the
    // output buffer. Returns false (pass-through, Ready() == false) on failure.
    // aDb: any database type with Loaded / NumBones / BoneParent / BoneName (v1 Database, offline v2 MotionDatabase: same topology).
    template <class Db>
    bool Init(const Db& aDb, const FootLockConfig& aCfg = {})
    {
        m_ok = false;
        if (!aDb.Loaded() || !ValidConfig(aCfg))
            return false;
        static constexpr std::string_view kToeNames[2] = {"LeftToeBase", "RightToeBase"};
        static constexpr std::string_view kNames[2][3] = {{"LeftUpLeg", "LeftLeg", "LeftFoot"},
                                                          {"RightUpLeg", "RightLeg", "RightFoot"}};
        const uint32_t nb = aDb.NumBones();
        m_parents.assign(nb, -1);
        for (uint32_t b = 0; b < nb; ++b)
        {
            const int32_t p = aDb.BoneParent(b);
            if (p >= static_cast<int32_t>(b))
                return false; // parents must precede children
            m_parents[b] = p;
        }
        for (int s = 0; s < 2; ++s)
        {
            for (int k = 0; k < 3; ++k)
            {
                m_chain[s][k] = -1;
                for (uint32_t b = 0; b < nb; ++b)
                    if (aDb.BoneName(b) == kNames[s][k])
                        m_chain[s][k] = static_cast<int32_t>(b);
                if (m_chain[s][k] < 0)
                    return false;
            }
            if (m_parents[size_t(m_chain[s][1])] != m_chain[s][0] || m_parents[size_t(m_chain[s][2])] != m_chain[s][1] ||
                m_parents[size_t(m_chain[s][0])] < 0)
                return false;
            // ancestors of UpLeg (root first)
            int32_t tmp[kMaxDepth];
            int32_t n = 0;
            for (int32_t b = m_parents[size_t(m_chain[s][0])]; b >= 0; b = m_parents[size_t(b)])
            {
                if (n >= kMaxDepth)
                    return false;
                tmp[n++] = b;
            }
            m_pathLen[s] = n;
            for (int32_t i = 0; i < n; ++i)
                m_path[s][i] = tmp[n - 1 - i];
            // lock point: toe below the foot (path foot -> toe, foot exclusive); absent -> ankle
            m_toePathLen[s] = 0;
            int32_t toe = -1;
            for (uint32_t b = 0; b < nb; ++b)
                if (aDb.BoneName(b) == kToeNames[s])
                    toe = static_cast<int32_t>(b);
            n = 0;
            bool below = false;
            for (int32_t b = toe; b >= 0 && n < kMaxDepth; b = m_parents[size_t(b)])
            {
                if (b == m_chain[s][2])
                {
                    below = true;
                    break;
                }
                tmp[n++] = b;
            }
            if (below)
            {
                m_toePathLen[s] = n;
                for (int32_t i = 0; i < n; ++i)
                    m_toePath[s][i] = tmp[n - 1 - i];
            }
        }
        m_cfg = aCfg;
        m_nb = nb;
        m_out.assign(nb, BoneTransform{});
        m_ok = true;
        Reset();
        return true;
    }

    bool Ready() const { return m_ok; }
    void SetEnabled(bool aOn) { m_enabled = aOn; }
    bool Enabled() const { return m_enabled; }
    const FootLockConfig& Config() const { return m_cfg; }
    // 0.5.9 soft lock (UTPPSetMMOffsetCatchup): while locked, the part of |locked toe - animated toe| above aStart decays with
    // aHalflife (the lock point slides towards the animated toe), so a capsule / offset catch-up mismatch releases the foot
    // gradually instead of stretching the leg to maxDrift and snapping. aStart <= 0 = off (0.5.8).
    void SetSlip(float aStart, float aHalflife)
    {
        m_slipStart = (std::isfinite(aStart) && aStart > 0.0f) ? aStart : 0.0f;
        m_slipHalflife = (std::isfinite(aHalflife) && aHalflife > 0.005f) ? aHalflife : 0.04f;
        m_slipRampGoal = -1.0f; // a SetSlipRamp lowering in progress ends
    }
    // Per-state slip start (2026-11-06, FootSlipProfile): the soft slip start follows aTarget - raised at once (a larger offset is
    // simply allowed), lowered at most aDownRate m/s (a foot that is further off than the lowered start would otherwise decay its excess
    // with the fast half-life = a visible quick slide at a clip class change). aTarget <= 0 = slip off (as SetSlip(0, ..)).
    void SetSlipRamp(float aTarget, float aHalflife, float aDownRate)
    {
        const float t = (std::isfinite(aTarget) && aTarget > 0.0f) ? aTarget : 0.0f;
        m_slipHalflife = (std::isfinite(aHalflife) && aHalflife > 0.005f) ? aHalflife : 0.04f;
        m_slipDownRate = (std::isfinite(aDownRate) && aDownRate > 0.0f) ? aDownRate : 0.0f;
        if (t <= 0.0f || m_slipStart <= 0.0f || t >= m_slipStart || !(m_slipDownRate > 0.0f))
            m_slipRampGoal = -1.0f, m_slipStart = t;
        else
            m_slipRampGoal = t; // lowering: eased in Update (m_slipStart > goal)
    }
    // Only the bounded cancelled-start transition requests this. Capture an
    // already established support; never plant a new foot or retain a swing.
    // All drift/reach/contact guards still apply. The other foot keeps its
    // authored motion and the normal release path.
    void SetSupportHold(int aSide)
    {
        const int side = aSide == kLeft || aSide == kRight ? aSide : -1;
        if (side != m_supportHoldSide)
        {
            m_supportHoldSide = side;
            m_supportHoldEligible = side >= 0 && m_foot[side].locked &&
                                    m_foot[side].contact && m_foot[side].s >= 0.999f;
        }
    }
    // Ordinary standing stop only; a plain idle may continue an existing stop
    // episode, never start one. The driver clears this on movement, air, stance,
    // turns or ownership/gate changes. This does not acquire or prolong contacts.
    void SetStandingStopSupport(bool aStop, bool aIdle)
    {
        m_standingStopSupport = aStop || (aIdle && m_standingStopSupport);
    }
    // loco C1b plant / replant (see FootReplantConfig): replant mode for the next updates (the driver sets it per feed from the
    // stance and the UTPPSetMMFootReplant mask) and the Stops profile (playing clip idle / stop). Off (default) = the 0.5.9 path,
    // bit for bit, for every foot not in a replant step / handover.
    void SetReplant(bool aOn, bool aStops)
    {
        m_replantOn = aOn;
        m_replantStops = aStops;
    }
    bool ReplantOn() const { return m_replantOn; }
    // V2 grounded crouch only. Authored contacts/heel roll own the step; a
    // capsule/animation mismatch may yield the planar lock smoothly, but must
    // not add repeated radius sine hops during the same support phase.
    void SetCrouchContactPolicy(bool aOn) { m_crouchContactPolicy = aOn; }
    // batch 2 knee fix (UTPPSetMMKneeFix bit 2): pole carried from the animated hip-ankle axis in the two-bone solve (see
    // legik::SolveTwoBone aPoleCarry). Off (default) = the loco batch 1 solve bit for bit.
    void SetPoleCarry(bool aOn) { m_poleCarry = aOn; }
    // False = invalid (FootReplantConfig::Valid), previous kept. Applies from the next step on.
    bool SetReplantConfig(const FootReplantConfig& aCfg)
    {
        if (!FootReplantConfig::Valid(aCfg))
            return false;
        m_rcfg = aCfg;
        return true;
    }
    const FootReplantConfig& ReplantConfig() const { return m_rcfg; }

    // Instant reset (no blend): both feet unlocked, character transform at the origin, statistics cleared.
    void Reset()
    {
        m_supportHoldSide = -1;
        m_supportHoldEligible = false;
        m_standingStopSupport = false;
        for (Foot& f : m_foot)
            f = Foot{};
        m_charX = m_charY = m_charYaw = 0.0f;
        m_capsuleSpeed = 0.0f;
        m_diag = FootLockDiagnostics{};
    }

    // aDt real seconds; aVelX/aVelY capsule world velocity (m/s); aYaw player yaw (rad); contacts in [0,1];
    // aIn = matcher output (NumBones entries). Output() receives the corrected pose.
    void Update(float aDt, float aVelX, float aVelY, float aYaw, float aContactL, float aContactR,
                std::span<const BoneTransform> aIn)
    {
        if (!m_ok || aIn.size() != m_nb)
            return;
        for (uint32_t b = 0; b < m_nb; ++b)
            m_out[b] = aIn[b];
        ++m_diag.updates;
        float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        dt = std::fmin(dt, m_cfg.maxDt);
        if (m_slipRampGoal >= 0.0f) // lowering slip start (SetSlipRamp)
        {
            m_slipStart = std::fmax(m_slipRampGoal, m_slipStart - m_slipDownRate * dt);
            if (m_slipStart <= m_slipRampGoal)
                m_slipRampGoal = -1.0f;
        }
        const float vx = std::isfinite(aVelX) ? aVelX : 0.0f;
        const float vy = std::isfinite(aVelY) ? aVelY : 0.0f;
        m_capsuleSpeed = std::sqrt(vx * vx + vy * vy);
        m_charX += vx * dt;
        m_charY += vy * dt;
        if (std::isfinite(aYaw))
            m_charYaw = math::WrapAngle(aYaw);
        if (!std::isfinite(m_charX) || !std::isfinite(m_charY) || std::fabs(m_charX) > 1e6f || std::fabs(m_charY) > 1e6f)
        {
            m_charX = m_charY = 0.0f; // also re-centres (precision); locks dropped
            ResetFeet();
            ++m_diag.nonFiniteResets;
        }
        m_cos = std::cos(m_charYaw);
        m_sin = std::sin(m_charYaw);
        // animated lock points (toes) of both feet before any correction: the standing-stop swing-contact check of a foot compares its
        // own toe with the other foot's animated toe (loco-stop-knees 2026-11-09)
        for (int s = 0; s < 2; ++s)
            m_animToe[s] = ModelSpace(m_out, LockBone(s)).p;
        const float contacts[2] = {aContactL, aContactR};
        for (int s = 0; s < 2; ++s)
            UpdateFoot(s, dt, std::isfinite(contacts[s]) ? contacts[s] : 0.0f);
        if (m_diag.foot[0].stepped && m_diag.foot[1].stepped) // loco C1b invariant (one foot at a time, also within an update)
            ++m_diag.simultaneousSteps;
    }

    std::span<const BoneTransform> Output() const { return m_out; }
    const FootLockDiagnostics& Diagnostics() const { return m_diag; }
    // Bone whose position is locked (toe, or the foot if the database has no toe below the foot).
    int32_t LockBone(int aSide) const
    {
        return m_toePathLen[aSide & 1] > 0 ? m_toePath[aSide & 1][m_toePathLen[aSide & 1] - 1] : m_chain[aSide & 1][2];
    }
    int32_t ChainBone(int aSide, int aLink) const { return m_chain[aSide & 1][aLink < 0 ? 0 : (aLink > 2 ? 2 : aLink)]; }
    std::span<const int32_t> Parents() const { return m_parents; }

    // Contact value of the played frame, interpolated towards the next frame of the same clip.
    static float SampleContact(const Database& aDb, int32_t aFrame, float aAlpha, int aSide)
    {
        if (aFrame < 0 || static_cast<uint32_t>(aFrame) >= aDb.NumFrames())
            return 0.0f;
        const uint32_t f0 = static_cast<uint32_t>(aFrame);
        const uint32_t f1 = (f0 + 1 < aDb.NumFrames() && aDb.ClipOfFrame(f0 + 1) == aDb.ClipOfFrame(f0)) ? f0 + 1 : f0;
        const float a = (std::isfinite(aAlpha) ? std::fmin(std::fmax(aAlpha, 0.0f), 1.0f) : 0.0f);
        const float c0 = aDb.Contacts(f0)[size_t(aSide & 1)];
        const float c1 = aDb.Contacts(f1)[size_t(aSide & 1)];
        return c0 + (c1 - c0) * a;
    }

    // Model-space (character frame) transform of bone aBone in aPose (walks the parent chain; tests / tools).
    BoneTransform ModelSpace(std::span<const BoneTransform> aPose, int32_t aBone) const
    {
        int32_t path[kMaxDepth * 2];
        int32_t n = 0;
        for (int32_t b = aBone; b >= 0 && n < kMaxDepth * 2; b = m_parents[size_t(b)])
            path[n++] = b;
        BoneTransform g{};
        for (int32_t i = n - 1; i >= 0; --i)
            g = Mul(g, aPose[size_t(path[i])]);
        return g;
    }

    void CharacterToWorld(math::Vec3 aP, float& aX, float& aY) const
    {
        aX = m_charX + m_cos * aP.x - m_sin * aP.y;
        aY = m_charY + m_sin * aP.x + m_cos * aP.y;
    }

private:
    static constexpr int32_t kMaxDepth = 32;

    struct Foot
    {
        bool contact = false;
        bool locked = false;
        bool suppressed = false;  // drift unlock: no relock until the contact ends
        float s = 0.0f;           // linear blend parameter
        float releaseWeight = 1.0f; // weight already baked into delta by the last positional solve
        math::Quat delta[3];      // last IK local rotation deltas (UpLeg, Leg, Foot): local = delta * animated
        float lockX = 0.0f, lockY = 0.0f; // world
        math::Vec3 offset{};      // character frame, target - animated ankle (z = 0)
        // loco C1b plant / replant (only while locked)
        bool replant = false;     // this lock runs in replant mode
        bool stepping = false;    // replant step in progress
        float stepD0 = 0.0f;      // m, offset at the step start
        float stepRem = 0.0f;     // m, offset left after the last advance (closes at stepSpeed)
        float stepEnd = 0.0f;     // m, the step ends at this offset
        float stepSpeed = 0.0f;   // m/s
        float lift = 0.0f;        // m, current ankle-target lift
        float tidyTime = 0.0f;    // s the tidy condition holds
        float clampTime = 0.0f;   // s the IK reach has been clamped in a row
        bool waiting = false;     // a radius / clamp step is held back while the other foot steps
        bool crouchContact = false; // retained through a smooth policy handover
        bool crouchTidy = false;
        math::Vec3 crouchVelocity{}; // planar WORLD velocity of the lock correction
        math::Vec3 crouchSourceWorld{}; // uncorrected toe, including capsule/yaw and prior pose stages
        bool crouchSourceValid = false;
        bool crouchSourceQuiet = false;
        math::Vec3 stopRel{};     // loco-stop-knees: at the lock, animated toe - the other foot's animated toe (character frame, z = 0)
    };

    static bool ValidConfig(const FootLockConfig& c)
    {
        auto pos = [](float v) { return std::isfinite(v) && v > 0.0f; };
        return std::isfinite(c.onThreshold) && std::isfinite(c.offThreshold) && c.offThreshold <= c.onThreshold &&
               pos(c.blendInTime) && pos(c.blendOutTime) && pos(c.maxDrift) && pos(c.maxOffset) && pos(c.maxExtension) && c.maxExtension < 1.0f &&
               pos(c.softExtension) && c.softExtension <= c.maxExtension &&
               pos(c.maxDt) &&
               std::isfinite(c.settleSpeed) && c.settleSpeed >= 0.0f && std::isfinite(c.settleCapsuleSpeed) && c.settleCapsuleSpeed >= 0.0f;
    }

    static BoneTransform Mul(const BoneTransform& aParent, const BoneTransform& aLocal)
    {
        return BoneTransform{aParent.p + math::QuatRotate(aParent.q, aLocal.p), math::QuatMul(aParent.q, aLocal.q)};
    }
    static float Len(math::Vec3 v) { return std::sqrt(math::Dot(v, v)); }
    static float Smoothstep(float s) { return s * s * (3.0f - 2.0f * s); }

    void ResetFeet()
    {
        m_supportHoldSide = -1;
        m_supportHoldEligible = false;
        m_standingStopSupport = false;
        for (Foot& f : m_foot)
            f = Foot{};
    }

    // loco C0: records a foot event (diagnostics only; the pose is not affected)
    void FootEvent(int aSide, uint8_t aReason, float aDist)
    {
        FootLockEvent& e = m_diag.lastEvent[aSide & 1];
        e.seq = ++m_eventSeq;
        e.reason = aReason;
        e.dist = aDist;
    }

    void WorldToCharacter(float aX, float aY, float& aCx, float& aCy) const
    {
        const float dx = aX - m_charX, dy = aY - m_charY;
        aCx = m_cos * dx + m_sin * dy;
        aCy = -m_sin * dx + m_cos * dy;
    }

    // loco C1b: a locked foot in replant mode (aOff = locked toe - animated toe, character frame, |aOff| <= maxDrift). Starts /
    // advances a step, or hands the foot over to the 0.5.9 path after a replant -> off switch. Sets f.offset, the lock point and
    // f.lift (see FootReplantConfig).
    void UpdateReplant(int aSide, float aDt, math::Vec3 aToe, math::Vec3 aOff)
    {
        Foot& f = m_foot[aSide];
        if (m_crouchContactPolicy || f.crouchContact)
        {
            UpdateCrouchContact(aSide, aDt, aToe, aOff);
            return;
        }
        const Foot& o = m_foot[aSide ^ 1];
        const FootReplantConfig& rc = m_rcfg;
        const float len = Len(aOff);
        f.offset = aOff;
        if (!f.stepping)
        {
            const bool stops = m_replantStops;
            const float radius = stops ? rc.radiusStops : rc.radius;
            if (m_replantOn && m_capsuleSpeed < rc.tidyCapsuleSpeed && len > rc.tidyRadius)
                f.tidyTime += aDt;
            else
                f.tidyTime = 0.0f;
            uint8_t reason = kFootEventNone;
            if (!m_replantOn)
            {
                // replant switched off while planted: hand over at once when the 0.5.9 path keeps this offset (slip start; slip
                // off: anything up to maxDrift), otherwise one step to the animated toe first
                const float keep = m_slipStart > 0.0f ? m_slipStart : m_cfg.maxDrift;
                if (len <= keep)
                {
                    f.replant = false;
                    f.lift = f.tidyTime = 0.0f;
                    f.waiting = false;
                    ++m_diag.handovers;
                    return;
                }
                reason = kFootEventHandover;
            }
            else if (len > radius)
                reason = kFootEventRadius;
            else if (f.clampTime >= rc.clampTime)
                reason = kFootEventClamp;
            else if (f.tidyTime >= rc.tidyDelay && o.locked && !o.stepping)
                reason = kFootEventTidy;
            // one foot at a time (the right foot also waits for a left step that ended in this very update); a foot held back from a
            // radius / clamp step, or already reach-limited (dragged), makes the other foot's step hurry
            const bool otherBusy = (o.locked && o.stepping) || (aSide == kRight && m_diag.foot[kLeft].stepped);
            f.waiting = (reason == kFootEventRadius || reason == kFootEventClamp || f.clampTime > 0.0f) && otherBusy;
            if (reason == kFootEventNone || otherBusy)
            {
                f.lift = 0.0f;
                return;
            }
            // tidy / handover / clamp steps go to the animated toe (a reach-limited foot is only reachable again near it)
            const bool toToe = reason == kFootEventTidy || reason == kFootEventHandover || reason == kFootEventClamp;
            f.stepping = true;
            f.stepD0 = len;
            f.stepRem = len;
            f.stepEnd = toToe ? rc.tidyRelock : (stops ? rc.ratioStops : rc.ratio) * radius;
            if (!(f.stepEnd < len))
                f.stepEnd = std::fmin(rc.tidyRelock, 0.5f * len);
            f.stepSpeed = stops ? rc.stepSpeedStops : rc.stepSpeed;
            f.tidyTime = f.clampTime = 0.0f;
            ++m_diag.footSteps[aSide];
            ++m_diag.stepReasons[reason];
            FootEvent(aSide, reason, len);
        }
        // advance: the remaining offset closes at stepSpeed relative to the animated toe (the lock point also follows the toe's own
        // world motion, so a step ends within (d0 - stepEnd) / stepSpeed whatever the capsule / animation mismatch), never past stepEnd
        ++m_diag.stepUpdates;
        m_diag.foot[aSide].stepped = true;
        f.waiting = false;
        const bool hurry = o.locked && o.waiting;
        m_diag.foot[aSide].hurried = hurry;
        const float speed = hurry ? f.stepSpeed * rc.hurry : f.stepSpeed;
        const float from = std::fmin(len, f.stepRem);
        const float newLen = from > f.stepEnd ? std::fmax(from - speed * aDt, f.stepEnd) : from;
        const float k = len > 1e-6f ? newLen / len : 0.0f;
        f.stepRem = newLen;
        f.offset = math::Vec3{aOff.x * k, aOff.y * k, 0.0f};
        CharacterToWorld(math::Vec3{aToe.x + f.offset.x, aToe.y + f.offset.y, aToe.z}, f.lockX, f.lockY);
        if (newLen <= f.stepEnd)
        {
            f.stepping = false; // planted again where the toe is shown
            f.lift = 0.0f;
            return;
        }
        const float span = f.stepD0 - f.stepEnd;
        const float progress = span > 1e-6f ? std::fmin(1.0f, std::fmax(0.0f, (f.stepD0 - newLen) / span)) : 1.0f;
        const float h = std::fmin(rc.lift, rc.liftFraction * f.stepD0);
        f.lift = std::fmax(0.0f, h * std::sin(math::kPi * progress));
        if (f.lift > m_diag.maxLift)
            m_diag.maxLift = f.lift;
    }

    void UpdateCrouchContact(int aSide, float dt, math::Vec3 toe, math::Vec3 off)
    {
        Foot& f = m_foot[aSide];
        const Foot& other = m_foot[aSide ^ 1];
        const FootReplantConfig& rc = m_rcfg;
        const float len = Len(off);
        const bool active = m_crouchContactPolicy && m_replantOn;
        // Capsule braking can finish before the authored stop. Do not apply
        // an idle rate while its source toe is still closing at walking speed.
        const bool still = m_capsuleSpeed < rc.tidyCapsuleSpeed && f.crouchSourceQuiet;
        const bool reach = f.clampTime >= rc.clampTime;
        if (!f.crouchContact)
        {
            // A legacy step may be in flight when stance changes. Retain its
            // current planar lock and fade only the existing lift below.
            f.crouchContact = true;
            f.stepping = f.waiting = false;
            f.tidyTime = 0.0f;
        }
        if (!active || !still || !(m_cfg.settleSpeed > 0.0f))
        {
            f.crouchTidy = false;
            f.tidyTime = 0.0f;
        }
        else if (!f.crouchTidy)
        {
            f.tidyTime = len > rc.tidyRadius ? f.tidyTime + dt : 0.0f;
            const bool busy = other.crouchTidy || other.stepping ||
                              (aSide == kRight && m_diag.foot[kLeft].stepped);
            if (f.tidyTime >= rc.tidyDelay && other.locked && !busy)
            {
                f.crouchTidy = true;
                ++m_diag.footSteps[aSide];
                ++m_diag.stepReasons[kFootEventTidy];
                FootEvent(aSide, kFootEventTidy, len);
            }
        }
        // Hold all safe support; only give the excess, or the bounded idle
        // tidy. A reach-limited leg can give earlier, never bypassing IK or
        // the maxDrift guard in UpdateFoot. No hurry and no generated lift.
        const float keep = m_slipStart > 0.0f ? m_slipStart : m_cfg.maxDrift;
        const float radius = m_replantStops ? rc.radiusStops : rc.radius;
        const bool gentle = still && !reach;
        // Use the existing soft-lock range for a moving support. A fixed 1m/s
        // correction cap accumulated >20cm during fast rotating starts and
        // caused maxDrift to release a still-planted foot. Exponential give of
        // only the excess is bounded by that excess, with no invented step/Z.
        const float movingRadius = m_slipStart > 0.0f ? std::fmin(radius, m_slipStart) : radius;
        const float target = !active ? keep : (f.crouchTidy || reach ? rc.tidyRelock :
                                              (gentle ? radius : movingRadius));
        const float excess = std::fmax(0.0f, len - target);
        const float cap = std::fmin(rc.stepSpeedStops, m_cfg.settleSpeed);
        const float speed = gentle ? std::fmin(cap, 4.0f * excess) :
            (dt > 0.0f ? excess * (1.0f - std::exp2(-dt / m_slipHalflife)) / dt : 0.0f);
        const float inv = len > 1e-6f ? 1.0f / len : 0.0f;
        const math::Vec3 goal{(-m_cos * off.x + m_sin * off.y) * inv * speed,
                              (-m_sin * off.x - m_cos * off.y) * inv * speed, 0.0f};
        const float alpha = dt > 0.0f ? (gentle ? 1.0f - std::exp(-dt / 0.06f) : 1.0f) : 0.0f;
        f.crouchVelocity = f.crouchVelocity + (goal - f.crouchVelocity) * alpha;
        // Entering idle must not carry a fast moving correction into tidy.
        // Keep its direction, but honor the same displacement limit immediately.
        const float velocity = Len(f.crouchVelocity);
        if (gentle && dt > 0.0f && velocity > cap && velocity > 1e-8f)
            f.crouchVelocity = f.crouchVelocity * (cap / velocity);
        math::Vec3 move = f.crouchVelocity * dt;
        const float travel = Len(move);
        if (travel > excess && travel > 1e-8f)
        {
            move = move * (excess / travel);
            if (dt > 0.0f)
                f.crouchVelocity = move * (1.0f / dt);
        }
        f.lockX += move.x;
        f.lockY += move.y;
        float x = 0.0f, y = 0.0f;
        WorldToCharacter(f.lockX, f.lockY, x, y);
        f.offset = {x - toe.x, y - toe.y, 0.0f};
        f.lift = std::fmax(0.0f, f.lift - 0.2f * dt);
        f.waiting = false;
        f.stepping = f.crouchTidy;
        if (travel > 1e-8f && excess > 0.0f)
        {
            if (f.crouchTidy)
            {
                ++m_diag.stepUpdates;
                m_diag.foot[aSide].stepped = true;
            }
            else
            {
                ++m_diag.slips;
                ++m_diag.footSlips[aSide];
            }
        }
        if (f.crouchTidy && Len(f.offset) <= rc.tidyRelock + 0.001f && Len(f.crouchVelocity) < 0.01f)
        {
            f.crouchTidy = f.stepping = false;
            f.tidyTime = 0.0f;
        }
        if (!active && len <= keep + 0.001f && Len(f.crouchVelocity) < 0.01f && f.lift <= 0.0f)
        {
            f.crouchContact = f.crouchTidy = f.stepping = f.replant = false;
            f.crouchVelocity = {};
            ++m_diag.handovers;
        }
    }

    void UpdateFoot(int aSide, float aDt, float aContact)
    {
        Foot& f = m_foot[aSide];
        FootLockFootDiag& d = m_diag.foot[aSide];
        d.clamped = false;
        d.correction = 0.0f;
        d.error = 0.0f;
        d.contactIn = aContact; // loco C0
        d.stepped = d.hurried = false; // loco C1b
        d.stopReleased = false;        // loco-stop-knees

        // contact hysteresis (disabled -> contact off -> blend out)
        if (!m_enabled)
            f.contact = false;
        else if (!f.contact && aContact > m_cfg.onThreshold)
            f.contact = true;
        else if (f.contact && aContact < m_cfg.offThreshold)
            f.contact = false;
        if (!f.contact)
        {
            f.suppressed = false;
            if (m_supportHoldSide == aSide)
                m_supportHoldEligible = false;
        }

        // animated chain (model space)
        const int32_t ia = m_chain[aSide][0], ib = m_chain[aSide][1], ic = m_chain[aSide][2];
        BoneTransform gp{};
        for (int32_t i = 0; i < m_pathLen[aSide]; ++i)
            gp = Mul(gp, m_out[size_t(m_path[aSide][i])]);
        const BoneTransform ga = Mul(gp, m_out[size_t(ia)]);
        const BoneTransform gb = Mul(ga, m_out[size_t(ib)]);
        const BoneTransform gc = Mul(gb, m_out[size_t(ic)]);
        if (!math::IsFinite(gc.p) || !math::IsFinite(gc.q) || !math::IsFinite(gb.p) || !math::IsFinite(ga.q))
        {
            f = Foot{};
            if (m_supportHoldSide == aSide)
                m_supportHoldEligible = false;
            ++m_diag.nonFiniteResets;
            d = FootLockFootDiag{};
            return;
        }

        BoneTransform gl = gc; // lock point (toe)
        for (int32_t i = 0; i < m_toePathLen[aSide]; ++i)
            gl = Mul(gl, m_out[size_t(m_toePath[aSide][i])]);
        if (!math::IsFinite(gl.p))
        {
            f = Foot{};
            if (m_supportHoldSide == aSide)
                m_supportHoldEligible = false;
            ++m_diag.nonFiniteResets;
            d = FootLockFootDiag{};
            return;
        }

        if (m_crouchContactPolicy || f.crouchContact)
        {
            if (aDt > 0.0f)
            {
                math::Vec3 source{};
                CharacterToWorld(gl.p, source.x, source.y);
                const float allowed = m_rcfg.tidyCapsuleSpeed * aDt;
                const math::Vec3 delta = source - f.crouchSourceWorld;
                f.crouchSourceQuiet = f.crouchSourceValid && f.locked &&
                    math::Dot(delta, delta) < allowed * allowed;
                f.crouchSourceWorld = source;
                f.crouchSourceValid = true;
            }
        }
        else
            f.crouchSourceValid = f.crouchSourceQuiet = false;

        const float wPrev = Smoothstep(f.s);
        if (f.contact && !f.locked && !f.suppressed)
        {
            math::Vec3 shown = gl.p;
            if (wPrev > 0.0f)
            {
                // still blending out (rotation space): lock where the toe is displayed, full weight, no re-blend
                shown = BlendedLockPoint(aSide, gp, ReleaseWeight(f, wPrev));
                f.s = 1.0f;
                ++m_diag.relocks;
            }
            CharacterToWorld(shown, f.lockX, f.lockY);
            f.offset = math::Vec3{shown.x - gl.p.x, shown.y - gl.p.y, 0.0f};
            f.locked = true;
            f.releaseWeight = 1.0f;
            ++m_diag.locks;
            // loco C1b: the mode is taken at the lock; no step / lift / timers carried over
            f.replant = m_replantOn;
            f.stepping = false;
            f.crouchContact = f.crouchTidy = false;
            f.crouchVelocity = {};
            f.lift = f.tidyTime = f.clampTime = 0.0f;
            f.waiting = false;
            const math::Vec3 rel = gl.p - m_animToe[aSide ^ 1]; // loco-stop-knees: reference of the standing-stop swing-contact check
            f.stopRel = math::Vec3{rel.x, rel.y, 0.0f};
        }
        if (f.locked)
        {
            if (!f.contact)
            {
                f.releaseWeight = wPrev;
                f.locked = false; // blend out with the last IK rotation deltas
                f.stepping = f.waiting = false; // loco C1b
                f.lift = 0.0f;
            }
            else
            {
                float tx = 0.0f, ty = 0.0f;
                WorldToCharacter(f.lockX, f.lockY, tx, ty);
                const math::Vec3 off{tx - gl.p.x, ty - gl.p.y, 0.0f};
                if (!(Len(off) <= m_cfg.maxDrift))
                {
                    f.releaseWeight = wPrev;
                    f.locked = false; // teleport / big correction: blend out with the previous update's deltas
                    f.suppressed = true;
                    f.stepping = f.waiting = false; // loco C1b
                    f.lift = 0.0f;
                    ++m_diag.driftUnlocks;
                    ++m_diag.footDriftUnlocks[aSide]; // loco C0
                    FootEvent(aSide, kFootEventDrift, Len(off));
                }
                else if (f.replant || f.stepping || m_replantOn)
                {
                    f.replant = true; // loco C1b (an off -> replant switch takes the planted foot over at once)
                    UpdateReplant(aSide, aDt, gl.p, off);
                }
#if !(defined(UTPP_FLSTOP_MUTATION) && UTPP_FLSTOP_MUTATION == 1) // mutation 1: no standing-stop swing-contact release
                else if (StopSwingContact(aSide, gl.p))
                {
                    // a swing, not a plant: fades out from its small blend-in weight with the previous update's deltas (as a drift
                    // unlock), no relock until this contact ends
                    f.releaseWeight = wPrev;
                    f.locked = false;
                    f.suppressed = true;
                    f.stepping = f.waiting = false;
                    f.lift = 0.0f;
                    d.stopReleased = true;
                    ++m_diag.stopSwingReleases;
                }
#endif
                else
                {
                    f.offset = off;
                    // A valid support during a short start cancellation should
                    // not be dragged along by idle settling. Stop protecting it
                    // before a large correction or a near-straight reach would
                    // fight the new pose. No bone lengths or IK limits change.
                    bool holdSupport = false, holdStopSettle = false;
                    const bool cancelledSupport = m_supportHoldEligible && m_supportHoldSide == aSide;
                    const bool standingSupport = m_standingStopSupport && f.s >= 0.999f;
                    if ((cancelledSupport || standingSupport) && aContact > m_cfg.onThreshold &&
                        m_capsuleSpeed < m_cfg.settleCapsuleSpeed && Len(off) <= 0.08f)
                    {
                        const float length = Len(gb.p - ga.p) + Len(gc.p - gb.p);
                        const float safeReach = std::fmax(Len(gc.p - ga.p), length * m_cfg.softExtension);
                        const bool reachable = Len(gc.p + off - ga.p) <= safeReach;
                        holdSupport = cancelledSupport && reachable;
                        holdStopSettle = standingSupport && reachable;
                    }
                    // settling towards the animated toe while the capsule stands (world-space lock moves)
                    const float len = Len(off);
                    const float step = m_cfg.settleSpeed * aDt;
                    if (!holdSupport && !holdStopSettle && m_capsuleSpeed < m_cfg.settleCapsuleSpeed && step > 0.0f && len > 1e-5f)
                    {
                        const float k = std::fmin(1.0f, step / len);
                        f.offset = math::Vec3{off.x * (1.0f - k), off.y * (1.0f - k), 0.0f};
                        CharacterToWorld(math::Vec3{gl.p.x + f.offset.x, gl.p.y + f.offset.y, gl.p.z}, f.lockX, f.lockY);
                        ++m_diag.settles;
                    }
                    const float ls = Len(f.offset);
                    // The ordinary-stop hold suppresses cosmetic settling only.
                    // Necessary excess still uses the proven soft-slip path.
                    if (!holdSupport && m_slipStart > 0.0f && ls > m_slipStart)
                    {
                        const float ex = (ls - m_slipStart) * std::exp2(-aDt / m_slipHalflife);
                        const float k = (m_slipStart + ex) / ls;
                        f.offset = math::Vec3{f.offset.x * k, f.offset.y * k, 0.0f};
                        CharacterToWorld(math::Vec3{gl.p.x + f.offset.x, gl.p.y + f.offset.y, gl.p.z}, f.lockX, f.lockY);
                        ++m_diag.slips;
                        ++m_diag.footSlips[aSide]; // loco C0
                    }
#if !(defined(UTPP_FLCAP_MUTATION) && UTPP_FLCAP_MUTATION == 1) // mutation 1: no soft offset bound (only maxDrift releases)
                    if (!holdSupport && m_slipStart > 0.0f && Len(f.offset) > m_cfg.maxOffset)
                    {
                        // the animated toe ran away from the soft lock faster than the slip follows: released like a drift unlock, so
                        // the blend-out starts from the last solved correction instead of pulling the leg further than maxOffset
                        f.releaseWeight = wPrev;
                        f.locked = false;
                        f.suppressed = true;
                        f.stepping = f.waiting = false;
                        f.lift = 0.0f;
                        ++m_diag.offsetReleases;
                        ++m_diag.footOffsetReleases[aSide];
                    }
#endif
                }
            }
        }
        if (!f.locked)
        {
            f.crouchContact = f.crouchTidy = false;
            f.crouchVelocity = {};
        }
        f.s = f.locked ? std::fmin(1.0f, f.s + aDt / m_cfg.blendInTime) : std::fmax(0.0f, f.s - aDt / m_cfg.blendOutTime);
        const float w = Smoothstep(f.s);
        d.contact = f.contact;
        d.locked = f.locked;
        d.weight = w;
        d.offset = f.locked ? Len(f.offset) : 0.0f;
        d.replant = f.locked && f.replant; // loco C1b
        d.stepping = f.locked && f.stepping;
        d.lift = f.locked ? f.lift : 0.0f;
        if (w <= 0.0f)
        {
            f.offset = {};
            for (math::Quat& q : f.delta)
                q = math::Quat{};
            return;
        }
        if (f.locked)
        {
            math::Vec3 target = gc.p + f.offset * w;
            if (f.lift > 0.0f) // loco C1b replant step: lifted ankle target (never in the 0.5.9 path)
                target.z += f.lift * w;
            Solve(aSide, gp, ga, gb, gc, target, gl.p, d);
            // loco C1b clamp trigger (read only in replant mode): hard clamp or a target missed in the soft reach zone
            f.clampTime = (d.clamped || d.error > m_rcfg.reachError) ? f.clampTime + aDt : 0.0f;
            return;
        }
        // Blend-out in rotation space: local = nlerp(identity, delta, w) * animated local. The correction fades with
        // the animation's own joint motion, relative to its weight at release (a frozen positional offset would be amplified by the leg geometry during
        // the swing).
        const float release = ReleaseWeight(f, w);
        const int32_t bones[3] = {ia, ib, ic};
        for (int k = 0; k < 3; ++k)
        {
            BoneTransform& o = m_out[size_t(bones[k])];
            o.q = math::QuatNormalize(math::QuatMul(math::QuatNlerpShortest(math::Quat{}, f.delta[k], release), o.q));
        }
        const BoneTransform fa = Mul(gp, m_out[size_t(ia)]);
        BoneTransform fc = Mul(Mul(fa, m_out[size_t(ib)]), m_out[size_t(ic)]);
        // Sink bound of the blend-out (loco-fix 2026-11-09): the faded rotation deltas belong to
        // the leg as it was at the release; on a leg the animation folds fast (sprint -> slide entry) they took the ankle 6.5 cm below the
        // animated one (sole -3.2 cm). The lock never moves the ankle vertically (the target z is the animated ankle z), so the fade may
        // not take it more than kBlendOutSinkMax below that either: beyond, the FADED leg is lifted to that bound by the relative two-bone
        // solve (pole carried from the faded knee and preserved, foot rotation kept): identity as the lift tends to zero, so the bound
        // switches on / off without a rotation step (a solve from the animated leg did: footlock (d) leg step 25 -> 42 deg). The stored
        // deltas are untouched (the fade goes on as before) and a fade within the bound is unchanged.
#if !(defined(UTPP_FLSINK_MUTATION) && UTPP_FLSINK_MUTATION == 1) // mutation 1: no sink bound
        if (std::isfinite(fc.p.z) && fc.p.z < gc.p.z - kBlendOutSinkMax)
        {
            const BoneTransform fb = Mul(fa, m_out[size_t(ib)]);
            const legik::TwoBoneSolution s = legik::SolveTwoBone(gp, fa, fb, fc, math::Vec3{fc.p.x, fc.p.y, gc.p.z - kBlendOutSinkMax},
                                                                 gl.p, m_cfg.maxExtension, m_cfg.softExtension, true, true);
            if (s.status == legik::TwoBoneStatus::kOk && math::IsFinite(s.ankle))
            {
                for (int k = 0; k < 3; ++k)
                    m_out[size_t(bones[k])].q = s.local[k];
                fc.p = s.ankle;
                ++m_diag.blendOutLifts;
            }
        }
#endif
        d.correction = Len(fc.p - gc.p);
        if (d.correction > m_diag.maxCorrection)
            m_diag.maxCorrection = d.correction;
    }

    // Standing-stop swing contact (loco-stop-knees 2026-11-09). In an ordinary standing stop
    // the blended contact of a crossfade can rise on the SWING foot (loco film walk stop f1559: contact 0.85 while that foot is 9 cm up and
    // passes the planted foot at 3 m/s). Locked, the soft slip dragged it ~12 cm behind its animation and the blend-out carried that lag
    // straight into the other foot, which its lock holds ~10 cm ahead of its animation: both ankles in one point (0.9 cm apart). A lock still
    // blending in (a fresh lock; a relock while blending out starts at full weight and is not checked) while the other foot is planted at
    // full weight is released when its animated toe has moved more than kStopSwingMax relative to that foot's animated toe since the lock:
    // both toes of a double support move together in the character frame (also under a capsule / animation mismatch, which moves both), the
    // toe of a passing swing foot does not (offline: swing 5.0 / 11.2 cm after 1 / 2 updates, real plants <= 2 cm over the blend-in).
    bool StopSwingContact(int aSide, math::Vec3 aToe) const
    {
        const Foot& f = m_foot[aSide];
        const Foot& o = m_foot[aSide ^ 1];
        if (!m_standingStopSupport || !(f.s < 1.0f) || !o.locked || !(o.s >= 0.999f))
            return false;
        const math::Vec3 rel = aToe - m_animToe[aSide ^ 1];
        const float dx = rel.x - f.stopRel.x, dy = rel.y - f.stopRel.y;
        return dx * dx + dy * dy > kStopSwingMax * kStopSwingMax;
    }

    // Displayed lock point (model space) when the stored deltas are applied with weight aW (pose not modified).
    // Delta already contains the positional blend-in weight: release it relative
    // to its captured weight, otherwise a partial contact loss applies the weight twice.
    static float ReleaseWeight(const Foot& f, float w)
    {
        return f.releaseWeight > 0.0f ? std::fmin(1.0f, w / f.releaseWeight) : 0.0f;
    }

    math::Vec3 BlendedLockPoint(int aSide, const BoneTransform& gp, float aW) const
    {
        const Foot& f = m_foot[aSide];
        BoneTransform g = gp;
        for (int k = 0; k < 3; ++k)
        {
            BoneTransform l = m_out[size_t(m_chain[aSide][k])];
            l.q = math::QuatNormalize(math::QuatMul(math::QuatNlerpShortest(math::Quat{}, f.delta[k], aW), l.q));
            g = Mul(g, l);
        }
        for (int32_t i = 0; i < m_toePathLen[aSide]; ++i)
            g = Mul(g, m_out[size_t(m_toePath[aSide][i])]);
        return g.p;
    }

    // Two-bone IK in model space (shared analytic solver mm/LegIk.hpp, extracted bit-exactly from this function); writes
    // UpLeg/Leg/Foot local rotations of m_out (foot keeps its animated model-space rotation). Leaves the animated pose on any
    // non-finite result.
    void Solve(int aSide, const BoneTransform& gp, const BoneTransform& ga, const BoneTransform& gb, const BoneTransform& gc,
               math::Vec3 t, math::Vec3 aToe, FootLockFootDiag& d)
    {
        const legik::TwoBoneSolution s = legik::SolveTwoBone(gp, ga, gb, gc, t, aToe, m_cfg.maxExtension, m_cfg.softExtension, m_poleCarry);
        if (s.status == legik::TwoBoneStatus::kBadLegLength)
            return;
        if (s.clamped)
        {
            d.clamped = true;
            ++m_diag.clamps;
        }
        if (s.status == legik::TwoBoneStatus::kBadTarget)
            return;
        if (s.status == legik::TwoBoneStatus::kNonFinite)
        {
            m_foot[aSide] = Foot{};
            if (m_supportHoldSide == aSide)
                m_supportHoldEligible = false;
            ++m_diag.nonFiniteResets;
            return;
        }
        Foot& f = m_foot[aSide];
        for (int k = 0; k < 3; ++k)
        {
            math::Quat& o = m_out[size_t(m_chain[aSide][k])].q;
            f.delta[k] = math::QuatNormalize(math::QuatMul(s.local[k], math::QuatInv(o))); // local = delta * animated
            o = s.local[k];
        }
        d.correction = Len(s.ankle - gc.p);
        d.error = Len(s.ankle - t);
        if (d.correction > m_diag.maxCorrection)
            m_diag.maxCorrection = d.correction;
    }

    bool m_ok = false;
    bool m_enabled = true;
    FootLockConfig m_cfg{};
    float m_slipStart = 0.0f, m_slipHalflife = 0.04f; // 0.5.9
    float m_slipRampGoal = -1.0f, m_slipDownRate = 0.0f; // SetSlipRamp (goal < 0 = no ramp running)
    bool m_replantOn = false, m_replantStops = false;  // loco C1b
    bool m_crouchContactPolicy = false;               // explicit V2 crouch policy, off preserves legacy path
    bool m_poleCarry = false;                          // batch 2 knee fix
    int m_supportHoldSide = -1;
    bool m_supportHoldEligible = false;
    bool m_standingStopSupport = false;
    FootReplantConfig m_rcfg{};
    uint32_t m_nb = 0;
    std::vector<int32_t> m_parents;
    std::vector<BoneTransform> m_out;
    int32_t m_chain[2][3] = {{-1, -1, -1}, {-1, -1, -1}};
    int32_t m_path[2][kMaxDepth] = {};
    int32_t m_pathLen[2] = {0, 0};
    int32_t m_toePath[2][kMaxDepth] = {};
    int32_t m_toePathLen[2] = {0, 0};
    Foot m_foot[2];
    float m_charX = 0.0f, m_charY = 0.0f, m_charYaw = 0.0f;
    float m_capsuleSpeed = 0.0f;
    float m_cos = 1.0f, m_sin = 0.0f;
    math::Vec3 m_animToe[2] = {};  // animated lock points of this update (character frame), before any correction
    FootLockDiagnostics m_diag{};
    uint64_t m_eventSeq = 0; // loco C0: foot event sequence (not cleared by Reset, so a reader never misses a new event)
};

// Per-feed mode selection of the driver's leg tail (MotionHost.cpp ApplyLegPipelineLocked) in one place, shared with the offline replay
// (tests/FeedReplayMotion.cpp --footlock) so that both run the same policy. Stance bits: 1 = stand / sprint, 2 = crouch (kReplantStand /
// kReplantCrouch of the driver).
struct FootFeedPolicy
{
    int32_t stance = 0;              // kLocoStand 1, kLocoCrouch 2, kLocoSprint 3
    bool stops = false;              // clip class idle / stop (replant Stops profile)
    bool v2 = false;
    bool airborne = false;
    float contactScale = 1.0f;       // air area landing ramp
    int32_t replantMask = 2;
    bool airHoldGroundReplant = false;
    bool poleCarry = true;
    int32_t cancelStartSupport = -1;
    bool standingStopGate = false;   // gate open, no hold, tpp, stick <= 0.15 (driver side)
    bool classStop = false, classIdle = false;
};
// Per clip-class slip start of the locked toe (soft slip of the 0.5.9 path; the crouch contact policy takes min(radius, slip start) as its
// moving radius): starts (Start / StartSprint classes) and turns / pivots / refaces may drift further from the animated toe before the lock
// slides, which trades a larger ankle correction for a shorter visible slide. active = false (default) = the single slip of SetSlip.
struct FootSlipProfile
{
    bool active = false;
    float base = 0.05f;   // m, every other class (the global slip start, UTPPSetMMOffsetCatchupParams)
    float start = 0.05f;  // m, Start / StartSprint
    float turn = 0.05f;   // m, Reface / Pivot* / MoveTurn / Turn* / TurnInPlace
    float halflife = 0.02f;
    float downRate = 0.15f; // m/s, how fast the slip start may be lowered after a class change
};
// 0 = other, 1 = start, 2 = turn / pivot / reface
inline int FootSlipGroup(uint8_t aClipClass)
{
    switch (aClipClass)
    {
    case steer::kClassStart:
    case steer::kClassStartSprint: return 1;
    case steer::kClassReface:
    case steer::kClassPivot:
    case steer::kClassPivotWalk:
    case steer::kClassPivotRun:
    case steer::kClassPivotSprint:
    case steer::kClassMoveTurn:
    case steer::kClassTurnWalk:
    case steer::kClassTurnRun:
    case steer::kClassTurnSprint:
    case steer::kClassTurnInPlace: return 2;
    default: return 0;
    }
}
inline void ApplyFootSlipProfile(FootLocker& aLocker, const FootSlipProfile& aProfile, uint8_t aClipClass)
{
    if (!aProfile.active)
        return;
    const int g = FootSlipGroup(aClipClass);
    aLocker.SetSlipRamp(g == 1 ? aProfile.start : (g == 2 ? aProfile.turn : aProfile.base), aProfile.halflife, aProfile.downRate);
}
inline void ApplyFootFeedPolicy(FootLocker& aLocker, const FootFeedPolicy& p)
{
    const int32_t stanceBit = p.stance == 2 ? 2 : ((p.stance == 1 || p.stance == 3) ? 1 : 0);
    aLocker.SetReplant(!p.airborne && p.contactScale > 0.0f && ((p.replantMask & stanceBit) != 0 || p.airHoldGroundReplant), p.stops);
    aLocker.SetCrouchContactPolicy(p.v2 && !p.airborne && p.contactScale > 0.f && p.stance == 2 && (p.replantMask & 2) != 0);
    aLocker.SetPoleCarry(p.poleCarry);
    const float k = p.airborne ? 0.0f : p.contactScale;
    aLocker.SetSupportHold(p.v2 && !p.airborne && k > 0.f && p.stance == 1 ? p.cancelStartSupport : -1);
    const bool standingStop = p.v2 && !p.airborne && k == 1.f && p.stance == 1 && p.standingStopGate;
    aLocker.SetStandingStopSupport(standingStop && p.classStop, standingStop && p.classIdle);
}
} // namespace utpp::mm
