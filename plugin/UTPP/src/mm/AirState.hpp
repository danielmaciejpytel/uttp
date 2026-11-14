// UTPP motion matching runtime - air phase machine (B1 air design, step S4). Header-only, C++20, deterministic,
// no allocations, O(1) per feed, NaN-safe inputs, no globals.
//
// AirState decides WHEN the driver takes clip playback away from ground MM while V is airborne and WHAT kind of air
// clip it wants. It never resolves a clip: the pick REQUEST (kind / direction / band / weight / foot hint) is resolved
// to a global frame by AirTable (src\mm\AirTable.hpp), which also feeds clip timing back through AirInput
// (clipRemainSeconds, clipRootSpeed).
//
// Phases: Ground -> TakeOff (rise) -> Descend -> [FallLoop] -> LandApproach -> Recover -> Ground (HandBack).
//         Ground -> Descend (Off clip, walking or faster) / FallLoop (standing) on a ledge drop.
// Modes:  Off   = no decisions (exact 0.6.5 behaviour);
//         Hold  = no air clips (no valid table): suppress search, contacts 0, OffsetRoot released, stride rate x holdRate
//                 while airborne; HandBack (dwellExempt) at touchdown;
//         Clips = full playback (requires AirInput.tableValid; otherwise the effective mode is Hold).
//
// Precedence (cross-design rule): AirOwns > Strafe > ground. While AirDecision.airOwns is true the driver must not let
// strafe (or any other area) unsuppress search or force a transition; strafe reads the PREVIOUS feed's airOwns.
// UpdateAirLocked runs before ComputeLegContextLocked; contactScale / releaseOffsetRoot feed the shared LegContext.
//
// Signals (AirInput, one per feed):
//   onGround      scriptInterface.IsOnGround(); if groundStale (air feed not stamped this frame) it is inferred from
//                 psmDetail instead (air details -> airborne, everything else -> grounded).
//   distToGround  downward probe distance in m; -1 = no hit (void), -2 = not probed this feed. Treated as the height
//                 h above the landing plane. A probe that disagrees with the dead-reckoned height by more than
//                 probeTolerance (0.5 m) is ignored; two consecutive disagreeing probes re-anchor (new floor).
//   psmFallPrev   PSM.Fall as read by the UTPP wrap: it is written by LocomotionAirEvents AFTER the wrap ran, so it
//                 is the PREVIOUS frame's value (0 on ground, 1 fall, 2.. heavier). The Heavy rule is latched per
//                 episode and is also evaluated on the first Recover feed, so a one-frame-late value still counts.
//   velBodyX/Y    capsule velocity in the body frame (x right, y forward, m/s).
//   stickMagnitude normalised stick 0..1; >= stickWalk counts as "stick at walk or faster".
//
// TTL sign convention: h = height of the character root ABOVE the landing plane, h >= 0 while airborne.
//   zRel = integral of vz dt since take-off (positive up, trapezoidal). For a jump the landing plane is assumed to be
//   the take-off plane, so the dead-reckoned h = zRel (valid while zRel >= -flatTolerance, i.e. -0.3 m).
//   Descending (vz <= 0): TTL = (vz + sqrt(vz^2 + 2 gDown h)) / gDown.
//   Rising (vz > 0):      TTL = vz / gUp + sqrt(2 hApex / gDown), hApex = h + vz^2 / (2 gUp).
//   No information (no probe, no valid dead reckoning): TTL = noHitTtl (1.5 s, motion no-hit value), flagged ttlKnown=0.
// gUp / gDown are learned online from successive airborne vz samples (EMA, clamped 8..30 m/s^2, defaults 19.2 / 16).
#pragma once

#include <cmath>
#include <cstdint>

#include "GameIntegration.hpp"

namespace utpp::mm::air
{
// ---- PSM detail helpers: single source of truth in GameIntegration.hpp (kPsmReleaseDetails shared with HoldEligible) ----
constexpr bool IsAirDetail(int d) noexcept { return integ::PsmIsAirDetail(d); }
constexpr bool IsJumpDetail(int d) noexcept { return integ::PsmIsJumpDetail(d); }
constexpr bool IsLandDetail(int d) noexcept { return integ::PsmIsLandDetail(d); }
constexpr bool IsReleaseDetail(int d) noexcept { return integ::PsmDetailReleases(d); }
constexpr bool IsClimbDetail(int d) noexcept { return integ::PsmIsClimbDetail(d); }
constexpr bool IsNoTriggerDetail(int d) noexcept { return integ::PsmIsNoTriggerDetail(d); }

// AirConfig::stepDownS the runtime uses (= MapConfig::groundDropGraceS of the chooser mapping, so both sides agree on a step-down).
inline constexpr float kStepDownSeconds = 0.12f;
// AirConfig::stepDownDepthM / stepDownMaxS the runtime uses (= adapter::kGroundDropStepM / kGroundDropStepMaxS, the step-down reach of the chooser mapping).
inline constexpr float kStepDownDepthM = 0.30f;
inline constexpr float kStepDownMaxSeconds = 0.20f;

enum class Mode : uint8_t { Off, Hold, Clips };
enum class Phase : uint8_t { Ground, TakeOff, Descend, FallLoop, LandApproach, Recover };
enum class Action : uint8_t { None, PlayTakeOff, PlayOff, PlayFallLoop, PlayLand, HandBack, Reset };
enum class PickKind : uint8_t { None, Start, Off, FallLoop, Land };
enum class Dir : uint8_t { F, B, L, R };
enum class Band : uint8_t { Stand, Walk, Run, Sprint };
enum class Weight : uint8_t { Light, Heavy };
enum class Foot : uint8_t { Any, L, R }; // Any: AirTable picks by leg-pose cost
enum class Entry : uint8_t
{
    None,
    LiftoffMinus, // clip liftoff frame - entryFrames (take-off lead, default 2)
    DropFrame,    // Off clip drop frame
    LoopStart,    // fall loop first frame
    TouchMinus,   // land clip touch frame - entryFrames (round(TTL * 30)), clamped >= 0 by AirTable
};

constexpr const char* PhaseName(Phase p) noexcept
{
    switch (p)
    {
    case Phase::Ground: return "Ground";
    case Phase::TakeOff: return "TakeOff";
    case Phase::Descend: return "Descend";
    case Phase::FallLoop: return "FallLoop";
    case Phase::LandApproach: return "LandApproach";
    case Phase::Recover: return "Recover";
    }
    return "?";
}

struct AirConfig
{
    float minAir = 0.10f;          // s airborne before a ledge fall may start
    float takeoffVz = 1.5f;        // m/s, generic take-off trigger
    float fallVz = -1.0f;          // m/s, ledge-fall trigger (vz <= fallVz)
    float fallMinTtl = 0.35f;      // s, ledge fall needs TTL >= this (or a probe miss)
    float landLead = 0.20f;        // s, LandApproach when TTL <= landLead
    float noHitTtl = 1.5f;         // s, TTL without any height information
    float probeTolerance = 0.5f;   // m, probe vs dead-reckoned height
    float flatTolerance = 0.3f;    // m, dead-reckoned flat landing valid while zRel >= -flatTolerance
    float gUpDefault = 19.2f;      // m/s^2 (measured)
    float gDownDefault = 16.0f;    // m/s^2 (script default)
    float gMin = 8.0f, gMax = 30.0f;
    float gEmaAlpha = 0.2f;        // per accepted sample
    float heavyVz = 7.0f;          // m/s predicted impact (motion 700 cm/s)
    int heavyFall = 3;             // PSM.Fall (previous frame) >= this -> Heavy
    float recoverLight = 0.30f, recoverHeavy = 0.50f, recoverStick = 0.20f; // s
    float stickWalk = 0.35f;       // stick magnitude counted as walk or faster
    float recoverTurnDeg = 45.0f;  // AIR CLIPS stage: stick (walk+) this far from the body at / after the touchdown -> no Recover
                                   // (hand back at once: ground MM turns; a straight land clip under a turning body pops the feet)
    float contactRamp = 0.08f;     // s, contactScale 0 -> 1 after touchdown
    float takeoffHalflife = 0.15f, landHalflife = 0.20f, fallHalflife = 0.20f; // s
    float holdRate = 0.5f;         // Hold mode stride-rate multiplier while airborne
    int takeoffLeadFrames = 2;     // entry = liftoff - 2
    float rateMin = 0.7f, rateMax = 1.8f;           // air warp clamp
    float recoverRateMin = 0.8f, recoverRateMax = 1.3f;
    float rateHalflife = 0.05f;    // s, warp smoothing
    float descendMaxTtl = 0.6f;    // s, Descend -> FallLoop when the clip remainder is unknown and TTL exceeds this
    float abortTime = 0.25f;       // s, a take-off that never leaves the ground hands back after this
    float clipFps = 30.0f;
    // Speed bands for the pick request (driver fills them from g_speedMap[stand]).
    float standMax = 0.6f;         // m/s
    float walkRunMid = 2.75f;      // m/s, mid(walk, run)
    float runSprintMid = 5.0f;     // m/s, mid(run, sprint)
    float maxDt = 0.25f;           // s, larger steps are clamped
    // BATCH 6 AIR stage (2026-10-14; both default false here = the previous machine bit for bit; the driver turns them on via
    // UTPPSetMMAirRobust, CET mm.airRobust). physFall: a fall that is not a jump (walk / run off a car roof, PSM Fall detail 14,
    // vehicle exit launch) starts an episode from physics alone - not grounded and (vz < -physFallVz or vz > +physFallVz or probe
    // distance > physFallDist) for physFallTicks feeds, any PSM detail except the never-trigger ones (slide / dodge / climb / vault /
    // release); an external release lockout (vehicle, gate) no longer blocks it once the release reason is gone. Short drops go
    // straight to LandApproach (probe ttl <= landLead), no fallMinTtl. touchGuard: a touchdown needs onGround for touchFrames feeds
    // consecutive grounded feeds, and never while vz > touchRiseVz or within touchRiseTime of a take-off with z rising; LandApproach needs a
    // probe anchor on a non-jump episode and falls back to FallLoop after predicted touch + landAbort; Recover ignores a non-jump
    // take-off trigger for retakeBlock s.
    bool physFall = false;
    float physFallVz = 1.5f;       // m/s
    float physFallDist = 0.5f;     // m probe distance
    int physFallTicks = 2;
    bool touchGuard = false;
    int touchFrames = 2;
    float touchVzMax = 0.5f;       // m/s
    float touchRiseVz = 1.0f;      // m/s
    float touchRiseTime = 0.25f;   // s after take-off
    float landAbort = 0.15f;       // s past the predicted touch
    float retakeBlock = 0.15f;     // s
    // fix/jumpout-cam (game log 2026-11-03 17:34:36, car jump-out B): a contact that is gone again within falseTouchWindow s of the touchdown while
    // the capsule still falls faster than falseTouchVz m/s (the capsule grazed the exiting car / a floor probe flicker at 0.23 m, vz -11 m/s) was no
    // landing: Recover handed back to the ground (HandBack, "walked off right after landing"), the next feed started a second episode and the land
    // clip played twice. touchGuard only: the episode goes back to the descent (FallLoop, land re-armed by the probe) and the touchdown is not counted.
    // falseTouchWindow 0 = off (the previous hand-back). Only for an episode of a vehicle exit (AirInput::vehicleExit true on a feed of the episode,
    // latched to its end: the exit window may close before the landing): an ordinary jump / fall - a downhill ramp or stairs at vz -3.5..-7 m/s with one
    // onGround flicker after the touchdown - keeps the hand-back.
    float falseTouchWindow = 0.25f; // s
    float falseTouchVz = 3.0f;      // m/s downward
    // CAR-2 (vehicle batch): planar speed cap while AirInput::vehicleExit (motion sprint cap; 0 = no clamp)
    float vehicleExitMaxSpeed = 7.0f; // m/s
    // stepDownS: a fall out of the Ground phase (ledge fall or physics fall) whose only evidence is a falling capsule needs this much
    // continuous air time (and at least minAir): a step-down off a curb or a ledge edge the capsule clips (0.05-0.10 s, 3-10 cm) keeps the
    // ground clips. Not waited for: a PSM air detail, a rising capsule (vz > physFallVz), a probe farther than physFallDist, a vehicle exit.
    // 0 = off (minAir alone, the previous machine).
    float stepDownS = 0.0f;
    // stepDownDepthM / stepDownMaxS: step-down reach. Past stepDownS, a capsule whose floor is known from a probe hit (dead-reckoned) and whose
    // whole drop (fallen so far + floor distance) is at most stepDownDepthM still runs down a step: no fall until stepDownMaxS of air time.
    // The other fall evidence (PSM air detail, far probe, vehicle exit) is not delayed. stepDownDepthM 0 = off.
    float stepDownDepthM = 0.0f;
    float stepDownMaxS = 0.0f;
};

struct AirInput
{
    float dt = 0.0f;
    bool onGround = true;
    bool groundStale = false;      // air feed not stamped this frame -> infer onGround from psmDetail
    float distToGround = -2.0f;    // -1 no hit, -2 not probed
    int psmFallPrev = 0;           // previous-frame PSM.Fall (see header comment)
    int psmDetail = 0;
    float vz = 0.0f;               // m/s, up positive
    float planarSpeed = 0.0f;      // m/s capsule
    float velBodyX = 0.0f, velBodyY = 0.0f; // capsule velocity in the body frame (x right, y forward)
    bool strafeActive = false;
    float stickMagnitude = 0.0f;   // 0..1
    float stickTurnDeg = 0.0f;     // AIR CLIPS stage: stick target vs body yaw (deg, 0 = none / no target)
    bool release = false;          // any external release reason (weight 0, TPP off, vehicle bit, ...)
    bool tableValid = false;       // AirTable valid for the loaded DB
    int currentFrame = -1;         // matcher global frame this feed (for the last clean ground frame)
    // Feedback from AirTable for the clip currently forced (<= 0 = unknown):
    float clipRemainSeconds = 0.0f; // at rate 1: TakeOff -> to apex, Descend -> to the root_z zero crossing / end
    float clipRootSpeed = 0.0f;     // Recover: land clip planar root speed
    bool vehicleExit = false;       // CAR-2: inside the vehicle exit window (planar speed clamped to vehicleExitMaxSpeed)
};

struct PickRequest
{
    PickKind kind = PickKind::None;
    Dir dir = Dir::F;
    Band band = Band::Stand;
    Weight weight = Weight::Light;
    Foot foot = Foot::Any;
    Entry entry = Entry::None;
    int entryFrames = 0;           // subtracted from the entry event frame (LiftoffMinus / TouchMinus)
};

struct AirDecision
{
    Action action = Action::None;
    PickRequest pick{};
    float halflife = -1.0f;        // inertialization halflife for the forced transition (-1 = default)
    float rate = 1.0f;             // Clips: playback rate of the forced clip; Hold: multiplier on the stride rate
    bool rateIsMultiplier = false;
    bool suppressSearch = false;
    int queryBoneFrame = -1;       // >= 0: next search takes bone features from this (last clean ground) frame
    bool dwellExempt = false;      // HandBack: first search may switch immediately
    float contactScale = 1.0f;     // FootLocker contacts x this (0 airborne, ramp after touchdown)
    bool releaseOffsetRoot = false;
    bool airOwns = false;          // AirOwns > Strafe > ground
    Phase phase = Phase::Ground;
    Mode effectiveMode = Mode::Off;
    float ttl = -1.0f;             // s, -1 when not airborne
    bool ttlKnown = false;
};

struct AirStats
{
    uint32_t episodes = 0, takeoffs = 0, falls = 0, lands = 0, handbacks = 0, resets = 0;
    uint32_t probesAccepted = 0, probesRejected = 0;
    uint32_t physFalls = 0, touchRejects = 0, landAborts = 0, retakeBlocks = 0, falseTouches = 0; // BATCH 6 AIR stage; falseTouches: fix/jumpout-cam
    float lastTtlErr = 0.0f;       // s, actual touchdown - predicted at LandApproach entry (+ = late landing)
    bool lastTtlErrValid = false;
};

class AirState
{
public:
    explicit AirState(const AirConfig& c = {}) noexcept : cfg_(c) { ResetGravity(); }

    void SetConfig(const AirConfig& c) noexcept { cfg_ = c; }
    const AirConfig& Config() const noexcept { return cfg_; }
    void SetMode(Mode m) noexcept { mode_ = m; }
    // External reset (DB swap, mode change by the driver): emitted as Action::Reset on the next Update.
    void RequestReset() noexcept { resetPending_ = true; }
    void ResetGravity() noexcept { gUp_ = cfg_.gUpDefault; gDown_ = cfg_.gDownDefault; }

    Phase CurrentPhase() const noexcept { return phase_; }
    const AirStats& Stats() const noexcept { return stats_; }
    float GUp() const noexcept { return gUp_; }
    float GDown() const noexcept { return gDown_; }
    bool Heavy() const noexcept { return heavy_; }
    float HeightEstimate() const noexcept { return hValid_ ? h_ : -1.0f; }

    // Pure TTL helper (h >= 0 above the landing plane, vz up positive, g magnitudes > 0).
    static float TimeToLand(float h, float vz, float gUp, float gDown) noexcept
    {
        h = h > 0.0f ? h : 0.0f;
        if (vz > 0.0f)
        {
            const float hApex = h + vz * vz / (2.0f * gUp);
            return vz / gUp + std::sqrt(2.0f * hApex / gDown);
        }
        return (vz + std::sqrt(vz * vz + 2.0f * gDown * h)) / gDown;
    }

    AirDecision Update(const AirInput& raw) noexcept
    {
        AirInput in = Sanitize(raw);
        if (phase_ != Phase::Ground && in.vehicleExit) exitEpisode_ = true; // the episode of a vehicle exit (latched to its end; BeginEpisode seeds it)
        AirDecision d{};
        const Mode eff = mode_ == Mode::Clips && !in.tableValid ? Mode::Hold : mode_;
        d.effectiveMode = eff;

        // A fresh physics contact wins over lagging PSM detail. With robust touchdown validation enabled,
        // detail 23 alone cannot manufacture a contact when the fresh sensor still says airborne.
        const bool staleLandingUnreliable = cfg_.touchGuard && phase_ != Phase::Ground && in.groundStale &&
                                            IsLandDetail(in.psmDetail) && in.vz < -cfg_.touchVzMax;
        const bool grounded = in.groundStale ? (!IsAirDetail(in.psmDetail) && !staleLandingUnreliable)
                                             : (in.onGround || (!cfg_.touchGuard && IsLandDetail(in.psmDetail)));
        const bool jumpEdge = IsJumpDetail(in.psmDetail) && !IsJumpDetail(prevDetail_);
        const bool climbEdge = IsClimbDetail(in.psmDetail) && !IsClimbDetail(prevDetail_);
        const bool release = in.release || IsReleaseDetail(in.psmDetail);
        prevDetail_ = in.psmDetail;
        // Critique MUST: directional picks only when strafe was active in the feed BEFORE take-off / fall entry.
        strafeBefore_ = prevStrafe_;
        prevStrafe_ = in.strafeActive;

        // Mode switch mid-episode, external reset, release or climb/vault entry -> Reset.
        const bool modeChanged = eff != lastEff_;
        lastEff_ = eff;
        if (resetPending_ || (modeChanged && phase_ != Phase::Ground) || (phase_ != Phase::Ground && (release || climbEdge)))
        {
            resetPending_ = false;
            const bool wasEpisode = phase_ != Phase::Ground;
            const bool dirty = dirty_;
            EndEpisode();
            lockout_ = !grounded; // no new episode until grounded again
            if (wasEpisode || dirty)
            {
                d.action = Action::Reset;
                d.queryBoneFrame = dirty ? lastCleanFrame_ : -1;
                ++stats_.resets;
            }
            airTime_ = grounded ? 0.0f : airTime_ + in.dt;
            airFeeds_ = grounded ? 0 : airFeeds_ + 1;
            Finish(d, eff, in);
            return d;
        }
        if (eff == Mode::Off)
        {
            airTime_ = 0.0f;
            airFeeds_ = 0;
            lockout_ = false;
            armTime_ = -1.0f;
            Finish(d, eff, in);
            return d;
        }
        if (grounded) lockout_ = lockoutHard_ = false;
        groundRun_ = grounded ? groundRun_ + 1 : 0;
        // Release / slide / dodge / climb / vault while not grounded: no episode may start until grounded again
        // (a vault or dodge-air must not turn into a take-off when its detail ends mid-air).
        if (!grounded && (release || IsNoTriggerDetail(in.psmDetail))) lockout_ = true;
        if (!grounded && IsNoTriggerDetail(in.psmDetail)) lockoutHard_ = true;

        // Keep the previous sampled velocity for validating a genuine upward impulse.
        const float airVzBeforeFeed = vzPrev_;
        // Time bookkeeping and gravity estimate.
        const bool wasAirborne = airTime_ > 0.0f;
        airTime_ = grounded ? 0.0f : airTime_ + in.dt;
        airFeeds_ = grounded ? 0 : airFeeds_ + 1;
        airDropM_ = grounded ? 0.0f : airDropM_ + (in.vz < 0.0f ? -in.vz * in.dt : 0.0f);
        if (phase_ != Phase::Ground) episodeTime_ += in.dt;
        if (!grounded && wasAirborne && phase_ != Phase::Ground && in.dt > 1e-4f) LearnGravity(in.vz, in.dt);
        // Dead reckoning (trapezoid) and probe fusion.
        if (hValid_) h_ += 0.5f * (vzPrev_ + in.vz) * in.dt;
        vzPrev_ = in.vz;
        bool probeMiss = false;
        if (!grounded) FuseProbe(in.distToGround, probeMiss);
        // BATCH 6 AIR stage: physics fall detector (counts on every feed, used only in Ground)
        {
            const bool physNoTrig = release || IsNoTriggerDetail(in.psmDetail) || lockoutHard_;
            const bool sig = in.vz < -cfg_.physFallVz || in.vz > cfg_.physFallVz || in.distToGround > cfg_.physFallDist;
            // A fresh floor report can be a one-frame false positive during flight. Physics still owns this feed
            // while moving upward fast; do not break the consecutive detector or lose the last impact velocity.
            const bool physicsAirborne = !grounded || (cfg_.touchGuard && phase_ != Phase::Ground && in.vz > cfg_.touchRiseVz);
            physRun_ = (physicsAirborne && !physNoTrig && sig) ? physRun_ + 1 : 0;
            if (physicsAirborne && in.vz < 0.0f) lastAirVz_ = in.vz;
        }
        if (jumpDr_ && hValid_ && h_ < -cfg_.flatTolerance && !probeAnchored_) hValid_ = false; // fell below the take-off plane
        const bool ttlKnown = hValid_;
        const float ttl = ttlKnown ? TimeToLand(h_, in.vz, gUp_, gDown_) : cfg_.noHitTtl;
        if (phase_ == Phase::Ground && grounded && in.currentFrame >= 0) lastCleanFrame_ = in.currentFrame;

        // An external release reason (TPP off, no usable pose, PSM release bits, gate closed without hold) blocks any new
        // episode, on the ground too (hold repair: a grounded jump edge under release must not take off for one feed).
        const bool noTrigger = release || IsNoTriggerDetail(in.psmDetail) || lockout_;
        // AIR CLIPS repair: in Clips mode a jump edge on a fresh grounded feed that is not rising (knockdown / car hit reported as
        // detail 18) only arms the take-off: no episode, no Start clip. The take-off follows when the capsule leaves the ground
        // within abortTime (a jump whose first feed is still grounded starts one feed later); otherwise nothing happens.
        bool edgeTrig = jumpEdge;
        if (eff == Mode::Clips && phase_ == Phase::Ground)
        {
            if (armTime_ >= 0.0f) armTime_ += in.dt;
            if (armTime_ > cfg_.abortTime) armTime_ = -1.0f;
            if (jumpEdge && grounded && !in.groundStale && in.vz < cfg_.takeoffVz)
            {
                edgeTrig = false;
                if (!noTrigger) armTime_ = 0.0f;
            }
            else if (armTime_ >= 0.0f && !grounded)
                edgeTrig = true;
        }
        else
            armTime_ = -1.0f;
        const bool takeoffTrig = !noTrigger && (edgeTrig || (!grounded && in.vz >= cfg_.takeoffVz));
        if (takeoffTrig) armTime_ = -1.0f;

        switch (phase_)
        {
        case Phase::Ground:
        {
            if (takeoffTrig)
            {
                StartTakeOff(d, in, eff);
                break;
            }
            // Ledge fall: hDR is not valid (no jump); the probe (if any) has been fused above.
            const float fallTtl = hValid_ ? TimeToLand(h_, in.vz, gUp_, gDown_) : cfg_.noHitTtl;
            // stepDownS: only a capsule that keeps falling past a step-down counts; the other fall evidence starts at once
            const bool fallEvidence = IsAirDetail(in.psmDetail) || in.vehicleExit || in.distToGround > cfg_.physFallDist;
            // step-down reach: a known floor within stepDownDepthM of the drop start keeps the step-down until stepDownMaxS
            // (h_ dead-reckoned below 0 = the floor is reached, the touchdown is a feed late: clamped like the chooser floor, AirFloorStep)
            const bool stepReach = cfg_.stepDownDepthM > 0.0f && hValid_ && probeAnchored_ && airDropM_ + std::fmax(h_, 0.0f) <= cfg_.stepDownDepthM &&
                                   airTime_ < cfg_.stepDownMaxS;
            const bool stepDownOver =
                cfg_.stepDownS <= 0.0f || fallEvidence || (airTime_ >= cfg_.stepDownS && airFeeds_ >= 2 && !stepReach); // one long frame is no fall
            const bool physStepDownOver = stepDownOver || in.vz > cfg_.physFallVz;
            if (!noTrigger && !grounded && airTime_ >= cfg_.minAir && stepDownOver && in.vz <= cfg_.fallVz &&
                (fallTtl >= cfg_.fallMinTtl || probeMiss))
            {
                BeginEpisode(in);
                jumpDr_ = false;
                ++stats_.falls;
                const Band b = BandOf(in.planarSpeed);
                if (b >= Band::Walk)
                {
                    phase_ = Phase::Descend;
                    Play(d, eff, Action::PlayOff, PickKind::Off, Entry::DropFrame, 0, cfg_.fallHalflife, in);
                }
                else
                {
                    phase_ = Phase::FallLoop;
                    Play(d, eff, Action::PlayFallLoop, PickKind::FallLoop, Entry::LoopStart, 0, cfg_.fallHalflife, in);
                }
            }
            else if (cfg_.physFall && physRun_ >= cfg_.physFallTicks && physStepDownOver)
            {
                // BATCH 6 AIR stage: physics fall (no jump, any detail): straight to LandApproach on a short probed drop,
                // FallLoop while rising (vehicle launch) or standing, Descend (Off clip) at walk or faster.
                BeginEpisode(in);
                jumpDr_ = false;
                ++stats_.falls;
                ++stats_.physFalls;
                lockout_ = false;
                if (probeAnchored_ && hValid_ && in.vz <= 0.0f && fallTtl <= cfg_.landLead)
                {
                    UpdateHeavy(in, fallTtl, true);
                    EnterLand(d, in, eff, fallTtl);
                }
                else if (in.vz > 0.0f || BandOf(in.planarSpeed) < Band::Walk)
                {
                    phase_ = Phase::FallLoop;
                    Play(d, eff, Action::PlayFallLoop, PickKind::FallLoop, Entry::LoopStart, 0, cfg_.fallHalflife, in);
                }
                else
                {
                    phase_ = Phase::Descend;
                    Play(d, eff, Action::PlayOff, PickKind::Off, Entry::DropFrame, 0, cfg_.fallHalflife, in);
                }
            }
            else if (grounded)
            {
                hValid_ = false; // not an episode: forget any probe anchor gathered during a blip
                probeAnchored_ = false;
            }
            break;
        }
        case Phase::TakeOff:
        case Phase::Descend:
        case Phase::FallLoop:
        case Phase::LandApproach:
        {
            if (!grounded) seenAir_ = true;
            // PSM jump detail can arrive late during a descent. A mid-air restart needs an
            // observed rising impulse, not a detail transition alone (robust mode only).
            if (jumpEdge && !noTrigger && seenAir_ &&
                (!cfg_.touchGuard || (!grounded && in.vz >= cfg_.takeoffVz && in.vz > airVzBeforeFeed)))
            {
                StartTakeOff(d, in, eff, true); // double jump: same landing plane, keep the height estimate
                break;
            }
            UpdateHeavy(in, ttl, ttlKnown);
            if (grounded && seenAir_)
            {
                if (!cfg_.touchGuard || TouchAccepted(in))
                {
                    Touchdown(d, in, eff);
                    break;
                }
                ++stats_.touchRejects; // BATCH 6 AIR stage: one-frame / rising onGround ignored
            }
            if (!seenAir_ && episodeTime_ > cfg_.abortTime)
            {
                HandBack(d);
                break;
            }
            if (phase_ == Phase::TakeOff)
            {
                // Rise warp: remaining clip rise time / remaining physical rise time.
                if (in.vz > 0.0f && in.clipRemainSeconds > 0.0f)
                    SmoothRate(Clamp(in.clipRemainSeconds / (in.vz / gUp_), cfg_.rateMin, cfg_.rateMax), in.dt);
                if (in.vz <= 0.0f) phase_ = Phase::Descend;
            }
            if (phase_ == Phase::Descend)
            {
                if (in.clipRemainSeconds > 0.0f && ttl > 0.0f)
                    SmoothRate(Clamp(in.clipRemainSeconds / ttl, cfg_.rateMin, cfg_.rateMax), in.dt);
                const bool tooLong = in.clipRemainSeconds > 0.0f ? ttl > in.clipRemainSeconds / cfg_.rateMin
                                                                 : (ttl > cfg_.descendMaxTtl || !ttlKnown);
                if (ttlKnown && ttl <= cfg_.landLead && LandAllowed())
                    EnterLand(d, in, eff, ttl);
                else if (tooLong && in.vz <= 0.0f)
                {
                    phase_ = Phase::FallLoop;
                    rate_ = 1.0f;
                    Play(d, eff, Action::PlayFallLoop, PickKind::FallLoop, Entry::LoopStart, 0, cfg_.fallHalflife, in);
                }
            }
            else if (phase_ == Phase::FallLoop)
            {
                rate_ = 1.0f;
                if (ttlKnown && ttl <= cfg_.landLead && LandAllowed()) EnterLand(d, in, eff, ttl);
            }
            else if (phase_ == Phase::LandApproach)
            {
                rate_ = 1.0f;
                if (cfg_.touchGuard && !TouchAccepted(in) && predictedTouch_ >= 0.0f &&
                    episodeTime_ > predictedTouch_ + cfg_.landAbort)
                {
                    // BATCH 6 AIR stage: still airborne well past the predicted touch -> fall loop until a probe anchors again
                    ++stats_.landAborts;
                    predictedTouch_ = -1.0f;
                    // The original predicted floor has expired. Require a *new* consecutive probe pair before
                    // scheduling another land, even if the earlier probe was internally consistent.
                    probeAnchored_ = false;
                    hValid_ = false;
                    disagree_ = 0;
                    lastRejected_ = -1.0f;
                    landAborted_ = true;
                    phase_ = Phase::FallLoop;
                    Play(d, eff, Action::PlayFallLoop, PickKind::FallLoop, Entry::LoopStart, 0, cfg_.fallHalflife, in);
                }
            }
            break;
        }
        case Phase::Recover:
        {
            recoverTime_ += in.dt;
            if (recoverTime_ <= in.dt + 1e-6f && in.psmFallPrev >= cfg_.heavyFall) heavy_ = true; // one-frame-late PSM.Fall
            // A fresh jump edge by itself is not proof of a new launch during Recover.
            // Allow a real upward impulse to override the brief landing lockout.
            const bool freshLaunch = in.vz >= cfg_.takeoffVz && in.vz > airVzBeforeFeed &&
                                     (!grounded || (!in.groundStale && in.onGround && jumpEdge));
            const bool retakeBlocked = cfg_.touchGuard && takeoffTrig && !freshLaunch && recoverTime_ < cfg_.retakeBlock;
            if (retakeBlocked) ++stats_.retakeBlocks; // BATCH 6 AIR stage: no re-take-off right after a touchdown
            if (takeoffTrig && !retakeBlocked)
            {
                StartTakeOff(d, in, eff);
                break;
            }
#if !(defined(UTPP_VEHEXIT_MUTATION) && UTPP_VEHEXIT_MUTATION == 19) // mutation 19: a short contact in a fast fall ends the episode (the previous hand-back)
#if defined(UTPP_VEHEXIT_MUTATION) && UTPP_VEHEXIT_MUTATION == 20
            const bool exitEpisode = true; // mutation 20: the take-back for every landing (ordinary jumps and falls too)
#elif defined(UTPP_VEHEXIT_MUTATION) && UTPP_VEHEXIT_MUTATION == 21
            const bool exitEpisode = in.vehicleExit; // mutation 21: the flag of this feed only (no latch: the window closes before the landing)
#else
            const bool exitEpisode = exitEpisode_;
#endif
            if (cfg_.touchGuard && exitEpisode && !grounded && cfg_.falseTouchWindow > 0.0f && recoverTime_ <= cfg_.falseTouchWindow && in.vz < -cfg_.falseTouchVz)
            {
                // false touch: still falling fast - back to the descent, the touchdown is taken back
                ++stats_.falseTouches;
                if (stats_.lands > 0) --stats_.lands;
                touchTime_ = -1.0f;
                predictedTouch_ = -1.0f;
                phase_ = Phase::FallLoop;
                rate_ = 1.0f;
                Play(d, eff, Action::PlayFallLoop, PickKind::FallLoop, Entry::LoopStart, 0, cfg_.fallHalflife, in);
                break;
            }
#endif
            if (!grounded && !retakeBlocked)
            {
                HandBack(d); // walked off right after landing: ground tracking takes over (ledge fall rules)
                break;
            }
            if (TurnsOnLanding(in))
            {
                HandBack(d); // AIR CLIPS stage: turning after the landing -> ground MM
                break;
            }
            if (in.clipRootSpeed > 0.0f)
                rate_ = Clamp(in.planarSpeed / in.clipRootSpeed, cfg_.recoverRateMin, cfg_.recoverRateMax);
            float dur = heavy_ ? cfg_.recoverHeavy : cfg_.recoverLight;
            if (in.stickMagnitude >= cfg_.stickWalk && cfg_.recoverStick < dur) dur = cfg_.recoverStick;
            if (recoverTime_ >= dur) HandBack(d);
            break;
        }
        }
        d.ttl = (phase_ != Phase::Ground && phase_ != Phase::Recover) ? ttl : -1.0f;
        d.ttlKnown = ttlKnown && d.ttl >= 0.0f;
        Finish(d, eff, in);
        return d;
    }

private:
    static float Clamp(float v, float lo, float hi) noexcept { return v < lo ? lo : (v > hi ? hi : v); }
    static float Fin(float v, float def) noexcept { return std::isfinite(v) ? v : def; }

    AirInput Sanitize(const AirInput& r) const noexcept
    {
        AirInput in = r;
        in.dt = Fin(in.dt, 0.0f);
        in.dt = Clamp(in.dt, 0.0f, cfg_.maxDt);
        in.vz = Clamp(Fin(in.vz, 0.0f), -100.0f, 100.0f);
        in.distToGround = Fin(in.distToGround, -2.0f);
        in.planarSpeed = Fin(in.planarSpeed, 0.0f);
        in.velBodyX = Fin(in.velBodyX, 0.0f);
        in.velBodyY = Fin(in.velBodyY, 0.0f);
        in.stickMagnitude = Fin(in.stickMagnitude, 0.0f);
        in.stickTurnDeg = Fin(in.stickTurnDeg, 0.0f);
        in.clipRemainSeconds = Fin(in.clipRemainSeconds, 0.0f);
        in.clipRootSpeed = Fin(in.clipRootSpeed, 0.0f);
#if !(defined(UTPP_VEHEXIT_MUTATION) && UTPP_VEHEXIT_MUTATION == 5) // mutation 5: no vehicle-exit speed clamp
        // CAR-2 (vehicle batch): after a jump-out the capsule still carries the vehicle's speed (5-16 m/s in the 2026-10-22
        // recording); the air episode is seeded with at most the motion sprint cap so the landing does not play at the car's speed
        if (in.vehicleExit && cfg_.vehicleExitMaxSpeed > 0.0f && in.planarSpeed > cfg_.vehicleExitMaxSpeed)
        {
            const float k = cfg_.vehicleExitMaxSpeed / in.planarSpeed;
            in.planarSpeed = cfg_.vehicleExitMaxSpeed;
            in.velBodyX *= k;
            in.velBodyY *= k;
        }
#endif
        return in;
    }

    // BATCH 6 AIR stage (touchGuard)
    bool TouchAccepted(const AirInput& in) const noexcept
    {
        if (in.vz > cfg_.touchRiseVz) return false;
        if (jumpDr_ && in.vz > 0.0f && episodeTime_ < cfg_.touchRiseTime) return false;
        // Stale and fresh contact indications both need consecutive accepted grounded feeds.
        // Rising velocity rejects a contact, even after repeated stale land details.
        if (groundRun_ >= cfg_.touchFrames) return true;
        // Zero-velocity alone may be a one-frame collision glitch. Require a stable contact pair.
        return false;
    }
    bool LandAllowed() const noexcept
    {
        if (!cfg_.touchGuard) return true;
        if (landAborted_ || !jumpDr_) return probeAnchored_; // no LandApproach on an unknown floor
        return true;
    }

    bool TurnsOnLanding(const AirInput& in) const noexcept
    {
        return in.stickMagnitude >= cfg_.stickWalk && std::fabs(in.stickTurnDeg) > cfg_.recoverTurnDeg;
    }

    Band BandOf(float speed) const noexcept
    {
        if (speed < cfg_.standMax) return Band::Stand;
        if (speed < cfg_.walkRunMid) return Band::Walk;
        if (speed < cfg_.runSprintMid) return Band::Run;
        return Band::Sprint;
    }

    // Directional picks only when strafe was active at take-off (critique MUST); otherwise F.
    Dir DirOf(const AirInput& in) const noexcept
    {
        if (!strafeLatched_ || in.planarSpeed < cfg_.standMax) return Dir::F;
        const float a = std::atan2(in.velBodyX, in.velBodyY) * 57.29578f; // 0 = forward, + = right
        const float aa = std::fabs(a);
        if (aa <= 60.0f) return Dir::F;
        if (aa >= 120.0f) return Dir::B;
        return a > 0.0f ? Dir::R : Dir::L;
    }

    void LearnGravity(float vz, float dt) noexcept
    {
        const float a = (vz - vzPrev_) / dt; // vzPrev_ still holds the previous sample here
        if (vz > 0.0f && vzPrev_ > 0.0f && -a > 4.0f && -a < 40.0f)
            gUp_ = Clamp(gUp_ + cfg_.gEmaAlpha * (-a - gUp_), cfg_.gMin, cfg_.gMax);
        else if (vz < 0.0f && vzPrev_ < 0.0f && -a > 4.0f && -a < 40.0f)
            gDown_ = Clamp(gDown_ + cfg_.gEmaAlpha * (-a - gDown_), cfg_.gMin, cfg_.gMax);
    }

    void FuseProbe(float dist, bool& miss) noexcept
    {
        if (dist <= -1.5f) return; // -2: not probed
        if (dist < 0.0f)
        {
            miss = true;
            return;
        }
        if (!hValid_)
        {
            h_ = dist;
            hValid_ = true;
            probeAnchored_ = true;
            disagree_ = 0;
            ++stats_.probesAccepted;
            return;
        }
        if (std::fabs(dist - h_) <= cfg_.probeTolerance)
        {
            h_ = dist;
            probeAnchored_ = true;
            disagree_ = 0;
            lastRejected_ = -1.0f;
            ++stats_.probesAccepted;
            return;
        }
        // Disagreement: ignore, unless two consecutive rejected probes agree with each other (new floor).
        if (disagree_ > 0 && lastRejected_ >= 0.0f && std::fabs(dist - lastRejected_) <= cfg_.probeTolerance)
        {
            h_ = dist;
            probeAnchored_ = true;
            disagree_ = 0;
            lastRejected_ = -1.0f;
            ++stats_.probesAccepted;
            return;
        }
        ++disagree_;
        lastRejected_ = dist;
        ++stats_.probesRejected;
    }

    void BeginEpisode(const AirInput& in) noexcept
    {
        ++stats_.episodes;
        episodeTime_ = 0.0f;
        seenAir_ = !(in.onGround && !in.groundStale);
        heavy_ = false;
        predictedTouch_ = -1.0f;
        landAborted_ = false;
        recoverTime_ = 0.0f;
        rate_ = 1.0f;
        exitEpisode_ = in.vehicleExit;
        strafeLatched_ = strafeBefore_; // previous feed's StrafeMode, not the take-off feed's
    }

    void StartTakeOff(AirDecision& d, const AirInput& in, Mode eff, bool midAir = false) noexcept
    {
        const bool keepH = midAir && hValid_;
        const bool wasAnchored = probeAnchored_;
        const bool exitKeep = exitEpisode_;
        BeginEpisode(in);
        if (midAir)
        {
            --stats_.episodes; // same episode
            exitEpisode_ = exitKeep || in.vehicleExit;
        }
        seenAir_ = midAir;
        ++stats_.takeoffs;
        phase_ = Phase::TakeOff;
        jumpDr_ = true;
        h_ = keepH ? h_ : 0.0f; // take-off plane
        hValid_ = true;
        probeAnchored_ = keepH && wasAnchored;
        disagree_ = 0;
        vzPrev_ = in.vz;
        Play(d, eff, Action::PlayTakeOff, PickKind::Start, Entry::LiftoffMinus, cfg_.takeoffLeadFrames,
             cfg_.takeoffHalflife, in);
    }

    void UpdateHeavy(const AirInput& in, float ttl, bool ttlKnown) noexcept
    {
        if (in.psmFallPrev >= cfg_.heavyFall) heavy_ = true;
        if (!ttlKnown) return;
        const float hApex = in.vz > 0.0f ? h_ + in.vz * in.vz / (2.0f * gUp_) : -1.0f;
        const float impact = in.vz > 0.0f ? std::sqrt(2.0f * gDown_ * (hApex > 0.0f ? hApex : 0.0f))
                                          : std::fabs(in.vz) + gDown_ * ttl;
        if (impact >= cfg_.heavyVz) heavy_ = true;
    }

    void EnterLand(AirDecision& d, const AirInput& in, Mode eff, float ttl) noexcept
    {
        phase_ = Phase::LandApproach;
        rate_ = 1.0f;
        predictedTouch_ = episodeTime_ + ttl;
        const int lead = static_cast<int>(std::lround(ttl * cfg_.clipFps));
        Play(d, eff, Action::PlayLand, PickKind::Land, Entry::TouchMinus, lead > 0 ? lead : 0, cfg_.landHalflife, in);
    }

    void Touchdown(AirDecision& d, const AirInput& in, Mode eff) noexcept
    {
        ++stats_.lands;
        if (predictedTouch_ >= 0.0f)
        {
            stats_.lastTtlErr = episodeTime_ - predictedTouch_;
            stats_.lastTtlErrValid = true;
        }
        if (in.psmFallPrev >= cfg_.heavyFall) heavy_ = true;
        if (cfg_.physFall && -lastAirVz_ >= cfg_.heavyVz) heavy_ = true; // BATCH 6 AIR stage: measured impact speed
        touchTime_ = 0.0f;
        if (eff == Mode::Hold || TurnsOnLanding(in)) // AIR CLIPS stage: a turn on landing skips Recover (ground MM turns)
        {
            HandBack(d);
            return;
        }
        const bool landPlaying = phase_ == Phase::LandApproach;
        phase_ = Phase::Recover;
        recoverTime_ = 0.0f;
        rate_ = 1.0f;
        if (!landPlaying) // late land: ForcePlay at the touch frame
            Play(d, eff, Action::PlayLand, PickKind::Land, Entry::TouchMinus, 0, cfg_.landHalflife, in);
    }

    void HandBack(AirDecision& d) noexcept
    {
        const bool wasRecover = phase_ == Phase::Recover || touchTime_ >= 0.0f;
        EndEpisode();
        if (wasRecover) touchTime_ = touchTimeKeep_; // keep the contact ramp running after hand-back
        d.action = Action::HandBack;
        d.dwellExempt = true;
        ++stats_.handbacks;
    }

    void EndEpisode() noexcept
    {
        touchTimeKeep_ = touchTime_;
        phase_ = Phase::Ground;
        dirty_ = false;
        hValid_ = false;
        probeAnchored_ = false;
        jumpDr_ = false;
        disagree_ = 0;
        lastRejected_ = -1.0f;
        rate_ = 1.0f;
        episodeTime_ = 0.0f;
        recoverTime_ = 0.0f;
        touchTime_ = -1.0f;
        predictedTouch_ = -1.0f;
        landAborted_ = false;
        physRun_ = 0;
        groundRun_ = 0;
        lastAirVz_ = 0.0f;
        exitEpisode_ = false;
    }

    void Play(AirDecision& d, Mode eff, Action a, PickKind k, Entry e, int frames, float hl, const AirInput& in) noexcept
    {
        if (eff != Mode::Clips) return; // Hold: no clip requests
        d.action = a;
        d.pick.kind = k;
        d.pick.dir = (k == PickKind::Start || k == PickKind::Land) ? DirOf(in) : Dir::F;
        d.pick.band = BandOf(in.planarSpeed);
        d.pick.weight = heavy_ ? Weight::Heavy : Weight::Light;
        d.pick.foot = Foot::Any;
        d.pick.entry = e;
        d.pick.entryFrames = frames;
        d.halflife = hl;
        dirty_ = true;
    }

    void SmoothRate(float target, float dt) noexcept
    {
        const float k = cfg_.rateHalflife > 1e-4f ? 1.0f - std::exp2(-dt / cfg_.rateHalflife) : 1.0f;
        rate_ += (target - rate_) * k;
    }

    void Finish(AirDecision& d, Mode eff, const AirInput& in) noexcept
    {
        d.phase = phase_;
        d.effectiveMode = eff;
        const bool airborne = phase_ == Phase::TakeOff || phase_ == Phase::Descend || phase_ == Phase::FallLoop ||
                              phase_ == Phase::LandApproach;
        d.airOwns = eff != Mode::Off && phase_ != Phase::Ground;
        d.suppressSearch = d.airOwns;
        d.releaseOffsetRoot = d.airOwns && airborne;
        if (eff == Mode::Hold)
        {
            d.rate = airborne ? cfg_.holdRate : 1.0f;
            d.rateIsMultiplier = true;
        }
        else
        {
            d.rate = d.airOwns ? rate_ : 1.0f;
        }
        // Contact scale: 0 while airborne (episode), ramp after touchdown (also across the hand-back).
        if (eff == Mode::Off)
            d.contactScale = 1.0f;
        else if (airborne)
            d.contactScale = 0.0f;
        else if (touchTime_ >= 0.0f)
        {
            // The touchdown feed is the start of contact recovery (scale 0), not one whole
            // frame into it. Preserve the old sample/advance order when robust mode is OFF.
            const float elapsed = cfg_.touchGuard ? touchTime_ : touchTime_ + in.dt;
            const float s = cfg_.contactRamp > 1e-4f ? elapsed / cfg_.contactRamp : 1.0f;
            d.contactScale = s >= 1.0f ? 1.0f : s;
            touchTime_ += in.dt;
            if (s >= 1.0f && phase_ == Phase::Ground) touchTime_ = -1.0f;
        }
        else
            d.contactScale = 1.0f;
    }

    AirConfig cfg_;
    Mode mode_ = Mode::Off;
    Mode lastEff_ = Mode::Off;
    Phase phase_ = Phase::Ground;
    AirStats stats_{};
    float gUp_ = 19.2f, gDown_ = 16.0f;
    float airTime_ = 0.0f, episodeTime_ = 0.0f, recoverTime_ = 0.0f;
    float touchTime_ = -1.0f, touchTimeKeep_ = -1.0f; // time since touchdown (contact ramp), -1 = none
    float armTime_ = -1.0f;           // AIR CLIPS repair: s since a grounded, non-rising jump edge (Clips), -1 = not armed
    float h_ = 0.0f;                  // m above the landing plane (see header)
    float vzPrev_ = 0.0f;
    float rate_ = 1.0f;
    float predictedTouch_ = -1.0f;
    float lastRejected_ = -1.0f;
    int disagree_ = 0;
    int prevDetail_ = 0;
    int lastCleanFrame_ = -1;
    bool hValid_ = false, probeAnchored_ = false, jumpDr_ = false;
    bool seenAir_ = false, heavy_ = false, strafeLatched_ = false, prevStrafe_ = false, strafeBefore_ = false;
    bool dirty_ = false, lockout_ = false, resetPending_ = false;
    bool lockoutHard_ = false, landAborted_ = false; // BATCH 6 AIR stage
    int physRun_ = 0, groundRun_ = 0;
    int airFeeds_ = 0; // consecutive not-grounded feeds (AirConfig::stepDownS)
    float airDropM_ = 0.0f; // capsule drop over those feeds (m, AirConfig::stepDownDepthM)
    float lastAirVz_ = 0.0f;
    bool exitEpisode_ = false; // a feed of this episode carried AirInput::vehicleExit (false-touch take-back only)
};
} // namespace utpp::mm::air
