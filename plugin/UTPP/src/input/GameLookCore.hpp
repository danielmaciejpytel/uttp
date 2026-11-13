// UTPP 0.6.5 game look write: pure math / controller (no game, no Windows API; offline tests in tests/GameLookTest.cpp).
//
// The FPP look of 2.31 is not stored in gameFPPCameraComponent: each tick the component feeds yaw / pitch DELTAS (degrees)
// into its AnimFeature_FPPCamera (deltaYaw / deltaPitch) and the animation graph integrates them; the graph outputs
// "YawInput" / "PitchInput" (degrees, relative to the entity) are read back into the component (+0x42C / +0x430) at the
// start of the next update. RotateEntityToCamera (FUN_14080404c) turns V by entityYaw + YawInput, so
//   world look yaw = entity yaw + YawInput        (yaw CCW positive = CET CM.yawPitchFromForward = atan2(-x, y))
//   world look pitch = PitchInput                 (up positive: ForceDownwardPitch sets pitchMin = pitchMax = -70)
// The controller turns a wanted world yaw / pitch into the delta that the game then applies (closed loop: the next tick
// reads back what the graph made of it). Evidence and offsets: GameLook.cpp.
//
// 0.6.5b controller (2026-10-13; the 0.6.5 error-only loop with a lag estimate went unstable whenever the estimate was
// wrong: estimate 1 on a lag-0 graph = poles z^2 + z - 1 (z = -1.62), estimate 0 on a lag-1 graph = z^2 - z + 1
// (|z| = 1, undamped) - the error grew with the input speed; in game the estimate flipped 0 -> 1 -> 0 during body turns
// and the divergence guard tripped). Now FEED-FORWARD + low-gain feedback, stable for a graph delay of 0 or 1 tick
// without estimating it:
//   r_n   = wrap(targetYaw - (entityYaw + aheadEff))   wanted YawInput this tick (aheadEff: the entity turn requested by
//           utpp_bodyturn adjustTransform that lands between this read and the next, see "ahead order"); pitch: clamp
//   c_n   = c_{n-1} + clamp(r_n - c_{n-1}, +-kMaxStep) commanded reference (kMaxStep 180: c = r)
//   u_n   = [(c_n - c_{n-1}) + g * wrap(c_{n-1} - Y_n)] / m   feed-forward of the wanted motion + feedback on the
//           read-back Y_n (g = k normally; m = the game's yawMultiplier / pitchMultiplier of the last apply when
//           0.2 < m < 1, else 1)
//   first write after (re)activation: c_{-1} = r_0 + aheadEff (no feed-forward of the target, feedback only, smooth
//   activation; the turn announced for the tick is compensated); |u_n| <= kMaxStep
// The read-back Y_n is the graph output of the PREVIOUS tick, so it is compared with c_{n-1} (the reference of that
// tick), not with c_n: with c_n the loop settles one tick AHEAD of the target (L = 0: steady error -1 tick of motion and
// an overshoot of one tick of motion after every stop). Error e_n = c_{n-1} - Y_n (graph Y_{n+1} = Y_n + u_{n-L}):
//   delay 0: e_{n+1} = (1 - k) e_n                       pole z = 1 - k = 0.75; target motion causes NO error
//   delay 1: e_{n+1} = e_n - k e_{n-1} + (ff_n - ff_{n-1})  poles z^2 - z + k = 0 -> z = 0.5 (double, critically damped
//            at k = 0.25; k = 0.3 would give 0.5 +- 0.22i, |z| 0.55); target motion: transient of one tick of motion at
//            each start / stop, 0 in steady motion
// A delay switching between 0 and 1 (queue) is a switch between those two stable loops (offline test: bounded, no trip).
//
// 0.6.5b round 2 (2026-10-13 review of the in-game logs 01:05 / 01:25):
// * Override ticks. While utpp_bodyturn turns V (Locomotion apply, aim align applyAim: an adjustTransform request every
//   frame) the game overrides the FPP look: the camera trace showed the look pulled onto V's yaw and pitch ~0 (tau
//   ~0.09 s, the delta ignored) and PitchInput read back = the target while the shown pitch was 0. A tick whose read-back
//   may carry that override - an entity turn was announced (ahead != 0) in this tick or one of the last two, or less than
//   kOvrHold s ago - is not trusted: the feedback uses the model prediction (last estimate + what the graph integrates
//   from the last write; NoteApplied = the feature after the game's apply) and nothing counts for the divergence guard or
//   the lag hint. The first clean tick after an override is a RELEASE: the whole remaining error is corrected at once
//   (g = 1) and the feedback skips the next tick (g = 0; exact for a graph delay of 0 and of 1). Without this the loop
//   wound the game's look up against the override (k = 0.25 per tick on a wrong read-back) and the guard tripped in both
//   sessions during Locomotion turns.
// * Ahead order. The order of V's turn and the FPP camera update is not known for sure, so it is measured from the
//   entity yaw: the turn between two reads is compared with the ahead of the last tick (after: the turn lands after the
//   camera update, assumed), of this tick (before: it landed before this read) and of two ticks ago (late). Only samples
//   whose hypotheses differ by >= kAheadDiscMin count; once kAheadModeMinN samples exist, the order whose decayed
//   residual is below half of the current one is used (aheadEff: after = this tick's ahead, before = 0, late = the last
//   tick's ahead). Status: aheadAfter / aheadBefore / aheadLate means and aheadMode.
// Divergence guard: only error that motion cannot explain trips. While |e| > kDivergeDeg and the target moves
// < kDivergeSlowRate deg/s (round 2: per second, not per tick), and not in or within kOvrGuardHold s after an override,
// windows of kDivergeWindow s are compared: (a) |e| grew by > kDivergeGrow in kDivergeWindows consecutive windows (a
// graph that ignores the delta while the target moves), or (b) the read-back moved > kDivergeChurn deg beyond the target
// motion in a window while |e| did not halve (positive feedback / wrong sign: the look spins, the error stays). Flicks
// (fast target) restart the count; kDivergeCalm s below kDivergeDeg end it. A large error that rises once and then stays
// (a clamp, the game pulling the look) does not trip. A trip turns the write off; the native re-arms it after
// kTripCooldown s, up to kMaxTrips trips per game session (round 2: was off for the session after the first trip).
#pragma once

#include <cmath>
#include <cstdint>

namespace utpp::gamelook
{
// 2.31 code sites checked before the hook is attached (also offline against the 2.31 exe by utpp_gamelook_test)
inline constexpr uint32_t kRvaApply = 0x39CD5C;     // FUN_1404e0d88: (comp, dt, yawDelta, pitchDelta, extYaw, extPitch)
inline constexpr uint32_t kRvaCallSite = 0x39CBD7;  // in FUN_1404e1638: MOV RAX,[RDI+0x360]; MOVSS [RAX+0x7C],XMM6; CALL apply
inline constexpr uint32_t kRvaRefresh = 0x39CAD2;   // in FUN_1404e1638: LEA R8,[RCX+0x430]; LEA RDX,[RCX+0x42C]; CALL 0x5FD7CC
inline constexpr uint32_t kRvaReadVars = 0x5FD7CC;  // FUN_1404e2968: graph variable source at +0x380
inline constexpr uint32_t kRvaEntityRot = 0x7C0C28; // FUN_140520f84: entity world orientation = *(entity+0xB0)+0xF0
inline constexpr uint8_t kApplyPrologue[] = {0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x10, 0x48, 0x89, 0x78, 0x18, 0x55, 0x48,
                                      0x8D, 0x68, 0xB1, 0x48, 0x81, 0xEC, 0xF0, 0x00, 0x00, 0x00, 0x0F, 0x29, 0x70,
                                      0xE8, 0x48, 0x8B, 0xD9, 0x0F, 0x29, 0x78, 0xD8, 0x0F, 0x28, 0xF2, 0x44, 0x0F,
                                      0x29, 0x40, 0xC8, 0x0F, 0x57, 0xFF, 0x0F, 0x2F, 0xB9, 0xB8, 0x04, 0x00, 0x00};
inline constexpr uint8_t kCallSite[] = {0x48, 0x8B, 0x87, 0x60, 0x03, 0x00, 0x00, 0xF3, 0x0F,
                                 0x11, 0x70, 0x7C, 0xE8, 0x74, 0x01, 0x00, 0x00};
inline constexpr uint8_t kRefreshSite[] = {0x4C, 0x8D, 0x81, 0x30, 0x04, 0x00, 0x00, 0x48, 0x8D, 0x91,
                                    0x2C, 0x04, 0x00, 0x00, 0xE8, 0xE7, 0x0C, 0x26, 0x00};
inline constexpr uint8_t kReadVarsPrologue[] = {0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x10, 0x48, 0x89, 0x68, 0x18, 0x48, 0x89, 0x70,
                                         0x20, 0x57, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B, 0x99, 0x80, 0x03, 0x00, 0x00};
inline constexpr uint8_t kEntityRotHead[] = {0x4C, 0x8B, 0x81, 0xB0, 0x00, 0x00, 0x00, 0x33, 0xC0, 0x4D, 0x85, 0xC0, 0x0F, 0x84};
inline constexpr uint8_t kEntityRotRead[] = {0x41, 0x8B, 0x80, 0xF0, 0x00, 0x00, 0x00}; // at +0x12

// 2026-10-28 round 3 (2.50 static analysis, capstone; same code as 2.31 FUN_14080404c / FUN_1404e32c8):
// RotateEntityToCamera (unlock) @0x7E60E0, its only caller @0xB45269. At +0x48 it calls the graph look read @0x5FD84C
// (comp, out): out+0 PitchInput, +4 PitchRef, +8 YawInput, +0xC YawRef (999 when missing); +0x7E it takes YawInput
// (movss xmm6,[rbp-0x11] = out+8) as the entity turn (target yaw = entity yaw + YawInput) and later stores it as the
// unlock blend start (+0x470) and sets the unlock blend +0x4B8 = 0.3 (@0x7E6315).
inline constexpr uint32_t kRvaRotate = 0x7E60E0;
inline constexpr uint8_t kRotateHead[] = {0x48, 0x8B, 0xC4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x56, 0x48, 0x8D, 0x68,
                                          0xA1, 0x48, 0x81, 0xEC, 0xC0, 0x00, 0x00, 0x00, 0x48, 0x8B, 0x51, 0x50,
                                          0x4C, 0x8B, 0xF1, 0x48, 0x8D, 0x4D, 0x07, 0x0F, 0x29, 0x70, 0xC8};
// lea rdx,[rbp-0x19] (out); mov rcx,r14 (comp); movss [rbp-0x11],xmm0 (out+8 = 999); call rel32 -> kRvaLookVars
inline constexpr uint32_t kRvaRotateReadSite = 0x7E6130;
inline constexpr uint8_t kRotateReadSite[] = {0x48, 0x8D, 0x55, 0xE7, 0x49, 0x8B, 0xCE, 0xF3, 0x0F, 0x11, 0x45, 0xEF, 0xE8};
inline constexpr uint32_t kRvaRotateReadRet = kRvaRotateReadSite + sizeof(kRotateReadSite) + 4; // 0x7E6141
// movss xmm6,[rbp-0x11] (YawInput); lea rdx,[rbp+0x17]; addss xmm0,xmm6 (entity yaw + YawInput)
inline constexpr uint32_t kRvaRotateUse = 0x7E615E;
inline constexpr uint8_t kRotateUse[] = {0xF3, 0x0F, 0x10, 0x75, 0xEF, 0x48, 0x8D, 0x55, 0x17, 0xF3, 0x0F, 0x58, 0xC6};
// mov dword [r14+0x4B8], 0.3f (the unlock blend)
inline constexpr uint32_t kRvaRotateBlend = 0x7E6315;
inline constexpr uint8_t kRotateBlend[] = {0x41, 0xC7, 0x86, 0xB8, 0x04, 0x00, 0x00, 0x9A, 0x99, 0x99, 0x3E};
inline constexpr uint32_t kRvaLookVars = 0x5FD84C;
inline constexpr uint8_t kLookVarsHead[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xEC,
                                            0x20, 0x48, 0x8B, 0x99, 0x80, 0x03, 0x00, 0x00, 0x48, 0x8B, 0xF2, 0x48, 0x8B, 0xF9};
// lea r8,[rsi+8] - the third variable (YawInput) goes to out+8
inline constexpr uint32_t kRvaLookVarsYaw = 0x5FD8D2;
inline constexpr uint8_t kLookVarsYaw[] = {0x4C, 0x8D, 0x46, 0x08};
inline constexpr uint32_t kLookVarsYawOffset = 8;


constexpr float kSentinel = 999.0f;    // FUN_1404e339c default when the graph variable is missing (DAT_1431ef9e4)
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kGain = 0.25f;         // feedback gain k (poles: delay 0 -> 0.75, delay 1 -> 0.5 double)
constexpr float kMaxStep = 180.0f;     // deg per tick: command clamp (round 2: 90 -> 180, a jump is no longer rate limited)
constexpr float kMultMin = 0.2f;       // yawMultiplier compensation only for kMultMin < m < 1
constexpr float kOvrHold = 0.03f;      // s after the last announced entity turn in which the read-back is not trusted
constexpr float kOvrGuardHold = 0.25f; // s after an override tick without divergence counting
constexpr float kDivergeDeg = 30.0f;   // |error| above this counts towards the divergence window
constexpr float kDivergeSlowRate = 300.0f; // deg/s: the target counts as slow below this (yaw and pitch; was 5 deg/tick)
constexpr float kDivergeWindow = 0.5f; // s of large error with a slow target per comparison
constexpr float kDivergeGrow = 5.0f;   // deg: error growth over one window that counts
constexpr int kDivergeWindows = 2;     // consecutive growing windows that trip
constexpr float kDivergeChurn = 180.0f; // deg of read-back motion beyond the target motion in one window: trip
constexpr float kDivergeCalm = 0.25f;  // s below kDivergeDeg that end a divergence episode
constexpr float kTripCooldown = 3.0f;  // s the write stays off after a trip before the native re-arms it
constexpr int kMaxTrips = 3;           // trips per game session; then the write stays off
constexpr float kDefaultDt = 1.0f / 60.0f;
constexpr float kLagMinStep = 0.5f;    // diagnostic lag hint only: deltas below this (deg) carry no evidence
constexpr float kAheadDiscMin = 0.5f;  // deg: an ahead sample counts for the order when its hypotheses differ by this
constexpr int kAheadModeMinN = 20;     // discriminating samples before the ahead order may change
constexpr float kAheadDecay = 0.97f;   // per discriminating sample: decayed residuals for the order choice

inline bool Finite(float aV)
{
    return std::isfinite(aV);
}

inline float ClampAbs(float aV, float aMax)
{
    return aV < -aMax ? -aMax : (aV > aMax ? aMax : aV);
}

inline float Wrap180(float aDeg)
{
    if (!Finite(aDeg))
        return aDeg;
    float d = std::fmod(aDeg + 180.0f, 360.0f);
    if (d < 0.0f)
        d += 360.0f;
    return d - 180.0f;
}

// Yaw (deg) of a rotation quaternion (x, y, z, w) in the game convention: forward = q * (0, 1, 0), yaw = atan2(-fx, fy).
// Same formula as the game's quaternion -> yaw (FUN_1402d057c, general branch): atan2(2(wz - xy), 1 - 2(x^2 + z^2)).
inline float YawFromQuat(float aX, float aY, float aZ, float aW)
{
    const float fx = 2.0f * (aX * aY - aW * aZ);
    const float fy = 1.0f - 2.0f * (aX * aX + aZ * aZ);
    return std::atan2(-fx, fy) * kRadToDeg;
}

// Native trip policy (pure, tested offline): may a tripped write be re-armed now?
inline bool RearmDue(float aSecondsSinceTrip, int aTripCount)
{
    return aTripCount < kMaxTrips && aSecondsSinceTrip >= kTripCooldown;
}

struct Inputs
{
    float targetYaw = 0.0f;   // wanted world yaw (deg)
    float targetPitch = 0.0f; // wanted world pitch (deg, up positive)
    float entityYaw = 0.0f;   // V's world yaw now (deg)
    float yawInput = 0.0f;    // FPP camera component +0x42C (graph output YawInput, deg relative to the entity)
    float pitchInput = 0.0f;  // +0x430 (PitchInput, deg)
    float pitchMin = -80.0f;  // +0x3E0 (reflected pitchMin)
    float pitchMax = 80.0f;   // +0x3E4 (reflected pitchMax)
    bool headingLocked = false; // +0x3F6 (reflected headingLocked)
    float aheadYaw = 0.0f;    // entity yaw turn requested for this tick (deg; utpp_bodyturn adjustTransform; 0 = none)
    float dt = 0.0f;          // tick time (s; <= 0 / invalid = 1/60)
    float yawMult = 1.0f;     // the game's yawMultiplier of the last apply (feature +0xC4; rubber band; 1 = none)
    float pitchMult = 1.0f;   // pitchMultiplier of the last apply (feature +0xC8)
};

enum class Skip : uint8_t
{
    None = 0,
    NotLocked,  // heading not locked: the game turns V with its look -> vanilla input
    Sentinel,   // the graph outputs are not available (999)
    NonFinite,  // a NaN / inf input
    Tripped,    // the loop diverged earlier (fail closed until Rearm)
};

inline const char* SkipName(Skip aSkip)
{
    switch (aSkip)
    {
    case Skip::None:
        return "none";
    case Skip::NotLocked:
        return "notLocked";
    case Skip::Sentinel:
        return "sentinel";
    case Skip::NonFinite:
        return "nonFinite";
    case Skip::Tripped:
        return "tripped";
    }
    return "?";
}

// Order of V's requested turn relative to the FPP camera update (see the header comment)
enum class AheadOrder : uint8_t
{
    After = 0,  // lands after the camera update of the tick that consumed the ahead (assumed default)
    Before = 1, // landed before that read (the entity yaw read already contains it)
    Late = 2,   // lands after the camera update of the NEXT tick
};

inline const char* AheadOrderName(AheadOrder aO)
{
    switch (aO)
    {
    case AheadOrder::After:
        return "after";
    case AheadOrder::Before:
        return "before";
    case AheadOrder::Late:
        return "late";
    }
    return "?";
}

struct Output
{
    bool write = false;
    Skip skip = Skip::None;
    float yawDelta = 0.0f;   // to the game (deg)
    float pitchDelta = 0.0f; // to the game (deg)
    float desiredYaw = 0.0f; // wanted YawInput r_n (deg, relative to the entity after the expected turn)
    float desiredPitch = 0.0f;
    float errYaw = 0.0f;     // feedback error c_{n-1} - Y_n (deg; Y_n predicted in an override tick)
    float errPitch = 0.0f;
    float rawErrYaw = 0.0f;  // c_{n-1} - the read-back as read (diagnostic; = errYaw outside overrides)
    float rawErrPitch = 0.0f;
    float ffYaw = 0.0f;      // feed-forward part of the delta (deg)
    float ffPitch = 0.0f;
    float aheadEff = 0.0f;   // entity turn taken out of the wanted YawInput this tick (deg, by the ahead order)
    int lag = 0;             // diagnostic hint only (never used for the command): 0 / 1 tick graph delay
    bool ovr = false;        // override tick: the read-back is not trusted (prediction used)
    bool release = false;    // first clean tick after an override: the remaining error corrected at once
    bool trippedNow = false; // this step tripped the divergence guard
};

// Feed-forward + feedback (see the header comment). A loop whose error keeps growing while the target is slow (wrong
// sign / mapping, a graph that ignores the delta while the target moves) trips and stays off (fail closed) until Rearm().
struct Controller
{
    bool have = false;
    float refYaw = 0.0f, refPitch = 0.0f;         // commanded reference c_{n-1}
    float prevRelYaw = 0.0f, prevRelPitch = 0.0f; // r_{n-1} (target speed for the divergence guard)
    float estYaw = 0.0f, estPitch = 0.0f;         // read-back used by the last step (measured, or predicted in an override)
    float appYaw = 0.0f, appPitch = 0.0f;         // what the graph integrates from the last write (NoteApplied)
    float a1 = 0.0f, a2 = 0.0f;                   // ahead of the last two writes
    float sinceAhead = 1e9f;                      // s since the last write with an ahead
    float sinceOvr = 1e9f;                        // s since the last override tick
    bool inOvr = false;                           // the last step was an override tick
    bool skipFb = false;                          // no feedback in this step (the tick after a release)
    // divergence guard
    float divT = 0.0f;   // s of large error with a slow target in the current window
    float divRef = 0.0f; // |error| at the window start
    int divGrows = 0;    // consecutive windows in which the error grew
    float divFrom = 0.0f; // |error| at the start of the first of them
    float divChurn = 0.0f; // read-back motion in the current window (deg)
    float divCalm = 0.0f;  // s below kDivergeDeg
    bool tripped = false;
    int tripKind = 0;      // 1: the error kept growing, 2: the read-back churned without convergence
    int tripCount = 0;     // trips since the controller was made (kept by Rearm)
    float tripErr = 0.0f;  // |error| when tripped
    float tripFrom = 0.0f; // |error| at the start of the tripping window(s)
    float tripChurn = 0.0f; // read-back motion of the last window (deg)
    // diagnostics (never used for the command): lag hint from the read-back steps, override, ahead order
    float prevYawIn = 0.0f, prevPitchIn = 0.0f; // raw read-back of the last step
    float d1Yaw = 0.0f, d1Pitch = 0.0f; // delta integrated from the last step
    float d2Yaw = 0.0f, d2Pitch = 0.0f; // delta integrated from the step before
    float score0 = 0.0f, score1 = 0.0f; // decayed mismatch of the lag 0 / lag 1 hypotheses
    int lag = 0;
    uint32_t lagSwitches = 0;
    uint64_t ovrTicks = 0, releases = 0;
    float lastRelease = 0.0f, maxRelease = 0.0f; // |error| corrected at a release (deg)
    float prevEntYaw = 0.0f;
    uint64_t aheadN = 0;     // reads after a tick with an ahead (any of the last three)
    double aheadAbs = 0.0;   // sum |ahead of the last tick|
    double aheadNoFeed = 0.0; // sum |measured turn| (what the look error would have been without the feed)
    uint64_t aheadDiscN = 0; // samples whose order hypotheses differ by >= kAheadDiscMin
    double resAfter = 0.0, resBefore = 0.0, resLate = 0.0; // sums |turn - ahead of the hypothesis| (discriminating)
    float wAfter = 0.0f, wBefore = 0.0f, wLate = 0.0f;     // decayed residuals (order choice)
    AheadOrder aheadOrder = AheadOrder::After;
    uint32_t aheadOrderSwitches = 0;
    float maxAbsErr = 0.0f;  // max |feedback error| since the last TakeMaxErr() (release ticks excluded)

    // History only (a pause of the write, a new component): the next write is a first write (feedback only).
    void ResetHistory()
    {
        have = false;
        d1Yaw = d1Pitch = d2Yaw = d2Pitch = 0.0f;
        appYaw = appPitch = 0.0f;
        a1 = a2 = 0.0f;
        sinceAhead = sinceOvr = 1e9f;
        inOvr = skipFb = false;
        divT = 0.0f;
        divRef = 0.0f;
        divGrows = 0;
        divChurn = 0.0f;
        divCalm = 0.0f;
    }
    void Rearm()
    {
        ResetHistory();
        tripped = false;
        tripKind = 0;
        tripErr = tripFrom = tripChurn = 0.0f;
    }
    float TakeMaxErr()
    {
        const float m = maxAbsErr;
        maxAbsErr = 0.0f;
        return m;
    }
    // After the game's apply of the last written delta: what the graph integrates (feature deltaYaw * yawMultiplier,
    // deltaPitch * pitchMultiplier; the apply may clamp or replace the delta). Used by the next prediction.
    void NoteApplied(float aYaw, float aPitch)
    {
        if (!have || !Finite(aYaw) || !Finite(aPitch) || std::fabs(aYaw) > 360.0f || std::fabs(aPitch) > 360.0f)
            return;
        appYaw = aYaw;
        appPitch = aPitch;
        d1Yaw = aYaw;
        d1Pitch = aPitch;
    }

    Output Step(const Inputs& aIn)
    {
        Output out;
        out.lag = lag;
        if (tripped)
        {
            out.skip = Skip::Tripped;
            return out;
        }
        if (!Finite(aIn.targetYaw) || !Finite(aIn.targetPitch) || !Finite(aIn.entityYaw) || !Finite(aIn.yawInput) ||
            !Finite(aIn.pitchInput) || !Finite(aIn.pitchMin) || !Finite(aIn.pitchMax) || !Finite(aIn.aheadYaw))
        {
            out.skip = Skip::NonFinite;
            ResetHistory();
            return out;
        }
        if (!aIn.headingLocked)
        {
            out.skip = Skip::NotLocked;
            ResetHistory();
            return out;
        }
        if (aIn.yawInput == kSentinel || aIn.pitchInput == kSentinel)
        {
            out.skip = Skip::Sentinel;
            ResetHistory();
            return out;
        }
        const float dt = (Finite(aIn.dt) && aIn.dt > 0.0f && aIn.dt < 0.5f) ? aIn.dt : kDefaultDt;
        const float a0 = aIn.aheadYaw;
        sinceAhead = a0 != 0.0f ? 0.0f : sinceAhead + dt;
        // override tick: an entity turn was announced now or in the last two ticks / kOvrHold s - the read-back may be
        // the game's override (not trusted; the prediction is used)
        const bool ovr = have && (a0 != 0.0f || a1 != 0.0f || a2 != 0.0f || sinceAhead <= kOvrHold);
        const bool release = have && !ovr && inOvr;
        out.ovr = ovr;
        out.release = release;
        if (have)
        {
            // diagnostics only: which of the last two deltas the read-back step matches (lag hint; clean reads only)
            if (!ovr && !inOvr)
            {
                const float sYaw = Wrap180(aIn.yawInput - prevYawIn);
                const float sPitch = aIn.pitchInput - prevPitchIn;
                const float evidence = std::fabs(d1Yaw) + std::fabs(d2Yaw) + std::fabs(d1Pitch) + std::fabs(d2Pitch);
                if (evidence > kLagMinStep)
                {
                    score0 = 0.9f * score0 + std::fabs(Wrap180(sYaw - d1Yaw)) + std::fabs(sPitch - d1Pitch);
                    score1 = 0.9f * score1 + std::fabs(Wrap180(sYaw - d2Yaw)) + std::fabs(sPitch - d2Pitch);
                    if (lag == 0 && score0 > 1.0f && score1 < 0.5f * score0)
                    {
                        lag = 1;
                        ++lagSwitches;
                    }
                    else if (lag == 1 && score1 > 1.0f && score0 < 0.5f * score1)
                    {
                        lag = 0;
                        ++lagSwitches;
                    }
                }
            }
            // ahead order: the entity turn measured between the last read and this one vs the three hypotheses
            if (a0 != 0.0f || a1 != 0.0f || a2 != 0.0f)
            {
                const float turn = Wrap180(aIn.entityYaw - prevEntYaw);
                ++aheadN;
                aheadAbs += std::fabs(a1);
                aheadNoFeed += std::fabs(turn);
                const float spread =
                    std::fmax(std::fabs(a0 - a1), std::fmax(std::fabs(a1 - a2), std::fabs(a0 - a2)));
                if (spread >= kAheadDiscMin)
                {
                    const float rA = std::fabs(Wrap180(turn - a1));
                    const float rB = std::fabs(Wrap180(turn - a0));
                    const float rL = std::fabs(Wrap180(turn - a2));
                    ++aheadDiscN;
                    resAfter += rA;
                    resBefore += rB;
                    resLate += rL;
                    wAfter = kAheadDecay * wAfter + rA;
                    wBefore = kAheadDecay * wBefore + rB;
                    wLate = kAheadDecay * wLate + rL;
                    if (aheadDiscN >= static_cast<uint64_t>(kAheadModeMinN))
                    {
                        AheadOrder best = AheadOrder::After;
                        float wBest = wAfter;
                        if (wBefore < wBest)
                        {
                            best = AheadOrder::Before;
                            wBest = wBefore;
                        }
                        if (wLate < wBest)
                        {
                            best = AheadOrder::Late;
                            wBest = wLate;
                        }
                        const float wCur = aheadOrder == AheadOrder::After
                                               ? wAfter
                                               : (aheadOrder == AheadOrder::Before ? wBefore : wLate);
                        if (best != aheadOrder && wBest < 0.5f * wCur)
                        {
                            aheadOrder = best;
                            ++aheadOrderSwitches;
                        }
                    }
                }
            }
        }
        out.lag = lag;
        // the entity turn that lands between this read and the next one (by the measured order)
        const float aEff = aheadOrder == AheadOrder::After ? a0 : (aheadOrder == AheadOrder::Late ? a1 : 0.0f);
        out.aheadEff = aEff;
        // wanted YawInput / PitchInput of this tick (relative to the entity after the turn expected in this tick)
        const float relYaw = Wrap180(aIn.targetYaw - aIn.entityYaw - aEff);
        float relPitch = aIn.targetPitch;
        if (aIn.pitchMin <= aIn.pitchMax)
            relPitch = relPitch < aIn.pitchMin ? aIn.pitchMin : (relPitch > aIn.pitchMax ? aIn.pitchMax : relPitch);
        out.desiredYaw = relYaw;
        out.desiredPitch = relPitch;
        if (!have) // first write after (re)activation: no feed-forward of the target, feedback only; the entity turn
        {          // announced for this tick is still compensated (reference = the relative target without it)
            refYaw = Wrap180(relYaw + aEff);
            refPitch = relPitch;
            prevRelYaw = refYaw;
            prevRelPitch = relPitch;
            divT = 0.0f;
        }
        // the read-back used by the feedback: measured, or in an override tick the model prediction (graph delay 0)
        float yIn = aIn.yawInput, pIn = aIn.pitchInput;
        if (ovr)
        {
            yIn = Wrap180(estYaw + appYaw);
            pIn = estPitch + appPitch;
            ++ovrTicks;
        }
        out.ffYaw = ClampAbs(Wrap180(relYaw - refYaw), kMaxStep);
        out.ffPitch = ClampAbs(relPitch - refPitch, kMaxStep);
        out.errYaw = Wrap180(refYaw - yIn);
        out.errPitch = refPitch - pIn;
        out.rawErrYaw = Wrap180(refYaw - aIn.yawInput);
        out.rawErrPitch = refPitch - aIn.pitchInput;
        const float g = release ? 1.0f : (skipFb ? 0.0f : kGain);
        // rubber band compensation (the graph integrates yawMultiplier * deltaYaw, pitchMultiplier * deltaPitch)
        const float m = (Finite(aIn.yawMult) && aIn.yawMult > kMultMin && aIn.yawMult < 1.0f) ? aIn.yawMult : 1.0f;
        const float mp =
            (Finite(aIn.pitchMult) && aIn.pitchMult > kMultMin && aIn.pitchMult < 1.0f) ? aIn.pitchMult : 1.0f;
        out.yawDelta = ClampAbs((out.ffYaw + g * out.errYaw) / m, kMaxStep);
        out.pitchDelta = ClampAbs((out.ffPitch + g * out.errPitch) / mp, kMaxStep);
        const float err = std::fmax(std::fabs(out.errYaw), std::fabs(out.errPitch));
        if (release)
        {
            ++releases;
            lastRelease = err;
            if (err > maxRelease)
                maxRelease = err;
        }
        skipFb = release;
        // divergence guard: growth of the error that the target motion does not explain (clean ticks only)
        sinceOvr = ovr ? 0.0f : sinceOvr + dt;
        const float dRelYaw = Wrap180(relYaw - prevRelYaw), dRelPitch = relPitch - prevRelPitch;
        const float moveRate = std::fmax(std::fabs(dRelYaw), std::fabs(dRelPitch)) / dt;
        if (!release && err > maxAbsErr) // (the error corrected at a release is reported as lastRelease / maxRelease)
            maxAbsErr = err;
        // read-back motion the target motion does not explain (a healthy loop follows the target; wrong sign: it spins)
        const float churn = (have && !ovr && !inOvr)
                                ? std::fabs(Wrap180(Wrap180(aIn.yawInput - prevYawIn) - dRelYaw)) +
                                      std::fabs((aIn.pitchInput - prevPitchIn) - dRelPitch)
                                : 0.0f;
        if (ovr || release || sinceOvr < kOvrGuardHold)
        {
            divT = 0.0f; // V is being turned by utpp_bodyturn (the game overrides the look): no count
            divGrows = 0;
            divCalm = 0.0f;
        }
        else if (moveRate >= kDivergeSlowRate)
        {
            divT = 0.0f; // flick: a new count starts from the error after it
            divGrows = 0;
            divCalm = 0.0f;
        }
        else if (err > kDivergeDeg)
        {
            divCalm = 0.0f;
            if (divT <= 0.0f)
            {
                divRef = err;
                divChurn = 0.0f;
            }
            divT += dt;
            divChurn += churn;
            if (divT >= kDivergeWindow)
            {
                const bool grew = err > divRef + kDivergeGrow;
                const bool spun = divChurn > kDivergeChurn && err > 0.5f * divRef;
                if (grew)
                {
                    if (divGrows == 0)
                        divFrom = divRef;
                    ++divGrows;
                }
                else
                    divGrows = 0; // large but not growing (a clamp, the game pulling the look): no count
                if (divGrows >= kDivergeWindows || spun)
                {
                    tripped = true;
                    ++tripCount;
                    tripKind = divGrows >= kDivergeWindows ? 1 : 2;
                    tripErr = err;
                    tripFrom = tripKind == 1 ? divFrom : divRef;
                    tripChurn = divChurn;
                    out.trippedNow = true;
                    out.skip = Skip::Tripped;
                    ResetHistory();
                    return out;
                }
                divT = 0.0f; // next window
            }
        }
        else
        {
            divCalm += dt; // a dip below the threshold keeps the window; a calm period ends the episode
            if (divCalm >= kDivergeCalm)
            {
                divT = 0.0f;
                divGrows = 0;
            }
        }
        refYaw = Wrap180(refYaw + out.ffYaw);
        refPitch += out.ffPitch;
        prevRelYaw = relYaw;
        prevRelPitch = relPitch;
        out.write = true;
        d2Yaw = d1Yaw;
        d2Pitch = d1Pitch;
        appYaw = out.yawDelta * m; // what the graph integrates (NoteApplied replaces it with the feature after the apply)
        appPitch = out.pitchDelta * mp;
        d1Yaw = appYaw;
        d1Pitch = appPitch;
        estYaw = yIn;
        estPitch = pIn;
        prevYawIn = aIn.yawInput;
        prevPitchIn = aIn.pitchInput;
        prevEntYaw = aIn.entityYaw;
        a2 = a1;
        a1 = a0;
        inOvr = ovr;
        have = true;
        return out;
    }
};

// ---- Vehicle edge zeroing (2026-10-28, unmount swivel) ----
// At the end of the PSM vehicle state the game's on-foot controller turns V from the exit yaw onto the held FPP look
// (world look = entity yaw + YawInput): feed yaw +10.9 -> -8.3 in 0.2 s (~100 deg/s), a visible swivel. The UTPP orbit
// camera does not use the game's look yaw outside aim (it writes its own world pose in entity space), so the swivel can be
// removed on the look side: for kEdgeZeroSeconds after the first on-foot feed after a vehicle feed (third person only)
// the hook drives YawInput to 0 (look heading = entity heading), independent of the heading lock and of UTPPSetGameLook;
// then the call passes through again. Pitch is never touched.
// Round 2 (in-game log 2026-10-28 23:04, two bike exits): armed at the edge it came too late - yawIn 18.2 / -18.3 at the
// edge, unlockBlend 0.170, no hook call for the first 177 ms, drift 17.2 / 16.9 deg. Cause (2.31 decompile
// RotateEntityToCamera FUN_14080404c): at the unmount the game calls it ~0.13 s
// BEFORE the PSM vehicle bit clears; it reads the graph YawInput, requests the entity rotation to entity yaw + YawInput
// (the swivel) and sets the unlock blend +0x4B8 = 0.3 s, during which the camera update skips the delta apply (our hook).
// So YawInput must already be 0 when it is called: the zeroing now also runs through the whole exit, from the PSM exit
// state (ExitingEventsBase.OnEnter, scripts\utpp_gamelook_exit.reds -> VehicleExitStart) until the edge window ends
// (kExitMaxSeconds cap for an exit that never reaches the edge). Third person only (UTPP third person and the ride's
// vehicle camera not first person). The heading lock no longer skips a tick (YawInput 0 = look along V either way; a
// normal write of the tick keeps priority). The exit phase starts kExitZeroDelay s after the exit start: utppCamera takes
// the view from the vehicle camera with a 0.30 s blend at the exit start (utpp_vehiclecam 'utppCamera take (exit start)'),
// and the vehicle camera is first person during the exit, so the look is not moved while that camera may still show;
// the unmount call comes >= ~0.85 s after the exit start (quick exits ~1.0 s, normal ~1.85 s).
// Command (exact for a graph delay of 0 or 1 tick, also when it switches): a correction tick writes -YawInput (/ the
// rubber band factor), the next tick writes 0 (the correction may still be in flight), then the next correction reads
// the settled value. Ticks with a sentinel read-back (999: graph outputs missing around the graph swap) or a non-finite
// input pass through unchanged and do not end the window.
// Round 3 (in-game log 2026-10-29 00:29, two bike exits): during the whole exit the heading is LOCKED (locked=93 of 93
// ticks, YawInput 0): the look holds the vehicle heading (-8.35) while the exit clip turns V +19 deg; the unlock at the
// unmount then sees YawInput -18.2 and RotateEntityToCamera turns V back (drift 17.6 / 16.6 deg). YawInput cannot be
// zeroed under the lock, so the hook now sits on the read RotateEntityToCamera does (kRvaLookVars, called from
// kRvaRotateReadSite, identified by its return address): inside the gate (RotateZeroGate) the YawInput it reads is 0,
// so V keeps the exit yaw and the unlock blend starts from 0 (the game's look starts at V's heading; the UTPP orbit
// camera does not use it). The graph's own YawInput is then zeroed by the edge window after the 0.3 s unlock blend
// (kEdgeZeroSeconds 0.5: the blend ends ~0.17 s after the edge, the first hook call comes ~0.18 s after it).
constexpr float kEdgeZeroSeconds = 0.5f;   // s of zeroing after the vehicle edge
constexpr float kEdgeReportSeconds = 0.6f; // s after the edge for the summary line (entity yaw drift check window)
constexpr float kExitMaxSeconds = 5.0f;    // s of exit-phase zeroing at most (the exit clip is ~2 s; aborted exits)
constexpr float kExitZeroDelay = 0.4f;     // s after the exit start before the exit phase writes (camera take-over blend)

// Is the zeroing on for this tick? Ages in seconds since the exit start / the edge arm (< 0 = not armed).
inline bool EdgeZeroWanted(double aExitAge, double aEdgeAge)
{
    const bool exitOn =
        aExitAge >= static_cast<double>(kExitZeroDelay) && aExitAge <= static_cast<double>(kExitMaxSeconds);
    const bool edgeOn = aEdgeAge >= 0.0 && aEdgeAge <= static_cast<double>(kEdgeZeroSeconds);
    return exitOn || edgeOn;
}

// Exit start (reds): zeroing only when UTPP shows third person and the ride's vehicle camera is not first person
// (aRideTpp: 1 TPP, 0 FPP, -1 unknown - no vehicle camera manager).
inline bool ExitZeroAllowed(bool aUtppTpp, int aRideTpp)
{
    return aUtppTpp && aRideTpp == 1;
}

// Round 3: is the YawInput read by RotateEntityToCamera zeroed? Only for the player's FPP camera component, only for the
// read at kRvaRotateReadRet (the caller check is in the hook), while the third-person exit phase (from the exit start, no
// kExitZeroDelay: the unmount call may come before it ends in a very quick exit; capped) or the edge window is open.
inline bool RotateZeroGate(bool aPlayerComp, double aExitAge, double aEdgeAge)
{
    if (!aPlayerComp)
        return false;
    const bool exitOn = aExitAge >= 0.0 && aExitAge <= static_cast<double>(kExitMaxSeconds);
    const bool edgeOn = aEdgeAge >= 0.0 && aEdgeAge <= static_cast<double>(kEdgeZeroSeconds);
    return exitOn || edgeOn;
}

// The read value that is replaced by 0: a real YawInput (not the 999 'missing' default, finite).
inline bool RotateYawZeroable(float aYawIn)
{
    return Finite(aYawIn) && aYawIn != kSentinel;
}

// Feed side (game thread): the first on-foot feed after any feed with the vehicle bit is the edge (also after a feed gap
// during the ride). A feed with the vehicle bit cancels a running window.
enum class EdgeEvent : uint8_t
{
    None = 0,
    Arm,    // edge in third person: start a zeroing window
    NoTpp,  // edge in first person: the game look is the rendered view, nothing written
    Cancel, // the vehicle bit is back
};

struct VehicleEdge
{
    bool sawVehicle = false;

    EdgeEvent Feed(bool aVehicleBit, bool aTpp)
    {
        if (aVehicleBit)
        {
            sawVehicle = true;
            return EdgeEvent::Cancel;
        }
        if (!sawVehicle)
            return EdgeEvent::None;
        sawVehicle = false;
        return aTpp ? EdgeEvent::Arm : EdgeEvent::NoTpp;
    }
};

enum class EdgeSkip : uint8_t
{
    None = 0,
    Sentinel,  // YawInput 999
    NonFinite, // NaN / inf
};

struct EdgeZeroOut
{
    bool write = false;
    bool correction = false; // a correction tick (else a hold tick writing 0)
    EdgeSkip skip = EdgeSkip::None;
    float yawDelta = 0.0f;   // to the game (deg)
};

// Hook side (one window; Reset() at each arm).
struct EdgeZero
{
    int ticks = 0;        // Step calls in this window
    int writes = 0;       // ticks written (corrections + holds)
    int corrections = 0;
    int sinceCorr = 1000; // ticks since the last correction write
    float firstYawIn = 0.0f; // read-back of the first written tick
    bool haveFirst = false;
    int locked = 0, sentinel = 0, nonFinite = 0; // locked: diagnostic only (the tick is written)

    void Reset()
    {
        *this = EdgeZero{};
    }

    // aYawMult: the game's yawMultiplier of the last apply (compensated for kMultMin < m < 1, as the controller does)
    EdgeZeroOut Step(float aYawInput, bool aLocked, float aYawMult)
    {
        EdgeZeroOut out;
        ++ticks;
        ++sinceCorr;
        if (!Finite(aYawInput))
        {
            ++nonFinite;
            out.skip = EdgeSkip::NonFinite;
            return out;
        }
        if (aLocked)
            ++locked;
        if (aYawInput == kSentinel)
        {
            ++sentinel;
            out.skip = EdgeSkip::Sentinel;
            return out;
        }
        if (!haveFirst)
        {
            haveFirst = true;
            firstYawIn = aYawInput;
        }
        out.write = true;
        ++writes;
        if (sinceCorr >= 2)
        {
            const float m = (Finite(aYawMult) && aYawMult > kMultMin && aYawMult < 1.0f) ? aYawMult : 1.0f;
            out.yawDelta = ClampAbs(-Wrap180(aYawInput) / m, kMaxStep);
            out.correction = true;
            ++corrections;
            sinceCorr = 0;
        }
        return out;
    }
};
} // namespace utpp::gamelook
