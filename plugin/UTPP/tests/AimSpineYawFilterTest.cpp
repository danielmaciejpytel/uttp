// Offline test of the aim-offset spine yaw filter (src/mm/AimSpineYawFilter.hpp), fix batch 2 group AO. No game data:
// angles only. Usage: utpp_mm_aospineyaw_test [recording_1215_aim_offset_replay.csv]   Exit code 0 = all checks passed.
//
// Defects of the recording 2026-10-21 12:15 (diag P2 C1/C3, P3 RC1, P5 A/B, NEW 1):
//   A) the target yaw wraps at +-180 (camera behind the visual root) and the +-60 clamp jumps +60 -> -60 in one frame:
//      120 deg of spine twist (30 deg on each of the 4 spine bones), 11 events;
//   B) `haveAim` drops in one frame (end of the hipfire hold, AIM off, reload start / end of the aim hold) while the Aiming
//      weight is ~1: the whole AO (up to 60 deg of spine twist) ends in one frame, 17 events.
// Checks (every bound is derived from the filter constants / the recorded jump sizes and stated at the check):
//   1) dt <= 0 probe frames: stateless pass-through = clamp(raw), no state (the MotionPoseTest probes rely on it);
//   2) calm aiming (inside the rate limit): bit-identical to clamp(raw);
//   3) recorded wrap 91.952..91.969 s: no step above kAoYawRateDegPerSec*dt, the side is kept while |yaw| > 150, the side
//      change takes >= 15 frames; cycles +170/+178/-179/-172/+177 never change the side;
//   4) lost target: angles frozen, applied twist falls with min(Aiming weight, release) in steps <= 60*dt/0.5 (machine
//      fade) or <= 60*dt/0.25 (weight stays 1), and ends within kAoLatchMaxAgeSeconds;
//   5) re-acquire during the latch: release climbs at most dt/0.15 per frame, no jump of the yaw;
//   6) resets: weapon class, gap, zero weight, Reset();
//   7) replay of the recording (tests/data/recording_1215_aim_offset_replay.csv, 30 jump events + 3 calm ADS windows, the Aiming weight
//      from the real OverlayMachine): the former logic has steps up to 120 deg, the filter has none >= 20 deg (and no
//      per-bone step >= 10 deg = 40 deg of twist from a lost target); calm windows are unchanged.
// Mutations (UTPP_POSEV2_AO_MUTATION): 5 = no continuity, 6 = no latch; both MUST fail checks 3 / 4 / 7.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/AimSpineYawFilter.hpp"
#include "../src/mm/Overlay.hpp"

using namespace utpp::mm;
using motionpose::AimSpineYawFilter;

static int g_pass = 0, g_fail = 0;
static void Check(bool aOk, const std::string& aWhat)
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
}
template <class... A> static std::string F(const char* aFmt, A... a)
{
    char b[512];
    std::snprintf(b, sizeof(b), aFmt, a...);
    return b;
}

static constexpr float kDt = 1.0f / 60.0f;
static constexpr float kClamp = 60.0f;
static constexpr float kStepMax = motionpose::kAoYawRateDegPerSec * kDt + 1e-3f; // 8 deg at 60 fps

static float Clamp60(float aYaw) { return std::max(-kClamp, std::min(kClamp, aYaw)); }
static float WrapDeg(float aDeg) { return aDeg - 360.0f * std::floor((aDeg + 180.0f) / 360.0f); }

static bool Step(AimSpineYawFilter& aF, bool aHave, float aW, float aYaw, float aPitch, float aDtSec, int aCtx,
                 AimSpineYawFilter::Output& aOut, bool aSuppress = false)
{
    return aF.Step(aHave, aW, aYaw, aPitch, aDtSec, kClamp, aCtx, aSuppress, aOut);
}

// The applied twist of a frame: weight * yaw, 0 when nothing is applied.
static float Applied(bool aHave, const AimSpineYawFilter::Output& aOut, float aW)
{
    if (!aHave)
        return 0.0f;
    const float alpha = aOut.release < 1.0f ? std::min(aW, aOut.release) : aW;
    return alpha * aOut.yawDeg;
}

static void TestProbeFrames()
{
    AimSpineYawFilter f;
    AimSpineYawFilter::Output o;
    bool allExact = true;
    for (const float dt : {0.0f, -0.01f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()})
    {
        for (const float yaw : {0.0f, 30.0f, 120.0f, -179.0f, 60.0f, -60.0f})
        {
            const bool have = Step(f, true, 1.0f, yaw, 12.5f, dt, 0, o);
            const float want = Clamp60(yaw);
            allExact = allExact && have && std::memcmp(&o.yawDeg, &want, sizeof(float)) == 0 && o.pitchDeg == 12.5f &&
                       o.release == 1.0f && !f.Valid();
        }
        allExact = allExact && !Step(f, false, 1.0f, 30.0f, 0.0f, dt, 0, o);
    }
    Check(allExact, "1) dt <= 0 / NaN probe frames: stateless clamp(raw), no state, no AO without a target");
    // a probe after real steps holds the last output (no raw pass-through)
    AimSpineYawFilter g;
    for (int i = 0; i < 10; ++i)
        Step(g, true, 1.0f, 40.0f, 5.0f, kDt, 0, o);
    const bool held = Step(g, true, 1.0f, -50.0f, 9.0f, 0.0f, 0, o) && o.yawDeg == 40.0f && o.pitchDeg == 5.0f;
    Check(held, F("1) a dt = 0 frame on a running filter holds its last output (yaw %.2f pitch %.2f)", o.yawDeg, o.pitchDeg));
    Check(Step(g, false, 1.0f, 0.0f, 0.0f, 0.0f, 0, o) && o.yawDeg == 40.0f && !o.live,
          "1) ... and so does a dt = 0 frame without a target while the latch runs");
}

static void TestCalm()
{
    AimSpineYawFilter f;
    AimSpineYawFilter::Output o;
    int bad = 0, n = 0;
    float maxRate = 0.0f, prev = 0.0f;
    for (int i = 0; i < 1200; ++i)
    {
        // +-140 deg sweep, 5 s period: peak 176 deg/s = 2.9 deg/frame, through and beyond the +-60 clamp
        const float yaw = 140.0f * std::sin(6.2831853f * static_cast<float>(i) * kDt / 5.0f);
        const float pitch = 20.0f * std::sin(0.01f * static_cast<float>(i));
        const bool have = Step(f, true, 0.8f, yaw, pitch, kDt, 1, o);
        const float want = Clamp60(yaw);
        if (!have || std::memcmp(&o.yawDeg, &want, sizeof(float)) != 0 || o.pitchDeg != pitch || o.release != 1.0f || !o.live)
            ++bad;
        if (i > 0)
            maxRate = std::max(maxRate, std::fabs(yaw - prev) / kDt);
        prev = yaw;
        ++n;
    }
    Check(bad == 0, F("2) calm aiming (%d frames, peak target rate %.0f deg/s): output == clamp(raw) bit for bit, %d differ", n,
                      maxRate, bad));
    // fast tracking on one side is NOT limited: -60 -> +60 through the front in 6 frames (20 deg/frame = 1200 deg/s), and the
    // recorded flick of 109.406 .. 109.495 s (hipfire shots, 1000-1300 deg/s: -56.0 -38.9 -20.6 -2.1 17.7 37.9 59.5)
    AimSpineYawFilter g;
    int badFast = 0;
    float maxFast = 0.0f, prevFast = 0.0f;
    const float fast[] = {-60.0f, -40.0f, -20.0f, 0.0f, 20.0f, 40.0f, 60.0f, 40.0f, 20.0f, 0.0f, -20.0f,
                          -56.0f, -38.9f, -20.6f, -2.1f, 17.7f, 37.9f, 59.5f};
    for (size_t i = 0; i < sizeof(fast) / sizeof(fast[0]); ++i)
    {
        Step(g, true, 1.0f, fast[i], 0.0f, kDt, 0, o);
        const float want = Clamp60(fast[i]);
        badFast += std::memcmp(&o.yawDeg, &want, sizeof(float)) != 0 ? 1 : 0;
        if (i > 0)
            maxFast = std::max(maxFast, std::fabs(o.yawDeg - prevFast));
        prevFast = o.yawDeg;
    }
    Check(badFast == 0, F("2) fast tracking on one side (max %.1f deg/frame = %.0f deg/s, below the %.0f deg jump): output == clamp(raw), %d differ",
                          maxFast, maxFast / kDt, motionpose::kAoJumpDeg, badFast));
}

static void TestWrap()
{
    // settle on the left side (camera behind to the left), then the camera crosses the back: the recorded raw yaw profile of
    // 91.952..91.969 s (NEW 7f: -169.4 -176.5 +176.3 +168.5 +161.8 +154.5) continuing at -3 deg/frame to +20
    AimSpineYawFilter f;
    AimSpineYawFilter::Output o;
    for (int i = 0; i < 30; ++i)
        Step(f, true, 1.0f, -165.0f, 0.0f, kDt, 0, o);
    std::vector<float> raw = {-169.4f, -176.5f, 176.3f, 168.5f, 161.8f, 154.5f};
    for (float y = 151.5f; y >= 20.0f; y -= 3.0f)
        raw.push_back(y);
    float prev = o.yawDeg, maxStep = 0.0f;
    int sideKept = 0, firstFlip = -1, arrived = -1;
    for (size_t i = 0; i < raw.size(); ++i)
    {
        Step(f, true, 1.0f, raw[i], 0.0f, kDt, 0, o);
        maxStep = std::max(maxStep, std::fabs(o.yawDeg - prev));
        prev = o.yawDeg;
        if (std::fabs(raw[i]) > motionpose::kAoBackHysteresisDeg && o.yawDeg == -kClamp)
            ++sideKept;
        if (firstFlip < 0 && o.yawDeg > -kClamp + 0.01f)
            firstFlip = static_cast<int>(i);
        if (arrived < 0 && o.yawDeg >= kClamp - 0.01f)
            arrived = static_cast<int>(i);
    }
    int behind = 0;
    for (float r : raw)
        behind += std::fabs(r) > motionpose::kAoBackHysteresisDeg ? 1 : 0;
    Check(maxStep <= kStepMax, F("3) recorded wrap -176.5 -> +176.3: max step %.2f deg/frame <= %.2f (the former clamp: 120)", maxStep, kStepMax));
    Check(sideKept == behind, F("3) the side (-60) is kept while |raw| > %.0f: %d of %d frames", motionpose::kAoBackHysteresisDeg, sideKept, behind));
    Check(firstFlip == behind && arrived - firstFlip >= 14,
          F("3) the side change starts when raw leaves the back sector (frame %d = %d) and takes %d frames to +60 (>= 15 = 120 deg / %.0f deg per frame)",
            firstFlip, behind, arrived - firstFlip + 1, kStepMax));
    Check(std::fabs(o.yawDeg - raw.back()) < 1e-4f, F("3) ... then follows raw again (yaw %.2f, raw %.2f)", o.yawDeg, raw.back()));
    // the same crossing the other way and cycles around the back
    AimSpineYawFilter g;
    for (int i = 0; i < 30; ++i)
        Step(g, true, 1.0f, 100.0f, 0.0f, kDt, 0, o);
    const float cyc[5] = {170.0f, 178.0f, -179.0f, -172.0f, 177.0f};
    bool signKept = true;
    float cycMax = 0.0f;
    prev = o.yawDeg;
    for (int rep = 0; rep < 5; ++rep)
        for (float c : cyc)
            for (int k = 0; k < 6; ++k)
            {
                Step(g, true, 1.0f, c, 0.0f, kDt, 0, o);
                signKept = signKept && o.yawDeg == kClamp;
                cycMax = std::max(cycMax, std::fabs(o.yawDeg - prev));
                prev = o.yawDeg;
            }
    Check(signKept && cycMax == 0.0f, F("3) cycles +170 +178 -179 -172 +177 around the back keep +60 (max step %.2f)", cycMax));
}

static void TestLoss()
{
    // 20 live frames at yaw 60 / pitch -20 after a settle, then the target is lost. Machine fade: weight 1 -> 0 in 0.5 s.
    for (int variant = 0; variant < 2; ++variant)
    {
        const bool plateau = variant == 1; // the Aiming weight stays 1 (the machine has not started to fade)
        AimSpineYawFilter f;
        AimSpineYawFilter::Output o;
        float w = 1.0f;
        for (int i = 0; i < 50; ++i)
            Step(f, true, 1.0f, 60.0f, -20.0f, kDt, 0, o);
        float prev = Applied(true, o, 1.0f), maxStep = 0.0f, firstStep = 0.0f;
        bool frozen = true;
        int framesApplied = 0, endFrame = -1;
        for (int i = 1; i <= 60; ++i)
        {
            if (!plateau)
                w = std::max(0.0f, 1.0f - static_cast<float>(i) * kDt / 0.5f);
            const bool have = Step(f, false, w, 123.0f, 77.0f, kDt, 0, o);
            if (have)
            {
                ++framesApplied;
                frozen = frozen && o.yawDeg == 60.0f && o.pitchDeg == -20.0f && !o.live;
            }
            else if (endFrame < 0)
                endFrame = i;
            const float cur = Applied(have, o, w);
            if (i == 1)
                firstStep = std::fabs(cur - prev);
            maxStep = std::max(maxStep, std::fabs(cur - prev));
            prev = cur;
        }
        const float bound = plateau ? 60.0f * kDt / motionpose::kAoReleaseTailSeconds + 0.01f : 60.0f * kDt / 0.5f + 0.01f;
        const float endAt = static_cast<float>(endFrame) * kDt;
        Check(maxStep <= bound, F("4%c) lost target, %s: max applied-twist step %.3f deg/frame <= %.3f (the former hard gate: 60)",
                                  plateau ? 'b' : 'a', plateau ? "weight stays 1" : "weight fades 0.5 s", maxStep, bound));
        Check(frozen && firstStep < 2.1f, F("4%c) ... angles frozen at the last yaw 60 / pitch -20, first step %.3f deg", plateau ? 'b' : 'a', firstStep));
        Check(endFrame > 0 && endAt <= motionpose::kAoLatchMaxAgeSeconds + 1.5f * kDt && (plateau || endAt >= 0.45f),
              F("4%c) ... ends after %.3f s (<= %.2f latch cap)", plateau ? 'b' : 'a', endAt, motionpose::kAoLatchMaxAgeSeconds));
        (void)framesApplied;
    }
    // no latch without a previous target; a zero weight ends the latch at once
    AimSpineYawFilter g;
    AimSpineYawFilter::Output o;
    Check(!Step(g, false, 1.0f, 0.0f, 0.0f, kDt, 0, o), "4) no previous target: no AO");
    for (int i = 0; i < 5; ++i)
        Step(g, true, 1.0f, 30.0f, 0.0f, kDt, 0, o);
    Check(!Step(g, false, 0.0f, 0.0f, 0.0f, kDt, 0, o) && !g.Valid(), "4) lost target with Aiming weight 0: nothing to fade, state dropped");
}

static void TestReacquire()
{
    AimSpineYawFilter f;
    AimSpineYawFilter::Output o;
    for (int i = 0; i < 30; ++i)
        Step(f, true, 1.0f, 50.0f, 0.0f, kDt, 0, o);
    // latch for 0.5 s (release 0.4), then the target returns at yaw -20
    for (int i = 0; i < 30; ++i)
        Step(f, false, 1.0f, 0.0f, 0.0f, kDt, 0, o);
    const float relStart = o.release;
    float prevRel = relStart, prevYaw = o.yawDeg, maxRelStep = 0.0f, maxYawStep = 0.0f;
    bool mono = true;
    int framesToFull = -1;
    for (int i = 1; i <= 30; ++i)
    {
        Step(f, true, 1.0f, -20.0f, 0.0f, kDt, 0, o);
        maxRelStep = std::max(maxRelStep, o.release - prevRel);
        mono = mono && o.release >= prevRel;
        maxYawStep = std::max(maxYawStep, std::fabs(o.yawDeg - prevYaw));
        if (framesToFull < 0 && o.release >= 1.0f)
            framesToFull = i;
        prevRel = o.release;
        prevYaw = o.yawDeg;
    }
    Check(relStart > 0.3f && relStart < 0.5f, F("5) after 0.5 s of latch the release is %.3f (0.4 expected)", relStart));
    Check(mono && maxRelStep <= kDt / motionpose::kAoReacquireSeconds + 1e-4f && framesToFull > 0,
          F("5) target returns: release climbs <= %.3f per frame (max %.3f), full after %d frames", kDt / motionpose::kAoReacquireSeconds, maxRelStep, framesToFull));
    Check(maxYawStep <= kStepMax, F("5) ... the yaw moves from the latched 50 to -20 at <= %.2f deg/frame (max %.2f)", kStepMax, maxYawStep));
    // a small difference (30 deg, below the jump size) is caught up at the rate as well when the target returns after the latch,
    // then tracking is exact again
    AimSpineYawFilter h;
    for (int i = 0; i < 30; ++i)
        Step(h, true, 1.0f, 50.0f, 0.0f, kDt, 0, o);
    for (int i = 0; i < 12; ++i)
        Step(h, false, 1.0f, 0.0f, 0.0f, kDt, 0, o);
    float ys[6];
    for (int i = 0; i < 6; ++i)
    {
        Step(h, true, 1.0f, i < 5 ? 20.0f : 18.0f, 0.0f, kDt, 0, o);
        ys[i] = o.yawDeg;
    }
    Check(std::fabs(ys[0] - 42.0f) < 1e-3f && std::fabs(ys[1] - 34.0f) < 1e-3f && std::fabs(ys[2] - 26.0f) < 1e-3f && ys[3] == 20.0f &&
              ys[5] == 18.0f,
          F("5) target returns 30 deg away after 0.2 s of latch: yaw %.1f %.1f %.1f %.1f (rate 8 deg/frame), then exact tracking %.1f", ys[0], ys[1],
            ys[2], ys[3], ys[5]));
}

static void TestPitch()
{
    // the sweep pitch is as continuous as the yaw: a target that returns after the latch catches up at kAoPitchSlewDegPerSec
    const float pitchStep = motionpose::kAoPitchSlewDegPerSec * kDt + 1e-3f;
    AimSpineYawFilter f;
    AimSpineYawFilter::Output o;
    for (int i = 0; i < 10; ++i)
        Step(f, true, 1.0f, 0.0f, -60.0f, kDt, 0, o);
    for (int i = 0; i < 12; ++i) // 0.2 s of latch: the camera moves on, the pitch stays at -60
    {
        Step(f, false, 1.0f, 0.0f, 0.0f, kDt, 0, o);
    }
    const bool frozen = o.pitchDeg == -60.0f;
    float prev = o.pitchDeg, maxStep = 0.0f;
    int framesToZero = -1;
    for (int i = 1; i <= 30; ++i)
    {
        Step(f, true, 1.0f, 0.0f, 0.0f, kDt, 0, o);
        maxStep = std::max(maxStep, std::fabs(o.pitchDeg - prev));
        prev = o.pitchDeg;
        if (framesToZero < 0 && o.pitchDeg == 0.0f)
            framesToZero = i;
    }
    Check(frozen && maxStep <= pitchStep && framesToZero >= 15 && framesToZero <= 16,
          F("8) target returns at pitch 0 after 0.2 s of latch at -60: max pitch step %.2f deg/frame <= %.2f (was 60), reaches 0 after %d frames", maxStep,
            pitchStep, framesToZero));
    // the diag review example: latched -5, the player lifts the camera 35 deg (below the jump size) and shoots again
    AimSpineYawFilter g;
    for (int i = 0; i < 10; ++i)
        Step(g, true, 1.0f, 0.0f, -5.0f, kDt, 0, o);
    for (int i = 0; i < 6; ++i)
        Step(g, false, 1.0f, 0.0f, 0.0f, kDt, 0, o);
    Step(g, true, 1.0f, 0.0f, 30.0f, kDt, 0, o);
    Check(std::fabs(o.pitchDeg - (-5.0f + motionpose::kAoPitchSlewDegPerSec * kDt)) < 1e-3f,
          F("8) latched -5, target returns at +30: the first frame is %.2f (-1 = rate limited)", o.pitchDeg));
    // fast pitch tracking without a latch is exact (a 30 deg per frame flick)
    AimSpineYawFilter h;
    bool exact = true;
    const float flick[] = {0.0f, 30.0f, 60.0f, 30.0f, 0.0f, -30.0f, -60.0f};
    for (float p : flick)
    {
        Step(h, true, 1.0f, 0.0f, p, kDt, 0, o);
        exact = exact && o.pitchDeg == p;
    }
    Check(exact, "8) pitch tracking without a latch is exact (30 deg per frame)");
}

static void TestHandback()
{
    // The game owns the upper body (handback): the target is ignored and the latched angles fade out in 0.15 s, not in one frame
    // and not in the 0.35 s hold of an ordinary lost target.
    AimSpineYawFilter f;
    AimSpineYawFilter::Output o;
    for (int i = 0; i < 30; ++i)
        Step(f, true, 1.0f, 60.0f, -20.0f, kDt, 0, o);
    float prev = Applied(true, o, 1.0f), maxStep = 0.0f;
    int endFrame = -1;
    bool frozen = true;
    for (int i = 1; i <= 20; ++i)
    {
        const bool have = Step(f, true, 1.0f, -30.0f, 40.0f, kDt, 0, o, true); // a live target is ignored under a handback
        if (have)
            frozen = frozen && o.yawDeg == 60.0f && o.pitchDeg == -20.0f && !o.live;
        else if (endFrame < 0)
            endFrame = i;
        const float cur = have ? Applied(true, o, 1.0f) : 0.0f;
        maxStep = std::max(maxStep, std::fabs(cur - prev));
        prev = cur;
    }
    const float bound = 60.0f * kDt / motionpose::kAoHandbackFadeSeconds + 0.01f;
    Check(frozen && maxStep <= bound && endFrame >= 9 && endFrame <= 10,
          F("9) handback with a live target: the latched 60 deg fade at <= %.2f deg/frame (max %.2f, was 60), end after %d frames (%.2f s)", bound, maxStep,
            endFrame, static_cast<float>(endFrame) * kDt));
    // no state, nothing to fade; after the fade the next target starts the filter from the raw angles
    AimSpineYawFilter g;
    Check(!Step(g, true, 1.0f, 30.0f, 0.0f, kDt, 0, o, true) && !g.Valid(), "9) a handback without a filter state applies nothing and starts nothing");
    Check(Step(f, true, 1.0f, -30.0f, 40.0f, kDt, 0, o) && o.yawDeg == -30.0f && o.pitchDeg == 40.0f && o.release == 1.0f,
          "9) ... after the fade the next target starts the filter from the raw angles");
    // the handback ends while the latch still runs: release climbs back, the target is caught up
    AimSpineYawFilter h;
    for (int i = 0; i < 30; ++i)
        Step(h, true, 1.0f, 50.0f, 0.0f, kDt, 0, o);
    for (int i = 0; i < 3; ++i)
        Step(h, true, 1.0f, 50.0f, 0.0f, kDt, 0, o, true);
    const float relAfter = o.release;
    Step(h, true, 1.0f, 50.0f, 0.0f, kDt, 0, o);
    Check(relAfter < 1.0f && relAfter > 0.5f && o.live && o.release > relAfter && o.release - relAfter <= kDt / motionpose::kAoReacquireSeconds + 1e-4f &&
              o.yawDeg == 50.0f,
          F("9) the handback ends after 3 frames: release %.3f -> %.3f (climbs back at 1/0.15 s), yaw %.2f", relAfter, o.release, o.yawDeg));
}

static void TestResets()
{
    AimSpineYawFilter::Output o;
    AimSpineYawFilter f;
    for (int i = 0; i < 20; ++i)
        Step(f, true, 1.0f, 55.0f, 3.0f, kDt, 0, o);
    // weapon class change: the angles are kept and the new target is caught up at the rate (the old weapon yaw 55 -> -50 is a
    // 105 deg jump; a restart would set -50 at once)
    Step(f, true, 1.0f, -50.0f, 3.0f, kDt, 1, o);
    Check(std::fabs(o.yawDeg - (55.0f - motionpose::kAoYawRateDegPerSec * kDt)) < 1e-3f,
          F("6) weapon class change keeps the filter: yaw %.2f (47 = rate limited from 55; a restart gave -50 at once)", o.yawDeg));
    // same class, 100 deg jump: rate limited as well, until it arrives; then exact tracking
    int frames = 1;
    while (frames < 40 && o.yawDeg != -50.0f)
    {
        Step(f, true, 1.0f, -50.0f, 3.0f, kDt, 1, o);
        ++frames;
    }
    Check(frames == 14 && o.yawDeg == -50.0f, F("6) ... the catch-up from 55 to -50 takes %d frames (105 deg at 8 deg/frame)", frames));
    Step(f, true, 1.0f, -40.0f, 3.0f, kDt, 1, o);
    Check(o.yawDeg == -40.0f, "6) ... and the tracking is exact again");
    // a long frame (pause / hitch) behind the back keeps the side: a restart would take the side of the raw yaw (-165) at once
    AimSpineYawFilter g;
    for (int i = 0; i < 20; ++i)
        Step(g, true, 1.0f, 165.0f, 0.0f, kDt, 0, o);
    Step(g, true, 1.0f, -165.0f, 0.0f, motionpose::kAoGapSeconds, 0, o);
    Check(o.yawDeg == kClamp, F("6) a %.2f s frame behind the back keeps the side: yaw %.2f (+60; a restart gave -60, a 120 deg jump)", motionpose::kAoGapSeconds, o.yawDeg));
    // a long frame with a far target: the catch-up is limited to 1/30 s of rate (16 deg), not dt (120 deg)
    AimSpineYawFilter h;
    for (int i = 0; i < 5; ++i)
        Step(h, true, 1.0f, -60.0f, 0.0f, kDt, 0, o);
    Step(h, true, 1.0f, 60.0f, 0.0f, 0.2f, 0, o);
    Check(std::fabs(o.yawDeg - (-60.0f + motionpose::kAoMaxCatchUpSeconds * motionpose::kAoYawRateDegPerSec)) < 1e-3f,
          F("6) a 0.2 s frame moves the catch-up by at most %.0f deg: yaw %.2f", motionpose::kAoMaxCatchUpSeconds * motionpose::kAoYawRateDegPerSec, o.yawDeg));
    // a restart inside a fading latch does not end the AO in one frame
    AimSpineYawFilter l;
    for (int i = 0; i < 20; ++i)
        Step(l, true, 1.0f, 40.0f, -20.0f, kDt, 0, o);
    for (int i = 0; i < 6; ++i)
        Step(l, false, 0.8f, 0.0f, 0.0f, kDt, 0, o);
    const bool classKept = Step(l, false, 0.8f, 0.0f, 0.0f, kDt, 1, o) && o.yawDeg == 40.0f && !o.live;
    const bool gapKept = Step(l, false, 0.8f, 0.0f, 0.0f, 0.1f, 1, o) && o.yawDeg == 40.0f && o.release > 0.0f;
    Check(classKept && gapKept, F("6) a class change and a 0.1 s frame inside the latch keep fading it (yaw %.2f, release %.2f)", o.yawDeg, o.release));
    g.Reset();
    Check(!g.Valid() && !Step(g, false, 1.0f, 0.0f, 0.0f, kDt, 0, o), "6) Reset(): no state, no latch");
    // non-finite input counts as a lost target
    AimSpineYawFilter n;
    for (int i = 0; i < 5; ++i)
        Step(n, true, 1.0f, 20.0f, 0.0f, kDt, 0, o);
    const bool latched = Step(n, true, 1.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f, kDt, 0, o);
    Check(latched && !o.live && o.yawDeg == 20.0f, "6) a non-finite target is a lost target: the last angles are latched");
}

// ---- replay of the recording ------------------------------------------------------------------------------------------
struct Row
{
    int seg;
    float t, dt, yaw, pitch;
    bool have;
    int overlay;
};
static std::vector<Row> LoadCsv(const char* aPath)
{
    std::vector<Row> rows;
    std::ifstream in(aPath);
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#' || line[0] == 's')
            continue;
        std::vector<std::string> c;
        std::istringstream ls(line);
        for (std::string field; std::getline(ls, field, ',');)
            c.push_back(field);
        if (c.size() != 7)
            continue;
        Row r{};
        r.seg = std::atoi(c[0].c_str());
        r.t = std::strtof(c[1].c_str(), nullptr);
        r.dt = std::strtof(c[2].c_str(), nullptr);
        r.yaw = std::strtof(c[3].c_str(), nullptr);
        r.pitch = std::strtof(c[4].c_str(), nullptr);
        r.have = std::atoi(c[5].c_str()) != 0;
        r.overlay = std::atoi(c[6].c_str());
        rows.push_back(r);
    }
    return rows;
}

static void TestReplay(const char* aPath)
{
    const std::vector<Row> rows = LoadCsv(aPath);
    Check(rows.size() > 4000, F("7) fixture %s: %zu rows", aPath, rows.size()));
    if (rows.size() < 4000)
        return;
    // Steps are taken between consecutive frames of a segment. "target" pairs have a target in both frames (camera tracking and
    // the wrap jumps); the end of a target is a separate class (the latch).
    struct Stats
    {
        float maxStep = 0.0f, maxJumpStep = 0.0f;
        int n20 = 0, n40 = 0, nJump = 0, nLossBig = 0;
    } oldS, newS;
    int overCamera = 0; // target pairs where the filter steps more than the former logic AND more than the catch-up rate
    float flickDiff = 0.0f, calmMaxDiff = 0.0f, calmRateMax = 0.0f;
    int flickFrames = 0, segments = 0, calmSegments = 0;
    size_t i = 0;
    while (i < rows.size())
    {
        size_t j = i;
        while (j < rows.size() && rows[j].seg == rows[i].seg)
            ++j;
        ++segments;
        ov::OverlayMachine machine;
        AimSpineYawFilter filter;
        float prevOld = 0.0f, prevNew = 0.0f;
        bool prevHave = false;
        std::vector<float> oldV, newV;
        for (size_t k = i; k < j; ++k)
        {
            const Row& r = rows[k];
            ov::OverlayInputs in;
            in.overlay = static_cast<ov::PoseOverlay>(r.overlay);
            in.style = ov::Style::Feminine;
            in.wantsToAim = r.have;
            in.wantsReady = r.have;
            machine.Update(in, r.dt);
            const float a = ov::Clamp01(machine.AimingWeight());
            // former logic: clamp(raw) while a target exists, hard gate
            const float oldApplied = (r.have && a > ov::kZeroWeight) ? a * Clamp60(r.yaw) : 0.0f;
            AimSpineYawFilter::Output o;
            const bool have = Step(filter, r.have, a, r.yaw, r.pitch, r.dt, in.overlay == ov::PoseOverlay::Rifle ? 1 : 0, o);
            const float newApplied = (have && a > ov::kZeroWeight) ? Applied(true, o, a) : 0.0f;
            oldV.push_back(oldApplied);
            newV.push_back(newApplied);
            const bool flick = (r.t >= 109.40f && r.t <= 109.53f) || (r.t >= 111.45f && r.t <= 111.62f);
            if (flick && r.have)
            {
                ++flickFrames;
                flickDiff = std::max(flickDiff, std::fabs(oldApplied - newApplied));
            }
            if (k > i)
            {
                const float dOld = std::fabs(oldApplied - prevOld), dNew = std::fabs(newApplied - prevNew);
                oldS.maxStep = std::max(oldS.maxStep, dOld);
                newS.maxStep = std::max(newS.maxStep, dNew);
                oldS.n20 += dOld >= 20.0f;
                newS.n20 += dNew >= 20.0f;
                oldS.n40 += dOld >= 40.0f;
                newS.n40 += dNew >= 40.0f;
                if (r.have && prevHave)
                {
                    if (dOld >= 45.0f)
                    {
                        ++oldS.nJump;
                        oldS.maxJumpStep = std::max(oldS.maxJumpStep, dOld);
                        newS.maxJumpStep = std::max(newS.maxJumpStep, dNew);
                    }
                    newS.nJump += dNew >= 45.0f;
                    // a catch-up step is the rate limit (8 deg + the weight ramp), tracking is the former step
                    overCamera += dNew > std::max(dOld, 16.0f) + 0.01f ? 1 : 0;
                }
                else if (dNew >= 40.0f)
                    ++newS.nLossBig;
            }
            prevOld = oldApplied;
            prevNew = newApplied;
            prevHave = r.have;
        }
        float segOldMax = 0.0f, segRate = 0.0f;
        for (size_t k = 1; k < oldV.size(); ++k)
            segOldMax = std::max(segOldMax, std::fabs(oldV[k] - oldV[k - 1]));
        for (size_t k = i + 1; k < j; ++k)
            segRate = std::max(segRate, std::fabs(WrapDeg(rows[k].yaw - rows[k - 1].yaw)) / std::max(rows[k].dt, 1e-4f));
        // calm ADS: no step of the former logic >= 20 deg and a target slower than 400 deg/s
        if (segOldMax < 20.0f && segRate < 400.0f)
        {
            ++calmSegments;
            calmRateMax = std::max(calmRateMax, segRate);
            for (size_t k = 0; k < oldV.size(); ++k)
                calmMaxDiff = std::max(calmMaxDiff, std::fabs(oldV[k] - newV[k]));
        }
        i = j;
    }
    std::printf("INFO replay: %d segments (%d calm); steps >= 20 / >= 40 deg over the whole recording: former logic %d / %d (max %.1f), "
                "filter %d / %d (max %.2f); wrap jumps (target in both frames, former step >= 45): former %d (max %.1f), filter %d (max %.2f); "
                "ends of the target >= 40 deg: filter %d; flick windows 109.40-109.53 / 111.45-111.62 s: %d frames, filter vs former max %.6f deg\n",
                segments, calmSegments, oldS.n20, oldS.n40, oldS.maxStep, newS.n20, newS.n40, newS.maxStep, oldS.nJump, oldS.maxJumpStep,
                newS.nJump, newS.maxJumpStep, newS.nLossBig, flickFrames, flickDiff);
    Check(oldS.maxStep >= 100.0f && oldS.nJump >= 10 && oldS.n40 >= 25,
          F("7) fixture reproduces the defects with the former logic: max step %.1f deg, %d wrap jumps >= 45 deg, %d steps >= 40 deg (>= 100 / >= 10 / >= 25)",
            oldS.maxStep, oldS.nJump, oldS.n40));
    Check(newS.nJump == 0 && newS.maxJumpStep <= 16.0f,
          F("7) filter: the %d wrap jumps of the recording become steps <= %.2f deg/frame (<= 16 = catch-up 8 + weight ramp; none >= 45: %d)", oldS.nJump,
            newS.maxJumpStep, newS.nJump));
    Check(newS.nLossBig == 0, F("7) filter: %d steps >= 40 deg (>= 10 deg on a spine bone) where the target ends (former logic: %d >= 40 in all)", newS.nLossBig, oldS.n40));
    Check(overCamera == 0, F("7) filter: %d target pairs with a step larger than the former logic and the catch-up rate", overCamera));
    // a recording: V holds the pistol still and the mouse flicks 1000-1300 deg/s inside +-60 (hipfire shots)
    Check(flickFrames >= 12 && flickDiff == 0.0f,
          F("7) fast flicks 109.40-109.53 / 111.45-111.62 s (%d frames): filter == former logic, max difference %.6f deg (limited tracking lagged up to 64 deg)", flickFrames,
            flickDiff));
    Check(calmSegments >= 3 && calmMaxDiff == 0.0f, F("7) %d windows without a jump: filter == former logic, max difference %.6f deg (calm ADS; max target rate %.0f deg/s)", calmSegments, calmMaxDiff, calmRateMax));
}

// (C) The jump size follows the frame time: the same camera flicks at 30 / 20 fps and in a single long frame stay exact; the wrap
// jumps of the recording are still caught up. Decimated copy of the fixture: every 2nd / 3rd frame of a segment, the dt of the
// skipped frames added.
static void TestLowFps(const std::vector<Row>& aRows)
{
    for (const int decim : {2, 3})
    {
        std::vector<Row> rows;
        int lastSeg = -1, idx = 0;
        float acc = 0.0f;
        for (const Row& r : aRows)
        {
            if (r.seg != lastSeg)
            {
                lastSeg = r.seg;
                idx = 1;
                acc = 0.0f;
                Row f = r;
                f.dt = 0.0f;
                rows.push_back(f);
                continue;
            }
            acc += r.dt;
            if (idx % decim == 0)
            {
                Row f = r;
                f.dt = acc;
                acc = 0.0f;
                rows.push_back(f);
            }
            ++idx;
        }
        float flickDiff = 0.0f, flipStep = 0.0f, flickStep = 0.0f;
        int flickFrames = 0, flips = 0;
        size_t i = 0;
        while (i < rows.size())
        {
            size_t j = i;
            while (j < rows.size() && rows[j].seg == rows[i].seg)
                ++j;
            ov::OverlayMachine machine;
            AimSpineYawFilter filter;
            float prevOld = 0.0f, prevNew = 0.0f, prevYaw = 0.0f;
            bool prevHave = false;
            for (size_t k = i; k < j; ++k)
            {
                const Row& r = rows[k];
                ov::OverlayInputs in;
                in.overlay = static_cast<ov::PoseOverlay>(r.overlay);
                in.style = ov::Style::Feminine;
                in.wantsToAim = r.have;
                in.wantsReady = r.have;
                machine.Update(in, r.dt);
                const float a = ov::Clamp01(machine.AimingWeight());
                const float oldApplied = (r.have && a > ov::kZeroWeight) ? a * Clamp60(r.yaw) : 0.0f;
                AimSpineYawFilter::Output o;
                const bool have = Step(filter, r.have, a, r.yaw, r.pitch, r.dt, 0, o);
                const float newApplied = (have && a > ov::kZeroWeight) ? Applied(true, o, a) : 0.0f;
                if ((r.t >= 109.40f && r.t <= 109.53f) || (r.t >= 111.45f && r.t <= 111.62f))
                {
                    if (r.have)
                    {
                        ++flickFrames;
                        flickDiff = std::max(flickDiff, std::fabs(oldApplied - newApplied));
                        if (k > i)
                            flickStep = std::max(flickStep, std::fabs(Clamp60(r.yaw) - Clamp60(prevYaw)));
                    }
                }
                if (k > i && r.have && prevHave && std::fabs(oldApplied - prevOld) >= 100.0f)
                {
                    ++flips;
                    flipStep = std::max(flipStep, std::fabs(newApplied - prevNew));
                }
                prevOld = oldApplied;
                prevNew = newApplied;
                prevYaw = r.yaw;
                prevHave = r.have;
            }
            i = j;
        }
        // the weight ramp (60 deg in 0.2 s) adds up to 60 * dt / 0.2 per frame to the 16 deg catch-up step (1/30 s of the rate)
        const float rampBound = 60.0f * 0.05f / 0.2f;
        Check(flickFrames >= 5 && flickDiff == 0.0f,
              F("7) %d fps (every %dth frame): the %d flick frames of 109.40-109.53 / 111.45-111.62 s (camera steps up to %.0f deg/frame) are exact, max difference %.6f deg",
                60 / decim, decim, flickFrames, flickStep, flickDiff));
        Check(flips >= 4 && flipStep <= motionpose::kAoYawRateDegPerSec * motionpose::kAoMaxCatchUpSeconds + rampBound + 1.0f,
              F("7) %d fps: the %d wrap jumps (former step >= 100 deg) are still caught up, max step %.1f deg <= %.1f (catch-up 16 + weight ramp)", 60 / decim, flips,
                flipStep, motionpose::kAoYawRateDegPerSec * motionpose::kAoMaxCatchUpSeconds + rampBound + 1.0f));
    }
    // one long frame (50 ms: streaming, a shader) inside a 60 fps flick of 21.7 deg/frame: the 65 deg step is camera motion
    AimSpineYawFilter f;
    AimSpineYawFilter::Output o;
    float yaw = -60.0f;
    bool exact = true;
    for (int i = 0; i < 4; ++i)
        Step(f, true, 1.0f, yaw, 0.0f, kDt, 0, o);
    for (int i = 0; i < 8; ++i)
    {
        const float dt = i == 3 ? 0.05f : kDt;
        yaw = std::min(60.0f, yaw + 1300.0f * dt);
        Step(f, true, 1.0f, yaw, 0.0f, dt, 0, o);
        const float want = Clamp60(yaw);
        exact = exact && std::memcmp(&o.yawDeg, &want, sizeof(float)) == 0;
    }
    Check(exact, "7) a single 50 ms frame inside a 1300 deg/s flick (65 deg step): the output stays clamp(raw)");
    // a side change recognised on its own: at 10 fps (jump size capped at 90 deg) the camera crosses the back from yaw +20 to the other
    // side in one frame, the clamp swing is 80 deg, below the cap; the side change itself starts the catch-up (16 deg), it is not set at once
    AimSpineYawFilter g;
    for (int i = 0; i < 20; ++i)
        Step(g, true, 1.0f, 20.0f, 0.0f, 0.1f, 0, o); // side +, yaw +20
    Step(g, true, 1.0f, -100.0f, 0.0f, 0.1f, 0, o);   // side -, target -60
    Check(std::fabs(o.yawDeg - (20.0f - motionpose::kAoYawRateDegPerSec * motionpose::kAoMaxCatchUpSeconds)) < 1e-3f,
          F("7) 10 fps, the camera crosses the back in one frame (+20 -> -100): the 80 deg side change is caught up (yaw %.1f = 20 - 16), not set to -60 at once", o.yawDeg));
}

int main(int argc, char** argv)
{
    TestProbeFrames();
    TestCalm();
    TestWrap();
    TestLoss();
    TestReacquire();
    TestPitch();
    TestHandback();
    TestResets();
    if (argc > 1)
    {
        TestReplay(argv[1]);
        TestLowFps(LoadCsv(argv[1]));
    }
    else
        std::printf("[SKIP] 7) replay: pass tests/data/recording_1215_aim_offset_replay.csv as argv1\n");
    std::printf("aospineyaw: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
