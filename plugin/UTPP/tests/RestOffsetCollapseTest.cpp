// P4 follow-up (review of 4bdffd6 / 16d03c3, recording 2026-10-21 12:15): MotionOffsetRoot's rest recentering, the FALLBACK of the
// rest turn-in-place.
//
// With the offset carried through the reload (no spin at the reload edge) 18 of the 20 stationary reloads with |offset| > 45 deg
// were followed by ADS or a shot within 0.7 s of the reload end: V stood 85-176 deg off the camera. The primary mechanism is the
// matcher's authored turn-in-place (MotionAdapter restTurn, real-host cases in CameraTargetRootTest.cpp: TIP chosen at once, every degree
// authored, legs step). This test replays the root alone, i.e. what happens when no turn is chosen: nothing for 0.5 s, then the
// part of the offset beyond the 40 deg knee is folded back at <= 120 deg/s (a turn without footwork: the feet slide about r * angle).
// tests/data/recording_1215_rest_offset.inc: the 20 recorded reload windows (reload edge .. first ADS / Shoot / movement) with the
// recorded dt, capsule yaw delta, reload / ADS / moving flags, the measured offset before the reload and the camera / capsule yaw
// at the event. The authored yaw is 0 (Idle loop; measured <= 3 deg/s). Compiled with -DUTPP_ROOT_REST_MUTATION=1 (no recentering)
// this test MUST report FAIL lines.
#include "../src/mm/MotionOffsetRoot.hpp"
#include "../src/mm/RootMontage.hpp"
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace utpp::mm;
using motionRoot::OffsetRoot;
using motionRoot::Step;

namespace
{
struct RestFrame
{
    float dt, capDeg;
    int flags; // 1 reload upper weight > 0, 2 ADS, 4 moving
};
struct RestEpisode
{
    const char *name;
    int first, count;
    float offsetDeg, capEventDeg, camEventDeg;
    int event; // 1 ADS, 2 shot, 3 move
    float preVisToCamDeg, heldVisToCamDeg;
};
#include "data/recording_1215_rest_offset.inc"

constexpr float kDeg = math::kPi / 180.f;
int passed = 0, failed = 0;
void Check(const std::string &name, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name.c_str());
    ok ? ++passed : ++failed;
}
float WrapDeg(float d)
{
    return float(std::remainder(double(d), 360.0));
}
struct Result
{
    std::vector<float> time, visible;
    float offsetEvent = 0, visToCamera = 0, peakRate = 0, change05 = 0, change08 = 0, capMoved = 0, windowSeconds = 0;
    bool releaseOnMove = false;
};
enum class Mode
{
    Production,  // RootMontage + latch + rest recentering (what MotionDriver feeds)
    Held,        // 4bdffd6: offset carried, no rest recentering
    Legacy       // pre-4bdffd6: every reload is montage, unbounded release
};
Result Run(const RestEpisode &e, Mode mode)
{
    Result r;
    OffsetRoot root;
    root.Seed(0, 0, e.offsetDeg * kDeg);
    rootframe::ReloadRootLatch latch;
    float actor = 0, t = 0, capSum = 0;
    for (int i = 0; i < e.count; ++i)
    {
        const RestFrame &f = kRestFrames[e.first + i];
        const bool reload = f.flags & 1, ads = f.flags & 2, moving = f.flags & 4;
        Step s;
        s.dt = f.dt;
        s.capsuleYawDelta = f.capDeg * kDeg;
        s.movingGround = moving;
        s.aimFacing = ads;
        const float w = reload ? 1.f : 0.f;
        const bool began = latch.Step(w, moving);
        if (mode == Mode::Legacy)
        {
            s.montage = (ads) || reload; // pre-4bdffd6: aimGameUpper && aimingNow || reload upper weight
            s.releaseMaxRate = 0;
        }
        else
            s.montage = rootframe::RootMontage(true, true, ads, w, moving, began);
        s.restRecenter = mode == Mode::Production;
        root.Update(s);
        actor += s.capsuleYawDelta;
        capSum += std::fabs(f.capDeg);
        t += f.dt;
        r.time.push_back(t);
        r.visible.push_back(WrapDeg((actor + root.Yaw()) / kDeg));
        if (moving && root.Output().rotationRelease && !ads)
            r.releaseOnMove = true;
    }
    r.windowSeconds = t;
    r.offsetEvent = root.Yaw() / kDeg;
    r.capMoved = capSum;
    // visible body relative to the camera at the event (capsule yaw at the event + remaining offset - camera)
    r.visToCamera = std::fabs(WrapDeg(e.capEventDeg + r.offsetEvent - e.camEventDeg));
    const float ref0 = r.visible.empty() ? 0 : WrapDeg(r.visible[0]);
    for (size_t k = 3; k < r.time.size(); ++k)
    {
        r.peakRate = std::fmax(r.peakRate, std::fabs(WrapDeg(r.visible[k] - r.visible[k - 3])) / (r.time[k] - r.time[k - 3]));
        if (r.time[k] < 0.5f - 0.02f)
            r.change05 = std::fmax(r.change05, std::fabs(WrapDeg(r.visible[k] - ref0)));
        if (r.time[k] <= 0.8f)
            r.change08 = std::fmax(r.change08, std::fabs(WrapDeg(r.visible[k] - ref0)));
    }
    return r;
}
} // namespace

int main()
{
    int longWindows = 0, longFolded = 0, camAligned = 0, camAlignedBetter = 0, largeOffsets = 0, largeSpun = 0;
    float sumNew = 0, sumHeld = 0, worstPeak = 0, worstChange = 0;
    std::printf("episode | offset | event | window s | pre-4bdffd6 / carried / now : visible-to-camera at the event | offset at the event | "
                "peak deg/s now (pre-4bdffd6) | capsule-to-camera at the event\n");
    for (const RestEpisode &e : kRestEpisodes)
    {
        const Result now = Run(e, Mode::Production), held = Run(e, Mode::Held), legacy = Run(e, Mode::Legacy);
        const float capToCamera = std::fabs(WrapDeg(e.capEventDeg - e.camEventDeg));
        std::printf("%s | %5.0f | %d | %.2f | %5.0f / %5.0f / %5.0f | %5.1f | %5.1f (%.0f) | %.0f deg, capsule moved %.1f deg\n", e.name,
                    e.offsetDeg, e.event, now.windowSeconds, e.preVisToCamDeg, held.visToCamera, now.visToCamera, now.offsetEvent,
                    now.peakRate, legacy.peakRate, capToCamera, now.capMoved);
        worstPeak = std::fmax(worstPeak, now.peakRate);
        worstChange = std::fmax(worstChange, now.change08);
        sumNew += now.visToCamera;
        sumHeld += held.visToCamera;
        const bool still = now.capMoved < 10.f;
        if (std::fabs(e.offsetDeg) > 60.f && now.windowSeconds >= 2.6f && still)
        {
            ++longWindows;
            longFolded += std::fabs(now.offsetEvent) <= 55.f;
        }
        // the capsule is where the camera is (hipfire lease): V ends within ~60 deg of the camera instead of 120-170
        if (std::fabs(e.offsetDeg) > 60.f && capToCamera <= 30.f && still && now.windowSeconds >= 2.6f)
        {
            ++camAligned;
            camAlignedBetter += now.visToCamera <= 65.f && now.visToCamera < held.visToCamera - 40.f;
        }
        if (std::fabs(e.offsetDeg) >= 90.f)
        {
            ++largeOffsets;
            largeSpun += legacy.peakRate >= 400.f;
        }
        Check(std::string(e.name) + ": rest recentering never turns the visible V faster than 125 deg/s (pre-4bdffd6 release: " +
                  std::to_string(int(legacy.peakRate)) + " deg/s)", now.peakRate <= 125.f);
        Check(std::string(e.name) + ": the root waits 0.5 s for the matcher's turn before it turns anything", now.change05 <= 0.01f);
        Check(std::string(e.name) + ": the visible V turns <= 25 deg in the first 0.8 s of the reload", now.change08 <= 25.f);
        Check(std::string(e.name) + ": no rotation release on any moving frame of the window", !now.releaseOnMove);
    }
    const int total = int(sizeof kRestEpisodes / sizeof kRestEpisodes[0]);
    std::printf("mean visible-to-camera angle at the event: now %.1f deg, offset carried %.1f deg (%d episodes; the capsule itself is "
                "not always on the camera)\n", sumNew / float(total), sumHeld / float(total), total);
    Check("the 20 recorded reload windows are all present", total == 20);
    Check("pre-4bdffd6 release spun the visible V >= 400 deg/s in every window with |offset| >= 90 deg (" + std::to_string(largeSpun) +
              " of " + std::to_string(largeOffsets) + ": the defect)", largeOffsets >= 10 && largeSpun == largeOffsets);
    Check("capsule still, |offset| > 60 deg, >= 2.6 s to the event: the offset is folded to <= 55 deg at the event (" +
              std::to_string(longFolded) + " of " + std::to_string(longWindows) + ")", longWindows >= 2 && longFolded == longWindows);
    Check("capsule on the camera, |offset| > 60 deg, >= 2.6 s: V ends <= 65 deg from the camera, >= 40 deg better than the carried offset (" +
              std::to_string(camAlignedBetter) + " of " + std::to_string(camAligned) + ")", camAligned >= 1 && camAlignedBetter == camAligned);
    Check("mean visible-to-camera angle at the event is not worse than with the carried offset", sumNew <= sumHeld);
    // ---- V starts moving in the middle of the reload / the recentering: it stops at once, nothing is released, the visible yaw stays
    for (int hz : {60, 144})
        for (float seed : {117.f, -176.f})
        {
            const float dt = 1.f / float(hz);
            OffsetRoot root;
            root.Seed(0, 0, seed * kDeg);
            rootframe::ReloadRootLatch latch;
            const int standing = int(std::lround(0.5f * hz)), moving = int(std::lround(0.8f * hz));
            float atStart = 0, drift = 0, moved = 0;
            bool release = false;
            for (int i = 0; i < standing + moving; ++i)
            {
                const bool walk = i >= standing;
                Step s;
                s.dt = dt;
                s.movingGround = walk;
                const bool began = latch.Step(1.f, walk);
                s.montage = rootframe::RootMontage(true, true, false, 1.f, walk, began);
                s.restRecenter = true;
                root.Update(s);
                if (i == standing - 1)
                    atStart = root.Yaw();
                if (walk)
                {
                    drift = std::fmax(drift, std::fabs(root.Yaw() - atStart));
                    moved += std::fabs(root.Output().restStep);
                    release = release || root.Output().rotationRelease;
                }
            }
            char n[200];
            std::snprintf(n, sizeof n, "reload on the spot (%.0f deg, %d Hz), V starts moving after 0.5 s: no release, no recentering, the offset stays (%.2f deg)", seed,
                          hz, drift / kDeg);
            Check(n, !release && moved == 0 && drift == 0.f && std::fabs(atStart) < std::fabs(seed * kDeg));
        }
    std::printf("rest offset collapse: %d PASS %d FAIL (worst peak %.0f deg/s, worst first-0.8 s change %.1f deg)\n", passed, failed, worstPeak,
                worstChange);
    return failed ? 1 : 0;
}
