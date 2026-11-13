// PoseStepLimiter (src/mm/PoseStepLimit.hpp): rotation step cap of a published pose, bike jump-out (recording 2026-10-28 05-03-09).
// The MM start clip played in mid-air gives leg steps of 15 / 21 / 31 deg per 60 Hz frame (jump 2, frames 5-7); the cap holds every bone to
// <= 14.5 deg per frame for the window and never lets a bone fall behind for good. Built with -DUTPP_STEPLIMIT_MUTATION=1 (no cap) this test
// MUST report FAIL lines.
#include "../src/mm/PoseStepLimit.hpp"
#include <cmath>
#include <cstdio>
#include <string>

using namespace utpp::mm;

namespace
{
int passed = 0, failed = 0;
void Check(const std::string& name, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name.c_str());
    ok ? ++passed : ++failed;
}
void SetRot(float* aBone, float aDeg, int aAxis)
{
    const float h = 0.5f * aDeg * 3.14159265358979f / 180.0f;
    aBone[3] = aBone[4] = aBone[5] = 0.0f;
    aBone[6] = std::cos(h);
    aBone[3 + aAxis] = std::sin(h);
}
float AngleDeg(const float* a, const float* b)
{
    const float d = std::fabs(a[3] * b[3] + a[4] * b[4] + a[5] * b[5] + a[6] * b[6]);
    return 2.0f * std::acos(std::fmin(1.0f, d)) * 180.0f / 3.14159265358979f;
}
} // namespace

int main()
{
    const float dt = 1.0f / 60.0f;
    // the recorded right-leg rotation (per 60 Hz frame +9 / +15 / +21 / +31 / +14 deg: 90 deg in 5 frames), then still
    const float target[] = {0, 0, 5, 20, 41, 72, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86};
    {
        PoseStepLimiter lim;
        float pose[4 * 7] = {};
        float prev[7] = {};
        float maxStep = 0.0f, last = 0.0f;
        bool caught = false;
        for (int i = 0; i < 20; ++i)
        {
            for (int b = 0; b < 4; ++b)
                SetRot(pose + b * 7, b == 2 ? target[i] : 0.0f, 0);
            lim.Apply(pose, 1, 4, dt, kJumpOutSlewDegPerS, true);
            float* leg = pose + 2 * 7;
            if (i > 0)
                maxStep = std::fmax(maxStep, AngleDeg(prev, leg));
            for (int k = 0; k < 7; ++k)
                prev[k] = leg[k];
            last = AngleDeg(pose + 0 * 7, leg);
            caught = caught || (i >= 12 && std::fabs(last - 86.0f) < 0.5f);
        }
        Check("steps: a 31 deg per frame leg kick is held to <= 14.5 deg per frame", maxStep <= 14.51f);
        Check("steps: the bone catches up with the clip once it slows down (86 deg reached by frame 12-19)", caught && std::fabs(last - 86.0f) < 0.5f);
        Check("steps: bones 1 and 3 (not moving) stay exactly where they are",
              AngleDeg(pose + 1 * 7, pose + 0 * 7) < 0.1f && AngleDeg(pose + 3 * 7, pose + 0 * 7) < 0.1f);
        Check("steps: the cap acted (limited bone count > 0)", lim.LimitedBones() > 0);
    }
    {
        // inactive: the pose is untouched and no history is kept (the first active call starts from its own input - no pull to a stale pose)
        PoseStepLimiter lim;
        float a[2 * 7] = {};
        SetRot(a + 7, 0.0f, 0);
        lim.Apply(a, 1, 2, dt, kJumpOutSlewDegPerS, true);
        float b[2 * 7] = {};
        SetRot(b + 7, 90.0f, 0);
        const uint32_t n0 = lim.Apply(b, 1, 2, dt, kJumpOutSlewDegPerS, false);
        const float ref[7] = {0, 0, 0, std::sin(0.7853982f), 0, 0, std::cos(0.7853982f)};
        const float untouched = AngleDeg(b + 7, ref);
        float c[2 * 7] = {};
        SetRot(c + 7, 90.0f, 0);
        const uint32_t n1 = lim.Apply(c, 1, 2, dt, kJumpOutSlewDegPerS, true);
        Check("inactive: nothing limited, pose untouched, the next active call has no history (no stale pull)",
              n0 == 0 && untouched < 0.1f && n1 == 0 && AngleDeg(c + 7, b + 7) < 0.1f);
    }
    {
        // sign flips of the quaternion do not read as a half turn; a long frame is still capped at 14.5 deg, a 144 Hz frame at ~5.8 deg
        PoseStepLimiter lim;
        float a[2 * 7] = {};
        SetRot(a + 7, 10.0f, 1);
        lim.Apply(a, 1, 2, dt, kJumpOutSlewDegPerS, true);
        float b[2 * 7] = {};
        SetRot(b + 7, 12.0f, 1);
        for (int k = 3; k < 7; ++k)
            b[7 + k] = -b[7 + k];
        lim.Apply(b, 1, 2, dt, kJumpOutSlewDegPerS, true);
        Check("sign: a negated quaternion of a 2 deg step is not limited", AngleDeg(b + 7, a + 7) < 2.5f && lim.LimitedBones() == 0);
        float c[2 * 7] = {};
        SetRot(c + 7, 100.0f, 1);
        lim.Apply(c, 1, 2, 0.05f, kJumpOutSlewDegPerS, true); // a 50 ms frame: 840 x 0.05 = 42 deg, clamped to 14.5
        Check("dt: a 50 ms frame is capped at 14.5 deg", AngleDeg(c + 7, b + 7) <= 14.51f);
        float d[2 * 7] = {};
        SetRot(d + 7, 100.0f, 1);
        lim.Apply(d, 1, 2, 1.0f / 144.0f, kJumpOutSlewDegPerS, true);
        Check("dt: a 144 Hz frame is capped near 5.8 deg", AngleDeg(d + 7, c + 7) <= 5.9f && AngleDeg(d + 7, c + 7) >= 5.0f);
    }
    {
        // the lag is reported (a driver keeps the cap past its window while a bone still lags) and Restart drops the history
        PoseStepLimiter lim;
        float a[2 * 7] = {};
        SetRot(a + 7, 0.0f, 0);
        lim.Apply(a, 1, 2, dt, kJumpOutSlewDegPerS, true);
        float b[2 * 7] = {};
        SetRot(b + 7, 60.0f, 0);
        lim.Apply(b, 1, 2, dt, kJumpOutSlewDegPerS, true);
        const float lag1 = lim.LagRad() * 180.0f / 3.14159265f;
        int frames = 0;
        for (; frames < 12 && lim.LagRad() > 0.01f; ++frames)
        {
            float c[2 * 7] = {};
            SetRot(c + 7, 60.0f, 0);
            lim.Apply(c, 1, 2, dt, kJumpOutSlewDegPerS, true);
        }
        Check("lag: 60 deg step - lag 45.5 deg after the first frame, gone after 3-4 more frames (no one-frame release)",
              lag1 > 44.0f && lag1 < 47.0f && frames >= 3 && frames <= 4 && lim.LagRad() <= 0.01f);
        float d[2 * 7] = {};
        SetRot(d + 7, 10.0f, 0);
        lim.Apply(d, 1, 2, dt, kJumpOutSlewDegPerS, true);
        lim.Restart();
        float e[2 * 7] = {};
        SetRot(e + 7, 90.0f, 0);
        const uint32_t n = lim.Apply(e, 1, 2, dt, kJumpOutSlewDegPerS, true);
        Check("restart: after Restart the next pose is taken as it is (the hand-in start moves the visible pose)", n == 0 && lim.LagRad() == 0.0f && AngleDeg(e + 7, a + 7) > 89.0f);
    }
    std::printf("pose step limit: %d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}
