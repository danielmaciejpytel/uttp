// P4 (recording 2026-10-21 12:15): a reload must not reveal the yaw offset the native root has hidden.
//
// Recorded chain: the capsule is turned by the camera while V stands (offset root absorbs it: the visible body stays), then a
// reload starts, mm::rootframe sees montage && !aimFacing -> Release: the offset decays with half-life 0.1 s and no rate limit,
// so the visible V spins 100-180 deg in ~0.3 s without footwork (measured: 111.6 / 99.3 / 116.8 deg, 819 / 663 / 783 deg/s at
// 83.1 / 88.4 / 95.0 s). tests/data/recording_1215_reload_root.inc holds, per pose frame of those windows, the inputs the root saw
// and the MEASURED offset (Hips yaw final minus game pose). The pre-fix policy is kept here as an oracle: replayed through the
// real motionRoot::OffsetRoot (unbounded release = the motion contract) it must reproduce the recording; the production policy
// (mm/RootMontage.hpp: RootMontage + ReloadRootLatch) must not spin V.
//
// Second chain (review of 4bdffd6): a reload that began on the spot and then V moves. The first moving frame made it montage and
// released the whole offset (~770 deg/s, 102 deg in 0.3 s for 117 deg). The recording has no such case with a large offset (0 of 20
// stationary reloads), so the window is synthetic: the recorded offset (117 deg), 0.5 s of standing reload, then movement.
//
// This test drives the montage policy only (Step::restRecenter stays off; the rest recentering has tests/RestOffsetCollapseTest.cpp).
// Compiled with -DUTPP_ROOT_MONTAGE_MUTATION=1 (every reload is montage) or =2 (no began-moving latch) it MUST report FAIL lines.
#include "../src/mm/MotionOffsetRoot.hpp"
#include "../src/mm/RootMontage.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace utpp::mm;
using motionRoot::OffsetRoot;
using motionRoot::Step;

namespace
{
struct RootFrame
{
    float dt, capDeg, animDeg, reloadW, offsetDeg;
    int ads, moving;
};
struct RootEpisode
{
    const char *name;
    int first, count, edge;
    float seedDeg;
    int control;
};
#include "data/recording_1215_reload_root.inc"

constexpr float kDeg = math::kPi / 180.f;
int passed = 0, failed = 0;
void Check(const char *name, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
    ok ? ++passed : ++failed;
}
float WrapDeg(float d)
{
    return float(std::remainder(double(d), 360.0));
}

// Pre-4bdffd6: every reload (reload upper weight > 0) is montage, standing or not.
bool LegacyMontage(bool weaponOverlay, bool aimGameUpper, bool aimingNow, float reloadW, bool /*movingGround*/, bool /*began*/)
{
    return weaponOverlay && ((aimGameUpper && aimingNow) || reloadW > 0);
}
// 4bdffd6: a reload is montage while V moves (also one that began on the spot).
bool MovingMontage(bool weaponOverlay, bool aimGameUpper, bool aimingNow, float reloadW, bool movingGround, bool /*began*/)
{
    return weaponOverlay && ((aimGameUpper && aimingNow) || (reloadW > 0 && movingGround));
}
using Policy = bool (*)(bool, bool, bool, float, bool, bool);

struct Replay
{
    std::vector<float> time, visibleDeg, modelDeg, measuredDeg;
    std::vector<motionRoot::State> state;
    std::vector<char> montage;
};
// One frame of the root as MotionDriver feeds it (aimFacing = ADS; the reload clears the hipfire / shot-turn facing).
struct Frame
{
    float dt, capDeg, animDeg, reloadW;
    bool ads, moving;
};
class Rig
{
  public:
    Rig(Policy policy, bool unbounded, float seedDeg, bool control = false) : m_policy(policy), m_unbounded(unbounded), m_control(control)
    {
        m_root.Seed(0, 0, seedDeg * kDeg);
    }
    void Feed(const Frame &f, float measuredDeg = 0)
    {
        Step s;
        s.dt = f.dt;
        s.capsuleYawDelta = f.capDeg * kDeg;
        s.animationYawDelta = f.animDeg * kDeg;
        s.movingGround = f.moving;
        const bool began = m_latch.Step(f.reloadW, f.moving);
        s.montage = m_policy(true, true, f.ads, f.reloadW, f.moving, began);
        s.aimFacing = f.ads;
        if (m_unbounded)
            s.releaseMaxRate = 0; // the motion contract
        if (m_control)
        {
            // running: the capsule moves 3.3 m/s forward and the authored root follows it
            s.capsuleDeltaCurrent = {0, 3.3f * f.dt, 0};
            s.animationDeltaVisual = {0, 3.3f * f.dt, 0};
        }
        m_root.Update(s);
        m_actor += s.capsuleYawDelta;
        m_t += f.dt;
        r.time.push_back(m_t);
        r.visibleDeg.push_back(WrapDeg((m_actor + m_root.Yaw()) / kDeg));
        r.modelDeg.push_back(m_root.Yaw() / kDeg);
        r.measuredDeg.push_back(measuredDeg);
        r.state.push_back(m_root.Output());
        r.montage.push_back(s.montage);
    }
    Replay r;

  private:
    Policy m_policy;
    bool m_unbounded, m_control;
    OffsetRoot m_root;
    rootframe::ReloadRootLatch m_latch;
    float m_actor = 0, m_t = 0;
};
Replay Run(const RootEpisode &e, Policy policy, bool unbounded = false)
{
    Rig rig(policy, unbounded, e.seedDeg, e.control != 0);
    for (int i = 0; i < e.count; ++i)
    {
        const RootFrame &f = kRootFrames[e.first + i];
        rig.Feed({f.dt, f.capDeg, f.animDeg, f.reloadW, f.ads != 0, f.moving != 0}, f.offsetDeg);
    }
    return rig.r;
}
// Visible yaw change within `window` s after frame `from` and its peak rate (3-frame smoothing), deg and deg/s.
void Visible(const Replay &r, int from, float window, float &change, float &peakRate)
{
    change = peakRate = 0;
    const float ref = r.visibleDeg[size_t(from) - 1], t0 = r.time[size_t(from) - 1];
    for (size_t k = size_t(from); k < r.time.size() && r.time[k] - t0 <= window; ++k)
    {
        change = std::fmax(change, std::fabs(WrapDeg(r.visibleDeg[k] - ref)));
        if (k >= 3)
            peakRate = std::fmax(peakRate, std::fabs(WrapDeg(r.visibleDeg[k] - r.visibleDeg[k - 3])) / (r.time[k] - r.time[k - 3]));
    }
}
} // namespace

int main()
{
    // ---- policy truth table. Reference policies: pre-4bdffd6 (every reload montage) and 4bdffd6 (montage while V moves).
    {
        bool onlyDiffers = true, standingHeld = true, beganMovingKept = true, beganStandingHeld = true, noWeaponNeverMontage = true;
        int differing = 0, differingPrev = 0;
        for (int bits = 0; bits < 32; ++bits)
            for (float rw : {0.f, 0.05f, 1.f})
            {
                const bool overlay = bits & 1, gameUpper = bits & 2, aiming = bits & 4, moving = bits & 8, began = bits & 16;
                const bool now = rootframe::RootMontage(overlay, gameUpper, aiming, rw, moving, began);
                const bool legacy = LegacyMontage(overlay, gameUpper, aiming, rw, moving, began);
                const bool prev = MovingMontage(overlay, gameUpper, aiming, rw, moving, began);
                const bool aimMontage = overlay && gameUpper && aiming;
                const bool reload = overlay && rw > 0 && !aimMontage;
                differing += now != legacy;
                differingPrev += now != prev;
                // differs from the pre-4bdffd6 policy exactly when the reload is not (moving && began moving)
                onlyDiffers &= (now != legacy) == (reload && !(moving && began));
                // differs from 4bdffd6 exactly for a reload that began on the spot and now moves
                onlyDiffers &= (now != prev) == (reload && moving && !began);
                standingHeld &= !(reload && !moving) || !now;
                beganMovingKept &= !(reload && moving && began) || now;
                beganStandingHeld &= !(reload && moving && !began) || !now;
                noWeaponNeverMontage &= overlay || !now;
            }
        Check("policy: differs from the pre-4bdffd6 and 4bdffd6 policies only where intended", onlyDiffers && differing > 0 && differingPrev > 0);
        Check("policy: a reload on the spot is not montage (the offset is carried)", standingHeld);
        Check("policy: a reload that began on the spot stays out of montage when V moves", beganStandingHeld);
        Check("policy: a reload that began in motion keeps the montage policy while V moves", beganMovingKept);
        Check("policy: ADS montage and an ADS reload are unchanged (aimGameUpper && aimingNow)",
              rootframe::RootMontage(true, true, true, 0.f, false) && rootframe::RootMontage(true, true, true, 1.f, false, false) &&
                  !rootframe::RootMontage(true, false, true, 0.f, false));
        Check("policy: no weapon overlay never montage", noWeaponNeverMontage);
        rootframe::ReloadRootLatch l;
        bool ok = !l.Step(0.f, true) && l.Step(0.3f, true) && l.Step(1.f, false) && !l.Step(0.f, false) && !l.Step(0.2f, false) &&
                  !l.Step(1.f, true) && !l.Step(-1.f, true) && l.Step(0.5f, true) && l.Step(0.5f, false);
        Check("latch: takes the movement state of the first reload frame, resets when the reload weight is 0", ok);
    }

    // ---- recorded stationary reloads
    for (const RootEpisode &e : kRootEpisodes)
    {
        if (e.control)
            continue;
        const Replay legacy = Run(e, LegacyMontage, true), now = Run(e, rootframe::RootMontage);
        float lc, lr, nc, nr;
        Visible(legacy, e.edge, 0.6f, lc, lr);
        Visible(now, e.edge, 0.6f, nc, nr);
        float err = 0, offsetAtEdge = std::fabs(legacy.measuredDeg[size_t(e.edge) - 1]);
        for (size_t k = 0; k < legacy.modelDeg.size(); ++k)
            err = std::fmax(err, std::fabs(WrapDeg(legacy.modelDeg[k] - legacy.measuredDeg[k])));
        std::printf("%s: offset before the reload %.1f deg | pre-fix replay: visible turn %.1f deg in 0.6 s, peak %.0f deg/s, model vs "
                    "measurement max %.2f deg | fixed: %.2f deg, peak %.1f deg/s\n",
                    e.name, offsetAtEdge, lc, lr, err, nc, nr);
        char n[256];
        std::snprintf(n, sizeof n, "%s: the recorded offset before the reload is large (>= 95 deg)", e.name);
        Check(n, offsetAtEdge >= 95.f);
        std::snprintf(n, sizeof n, "%s: pre-fix policy replay reproduces the recording (model within 5 deg of the measured offset)", e.name);
        Check(n, err <= 5.f);
        std::snprintf(n, sizeof n, "%s: pre-fix policy replay spins the visible V >= 90 deg at >= 400 deg/s (the defect)", e.name);
        Check(n, lc >= 90.f && lr >= 400.f);
        std::snprintf(n, sizeof n, "%s: reload on the spot changes the visible yaw by <= 5 deg in 0.6 s", e.name);
        Check(n, nc <= 5.f);
        std::snprintf(n, sizeof n, "%s: reload on the spot never turns V faster than 60 deg/s", e.name);
        Check(n, nr <= 60.f);
        bool sameBefore = true;
        for (int k = 0; k < e.edge; ++k)
            sameBefore &= std::memcmp(&legacy.state[size_t(k)], &now.state[size_t(k)], sizeof(motionRoot::State)) == 0;
        std::snprintf(n, sizeof n, "%s: frames before the reload (shot, ADS exit, authored turn) are bit-identical to the pre-fix policy", e.name);
        Check(n, sameBefore);
        // the offset is carried, not lost: it is still there (and equal) at the end of the recorded reload window
        std::snprintf(n, sizeof n, "%s: the hidden offset is kept through the window (|offset| >= 95 deg at the end)", e.name);
        Check(n, std::fabs(now.modelDeg.back()) >= 95.f);
    }

    // ---- recorded reload in motion (354.076, 3.3 m/s): decisions and root states unchanged
    for (const RootEpisode &e : kRootEpisodes)
    {
        if (!e.control)
            continue;
        const Replay legacy = Run(e, LegacyMontage, true), now = Run(e, rootframe::RootMontage);
        // V runs into the reload and stops during it (recorded: 3.3 m/s at the edge). While V moves nothing may change; the
        // frames after it stopped are the reload on the spot (held offset instead of a release of the small residual).
        int reloadMoving = 0, firstStopped = e.count;
        bool identical = true;
        for (int i = 0; i < e.count; ++i)
        {
            const RootFrame &f = kRootFrames[e.first + i];
            if (i >= e.edge && !f.moving && firstStopped == e.count)
                firstStopped = i;
            if (i >= firstStopped)
                continue;
            reloadMoving += f.reloadW > 0 && f.moving;
            identical &= now.montage[size_t(i)] == legacy.montage[size_t(i)];
            identical &= std::memcmp(&legacy.state[size_t(i)], &now.state[size_t(i)], sizeof(motionRoot::State)) == 0;
        }
        float residual = 0, apart = 0;
        for (int i = firstStopped; i < e.count; ++i)
        {
            residual = std::fmax(residual, std::fabs(now.modelDeg[size_t(i)]));
            apart = std::fmax(apart, std::fabs(WrapDeg(now.visibleDeg[size_t(i)] - legacy.visibleDeg[size_t(i)])));
        }
        std::printf("%s: moving reload frames %d, V stopped at frame %d of %d, residual offset after the stop %.2f deg, visible yaw "
                    "difference to the pre-fix policy %.2f deg\n", e.name, reloadMoving, firstStopped, e.count, residual, apart);
        char n[256];
        std::snprintf(n, sizeof n, "%s: the window contains the reload edge while moving (%d frames)", e.name, reloadMoving);
        Check(n, reloadMoving >= 5);
        std::snprintf(n, sizeof n, "%s: while V moves, montage, translationRelease and rotationRelease equal the pre-fix policy", e.name);
        Check(n, identical);
        std::snprintf(n, sizeof n, "%s: after V stopped the small residual offset (%.1f deg) is held, <= 5 deg from the pre-fix result", e.name, residual);
        Check(n, residual <= 5.f && apart <= 5.f);
    }

    // ---- reload on the spot, then V moves (synthetic around the recorded offset): 0.5 s standing reload, 0.8 s moving
    for (int hz : {60, 144})
        for (float seed : {-117.f, 117.f, 176.f})
        {
            const float dt = 1.f / float(hz);
            auto run = [&](Policy policy, bool unbounded) {
                Rig rig(policy, unbounded, seed);
                const int standing = int(std::lround(0.5f * hz)), moving = int(std::lround(0.8f * hz));
                for (int i = 0; i < standing; ++i)
                    rig.Feed({dt, 0, 0, 1.f, false, false});
                for (int i = 0; i < moving; ++i)
                    rig.Feed({dt, 0, 0, 1.f, false, true});
                return rig.r;
            };
            const int first = int(std::lround(0.5f * hz)); // first moving frame
            const Replay prev = run(MovingMontage, true), now = run(rootframe::RootMontage, false);
            float pc, pr, nc, nr;
            Visible(prev, first, 0.3f, pc, pr);
            Visible(now, first, 0.3f, nc, nr);
            bool noRelease = true;
            for (size_t k = size_t(first); k < now.state.size(); ++k)
                noRelease &= !now.state[k].rotationRelease;
            std::printf("reload on the spot then moving, offset %.0f deg, %d Hz: 4bdffd6 %.1f deg in 0.3 s (peak %.0f deg/s) | now %.2f deg "
                        "(peak %.1f deg/s)\n", seed, hz, pc, pr, nc, nr);
            char n[256];
            std::snprintf(n, sizeof n, "reload on the spot then moving (%.0f deg, %d Hz): 4bdffd6 sheds the offset at the first moving frame (the defect)", seed, hz);
            Check(n, pc >= 80.f && pr >= 600.f);
            std::snprintf(n, sizeof n, "reload on the spot then moving (%.0f deg, %d Hz): no rotation release in any moving frame", seed, hz);
            Check(n, noRelease);
            std::snprintf(n, sizeof n, "reload on the spot then moving (%.0f deg, %d Hz): the visible yaw changes <= 5 deg in 0.3 s after V starts", seed, hz);
            Check(n, nc <= 5.f);
        }
    std::printf("reload root hold: %d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}
