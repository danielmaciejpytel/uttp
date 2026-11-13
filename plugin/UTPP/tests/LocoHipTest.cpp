// Hipfire / crouch-stop jerks of the locomotion probe 2026-10-31 (the recorded log, utpp-2026-10-31-22-55-06.log,
// diag_hip.mp4 / diag_b.mp4). The overlay machine (Overlay.hpp) and the hip-fire pose hold (HipFirePose.hpp) are driven together the
// way MotionDriver.cpp does it: HipFirePoseHold::Step on the weapon action, then UseHipFirePose -> wantsToAim + wantsReady +
// hipFireRaise for the OverlayMachine. The feed of the probe has no weapon action column, the shot schedule comes from the log
// (Shoot=8 runs of ~0.217 s, starts 22:57:36.749 and 37.758 = 1.009 s apart).
//   HIP-1  first crouched hip shot out of Relaxed: the Relaxed -> Ready -> Aiming chain (0.05 s gate + 0.1 s) moved the hands and the
//          AO alpha by 1 / 0.1 s; now one SineOut raise of 0.25 s: per-frame step <= dt * (pi / 2) / 0.25 (+0.01).
//   HIP-2  ~1 s cadence: the 0.75 s isolated hold ran out ~50 ms before the next shot (aimW 0.97 -> 0.78 -> 1.00); now the hold
//          is kIsolatedHoldS (1.2 s) and a shot inside an Aiming -> Ready fade re-raises at once.
//   CR-5   crouch walk -> stop (Pistol2H): B switched in 0.1 s against a 0.5 s body inertialization; now crouchMoveSwitch (0.3 s).
// Built with -DUTPP_LOCOHIP_MUTATION=1 (the old Relaxed chain), 2 (no re-raise from a partial Ready), 3 (moveSwitch in the crouch)
// 4 (the 0.75 s isolated hold) or 5 (a sprint request also cuts the vehicle exit-window tail) this test MUST print "FAIL: lochip" lines
// (tests/locohip.cmake).
#include "../src/mm/HipFirePose.hpp"
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace utpp::mm::ov;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat)
{
    std::printf("%s: lochip %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
    aOk ? ++g_pass : ++g_fail;
}

constexpr float kHalfPi = 1.57079633f;
constexpr float kShootRunS = 0.217f; // median rifle Shoot run (log 22:57:36.749 -> 36.968; integration test "shot")

// summed weight of the Aiming-state pose keys (the hands / arms crossfade; AimingWeight() is the AO alpha)
float AimPoseW(const OverlayMachine& aM)
{
    float w = 0.0f;
    for (int i = 0; i < aM.poses.count; ++i)
    {
        PoseOverlay ko;
        AimState ka;
        bool kc, km;
        DecodePoseKey(aM.poses.key[i], ko, ka, kc, km);
        if (ka == AimState::Aiming)
            w += aM.poses.w[i];
    }
    return w;
}

struct Sim
{
    OverlayMachine m;
    HipFirePoseHold hold;
    OverlayInputs in; // the stance / overlay of the scene; the hip pose bits are added per frame
    float prev[72] = {};
    float keyStep = 0.0f; // largest per-frame change of one pose key weight (last Step)
    float aoStep = 0.0f;  // per-frame change of the AO alpha (last Step)
    bool hip = false;

    void Snap(float aOut[72]) const
    {
        for (int k = 0; k < 72; ++k)
            aOut[k] = 0.0f;
        for (int i = 0; i < m.poses.count; ++i)
            aOut[m.poses.key[i]] = m.poses.w[i];
    }
    void Step(int aAction, float aDt, bool aSprintRequested = false)
    {
        hold.Step(aAction, aDt, true, aSprintRequested);
        hip = UseHipFirePose(false, aAction, hold.Active(), false, true, true, in.overlay, in.overlay);
        OverlayInputs x = in;
        if (hip)
        {
            x.wantsToAim = x.wantsReady = x.hipFireRaise = true;
            x.wantsToSprint = x.sprinting = false;
        }
        const float ao0 = m.AimingWeight();
        m.Update(x, aDt);
        float now[72];
        Snap(now);
        keyStep = 0.0f;
        for (int k = 0; k < 72; ++k)
            keyStep = std::max(keyStep, std::fabs(now[k] - prev[k]));
        for (int k = 0; k < 72; ++k)
            prev[k] = now[k];
        aoStep = std::fabs(m.AimingWeight() - ao0);
    }
    void Settle(float aSeconds, float aDt)
    {
        for (float t = 0.0f; t < aSeconds; t += aDt)
            Step(0, aDt);
    }
};

const char* OvName(PoseOverlay o)
{
    return o == PoseOverlay::Rifle ? "Rifle" : o == PoseOverlay::Pistol1H ? "Pistol1H" : o == PoseOverlay::Pistol2H ? "Pistol2H" : "?";
}
} // namespace

int main()
{
    for (int hz : {30, 60, 144})
    {
        const float dt = 1.0f / float(hz);
        const std::string at = " at " + std::to_string(hz) + " Hz";
        const int shotFrames = std::max(1, int(std::lround(kShootRunS * hz)));

        // ---- HIP-1: a hip shot out of the Relaxed pose (crouched: the recorded case; standing: the same path)
        for (PoseOverlay ov : {PoseOverlay::Rifle, PoseOverlay::Pistol1H, PoseOverlay::Pistol2H})
            for (bool crouch : {true, false})
            {
                Sim s;
                s.in.overlay = ov;
                s.in.crouch = crouch;
                s.Settle(1.0f, dt);
                const bool relaxedBefore = s.m.aim == AimState::Relaxed && AimPoseW(s.m) == 0.0f;
                float maxKey = 0.0f, maxAo = 0.0f, sync = 0.0f, at100ms = -1.0f, full = -1.0f;
                for (int f = 0; f < int(0.6f * hz); ++f)
                {
                    s.Step(f < shotFrames ? 8 : 0, dt);
                    const float t = float(f + 1) * dt;
                    maxKey = std::max(maxKey, s.keyStep);
                    maxAo = std::max(maxAo, s.aoStep);
                    sync = std::max(sync, std::fabs(s.m.AimingWeight() - AimPoseW(s.m)));
                    if (at100ms < 0.0f && t >= 0.1f - 1e-4f)
                        at100ms = AimPoseW(s.m);
                    if (full < 0.0f && AimPoseW(s.m) >= 0.999f && s.m.AimingWeight() >= 0.999f)
                        full = t;
                }
                const float limit = dt * kHalfPi / 0.25f + 0.01f; // the old chain: dt / 0.1 (0.33 at 30 Hz)
                const std::string tag = std::string(" (") + OvName(ov) + (crouch ? " crouch" : " stand") + ")" + at;
                std::printf("HIP-1%s: max pose key step %.3f, AO step %.3f (limit %.3f), hands at 0.1 s %.2f, full at %.3f s, sync %.4f\n",
                            tag.c_str(), double(maxKey), double(maxAo), double(limit), double(at100ms), double(full), double(sync));
                Check(relaxedBefore, "HIP-1: the scene starts in the Relaxed pose" + tag);
                Check(maxKey <= limit && maxAo <= limit,
                      "HIP-1: the raise out of Relaxed moves a pose key / the AO alpha by at most dt * (pi/2) / 0.25 per frame (" +
                          std::to_string(maxKey) + " / " + std::to_string(maxAo) + " vs " + std::to_string(limit) + ")" + tag);
                // latency: not later than the chain (~0.5 at 0.1 s), complete within the 0.25 s raise
                Check(at100ms >= 0.5f && full > 0.0f && full <= 0.25f + 1.5f * dt,
                      "HIP-1: the hands are >= 50 % up at 0.1 s and fully up by 0.25 s (" + std::to_string(at100ms) + ", " +
                          std::to_string(full) + " s)" + tag);
                Check(sync <= 0.01f, "HIP-1: the AO alpha (and the barrel loop on it) takes the raise with the hands, max diff " +
                                         std::to_string(sync) + tag);
            }
        // ---- HIP-1: an overlay change on the raise frame (Overture: 1H Relaxed -> resolved 2H Aim) keeps hands and AO alpha together
        {
            OverlayMachine m;
            OverlayInputs in;
            in.overlay = PoseOverlay::Pistol1H;
            for (int i = 0; i < hz; ++i)
                m.Update(in, dt);
            in.overlay = PoseOverlay::Pistol2H;
            in.wantsToAim = in.wantsReady = in.hipFireRaise = true;
            float sync = 0.0f;
            m.Update(in, dt);
            const bool same = m.poses.duration == m.aimW.duration && m.poses.curve == m.aimW.curve &&
                              m.poses.duration >= m.times.pistol1HTo2HAim;
            for (int i = 0; i < hz; ++i)
            {
                m.Update(in, dt);
                sync = std::max(sync, std::fabs(m.AimingWeight() - AimPoseW(m)));
            }
            Check(same && sync <= 0.01f, "HIP-1: 1H Relaxed -> 2H Aim on the shot frame: one time / curve for hands and AO alpha (sync " +
                                             std::to_string(sync) + ")" + at);
        }
        // ---- HIP-1 + P9 A: a stance / move key change inside the Relaxed raise does not restart the hands at poseSwitch
        for (PoseOverlay ov : {PoseOverlay::Rifle, PoseOverlay::Pistol2H})
            for (int kind = 1; kind <= 2; ++kind)
                for (int frame = 0; frame <= 4; ++frame)
                {
                    Sim s;
                    s.in.overlay = ov;
                    s.Settle(1.0f, dt);
                    float gap = 0.0f, maxKey = 0.0f, full = -1.0f;
                    for (int f = 0; f < int(0.6f * hz); ++f)
                    {
                        if (f == frame && kind == 1) s.in.crouch = true;
                        if (f == frame && kind == 2) s.in.moving = s.in.hasVelocity = true;
                        s.Step(f < shotFrames ? 8 : 0, dt);
                        gap = std::max(gap, s.m.AimingWeight() - AimPoseW(s.m));
                        maxKey = std::max(maxKey, s.keyStep);
                        if (full < 0.0f && AimPoseW(s.m) >= 0.999f)
                            full = float(f + 1) * dt;
                    }
                    // the key change itself may move a key by dt / (half of the P9 C raise) (the floor of the guard)
                    const float limit = std::max(dt * kHalfPi / 0.25f, dt / (0.5f * 0.1f)) + 0.01f;
                    const std::string tag = std::string(" (") + OvName(ov) + (kind == 1 ? " crouch" : " move") + " at frame " +
                                            std::to_string(frame) + ")" + at;
                    Check(full > 0.0f && full <= 0.25f + 1.5f * dt && gap <= 0.16f && maxKey <= limit,
                          "HIP-1: a key change inside the Relaxed raise keeps its end, the AO alpha <= 0.16 ahead (full " +
                              std::to_string(full) + " s, gap " + std::to_string(gap) + ", step " + std::to_string(maxKey) + ")" + tag);
                }

        // ---- HIP-2: the recorded 1.0 s cadence (and 0.6 - 1.3 s): the pose and the strafe facing stay up between the shots
        for (bool crouch : {false, true})
            for (float cadence : {0.6f, 0.8f, 1.009f, 1.2f, 1.3f})
            {
                Sim s;
                s.in.overlay = PoseOverlay::Rifle;
                s.in.moving = s.in.hasVelocity = true; // fire_stand_walk (key 21) / the crouch walk
                s.in.crouch = crouch;
                s.Settle(1.0f, dt);
                const int second = int(std::lround(cadence * hz));
                float minAo = 2.0f, minHands = 2.0f;
                bool hipThrough = true, reached = false;
                for (int f = 0; f < second + int(1.0f * hz); ++f)
                {
                    const bool shooting = f < shotFrames || (f >= second && f < second + shotFrames);
                    s.Step(shooting ? 8 : 0, dt);
                    reached = reached || (s.m.AimingWeight() >= 0.999f && AimPoseW(s.m) >= 0.999f);
                    if (reached && f <= second + int(0.5f * hz))
                    {
                        minAo = std::min(minAo, s.m.AimingWeight());
                        minHands = std::min(minHands, AimPoseW(s.m));
                        hipThrough = hipThrough && s.hip;
                    }
                }
                const std::string tag = " (cadence " + std::to_string(cadence) + " s, " + (crouch ? "crouch" : "stand") + " walk)" + at;
                std::printf("HIP-2%s: min AO alpha %.3f, min hands %.3f, hip pose through %d\n", tag.c_str(), double(minAo),
                            double(minHands), hipThrough ? 1 : 0);
                Check(reached && minAo >= 0.99f && minHands >= 0.99f && hipThrough,
                      "HIP-2: no dip of the arms / AO alpha and no facing gap between two shots (min " + std::to_string(minAo) + " / " +
                          std::to_string(minHands) + ")" + tag);
            }
        // ---- HIP-2: a shot 50 ms into the Aiming -> Ready fade (just past the isolated hold) re-raises at once
        {
            Sim s;
            s.in.overlay = PoseOverlay::Rifle;
            s.in.moving = s.in.hasVelocity = true;
            s.Settle(1.0f, dt);
            const int second = shotFrames + int(std::lround((kIsolatedHoldS + 0.05f) * hz));
            float atShot = -1.0f, minAfter = 2.0f, full = -1.0f;
            for (int f = 0; f < second + int(0.5f * hz); ++f)
            {
                const bool shooting = f < shotFrames || (f >= second && f < second + shotFrames);
                s.Step(shooting ? 8 : 0, dt);
                if (f == second - 1)
                    atShot = s.m.AimingWeight();
                if (f >= second)
                {
                    minAfter = std::min(minAfter, s.m.AimingWeight());
                    if (full < 0.0f && s.m.AimingWeight() >= 0.999f && AimPoseW(s.m) >= 0.999f)
                        full = float(f - second + 1) * dt;
                }
            }
            std::printf("HIP-2 re-raise%s: AO alpha before the shot %.3f, min after it %.3f, up again after %.3f s\n", at.c_str(),
                        double(atShot), double(minAfter), double(full));
            Check(atShot > 0.5f && atShot < 0.99f && minAfter >= atShot - 1e-4f && full > 0.0f && full <= 0.1f + 1.5f * dt,
                  "HIP-2: a hip shot inside the Aiming -> Ready fade re-raises on its first frame (before " + std::to_string(atShot) +
                      ", min after " + std::to_string(minAfter) + ", up after " + std::to_string(full) + " s)" + at);
        }
        // ---- HIP-2: the re-raise never skips the Ready gate while Relaxed weight is left (a wantsReady start just before the shot)
        {
            OverlayMachine m;
            OverlayInputs in;
            in.overlay = PoseOverlay::Rifle;
            for (int i = 0; i < hz; ++i)
                m.Update(in, dt);
            in.wantsReady = true;
            m.Update(in, dt); // Relaxed -> Ready starts (Relaxed weight ~1)
            in.wantsToAim = in.hipFireRaise = true;
            m.Update(in, dt);
            // one frame of Relaxed -> Ready (0.2 s) leaves Ready < readyWeightToAim at every rate: the shot frame must stay in Ready
            Check(m.aim == AimState::Ready, "HIP-2: with Relaxed weight left the Ready gate is kept" + at);
        }
        // ---- HIP-2: the isolated hold is kIsolatedHoldS, a sprint request keeps the former 0.75 s
        {
            HipFirePoseHold a, b;
            for (int i = 0; i < shotFrames; ++i)
            {
                a.Step(8, dt, true);
                b.Step(8, dt, true);
            }
            float heldA = 0.0f, heldB = 0.0f;
            while (a.Active() && heldA < 5.0f)
            {
                a.Step(0, dt, true);
                heldA += dt;
            }
            while (b.Active() && heldB < 5.0f)
            {
                b.Step(0, dt, true, heldB >= 0.1f); // sprint requested 0.1 s after the shot
                heldB += dt;
            }
            Check(std::fabs(heldA - 1.2f) <= dt + 1e-4f && std::fabs(heldB - OverlayTimes{}.aimingMinTime) <= dt + 1e-4f,
                  "HIP-2: an isolated shot holds 1.2 s, 0.75 s with a sprint request (" + std::to_string(heldA) + " / " +
                      std::to_string(heldB) + " s)" + at);
            // the vehicle exit window (synthetic action 8) keeps its 0.75 s tail with a sprint request, as before HIP-2
            HipFirePoseHold w;
            for (float t = 0.0f; t < 1.0f; t += dt)
                w.Step(8, dt, true, false, true);
            float tailW = 0.0f;
            while (w.Active() && tailW < 5.0f)
            {
                w.Step(0, dt, true, true);
                tailW += dt;
            }
            Check(std::fabs(tailW - OverlayTimes{}.aimingMinTime) <= dt + 1e-4f,
                  "HIP-2: the exit-window tail is not cut by a sprint request (" + std::to_string(tailW) + " s)" + at);
        }

        // ---- CR-5: crouch walk -> stop / start (Pistol2H: one crouch pose, the arms move with B only)
        for (bool stop : {true, false})
        {
            OverlayMachine m;
            OverlayInputs in;
            in.overlay = PoseOverlay::Pistol2H;
            in.crouch = true;
            in.moving = in.hasVelocity = stop;
            for (int i = 0; i < 2 * hz; ++i)
                m.Update(in, dt);
            in.moving = in.hasVelocity = !stop;
            float maxB = 0.0f, prevIdle = m.bases.WeightOf(BaseKey(true, false));
            for (int i = 0; i < hz; ++i)
            {
                m.Update(in, dt);
                const float idle = m.bases.WeightOf(BaseKey(true, false));
                maxB = std::max(maxB, std::fabs(idle - prevIdle));
                prevIdle = idle;
            }
            const float limit = dt / 0.3f + 0.005f; // moveSwitch: dt / 0.1
            std::printf("CR-5%s %s: max B step %.3f (limit %.3f)\n", at.c_str(), stop ? "stop" : "start", double(maxB), double(limit));
            Check(maxB <= limit, std::string("CR-5: crouch ") + (stop ? "stop" : "start") +
                                     ": the base pose B moves by at most dt / 0.3 s per frame (" + std::to_string(maxB) + ")" + at);
        }
        {
            // unchanged: the stand move change (0.1 s), the stance change (1.0 s) and a move change inside a stance fade (0.1 s)
            OverlayMachine m;
            OverlayInputs in;
            in.overlay = PoseOverlay::Pistol2H;
            in.moving = in.hasVelocity = true;
            for (int i = 0; i < hz; ++i)
                m.Update(in, dt);
            in.moving = in.hasVelocity = false;
            m.Update(in, dt);
            const bool stand = m.bases.duration == m.times.moveSwitch;
            for (int i = 0; i < hz; ++i)
                m.Update(in, dt);
            in.crouch = true;
            m.Update(in, dt);
            const bool stance = m.bases.duration == m.times.stanceSwitch;
            for (int i = 0; i < hz / 4; ++i)
                m.Update(in, dt);
            in.moving = in.hasVelocity = true;
            m.Update(in, dt);
            const bool inStance = m.bases.duration == m.times.moveSwitch;
            Check(stand && stance && inStance, "CR-5: stand move change, stance change and a move change inside the stance fade keep "
                                               "moveSwitch / stanceSwitch" + at);
        }
    }
    std::printf("locohip: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
