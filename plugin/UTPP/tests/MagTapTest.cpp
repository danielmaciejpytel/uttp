// Magazine top-up gesture (src/mm/MagTap.hpp), header level: the envelope at 30 / 60 / 144 Hz (bounded per-frame change, peak 1, exactly
// 0 and idle after 0.45 s), a second trigger at every frame of the gesture (no pop), the ownership fade, and the pose pass on a synthetic
// arm rig (weight 0 bit-identical, peak target reached within 1 cm, every bone outside the left arm bit-identical, the hand model rotation
// kept, forearm roll moved into the hand, bounded per-frame hand movement over the whole gesture at 30 / 60 / 144 Hz).
// Slide roll-in wait (SlideDefer) truth table and start time at 30 / 60 / 144 Hz.
// Mutations (tests/magtap.cmake, UTPP_MAGTAP_MUTATION): 1 = trigger ignored, 2 = naive restart, 4 = no slide wait; each MUST print "FAIL ".
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "../src/mm/MagTap.hpp"

namespace mm = utpp::mm;
namespace magtap = utpp::mm::magtap;
using mm::math::Quat;
using mm::math::Vec3;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const char* aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat, aDetail.empty() ? "" : " | ", aDetail.c_str());
}
std::string F(double v) { char b[32]; std::snprintf(b, sizeof b, "%.5f", v); return b; }

constexpr float kRiseSlope = 1.5f / magtap::kOutSeconds; // max d(weight)/dt of the envelope (smoothstep rise)

// root 0, spine 1, LeftArm 2, LeftForeArm 3, LeftHand 4, RightArm 5, RightForeArm 6, RightHand 7, WeaponRight 8 (child of RightHand),
// WeaponLeft 9 (child of LeftHand), l_forearm twist helper 10 (child of LeftForeArm), left finger 11 (child of LeftHand).
// CP model space: Z up, Y forward. The arms hold a weapon in front of the chest (right hand on the grip, left hand forward on it).
struct Synth
{
    mm::ov::Skeleton sk;
    mm::ov::Pose pose;
    mm::gik::Rig rig;
};
Quat AxisAngle(Vec3 a, float deg)
{
    const float h = deg * 3.14159265f / 360.0f;
    const float l = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    return Quat{a.x / l * std::sin(h), a.y / l * std::sin(h), a.z / l * std::sin(h), std::cos(h)};
}
Synth MakeSynth()
{
    Synth s;
    const int parent[12] = {-1, 0, 1, 2, 3, 1, 5, 6, 7, 4, 3, 4};
    s.sk.n = s.pose.n = 12;
    for (int i = 0; i < 12; ++i)
        s.sk.parent[i] = static_cast<int16_t>(parent[i]);
    auto set = [&](int b, Vec3 t, Quat r = Quat{}) {
        s.pose.b[b].t = t;
        s.pose.b[b].r = r;
    };
    set(0, {0, 0, 0});
    set(1, {0, 0, 1.40f}, AxisAngle({0, 0, 1}, 4.0f));
    set(2, {-0.19f, 0.0f, 0.0f}, AxisAngle({1, 0.2f, 0}, -25.0f));
    set(3, {0.075f, 0.20f, -0.275f}, AxisAngle({0, 1, 0.3f}, 35.0f)); // forearm local rotation carries some roll
    set(4, {0.125f, 0.275f, 0.09f}, AxisAngle({1, 0, 0}, 10.0f));
    set(5, {0.19f, 0.0f, 0.0f}, AxisAngle({1, -0.2f, 0}, -20.0f));
    set(6, {-0.03f, 0.14f, -0.24f});
    set(7, {-0.08f, 0.17f, 0.06f}, AxisAngle({0, 1, 0}, -12.0f));
    set(8, {0.0f, 0.03f, -0.02f}, AxisAngle({0, 0, 1}, 6.0f));
    set(9, {0.0f, 0.02f, 0.0f});
    set(10, {0.05f, 0.11f, 0.035f});
    set(11, {0.0f, 0.06f, -0.01f});
    s.rig.left = {2, 3, 4};
    s.rig.right = {5, 6, 7};
    s.rig.weaponLeft = 9;
    s.rig.weaponRight = 8;
    return s;
}
bool SameBone(const mm::ov::Xf& a, const mm::ov::Xf& b) { return std::memcmp(&a, &b, sizeof(mm::ov::Xf)) == 0; }
bool LeftArmBone(int b) { return b == 2 || b == 3 || b == 4 || b == 10; } // LeftArm, LeftForeArm and the children of LeftForeArm
Vec3 HandPos(const Synth& s, const mm::ov::Pose& p)
{
    Quat r;
    Vec3 t;
    mm::gik::ModelOf(s.sk, p, s.rig.left.hand, r, t);
    return t;
}
float Len(Vec3 v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

struct EnvRun
{
    float maxStep = 0.0f, peak = 0.0f, activeSeconds = 0.0f, minW = 0.0f, maxW = 0.0f;
    bool endsZero = false, peakAgain = false;
};
// aRetrigger: frame of a second trigger (-1 none); aLoseAt: frame from which ownership is lost (-1 never)
EnvRun RunEnvelope(float aHz, int aRetrigger, int aLoseAt = -1)
{
    EnvRun r;
    magtap::Envelope e;
    const float dt = 1.0f / aHz;
    e.Trigger();
    float last = 0.0f;
    bool retriggered = false;
    for (int i = 0; i < static_cast<int>(2.0f * aHz); ++i)
    {
        if (i == aRetrigger)
        {
            e.Trigger();
            retriggered = true;
        }
        const float w = e.Step(dt, aLoseAt < 0 || i < aLoseAt);
        r.maxStep = std::fmax(r.maxStep, std::fabs(w - last));
        r.peak = std::fmax(r.peak, w);
        r.minW = std::fmin(r.minW, w);
        r.maxW = std::fmax(r.maxW, w);
        if (retriggered && w >= 1.0f)
            r.peakAgain = true;
        if (!e.Idle())
            r.activeSeconds += dt;
        last = w;
    }
    r.endsZero = last == 0.0f && e.Idle() && e.weight == 0.0f;
    return r;
}
} // namespace

int main()
{
    // ---- envelope at 30 / 60 / 144 Hz
    for (const float hz : {30.0f, 60.0f, 144.0f})
    {
        const EnvRun r = RunEnvelope(hz, -1);
        const float dt = 1.0f / hz;
        const std::string d = std::to_string(static_cast<int>(hz)) + " Hz: peak " + F(r.peak) + " active " + F(r.activeSeconds) +
                              " s max step " + F(r.maxStep) + " (bound " + F(kRiseSlope * dt) + ")";
        Check(r.peak == 1.0f, ("envelope " + std::to_string(static_cast<int>(hz)) + " Hz: the hold reaches weight 1").c_str(), d);
        Check(r.maxStep <= kRiseSlope * dt + 1e-5f, ("envelope " + std::to_string(static_cast<int>(hz)) + " Hz: per-frame change <= smoothstep slope").c_str(), d);
        Check(std::fabs(r.activeSeconds - magtap::Envelope::Total()) <= dt + 1e-4f,
              ("envelope " + std::to_string(static_cast<int>(hz)) + " Hz: lasts 0.45 s").c_str(), d);
        Check(r.endsZero && r.minW >= 0.0f && r.maxW <= 1.0f,
              ("envelope " + std::to_string(static_cast<int>(hz)) + " Hz: ends exactly 0 and idle, stays in [0, 1]").c_str(), d);
    }
    // ---- a second trigger at every frame of the gesture (60 Hz and 144 Hz): no pop, the gesture ends at 0
    for (const float hz : {60.0f, 144.0f})
    {
        float worst = 0.0f;
        int worstAt = -1, notEnded = 0, fallNoPeak = 0;
        const int frames = static_cast<int>(magtap::Envelope::Total() * hz) + 1;
        for (int k = 1; k < frames; ++k)
        {
            const EnvRun r = RunEnvelope(hz, k);
            if (r.maxStep > worst)
            {
                worst = r.maxStep;
                worstAt = k;
            }
            notEnded += r.endsZero ? 0 : 1;
            fallNoPeak += r.peakAgain ? 0 : 1;
        }
        const float bound = kRiseSlope / hz + 1e-5f;
        const std::string d = std::to_string(static_cast<int>(hz)) + " Hz: worst step " + F(worst) + " at frame " + std::to_string(worstAt) +
                              " (bound " + F(bound) + "), not ended " + std::to_string(notEnded) + ", no second peak " + std::to_string(fallNoPeak);
        Check(worst <= bound, ("retrigger " + std::to_string(static_cast<int>(hz)) + " Hz at every frame: no weight jump").c_str(), d);
        Check(notEnded == 0 && fallNoPeak == 0,
              ("retrigger " + std::to_string(static_cast<int>(hz)) + " Hz: the gesture holds again and ends exactly at 0").c_str(), d);
    }
    // ---- ownership lost at the peak: smooth fade out within kCancelSeconds
    {
        const EnvRun r = RunEnvelope(60.0f, -1, 12); // frame 12 = 0.2 s: in the hold
        const float bound = 1.5f / magtap::kCancelSeconds / 60.0f + 1e-5f;
        Check(r.maxStep <= bound && r.endsZero && r.activeSeconds <= 0.2f + magtap::kCancelSeconds + 1.0f / 60.0f + 1e-4f,
              "ownership lost in the hold: fade out from the current weight, bounded per frame, idle after kCancelSeconds",
              "max step " + F(r.maxStep) + " bound " + F(bound) + " active " + F(r.activeSeconds));
    }
    // ---- an idle envelope never produces weight; a trigger is needed
    {
        magtap::Envelope e;
        float w = 0.0f;
        for (int i = 0; i < 30; ++i)
            w = std::fmax(w, e.Step(1.0f / 60.0f, true));
        Check(w == 0.0f && e.Idle(), "no trigger: weight exactly 0");
        e.Trigger();
        w = e.Step(1.0f / 60.0f, true);
        Check(w > 0.0f && !e.Idle(), "trigger: the gesture starts on the same Step", F(w));
    }

    // ---- pose pass on the synthetic rig
    const Synth s = MakeSynth();
    Check(s.rig.Valid(s.sk), "synthetic rig valid");
    for (const magtap::Kind kind : {magtap::Kind::Pistol, magtap::Kind::Rifle})
    {
        const char* kn = kind == magtap::Kind::Rifle ? "rifle" : "pistol";
        mm::ov::Pose p = s.pose;
        magtap::ApplyResult res;
        const bool z0 = !magtap::Apply(s.sk, p, s.rig, kind, 0.0f, res) && std::memcmp(&p, &s.pose, sizeof p) == 0;
        const bool zTiny = !magtap::Apply(s.sk, p, s.rig, kind, mm::ov::kZeroWeight * 0.5f, res) && std::memcmp(&p, &s.pose, sizeof p) == 0;
        Check(z0 && zTiny, (std::string(kn) + ": weight 0 / below kZeroWeight leaves the pose bit-identical").c_str());
        // full weight
        p = s.pose;
        const bool ok = magtap::Apply(s.sk, p, s.rig, kind, 1.0f, res);
        Quat rw, rh0, rh1;
        Vec3 pw, ph0, ph1;
        mm::gik::ModelOf(s.sk, p, s.rig.weaponRight, rw, pw);
        const Vec3 target = pw + mm::math::QuatRotate(rw, magtap::TargetOffset(kind));
        mm::gik::ModelOf(s.sk, s.pose, s.rig.left.hand, rh0, ph0);
        mm::gik::ModelOf(s.sk, p, s.rig.left.hand, rh1, ph1);
        const float err = Len(ph1 - target);
        const std::string d = std::string(kn) + ": err " + F(err) + " m, solve err " + F(res.handErr) + ", moved " + F(res.handMoved) +
                              " m of " + F(res.targetDist) + (res.clamped ? " CLAMPED" : "");
        std::printf("INFO %s\n", d.c_str());
        Check(ok && res.applied && !res.clamped && err <= 0.01f, (std::string(kn) + ": full weight reaches the target within 1 cm").c_str(), d);
        bool others = true;
        for (int b = 0; b < s.sk.n; ++b)
            if (!LeftArmBone(b))
                others = others && SameBone(p.b[b], s.pose.b[b]);
        Check(others, (std::string(kn) + ": right arm, WeaponRight, spine and every bone outside the left arm bit-identical").c_str());
        Check(mm::ov::AngleDeg(rh0, rh1) < 0.05f, (std::string(kn) + ": model rotation of the hand kept").c_str(),
              F(mm::ov::AngleDeg(rh0, rh1)) + " deg");
        const Vec3 axis = p.b[4].t * (1.0f / Len(p.b[4].t));
        const float twist = mm::ov::TwistAngleDeg(p.b[3].r, axis);
        Check(std::fabs(twist) < 0.01f, (std::string(kn) + ": no roll left in LeftForeArm (game convention)").c_str(), F(twist) + " deg");
        // the whole gesture at 30 / 60 / 144 Hz on the static synthetic pose
        for (const float hz : {30.0f, 60.0f, 144.0f})
        {
            magtap::Envelope e;
            e.Trigger();
            const float dt = 1.0f / hz;
            Vec3 last = ph0;
            float maxMove = 0.0f, peakErr = 1e9f, peakW = 0.0f;
            bool endIdentical = false;
            for (int i = 0; i < static_cast<int>(0.7f * hz); ++i)
            {
                const float w = e.Step(dt, true);
                mm::ov::Pose q = s.pose;
                magtap::ApplyResult rr;
                (void)magtap::Apply(s.sk, q, s.rig, kind, w, rr);
                const Vec3 h = HandPos(s, q);
                maxMove = std::fmax(maxMove, Len(h - last));
                last = h;
                if (w >= peakW)
                {
                    peakW = w;
                    peakErr = Len(h - target);
                }
                endIdentical = w == 0.0f && std::memcmp(&q, &s.pose, sizeof q) == 0;
            }
            const float bound = 1.3f * kRiseSlope * dt * res.targetDist + 1e-4f;
            const std::string dd = std::string(kn) + " " + std::to_string(static_cast<int>(hz)) + " Hz: max hand step " + F(maxMove) +
                                   " m (bound " + F(bound) + "), peak err " + F(peakErr) + " m";
            Check(maxMove <= bound && peakErr <= 0.01f && endIdentical,
                  (std::string(kn) + " " + std::to_string(static_cast<int>(hz)) +
                   " Hz gesture: bounded hand steps, target at the peak, bit-identical at the end").c_str(), dd);
        }
    }
    // slide roll-in wait (SlideDefer): a new rifle gesture on a slide frame waits for the carry de-roll (>= kSlideSettledWeight) or
    // kSlideDelaySeconds of feed time; off slide frames / pistols / settled de-roll: at once. Driver accumulation (MagTapStepLocked):
    // waited += min(dt, .25) per deferred Step, the check sees the time including this Step.
    {
        const auto R = magtap::Kind::Rifle;
        Check(magtap::SlideDefer(R, true, 0.0f, 0.0f) && magtap::SlideDefer(R, true, 0.89f, 0.34f),
              "slide wait: rifle request on a slide frame before the de-roll settles waits");
        Check(!magtap::SlideDefer(R, true, 0.9f, 0.0f) && !magtap::SlideDefer(R, true, 0.0f, magtap::kSlideDelaySeconds) &&
                  !magtap::SlideDefer(R, false, 0.0f, 0.0f) && !magtap::SlideDefer(magtap::Kind::Pistol, true, 0.0f, 0.0f),
              "slide wait: settled de-roll, kSlideDelaySeconds waited, a ground frame or a pistol -> starts at once");
        Check(magtap::SlideDefer(R, true, std::nanf(""), 0.0f) && !magtap::SlideDefer(R, true, 0.0f, std::nanf("")),
              "slide wait: NaN de-roll weight waits only up to the time bound, NaN waited time never waits");
        for (const float hz : {30.0f, 60.0f, 144.0f})
        {
            const float dt = 1.0f / hz;
            // (a) de-roll never settles (slide under the roll range): starts after kSlideDelaySeconds; (b) de-roll weight smoothstep over
            // .15 s from the request (rolled carry slide): starts when it reaches kSlideSettledWeight
            for (const bool settles : {false, true})
            {
                float waited = 0.0f, tStart = -1.0f, t = 0.0f;
                for (int i = 0; i < int(hz) && tStart < 0.0f; ++i)
                {
                    t += dt;
                    const float x = std::fmin(1.0f, t / 0.15f), w = settles ? x * x * (3.0f - 2.0f * x) : 0.0f;
                    if (magtap::SlideDefer(R, true, w, waited + dt))
                        waited += dt;
                    else
                        tStart = t;
                }
                const float expect = settles ? 0.15f * magtap::SmoothInv(magtap::kSlideSettledWeight) : magtap::kSlideDelaySeconds;
                const std::string d = "start " + F(tStart) + " s, expected " + F(expect) + " s";
                Check(tStart >= expect - 1e-4f && tStart <= expect + dt + 1e-4f,
                      ("slide wait " + std::to_string(static_cast<int>(hz)) + (settles ? " Hz: starts once the de-roll settles" :
                                                                                         " Hz: starts after kSlideDelaySeconds at the latest")).c_str(), d);
            }
        }
    }
    std::printf("magtap: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
