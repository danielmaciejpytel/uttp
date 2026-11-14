// Offline tests of the melee full-body handover (src/mm/MeleeFullBody.hpp) and its blend term (PoseSnapshot::lowerScale in
// src/mm/GameIntegration.hpp BlendIntoRecords), not shipped.
// Usage: utpp_melee_fullbody_test   Exit code 0 = all checks passed.
// Covers: every decision condition, the speed hysteresis, the fade (integ::UpperMaskBlend with kFadeSeconds, both ways, hold
// under a closing gate, snap when nothing is shown), lowerScale 1 (flag off, the default) = the pre-change blend bit for bit, and
// lowerScale 0 = the lower-body bones keep the game pose. Rig: the real player rig 0..28 (names + parents from
// the player skeleton listing, as in tests/AimOffsetTest.cpp).
// Round 3 (2026-11-09 review): the allowed PSM.MeleeWeapon states (default = attacks without ThrowAttack), the speed bridge
// (pilot alternation 0.2 / 0.97 m/s), the guard gate (GuardGate) and the driver terms with the guard off = the pre-guard
// expressions / fades bit for bit.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "../src/mm/GameIntegration.hpp"
#include "../src/mm/MeleeFullBody.hpp"

using namespace utpp::mm;
using namespace utpp::mm::integ;
namespace melee = utpp::mm::melee;

static int g_fail = 0;
static int g_pass = 0;

static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}

static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0)
{
    char buf[256];
    std::snprintf(buf, sizeof(buf), aFmt, a, b, c);
    return buf;
}

// ------------------------------------------------------------------------------------------------ decision

static melee::Input StandingMelee()
{
    melee::Input in;
    in.flag = true;
    in.meleeMask = true;
    in.owner = true;
    in.stance = 1;     // stand
    in.psmDetail = 1;  // LocomotionDetailed Stand
    in.haveAir = true;
    in.onGround = true;
    in.vz = 0.0f;
    in.maxVertical = 1.0f;
    in.planarSpeed = 0.0f;
    in.meleeState = melee::kMwComboAttack; // an attack state with third-person clips (default allowed-state mask)
    return in;
}

static void TestDecision()
{
    melee::Gate g;
    Expect("decision: default gate is off (why=flag)", g.LastWhy() == melee::kWhyFlag && !g.FastOff());
    Expect("decision: standing melee, every condition -> full body (why=on)", g.Update(StandingMelee()) && g.LastWhy() == melee::kWhyOn);

    struct Case
    {
        const char* name;
        void (*edit)(melee::Input&);
        melee::Why why;
    };
    const Case cases[] = {
        {"script flag / setting off -> why=flag", [](melee::Input& i) { i.flag = false; }, melee::kWhyFlag},
        {"no melee mask reason -> why=melee", [](melee::Input& i) { i.meleeMask = false; }, melee::kWhyMelee},
        {"base not the owner / not TPP / release -> why=owner", [](melee::Input& i) { i.owner = false; }, melee::kWhyOwner},
        {"crouch stance -> why=stance", [](melee::Input& i) { i.stance = 2; }, melee::kWhyStance},
        {"sprint stance -> why=stance", [](melee::Input& i) { i.stance = 3; }, melee::kWhyStance},
        {"unknown stance 0 -> why=stance", [](melee::Input& i) { i.stance = 0; }, melee::kWhyStance},
        {"detail Crouch 3 -> why=detail", [](melee::Input& i) { i.psmDetail = 3; }, melee::kWhyDetail},
        {"detail Sprint 4 -> why=detail", [](melee::Input& i) { i.psmDetail = 4; }, melee::kWhyDetail},
        {"detail Slide 5 -> why=detail", [](melee::Input& i) { i.psmDetail = 5; }, melee::kWhyDetail},
        {"detail Dodge 7 -> why=detail", [](melee::Input& i) { i.psmDetail = 7; }, melee::kWhyDetail},
        {"detail Jump 18 -> why=detail", [](melee::Input& i) { i.psmDetail = 18; }, melee::kWhyDetail},
        {"detail Fall 14 -> why=detail", [](melee::Input& i) { i.psmDetail = 14; }, melee::kWhyDetail},
        {"detail RegularLand 23 -> why=detail", [](melee::Input& i) { i.psmDetail = 23; }, melee::kWhyDetail},
        {"detail Knockdown 29 -> why=detail", [](melee::Input& i) { i.psmDetail = 29; }, melee::kWhyDetail},
        {"detail CrouchSprint 30 -> why=detail", [](melee::Input& i) { i.psmDetail = 30; }, melee::kWhyDetail},
        {"no air feed, |vz| >= maxVertical -> why=air", [](melee::Input& i) { i.haveAir = false; i.vz = -1.2f; }, melee::kWhyAir},
        {"no air feed, non-finite vz -> why=air", [](melee::Input& i) { i.haveAir = false; i.vz = NAN; }, melee::kWhyAir},
        {"stick held -> why=speed on the same feed", [](melee::Input& i) { i.stickHeld = true; }, melee::kWhySpeed},
        {"AimWalk (detail 2) + stick held (block while walking) -> why=speed",
         [](melee::Input& i) { i.psmDetail = 2; i.stickHeld = true; i.planarSpeed = 0.4f; }, melee::kWhySpeed},
    };
    for (const Case& c : cases)
    {
        melee::Gate gc;
        melee::Input in = StandingMelee();
        c.edit(in);
        const bool on = gc.Update(in);
        Expect((std::string("decision: ") + c.name).c_str(), !on && gc.LastWhy() == c.why, melee::WhyName(gc.LastWhy()));
    }
    // standing details that still count: not reported, NotInBase 0, Stand 1, AimWalk 2 (fists block / targeting)
    bool standingOk = true;
    for (int32_t d : {-1, 0, 1, 2})
    {
        melee::Gate gd;
        melee::Input in = StandingMelee();
        in.psmDetail = d;
        standingOk = standingOk && gd.Update(in);
    }
    Expect("decision: detail -1 / 0 / 1 / 2 count as standing", standingOk);
    // no air feed this frame: the ground state is not judged, vz still is
    melee::Gate ga;
    melee::Input noAir = StandingMelee();
    noAir.haveAir = false;
    noAir.onGround = false;
    Expect("decision: no air feed (haveAir false) -> onGround not judged, full body", ga.Update(noAir));
    noAir.vz = 2.0f;
    Expect("decision: no air feed but vz 2 m/s -> why=air", !ga.Update(noAir) && ga.LastWhy() == melee::kWhyAir);
    // priority: the first failing condition is reported
    melee::Input both = StandingMelee();
    both.flag = false;
    both.planarSpeed = 3.0f;
    melee::Gate gp;
    gp.Update(both);
    Expect("decision: several failures -> the first one (flag before speed)", gp.LastWhy() == melee::kWhyFlag);
    Expect("decision: WhyName covers every value",
           std::string_view(melee::WhyName(melee::kWhyOn)) == "on" && std::string_view(melee::WhyName(melee::kWhySpeed)) == "speed" &&
               std::string_view(melee::WhyName(static_cast<melee::Why>(99))) == "?");
}

// Feeds aN updates of aDt each with aIn; returns true when every one of them gave full body.
static bool FeedAll(melee::Gate& aG, melee::Input aIn, float aDt, int aN)
{
    aIn.dt = aDt;
    bool allOn = true;
    for (int i = 0; i < aN; ++i)
        allOn = aG.Update(aIn) && allOn;
    return allOn;
}

// Moving = stick intent; capsule speed only as a sustained condition (pilot 2026-11-09, recorded session
// melee-cases-20261109-122402: every punch moved the capsule 33-66 ms at ~0.97 m/s, combo finishers ~0.18 s at ~3.9 m/s).
static void TestSpeedAndStick()
{
    const float dt60 = 1.0f / 60.0f, dt30 = 1.0f / 30.0f;
    melee::Input in = StandingMelee();
    {
        melee::Gate g;
        in.planarSpeed = 0.97f;
        Expect("speed: 0.97 m/s for 2 feeds (30 Hz) without stick -> stays on (punch movement)", FeedAll(g, in, dt30, 2) && g.LastWhy() == melee::kWhyOn);
    }
    {
        melee::Gate g;
        in.planarSpeed = 3.9f;
        Expect("speed: 3.9 m/s for 0.18 s (11 feeds at 60 Hz) without stick -> stays on (finisher lunge)", FeedAll(g, in, dt60, 11),
               Fmt("fastT %.3f", g.FastSeconds()));
    }
    {
        melee::Gate g;
        in.planarSpeed = 1.0f;
        const bool on033 = FeedAll(g, in, dt60, 20); // 0.333 s
        const bool off04 = !FeedAll(g, in, dt60, 4) && g.LastWhy() == melee::kWhySpeed && g.FastOff(); // 0.4 s
        Expect("speed: 1.0 m/s without stick -> on at 0.33 s, off by 0.4 s (sustained >= 0.35 s, why=speed)", on033 && off04,
               Fmt("fastT %.3f", g.FastSeconds()));
        in.planarSpeed = 0.5f;
        Expect("speed: after a sustained stop, 0.5 m/s -> still off (>= kOnSpeed 0.35)", !FeedAll(g, in, dt60, 3));
        in.planarSpeed = 0.35f;
        Expect("speed: 0.35 m/s -> still off (boundary, strict <)", !FeedAll(g, in, dt60, 12));
        in.planarSpeed = 0.3f;
        Expect("speed: 0.3 m/s for 8 feeds (0.13 s < kBridgeSeconds) -> still off (release needs the bridge time)",
               !FeedAll(g, in, dt60, 8) && g.FastOff());
        FeedAll(g, in, dt60, 1); // the 9th feed sits on the 0.15 s boundary (float sum): not judged
        Expect("speed: 0.3 m/s by the 10th feed (0.167 s) -> on again", FeedAll(g, in, dt60, 1) && !g.FastOff());
    }
    {
        melee::Gate g;
        in.planarSpeed = 0.97f;
        FeedAll(g, in, dt60, 6);        // 0.1 s fast
        in.planarSpeed = 0.5f;          // a break longer than kBridgeSeconds (10 feeds, one feed of margin) clears the fast time
        FeedAll(g, in, dt60, 10);
        in.planarSpeed = 0.97f;
        Expect("speed: a break > kBridgeSeconds clears the fast time (0.1 s + 0.167 s break + 0.3 s -> on)", FeedAll(g, in, dt60, 18),
               Fmt("fastT %.3f", g.FastSeconds()));
    }
    {
        melee::Gate g;
        in.planarSpeed = 1.0f;
        Expect("speed: one 1 s hitch feed counts kMaxStepDt only (stays on)", FeedAll(g, in, 1.0f, 1) && std::fabs(g.FastSeconds() - melee::kMaxStepDt) < 1e-6f);
        in.planarSpeed = NAN;
        Expect("speed: non-finite speed counts as fast (off only when sustained)", FeedAll(g, in, dt60, 2) && !FeedAll(g, in, dt60, 20));
    }
    {
        melee::Gate g;
        in.planarSpeed = 0.0f;
        in.stickHeld = true;
        Expect("stick: held -> off on that feed (why=speed), whatever the speed", !FeedAll(g, in, dt60, 1) && g.LastWhy() == melee::kWhySpeed);
        in.stickHeld = false;
        Expect("stick: released -> on again on the next feed (the 0.2 s fade does the rest)", FeedAll(g, in, dt60, 1));
        in.psmDetail = 2;
        in.stickHeld = true;
        in.planarSpeed = 0.4f;
        Expect("stick: AimWalk (detail 2) + stick held (block while walking) -> off", !FeedAll(g, in, dt60, 1) && g.LastWhy() == melee::kWhySpeed);
        in.psmDetail = 1;
        in.stickHeld = false;
    }
    {
        // the speed state is tracked while another condition keeps the full body off: running into the attack
        melee::Gate g;
        melee::Input run = StandingMelee();
        run.meleeMask = false;
        run.stickHeld = true;
        run.planarSpeed = 3.8f;
        FeedAll(g, run, dt60, 30); // 0.5 s run, no melee
        run.meleeMask = true;
        run.stickHeld = false;
        run.planarSpeed = 2.0f; // stick released, the capsule still decelerating
        Expect("speed: running before the attack, released stick at 2.0 m/s -> off (legs stay on MM until below 0.35)",
               !FeedAll(g, run, dt60, 1) && g.LastWhy() == melee::kWhySpeed);
        run.planarSpeed = 0.2f;
        const bool off8 = !FeedAll(g, run, dt60, 8);
        FeedAll(g, run, dt60, 1); // boundary feed, not judged
        Expect("speed: decelerated below 0.35 m/s -> still off for 8 feeds, on by the 10th (kBridgeSeconds)", off8 && FeedAll(g, run, dt60, 1));
        g.Reset();
        Expect("speed: Reset -> timers 0, why=flag", g.FastSeconds() == 0.0f && !g.FastOff() && g.AirSeconds() == 0.0f && g.LastWhy() == melee::kWhyFlag);
    }
}

static void TestAir()
{
    const float dt60 = 1.0f / 60.0f;
    melee::Input in = StandingMelee();
    {
        melee::Gate g;
        in.vz = -1.27f; // lunge on uneven ground, logged with the air feed on the ground
        Expect("air: air feed on the ground with vz -1.27 -> on (vz judged only without an air feed)", FeedAll(g, in, dt60, 3));
        in.vz = 0.0f;
    }
    {
        melee::Gate g;
        in.onGround = false;
        const bool on005 = FeedAll(g, in, dt60, 3); // 0.05 s
        Expect("air: ungrounded 0.05 s -> on (debounced)", on005, Fmt("airT %.3f", g.AirSeconds()));
        const bool off015 = !FeedAll(g, in, dt60, 6) && g.LastWhy() == melee::kWhyAir; // 0.15 s
        Expect("air: ungrounded 0.15 s -> off (why=air)", off015, Fmt("airT %.3f", g.AirSeconds()));
        in.onGround = true;
        Expect("air: back on the ground -> on, timer 0", FeedAll(g, in, dt60, 1) && g.AirSeconds() == 0.0f);
    }
    {
        melee::Gate g;
        in.haveAir = false;
        in.vz = 1.2f;
        Expect("air: no air feed, |vz| 1.2 -> off at once", !FeedAll(g, in, dt60, 1) && g.LastWhy() == melee::kWhyAir);
        in.vz = 0.3f;
        Expect("air: no air feed, |vz| 0.3 -> on", FeedAll(g, in, dt60, 1));
    }
}

// ------------------------------------------------------------------------------------------------ fade

static void TestFade()
{
    const float dt = 1.0f / 60.0f;
    UpperMaskBlend b;
    float maxStep = 0.0f, prev = 0.0f;
    float at01 = -1.0f;
    float at02 = -1.0f;
    for (int i = 1; i <= 13; ++i)
    {
        const float w = b.Update(true, true, true, dt, melee::kFadeSeconds);
        maxStep = std::fmax(maxStep, std::fabs(w - prev));
        prev = w;
        if (i == 6)
            at01 = w;
        if (i == 12)
            at02 = w;
    }
    Expect("fade in: 0.1 s -> weight 0.5 (smoothstep midpoint of the 0.2 s fade)", std::fabs(at01 - 0.5f) < 1e-3f, Fmt("%.4f", at01));
    Expect("fade in: 0.2 s -> weight 1 (float rounding: one more frame at most)", at02 > 0.9999f && b.Weight() == 1.0f && b.Progress() == 1.0f,
           Fmt("%.6f", at02));
    Expect("fade in: no frame step above 1.5 x dt / kFadeSeconds (no pop)", maxStep <= 1.5f * dt / melee::kFadeSeconds + 1e-4f,
           Fmt("max step %.4f", maxStep));
    // whole body fading out (effective gate closed): the full body holds, the legs never flash back to MM
    for (int i = 0; i < 30; ++i)
        b.Update(false, false, true, dt, melee::kFadeSeconds);
    Expect("fade: not wanted + gate closed -> held at 1", b.Weight() == 1.0f);
    // fade out with the gate open (speed rose, attack ended)
    prev = b.Weight();
    maxStep = 0.0f;
    int frames = 0;
    while (b.Progress() > 0.0f && frames < 100)
    {
        const float w = b.Update(false, true, true, dt, melee::kFadeSeconds);
        maxStep = std::fmax(maxStep, std::fabs(w - prev));
        prev = w;
        ++frames;
    }
    Expect("fade out: back to MM in kFadeSeconds (12 frames at 60 Hz, +1 for float rounding), no pop", (frames == 12 || frames == 13) && b.Weight() == 0.0f &&
                                                                                  maxStep <= 1.5f * dt / melee::kFadeSeconds + 1e-4f,
           Fmt("frames %g, max step %.4f", frames, maxStep));
    // reversal mid-fade continues from the current progress (no restart)
    for (int i = 0; i < 3; ++i)
        b.Update(true, true, true, dt, melee::kFadeSeconds);
    const float mid = b.Progress();
    b.Update(false, true, true, dt, melee::kFadeSeconds);
    Expect("fade: reversal mid-fade steps back by dt / kFadeSeconds from where it was", std::fabs(b.Progress() - (mid - dt / melee::kFadeSeconds)) < 1e-5f);
    // nothing shown (MM weight 0): snaps
    b.Update(true, true, false, dt, melee::kFadeSeconds);
    Expect("fade: MM not shown -> snaps to wanted", b.Weight() == 1.0f);
    b.Update(false, false, false, dt, melee::kFadeSeconds);
    Expect("fade: MM not shown, not wanted -> snaps to 0", b.Weight() == 0.0f);
    // flag off for a whole sequence: lowerScale = 1 - weight stays exactly 1
    melee::Gate g;
    UpperMaskBlend fb;
    bool exactOne = true;
    for (int i = 0; i < 240; ++i)
    {
        melee::Input in = StandingMelee();
        in.flag = false;
        in.planarSpeed = (i % 80) * 0.01f;
        in.meleeMask = (i / 40) % 2 == 0;
        fb.Update(g.Update(in), (i % 7) != 0, true, dt, melee::kFadeSeconds);
        const float lowerScale = 1.0f - fb.Weight();
        exactOne = exactOne && lowerScale == 1.0f;
    }
    Expect("flag off: lowerScale exactly 1 on every frame of a melee sequence", exactOne);
}

// ------------------------------------------------------------------------------------------------ blend term

struct GameBone
{
    float p[4];
    float q[4];
    float s[4];
};
static_assert(sizeof(GameBone) == kGameBoneStride);

static const char* kNames[] = {"Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1",
                               "LeftLeg", "RightLeg", "Spine2", "LeftFoot", "RightFoot", "Spine3", "LeftShoulder",
                               "RightShoulder", "Neck", "LeftArm", "RightArm", "Neck1", "LeftForeArm", "RightForeArm", "Head",
                               "LeftHand", "RightHand", "LeftHandEnd", "WeaponLeft", "RightHandEnd", "WeaponRight"};
static constexpr uint32_t kNb = 29;

// The pre-change BlendIntoRecords (0.6.2 .. 2026-11-08): every bone outside upperBits at the full weight.
static uint32_t ReferenceBlend(uint8_t* aBones, uint32_t aCount, const PoseSnapshot& aSnap, float aWeight)
{
    if (!aBones || !(aWeight > 0.0f))
        return 0;
    const float wFull = aWeight >= 1.0f ? 1.0f : aWeight;
    const float us = std::isfinite(aSnap.upperScale) ? (aSnap.upperScale < 0.0f ? 0.0f : (aSnap.upperScale > 1.0f ? 1.0f : aSnap.upperScale))
                                                     : 1.0f;
    const float wUpper = wFull * us;
    const uint32_t nb = aSnap.numBones <= kMaxMMBones ? aSnap.numBones : kMaxMMBones;
    uint32_t written = 0;
    for (uint32_t b = aSnap.firstBone; b < nb; ++b)
    {
        const int32_t g = aSnap.gameIndex[b];
        if (g < 0 || static_cast<uint32_t>(g) >= aCount)
            continue;
        const float w = SnapshotUpperBone(aSnap, b) ? wUpper : wFull;
        if (!(w > 0.0f))
            continue;
        const float* m = &aSnap.pose[size_t(b) * 7];
        bool finite = true;
        for (int k = 0; k < 7; ++k)
            finite = finite && std::isfinite(m[k]);
        if (!finite)
            continue;
        float* rec = reinterpret_cast<float*>(aBones + size_t(g) * kGameBoneStride);
        float* q = rec + kGameRotOffsetFloats;
        float mq[4] = {m[3], m[4], m[5], m[6]};
        if (q[0] * mq[0] + q[1] * mq[1] + q[2] * mq[2] + q[3] * mq[3] < 0.0f)
            for (float& v : mq)
                v = -v;
        if (w >= 1.0f)
        {
            rec[0] = m[0];
            rec[1] = m[1];
            rec[2] = m[2];
            q[0] = mq[0];
            q[1] = mq[1];
            q[2] = mq[2];
            q[3] = mq[3];
        }
        else
        {
            rec[0] += (m[0] - rec[0]) * w;
            rec[1] += (m[1] - rec[1]) * w;
            rec[2] += (m[2] - rec[2]) * w;
            float r[4];
            for (int k = 0; k < 4; ++k)
                r[k] = q[k] + (mq[k] - q[k]) * w;
            const float n2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3];
            if (!(n2 > 1e-20f) || !std::isfinite(n2))
                continue;
            const float inv = 1.0f / std::sqrt(n2);
            for (int k = 0; k < 4; ++k)
                q[k] = r[k] * inv;
        }
        ++written;
    }
    return written;
}

static void Normalize4(float* q)
{
    const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int k = 0; k < 4; ++k)
        q[k] /= n;
}

static void TestBlendTerm()
{
    const UpperMask m = BuildUpperMask(kNb, 2, [](uint32_t b) -> std::string_view { return b < kNb ? kNames[b] : ""; });
    Expect("blend: player rig mask ok (Hips, Spine, legs lower; Spine1 and above upper)",
           m.ok && !m.Upper(2) && !m.Upper(4) && !m.Upper(5) && !m.Upper(11) && m.Upper(7) && m.Upper(13) && m.Upper(28));
    static PoseSnapshot snap{};
    snap = PoseSnapshot{};
    Expect("blend: snapshot defaults lowerScale 1, meleeAimWeight 0", snap.lowerScale == 1.0f && snap.meleeAimWeight == 0.0f);
    snap.numBones = kNb;
    snap.firstBone = 2;
    for (uint32_t b = 0; b < kMaxMMBones; ++b)
        snap.gameIndex[b] = b < kNb ? int32_t(b) : -1;
    for (uint32_t i = 0; i < kMaskWords; ++i)
        snap.upperBits[i] = m.bits[i];
    std::vector<GameBone> orig(kNb);
    for (uint32_t b = 0; b < kNb; ++b)
    {
        float* p = &snap.pose[size_t(b) * 7];
        p[0] = 0.1f * float(b) + 0.3f, p[1] = -0.05f * float(b), p[2] = 0.02f * float(b % 5) + 0.1f;
        p[3] = 0.1f + 0.01f * float(b), p[4] = -0.2f, p[5] = 0.05f * float(b % 3), p[6] = 0.9f;
        Normalize4(p + 3);
        GameBone& g = orig[b];
        g = GameBone{{0.2f - 0.01f * float(b), 0.03f * float(b), 0.5f, 0.0f}, {-0.1f, 0.15f, 0.02f * float(b % 4), -0.95f}, {1, 1, 1, 1}};
        Normalize4(g.q);
    }
    // lowerScale 1 (flag off, the default): bit for bit the pre-change blend, for every weight / upperScale combination
    bool same = true;
    std::string detail;
    for (float w : {0.0f, 0.25f, 0.7f, 1.0f, 1.5f})
        for (float us : {0.0f, 0.4f, 1.0f, NAN, -1.0f})
            for (float ls : {1.0f, NAN, 7.0f})
            {
                std::vector<GameBone> a = orig, r = orig;
                snap.upperScale = us;
                snap.lowerScale = ls;
                const uint32_t na = BlendIntoRecords(reinterpret_cast<uint8_t*>(a.data()), kNb, snap, w);
                const uint32_t nr = ReferenceBlend(reinterpret_cast<uint8_t*>(r.data()), kNb, snap, w);
                if (na != nr || std::memcmp(a.data(), r.data(), sizeof(GameBone) * kNb) != 0)
                {
                    same = false;
                    detail = Fmt("w %g us %g ls %g", w, us, ls);
                }
            }
    Expect("blend: lowerScale 1 / non-finite / > 1 = the pre-change blend bit for bit (weights 0..1.5, upperScale 0..1, NaN)", same, detail);
    // lowerScale 0 (full body at 1) with upperScale 0 (upper body on the game): every record bitwise the game pose
    snap.upperScale = 0.0f;
    snap.lowerScale = 0.0f;
    std::vector<GameBone> a = orig;
    uint32_t n = BlendIntoRecords(reinterpret_cast<uint8_t*>(a.data()), kNb, snap, 1.0f);
    Expect("blend: full body (lowerScale 0, upperScale 0) -> nothing written, records bitwise the game pose",
           n == 0 && std::memcmp(a.data(), orig.data(), sizeof(GameBone) * kNb) == 0, Fmt("written %g", n));
    // lowerScale 0, upperScale 1: only the upper bones written, the lower ones stay on the game pose
    snap.upperScale = 1.0f;
    a = orig;
    n = BlendIntoRecords(reinterpret_cast<uint8_t*>(a.data()), kNb, snap, 1.0f);
    bool lowerKept = true, upperWritten = true;
    for (uint32_t b = 2; b < kNb; ++b)
    {
        const bool kept = std::memcmp(&a[b], &orig[b], sizeof(GameBone)) == 0;
        if (m.Upper(b))
            upperWritten = upperWritten && a[b].p[0] == snap.pose[size_t(b) * 7];
        else
            lowerKept = lowerKept && kept;
    }
    Expect("blend: lowerScale 0 -> Hips / Spine / legs bitwise the game pose, upper bones MM", lowerKept && upperWritten && n == m.upper);
    // lowerScale 0.5 at weight 1: the lower bones halfway (position midpoint)
    snap.upperScale = 0.0f;
    snap.lowerScale = 0.5f;
    a = orig;
    n = BlendIntoRecords(reinterpret_cast<uint8_t*>(a.data()), kNb, snap, 1.0f);
    const float hipsMid = orig[2].p[0] + (snap.pose[2 * 7] - orig[2].p[0]) * 0.5f;
    Expect("blend: lowerScale 0.5 -> Hips position midpoint, upper bones untouched",
           std::fabs(a[2].p[0] - hipsMid) < 1e-6f && std::memcmp(&a[13], &orig[13], sizeof(GameBone)) == 0 && n == m.lower,
           Fmt("%g vs %g", a[2].p[0], hipsMid));
    // Root / Trajectory (below firstBone) are never written, whatever the scales
    Expect("blend: Root / Trajectory untouched", std::memcmp(&a[0], &orig[0], sizeof(GameBone) * 2) == 0);
    Expect("blend: SnapshotLowerScale clamps (-1 -> 0, 2 -> 1, NaN -> 1)",
           [] {
               PoseSnapshot s{};
               s.lowerScale = -1.0f;
               const float lo = SnapshotLowerScale(s);
               s.lowerScale = 2.0f;
               const float hi = SnapshotLowerScale(s);
               s.lowerScale = NAN;
               return lo == 0.0f && hi == 1.0f && SnapshotLowerScale(s) == 1.0f;
           }());
}

// ------------------------------------------------------------------------------------------------ allowed states (round 3)

// Review findings 1 / 2 (2026-11-09): the full body only in PSM.MeleeWeapon states whose third-person clips exist. Default mask
// = the attack states without ThrowAttack; block / targeting / hold / parried / deflect / the release hold (Idle) / equipping /
// throws keep the legs on MM (their clips were never rebuilt: final-mb7 coverage.txt "FOR THE DLL").
static void TestStates()
{
    bool attacksOn = true;
    for (int32_t s : {11, 12, 13, 14, 15, 16, 17, 18, 20, 21})
    {
        melee::Gate g;
        melee::Input in = StandingMelee();
        in.meleeState = s;
        attacksOn = attacksOn && g.Update(in);
    }
    Expect("states: default mask -> attack states 11..18, 20, 21 take the full body", attacksOn);
    bool othersOff = true;
    std::string offDetail;
    for (int32_t s : {-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 19, 22, 31})
    {
        melee::Gate g;
        melee::Input in = StandingMelee();
        in.meleeState = s;
        const bool off = !g.Update(in) && g.LastWhy() == melee::kWhyState;
        if (!off)
            offDetail += std::to_string(s) + " ";
        othersOff = othersOff && off;
    }
    Expect("states: default mask -> not reported, NotReady, Equipping, Idle (release hold), Safe, PublicSafe, Parried, Hold, "
           "ChargedHold, Block, Targeting, Deflect, ThrowAttack, Default -> why=state",
           othersOff, offDetail);
    {
        melee::Gate g;
        melee::Input in = StandingMelee();
        in.meleeState = melee::kMwBlock;
        in.psmDetail = 2;
        Expect("states: fists block standing (pilot 'MMMELEEFULL on ... detail=2 reasons=0x3') -> off why=state",
               !g.Update(in) && g.LastWhy() == melee::kWhyState);
        in.allowedStates = melee::kFullBodyDefaultStates | melee::StateBit(melee::kMwBlock);
        Expect("states: Block 8 added to the mask (block clips installed later) -> on", g.Update(in));
        in.allowedStates = 0;
        in.meleeState = melee::kMwComboAttack;
        Expect("states: empty mask -> even attacks stay off (why=state)", !g.Update(in) && g.LastWhy() == melee::kWhyState);
    }
    {
        // the attack ends (Idle 2) while the melee reason still holds (0.3 s release hold): the legs go back to MM
        melee::Gate g;
        melee::Input in = StandingMelee();
        const bool attack = g.Update(in);
        in.meleeState = melee::kMwIdle;
        Expect("states: attack -> release hold (Idle 2, melee reason still on) -> off why=state", attack && !g.Update(in) && g.LastWhy() == melee::kWhyState);
    }
    Expect("states: mask constants (0x37F800, no ThrowAttack, StateBit out of range 0)",
           melee::kFullBodyDefaultStates == 0x37F800u && !melee::StateAllowed(melee::kFullBodyDefaultStates, melee::kMwThrowAttack) &&
               melee::StateBit(22) == 0u && melee::StateBit(-1) == 0u && !melee::StateAllowed(0xFFFFFFFFu, -1));
    Expect("states: ValidStates accepts bits 0..21 only",
           melee::ValidStates(0) && melee::ValidStates(0x3FFFFF) && !melee::ValidStates(0x400000) && !melee::ValidStates(-1) &&
               !melee::ValidStates(int64_t(1) << 31));
    Expect("states: WhyName(kWhyState) = state", std::string_view(melee::WhyName(melee::kWhyState)) == "state");
    {
        // stage 2 of the guard (melee.guardFullBody): standing in the guard (Idle 2) without a melee reason -> on; it widens the
        // allowed states by Idle 2 only
        melee::Gate g;
        melee::Input in = StandingMelee();
        in.meleeMask = false;
        in.guard = true;
        in.meleeState = melee::kMwIdle;
        Expect("states: stage-2 guard standing in Idle 2 -> on (no melee reason needed)", g.Update(in));
        bool onlyIdle = true;
        for (int32_t s : {-1, 1, 3, 4, 5, 6, 7, 8, 9, 10, 19})
        {
            in.meleeState = s;
            in.meleeMask = s >= 5;
            onlyIdle = onlyIdle && !g.Update(in) && g.LastWhy() == melee::kWhyState;
        }
        Expect("states: stage-2 guard widens the mask by Idle 2 only (equip, safe, hold, block, targeting, throw stay off)", onlyIdle);
        in.meleeState = melee::kMwComboAttack;
        in.meleeMask = true;
        Expect("states: stage-2 guard + attack -> on", g.Update(in));
        in.guard = false;
        in.meleeMask = false;
        in.meleeState = melee::kMwIdle;
        Expect("states: no guard and no melee reason -> why=melee", !g.Update(in) && g.LastWhy() == melee::kWhyMelee);
    }
}

// ------------------------------------------------------------------------------------------------ speed bridge (round 3)

// Review finding 5: the sustained-speed timer was reset by any feed <= 0.6 m/s, so the pilot alternation ~0.2 / ~0.97 m/s never
// tripped it. Now dips shorter than kBridgeSeconds are bridged (the fast time is kept, the dip does not count) and the stop ends
// only after kBridgeSeconds below kOnSpeed.
static void TestSpeedBridge()
{
    const float dt60 = 1.0f / 60.0f, dt30 = 1.0f / 30.0f;
    for (float dt : {dt60, dt30})
    {
        melee::Gate g;
        melee::Input in = StandingMelee();
        in.dt = dt;
        float t = 0.0f, tripAt = -1.0f;
        bool releasedAfter = false;
        const int n = int(2.0f / dt + 0.5f);
        for (int i = 0; i < n; ++i)
        {
            in.planarSpeed = (i % 2 == 0) ? 0.97f : 0.2f;
            const bool on = g.Update(in);
            t += dt;
            if (!on && tripAt < 0.0f && g.LastWhy() == melee::kWhySpeed)
                tripAt = t;
            if (tripAt >= 0.0f && on)
                releasedAfter = true;
        }
        Expect(Fmt("bridge: 0.97 / 0.2 m/s alternation at %.0f Hz -> trips by 0.8 s and stays tripped for 2 s (why=speed)", 1.0 / dt).c_str(),
               tripAt > 0.0f && tripAt <= 0.8f && !releasedAfter && g.FastOff(), Fmt("trip at %.3f s", tripAt));
        // the last alternation feed (0.2 m/s) already counts as slow time
        const float carried = in.planarSpeed < melee::kOnSpeed ? dt : 0.0f;
        in.planarSpeed = 0.0f;
        int feeds = 0;
        while (feeds < 100 && !g.Update(in))
            ++feeds;
        const float tOn = float(feeds + 1) * dt + carried;
        Expect(Fmt("bridge: %.0f Hz alternation, then still -> on once below kOnSpeed for kBridgeSeconds", 1.0 / dt).c_str(),
               tOn >= melee::kBridgeSeconds - 1e-4f && tOn <= melee::kBridgeSeconds + dt + 1e-4f, Fmt("slow for %.3f s", tOn));
    }
    for (float gap : {0.3f, 0.5f})
    {
        // punches: each 66 ms at ~0.97 m/s (pilot), then ~0.2 m/s
        melee::Gate g;
        melee::Input in = StandingMelee();
        in.dt = dt30;
        bool allOn = true;
        float t = 0.0f, maxFast = 0.0f;
        for (int i = 0; i < 90; ++i)
        {
            in.planarSpeed = std::fmod(t, gap) < 0.066f ? 0.97f : 0.2f;
            allOn = g.Update(in) && allOn;
            maxFast = std::fmax(maxFast, g.FastSeconds());
            t += dt30;
        }
        Expect(Fmt("bridge: punches every %.1f s (66 ms at 0.97 m/s each, 3 s at 30 Hz) -> never off", gap).c_str(), allOn,
               Fmt("max fast time %.3f s", maxFast));
    }
    {
        melee::Gate g;
        melee::Input in = StandingMelee();
        in.planarSpeed = 3.9f;
        Expect("bridge: finisher lunge 3.9 m/s for 0.30 s (logged 0.18 s, margin 0.12 s) -> on", FeedAll(g, in, dt60, 18),
               Fmt("fastT %.3f", g.FastSeconds()));
        in.planarSpeed = 0.2f;
        FeedAll(g, in, dt60, 10);
        Expect("bridge: 10 feeds (> kBridgeSeconds) below kOffSpeed after it -> fast time cleared", g.FastSeconds() == 0.0f);
    }
}

// ------------------------------------------------------------------------------------------------ guard (round 3)

static melee::GuardInput GuardStanding()
{
    melee::GuardInput g;
    g.flag = true;
    g.setting = true;
    g.maskOk = true;
    g.owner = true;
    g.gateOpen = true;
    g.meleeState = melee::kMwIdle;
    g.upperBody = 0;
    g.stance = 1;
    g.psmDetail = 1;
    g.haveAir = true;
    g.onGround = true;
    g.vz = 0.0f;
    g.maxVertical = 1.0f;
    g.dt = 1.0f / 60.0f;
    return g;
}

static void TestGuard()
{
    {
        melee::GuardGate g;
        Expect("guard: default gate why=flag", g.LastWhy() == melee::kGuardFlag);
        melee::GuardInput in = GuardStanding();
        Expect("guard: fists drawn, standing in Idle 2 -> on", g.Update(in) == melee::kGuardOn);
        bool cont = true;
        // 2026-11-13: Safe 3 joined the guard states (old: why=state). The third-person melee_safepose is shipped, so a friendly NPC in
        // reach lowers the guard instead of dropping the hands to the locomotion arms.
        for (int32_t s : {2, 6, 6, 11, 11, 12, 2, 7, 13, 2, 8, 15, 2, 9, 10, 20, 5, 2, 3, 3, 2})
        {
            in.meleeState = s;
            cont = cont && g.Update(in) == melee::kGuardOn;
        }
        Expect("guard: Idle -> Hold -> ComboAttack -> FinalAttack -> Idle -> ChargedHold -> StrongAttack -> Block -> BlockAttack -> "
               "Targeting -> Deflect -> DeflectAttack -> Parried -> Idle -> Safe -> Idle: on on every feed (no gap at the strike start)",
               cont);
        in.upperBody = 6;
        const bool aim = g.Update(in) == melee::kGuardOn;
        in.upperBody = -1;
        const bool none = g.Update(in) == melee::kGuardOn;
        Expect("guard: UpperBody Aim 6 (block) and not reported -1 keep it", aim && none);
        in.upperBody = 0;
        in.psmDetail = 2;
        Expect("guard: AimWalk detail 2 (block / targeting) keeps it", g.Update(in) == melee::kGuardOn);
    }
    struct Case
    {
        const char* name;
        void (*edit)(melee::GuardInput&);
        melee::GuardWhy why;
    };
    const Case cases[] = {
        {"script flag off (holstered, other weapon, Gorilla Arms) -> why=flag", [](melee::GuardInput& i) { i.flag = false; }, melee::kGuardFlag},
        {"setting melee.guard off (default) -> why=flag", [](melee::GuardInput& i) { i.setting = false; }, melee::kGuardFlag},
        {"upper mask off / not resolved -> why=mask", [](melee::GuardInput& i) { i.maskOk = false; }, melee::kGuardMask},
        {"base not the owner / not TPP / release reason -> why=owner", [](melee::GuardInput& i) { i.owner = false; }, melee::kGuardOwner},
        {"effective gate closed / exit release -> why=gate", [](melee::GuardInput& i) { i.gateOpen = false; }, melee::kGuardGate},
        {"Equipping 1 (first equip stays vanilla) -> why=state", [](melee::GuardInput& i) { i.meleeState = 1; }, melee::kGuardState},
        {"PublicSafe 4 -> why=state", [](melee::GuardInput& i) { i.meleeState = 4; }, melee::kGuardState},
        {"NotReady 0 -> why=state", [](melee::GuardInput& i) { i.meleeState = 0; }, melee::kGuardState},
        {"MeleeWeapon not reported -> why=state", [](melee::GuardInput& i) { i.meleeState = -1; }, melee::kGuardState},
        {"Default 22 -> why=state", [](melee::GuardInput& i) { i.meleeState = 22; }, melee::kGuardState},
        {"UpperBody SwitchItems 1 (holster / swap) -> why=upper", [](melee::GuardInput& i) { i.upperBody = 1; }, melee::kGuardUpper},
        {"UpperBody SwitchCyberware 2 -> why=upper", [](melee::GuardInput& i) { i.upperBody = 2; }, melee::kGuardUpper},
        {"UpperBody Reload 3 -> why=upper", [](melee::GuardInput& i) { i.upperBody = 3; }, melee::kGuardUpper},
        {"UpperBody TemporaryUnequip 4 -> why=upper", [](melee::GuardInput& i) { i.upperBody = 4; }, melee::kGuardUpper},
        {"UpperBody ForceEmptyHands 5 -> why=upper", [](melee::GuardInput& i) { i.upperBody = 5; }, melee::kGuardUpper},
        {"crouch stance -> why=stance", [](melee::GuardInput& i) { i.stance = 2; }, melee::kGuardStance},
        {"sprint stance -> why=stance", [](melee::GuardInput& i) { i.stance = 3; }, melee::kGuardStance},
        {"detail Crouch 3 -> why=detail", [](melee::GuardInput& i) { i.psmDetail = 3; }, melee::kGuardDetail},
        {"detail Sprint 4 -> why=detail", [](melee::GuardInput& i) { i.psmDetail = 4; }, melee::kGuardDetail},
        {"detail Slide 5 -> why=detail", [](melee::GuardInput& i) { i.psmDetail = 5; }, melee::kGuardDetail},
        {"detail Dodge 7 -> why=detail", [](melee::GuardInput& i) { i.psmDetail = 7; }, melee::kGuardDetail},
        {"detail Fall 14 -> why=detail", [](melee::GuardInput& i) { i.psmDetail = 14; }, melee::kGuardDetail},
        {"detail Jump 18 -> why=detail", [](melee::GuardInput& i) { i.psmDetail = 18; }, melee::kGuardDetail},
        {"no air feed, |vz| 1.5 -> why=air", [](melee::GuardInput& i) { i.haveAir = false; i.vz = 1.5f; }, melee::kGuardAir},
    };
    for (const Case& c : cases)
    {
        melee::GuardGate g;
        melee::GuardInput in = GuardStanding();
        c.edit(in);
        const melee::GuardWhy w = g.Update(in);
        Expect((std::string("guard: ") + c.name).c_str(), w == c.why && g.LastWhy() == c.why, melee::GuardWhyName(w));
        // the owner-free decision (the motion driver yields on it): only the missing ownership leaves it on
        Expect((std::string("guard without owner: ") + c.name).c_str(), g.OnWithoutOwner() == (c.why == melee::kGuardOwner));
    }
    {
        melee::GuardGate g;
        melee::GuardInput in = GuardStanding();
        const bool on = g.Update(in) == melee::kGuardOn && g.OnWithoutOwner();
        in.setting = false; // melee.guard off (default): never asks the motion driver to yield
        const bool offSetting = g.Update(in) == melee::kGuardFlag && !g.OnWithoutOwner();
        g.Reset();
        Expect("guard without owner: on with the guard, off with melee.guard off and after Reset", on && offSetting && !g.OnWithoutOwner());
    }
    {
        melee::GuardGate g;
        melee::GuardInput in = GuardStanding();
        in.onGround = false;
        bool on005 = true;
        for (int i = 0; i < 3; ++i)
            on005 = g.Update(in) == melee::kGuardOn && on005;
        bool off015 = false;
        for (int i = 0; i < 6; ++i)
            off015 = g.Update(in) == melee::kGuardAir;
        in.onGround = true;
        const bool back = g.Update(in) == melee::kGuardOn && g.AirSeconds() == 0.0f;
        Expect("guard: ungrounded 0.05 s -> on (debounced), 0.15 s -> why=air, grounded -> on, timer 0", on005 && off015 && back);
        g.Reset();
        Expect("guard: Reset -> why=flag, air timer 0", g.LastWhy() == melee::kGuardFlag && g.AirSeconds() == 0.0f);
    }
    {
        // soft / hard end (2026-11-13): soft = the arms may ease out (0.35 s): the weapon state left the guard states, a crouch / sprint
        // stance or detail. Hard = the quick fade as before: flag, mask, owner, closed gate, UpperBody swap, air, slide, dodge, climb...
        struct SoftCase
        {
            const char* name;
            void (*edit)(melee::GuardInput&);
            bool soft;
        };
        const SoftCase soft[] = {
            {"holstered / no state (NotReady 0)", [](melee::GuardInput& i) { i.meleeState = 0; }, true},
            {"Equipping 1", [](melee::GuardInput& i) { i.meleeState = 1; }, true},
            {"PublicSafe 4", [](melee::GuardInput& i) { i.meleeState = 4; }, true},
            {"crouch stance", [](melee::GuardInput& i) { i.stance = 2; }, true},
            {"sprint stance", [](melee::GuardInput& i) { i.stance = 3; }, true},
            {"detail Crouch 3", [](melee::GuardInput& i) { i.psmDetail = 3; }, true},
            {"detail Sprint 4", [](melee::GuardInput& i) { i.psmDetail = 4; }, true},
            {"detail CrouchSprint 30", [](melee::GuardInput& i) { i.psmDetail = 30; }, true},
            {"script flag off", [](melee::GuardInput& i) { i.flag = false; }, false},
            {"setting off", [](melee::GuardInput& i) { i.setting = false; }, false},
            {"mask off", [](melee::GuardInput& i) { i.maskOk = false; }, false},
            {"not the owner", [](melee::GuardInput& i) { i.owner = false; }, false},
            {"gate closed", [](melee::GuardInput& i) { i.gateOpen = false; }, false},
            {"UpperBody SwitchItems 1 (holster / swap to a gun)", [](melee::GuardInput& i) { i.upperBody = 1; }, false},
            {"UpperBody Reload 3", [](melee::GuardInput& i) { i.upperBody = 3; }, false},
            {"detail Slide 5", [](melee::GuardInput& i) { i.psmDetail = 5; }, false},
            {"detail Dodge 7", [](melee::GuardInput& i) { i.psmDetail = 7; }, false},
            {"detail Climb 8", [](melee::GuardInput& i) { i.psmDetail = 8; }, false},
            {"detail Fall 14", [](melee::GuardInput& i) { i.psmDetail = 14; }, false},
            {"detail Jump 18", [](melee::GuardInput& i) { i.psmDetail = 18; }, false},
            {"detail Knockdown 29", [](melee::GuardInput& i) { i.psmDetail = 29; }, false},
            {"ungrounded (air)", [](melee::GuardInput& i) { i.haveAir = false; i.vz = 1.5f; }, false},
        };
        bool allOk = true;
        std::string bad;
        for (const SoftCase& c : soft)
        {
            melee::GuardGate g;
            melee::GuardInput in = GuardStanding();
            c.edit(in);
            const bool off = g.Update(in) != melee::kGuardOn;
            if (!off || g.LastSoft() != c.soft)
            {
                allOk = false;
                bad += std::string(" [") + c.name + "]";
            }
        }
        melee::GuardGate on;
        const bool onNotSoft = on.Update(GuardStanding()) == melee::kGuardOn && !on.LastSoft();
        on.Reset();
        Expect("guard: soft end (state / crouch / sprint) vs hard end (flag, mask, owner, gate, UpperBody, air, slide, dodge, climb, jump ...); on and Reset -> not soft",
               allOk && onNotSoft && !on.LastSoft(), bad);
    }
    Expect("guard: GuardWhyName covers every value",
           std::string_view(melee::GuardWhyName(melee::kGuardOn)) == "on" && std::string_view(melee::GuardWhyName(melee::kGuardAir)) == "air" &&
               std::string_view(melee::GuardWhyName(melee::kGuardUpper)) == "upper" &&
               std::string_view(melee::GuardWhyName(static_cast<melee::GuardWhy>(99))) == "?");
    {
        // exhaustive: with the script flag or the user setting off the guard is never on, whatever the other inputs
        bool neverOn = true;
        long n = 0;
        for (int fs = 0; fs < 3; ++fs)
            for (int ms = -1; ms <= 23; ++ms)
                for (int ub = -1; ub <= 7; ++ub)
                    for (int st = 0; st <= 3; ++st)
                        for (int dd = -1; dd <= 31; ++dd)
                            for (int bits = 0; bits < 32; ++bits)
                            {
                                melee::GuardGate g;
                                melee::GuardInput in;
                                in.flag = fs == 1;
                                in.setting = fs == 2;
                                in.meleeState = ms;
                                in.upperBody = ub;
                                in.stance = st;
                                in.psmDetail = dd;
                                in.maskOk = (bits & 1) != 0;
                                in.owner = (bits & 2) != 0;
                                in.gateOpen = (bits & 4) != 0;
                                in.onGround = (bits & 8) != 0;
                                in.haveAir = (bits & 16) != 0;
                                in.dt = 1.0f / 60.0f;
                                neverOn = neverOn && g.Update(in) != melee::kGuardOn;
                                ++n;
                            }
        Expect("guard: flag or setting off -> never on (exhaustive sweep)", neverOn, Fmt("%g combinations", double(n)));
    }
}

// ------------------------------------------------------------------------------------------------ driver terms (round 3)

static void TestDriverTerms()
{
    bool same = true;
    for (int m = 0; m < 64; ++m)
    {
        melee::DriverIn in;
        in.maskReasonsUpper = (m & 1) != 0;
        in.weaponMaskWanted = (m & 2) != 0;
        in.aimFadeEligible = (m & 4) != 0;
        in.meleeMask = (m & 8) != 0;
        in.guardFullSetting = (m & 16) != 0;
        in.guardTiltSetting = (m & 32) != 0;
        in.guardWanted = false;
        const melee::DriverTerms t = melee::CombineDriverTerms(in);
        // the pre-guard expressions of MotionHost.cpp (g_maskBlend wanted, maskTime, full-body meleeMask, g_meleeAimBlend wanted)
        same = same && t.maskWanted == (in.maskReasonsUpper || in.weaponMaskWanted) &&
               t.aimFadeLong == (in.aimFadeEligible && !in.weaponMaskWanted) && t.fullMelee == in.meleeMask && !t.fullGuard &&
               t.tiltWanted == in.meleeMask && !t.guardOnly;
    }
    Expect("driver terms: guard not wanted -> mask, fade time, full-body input and tilt equal the pre-guard expressions (64 inputs)", same);
    {
        // the fade times with the guard off = the pre-guard pair (maskTime = aimFadeLong ? 0.25 : 0.15; out = 0.35 when the melee release
        // is armed and the mask is not wanted and no aim fade), for every term combination and both latches
        bool ident = true;
        for (int m = 0; m < 256 && ident; ++m)
        {
            melee::DriverIn in;
            in.maskReasonsUpper = (m & 1) != 0;
            in.weaponMaskWanted = (m & 2) != 0;
            in.aimFadeEligible = (m & 4) != 0;
            in.meleeMask = (m & 8) != 0;
            const bool longOut = (m & 16) != 0;
            const melee::DriverTerms t = melee::CombineDriverTerms(in);
            const melee::MaskTimes mt = melee::ComputeMaskTimes(t, longOut, false, kMaskFadeSeconds, 0.25f, kMeleeReleaseFadeOutSeconds); // no guard: no soft latch
            const float maskTime = t.aimFadeLong ? 0.25f : kMaskFadeSeconds;
            const float outOld = (!t.maskWanted && longOut && !t.aimFadeLong) ? kMeleeReleaseFadeOutSeconds : maskTime;
            ident = ident && mt.in == maskTime && mt.out == outOld && mt.tiltOut == outOld;
        }
        Expect("fade times: guard not wanted -> (in, out) equal the pre-guard maskTime / maskOutTime (256 inputs)", ident);
        melee::DriverIn g;
        g.guardWanted = true;
        melee::DriverTerms t = melee::CombineDriverTerms(g);
        melee::MaskTimes mt = melee::ComputeMaskTimes(t, false, false, kMaskFadeSeconds, 0.25f, kMeleeReleaseFadeOutSeconds);
        Expect("fade times: guard alone holds the mask -> the raise takes kGuardFadeInSeconds 0.25, the quick fade out stays 0.15",
               t.guardOnly && mt.in == melee::kGuardFadeInSeconds && melee::kGuardFadeInSeconds == 0.25f && mt.out == kMaskFadeSeconds);
        g.maskReasonsUpper = true;
        t = melee::CombineDriverTerms(g);
        mt = melee::ComputeMaskTimes(t, false, false, kMaskFadeSeconds, 0.25f, kMeleeReleaseFadeOutSeconds);
        Expect("fade times: guard with a melee reason (a strike): the raise stays 0.15", !t.guardOnly && mt.in == kMaskFadeSeconds);
        g.maskReasonsUpper = false;
        g.guardWanted = false;
        t = melee::CombineDriverTerms(g);
        mt = melee::ComputeMaskTimes(t, false, true, kMaskFadeSeconds, 0.25f, kMeleeReleaseFadeOutSeconds);
        Expect("fade times: soft guard end, nothing else wanted -> eased out 0.35, in unchanged", mt.out == kMeleeReleaseFadeOutSeconds && mt.in == kMaskFadeSeconds);
        g.weaponMaskWanted = true;
        t = melee::CombineDriverTerms(g);
        mt = melee::ComputeMaskTimes(t, false, true, kMaskFadeSeconds, 0.25f, kMeleeReleaseFadeOutSeconds);
        Expect("fade times: a soft latch does not slow a mask that is wanted again (out unused, in 0.15)", t.maskWanted && mt.in == kMaskFadeSeconds);
    }
    melee::DriverIn g;
    g.guardWanted = true;
    g.aimFadeEligible = true;
    melee::DriverTerms t = melee::CombineDriverTerms(g);
    Expect("driver terms: guard on -> upper mask wanted at the standard fade, no stage-2 full body / tilt without their settings",
           t.maskWanted && !t.aimFadeLong && !t.fullGuard && !t.tiltWanted && !t.fullMelee);
    g.guardFullSetting = true;
    g.guardTiltSetting = true;
    t = melee::CombineDriverTerms(g);
    Expect("driver terms: guard on + guardFullBody + guardAimTilt -> stage-2 full body and tilt", t.fullGuard && t.tiltWanted);
    {
        // end to end with every guard setting off but the stage-2 / tilt settings on: GuardGate + CombineDriverTerms + the three
        // fades over a 600-feed melee sequence equal the pre-guard path bit for bit
        const float dt = 1.0f / 60.0f;
        melee::GuardGate gg;
        melee::Gate fg, fgRef;
        UpperMaskBlend mask, maskRef, full, fullRef, tilt, tiltRef;
        bool ident = true, guardSeen = false;
        for (int i = 0; i < 600; ++i)
        {
            melee::GuardInput gi = GuardStanding();
            gi.setting = false;
            gi.meleeState = (i / 20) % 24 - 1;
            const bool guardWanted = gg.Update(gi) == melee::kGuardOn;
            guardSeen = guardSeen || guardWanted;
            melee::DriverIn di;
            di.maskReasonsUpper = (i / 30) % 2 == 0;
            di.meleeMask = di.maskReasonsUpper && (i % 3 != 0);
            di.weaponMaskWanted = (i / 70) % 2 == 1;
            di.aimFadeEligible = (i / 50) % 2 == 0;
            di.guardWanted = guardWanted;
            di.guardFullSetting = true;
            di.guardTiltSetting = true;
            const melee::DriverTerms tt = melee::CombineDriverTerms(di);
            const bool gateOpen = i % 11 != 0;
            mask.Update(tt.maskWanted, gateOpen, true, dt, tt.aimFadeLong ? 0.25f : kMaskFadeSeconds);
            maskRef.Update(di.maskReasonsUpper || di.weaponMaskWanted, gateOpen, true, dt,
                           di.aimFadeEligible && !di.weaponMaskWanted ? 0.25f : kMaskFadeSeconds);
            melee::Input fi = StandingMelee();
            fi.meleeState = gi.meleeState;
            fi.planarSpeed = (i % 40) * 0.03f;
            fi.dt = dt;
            fi.meleeMask = tt.fullMelee;
            fi.guard = tt.fullGuard;
            melee::Input fr = fi;
            fr.meleeMask = di.meleeMask;
            fr.guard = false;
            full.Update(fg.Update(fi), gateOpen, true, dt, melee::kFadeSeconds);
            fullRef.Update(fgRef.Update(fr), gateOpen, true, dt, melee::kFadeSeconds);
            tilt.Update(tt.tiltWanted, true, true, dt, kMaskFadeSeconds);
            tiltRef.Update(di.meleeMask, true, true, dt, kMaskFadeSeconds);
            ident = ident && mask.Weight() == maskRef.Weight() && full.Weight() == fullRef.Weight() && tilt.Weight() == tiltRef.Weight() &&
                    fg.LastWhy() == fgRef.LastWhy();
        }
        Expect("driver terms: melee.guard off, 600-feed sequence -> upper mask / full body / tilt weights bit-identical to the pre-guard path",
               ident && !guardSeen);
    }
}

// ------------------------------------------------------------------------------------------------ release (2026-11-12)

// Fists review C1 / C2: the hands dropped from the guard in 3-4 frames ~0.3 s after the last strike (hold 0.3 s, fade 0.15 s).
// Now: a melee WEAPON state is held 0.55 s after the melee bits clear and the mask eases out over 0.35 s; the fade IN, the gun bash
// (QuickMelee: MeleeWeapon NotReady) and every closing gate are as before.
static void TestRelease()
{
    const float dt = 1.0f / 60.0f;
    const MeleeGateConfig cfg;
    const float kDeb = 0.06f;
    Expect("release: constants (hold 0.55 / 0.3 short, fade out 0.35, fade in 0.15 unchanged, legs 0.2 in / 0.35 out)",
           cfg.releaseHold == 0.55f && cfg.releaseHoldShort == 0.3f && kMeleeReleaseFadeOutSeconds == 0.35f && kMaskFadeSeconds == 0.15f &&
               melee::kFadeSeconds == 0.2f && melee::kReleaseFadeSeconds == 0.35f && kMeleeReleaseHoldSeconds == cfg.releaseHold);

    // ---- gate hold: melee weapon 0.55 s, gun bash 0.3 s
    for (int weapon = 1; weapon >= 0; --weapon)
    {
        MeleeGate g;
        for (int i = 0; i < 30; ++i)
            g.Update(kPsmMeleeAttack, dt, cfg, kDeb, weapon != 0); // 0.5 s of strikes
        int heldFrames = 0;
        bool dropped = false;
        for (int i = 0; i < 90 && !dropped; ++i)
        {
            if (g.Update(0u, dt, cfg, kDeb, false)) // the state of the last feed with melee bits decides, not this feed's argument
                ++heldFrames;
            else
                dropped = true;
        }
        const float held = heldFrames * dt;
        const float want = weapon ? 0.55f : 0.3f;
        Expect(weapon ? "release: melee weapon -> melee gate held 0.55 s after the bits clear" : "release: gun bash (QuickMelee, MeleeWeapon NotReady) -> held 0.3 s as before",
               dropped && std::fabs(held - want) <= dt + 1e-4f && g.LongRelease() == (weapon != 0), Fmt("held %.3f s", held));
    }
    {
        // a new strike inside the hold keeps the gate (no drop, no new episode: attacks chain) and the hold restarts from 0
        MeleeGate g;
        for (int i = 0; i < 12; ++i)
            g.Update(kPsmMeleeAttack, dt, cfg, kDeb, true);
        bool alwaysOn = true;
        for (int i = 0; i < 30; ++i) // 0.5 s: inside the hold
            alwaysOn = alwaysOn && g.Update(0u, dt, cfg, kDeb, true);
        alwaysOn = alwaysOn && g.Update(kPsmMeleePose, dt, cfg, kDeb, true); // block 0.5 s after the strike
        const uint64_t episodes = g.Episodes();
        for (int i = 0; i < 25; ++i) // 0.42 s of hold again (a hold counted from the first release would end at 0.55 s)
            alwaysOn = alwaysOn && g.Update(0u, dt, cfg, kDeb, true);
        Expect("release: bits again inside the hold -> gate stays (immediate re-entry, same episode, hold restarts)", alwaysOn && episodes == 1 && g.Episodes() == 1);
        MeleeGate z;
        for (int i = 0; i < 10; ++i)
            z.Update(kPsmMeleeAttack, dt, cfg, kDeb, true);
        Expect("release: gate is active before it is disabled", z.Active() && z.LongRelease());
        MeleeGateConfig off;
        off.enabled = false;
        Expect("release: gate disabled (UTPPSetMMMeleeGate off) ends the hold at once", !z.Update(0u, dt, off, kDeb, true) && !z.Active() && !z.LongRelease());
    }

    // ---- the mask: hold 0.55 s with the mask wanted, then the eased 0.35 s fade out; the legs are back on MM before the arms move
    {
        MeleeGate g;
        UpperMaskBlend mask, legs, tilt;
        melee::Gate fg;
        auto step = [&](uint32_t aBits, bool aStateAllowed, float& aMaskW, float& aLegW) {
            const bool active = g.Update(aBits, dt, cfg, kDeb, true);
            melee::Input in = StandingMelee();
            in.meleeMask = active;
            in.meleeState = aStateAllowed ? melee::kMwComboAttack : melee::kMwIdle; // Idle 2: no third-person clips -> legs off at once
            const bool full = fg.Update(in);
            const float maskOut = (!active && g.LongRelease()) ? kMeleeReleaseFadeOutSeconds : kMaskFadeSeconds;
            aMaskW = mask.UpdateInOut(active, true, true, dt, kMaskFadeSeconds, maskOut);
            tilt.UpdateInOut(active, true, true, dt, kMaskFadeSeconds, maskOut);
            aLegW = legs.UpdateInOut(full, true, true, dt, melee::kFadeSeconds, melee::LegFadeOutSeconds(fg.LastWhy(), g.LongRelease()));
        };
        float mw = 0.0f, lw = 0.0f;
        for (int i = 0; i < 40; ++i) // 0.67 s of strikes: mask and legs fully in (fade IN 0.15 / 0.2 s unchanged)
            step(kPsmMeleeAttack, true, mw, lw);
        Expect("release: strikes -> mask and legs at weight 1 (fade in unchanged)", mw == 1.0f && lw == 1.0f);
        // fade IN is unchanged: a fresh blend reaches 1 in 0.15 s with the new two-time call
        UpperMaskBlend fresh;
        int inFrames = 0;
        while (fresh.Progress() < 1.0f && inFrames < 100)
        {
            fresh.UpdateInOut(true, true, true, dt, kMaskFadeSeconds, kMeleeReleaseFadeOutSeconds);
            ++inFrames;
        }
        Expect("release: fade IN still 0.15 s (9 frames at 60 Hz, +1 for float rounding) with the slow out time set", inFrames == 9 || inFrames == 10, Fmt("frames %g", inFrames));
        // release: state ends (Idle 2). Track the mask and the legs frame by frame
        std::vector<float> maskW, legW;
        float prevM = 1.0f, maxStepM = 0.0f, prevL = 1.0f, maxStepL = 0.0f;
        int firstDropMask = -1, firstDropLeg = -1, legsZero = -1, maskZero = -1;
        bool monoM = true, monoL = true;
        for (int i = 0; i < 90; ++i)
        {
            step(0u, false, mw, lw);
            maskW.push_back(mw);
            legW.push_back(lw);
            monoM = monoM && mw <= prevM + 1e-6f;
            monoL = monoL && lw <= prevL + 1e-6f;
            maxStepM = std::fmax(maxStepM, std::fabs(mw - prevM));
            maxStepL = std::fmax(maxStepL, std::fabs(lw - prevL));
            if (firstDropMask < 0 && mw < 0.999f)
                firstDropMask = i;
            if (firstDropLeg < 0 && lw < 0.999f)
                firstDropLeg = i;
            if (legsZero < 0 && lw == 0.0f)
                legsZero = i;
            if (maskZero < 0 && mw == 0.0f)
                maskZero = i;
            prevM = mw;
            prevL = lw;
        }
        const float maskStartS = (firstDropMask + 1) * dt, maskEndS = (maskZero + 1) * dt, legsEndS = (legsZero + 1) * dt;
        Expect("release: the mask holds at weight 1 for 0.55 s after the state ends, then fades (monotonic)",
               monoM && std::fabs(maskStartS - 0.55f) <= 2 * dt + 1e-4f && maskW[std::max(firstDropMask - 1, 0)] == 1.0f,
               Fmt("starts at %.3f s", maskStartS));
        Expect("release: the mask eases out over 0.35 s (to 0 at 0.90 s), no frame step above 1.5 x dt / 0.35 (no pop, was 0.15 s)",
               std::fabs((maskEndS - maskStartS) - 0.35f) <= 2 * dt + 1e-4f && maxStepM <= 1.5f * dt / kMeleeReleaseFadeOutSeconds + 1e-4f,
               Fmt("fade %.3f s, max step %.4f", maskEndS - maskStartS, maxStepM));
        Expect("release: smoothstep ease (the first fade-out frames move by less than a linear ramp)",
               (1.0f - maskW[firstDropMask]) < 0.5f * dt / kMeleeReleaseFadeOutSeconds && maskW[maskZero - 1] < 0.5f * dt / kMeleeReleaseFadeOutSeconds,
               Fmt("first step %.5f, last weight %.5f", 1.0f - maskW[firstDropMask], maskW[maskZero - 1]));
        Expect("release: legs go back to MM at once with an eased 0.35 s fade (monotonic, no pop) ...",
               monoL && firstDropLeg <= 1 && std::fabs(legsEndS - 0.35f) <= 3 * dt + 1e-4f && maxStepL <= 1.5f * dt / melee::kReleaseFadeSeconds + 1e-4f,
               Fmt("legs done at %.3f s, max step %.4f", legsEndS, maxStepL));
        Expect("release: ... and are fully back before the arms start to fade (no leg and arm move in the same instant)", legsEndS < maskStartS - 0.15f,
               Fmt("legs %.3f s, arms start %.3f s", legsEndS, maskStartS));
        Expect("release: tilt weight follows the mask fade", std::fabs(tilt.Weight() - mask.Weight()) < 1e-6f);

        // re-entry inside the hold: the mask never leaves 1; re-entry mid fade-out: fades in at the unchanged 0.15 s rate from where it is
        MeleeGate g2;
        UpperMaskBlend m2;
        for (int i = 0; i < 40; ++i)
            m2.UpdateInOut(g2.Update(kPsmMeleeAttack, dt, cfg, kDeb, true), true, true, dt, kMaskFadeSeconds, kMeleeReleaseFadeOutSeconds);
        float minInHold = 1.0f;
        for (int i = 0; i < 25; ++i) // 0.42 s of hold, then a new strike
        {
            const bool a = g2.Update(0u, dt, cfg, kDeb, true);
            minInHold = std::fmin(minInHold, m2.UpdateInOut(a, true, true, dt, kMaskFadeSeconds, a ? kMaskFadeSeconds : kMeleeReleaseFadeOutSeconds));
        }
        const bool a1 = g2.Update(kPsmMeleeAttack, dt, cfg, kDeb, true);
        minInHold = std::fmin(minInHold, m2.UpdateInOut(a1, true, true, dt, kMaskFadeSeconds, kMeleeReleaseFadeOutSeconds));
        Expect("release: a strike inside the hold -> the mask never moves off 1 (punches start at once)", minInHold == 1.0f);
        for (int i = 0; i < 45; ++i) // 0.75 s after the last strike: mid fade-out
        {
            const bool a = g2.Update(0u, dt, cfg, kDeb, true);
            m2.UpdateInOut(a, true, true, dt, kMaskFadeSeconds, a ? kMaskFadeSeconds : kMeleeReleaseFadeOutSeconds);
        }
        // the gate is off here (hold over), so the new strike goes through the existing 0.06 s debounce (fade-out goes on meanwhile);
        // on the first frame the gate is active again the mask rises by dt / 0.15 from where it is
        float beforeP = m2.Progress();
        bool a2 = false;
        int guard = 0;
        while (!a2 && guard++ < 20)
        {
            beforeP = m2.Progress();
            a2 = g2.Update(kPsmMeleeAttack, dt, cfg, kDeb, true);
            m2.UpdateInOut(a2, true, true, dt, kMaskFadeSeconds, a2 ? kMaskFadeSeconds : kMeleeReleaseFadeOutSeconds);
        }
        Expect("release: a strike after the hold, mid fade-out -> (after the unchanged 0.06 s debounce) progress rises by dt / 0.15 from where it was",
               a2 && beforeP > 0.0f && beforeP < 1.0f && std::fabs(m2.Progress() - (beforeP + dt / kMaskFadeSeconds)) < 1e-5f,
               Fmt("from %.3f to %.3f", beforeP, m2.Progress()));
    }

    // ---- UpperMaskBlend: UpdateInOut with equal times = Update bit for bit (every other mask user is unchanged)
    {
        UpperMaskBlend a, b;
        bool same = true;
        uint32_t seed = 12345u;
        for (int i = 0; i < 4000; ++i)
        {
            seed = seed * 1664525u + 1013904223u;
            const bool wanted = (seed >> 28) & 1u, open = (seed >> 27) & 1u, visible = ((seed >> 26) & 7u) != 0, rel = ((seed >> 25) & 7u) == 0;
            const float time = (seed >> 24) & 1u ? 0.15f : 0.25f;
            const float d = ((seed >> 8) & 0xFFu) / 2000.0f;
            const float wa = a.Update(wanted, open, visible, d, time, rel);
            const float wb = b.UpdateInOut(wanted, open, visible, d, time, time, rel);
            same = same && wa == wb && a.Progress() == b.Progress();
        }
        Expect("release: UpdateInOut(in == out) equals Update bit for bit over 4000 random frames", same);
        UpperMaskBlend c;
        c.UpdateInOut(true, true, true, 10.0f, 0.15f, 0.35f);
        c.UpdateInOut(false, false, true, 0.2f, 0.15f, 0.35f);
        Expect("release: closed gate still holds the weight with the slow out time (the whole body fades out, no flash)", c.Weight() == 1.0f);
        c.UpdateInOut(false, true, true, 0.175f, 0.15f, 0.35f);
        Expect("release: out time is the one used while not wanted (0.175 s of 0.35 s -> progress 0.5)", std::fabs(c.Progress() - 0.5f) < 1e-5f);
    }

    // ---- leg fade out time: slow only when the melee state ended after a melee weapon
    Expect("release: LegFadeOutSeconds - state / melee after a melee weapon 0.35, every other reason 0.2, gun bash 0.2",
           melee::LegFadeOutSeconds(melee::kWhyState, true) == 0.35f && melee::LegFadeOutSeconds(melee::kWhyMelee, true) == 0.35f &&
               melee::LegFadeOutSeconds(melee::kWhyState, false) == 0.2f && melee::LegFadeOutSeconds(melee::kWhySpeed, true) == 0.2f &&
               melee::LegFadeOutSeconds(melee::kWhyAir, true) == 0.2f && melee::LegFadeOutSeconds(melee::kWhyDetail, true) == 0.2f &&
               melee::LegFadeOutSeconds(melee::kWhyStance, true) == 0.2f && melee::LegFadeOutSeconds(melee::kWhyOwner, true) == 0.2f &&
               melee::LegFadeOutSeconds(melee::kWhyFlag, true) == 0.2f && melee::LegFadeOutSeconds(melee::kWhyOn, true) == 0.2f);

    // ---- gates that end the mask AT ONCE even inside the (longer) hold: DecideUpperMask with the melee reason active
    {
        UpperMaskInput mi;
        mi.meleeActive = true;
        mi.states = kMaskStatesAll;
        mi.gate = kGateMelee;
        UpperMaskDecision d = DecideUpperMask(mi);
        Expect("release gates: only the melee gate -> melee mask reason (the hold)", d.reasons == kMaskStateMelee && d.upperFromGame);
        struct Case
        {
            const char* name;
            uint32_t gateExtra;
            uint32_t psm;
            int detail;
            float vz;
        };
        const Case cases[] = {
            {"vehicle (PSM vehicle bit)", kGatePsmBlock, kPsmVehicle, -1, 0.0f},
            {"scene", kGatePsmBlock, kPsmScene, -1, 0.0f},
            {"swimming", kGatePsmBlock, kPsmSwimming, -1, 0.0f},
            {"workspot", kGatePsmBlock, kPsmWorkspot, -1, 0.0f},
            {"takedown / grapple", kGatePsmBlock, kPsmTakedown, -1, 0.0f},
            {"body carry", kGatePsmBlock, kPsmCarry, -1, 0.0f},
            {"death / vitals", kGatePsmBlock, kPsmVitals, -1, 0.0f},
            {"menu / photo mode", kGatePsmBlock, kPsmPhotoOrMenu, -1, 0.0f},
            {"stagger (reaction)", kGatePsmBlock, kPsmReaction, -1, 0.0f},
            {"jump (LocomotionDetailed 18)", kGatePsmBlock | kGateLocoState, kPsmDetailed, 18, 0.0f},
            {"fall (14)", kGatePsmBlock | kGateLocoState, kPsmDetailed, 14, 0.0f},
            {"landing (24)", kGatePsmBlock | kGateLocoState, kPsmDetailed, 24, 0.0f},
            {"dodge (7)", kGatePsmBlock | kGateLocoState, kPsmDetailed, 7, 0.0f},
            {"slide (5)", kGatePsmBlock | kGateLocoState, kPsmDetailed, 5, 0.0f},
            {"climb / vault (8)", kGatePsmBlock | kGateLocoState, kPsmDetailed, 8, 0.0f},
            {"ladder (10)", kGatePsmBlock | kGateLocoState, kPsmDetailed, 10, 0.0f},
            {"AimWalk with vertical speed (jump rising)", kGatePsmBlock | kGateLocoState, kPsmDetailed, 2, 3.0f},
            {"over-speed (dash)", kGateOverSpeed, 0u, -1, 0.0f},
            {"stale feed", kGateStaleFeed, 0u, -1, 0.0f},
            {"TPP off", kGateTppInactive, 0u, -1, 0.0f},
            {"FK fault", kGateFault, 0u, -1, 0.0f},
            {"MM database not mapped", kGateNotMapped, 0u, -1, 0.0f},
            {"MM disabled", kGateMMDisabled, 0u, -1, 0.0f},
        };
        bool all = true;
        std::string failed;
        for (const Case& c : cases)
        {
            UpperMaskInput x = mi;
            x.gate = kGateMelee | c.gateExtra;
            x.psm = c.psm;
            x.psmDetail = c.detail;
            x.vz = c.vz;
            const UpperMaskDecision dd = DecideUpperMask(x);
            if (dd.reasons != 0)
            {
                all = false;
                failed += std::string(" ") + c.name;
            }
        }
        Expect("release gates: vehicle, scene, swim, workspot, takedown, carry, death, menu, stagger, jump / fall / land / dodge / slide / climb / ladder, "
               "vertical speed, over-speed, stale feed, TPP off, fault, not mapped, MM off all end the mask on the very feed (reasons 0) even in the hold",
               all, failed);
        // not a gate: AimWalk (melee block / targeting) still holds
        UpperMaskInput blk = mi;
        blk.gate = kGateMelee | kGatePsmBlock | kGateLocoState;
        blk.psm = kPsmDetailed;
        blk.psmDetail = kPsmDetailAimWalk;
        d = DecideUpperMask(blk);
        Expect("release gates: AimWalk (fists block / targeting) is covered by the mask as before", d.reasons == kMaskStateMelee);
    }
}

static void TestStandingGrounded()
{
    // the chest base correction reads this predicate: stance + detail + air, on their own (LastWhy only keeps the FIRST failing reason)
    melee::Gate g;
    melee::Input in;
    in.stance = melee::kStanceStand;
    in.psmDetail = 1;
    in.dt = 0.016f;
    in.flag = false; // the first failing reason is the flag ...
    g.Update(in);
    Expect("standing: flag off (Why flag) but stance Stand + detail Stand + grounded -> StandingGrounded", g.LastWhy() == melee::kWhyFlag && g.StandingGrounded());
    in.stance = 2; // crouch
    g.Update(in);
    Expect("standing: flag off + crouch -> not standing although Why still says flag", g.LastWhy() == melee::kWhyFlag && !g.StandingGrounded());
    in.stance = melee::kStanceStand;
    in.psmDetail = 4; // sprint
    g.Update(in);
    Expect("standing: flag off + LocomotionDetailed sprint -> not standing", g.LastWhy() == melee::kWhyFlag && !g.StandingGrounded());
    in.psmDetail = 1;
    in.haveAir = true;
    in.onGround = false;
    for (int i = 0; i < 10; ++i)
        g.Update(in);
    Expect("standing: flag off + ungrounded for 0.16 s -> not standing", !g.StandingGrounded());
    in.onGround = true;
    in.flag = true;
    in.meleeMask = true;
    in.owner = true;
    in.meleeState = melee::kMwComboAttack;
    g.Update(in);
    Expect("standing: everything on -> StandingGrounded and Why on", g.LastWhy() == melee::kWhyOn && g.StandingGrounded());
    in.meleeState = melee::kMwIdle; // state not allowed (Why state): still standing
    g.Update(in);
    Expect("standing: state not allowed (Why state) is still standing (the Idle release hold keeps the correction)", g.LastWhy() == melee::kWhyState && g.StandingGrounded());
    g.Reset();
    Expect("standing: Reset clears it", !g.StandingGrounded());
}

// ------------------------------------------------------------------------------------------------ guard sequences (2026-11-13)

// The Feed wiring of the melee mask (MotionHost.cpp Feed: MeleeGate -> GuardGate -> CombineDriverTerms -> MaskTimeTracker ->
// UpperMaskBlend, the tilt and the leg fades), driven with the real header classes over a PSM timeline: punch - pause - punch - walk
// - sprint - stand. With UTPP_GUARD_TIMELINE_DIR set the per-frame weights are written as CSV for the offline figures
// (tools\melee\fists_guard_figures.py).
namespace
{
struct Seg
{
    const char* name;
    float seconds;
    int state;      // PSM.MeleeWeapon
    int detail = 1; // LocomotionDetailed
    int stance = 1; // conditioned Locomotion state
    int upper = 0;  // PSM.UpperBody
    bool stick = false;
    float speed = 0.0f;   // planar capsule speed m/s
    bool hardGate = false; // vehicle / scene: the raw gate is closed by something the mask does not cover
    bool airborne = false;
};

struct Frame
{
    float t;
    int state;
    int detail;
    int stance;
    bool stick;
    float mask, tilt, legs;
    bool guard, active;
    int why;
};

struct SimCfg
{
    bool guardOn = false;
    bool tiltOn = false;
    bool legsOn = false; // melee.guardFullBody
    float releaseHold = kMeleeReleaseHoldSeconds;
    float releaseOut = kMeleeReleaseFadeOutSeconds;
    float dt = 1.0f / 60.0f;
};

std::vector<Frame> RunTimeline(const std::vector<Seg>& aSegs, const SimCfg& aCfg)
{
    MeleeGateConfig cfg;
    cfg.releaseHold = aCfg.releaseHold;
    MeleeGate gate;
    melee::GuardGate gg;
    melee::Gate fg;
    melee::MaskTimeTracker times;
    UpperMaskBlend mask, tilt, legs;
    std::vector<Frame> out;
    float t = 0.0f;
    const float dt = aCfg.dt;
    for (const Seg& s : aSegs)
    {
        const int n = int(std::lround(s.seconds / dt));
        for (int i = 0; i < n; ++i, t += dt)
        {
            uint32_t bits = 0;
            if (s.state >= melee::kMwComboAttack && s.state <= melee::kMwEquipAttack)
                bits = kPsmMeleeAttack;
            else if (s.state >= melee::kMwParried && s.state <= melee::kMwDeflect)
                bits = kPsmMeleePose;
            const bool active = gate.Update(bits, dt, cfg, 0.06f, s.state >= kPsmMeleeWeaponIdle);
            melee::GuardInput gi = GuardStanding();
            gi.setting = aCfg.guardOn;
            gi.meleeState = s.state;
            gi.upperBody = s.upper;
            gi.stance = s.stance;
            gi.psmDetail = s.detail;
            gi.onGround = !s.airborne;
            gi.gateOpen = !s.hardGate;
            gi.dt = dt;
            const bool guard = gg.Update(gi) == melee::kGuardOn;
            melee::DriverIn di;
            di.maskReasonsUpper = active;
            di.meleeMask = active;
            di.guardWanted = guard;
            di.guardFullSetting = aCfg.legsOn;
            di.guardTiltSetting = aCfg.tiltOn;
            const melee::DriverTerms terms = melee::CombineDriverTerms(di);
            melee::MaskTimeIn mti;
            mti.meleeReasonOnly = active;
            mti.guardWanted = guard;
            mti.guardSoft = gg.LastSoft();
            mti.longRelease = gate.LongRelease();
            mti.maskProgress = mask.Progress();
            const melee::MaskTimes mt = times.Update(terms, mti, kMaskFadeSeconds, 0.25f, aCfg.releaseOut);
            const bool rawGateOpen = !active && !s.hardGate;
            mask.UpdateInOut(terms.maskWanted, rawGateOpen, true, dt, mt.in, mt.out);
            tilt.UpdateInOut(terms.tiltWanted, true, true, dt, kMaskFadeSeconds, mt.tiltOut);
            melee::Input fi = StandingMelee();
            fi.meleeState = s.state;
            fi.allowedStates = 3669472u; // fists line of utpp_melee.reds UTPPMeleeFullBodyStates
            fi.meleeMask = terms.fullMelee;
            fi.guard = terms.fullGuard;
            fi.stance = s.stance;
            fi.psmDetail = s.detail;
            fi.stickHeld = s.stick;
            fi.planarSpeed = s.speed;
            fi.dt = dt;
            const bool full = fg.Update(fi);
            legs.UpdateInOut(full, !s.hardGate, true, dt, melee::kFadeSeconds, melee::LegFadeOutSeconds(fg.LastWhy(), gate.LongRelease()));
            Frame f;
            f.t = t;
            f.state = s.state;
            f.detail = s.detail;
            f.stance = s.stance;
            f.stick = s.stick;
            f.mask = mask.Weight();
            f.tilt = tilt.Weight();
            f.legs = legs.Weight();
            f.guard = guard;
            f.active = active;
            f.why = int(gg.LastWhy());
            out.push_back(f);
        }
    }
    return out;
}

// punch - pause - punch - pause - walk - sprint - stand, fists drawn (Idle 2 since the draw). A click enters Hold 6 for 0.1 s, the
// strike is ComboAttack 11 for 0.85 s (state=11 -> Idle 2 in the logged run 2026-11-10 07:26:12-13).
std::vector<Seg> PunchPausePunchWalk()
{
    std::vector<Seg> v;
    v.push_back({"idle", 1.0f, melee::kMwIdle});
    v.push_back({"hold", 0.1f, melee::kMwHold});
    v.push_back({"punch 1", 0.85f, melee::kMwComboAttack});
    v.push_back({"pause", 2.0f, melee::kMwIdle});
    v.push_back({"hold", 0.1f, melee::kMwHold});
    v.push_back({"punch 2", 0.85f, melee::kMwComboAttack});
    v.push_back({"pause", 1.5f, melee::kMwIdle});
    Seg walk{"walk", 2.5f, melee::kMwIdle};
    walk.stick = true;
    walk.speed = 1.6f;
    v.push_back(walk);
    Seg sprint{"sprint", 2.0f, melee::kMwIdle};
    sprint.stick = true;
    sprint.speed = 6.0f;
    sprint.stance = 3;
    sprint.detail = 4;
    v.push_back(sprint);
    v.push_back({"stand", 1.5f, melee::kMwIdle});
    return v;
}

float TimeOf(const std::vector<Frame>& aF, float aT0, float aT1, float& aMin, float& aMaxStep)
{
    aMin = 1.0f;
    aMaxStep = 0.0f;
    float prev = -1.0f;
    for (const Frame& f : aF)
        if (f.t >= aT0 && f.t < aT1)
        {
            aMin = std::fmin(aMin, f.mask);
            if (prev >= 0.0f)
                aMaxStep = std::fmax(aMaxStep, std::fabs(f.mask - prev));
            prev = f.mask;
        }
    return aMin;
}

void DumpTimeline(const char* aName, const std::vector<Frame>& aF)
{
#ifdef _WIN32
    char* dir = nullptr;
    size_t len = 0;
    if (_dupenv_s(&dir, &len, "UTPP_GUARD_TIMELINE_DIR") != 0 || !dir || !*dir)
    {
        std::free(dir);
        return;
    }
    const std::string path = std::string(dir) + "\\timeline_" + aName + ".csv";
    std::free(dir);
    FILE* f = nullptr;
    if (fopen_s(&f, path.c_str(), "wb") == 0 && f)
    {
        std::fprintf(f, "t,state,detail,stance,stick,mask,tilt,legs,guard,active,why\n");
        for (const Frame& r : aF)
            std::fprintf(f, "%.5f,%d,%d,%d,%d,%.6f,%.6f,%.6f,%d,%d,%d\n", r.t, r.state, r.detail, r.stance, r.stick ? 1 : 0, r.mask, r.tilt, r.legs,
                         r.guard ? 1 : 0, r.active ? 1 : 0, r.why);
        std::fclose(f);
    }
#else
    (void)aName;
    (void)aF;
#endif
}
} // namespace

static void TestGuardSequences()
{
    const float dt = 1.0f / 60.0f;
    const std::vector<Seg> seq = PunchPausePunchWalk();
    // t of the segment starts: idle 0, hold 1.0, punch 1 1.1, pause 1.95, hold 3.95, punch 2 4.05, pause 4.9, walk 6.4, sprint 8.9, stand 10.9
    const float tPunch1End = 1.95f, tPunch2Start = 3.95f, tPunch2End = 4.9f, tWalk = 6.4f, tSprint = 8.9f, tStand = 10.9f;

    SimCfg before;           // the defaults of main (guard off): hold 0.55 s, eased fade out 0.35 s
    SimCfg beforeOld = before; // the behaviour the review found (C1 / C2): hold 0.3 s, fade out 0.15 s
    beforeOld.releaseHold = 0.3f;
    beforeOld.releaseOut = kMaskFadeSeconds;
    SimCfg after = before;
    after.guardOn = true;
    after.tiltOn = true;

    const std::vector<Frame> fOld = RunTimeline(seq, beforeOld);
    const std::vector<Frame> fBefore = RunTimeline(seq, before);
    const std::vector<Frame> fAfter = RunTimeline(seq, after);
    DumpTimeline("before_review", fOld);
    DumpTimeline("before_main", fBefore);
    DumpTimeline("after_guard", fAfter);

    float mn = 0.0f, st = 0.0f;
    // guard off: the mask is off before the first strike and drops in the pauses and the walk (the "dip" the guard removes)
    TimeOf(fBefore, 0.0f, 1.0f, mn, st);
    Expect("sequence, guard off: idle before the first strike -> mask 0 (arms on the locomotion)", mn == 0.0f);
    TimeOf(fBefore, tPunch1End + 1.2f, tPunch2Start, mn, st);
    Expect("sequence, guard off: the 2 s pause between the punches drops the hands off the guard (mask < 0.05 before the next click)", mn < 0.05f,
           Fmt("min %.3f", mn));
    TimeOf(fBefore, tWalk, tSprint, mn, st);
    Expect("sequence, guard off: walking after the punches = mask 0", mn == 0.0f);

    // guard on: the mask goes up once, stays at 1 through the strikes, the pause and the walk, and only leaves for the sprint
    int firstOn = -1;
    for (size_t i = 0; i < fAfter.size(); ++i)
        if (fAfter[i].guard)
        {
            firstOn = int(i);
            break;
        }
    Expect("sequence, guard on: the guard is on from the first feed in Idle 2 (flag, setting, standing)", firstOn == 0);
    TimeOf(fAfter, 0.5f, tSprint, mn, st);
    Expect("sequence, guard on: from 0.5 s (raise done) through punch, pause, punch, pause and walk the mask never leaves 1", mn == 1.0f, Fmt("min %.4f", mn));
    TimeOf(fAfter, 0.0f, 0.5f, mn, st);
    Expect("sequence, guard on: the raise out of the locomotion arms is eased over kGuardFadeInSeconds (step <= 1.5 dt / 0.25)",
           st <= 1.5f * dt / melee::kGuardFadeInSeconds + 1e-4f && st > 0.0f, Fmt("max step %.4f", st));
    bool guardEveryFeed = true;
    for (const Frame& f : fAfter)
        if (f.t >= 0.0f && f.t < tSprint - dt)
            guardEveryFeed = guardEveryFeed && f.guard;
    Expect("sequence, guard on: MMGUARD never off between the first feed and the sprint (no off between the strikes, none for the walk)", guardEveryFeed);
    TimeOf(fAfter, tSprint, tSprint + 0.6f, mn, st);
    Expect("sequence, guard on: sprint (soft end) -> the arms ease out over 0.35 s (max step <= 1.5 dt / 0.35), not the quick 0.15 s",
           st <= 1.5f * dt / kMeleeReleaseFadeOutSeconds + 1e-4f && mn < 0.01f, Fmt("max step %.4f, min %.4f", st, mn));
    float legsMax = 0.0f;
    for (const Frame& f : fAfter)
        legsMax = std::fmax(legsMax, f.legs);
    Expect("sequence, guard on without melee.guardFullBody: the legs follow the existing rules (on only for the strikes)",
           legsMax > 0.9f);
    float legsPause = 1.0f;
    for (const Frame& f : fAfter)
        if (f.t >= tPunch1End + 0.8f && f.t < tPunch2Start)
            legsPause = std::fmin(legsPause, f.legs);
    Expect("sequence, guard on without guardFullBody: the legs are back on the locomotion in a 2 s pause (unchanged existing rule)", legsPause < 0.01f);
    // tilt: with guardAimTilt on it stays up through the pause (no fade in and out at every strike)
    float tiltMin = 1.0f;
    for (const Frame& f : fAfter)
        if (f.t >= 0.5f && f.t < tSprint)
            tiltMin = std::fmin(tiltMin, f.tilt);
    SimCfg noTilt = after;
    noTilt.tiltOn = false;
    const std::vector<Frame> fNoTilt = RunTimeline(seq, noTilt);
    float tiltMinOff = 1.0f;
    for (const Frame& f : fNoTilt)
        if (f.t >= 0.5f && f.t < tSprint)
            tiltMinOff = std::fmin(tiltMinOff, f.tilt);
    Expect("sequence, guard on: tilt weight stays 1 with guardAimTilt, but fades out in the pauses without it", tiltMin == 1.0f && tiltMinOff < 0.05f,
           Fmt("with %.3f, without %.3f", tiltMin, tiltMinOff));
    {
        // guard on, guardAimTilt off: after a strike the tilt unwinds with the eased 0.35 s (not the quick 0.15 s of the held mask)
        float maxStep = 0.0f, prev = -1.0f;
        for (const Frame& f : fNoTilt)
            if (f.t >= tPunch1End - 1.0f && f.t < tPunch2Start)
            {
                if (prev >= 0.0f && f.tilt < prev)
                    maxStep = std::fmax(maxStep, prev - f.tilt);
                prev = f.tilt;
            }
        Expect("sequence, guard on, guardAimTilt off: the tilt after a strike eases out over 0.35 s (step <= 1.5 dt / 0.35, not 0.15)",
               maxStep > 0.0f && maxStep <= 1.5f * dt / kMeleeReleaseFadeOutSeconds + 1e-4f, Fmt("max step %.4f", maxStep));
    }
    (void)tPunch2End;
    (void)tStand;

    // ---- legs with the stage-2 setting: standing in the guard the legs are on the game pose, the walk (stick) ends them
    {
        SimCfg legs = after;
        legs.legsOn = true;
        const std::vector<Frame> fl = RunTimeline(seq, legs);
        float lmin = 1.0f;
        for (const Frame& f : fl)
            if (f.t >= 1.0f && f.t < tWalk - 0.3f)
                lmin = std::fmin(lmin, f.legs);
        float lWalk = 1.0f;
        for (const Frame& f : fl)
            if (f.t >= tWalk + 0.5f && f.t < tSprint)
                lWalk = std::fmin(lWalk, f.legs);
        Expect("sequence, melee.guardFullBody on: legs stay on the game pose standing in the guard, the walk gives them back to the locomotion",
               lmin > 0.9f && lWalk < 0.01f, Fmt("standing min %.3f, walk %.3f", lmin, lWalk));
    }

    // ---- hard ends keep the quick fade: a jump (detail 18, ungrounded) and the UpperBody swap (holster / swap to a gun)
    {
        std::vector<Seg> v;
        v.push_back({"idle", 1.0f, melee::kMwIdle});
        Seg jump{"jump", 1.0f, melee::kMwIdle};
        jump.detail = 18;
        jump.airborne = true;
        v.push_back(jump);
        SimCfg c = after;
        const std::vector<Frame> f = RunTimeline(v, c);
        DumpTimeline("hard_jump", f);
        TimeOf(f, 1.0f, 2.0f, mn, st);
        Expect("sequence, hard end (jump): the quick 0.15 s fade (reaches 0, step <= 1.5 dt / 0.15, above the eased limit)", mn < 0.01f &&
               st > 1.5f * dt / kMeleeReleaseFadeOutSeconds && st <= 1.5f * dt / kMaskFadeSeconds + 1e-4f, Fmt("max step %.4f", st));
        std::vector<Seg> w;
        w.push_back({"idle", 1.0f, melee::kMwIdle});
        Seg swap{"swap", 1.0f, melee::kMwIdle};
        swap.upper = 1;
        w.push_back(swap);
        const std::vector<Frame> g = RunTimeline(w, c);
        DumpTimeline("hard_swap", g);
        TimeOf(g, 1.0f, 2.0f, mn, st);
        Expect("sequence, hard end (UpperBody SwitchItems: holster / swap to a gun): the quick fade", mn < 0.01f && st > 1.5f * dt / kMeleeReleaseFadeOutSeconds,
               Fmt("max step %.4f", st));
        // vehicle / scene: the raw gate closes the whole body; the mask weight is held, no arm pop under the fading body
        std::vector<Seg> x;
        x.push_back({"idle", 1.0f, melee::kMwIdle});
        Seg veh{"vehicle", 1.0f, melee::kMwIdle};
        veh.hardGate = true;
        x.push_back(veh);
        const std::vector<Frame> h = RunTimeline(x, c);
        TimeOf(h, 1.0f, 2.0f, mn, st);
        Expect("sequence, vehicle / scene (raw gate closed): the guard ends at once, the mask weight is held at 1 (the whole body fades, no arm pop)",
               mn == 1.0f && !h.back().guard && h.back().why == int(melee::kGuardGate));
    }

    // ---- soft end and a re-raise in the middle of the fade: continuous (the raise starts from where the arms are)
    {
        std::vector<Seg> v;
        v.push_back({"idle", 1.0f, melee::kMwIdle});
        Seg crouch{"crouch", 0.3f, melee::kMwIdle};
        crouch.stance = 2;
        crouch.detail = 3;
        v.push_back(crouch);
        v.push_back({"stand", 1.0f, melee::kMwIdle});
        const std::vector<Frame> f = RunTimeline(v, after);
        DumpTimeline("soft_crouch", f);
        TimeOf(f, 1.0f, 2.3f, mn, st);
        Expect("sequence, crouch for 0.3 s then stand: the arms ease down and back up with no frame step above the eased limit (no pop)",
               st <= 1.5f * dt / melee::kGuardFadeInSeconds + 1e-4f && mn > 0.0f && mn < 1.0f, Fmt("max step %.4f, min %.3f", st, mn));
    }

    // ---- Safe (a friendly NPC in reach) keeps the guard: Idle -> Safe -> Idle never drops the mask
    {
        std::vector<Seg> v;
        v.push_back({"idle", 1.0f, melee::kMwIdle});
        v.push_back({"safe", 1.0f, melee::kMwSafe});
        v.push_back({"idle", 1.0f, melee::kMwIdle});
        const std::vector<Frame> f = RunTimeline(v, after);
        TimeOf(f, 0.5f, 3.0f, mn, st);
        Expect("sequence, Idle -> Safe (friendly NPC) -> Idle: the guard and the mask stay on", mn == 1.0f);
        // the first-person melee_first_equip: Equipping 1 stays off the guard; the raise after the draw is the eased 0.25 s one
        std::vector<Seg> d;
        d.push_back({"equipping", 1.0f, melee::kMwEquipping});
        d.push_back({"idle", 1.0f, melee::kMwIdle});
        const std::vector<Frame> g = RunTimeline(d, after);
        TimeOf(g, 0.0f, 0.99f, mn, st); // (feed times are sums of dt: the first Idle frame sits at ~1.0)
        float mn2 = 0.0f, st2 = 0.0f;
        TimeOf(g, 0.99f, 2.0f, mn2, st2);
        Expect("sequence, draw: Equipping 1 keeps the locomotion arms (mask 0), the Idle 2 after it raises them eased (step <= 1.5 dt / 0.25)",
               mn == 0.0f && st == 0.0f && st2 > 0.0f && st2 <= 1.5f * dt / melee::kGuardFadeInSeconds + 1e-4f, Fmt("max raise step %.4f", st2));
    }

    // ---- 30 fps feeds (the game thread can run slower): the same sequence keeps the mask at 1 and the steps below the eased limit
    {
        SimCfg c = after;
        c.dt = 1.0f / 30.0f;
        const std::vector<Frame> f = RunTimeline(seq, c);
        TimeOf(f, 0.5f, tSprint, mn, st);
        float mn2 = 0.0f, st2 = 0.0f;
        TimeOf(f, tSprint, tSprint + 0.6f, mn2, st2);
        Expect("sequence at 30 fps feeds: mask 1 through punch / pause / punch / walk, the sprint fade out stays eased", mn == 1.0f &&
               st2 <= 1.5f * (1.0f / 30.0f) / kMeleeReleaseFadeOutSeconds + 1e-4f, Fmt("min %.3f, sprint step %.4f", mn, st2));
    }
}

int main()
{
    TestStandingGrounded();
    TestDecision();
    TestSpeedAndStick();
    TestAir();
    TestFade();
    TestBlendTerm();
    TestStates();
    TestSpeedBridge();
    TestGuard();
    TestDriverTerms();
    TestRelease();
    TestGuardSequences();
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
