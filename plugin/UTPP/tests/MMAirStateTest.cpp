// UTPP offline test: AirState phase machine (src\mm\AirState.hpp). Pure, no DB needed.
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "mm/AirState.hpp"

using namespace utpp::mm::air;

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...)                                                                                              \
    do                                                                                                                \
    {                                                                                                                 \
        ++g_checks;                                                                                                   \
        if (!(cond))                                                                                                  \
        {                                                                                                             \
            ++g_fail;                                                                                                 \
            std::printf("FAIL %s:%d: %s | ", __FILE__, __LINE__, #cond);                                              \
            std::printf(__VA_ARGS__);                                                                                 \
            std::printf("\n");                                                                                        \
        }                                                                                                             \
    } while (0)

struct Trace
{
    std::vector<Phase> phases;
    int plays[7] = {};
    float landEntryT = -1.0f, landEntryTtl = -1.0f;
    Entry landEntry = Entry::None;
    int landEntryFrames = -1;
    float touchT = -1.0f;
    bool sawHeavyLand = false;
    int resetQueryFrame = -2;
    bool handbackDwell = false;
    bool finiteOk = true;
    // Airborne invariants (episode phases TakeOff..LandApproach)
    bool airSuppressOk = true, airContactOk = true, airReleaseOk = true, airOwnsOk = true;
    float maxHoldRateDev = 0.0f;
    std::string Seq() const
    {
        std::string s;
        for (Phase p : phases) { s += PhaseName(p); s += ' '; }
        return s;
    }
    void Add(const AirDecision& d, float t)
    {
        if (phases.empty() || phases.back() != d.phase) phases.push_back(d.phase);
        ++plays[static_cast<int>(d.action)];
        if (d.action == Action::PlayLand && landEntryT < 0.0f)
        {
            landEntryT = t;
            landEntryTtl = d.ttl;
            landEntry = d.pick.entry;
            landEntryFrames = d.pick.entryFrames;
            sawHeavyLand = d.pick.weight == Weight::Heavy;
        }
        if (d.action == Action::Reset) resetQueryFrame = d.queryBoneFrame;
        if (d.action == Action::HandBack) handbackDwell = d.dwellExempt;
        if (!std::isfinite(d.rate) || !std::isfinite(d.contactScale) || !std::isfinite(d.ttl) || !std::isfinite(d.halflife))
            finiteOk = false;
        const bool air = d.phase == Phase::TakeOff || d.phase == Phase::Descend || d.phase == Phase::FallLoop ||
                         d.phase == Phase::LandApproach;
        if (air)
        {
            if (!d.suppressSearch) airSuppressOk = false;
            if (d.contactScale != 0.0f) airContactOk = false;
            if (!d.releaseOffsetRoot) airReleaseOk = false;
            if (!d.airOwns) airOwnsOk = false;
            if (d.rateIsMultiplier) maxHoldRateDev = std::fmax(maxHoldRateDev, std::fabs(d.rate - 0.5f));
        }
    }
};

static float Jit(int i, float dt, bool jitter) { return jitter ? dt * (1.0f + 0.2f * std::sin(1.7f * static_cast<float>(i))) : dt; }

struct JumpOpts
{
    float hz = 60.0f;
    bool jitter = true;
    float vz0 = 4.8f, gUp = 19.2f, gDown = 16.0f;
    float speed = 4.0f;
    bool strafe = false;
    float velX = 0.0f;
    bool stale = false;      // groundStale + garbage onGround
    int releaseDetailAt = -1; // feed index after take-off at which a release / climb detail appears
    int releaseDetail = 10;
    bool releaseFlag = false;
    int bogusProbeAt = -1;   // feed index after take-off: probe 0.05 m
    int nanAt = -1;          // feed index after take-off: NaN inputs
    int fallPrevAtTouch = 0; // psmFallPrev on the touchdown feed
    int fallPrevAfterTouch = 0; // psmFallPrev on the first feed after touchdown
    int rejumpAfterTouch = -1;  // feeds after touchdown: new jump (detail 18 edge)
    float stick = 1.0f;
    float stickTurnDeg = 0.0f; // AIR CLIPS stage
};

static float Touch(const JumpOpts& o)
{
    const float hA = o.vz0 * o.vz0 / (2.0f * o.gUp);
    return o.vz0 / o.gUp + std::sqrt(2.0f * hA / o.gDown);
}

static Trace RunJump(AirState& s, const JumpOpts& o)
{
    Trace tr;
    const float base = 1.0f / o.hz;
    int i = 0;
    int frame = 100;
    auto feed = [&](AirInput in, float t) {
        in.planarSpeed = o.speed;
        in.velBodyX = o.velX;
        in.velBodyY = std::sqrt(std::fmax(0.0f, o.speed * o.speed - o.velX * o.velX));
        in.strafeActive = o.strafe;
        in.stickMagnitude = o.stick;
        in.stickTurnDeg = o.stickTurnDeg;
        in.tableValid = true;
        in.currentFrame = frame++;
        if (o.stale) { in.groundStale = true; in.onGround = true; }
        tr.Add(s.Update(in), t);
    };
    for (float t = 0.0f; t < 0.3f; ++i)
    {
        AirInput in;
        in.dt = Jit(i, base, o.jitter);
        t += in.dt;
        feed(in, -1.0f);
    }
    const float T = Touch(o);
    float t = 0.0f;
    bool touched = false;
    float sinceTouch = 0.0f;
    int k = 0, kTouch = -1;
    for (; t < T + 1.2f; ++i, ++k)
    {
        AirInput in;
        in.dt = k == 0 ? Jit(i, base, o.jitter) : Jit(i, base, o.jitter);
        if (k > 0) t += in.dt;
        const float tA = o.vz0 / o.gUp;
        float z, vz;
        if (t < tA) { z = o.vz0 * t - 0.5f * o.gUp * t * t; vz = o.vz0 - o.gUp * t; }
        else
        {
            const float hA = o.vz0 * o.vz0 / (2.0f * o.gUp);
            const float u = t - tA;
            z = hA - 0.5f * o.gDown * u * u;
            vz = -o.gDown * u;
        }
        if (k > 0 && z <= 0.0f && !touched) { touched = true; kTouch = k; tr.touchT = t; }
        if (touched) { z = 0.0f; vz = 0.0f; sinceTouch = t - tr.touchT; }
        in.vz = vz;
        in.onGround = k == 0 || touched;
        in.psmDetail = touched ? (sinceTouch < 0.3f ? 23 : 0) : (vz > 0.0f || k == 0 ? 18 : 14);
        in.distToGround = -2.0f;
        if (k == o.bogusProbeAt) in.distToGround = 0.05f;
        if (k == o.releaseDetailAt) { in.psmDetail = o.releaseDetail; in.release = o.releaseFlag; }
        if (k == o.nanAt)
        {
            const float n = std::numeric_limits<float>::quiet_NaN();
            in.vz = n; in.dt = n; in.distToGround = n;
        }
        if (touched && k == kTouch) in.psmFallPrev = o.fallPrevAtTouch;
        if (touched && k == kTouch + 1) in.psmFallPrev = o.fallPrevAfterTouch;
        if (o.rejumpAfterTouch >= 0 && touched && k == kTouch + o.rejumpAfterTouch)
        {
            in.psmDetail = 18;
            in.onGround = true;
        }
        in.clipRemainSeconds = 0.0f;
        feed(in, t);
        if (o.rejumpAfterTouch >= 0 && touched && k == kTouch + o.rejumpAfterTouch) break;
    }
    return tr;
}

struct DropOpts
{
    float hz = 60.0f;
    float height = 2.0f;
    float g = 16.0f;
    float speed = 4.0f;
    bool probe = true;
    bool probeMiss = false;
};

static Trace RunDrop(AirState& s, const DropOpts& o)
{
    Trace tr;
    const float dt = 1.0f / o.hz;
    int frame = 500;
    for (int i = 0; i < 20; ++i)
    {
        AirInput in;
        in.dt = dt;
        in.planarSpeed = o.speed;
        in.velBodyY = o.speed;
        in.tableValid = true;
        in.currentFrame = frame++;
        tr.Add(s.Update(in), -1.0f);
    }
    float t = 0.0f;
    bool touched = false;
    for (int k = 0; t < 4.0f; ++k)
    {
        t += dt;
        float z = o.height - 0.5f * o.g * t * t, vz = -o.g * t;
        if (z <= 0.0f && !touched) { touched = true; tr.touchT = t; }
        if (touched) { z = 0.0f; vz = 0.0f; }
        AirInput in;
        in.dt = dt;
        in.vz = vz;
        in.onGround = touched;
        in.psmDetail = touched ? (t - tr.touchT < 0.3f ? 23 : 0) : (t > 0.1f ? 14 : 0);
        in.psmFallPrev = touched ? 0 : (t > 0.12f ? 1 : 0);
        in.distToGround = (!touched && o.probe && (k % 2) == 0) ? (o.probeMiss ? -1.0f : z) : -2.0f;
        in.planarSpeed = o.speed;
        in.velBodyY = o.speed;
        in.tableValid = true;
        in.currentFrame = frame++;
        tr.Add(s.Update(in), t);
        if (touched && t - tr.touchT > 1.0f) break;
    }
    return tr;
}

static AirState MakeClips()
{
    AirState s;
    s.SetMode(Mode::Clips);
    return s;
}

static void TestFlatJump()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
        for (float vz0 : {4.5f, 4.8f, 5.0f})
        {
            AirState s = MakeClips();
            JumpOpts o;
            o.hz = hz;
            o.vz0 = vz0;
            Trace tr = RunJump(s, o);
            const std::string seq = tr.Seq();
            CHECK(seq == "Ground TakeOff Descend LandApproach Recover Ground ", "hz %.0f vz0 %.1f seq %s", hz, vz0, seq.c_str());
            const float predicted = tr.landEntryT + tr.landEntryTtl;
            CHECK(tr.landEntryT >= 0.0f && std::fabs(predicted - tr.touchT) <= 0.05f, "hz %.0f ttl err %.3f", hz,
                  predicted - tr.touchT);
            CHECK(tr.plays[static_cast<int>(Action::PlayTakeOff)] == 1 && tr.plays[static_cast<int>(Action::PlayLand)] == 1,
                  "hz %.0f plays", hz);
            CHECK(tr.handbackDwell, "hz %.0f handback dwellExempt", hz);
            CHECK(tr.airSuppressOk && tr.airContactOk && tr.airReleaseOk && tr.airOwnsOk, "hz %.0f air invariants", hz);
            CHECK(!tr.sawHeavyLand && !s.Heavy(), "hz %.0f light land", hz);
            CHECK(std::fabs(s.GUp() - 19.2f) < 1.0f && std::fabs(s.GDown() - 16.0f) < 1.0f, "gravity %.2f %.2f", s.GUp(),
                  s.GDown());
            CHECK(s.Stats().lastTtlErrValid && std::fabs(s.Stats().lastTtlErr) <= 0.05f + 1.2f / hz, "internal ttl err %.3f",
                  s.Stats().lastTtlErr);
            CHECK(tr.finiteOk, "finite");
            // LandApproach lead: entered on the first feed with TTL <= landLead (0.20 s), i.e. TTL within one frame of it.
            const float frameMax = 1.25f / hz;
            CHECK(tr.landEntryTtl <= 0.20f + 1e-5f && tr.landEntryTtl > 0.20f - frameMax, "hz %.0f land lead ttl %.3f", hz,
                  tr.landEntryTtl);
            CHECK(tr.landEntry == Entry::TouchMinus && tr.landEntryFrames == static_cast<int>(std::lround(tr.landEntryTtl * 30.0f)),
                  "hz %.0f land entry policy %d frames %d", hz, static_cast<int>(tr.landEntry), tr.landEntryFrames);
            CHECK(tr.landEntryFrames >= 5 && tr.landEntryFrames <= 6, "hz %.0f land entry frames %d", hz, tr.landEntryFrames);
        }
}

static void TestStandingJumpAndDirection()
{
    AirState s = MakeClips();
    JumpOpts o;
    o.speed = 0.0f;
    o.vz0 = 4.5f;
    Trace tr = RunJump(s, o);
    CHECK(tr.Seq() == "Ground TakeOff Descend LandApproach Recover Ground ", "standing seq %s", tr.Seq().c_str());

    // Direction: strafe right at take-off -> R; same velocity without strafe -> F.
    for (bool strafe : {true, false})
    {
        AirState s2 = MakeClips();
        AirInput in;
        in.dt = 1.0f / 60.0f;
        in.tableValid = true;
        in.planarSpeed = 3.0f;
        in.velBodyX = 3.0f;
        in.strafeActive = strafe;
        s2.Update(in);
        in.psmDetail = 18;
        in.vz = 4.8f;
        const AirDecision d = s2.Update(in);
        CHECK(d.action == Action::PlayTakeOff && d.pick.kind == PickKind::Start, "take-off");
        CHECK(d.pick.dir == (strafe ? Dir::R : Dir::F), "dir strafe=%d got %d", strafe ? 1 : 0, static_cast<int>(d.pick.dir));
        CHECK(d.pick.entry == Entry::LiftoffMinus && d.pick.entryFrames == 2 && std::fabs(d.halflife - 0.15f) < 1e-6f,
              "entry policy");
        CHECK(d.pick.band == Band::Run, "band");
    }
    // Strafe switched on only in the take-off feed (not before) -> F (critique MUST: StrafeMode in the feed BEFORE take-off).
    {
        AirState s2 = MakeClips();
        AirInput in;
        in.dt = 1.0f / 60.0f;
        in.tableValid = true;
        in.planarSpeed = 3.0f;
        in.velBodyX = 3.0f;
        in.strafeActive = false;
        s2.Update(in);
        in.strafeActive = true;
        in.psmDetail = 18;
        in.vz = 4.8f;
        const AirDecision d = s2.Update(in);
        CHECK(d.action == Action::PlayTakeOff && d.pick.dir == Dir::F, "strafe on at take-off feed only -> F, got %d",
              static_cast<int>(d.pick.dir));
    }
}

static void TestTtlSign()
{
    // h above the landing plane >= 0; rising and falling symmetric case.
    const float t1 = AirState::TimeToLand(0.0f, 4.8f, 19.2f, 16.0f);
    const float hA = 4.8f * 4.8f / (2.0f * 19.2f);
    CHECK(std::fabs(t1 - (0.25f + std::sqrt(2.0f * hA / 16.0f))) < 1e-5f, "rise ttl %f", t1);
    CHECK(AirState::TimeToLand(0.0f, -3.0f, 19.2f, 16.0f) == 0.0f, "on plane descending -> 0");
    CHECK(std::fabs(AirState::TimeToLand(2.0f, 0.0f, 19.2f, 16.0f) - 0.5f) < 1e-5f, "2 m from rest at g16 -> 0.5 s");
    CHECK(AirState::TimeToLand(-0.2f, -1.0f, 19.2f, 16.0f) == 0.0f, "negative h clamped");
}

static void TestDrops()
{
    struct C { float h; bool heavy; const char* seq; };
    const C cases[] = {
        // 1.2 m: at minAir (0.1 s) TTL is ~0.29 s < fallMinTtl 0.35 -> no episode (step-down / short ledge). With
        // gDown 16 every drop that does enter (>= ~1.7 m) predicts >= 7 m/s impact, so a Light ledge land needs tuning.
        {1.2f, false, "Ground "},
        // 2 m at gDown 16: impact sqrt(2*16*2) = 8 m/s >= 7 -> Heavy by the 7 m/s rule.
        {2.0f, true, "Ground Descend LandApproach Recover Ground "},
        {6.0f, true, "Ground Descend FallLoop LandApproach Recover Ground "},
    };
    for (const C& c : cases)
        for (float hz : {30.0f, 60.0f, 144.0f})
        {
            AirState s = MakeClips();
            DropOpts o;
            o.height = c.h;
            o.hz = hz;
            Trace tr = RunDrop(s, o);
            CHECK(tr.Seq() == c.seq, "drop %.1f hz %.0f seq %s", c.h, hz, tr.Seq().c_str());
            CHECK(tr.sawHeavyLand == c.heavy, "drop %.1f hz %.0f heavy %d", c.h, hz, tr.sawHeavyLand ? 1 : 0);
            const float predicted = tr.landEntryT + tr.landEntryTtl;
            if (c.h < 1.5f)
            {
                CHECK(s.Stats().episodes == 0 && tr.landEntryT < 0.0f, "short drop: no episode");
                continue;
            }
            CHECK(tr.landEntryT >= 0.0f && std::fabs(predicted - tr.touchT) <= 0.15f, "drop %.1f hz %.0f ttl err %.3f",
                  c.h, hz, predicted - tr.touchT);
            CHECK(tr.plays[static_cast<int>(Action::PlayOff)] == 1, "drop Off clip at run speed");
            CHECK(s.Stats().falls == 1 && s.Stats().takeoffs == 0, "fall counted");
        }
    // Probe miss: no LandApproach before touchdown, late land at touchdown (not a hang).
    {
        AirState s = MakeClips();
        DropOpts o;
        o.height = 2.0f;
        o.probeMiss = true;
        Trace tr = RunDrop(s, o);
        CHECK(tr.landEntryT >= tr.touchT - 1e-4f, "probe miss: land at touchdown (%.3f vs %.3f)", tr.landEntryT, tr.touchT);
        CHECK(tr.phases.back() == Phase::Ground && tr.plays[static_cast<int>(Action::HandBack)] == 1, "probe miss handback");
    }
    // Standing drop -> fall loop; gravity learned (true g 19 vs default 16).
    {
        AirState s = MakeClips();
        DropOpts o;
        o.height = 3.0f;
        o.speed = 0.0f;
        o.g = 19.0f;
        Trace tr = RunDrop(s, o);
        CHECK(tr.plays[static_cast<int>(Action::PlayFallLoop)] >= 1 && tr.plays[static_cast<int>(Action::PlayOff)] == 0,
              "standing drop uses fall loop");
        CHECK(std::fabs(s.GDown() - 19.0f) <= 1.9f, "gDown learned %.2f", s.GDown());
    }
}

static void RunBlip(AirState& s, float hz, float dur, int detail, float vz, bool& anyAction)
{
    const float dt = 1.0f / hz;
    for (int i = 0; i < 10; ++i) { AirInput in; in.dt = dt; in.tableValid = true; if (s.Update(in).action != Action::None) anyAction = true; }
    for (float t = 0.0f; t < dur; t += dt)
    {
        AirInput in;
        in.dt = dt;
        in.onGround = false;
        in.psmDetail = detail;
        in.vz = vz;
        in.planarSpeed = 4.0f;
        in.tableValid = true;
        const AirDecision d = s.Update(in);
        if (d.action != Action::None || d.airOwns) anyAction = true;
    }
    for (int i = 0; i < 10; ++i) { AirInput in; in.dt = dt; in.tableValid = true; if (s.Update(in).action != Action::None) anyAction = true; }
}

static void TestNoTrigger()
{
    for (float dur : {0.04f, 0.1f, 0.2f, 0.32f})
        for (float hz : {30.0f, 60.0f, 144.0f})
        {
            AirState s = MakeClips();
            bool any = false;
            RunBlip(s, hz, dur, 14, -0.6f, any);
            CHECK(!any && s.Stats().episodes == 0, "blip %.2f hz %.0f", dur, hz);
        }
    // minAir guard: vz -2 (below fallVz -1), no probe (TTL = noHitTtl) but airborne < 0.10 s -> no episode.
    for (float hz : {30.0f, 60.0f, 144.0f})
    {
        AirState s = MakeClips();
        bool any = false;
        RunBlip(s, hz, 0.06f, 14, -2.0f, any);
        CHECK(!any && s.Stats().episodes == 0, "steep blip 0.06 hz %.0f", hz);
        AirState s2 = MakeClips();
        bool any2 = false;
        RunBlip(s2, hz, 0.2f, 14, -2.0f, any2);
        CHECK(any2 && s2.Stats().falls == 1, "steep 0.2 s hz %.0f: fall starts after minAir", hz);
    }
    // Climb / vault pull-up (vz > 1.5, not grounded), dodge-air, slide on a slope: never an episode.
    struct C { int detail; float vz; float dur; };
    for (C c : {C{8, 2.5f, 0.6f}, C{9, 2.0f, 0.4f}, C{7, 1.8f, 0.25f}, C{5, -1.5f, 0.6f}})
    {
        AirState s = MakeClips();
        bool any = false;
        RunBlip(s, 60.0f, c.dur, c.detail, c.vz, any);
        CHECK(!any && s.Stats().episodes == 0, "no-trigger detail %d", c.detail);
    }
    // Climb ends with an air detail still rising (mantle top): lockout until grounded.
    {
        AirState s = MakeClips();
        bool any = false;
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 30; ++i)
        {
            AirInput in;
            in.dt = dt;
            in.onGround = false;
            in.psmDetail = i < 20 ? 8 : 14;
            in.vz = 2.0f;
            in.tableValid = true;
            if (s.Update(in).action != Action::None) any = true;
        }
        CHECK(!any && s.Stats().takeoffs == 0, "climb -> air detail rising: no take-off");
    }
}

static void TestReleaseAndClimbMidAir()
{
    for (int detail : {10, 31, 9})
    {
        AirState s = MakeClips();
        JumpOpts o;
        o.releaseDetailAt = 12;
        o.releaseDetail = detail;
        Trace tr = RunJump(s, o);
        CHECK(tr.plays[static_cast<int>(Action::Reset)] == 1, "detail %d reset count %d", detail,
              tr.plays[static_cast<int>(Action::Reset)]);
        CHECK(tr.resetQueryFrame >= 100, "detail %d reset queryBoneFrame %d (last clean ground frame)", detail,
              tr.resetQueryFrame);
        CHECK(s.Stats().takeoffs == 1, "detail %d no new take-off after reset (%u)", detail, s.Stats().takeoffs);
        CHECK(s.CurrentPhase() == Phase::Ground, "back on ground");
    }
    {
        AirState s = MakeClips();
        JumpOpts o;
        o.releaseDetailAt = 12;
        o.releaseDetail = 14;
        o.releaseFlag = true;
        Trace tr = RunJump(s, o);
        CHECK(tr.plays[static_cast<int>(Action::Reset)] == 1 && s.Stats().takeoffs == 1, "release flag reset");
    }
    // External reset (DB swap) mid-air.
    {
        AirState s = MakeClips();
        AirInput in;
        in.dt = 1.0f / 60.0f;
        in.tableValid = true;
        in.currentFrame = 42;
        s.Update(in);
        in.psmDetail = 18;
        in.vz = 4.8f;
        in.currentFrame = 43;
        s.Update(in);
        s.RequestReset();
        in.onGround = false;
        in.vz = 4.0f;
        const AirDecision d = s.Update(in);
        CHECK(d.action == Action::Reset && d.queryBoneFrame == 43 && !d.airOwns, "external reset");
    }
}

static void TestStaleAndNaN()
{
    {
        AirState s = MakeClips();
        JumpOpts o;
        o.stale = true;
        Trace tr = RunJump(s, o);
        CHECK(tr.Seq() == "Ground TakeOff Descend LandApproach Recover Ground ", "stale seq %s", tr.Seq().c_str());
    }
    {
        AirState s = MakeClips();
        JumpOpts o;
        o.nanAt = 6;
        Trace tr = RunJump(s, o);
        CHECK(tr.finiteOk, "NaN mid-air outputs finite");
        CHECK(tr.phases.back() == Phase::Ground && s.Stats().lands == 1, "NaN mid-air still lands: %s", tr.Seq().c_str());
    }
    {
        AirState s = MakeClips();
        const float n = std::numeric_limits<float>::quiet_NaN();
        bool ok = true;
        for (int i = 0; i < 60; ++i)
        {
            AirInput in;
            in.dt = n;
            in.vz = n;
            in.distToGround = n;
            in.planarSpeed = n;
            in.velBodyX = n;
            in.velBodyY = n;
            in.stickMagnitude = n;
            in.clipRemainSeconds = n;
            in.clipRootSpeed = n;
            in.onGround = (i % 2) == 0;
            in.tableValid = true;
            const AirDecision d = s.Update(in);
            if (d.action != Action::None || !std::isfinite(d.rate) || !std::isfinite(d.contactScale)) ok = false;
        }
        CHECK(ok && s.Stats().episodes == 0, "NaN-only input: no episode, finite");
    }
}

static void TestHoldMode()
{
    AirState s;
    s.SetMode(Mode::Clips); // no valid table -> Hold
    JumpOpts o;
    Trace tr;
    {
        // RunJump sets tableValid = true; drive Hold through Mode::Hold instead.
        s.SetMode(Mode::Hold);
        tr = RunJump(s, o);
    }
    CHECK(tr.plays[static_cast<int>(Action::PlayTakeOff)] == 0 && tr.plays[static_cast<int>(Action::PlayLand)] == 0,
          "hold: no clip plays");
    CHECK(tr.airSuppressOk && tr.airContactOk && tr.airReleaseOk && tr.airOwnsOk, "hold air invariants");
    CHECK(tr.maxHoldRateDev < 1e-6f, "hold rate multiplier 0.5");
    CHECK(tr.plays[static_cast<int>(Action::HandBack)] == 1 && tr.handbackDwell, "hold handback dwellExempt");
    CHECK(tr.Seq() == "Ground TakeOff Descend LandApproach Ground ", "hold seq %s", tr.Seq().c_str());
    // Clips requested but table invalid -> effective Hold.
    AirState s2 = MakeClips();
    AirInput in;
    in.dt = 1.0f / 60.0f;
    in.tableValid = false;
    CHECK(s2.Update(in).effectiveMode == Mode::Hold, "clips without table -> hold");
    // Contact ramp after the hold hand-back.
    AirState s3;
    s3.SetMode(Mode::Hold);
    in.tableValid = false;
    s3.Update(in);
    in.psmDetail = 18; in.vz = 4.8f; s3.Update(in);
    in.onGround = false; in.psmDetail = 14; in.vz = 2.0f; CHECK(s3.Update(in).contactScale == 0.0f, "hold air contacts 0");
    in.onGround = true; in.psmDetail = 23; in.vz = 0.0f;
    AirDecision d = s3.Update(in);
    CHECK(d.action == Action::HandBack && d.contactScale > 0.0f && d.contactScale < 1.0f, "touchdown ramp start %.3f",
          d.contactScale);
    for (int i = 0; i < 5; ++i) d = s3.Update(in);
    CHECK(d.contactScale == 1.0f, "ramp complete");
}

static void TestRecoverRejumpAndHeavyLate()
{
    {
        AirState s = MakeClips();
        JumpOpts o;
        o.rejumpAfterTouch = 4;
        Trace tr = RunJump(s, o);
        CHECK(s.Stats().takeoffs == 2 && s.CurrentPhase() == Phase::TakeOff, "re-jump from Recover (%u, %s)",
              s.Stats().takeoffs, PhaseName(s.CurrentPhase()));
        CHECK(tr.plays[static_cast<int>(Action::HandBack)] == 0, "no handback before re-jump");
    }
    {
        AirState s = MakeClips();
        JumpOpts o;
        o.vz0 = 3.0f;
        o.fallPrevAtTouch = 3;
        RunJump(s, o);
        CHECK(s.Stats().lands == 1, "lands");
        // heavy_ is cleared on hand-back episode end only by BeginEpisode; check via recover length below.
    }
    // Heavy from PSM.Fall seen one frame late (first Recover feed) extends Recover to 0.5 s.
    for (int late = 0; late < 3; ++late)
    {
        AirState s = MakeClips();
        JumpOpts o;
        o.vz0 = 3.0f;
        o.stick = 0.0f;
        if (late == 1) o.fallPrevAtTouch = 3;
        if (late == 2) o.fallPrevAfterTouch = 3;
        Trace tr = RunJump(s, o);
        CHECK(s.Heavy() == (late != 0), "heavy late=%d -> %d", late, s.Heavy() ? 1 : 0);
        (void)tr;
    }
    // Recover durations: stick at walk -> 0.2 s, idle light -> 0.3 s.
    for (float stick : {0.0f, 1.0f})
    {
        AirState s = MakeClips();
        AirInput in;
        in.dt = 1.0f / 100.0f;
        in.tableValid = true;
        in.stickMagnitude = stick;
        s.Update(in);
        in.psmDetail = 18; in.vz = 3.0f; s.Update(in);
        in.onGround = false; in.psmDetail = 14; in.vz = 1.0f; s.Update(in);
        in.onGround = true; in.psmDetail = 23; in.vz = 0.0f;
        s.Update(in); // touchdown -> Recover
        int n = 0;
        while (s.CurrentPhase() == Phase::Recover && n < 200) { s.Update(in); ++n; }
        const float expect = stick > 0.5f ? 0.20f : 0.30f;
        CHECK(std::fabs(n * 0.01f - expect) <= 0.0151f, "recover stick %.0f lasted %.2f", stick, n * 0.01f);
    }
}

static void TestProbeDisagreement()
{
    AirState s = MakeClips();
    JumpOpts o;
    o.bogusProbeAt = 14; // ~0.23 s after take-off at 60 Hz: near apex, h ~0.6 m; probe says 0.05 m
    Trace tr = RunJump(s, o);
    CHECK(s.Stats().probesRejected == 1, "bogus probe rejected (%u)", s.Stats().probesRejected);
    const float predicted = tr.landEntryT + tr.landEntryTtl;
    CHECK(std::fabs(predicted - tr.touchT) <= 0.05f, "ttl unaffected by bogus probe (%.3f)", predicted - tr.touchT);
    CHECK(tr.Seq() == "Ground TakeOff Descend LandApproach Recover Ground ", "seq %s", tr.Seq().c_str());
}

static void TestJumpDownNoProbe()
{
    // Jump down to a floor 1.5 m lower, no probe: dead-reckoned h must be dropped once zRel < -0.3 m
    // (TTL unknown = noHitTtl), and the episode still lands at the real touchdown.
    for (float hz : {30.0f, 60.0f, 144.0f})
    {
        AirState s = MakeClips();
        const float dt = 1.0f / hz, vz0 = 4.8f, gU = 19.2f, gD = 16.0f, floorZ = -1.5f;
        for (int i = 0; i < 20; ++i) { AirInput in; in.dt = dt; in.tableValid = true; in.planarSpeed = 4.0f; s.Update(in); }
        const float tA = vz0 / gU, hA = vz0 * vz0 / (2.0f * gU);
        bool touched = false, sawDeep = false, deepUnknown = true, handback = false;
        float t = 0.0f, touchT = -1.0f;
        for (int k = 0; t < 3.0f; ++k)
        {
            if (k > 0) t += dt;
            float z, vz;
            if (t < tA) { z = vz0 * t - 0.5f * gU * t * t; vz = vz0 - gU * t; }
            else { const float u = t - tA; z = hA - 0.5f * gD * u * u; vz = -gD * u; }
            if (k > 0 && z <= floorZ && !touched) { touched = true; touchT = t; }
            AirInput in;
            in.dt = dt;
            in.tableValid = true;
            in.planarSpeed = 4.0f;
            in.vz = touched ? 0.0f : vz;
            in.onGround = k == 0 || touched;
            in.psmDetail = touched ? 23 : (vz > 0.0f || k == 0 ? 18 : 14);
            const AirDecision d = s.Update(in);
            if (!touched && k > 0 && z < -0.5f)
            {
                sawDeep = true;
                if (d.ttlKnown || std::fabs(d.ttl - 1.5f) > 1e-5f) deepUnknown = false;
            }
            if (d.action == Action::HandBack) handback = true;
            if (touched && t - touchT > 1.0f) break;
        }
        CHECK(sawDeep && deepUnknown, "hz %.0f below take-off plane -0.3 m: TTL unknown", hz);
        CHECK(handback && s.Stats().lands == 1 && s.CurrentPhase() == Phase::Ground, "hz %.0f lower floor lands", hz);
    }
}

// Hold repair (2026-10-13): an external release reason blocks a grounded jump edge (no one-feed take-off + Reset), and a
// grounded jump edge without release (knockdown / car hit reported as detail 18 with onGround) aborts after abortTime.
static void TestGroundedEdges()
{
    for (Mode m : {Mode::Hold, Mode::Clips})
        for (float hz : {30.0f, 60.0f, 144.0f})
        {
            const float dt = 1.0f / hz;
            // release + grounded jump edge (FPP jump, gate closed without hold): nothing at all
            {
                AirState s;
                s.SetMode(m);
                int acts = 0, owned = 0;
                for (int i = 0; i < int(1.0f * hz); ++i)
                {
                    AirInput in;
                    in.dt = dt;
                    in.tableValid = true;
                    in.release = true;
                    const bool edge = i >= int(0.2f * hz);
                    in.psmDetail = edge ? 18 : -1;
                    in.onGround = i < int(0.3f * hz);
                    in.vz = edge ? 4.0f : 0.0f;
                    const AirDecision d = s.Update(in);
                    acts += d.action != Action::None ? 1 : 0;
                    owned += d.airOwns ? 1 : 0;
                }
                CHECK(acts == 0 && owned == 0 && s.Stats().takeoffs == 0 && s.Stats().resets == 0 && s.Stats().episodes == 0,
                      "mode %d hz %.0f release + grounded jump edge: acts %d owned %d to %u resets %u", int(m), hz, acts, owned,
                      s.Stats().takeoffs, s.Stats().resets);
            }
            // no release: grounded detail 18 for 3 s (knockdown) -> one take-off, hand-back after abortTime, no second episode
            {
                AirState s;
                s.SetMode(m);
                int owned = 0, hb = 0, hbAt = -1;
                const int edgeAt = int(0.2f * hz);
                for (int i = 0; i < int(3.5f * hz); ++i)
                {
                    AirInput in;
                    in.dt = dt;
                    in.tableValid = true;
                    in.psmDetail = i >= edgeAt ? 18 : -1;
                    in.onGround = true;
                    const AirDecision d = s.Update(in);
                    owned += d.airOwns ? 1 : 0;
                    if (d.action == Action::HandBack && hbAt < 0) hbAt = i;
                    hb += d.action == Action::HandBack ? 1 : 0;
                }
                const float ownedS = owned * dt;
                if (m == Mode::Clips) // AIR CLIPS repair: a grounded, non-rising edge only arms the take-off (no Start clip)
                {
                    CHECK(s.Stats().takeoffs == 0 && s.Stats().episodes == 0 && hb == 0 && owned == 0 && s.Stats().resets == 0,
                          "clips hz %.0f grounded knockdown edge: armed only (to %u ep %u hb %d owned %d)", hz, s.Stats().takeoffs,
                          s.Stats().episodes, hb, owned);
                }
                else
                {
                    CHECK(s.Stats().takeoffs == 1 && s.Stats().episodes == 1 && hb == 1 && s.Stats().resets == 0,
                          "mode %d hz %.0f grounded knockdown edge: to %u ep %u hb %d resets %u", int(m), hz, s.Stats().takeoffs,
                          s.Stats().episodes, hb, s.Stats().resets);
                    CHECK(ownedS > 0.0f && ownedS <= s.Config().abortTime + 1.5f * dt && hbAt > edgeAt,
                          "mode %d hz %.0f grounded knockdown edge: owned %.3f s (abort %.2f)", int(m), hz, ownedS, s.Config().abortTime);
                }
            }
            // AIR CLIPS repair: armed edge (grounded, vz 0) then the capsule leaves the ground after `lag` s: a take-off with a
            // Start clip on the leaving feed when lag <= abortTime, nothing when later (the window expired: ledge rules only)
            if (m == Mode::Clips)
                for (float lag : {1.0f / hz, 0.1f, 0.4f})
                {
                    AirState s;
                    s.SetMode(m);
                    const int edgeAt = int(0.2f * hz), leaveAt = edgeAt + std::max(1, int(std::lround(lag * hz)));
                    int takeoffAt = -1;
                    for (int i = 0; i < leaveAt + 2; ++i)
                    {
                        AirInput in;
                        in.dt = dt;
                        in.tableValid = true;
                        in.psmDetail = i >= edgeAt ? 18 : -1;
                        in.onGround = i < leaveAt;
                        in.vz = i >= leaveAt ? 1.0f : 0.0f; // below takeoffVz: only the armed edge can take off
                        const AirDecision d = s.Update(in);
                        if (d.action == Action::PlayTakeOff && takeoffAt < 0) takeoffAt = i;
                    }
                    const bool expect = lag <= s.Config().abortTime;
                    CHECK(expect ? (takeoffAt == leaveAt && s.Stats().takeoffs == 1) : (takeoffAt < 0 && s.Stats().takeoffs == 0),
                          "clips hz %.0f armed edge, leaves after %.3f s: take-off at %d (leave %d) to %u", hz, lag, takeoffAt, leaveAt,
                          s.Stats().takeoffs);
                }
        }
}

static void TestModeOff()
{
    AirState s; // Off by default
    JumpOpts o;
    Trace tr = RunJump(s, o);
    int acts = 0;
    for (int a = 1; a < 7; ++a) acts += tr.plays[a];
    CHECK(acts == 0 && tr.Seq() == "Ground ", "off: no decisions (%s)", tr.Seq().c_str());
}

// AIR CLIPS stage: stick 90 deg off the body at the touchdown -> hand back at once (no Recover); 30 deg -> Recover as before
static void TestTurnOnLanding()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
    {
        AirState a = MakeClips(), b = MakeClips();
        JumpOpts o;
        o.hz = hz;
        o.stickTurnDeg = 90.0f;
        Trace ta = RunJump(a, o);
        o.stickTurnDeg = 30.0f;
        Trace tb = RunJump(b, o);
        const bool aNoRecover = ta.Seq().find("Recover") == std::string::npos;
        const bool bRecover = tb.Seq().find("Recover") != std::string::npos;
        CHECK(aNoRecover && a.Stats().handbacks == 1 && a.Stats().lands == 1 && a.Stats().resets == 0 && bRecover && b.Stats().handbacks == 1,
              "turn on landing %.0f Hz: 90 deg -> no Recover (%s), 30 deg -> Recover (%s)", hz, ta.Seq().c_str(), tb.Seq().c_str());
    }
}


// ---- BATCH 6 AIR stage (2026-10-14): physics fall + touchdown guard ----
static AirState MakeRobust(bool on)
{
    AirConfig c;
    c.heavyVz = 8.7f;
    c.physFall = on;
    c.touchGuard = on;
    AirState s(c);
    s.SetMode(Mode::Clips);
    return s;
}

// Female v1 6:50.6: run off the car roof (1.0-1.3 m, PSM Fall detail 14): the 0.6.5 ledge rule needs ttl >= 0.35 s and never
// starts; with physFall the drop goes straight to LandApproach and plays one land clip before the touchdown.
static void TestPhysFallShortDrop()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
        for (float hgt : {0.9f, 1.3f})
        {
            DropOpts o;
            o.hz = hz;
            o.height = hgt;
            o.speed = 4.0f;
            AirState off = MakeRobust(false);
            const Trace a = RunDrop(off, o);
            CHECK(a.plays[static_cast<int>(Action::PlayLand)] == 0, "hz %.0f h %.1f off: no land (previous behaviour) %s", hz, hgt,
                  a.Seq().c_str());
            AirState on = MakeRobust(true);
            const Trace b = RunDrop(on, o);
            CHECK(b.plays[static_cast<int>(Action::PlayLand)] == 1 && b.landEntryT >= 0.0f && b.landEntryT < b.touchT,
                  "hz %.0f h %.1f on: land before touch (%d, %.3f / %.3f) %s", hz, hgt, b.plays[static_cast<int>(Action::PlayLand)],
                  b.landEntryT, b.touchT, b.Seq().c_str());
            CHECK(b.Seq().find("Recover") != std::string::npos && b.phases.back() == Phase::Ground, "hz %.0f recover %s", hz,
                  b.Seq().c_str());
            CHECK(on.Stats().physFalls == 1, "hz %.0f physFalls %u", hz, on.Stats().physFalls);
        }
}

// Female v1 8:30: vehicle exit launch (released while mounted, vz +9.8 -> -12, no PSM jump): FallLoop, heavy land at touchdown.
static void TestPhysFallVehicleLaunch()
{
    for (bool on : {false, true})
    {
        AirState s = MakeRobust(on);
        Trace tr;
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 20; ++i)
        {
            AirInput in;
            in.dt = dt;
            in.release = true; // mounted
            in.tableValid = true;
            tr.Add(s.Update(in), -1.0f);
        }
        float t = 0.0f, z = 0.0f, vz = 9.8f;
        bool touched = false;
        for (int k = 0; k < 400 && !(touched && t - tr.touchT > 1.0f); ++k)
        {
            t += dt;
            if (!touched)
            {
                vz -= 16.0f * dt;
                z += vz * dt;
                if (z <= 0.0f && t > 0.2f) { touched = true; tr.touchT = t; }
            }
            AirInput in;
            in.dt = dt;
            in.vz = touched ? 0.0f : vz;
            in.onGround = touched;
            in.release = t < 0.05f; // unmount finishes right after the launch
            in.planarSpeed = 2.0f;
            in.velBodyY = 2.0f;
            in.tableValid = true;
            tr.Add(s.Update(in), t);
        }
        const bool landed = tr.Seq().find("Recover") != std::string::npos;
        if (on)
        {
            CHECK(landed && tr.Seq().find("FallLoop") != std::string::npos && tr.sawHeavyLand, "launch on: %s heavy %d",
                  tr.Seq().c_str(), tr.sawHeavyLand ? 1 : 0);
        }
        else
            CHECK(!landed, "launch off: no episode (previous behaviour) %s", tr.Seq().c_str());
    }
}

// fix/jumpout-cam (game log 2026-11-03 17:34:36, car jump-out B): the capsule reported onGround for two feeds at 0.23 m above the floor while it
// still fell at 11 m/s (it grazed the exiting car), the machine went to Recover and the next feed - airborne again at vz -10.8 - ended the episode
// ("walked off right after landing"); a second episode and a second land clip followed. The touchdown guard must take the false touch back:
// the episode stays one descent and ends with the real landing. Mutation 19 restores the hand-back.
static void TestFalseTouchInFastFall()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
    {
        AirState s = MakeRobust(true);
        const float dt = 1.0f / hz;
        for (int i = 0; i < 20; ++i)
        {
            AirInput in;
            in.dt = dt;
            in.release = true; // mounted
            in.tableValid = true;
            s.Update(in);
        }
        float t = 0.0f, z = 0.0f, vz = 9.8f;
        int flick = 0;       // contact feeds of the false touch still to send
        bool flicked = false, touched = false;
        int realTouch = 0;
        Phase afterFlick = Phase::Ground;
        for (int k = 0; k < 1200 && realTouch < int(hz * 1.2f); ++k)
        {
            t += dt;
            if (!touched && flick == 0)
            {
                vz -= 16.0f * dt;
                z += vz * dt;
            }
            if (!touched && !flicked && t > 0.5f && vz < -8.0f && z <= 0.9f)
            {
                flick = 2; // two contact feeds at vz -11
                flicked = true;
            }
            if (!touched && flicked && flick == 0 && z <= 0.0f && t > 0.5f)
                touched = true;
            AirInput in;
            in.dt = dt;
            in.release = t < 0.05f;
            in.vehicleExit = true; // a jump-out flight: the take-back belongs to the vehicle exit (ordinary jumps: TestFalseTouchGate)
            in.planarSpeed = 2.0f;
            in.velBodyY = 2.0f;
            in.tableValid = true;
            in.vz = vz;
            in.onGround = false;
            if (flick > 0)
            {
                in.onGround = true;
                --flick;
            }
            else if (touched)
            {
                in.vz = 0.0f;
                in.onGround = true;
                ++realTouch;
            }
            const AirDecision d = s.Update(in);
            if (flicked && !in.onGround && !touched && afterFlick == Phase::Ground)
                afterFlick = d.phase; // the first feed after the contact feeds: airborne again
        }
        CHECK(flicked && s.Stats().falseTouches == 1 && afterFlick == Phase::FallLoop,
              "false touch %.0f Hz: contact feeds in a fast fall are taken back (falseTouches %u, phase after %s)", hz, s.Stats().falseTouches, PhaseName(afterFlick));
        CHECK(s.Stats().lands == 1 && s.Stats().handbacks == 1 && s.Stats().falls == 1,
              "false touch %.0f Hz: one episode, one landing, one hand-back (falls %u lands %u handbacks %u)", hz, s.Stats().falls, s.Stats().lands,
              s.Stats().handbacks);
    }
}

// Review r2 finding 14: the false-touch take-back acts only in an episode of a vehicle exit (AirInput::vehicleExit on a feed of the episode, latched to its
// end). An ordinary jump / fall - e.g. a landing on a downhill ramp or stairs, the capsule following the slope at vz -3.5 m/s, one onGround flicker in the
// 0.25 s after the touchdown - keeps the previous hand-back (no fall-loop flash, no second land clip). exitMode: 0 no exit window, 1 the window open through
// the whole flight and landing, 2 open for the first 0.3 s of the flight only (the gate closed it before the landing: the episode keeps the latch).
struct FlickerRun
{
    Phase afterFlick = Phase::Ground;
    uint32_t falseTouches = 0, lands = 0, handbacks = 0, falls = 0;
    std::vector<int> trace; // phase / action per feed, to compare two configurations feed by feed
};
static FlickerRun RunTouchFlicker(AirConfig cfg, float hz, float vzTouch, int exitMode)
{
    AirState s(cfg);
    s.SetMode(Mode::Clips);
    const float dt = 1.0f / hz;
    FlickerRun r;
    auto feed = [&](bool onGround, float vz, bool exitOpen) {
        AirInput in;
        in.dt = dt;
        in.onGround = onGround;
        in.vz = vz;
        in.vehicleExit = exitOpen;
        in.planarSpeed = 2.0f;
        in.velBodyY = 2.0f;
        in.tableValid = true;
        const AirDecision d = s.Update(in);
        r.trace.push_back(static_cast<int>(d.phase) * 16 + static_cast<int>(d.action));
        return d;
    };
    for (int i = 0; i < int(hz * 0.3f); ++i)
        feed(true, 0.0f, false);
    // the flight: ballistic from 6 m/s up, down to vzTouch (the slope landing keeps that speed, the free fall of a jump-out reaches it)
    float vz = 6.0f, t = 0.0f;
    while (vz > vzTouch)
    {
        vz = std::fmax(vzTouch, vz - 16.0f * dt);
        t += dt;
        feed(false, vz, exitMode == 1 || (exitMode == 2 && t < 0.3f));
    }
    const bool open = exitMode == 1;
    for (int i = 0; i < 2; ++i)
        feed(true, vzTouch, open); // the touchdown (two contact feeds)
    r.afterFlick = feed(false, vzTouch, open).phase; // the flicker: contact gone again, the capsule still falls at the slope speed
    for (int i = 0; i < int(hz * 0.8f); ++i)
        feed(true, i < 3 ? vzTouch : 0.0f, open);
    r.falseTouches = s.Stats().falseTouches;
    r.lands = s.Stats().lands;
    r.handbacks = s.Stats().handbacks;
    r.falls = s.Stats().falls;
    return r;
}

static void TestFalseTouchGate()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
        for (float vzTouch : {-3.5f, -7.0f, -11.0f})
        {
            AirConfig prod;
            prod.heavyVz = 8.7f;
            prod.physFall = true;
            prod.touchGuard = true;
            AirConfig before = prod;
            before.falseTouchWindow = 0.0f; // the hand-back of the code before the take-back existed
            const FlickerRun on = RunTouchFlicker(prod, hz, vzTouch, 0);
            const FlickerRun ref = RunTouchFlicker(before, hz, vzTouch, 0);
            CHECK(on.falseTouches == 0 && on.afterFlick != Phase::FallLoop,
                  "false touch gate %.0f Hz vz %.1f: an ordinary landing keeps the hand-back (falseTouches %u, phase after %s)", hz, vzTouch, on.falseTouches,
                  PhaseName(on.afterFlick));
            CHECK(on.trace == ref.trace && on.lands == ref.lands && on.handbacks == ref.handbacks && on.falls == ref.falls,
                  "false touch gate %.0f Hz vz %.1f: an ordinary landing is feed-by-feed identical with the take-back off (lands %u / %u)", hz, vzTouch,
                  on.lands, ref.lands);
            for (int mode : {1, 2})
            {
                const FlickerRun ex = RunTouchFlicker(prod, hz, vzTouch, mode);
                CHECK(ex.falseTouches == 1 && ex.afterFlick == Phase::FallLoop,
                      "false touch gate %.0f Hz vz %.1f exit mode %d: the vehicle exit episode takes the touch back (falseTouches %u, phase after %s)", hz,
                      vzTouch, mode, ex.falseTouches, PhaseName(ex.afterFlick));
            }
        }
}

// Vehicle batch W1 CAR-2 (recording 2026-10-22: jump-outs at 5.3-15.6 m/s): inside the vehicle exit window (AirInput::vehicleExit)
// the planar speed the air episode sees is clamped to vehicleExitMaxSpeed (7 m/s, motion sprint cap): a 15 m/s jump-out lands with
// the land clip at 7 / 6.5 = x1.08 instead of the x1.3 clamp. Without the flag (on foot) nothing changes. Mutation 5 removes it.
static void TestVehicleExitSpeedClamp()
{
    float recoverRate[2] = {-1.0f, -1.0f};
    Band band[2] = {Band::Stand, Band::Stand};
    for (int v = 0; v < 2; ++v)
    {
        AirState s = MakeRobust(true);
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 20; ++i)
        {
            AirInput in;
            in.dt = dt;
            in.tableValid = true;
            s.Update(in);
        }
        float t = 0.0f, z = 0.0f, vz = 4.0f, touchT = -1.0f;
        for (int k = 0; k < 400 && !(touchT >= 0.0f && t - touchT > 1.0f); ++k)
        {
            t += dt;
            if (touchT < 0.0f)
            {
                vz -= 16.0f * dt;
                z += vz * dt;
                if (z <= 0.0f && t > 0.2f)
                    touchT = t;
            }
            AirInput in;
            in.dt = dt;
            in.vz = touchT >= 0.0f ? 0.0f : vz;
            in.onGround = touchT >= 0.0f;
            in.planarSpeed = 15.0f;
            in.velBodyY = 15.0f;
            in.clipRootSpeed = 6.5f;
            in.tableValid = true;
            in.vehicleExit = v == 1;
            const AirDecision d = s.Update(in);
            if (d.phase == Phase::Recover && !d.rateIsMultiplier)
                recoverRate[v] = d.rate;
            if (d.action == Action::PlayLand || d.action == Action::PlayOff || d.action == Action::PlayFallLoop)
                band[v] = d.pick.band;
        }
    }
    CHECK(std::fabs(recoverRate[0] - 1.3f) < 0.02f, "vehicle exit clamp: on foot 15 m/s recover rate %.3f (x1.3 clamp, unchanged)", recoverRate[0]);
    CHECK(std::fabs(recoverRate[1] - 7.0f / 6.5f) < 0.02f, "vehicle exit clamp: jump-out 15 m/s recover rate %.3f (7 / 6.5 expected)",
          recoverRate[1]);
    CHECK(band[0] == Band::Sprint && band[1] == Band::Sprint, "vehicle exit clamp: pick band sprint both (%d / %d)", static_cast<int>(band[0]),
          static_cast<int>(band[1]));
}

// Female v1 car hood: a one-frame onGround while rising (vz +3) must not land; the real touchdown later does.
static void TestTouchGuardRisingBlip()
{
    for (bool on : {false, true})
    {
        AirState s = MakeRobust(on);
        Trace tr;
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 20; ++i)
        {
            AirInput in;
            in.dt = dt;
            in.tableValid = true;
            tr.Add(s.Update(in), -1.0f);
        }
        float t = 0.0f, z = 0.0f, vz = 5.0f;
        int lands = 0;
        bool touched = false;
        for (int k = 0; k < 200 && !(touched && t - tr.touchT > 0.6f); ++k)
        {
            t += dt;
            if (!touched)
            {
                vz -= 16.0f * dt;
                z += vz * dt;
                if (z <= 0.0f && t > 0.2f) { touched = true; tr.touchT = t; }
            }
            AirInput in;
            in.dt = dt;
            in.vz = touched ? 0.0f : vz;
            in.onGround = touched || k == 6; // blip at t 0.117 s, vz ~ +3.1
            in.psmDetail = k == 0 ? 17 : (touched ? 0 : 14);
            in.planarSpeed = 3.0f;
            in.velBodyY = 3.0f;
            in.tableValid = true;
            const AirDecision d = s.Update(in);
            if (d.phase == Phase::Recover && (tr.phases.empty() || tr.phases.back() != Phase::Recover)) ++lands;
            if (k == 7 && on) CHECK(d.phase != Phase::Recover && d.phase != Phase::Ground, "blip: still airborne %s", PhaseName(d.phase));
            tr.Add(d, t);
        }
        if (on) CHECK(s.Stats().touchRejects >= 1 && s.Stats().lands == 1, "guard: rejects %u lands %u %s", s.Stats().touchRejects,
                      s.Stats().lands, tr.Seq().c_str());
        else CHECK(s.Stats().lands >= 1 && tr.phases.size() > 1 && tr.phases[1] != Phase::Ground, "off ran %s", tr.Seq().c_str());
    }
}

// Physically airborne contacts can disagree with lagging PSM or flicker for one frame on descent.
// The robust machine requires a real contact before giving control back and always preserves foot contact ramp.
static void TestTouchGuardContactEvidence()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
    {
        const float dt = 1.0f / hz;
        for (int scenario = 0; scenario < 4; ++scenario)
        {
            AirState s = MakeRobust(true);
            for (int i = 0; i < 12; ++i)
            {
                AirInput in; in.dt = dt; in.tableValid = true;
                s.Update(in);
            }
            AirInput in;
            in.dt = dt; in.tableValid = true; in.psmDetail = 18; in.vz = 5.0f;
            in.onGround = false;
            s.Update(in);
            in.psmDetail = 14;
            for (int i = 0; i < int(hz * 0.4f); ++i)
            {
                in.vz = i < int(hz * 0.20f) ? 3.0f : -5.0f;
                if (scenario == 0 && i == 3) { in.psmDetail = 23; in.onGround = false; }
                else if (scenario == 1 && i == int(hz * 0.3f)) { in.psmDetail = 14; in.onGround = true; }
                else if (scenario == 2 && i == 4) { in.psmDetail = 23; in.onGround = true; in.vz = 3.0f; }
                else if (scenario == 3 && i == int(hz * 0.3f)) { in.psmDetail = 14; in.onGround = true; in.vz = 0.0f; }
                else { in.psmDetail = 14; in.onGround = false; }
                const AirDecision d = s.Update(in);
                CHECK(d.airOwns && d.contactScale == 0.0f && s.Stats().lands == 0,
                      "%.0f Hz scenario %d feed %d false contact: phase %s lands %u", hz, scenario, i,
                      PhaseName(d.phase), s.Stats().lands);
            }
            // Genuine contact: after two consecutive feeds, recovery starts and contacts ramp monotonically.
            in.onGround = true; in.psmDetail = 23; in.vz = 0.0f;
            const AirDecision first = s.Update(in);
            const AirDecision second = s.Update(in);
            CHECK(s.Stats().lands == 1 && second.phase == Phase::Recover,
                  "%.0f Hz scenario %d confirmed touch lands %u phase %s", hz, scenario,
                  s.Stats().lands, PhaseName(second.phase));
            CHECK(first.contactScale <= second.contactScale && second.contactScale >= 0.0f && second.contactScale <= 1.0f,
                  "%.0f Hz ramp first %.3f second %.3f", hz, first.contactScale, second.contactScale);
            float previous = second.contactScale;
            for (int i = 0; i < int(hz); ++i)
            {
                const AirDecision d = s.Update(in);
                CHECK(d.contactScale + 1e-6f >= previous && d.contactScale <= 1.0f,
                      "%.0f Hz ramp monotonic: %.3f after %.3f", hz, d.contactScale, previous);
                previous = d.contactScale;
            }
            CHECK(previous == 1.0f && s.CurrentPhase() == Phase::Ground,
                  "%.0f Hz ramp ends grounded %.3f", hz, previous);
        }
    }
}

// A late floor prediction must expire once, then require a fresh probe. Repeated old predictions
// must not generate PlayLand spam; Reset and OFF restore the pre-episode contract.
static void TestPredictedTouchExpiryAndReset()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
    {
        AirState s = MakeRobust(true);
        const float dt = 1.0f / hz;
        AirInput in; in.dt = dt; in.tableValid = true;
        for (int i = 0; i < 12; ++i) s.Update(in);
        in.onGround = false; in.psmDetail = 14; in.vz = -5.0f;
        in.distToGround = 0.1f; in.planarSpeed = 3.0f;
        int landPlays = 0, loops = 0;
        for (int i = 0; i < int(hz * 0.8f); ++i)
        {
            // First few feeds carry the original floor, followed by no-hit trace.
            in.distToGround = i < 3 ? 0.1f : -2.0f;
            const AirDecision d = s.Update(in);
            landPlays += d.action == Action::PlayLand ? 1 : 0;
            loops += d.action == Action::PlayFallLoop ? 1 : 0;
        }
        CHECK(s.Stats().landAborts == 1 && loops >= 1 && landPlays == 1,
              "%.0f Hz expiry: abort %u loops %d land %d", hz, s.Stats().landAborts, loops, landPlays);
        CHECK(s.CurrentPhase() == Phase::FallLoop && s.HeightEstimate() < 0.0f,
              "%.0f Hz expired floor invalidated", hz);
        s.RequestReset();
        in.onGround = true; in.psmDetail = 0; in.vz = 0.0f; in.distToGround = -2.0f;
        const AirDecision reset = s.Update(in);
        CHECK(reset.action == Action::Reset && !reset.airOwns && reset.contactScale == 1.0f,
              "%.0f Hz reset action %d owns %d contact %.2f", hz, int(reset.action), int(reset.airOwns), reset.contactScale);
        s.SetMode(Mode::Off);
        in.onGround = false; in.vz = -9.0f;
        for (int i = 0; i < int(hz * 0.2f); ++i)
        {
            const AirDecision d = s.Update(in);
            CHECK(d.action == Action::None && !d.airOwns && d.contactScale == 1.0f,
                  "%.0f Hz OFF remains inert", hz);
        }
    }
}

// Stale PSM land state can lag behind actual movement for several ticks. Negative vertical velocity
// without a live contact is not enough evidence to complete a robust landing even after two frames.
static void TestStaleLandDetailWhileFalling()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
    {
        for (bool robust : {false, true})
        {
            AirState s = MakeRobust(robust);
            AirInput in;
            in.dt = 1.0f / hz;
            in.tableValid = true;
            for (int i = 0; i < 12; ++i) s.Update(in);
            in.psmDetail = 18; in.onGround = false; in.vz = 5.0f;
            s.Update(in);
            in.psmDetail = 14; in.vz = -5.0f;
            for (int i = 0; i < 3; ++i) s.Update(in);
            CHECK(s.CurrentPhase() != Phase::Ground, "%.0f Hz mode %d airborne before stale report", hz, robust);
            in.psmDetail = 23;
            in.groundStale = true;
            in.onGround = true; // ignored when stale
            in.vz = -4.0f;
            in.distToGround = -2.0f;
            for (int i = 0; i < 5; ++i)
            {
                const AirDecision d = s.Update(in);
                if (robust)
                    CHECK(d.airOwns && d.contactScale == 0.0f && s.Stats().lands == 0,
                          "%.0f Hz stale land feed %d: phase %s lands %u", hz, i,
                          PhaseName(d.phase), s.Stats().lands);
            }
            if (!robust)
                CHECK(s.Stats().lands == 1, "%.0f Hz legacy stale detail unchanged", hz);
            else
            {
                in.groundStale = false; in.onGround = true; in.vz = 0.0f;
                s.Update(in);
                const AirDecision d = s.Update(in);
                CHECK(s.Stats().lands == 1 && !d.releaseOffsetRoot,
                      "%.0f Hz fresh confirmed touchdown %u", hz, s.Stats().lands);
            }
        }
    }
}

static void TestJumpEdgeRequiresImpulse()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
    {
        const float dt = 1.0f / hz;
        // A jump detail during negative velocity is stale, even if it crosses a PSM edge.
        AirState s = MakeRobust(true);
        AirInput in; in.dt = dt; in.tableValid = true;
        for (int i = 0; i < 12; ++i) s.Update(in);
        in.onGround = false; in.psmDetail = 18; in.vz = 5.0f;
        s.Update(in);
        in.psmDetail = 14; in.vz = -3.0f;
        for (int i = 0; i < 3; ++i) s.Update(in);
        const uint32_t takeoffs = s.Stats().takeoffs;
        in.psmDetail = 18; in.groundStale = true; in.vz = -4.0f;
        const AirDecision stale = s.Update(in);
        CHECK(s.Stats().takeoffs == takeoffs && stale.action != Action::PlayTakeOff && stale.phase != Phase::TakeOff,
              "%.0f Hz descending PSM 18 must not double-jump", hz);
        in.psmDetail = 14; in.groundStale = false; in.vz = -4.5f;
        s.Update(in);
        in.psmDetail = 18; in.vz = 5.0f;
        const AirDecision real = s.Update(in);
        CHECK(s.Stats().takeoffs == takeoffs + 1 && real.action == Action::PlayTakeOff && real.phase == Phase::TakeOff,
              "%.0f Hz observed upward impulse permits double jump", hz);

        // A stale edge during Recover must respect retakeBlock; observed launch may bypass it.
        AirState r = MakeRobust(true);
        in = {}; in.dt = dt; in.tableValid = true;
        for (int i = 0; i < 12; ++i) r.Update(in);
        in.onGround = false; in.psmDetail = 18; in.vz = 5.0f; r.Update(in);
        in.psmDetail = 14; in.vz = -5.0f; r.Update(in);
        in.onGround = true; in.psmDetail = 23; in.vz = 0.0f;
        r.Update(in); r.Update(in);
        CHECK(r.CurrentPhase() == Phase::Recover, "%.0f Hz entered Recover", hz);
        in.psmDetail = 18; in.vz = 0.0f;
        const AirDecision phantom = r.Update(in);
        CHECK(phantom.phase == Phase::Recover && r.Stats().takeoffs == 1,
              "%.0f Hz grounded stale edge blocked in Recover", hz);
        in.onGround = false; in.psmDetail = 14; in.vz = 0.0f;
        // Keep Recover active for the next feed; the transition is a physical rising impulse.
        in.psmDetail = 18; in.vz = 5.0f;
        const AirDecision relaunch = r.Update(in);
        CHECK(relaunch.phase == Phase::TakeOff && r.Stats().takeoffs == 2,
              "%.0f Hz actual upward launch bypasses Recover lockout", hz);
    }
}

// Touchdown is sampled at time zero for robust mode at every FPS. After that the ramp
// follows elapsed real seconds; a HandBack midway does not restart or skip it.
static void TestContactRampZeroAtTouch()
{
    for (float hz : {30.0f, 60.0f, 144.0f})
    {
        for (bool robust : {false, true})
        {
            AirState s = MakeRobust(robust);
            AirInput in; in.dt = 1.0f / hz; in.tableValid = true;
            for (int i = 0; i < 12; ++i) s.Update(in);
            in.onGround = false; in.psmDetail = 18; in.vz = 5.0f;
            s.Update(in);
            in.psmDetail = 14; in.vz = -3.0f; s.Update(in);
            in.onGround = true; in.psmDetail = 23; in.vz = 0.0f;
            const AirDecision contact0 = s.Update(in);
            const AirDecision touch = robust ? s.Update(in) : contact0;
            CHECK(s.Stats().lands == 1, "%.0f Hz robust %d touched", hz, robust);
            const float expectedFirst = robust ? 0.0f : std::fmin(1.0f, in.dt / s.Config().contactRamp);
            CHECK(std::fabs(touch.contactScale - expectedFirst) < 1e-5f,
                  "%.0f Hz robust %d first contact %.5f expected %.5f", hz, robust, touch.contactScale, expectedFirst);
            CHECK(touch.phase == Phase::Recover && touch.airOwns && !touch.releaseOffsetRoot,
                  "%.0f Hz robust %d Recover ownership", hz, robust);
            const AirDecision next = s.Update(in);
            const float expectedNext = std::fmin(1.0f, (robust ? 1.0f : 2.0f) * in.dt / s.Config().contactRamp);
            CHECK(std::fabs(next.contactScale - expectedNext) < 1e-5f,
                  "%.0f Hz robust %d next contact %.5f expected %.5f", hz, robust, next.contactScale, expectedNext);
            s.RequestReset();
            const AirDecision reset = s.Update(in);
            CHECK(reset.action == Action::Reset && !reset.airOwns && reset.contactScale == 1.0f,
                  "%.0f Hz robust %d reset contact %.5f", hz, robust, reset.contactScale);
        }
    }
}

int main()
{
    TestTtlSign();
    TestFlatJump();
    TestStandingJumpAndDirection();
    TestDrops();
    TestNoTrigger();
    TestReleaseAndClimbMidAir();
    TestStaleAndNaN();
    TestHoldMode();
    TestRecoverRejumpAndHeavyLate();
    TestProbeDisagreement();
    TestJumpDownNoProbe();
    TestModeOff();
    TestGroundedEdges();
    TestTurnOnLanding(); // AIR CLIPS stage
    TestPhysFallShortDrop(); // BATCH 6 AIR stage
    TestPhysFallVehicleLaunch();
    TestVehicleExitSpeedClamp(); // vehicle batch W1 CAR-2
    TestFalseTouchInFastFall(); // fix/jumpout-cam
    TestFalseTouchGate(); // review r2 finding 14
    TestTouchGuardRisingBlip();
    TestTouchGuardContactEvidence();
    TestPredictedTouchExpiryAndReset();
    TestStaleLandDetailWhileFalling();
    TestJumpEdgeRequiresImpulse();
    TestContactRampZeroAtTouch();
    std::printf("MMAirStateTest: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
