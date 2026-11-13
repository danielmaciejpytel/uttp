// Offline test for the game look write controller (src/input/GameLookCore.hpp; 0.6.5b feed-forward + low-gain feedback,
// round 2: override ticks, release correction, measured ahead order, rate based divergence guard, re-arm policy)
// and, with an argument, its code sites against a COPY of the game executable (CMake UTPP_EXE_COPY =
// Extracted\reverse\exe250\Cyberpunk2077.exe, game 2.5: the kRva* sites in GameLookCore.hpp match 2.5 and fail on exe231, checked
// 2026-10-23; never the installed game file). No game, no RED4ext, no Windows API.
// Usage: utpp_gamelook_test [<Cyberpunk2077.exe copy>]
//   controller: yaw convention, wrap, pitch clamp, skips (not locked / 999 / NaN), first write = feedback only, exact
//   feed-forward, entity turn ahead, lag hint never changes the command, ahead order residuals (after / before / late),
//   override prediction (NoteApplied), release correction, yawMultiplier compensation, re-arm policy;
//   closed loop against a simulated graph (YawInput += mult * delta) with a delay of 0, 1 or randomly 0 / 1 per tick
//   (queue: every delta applied once) and a "sample" graph (the feature read before or after the write at random: deltas
//   lost / doubled), mouse flicks up to 2000 deg/s yaw and 600 deg/s pitch at 30 / 60 / 144 fps, V turned by the aim
//   align (<= 1080 deg/s) with and without the ahead feed: bounded steady error, no ringing, transient <= one tick of the
//   target motion, no trip in legitimate runs;
//   in-game override plants (the game overrides the FPP look in the ticks in which utpp_bodyturn's adjustTransform lands,
//   log 2026-10-13 01:05 / 01:25): "decay" (delta ignored, YawInput / PitchInput pulled to 0 with tau 0.0927 s), "freeze"
//   (delta ignored, state kept), "blend" (the read-back overwritten with 0 while the graph integrates the delta): no trip
//   while V turns away from / toward a still camera at 30-240 deg/s, no wind-up, the error gone a few ticks after V stops;
//   ahead order "before" / "late" measured and used;
//   divergence trip on a graph that ignores the delta / a wrong sign while the target moves slowly; no trip on a constant
//   large error (clamp, the game pulling the look);
//   sites: every byte pattern the plugin checks before attaching matches exactly once at its RVA, the apply call site
//   targets the apply function, the refresh call targets the graph variable read.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <utility>
#include <vector>

#include "../src/input/GameLookCore.hpp"

using namespace utpp::gamelook;

namespace
{
int g_fail = 0;
int g_pass = 0;

void Check(bool aOk, const char* aName, double aV = 0.0)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s (%.6f)\n", aName, aV);
    }
}

constexpr float kPi = 3.14159265358979f;

// Simulated FPP graph: integrates the deltas the camera update writes.
//   D0: applied before the next read; D1: one tick later; Random: 0 or 1 per tick (queue, every delta applied once);
//   Sample: the graph reads the feature before (old value) or after (new value) the write at random (lost / doubled).
enum class Delay
{
    D0,
    D1,
    Random,
    Sample
};
const char* DelayName(Delay aD)
{
    switch (aD)
    {
    case Delay::D0:
        return "delay 0";
    case Delay::D1:
        return "delay 1";
    case Delay::Random:
        return "delay 0/1 random";
    case Delay::Sample:
        return "sample (lost/doubled)";
    }
    return "?";
}

// What the game does to the FPP look in a tick in which V's adjustTransform turn lands (in-game override)
enum class Ovr
{
    None,  // nothing (the 0.6.5b round 1 assumption)
    Decay, // the delta is ignored and YawInput / PitchInput decay to 0 with tau 0.0927 s (log 01:05, Locomotion rows)
    Freeze, // the delta is ignored, the state kept
    Blend, // the graph integrates the delta, the read-back of the next tick is overwritten with 0 / 0 (output override)
};
const char* OvrName(Ovr aO)
{
    switch (aO)
    {
    case Ovr::None:
        return "no override";
    case Ovr::Decay:
        return "override decay";
    case Ovr::Freeze:
        return "override freeze";
    case Ovr::Blend:
        return "override blend";
    }
    return "?";
}

struct Graph
{
    float yawIn = 0.0f, pitchIn = 0.0f; // internal state = the game's look (relative to the entity)
    Delay delay = Delay::D0;
    float mult = 1.0f;
    float sign = 1.0f;     // -1: wrong mapping (the loop must trip)
    bool ignore = false;   // the graph ignores the delta (divergence model)
    float pull = 0.0f;     // per tick fraction the game pulls YawInput / PitchInput toward 0 (bounded fight)
    float yawLimit = 0.0f; // > 0: YawInput clamped to +-yawLimit (bounded)
    Ovr ovr = Ovr::None;
    float tau = 0.0927f;
    bool readOverridden = false; // Blend: the read-back of the next tick is the override (0 / 0)
    uint32_t rng = 12345u;
    std::deque<std::pair<float, float>> q;
    float featYaw = 0.0f, featPitch = 0.0f;

    int Rand01()
    {
        rng = rng * 1664525u + 1013904223u;
        return static_cast<int>((rng >> 16) & 1u);
    }
    float ReadYaw() const
    {
        return readOverridden ? 0.0f : yawIn;
    }
    float ReadPitch() const
    {
        return readOverridden ? 0.0f : pitchIn;
    }
    void Integrate(float aYaw, float aPitch)
    {
        if (ignore)
            return;
        yawIn = Wrap180(yawIn + sign * mult * aYaw);
        pitchIn += sign * mult * aPitch;
        if (yawLimit > 0.0f)
            yawIn = ClampAbs(yawIn, yawLimit);
    }
    // aDt: tick time; aTurn: V's adjustTransform turn lands in this tick (after the camera update)
    void Apply(float aYaw, float aPitch, float aDt = 1.0f / 60.0f, bool aTurn = false)
    {
        const bool ov = aTurn && ovr != Ovr::None;
        readOverridden = ov && ovr == Ovr::Blend;
        if (ov && ovr != Ovr::Blend)
        {
            // the delta is ignored (the queue is dropped too: the game does not apply it later)
            q.clear();
            featYaw = featPitch = 0.0f;
            if (ovr == Ovr::Decay)
            {
                const float f = std::exp(-aDt / tau);
                yawIn *= f;
                pitchIn *= f;
            }
            return;
        }
        if (delay == Delay::Sample)
        {
            const bool late = Rand01() == 1;
            const float uy = late ? featYaw : aYaw;
            const float up = late ? featPitch : aPitch;
            featYaw = aYaw;
            featPitch = aPitch;
            Integrate(uy, up);
        }
        else
        {
            const int lag = delay == Delay::D0 ? 0 : (delay == Delay::D1 ? 1 : Rand01());
            q.emplace_back(aYaw, aPitch);
            while (static_cast<int>(q.size()) > lag)
            {
                Integrate(q.front().first, q.front().second);
                q.pop_front();
            }
        }
        if (pull > 0.0f)
        {
            yawIn *= 1.0f - pull;
            pitchIn *= 1.0f - pull;
        }
    }
};

// One run: the mouse (target) and V (entity) move; each tick the camera update reads the graph and writes the delta,
// then the graph applies it and the entity turn requested for the tick lands (order: after the camera update by
// default; Before: before the read of the same tick; Late: after the camera update of the next tick).
struct Scenario
{
    float fps = 60.0f;
    float seconds = 1.5f;
    float yaw0 = 0.0f;                                        // target = ent0 + yaw0 at t = 0 (settled start)
    float yawRate = 0.0f, yawStart = 0.2f, yawDur = 0.0f;     // mouse flick (deg/s)
    float pitchRate = 0.0f, pitchStart = 0.2f, pitchDur = 0.0f;
    float pitch0 = 0.0f;
    float sineAmp = 0.0f, sineHz = 0.0f, sinePitchAmp = 0.0f; // continuous motion
    float walk = 0.0f;                                        // > 0: random walk of the mouse (deg/s scale)
    float walkDur = 1e9f;                                     // s of random walk
    float alignRate = 0.0f;                                   // utpp_bodyturn aim align (deg/s, deadband 2)
    float turnRate = 0.0f, turnStart = 0.0f, turnDur = 1e9f;  // constant entity turn (deg/s; Locomotion)
    int turnEvery = 1;                                        // the Locomotion turn requested only every n-th tick
    bool ahead = false;                                       // the requested turn passed as Inputs.aheadYaw
    AheadOrder order = AheadOrder::After;                     // when the requested turn really lands
    float ent0 = 10.0f;
    float jitter = 0.0f;                                      // frame time jitter (fraction, uniform)
    uint32_t seed = 99u;
    bool featureRead = true; // as the native: yaw / pitch multiplier of the last apply and NoteApplied(feature delta)
};

struct Result
{
    float maxErr = 0.0f;   // max |visible error| (yaw / pitch) over the run (internal state = the game's look)
    float maxRel = 0.0f;   // max relative target motion per tick (yaw: target change - entity turn; pitch)
    float maxEnt = 0.0f;   // max entity turn per tick
    float lastMotion = 0.0f;
    float errAfter = 0.0f; // max |error| from lastMotion + settle on
    int reversals = 0;     // sign reversals of the error with |e| > 1 deg after the last motion
    int writes = 0;
    int ticks = 0;
    float maxCmd = 0.0f;
    bool tripped = false;
    float tripAt = -1.0f;
    int tripKind = 0;       // Controller::tripKind (1 growth, 2 spin)
    uint64_t releases = 0, ovrTicks = 0;
    AheadOrder order = AheadOrder::After;
    uint32_t orderSwitches = 0;
    double resAfter = 0, resBefore = 0, resLate = 0;
    uint64_t discN = 0;
};

struct Rng
{
    uint32_t s;
    float U() // [0, 1)
    {
        s = s * 1664525u + 1013904223u;
        return static_cast<float>((s >> 8) & 0xFFFFFFu) / 16777216.0f;
    }
};

Result Run(Graph aG, const Scenario& aS, float aSettle)
{
    Controller c;
    Result r;
    Rng rng{aS.seed};
    const float dt0 = 1.0f / aS.fps;
    float ent = aS.ent0;
    float t = 0.0f;
    float ty = Wrap180(aS.ent0 + aS.yaw0), tp = aS.pitch0;
    float walkV = 0.0f;
    float prevT = ty, prevP = tp;
    float lateStep = 0.0f; // Late order: the turn of the last tick, lands after this tick's camera update
    struct Rec
    {
        float t, ey, ep;
    };
    std::vector<Rec> recs;
    while (t < aS.seconds)
    {
        const float dt = dt0 * (1.0f + aS.jitter * (2.0f * rng.U() - 1.0f));
        // target of this tick
        float base = aS.ent0 + aS.yaw0;
        if (t > aS.yawStart)
            base += aS.yawRate * std::fmin(t - aS.yawStart, aS.yawDur);
        base += aS.sineAmp * std::sin(2.0f * kPi * aS.sineHz * t);
        float p = aS.pitch0;
        if (t > aS.pitchStart)
            p += aS.pitchRate * std::fmin(t - aS.pitchStart, aS.pitchDur);
        p += aS.sinePitchAmp * std::sin(2.0f * kPi * aS.sineHz * 0.7f * t);
        if (aS.walk > 0.0f && t < aS.walkDur && r.ticks > 0) // settled first tick
        {
            walkV = 0.9f * walkV + 0.1f * aS.walk * (2.0f * rng.U() - 1.0f) * 3.0f;
            ty = Wrap180(ty + walkV * dt);
            tp = ClampAbs(tp + 0.3f * walkV * dt * (rng.U() < 0.5f ? 1.0f : -1.0f), 70.0f);
        }
        else if (aS.walk <= 0.0f)
        {
            ty = Wrap180(base);
            tp = p;
        }
        // V's turn requested this tick (utpp_bodyturn): Locomotion constant turn and / or the aim align onto the target
        float step = 0.0f;
        if (t >= aS.turnStart && t < aS.turnStart + aS.turnDur && (r.ticks % aS.turnEvery) == 0)
            step += aS.turnRate * dt * static_cast<float>(aS.turnEvery);
        if (aS.alignRate > 0.0f)
        {
            const float err = Wrap180(ty - ent);
            if (std::fabs(err) >= 2.0f)
            {
                const float lim = aS.alignRate * std::fmin(std::fmax(dt, 1.0f / 240.0f), 0.1f);
                step += ClampAbs(err, lim);
            }
        }
        if (aS.order == AheadOrder::Before)
            ent = Wrap180(ent + step); // landed before this read
        Inputs in;
        in.targetYaw = ty;
        in.targetPitch = tp;
        in.entityYaw = ent;
        in.yawInput = aG.ReadYaw();
        in.pitchInput = aG.ReadPitch();
        in.headingLocked = true;
        in.aheadYaw = aS.ahead ? step : 0.0f;
        in.dt = dt;
        if (aS.featureRead)
            in.yawMult = in.pitchMult = aG.mult;
        const Output o = c.Step(in);
        if (o.write && aS.featureRead) // the feature after the game's apply: the delta as written, times the multiplier
            c.NoteApplied(o.yawDelta * aG.mult, o.pitchDelta * aG.mult);
        if (o.trippedNow && !r.tripped)
            r.tripAt = t;
        r.tripped = r.tripped || o.trippedNow;
        // the turn that lands after this camera update (the graph of this tick is overridden by it)
        float landed = 0.0f;
        if (aS.order == AheadOrder::After)
            landed = step;
        else if (aS.order == AheadOrder::Late)
        {
            landed = lateStep;
            lateStep = step;
        }
        const bool turnLands = landed != 0.0f || (aS.order == AheadOrder::Before && step != 0.0f);
        if (o.write)
        {
            ++r.writes;
            r.maxCmd = std::fmax(r.maxCmd, std::fmax(std::fabs(o.yawDelta), std::fabs(o.pitchDelta)));
            aG.Apply(o.yawDelta, o.pitchDelta, dt, turnLands);
        }
        else
            aG.Apply(0.0f, 0.0f, dt, turnLands);
        if (aS.order != AheadOrder::Before)
            ent = Wrap180(ent + landed);
        const float cp = tp < -80.0f ? -80.0f : (tp > 80.0f ? 80.0f : tp);
        const float ey = Wrap180(ty - (ent + aG.yawIn));
        const float ep = cp - aG.pitchIn;
        const float dT = Wrap180(ty - prevT), dP = tp - prevP;
        r.maxRel = std::fmax(r.maxRel, std::fmax(std::fabs(Wrap180(dT - step)), std::fabs(dP)));
        r.maxEnt = std::fmax(r.maxEnt, std::fabs(step));
        if (std::fabs(dT) > 1e-4f || std::fabs(dP) > 1e-4f || std::fabs(step) > 1e-4f || std::fabs(landed) > 1e-4f)
            r.lastMotion = t;
        prevT = ty;
        prevP = tp;
        recs.push_back({t, ey, ep});
        t += dt;
        ++r.ticks;
    }
    int prevSign = 0;
    for (const Rec& e : recs)
    {
        const float m = std::fmax(std::fabs(e.ey), std::fabs(e.ep));
        r.maxErr = std::fmax(r.maxErr, m);
        if (e.t >= r.lastMotion + aSettle - 1e-4f)
            r.errAfter = std::fmax(r.errAfter, m);
        if (e.t > r.lastMotion)
        {
            const float v = std::fabs(e.ey) >= std::fabs(e.ep) ? e.ey : e.ep;
            if (std::fabs(v) > 1.0f)
            {
                const int s = v > 0.0f ? 1 : -1;
                if (prevSign != 0 && s != prevSign)
                    ++r.reversals;
                prevSign = s;
            }
        }
    }
    r.tripKind = c.tripKind;
    r.releases = c.releases;
    r.ovrTicks = c.ovrTicks;
    r.order = c.aheadOrder;
    r.orderSwitches = c.aheadOrderSwitches;
    r.resAfter = c.resAfter;
    r.resBefore = c.resBefore;
    r.resLate = c.resLate;
    r.discN = c.aheadDiscN;
    return r;
}

// Settling time after the motion ended: delay 0 and delay 1 0.2 s, except delay 1 at 30 fps 0.4 s (0.2 s are 6 ticks
// there: the delay-1 loop cannot decay a one-frame transient of 66 deg (2000 deg/s) 130x in 6 ticks, poles 0.5); the
// random / sample graphs (a switched loop, deltas lost / doubled) 0.4 s, at 30 fps 0.6 s.
float SettleTime(Delay aD, float aFps)
{
    if (aD == Delay::D0)
        return 0.2f;
    if (aD == Delay::D1)
        return aFps < 45.0f ? 0.4f : 0.2f;
    return aFps < 45.0f ? 0.6f : 0.4f;
}

void Report(const char* aWhat, const char* aPlant, float aFps, const Result& aR)
{
    std::printf("  %-36s %-24s %5.0f fps: maxErr %7.3f maxRel %6.2f maxEnt %5.2f errAfter %.4f rev %d maxCmd %.1f "
                "ovr %llu rel %llu trip %d\n",
                aWhat, aPlant, static_cast<double>(aFps), static_cast<double>(aR.maxErr), static_cast<double>(aR.maxRel),
                static_cast<double>(aR.maxEnt), static_cast<double>(aR.errAfter), aR.reversals,
                static_cast<double>(aR.maxCmd), static_cast<unsigned long long>(aR.ovrTicks),
                static_cast<unsigned long long>(aR.releases), aR.tripped ? 1 : 0);
}

// Transient bound:
//   delay 0: exact (the feed-forward carries the motion; with the ahead feed also V's turn)
//   delay 1 / random: one tick of the relative target motion, +20 % when it changes on consecutive ticks (the error of
//            one change lasts two ticks: impulse response (n + 1) 0.5^n)
//   sample: two ticks (a lost delta and a doubled one)
float TransientBound(Delay aD, const Result& aR, bool aExact)
{
    if (aD == Delay::D0 && aExact)
        return 0.5f;
    const float f = aD == Delay::Sample ? 2.0f : (aD == Delay::D0 ? 1.0f : 1.2f);
    return f * aR.maxRel + 0.5f;
}
bool TransientOk(Delay aD, const Result& aR, bool aExact)
{
    return aR.maxErr <= TransientBound(aD, aR, aExact);
}

void ClosedLoopTests()
{
    const Delay delays[] = {Delay::D0, Delay::D1, Delay::Random, Delay::Sample};
    const float fpsList[] = {30.0f, 60.0f, 144.0f};
    char name[240];
    std::printf("closed loop (k=%.2f; poles delay 0: %.2f, delay 1: z^2 - z + k = 0 -> %.2f double):\n",
                static_cast<double>(kGain), static_cast<double>(1.0f - kGain), 0.5);
    for (const Delay d : delays)
    {
        for (const float fps : fpsList)
        {
            const float settle = SettleTime(d, fps);
            // 1. mouse flick: yaw 2000 deg/s for 0.25 s and pitch 600 deg/s for 0.1 s (V stands)
            {
                Scenario s;
                s.fps = fps;
                s.yawRate = 2000.0f;
                s.yawDur = 0.25f;
                s.pitchRate = 600.0f;
                s.pitchDur = 0.1f;
                Graph g;
                g.delay = d;
                const Result r = Run(g, s, settle);
                Report("flick 2000 / 600 deg/s", DelayName(d), fps, r);
                std::snprintf(name, sizeof(name), "%s %.0f fps flick 2000/600: no trip, every tick written", DelayName(d), fps);
                Check(!r.tripped && r.writes == r.ticks, name, r.writes);
                std::snprintf(name, sizeof(name), "%s %.0f fps flick: transient <= one tick of the motion (%s)", DelayName(d),
                              fps, d == Delay::D0 ? "delay 0: exact" : "max |dTarget|");
                Check(TransientOk(d, r, true), name, r.maxErr);
                std::snprintf(name, sizeof(name), "%s %.0f fps flick: steady error < 0.5 deg %.1f s after the stop",
                              DelayName(d), fps, settle);
                Check(r.errAfter < 0.5f, name, r.errAfter);
                std::snprintf(name, sizeof(name), "%s %.0f fps flick: no ringing after the stop (<= 1 reversal > 1 deg)",
                              DelayName(d), fps);
                Check(r.reversals <= 1, name, r.reversals);
            }
            // 2. aim align: the mouse flicks 2000 deg/s, V follows at <= 1080 deg/s (request lands after the camera
            //    update), with the ahead feed (every tick with a turn is an override tick: prediction + release)
            for (int withAhead = 1; withAhead >= 0; --withAhead)
            {
                Scenario s;
                s.fps = fps;
                s.yawRate = 2000.0f;
                s.yawDur = 0.25f;
                s.pitchRate = -600.0f;
                s.pitchDur = 0.1f;
                s.alignRate = 1080.0f;
                s.ahead = withAhead == 1;
                s.seconds = 2.0f;
                Graph g;
                g.delay = d;
                const Result r = Run(g, s, settle);
                Report(withAhead ? "flick + aim align, ahead feed" : "flick + aim align, NO ahead", DelayName(d), fps, r);
                std::snprintf(name, sizeof(name), "%s %.0f fps aim align (%s): no trip", DelayName(d), fps,
                              withAhead ? "ahead" : "no ahead");
                Check(!r.tripped && r.writes == r.ticks, name, r.writes);
                if (withAhead)
                {
                    std::snprintf(name, sizeof(name), "%s %.0f fps aim align with ahead: transient <= one tick of the "
                                  "relative motion (delay 0: exact)", DelayName(d), fps);
                    Check(TransientOk(d, r, true), name, r.maxErr);
                    std::snprintf(name, sizeof(name), "%s %.0f fps aim align with ahead: override ticks and >= 1 release",
                                  DelayName(d), fps);
                    Check(r.ovrTicks > 0 && r.releases >= 1, name, static_cast<double>(r.releases));
                }
                else if (d == Delay::D0)
                {
                    // without the feed V's turn of the tick shows up as error (<= one tick of V's turn), so the feed matters
                    std::snprintf(name, sizeof(name), "delay 0 %.0f fps aim align WITHOUT ahead: error = one tick of V's turn "
                                  "(> 1 deg, <= max turn)", fps);
                    Check(r.maxErr > 1.0f && r.maxErr <= r.maxEnt + 0.5f, name, r.maxErr);
                }
                else
                {
                    std::snprintf(name, sizeof(name), "%s %.0f fps aim align without ahead: bounded (<= one tick of target "
                                  "+ V's turn)", DelayName(d), fps);
                    const float f = d == Delay::Sample ? 2.0f : 1.2f;
                    Check(r.maxErr <= f * (r.maxRel + r.maxEnt) + 0.5f, name, r.maxErr);
                }
                std::snprintf(name, sizeof(name), "%s %.0f fps aim align (%s): steady error < 0.5 deg %.1f s after V stops",
                              DelayName(d), fps, withAhead ? "ahead" : "no ahead", settle);
                Check(r.errAfter < 0.5f, name, r.errAfter);
                std::snprintf(name, sizeof(name), "%s %.0f fps aim align (%s): no ringing after the stop", DelayName(d), fps,
                              withAhead ? "ahead" : "no ahead");
                Check(r.reversals <= 2, name, r.reversals);
            }
            // 3. continuous motion for 10 s (yaw sine 120 deg at 1.5 Hz: peak 1130 deg/s, pitch 40 deg) with the aim align
            //    and the ahead feed: never trips, bounded by one tick of the relative motion (sample graph: the override
            //    ticks run on the delay-0 prediction, so lost / doubled deltas add up until the release - bounded by 6 ticks
            //    of motion here, corrected after it; the in-game graph is delay 0: 4 min with lwerr 0.00 at up to 760 deg/s)
            {
                Scenario s;
                s.fps = fps;
                s.seconds = 10.0f;
                s.sineAmp = 120.0f;
                s.sineHz = 1.5f;
                s.sinePitchAmp = 40.0f;
                s.alignRate = 1080.0f;
                s.ahead = true;
                Graph g;
                g.delay = d;
                g.rng = 777u + static_cast<uint32_t>(fps);
                const Result r = Run(g, s, 0.2f);
                Report("10 s continuous + aim align", DelayName(d), fps, r);
                // delay 1 / random: one tick of the command (open loop on the delay-0 prediction while V turns);
                // sample: lost / doubled deltas add up until the release (reported, not bounded per tick)
                bool bounded = true;
                if (d == Delay::D0)
                    bounded = TransientOk(d, r, true);
                else if (d != Delay::Sample)
                    bounded = r.maxErr <= 1.2f * std::fmax(r.maxRel, r.maxCmd) + 0.5f;
                std::snprintf(name, sizeof(name), "%s %.0f fps 10 s continuous motion + aim align + ahead: no trip, bounded%s",
                              DelayName(d), fps, d == Delay::Sample ? " (sample: not checked)" : "");
                Check(!r.tripped && r.writes == r.ticks && bounded, name, r.maxErr);
                std::snprintf(name, sizeof(name), "%s %.0f fps 10 s continuous: error < 0.5 deg 0.2 s after the motion",
                              DelayName(d), fps);
                Check(r.errAfter < 0.5f, name, r.errAfter);
            }
            // 4. Locomotion-like constant turn of V (300 deg/s) while the mouse stands: delay 0 exact with the feed
            {
                Scenario s;
                s.fps = fps;
                s.turnRate = 300.0f;
                s.ahead = true;
                Graph g;
                g.delay = d;
                const Result r = Run(g, s, settle);
                std::snprintf(name, sizeof(name), "%s %.0f fps V turning 300 deg/s, ahead feed: no trip, transient <= one tick",
                              DelayName(d), fps);
                Check(!r.tripped && TransientOk(d, r, true), name, r.maxErr);
            }
        }
    }
}

// The in-game override (round 2 review): utpp_bodyturn turns V by adjustTransform and the game overrides the FPP look in
// those ticks. Locomotion in the free camera (V turns toward the movement, away from / back to the still camera) and the aim
// align while the mouse moves.
void OverrideTests()
{
    const float fpsList[] = {30.0f, 60.0f, 144.0f};
    const Ovr plants[] = {Ovr::Decay, Ovr::Freeze, Ovr::Blend};
    char name[260];
    std::printf("override plants (V turned by utpp_bodyturn adjustTransform):\n");
    for (const Ovr pl : plants)
    {
        for (const float fps : fpsList)
        {
            // Locomotion: V turns away from the still camera at 30..240 deg/s (up to 179 deg), stops, and the shape of the
            // 01:10 episode (V turns back onto the camera)
            const float rates[] = {30.0f, 60.0f, 120.0f, 180.0f, 240.0f};
            for (const float rate : rates)
            {
                for (int back = 0; back < 2; ++back)
                {
                    Scenario s;
                    s.fps = fps;
                    s.seconds = 3.0f;
                    s.turnRate = back ? -rate : rate;
                    s.turnStart = 0.3f;
                    s.turnDur = std::fmin(179.0f / rate, 1.6f);
                    s.yaw0 = back ? -std::fmin(179.0f, rate * s.turnDur) : 0.0f; // back: the camera where V turns to
                    s.ahead = true;
                    s.pitch0 = 14.0f; // camera pitch of the 01:10 episode
                    Graph g;
                    g.ovr = pl;
                    g.yawIn = s.yaw0;
                    g.pitchIn = s.pitch0;
                    const Result r = Run(g, s, 0.1f);
                    std::snprintf(name, sizeof(name), "%s %.0f fps V turns %s the camera at %.0f deg/s", OvrName(pl), fps,
                                  back ? "back onto" : "away from", static_cast<double>(rate));
                    Report(name, "delay 0", fps, r);
                    std::snprintf(name, sizeof(name), "%s %.0f fps V turns %s the still camera at %.0f deg/s (ahead): no trip",
                                  OvrName(pl), fps, back ? "back onto" : "away from", static_cast<double>(rate));
                    Check(!r.tripped, name, r.tripAt);
                    std::snprintf(name, sizeof(name), "%s %.0f fps %.0f deg/s %s: error < 0.5 deg 0.1 s after V stops "
                                  "(release correction)", OvrName(pl), fps, static_cast<double>(rate), back ? "back" : "away");
                    Check(r.errAfter < 0.5f, name, r.errAfter);
                    std::snprintf(name, sizeof(name), "%s %.0f fps %.0f deg/s %s: no ringing after the release", OvrName(pl),
                                  fps, static_cast<double>(rate), back ? "back" : "away");
                    Check(r.reversals <= 1, name, r.reversals);
                }
            }
            // the turn requested only every 2nd / 3rd tick (steps below the executor's minimum in between): the clean ticks
            // between the overrides do not count for the guard (kOvrGuardHold) - no trip, error gone after V stops
            for (int every = 2; every <= 3; ++every)
            {
                Scenario s;
                s.fps = fps;
                s.seconds = 3.0f;
                s.turnRate = 60.0f;
                s.turnStart = 0.3f;
                s.turnDur = 1.6f;
                s.turnEvery = every;
                s.ahead = true;
                s.pitch0 = 14.0f;
                Graph g;
                g.ovr = pl;
                g.pitchIn = s.pitch0;
                const Result r = Run(g, s, 0.15f);
                std::snprintf(name, sizeof(name), "%s %.0f fps V turns away 60 deg/s, a request every %d ticks: no trip, "
                              "error < 0.5 deg after V stops", OvrName(pl), fps, every);
                Check(!r.tripped && r.errAfter < 0.5f, name, r.errAfter);
            }
            // the camera pitches slowly (40 deg/s) while V turns under the override for 1.5 s: no trip
            {
                Scenario s;
                s.fps = fps;
                s.seconds = 2.5f;
                s.turnRate = 120.0f;
                s.turnStart = 0.2f;
                s.turnDur = 1.4f;
                s.pitchRate = 40.0f;
                s.pitchStart = 0.2f;
                s.pitchDur = 1.4f;
                s.ahead = true;
                Graph g;
                g.ovr = pl;
                const Result r = Run(g, s, 0.1f);
                std::snprintf(name, sizeof(name), "%s %.0f fps slow camera pitch 40 deg/s while V turns 120 deg/s: no trip, "
                              "error gone after V stops", OvrName(pl), fps);
                Check(!r.tripped && r.errAfter < 0.5f, name, r.errAfter);
            }
            // aim align under the override: flicks, random walk (400 deg/s scale), sine 1 Hz; the error after V stops
            // (the game's look once the override ends) - the 0.6.5b round 1 loop wound up 50-180 deg here (blend)
            for (int kind = 0; kind < 3; ++kind)
            {
                Scenario s;
                s.fps = fps;
                s.alignRate = 1080.0f;
                s.ahead = true;
                s.jitter = 0.3f;
                s.seed = 4242u + static_cast<uint32_t>(fps) + static_cast<uint32_t>(kind);
                const char* what = "?";
                if (kind == 0)
                {
                    s.seconds = 2.0f;
                    s.yawRate = 2000.0f;
                    s.yawDur = 0.25f;
                    s.pitchRate = -300.0f;
                    s.pitchDur = 0.1f;
                    what = "flick 2000 deg/s";
                }
                else if (kind == 1)
                {
                    s.seconds = 4.0f;
                    s.walk = 400.0f;
                    s.walkDur = 3.0f;
                    what = "random walk 400";
                }
                else
                {
                    s.seconds = 4.0f;
                    s.sineAmp = 60.0f;
                    s.sineHz = 1.0f;
                    s.sinePitchAmp = 20.0f;
                    what = "sine 60 deg 1 Hz";
                }
                Graph g;
                g.ovr = pl;
                const Result r = Run(g, s, 0.15f);
                std::snprintf(name, sizeof(name), "aim align %s", what);
                Report(name, OvrName(pl), fps, r);
                std::snprintf(name, sizeof(name), "%s %.0f fps aim align, %s: no trip, no wind-up (error < 0.5 deg 0.15 s "
                              "after V stops)", OvrName(pl), fps, what);
                Check(!r.tripped && r.errAfter < 0.5f, name, r.errAfter);
                if (pl == Ovr::Blend)
                {
                    // the graph integrated every delta (only the read-back was overridden): the game's look stayed exact
                    std::snprintf(name, sizeof(name), "blend %.0f fps aim align, %s: the game's look exact throughout "
                                  "(prediction during the override)", fps, what);
                    Check(r.maxErr < 0.5f, name, r.maxErr);
                }
            }
        }
    }
    // a graph delay of 1 with the override (decay) and the release: no overshoot ringing after V stops
    for (const float fps : fpsList)
    {
        Scenario s;
        s.fps = fps;
        s.seconds = 2.5f;
        s.turnRate = 150.0f;
        s.turnStart = 0.2f;
        s.turnDur = 1.0f;
        s.ahead = true;
        Graph g;
        g.ovr = Ovr::Decay;
        g.delay = Delay::D1;
        const Result r = Run(g, s, 0.2f);
        std::snprintf(name, sizeof(name), "delay 1 + override decay %.0f fps V turns 150 deg/s: release exact (no ringing, "
                      "error < 0.5 deg 0.2 s after)", fps);
        Check(!r.tripped && r.errAfter < 0.5f && r.reversals <= 1, name, r.errAfter);
    }
    // the ahead never arrives (old CET / a lost step): the loop fights the override and may trip; the trip is not for the
    // session any more (native re-arm policy)
    {
        Scenario s;
        s.fps = 60.0f;
        s.seconds = 4.0f;
        s.turnRate = 60.0f;
        s.turnStart = 0.2f;
        s.turnDur = 1.5f;
        s.ahead = false;
        Graph g;
        g.ovr = Ovr::Decay;
        const Result r = Run(g, s, 0.2f);
        std::printf("  without the ahead (old CET), decay, V turns 60 deg/s away: trip %d at %.2f s (kind %d)\n",
                    r.tripped ? 1 : 0, static_cast<double>(r.tripAt), r.tripKind);
        Check(!RearmDue(kTripCooldown - 0.1f, 1) && RearmDue(kTripCooldown, 1) && RearmDue(kTripCooldown + 5.0f, kMaxTrips - 1) &&
                  !RearmDue(1e6f, kMaxTrips),
              "re-arm policy: after kTripCooldown s, at most kMaxTrips trips per session");
    }
}

// The order of V's turn and the camera update is measured and used
void AheadOrderTests()
{
    const float fpsList[] = {30.0f, 60.0f, 144.0f};
    const AheadOrder orders[] = {AheadOrder::After, AheadOrder::Before, AheadOrder::Late};
    char name[240];
    for (const AheadOrder ord : orders)
    {
        for (const float fps : fpsList)
        {
            Scenario s;
            s.fps = fps;
            s.seconds = 6.0f;
            s.sineAmp = 80.0f;
            s.sineHz = 0.8f;
            s.alignRate = 1080.0f;
            s.ahead = true;
            s.order = ord;
            s.jitter = 0.3f;
            s.seed = 11u + static_cast<uint32_t>(fps);
            Graph g;
            const Result r = Run(g, s, 0.2f);
            std::printf("  ahead order %-6s %5.0f fps: measured %s (switches %u, disc %llu, means after %.3f before %.3f late "
                        "%.3f) maxErr %.3f errAfter %.4f\n",
                        AheadOrderName(ord), static_cast<double>(fps), AheadOrderName(r.order), r.orderSwitches,
                        static_cast<unsigned long long>(r.discN), r.discN ? r.resAfter / r.discN : 0.0,
                        r.discN ? r.resBefore / r.discN : 0.0, r.discN ? r.resLate / r.discN : 0.0,
                        static_cast<double>(r.maxErr), static_cast<double>(r.errAfter));
            std::snprintf(name, sizeof(name), "ahead order %s %.0f fps: measured as %s, residual of the true order ~0",
                          AheadOrderName(ord), fps, AheadOrderName(ord));
            const double resTrue = ord == AheadOrder::After ? r.resAfter : (ord == AheadOrder::Before ? r.resBefore : r.resLate);
            Check(r.order == ord && r.discN >= 20 && resTrue < 1e-3 * static_cast<double>(r.discN), name,
                  static_cast<double>(r.order));
            std::snprintf(name, sizeof(name), "ahead order %s %.0f fps: no trip, error < 0.5 deg after V stops",
                          AheadOrderName(ord), fps);
            Check(!r.tripped && r.errAfter < 0.5f, name, r.errAfter);
        }
    }
}

void DivergenceTests()
{
    const float fpsList[] = {30.0f, 60.0f, 144.0f};
    char name[200];
    for (const float fps : fpsList)
    {
        // a graph that ignores the delta while the mouse moves slowly (60 deg/s): the error grows -> trip
        {
            Scenario s;
            s.fps = fps;
            s.seconds = 4.0f;
            s.yawRate = 60.0f;
            s.yawDur = 10.0f;
            Graph g;
            g.ignore = true;
            const Result r = Run(g, s, 0.2f);
            std::snprintf(name, sizeof(name), "%.0f fps graph ignores the delta, target 60 deg/s: trips (fail closed, the "
                          "error grew in 2 windows)", fps);
            Check(r.tripped && r.writes < r.ticks && r.tripKind == 1, name, r.writes);
            // the same at 150 deg/s (slow in deg/s at every frame rate; round 1 counted it as fast at 30 fps: 5 deg/tick)
            Scenario s2 = s;
            s2.yawRate = 150.0f;
            const Result r2 = Run(g, s2, 0.2f);
            std::snprintf(name, sizeof(name), "%.0f fps graph ignores the delta, target 150 deg/s (< %.0f deg/s): trips",
                          fps, static_cast<double>(kDivergeSlowRate));
            Check(r2.tripped, name, r2.writes);
        }
        // wrong sign (mapping): the error grows -> trip
        {
            Scenario s;
            s.fps = fps;
            s.seconds = 4.0f;
            s.yawRate = 30.0f;
            s.yawDur = 10.0f;
            Graph g;
            g.sign = -1.0f;
            const Result r = Run(g, s, 0.2f);
            std::snprintf(name, sizeof(name), "%.0f fps wrong sign, target 30 deg/s: trips within 2 s", fps);
            Check(r.tripped && r.tripAt < 2.0f, name, r.tripAt);
            // the same with a still target after one small offset (no feed-forward excitation): trips as well
            Scenario s2;
            s2.fps = fps;
            s2.seconds = 4.0f;
            s2.yaw0 = 3.0f;
            Graph g2;
            g2.sign = -1.0f;
            g2.delay = Delay::D1;
            const Result r2 = Run(g2, s2, 0.2f);
            std::snprintf(name, sizeof(name), "%.0f fps wrong sign, delay 1, still target 3 deg off: trips", fps);
            Check(r2.tripped, name, r2.writes);
        }
        // legitimate large but constant errors: no trip (5 s)
        {
            Scenario s;
            s.fps = fps;
            s.seconds = 5.0f;
            s.yaw0 = 90.0f;
            Graph g;
            g.yawLimit = 45.0f; // YawInput clamped at 45: error 45 forever
            const Result r = Run(g, s, 0.2f);
            std::snprintf(name, sizeof(name), "%.0f fps YawInput clamp (constant 45 deg error, still target): no trip", fps);
            Check(!r.tripped && r.maxErr > 40.0f, name, r.maxErr);
            Graph g2;
            g2.pull = 0.16f; // the game pulls the look toward V by 16 % per tick (bounded fight)
            Scenario s2 = s;
            s2.yaw0 = 100.0f;
            const Result r2 = Run(g2, s2, 0.2f);
            std::snprintf(name, sizeof(name), "%.0f fps the game pulls the look 16 %%/tick (bounded fight, still target): no "
                          "trip", fps);
            Check(!r2.tripped && r2.maxErr > 30.0f, name, r2.maxErr);
            Graph g3;
            g3.ignore = true; // a dead graph with a still target: constant error (harmless): no trip
            const Result r3 = Run(g3, s, 0.2f);
            std::snprintf(name, sizeof(name), "%.0f fps graph ignores the delta, still target (constant error): no trip", fps);
            Check(!r3.tripped, name, r3.maxErr);
            // the pull starts while the look is on the target (V faces 150 deg away): the error RISES once from 0 to its
            // bounded level (~65 deg) and stays - one growing window, not a divergence; also while V aligns at 200 deg/s
            Scenario s4 = s;
            s4.yaw0 = 150.0f;
            Graph g4;
            g4.pull = 0.16f;
            g4.yawIn = 150.0f;
            const Result r4 = Run(g4, s4, 0.2f);
            std::snprintf(name, sizeof(name), "%.0f fps the game starts pulling a settled look (error rises 0 -> bounded, "
                          "still target): no trip", fps);
            Check(!r4.tripped && r4.maxErr > 45.0f, name, r4.maxErr);
            s4.alignRate = 200.0f;
            s4.ahead = true;
            const Result r5 = Run(g4, s4, 0.2f);
            std::snprintf(name, sizeof(name), "%.0f fps the game pulls while V aligns at 200 deg/s: no trip", fps);
            Check(!r5.tripped && r5.maxErr > 30.0f, name, r5.maxErr);
        }
        // activation far from the target (first write = feedback only): converges, no trip, command <= kMaxStep; also
        // with the rubber band multiplier 0.5 (compensated: the command is divided by the yawMultiplier read back)
        for (int v = 0; v < 4; ++v)
        {
            const int d = v % 2;
            const float mult = v < 2 ? 1.0f : 0.5f;
            Controller c;
            Graph g;
            g.delay = d == 0 ? Delay::D0 : Delay::D1;
            g.mult = mult;
            const float dt = 1.0f / fps;
            bool trip = false;
            float maxCmd = 0.0f, errLate = 0.0f;
            const int n = static_cast<int>(3.0f * fps);
            const int late = 25; // 0.75^25: < 0.01 of 170 (delay 0); 26 * 0.5^25 (delay 1)
            for (int i = 0; i < n; ++i)
            {
                Inputs in;
                in.targetYaw = 170.0f;
                in.targetPitch = -60.0f;
                in.entityYaw = 0.0f;
                in.yawInput = g.yawIn;
                in.pitchInput = g.pitchIn;
                in.headingLocked = true;
                in.dt = dt;
                in.yawMult = in.pitchMult = mult;
                const Output o = c.Step(in);
                trip = trip || o.trippedNow;
                maxCmd = std::fmax(maxCmd, std::fmax(std::fabs(o.yawDelta), std::fabs(o.pitchDelta)));
                g.Apply(o.yawDelta, o.pitchDelta, dt);
                if (i >= late)
                    errLate = std::fmax(errLate, std::fmax(std::fabs(Wrap180(170.0f - g.yawIn)), std::fabs(-60.0f - g.pitchIn)));
            }
            std::snprintf(name, sizeof(name), "%.0f fps delay %d mult %.1f activation 170 deg / -60 deg away: converges (< 0.5 "
                          "deg after %d ticks), no trip, command <= %.0f", fps, d, static_cast<double>(mult), late,
                          static_cast<double>(kMaxStep));
            Check(!trip && errLate < 0.5f && maxCmd <= kMaxStep + 1e-4f, name, errLate);
        }
    }
}

// ---- exe sites
struct Sec
{
    uint32_t va = 0, vsize = 0, raw = 0, rawSize = 0;
};
bool LoadImage(const char* aPath, std::vector<uint8_t>& aMem, uint32_t& aTextVa, uint32_t& aTextSize)
{
    std::ifstream in(aPath, std::ios::binary);
    if (!in)
        return false;
    std::vector<uint8_t> f((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (f.size() < 0x400 || f[0] != 'M' || f[1] != 'Z')
        return false;
    auto rd32 = [&](size_t o) { uint32_t v = 0; if (o + 4 <= f.size()) std::memcpy(&v, f.data() + o, 4); return v; };
    auto rd16 = [&](size_t o) { uint16_t v = 0; if (o + 2 <= f.size()) std::memcpy(&v, f.data() + o, 2); return v; };
    const uint32_t pe = rd32(0x3C);
    if (rd32(pe) != 0x00004550)
        return false;
    const uint16_t ns = rd16(pe + 6);
    const uint16_t optSize = rd16(pe + 20);
    aMem.assign(rd32(pe + 24 + 56), 0);
    aTextVa = aTextSize = 0;
    for (uint16_t i = 0; i < ns; ++i)
    {
        const size_t o = pe + 24 + optSize + 40u * i;
        char name[9] = {};
        std::memcpy(name, f.data() + o, 8);
        Sec s;
        s.vsize = rd32(o + 8);
        s.va = rd32(o + 12);
        s.rawSize = rd32(o + 16);
        s.raw = rd32(o + 20);
        size_t n = s.rawSize < s.vsize ? s.rawSize : s.vsize;
        if (s.raw + n > f.size())
            n = f.size() - s.raw;
        if (s.va + n <= aMem.size())
            std::memcpy(aMem.data() + s.va, f.data() + s.raw, n);
        if (std::strcmp(name, ".text") == 0)
        {
            aTextVa = s.va;
            aTextSize = s.vsize;
        }
    }
    return aTextSize != 0;
}

int CountIn(const std::vector<uint8_t>& aMem, uint32_t aVa, uint32_t aSize, const uint8_t* aPat, size_t aLen)
{
    int n = 0;
    for (uint32_t i = aVa; i + aLen <= aVa + aSize; ++i)
        if (aMem[i] == aPat[0] && std::memcmp(aMem.data() + i, aPat, aLen) == 0)
            ++n;
    return n;
}

int32_t Rel32(const std::vector<uint8_t>& aMem, uint32_t aRva)
{
    int32_t v = 0;
    std::memcpy(&v, aMem.data() + aRva, 4);
    return v;
}

void SiteTests(const char* aExe)
{
    std::vector<uint8_t> mem;
    uint32_t tva = 0, tsz = 0;
    if (!LoadImage(aExe, mem, tva, tsz))
    {
        Check(false, "exe copy loads (PE, .text)");
        return;
    }
    struct S
    {
        uint32_t rva;
        const uint8_t* bytes;
        size_t len;
        const char* name;
        bool unique;
    };
    const S sites[] = {
        {kRvaApply, kApplyPrologue, sizeof(kApplyPrologue), "apply prologue @0x39CD5C (unique in .text)", true},
        {kRvaCallSite, kCallSite, sizeof(kCallSite), "apply call site @0x39CBD7 (unique in .text)", true},
        {kRvaRefresh, kRefreshSite, sizeof(kRefreshSite), "YawInput/PitchInput refresh @0x39CAD2 (unique in .text)", true},
        {kRvaReadVars, kReadVarsPrologue, sizeof(kReadVarsPrologue), "graph variable read @0x5FD7CC", false},
        {kRvaEntityRot, kEntityRotHead, sizeof(kEntityRotHead), "entity orientation head @0x7C0C28", false},
        {kRvaEntityRot + 0x12, kEntityRotRead, sizeof(kEntityRotRead), "entity orientation +0xF0 read @0x7C0C3A", false},
    };
    for (const S& s : sites)
    {
        const bool at = s.rva + s.len <= mem.size() && std::memcmp(mem.data() + s.rva, s.bytes, s.len) == 0;
        Check(at, s.name);
        if (s.unique)
            Check(CountIn(mem, tva, tsz, s.bytes, s.len) == 1, s.name);
    }
    // CALL rel32 at kRvaCallSite + 12 -> apply; CALL at kRvaRefresh + 14 -> graph variable read
    const uint32_t callAt = kRvaCallSite + 12;
    Check(mem[callAt] == 0xE8 && callAt + 5 + Rel32(mem, callAt + 1) == kRvaApply, "apply call site targets 0x39CD5C");
    const uint32_t refAt = kRvaRefresh + 14;
    Check(mem[refAt] == 0xE8 && refAt + 5 + Rel32(mem, refAt + 1) == kRvaReadVars, "refresh call targets 0x5FD7CC");
    // the apply prologue reads +0x4B8 (unlock blend gate) - same function as decompiled
    Check(std::memcmp(kApplyPrologue + 45, "\x0F\x2F\xB9\xB8\x04\x00\x00", 7) == 0, "apply prologue contains COMISS [RCX+0x4B8]");
    // round 3: RotateEntityToCamera and its look read (the guard's fail-closed install checks)
    const S rot[] = {
        {kRvaRotate, kRotateHead, sizeof(kRotateHead), "RotateEntityToCamera head @0x7E60E0 (unique in .text)", true},
        {kRvaRotateReadSite, kRotateReadSite, sizeof(kRotateReadSite), "rotate look read call @0x7E6130", false},
        {kRvaRotateUse, kRotateUse, sizeof(kRotateUse), "rotate YawInput use @0x7E615E", false},
        {kRvaRotateBlend, kRotateBlend, sizeof(kRotateBlend), "rotate unlock blend 0.3 @0x7E6315 (unique in .text)", true},
        {kRvaLookVars, kLookVarsHead, sizeof(kLookVarsHead), "graph look read head @0x5FD84C", false},
        {kRvaLookVarsYaw, kLookVarsYaw, sizeof(kLookVarsYaw), "graph look read YawInput -> out+8 @0x5FD8D2", false},
    };
    for (const S& s : rot)
    {
        const bool at = s.rva + s.len <= mem.size() && std::memcmp(mem.data() + s.rva, s.bytes, s.len) == 0;
        Check(at, s.name);
        if (s.unique)
            Check(CountIn(mem, tva, tsz, s.bytes, s.len) == 1, s.name);
    }
    const uint32_t rotCall = kRvaRotateReadSite + sizeof(kRotateReadSite) - 1;
    Check(mem[rotCall] == 0xE8 && rotCall + 5 + Rel32(mem, rotCall + 1) == kRvaLookVars,
          "rotate look read call targets 0x5FD84C");
    Check(kRvaRotateReadRet == 0x7E6141 && kRvaRotateReadRet == rotCall + 5, "rotate look read return address 0x7E6141");
    // the read's out pointer is [rbp-0x19] and the YawInput used is [rbp-0x11] = out + kLookVarsYawOffset
    Check(kRotateReadSite[3] == 0xE7 && kRotateUse[4] == 0xEF && 0xEF - 0xE7 == static_cast<int>(kLookVarsYawOffset),
          "rotate: the YawInput it uses is out+8");
    // the blend store is inside the rotate function (between the head and the next function)
    Check(kRvaRotateBlend > kRvaRotate && kRvaRotateBlend - kRvaRotate == 0x235, "rotate: blend store at +0x235");
}
} // namespace

// 2026-10-28 vehicle edge zeroing (GameLookCore.hpp VehicleEdge / EdgeZero). Swivel model: after the vehicle edge the
// game's on-foot controller turns V onto its look at up to kAlignRate deg/s, keeping the world look (YawInput -= turn);
// in game the feed yaw went +10.94 -> -8.32 in 0.20 s with YawInput about -19.3.
constexpr float kAlignRate = 100.0f;

struct EdgeRun
{
    float maxDrift = 0.0f;   // max |entity yaw - edge yaw| (deg)
    float finalDrift = 0.0f; // at the end of the run
    float maxAbsYawAfter3 = 0.0f; // max |YawInput| read from the 4th window tick to the window end
    float yawAtEnd = 0.0f;   // YawInput read at the end of the window
    int writes = 0, corrections = 0;
};

// aZero: zeroing on; aLatency: ticks of the window that pass through before the arm reaches the hook
EdgeRun RunEdge(Delay aDelay, bool aZero, float aYaw0 = -19.3f, float aFps = 60.0f, int aLatency = 0, float aMult = 1.0f)
{
    Graph g;
    g.delay = aDelay;
    g.mult = aMult;
    g.yawIn = aYaw0;
    EdgeZero z;
    z.Reset();
    float ent = 0.0f;
    const float dt = 1.0f / aFps;
    const int windowTicks = static_cast<int>(kEdgeZeroSeconds * aFps);
    const int total = static_cast<int>(kEdgeReportSeconds * aFps);
    EdgeRun r;
    for (int n = 0; n < total; ++n)
    {
        float u = 0.0f; // the player's own mouse yaw delta (none here)
        const float y = g.ReadYaw();
        if (aZero && n >= aLatency && n < windowTicks)
        {
            const EdgeZeroOut o = z.Step(y, false, aMult);
            if (o.write)
                u = o.yawDelta;
            if (n >= aLatency + 3)
                r.maxAbsYawAfter3 = std::fmax(r.maxAbsYawAfter3, std::fabs(y));
            if (n == windowTicks - 1)
                r.yawAtEnd = y;
        }
        g.Apply(u, 0.0f, dt);
        // the game's on-foot alignment: V turns onto the look, the world look is kept
        const float turn = ClampAbs(g.yawIn, kAlignRate * dt);
        ent += turn;
        g.yawIn -= turn;
        r.maxDrift = std::fmax(r.maxDrift, std::fabs(ent));
    }
    r.finalDrift = std::fabs(ent);
    r.writes = z.writes;
    r.corrections = z.corrections;
    return r;
}

// Round 2 model of the in-game unmount (log 2026-10-28 23:04): the exit clip runs aExitLen s with V slowly turning
// (exit animation, the world look kept: YawInput -= turn); kUnmountLead s before the vehicle edge the game calls
// RotateEntityToCamera: it reads YawInput, turns V by it over 0.2 s and blends YawInput to 0 over 0.3 s, during which the
// camera update skips the delta apply (no hook call). Returns |V's turn from the call| (the swivel).
constexpr float kUnmountLead = 0.13f;

float RunExit(Delay aDelay, bool aExitZero, float aFps, float aYaw0 = 18.2f, float aExitTurnRate = 2.0f,
              float aExitLen = 1.97f)
{
    Graph g;
    g.delay = aDelay;
    g.yawIn = aYaw0;
    EdgeZero z;
    z.Reset();
    const float dt = 1.0f / aFps;
    float swivel = 0.0f, swivelLeft = 0.0f, swivelRate = 0.0f, blendLeft = 0.0f;
    bool called = false;
    const int ticks = static_cast<int>((aExitLen + kEdgeReportSeconds) * aFps);
    for (int n = 0; n < ticks; ++n)
    {
        const float t = static_cast<float>(n) * dt;
        const double exitAge = aExitZero && t <= aExitLen ? static_cast<double>(t) : -1.0;
        const double edgeAge = t >= aExitLen ? static_cast<double>(t - aExitLen) : -1.0;
        if (!called && t >= aExitLen - kUnmountLead)
        {
            called = true;
            swivelLeft = g.yawIn; // RotateEntityToCamera: target = entity yaw + YawInput
            swivelRate = std::fabs(swivelLeft) / 0.2f;
            blendLeft = 0.3f;
            g.q.clear();
        }
        if (blendLeft > 0.0f)
        {
            // unlock blend: no camera update (no hook call), YawInput blended to 0
            blendLeft -= dt;
            g.yawIn *= blendLeft > 0.0f ? 0.8f : 0.0f;
        }
        else
        {
            float u = 0.0f;
            if (EdgeZeroWanted(exitAge, edgeAge))
            {
                const EdgeZeroOut o = z.Step(g.ReadYaw(), false, 1.0f);
                if (o.write)
                    u = o.yawDelta;
            }
            g.Apply(u, 0.0f, dt);
        }
        if (swivelLeft != 0.0f)
        {
            const float step = ClampAbs(swivelLeft, swivelRate * dt);
            swivel += step;
            swivelLeft -= step;
        }
        if (t < aExitLen - kUnmountLead)
            g.yawIn -= aExitTurnRate * dt; // the exit clip turns V, the world look is kept
    }
    return std::fabs(swivel);
}

// Round 3 model of the in-game unmount (log 2026-10-29 00:29): the heading is LOCKED through the exit - the look holds
// the vehicle heading while the exit clip turns V by aClipTurn; at the unlock (kUnmountLead s before the edge)
// RotateEntityToCamera reads YawInput = -aClipTurn (through the guard) and turns V by what it read over 0.2 s; its 0.3 s
// unlock blend skips the camera update; afterwards the edge window zeroes the graph's YawInput (delay 1 graph).
struct LockedExit
{
    float swivel = 0.0f;  // |V's turn caused by the unmount call|
    float yawAtEnd = 0.0f; // graph YawInput at the end of the edge window
};

LockedExit RunLockedExit(bool aGuard, float aClipTurn = 19.0f, float aFps = 60.0f)
{
    const float exitLen = 1.99f;
    const float dt = 1.0f / aFps;
    Graph g;
    g.delay = Delay::D1;
    EdgeZero z;
    z.Reset();
    LockedExit out;
    float swivelLeft = 0.0f, swivelRate = 0.0f, blendLeft = 0.0f;
    bool called = false;
    const int ticks = static_cast<int>((exitLen + kEdgeReportSeconds) * aFps);
    for (int n = 0; n < ticks; ++n)
    {
        const float t = static_cast<float>(n) * dt;
        const double exitAge = t <= exitLen ? static_cast<double>(t) : -1.0;
        const double edgeAge = t >= exitLen ? static_cast<double>(t - exitLen) : -1.0;
        if (!called && t >= exitLen - kUnmountLead)
        {
            called = true;
            g.yawIn = -aClipTurn; // the unlock: the held look relative to V after the clip turn
            float read = g.yawIn;
            if (aGuard && RotateZeroGate(true, exitAge, edgeAge) && RotateYawZeroable(read))
                read = 0.0f;
            swivelLeft = read;
            swivelRate = std::fabs(read) / 0.2f;
            blendLeft = 0.3f;
        }
        if (blendLeft > 0.0f)
            blendLeft -= dt; // camera update skipped
        else if (called && EdgeZeroWanted(-1.0, edgeAge))
        {
            const EdgeZeroOut o = z.Step(g.ReadYaw(), false, 1.0f);
            g.Apply(o.write ? o.yawDelta : 0.0f, 0.0f, dt);
            out.yawAtEnd = g.yawIn;
        }
        if (swivelLeft != 0.0f)
        {
            const float step = ClampAbs(swivelLeft, swivelRate * dt);
            out.swivel += step;
            swivelLeft -= step;
        }
    }
    out.swivel = std::fabs(out.swivel);
    return out;
}

void EdgeZeroTests()
{
    // round 3: the rotate guard gate
    {
        Check(!RotateZeroGate(false, 1.86, -1.0) && !RotateZeroGate(false, -1.0, 0.1),
              "rotate guard: never for another component");
        Check(RotateZeroGate(true, 1.86, -1.0) && RotateZeroGate(true, 0.1, -1.0) && RotateZeroGate(true, -1.0, 0.1),
              "rotate guard: in the exit phase (also before kExitZeroDelay) and in the edge window");
        Check(!RotateZeroGate(true, -1.0, -1.0) && !RotateZeroGate(true, kExitMaxSeconds + 0.1, -1.0) &&
                  !RotateZeroGate(true, -1.0, kEdgeZeroSeconds + 0.01),
              "rotate guard: not without an exit phase / edge window (on foot, first person exits, expired)");
        Check(RotateYawZeroable(-18.2f) && RotateYawZeroable(0.0f) && !RotateYawZeroable(kSentinel) &&
                  !RotateYawZeroable(std::nanf("")),
              "rotate guard: only a real YawInput is replaced (999 / NaN kept)");
    }
    // round 3: the locked-exit model of the in-game failure and the guard
    {
        const LockedExit off = RunLockedExit(false);
        Check(off.swivel > 18.0f, "locked exit model: no guard -> V turned back by the clip turn (in game 17.6 / 16.6)",
              off.swivel);
        const LockedExit on = RunLockedExit(true);
        Check(on.swivel < 0.01f && std::fabs(on.yawAtEnd) < 0.01f,
              "locked exit model: guard -> no swivel, YawInput 0 by the end of the edge window", on.swivel);
        const LockedExit on30 = RunLockedExit(true, -19.0f, 30.0f);
        Check(on30.swivel < 0.01f && std::fabs(on30.yawAtEnd) < 0.01f, "locked exit model, 30 fps, other side: guard holds",
              on30.swivel);
    }
    // round 2: window decision and the third person gate of the exit start
    {
        Check(!EdgeZeroWanted(0.0, -1.0) && !EdgeZeroWanted(kExitZeroDelay - 0.01, -1.0) &&
                  EdgeZeroWanted(kExitZeroDelay, -1.0) && EdgeZeroWanted(1.9, -1.0) &&
                  EdgeZeroWanted(kExitMaxSeconds, -1.0) && !EdgeZeroWanted(kExitMaxSeconds + 0.01, -1.0),
              "exit phase: zeroing from kExitZeroDelay after the exit start up to kExitMaxSeconds");
        Check(EdgeZeroWanted(-1.0, 0.0) && EdgeZeroWanted(-1.0, kEdgeZeroSeconds) &&
                  !EdgeZeroWanted(-1.0, kEdgeZeroSeconds + 0.01) && !EdgeZeroWanted(-1.0, -1.0),
              "edge window: zeroing for kEdgeZeroSeconds, nothing when not armed");
        Check(ExitZeroAllowed(true, 1) && !ExitZeroAllowed(true, 0) && !ExitZeroAllowed(true, -1) &&
                  !ExitZeroAllowed(false, 1),
              "exit start: only UTPP third person with a third person ride camera");
    }
    // round 2: the in-game failure and its fix in the unmount model
    {
        const float late = RunExit(Delay::D0, false, 60.0f, 18.2f, 0.0f);
        Check(late > 17.0f, "exit model: zeroing armed only at the edge -> V swivels (in game 17.2 / 16.9 deg)", late);
        const Delay delays[] = {Delay::D0, Delay::D1, Delay::Random};
        const float fpsList[] = {30.0f, 60.0f, 144.0f};
        for (Delay d : delays)
            for (float fps : fpsList)
            {
                const float sw = RunExit(d, true, fps);
                char name[160];
                std::snprintf(name, sizeof(name), "exit model, %s, %.0f fps: zeroing through the exit -> swivel < 0.5 deg",
                              DelayName(d), static_cast<double>(fps));
                Check(sw < 0.5f, name, sw);
            }
        // quick exit (unmount ~1.0 s after the exit start): still zeroed before the unmount call
        const float quick = RunExit(Delay::D1, true, 60.0f, 18.2f, 2.0f, 1.0f);
        Check(quick < 0.5f, "exit model, quick exit (1.0 s), delay 1: swivel < 0.5 deg", quick);
        // a fast exit turn leaves at most ~3 ticks of it (correction every other tick, graph delay 1)
        const float fast = RunExit(Delay::D1, true, 60.0f, -18.3f, 30.0f);
        Check(fast <= 3.0f * 30.0f / 60.0f + 0.05f, "exit model, delay 1, exit clip turning V at 30 deg/s: swivel <= 3 ticks of it",
              fast);
    }

    // edge detection: one-shot per vehicle episode, third person only, cancel on the vehicle bit
    {
        VehicleEdge e;
        Check(e.Feed(false, true) == EdgeEvent::None && e.Feed(false, true) == EdgeEvent::None,
              "edge: no vehicle feed before -> no arm");
        Check(e.Feed(true, true) == EdgeEvent::Cancel && e.Feed(true, true) == EdgeEvent::Cancel,
              "edge: vehicle feeds -> cancel event (no window)");
        Check(e.Feed(false, true) == EdgeEvent::Arm, "edge: first on-foot feed after a vehicle feed in TPP -> arm");
        Check(e.Feed(false, true) == EdgeEvent::None, "edge: one-shot (the next on-foot feed does not re-arm)");
        Check(e.Feed(true, false) == EdgeEvent::Cancel && e.Feed(false, false) == EdgeEvent::NoTpp &&
                  e.Feed(false, true) == EdgeEvent::None,
              "edge: first person edge -> no arm, consumed");
        // a ride with a feed gap: one vehicle feed, then the next feed is on foot
        VehicleEdge e2;
        e2.Feed(true, true);
        Check(e2.Feed(false, true) == EdgeEvent::Arm, "edge: single vehicle feed (feed gap during the ride) -> arm");
    }
    // command: correction = -YawInput, then a hold tick (0), then the next correction
    {
        EdgeZero z;
        z.Reset();
        const EdgeZeroOut a = z.Step(-19.3f, false, 1.0f);
        Check(a.write && a.correction && std::fabs(a.yawDelta - 19.3f) < 1e-4f, "edge zero: first tick corrects -YawInput",
              a.yawDelta);
        const EdgeZeroOut b = z.Step(-19.3f, false, 1.0f);
        Check(b.write && !b.correction && b.yawDelta == 0.0f, "edge zero: second tick holds (0, the correction in flight)",
              b.yawDelta);
        const EdgeZeroOut c = z.Step(0.4f, false, 1.0f);
        Check(c.write && c.correction && std::fabs(c.yawDelta + 0.4f) < 1e-5f, "edge zero: third tick corrects the rest",
              c.yawDelta);
        Check(z.writes == 3 && z.corrections == 2 && std::fabs(z.firstYawIn + 19.3f) < 1e-5f, "edge zero: counters");
    }
    // round 2: a locked heading is written too (YawInput 0 = look along V either way), counted only
    {
        EdgeZero z;
        z.Reset();
        const EdgeZeroOut l = z.Step(-19.0f, true, 1.0f);
        Check(l.write && l.correction && std::fabs(l.yawDelta - 19.0f) < 1e-4f && z.locked == 1,
              "edge zero: heading locked -> still corrected (counted)", l.yawDelta);
    }
    // skips: sentinel / NaN pass through and keep the window; the first written tick after them corrects
    {
        EdgeZero z;
        z.Reset();
        const EdgeZeroOut s = z.Step(kSentinel, false, 1.0f);
        const EdgeZeroOut f = z.Step(std::nanf(""), false, 1.0f);
        Check(!s.write && s.skip == EdgeSkip::Sentinel && !f.write && f.skip == EdgeSkip::NonFinite,
              "edge zero: sentinel / NaN -> no write");
        const EdgeZeroOut a = z.Step(-19.0f, false, 1.0f);
        Check(a.write && a.correction && std::fabs(a.yawDelta - 19.0f) < 1e-4f && z.sentinel == 1 && z.nonFinite == 1,
              "edge zero: first written tick after skips corrects", a.yawDelta);
        // a skip right after a correction counts as the in-flight tick: the next tick corrects again
        const EdgeZeroOut s2 = z.Step(kSentinel, false, 1.0f);
        const EdgeZeroOut b = z.Step(0.5f, false, 1.0f);
        Check(!s2.write && b.write && b.correction, "edge zero: a skipped tick covers the in-flight tick");
    }
    // wrap and rubber band compensation
    {
        EdgeZero z;
        z.Reset();
        const EdgeZeroOut a = z.Step(190.0f, false, 1.0f);
        Check(std::fabs(a.yawDelta - 170.0f) < 1e-3f, "edge zero: YawInput 190 -> wrapped delta +170", a.yawDelta);
        EdgeZero z2;
        z2.Reset();
        const EdgeZeroOut b = z2.Step(-10.0f, false, 0.5f);
        Check(std::fabs(b.yawDelta - 20.0f) < 1e-4f, "edge zero: yawMultiplier 0.5 compensated", b.yawDelta);
        EdgeZero z3;
        z3.Reset();
        const EdgeZeroOut c = z3.Step(-10.0f, false, 0.1f);
        Check(std::fabs(c.yawDelta - 10.0f) < 1e-4f, "edge zero: multiplier below kMultMin not compensated", c.yawDelta);
    }
    // closed loop against the swivel model
    {
        const EdgeRun base = RunEdge(Delay::D0, false);
        Check(base.finalDrift > 19.0f, "edge model: without zeroing V swivels onto the look (19.3 deg)", base.finalDrift);
        const float tick = kAlignRate / 60.0f;
        const EdgeRun d0 = RunEdge(Delay::D0, true);
        Check(d0.maxDrift < 0.01f && d0.maxAbsYawAfter3 < 0.01f && std::fabs(d0.yawAtEnd) < 0.01f,
              "edge zero, delay 0: no swivel, YawInput 0", d0.maxDrift);
        const EdgeRun d1 = RunEdge(Delay::D1, true);
        Check(d1.maxDrift <= tick + 1e-3f && d1.finalDrift < 2.0f && d1.maxAbsYawAfter3 < 0.01f &&
                  std::fabs(d1.yawAtEnd) < 0.01f,
              "edge zero, delay 1: drift <= one tick of alignment, settled", d1.maxDrift);
        Check(d1.maxAbsYawAfter3 < 0.5f, "edge zero, delay 1: |YawInput| < 0.5 from the 4th tick", d1.maxAbsYawAfter3);
        for (int seed = 0; seed < 3; ++seed)
        {
            const EdgeRun dr = RunEdge(Delay::Random, true, -19.3f + 3.0f * static_cast<float>(seed));
            Check(dr.maxDrift <= tick + 1e-3f && dr.finalDrift < 2.0f && std::fabs(dr.yawAtEnd) < 0.5f,
                  "edge zero, delay 0/1 random: drift <= one tick of alignment, settled", dr.maxDrift);
        }
        const EdgeRun ds = RunEdge(Delay::Sample, true);
        Check(ds.finalDrift < 2.0f * tick + 1e-3f && std::fabs(ds.yawAtEnd) < 1.0f,
              "edge zero, sample graph (lost / doubled deltas): bounded, settled", ds.finalDrift);
        const EdgeRun lat = RunEdge(Delay::D0, true, -19.3f, 60.0f, 1);
        Check(lat.maxDrift <= tick + 1e-3f, "edge zero, one tick of arm latency: drift <= one tick of alignment",
              lat.maxDrift);
        const EdgeRun mult = RunEdge(Delay::D1, true, -19.3f, 60.0f, 0, 0.5f);
        Check(mult.maxDrift <= tick + 1e-3f && std::fabs(mult.yawAtEnd) < 0.01f,
              "edge zero, delay 1, yawMultiplier 0.5: drift <= one tick, settled", mult.maxDrift);
        const EdgeRun hi = RunEdge(Delay::D1, true, -19.3f, 144.0f);
        Check(hi.maxDrift <= kAlignRate / 144.0f + 1e-3f && std::fabs(hi.yawAtEnd) < 0.01f,
              "edge zero, delay 1 at 144 fps: drift <= one tick", hi.maxDrift);
        const EdgeRun lo = RunEdge(Delay::D1, true, -19.3f, 30.0f);
        Check(lo.maxDrift <= kAlignRate / 30.0f + 1e-3f && std::fabs(lo.yawAtEnd) < 0.01f,
              "edge zero, delay 1 at 30 fps: drift <= one tick", lo.maxDrift);
        Check(d0.writes == static_cast<int>(kEdgeZeroSeconds * 60.0f) && d0.corrections == (d0.writes + 1) / 2,
              "edge zero: every window tick written, every other one a correction", d0.writes);
    }
}

int main(int argc, char** argv)
{
    // convention: a rotation of +90 deg about Z (CCW) turns forward (0,1,0) to (-1,0,0): yaw +90 = atan2(-x, y)
    {
        const float h = 0.5f * 90.0f / kRadToDeg;
        Check(std::fabs(YawFromQuat(0, 0, std::sin(h), std::cos(h)) - 90.0f) < 1e-3f, "YawFromQuat(+90 about Z) = 90");
        const float h2 = 0.5f * -135.0f / kRadToDeg;
        Check(std::fabs(YawFromQuat(0, 0, std::sin(h2), std::cos(h2)) + 135.0f) < 1e-3f, "YawFromQuat(-135) = -135");
        Check(std::fabs(YawFromQuat(0, 0, 0, 1)) < 1e-6f, "identity -> yaw 0");
        // pitched + yawed (q = yaw * pitch): yaw unaffected by the pitch
        const float hy = 0.5f * 30.0f / kRadToDeg, hp = 0.5f * 20.0f / kRadToDeg;
        const float yw = std::cos(hy), yz = std::sin(hy), px = std::sin(hp), pw = std::cos(hp);
        // q = (0,0,yz,yw) * (px,0,0,pw)
        const float qx = yw * px, qy = yz * px, qz = yz * pw, qw = yw * pw;
        Check(std::fabs(YawFromQuat(qx, qy, qz, qw) - 30.0f) < 1e-3f, "YawFromQuat(yaw 30 * pitch 20) = 30",
              YawFromQuat(qx, qy, qz, qw));
    }
    Check(std::fabs(Wrap180(190.0f) + 170.0f) < 1e-4f && std::fabs(Wrap180(-190.0f) - 170.0f) < 1e-4f &&
              std::fabs(Wrap180(720.5f) - 0.5f) < 1e-4f,
          "Wrap180");
    // poles of the two closed loops (documented in GameLookCore.hpp): delay 0 -> 1 - k; delay 1 -> z^2 - z + k = 0
    {
        const float disc = 1.0f - 4.0f * kGain;
        Check(kGain > 0.0f && kGain <= 0.3f, "gain k in (0, 0.3]", kGain);
        Check(std::fabs(1.0f - kGain) < 1.0f && disc >= -1e-6f && 0.5f * (1.0f + std::sqrt(std::fmax(disc, 0.0f))) < 0.75f,
              "poles: delay 0 |1 - k| < 1, delay 1 real (critically damped) and <= 0.75", disc);
    }
    // first write: feedback only (no feed-forward); desired = wrap(target - entity), pitch clamped
    {
        Controller c;
        Inputs in;
        in.targetYaw = 179.0f;
        in.entityYaw = -179.0f;
        in.yawInput = 170.0f;
        in.targetPitch = 85.0f;
        in.pitchInput = 10.0f;
        in.pitchMin = -80.0f;
        in.pitchMax = 80.0f;
        in.headingLocked = true;
        const Output o = c.Step(in);
        Check(o.write && std::fabs(o.desiredYaw + 2.0f) < 1e-4f, "desired local yaw = wrap(179 - -179) = -2", o.desiredYaw);
        Check(std::fabs(o.ffYaw) < 1e-6f && std::fabs(o.errYaw + 172.0f) < 1e-4f &&
                  std::fabs(o.yawDelta - kGain * -172.0f) < 1e-4f,
              "first write: no feed-forward, delta = k * wrap(-2 - 170) = k * -172", o.yawDelta);
        Check(std::fabs(o.desiredPitch - 80.0f) < 1e-6f && std::fabs(o.pitchDelta - kGain * 70.0f) < 1e-4f,
              "pitch clamped to pitchMax 80, first delta = k * 70", o.pitchDelta);
        Check(!o.ovr && !o.release, "first write: no override, no release");
        // second tick (delay 0: the graph applied the first delta), target +3 deg, pitch target unchanged: delta = the
        // motion (feed-forward) + k * the remaining error of the last tick
        Inputs in2 = in;
        in2.targetYaw = Wrap180(179.0f + 3.0f);
        in2.yawInput = Wrap180(170.0f + o.yawDelta);
        in2.pitchInput = 10.0f + o.pitchDelta;
        const Output o2 = c.Step(in2);
        const float rest = Wrap180(-2.0f - in2.yawInput);
        Check(std::fabs(o2.ffYaw - 3.0f) < 1e-4f && std::fabs(o2.errYaw - rest) < 1e-3f &&
                  std::fabs(o2.yawDelta - (3.0f + kGain * rest)) < 1e-3f,
              "second write: delta = feed-forward 3 + k * (last reference - read-back)", o2.yawDelta);
    }
    // round 2: kMaxStep 180 - a 170 deg jump of the target goes out in one tick (round 1: 90 + 80)
    {
        Controller c;
        Graph g;
        float target = 0.0f;
        bool tripped = false;
        float maxCmd = 0.0f, errAt51 = 1e9f;
        for (int t = 0; t < 200; ++t)
        {
            if (t == 50)
                target = 170.0f;
            Inputs in;
            in.targetYaw = target;
            in.yawInput = g.yawIn;
            in.pitchInput = g.pitchIn;
            in.headingLocked = true;
            const Output o = c.Step(in);
            tripped = tripped || o.trippedNow;
            maxCmd = std::fmax(maxCmd, std::fabs(o.yawDelta));
            if (t == 50)
                Check(std::fabs(o.yawDelta - 170.0f) < 1e-3f, "170 deg flick: the whole 170 in one tick", o.yawDelta);
            if (t == 51)
                errAt51 = std::fabs(o.errYaw);
            g.Apply(o.write ? o.yawDelta : 0.0f, o.write ? o.pitchDelta : 0.0f);
        }
        Check(errAt51 < 1e-3f, "170 deg flick: no error one tick later (delay 0)", errAt51);
        Check(!tripped && maxCmd <= kMaxStep + 1e-4f && std::fabs(Wrap180(g.yawIn - 170.0f)) < 1e-3f,
              "170 deg flick: no trip, command <= 180, reaches the target");
    }
    // ahead: the entity turn requested for this tick is taken out of the wanted YawInput; the order residuals
    {
        Controller c;
        Inputs in;
        in.targetYaw = 50.0f;
        in.entityYaw = 20.0f;
        in.yawInput = 30.0f;
        in.headingLocked = true;
        in.aheadYaw = 12.0f;
        Output o = c.Step(in);
        Check(std::fabs(o.desiredYaw - 18.0f) < 1e-4f && std::fabs(o.aheadEff - 12.0f) < 1e-6f,
              "ahead 12 (order after): desired YawInput = 50 - (20 + 12) = 18", o.desiredYaw);
        in.entityYaw = 32.0f; // the entity turned exactly the announced 12 deg
        in.yawInput = 30.0f + o.yawDelta;
        in.aheadYaw = 0.0f;
        o = c.Step(in);
        Check(o.ovr, "the tick after an ahead is an override tick (read-back not trusted)");
        Check(c.aheadN == 1 && c.aheadDiscN == 1 && std::fabs(static_cast<float>(c.resAfter)) < 1e-4f &&
                  std::fabs(static_cast<float>(c.resBefore) - 12.0f) < 1e-4f &&
                  std::fabs(static_cast<float>(c.resLate) - 12.0f) < 1e-4f &&
                  std::fabs(static_cast<float>(c.aheadNoFeed) - 12.0f) < 1e-4f,
              "ahead residuals: after 0, before 12, late 12, no-feed 12", c.resAfter);
        in.aheadYaw = NAN;
        Check(c.Step(in).skip == Skip::NonFinite, "NaN ahead -> no write");
    }
    // override: the read-back is replaced by the prediction (last estimate + what the graph integrated, NoteApplied);
    // the first clean tick corrects the whole error (release), the next tick has no feedback
    {
        Controller c;
        Inputs in;
        in.targetYaw = 40.0f;
        in.entityYaw = 0.0f;
        in.yawInput = 40.0f; // settled
        in.targetPitch = 10.0f;
        in.pitchInput = 10.0f;
        in.headingLocked = true;
        c.Step(in);
        in.aheadYaw = 5.0f; // V turns 5 deg after this camera update
        Output o = c.Step(in);
        Check(o.ovr && std::fabs(o.yawDelta + 5.0f) < 1e-4f,
              "ahead 5 on a settled look: override tick (prediction = the read-back here), delta -5 (the look stays)",
              o.yawDelta);
        c.NoteApplied(-4.0f, 0.0f); // the game's apply clamped the delta to -4
        in.entityYaw = 5.0f;
        in.aheadYaw = 0.0f;
        in.yawInput = 0.0f; // overridden read-back (the game shows V's yaw)
        in.pitchInput = 0.0f;
        o = c.Step(in);
        Check(o.ovr && std::fabs(o.errYaw - -1.0f) < 1e-4f && std::fabs(o.rawErrYaw - 35.0f) < 1e-4f &&
                  std::fabs(o.errPitch) < 1e-4f,
              "override tick: error from the prediction 40 - 4 = 36 (wanted 35: -1), not from the read-back 0 (raw 35)",
              o.errYaw);
        // two more ticks without ahead pass kOvrHold at 60 fps: then a release on the real read-back
        in.yawInput = 20.0f;
        in.pitchInput = 4.0f;
        in.dt = 1.0f / 60.0f;
        o = c.Step(in); // a2 != 0 -> still override
        Check(o.ovr, "two ticks after the ahead: still an override tick");
        o = c.Step(in);
        Check(!o.ovr && o.release && std::fabs(o.errYaw - 15.0f) < 1e-3f && std::fabs(o.yawDelta - 15.0f) < 1e-3f &&
                  std::fabs(o.pitchDelta - 6.0f) < 1e-3f,
              "release: the whole error (yaw 35 - 20, pitch 10 - 4) corrected at once", o.yawDelta);
        in.yawInput = 26.0f; // delay 1: the release correction has not arrived yet
        in.pitchInput = 4.0f;
        o = c.Step(in);
        Check(!o.release && std::fabs(o.yawDelta) < 1e-4f && std::fabs(o.pitchDelta) < 1e-4f,
              "the tick after a release: no feedback (exact for delay 0 and delay 1)", o.yawDelta);
        Check(c.releases == 1 && std::fabs(c.lastRelease - 15.0f) < 1e-3f, "release counted with its error", c.lastRelease);
    }
    // yawMultiplier compensation: the command is divided by the rubber band factor read back (0.2 < m < 1 only)
    {
        Controller a, b, cc;
        Inputs in;
        in.targetYaw = 30.0f;
        in.yawInput = 10.0f;
        in.headingLocked = true;
        in.yawMult = 0.5f;
        const Output oa = a.Step(in);
        Check(std::fabs(oa.yawDelta - kGain * 20.0f / 0.5f) < 1e-4f, "yawMult 0.5: command doubled", oa.yawDelta);
        in.yawMult = 0.1f;
        const Output ob = b.Step(in);
        Check(std::fabs(ob.yawDelta - kGain * 20.0f) < 1e-4f, "yawMult 0.1 (out of range): not compensated", ob.yawDelta);
        in.yawMult = NAN;
        const Output oc = cc.Step(in);
        Check(oc.write && std::fabs(oc.yawDelta - kGain * 20.0f) < 1e-4f, "yawMult NaN: ignored", oc.yawDelta);
        // moving with the rubber band 0.75 / 0.5: the steady error stays small (round 1: (1-m) ff / (m k))
        for (int v = 0; v < 2; ++v)
        {
            const float m = v == 0 ? 0.75f : 0.5f;
            Controller c;
            Graph g;
            g.mult = m;
            float ent = 0.0f, worst = 0.0f;
            for (int i = 0; i < 240; ++i)
            {
                Inputs x;
                x.targetYaw = Wrap180(static_cast<float>(i) * 3.0f);
                x.entityYaw = ent;
                x.yawInput = g.yawIn;
                x.headingLocked = true;
                x.dt = 1.0f / 60.0f;
                x.yawMult = m;
                const Output o = c.Step(x);
                g.Apply(o.yawDelta, o.pitchDelta);
                if (i > 30)
                    worst = std::fmax(worst, std::fabs(Wrap180(x.targetYaw - g.yawIn)));
            }
            char nm[120];
            std::snprintf(nm, sizeof(nm), "yawMult %.2f, target 180 deg/s: steady error < 0.5 deg", static_cast<double>(m));
            Check(worst < 0.5f, nm, worst);
        }
    }
    // the lag hint is a diagnostic only: the same inputs give the same command whatever the hint says
    {
        Controller a, b;
        b.lag = 1;
        Inputs in;
        in.targetYaw = 40.0f;
        in.entityYaw = 10.0f;
        in.yawInput = 5.0f;
        in.targetPitch = 12.0f;
        in.headingLocked = true;
        bool same = true;
        for (int i = 0; i < 20; ++i)
        {
            in.targetYaw += 2.5f;
            const Output oa = a.Step(in), ob = b.Step(in);
            same = same && oa.yawDelta == ob.yawDelta && oa.pitchDelta == ob.pitchDelta;
            in.yawInput = Wrap180(in.yawInput + oa.yawDelta);
            in.pitchInput += oa.pitchDelta;
        }
        Check(same, "lag hint never changes the command");
    }
    // skips
    {
        Controller c;
        Inputs in;
        in.headingLocked = false;
        Check(!c.Step(in).write && c.Step(in).skip == Skip::NotLocked, "heading not locked -> no write (vanilla input)");
        in.headingLocked = true;
        in.yawInput = kSentinel;
        Check(c.Step(in).skip == Skip::Sentinel, "YawInput 999 (graph variable missing) -> no write");
        in.yawInput = 0.0f;
        in.targetYaw = NAN;
        Check(c.Step(in).skip == Skip::NonFinite, "NaN target -> no write");
        in.targetYaw = 0.0f;
        Check(c.Step(in).write, "valid again -> write");
    }
    // a pause (ResetHistory) makes the next write a first write again: a target that jumped meanwhile is approached by
    // the feedback only (no feed-forward jump)
    {
        Controller c;
        Inputs in;
        in.targetYaw = 10.0f;
        in.headingLocked = true;
        c.Step(in);
        c.ResetHistory();
        in.targetYaw = 70.0f;
        const Output o = c.Step(in);
        Check(std::fabs(o.ffYaw) < 1e-6f && std::fabs(o.yawDelta - kGain * 70.0f) < 1e-4f,
              "after a pause: first write = feedback only", o.yawDelta);
    }
    // tripped -> no write until Rearm; the trip count survives Rearm
    {
        Controller c;
        c.tripped = true;
        c.tripCount = 2;
        Inputs in;
        in.headingLocked = true;
        Check(c.Step(in).skip == Skip::Tripped, "tripped -> no write until Rearm");
        c.Rearm();
        Check(c.Step(in).write && c.tripCount == 2, "Rearm -> write, trip count kept");
    }
    // rubber band multiplier 0.5: not reported to the controller (feature read failed) - converges with a still target;
    // reported (as in game) - bounded while moving and V turns (override ticks: prediction with the multiplier)
    {
        Scenario s;
        s.yaw0 = 60.0f;
        s.seconds = 3.0f;
        s.featureRead = false;
        Graph g;
        g.mult = 0.5f;
        const Result r = Run(g, s, 1.0f); // loop pole 1 - k * 0.5 = 0.875
        Check(!r.tripped && r.errAfter < 0.05f, "multiplier 0.5, still target (60 deg away at the start): converges",
              r.errAfter);
        Scenario s2;
        s2.yawRate = 180.0f;
        s2.yawDur = 1.5f;
        s2.turnRate = 120.0f;
        s2.ahead = true;
        s2.seconds = 3.0f;
        Graph g2;
        g2.mult = 0.5f;
        const Result r2 = Run(g2, s2, 0.5f);
        Check(!r2.tripped && r2.maxErr < 15.0f && r2.errAfter < 0.5f, "multiplier 0.5, moving: bounded, no trip, settles",
              r2.maxErr);
    }
    ClosedLoopTests();
    OverrideTests();
    AheadOrderTests();
    DivergenceTests();
    EdgeZeroTests();
    if (argc > 1)
        SiteTests(argv[1]);
    else
        std::printf("note: no exe copy given, code site checks skipped\n");
    std::printf("gamelook test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
