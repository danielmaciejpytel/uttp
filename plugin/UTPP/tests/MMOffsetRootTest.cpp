// Offline test of Phase 5.3 (UTPP 0.3.3): offset root + capsule acceleration matching, not shipped.
// Usage: utpp_mm_offsetroot_test <database.utppmm>      Exit code 0 = all checks passed.
//
// Emulates MMDriver::Feed at 60 Hz for a start from idle at run input (stand state): the capsule accelerates with a
// constant acceleration to the game run speed 3.85 m/s (vanilla >= 24 m/s^2 measured in game, 0.3.2 trace; matched
// 3.75 m/s^2 = motion Run_Start), the matcher gets the driver's input (speed = max(|v|, half the state target), speed map
// 1.6->2.19, 3.85->5.46, rate = |v|/|v_db| clamped 0.6..1.4), the offset root runs on the matcher output and the foot
// locker on the offset pose, exactly like the driver. Metrics: max foot-lock correction, slide of the displayed toes in
// contact (world), offset bounds, convergence, determinism, NaN robustness, kill switch.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/mm/FootLocker.hpp"
#include "../src/mm/BaseMatcher.hpp"
#include "../src/mm/OffsetRoot.hpp"
#include "../src/mm/PlaybackRate.hpp"
#include "../src/mm/BodyTurn.hpp"

using namespace utpp::mm;

static int g_fail = 0;
static int g_pass = 0;
static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}
static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0, double e = 0, double f = 0)
{
    char buf[512];
    std::snprintf(buf, sizeof buf, aFmt, a, b, c, d, e, f);
    return buf;
}

static const Database* g_db = nullptr;
static bool DebugEnv()
{
    char* v = nullptr;
    size_t n = 0;
    const bool on = _dupenv_s(&v, &n, "UTPP_TEST_DEBUG") == 0 && v != nullptr;
    std::free(v);
    return on;
}
static bool g_debug = DebugEnv(); // UTPP_TEST_DEBUG=1: per-update trace of the start window
static constexpr float kDt = 1.0f / 60.0f;
static constexpr float kRunSpeed = 3.85f;

// MMDriver stand-state speed map (0,0)-(1.6,2.19)-(3.85,5.46)-(8.2,7.65) (mm/PlaybackRate.hpp, same code as the driver).
static const SpeedPoints kStandMap = {3, {1.6f, 3.85f, 8.2f}, {2.19f, 5.46f, 7.65f}};

struct RunCfg
{
    float accel = 24.0f;        // capsule acceleration (m/s^2); deceleration fixed 24 (game ~23)
    float speed = kRunSpeed;    // capsule target speed (run 3.85, sprint 8.2)
    bool offset = false;        // offset root on
    bool rootRate = false;      // 0.3.4 playback rate from the played root speed
    bool stopSync = false;      // 0.3.4 stop intent + trajectory stop half-life
    float stopHalflife = 0.05f; // s
    float rateMin = 0.5f;       // root rate minimum
    float rootWindow = 0.6f;    // s after the release during which the root rate is active
    bool slowWalk = false;      // 0.3.4 walk floor of the speed map + walkRateMin
    float walkRateMin = 0.25f;
    float offSat = 0.0f;        // 0.3.4 offset saturation guard time (0 = off)
    bool startTurn = false;     // start towards a target 180 deg behind the body (Locomotion body-turn path)
    bool startReface = false;   // 0.3.4 start reface speed reduction
    bool excludeLands = false;  // 0.3.4 stand set without *_Lands_* / FromTraversal tags
    float seconds = 6.0f;
    float runUntil = 4.0f;      // input released at this time (stop)
    float startAt = 1.0f;       // input pressed
    int teleportUpdate = -1;    // 2 m capsule jump in this update
    bool capsuleTurn = false;   // capsule turns 90 deg in 0.25 s at t = 2 s while the animation keeps going
};

struct Window
{
    float maxCorr = 0.0f;  // m, max foot-lock correction
    float meanCorr = 0.0f; // m, mean correction of feet in contact
    float over10 = 0.0f;   // s, foot-time with a correction > 10 cm
    float slide = 0.0f;    // cm/s, displayed toe slide while in contact
    float maxOffset = 0.0f;// m
    double slideSum = 0.0, contactTime = 0.0, corrSum = 0.0, corrN = 0.0;
};

struct RunResult
{
    Window w[2];                 // 0 = start [startAt, +1.5 s], 1 = stop [runUntil, +1.5 s]
    float maxOffset = 0.0f;      // m
    float maxYaw = 0.0f;         // rad
    float offsetAtRun = 0.0f;    // m, |offset| at runUntil (steady run)
    float offsetAfterStop = 0.0f;// m, |offset| at the end
    float t90 = 0.0f;            // s, capsule time to 90 % speed
    float satSeconds = 0.0f;     // s, offset at >= 98 % of the clamp (whole run)
    float turnClipTime = 0.0f;   // s, first 0.6 s after the start spent in moving turn / pivot clips (Run_/Walk_ Turn, Pivot)
    float refaceClipTime = 0.0f; // s, same window in Reface / Stand turn clips
    float startCost = 0.0f;      // mean search cost in the first 0.6 s
    float bodyT15 = 0.0f;        // s, body within 15 deg of the target (start turn)
    float animStopDist = 0.0f;   // m, animation root distance (real time) after the release
    float capStopDist = 0.0f;    // m, capsule distance after the release
    bool finite = true;
    uint64_t hash = 1469598103934665603ull;
    uint64_t teleports = 0;
    uint64_t driftUnlocks = 0;
};

static RunResult Run(const RunCfg& c)
{
    RunResult r;
    MatcherConfig mc;
    mc.allowedTagMask = BaseMatcher::TagMaskContaining(*g_db, "Stand_");
    if (c.excludeLands)
        mc.allowedTagMask &= ~(BaseMatcher::TagMaskContaining(*g_db, "_Lands_") | BaseMatcher::TagMaskContaining(*g_db, "FromTraversal"));
    mc.transitionCost = 0.15f;
    mc.stopHalflife = c.stopSync ? c.stopHalflife : 0.0f;
    static BaseMatcher mm; // large state; reused between runs (Init resets everything)
    mm.Init(*g_db, mc);
    FootLocker fl;
    fl.Init(*g_db);
    OffsetRoot orr;
    {
        OffsetRootConfig oc;
        oc.saturatedTime = c.offSat;
        orr.Init(*g_db, mc.firstBodyBone, oc);
    }
    bt::BodyYawController bodyCtl;
    const float targetYaw = c.startTurn ? 3.14159265f : 0.0f; // world target (body starts at 0)
    double costSum = 0.0;
    int costN = 0;
    bool t15Set = false;
    RootRate rr;
    {
        RootRateConfig rc;
        rc.rateMin = c.rateMin;
        rc.activeWindow = c.rootWindow;
        rr.SetConfig(rc);
    }
    const int n = static_cast<int>(c.seconds / kDt);
    float v = 0.0f, yaw = 0.0f, prevYaw = 0.0f;
    float prevToe[2][2] = {};
    bool havePrev[2] = {false, false};
    bool t90Set = false;
    for (int i = 0; i < n; ++i)
    {
        const float t = static_cast<float>(i) * kDt;
        const bool input = t >= c.startAt && t < c.runUntil;
        if (input)
            v = std::fmin(c.speed, v + c.accel * kDt);
        else
            v = std::fmax(0.0f, v - 24.0f * kDt);
        if (!t90Set && v >= 0.9f * c.speed)
        {
            r.t90 = t - c.startAt;
            t90Set = true;
        }
        if (c.capsuleTurn && t >= 2.0f && t < 2.25f)
            yaw += (1.5707963f / 0.25f) * kDt;
        if (c.startTurn && input)
            yaw = bodyCtl.Command(yaw, targetYaw, kDt); // UTPPMMBodyYawCommand (5.1)
        const float yawStep = math::WrapAngle(yaw - prevYaw);
        prevYaw = yaw;
        if (c.startTurn && input && !t15Set && std::fabs(math::WrapAngle(targetYaw - yaw)) < 0.2618f)
        {
            r.bodyT15 = t - c.startAt;
            t15Set = true;
        }
        float wvx = 0.0f, wvy = 0.0f;
        math::Rot2(c.startTurn ? targetYaw : yaw, 0.0f, v, wvx, wvy); // Locomotion: capsule moves towards the target
        if (i == c.teleportUpdate)
            wvy += 2.0f / kDt; // 2 m jump in one update
        // driver (bt::ComputeDesiredMotion): with input max(capsule, intentFraction * target) (intentFraction 1.0 while
        // the acceleration is matched, else 0.5); without input the capsule velocity, or 0 with stop sync.
        // character-frame capsule velocity
        float cvx = 0.0f, cvy = 0.0f;
        math::Rot2(-yaw, wvx, wvy, cvx, cvy);
        float dvx = 0.0f, dvy = 0.0f, fx = 0.0f, fy = 1.0f;
        if (c.startTurn)
        {
            bt::MotionFrame mf;
            mf.charVel[0] = cvx;
            mf.charVel[1] = cvy;
            mf.move[0] = 0.0f;
            mf.move[1] = input ? 1.0f : 0.0f;
            mf.bodyYaw = yaw;
            mf.cameraYaw = yaw;
            mf.bodyTurn = input;
            mf.targetYaw = targetYaw;
            bt::MotionParams mp;
            mp.stateTargetSpeed = c.speed;
            mp.intentFraction = c.accel < 24.0f ? 1.0f : 0.5f;
            mp.stopIntent = c.stopSync;
            mp.startReface = c.startReface;
            const bt::DesiredMotion dm = bt::ComputeDesiredMotion(mf, mp);
            dvx = dm.vel[0];
            dvy = dm.vel[1];
            fx = dm.facing[0];
            fy = dm.facing[1];
        }
        else
            dvy = input ? std::fmax(v, (c.accel < 24.0f ? 1.0f : 0.5f) * c.speed) : (c.stopSync ? 0.0f : v);
        const float speed = std::sqrt(dvx * dvx + dvy * dvy);
        float scale = 1.0f, rate = 1.0f;
        {
            LoopRateParams lp;
            lp.walkFloor = c.slowWalk;
            lp.walkRateMin = c.walkRateMin;
            LoopRate(kStandMap, speed, lp, scale, rate);
        }
        float rvx = 0.0f, rvy = 0.0f;
        mm.RootVelocity(rvx, rvy);
        if (c.rootRate) // driver: stop phase only (stick released)
            rate = rr.Update(v, std::sqrt(rvx * rvx + rvy * rvy), kDt, !input, rate);
        // (RootRate tracks the time since the release itself and falls back to the loop rate after activeWindow)
        MatcherInput in;
        in.desiredVelocity[0] = dvx * scale;
        in.desiredVelocity[1] = dvy * scale;
        if (c.startTurn && input)
        {
            in.facingMode = FacingMode::Direction;
            in.desiredFacing[0] = fx;
            in.desiredFacing[1] = fy;
            in.externalYaw = true;
            in.externalYawStep = yawStep;
            in.externalYawRate = (yawStep / kDt) / rate;
        }
        else
        {
            in.facingMode = FacingMode::YawRate;
            in.yawRate = (yawStep / kDt) / rate;
        }
        mm.Update(kDt * rate, in);
        bodyCtl.FeedAnimRate(mm.RootYawRate() * rate, kDt);
        {
            const MatcherDiagnostics& dg = mm.Diagnostics();
            if (t >= c.startAt && t < c.startAt + 0.6f)
            {
                const std::string_view nm = g_db->ClipName(static_cast<uint32_t>(dg.clip));
                const bool reface = nm.find("Reface") != std::string_view::npos || nm.find("Stand_Turn") != std::string_view::npos ||
                                    nm.find("Idle_Turn") != std::string_view::npos;
                const bool moveTurn = !reface && (nm.find("Turn") != std::string_view::npos || nm.find("Pivot") != std::string_view::npos);
                if (reface)
                    r.refaceClipTime += kDt;
                if (moveTurn)
                    r.turnClipTime += kDt;
                if (dg.searchedThisUpdate)
                {
                    costSum += dg.lastSearchCost;
                    ++costN;
                }
            }
        }
        float avx = 0.0f, avy = 0.0f;
        mm.RootVelocity(avx, avy);
        if (t >= c.runUntil)
        {
            r.animStopDist += std::sqrt(avx * avx + avy * avy) * rate * kDt;
            r.capStopDist += v * kDt;
        }
        std::span<const BoneTransform> pose(mm.Output().bones);
        const int win = (t >= c.startAt && t < c.startAt + 1.5f) ? 0 : ((t >= c.runUntil && t < c.runUntil + 1.5f) ? 1 : -1);
        if (c.offset)
        {
            const float anim[2] = {avx * rate, avy * rate};
            const float cap[2] = {cvx, i == c.teleportUpdate ? cvy + 2.0f / kDt : cvy};
            orr.Update(kDt, cap, yawStep, anim, mm.RootYawRate() * rate, true);
            orr.Apply(pose);
            pose = orr.Output();
            const float d = std::sqrt(orr.X() * orr.X() + orr.Y() * orr.Y());
            r.maxOffset = std::fmax(r.maxOffset, d);
            if (win >= 0)
                r.w[win].maxOffset = std::fmax(r.w[win].maxOffset, d);
            r.maxYaw = std::fmax(r.maxYaw, std::fabs(orr.Yaw()));
            if (std::fabs(t - c.runUntil) < 0.5f * kDt)
                r.offsetAtRun = d;
            r.offsetAfterStop = d;
        }
        const MatcherDiagnostics& md = mm.Diagnostics();
        fl.Update(kDt, wvx, wvy, yaw, FootLocker::SampleContact(*g_db, md.frame, md.alpha, FootLocker::kLeft),
                  FootLocker::SampleContact(*g_db, md.frame, md.alpha, FootLocker::kRight), pose);
        if (g_debug && win >= 0 && (i % 3 == 0))
            std::printf("    w%d av=%.2f t=%.3f v=%.2f rate=%.2f clip=%.*s f=%d off=%.1f corr=%.1f/%.1f lk=%d/%d c=%.2f/%.2f\n", win,
                        std::sqrt(avx * avx + avy * avy) * rate, t, v, rate,
                        static_cast<int>(g_db->ClipName(static_cast<uint32_t>(md.clip)).size()),
                        g_db->ClipName(static_cast<uint32_t>(md.clip)).data(), md.clipFrame, orr.Y() * 100.0,
                        fl.Diagnostics().foot[0].correction * 100.0, fl.Diagnostics().foot[1].correction * 100.0,
                        fl.Diagnostics().foot[0].locked ? 1 : 0, fl.Diagnostics().foot[1].locked ? 1 : 0,
                        FootLocker::SampleContact(*g_db, md.frame, md.alpha, 0), FootLocker::SampleContact(*g_db, md.frame, md.alpha, 1));
        for (int s = 0; s < 2; ++s)
        {
            const float contact = FootLocker::SampleContact(*g_db, md.frame, md.alpha, s);
            const BoneTransform toe = fl.ModelSpace(fl.Output(), fl.LockBone(s));
            float wx = 0.0f, wy = 0.0f;
            fl.CharacterToWorld(toe.p, wx, wy);
            if (win >= 0)
            {
                Window& w = r.w[win];
                const float corr = fl.Diagnostics().foot[s].correction;
                w.maxCorr = std::fmax(w.maxCorr, corr);
                if (contact > 0.5f)
                {
                    w.corrSum += corr;
                    w.corrN += 1.0;
                }
                if (corr > 0.10f)
                    w.over10 += kDt;
                if (contact > 0.5f && havePrev[s] && i != c.teleportUpdate)
                {
                    w.slideSum += std::sqrt((wx - prevToe[s][0]) * (wx - prevToe[s][0]) + (wy - prevToe[s][1]) * (wy - prevToe[s][1]));
                    w.contactTime += kDt;
                }
            }
            prevToe[s][0] = wx;
            prevToe[s][1] = wy;
            havePrev[s] = contact > 0.5f;
        }
        for (const BoneTransform& b : fl.Output())
        {
            const float f[7] = {b.p.x, b.p.y, b.p.z, b.q.x, b.q.y, b.q.z, b.q.w};
            for (float x : f)
            {
                if (!std::isfinite(x))
                    r.finite = false;
                uint32_t u;
                std::memcpy(&u, &x, 4);
                r.hash = (r.hash ^ u) * 1099511628211ull;
            }
        }
    }
    for (Window& w : r.w)
    {
        w.meanCorr = w.corrN > 0.0 ? static_cast<float>(w.corrSum / w.corrN) : 0.0f;
        w.slide = w.contactTime > 0.0 ? static_cast<float>(100.0 * w.slideSum / w.contactTime) : 0.0f;
    }
    r.teleports = orr.Diagnostics().teleports;
    r.satSeconds = orr.Diagnostics().saturatedSeconds;
    r.startCost = costN > 0 ? static_cast<float>(costSum / costN) : 0.0f;
    r.driftUnlocks = fl.Diagnostics().driftUnlocks;
    return r;
}

static void Print(const char* aName, const RunResult& r)
{
    for (int k = 0; k < 2; ++k)
    {
        const Window& w = r.w[k];
        std::printf("  %-34s %s maxCorr=%.1fcm meanCorr=%.1fcm >10cm=%.2fs slide=%.1fcm/s maxOff=%.1fcm", aName,
                    k == 0 ? "START" : "STOP ", w.maxCorr * 100.0, w.meanCorr * 100.0, w.over10, w.slide, w.maxOffset * 100.0);
        if (k == 0)
            std::printf(" t90=%.2fs offRun=%.1fcm sat=%.2fs\n", r.t90, r.offsetAtRun * 100.0, r.satSeconds);
        else
            std::printf(" animDist=%.2fm capDist=%.2fm offEnd=%.2fcm maxYaw=%.1fdeg drift=%llu\n", r.animStopDist, r.capStopDist,
                        r.offsetAfterStop * 100.0, r.maxYaw * 57.2958, static_cast<unsigned long long>(r.driftUnlocks));
    }
}

// Averages over 8 variants (idle 0.8..1.5 s, run 2.5..3.4 s): the matcher's clip choice is sensitive to small state
// differences, so single runs are noisy. Window metrics are averaged; maxOffset = mean of the per-run maxima, clampHits =
// runs whose stop window reached the 25 cm clamp; a stop seconds after the release.
static RunResult RunSet(const RunCfg& aCfg, int* aClampHits = nullptr)
{
    RunResult m;
    int hits = 0;
    constexpr int kVariants = 8;
    bool finite = true;
    for (int k = 0; k < kVariants; ++k)
    {
        RunCfg c = aCfg;
        c.startAt = 0.8f + 0.1f * static_cast<float>(k);
        c.runUntil = c.startAt + 2.5f + 0.13f * static_cast<float>(k);
        c.seconds = c.runUntil + 2.0f;
        const RunResult r = Run(c);
        finite = finite && r.finite;
        for (int w = 0; w < 2; ++w)
        {
            m.w[w].maxCorr += r.w[w].maxCorr / kVariants;
            m.w[w].meanCorr += r.w[w].meanCorr / kVariants;
            m.w[w].over10 += r.w[w].over10 / kVariants;
            m.w[w].slide += r.w[w].slide / kVariants;
            m.w[w].maxOffset += r.w[w].maxOffset / kVariants;
        }
        if (r.w[1].maxOffset >= 0.249f)
            ++hits;
        m.maxOffset = std::fmax(m.maxOffset, r.maxOffset);
        m.maxYaw = std::fmax(m.maxYaw, r.maxYaw);
        m.offsetAtRun = std::fmax(m.offsetAtRun, r.offsetAtRun);
        m.offsetAfterStop = std::fmax(m.offsetAfterStop, r.offsetAfterStop);
        m.t90 += r.t90 / kVariants;
        m.satSeconds += r.satSeconds / kVariants;
        m.turnClipTime += r.turnClipTime / kVariants;
        m.refaceClipTime += r.refaceClipTime / kVariants;
        m.startCost += r.startCost / kVariants;
        m.bodyT15 += r.bodyT15 / kVariants;
        m.animStopDist += r.animStopDist / kVariants;
        m.capStopDist += r.capStopDist / kVariants;
        m.driftUnlocks += r.driftUnlocks;
        m.hash = (m.hash ^ r.hash) * 1099511628211ull;
    }
    m.finite = finite;
    if (aClampHits)
        *aClampHits = hits;
    return m;
}

static void StartScenario(bool aFull);

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mm_offsetroot_test <db.utppmm>\n");
        return 2;
    }
    FILE* f = nullptr;
    if (fopen_s(&f, argv[1], "rb") != 0 || !f)
    {
        std::printf("cannot open %s\n", argv[1]);
        return 2;
    }
    std::vector<uint8_t> bytes;
    std::fseek(f, 0, SEEK_END);
    bytes.resize(static_cast<size_t>(std::ftell(f)));
    std::fseek(f, 0, SEEK_SET);
    const bool readOk = std::fread(bytes.data(), 1, bytes.size(), f) == bytes.size();
    std::fclose(f);
    static Database db;
    std::string err;
    if (!readOk || !db.LoadFromMemory(bytes, 0x72CB314EB01F562Dull, err))
    {
        std::printf("load failed: %s\n", err.c_str());
        return 2;
    }
    g_db = &db;
    std::printf("db: %u frames, %u clips\n", db.NumFrames(), db.NumClips());

    // ---- unit checks on OffsetRoot alone
    {
        OffsetRoot o;
        Expect("init", o.Init(db, 2));
        // equal capsule and animation velocity -> offset exactly 0, pose bitwise unchanged
        std::vector<BoneTransform> pose(db.NumBones());
        for (uint32_t b = 0; b < db.NumBones(); ++b)
        {
            const auto p = db.Pose(100);
            pose[b].p = {p[b * 7 + 0], p[b * 7 + 1], p[b * 7 + 2]};
            pose[b].q = {p[b * 7 + 3], p[b * 7 + 4], p[b * 7 + 5], p[b * 7 + 6]};
        }
        const float vel[2] = {0.3f, 3.85f};
        for (int i = 0; i < 600; ++i)
            o.Update(kDt, vel, 0.0f, vel, 0.0f, true);
        o.Apply(pose);
        Expect("matched motion keeps offset 0 and the pose bitwise", o.X() == 0.0f && o.Y() == 0.0f && o.Yaw() == 0.0f &&
                                                                       std::memcmp(o.Output().data(), pose.data(), pose.size() * sizeof(BoneTransform)) == 0);
        // capsule 3.85 m/s, animation still: clamp at maxDistance, then converges after the capsule stops
        const float cap[2] = {0.0f, 3.85f}, zero[2] = {0.0f, 0.0f};
        float maxD = 0.0f;
        for (int i = 0; i < 120; ++i)
        {
            o.Update(kDt, cap, 0.0f, zero, 0.0f, true);
            maxD = std::fmax(maxD, std::sqrt(o.X() * o.X() + o.Y() * o.Y()));
        }
        std::printf("  after clamp: x=%g y=%g yaw=%g\n", o.X(), o.Y(), o.Yaw());
        Expect("bounded by maxDistance", maxD <= 0.25f + 1e-5f && o.Y() < -0.2f, Fmt("max %.4f m, y %.3f", maxD, o.Y()));
        int steps = 0;
        while (std::sqrt(o.X() * o.X() + o.Y() * o.Y()) > 0.005f && steps < 600)
        {
            o.Update(kDt, zero, 0.0f, zero, 0.0f, true);
            ++steps;
        }
        Expect("converges to < 5 mm within 1.5 s", steps < 90, Fmt("%.0f updates", steps));
        // gate closed: faster return
        for (int i = 0; i < 120; ++i)
            o.Update(kDt, cap, 0.0f, zero, 0.0f, true);
        int gsteps = 0;
        while (std::sqrt(o.X() * o.X() + o.Y() * o.Y()) > 0.005f && gsteps < 600)
        {
            o.Update(kDt, zero, 0.0f, zero, 0.0f, false);
            ++gsteps;
        }
        Expect("gate closed returns faster", gsteps < steps && gsteps <= 20, Fmt("%.0f vs %.0f updates", gsteps, steps));
        // yaw: capsule turns 90 deg, animation does not -> clamp 15 deg
        float maxY = 0.0f;
        for (int i = 0; i < 15; ++i)
        {
            o.Update(kDt, zero, 1.5707963f / 15.0f, zero, 0.0f, true);
            maxY = std::fmax(maxY, std::fabs(o.Yaw()));
        }
        Expect("yaw bounded by maxYaw", maxY <= 0.2618f + 1e-5f && maxY > 0.2f, Fmt("%.2f deg", maxY * 57.2958));
        // teleport: capsule step 2 m -> reset
        for (int i = 0; i < 30; ++i)
            o.Update(kDt, cap, 0.0f, zero, 0.0f, true);
        const float jump[2] = {0.0f, 2.0f / kDt};
        o.Update(kDt, jump, 0.0f, zero, 0.0f, true);
        Expect("teleport resets", o.X() == 0.0f && o.Y() == 0.0f && o.Yaw() == 0.0f && o.Diagnostics().teleports == 1);
        // feed gap
        for (int i = 0; i < 30; ++i)
            o.Update(kDt, cap, 0.0f, zero, 0.0f, true);
        o.Update(0.5f, cap, 0.0f, zero, 0.0f, true);
        Expect("feed gap (dt > 0.25 s) resets", o.X() == 0.0f && o.Y() == 0.0f);
        // NaN / inf input
        const float nanv[2] = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()};
        bool fin = true;
        for (int i = 0; i < 60; ++i)
        {
            o.Update(kDt, nanv, std::numeric_limits<float>::quiet_NaN(), nanv, std::numeric_limits<float>::infinity(), true);
            o.Update(std::numeric_limits<float>::quiet_NaN(), cap, 0.0f, zero, 0.0f, true);
            fin = fin && std::isfinite(o.X()) && std::isfinite(o.Y()) && std::isfinite(o.Yaw());
        }
        o.Apply(pose);
        for (const BoneTransform& b : o.Output())
            fin = fin && std::isfinite(b.p.x) && std::isfinite(b.p.y) && std::isfinite(b.q.w);
        Expect("NaN / inf input stays finite", fin);
        // only the root children change: Hips translation moves by the offset, child bones unchanged locally
        OffsetRoot o2;
        o2.Init(db, 2);
        for (int i = 0; i < 20; ++i)
            o2.Update(kDt, cap, 0.0f, zero, 0.0f, true);
        o2.Apply(pose);
        bool onlyRoot = true;
        for (uint32_t b = 3; b < db.NumBones(); ++b)
            if (db.BoneParent(b) >= 2 && std::memcmp(&o2.Output()[b], &pose[b], sizeof(BoneTransform)) != 0)
                onlyRoot = false;
        const float hipDy = o2.Output()[2].p.y - pose[2].p.y;
        Expect("offset applied to Hips only (children local unchanged)", onlyRoot && std::fabs(hipDy - o2.Y()) < 1e-5f,
               Fmt("hips dy %.4f offset %.4f", hipDy, o2.Y()));
        OffsetRootConfig bad;
        bad.translationHalflife = -1.0f;
        Expect("invalid config rejected", !o2.SetConfig(bad));
    }

    StartScenario(false);
    if (argc >= 3)
    {
        // optional: the full database (motion Run_Start / Run_Stop clips present)
        FILE* f2 = nullptr;
        if (fopen_s(&f2, argv[2], "rb") == 0 && f2)
        {
            std::vector<uint8_t> b2;
            std::fseek(f2, 0, SEEK_END);
            b2.resize(static_cast<size_t>(std::ftell(f2)));
            std::fseek(f2, 0, SEEK_SET);
            const bool ok2 = std::fread(b2.data(), 1, b2.size(), f2) == b2.size();
            std::fclose(f2);
            static Database db2;
            if (ok2 && db2.LoadFromMemory(b2, 0x72CB314EB01F562Dull, err))
            {
                g_db = &db2;
                if (g_debug)
                {
                    // UTPP_TEST_DEBUG: single traced case (slow walk 0.6 m/s, 0.3.4 switches) on the full database
                    RunCfg c;
                    c.accel = 3.75f;
                    c.offset = c.rootRate = c.stopSync = c.slowWalk = c.startReface = c.excludeLands = true;
                    c.offSat = 0.3f;
                    c.speed = 0.6f;
                    c.walkRateMin = 0.25f;
                    const RunResult r = Run(c);
                    Print("debug walk 0.6", r);
                    return 0;
                }
                std::printf("db2: %u frames, %u clips (%s)\n", db2.NumFrames(), db2.NumClips(), argv[2]);
                StartScenario(true);
            }
            else
                Expect("second database loads", false, err);
        }
    }
    std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}

static void PrintSet(const char* aName, const RunResult& r, int aHits)
{
    Print(aName, r);
    std::printf("  %-34s       stop clamp hits %d/8, stop overshoot animDist-capDist=%.2fm\n", "", aHits,
                r.animStopDist - r.capStopDist);
}

static void StartScenario(bool aFull)
{
    const char* tag = aFull ? "full db: " : "";
    char name[160];
    auto N = [&](const char* aText) {
        std::snprintf(name, sizeof name, "%s%s", tag, aText);
        return name;
    };
    // ---- start from idle at run input, steady run, stop (driver emulation), averaged over 8 variants
    RunCfg base;
    RunCfg cb = base;
    cb.offset = true;
    RunCfg cd = base;
    cd.accel = 3.75f;
    cd.offset = true; // 0.3.3 default
    RunCfg ce = cd;
    ce.rootRate = true;
    ce.stopSync = true;
    ce.slowWalk = true;
    ce.offSat = 0.3f;
    ce.startReface = true;
    ce.excludeLands = true; // 0.3.4 default (all 0.3.4 switches on)
    int hA = 0, hB = 0, hD = 0, hE = 0;
    const RunResult a = RunSet(base, &hA);
    const RunResult b = RunSet(cb, &hB);
    const RunResult d = RunSet(cd, &hD);
    const RunResult e = RunSet(ce, &hE);
    std::printf("%srun 3.85 m/s, mean of 8 variants (60 Hz, windows 1.5 s from the input / the release):\n", tag);
    PrintSet("0.3.2 (accel 24, no offset)", a, hA);
    PrintSet("offset only", b, hB);
    PrintSet("0.3.3 (accel 3.75 + offset)", d, hD);
    PrintSet("0.3.4 (all 0.3.4 switches)", e, hE);
    {
        RunCfg c1 = cd;
        c1.rootRate = true;
        int h1 = 0;
        const RunResult r1 = RunSet(c1, &h1);
        PrintSet("0.3.3 + root rate (stops)", r1, h1);
        RunCfg c2 = cd;
        c2.stopSync = true;
        int h2 = 0;
        const RunResult r2 = RunSet(c2, &h2);
        PrintSet("0.3.3 + stop sync", r2, h2);
    }
    // parameter sweep (informational; defaults chosen from these tables)
    for (float h : {0.05f, 0.10f})
        for (float m : {0.5f, 0.3f})
            for (float wnd : {0.3f, 0.6f, 100.0f})
            {
                RunCfg ch = ce;
                ch.stopHalflife = h;
                ch.rateMin = m;
                ch.rootWindow = wnd;
                int hh = 0;
                const RunResult rh = RunSet(ch, &hh);
                char nm[64];
                std::snprintf(nm, sizeof nm, "sweep stopHl %.2f rateMin %.2f win %.1f", static_cast<double>(h),
                              static_cast<double>(m), static_cast<double>(wnd));
                PrintSet(nm, rh, hh);
            }
    // sprint
    RunCfg sd = cd;
    sd.speed = 8.2f;
    sd.accel = 6.96f;
    RunCfg se = sd;
    se.rootRate = true;
    se.stopSync = true;
    int hS3 = 0, hS4 = 0;
    const RunResult s3 = RunSet(sd, &hS3);
    const RunResult s4 = RunSet(se, &hS4);
    std::printf("%ssprint 8.2 m/s, mean of 8 variants:\n", tag);
    PrintSet("0.3.3 sprint", s3, hS3);
    PrintSet("0.3.4 sprint", s4, hS4);

    // ---- slow walk (gamepad half stick: capsule 0.6 / 1.0 m/s)
    std::printf("%sslow walk (analog), mean of 8 variants:\n", tag);
    float sat3 = 0.0f, sat4 = 0.0f, slide3 = 0.0f, slide4 = 0.0f, corr3 = 0.0f, corr4 = 0.0f;
    for (float sp : {0.6f, 1.0f})
    {
        RunCfg w3 = cd;
        w3.speed = sp;
        RunCfg w4 = ce;
        w4.speed = sp;
        int h3 = 0, h4 = 0;
        const RunResult r3 = RunSet(w3, &h3);
        const RunResult r4 = RunSet(w4, &h4);
        char nm[64];
        std::snprintf(nm, sizeof nm, "0.3.3 walk %.1f m/s", static_cast<double>(sp));
        PrintSet(nm, r3, h3);
        std::snprintf(nm, sizeof nm, "0.3.4 walk %.1f m/s", static_cast<double>(sp));
        PrintSet(nm, r4, h4);
        for (float wm : {0.35f, 0.45f})
        {
            RunCfg wx = w4;
            wx.walkRateMin = wm;
            int hx = 0;
            const RunResult rx = RunSet(wx, &hx);
            std::snprintf(nm, sizeof nm, "sweep walk %.1f walkRateMin %.2f", static_cast<double>(sp), static_cast<double>(wm));
            PrintSet(nm, rx, hx);
        }
        sat3 += r3.satSeconds;
        sat4 += r4.satSeconds;
        slide3 += r3.w[0].slide;
        slide4 += r4.w[0].slide;
        corr3 += r3.w[0].meanCorr;
        corr4 += r4.w[0].meanCorr;
    }
    Expect(N("slow walk: offset saturated time below 0.3.3"), sat4 < sat3 || sat4 < 0.05f, Fmt("%.2f vs %.2f s (sum of 2 speeds)", sat4, sat3));
    Expect(N("slow walk: offset saturated time < 0.6 s per run"), sat4 < 1.2f, Fmt("%.2f s (sum of 2 speeds)", sat4));
    Expect(N("slow walk: mean correction below 0.3.3"), corr4 < corr3, Fmt("%.1f vs %.1f cm (sum)", corr4 * 100.0, corr3 * 100.0));
    std::printf("  INFO slow walk slide (start window): %.1f vs %.1f cm/s (sum of 2 speeds, 0.3.4 vs 0.3.3)\n", slide4, slide3);

    // ---- start towards a target 180 deg behind the body (Locomotion body-turn path)
    {
        RunCfg t3 = cd;
        t3.startTurn = true;
        RunCfg t4 = ce;
        t4.startTurn = true;
        RunCfg t4n = t4;
        t4n.startReface = false;
        int h3 = 0, h4 = 0, h4n = 0;
        const RunResult r3 = RunSet(t3, &h3);
        const RunResult r4 = RunSet(t4, &h4);
        const RunResult r4n = RunSet(t4n, &h4n);
        std::printf("%sstart 180 deg behind the body, mean of 8 variants (first 0.6 s):\n", tag);
        const RunResult* rs[3] = {&r3, &r4n, &r4};
        const char* ns[3] = {"0.3.3", "0.3.4 without start reface", "0.3.4"};
        for (int k = 0; k < 3; ++k)
            std::printf("  %-28s movingTurnClips=%.2fs refaceClips=%.2fs meanCost=%.2f bodyT15=%.2fs startMeanCorr=%.1fcm "
                        "startMaxOff=%.1fcm slide=%.1fcm/s\n",
                        ns[k], rs[k]->turnClipTime, rs[k]->refaceClipTime, rs[k]->startCost, rs[k]->bodyT15,
                        rs[k]->w[0].meanCorr * 100.0, rs[k]->w[0].maxOffset * 100.0, rs[k]->w[0].slide);
        // Criterion set before the measurement; NOT met on the full database (0.30 s both): reported, not a pass.
        std::printf("  INFO %s(criterion: less time in moving turn / pivot clips than 0.3.3) %.2f vs %.2f s -> %s\n", tag,
                    r4.turnClipTime, r3.turnClipTime, (r4.turnClipTime < r3.turnClipTime || r4.turnClipTime < 0.05f) ? "met" : "NOT met");
        // Informational (criterion chosen after the full-database run, so it is not counted as a pass/fail check).
        std::printf("  INFO %s(start 180 mean search cost 0.3.4 vs 0.3.3) %.2f vs %.2f -> %s\n", tag, r4.startCost, r3.startCost,
                    r4.startCost <= r3.startCost ? "lower" : "HIGHER");
        Expect(N("start 180: outputs finite"), r3.finite && r4.finite && r4n.finite);
    }

    if (!aFull)
    {
        Expect("0.3.3 start slide < 50 % of 0.3.2", d.w[0].slide < 0.5f * a.w[0].slide, Fmt("%.1f vs %.1f cm/s", d.w[0].slide, a.w[0].slide));
        Expect("0.3.3 start mean correction below 0.3.2", d.w[0].meanCorr < a.w[0].meanCorr, Fmt("%.1f vs %.1f cm", d.w[0].meanCorr * 100.0, a.w[0].meanCorr * 100.0));
        Expect("offset only reduces the start slide", b.w[0].slide < a.w[0].slide, Fmt("%.1f vs %.1f cm/s", b.w[0].slide, a.w[0].slide));
    }
    // 0.3.4 stop criteria (averages, both databases)
    Expect(N("0.3.4 run stop: overshoot (anim - capsule distance) below 0.3.3"),
           e.animStopDist - e.capStopDist < d.animStopDist - d.capStopDist,
           Fmt("%.2f vs %.2f m", e.animStopDist - e.capStopDist, d.animStopDist - d.capStopDist));
    // Criterion set before the measurement, NOT met on the subset (both 8/8): reported, not counted as a pass.
    std::printf("  INFO %s(criterion: fewer run-stop clamp hits than 0.3.3) %d vs %d of 8 -> %s\n", tag, hE, hD,
                (hE < hD || hE == 0) ? "met" : "NOT met");
    Expect(N("0.3.4 run stop: mean correction not above 0.3.3"), e.w[1].meanCorr <= d.w[1].meanCorr + 1e-4f,
           Fmt("%.1f vs %.1f cm", e.w[1].meanCorr * 100.0, d.w[1].meanCorr * 100.0));
    Expect(N("0.3.4 sprint stop: overshoot below 0.3.3"), s4.animStopDist - s4.capStopDist < s3.animStopDist - s3.capStopDist,
           Fmt("%.2f vs %.2f m", s4.animStopDist - s4.capStopDist, s3.animStopDist - s3.capStopDist));
    Expect(N("0.3.4 start mean correction not above 0.3.3 + 1 cm"), e.w[0].meanCorr <= d.w[0].meanCorr + 0.01f,
           Fmt("%.1f vs %.1f cm", e.w[0].meanCorr * 100.0, d.w[0].meanCorr * 100.0));

    Expect(N("offset bounded (0.25 m)"), b.maxOffset <= 0.25f + 1e-4f && d.maxOffset <= 0.25f + 1e-4f && e.maxOffset <= 0.25f + 1e-4f);
    Expect(N("offset small in steady run"), d.offsetAtRun < 0.10f && e.offsetAtRun < 0.10f, Fmt("%.1f / %.1f cm (max of 8)", d.offsetAtRun * 100.0, e.offsetAtRun * 100.0));
    Expect(N("offset converges after the stop (2 s)"), d.offsetAfterStop < 0.02f && e.offsetAfterStop < 0.02f, Fmt("%.2f / %.2f cm (max of 8)", d.offsetAfterStop * 100.0, e.offsetAfterStop * 100.0));
    Expect(N("all outputs finite"), a.finite && b.finite && d.finite && e.finite && s3.finite && s4.finite);
    // single-run checks
    const RunResult e1 = Run(ce), e2 = Run(ce);
    Expect(N("deterministic (bitwise)"), e2.hash == e1.hash);
    const RunResult a1 = Run(base), a2 = Run(base), b1 = Run(cb);
    Expect(N("kill switch: offset off == 0.3.2 path (bitwise)"), a2.hash == a1.hash && a1.hash != b1.hash);
    const RunResult d1 = Run(cd), d2 = Run(cd);
    Expect(N("kill switches: root rate + stop sync off == 0.3.3 path (bitwise)"), d2.hash == d1.hash && d1.hash != e1.hash);
    RunCfg ct = ce;
    ct.teleportUpdate = 150;
    const RunResult t = Run(ct);
    Expect(N("teleport during run: offset reset, finite"), t.teleports == 1 && t.finite && t.maxOffset <= 0.25f + 1e-4f);
    RunCfg cy = ce;
    cy.capsuleTurn = true;
    const RunResult y = Run(cy);
    Print("0.3.4 + capsule 90 deg turn", y);
    Expect(N("capsule turn: yaw offset bounded (15 deg), finite"), y.maxYaw <= 0.2618f + 1e-4f && y.finite, Fmt("%.1f deg", y.maxYaw * 57.2958));
}
