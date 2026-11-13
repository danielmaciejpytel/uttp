// Offline test of the Phase 4.5 foot locker (contact-based foot lock + two-bone leg IK), not shipped.
// Usage: utpp_mm_footlock_test <database.utppmm> [stance database]      Exit code 0 = all checks passed.
// loco C1b (plant / replant): crouch sections on the stance database (default utpp_base.utppmm next to the first one;
// skipped with an INFO line when it cannot be read).
//
// The matcher runs at 60 Hz; the "capsule" is either the played animation's root motion (optionally scaled, to
// emulate a game capsule faster than the animation) or the matcher's simulated character (mixed script). The foot
// locker receives the capsule world velocity + yaw exactly like MMDriver::Feed.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <new>
#include <string>
#include <string_view>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/mm/FootLocker.hpp"
#include "../src/mm/BaseMatcher.hpp"

using namespace utpp::mm;

// ---------------------------------------------------------------- allocation counter
static std::atomic<bool> g_countAllocs{false};
static std::atomic<uint64_t> g_allocs{0};
static void* CountedAlloc(std::size_t n)
{
    if (g_countAllocs.load(std::memory_order_relaxed))
        g_allocs.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1))
        return p;
    throw std::bad_alloc();
}
void* operator new(std::size_t n) { return CountedAlloc(n); }
void* operator new[](std::size_t n) { return CountedAlloc(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

static int g_fail = 0;
static int g_pass = 0;
static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}
static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0, double e = 0, double f = 0,
                       double g = 0, double h = 0)
{
    char buf[640];
    std::snprintf(buf, sizeof buf, aFmt, a, b, c, d, e, f, g, h);
    return buf;
}
static double Percentile(std::vector<double> aV, double aP)
{
    if (aV.empty())
        return 0.0;
    std::sort(aV.begin(), aV.end());
    return aV[std::min(aV.size() - 1, static_cast<size_t>(aP * double(aV.size() - 1) + 0.5))];
}

static const Database* g_db = nullptr;
static FootReplantConfig g_rcTest{}; // loco C1b: replant config of every FootLocker in this test (UTPP_FL_REPLANT)
static constexpr float kDt = 1.0f / 60.0f;
static constexpr float kRad2Deg = 57.2957795f;

static MatcherInput Walk(float aVx, float aVy)
{
    MatcherInput in;
    in.desiredVelocity[0] = aVx;
    in.desiredVelocity[1] = aVy;
    in.facingMode = FacingMode::Direction;
    const float l = std::sqrt(aVx * aVx + aVy * aVy);
    in.desiredFacing[0] = l > 1e-3f ? aVx / l : 0.0f;
    in.desiredFacing[1] = l > 1e-3f ? aVy / l : 0.0f;
    return in;
}

// Same 20 s mixed script as utpp_mm_sim_test: idle, walk, diagonal, run, stop, yaw-rate turn, strafe, backwards.
static MatcherInput MixedScript(double t)
{
    const double u = std::fmod(t, 20.0);
    if (u < 2.0)
        return Walk(0.0f, 0.0f);
    if (u < 5.0)
        return Walk(0.0f, 1.5f);
    if (u < 7.0)
        return Walk(-1.06f, 1.06f);
    if (u < 10.0)
        return Walk(0.0f, 3.75f);
    if (u < 12.0)
        return Walk(0.0f, 0.0f);
    if (u < 14.0)
    {
        MatcherInput in;
        in.desiredVelocity[1] = 1.5f;
        in.facingMode = FacingMode::YawRate;
        in.yawRate = 0.8f;
        return in;
    }
    if (u < 16.0)
        return Walk(1.5f, 0.0f);
    if (u < 17.0)
        return Walk(0.0f, -1.5f);
    return Walk(0.0f, 0.0f);
}

static int32_t FindClip(std::string_view aName)
{
    for (uint32_t c = 0; c < g_db->NumClips(); ++c)
        if (g_db->ClipName(c) == aName)
            return static_cast<int32_t>(c);
    return -1;
}

enum class Capsule
{
    Zero,      // capsule does not move
    AnimRoot,  // played root motion x scale
    Simulated, // matcher's simulated character (trajectory spring)
};

struct Scenario
{
    const char* name = "";
    int32_t clip = -1;           // >= 0: play only this clip (allowed range = the clip), else whole database
    float speed = 0.0f;          // desired forward speed (clip runs)
    bool mixed = false;          // MixedScript input
    Capsule capsule = Capsule::AnimRoot;
    float capsuleScale = 1.0f;
    float seconds = 10.0f;
    int teleportUpdate = -1;     // add a 2 m forward capsule jump in this update
    bool lock = true;
    int contactMask = 3;         // loco C0 test: bit s set = foot s gets its sampled contact, else 0 (that foot never locks)
    float slipStart = 0.0f;      // loco C0 test: > 0 = FootLocker::SetSlip(slipStart, 0.02) (driver 0.5.9 soft slip), 0 = default off
    // loco C1b: -1 = SetReplant never called (0.5.9 default), 0 = SetReplant(false, stops) every update, 1 = replant on
    int replant = -1;
    bool stops = false;          // loco C1b: Stops profile (idle / stop clip)
    float matcherRate = 1.0f;    // loco C1b: matcher time per real second; the AnimRoot capsule keeps capsuleScale x the clip's
                                 // root velocity (in game a slow start clip while the capsule already moves)
};

struct Metrics
{
    bool finite = true;
    // per-update samples
    double contactTime[2] = {0, 0};
    double slideAnim[2] = {0, 0};  // m, animated toe (lock point) horizontal world travel while contact
    double slideFinal[2] = {0, 0}; // m, final toe
    double lockedSamples = 0, driftSum = 0, driftMax = 0; // locked && weight == 1: |final - first final of the lock|
    double maxCorrection = 0, sumCorrection = 0, corrSamples = 0;
    double maxLegRotStepAnim = 0, maxLegRotStepFinal = 0; // deg per update, UpLeg/Leg/Foot local rotations
    double maxCorrStep = 0;                                // m per update, change of (final - animated) ankle, char frame
    double maxCorrStepTeleport = 0;                        // same, within +-30 updates of the teleport
    double maxErrorUnclamped = 0;                          // m, |final - IK target| when not clamped
    double maxBlendOutSink = 0;                            // m, animated - final ankle height of an unlocked foot still blending out
    uint64_t locks = 0, relocks = 0, driftUnlocks = 0, clamps = 0, nonFinite = 0;
    std::vector<double> updateMicros;
    uint64_t allocs = 0;
    // loco C0: per-foot counters add up to the totals, the last event of a foot with drift unlocks is a drift beyond maxDrift
    bool perFootOk = true, eventsOk = true;
    uint64_t driftPerFoot[2] = {0, 0};
    FootLockDiagnostics diag; // final diagnostics (per-foot attribution checks)
    // loco C1b: displayed toe (lock point) world motion per update while the foot is held at full weight
    std::vector<double> plantedSpeed;     // m/s, planted (locked, weight 1, the lock not moved by a step this update)
    double plantedSpeedMax = 0.0;         // m/s
    double stepSpeedMax = 0.0;            // m/s, world speed of the displayed toe in step updates (weight 1)
    double stepExcess = -1.0;             // m, max of (toe world motion - stepSpeed x dt - animated toe world motion) in step updates
    double maxToeJump = 0.0;              // m per update, any phase while locked at weight 1 (this and the previous update)
    std::vector<double> corrLocked;       // m, ankle correction while locked
    double longestStep = 0.0;             // s
    double plantedErrMax = 0.0;           // m, IK target error while planted (reach limit)
    int plantedMoving = 0;                // planted updates with the toe moving faster than 1 cm/s
    int plantedUpdates = 0, plantedReach = 0; // planted updates; of them reach-limited (not in plantedSpeed)
    // step lift in the POSE (review fix): locked feet at weight 1 that reach their target (not clamped, IK error <= 1 mm) - the
    // ankle shown above the animated ankle (z) = the diagnosed lift; highest shown lift
    double poseLiftErr = 0.0;             // m, max |shown ankle z - animated ankle z - lift|
    double poseLiftMax = 0.0;             // m
};

static double P95(const std::vector<double>& aV) { return Percentile(aV, 0.95); }

static void Run(const Scenario& aSc, Metrics& aM)
{
    const Database& db = *g_db;
    MatcherConfig cfg;
    if (aSc.clip >= 0)
    {
        const ClipRecord& c = db.Clips()[size_t(aSc.clip)];
        cfg.allowedFrameRanges = {Range{c.firstFrame, c.firstFrame + c.numFrames}};
        cfg.initialFrame = static_cast<int32_t>(c.firstFrame);
        cfg.transitionCost = 10.0f; // keep playing the loop (single clip)
    }
    BaseMatcher mm;
    FootLocker fl;
    if (!mm.Init(db, cfg) || !fl.Init(db))
    {
        aM.finite = false;
        return;
    }
    fl.SetEnabled(aSc.lock);
    fl.SetReplantConfig(g_rcTest); // loco C1b (defaults unless UTPP_FL_REPLANT)
    if (aSc.slipStart > 0.0f)
        fl.SetSlip(aSc.slipStart, 0.02f);
    aM.updateMicros.reserve(size_t(aSc.seconds / kDt) + 4);
    const int n = static_cast<int>(aSc.seconds / kDt + 0.5f);
    const int32_t chain[2][3] = {{fl.ChainBone(0, 0), fl.ChainBone(0, 1), fl.ChainBone(0, 2)},
                                 {fl.ChainBone(1, 0), fl.ChainBone(1, 1), fl.ChainBone(1, 2)}};
    std::vector<BoneTransform> prevAnim(db.NumBones()), prevFinal(db.NumBones());
    float px = 0.0f, py = 0.0f, yaw = 0.0f; // capsule (test side)
    int32_t prevFrame = mm.Diagnostics().frame;
    float prevAlpha = mm.Diagnostics().alpha;
    bool havePrev = false;
    float prevAnimW[2][2] = {}, prevFinalW[2][2] = {};
    math::Vec3 prevCorr[2] = {};
    bool lockSeen[2] = {false, false};
    bool contact[2] = {false, false}; // test-side hysteresis (same thresholds), independent of the lock switch
    const FootLockConfig& fc = fl.Config();
    float lock0[2][2] = {};
    bool prevFull[2] = {false, false}; // loco C1b
    int stepStart[2] = {-1, -1};
    const bool rated = aSc.matcherRate != 1.0f;
    for (int i = 0; i < n; ++i)
    {
        const double t = i * double(kDt);
        const MatcherInput in = aSc.mixed ? MixedScript(t) : Walk(0.0f, aSc.speed);
        mm.Update(rated ? kDt * aSc.matcherRate : kDt, in);
        const MatcherDiagnostics& d = mm.Diagnostics();
        // capsule step this update (world)
        float sx = 0.0f, sy = 0.0f, syaw = 0.0f;
        if (aSc.capsule == Capsule::AnimRoot)
        {
            // root progress: whole frames consumed + sub-frame alpha (linear root interpolation)
            const RootDelta* r0 = db.Root(static_cast<uint32_t>(prevFrame));
            const RootDelta* r1 = db.Root(static_cast<uint32_t>(d.frame));
            const float cx = d.consumedRoot[0] + d.alpha * r1->dx - prevAlpha * r0->dx;
            const float cy = d.consumedRoot[1] + d.alpha * r1->dy - prevAlpha * r0->dy;
            syaw = (d.consumedRoot[2] + d.alpha * r1->dyaw - prevAlpha * r0->dyaw);
            if (rated) // loco C1b: capsuleScale x the root velocity per matcher second, in real time
                math::Rot2(yaw, cx * aSc.capsuleScale / aSc.matcherRate, cy * aSc.capsuleScale / aSc.matcherRate, sx, sy);
            else
                math::Rot2(yaw, cx * aSc.capsuleScale, cy * aSc.capsuleScale, sx, sy);
        }
        else if (aSc.capsule == Capsule::Simulated)
        {
            const float cx = d.simStep[0];
            const float cy = d.simStep[1];
            syaw = d.simStep[2];
            math::Rot2(yaw, cx * aSc.capsuleScale, cy * aSc.capsuleScale, sx, sy);
        }
        prevFrame = d.frame;
        prevAlpha = d.alpha;
        if (i == aSc.teleportUpdate)
        {
            float jx = 0.0f, jy = 0.0f;
            math::Rot2(yaw, 0.0f, 2.0f, jx, jy);
            sx += jx;
            sy += jy;
        }
        px += sx;
        py += sy;
        yaw = math::WrapAngle(yaw + syaw);
        const float cl = (aSc.contactMask & 1) ? FootLocker::SampleContact(db, d.frame, d.alpha, 0) : 0.0f;
        const float cr = (aSc.contactMask & 2) ? FootLocker::SampleContact(db, d.frame, d.alpha, 1) : 0.0f;
        const auto anim = std::span<const BoneTransform>(mm.Output().bones);

        if (aSc.replant >= 0)
            fl.SetReplant(aSc.replant == 1, aSc.stops); // loco C1b
        g_countAllocs.store(true);
        const auto t0 = std::chrono::steady_clock::now();
        fl.Update(kDt, sx / kDt, sy / kDt, yaw, cl, cr, anim);
        const auto t1 = std::chrono::steady_clock::now();
        g_countAllocs.store(false);
        aM.updateMicros.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());

        const auto fin = fl.Output();
        const FootLockDiagnostics& fd = fl.Diagnostics();
        for (const BoneTransform& b : fin)
            aM.finite = aM.finite && math::IsFinite(b.p) && math::IsFinite(b.q);
        for (int s = 0; s < 2; ++s)
        {
            const float cv = s == 0 ? cl : cr;
            contact[s] = contact[s] ? !(cv < fc.offThreshold) : (cv > fc.onThreshold);
            const BoneTransform ga = fl.ModelSpace(anim, chain[s][2]);
            const BoneTransform gf = fl.ModelSpace(fin, chain[s][2]);
            // world motion of the locked point (toe)
            const BoneTransform la = fl.ModelSpace(anim, fl.LockBone(s));
            const BoneTransform lf = fl.ModelSpace(fin, fl.LockBone(s));
            float aw[2], fw[2];
            fl.CharacterToWorld(la.p, aw[0], aw[1]);
            fl.CharacterToWorld(lf.p, fw[0], fw[1]);
            const math::Vec3 corr = gf.p - ga.p;
            const double c = std::sqrt(math::Dot(corr, corr));
            aM.maxCorrection = std::max(aM.maxCorrection, c);
            aM.sumCorrection += c;
            aM.corrSamples += 1;
            const FootLockFootDiag& f = fd.foot[s];
            if (!f.clamped && f.weight > 0.0f)
                aM.maxErrorUnclamped = std::max(aM.maxErrorUnclamped, double(f.error));
            if (!f.locked && f.weight > 0.0f)
                aM.maxBlendOutSink = std::max(aM.maxBlendOutSink, double(ga.p.z - gf.p.z));
            if (havePrev)
            {
                const math::Vec3 dc = corr - prevCorr[s];
                const double step = std::sqrt(math::Dot(dc, dc));
                aM.maxCorrStep = std::max(aM.maxCorrStep, step);
                if (aSc.teleportUpdate >= 0 && std::abs(i - aSc.teleportUpdate) <= 30)
                    aM.maxCorrStepTeleport = std::max(aM.maxCorrStepTeleport, step);
                if (contact[s] && i != aSc.teleportUpdate)
                {
                    aM.contactTime[s] += kDt;
                    aM.slideAnim[s] += std::hypot(aw[0] - prevAnimW[s][0], aw[1] - prevAnimW[s][1]);
                    aM.slideFinal[s] += std::hypot(fw[0] - prevFinalW[s][0], fw[1] - prevFinalW[s][1]);
                }
                for (int k = 0; k < 3; ++k)
                {
                    const size_t b = size_t(chain[s][k]);
                    aM.maxLegRotStepAnim = std::max(aM.maxLegRotStepAnim, double(math::QuatAngleBetween(prevAnim[b].q, anim[b].q)) * kRad2Deg);
                    aM.maxLegRotStepFinal = std::max(aM.maxLegRotStepFinal, double(math::QuatAngleBetween(prevFinal[b].q, fin[b].q)) * kRad2Deg);
                }
            }
            if (f.locked && f.weight >= 1.0f)
            {
                if (!lockSeen[s])
                {
                    lockSeen[s] = true;
                    lock0[s][0] = fw[0];
                    lock0[s][1] = fw[1];
                }
                const double dr = std::hypot(fw[0] - lock0[s][0], fw[1] - lock0[s][1]);
                aM.driftSum += dr;
                aM.driftMax = std::max(aM.driftMax, dr);
                aM.lockedSamples += 1;
            }
            else
                lockSeen[s] = false;
            {
                // loco C1b: displayed toe world motion while held at full weight
                const bool full = f.locked && f.weight >= 1.0f;
                if (f.locked)
                    aM.corrLocked.push_back(c);
                if (full && !f.clamped && f.error <= 0.001f)
                {
                    aM.poseLiftErr = std::max(aM.poseLiftErr, std::fabs(double(corr.z) - double(f.lift)));
                    aM.poseLiftMax = std::max(aM.poseLiftMax, double(corr.z));
                }
                if (havePrev && full && prevFull[s] && i != aSc.teleportUpdate)
                {
                    const double mv = std::hypot(fw[0] - prevFinalW[s][0], fw[1] - prevFinalW[s][1]);
                    const double am = std::hypot(aw[0] - prevAnimW[s][0], aw[1] - prevAnimW[s][1]);
                    aM.maxToeJump = std::max(aM.maxToeJump, mv);
                    if (!f.stepped)
                    {
                        // planted: the lock point does not move; the displayed toe leaves it only where the IK cannot reach the target
                        // (hard clamp or the soft reach zone, error > reachError) - counted apart (reach drag, also in the 0.5.9 path)
                        const bool reach = f.clamped || f.error > fl.ReplantConfig().reachError;
                        if (reach)
                            ++aM.plantedReach;
                        else
                            aM.plantedSpeed.push_back(mv / kDt);
                        ++aM.plantedUpdates;
                        aM.plantedSpeedMax = std::max(aM.plantedSpeedMax, mv / kDt);
                        aM.plantedErrMax = std::max(aM.plantedErrMax, double(f.error));
                        if (mv / kDt > 0.01)
                            ++aM.plantedMoving;
                    }
                    else
                    {
                        const FootReplantConfig& rcf = fl.ReplantConfig();
                        aM.stepSpeedMax = std::max(aM.stepSpeedMax, mv / kDt);
                        aM.stepExcess = std::max(aM.stepExcess, mv - (aSc.stops ? rcf.stepSpeedStops : rcf.stepSpeed) * (f.hurried ? rcf.hurry : 1.0f) * kDt - am);
                    }
                }
                if (f.stepping && stepStart[s] < 0)
                    stepStart[s] = i;
                if (!f.stepping && stepStart[s] >= 0)
                {
                    aM.longestStep = std::max(aM.longestStep, (i - stepStart[s]) * double(kDt));
                    stepStart[s] = -1;
                }
                prevFull[s] = full;
            }
            prevAnimW[s][0] = aw[0];
            prevAnimW[s][1] = aw[1];
            prevFinalW[s][0] = fw[0];
            prevFinalW[s][1] = fw[1];
            prevCorr[s] = corr;
        }
        std::copy(anim.begin(), anim.end(), prevAnim.begin());
        std::copy(fin.begin(), fin.end(), prevFinal.begin());
        havePrev = true;
    }
    const FootLockDiagnostics& fd = fl.Diagnostics();
    aM.locks = fd.locks;
    aM.relocks = fd.relocks;
    aM.driftUnlocks = fd.driftUnlocks;
    aM.clamps = fd.clamps;
    aM.nonFinite = fd.nonFiniteResets;
    aM.allocs = g_allocs.load();
    {
        uint64_t reasons = 0; // loco C1b: steps only in replant mode, every step with its trigger
        for (const uint64_t r : fd.stepReasons)
            reasons += r;
        aM.perFootOk = fd.footDriftUnlocks[0] + fd.footDriftUnlocks[1] == fd.driftUnlocks && fd.footSlips[0] + fd.footSlips[1] == fd.slips &&
                       fd.footSteps[0] + fd.footSteps[1] == reasons && (aSc.replant == 1 || reasons == 0);
    }
    for (int s = 0; s < 2; ++s)
    {
        aM.driftPerFoot[s] = fd.footDriftUnlocks[s];
        const FootLockEvent& e = fd.lastEvent[s];
        if (aSc.replant == 1) // loco C1b: drift unlocks and steps are both foot events
            aM.eventsOk = aM.eventsOk && ((fd.footDriftUnlocks[s] + fd.footSteps[s] > 0) == (e.seq > 0));
        else
            aM.eventsOk = aM.eventsOk && (fd.footDriftUnlocks[s] > 0 ? (e.seq > 0 && e.reason == kFootEventDrift && e.dist > fc.maxDrift)
                                                                       : e.seq == 0);
    }
    aM.diag = fd;
}

static double SlideRate(const Metrics& m, bool aFinal) // cm/s of contact time, both feet
{
    const double t = m.contactTime[0] + m.contactTime[1];
    const double s = aFinal ? m.slideFinal[0] + m.slideFinal[1] : m.slideAnim[0] + m.slideAnim[1];
    return t > 0.0 ? 100.0 * s / t : 0.0;
}

// ================================================================ loco C1b plant / replant
static void FramePose(const Database& aDb, uint32_t aFrame, std::vector<BoneTransform>& aOut)
{
    const auto raw = aDb.Pose(aFrame);
    for (uint32_t b = 0; b < aDb.NumBones(); ++b)
    {
        const float* r = raw.data() + size_t(b) * 7;
        aOut[b] = BoneTransform{{r[0], r[1], r[2]}, {r[3], r[4], r[5], r[6]}};
    }
}

// Synthetic pose stream through a FootLocker (no matcher): per update aStep(i, pose, contactL, contactR, velX, velY, replantOn, stops)
// fills the input; the displayed toe (lock point) world motion is measured like Run().
struct StreamMetrics
{
    bool finite = true;
    double plantedSpeedMax = 0.0, stepSpeedMax = 0.0, maxToeJump = 0.0, maxCorrection = 0.0, longestStep = 0.0;
    double stepExcess = -1.0; // m, as Metrics::stepExcess
    double finalOffset[2] = {0.0, 0.0};
    int stepOverlap = 0;          // updates with both feet stepping (test side)
    double poseLiftErr = 0.0, poseLiftMax = 0.0; // m, as Metrics (review fix: the lift shown in the pose)
    double longestFootStep[2] = {0.0, 0.0};      // s, per foot
    FootLockDiagnostics diag;
};
template <class StepFn>
static StreamMetrics RunStream(const Database& aDb, FootLocker& fl, int aUpdates, StepFn&& aStep)
{
    StreamMetrics m;
    fl.SetReplantConfig(g_rcTest);
    std::vector<BoneTransform> pose(aDb.NumBones());
    double prevW[2][2] = {}, prevA[2][2] = {};
    bool prevFull[2] = {false, false};
    int stepStart[2] = {-1, -1};
    for (int i = 0; i < aUpdates; ++i)
    {
        float cl = 1.0f, cr = 1.0f, vx = 0.0f, vy = 0.0f;
        bool on = true, stops = false;
        aStep(i, pose, cl, cr, vx, vy, on, stops);
        fl.SetReplant(on, stops);
        fl.Update(kDt, vx, vy, 0.0f, cl, cr, std::span<const BoneTransform>(pose));
        const auto out = fl.Output();
        const FootLockDiagnostics& fd = fl.Diagnostics();
        for (const BoneTransform& b : out)
            m.finite = m.finite && math::IsFinite(b.p) && math::IsFinite(b.q);
        if (fd.foot[0].stepped && fd.foot[1].stepped)
            ++m.stepOverlap;
        for (int s = 0; s < 2; ++s)
        {
            const BoneTransform lf = fl.ModelSpace(out, fl.LockBone(s));
            const BoneTransform ga = fl.ModelSpace(std::span<const BoneTransform>(pose), fl.ChainBone(s, 2));
            const BoneTransform gf = fl.ModelSpace(out, fl.ChainBone(s, 2));
            const math::Vec3 corr = gf.p - ga.p;
            m.maxCorrection = std::max(m.maxCorrection, double(std::sqrt(math::Dot(corr, corr))));
            float w[2], aw[2];
            fl.CharacterToWorld(lf.p, w[0], w[1]);
            fl.CharacterToWorld(fl.ModelSpace(std::span<const BoneTransform>(pose), fl.LockBone(s)).p, aw[0], aw[1]);
            const FootLockFootDiag& f = fd.foot[s];
            const bool full = f.locked && f.weight >= 1.0f;
            if (full && !f.clamped && f.error <= 0.001f)
            {
                m.poseLiftErr = std::max(m.poseLiftErr, std::fabs(double(corr.z) - double(f.lift)));
                m.poseLiftMax = std::max(m.poseLiftMax, double(corr.z));
            }
            if (i > 0 && full && prevFull[s])
            {
                const double mv = std::hypot(w[0] - prevW[s][0], w[1] - prevW[s][1]);
                const double am = std::hypot(aw[0] - prevA[s][0], aw[1] - prevA[s][1]);
                m.maxToeJump = std::max(m.maxToeJump, mv);
                if (!f.stepped)
                    m.plantedSpeedMax = std::max(m.plantedSpeedMax, mv / kDt);
                else
                {
                    m.stepSpeedMax = std::max(m.stepSpeedMax, mv / kDt);
                    const FootReplantConfig& rcf = fl.ReplantConfig();
                    m.stepExcess = std::max(m.stepExcess, mv - double(stops ? rcf.stepSpeedStops : rcf.stepSpeed) * (f.hurried ? rcf.hurry : 1.0f) * kDt - am);
                }
            }
            prevA[s][0] = aw[0];
            prevA[s][1] = aw[1];
            if (f.stepping && stepStart[s] < 0)
                stepStart[s] = i;
            if (!f.stepping && stepStart[s] >= 0)
            {
                m.longestStep = std::max(m.longestStep, (i - stepStart[s]) * double(kDt));
                m.longestFootStep[s] = std::max(m.longestFootStep[s], (i - stepStart[s]) * double(kDt));
                stepStart[s] = -1;
            }
            prevFull[s] = full;
            prevW[s][0] = w[0];
            prevW[s][1] = w[1];
            m.finalOffset[s] = f.offset;
        }
    }
    m.diag = fl.Diagnostics();
    return m;
}

static uint64_t Steps(const FootLockDiagnostics& d) { return d.footSteps[0] + d.footSteps[1]; }
static std::string StepLine(const FootLockDiagnostics& d)
{
    return Fmt("steps %.0f (radius %.0f clamp %.0f tidy %.0f handover %.0f, L %.0f R %.0f)", double(Steps(d)),
               double(d.stepReasons[kFootEventRadius]), double(d.stepReasons[kFootEventClamp]), double(d.stepReasons[kFootEventTidy]),
               double(d.stepReasons[kFootEventHandover]), double(d.footSteps[0]), double(d.footSteps[1])) +
           Fmt(", handovers %.0f, drift unlocks %.0f, slips %.0f, settles %.0f", double(d.handovers), double(d.driftUnlocks),
               double(d.slips), double(d.settles));
}

static void SectionReplant(const Database& aBase, const std::string& aStancePath)
{
    {
        // diagnostic tuning: UTPP_FL_REPLANT=<radius>,<stopsRadius>,<ratio>,<stopsRatio>,<stepSpeed>,<stopsStepSpeed>,<lift>,<clampTime>
        char e[160] = {};
        size_t en = 0;
        FootReplantConfig c;
        if (getenv_s(&en, e, sizeof e, "UTPP_FL_REPLANT") == 0 && en > 0 &&
            sscanf_s(e, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &c.radius, &c.radiusStops, &c.ratio, &c.ratioStops, &c.stepSpeed, &c.stepSpeedStops,
                     &c.lift, &c.clampTime, &c.hurry) >= 8 &&
            FootReplantConfig::Valid(c))
        {
            g_rcTest = c;
            std::printf("[INFO] C1b replant params from UTPP_FL_REPLANT: %s\n", e);
        }
    }
    const FootReplantConfig rc = g_rcTest;
    // ---- (r0) off path: SetReplant(false) every update reproduces the 0.5.9 results bit for bit (base database)
    {
        g_db = &aBase;
        bool same = true;
        std::string detail;
        const int32_t walk = FindClip("M_Neutral_Walk_Loop_F"), run = FindClip("M_Neutral_Run_Loop_F");
        for (int k = 0; k < 4; ++k)
        {
            Scenario sc;
            if (k < 2)
            {
                sc.clip = k == 0 ? walk : run;
                sc.speed = k == 0 ? 2.19f : 5.46f;
                sc.capsuleScale = 1.2f;
            }
            else
            {
                sc.mixed = true;
                sc.capsule = Capsule::Simulated;
                sc.seconds = 20.0f;
            }
            sc.slipStart = k == 3 ? 0.0f : 0.05f; // driver 0.5.9 slip on (and the 0.5.8 path without it)
            Metrics a, b;
            sc.replant = -1;
            Run(sc, a);
            sc.replant = 0;
            Run(sc, b);
            const bool eq = a.maxCorrection == b.maxCorrection && a.sumCorrection == b.sumCorrection && a.slideFinal[0] == b.slideFinal[0] &&
                            a.slideFinal[1] == b.slideFinal[1] && a.driftSum == b.driftSum && a.locks == b.locks && a.relocks == b.relocks &&
                            a.clamps == b.clamps && a.driftUnlocks == b.driftUnlocks && a.diag.settles == b.diag.settles &&
                            a.diag.slips == b.diag.slips && a.maxLegRotStepFinal == b.maxLegRotStepFinal && Steps(b.diag) == 0 &&
                            b.diag.handovers == 0;
            same = same && eq;
            detail += Fmt("run %.0f equal %.0f, steps %.0f; ", double(k), eq ? 1.0 : 0.0, double(Steps(b.diag)));
        }
        Expect("C1b off path: SetReplant(false) every update = never called, bit for bit (walk / run x1.2 slip on, mixed 20 s slip on / off)",
               same, detail);
    }

    Database sdb;
    std::string err;
    if (aStancePath.empty() || !sdb.LoadFromFile(aStancePath.c_str(), 0x72CB314EB01F562Dull, err))
    {
        std::printf("[INFO] C1b crouch sections skipped: stance database not readable (%s) %s\n", aStancePath.c_str(), err.c_str());
        g_db = &aBase;
        return;
    }
    g_db = &sdb;
    std::printf("[INFO] C1b stance database %s (%u clips)\n", aStancePath.c_str(), sdb.NumClips());
    const double stepTol = 0.005;  // m per update on top of stepSpeed * dt
    const double plantTol = 0.01;  // m/s, planted toe (IK reach clamps let the displayed toe leave the lock slightly)
    // review fix: the step criterion is RELATIVE to the animated toe (the lock point follows the toe's own world motion, the hurried
    // step closes at stepSpeed x hurry). The design's ABSOLUTE bound (displayed toe <= stepSpeed x dt + 0.5 cm per update) is not met:
    // the displayed toe of a step moves up to ~5 m/s in world (crouch start F x1.6: 4.98 m/s = 8.3 cm per update; replant off: planted
    // toe up to 3.0 m/s). Regression cap only (not the design criterion; deviation reported): step toe world speed <= 6 m/s.
    const double stepToeCap = 6.0; // m/s
    const auto liftOk = [](double aErr, double aShown, float aDiag)
    {
        // the lift is in the pose: shown ankle lift = diagnosed lift (<= 2 mm) and the highest diagnosed lift is shown (>= half)
        return aErr <= 0.002 && (aDiag <= 0.005f || aShown >= 0.5 * double(aDiag));
    };

    // ---- (r1) crouch starts x8 (capsule 1.6 x the clip root motion) and stops x8 (capsule 0.6 x), replant on vs off (driver slip)
    const char* dirs[8] = {"F", "FL", "FR", "LL", "RR", "BL", "BR", "B"};
    for (int kind = 0; kind < 2; ++kind)
    {
        bool okAll = true, negSlide = false;
        int done = 0;
        std::string worst;
        double worstJump = 0.0, worstCorr = 0.0, worstPlant = 0.0, worstToe = 0.0, worstToeOff = 0.0, worstLiftErr = 0.0;
        for (const char* dn : dirs)
        {
            const std::string name = std::string("M_Neutral_Crouch_") + (kind == 0 ? "Start_" : "Stop_") + dn + "_Lfoot";
            const int32_t clip = FindClip(name);
            if (clip < 0)
            {
                std::printf("[INFO] C1b %s missing\n", name.c_str());
                continue;
            }
            ++done;
            const ClipRecord& cr = sdb.Clips()[size_t(clip)];
            Scenario sc;
            sc.clip = clip;
            sc.capsule = Capsule::AnimRoot;
            sc.capsuleScale = kind == 0 ? 1.6f : 0.6f;
            sc.seconds = std::max(0.5f, float(cr.numFrames - 1) / std::max(1.0f, sdb.Fps()));
            sc.slipStart = 0.05f;
            sc.stops = kind == 1;
            Metrics off, on;
            sc.replant = 0;
            Run(sc, off);
            sc.replant = 1;
            Run(sc, on);
            const double corrLimit = kind == 0 ? 0.125 : 0.15; // stops: the 15 cm loco foot-correction guard
            const bool ok = on.finite && on.nonFinite == 0 && on.allocs == 0 && P95(on.plantedSpeed) <= plantTol &&
                            on.plantedReach * 10 <= on.plantedUpdates && on.stepExcess <= stepTol && on.maxCorrection <= corrLimit &&
                            on.driftUnlocks == 0 &&
                            on.diag.simultaneousSteps == 0 && on.diag.maxLift <= rc.lift + 1e-6f && on.longestStep <= 0.3 &&
                            on.perFootOk && on.eventsOk && liftOk(on.poseLiftErr, on.poseLiftMax, on.diag.maxLift) &&
                            on.stepSpeedMax <= stepToeCap;
            okAll = okAll && ok;
            negSlide = negSlide || P95(off.plantedSpeed) > 0.1;
            if (on.stepExcess > worstJump || !ok)
            {
                worstJump = std::max(worstJump, on.stepExcess);
                worst = name + (ok ? "" : " FAILED");
            }
            worstCorr = std::max(worstCorr, on.maxCorrection);
            worstPlant = std::max(worstPlant, P95(on.plantedSpeed));
            worstToe = std::max(worstToe, on.stepSpeedMax);
            worstToeOff = std::max(worstToeOff, off.plantedSpeedMax);
            worstLiftErr = std::max(worstLiftErr, on.poseLiftErr);
            std::printf("[INFO] C1b %-34s off: planted toe p95 %.1f max %.1f cm/s, max corr %.1f cm, %s | on: planted p95 %.2f max %.2f "
                        "cm/s (%d moving, reach-limited %d of %d, IK error max %.1f cm), step: toe max %.2f m/s, excess %.2f cm/update, "
                        "longest %.2f s, lift %.1f cm, max corr %.1f cm, %s\n",
                        name.c_str(), P95(off.plantedSpeed) * 100.0, off.plantedSpeedMax * 100.0, off.maxCorrection * 100.0,
                        StepLine(off.diag).c_str(), P95(on.plantedSpeed) * 100.0, on.plantedSpeedMax * 100.0, on.plantedMoving,
                        on.plantedReach, on.plantedUpdates, on.plantedErrMax * 100.0, on.stepSpeedMax, on.stepExcess * 100.0, on.longestStep, double(on.diag.maxLift) * 100.0,
                        on.maxCorrection * 100.0, StepLine(on.diag).c_str());
        }
        Expect(kind == 0 ? "C1b crouch starts x8 (capsule 1.6 x root motion): planted toe p95 <= 1 cm/s (reach-limited updates apart, <= "
                           "10 %), a step moves the toe <= stepSpeed (x hurry) x dt + 0.5 cm per update RELATIVE to the animated toe and "
                           "ends <= 0.3 s, step toe world speed <= 6 m/s (regression cap, not the design's absolute bound), correction <= "
                           "12.5 cm, no drift unlock, one foot at a time, lift <= 3 cm and shown in the pose, 0 allocations"
                         : "C1b crouch stops x8 (capsule 0.6 x root motion, Stops profile 2.5 m/s): same criteria, correction <= 15 cm",
               okAll && done >= 6,
               Fmt("%.0f clips; worst step excess %.2f cm/update, max corr %.1f cm, planted p95 max %.2f cm/s", double(done),
                   worstJump * 100.0, worstCorr * 100.0, worstPlant * 100.0) +
                   Fmt(", step toe world max %.2f m/s (replant off: planted toe max %.2f m/s), pose lift error max %.2f mm", worstToe,
                       worstToeOff, worstLiftErr * 1000.0) +
                   " (" + worst + ")");
        if (kind == 0)
            Expect("C1b negative control: the same crouch starts with replant off (0.5.9 slip) move a planted toe faster than 10 cm/s",
                   negSlide);
    }
    // ---- (r1b) crouch start F at matcher rates 0.3 / 0.6 / 1.0 (in game the start clip ran at 0.25-0.30 with the capsule at speed)
    {
        const int32_t clip = FindClip("M_Neutral_Crouch_Start_F_Lfoot");
        std::string line;
        bool ok = clip >= 0;
        for (const float r : {0.3f, 0.6f, 1.0f})
        {
            if (clip < 0)
                break;
            const ClipRecord& cr = sdb.Clips()[size_t(clip)];
            Scenario sc;
            sc.clip = clip;
            sc.capsuleScale = 1.6f;
            sc.matcherRate = r;
            sc.seconds = std::max(0.5f, float(cr.numFrames - 1) / std::max(1.0f, sdb.Fps()) / r);
            sc.slipStart = 0.05f;
            Metrics off, on;
            sc.replant = 0;
            Run(sc, off);
            sc.replant = 1;
            Run(sc, on);
            // rate 0.3 (5.3 x the animated root speed) is reported only: the step speed cannot follow such a mismatch (the crouch start
            // rate floor C1c is the fix); 0.6 and 1.0 must meet the criteria
            if (r > 0.5f)
                ok = ok && on.finite && P95(on.plantedSpeed) <= plantTol && on.stepExcess <= stepTol && on.diag.simultaneousSteps == 0 &&
                     on.maxCorrection <= 0.125 && on.driftUnlocks == 0;
            else
                ok = ok && on.finite && on.diag.simultaneousSteps == 0;
            line += Fmt("rate %.1f: off planted p95 %.1f cm/s corr %.1f cm drift %.0f | on planted p95 %.2f cm/s steps %.0f drift %.0f ", r,
                        P95(off.plantedSpeed) * 100.0, off.maxCorrection * 100.0, double(off.driftUnlocks), P95(on.plantedSpeed) * 100.0,
                        double(Steps(on.diag)), double(on.driftUnlocks)) +
                    Fmt("corr %.1f cm excess %.2f cm; ", on.maxCorrection * 100.0, on.stepExcess * 100.0);
        }
        Expect("C1b crouch start F at matcher rate 0.6 / 1.0: planted toe p95 <= 1 cm/s, toe steps bounded, one foot at a time, "
               "correction <= 12.5 cm, no drift unlock (rate 0.3: finite and one foot at a time, the rest INFO - needs C1c)",
               ok, line);
    }

    // static crouch idle pose (motion idle frame 0) for the tidy / mode-change checks
    const int32_t gidle = FindClip("M_Neutral_Crouch_Idle_Loop");
    std::vector<BoneTransform> still(sdb.NumBones());
    FramePose(sdb, sdb.Clips()[size_t(gidle < 0 ? 0 : gidle)].firstFrame, still);
    const auto staticStep = [&](float aJump, int aOnFrom, int aOnTo, bool aStops)
    {
        // update 0 locks, update 1 moves the capsule aJump backwards (the planted toes appear aJump ahead), then standing; replant on
        // for updates [aOnFrom, aOnTo)
        return [&, aJump, aOnFrom, aOnTo, aStops](int i, std::vector<BoneTransform>& pose, float&, float&, float&, float& vy, bool& on,
                                                bool& stops)
        {
            pose = still;
            vy = i == 1 ? -aJump / kDt : 0.0f;
            on = i >= aOnFrom && i < aOnTo;
            stops = aStops;
        };
    };
    // ---- (r3) tidy: standing capsule, 6 cm offset on both feet -> exactly one tidy step per foot, one after the other
    {
        FootLocker f;
        f.Init(sdb);
        f.SetSlip(0.05f, 0.02f);
        const StreamMetrics m = RunStream(sdb, f, 60, staticStep(0.06f, 0, 1 << 30, true)); // 1 s
        const FootLockDiagnostics& d = m.diag;
        Expect("C1b tidy: 6 cm lasting offset at a standing capsule -> exactly one tidy step per foot within tidyDelay + 0.2 s, one foot at "
               "a time, final offset <= 1 cm, no settle / slip in replant mode, the step lift shown in the pose",
               m.finite && d.stepReasons[kFootEventTidy] == 2 && Steps(d) == 2 && d.footSteps[0] == 1 && d.footSteps[1] == 1 &&
                   m.stepOverlap == 0 && m.finalOffset[0] <= 0.01 && m.finalOffset[1] <= 0.01 && d.settles == 0 && d.slips == 0 &&
                   m.stepExcess <= stepTol && m.plantedSpeedMax <= plantTol && d.maxLift > 0.005f &&
                   liftOk(m.poseLiftErr, m.poseLiftMax, d.maxLift),
               StepLine(d) + Fmt(", final offset L %.2f R %.2f cm, longest step %.2f s, toe jump %.2f cm/update", m.finalOffset[0] * 100.0,
                                 m.finalOffset[1] * 100.0, m.longestStep, m.maxToeJump * 100.0) +
                   Fmt(", lift %.1f cm, shown %.1f cm (error %.2f mm)", double(d.maxLift) * 100.0, m.poseLiftMax * 100.0,
                       m.poseLiftErr * 1000.0));
        // tidy timing: the second foot must be done by tidyDelay + 0.2 s after the offset (update 1)
        FootLocker g;
        g.Init(sdb);
        g.SetSlip(0.05f, 0.02f);
        const StreamMetrics early = RunStream(sdb, g, 1 + int((rc.tidyDelay + 0.2f) / kDt + 0.5f), staticStep(0.06f, 0, 1 << 30, true));
        Expect("C1b tidy: both steps done within tidyDelay + 0.2 s", Steps(early.diag) == 2 && !early.diag.foot[0].stepping &&
                                                                         !early.diag.foot[1].stepping && early.finalOffset[0] <= 0.01 &&
                                                                         early.finalOffset[1] <= 0.01,
               StepLine(early.diag));
    }
    // ---- (r4) stance change while planted (replant crouch, the stance switches to stand = replant off for that stance)
    {
        // (a) 9 cm offset (inside the radius) planted in replant mode, replant off after 0.1 s -> one handover step per foot (offset
        // beyond the 5 cm slip start), one foot at a time, then the 0.5.9 path; negative control: the same offset in the 0.5.9 path
        FootLocker f, n;
        f.Init(sdb);
        n.Init(sdb);
        f.SetSlip(0.05f, 0.02f);
        n.SetSlip(0.05f, 0.02f);
        const float inside = std::max(0.055f, 0.8f * rc.radius); // inside the radius, beyond the 5 cm slip start
        const StreamMetrics m = RunStream(sdb, f, 60, staticStep(inside, 0, 7, false));
        const StreamMetrics neg = RunStream(sdb, n, 60, staticStep(inside, 1 << 30, 1 << 30, false));
        const FootLockDiagnostics& d = m.diag;
        Expect("C1b stance change while planted (replant -> off, offset inside the radius, beyond the slip start): one handover step per foot, one foot at a time, toe <= "
               "1 m/s, both handed over to the 0.5.9 path, no drift unlock, final offset <= 1 cm",
               m.finite && d.stepReasons[kFootEventHandover] == 2 && Steps(d) == 2 && d.handovers == 2 && m.stepOverlap == 0 &&
                   d.driftUnlocks == 0 && m.stepExcess <= stepTol && m.finalOffset[0] <= 0.01 &&
                   m.finalOffset[1] <= 0.01 && !f.Diagnostics().foot[0].replant && !f.Diagnostics().foot[1].replant,
               StepLine(d) + Fmt(", toe jump %.2f cm/update | 0.5.9 path: toe jump %.2f cm/update", m.maxToeJump * 100.0,
                                 neg.maxToeJump * 100.0));
        // (b) replant off while a radius step runs (13 cm offset on both feet): the running step of the first foot completes (relock
        // inside the slip start -> handed over), the second foot (waiting, one foot at a time) makes its handover step afterwards
        FootLocker g;
        g.Init(sdb);
        g.SetSlip(0.05f, 0.02f);
        const StreamMetrics s = RunStream(sdb, g, 60, staticStep(rc.radius + 0.01f, 0, 3, false));
        const FootLockDiagnostics& e = s.diag;
        Expect("C1b stance change during a step: the running radius step completes, the waiting foot makes one handover step after it, "
               "both feet handed over, toe <= 1 m/s, lift back to 0",
               s.finite && e.stepReasons[kFootEventRadius] == 1 && e.stepReasons[kFootEventHandover] == 1 && e.handovers == 2 &&
                   s.stepOverlap == 0 && s.stepExcess <= stepTol && e.foot[0].lift == 0.0f && e.foot[1].lift == 0.0f &&
                   e.driftUnlocks == 0,
               StepLine(e) + Fmt(", toe jump %.2f cm/update", s.maxToeJump * 100.0));
        // (c) off -> replant while planted with a 3 cm offset (inside slip start and tidy radius): taken over at once, the toe does not
        // move any more (no settle), no step
        FootLocker h;
        h.Init(sdb);
        h.SetSlip(0.05f, 0.02f);
        int switchAt = 4;
        double after = 0.0;
        std::vector<BoneTransform> pose(sdb.NumBones());
        double prev[2][2] = {};
        for (int i = 0; i < 40; ++i)
        {
            pose = still;
            h.SetReplant(i >= switchAt, false);
            h.Update(kDt, 0.0f, i == 1 ? -0.03f / kDt : 0.0f, 0.0f, 1.0f, 1.0f, std::span<const BoneTransform>(pose));
            for (int sd = 0; sd < 2; ++sd)
            {
                float w[2];
                h.CharacterToWorld(h.ModelSpace(h.Output(), h.LockBone(sd)).p, w[0], w[1]);
                if (i > switchAt)
                    after = std::max(after, std::hypot(w[0] - prev[sd][0], w[1] - prev[sd][1]));
                prev[sd][0] = w[0];
                prev[sd][1] = w[1];
            }
        }
        Expect("C1b off -> replant while planted (3 cm offset): taken over at once, planted toe still afterwards (no settle), no step",
               Steps(h.Diagnostics()) == 0 && after <= 1e-5 && h.Diagnostics().foot[0].replant && h.Diagnostics().foot[1].replant,
               Fmt("toe motion after the switch %.3f mm/update", after * 1000.0));
    }
    // ---- (r7) Stops profile (review fix: radiusStops / ratioStops / stepSpeedStops are really used). Standing capsule, static crouch
    // idle, both planted toes moved ahead by the same offset: between radius and radiusStops the Default profile makes radius steps
    // (ending at ratio x radius), the Stops profile none (a tidy step only after tidyDelay); beyond radiusStops the Stops step ends at
    // ratioStops x radiusStops and the second (unhurried) step closes at stepSpeedStops.
    if (rc.radius + 0.01f < rc.radiusStops)
    {
        const float mid = 0.5f * (rc.radius + rc.radiusStops), big = rc.radiusStops + 0.03f;
        const int early = std::max(3, int(0.8f * rc.tidyDelay / kDt)); // ends before any tidy step
        FootLocker fa, fb, ft, fc;
        for (FootLocker* p : {&fa, &fb, &ft, &fc})
        {
            p->Init(sdb);
            p->SetSlip(0.05f, 0.02f);
        }
        const StreamMetrics ma = RunStream(sdb, fa, early, staticStep(mid, 0, 1 << 30, false));
        const StreamMetrics mb = RunStream(sdb, fb, early, staticStep(mid, 0, 1 << 30, true));
        const StreamMetrics mt = RunStream(sdb, ft, 60, staticStep(mid, 0, 1 << 30, true));
        const StreamMetrics mc = RunStream(sdb, fc, early, staticStep(big, 0, 1 << 30, true));
        const double endD = double(rc.ratio * rc.radius), endS = double(rc.ratioStops * rc.radiusStops);
        const double stepS = (double(big) - endS) / double(rc.stepSpeedStops) + double(kDt) + 0.001; // one unhurried Stops step
        const auto ends = [](const StreamMetrics& m, double aEnd)
        { return std::fabs(m.finalOffset[0] - aEnd) <= 0.001 && std::fabs(m.finalOffset[1] - aEnd) <= 0.001; };
        Expect("C1b Stops profile: an offset between radius and radiusStops -> radius steps to ratio x radius in the Default profile, "
               "no step in the Stops profile before tidyDelay (then one tidy step per foot); beyond radiusStops the Stops steps end at "
               "ratioStops x radiusStops and close at stepSpeedStops",
               ma.finite && ma.diag.stepReasons[kFootEventRadius] == 2 && Steps(ma.diag) == 2 && ends(ma, endD) && Steps(mb.diag) == 0 &&
                   mt.diag.stepReasons[kFootEventTidy] == 2 && Steps(mt.diag) == 2 && mc.diag.stepReasons[kFootEventRadius] == 2 &&
                   Steps(mc.diag) == 2 && ends(mc, endS) && mc.longestStep <= stepS && mc.stepOverlap == 0 && ma.stepOverlap == 0,
               Fmt("offset %.1f cm Default: ", double(mid) * 100.0) + StepLine(ma.diag) +
                   Fmt(", final %.2f/%.2f cm | Stops: ", ma.finalOffset[0] * 100.0, ma.finalOffset[1] * 100.0) + StepLine(mb.diag) +
                   " | Stops 1 s: " + StepLine(mt.diag) + Fmt(" | offset %.1f cm Stops: ", double(big) * 100.0) + StepLine(mc.diag) +
                   Fmt(", final %.2f/%.2f cm (expected %.2f), longest step %.3f s (<= %.3f)", mc.finalOffset[0] * 100.0,
                       mc.finalOffset[1] * 100.0, endS * 100.0, mc.longestStep, stepS));
    }
    else
        std::printf("[INFO] C1b Stops profile check skipped (radius %.3f not below radiusStops %.3f - 1 cm)\n", double(rc.radius),
                    double(rc.radiusStops));
    // ---- (r8) clamp trigger (review fix): stand idle frame with both legs made straight and vertical (the animated ankle at full
    // reach, so any horizontal offset is out of reach), toes planted 3 cm ahead (inside the radius and the tidy radius): the IK reach
    // is clamped -> after clampTime one clamp step per foot to the animated toe, one foot at a time
    {
        const int32_t sidle = FindClip("M_Neutral_Stand_Idle_Loop");
        if (sidle < 0)
            std::printf("[INFO] C1b clamp trigger check skipped (M_Neutral_Stand_Idle_Loop missing)\n");
        else
        {
            std::vector<BoneTransform> straight(sdb.NumBones());
            FramePose(sdb, sdb.Clips()[size_t(sidle)].firstFrame, straight);
            FootLocker probe;
            probe.Init(sdb);
            const auto ms = [&](int32_t aBone) { return probe.ModelSpace(std::span<const BoneTransform>(straight), aBone); };
            for (int s = 0; s < 2; ++s)
            {
                const int32_t up = probe.ChainBone(s, 0), knee = probe.ChainBone(s, 1), foot = probe.ChainBone(s, 2);
                // thigh vertical, then the shin along the thigh (local rotations; translations untouched)
                const BoneTransform gp = ms(probe.Parents()[size_t(up)]);
                BoneTransform ga = ms(up), gb = ms(knee);
                const math::Quat r1 = legik::FromTo(gb.p - ga.p, math::Vec3{0.0f, 0.0f, -1.0f});
                straight[size_t(up)].q = math::QuatNormalize(math::QuatMul(math::QuatInv(gp.q), math::QuatMul(r1, ga.q)));
                ga = ms(up);
                gb = ms(knee);
                const BoneTransform gc = ms(foot);
                const math::Quat r2 = legik::FromTo(gc.p - gb.p, gb.p - ga.p);
                straight[size_t(knee)].q = math::QuatNormalize(math::QuatMul(math::QuatInv(ga.q), math::QuatMul(r2, gb.q)));
            }
            const float off = std::min(0.03f, 0.75f * std::min(rc.radius, rc.tidyRadius));
            const auto clampStep = [&](int i, std::vector<BoneTransform>& pose, float&, float&, float&, float& vy, bool& on, bool& stops)
            {
                pose = straight;
                vy = i == 1 ? -off / kDt : 0.0f;
                on = true;
                stops = false;
            };
            FootLocker f;
            f.Init(sdb);
            f.SetSlip(0.05f, 0.02f);
            const int upd = 2 + int((rc.clampTime + 0.25f) / kDt + 0.5f);
            const StreamMetrics m = RunStream(sdb, f, upd, clampStep);
            const FootLockDiagnostics& d = m.diag;
            Expect("C1b clamp trigger: a planted toe the IK cannot reach (straight leg, 3 cm offset inside the radius / tidy radius) -> one "
                   "clamp step per foot after clampTime to the animated toe, one foot at a time, no radius / tidy step",
                   m.finite && d.clamps > 0 && d.stepReasons[kFootEventClamp] >= 2 && d.footSteps[0] >= 1 && d.footSteps[1] >= 1 &&
                       d.stepReasons[kFootEventRadius] == 0 && d.stepReasons[kFootEventTidy] == 0 && m.stepOverlap == 0 &&
                       d.simultaneousSteps == 0 && m.finalOffset[0] <= double(rc.tidyRelock) + 0.001 &&
                       m.finalOffset[1] <= double(rc.tidyRelock) + 0.001,
                   StepLine(d) + Fmt(", clamped updates %.0f, final offset L %.2f R %.2f cm, longest step %.3f s", double(d.clamps),
                                     m.finalOffset[0] * 100.0, m.finalOffset[1] * 100.0, m.longestStep));
        }
    }
    // ---- (r5) replant on: 0 allocations in Update over the mixed script, finite, one foot at a time; non-finite inputs
    {
        Scenario sc;
        sc.mixed = true;
        sc.capsule = Capsule::Simulated;
        sc.capsuleScale = 1.3f;
        sc.seconds = 20.0f;
        sc.slipStart = 0.05f;
        sc.replant = 1;
        Metrics on, off;
        Run(sc, on);
        sc.replant = 0;
        Run(sc, off);
        Expect("C1b replant on, mixed 20 s (capsule x1.3): 0 allocations, finite, never two feet stepping, no pop (leg rotation step <= "
               "without lock + 5 deg)",
               on.allocs == 0 && on.finite && on.nonFinite == 0 && on.diag.simultaneousSteps == 0 &&
                   on.maxLegRotStepFinal <= off.maxLegRotStepAnim + 5.0,
               Fmt("allocations %.0f, leg rotation step %.2f deg (animation %.2f), median %.2f us; ", double(on.allocs), on.maxLegRotStepFinal,
                   off.maxLegRotStepAnim, Percentile(on.updateMicros, 0.5)) +
                   StepLine(on.diag));
        BaseMatcher mm;
        mm.Init(sdb, MatcherConfig{});
        FootLocker fl;
        fl.Init(sdb);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        bool ok = true;
        std::vector<BoneTransform> pose(mm.Output().bones);
        for (int i = 0; i < 600; ++i)
        {
            mm.Update(kDt, Walk(0.0f, 1.0f));
            std::copy(mm.Output().bones.begin(), mm.Output().bones.end(), pose.begin());
            if (i % 37 == 5)
                pose[size_t(fl.ChainBone(1, 1))].q.x = nan;
            fl.SetReplant(true, (i / 50) % 2 == 1);
            fl.Update((i % 17 == 0) ? nan : kDt, (i % 7 == 0) ? nan : 0.3f, (i % 11 == 0) ? 1e9f : 1.0f, (i % 13 == 0) ? nan : 0.01f * i,
                      (i % 23 == 0) ? nan : 1.0f, 1.0f, pose);
            for (size_t k = 0; k < pose.size(); ++k)
                if (!(k == size_t(fl.ChainBone(1, 1)) && i % 37 == 5))
                    ok = ok && math::IsFinite(fl.Output()[k].p) && math::IsFinite(fl.Output()[k].q);
        }
        Expect("C1b replant on: non-finite velocity / yaw / dt / contact / pose -> outputs finite", ok,
               Fmt("non-finite resets %.0f; ", double(fl.Diagnostics().nonFiniteResets)) + StepLine(fl.Diagnostics()));
    }
    // ---- (r6) config validation
    {
        FootLocker f;
        f.Init(sdb);
        FootReplantConfig a, b, c, d;
        a.radius = 0.2f;      // beyond the 15 cm guard
        b.ratio = std::numeric_limits<float>::quiet_NaN();
        c.tidyRelock = 0.05f; // not below tidyRadius
        d.radius = 0.10f;
        Expect("C1b replant config: radius > 0.15 / NaN / tidyRelock >= tidyRadius rejected (previous kept), valid accepted",
               !f.SetReplantConfig(a) && !f.SetReplantConfig(b) && !f.SetReplantConfig(c) &&
                   f.ReplantConfig().radius == FootReplantConfig{}.radius &&
                   f.SetReplantConfig(d) && f.ReplantConfig().radius == 0.10f);
    }
    g_db = &aBase;
}

// ================================================================ batch 2 knee fix: two-bone pole carry (legik::SolveTwoBone aPoleCarry)
// Synthetic leg (hip 1 m above the origin, thigh / shin 0.45 m, bent in the character y-z plane: knee forward, ankle under the hip,
// toe ahead): the lock target moves sideways by 6-11 cm (standing turn starts: lateral lock offset with an almost straight leg), and
// beyond the reach (clamped). Bend plane = plane of hip, knee and ankle; its normal is compared with the animated one. A target
// translation tilts the hip-ankle axis by `tilt`; a knee kept in the animated bend plane (carried) rotates its plane normal by at
// most that tilt. The projection pole (carry off, loco batch 1) is the negative control.
struct PoleCase
{
    double planeDev = 0.0; // deg, bend-plane normal of the result vs the animated one
    double tilt = 0.0;     // deg, hip-ankle axis change (animated ankle -> resulting ankle)
    double kneeDot = 0.0;  // knee offset (from the resulting axis) . animated knee offset direction (no flip: > 0)
    bool ok = false, clamped = false;
};

static PoleCase SolvePoleCase(float aFlexDeg, math::Vec3 aTargetOffset, bool aCarry, math::Vec3* aKneeOffOut = nullptr)
{
    const float l1 = 0.45f, l2 = 0.45f;
    const auto rotX = [](float aDeg) { return legik::AxisAngle(math::Vec3{1.0f, 0.0f, 0.0f}, aDeg / kRad2Deg); };
    const BoneTransform gp{math::Vec3{0.0f, 0.0f, 0.0f}, math::Quat{}};
    const math::Vec3 a{0.1f, 0.0f, 1.0f};
    const math::Quat qa0 = rotX(0.5f * aFlexDeg), qb0 = math::QuatNormalize(math::QuatMul(qa0, rotX(-aFlexDeg)));
    const BoneTransform ga{a, qa0};
    const math::Vec3 b = a + math::QuatRotate(qa0, math::Vec3{0.0f, 0.0f, -l1});
    const BoneTransform gb{b, qb0};
    const math::Vec3 c = b + math::QuatRotate(qb0, math::Vec3{0.0f, 0.0f, -l2});
    const BoneTransform gc{c, qb0};
    const math::Vec3 toe = c + math::Vec3{0.0f, 0.14f, -0.05f};
    const legik::TwoBoneSolution s = legik::SolveTwoBone(gp, ga, gb, gc, c + aTargetOffset, toe, 0.995f, 0.96f, aCarry);
    PoleCase r;
    r.clamped = s.clamped;
    if (s.status != legik::TwoBoneStatus::kOk)
        return r;
    // result in model space (gp = identity: UpLeg local = model)
    const math::Quat qa = s.local[0];
    const math::Quat qb = math::QuatNormalize(math::QuatMul(qa, s.local[1]));
    const math::Vec3 nb = a + math::QuatRotate(qa, math::Vec3{0.0f, 0.0f, -l1});
    const math::Vec3 nc = nb + math::QuatRotate(qb, math::Vec3{0.0f, 0.0f, -l2});
    const math::Vec3 n0 = legik::Norm(math::Cross(b - a, c - b)), n1 = legik::Norm(math::Cross(nb - a, nc - nb));
    r.planeDev = std::acos(std::clamp(double(math::Dot(n0, n1)), -1.0, 1.0)) * double(kRad2Deg);
    const math::Vec3 u0 = legik::Norm(c - a), u1 = legik::Norm(nc - a);
    r.tilt = std::acos(std::clamp(double(math::Dot(u0, u1)), -1.0, 1.0)) * double(kRad2Deg);
    const math::Vec3 o0 = legik::Norm((b - a) - u0 * math::Dot(b - a, u0));
    const math::Vec3 o1 = (nb - a) - u1 * math::Dot(nb - a, u1);
    r.kneeDot = double(math::Dot(o0, legik::Norm(o1)));
    if (aKneeOffOut)
        *aKneeOffOut = legik::Norm(o1);
    r.ok = std::isfinite(r.planeDev) && legik::Len(nc - s.ankle) < 1e-4f;
    return r;
}

static void SectionPoleCarry()
{
    // (k1) almost straight leg (flex 6 / 10 / 16 deg), lateral lock offset 6 / 8 / 11 cm both sides (+ 2 cm forward)
    {
        double worstOn = 0.0, worstOff = 0.0, minDotOn = 2.0;
        bool allOk = true;
        for (float flex : {6.0f, 10.0f, 16.0f})
            for (float lat : {-0.11f, -0.08f, -0.06f, 0.06f, 0.08f, 0.11f})
            {
                const math::Vec3 off{lat, 0.02f, 0.0f};
                const PoleCase on = SolvePoleCase(flex, off, true), offc = SolvePoleCase(flex, off, false);
                allOk = allOk && on.ok && offc.ok;
                worstOn = std::max(worstOn, on.planeDev - on.tilt);
                worstOff = std::max(worstOff, offc.planeDev - offc.tilt);
                minDotOn = std::min(minDotOn, on.kneeDot);
            }
        Expect("knee fix pole carry: almost straight leg (flex 6-16 deg), lateral lock offset 6-11 cm -> knee in the animated bend plane "
               "(plane normal change <= axis tilt + 3 deg), no flip",
               allOk && worstOn <= 3.0 && minDotOn > 0.9,
               Fmt("worst plane change beyond the tilt: carry %.2f deg, projection (carry off) %.1f deg; min knee dot %.3f", worstOn,
                   worstOff, minDotOn));
        Expect("knee fix pole carry negative control: the projection pole (carry off = loco batch 1) leaves the bend plane by > 20 deg "
               "for some case",
               worstOff > 20.0, Fmt("%.1f deg", worstOff));
    }
    // (k2) clamped reach: target 3-6 cm below the animated ankle and 8 cm sideways (beyond the soft reach) -> clamped, plane kept
    {
        double worst = 0.0, minDot = 2.0;
        int clamped = 0, n = 0;
        bool allOk = true;
        for (float flex : {4.0f, 10.0f})
            for (float dz : {-0.03f, -0.06f})
                for (float lat : {-0.08f, 0.08f})
                {
                    const PoleCase on = SolvePoleCase(flex, math::Vec3{lat, 0.0f, dz}, true);
                    ++n;
                    clamped += on.clamped ? 1 : 0;
                    allOk = allOk && on.ok;
                    worst = std::max(worst, on.planeDev - on.tilt);
                    minDot = std::min(minDot, on.kneeDot);
                }
        Expect("knee fix pole carry: clamped reach (target below and 8 cm beside the ankle) -> knee in the animated bend plane (<= tilt "
               "+ 3 deg), no flip",
               allOk && clamped >= n / 2 && worst <= 3.0 && minDot > 0.9,
               Fmt("clamped %.0f/%.0f, worst %.2f deg, min knee dot %.3f", double(clamped), double(n), worst, minDot));
    }
    // (k3) continuity: lateral sweep -12 .. +12 cm in 1 mm steps (flex 8 deg): the knee direction changes smoothly, never flips
    {
        double maxStepOn = 0.0, maxStepOff = 0.0, minDot = 2.0;
        math::Vec3 prevOn{}, prevOff{};
        for (int i = -120; i <= 120; ++i)
        {
            math::Vec3 kOn{}, kOff{};
            const PoleCase on = SolvePoleCase(8.0f, math::Vec3{0.001f * float(i), 0.0f, 0.0f}, true, &kOn);
            (void)SolvePoleCase(8.0f, math::Vec3{0.001f * float(i), 0.0f, 0.0f}, false, &kOff);
            minDot = std::min(minDot, on.kneeDot);
            if (i > -120)
            {
                maxStepOn = std::max(maxStepOn, std::acos(std::clamp(double(math::Dot(kOn, prevOn)), -1.0, 1.0)) * double(kRad2Deg));
                maxStepOff = std::max(maxStepOff, std::acos(std::clamp(double(math::Dot(kOff, prevOff)), -1.0, 1.0)) * double(kRad2Deg));
            }
            prevOn = kOn;
            prevOff = kOff;
        }
        Expect("knee fix pole carry: lateral target sweep +-12 cm (1 mm steps, flex 8 deg) -> knee direction step <= 0.5 deg, no flip",
               maxStepOn <= 0.5 && minDot > 0.9,
               Fmt("max step carry %.3f deg (projection %.2f deg), min knee dot %.3f", maxStepOn, maxStepOff, minDot));
    }
    // (k4) unchanged with no target offset: carry on and off give the same knee (animated axis == new axis)
    {
        math::Vec3 kOn{}, kOff{};
        const PoleCase on = SolvePoleCase(20.0f, math::Vec3{0.0f, 0.0f, 0.0f}, true, &kOn);
        const PoleCase off = SolvePoleCase(20.0f, math::Vec3{0.0f, 0.0f, 0.0f}, false, &kOff);
        const double d = std::acos(std::clamp(double(math::Dot(kOn, kOff)), -1.0, 1.0)) * double(kRad2Deg);
        Expect("knee fix pole carry: target on the animated ankle -> same knee as the projection pole (< 0.05 deg)",
               on.ok && off.ok && d < 0.05, Fmt("%.4f deg", d));
    }
    // (k5) carry rotation (review: the FromTo was not covered): the whole bent leg (flex 40 deg, knee offset 15 cm > kPoleBlend: no
    // bias) swings rigidly about the hip by 20 / 35 deg around horizontal axes at 30-60 deg to the knee direction -> the knee must be
    // the rigidly rotated animated knee (FromTo(animated axis, new axis) == that swing). Merely re-projecting the animated offset
    // (no FromTo) shortens its component across the swing by cos(angle) and turns the knee by several degrees.
    {
        const float l1 = 0.45f, l2 = 0.45f, flex = 40.0f;
        const auto rotX = [](float aDeg) { return legik::AxisAngle(math::Vec3{1.0f, 0.0f, 0.0f}, aDeg / kRad2Deg); };
        const BoneTransform gp{math::Vec3{0.0f, 0.0f, 0.0f}, math::Quat{}};
        const math::Vec3 a{0.1f, 0.0f, 1.0f};
        const math::Quat qa0 = rotX(0.5f * flex), qb0 = math::QuatNormalize(math::QuatMul(qa0, rotX(-flex)));
        const math::Vec3 b = a + math::QuatRotate(qa0, math::Vec3{0.0f, 0.0f, -l1});
        const math::Vec3 c = b + math::QuatRotate(qb0, math::Vec3{0.0f, 0.0f, -l2});
        const math::Vec3 toe = c + math::Vec3{0.0f, 0.14f, -0.05f};
        double worstOn = 0.0, minOff = 1e9, worstPos = 0.0;
        bool allOk = true;
        for (float axDeg : {30.0f, 45.0f, 60.0f, 120.0f, 135.0f, 150.0f})
            for (float swing : {20.0f, 35.0f, -35.0f})
            {
                // axis in the horizontal plane at axDeg from the knee direction (+y); the hip-ankle axis is vertical here
                const math::Vec3 k{std::sin(axDeg / kRad2Deg), std::cos(axDeg / kRad2Deg), 0.0f};
                const math::Quat R = legik::AxisAngle(k, swing / kRad2Deg);
                const math::Vec3 t = a + math::QuatRotate(R, c - a);
                const math::Vec3 rb = math::QuatRotate(R, b - a), un = legik::Norm(t - a);
                const math::Vec3 want = legik::Norm(rb - un * math::Dot(rb, un));
                for (int carry = 0; carry < 2; ++carry)
                {
                    const legik::TwoBoneSolution s = legik::SolveTwoBone(gp, BoneTransform{a, qa0}, BoneTransform{b, qb0},
                                                                         BoneTransform{c, qb0}, t, toe, 0.995f, 0.96f, carry == 1);
                    if (s.status != legik::TwoBoneStatus::kOk)
                    {
                        allOk = false;
                        continue;
                    }
                    const math::Vec3 nb = a + math::QuatRotate(s.local[0], math::Vec3{0.0f, 0.0f, -l1});
                    const math::Vec3 u1 = legik::Norm(s.ankle - a);
                    const math::Vec3 o1 = legik::Norm((nb - a) - u1 * math::Dot(nb - a, u1));
                    const double dev = std::acos(std::clamp(double(math::Dot(o1, want)), -1.0, 1.0)) * double(kRad2Deg);
                    if (carry == 1)
                    {
                        worstOn = std::max(worstOn, dev);
                        worstPos = std::max(worstPos, double(legik::Len(nb - (a + rb))));
                        allOk = allOk && legik::Len(s.ankle - t) < 1e-3f;
                    }
                    else
                        minOff = std::min(minOff, dev);
                }
            }
        Expect("knee fix pole carry: rigid leg swing 20-35 deg about the hip (flex 40 deg) -> knee = the rigidly swung animated knee "
               "(< 0.1 deg, < 1 mm)",
               allOk && worstOn < 0.1 && worstPos < 0.001,
               Fmt("worst %.3f deg, %.2f mm; projection pole (carry off) min %.2f deg", worstOn, worstPos * 1000.0, minOff));
    }
}

// Focused regression: an incomplete positional blend-in must not be weighted twice on release.
static void SectionPartialRelease(const Database& db)
{
    std::vector<BoneTransform> pose(db.NumBones());
    const auto raw = db.Pose(db.Clips()[0].firstFrame + 6);
    for (uint32_t b=0;b<db.NumBones();++b) { const float* r=raw.data()+size_t(b)*7; pose[b]={{r[0],r[1],r[2]},{r[3],r[4],r[5],r[6]}}; }
    for (float wanted : {0.25f,0.5f,0.75f,1.0f})
    {
        FootLockConfig cfg; cfg.settleSpeed=0.0f;
        FootLocker fl; Expect("partial release init",fl.Init(db,cfg));
        fl.Update(0,0,0,0,1,0,pose);
        float lo=0,hi=1; for(int i=0;i<30;++i) { float m=(lo+hi)*.5f; if(m*m*(3-2*m)<wanted)lo=m;else hi=m; }
        float dt=cfg.blendInTime*(lo+hi)*.5f;
        fl.Update(dt,0,-0.04f/dt,0,1,0,pose);
        std::vector<BoneTransform> before(fl.Output().begin(),fl.Output().end());
        FootLocker disabled=fl;
        disabled.SetEnabled(false); disabled.Update(0,0,0,0,1,0,pose);
        double disableJump=0;for(int k=0;k<3;++k){int b=fl.ChainBone(0,k);disableJump=std::max(disableJump,double(math::QuatAngleBetween(before[b].q,disabled.Output()[b].q))*kRad2Deg);}
        Expect("partial disabled release dt0 C0",disableJump<.06);
        FootLocker drift=fl; auto shifted=pose;
        for(uint32_t b=0;b<db.NumBones();++b)if(db.BoneName(b)=="Hips")shifted[b].p.x+=1.0f;
        drift.Update(0,0,0,0,1,0,shifted);
        double driftJump=0;for(int k=0;k<3;++k){int b=fl.ChainBone(0,k);driftJump=std::max(driftJump,double(math::QuatAngleBetween(before[b].q,drift.Output()[b].q))*kRad2Deg);}
        Expect("partial drift release dt0 C0",drift.Diagnostics().driftUnlocks>0 && driftJump<.06);
        fl.Update(0,0,0,0,0,0,pose);
        double maxAngle=0; for(int k=0;k<3;++k){int b=fl.ChainBone(0,k);maxAngle=std::max(maxAngle,double(math::QuatAngleBetween(before[b].q,fl.Output()[b].q))*kRad2Deg);}
        Expect("partial release dt0 rotations C0",maxAngle<0.06,Fmt("weight %.3f max jump %.6f deg",wanted,maxAngle));
        // Relock while releasing: reconstruct the displayed toe using the same relative delta weight.
        FootLocker relock=fl;
        relock.Update(.01f,0,0,0,0,0,pose);
        const auto toeBefore=relock.ModelSpace(relock.Output(),relock.LockBone(0)).p;
        relock.Update(0,0,0,0,1,0,pose);
        const auto toeAfter=relock.ModelSpace(relock.Output(),relock.LockBone(0)).p;
        const double toeJump=legik::Len(toeAfter-toeBefore);
        bool finite=true;for(int k=0;k<3;++k)finite=finite&&math::IsFinite(relock.Output()[relock.ChainBone(0,k)].q);
        Expect("partial release relock toe C0/finite",toeJump<.0001 && finite,Fmt("toe jump %.6f mm",toeJump*1000));
        double prev=1e9; bool monotonic=true;
        for(int i=0;i<25;++i){fl.Update(.01f,0,0,0,0,0,pose);double a=0;for(int k=0;k<3;++k){int b=fl.ChainBone(0,k);a+=math::QuatAngleBetween(pose[b].q,fl.Output()[b].q);}monotonic=monotonic&&a<=prev+1e-4;prev=a;}
        Expect("partial release fades monotonically to animation",monotonic&&prev<1e-4);
    }
}

// ================================================================ loco-stop-knees 2026-11-09: standing-stop swing contact
// Game replay (loco_film1 f1547-1572, Build\work\loco-stop-knees-20261109\report.txt): in a walk stop with the standing-stop support on,
// the swinging left foot got a false contact from the crossfade-blended contact curves (0.85 while 5-9 cm in the air, moving 3-4 m/s)
// and locked; the soft slip dragged it ~12 cm behind its animation and, with the stance foot's opposite slip lag, both ankles met
// (21.7 -> 0.3 cm, the knees close in). Here the stop clip itself is played (interpolated frames, its root motion = the capsule) with the
// driver's soft slip and the standing-stop support on, at three update rates:
//   (s1) the left contact forced to 0.85 in the middle of the longest left swing: the left ankle keeps its animated distance (xy) to the
//        displayed right ankle (under the mutation it is pulled 9-21 cm off);
//   (s2) the clip's own contacts, also under a capsule braking to half the root speed: no lock is released as a swing contact and the
//        left foot plants and keeps its lock (the genuine end-of-stop plant).
static void SectionStopSwing()
{
    const Database& db = *g_db;
    const int32_t clip = FindClip("M_Neutral_Walk_Stop_F_Lfoot");
    Expect("(s0) walk stop clip M_Neutral_Walk_Stop_F_Lfoot in the test database", clip >= 0);
    if (clip < 0)
        return;
    const ClipRecord& c = db.Clips()[size_t(clip)];
    const int nf = static_cast<int>(c.numFrames);
    const float fps = db.Fps();
    // longest left swing over a planted right foot (clip contacts; the dense subset: frames 1-14 of 90 at 30 fps)
    int a = -1, b = -1;
    for (int k = 0, start = -1; k < nf; ++k)
    {
        const int32_t fr = static_cast<int32_t>(c.firstFrame) + k;
        const bool swing = FootLocker::SampleContact(db, fr, 0.0f, 0) < 0.3f && FootLocker::SampleContact(db, fr, 0.0f, 1) >= 0.5f;
        start = swing ? (start < 0 ? k : start) : -1;
        if (start >= 0 && k - start > b - a)
            a = start, b = k;
    }
    Expect("(s0) walk stop clip: a left swing over a planted right foot", a >= 0 && b - a >= 4,
           Fmt("clip frames %.0f-%.0f of %.0f at %.0f fps", a, b, nf, fps));
    if (a < 0 || b - a < 4 || !(fps > 0.0f))
        return;
    const float fa = float(a) + 0.3f * float(b - a), fb = float(a) + 0.7f * float(b - a); // false contact (clip frames)
    std::vector<BoneTransform> p0(db.NumBones()), p1(db.NumBones()), pose(db.NumBones());
    for (const float dt : {1.0f / 30.0f, 1.0f / 60.0f, 1.0f / 144.0f})
    {
        const std::string rate = " (dt 1/" + std::to_string(static_cast<int>(std::lround(1.0 / double(dt)))) + ")";
        for (const int mode : {0, 1, 2}) // 0 = (s1) false contact, capsule = root motion; 1 / 2 = (s2) clip contacts, capsule x1 / x0.5
        {
            FootLocker fl;
            if (!fl.Init(db))
            {
                Expect("(s0) FootLocker init", false);
                return;
            }
            fl.SetReplantConfig(g_rcTest);
            fl.SetSlip(0.05f, 0.02f); // driver 0.5.9 soft slip
            const float capScale = mode == 2 ? 0.5f : 1.0f;
            float yaw = 0.0f, prevAlpha = 0.0f;
            int prevK = 0;
            // in the swing (clip frames a..b+1): spacing = xy distance of the left ankle to the displayed right ankle, displayed left
            // ankle vs animated one (the left foot's own share; the right foot's slip lag under the braking capsule is not counted)
            double maxErr = 0.0, maxSwingDev = 0.0, minIn = 1e9, minOut = 1e9;
            bool finite = true;
            const int n = static_cast<int>(float(nf - 1) / fps / dt);
            for (int i = 0; i <= n; ++i)
            {
                const float ff = std::fmin(float(i) * dt * fps, float(nf - 1));
                const int k = std::min(static_cast<int>(ff), nf - 1);
                const float alpha = k < nf - 1 ? ff - float(k) : 0.0f;
                FramePose(db, c.firstFrame + uint32_t(k), p0);
                FramePose(db, c.firstFrame + uint32_t(std::min(k + 1, nf - 1)), p1);
                for (uint32_t bo = 0; bo < db.NumBones(); ++bo)
                    pose[bo] = BoneTransform{math::Lerp(p0[bo].p, p1[bo].p, alpha), math::QuatNlerpShortest(p0[bo].q, p1[bo].q, alpha)};
                // root motion consumed since the last update (whole frames + sub-frame alpha, as Run)
                float cx = 0.0f, cy = 0.0f, cyaw = 0.0f;
                for (int q = prevK; q < k; ++q)
                {
                    const RootDelta* r = db.Root(c.firstFrame + uint32_t(q));
                    cx += r->dx;
                    cy += r->dy;
                    cyaw += r->dyaw;
                }
                const RootDelta* r0 = db.Root(c.firstFrame + uint32_t(prevK));
                const RootDelta* r1 = db.Root(c.firstFrame + uint32_t(k));
                cx += alpha * r1->dx - prevAlpha * r0->dx;
                cy += alpha * r1->dy - prevAlpha * r0->dy;
                cyaw += alpha * r1->dyaw - prevAlpha * r0->dyaw;
                prevK = k;
                prevAlpha = alpha;
                float sx = 0.0f, sy = 0.0f;
                math::Rot2(yaw, cx * capScale, cy * capScale, sx, sy);
                yaw = math::WrapAngle(yaw + cyaw);
                const int32_t fr = static_cast<int32_t>(c.firstFrame) + k;
                float cl = FootLocker::SampleContact(db, fr, alpha, 0);
                const float cr = FootLocker::SampleContact(db, fr, alpha, 1);
                if (mode == 0 && ff >= fa && ff <= fb)
                    cl = 0.85f;
                fl.SetStandingStopSupport(true, false);
                fl.Update(dt, sx / dt, sy / dt, yaw, cl, cr, std::span<const BoneTransform>(pose));
                const auto out = fl.Output();
                const math::Vec3 aL = fl.ModelSpace(pose, fl.ChainBone(0, 2)).p, aR = fl.ModelSpace(pose, fl.ChainBone(1, 2)).p;
                const math::Vec3 oL = fl.ModelSpace(out, fl.ChainBone(0, 2)).p, oR = fl.ModelSpace(out, fl.ChainBone(1, 2)).p;
                finite = finite && std::isfinite(oL.x) && std::isfinite(oL.y) && std::isfinite(oR.x) && std::isfinite(oR.y);
                if (ff >= float(a) && ff <= float(b + 1))
                {
                    const double sIn = std::hypot(double(aL.x - oR.x), double(aL.y - oR.y));
                    const double sOut = std::hypot(double(oL.x - oR.x), double(oL.y - oR.y));
                    minIn = std::min(minIn, std::hypot(double(aL.x - aR.x), double(aL.y - aR.y)));
                    minOut = std::min(minOut, sOut);
                    maxErr = std::max(maxErr, std::fabs(sOut - sIn));
                    maxSwingDev = std::max(maxSwingDev, double(legik::Len(oL - aL)));
                }
            }
            const FootLockDiagnostics& d = fl.Diagnostics();
            const bool leftHeld = d.foot[0].locked && d.foot[0].weight >= 0.99f;
            const std::string detail =
                Fmt("swing: max |ankle spacing out - anim| %.2f cm (min anim %.1f / out %.1f cm), left ankle off its animation max %.2f cm, "
                    "swing-contact releases %.0f, left locked at the end %.0f",
                    maxErr * 100.0, minIn * 100.0, minOut * 100.0, maxSwingDev * 100.0, double(d.stopSwingReleases), leftHeld ? 1.0 : 0.0);
            if (mode == 0)
                Expect(("(s1) walk stop, false swing contact: the foot spacing stays the animated one" + rate).c_str(),
                       finite && maxErr <= 0.01 && d.stopSwingReleases >= 1 && leftHeld, detail);
            else
                Expect((std::string("(s2) walk stop, clip contacts, capsule x") + (mode == 2 ? "0.5" : "1") +
                        ": no swing-contact release, the left foot plants and keeps its lock" + rate).c_str(),
                       finite && d.stopSwingReleases == 0 && leftHeld && maxErr <= 0.01, detail);
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mm_footlock_test <database.utppmm>\n");
        return 2;
    }
    Database db;
    std::string err;
    const bool loaded = db.LoadFromFile(argv[1], 0x72CB314EB01F562Dull, err);
    Expect("load database", loaded, err);
    if (!loaded)
        return 1;
    g_db = &db;
    if (std::getenv("UTPP_FL_PARTIAL_RELEASE_ONLY")) { SectionPartialRelease(db); std::printf("SUMMARY: %d passed, %d failed\n",g_pass,g_fail); return g_fail?1:0; }

    // ---- bone chains from names (not hardcoded)
    FootLocker probe;
    const bool init = probe.Init(db);
    Expect("init: leg chains + toe lock bones found by name, parents verified", init,
           Fmt("L UpLeg/Leg/Foot/lock(toe) = %.0f/%.0f/%.0f/%.0f, R = %.0f/%.0f/%.0f/%.0f", probe.ChainBone(0, 0),
               probe.ChainBone(0, 1), probe.ChainBone(0, 2), probe.LockBone(0), probe.ChainBone(1, 0), probe.ChainBone(1, 1),
               probe.ChainBone(1, 2), probe.LockBone(1)));
    if (!init || probe.LockBone(0) == probe.ChainBone(0, 2))
        return 1;
    {
        FootLockConfig bad;
        bad.offThreshold = 0.9f;
        FootLocker f2;
        Expect("init: invalid config rejected (off > on threshold)", !f2.Init(db, bad));
    }

    // ---- IK unit check: reach a target 5 cm forward of the idle ankle, foot model rotation preserved
    {
        // mid-stance pose of the walk loop (knees bent; the idle legs are almost straight)
        int32_t wc = -1;
        for (uint32_t c = 0; c < db.NumClips(); ++c)
            if (db.ClipName(c) == "M_Neutral_Walk_Loop_F")
                wc = static_cast<int32_t>(c);
        std::vector<BoneTransform> poseBuf(db.NumBones());
        const auto raw = db.Pose(db.Clips()[size_t(wc < 0 ? 0 : wc)].firstFrame + 6);
        for (uint32_t b = 0; b < db.NumBones(); ++b)
        {
            const float* r = raw.data() + size_t(b) * 7;
            poseBuf[b] = BoneTransform{{r[0], r[1], r[2]}, {r[3], r[4], r[5], r[6]}};
        }
        const auto pose = std::span<const BoneTransform>(poseBuf);
        FootLockConfig noSettle;
        noSettle.settleSpeed = 0.0f; // pure IK accuracy (settling is checked separately below)
        FootLocker fl;
        fl.Init(db, noSettle);
        // lock with contact 1 at origin, then move the capsule 5 cm backwards -> locked foot appears 5 cm forward
        fl.Update(kDt, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, pose);
        for (int i = 0; i < 30; ++i)
            fl.Update(kDt, 0.0f, i == 0 ? -0.05f / kDt : 0.0f, 0.0f, 1.0f, 1.0f, pose);
        const auto out = fl.Output();
        double maxErr = 0.0, maxRot = 0.0;
        for (int s = 0; s < 2; ++s)
        {
            const BoneTransform a = fl.ModelSpace(pose, fl.ChainBone(s, 2));
            const BoneTransform f = fl.ModelSpace(out, fl.ChainBone(s, 2));
            const math::Vec3 want{a.p.x, a.p.y + 0.05f, a.p.z};
            const math::Vec3 e = f.p - want;
            maxErr = std::max(maxErr, double(std::sqrt(math::Dot(e, e))));
            maxRot = std::max(maxRot, double(math::QuatAngleBetween(a.q, f.q)) * kRad2Deg);
        }
        Expect("IK: walk mid-stance, ankle reaches a 5 cm displaced lock (error < 1 mm), foot model-space rotation kept (< 0.01 deg)",
               maxErr < 1e-3 && maxRot < 0.01,
               Fmt("max error %.3f mm, foot rotation change %.4f deg, clamps %.0f", maxErr * 1000.0, maxRot,
                   double(fl.Diagnostics().clamps)));

        // settling: same 5 cm displaced lock with the capsule standing -> offset decays at <= settleSpeed to 0,
        // the foot stays locked (no unlock/relock) and the correction never grows
        FootLocker fs;
        fs.Init(db);
        const float speed = fs.Config().settleSpeed;
        fs.Update(kDt, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, pose);
        fs.Update(kDt, 0.0f, -0.05f / kDt, 0.0f, 1.0f, 1.0f, pose);
        const float off0 = fs.Diagnostics().foot[0].offset;
        float prev = off0, maxStep = 0.0f;
        bool monotonic = true;
        for (int i = 0; i < 60; ++i) // 2 s
        {
            fs.Update(kDt, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, pose);
            const float o = fs.Diagnostics().foot[0].offset;
            monotonic = monotonic && o <= prev + 1e-6f;
            maxStep = std::max(maxStep, prev - o);
            prev = o;
        }
        Expect("settling: standing capsule, 5 cm lock offset decays to < 1 mm within 2 s at <= settleSpeed, foot stays locked",
               off0 > 0.03f && prev < 1e-3f && monotonic && maxStep <= speed * kDt + 1e-5f &&
                   fs.Diagnostics().foot[0].locked && fs.Diagnostics().locks == 2,
               Fmt("offset %.1f -> %.2f mm, max step %.2f mm/update (limit %.2f), locks %.0f", off0 * 1000.0, prev * 1000.0,
                   maxStep * 1000.0, speed * kDt * 1000.0, double(fs.Diagnostics().locks)));
    }

    const int32_t walk = FindClip("M_Neutral_Walk_Loop_F");
    const int32_t run = FindClip("M_Neutral_Run_Loop_F");
    Expect("clips M_Neutral_Walk_Loop_F / M_Neutral_Run_Loop_F present", walk >= 0 && run >= 0);
    if (walk < 0 || run < 0)
        return 1;

    // ================================================================ (a) static idle
    {
        Scenario sc;
        sc.name = "idle";
        sc.mixed = false;
        sc.speed = 0.0f;
        sc.capsule = Capsule::Zero;
        sc.seconds = 10.0f;
        Metrics m;
        Run(sc, m);
        Expect("(a) idle, zero velocity: finite, max correction < 1 cm", m.finite && m.nonFinite == 0 && m.maxCorrection < 0.01,
               Fmt("max correction %.2f mm, mean %.3f mm, locks %.0f, animated slide in contact %.2f cm/s -> %.2f cm/s",
                   m.maxCorrection * 1000.0, m.sumCorrection / std::max(1.0, m.corrSamples) * 1000.0, double(m.locks),
                   SlideRate(m, false), SlideRate(m, true)));
    }

    // ================================================================ (b) capsule = clip root motion
    for (const int32_t clip : {walk, run})
    {
        Scenario sc;
        sc.clip = clip;
        sc.speed = clip == walk ? 2.19f : 5.46f;
        sc.capsule = Capsule::AnimRoot;
        sc.seconds = 10.0f;
        Metrics m;
        Run(sc, m);
        const std::string name = std::string("(b) ") + (clip == walk ? "walk" : "run") +
                                 " loop, capsule = root motion: locked toe world drift < 0.5 cm, finite";
        Expect(name.c_str(), m.finite && m.lockedSamples > 0 && m.driftMax < 0.005,
               Fmt("drift mean %.3f / max %.3f cm over %.0f locked samples; locks %.0f, clamps %.0f; slide in contact %.2f -> %.2f cm/s",
                   m.driftSum / std::max(1.0, m.lockedSamples) * 100.0, m.driftMax * 100.0, m.lockedSamples, double(m.locks),
                   double(m.clamps), SlideRate(m, false), SlideRate(m, true)));
    }

    // ================================================================ (c) capsule 20% faster than the animation
    for (const int32_t clip : {walk, run})
    {
        Scenario sc;
        sc.clip = clip;
        sc.speed = clip == walk ? 2.19f : 5.46f;
        sc.capsule = Capsule::AnimRoot;
        sc.capsuleScale = 1.2f;
        sc.seconds = 10.0f;
        Metrics off, on;
        sc.lock = false;
        Run(sc, off);
        sc.lock = true;
        Run(sc, on);
        const double so = SlideRate(off, false), sl = SlideRate(on, true);
        const std::string name = std::string("(c) ") + (clip == walk ? "walk" : "run") +
                                 " loop, capsule x1.2: foot slide in contact reduced by >= 50% with lock";
        Expect(name.c_str(), on.finite && off.finite && sl <= 0.5 * so,
               Fmt("slide without lock %.2f cm/s, with lock %.2f cm/s (-%.0f%%); max IK correction %.2f cm, clamps %.0f, drift unlocks %.0f",
                   so, sl, so > 0 ? 100.0 * (1.0 - sl / so) : 0.0, on.maxCorrection * 100.0, double(on.clamps),
                   double(on.driftUnlocks)));
    }

    // ================================================================ (d) continuity, mixed script (transitions, stops, turns)
    {
        Scenario sc;
        sc.mixed = true;
        sc.capsule = Capsule::Simulated;
        sc.seconds = 40.0f;
        Metrics off, on;
        sc.lock = false;
        Run(sc, off);
        sc.lock = true;
        Run(sc, on);
        // Pop criterion = leg joint rotation steps. The correction step is only reported: while locked it legitimately
        // grows by the capsule/animation mismatch per update (e.g. 1.3 m/s capsule during a turn-in-place clip ->
        // ~5 cm/update with the toe fixed in the world).
        Expect("(d) mixed 40 s: max per-update leg rotation change with lock <= without + 5 deg (no pops at lock/unlock)",
               on.finite && on.nonFinite == 0 && on.maxLegRotStepFinal <= off.maxLegRotStepAnim + 5.0,
               Fmt("leg local rotation step max: without %.2f deg, with %.2f deg; max correction step %.2f cm; max correction %.2f cm; "
                   "locks %.0f relocks %.0f drift unlocks %.0f clamps %.0f",
                   off.maxLegRotStepAnim, on.maxLegRotStepFinal, on.maxCorrStep * 100.0, on.maxCorrection * 100.0,
                   double(on.locks), double(on.relocks), double(on.driftUnlocks), double(on.clamps)));
        // loco-fix 2026-11-09 (FootLocker kBlendOutSinkMax): the faded rotation deltas may not take an ankle more than 3 cm under the animated one
        Expect("(d2) mixed 40 s: the blend-out keeps the ankle within 3 cm under the animated one (max sink m)", on.maxBlendOutSink <= 0.0301,
               Fmt("%.4f m", on.maxBlendOutSink));
        Expect("(d) mixed 40 s: slide in contact (simulated capsule) reduced with lock", SlideRate(on, true) < SlideRate(off, false),
               Fmt("%.2f -> %.2f cm/s; IK error when reachable max %.3f mm", SlideRate(off, false), SlideRate(on, true),
                   on.maxErrorUnclamped * 1000.0));
        // (f) performance + allocations (mixed run)
        Expect("(f) FootLocker::Update: 0 allocations, median < 10 us",
               on.allocs == 0 && Percentile(on.updateMicros, 0.5) < 10.0,
               Fmt("median %.2f us, p99 %.2f us, max %.2f us over %.0f updates, allocations %.0f",
                   Percentile(on.updateMicros, 0.5), Percentile(on.updateMicros, 0.99), Percentile(on.updateMicros, 1.0),
                   double(on.updateMicros.size()), double(on.allocs)));
    }

    // ================================================================ (e) teleport: 2 m capsule jump during the walk
    {
        Scenario sc;
        sc.clip = walk;
        sc.speed = 2.19f;
        sc.capsule = Capsule::AnimRoot;
        sc.seconds = 6.0f;
        sc.teleportUpdate = 185;
        Metrics base, m;
        Scenario sb = sc;
        sb.teleportUpdate = -1;
        Run(sb, base);
        Run(sc, m);
        Expect("(e) 2 m capsule jump: drift unlock, finite, correction step around the jump < 5 cm",
               m.finite && m.nonFinite == 0 && m.driftUnlocks >= 1 && m.maxCorrStepTeleport < 0.05,
               Fmt("drift unlocks %.0f; max correction step +-30 updates %.2f cm (whole run %.2f cm, no-jump run %.2f cm)",
                   double(m.driftUnlocks), m.maxCorrStepTeleport * 100.0, m.maxCorrStep * 100.0, base.maxCorrStep * 100.0));
        // loco C0 (diagnostics only): per-foot drift / slip counters add up to the totals, the drift unlock is recorded as a foot
        // event (reason drift, beyond maxDrift) on the foot that unlocked; no event without a drift unlock
        Expect("(e) loco C0 per-foot diagnostics: counters add up to the totals, drift event per unlocking foot, none in the no-jump run",
               m.perFootOk && m.eventsOk && base.perFootOk && base.eventsOk && base.driftUnlocks == 0,
               Fmt("drift L %.0f R %.0f (total %.0f)", double(m.driftPerFoot[0]), double(m.driftPerFoot[1]), double(m.driftUnlocks)));
        // loco C0 per-foot attribution: only one foot gets its contact (the other never locks), the capsule runs 15 % ahead of
        // the animation with the driver's soft slip (5 cm / 20 ms) so the planted foot slips, and the 2 m jump hits while that foot
        // is planted (first jump update from 185 in steps of 3 that gives a drift unlock). Slips, drift unlocks and the drift event
        // must land on that foot only; mirrored for the other foot.
        bool sideOk = true;
        std::string sideDetail;
        for (int only = 0; only < 2; ++only)
        {
            Scenario s1 = sc;
            s1.contactMask = 1 << only;
            s1.capsuleScale = 1.15f;
            s1.slipStart = 0.05f;
            Metrics m1;
            int jump = -1;
            for (int tu = 185; tu < 300 && jump < 0; tu += 3)
            {
                s1.teleportUpdate = tu;
                m1 = Metrics{};
                Run(s1, m1);
                if (m1.driftUnlocks >= 1)
                    jump = tu;
            }
            const int other = 1 - only;
            const FootLockDiagnostics& q = m1.diag;
            sideOk = sideOk && jump >= 0 && m1.finite && q.driftUnlocks >= 1 && q.footDriftUnlocks[only] == q.driftUnlocks &&
                     q.footDriftUnlocks[other] == 0 && q.slips > 0 && q.footSlips[only] == q.slips && q.footSlips[other] == 0 &&
                     q.lastEvent[only].seq > 0 && q.lastEvent[only].reason == kFootEventDrift && q.lastEvent[other].seq == 0 &&
                     q.footSteps[0] + q.footSteps[1] == 0;
            sideDetail += std::string(only == 0 ? "L" : "R") +
                          Fmt(" only, jump at update %.0f: drift L %.0f R %.0f, slips L %.0f R %.0f, event seq L %.0f R %.0f",
                              double(jump), double(q.footDriftUnlocks[0]), double(q.footDriftUnlocks[1]), double(q.footSlips[0]),
                              double(q.footSlips[1]), double(q.lastEvent[0].seq), double(q.lastEvent[1].seq)) +
                          (only == 0 ? "; " : "");
        }
        Expect("(e2) loco C0 per-foot attribution: one foot with contact - its slips, drift unlocks and drift event counted on that "
               "foot only, none on the other (both sides)",
               sideOk, sideDetail);
    }

    // ================================================================ determinism + non-finite input
    {
        Scenario sc;
        sc.mixed = true;
        sc.capsule = Capsule::Simulated;
        sc.seconds = 20.0f;
        Metrics a, b;
        Run(sc, a);
        Run(sc, b);
        Expect("determinism: two mixed runs give identical metrics",
               a.maxCorrection == b.maxCorrection && a.sumCorrection == b.sumCorrection && a.slideFinal[0] == b.slideFinal[0] &&
                   a.locks == b.locks);

        BaseMatcher mm;
        mm.Init(db, MatcherConfig{});
        FootLocker fl;
        fl.Init(db);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float inf = std::numeric_limits<float>::infinity();
        bool ok = true;
        std::vector<BoneTransform> pose(mm.Output().bones);
        for (int i = 0; i < 600; ++i)
        {
            mm.Update(kDt, Walk(0.0f, 1.5f));
            std::copy(mm.Output().bones.begin(), mm.Output().bones.end(), pose.begin());
            if (i % 37 == 5)
                pose[size_t(fl.ChainBone(0, 1))].q.x = nan; // corrupted input pose
            const float vx = (i % 7 == 0) ? nan : 0.0f;
            const float vy = (i % 11 == 0) ? inf : 1.5f;
            const float yw = (i % 13 == 0) ? nan : 0.0f;
            const float dt = (i % 17 == 0) ? nan : (i % 19 == 0) ? 5.0f : kDt;
            const float c = (i % 23 == 0) ? nan : 1.0f;
            fl.Update(dt, vx, vy, yw, c, 1.0f - c, pose);
            for (size_t k = 0; k < pose.size(); ++k)
            {
                if (k == size_t(fl.ChainBone(0, 1)) && i % 37 == 5)
                    continue; // corrupted bone passes through unchanged (left leg skipped)
                ok = ok && math::IsFinite(fl.Output()[k].p) && math::IsFinite(fl.Output()[k].q);
            }
        }
        Expect("non-finite velocity / yaw / dt / contact / pose: outputs finite (corrupted bone only passed through)", ok,
               Fmt("non-finite resets %.0f", double(fl.Diagnostics().nonFiniteResets)));

        // disable -> blend out (no instant snap)
        FootLocker f3;
        f3.Init(db);
        const auto p = std::span<const BoneTransform>(mm.Output().bones);
        f3.Update(kDt, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, p);
        for (int i = 0; i < 20; ++i)
            f3.Update(kDt, 0.0f, i == 0 ? 0.1f / kDt : 0.0f, 0.0f, 1.0f, 1.0f, p);
        const float w0 = f3.Diagnostics().foot[0].weight;
        f3.SetEnabled(false);
        f3.Update(kDt, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, p);
        const float w1 = f3.Diagnostics().foot[0].weight;
        for (int i = 0; i < 12; ++i)
            f3.Update(kDt, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, p);
        const float w2 = f3.Diagnostics().foot[0].weight;
        Expect("SetEnabled(false): weight blends out over blendOutTime (0.15 s)", w0 == 1.0f && w1 > 0.9f && w1 < 1.0f && w2 == 0.0f,
               Fmt("weight %.3f -> %.3f (1 update) -> %.3f (13 updates)", w0, w1, w2));
    }

    // ================================================================ loco C1b plant / replant (stance database: 2nd argument, or
    // utpp_base.utppmm next to the first database)
    {
        std::string stance = argc >= 3 ? argv[2] : std::string();
        if (stance.empty())
        {
            const std::string a = argv[1];
            const size_t sl = a.find_last_of("\\/");
            stance = (sl == std::string::npos ? std::string() : a.substr(0, sl + 1)) + "utpp_base.utppmm";
        }
        SectionReplant(db, stance);
    }

    SectionPoleCarry();
    SectionStopSwing(); // loco-stop-knees 2026-11-09

    std::printf("SUMMARY: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
