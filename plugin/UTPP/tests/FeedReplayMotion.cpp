// Offline replay of recorded in-game motion-matching feeds through the v2 stack (report 7 stage 10.1 p.4).
// Input: .feed files from tools/replay/extract_feeds.py (60 Hz reconstruction of MMTRACE lines: TRANS on every v1
// transition + ~1 Hz tick; NOT true per-frame feeds). Every frame goes through adapter::MakeMotionFeed -> MapToChooser ->
// MotionAdapter::Update (ChooserMatcher + chooser + QuinticInertializer + SamplePose) with the woman v2 database.
// Query trajectory (default, finding 8): adapter::BuildQueryTrajectory = the motion trajectory MapToChooser keeps in MapState
// (MotionTrajectory: capsule history + CMC ground prediction + gravity / floor pass + facing) sampled with GetSampleAtTime at
// every trajectory time of the database schemas (MotionAdapter.hpp). isStarting / isPivoting / gait come from MapToChooser.
// --legacy-spring (comparison only): the old replay trajectory - past = integrated recorded capsule velocity in game
// m/s (no speed map), future = damped spring (half-life 0.25 s [P]) from the current velocity to the stick direction at
// the speed the log shows 0.5 s later [P], facing damped toward the movement.
// Hard checks (exit 1): no pose / non-finite cost / jump to a pose outside the chooser-selected databases / continuing
// pose outside the chooser set under an interrupt mode where UE 5.8 would drop it (anything but DoNotInterrupt,
// PoseSearchLibrary.cpp IsForceInterrupt 326-344 + MotionMatch 1340-1373) / MatchResult.continuingOutsideSet not
// matching the frame / median search >= 0.3 ms. Continuing picks outside the set under DoNotInterrupt (motion
// Get_MMInterruptMode on frames without a MovementMode / MovementState / Gait / Stance change) are reported only.
// Everything else (pick rate, continuing share, v1 vs v2 class family) is reported only.
// Pose (replay5): every frame the MatchResult goes through motionpose::PoseEvaluator (SamplePose -> inertializer -> overlay
// layering -> grip IK) with overlay Default (unarmed; the feed has no weapon column), style Feminine / Masculine by body.
// Hard check: Evaluate returns a finite pose on every frame. Reported only (no threshold): foot sliding while the playing
// row's CONT contact says planted (>= 0.5 [P]: midpoint of the 0..1 motion contact curve) on this and the previous frame -
// LeftFoot / RightFoot model space (FK from Root, Trajectory excluded, FORMAT.txt "Coordinate conventions") placed in the
// world at the integrated feed capsule position and (a) the actor yaw MapToChooser used, (b) the replay root yaw (OffsetRoot
// Accumulate); per-bone angular step (bones >= 2) max / p99 per frame; pops = steps above that bone's max step in the
// source clips sampled with the same SamplePose at 1/60 s (data-derived); Evaluate wall time.
// --no-g10: MatcherConfig::steadyRunLoops off (steady Run gait rules, ChooserMatcher.hpp G10), for before / after runs.
// --facing game|motion: adapter::MapConfig::facingSource (Game = feed playerYaw, MotionActor = motion CMC actor yaw, MotionAdapter.hpp).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/FootLocker.hpp"
#include "../src/mm/MotionRootFrame.hpp"
#include "../src/mm/LocomotionSignalBlend.hpp"
#include "../src/mm/MotionPose.hpp"
#include "../src/mm/Steering.hpp"

using namespace utpp::mm;
using namespace utpp::mm::motion;

namespace
{
int g_fail = 0;

// --footlock (foot slide after the runtime FootLocker stage, 2026-11-06): the leg tail of the DLL (visual root -> FootLocker) run on the
// evaluator pose of every feed frame. Options below mirror the runtime settings; defaults = the shipped runtime defaults.
struct FlOpts
{
    bool on = false;
    bool brief = false;          // --brief: only the footlock tables (no per-feed matcher report)
    int32_t mask = 2;            // --replant-mask (UTPPSetMMFootReplant): 1 stand / sprint, 2 crouch
    float slipStart = 0.05f;     // --slip <start> <halflife> (g_slipStart / g_slipHalflife)
    float slipHalflife = 0.02f;
    FootReplantConfig rc;        // --replant r rs ratio ratioS step stepS lift
    FootLockConfig lc;
    std::string label = "run";   // --label (table header)
    std::vector<std::string> extra; // --opt key=value (experimental per-class tuning, read by the tuning code)
    float Opt(const char* aKey, float aDef) const
    {
        const std::string k = std::string(aKey) + "=";
        for (const std::string& e : extra)
            if (e.compare(0, k.size(), k) == 0)
                return static_cast<float>(std::atof(e.c_str() + k.size()));
        return aDef;
    }
};
FlOpts g_fl;

struct FlAcc
{
    uint32_t n = 0, nStep = 0, nNoStep = 0, reachPre = 0, reachPost = 0;
    double sumPre = 0, sumPost = 0, sumPostNoStep = 0, maxPre = 0, maxPost = 0, sumCorr = 0, maxCorr = 0;
    std::vector<double> pre, post, corr;
};
std::map<std::string, FlAcc> g_flAcc;
// --opt clipdump=1: the same contact-segment statistics per playing clip (FLCLIP lines), for the offline clip-vs-runtime slide split.
struct FlClipAcc
{
    uint32_t n = 0;
    double sumPre = 0, sumPost = 0;
    std::vector<double> pre;
};
std::map<std::string, FlClipAcc> g_flClip;
struct FlRunTotals
{
    uint32_t frames = 0, clamps = 0, drift = 0, steps = 0, newPops = 0, popsPost = 0, popsPre = 0, simultaneous = 0, hyperPre = 0, hyperPost = 0;
    double maxCorr = 0, maxLift = 0, maxPostStepDeg = 0, maxToeMs = 0;
};
FlRunTotals g_flTot;

double FracAbove(const std::vector<double>& v, double aLimit)
{
    if (v.empty())
        return 0.0;
    size_t n = 0;
    for (double x : v)
        n += x > aLimit;
    return 100.0 * static_cast<double>(n) / static_cast<double>(v.size());
}

std::string FlKey(const std::string& aClip)
{
    auto has = [&](const char* s) { return aClip.find(s) != std::string::npos; };
    const char* gait = has("Sprint") ? "Sprint" : has("Crouch") ? "Crouch" : has("Run_") ? "Run" : has("Walk") ? "Walk" : has("Idle") ? "Idle"
                       : has("Transition") ? "Trans" : "Other";
    const char* type = has("Reface") ? "Reface" : has("Pivot") ? "Pivot" : has("TurnInPlace") ? "TIP" : has("_Start") ? "Start" : has("_Stop") ? "Stop"
                       : has("_Turn_") ? "Turn" : has("Loop") ? "Loop" : has("Idle") ? "Idle" : has("Transition") ? "Trans" : "Other";
    return std::string(gait) + "/" + type;
}
bool g_steadyRun = true; // --no-g10: MatcherConfig::steadyRunLoops off (the motion behaviour; before / after comparison)

// Rotation angle (deg) between two unit quaternions: 4 atan2(|a - s b|, |a + s b|), s = sign(a.b) (no acos floor).
double QuatAngleDeg(const math::Quat& a, const math::Quat& b)
{
    const double d = static_cast<double>(a.x) * b.x + static_cast<double>(a.y) * b.y + static_cast<double>(a.z) * b.z +
                     static_cast<double>(a.w) * b.w;
    const double s = d < 0.0 ? -1.0 : 1.0;
    double m2 = 0.0, p2 = 0.0;
    const double av[4] = {a.x, a.y, a.z, a.w}, bv[4] = {b.x, b.y, b.z, b.w};
    for (int k = 0; k < 4; ++k)
    {
        m2 += (av[k] - s * bv[k]) * (av[k] - s * bv[k]);
        p2 += (av[k] + s * bv[k]) * (av[k] + s * bv[k]);
    }
    return 4.0 * std::atan2(std::sqrt(m2), std::sqrt(p2)) * 57.29577951308232;
}

struct V3d
{
    double x = 0, y = 0, z = 0;
};
struct Qd
{
    double x = 0, y = 0, z = 0, w = 1;
};
Qd Mul(const Qd& a, const Qd& b)
{
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
V3d Rot(const Qd& q, const V3d& v)
{
    // v + 2 w (u x v) + 2 u x (u x v)
    const V3d u{q.x, q.y, q.z};
    const V3d c{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};
    const V3d cc{u.y * c.z - u.z * c.y, u.z * c.x - u.x * c.z, u.x * c.y - u.y * c.x};
    return {v.x + 2 * (q.w * c.x + cc.x), v.y + 2 * (q.w * c.y + cc.y), v.z + 2 * (q.w * c.z + cc.z)};
}

// Model-space (Root frame, Trajectory bone 1 excluded) position of bone aBone (FORMAT.txt "Coordinate conventions").
V3d ModelPos(const MotionDatabase& db, const BoneTransform* aPose, uint32_t aBone)
{
    // chain from the bone up to Root, then compose root -> bone
    uint32_t chain[96];
    uint32_t n = 0;
    for (int32_t b = static_cast<int32_t>(aBone); b > 1 && n < 96; b = db.BoneParent(static_cast<uint32_t>(b)))
        chain[n++] = static_cast<uint32_t>(b);
    Qd q;
    V3d p;
    for (uint32_t k = n; k-- > 0;)
    {
        const BoneTransform& t = aPose[chain[k]];
        const V3d lp = Rot(q, V3d{t.p.x, t.p.y, t.p.z});
        p = {p.x + lp.x, p.y + lp.y, p.z + lp.z};
        q = Mul(q, Qd{t.q.x, t.q.y, t.q.z, t.q.w});
    }
    return p;
}

int32_t FindBone(const MotionDatabase& db, std::string_view aName)
{
    for (uint32_t i = 0; i < db.NumBones(); ++i)
        if (db.BoneName(i) == aName)
            return static_cast<int32_t>(i);
    return -1;
}

// Per bone: max rotation step (deg) between consecutive 1/60 s SamplePose samples over every clip of the database.
std::vector<double> SourceMaxStep(const MotionDatabase& db)
{
    const uint32_t nb = db.NumBones();
    std::vector<double> mx(nb, 0.0);
    std::vector<BoneTransform> a(nb), b(nb);
    const float fps = db.Header().fps > 0.0f ? db.Header().fps : 30.0f;
    for (uint32_t c = 0; c < db.NumClips(); ++c)
    {
        const MotionClipRecord& cr = db.Clip(c);
        if (cr.numFrames < 2)
            continue;
        const int steps = static_cast<int>(std::floor(static_cast<double>(cr.numFrames - 1) / fps * 60.0 + 1e-6));
        if (!adapter::SamplePose(db, static_cast<int32_t>(c), 0.0f, a.data(), nb))
            continue;
        for (int k = 1; k <= steps; ++k)
        {
            adapter::SamplePose(db, static_cast<int32_t>(c), static_cast<float>(k) / 60.0f, b.data(), nb);
            for (uint32_t i = 2; i < nb; ++i)
                mx[i] = std::max(mx[i], QuatAngleDeg(a[i].q, b[i].q));
            std::swap(a, b);
        }
    }
    return mx;
}

struct PoseCtx
{
    const ov::OverlayFile* ovf = nullptr;
    const ov::StyleFamilyData* style = nullptr;
    bool man = false;
    adapter::FacingSource facing = adapter::FacingSource::Game;
    bool slide = false; // --slide: MapConfig::slide (feed slide on; needs a bundle with the chooser slide section)
    std::vector<double> srcMax; // per bone, deg per 1/60 s
    int32_t footL = -1, footR = -1;
};

// fopen without C4996 on MSVC (fopen_s), plain fopen elsewhere.
FILE* OpenWrite(const char* aPath)
{
#ifdef _MSC_VER
    FILE* f = nullptr;
    return fopen_s(&f, aPath, "w") == 0 ? f : nullptr;
#else
    return std::fopen(aPath, "w");
#endif
}
#define CHECK(c, ...)                         \
    do                                        \
    {                                         \
        if (!(c))                             \
        {                                     \
            ++g_fail;                         \
            std::printf("FAIL: " __VA_ARGS__); \
            std::printf("\n");                \
        }                                     \
    } while (0)

struct Frame
{
    float t = 0, dt = 0, vx = 0, vy = 0, vz = 0, mvx = 0, mvy = 0, yawDeg = 0, camDeg = 0;
    int loco = 0, trans = 0;
    uint32_t psm = 0; // raw psmBlock; unsigned: the dual-wield flag (bit 31) makes a dump value exceed INT_MAX
    std::string baseClip;
    int baseFlags = -1;
    float baseCost = 0, baseMicros = 0;
    std::string step;
};

struct Feed
{
    std::string name;
    std::vector<Frame> fr;
    std::vector<float> px, py; // integrated capsule position per frame (world, m)
};

bool LoadFeed(const char* aPath, Feed& f)
{
    std::ifstream in(aPath);
    if (!in)
        return false;
    std::string line;
    f.name = aPath;
    const size_t sl = f.name.find_last_of("/\\");
    if (sl != std::string::npos)
        f.name = f.name.substr(sl + 1);
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream s(line);
        Frame r;
        s >> r.t >> r.dt >> r.vx >> r.vy >> r.vz >> r.mvx >> r.mvy >> r.yawDeg >> r.camDeg >> r.loco >> r.psm >> r.trans >> r.baseClip >>
            r.baseFlags >> r.baseCost >> r.baseMicros >> r.step;
        if (!s)
            return false;
        f.fr.push_back(r);
    }
    float x = 0, y = 0;
    for (const Frame& r : f.fr)
    {
        x += r.vx * r.dt;
        y += r.vy * r.dt;
        f.px.push_back(x);
        f.py.push_back(y);
    }
    return !f.fr.empty();
}

constexpr float kDeg = 3.14159265358979f / 180.0f;

std::string ClipClass(const std::string& c)
{
    auto has = [&](const char* s) { return c.find(s) != std::string::npos; };
    if (has("Transition_"))
        return "Transition";
    if (has("Reface"))
        return "Reface";
    if (has("_Pivot") || has("Pivot_"))
        return "Pivot";
    if (has("_Start") || has("Start_"))
        return "Start";
    if (has("_Stop") || has("Stop_"))
        return "Stop";
    if (has("_Turn_") || has("Turn_") || has("TurnInPlace"))
        return "Turn";
    if (has("Idle"))
        return "Idle";
    if (has("_Loop") || has("Loop_") || has("_Move_") || has("Walk") || has("Run") || has("Jog") || has("Sprint"))
        return "Loop";
    return "Other";
}

// Desired (stick) world velocity: stick x right / y forward in camera space; forward(yaw) = (-sin, cos) (integ convention).
void StickWorld(const Frame& r, float aSpeed, float& ox, float& oy)
{
    const float m = std::sqrt(r.mvx * r.mvx + r.mvy * r.mvy);
    if (m < 0.01f)
    {
        ox = oy = 0.0f;
        return;
    }
    const float c = std::cos(r.camDeg * kDeg), s = std::sin(r.camDeg * kDeg);
    const float wx = (r.mvx * c - r.mvy * s) / m, wy = (r.mvx * s + r.mvy * c) / m;
    ox = wx * aSpeed;
    oy = wy * aSpeed;
}

float Speed(const Frame& r) { return std::sqrt(r.vx * r.vx + r.vy * r.vy); }

// Trajectory over [-1.0, +1.6] s at 1/60 s (covers the schema sample times), positions relative to an arbitrary origin.
void BuildTraj(const Feed& f, size_t i, std::vector<TrajSample>& out, float& aFutVx, float& aFutVy)
{
    out.clear();
    const Frame& r = f.fr[i];
    const size_t look = std::min(f.fr.size() - 1, i + 30); // speed the log shows 0.5 s later
    float tgtSpeed = std::max(Speed(f.fr[look]), Speed(r));
    float dvx, dvy;
    StickWorld(r, tgtSpeed, dvx, dvy);
    const float halfLife = 0.25f, k = 0.69314718f / halfLife;
    for (int s = -60; s <= 96; ++s)
    {
        TrajSample ts;
        ts.t = s / 60.0f;
        if (s <= 0)
        {
            const long j = static_cast<long>(i) + s;
            const size_t jj = static_cast<size_t>(std::max(0L, j));
            ts.x = f.px[jj];
            ts.y = f.py[jj];
            ts.yaw = f.fr[jj].yawDeg * kDeg;
        }
        else
        {
            const float tau = ts.t;
            // closed form of x' = vd + (v0 - vd) e^{-k tau}
            const float e = (1.0f - std::exp(-k * tau)) / k;
            ts.x = f.px[i] + dvx * tau + (r.vx - dvx) * e;
            ts.y = f.py[i] + dvy * tau + (r.vy - dvy) * e;
            const float vx = dvx + (r.vx - dvx) * std::exp(-k * tau), vy = dvy + (r.vy - dvy) * std::exp(-k * tau);
            float yaw = r.yawDeg * kDeg;
            if (vx * vx + vy * vy > 0.04f)
            {
                const float tgt = std::atan2(-vx, vy); // forward (-sin, cos)
                const float d = math::WrapAngle(tgt - yaw);
                yaw += d * (1.0f - std::exp(-k * 1.5f * tau));
            }
            ts.yaw = yaw;
            if (s == 24)
            {
                aFutVx = vx;
                aFutVy = vy;
            }
        }
        out.push_back(ts);
    }
}

struct Stats
{
    std::vector<double> us;
    uint32_t updates = 0, noPose = 0, nonFinite = 0, notChosen = 0, continuing = 0, clipChanges = 0, jumps = 0;
    uint32_t contOutsideKept = 0, contOutsideIllegal = 0, contOutsideFlagMismatch = 0;
    uint32_t baseClipChanges = 0, famSame = 0, famCmp = 0;
    std::map<std::string, uint32_t> masks;            // "db1+db2" -> frames
    std::map<std::string, uint32_t> baseFam, motionFam;     // class family -> frames
    std::vector<std::string> timeline;                // set changes
    std::map<std::string, uint32_t> motionJumpFam;        // class family of the jumped-to clip -> jumps
    // pose quality
    uint32_t poseFrames = 0, poseFail = 0, pops = 0, planted[2] = {0, 0};
    std::vector<double> evalUs, steps;
    double maxStep = 0.0, slideActor[2] = {0, 0}, slideRoot[2] = {0, 0}, slideActorMax[2] = {0, 0};
    int32_t maxStepBone = -1;
};

double Pct(std::vector<double> v, double p)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    const size_t i = static_cast<size_t>(std::min<double>(static_cast<double>(v.size() - 1), p * static_cast<double>(v.size() - 1) + 0.5));
    return v[i];
}

std::string MaskName(const MotionDatabase& db, uint64_t m)
{
    std::string s;
    for (uint32_t d = 0; d < db.NumDatabases() && d < 64; ++d)
        if ((m >> d) & 1u)
        {
            if (!s.empty())
                s += '+';
            s += std::string(db.DbName(d));
        }
    return s.empty() ? std::string("<none>") : s;
}

// Leg tail state of one feed run (--footlock): DLL order visual root (rootframe::Processor) -> FootLocker on the evaluator pose.
struct FlState
{
    FootLocker fl;
    rootframe::Processor proc;
    locosignal::Blend blend;
    std::vector<BoneTransform> prevOut;
    bool havePrev = false, havePrevVis = false, havePrevYaw = false;
    double prevVx = 0, prevVy = 0;
    float prevYaw = 0;
    bool prevContact[2] = {false, false};
    double prevPre[2][2] = {}, prevPost[2][2] = {};
    uint64_t lastSimultaneous = 0;
    // per-frame samples of the lock bone (toe) for the segment statistics (contact segments, edge frames trimmed)
    struct Sample
    {
        double pre[2], post[2];
        bool contact, step;
        float corr;
        int key;
        int clip;
        float t, c;
    };
    std::vector<Sample> samp[2];
    std::vector<std::string> keyNames;
    std::vector<std::string> clipNames;
    int ClipId(const std::string& aClip)
    {
        for (size_t k = 0; k < clipNames.size(); ++k)
            if (clipNames[k] == aClip)
                return static_cast<int>(k);
        clipNames.push_back(aClip);
        return static_cast<int>(clipNames.size()) - 1;
    }
    int KeyId(const std::string& aKey)
    {
        for (size_t k = 0; k < keyNames.size(); ++k)
            if (keyNames[k] == aKey)
                return static_cast<int>(k);
        keyNames.push_back(aKey);
        return static_cast<int>(keyNames.size()) - 1;
    }
};

// Contact segments of the FootLocker (hysteresis state): kTrim frames (60 Hz) dropped at both ends (the clip contact curves ramp, the
// blend overlaps the neighbouring clip), segments shorter than kMinSeg after trimming ignored.
void FlFinalize(FlState& S, float aDt)
{
    constexpr size_t kTrim = 3, kMinSeg = 4;
    for (int s = 0; s < 2; ++s)
    {
        const auto& v = S.samp[s];
        size_t i = 0;
        while (i < v.size())
        {
            if (!(v[i].c >= 0.5f))
            {
                ++i;
                continue;
            }
            size_t j = i;
            while (j < v.size() && v[j].c >= 0.5f)
                ++j;
            // segment [i, j)
            if (g_fl.Opt("dbg", 0.0f) > 1.5f)
            {
                double sp = 0, mx = 0;
                size_t n = 0;
                for (size_t k = i + 1; k < j; ++k)
                {
                    const double q = std::hypot(v[k].pre[0] - v[k - 1].pre[0], v[k].pre[1] - v[k - 1].pre[1]) * 100.0 / aDt;
                    sp += q;
                    mx = std::max(mx, q);
                    ++n;
                }
                std::printf("SEG foot%d t=%.2f len=%zu key=%s meanPre=%.0f maxPre=%.0f c0=%.2f cmid=%.2f c1=%.2f\n", s, static_cast<double>(v[i].t), j - i,
                            S.keyNames[static_cast<size_t>(v[i].key)].c_str(), n ? sp / static_cast<double>(n) : 0.0, mx, static_cast<double>(v[i].c),
                            static_cast<double>(v[(i + j) / 2].c), static_cast<double>(v[j - 1].c));
            }
            if (j - i >= 2 * kTrim + kMinSeg)
                for (size_t k = i + kTrim + 1; k < j - kTrim; ++k)
                {
                    const FlState::Sample& a = v[k - 1];
                    const FlState::Sample& c = v[k];
                    const double sPre = std::hypot(c.pre[0] - a.pre[0], c.pre[1] - a.pre[1]) * 100.0 / aDt;
                    const double sPost = std::hypot(c.post[0] - a.post[0], c.post[1] - a.post[1]) * 100.0 / aDt;
                    FlAcc& acc = g_flAcc[S.keyNames[static_cast<size_t>(c.key)]];
                    {
                        FlClipAcc& ca = g_flClip[S.clipNames[static_cast<size_t>(c.clip)]];
                        ++ca.n;
                        ca.sumPre += sPre;
                        ca.sumPost += sPost;
                        ca.pre.push_back(sPre);
                    }
                    ++acc.n;
                    acc.sumPre += sPre;
                    acc.sumPost += sPost;
                    acc.maxPre = std::max(acc.maxPre, sPre);
                    acc.maxPost = std::max(acc.maxPost, sPost);
                    acc.pre.push_back(sPre);
                    acc.post.push_back(sPost);
                    acc.corr.push_back(c.corr * 100.0);
                    acc.sumCorr += c.corr * 100.0;
                    acc.maxCorr = std::max(acc.maxCorr, static_cast<double>(c.corr) * 100.0);
                    if (c.step)
                        ++acc.nStep;
                    else
                    {
                        ++acc.nNoStep;
                        acc.sumPostNoStep += sPost;
                    }
                    g_flTot.maxToeMs = std::max(g_flTot.maxToeMs, sPost / 100.0);
                }
            i = j;
        }
    }
}

double LegRatio(const FootLocker& fl, std::span<const BoneTransform> aPose, int aSide)
{
    const BoneTransform a = fl.ModelSpace(aPose, fl.ChainBone(aSide, 0)), b = fl.ModelSpace(aPose, fl.ChainBone(aSide, 1)),
                        c = fl.ModelSpace(aPose, fl.ChainBone(aSide, 2));
    auto len = [](math::Vec3 v) { return std::sqrt(static_cast<double>(v.x) * v.x + static_cast<double>(v.y) * v.y + static_cast<double>(v.z) * v.z); };
    const double l = len(b.p - a.p) + len(c.p - b.p);
    return l > 1e-6 ? len(c.p - a.p) / l : 0.0;
}

// One frame of the DLL leg tail on the evaluator pose aPose (visual root -> FootLocker, MotionHost.cpp ApplyLegPipelineLocked /
// MotionDriver.cpp SampleLocomotion): contacts = interpolated CONT of the playing clip through the locomotion signal blend (live base
// stack signals when the evaluator uses them), FootLocker frame = the visual root (capsule + root offset, the pose root of the DLL).
// Reported: toe world speed while the locker holds the contact, raw pose vs locker output, per gait / clip type.
void FlFrame(FlState& S, const MotionDatabase& db, const PoseCtx& pc, const Feed& f, size_t i, const Frame& r, const MatchResult& m,
             const ChooserState& cs, const adapter::MapState& ms, const motionpose::PoseEvaluator& pev, const motionpose::FrameInfo& fi,
             const std::vector<BoneTransform>& aPoseIn, const std::vector<BoneTransform>* aPrevRawIn, float fps)
{
    // The DLL hands the leg tail the body bones only (Root / Trajectory carry no motion there: the offset root is added separately).
    std::vector<BoneTransform> poseCopy(aPoseIn);
    poseCopy[0] = BoneTransform{};
    poseCopy[1] = BoneTransform{};
    const std::vector<BoneTransform>& aPose = poseCopy;
    std::vector<BoneTransform> prevCopy;
    const std::vector<BoneTransform>* aPrevRaw = nullptr;
    if (aPrevRawIn)
    {
        prevCopy = *aPrevRawIn;
        prevCopy[0] = BoneTransform{};
        prevCopy[1] = BoneTransform{};
        aPrevRaw = &prevCopy;
    }
    const float dt = r.dt;
    const float actorYaw = r.yawDeg * kDeg;
    // --- locomotion signals (SampleLocomotion)
    locosignal::Sample target;
    {
        const MotionClipRecord& cl = db.Clip(static_cast<uint32_t>(m.clip));
        const float local = std::isfinite(m.clipTime) ? std::fmax(0.0f, m.clipTime * fps) : 0.0f;
        const uint32_t f0 = cl.firstFrame + std::min(static_cast<uint32_t>(local), cl.numFrames - 1);
        const uint32_t f1 = f0 + 1 < cl.firstFrame + cl.numFrames ? f0 + 1 : f0;
        const float u = std::clamp(local - std::floor(local), 0.0f, 1.0f);
        const auto& c0 = db.Contact(f0);
        const auto& c1 = db.Contact(f1);
        target.contact[0] = c0.l + (c1.l - c0.l) * u;
        target.contact[1] = c0.r + (c1.r - c0.r) * u;
        const auto& a = db.Root(f0);
        const auto& z = db.Root(f1);
        const float rate = std::isfinite(m.playRate) && m.playRate > 0.0f ? m.playRate : 1.0f;
        target.rootVel[0] = (a.dx + (z.dx - a.dx) * u) * fps * rate;
        target.rootVel[1] = (a.dy + (z.dy - a.dy) * u) * fps * rate;
        target.rootYawRate = (a.dyaw + (z.dyaw - a.dyaw) * u) * fps * rate;
    }
    if (pev.UsesLiveBase())
    {
        const auto& base = pev.BaseSignals();
        target.rootVel[0] = base.rootX;
        target.rootVel[1] = base.rootY;
        target.rootYawRate = base.rootYaw;
        target.contact[0] = base.left;
        target.contact[1] = base.right;
        S.blend.Reset();
    }
    const bool air = cs.movementMode == 1;
    if (air)
    {
        S.blend.ClearContacts();
        target.contact[0] = target.contact[1] = 0.0f;
    }
    const locosignal::Sample sig = S.blend.Update(target, m.jumped && m.blend.valid, fi.inertialDuration, dt);
    const std::string clipName(db.ClipName(static_cast<uint32_t>(m.clip)));
    const uint8_t cls = steer::ClassifyClip(clipName);
    // --- visual root
    rootframe::Inputs ri;
    ri.dt = dt;
    ri.actorYaw = actorYaw;
    float cdx = 0.0f, cdy = 0.0f;
    math::Rot2(-actorYaw, r.vx * dt, r.vy * dt, cdx, cdy);
    ri.capsuleDeltaCurrent = {cdx, cdy, 0.0f};
    ri.capsuleYawDelta = S.havePrevYaw ? math::WrapAngle(actorYaw - S.prevYaw) : 0.0f;
    ri.animationDeltaPreviousVisual = {sig.rootVel[0] * dt, sig.rootVel[1] * dt, 0.0f};
    ri.animationYawDelta = sig.rootYawRate * dt;
    ri.movingGround = ms.st.isMoving && cs.movementMode == 0;
    ri.restTurnPlaying = cls == steer::kClassTurnInPlace;
    rootframe::Proposal prop;
    if (S.proc.Prepare(ri, prop))
        (void)S.proc.Commit(prop);
    S.prevYaw = actorYaw;
    S.havePrevYaw = true;
    const visualroot::Frame vf = S.proc.Frame();
    const double ca = std::cos(static_cast<double>(actorYaw)), sa = std::sin(static_cast<double>(actorYaw));
    const double vpx = f.px[i] + ca * vf.x - sa * vf.y, vpy = f.py[i] + sa * vf.x + ca * vf.y;
    const float vyaw = math::WrapAngle(actorYaw + vf.yaw);
    const float vvx = S.havePrevVis && dt > 0.0f ? static_cast<float>((vpx - S.prevVx) / dt) : 0.0f;
    const float vvy = S.havePrevVis && dt > 0.0f ? static_cast<float>((vpy - S.prevVy) / dt) : 0.0f;
    S.prevVx = vpx;
    S.prevVy = vpy;
    S.havePrevVis = true;
    // --- leg tail
    FootFeedPolicy pol;
    pol.stance = r.loco;
    pol.v2 = true;
    pol.airborne = air;
    pol.contactScale = 1.0f;
    pol.replantMask = g_fl.mask;
    pol.poleCarry = true;
    const std::string_view dn = m.database >= 0 ? db.DbName(static_cast<uint32_t>(m.database)) : std::string_view();
    pol.stops = !air && (dn.ends_with("_Stops") || dn.ends_with("_Idles"));
    pol.cancelStartSupport = pev.CancelledStartSupport();
    pol.standingStopGate = std::hypot(r.mvx, r.mvy) <= 0.15f;
    pol.classStop = cls == steer::kClassStop;
    pol.classIdle = cls == steer::kClassIdle;
    ApplyFootFeedPolicy(S.fl, pol);
    if (g_fl.Opt("slipStart", 0.0f) > 0.0f || g_fl.Opt("slipTurn", 0.0f) > 0.0f)
    {
        FootSlipProfile sp;
        sp.active = true;
        sp.base = g_fl.slipStart;
        sp.halflife = g_fl.slipHalflife;
        sp.start = g_fl.Opt("slipStart", g_fl.slipStart);
        sp.turn = g_fl.Opt("slipTurn", g_fl.slipStart);
        sp.downRate = g_fl.Opt("slipDown", 0.15f);
        ApplyFootSlipProfile(S.fl, sp, cls);
    }
    S.fl.Update(dt, vvx, vvy, vyaw, sig.contact[0], sig.contact[1], aPose);
    const std::span<const BoneTransform> out = S.fl.Output();
    const FootLockDiagnostics& dg = S.fl.Diagnostics();
    ++g_flTot.frames;
    g_flTot.maxCorr = std::max(g_flTot.maxCorr, static_cast<double>(dg.maxCorrection) * 100.0);
    g_flTot.maxLift = std::max(g_flTot.maxLift, static_cast<double>(dg.maxLift) * 100.0);
    if (dg.simultaneousSteps > S.lastSimultaneous)
        g_flTot.simultaneous += static_cast<uint32_t>(dg.simultaneousSteps - S.lastSimultaneous);
    S.lastSimultaneous = dg.simultaneousSteps;
    const std::string key = FlKey(clipName);
    FlAcc& acc = g_flAcc[key];
    for (int s = 0; s < 2; ++s)
    {
        const FootLockFootDiag& d = dg.foot[s];
        const int32_t lb = S.fl.LockBone(s);
        double pre[2], post[2];
        {
            float x, y;
            S.fl.CharacterToWorld(S.fl.ModelSpace(aPose, lb).p, x, y);
            pre[0] = x;
            pre[1] = y;
            S.fl.CharacterToWorld(S.fl.ModelSpace(out, lb).p, x, y);
            post[0] = x;
            post[1] = y;
        }
        const bool planted = d.contact && S.prevContact[s];
        const double rPre = LegRatio(S.fl, aPose, s), rPost = LegRatio(S.fl, out, s);
        if (d.contact)
        {
            acc.reachPre += rPre > 0.98;
            acc.reachPost += rPost > 0.98;
            g_flTot.hyperPre += rPre > 0.995;
            g_flTot.hyperPost += rPost > 0.995;
        }
        {
            FlState::Sample sm;
            sm.pre[0] = pre[0];
            sm.pre[1] = pre[1];
            sm.post[0] = post[0];
            sm.post[1] = post[1];
            sm.contact = d.contact;
            sm.step = d.stepped || d.stepping;
            sm.corr = d.correction;
            sm.key = S.KeyId(key);
            sm.clip = S.ClipId(clipName);
            sm.t = r.t;
            sm.c = s == 0 ? sig.contact[0] : sig.contact[1];
            S.samp[s].push_back(sm);
        }
        if (g_fl.Opt("dbg", 0.0f) > 0.5f && s == 0 && i % 6 == 0)
            std::printf("DBG t=%.2f %s cap=(%.2f,%.2f) vis=(%.2f,%.2f) off=(%.3f,%.3f,%.1fdeg) yaw=%.1f/%.1f c=%.2f planted=%d pre=(%.3f,%.3f) post=(%.3f,%.3f) rootv=(%.2f,%.2f) rate=%.2f clipT=%.2f\n", static_cast<double>(r.t),
                        clipName.c_str(), static_cast<double>(r.vx), static_cast<double>(r.vy), static_cast<double>(vvx), static_cast<double>(vvy),
                        static_cast<double>(vf.x), static_cast<double>(vf.y), static_cast<double>(vf.yaw) * 57.3, static_cast<double>(r.yawDeg),
                        static_cast<double>(vyaw) * 57.3, static_cast<double>(sig.contact[0]), planted ? 1 : 0, pre[0], pre[1], post[0], post[1],
                        static_cast<double>(sig.rootVel[0]), static_cast<double>(sig.rootVel[1]), static_cast<double>(m.playRate), static_cast<double>(m.clipTime));
        S.prevContact[s] = d.contact;
        S.prevPre[s][0] = pre[0];
        S.prevPre[s][1] = pre[1];
        S.prevPost[s][0] = post[0];
        S.prevPost[s][1] = post[1];
        // angular steps of the chain bones: output vs previous output, and raw vs previous raw (pops vs the source max step)
        if (S.havePrev && aPrevRaw)
            for (int k = 0; k < 3; ++k)
            {
                const uint32_t b = static_cast<uint32_t>(S.fl.ChainBone(s, k));
                const double aPost = QuatAngleDeg(S.prevOut[b].q, out[b].q), aPre = QuatAngleDeg((*aPrevRaw)[b].q, aPose[b].q);
                g_flTot.maxPostStepDeg = std::max(g_flTot.maxPostStepDeg, aPost);
                const bool popPost = aPost > pc.srcMax[b], popPre = aPre > pc.srcMax[b];
                g_flTot.popsPost += popPost;
                g_flTot.popsPre += popPre;
                g_flTot.newPops += popPost && !popPre;
            }
    }
    for (uint32_t b = 0; b < out.size(); ++b)
        S.prevOut[b] = out[b];
    S.havePrev = true;
}

void RunFeed(const MotionDatabase& db, const chooser::Table& table, const Feed& f, FILE* aCsv, bool aLegacySpring, const PoseCtx& pc)
{
    adapter::MotionAdapter drv;
    std::string err;
    MatcherConfig cfg;
    cfg.steadyRunLoops = g_steadyRun;
    if (!drv.Init(&db, &table, cfg, 2, err))
    {
        CHECK(false, "%s: MotionAdapter init: %s", f.name.c_str(), err.c_str());
        return;
    }
    auto pev = std::make_unique<motionpose::PoseEvaluator>();
    if (!pev->Init(&db, pc.ovf, pc.style, 2, err))
    {
        CHECK(false, "%s: PoseEvaluator init: %s", f.name.c_str(), err.c_str());
        return;
    }
    const uint32_t nb = db.NumBones();
    std::vector<BoneTransform> pose(nb), prevPose(nb);
    std::unique_ptr<FlState> fls;
    if (g_fl.on)
    {
        fls = std::make_unique<FlState>();
        if (!fls->fl.Init(db, g_fl.lc))
        {
            CHECK(false, "%s: FootLocker init on the motion database", f.name.c_str());
            return;
        }
        fls->fl.SetSlip(g_fl.slipStart, g_fl.slipHalflife);
        fls->fl.SetReplantConfig(g_fl.rc);
        fls->prevOut.resize(nb);
    }
    bool havePrevPose = false;
    double prevFoot[2][2][2] = {}; // [actor/root yaw][L/R][x/y] world, m
    bool prevPlanted[2] = {false, false};
    Stats st;
    adapter::MapConfig mc;
    mc.facingSource = pc.facing;
    mc.slide = pc.slide;
    mc.slideSpec.declared = table.GetSlide().present;
    mc.slideDeceleration = adapter::kSlideDecelerationMs2;
    mc.slideAirGraceS = pc.slide ? adapter::kSlideAirGraceSeconds : 0.0f; // MotionDriver Step (fix/slide-exit)
    mc.stanceFilterS = adapter::kStanceFilterSeconds;
    mc.stanceHoldInDodge = true;
    mc.stanceCleanEdge = true; // P1 (fix batch 2): mirrors MotionDriver.cpp
    mc.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
    mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
    adapter::MapState ms;
    std::vector<TrajSample> traj;
    const adapter::QueryTimes qtimes = adapter::BuildQueryTimes(db);
    if (!aLegacySpring && (qtimes.overflow || qtimes.n == 0))
    {
        CHECK(false, "%s: query trajectory times: %u, overflow %d", f.name.c_str(), qtimes.n, qtimes.overflow ? 1 : 0);
        return;
    }
    uint32_t noQuery = 0;
    int32_t lastClip = -1;
    int32_t prevDb = -1;
    float rootYawDeg = 0.0f;
    const float fps = db.Header().fps > 0.0f ? db.Header().fps : 30.0f;
    std::string lastBaseStack = f.fr.front().baseClip, lastMask;
    for (size_t i = 0; i < f.fr.size(); ++i)
    {
        const Frame& r = f.fr[i];
        float fvx = r.vx, fvy = r.vy;
        if (aLegacySpring)
            BuildTraj(f, i, traj, fvx, fvy);
        const float vel[3] = {r.vx, r.vy, r.vz};
        const float mv[2] = {r.mvx, r.mvy};
        const adapter::MotionFeed feed = adapter::MakeMotionFeed(vel, mv, r.dt, static_cast<uint32_t>(r.psm), r.loco, adapter::AirFeed{}, r.camDeg, r.yawDeg);
        // MapContext: previous result database tags (motion CurrentDatabaseTags, ABP l. 1901) and the root yaw. Root yaw = motion
        // OffsetRoot rotation in RotationMode Accumulate (ABP l. 1230, 1944-1950): it follows the animation root motion only,
        // so the replay integrates the playing frame's ROOT dyaw (rad/frame, CCW about +Z = integ convention) x fps x dt,
        // seeded with the actor yaw on the first frame. Closer to OffsetRoot than the actor-yaw track (which IS the capsule).
        if (i == 0)
            rootYawDeg = r.yawDeg;
        adapter::MapContext ctx;
        ctx.rootYawValid = true;
        ctx.rootYawDeg = rootYawDeg;
        ctx.currentDbPivots = prevDb >= 0 && adapter::DatabaseHasPivotsTag(db.DbName(static_cast<uint32_t>(prevDb)));
        ChooserState cs = adapter::MapToChooser(feed, mc, ms, ctx);
        if (!aLegacySpring)
        {
            traj.resize(adapter::kQueryTrajMax);
            const uint32_t nq = adapter::BuildQueryTrajectory(ms, qtimes, traj.data(), adapter::kQueryTrajMax);
            traj.resize(nq);
            if (nq == 0)
            {
                ++noQuery;
                continue;
            }
        }
        const float sp = Speed(r);
        (void)fvx;
        (void)fvy;
        const auto t0 = std::chrono::high_resolution_clock::now();
        const bool ok = drv.Update(r.dt, traj.data(), static_cast<uint32_t>(traj.size()), cs);
        const auto t1 = std::chrono::high_resolution_clock::now();
        const MatchResult& m = drv.Last();
        ++st.updates;
        st.us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        if (!ok || m.clip < 0 || m.database < 0)
        {
            ++st.noPose;
            continue;
        }
        prevDb = m.database;
        if (m.frame < db.NumFrames())
            rootYawDeg = motiontraj::NormalizeAxis(rootYawDeg + db.Root(m.frame).dyaw * fps * r.dt / kDeg);
        // ---- pose (replay5)
        double frameMaxStep = 0.0;
        {
            motionpose::OverlayFrameInputs oi;
            oi.machine.overlay = ov::PoseOverlay::Default;
            oi.machine.style = pc.man ? ov::Style::Masculine : ov::Style::Feminine;
            oi.machine.crouch = cs.stance == 1;
            oi.machine.moving = cs.movementState == 1;
            oi.machine.hasVelocity = ms.st.hasVelocity;
            oi.machine.sprinting = cs.gait == 2;
            motionpose::FrameInfo fi;
            const bool pok = pev->Evaluate(r.dt, m, oi, pose.data(), nb, &fi);
            ++st.poseFrames;
            if (!pok)
                ++st.poseFail;
            else
            {
                st.evalUs.push_back(fi.evalUs);
                if (havePrevPose)
                    for (uint32_t b = 2; b < nb; ++b)
                    {
                        const double a = QuatAngleDeg(prevPose[b].q, pose[b].q);
                        st.steps.push_back(a);
                        if (a > st.maxStep)
                        {
                            st.maxStep = a;
                            st.maxStepBone = static_cast<int32_t>(b);
                        }
                        frameMaxStep = std::max(frameMaxStep, a);
                        if (a > pc.srcMax[b])
                            ++st.pops;
                    }
                // foot sliding while planted (contact of the playing row on this and the previous frame)
                const ContactRecord& ct = db.Contact(m.frame);
                const bool planted[2] = {ct.l >= 0.5f, ct.r >= 0.5f};
                const int32_t fb[2] = {pc.footL, pc.footR};
                const double yawR[2] = {ms.actorYawDeg * static_cast<double>(kDeg), rootYawDeg * static_cast<double>(kDeg)};
                for (int s = 0; s < 2; ++s)
                {
                    if (fb[s] < 0)
                        continue;
                    const V3d mp = ModelPos(db, pose.data(), static_cast<uint32_t>(fb[s]));
                    double w[2][2];
                    for (int y = 0; y < 2; ++y)
                    {
                        const double c = std::cos(yawR[y]), sn = std::sin(yawR[y]);
                        w[y][0] = f.px[i] + c * mp.x - sn * mp.y;
                        w[y][1] = f.py[i] + sn * mp.x + c * mp.y;
                    }
                    if (havePrevPose && planted[s] && prevPlanted[s])
                    {
                        ++st.planted[s];
                        const double da = std::hypot(w[0][0] - prevFoot[0][s][0], w[0][1] - prevFoot[0][s][1]) * 100.0;
                        const double dr = std::hypot(w[1][0] - prevFoot[1][s][0], w[1][1] - prevFoot[1][s][1]) * 100.0;
                        st.slideActor[s] += da;
                        st.slideRoot[s] += dr;
                        st.slideActorMax[s] = std::max(st.slideActorMax[s], da);
                    }
                    for (int y = 0; y < 2; ++y)
                    {
                        prevFoot[y][s][0] = w[y][0];
                        prevFoot[y][s][1] = w[y][1];
                    }
                    prevPlanted[s] = planted[s];
                }
                if (fls)
                    FlFrame(*fls, db, pc, f, i, r, m, cs, ms, *pev, fi, pose, havePrevPose ? &prevPose : nullptr, fps);
                std::swap(prevPose, pose);
                havePrevPose = true;
            }
        }
        if (m.searched && !std::isfinite(m.cost))
            ++st.nonFinite;
        const bool inSet = (m.dbMask >> m.database) & 1u;
        if (!inSet && m.jumped)
            ++st.notChosen;
        if (!inSet && !m.jumped)
        {
            if (m.interrupt == Interrupt::DoNotInterrupt)
                ++st.contOutsideKept; // UE keeps evaluating the continuing pose of a database no longer chosen
            else
                ++st.contOutsideIllegal;
        }
        if (m.continuingOutsideSet != (!inSet && !m.jumped))
            ++st.contOutsideFlagMismatch;
        if (!m.jumped)
            ++st.continuing;
        else
            ++st.jumps;
        const std::string mk = MaskName(db, m.dbMask);
        ++st.masks[mk];
        if (mk != lastMask)
        {
            char b[256];
            std::snprintf(b, sizeof b, "%.2fs %s", static_cast<double>(r.t), mk.c_str());
            st.timeline.push_back(b);
            lastMask = mk;
        }
        if (m.clip != lastClip && lastClip >= 0)
            ++st.clipChanges;
        lastClip = m.clip;
        if (r.baseClip != lastBaseStack)
            ++st.baseClipChanges;
        lastBaseStack = r.baseClip;
        const std::string motionc(db.ClipName(static_cast<uint32_t>(m.clip)));
        const std::string f1 = ClipClass(r.baseClip), f2 = ClipClass(motionc);
        ++st.baseFam[f1];
        ++st.motionFam[f2];
        ++st.famCmp;
        if (f1 == f2)
            ++st.famSame;
        if (m.jumped)
            ++st.motionJumpFam[f2];
        if (aCsv)
            std::fprintf(aCsv, "%s,%.3f,%s,%.2f,%d,%d,%d,%s,%s,%s,%d,%.3f,%d,%.1f,%.3f,%.2f,%.3f,%d,%d,%d\n", f.name.c_str(), static_cast<double>(r.t), r.step.c_str(),
                         static_cast<double>(sp), r.loco, cs.gait, cs.movementState, mk.c_str(), r.baseClip.c_str(), motionc.c_str(),
                         m.jumped ? 1 : 0, static_cast<double>(m.cost), static_cast<int>(m.rowsEvaluated), st.us.back(),
                         static_cast<double>(m.clipTime), static_cast<double>(ms.actorYawDeg), frameMaxStep, cs.movementMode, cs.movementDirection, cs.movementModeLast);
    }
    if (fls)
    {
        FlFinalize(*fls, f.fr.front().dt);
        const FootLockDiagnostics& dg = fls->fl.Diagnostics();
        g_flTot.clamps += static_cast<uint32_t>(dg.clamps);
        g_flTot.drift += static_cast<uint32_t>(dg.driftUnlocks);
        g_flTot.steps += static_cast<uint32_t>(dg.footSteps[0] + dg.footSteps[1]);
    }
    if (g_fl.brief)
        return;
    const double secs = static_cast<double>(f.fr.back().t - f.fr.front().t + f.fr.front().dt);
    std::vector<double> sorted = st.us;
    const double mean = st.us.empty() ? 0.0 : [&] { double a = 0; for (double v : st.us) a += v; return a / static_cast<double>(st.us.size()); }();
    const double med = Pct(sorted, 0.5), p95 = Pct(sorted, 0.95), mx = sorted.empty() ? 0.0 : *std::max_element(sorted.begin(), sorted.end());
    std::printf("\n== %s (%.1f s, %u updates)\n", f.name.c_str(), secs, st.updates);
    std::printf("  search us: mean %.1f median %.1f p95 %.1f max %.1f\n", mean, med, p95, mx);
    std::printf("  v2 clip changes %u (%.2f/s), jumps %u, continuing %.1f%%; v1 clip changes %u (%.2f/s, from TRANS-sampled log)\n",
                st.clipChanges, st.clipChanges / secs, st.jumps, 100.0 * st.continuing / std::max(1u, st.updates - st.noPose),
                st.baseClipChanges, st.baseClipChanges / secs);
    std::printf("  noPose %u nonFinite %u pickOutsideChooserSet %u; class family base==v2 %.1f%% of %u frames\n", st.noPose,
                st.nonFinite, st.notChosen, 100.0 * st.famSame / std::max(1u, st.famCmp), st.famCmp);
    std::printf("  continuing outside chooser set: kept under DoNotInterrupt %u (UE-conform), under another interrupt mode %u, "
                "flag mismatch %u\n", st.contOutsideKept, st.contOutsideIllegal, st.contOutsideFlagMismatch);
    std::printf("  chooser sets (frames):");
    for (const auto& [k, v] : st.masks)
        std::printf(" [%s]=%u", k.c_str(), v);
    std::printf("\n  set timeline:");
    for (size_t i = 0; i < st.timeline.size() && i < 40; ++i)
        std::printf(" | %s", st.timeline[i].c_str());
    std::printf("%s\n  family frames v1:", st.timeline.size() > 40 ? " | ..." : "");
    for (const auto& [k, v] : st.baseFam)
        std::printf(" %s=%u", k.c_str(), v);
    std::printf("\n  family frames motion:");
    for (const auto& [k, v] : st.motionFam)
        std::printf(" %s=%u", k.c_str(), v);
    std::printf("\n");
    std::printf("  query trajectory: %s (%u samples), facing %s\n", aLegacySpring ? "legacy spring" : "MotionTrajectory",
                aLegacySpring ? 157u : qtimes.n, pc.facing == adapter::FacingSource::MotionActor ? "MotionActor" : "Game");
    std::printf("  jumps by family:");
    for (const auto& [k, v] : st.motionJumpFam)
        std::printf(" %s=%u", k.c_str(), v);
    std::printf("\n");
    {
        const double nStep = static_cast<double>(st.steps.size());
        std::printf("  POSE frames %u evalFail %u; Evaluate us median %.1f p95 %.1f max %.1f\n", st.poseFrames, st.poseFail,
                    Pct(st.evalUs, 0.5), Pct(st.evalUs, 0.95),
                    st.evalUs.empty() ? 0.0 : *std::max_element(st.evalUs.begin(), st.evalUs.end()));
        std::printf("  POSE angular step (bones>=2, per frame): max %.2f deg (%s) p99 %.2f deg; pops (> source max of the bone) %u "
                    "of %.0f bone-steps\n",
                    st.maxStep, st.maxStepBone >= 0 ? std::string(db.BoneName(static_cast<uint32_t>(st.maxStepBone))).c_str() : "-",
                    Pct(st.steps, 0.99), st.pops, nStep);
        for (int s = 0; s < 2; ++s)
            std::printf("  POSE foot slide %s: planted frames %u, actor-yaw total %.1f cm (%.3f cm/frame, max %.2f), root-yaw total "
                        "%.1f cm (%.3f cm/frame)\n",
                        s == 0 ? "L" : "R", st.planted[s], st.slideActor[s], st.slideActor[s] / std::max(1u, st.planted[s]),
                        st.slideActorMax[s], st.slideRoot[s], st.slideRoot[s] / std::max(1u, st.planted[s]));
    }
    CHECK(st.poseFail == 0, "%s: %u frames where PoseEvaluator::Evaluate gave no finite pose", f.name.c_str(), st.poseFail);
    CHECK(noQuery == 0, "%s: %u frames without a query trajectory", f.name.c_str(), noQuery);
    CHECK(st.noPose == 0, "%s: %u updates without a pose", f.name.c_str(), st.noPose);
    CHECK(st.nonFinite == 0, "%s: %u non-finite costs", f.name.c_str(), st.nonFinite);
    CHECK(st.notChosen == 0, "%s: %u jumps outside the chooser set", f.name.c_str(), st.notChosen);
    CHECK(st.contOutsideIllegal == 0, "%s: %u continuing picks outside the chooser set under an interrupting mode", f.name.c_str(),
          st.contOutsideIllegal);
    CHECK(st.contOutsideFlagMismatch == 0, "%s: %u frames with continuingOutsideSet != (outside set && !jumped)", f.name.c_str(),
          st.contOutsideFlagMismatch);
    CHECK(med < 300.0, "%s: median search %.1f us >= 300", f.name.c_str(), med);
}
} // namespace

int main(int argc, char** argv)
{
    std::string dbPath, csvPath, ovPath, stylePath;
    std::vector<std::string> feeds;
    bool legacySpring = false;
    PoseCtx pc;
    bool badArg = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--legacy-spring")
            legacySpring = true;
        else if (a == "--slide")
            pc.slide = true;
        else if (a == "--no-g10")
            g_steadyRun = false;
        else if (a == "--footlock")
            g_fl.on = true;
        else if (a == "--brief")
            g_fl.brief = true;
        else if (a == "--replant-mask" && i + 1 < argc)
            g_fl.mask = std::atoi(argv[++i]);
        else if (a == "--slip" && i + 2 < argc)
        {
            g_fl.slipStart = static_cast<float>(std::atof(argv[++i]));
            g_fl.slipHalflife = static_cast<float>(std::atof(argv[++i]));
        }
        else if (a == "--replant" && i + 7 < argc)
        {
            FootReplantConfig& c = g_fl.rc;
            c.radius = static_cast<float>(std::atof(argv[++i]));
            c.radiusStops = static_cast<float>(std::atof(argv[++i]));
            c.ratio = static_cast<float>(std::atof(argv[++i]));
            c.ratioStops = static_cast<float>(std::atof(argv[++i]));
            c.stepSpeed = static_cast<float>(std::atof(argv[++i]));
            c.stepSpeedStops = static_cast<float>(std::atof(argv[++i]));
            c.lift = static_cast<float>(std::atof(argv[++i]));
        }
        else if (a == "--label" && i + 1 < argc)
            g_fl.label = argv[++i];
        else if (a == "--opt" && i + 1 < argc)
            g_fl.extra.push_back(argv[++i]);
        else if (a == "--db" && i + 1 < argc)
            dbPath = argv[++i];
        else if (a == "--csv" && i + 1 < argc)
            csvPath = argv[++i];
        else if (a == "--overlay" && i + 1 < argc)
            ovPath = argv[++i];
        else if (a == "--style" && i + 1 < argc)
            stylePath = argv[++i];
        else if (a == "--facing" && i + 1 < argc)
        {
            const std::string v = argv[++i];
            if (v == "game")
                pc.facing = adapter::FacingSource::Game;
            else if (v == "motion")
                pc.facing = adapter::FacingSource::MotionActor;
            else
                badArg = true;
        }
        else
            feeds.push_back(a);
    }
    if (dbPath.empty() || feeds.empty() || badArg)
    {
        std::printf("usage: utpp_motion_replay --db <motion .utppmm> [--overlay .utppov] [--style weights.txt] [--facing game|motion set] "
                    "[--csv out.csv] [--legacy-spring] [--slide] [--no-g10] <feed>...\n  default overlay / style: <db dir>/overlay/"
                    "utpp_overlays_<body>.utppov, utpp_style_weights_<body>.txt\nFAIL: arguments\n");
        return 1;
    }
    std::string err;
    MotionDatabase db;
    const bool man = dbPath.find("_man") != std::string::npos;
    if (!db.LoadFromFile(dbPath.c_str(), man ? kRigIdentityMan : kRigIdentityWoman, err))
    {
        std::printf("FAIL: load %s: %s\n", dbPath.c_str(), err.c_str());
        return 1;
    }
    chooser::Table table;
    const std::string ch = dbPath.substr(0, dbPath.size() - 7) + ".chooser.json";
    if (!table.LoadFile(ch.c_str(), err))
    {
        std::printf("FAIL: chooser %s: %s\n", ch.c_str(), err.c_str());
        return 1;
    }
    std::printf("db %s: %u databases, %u rows, %u clips\n", dbPath.c_str(), db.NumDatabases(), db.NumRows(), db.NumClips());
    // overlay + style weights (PoseEvaluator)
    {
        const size_t sl = dbPath.find_last_of("/\\");
        const std::string dir = sl == std::string::npos ? std::string(".") : dbPath.substr(0, sl);
        const char* body = man ? "man" : "woman";
        if (ovPath.empty())
            ovPath = dir + "/motion/utpp_overlays_" + body + ".utppov";
        if (stylePath.empty())
            stylePath = dir + "/motion/utpp_style_weights_" + body + ".txt";
    }
    auto ovf = std::make_unique<ov::OverlayFile>();
    if (!ovf->LoadFromFile(ovPath.c_str(), man ? ov::kRigIdentityManOv : ov::kRigIdentityWomanOv, err))
    {
        std::printf("FAIL: overlay %s: %s\n", ovPath.c_str(), err.c_str());
        return 1;
    }
    ov::StyleFamilyData style;
    {
        std::ifstream sf(stylePath, std::ios::binary);
        std::stringstream ss;
        ss << sf.rdbuf();
        if (!sf.good() || !style.Parse(ss.str(), &err))
        {
            std::printf("FAIL: style weights %s: %s\n", stylePath.c_str(), err.c_str());
            return 1;
        }
    }
    pc.ovf = ovf.get();
    pc.style = &style;
    pc.man = man;
    pc.srcMax = SourceMaxStep(db);
    pc.footL = FindBone(db, "LeftFoot");
    pc.footR = FindBone(db, "RightFoot");
    if (pc.footL < 0 || pc.footR < 0)
    {
        std::printf("FAIL: LeftFoot / RightFoot not in the database skeleton\n");
        return 1;
    }
    std::printf("overlay %s, style %s, facing %s; source max step (deg / 1/60 s): LeftFoot %.2f RightFoot %.2f\n", ovPath.c_str(),
                stylePath.c_str(), pc.facing == adapter::FacingSource::MotionActor ? "MotionActor" : "Game",
                pc.srcMax[static_cast<size_t>(pc.footL)], pc.srcMax[static_cast<size_t>(pc.footR)]);
    FILE* csv = csvPath.empty() ? nullptr : OpenWrite(csvPath.c_str());
    if (csv)
        std::fprintf(csv, "feed,t,step,speed,loco,gait,moving,dbset,baseClip,motionclip,jumped,cost,rows,us,clipT,actorYaw,maxStep,mode,dir,modeLast\n");
    for (const std::string& p : feeds)
    {
        Feed f;
        if (!LoadFeed(p.c_str(), f))
        {
            CHECK(false, "feed %s unreadable", p.c_str());
            continue;
        }
        RunFeed(db, table, f, csv, legacySpring, pc);
    }
    if (csv)
        std::fclose(csv);
    if (g_fl.on)
    {
        std::printf("\nFOOTLOCK label=%s db=%s mask=%d slip=%.3f/%.3f\n", g_fl.label.c_str(), dbPath.c_str(), g_fl.mask,
                    static_cast<double>(g_fl.slipStart), static_cast<double>(g_fl.slipHalflife));
        std::printf("FLTAB key n pre_cm/s post_cm/s post_p95 pre_max post_max step%% post_nostep corr_mean corr_p95 corr_max reach98_pre/post\n");
        FlAcc all;
        for (auto& [k, a] : g_flAcc)
        {
            if (a.n == 0)
                continue;
            std::printf("FLTAB %-14s %5u %7.2f %7.2f %7.2f %7.1f %7.1f %5.1f %7.2f %6.2f %6.2f %6.2f %u/%u\n", k.c_str(), a.n, a.sumPre / a.n,
                        a.sumPost / a.n, Pct(a.post, 0.95), a.maxPre, a.maxPost, 100.0 * a.nStep / a.n,
                        a.nNoStep ? a.sumPostNoStep / a.nNoStep : 0.0, a.sumCorr / a.n, Pct(a.corr, 0.95), a.maxCorr, a.reachPre, a.reachPost);
            all.n += a.n;
            all.sumPre += a.sumPre;
            all.sumPost += a.sumPost;
            all.nStep += a.nStep;
            all.nNoStep += a.nNoStep;
            all.sumPostNoStep += a.sumPostNoStep;
            all.sumCorr += a.sumCorr;
            all.maxCorr = std::max(all.maxCorr, a.maxCorr);
            all.maxPre = std::max(all.maxPre, a.maxPre);
            all.maxPost = std::max(all.maxPost, a.maxPost);
            all.pre.insert(all.pre.end(), a.pre.begin(), a.pre.end());
            all.post.insert(all.post.end(), a.post.begin(), a.post.end());
            all.corr.insert(all.corr.end(), a.corr.begin(), a.corr.end());
        }
        if (all.n)
            std::printf("FLTAB %-14s %5u %7.2f %7.2f %7.2f %7.1f %7.1f %5.1f %7.2f %6.2f %6.2f %6.2f\n", "ALL", all.n, all.sumPre / all.n,
                        all.sumPost / all.n, Pct(all.post, 0.95), all.maxPre, all.maxPost, 100.0 * all.nStep / all.n,
                        all.nNoStep ? all.sumPostNoStep / all.nNoStep : 0.0, all.sumCorr / all.n, Pct(all.corr, 0.95), all.maxCorr);
        std::printf("FLROB key n pre_p50 post_p50 pre_over25%% post_over25%%\n");
        for (auto& [k, a] : g_flAcc)
            if (a.n)
                std::printf("FLROB %-14s %5u %7.2f %7.2f %6.1f %6.1f\n", k.c_str(), a.n, Pct(a.pre, 0.5), Pct(a.post, 0.5), FracAbove(a.pre, 25.0), FracAbove(a.post, 25.0));
        std::printf("FLROB %-14s %5u %7.2f %7.2f %6.1f %6.1f\n", "ALL", all.n, Pct(all.pre, 0.5), Pct(all.post, 0.5), FracAbove(all.pre, 25.0), FracAbove(all.post, 25.0));
        if (g_fl.Opt("clipdump", 0.0f) > 0.5f)
        {
            std::printf("FLCLIP clip n pre_mean_cm/s pre_p50 post_mean_cm/s\n");
            for (auto& [k, a] : g_flClip)
                if (a.n)
                    std::printf("FLCLIP %s %u %.2f %.2f %.2f\n", k.c_str(), a.n, a.sumPre / a.n, Pct(a.pre, 0.5), a.sumPost / a.n);
        }
        std::printf("FLTOT frames %u maxCorr %.1f cm maxLift %.1f cm clamps %u driftUnlocks %u steps %u simultaneous %u | leg-bone pops pre %u post %u NEW %u "
                    "maxPostStep %.1f deg | hyperext(>0.995) pre %u post %u | max planted toe %.2f m/s\n",
                    g_flTot.frames, g_flTot.maxCorr, g_flTot.maxLift, g_flTot.clamps, g_flTot.drift, g_flTot.steps, g_flTot.simultaneous,
                    g_flTot.popsPre, g_flTot.popsPost, g_flTot.newPops, g_flTot.maxPostStepDeg, g_flTot.hyperPre, g_flTot.hyperPost,
                    g_flTot.maxToeMs);
    }
    std::printf("\nREPLAY_FAILS=%d\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
