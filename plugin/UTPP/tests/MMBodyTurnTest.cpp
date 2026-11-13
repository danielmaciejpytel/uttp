// Offline test of the Phase 5.1 body yaw command (body yaw follows the motion matching animation). Not shipped.
// Usage: utpp_mm_bodyturn_test <database.utppmm>      Exit code 0 = all checks passed.
//
// Simulated game loop at 60 fps (camera fixed at yaw 0, Locomotion: MoveX/MoveY = (0,1) while the stick is held, the
// capsule velocity follows the world input direction with a 0.05 s half-life at the measured run speed 3.85 m/s):
//   A) fixed-time reference: body yaw = camera + smoothed raw stick angle (SmoothDampAngle style, smoothTime 0.20 s,
//      maxTurnSpeed 1440 deg/s, snapRate 1e5); matcher fed with the legacy (Phase 4) input = measured yaw rate.
//   B) UTPP: matcher fed with the body-turn input (target facing, trajectory yaw = measured body); body yaw =
//      BodyYawController::Command(current, target, dt) after each feed (the game applies the command exactly).
// The feed mirrors MMDriver::Feed (speed map stand 1.6/3.85/8.2 -> 2.19/5.46/7.65, rate clamp 0.6..1.4, transition
// cost 0.15, Stand_ clip set).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/mm/BodyTurn.hpp"
#include "../src/mm/BaseMatcher.hpp"

using namespace utpp::mm;

static int g_fail = 0;
static int g_pass = 0;

static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    if (aOk)
        ++g_pass;
    else
        ++g_fail;
}

static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0, double e = 0, double f = 0)
{
    char buf[512];
    std::snprintf(buf, sizeof buf, aFmt, a, b, c, d, e, f);
    return buf;
}

static constexpr std::string_view kBoneNames[73] = {
    "Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1", "LeftLeg",
    "RightLeg", "Spine2", "LeftFoot", "RightFoot", "Spine3", "LeftShoulder", "RightShoulder", "Neck", "LeftArm",
    "RightArm", "Neck1", "LeftForeArm", "RightForeArm", "Head", "LeftHand", "RightHand", "LeftHandEnd",
    "WeaponLeft", "RightHandEnd", "WeaponRight", "LeftHeel", "RightHeel", "LeftEye", "RightEye",
    "LeftInHandThumb", "LeftInHandIndex", "LeftInHandMiddle", "LeftInHandRing", "LeftInHandPinky",
    "RightInHandThumb", "RightInHandIndex", "RightInHandMiddle", "RightInHandRing", "RightInHandPinky",
    "LeftToeBase", "RightToeBase", "LeftHandThumb1", "LeftHandIndex1", "LeftHandMiddle1", "LeftHandRing1",
    "LeftHandPinky1", "RightHandThumb1", "RightHandIndex1", "RightHandMiddle1", "RightHandRing1",
    "RightHandPinky1", "LeftHandThumb2", "LeftHandIndex2", "LeftHandMiddle2", "LeftHandRing2", "LeftHandPinky2",
    "RightHandThumb2", "RightHandIndex2", "RightHandMiddle2", "RightHandRing2", "RightHandPinky2",
    "LeftHandIndex3", "LeftHandMiddle3", "LeftHandRing3", "LeftHandPinky3", "RightHandIndex3",
    "RightHandMiddle3", "RightHandRing3", "RightHandPinky3"};
static constexpr int32_t kBoneParents[73] = {
    -1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10, 13, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24, 24, 11, 12, 22, 22,
    23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 29, 30, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 45, 46, 47, 48, 49, 50, 51,
    52, 53, 54, 56, 57, 58, 59, 61, 62, 63, 64};

static constexpr float kDeg = 3.14159265358979323846f / 180.0f;
static constexpr float kRunSpeed = 3.85f;

// MMDriver MapSpeed for the stand state (defaults).
static float MapSpeedStand(float aGame)
{
    if (aGame <= 0.0f)
        return 0.0f;
    const float game[3] = {1.6f, 3.85f, 8.2f}, db[3] = {2.19f, 5.46f, 7.65f};
    float pg = 0.0f, pd = 0.0f;
    for (int i = 0; i < 3; ++i)
    {
        if (aGame <= game[i])
            return pd + (aGame - pg) * (db[i] - pd) / (game[i] - pg);
        pg = game[i];
        pd = db[i];
    }
    return aGame * (pd / pg);
}

// Reference smooth-turn model: normalizeDegrees / smoothed stick angle (degrees, doubles).
static double NormDeg(double a)
{
    double x = std::fmod(a + 180.0, 360.0);
    if (x < 0.0)
        x += 360.0;
    return x - 180.0;
}
static void LocomotionSmooth(double& aCurrent, double aTarget, double& aVel, double aSmoothTime, double aDt)
{
    if (aDt <= 0.0)
        return;
    if (aDt > 0.1)
        aDt = 0.1;
    const double st = std::max(aSmoothTime, 0.0001);
    const double omega = 2.0 / st;
    const double x = omega * aDt;
    const double expo = 1.0 / (1.0 + x + 0.48 * x * x + 0.2350 * x * x * x);
    double delta = NormDeg(aCurrent - aTarget);
    const double limit = 1440.0 * st;
    delta = std::clamp(delta, -limit, limit);
    const double temp = (aVel + omega * delta) * aDt;
    double newVel = (aVel - omega * temp) * expo;
    double out = NormDeg((aCurrent - delta) + (delta + temp) * expo);
    const double want = NormDeg(aTarget - aCurrent);
    const double got = NormDeg(out - aCurrent);
    if ((want > 0.0 && got > want) || (want < 0.0 && got < want))
    {
        out = aTarget;
        newVel = 0.0;
    }
    aCurrent = out;
    aVel = newVel;
}

// Copy of the Phase 4 (0.2.8) MMDriver::Feed desired-motion code, used to prove the legacy path is unchanged.
static void OldDesired(float cx, float cy, float mx, float my, float cam, float yaw, bool inputDriven, bool camRel, float stateSpeed,
                       float& dx, float& dy, float& fx, float& fy)
{
    dx = cx;
    dy = cy;
    fx = 0.0f;
    fy = 1.0f;
    const float mag = std::sqrt(mx * mx + my * my);
    const float capsule = std::sqrt(cx * cx + cy * cy);
    if (inputDriven && mag > 0.15f)
    {
        float wx = -std::sin(cam) * my + std::cos(cam) * mx;
        float wy = std::cos(cam) * my + std::sin(cam) * mx;
        const float wl = std::sqrt(wx * wx + wy * wy);
        if (wl > 1e-4f)
        {
            wx /= wl;
            wy /= wl;
            float ix = 0.0f, iy = 1.0f;
            if (camRel)
            {
                const float c = std::cos(yaw), s = std::sin(yaw);
                ix = c * wx + s * wy;
                iy = -s * wx + c * wy;
            }
            else
            {
                ix = mx / mag;
                iy = my / mag;
            }
            const float target = stateSpeed * std::fmin(mag, 1.0f);
            const float sp = std::fmax(capsule, 0.5f * target);
            dx = ix * sp;
            dy = iy * sp;
            fx = ix;
            fy = iy;
        }
    }
}

struct Sim
{
    BaseMatcher m;
    bt::BodyYawController ctl;
    float psi = 0.0f, prevPsi = 0.0f;
    bool havePrev = false;
    float vx = 0.0f, vy = 0.0f;
    float lastRate = 1.0f;
    float lastAnimRate = 0.0f; // rad/s real time
};

static bool InitSim(Sim& s, const Database& db)
{
    MatcherConfig cfg;
    cfg.transitionCost = 0.15f;
    cfg.allowedTagMask = BaseMatcher::TagMaskContaining(db, "Stand_");
    if (!s.m.Init(db, cfg))
        return false;
    s.ctl.Reset();
    s.psi = s.prevPsi = 0.0f;
    s.havePrev = false;
    s.vx = s.vy = 0.0f;
    return true;
}

// One MMDriver::Feed equivalent.
static void Feed(Sim& s, float dt, float mx, float my, bool bodyTurn, float target)
{
    const float yaw = s.psi;
    const float c = std::cos(yaw), sn = std::sin(yaw);
    const float cx = c * s.vx + sn * s.vy, cy = -sn * s.vx + c * s.vy;
    const float yawStep = s.havePrev ? bt::Wrap(yaw - s.prevPsi) : 0.0f;
    float yawRate = (s.havePrev && dt > 0.0f) ? bt::Wrap(yaw - s.prevPsi) / dt : 0.0f;
    yawRate = std::fmax(-12.0f, std::fmin(12.0f, yawRate));
    s.prevPsi = yaw;
    s.havePrev = true;

    bt::MotionFrame f;
    f.charVel[0] = cx;
    f.charVel[1] = cy;
    f.move[0] = mx;
    f.move[1] = my;
    f.cameraYaw = 0.0f;
    f.bodyYaw = yaw;
    f.bodyTurn = bodyTurn;
    f.targetYaw = target;
    bt::MotionParams p;
    p.stateTargetSpeed = kRunSpeed;
    const bt::DesiredMotion dm = bt::ComputeDesiredMotion(f, p);
    const float speed = std::sqrt(dm.vel[0] * dm.vel[0] + dm.vel[1] * dm.vel[1]);
    float scale = 1.0f, rate = 1.0f;
    if (speed > 0.05f)
    {
        const float dbs = MapSpeedStand(speed);
        scale = dbs / speed;
        rate = std::fmax(0.6f, std::fmin(1.4f, speed / dbs));
    }
    s.lastRate = rate;
    MatcherInput in;
    in.desiredVelocity[0] = dm.vel[0] * scale;
    in.desiredVelocity[1] = dm.vel[1] * scale;
    if (dm.direction)
    {
        in.facingMode = FacingMode::Direction;
        in.desiredFacing[0] = dm.facing[0];
        in.desiredFacing[1] = dm.facing[1];
    }
    else
    {
        in.facingMode = FacingMode::YawRate;
        in.yawRate = yawRate / rate;
    }
    if (bodyTurn)
    {
        in.externalYaw = true;
        in.externalYawStep = yawStep;
        in.externalYawRate = yawRate / rate;
    }
    in.gait = 1;
    s.m.Update(dt * rate, in);
    s.lastAnimRate = s.m.RootYawRate() * rate;
    s.ctl.FeedAnimRate(s.lastAnimRate, dt);
}

// Capsule: velocity follows the world input direction (camera yaw 0 + stick angle) at run speed.
static void MoveCapsule(Sim& s, float dt, bool moving, float dirYaw)
{
    const float gx = moving ? -std::sin(dirYaw) * kRunSpeed : 0.0f;
    const float gy = moving ? std::cos(dirYaw) * kRunSpeed : 0.0f;
    const float k = 1.0f - std::exp(-0.69314718f * dt / 0.05f);
    s.vx += (gx - s.vx) * k;
    s.vy += (gy - s.vy) * k;
}

struct TurnResult
{
    float t5 = std::numeric_limits<float>::infinity(); // s after the reversal until |err| < 5 deg for good
    int transitionsWindow = 0;                          // clip transitions in [0, 1.5 s] after the reversal
    int transitionsAll = 0;                             // [0, 3.5 s]
    float maxCost = 0.0f, meanCost = 0.0f;              // search costs in [0, 1.5 s]
    float overshootDeg = 0.0f;                          // turned beyond the target, deg
    float maxRateDeg = 0.0f;                            // max |body yaw rate| deg/s
    float mismatchDeg = 0.0f;                           // mean |body yaw rate - animation root yaw rate| deg/s in [0, 1.5 s]
    float animLedShare = 0.0f;                          // B: share of commands led by the animation in [0, 1.5 s]
    std::vector<float> psiTrace;
    std::vector<int32_t> frameTrace;
    std::vector<std::string> clips;
    bool finite = true;
};

// Run at 3.85 m/s towards yaw 0 for 2.5 s, then reverse to aTargetDeg and keep running 3.5 s.
static TurnResult RunReversal(const Database& db, bool aUtpp, float aTargetDeg, bool aNanStorm = false)
{
    TurnResult r;
    Sim s;
    if (!InitSim(s, db))
    {
        r.finite = false;
        return r;
    }
    const float dt = 1.0f / 60.0f;
    const int pre = 150, post = 210;
    double pub = 0.0, pubVel = 0.0; // published smoothed angle (deg) and its smoothing velocity
    const float target = bt::Wrap(aTargetDeg * kDeg);
    float turned = 0.0f, prevPsi = 0.0f;
    double costSum = 0.0;
    int costN = 0, animLed = 0, cmds = 0;
    double mismatch = 0.0;
    int mismatchN = 0;
    uint64_t trans0 = 0;
    bool inWindow = false;
    for (int i = 0; i < pre + post; ++i)
    {
        const bool after = i >= pre;
        const float tgt = after ? target : 0.0f;
        const float tAfter = (i - pre) * dt;
        if (i == pre)
            trans0 = s.m.Diagnostics().transitions;
        MoveCapsule(s, dt, true, tgt);
        float feedTarget = tgt;
        if (aNanStorm && after && (i % 7) == 0)
            feedTarget = std::numeric_limits<float>::quiet_NaN();
        Feed(s, dt, 0.0f, 1.0f, aUtpp, feedTarget);
        const MatcherDiagnostics& d = s.m.Diagnostics();
        inWindow = after && tAfter <= 1.5f;
        if (inWindow && d.searchedThisUpdate)
        {
            r.maxCost = std::max(r.maxCost, d.lastSearchCost);
            costSum += d.lastSearchCost;
            ++costN;
        }
        if (d.transitionedThisUpdate)
        {
            if (inWindow)
                r.clips.push_back(std::string(db.ClipName(static_cast<uint32_t>(d.clip))));
        }
        const float before = s.psi;
        if (aUtpp)
        {
            float cmdDt = dt;
            if (aNanStorm && after && (i % 11) == 0)
                cmdDt = std::numeric_limits<float>::quiet_NaN();
            s.psi = s.ctl.Command(s.psi, aNanStorm && (i % 13) == 0 ? std::numeric_limits<float>::quiet_NaN() : tgt, cmdDt);
            if (inWindow)
            {
                ++cmds;
                if (s.ctl.Diagnostics().animLed)
                    ++animLed;
            }
        }
        else
        {
            LocomotionSmooth(pub, static_cast<double>(tgt / kDeg), pubVel, 0.20, dt);
            s.psi = bt::Wrap(static_cast<float>(pub) * kDeg); // camera yaw 0 + published angle, snapRate 1e5
        }
        const float step = bt::Wrap(s.psi - before);
        const float rateDeg = std::fabs(step / dt) / kDeg;
        if (after)
        {
            r.maxRateDeg = std::max(r.maxRateDeg, rateDeg);
            turned += bt::Wrap(s.psi - prevPsi);
            const float over = (std::fabs(turned) - std::fabs(aTargetDeg * kDeg)) / kDeg;
            r.overshootDeg = std::max(r.overshootDeg, over);
            const float err = std::fabs(bt::Wrap(target - s.psi)) / kDeg;
            if (err >= 5.0f)
                r.t5 = std::numeric_limits<float>::infinity();
            else if (!std::isfinite(r.t5))
                r.t5 = tAfter + dt;
        }
        if (inWindow)
        {
            mismatch += std::fabs(step / dt - s.lastAnimRate) / kDeg;
            ++mismatchN;
        }
        prevPsi = s.psi;
        if (i == pre + static_cast<int>(1.5f / dt))
            r.transitionsWindow = static_cast<int>(d.transitions - trans0);
        r.psiTrace.push_back(s.psi);
        r.frameTrace.push_back(d.frame);
        for (const BoneTransform& b : s.m.Output().bones)
            if (!math::IsFinite(b.p) || !math::IsFinite(b.q))
                r.finite = false;
        if (!std::isfinite(s.psi))
            r.finite = false;
    }
    r.transitionsAll = static_cast<int>(s.m.Diagnostics().transitions - trans0);
    r.meanCost = costN ? static_cast<float>(costSum / costN) : 0.0f;
    r.mismatchDeg = mismatchN ? static_cast<float>(mismatch / mismatchN) : 0.0f;
    r.animLedShare = cmds ? static_cast<float>(animLed) / static_cast<float>(cmds) : 0.0f;
    return r;
}

static void Report(const char* aName, const TurnResult& r)
{
    std::printf("  %s: t5=%.3f s, transitions 1.5s=%d (3.5s=%d), cost max=%.2f mean=%.2f, overshoot=%.2f deg, max rate=%.0f deg/s, "
                "|body-anim yaw rate| mean=%.0f deg/s, animLed=%.0f%%\n",
                aName, static_cast<double>(r.t5), r.transitionsWindow, r.transitionsAll, static_cast<double>(r.maxCost),
                static_cast<double>(r.meanCost), static_cast<double>(r.overshootDeg), static_cast<double>(r.maxRateDeg),
                static_cast<double>(r.mismatchDeg), static_cast<double>(r.animLedShare * 100.0f));
    std::printf("    clips:");
    for (const std::string& c : r.clips)
        std::printf(" %s", c.c_str());
    std::printf("\n");
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mm_bodyturn_test <database.utppmm>\n");
        return 2;
    }
    Database db;
    std::string err;
    const bool loaded = db.LoadFromFile(argv[1], ComputeRigHash(kBoneNames, kBoneParents), err);
    Expect("load database", loaded, err);
    if (!loaded)
        return 1;

    // ---------------------------------------------------------------- controller unit checks
    {
        bt::BodyYawController c;
        Expect("default controller config valid", bt::ValidConfig(c.Config()));
        bt::ControllerConfig bad = c.Config();
        bad.correctionTime = std::numeric_limits<float>::quiet_NaN();
        Expect("invalid config rejected", !c.SetConfig(bad));
        bad = c.Config();
        bad.maxCorrectionRate = 0.1f; // below the floor
        Expect("cap below floor rejected", !c.SetConfig(bad));

        // fallback only (animation not turning): 180 deg -> converges, never overshoots, bounded rate
        float y = 0.0f;
        const float tgt = 179.0f * kDeg;
        float t5 = -1.0f, maxRate = 0.0f;
        bool overshoot = false;
        for (int i = 0; i < 600; ++i)
        {
            const float before = y;
            y = c.Command(y, tgt, 1.0f / 60.0f);
            maxRate = std::max(maxRate, std::fabs(bt::Wrap(y - before)) * 60.0f / kDeg);
            if (bt::Wrap(tgt - y) < -1e-5f)
                overshoot = true;
            if (t5 < 0.0f && std::fabs(bt::Wrap(tgt - y)) < 5.0f * kDeg)
                t5 = (i + 1) / 60.0f;
        }
        Expect("fallback converges 179 deg (no animation turn)", t5 > 0.0f && t5 < 2.5f && std::fabs(bt::Wrap(tgt - y)) < 1e-4f,
               Fmt("t5=%.3f s final err=%.5f deg", t5, std::fabs(bt::Wrap(tgt - y)) / kDeg));
        Expect("fallback: no overshoot, rate <= cap", !overshoot && maxRate <= 180.0f + 0.5f, Fmt("max rate %.1f deg/s", maxRate));

        // animation turning towards the target faster than the fallback leads; turning away is ignored
        c.Reset();
        for (int i = 0; i < 30; ++i)
            c.FeedAnimRate(8.0f, 1.0f / 60.0f);
        float y1 = c.Command(0.0f, 3.0f, 1.0f / 60.0f);
        Expect("animation leads when faster", c.Diagnostics().animLed && std::fabs(y1 - 8.0f / 60.0f) < 0.01f,
               Fmt("step %.4f rad (anim %.4f)", y1, c.AnimRate() / 60.0f));
        c.Reset();
        for (int i = 0; i < 30; ++i)
            c.FeedAnimRate(-8.0f, 1.0f / 60.0f);
        float y2 = c.Command(0.0f, 3.0f, 1.0f / 60.0f);
        Expect("animation turning away ignored (fallback towards target)", y2 > 0.0f && !c.Diagnostics().animLed,
               Fmt("step %.4f rad", y2));
        c.Reset();
        for (int i = 0; i < 30; ++i)
            c.FeedAnimRate(30.0f, 1.0f / 60.0f);
        float y3 = c.Command(0.0f, 3.0f, 1.0f / 60.0f);
        Expect("total rate capped (900 deg/s)", std::fabs(y3) <= c.Config().maxRate / 60.0f + 1e-6f, Fmt("step %.4f rad", y3));
        float y4 = c.Command(0.0f, 0.05f, 1.0f / 60.0f);
        Expect("never past the target (fast animation, small error)", std::fabs(y4 - 0.05f) < 1e-6f, Fmt("%.5f", y4));

        // random: never overshoots, finite, bounded
        std::mt19937 rng(1234);
        std::uniform_real_distribution<float> U(-4.0f, 4.0f), D(0.0f, 0.06f), A(-20.0f, 20.0f);
        bool ok = true;
        for (int i = 0; i < 200000 && ok; ++i)
        {
            c.FeedAnimRate(A(rng), D(rng));
            const float cur = U(rng), t = U(rng), dt = D(rng);
            const float e0 = bt::Wrap(t - cur);
            const float n = c.Command(cur, t, dt);
            const float e1 = bt::Wrap(t - n);
            if (!std::isfinite(n) || std::fabs(e1) > std::fabs(e0) + 1e-5f || (e0 * e1 < -1e-8f && std::fabs(e1) > 1e-4f))
                ok = false;
            if (std::fabs(bt::Wrap(n - cur)) > c.Config().maxRate * std::fmin(dt, c.Config().maxDt) + std::fabs(e0) * 0.0f + 1e-4f &&
                std::fabs(e0) > c.Config().snapAngle)
                ok = false;
        }
        Expect("random 200k commands: monotone error, no overshoot, rate bounded, finite", ok);

        // NaN robustness
        const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
        c.Reset();
        c.FeedAnimRate(nan, 1.0f / 60.0f);
        c.FeedAnimRate(inf, nan);
        const float n1 = c.Command(nan, 1.0f, 1.0f / 60.0f), n2 = c.Command(0.5f, nan, 1.0f / 60.0f),
                    n3 = c.Command(0.5f, 1.0f, nan), n4 = c.Command(inf, -inf, inf);
        Expect("NaN/Inf inputs -> finite, no motion on bad target/dt",
               std::isfinite(c.AnimRate()) && std::isfinite(n1) && std::fabs(n2 - 0.5f) < 1e-7f && std::fabs(n3 - 0.5f) < 1e-7f &&
                   std::isfinite(n4),
               Fmt("%.3f %.3f %.3f %.3f", n1, n2, n3, n4));
    }

    // ---------------------------------------------------------------- legacy desired motion unchanged (kill switch)
    {
        std::mt19937 rng(77);
        std::uniform_real_distribution<float> V(-9.0f, 9.0f), M(-1.2f, 1.2f), Y(-4.0f, 4.0f);
        bool same = true;
        for (int i = 0; i < 100000 && same; ++i)
        {
            bt::MotionFrame f;
            f.charVel[0] = V(rng);
            f.charVel[1] = V(rng);
            f.move[0] = (i % 5 == 0) ? 0.0f : M(rng);
            f.move[1] = M(rng);
            f.cameraYaw = Y(rng);
            f.bodyYaw = Y(rng);
            f.bodyTurn = false;
            f.targetYaw = Y(rng);
            bt::MotionParams p;
            p.inputDriven = (i % 3) != 0;
            p.inputCameraRelative = (i % 2) == 0;
            p.stateTargetSpeed = (i % 4 == 0) ? 2.7f : 3.85f;
            const bt::DesiredMotion d = bt::ComputeDesiredMotion(f, p);
            float dx, dy, fx, fy;
            OldDesired(f.charVel[0], f.charVel[1], f.move[0], f.move[1], f.cameraYaw, f.bodyYaw, p.inputDriven, p.inputCameraRelative,
                       p.stateTargetSpeed, dx, dy, fx, fy);
            same = std::memcmp(&d.vel[0], &dx, 4) == 0 && std::memcmp(&d.vel[1], &dy, 4) == 0 &&
                   std::memcmp(&d.facing[0], &fx, 4) == 0 && std::memcmp(&d.facing[1], &fy, 4) == 0 &&
                   d.direction == (p.inputDriven && p.inputCameraRelative);
        }
        Expect("kill switch: legacy desired motion bit-identical to 0.2.8 code (100k random inputs)", same);

        // externalYaw=false must not change the matcher: two matchers, one fed through MatcherInput defaults
        Sim a, b;
        bool ok = InitSim(a, db) && InitSim(b, db);
        for (int i = 0; i < 600 && ok; ++i)
        {
            MatcherInput in;
            in.desiredVelocity[1] = i < 300 ? 5.46f : 0.0f;
            in.facingMode = FacingMode::YawRate;
            in.yawRate = (i > 100 && i < 200) ? 2.0f : 0.0f;
            MatcherInput in2 = in;
            in2.externalYawStep = 1.0f; // ignored while externalYaw is false
            in2.externalYawRate = 5.0f;
            a.m.Update(1.0f / 60.0f, in);
            b.m.Update(1.0f / 60.0f, in2);
            for (size_t k = 0; k < a.m.Output().bones.size(); ++k)
                ok = ok && std::memcmp(&a.m.Output().bones[k], &b.m.Output().bones[k], sizeof(BoneTransform)) == 0;
        }
        Expect("externalYaw=false: external fields ignored (bit-identical output)", ok);
    }

    // ---------------------------------------------------------------- 180 deg reversal at run speed
    std::printf("Reversal at %.2f m/s, target 178 deg (camera fixed):\n", static_cast<double>(kRunSpeed));
    const TurnResult A = RunReversal(db, false, 178.0f);
    const TurnResult B = RunReversal(db, true, 178.0f);
    Report("A fixed-time reference (smooth 0.2 s)", A);
    Report("B UTPP body-yaw command      ", B);
    Expect("A/B finite", A.finite && B.finite);
    Expect("A: fixed-time reference turn reaches 5 deg", std::isfinite(A.t5), Fmt("t5=%.3f s", A.t5));
    Expect("B: converges within 5 deg in < 2.0 s", std::isfinite(B.t5) && B.t5 < 2.0f, Fmt("t5=%.3f s", B.t5));
    Expect("B: overshoot <= 10 deg", B.overshootDeg <= 10.0f, Fmt("%.2f deg", B.overshootDeg));
    Expect("B: yaw rate <= 900 deg/s", B.maxRateDeg <= 900.5f, Fmt("%.0f deg/s", B.maxRateDeg));
    Expect("B: fewer clip transitions than A in the turn (1.5 s)", B.transitionsWindow < A.transitionsWindow,
           Fmt("B %.0f vs A %.0f", B.transitionsWindow, A.transitionsWindow));
    Expect("B: lower max search cost than A in the turn", B.maxCost < A.maxCost, Fmt("B %.2f vs A %.2f", B.maxCost, A.maxCost));
    Expect("B: body yaw closer to the animation root yaw than A", B.mismatchDeg < A.mismatchDeg,
           Fmt("B %.0f vs A %.0f deg/s", B.mismatchDeg, A.mismatchDeg));

    // other reversal angles (both sides) + 90 deg turns
    for (float tdeg : {-178.0f, 135.0f, -90.0f, 90.0f})
    {
        const TurnResult a2 = RunReversal(db, false, tdeg);
        const TurnResult b2 = RunReversal(db, true, tdeg);
        std::printf("Turn %.0f deg:\n", static_cast<double>(tdeg));
        Report("A", a2);
        Report("B", b2);
        Expect(Fmt("turn %.0f: B converges < 2.0 s, overshoot <= 10 deg, rate <= 900 deg/s", tdeg).c_str(),
               b2.finite && std::isfinite(b2.t5) && b2.t5 < 2.0f && b2.overshootDeg <= 10.0f && b2.maxRateDeg <= 900.5f,
               Fmt("t5=%.3f over=%.2f rate=%.0f", b2.t5, b2.overshootDeg, b2.maxRateDeg));
    }

    // determinism
    {
        const TurnResult B2 = RunReversal(db, true, 178.0f);
        const bool same = B2.psiTrace.size() == B.psiTrace.size() &&
                          std::memcmp(B2.psiTrace.data(), B.psiTrace.data(), B.psiTrace.size() * sizeof(float)) == 0 &&
                          B2.frameTrace == B.frameTrace && B2.transitionsAll == B.transitionsAll;
        Expect("determinism: B repeated run bit-identical", same);
    }
    // NaN storm in the target / command dt
    {
        const TurnResult N = RunReversal(db, true, 178.0f, true);
        Expect("NaN target/dt storm: finite pose and yaw, still converges", N.finite && std::isfinite(N.t5),
               Fmt("t5=%.3f", N.t5));
    }

    // stationary: no stick, target = current yaw, idle 5 s -> no drift; also a target but no stick
    {
        Sim s;
        bool ok = InitSim(s, db);
        s.psi = 0.7f;
        float maxDrift = 0.0f;
        for (int i = 0; i < 300 && ok; ++i)
        {
            MoveCapsule(s, 1.0f / 60.0f, false, 0.0f);
            Feed(s, 1.0f / 60.0f, 0.0f, 0.0f, true, 0.7f);
            s.psi = s.ctl.Command(s.psi, 0.7f, 1.0f / 60.0f);
            maxDrift = std::max(maxDrift, std::fabs(bt::Wrap(s.psi - 0.7f)));
        }
        Expect("stationary (idle, target = current): no yaw drift", ok && maxDrift < 1e-6f, Fmt("max drift %.2e rad", maxDrift));
        const std::string_view clip = s.m.Diagnostics().clip >= 0 ? db.ClipName(uint32_t(s.m.Diagnostics().clip)) : "";
        Expect("stationary: idle clip", clip.find("Idle") != std::string_view::npos, std::string(clip));
    }

    // target latch (0.3.1): camera drift during a turn must not move the target
    {
        bt::TargetLatch l;
        const float raw = 180.0f * kDeg;
        float cam = -126.0f * kDeg, body = -126.0f * kDeg;
        const float want = bt::Wrap(cam + raw);
        float maxDev = 0.0f;
        // body turns 5 deg/frame towards the target, the camera drifts 2 deg/frame (0.3.0 leak) until the body is within
        // the latch angle (15 deg); after that the latch releases by design (camera steering is followed again)
        for (int i = 0; i < 64 && std::fabs(bt::Wrap(want - body)) > 15.0f * kDeg; ++i)
        {
            const float t = l.Update(true, bt::Wrap(cam + raw), cam, body);
            maxDev = std::max(maxDev, std::fabs(bt::Wrap(t - want)));
            body = bt::Wrap(body + 5.0f * kDeg);
            cam = bt::Wrap(cam + 2.0f * kDeg);
        }
        Expect("latch: camera drift during the turn does not move the target", maxDev < 1e-5f, Fmt("max dev %.4f deg", maxDev / kDeg));
        const float t2 = l.Update(true, bt::Wrap(cam + 90.0f * kDeg), cam, body);
        Expect("latch: stick change during the turn re-targets", std::fabs(bt::Wrap(t2 - bt::Wrap(cam + 90.0f * kDeg))) < 1e-5f);
        bt::TargetLatch l2;
        l2.Update(true, 0.0f, 0.0f, 0.0f);
        const float t3 = l2.Update(true, 0.2f, 0.2f, 0.0f); // body within 15 deg: camera steering followed
        Expect("latch: outside a turn the target follows camera steering", std::fabs(t3 - 0.2f) < 1e-6f);
        const float t4 = l2.Update(false, 0.7f, 0.0f, 0.0f);
        Expect("latch: no target -> reset", !l2.valid && t4 == 0.7f);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float t5 = l2.Update(true, 1.0f, nan, 0.0f);
        Expect("latch: NaN camera -> pass-through, not latched", t5 == 1.0f && !l2.latched);
    }

    std::printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
