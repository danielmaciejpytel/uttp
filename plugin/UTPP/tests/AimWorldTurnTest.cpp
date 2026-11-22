// Aim turns on the real driver + host path with the world aim target option (SetAimWorldSmoothing, mm.aimWorldSmooth) off and on.
// Stand ADS (pistol 2H overlay, the game upper body off as in the game setting) at a level target 60 m along the camera; the camera
// pans 90 deg at a given rate while the capsule follows it (aim facing, at most kCapsuleFollowDegPerSec), then holds. Per published
// frame: the barrel error (WeaponRight barrel axis against the muzzle -> target direction, deg, in the drawn actor frame) and the
// largest per-frame step of the spine twist (sum of the local rotation steps of Spine .. Spine3, deg). A last case ends one more pan
// with a teleport (0.67 m in one tick) while the crosshair holds still: the root frame reseeds the visual root onto the capsule.
//
//   AimWorldTurnTest <driver-data> <basedb> [woman|man] [--report]
//
// Checks (woman by default):
//   1. option off and on at 0 Hz are identical while the world target does not jump (steady hold and every pan);
//   2. 90 deg pans at 180 / 540 / 1300 deg/s: with the option on (0 Hz) the barrel error p95 during the turn is not above the error
//      of the option off, and the spine step is not larger (the error above the steady value in the faster pans is the +-60 deg
//      clamp of the aim offset while the visual root lags the capsule, the same with and without the option);
//   3. the teleport reseed: barrel error and spine step with the option not above off.
// --report prints the table for off / 0 Hz / 12 Hz without the checks (12 Hz lags the camera: larger barrel error in the pans).
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main

namespace utpp::motiondriver
{
void TestAimOptionFeeds(uint64_t& aNeutralAo, uint64_t& aAimSmooth);
bool SetAimWorldSmoothing(bool aOn, float aHz);
} // namespace utpp::motiondriver

namespace
{
constexpr float kDt = 1.0f / 60.0f;
constexpr float kTargetDist = 60.0f;
constexpr float kOriginZ = 1.4f;             // MotionDriver.cpp kAimOriginZ: the aim angles are taken about this height
constexpr float kMuzzle = 0.25f;             // aim::Params::muzzleOffset
constexpr float kCapsuleFollowDegPerSec = 540.0f; // the aim facing lock rate of the capsule
constexpr float kDeg = 3.14159265358979f / 180.0f;

float AoYaw() { return std::strtof(StatusField(motiondriver::Status(), "aoYaw=").c_str(), nullptr); }

struct XfM
{
    float p[3] = {0, 0, 0};
    float q[4] = {0, 0, 0, 1};
};
XfM ModelXf(const mm::integ::PoseSnapshot& s, const Rig& aR, int32_t aBone)
{
    int32_t path[64];
    int n = 0;
    for (int32_t b = aBone; b >= 0 && n < 64; b = aR.parents[size_t(b)])
        path[n++] = b;
    Xf g;
    for (int i = n - 1; i >= 0; --i)
        g = Mul(g, &s.pose[size_t(path[i]) * 7]);
    XfM r;
    std::memcpy(r.p, g.p, sizeof(r.p));
    std::memcpy(r.q, g.q, sizeof(r.q));
    return r;
}
float QuatAngleDeg(const float* a, const float* b)
{
    const float d = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    return 2.0f * std::acos(std::min(1.0f, d)) / kDeg;
}

struct Sample
{
    float barrelDeg = -1.0f; // < 0 = no published pose
    float spineStepDeg = 0.0f;
    float aoYaw = 0.0f;
    float rawRelDeg = 0.0f; // camera world yaw - committed visual root world yaw (the unclamped root-relative aim yaw)
    float cam = 0.0f, player = 0.0f;
};
struct Run
{
    std::vector<Sample> s;
    std::vector<int> phase; // 0 settle, 1 steady window, 2 turn window (pan + 0.5 s), 3 hold
};

struct Ctx
{
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    int32_t weaponRight = -1, spine[4] = {-1, -1, -1, -1};
};

// One variant (hz < 0 = option off) over the whole scenario sequence; returns the per-frame samples.
Run RunVariant(const Ctx& c, float aHz, bool aSnap)
{
    Run run;
    motiondriver::SetAimWorldSmoothing(false, 0.0f);
    Check(ResetBase(c.rig), "real base loaded");
    motionhost::SetOffsetRoot(true);
    motionhost::SetWeaponUpper(false);
    motionhost::SetHoldPose(true);
    motiondriver::SetEnabled(true);
    motiondriver::SetOverlayInputs(3, 2, false, false);
    motiondriver::SetAimGameUpper(false);
    motiondriver::SetWeaponUpper(false);
    motiondriver::SetWeaponAction(0);
    if (aHz >= 0.0f)
        motiondriver::SetAimWorldSmoothing(true, aHz);
    motionhost::FeedInput f;
    f.dt = kDt;
    f.tppActive = true;
    f.locoState = integ::kLocoStand;
    f.haveAir = true;
    f.onGround = true;
    f.distToGround = 0;
    f.psmBlock = integ::kPsmWeaponKnown | integ::kPsmWeaponDrawn | (uint32_t(6 + 1) << 13);
    float cam = 0.0f, player = 0.0f;
    float prevSpine[4][4];
    bool havePrev = false;
    // one feed: camera world yaw aCam, capsule yaw aPlayer now and aNext after the tick (the drawn actor)
    const auto feed = [&](float aCam, float aPlayer, float aNext, int aPhase) {
        const float rel = (aCam - aPlayer) * kDeg;
        const float point[3] = {-std::sin(rel) * kTargetDist, std::cos(rel) * kTargetDist, kOriginZ};
        motiondriver::SetCameraTarget(true, true, point);
        motiondriver::SetAimTarget(true, true, point);
        f.playerYawDeg = aPlayer;
        f.cameraYawDeg = aCam;
        f.yawAheadDeg = std::remainder(aNext - aPlayer, 360.0f);
        motiondriver::FeedAirFromInput(f);
        g_log = {};
        motionhost::Feed(f);
        Sample smp;
        smp.aoYaw = AoYaw();
        smp.cam = aCam;
        smp.player = aPlayer;
        smp.rawRelDeg = g_log.haveVisual ? std::remainder(aCam - g_log.visualYawDeg, 360.0f) : std::nanf("");
        if (ReadPublished() && g_read.numBones == c.rig.bones && c.weaponRight >= 0)
        {
            const XfM w = ModelXf(g_read, c.rig, c.weaponRight);
            float fwd[3];
            const float axis[3] = {0, 1, 0};
            QRot(w.q, axis, fwd);
            const float relDrawn = (aCam - aNext) * kDeg;
            const float tgt[3] = {-std::sin(relDrawn) * kTargetDist, std::cos(relDrawn) * kTargetDist, kOriginZ};
            float d[3];
            for (int k = 0; k < 3; ++k)
                d[k] = tgt[k] - (w.p[k] + fwd[k] * kMuzzle);
            const float dn = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            const float dot = (fwd[0] * d[0] + fwd[1] * d[1] + fwd[2] * d[2]) / std::max(1e-6f, dn);
            smp.barrelDeg = std::acos(std::clamp(dot, -1.0f, 1.0f)) / kDeg;
            float step = 0.0f;
            for (int k = 0; k < 4; ++k)
            {
                const float* q = &g_read.pose[size_t(c.spine[k]) * 7 + 3];
                if (havePrev)
                    step += QuatAngleDeg(q, prevSpine[k]);
                std::memcpy(prevSpine[k], q, sizeof(prevSpine[k]));
            }
            havePrev = true;
            smp.spineStepDeg = step;
        }
        run.s.push_back(smp);
        run.phase.push_back(aPhase);
    };
    const auto hold = [&](int aFrames, int aPhase) {
        for (int j = 0; j < aFrames; ++j)
            feed(cam, player, player, aPhase);
    };
    hold(150, 0); // settle into Aiming
    hold(60, 1);  // steady window
    for (float rate : {180.0f, 540.0f, 1300.0f})
    {
        const float target = cam + 90.0f;
        int window = -1;
        while (player < target - 1e-3f || window != 0)
        {
            const float nextCam = std::min(target, cam + rate * kDt);
            const float nextPlayer = std::min(nextCam, player + kCapsuleFollowDegPerSec * kDt);
            if (player >= target - 1e-3f && window < 0)
                window = 30; // 0.5 s after the capsule arrived
            feed(cam, player, nextPlayer, 2);
            cam = nextCam;
            player = nextPlayer;
            if (window > 0)
                --window;
            if (window == 0)
                break;
        }
        hold(120, 3);
    }
    if (aSnap)
    {
        // root snap: one more 90 deg pan at 540 deg/s leaves the visual root behind the capsule (the aim offset at its clamp); on the
        // first still frame the capsule moves 0.67 m in one tick (a teleport), the root frame reseeds the visual root onto the capsule
        // and the root-relative aim yaw jumps although the crosshair did not move
        const float target = cam + 90.0f;
        while (player < target - 1e-3f)
        {
            const float next = std::min(target, player + kCapsuleFollowDegPerSec * kDt);
            feed(cam, player, next, 5);
            cam = player = next;
        }
        f.velWorld[0] = 40.0f;
        feed(cam, player, player, 4);
        f.velWorld[0] = 0.0f;
        for (int j = 0; j < 30; ++j)
            feed(cam, player, player, 4);
        hold(60, 3);
    }
    motiondriver::SetAimWorldSmoothing(false, 0.0f);
    motiondriver::SetAimTarget(false, false, nullptr);
    motiondriver::SetCameraTarget(false, false, nullptr);
    return run;
}

struct Stats
{
    float p95 = 0, max = 0, spineMax = 0;
    int n = 0;
};
Stats StatsOf(const Run& r, int aPhase, int aFrom = 0, int aTo = 1 << 30)
{
    std::vector<float> v;
    Stats st;
    int k = 0;
    for (size_t i = 0; i < r.s.size(); ++i)
    {
        if (r.phase[i] != aPhase || r.s[i].barrelDeg < 0.0f)
            continue;
        ++k;
        if (k <= aFrom || k > aTo)
            continue;
        v.push_back(r.s[i].barrelDeg);
        st.max = std::max(st.max, r.s[i].barrelDeg);
        st.spineMax = std::max(st.spineMax, r.s[i].spineStepDeg);
    }
    st.n = static_cast<int>(v.size());
    if (!v.empty())
    {
        std::sort(v.begin(), v.end());
        st.p95 = v[std::min(v.size() - 1, static_cast<size_t>(0.95 * static_cast<double>(v.size())))];
    }
    return st;
}
// per pan window (phase 2 runs separated by phase 3)
std::vector<Stats> PanStats(const Run& r)
{
    std::vector<Stats> out;
    std::vector<float> v;
    Stats cur;
    bool in = false;
    const auto close = [&] {
        if (!in)
            return;
        cur.n = static_cast<int>(v.size());
        std::sort(v.begin(), v.end());
        if (!v.empty())
            cur.p95 = v[std::min(v.size() - 1, static_cast<size_t>(0.95 * static_cast<double>(v.size())))];
        out.push_back(cur);
        cur = {};
        v.clear();
        in = false;
    };
    for (size_t i = 0; i < r.s.size(); ++i)
    {
        if (r.phase[i] != 2)
        {
            close();
            continue;
        }
        in = true;
        if (r.s[i].barrelDeg < 0.0f)
            continue;
        v.push_back(r.s[i].barrelDeg);
        cur.max = std::max(cur.max, r.s[i].barrelDeg);
        cur.spineMax = std::max(cur.spineMax, r.s[i].spineStepDeg);
    }
    close();
    return out;
}
float MaxAoDiff(const Run& a, const Run& b, int aPhaseMax)
{
    float m = 0.0f;
    for (size_t i = 0; i < std::min(a.s.size(), b.s.size()); ++i)
    {
        if (a.phase[i] > aPhaseMax)
            break; // up to the snap pan only
        m = std::max(m, std::fabs(a.s[i].aoYaw - b.s[i].aoYaw) + std::fabs(a.s[i].barrelDeg - b.s[i].barrelDeg));
    }
    return m;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::puts("usage AimWorldTurnTest <driver-data> <basedb> [woman|man] [--report]");
        return 2;
    }
    const bool man = argc > 3 && std::strcmp(argv[3], "man") == 0;
    const bool report = argc > 4 && std::strcmp(argv[4], "--report") == 0;
    const char* body = man ? "man" : "woman";
    Ctx c;
    if (!RigFromDb(DataPaths(argv[1], body).files.db, c.rig, &c.db))
        return 3;
    const char* spineNames[4] = {"Spine", "Spine1", "Spine2", "Spine3"};
    for (uint32_t b = 0; b < c.db->NumBones(); ++b)
    {
        if (c.db->BoneName(b) == "WeaponRight")
            c.weaponRight = int32_t(b);
        for (int k = 0; k < 4; ++k)
            if (c.db->BoneName(b) == spineNames[k])
                c.spine[k] = int32_t(b);
    }
    Check(c.weaponRight >= 0 && std::all_of(std::begin(c.spine), std::end(c.spine), [](int32_t b) { return b >= 0; }),
          "WeaponRight and the spine bones resolve");
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetPathsOverride(0, DataPaths(argv[1], "woman"));
    motiondriver::SetPathsOverride(1, DataPaths(argv[1], "man"));
    motiondriver::SetRigBody(man ? 1 : 0);
    motiondriver::SetGripEnabled(true);
    motionhost::SetMotionHooks(&kHooks);
    Check(ResetBase(c.rig), "real base loaded");
    motiondriver::SetEnabled(true);
    motiondriver::Step({}, 0, kDt);
    motiondriver::WaitLoader();

    const Run off = RunVariant(c, -1.0f, true);
    const Run on0 = RunVariant(c, 0.0f, true);
    const Run on12 = report ? RunVariant(c, 12.0f, true) : Run{};
    const auto row = [&](const char* aName, const Run& r) {
        if (r.s.empty())
            return;
        const Stats steady = StatsOf(r, 1);
        const auto pans = PanStats(r);
        const Stats snap = StatsOf(r, 4, 0, 15);
        std::printf("AIMTURN body=%s variant=%s steadyP95=%.2f steadyMax=%.2f steadySpine=%.2f", body, aName, steady.p95, steady.max,
                    steady.spineMax);
        const char* rates[3] = {"180", "540", "1300"};
        for (size_t k = 0; k < pans.size() && k < 3; ++k)
            std::printf(" pan%s:p95=%.2f,max=%.2f,spine=%.2f,n=%d", rates[k], pans[k].p95, pans[k].max, pans[k].spineMax, pans[k].n);
        std::printf(" snap:p95=%.2f,max=%.2f,spine=%.2f\n", snap.p95, snap.max, snap.spineMax);
    };
    if (std::getenv("UTPP_AIMTURN_TRACE"))
        for (size_t i = 0; i < off.s.size(); ++i)
            if (off.phase[i] >= 4)
                std::printf("TRACE %zu ph=%d cam=%.1f player=%.1f rawRel=%.2f off:ao=%.2f,barrel=%.2f,spine=%.2f on:ao=%.2f,barrel=%.2f,spine=%.2f\n",
                            i, off.phase[i], off.s[i].cam, off.s[i].player, off.s[i].rawRelDeg, off.s[i].aoYaw, off.s[i].barrelDeg,
                            off.s[i].spineStepDeg, i < on0.s.size() ? on0.s[i].aoYaw : 0.0f, i < on0.s.size() ? on0.s[i].barrelDeg : 0.0f,
                            i < on0.s.size() ? on0.s[i].spineStepDeg : 0.0f);
    row("off", off);
    row("hz0", on0);
    row("hz12", on12);

    // 1. identical while nothing jumps (steady, pans)
    const float diff = MaxAoDiff(off, on0, 3);
    std::printf("  max |aoYaw| + |barrel| difference off vs 0 Hz before the snap: %.5f\n", diff);
    if (!report)
    {
        Check(diff < 1e-4f, "world aim target at 0 Hz equals the option off while nothing jumps");
        // 2. pans: not worse
        const auto po = PanStats(off), p0 = PanStats(on0);
        Check(po.size() == 3 && p0.size() == 3, "three pan windows");
        for (size_t k = 0; k < std::min(po.size(), p0.size()); ++k)
        {
            Check(p0[k].p95 <= po[k].p95 + 0.05f, "pan " + std::to_string(k) + ": barrel error p95 with the option not above off");
            Check(p0[k].spineMax <= po[k].spineMax + 0.05f, "pan " + std::to_string(k) + ": spine step with the option not above off");
        }
        // 3. root reseed (teleport) with a still crosshair: not worse with the option
        const Stats snapOn = StatsOf(on0, 4, 0, 15), snapOff = StatsOf(off, 4, 0, 15);
        Check(snapOn.n > 0 && snapOn.max <= snapOff.max + 0.05f, "root reseed: barrel error with the option not above off");
        Check(snapOn.spineMax <= snapOff.spineMax + 0.05f, "root reseed: spine step with the option not above off");
    }
    motionhost::SetMotionHooks(nullptr);
    std::printf("%s (%d failures)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
