// Shooter mode + H2 safeguard through the REAL v1 MMDriver + MotionDriver (no RED4ext / game), like the other focused driver tests: it includes the
// integration test for its helpers (feed loader, hooks, RunFeed with the foot metric inputs, FrameOut).
// Usage: utpp_motion_driver_shooter_mode_test <data dir> <v1 database.utppmm> <flick_180_run.feed> <run_arc_left.feed>
//        utpp_motion_driver_shooter_mode_test <data dir> <v1 db> <flick.feed> <arc.feed> --slide <slide bundle prefix>  (only the slide case, the bundle
//        must declare the slide section: <prefix>.utppmm + <prefix>.chooser.json)
//        utpp_motion_driver_shooter_mode_test ... --explore <case>   (development aid: time series of a synthetic case)
//
// Every case starts from RestoreDefaults() (SetShooterMode(false), SetH2Guard(true), SetBodyTurn(true)); before the OFF / identity cases the flags are
// asserted from Status() (shooterMode=0 h2Guard=1): the flags are process-lifetime and would otherwise leak from one case into the next.
// Cases: F-ORBIT-IDLE, F-ORBIT-START, F-TIP-RELEASE, A2 F-OPEN, A3 F-FLICK, A5 F-SPRINT, A6 H2, A7 OFF-ID, A10 F-AIM, the slide case.
// Mutation builds (-DUTPP_SHOOTER_MODE_MUTATION=k) MUST print a line starting "FAIL " (see tests/shooter_mode.cmake).
#define main UTPPUnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include "../src/mm/AimOffset.hpp"
#include "../src/mm/VisualRootFrame.hpp"
#include <functional>
#include <limits>

namespace
{
constexpr float kRadToDeg = 57.29577951f;
constexpr float kDt = 1.0f / 60.0f;

float Wrap(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a <= -180.0f)
        a += 360.0f;
    return a;
}
std::string Fmt(double v, int prec = 2)
{
    char b[48];
    std::snprintf(b, sizeof b, "%.*f", prec, v);
    return b;
}

// Setup problems (missing data, a feed that does not load, a bundle without the slide section) are NOT assertions: they print "SETUP-ERROR", never
// "FAIL ", and end the run with exit code 3, so a mutation build can only be killed by a real assertion of a case (tests/shooter_mode.cmake).
int g_setupErr = 0;
void Setup(bool aOk, const std::string& aWhat, const std::string& aDetail = {})
{
    if (aOk)
        return;
    ++g_setupErr;
    std::printf("SETUP-ERROR %s%s%s\n", aWhat.c_str(), aDetail.empty() ? "" : " | ", aDetail.c_str());
}

// ------------------------------------------------------------------ process-lifetime flags
void RestoreDefaults()
{
    motiondriver::SetShooterMode(false);
    motiondriver::SetH2Guard(true);
    motionhost::SetBodyTurn(true);
    motiondriver::SetPelvisYawTrim(0.0f);
    motiondriver::SetBodyYawOffset(0.0f);
    motiondriver::SetBarrelLoop(false);
    motiondriver::SetBarrelAxis(aim::kDefaultBarrelAxis);
    motiondriver::SetBarrelMuzzle(aim::Params{}.muzzleOffset);
}
bool DefaultsInStatus(std::string& aWhy)
{
    const std::string st = motiondriver::Status();
    const std::string fn = StatusField(st, "shooterMode="), hg = StatusField(st, "h2Guard="), bt = StatusField(st, "btSwitch=");
    aWhy = "shooterMode=" + fn + " h2Guard=" + hg + " btSwitch=" + bt;
    // btSwitch is the body-turn state of the LAST Step (true until a Step ran with the switch off); RestoreDefaults sets the switch itself
    return fn == "0" && hg == "1";
}
// the CET contract of Shooter mode: the native body turn is off, Locomotion is off (vanilla camera-relative movement)
void ShooterModeOn()
{
    motiondriver::SetShooterMode(true);
    motionhost::SetBodyTurn(false);
}

// ------------------------------------------------------------------ synthetic feed generator (closed loop capsule: yaw = camera, Shooter follow mode)
constexpr uint32_t kUpperAim = uint32_t(adapter::kUpperBodyAim + 1) << integ::kPsmUpperShift;
constexpr uint32_t kWeaponKnown = integ::kPsmWeaponKnown, kWeaponDrawn = integ::kPsmWeaponKnown | integ::kPsmWeaponDrawn;
// LocomotionDetailed d (PsmDetailOf) in the packed mask: (d + 1) << 16 with the kPsmDetailed bit
uint32_t WithDetail(uint32_t aPsm, int aDetail) { return aPsm | integ::kPsmDetailed | (uint32_t(aDetail + 1) << integ::kPsmDetailShift); }

struct Gen
{
    FeedFile f;
    std::vector<uint8_t> act; // weapon action per frame (0 none, 2 reload, 8 shoot)
    float t = 0, cam = 0, vx = 0, vy = 0;
    int loco = integ::kLocoStand;
    uint32_t psm = kWeaponKnown;  // empty hands known, nothing drawn
    int action = 0;
    float maxSpeed = 3.85f;       // game m/s at full stick, forward
    bool speedMul = false;        // the reds multiplier: sideways 0.70 / backward 0.60 of the forward speed
    float accel = 12.0f, brake = 16.0f; // m/s^2
    explicit Gen(const std::string& aName) { f.name = aName; }
    // aStickDeg: stick angle against the camera (0 forward, +90 left, 180 back), aMag 0 = released. aCamTo: camera yaw at the end (linear ramp,
    // aCamJump: the whole turn in the first frame instead).
    void Seg(float aDur, float aStickDeg, float aMag, float aCamTo, bool aCamJump = false)
    {
        const int n = std::max(1, static_cast<int>(std::lround(aDur / kDt)));
        const float cam0 = cam;
        for (int i = 0; i < n; ++i)
        {
            Frame fr;
            t += kDt;
            cam = Wrap(aCamJump ? aCamTo : cam0 + Wrap(aCamTo - cam0) * float(i + 1) / float(n));
            float tx = 0, ty = 0;
            if (aMag > 0.01f)
            {
                const float w = (cam + aStickDeg) / kRadToDeg;
                float spd = maxSpeed * (loco == integ::kLocoCrouch ? 0.7f : 1.0f) * std::fmin(aMag, 1.0f);
                if (speedMul) // exactly what the game does: UTPPMotionShooterModeSpeedMul scales MaxSpeed (body-relative stick)
                    spd *= adapter::ShooterModeSpeedRatio(-std::sin(aStickDeg / kRadToDeg) * aMag, std::cos(aStickDeg / kRadToDeg) * aMag,
                                                    loco == integ::kLocoCrouch ? adapter::kPsmCrouch : adapter::kPsmStand);
                tx = -std::sin(w) * spd;
                ty = std::cos(w) * spd;
            }
            const float dx = tx - vx, dy = ty - vy, d = std::hypot(dx, dy);
            const float lim = (aMag > 0.01f ? accel : brake) * kDt;
            if (d <= lim || d < 1e-6f)
                vx = tx, vy = ty;
            else
                vx += dx / d * lim, vy += dy / d * lim;
            fr.t = t;
            fr.dt = kDt;
            fr.vx = vx;
            fr.vy = vy;
            if (aMag > 0.01f)
            {
                const float s = aStickDeg / kRadToDeg;
                fr.mvx = -std::sin(s) * aMag; // camera-space stick: x right, y forward; stick angle +90 = left -> x = -1
                fr.mvy = std::cos(s) * aMag;
            }
            fr.yawDeg = cam;
            fr.camDeg = cam;
            fr.loco = loco;
            fr.psm = static_cast<int>(psm);
            f.fr.push_back(fr);
            act.push_back(static_cast<uint8_t>(action));
        }
    }
    void Idle(float aDur) { Seg(aDur, 0, 0, cam); }
    size_t Size() const { return f.fr.size(); }
};

// ------------------------------------------------------------------ driver runs
struct Rt
{
    const Rig* rig = nullptr;
    const mm::motion::MotionDatabase* db = nullptr;
    MetricCtx metrics;
};
struct Run
{
    std::vector<FrameOut> out;
    std::vector<motiondriver::StrafeProbe> probe; // probe[i] = the strafe state of frame i
    std::shared_ptr<FeedFile> feedOwner; // the run owns a copy of its feed (RootErr reads it)
    const FeedFile* feed = nullptr;
};
struct Opts
{
    bool raw = false;      // kHooksRaw (the pre-leg-tail v2 pose: the slide reference of check A)
    int overlay = 0;       // 0 none, 3 Pistol2H (weapon drawn + camera target heartbeat per frame)
    bool slide = false;    // mm.slide on
    size_t releaseFrom = 0, releaseTo = 0; // frames [from, to) with the overlay release (v2 skips them: gate / re-entry cases)
    std::function<void(size_t)> onFrame;   // optional probe, called before feed i (sees the driver state after feed i - 1)
    const motionhost::MotionHooks* hooks = nullptr; // optional frame-observing hooks (default kHooks; ignored with raw)
};
Run RunSynth(Rt& rt, const Gen& aG, const Opts& aO = {})
{
    const FeedFile& f = aG.f;
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(aO.overlay, 1, false, false);
    motiondriver::SetAimGameUpper(false);
    motiondriver::SetWeaponUpper(false);
    motiondriver::SetGripEnabled(aO.overlay != 0);
    motiondriver::SetWeaponAction(0);
    motiondriver::SetCameraTarget(false, false, nullptr);
    motiondriver::SetSlide(aO.slide);
    motionhost::SetMotionHooks(aO.raw ? &kHooksRaw : (aO.hooks ? aO.hooks : &kHooks));
    ResetBase(*rt.rig);
    motionhost::SetOffsetRoot(true);
    motionhost::SetHoldPose(true);
    motionhost::SetWeaponUpper(false);
    motiondriver::SetEnabled(false);
    motiondriver::SetEnabled(true);
    motionhost::Feed(ToInput(f.fr.front()));
    motiondriver::WaitLoader();
    ResetBase(*rt.rig);
    motionhost::SetOffsetRoot(true);
    motionhost::SetHoldPose(true);
    motionhost::SetWeaponUpper(false);
    motiondriver::SetEnabled(false);
    motiondriver::SetEnabled(true);
    Run r;
    r.feedOwner = std::make_shared<FeedFile>(f);
    r.feed = r.feedOwner.get();
    g_metrics = &rt.metrics;
    r.out = RunFeed(f, [&](size_t i) {
        if (i > 0)
            r.probe.push_back(motiondriver::TestStrafeProbe());
        if (aO.onFrame)
            aO.onFrame(i);
        const Frame& fr = f.fr[i];
        motiondriver::SetWeaponAction(aG.act[i]);
        const bool ads = aO.overlay != 0 && integ::PsmUpperBodyOf(static_cast<uint32_t>(fr.psm)) == adapter::kUpperBodyAim;
        motiondriver::SetOverlayInputs(aO.overlay, ads ? 2 : 1, i >= aO.releaseFrom && i < aO.releaseTo, false);
        if (aO.overlay != 0)
        {
            const float yaw = fr.camDeg * integ::kDegToRad;
            const float point[3] = {-std::sin(yaw) * 60.0f, std::cos(yaw) * 60.0f, 1.4f};
            motiondriver::SetCameraTarget(true, true, point);
        }
    });
    r.probe.push_back(motiondriver::TestStrafeProbe());
    g_metrics = nullptr;
    motiondriver::SetSlide(false);
    motiondriver::SetWeaponAction(0);
    motiondriver::SetCameraTarget(false, false, nullptr);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    motionhost::SetMotionHooks(&kHooks);
    return r;
}
Run RunOnFeed(Rt& rt, const FeedFile& aF, const Opts& aO = {})
{
    Gen g("recorded");
    g.f = aF;
    g.act.assign(aF.fr.size(), 0);
    return RunSynth(rt, g, aO);
}

std::string DbName(const Rt& rt, int32_t aDb) { return aDb >= 0 && uint32_t(aDb) < rt.db->NumDatabases() ? std::string(rt.db->DbName(uint32_t(aDb))) : std::string("-"); }
std::string ClipName(const Rt& rt, int32_t aClip) { return aClip >= 0 && uint32_t(aClip) < rt.db->NumClips() ? std::string(rt.db->ClipName(uint32_t(aClip))) : std::string("-"); }
bool Has(const std::string& s, const char* sub) { return s.find(sub) != std::string::npos; }

// new picks (another clip, or a clip-time jump) of frames whose database / clip satisfies aPred, in [aFrom, aTo)
template<typename P>
std::vector<size_t> Picks(const Rt& rt, const Run& r, size_t aFrom, size_t aTo, P&& aPred)
{
    std::vector<size_t> picks;
    bool prevValid = false;
    int32_t prevClip = -1;
    float prevT = 0;
    for (size_t i = 0; i < r.out.size() && i < aTo; ++i)
    {
        const FrameOut& o = r.out[i];
        if (!o.stepped || o.clip < 0)
        {
            prevValid = false;
            continue;
        }
        const bool newPick = !prevValid || o.clip != prevClip || std::fabs(o.clipTime - prevT - kDt) > 0.05f;
        if (i >= aFrom && newPick && aPred(DbName(rt, o.db), ClipName(rt, o.clip)))
            picks.push_back(i);
        prevValid = true;
        prevClip = o.clip;
        prevT = o.clipTime;
    }
    return picks;
}
bool IsTip(const std::string& db, const std::string&) { return Has(db, "TurnInPlace"); }
bool IsSpin(const std::string& db, const std::string&) { return Has(db, "SpinTransition"); }
bool IsReface(const std::string&, const std::string& clip) { return Has(clip, "Reface"); }
bool IsTurnLike(const std::string& db, const std::string& clip) { return IsTip(db, clip) || IsSpin(db, clip) || IsReface(db, clip); }
bool IsSpinOrReface(const std::string& db, const std::string& clip) { return IsSpin(db, clip) || IsReface(db, clip); }

float RootErr(const Run& r, size_t i) { return std::fabs(Wrap(r.feed->fr[i].yawDeg - r.out[i].visualYawDeg)); }
float MaxVisualStep(const Run& r, size_t aFrom, size_t aTo)
{
    float m = 0;
    for (size_t i = std::max<size_t>(aFrom, 1); i < aTo && i < r.out.size(); ++i)
        if (r.out[i].haveVisual && r.out[i - 1].haveVisual)
            m = std::fmax(m, std::fabs(Wrap(r.out[i].visualYawDeg - r.out[i - 1].visualYawDeg)));
    return m;
}
size_t SteppedCount(const Run& r, size_t aFrom, size_t aTo)
{
    size_t n = 0;
    for (size_t i = aFrom; i < aTo && i < r.out.size(); ++i)
        n += r.out[i].stepped && r.out[i].haveVisual;
    return n;
}
// first frame in [aFrom, aTo) whose actor-root error is <= aDeg; SIZE_MAX when none
size_t FirstWithin(const Run& r, size_t aFrom, size_t aTo, float aDeg)
{
    for (size_t i = aFrom; i < aTo && i < r.out.size(); ++i)
        if (r.out[i].haveVisual && RootErr(r, i) <= aDeg)
            return i;
    return SIZE_MAX;
}

// planted-toe slide (cm per planted frame) of a run in [aFrom, aTo): the contact the leg tail was given by the TAIL run (frame aligned), the toe of
// aRun: the integration check A metric (MotionDriverIntegrationTest.cpp "foot lock on v2 frames")
double PlantedSlide(const Run& aTail, const Run& aRun, size_t aFrom, size_t aTo, size_t& aN)
{
    double sum = 0;
    aN = 0;
    bool prevPlanted[2] = {false, false};
    float prevToe[2][2] = {{0, 0}, {0, 0}};
    const size_t n = std::min({aTo, aTail.out.size(), aRun.out.size()});
    for (size_t i = aFrom; i < n; ++i)
        for (int k = 0; k < 2; ++k)
        {
            const bool planted = aTail.out[i].haveContact && aRun.out[i].toeOk[k] && aTail.out[i].tailContact[k] >= 0.5f;
            if (planted && prevPlanted[k])
            {
                sum += std::hypot(aRun.out[i].toeW[k][0] - prevToe[k][0], aRun.out[i].toeW[k][1] - prevToe[k][1]);
                ++aN;
            }
            prevPlanted[k] = planted;
            prevToe[k][0] = aRun.out[i].toeW[k][0];
            prevToe[k][1] = aRun.out[i].toeW[k][1];
        }
    return aN ? 100.0 * sum / double(aN) : -1.0;
}
struct SlideCmp
{
    double tail = -1, raw = -1;
    size_t nTail = 0, nRaw = 0;
    bool Ok() const { return nTail > 20 && nRaw > 20 && tail < raw; }
    std::string Text() const { return "tail " + Fmt(tail, 3) + " cm (n " + std::to_string(nTail) + ") raw " + Fmt(raw, 3) + " cm (n " + std::to_string(nRaw) + ")"; }
};
SlideCmp CompareSlide(const Run& aTail, const Run& aRaw, size_t aFrom, size_t aTo)
{
    SlideCmp s;
    s.tail = PlantedSlide(aTail, aTail, aFrom, aTo, s.nTail);
    s.raw = PlantedSlide(aTail, aRaw, aFrom, aTo, s.nRaw);
    return s;
}

void PrintTable(const Rt& rt, const Run& r, size_t aFrom, size_t aTo, size_t aStride)
{
    std::printf("  frame    t   cam  stickMag vel   visual  err  rot fn h2 spin gait  db:clip@time\n");
    for (size_t i = aFrom; i < aTo && i < r.out.size(); i += aStride)
    {
        const Frame& f = r.feed->fr[i];
        const FrameOut& o = r.out[i];
        const motiondriver::StrafeProbe& p = r.probe[i];
        std::printf("  %5zu %6.2f %7.1f %4.2f %5.2f %8.1f %6.1f  %d  %d  %d  %d  %d  %s:%s@%.2f%s\n", i, static_cast<double>(f.t), static_cast<double>(f.yawDeg),
                    static_cast<double>(std::hypot(f.mvx, f.mvy)), static_cast<double>(std::hypot(f.vx, f.vy)), static_cast<double>(o.visualYawDeg),
                    static_cast<double>(RootErr(r, i)), p.rotationMode, p.shooterModeActive ? 1 : 0, p.h2Active ? 1 : 0, p.shouldSpin ? 1 : 0, p.gait,
                    DbName(rt, o.db).c_str(), ClipName(rt, o.clip).c_str(), static_cast<double>(o.clipTime), o.stepped ? "" : " (not stepped)");
    }
}

// ------------------------------------------------------------------ recorded-feed helpers (ported from the spin-guard harness, tests a77e03a)
struct Episode
{
    size_t first = 0, last = 0;
};
constexpr float kReversalMinSpeed = 1.5f, kReversalDeg = 130.0f, kEpisodeGapS = 0.5f;
// runs of frames at >= 1.5 m/s whose velocity heading is >= 130 deg away from the player yaw (aOnlyReversed false: every run at speed)
std::vector<Episode> ReversalEpisodes(const FeedFile& aF, bool aOnlyReversed = true)
{
    std::vector<Episode> eps;
    for (size_t i = 0; i < aF.fr.size(); ++i)
    {
        const Frame& f = aF.fr[i];
        const bool reversed = std::hypot(f.vx, f.vy) >= kReversalMinSpeed &&
            (!aOnlyReversed || std::fabs(Wrap(std::atan2(-f.vx, f.vy) * kRadToDeg - f.yawDeg)) >= kReversalDeg);
        if (!reversed)
            continue;
        if (!eps.empty() && aF.fr[i].t - aF.fr[eps.back().last].t <= kEpisodeGapS)
            eps.back().last = i;
        else
            eps.push_back({i, i});
    }
    return eps;
}
// a feed whose player yaw follows the velocity heading at aRateDegS (the normal mode of the game: Locomotion + native body turn)
FeedFile ClosedLoop(const FeedFile& aOpen, float aRateDegS, int aRepeat)
{
    FeedFile out;
    out.name = aOpen.name + "_follow" + std::to_string(static_cast<int>(aRateDegS)) + (aRepeat > 1 ? "x" + std::to_string(aRepeat) : std::string());
    float yaw = aOpen.fr.front().yawDeg;
    float t = 0.0f;
    for (int rep = 0; rep < aRepeat; ++rep)
        for (const Frame& src : aOpen.fr)
        {
            Frame f = src;
            const float dt = (std::isfinite(f.dt) && f.dt > 0.0f) ? f.dt : kDt;
            if (std::hypot(f.vx, f.vy) > 0.5f)
            {
                const float target = std::atan2(-f.vx, f.vy) * kRadToDeg;
                const float d = Wrap(target - yaw);
                yaw = Wrap(yaw + std::fmax(-aRateDegS * dt, std::fmin(aRateDegS * dt, d)));
            }
            f.yawDeg = std::round(yaw * 1000.0f) / 1000.0f;
            t += dt;
            f.t = t;
            out.fr.push_back(f);
        }
    return out;
}

struct Counter
{
    int fails0 = 0;
};

// ------------------------------------------------------------------ cases
// F-ORBIT-IDLE: V stands without input; the capsule (yaw = camera, follow mode) turns 0 -> 180 -> 0 in 2 s. The visual root must not move.
void OrbitIdleCase(Rt& rt, const std::string& aName, int aLoco, bool aArmed)
{
    RestoreDefaults();
    Gen g("orbit-idle " + aName);
    g.loco = aLoco;
    g.psm = aArmed ? kWeaponDrawn : kWeaponKnown;
    g.Idle(aLoco == integ::kLocoCrouch ? 3.5f : 1.5f);
    const size_t w0 = g.Size();
    g.Seg(1.0f, 0, 0, 180.0f);
    g.Seg(1.0f, 0, 0, 0.0f);
    const size_t w1 = g.Size();
    g.Idle(0.5f);
    ShooterModeOn();
    const Run r = RunSynth(rt, g, Opts{false, aArmed ? 3 : 0, false});
    float maxDelta = 0;
    for (size_t i = w0; i < w1; ++i)
        maxDelta = std::fmax(maxDelta, std::fabs(Wrap(r.out[i].visualYawDeg - r.out[w0 - 1].visualYawDeg)));
    const size_t stepped = SteppedCount(r, w0, w1);
    const auto tips = Picks(rt, r, w0, w1 + 30, IsTip);
    const auto turns = Picks(rt, r, w0, w1 + 30, IsTurnLike);
    size_t fnFrames = 0;
    for (size_t i = w0; i < w1; ++i)
        fnFrames += r.probe[i].shooterModeActive;
    Check(stepped == w1 - w0, "F-ORBIT-IDLE " + aName + ": motion stepped with a committed visual root on every frame of the orbit", std::to_string(stepped) + "/" + std::to_string(w1 - w0));
    Check(maxDelta <= 1.0f, "F-ORBIT-IDLE " + aName + ": the visual root does not move while the capsule orbits 0 -> 180 -> 0 (|delta| <= 1 deg)",
          "max |delta| " + Fmt(maxDelta, 3) + " deg");
    Check(tips.empty() && turns.empty(), "F-ORBIT-IDLE " + aName + ": no turn-in-place / spin / reface pick during the orbit",
          "TIP picks " + std::to_string(tips.size()) + ", turn-like picks " + std::to_string(turns.size()));
    Check(fnFrames * 100 >= (w1 - w0) * 95, "F-ORBIT-IDLE " + aName + ": Shooter Strafe mapping active on the orbit frames", std::to_string(fnFrames) + "/" + std::to_string(w1 - w0));
    RestoreDefaults();
}

// F-ORBIT-RELOAD: a standing reload (weapon action 2) with a held offset does not release the root either (reload / no action do not release)
void OrbitReloadCase(Rt& rt)
{
    RestoreDefaults();
    Gen g("orbit-reload");
    g.psm = kWeaponDrawn;
    g.Idle(1.5f);
    g.Seg(1.0f, 0, 0, 120.0f);
    g.Idle(0.3f);
    const size_t r0 = g.Size();
    g.action = 2;
    g.Idle(2.0f);
    g.action = 0;
    g.Idle(0.5f);
    ShooterModeOn();
    const Run r = RunSynth(rt, g, Opts{false, 3, false});
    float maxDelta = 0;
    for (size_t i = r0; i < g.Size(); ++i)
        maxDelta = std::fmax(maxDelta, std::fabs(Wrap(r.out[i].visualYawDeg - r.out[r0 - 1].visualYawDeg)));
    const auto turns = Picks(rt, r, r0, g.Size(), IsTurnLike);
    Check(maxDelta <= 1.0f && turns.empty(), "F-ORBIT-RELOAD: a standing reload with a held offset keeps the root (|delta| <= 1 deg, no turn pick)",
          "max |delta| " + Fmt(maxDelta, 3) + " deg, turn picks " + std::to_string(turns.size()));
    RestoreDefaults();
}

// F-ORBIT-IDLE-OWNED: an aim episode leaves a hidden offset that UTPP's own turn owns; a slow orbit afterwards must not hand it to the rest turn-in-place
void OrbitOwnedCase(Rt& rt, bool aShooterMode)
{
    RestoreDefaults();
    const std::string name = aShooterMode ? "owned offset, slow orbit" : "owned offset, slow orbit, Shooter OFF (reference)";
    Gen g("orbit-owned");
    g.psm = kWeaponDrawn;
    g.Idle(1.5f);
    g.Seg(0.05f, 0, 0, 110.0f, true); // the mouse takes the capsule 110 deg away
    g.psm = kWeaponDrawn | kUpperAim;
    g.Seg(0.12f, 0, 0, 110.0f);       // ADS for 0.12 s: the owner of the facing, the aim turn starts
    g.psm = kWeaponDrawn;
    const size_t release = g.Size();
    g.Seg(3.0f, 0, 0, 200.0f);        // slow orbit 30 deg/s
    g.Idle(0.3f);
    if (aShooterMode)
        ShooterModeOn();
    else
        motionhost::SetBodyTurn(false);
    const Run r = RunSynth(rt, g, Opts{false, 3, false});
    const auto tips = Picks(rt, r, release + 6, g.Size(), IsTip);
    float maxDelta = 0;
    for (size_t i = release + 12; i < g.Size(); ++i)
        maxDelta = std::fmax(maxDelta, std::fabs(Wrap(r.out[i].visualYawDeg - r.out[release + 12].visualYawDeg)));
    std::printf("INFO F-ORBIT-IDLE %s: TIP picks after the release %zu, root moved %s deg\n", name.c_str(), tips.size(), Fmt(maxDelta).c_str());
    if (aShooterMode)
        Check(tips.empty() && maxDelta <= 1.0f, "F-ORBIT-IDLE " + name + ": the orbit is foreign to the rest turn-in-place (no TIP, root holds)",
              "TIP picks " + std::to_string(tips.size()) + ", root moved " + Fmt(maxDelta) + " deg");
    else
        Check(!tips.empty() || maxDelta > 5.0f, "F-ORBIT-IDLE " + name + ": today's behaviour turns the owned offset back (the case discriminates)",
              "TIP picks " + std::to_string(tips.size()) + ", root moved " + Fmt(maxDelta) + " deg");
    RestoreDefaults();
}

// F-ORBIT-START: after the idle orbit the root holds aOffset deg behind the capsule; forward input from the first frame, velocity along the camera
void OrbitStartCase(Rt& rt, int aLoco, float aOffset)
{
    RestoreDefaults();
    const std::string name = std::string(aLoco == integ::kLocoCrouch ? "crouch" : "stand") + " offset " + std::to_string(static_cast<int>(aOffset));
    Gen g("orbit-start " + name);
    g.loco = aLoco;
    g.Idle(aLoco == integ::kLocoCrouch ? 3.5f : 1.0f);
    g.Seg(1.5f, 0, 0, aOffset);
    g.Idle(0.5f);
    const size_t s = g.Size();
    g.Seg(2.5f, 0, 1.0f, aOffset);
    ShooterModeOn();
    const Run tail = RunSynth(rt, g);
    const Run raw = RunSynth(rt, g, Opts{true, 0, false});
    const size_t win = 36; // 0.6 s
    const size_t within = FirstWithin(tail, s, s + 150, 30.0f);
    const float step = MaxVisualStep(tail, s, s + 150);
    const auto turns = Picks(rt, tail, s - 3, s + 180, IsTurnLike);
    const SlideCmp sl = CompareSlide(tail, raw, s, s + win);
    std::string pk;
    for (const size_t p : turns)
        pk += DbName(rt, tail.out[p].db) + ":" + ClipName(rt, tail.out[p].clip) + "@" + std::to_string(p - s) + " ";
    Check(SteppedCount(tail, s, s + 150) == 150, "F-ORBIT-START " + name + ": motion stepped with a committed visual root on every frame");
    Check(within != SIZE_MAX && within - s <= win, "F-ORBIT-START " + name + ": |actor - root| <= 30 deg within 0.6 s of the first moving frame",
          within == SIZE_MAX ? "never" : Fmt(double(within - s) * kDt) + " s");
    Check(step <= 10.0f, "F-ORBIT-START " + name + ": max visual yaw step <= 10 deg per feed (no snap)", Fmt(step) + " deg");
    Check(turns.size() <= 1, "F-ORBIT-START " + name + ": at most one turn / spin / reface pick in the episode", std::to_string(turns.size()) + " " + pk);
    Check(sl.Ok(), "F-ORBIT-START " + name + ": foot slide in the turn window (0.6 s) lower than the raw motion pose", sl.Text());
    RestoreDefaults();
}

// F-TIP-RELEASE: armed V, held offset after the orbit, then ADS (UpperBody 6) or a hip-fire shot, standing / crouching
void TipReleaseCase(Rt& rt, int aLoco, float aOffset, bool aAds)
{
    RestoreDefaults();
    const std::string name = std::string(aLoco == integ::kLocoCrouch ? "crouch " : "stand ") + (aAds ? "ADS" : "hip fire") + " offset " + std::to_string(static_cast<int>(aOffset));
    Gen g("tip-release " + name);
    g.loco = aLoco;
    g.psm = kWeaponDrawn;
    g.Idle(aLoco == integ::kLocoCrouch ? 3.5f : 1.5f);
    g.Seg(1.5f, 0, 0, aOffset);
    g.Idle(0.5f);
    const size_t s = g.Size();
    if (aAds)
    {
        g.psm = kWeaponDrawn | kUpperAim;
        g.Idle(3.0f);
    }
    else
    {
        g.action = 8;
        g.Idle(0.05f); // one shot pulse
        g.action = 0;
        g.Idle(3.0f);
    }
    ShooterModeOn();
    const Run tail = RunSynth(rt, g, Opts{false, 3, false});
    const Run raw = RunSynth(rt, g, Opts{true, 3, false});
    RestoreDefaults();
    const Run base = RunSynth(rt, g, Opts{false, 3, false}); // the current mode (Shooter off, body turn on): the existing ADS-TIP / shot-turn path
    const auto tips = Picks(rt, tail, s, g.Size(), IsTip);
    const size_t firstTip = tips.empty() ? SIZE_MAX : tips.front();
    const size_t within50 = FirstWithin(tail, s, s + 120, 49.9f);
    const float step = MaxVisualStep(tail, s, s + 150);
    size_t tipEnd = within50 == SIZE_MAX ? s + 120 : within50;
    const SlideCmp sl = CompareSlide(tail, raw, firstTip == SIZE_MAX ? s : firstTip, tipEnd);
    Check(firstTip != SIZE_MAX && firstTip - s <= 30, "F-TIP-RELEASE " + name + ": a TurnInPlace pick within 0.5 s of the release",
          firstTip == SIZE_MAX ? "none" : Fmt(double(firstTip - s) * kDt) + " s, " + DbName(rt, tail.out[firstTip].db));
    Check(within50 != SIZE_MAX && within50 - s <= 120, "F-TIP-RELEASE " + name + ": |actor - root| < 50 deg within 2.0 s", within50 == SIZE_MAX ? "never" : Fmt(double(within50 - s) * kDt) + " s");
    // the step of a standing 180 deg turn is set by the authored PSD_Dense_Stand_TurnInPlace clip (10.05 deg per feed): the bound is the step of the same clip
    // in the existing path of the current mode + 0.5 deg; every other case keeps 8 deg per feed
    const float baseStep = MaxVisualStep(base, s, s + 150);
    const bool authoredBound = aLoco == integ::kLocoStand && aOffset >= 179.0f;
    const float bound = authoredBound ? baseStep + 0.5f : 8.0f;
    Check(step <= bound, "F-TIP-RELEASE " + name + (authoredBound ? ": max visual yaw step <= the existing path's step of the same clip + 0.5 deg" : ": max visual yaw step <= 8 deg per feed"),
          Fmt(step) + " deg, bound " + Fmt(bound) + " (current mode " + Fmt(baseStep) + ")");
    Check(sl.Ok(), "F-TIP-RELEASE " + name + ": foot slide in the TIP window lower than the raw motion pose", sl.Text());
    RestoreDefaults();
}
// ------------------------------------------------------------------ recorded feeds: A2 F-OPEN (Shooter) and A6 H2 (Shooter OFF, body turn off)
double PlantedSlideMask(const Run& aTail, const Run& aRun, const std::vector<bool>& aMask, size_t& aN)
{
    double sum = 0;
    aN = 0;
    bool prevPlanted[2] = {false, false};
    float prevToe[2][2] = {{0, 0}, {0, 0}};
    const size_t n = std::min({aMask.size(), aTail.out.size(), aRun.out.size()});
    for (size_t i = 0; i < n; ++i)
        for (int k = 0; k < 2; ++k)
        {
            const bool planted = aTail.out[i].haveContact && aRun.out[i].toeOk[k] && aTail.out[i].tailContact[k] >= 0.5f;
            if (planted && prevPlanted[k] && aMask[i])
            {
                sum += std::hypot(aRun.out[i].toeW[k][0] - prevToe[k][0], aRun.out[i].toeW[k][1] - prevToe[k][1]);
                ++aN;
            }
            prevPlanted[k] = planted;
            prevToe[k][0] = aRun.out[i].toeW[k][0];
            prevToe[k][1] = aRun.out[i].toeW[k][1];
        }
    return aN ? 100.0 * sum / double(aN) : -1.0;
}
float SpeedOf(const Frame& f) { return std::hypot(f.vx, f.vy); }
float StickDegAgainstActor(const Frame& f)
{
    float wx = 0, wy = 0;
    adapter::StickToWorld(f.mvx, f.mvy, f.camDeg, wx, wy);
    if (wx * wx + wy * wy < 0.01f)
        return 999.0f;
    return Wrap(std::atan2(-wx, wy) * kRadToDeg - f.yawDeg);
}

// aShooterMode: A2 (Shooter on, body turn off as the CET contract); else A6 (Shooter OFF, body turn off, H2 guard on)
void OpenFeedCase(Rt& rt, const std::vector<FeedFile>& aFeeds, bool aShooterMode)
{
    const std::string tag = aShooterMode ? "A2 F-OPEN " : "A6 H2 ";
    for (size_t fi = 0; fi < aFeeds.size(); ++fi)
    {
        const FeedFile& f = aFeeds[fi];
        RestoreDefaults();
        if (aShooterMode)
            ShooterModeOn();
        else
            motionhost::SetBodyTurn(false);
        const Run tail = RunOnFeed(rt, f);
        const Run raw = [&] {
            Gen g("raw");
            g.f = f;
            g.act.assign(f.fr.size(), 0);
            return RunSynth(rt, g, Opts{true, 0, false});
        }();
        size_t stepped = 0, strafeFrames = 0;
        for (size_t i = 0; i < tail.out.size(); ++i)
        {
            stepped += tail.out[i].stepped;
            strafeFrames += tail.out[i].stepped && tail.probe[i].rotationMode == 1;
        }
        const auto spins = Picks(rt, tail, 0, f.fr.size(), IsSpinOrReface);
        const std::vector<Episode> eps = ReversalEpisodes(f);
        Check(stepped == f.fr.size(), tag + f.name + ": motion stepped on every frame", std::to_string(stepped) + "/" + std::to_string(f.fr.size()));
        Check(spins.empty(), tag + f.name + ": no SpinTransition / Reface pick on the whole feed", std::to_string(spins.size()) + " picks");
        if (aShooterMode)
            Check(strafeFrames == stepped, tag + f.name + ": rot = Strafe on 100 % of the stepped frames", std::to_string(strafeFrames) + "/" + std::to_string(stepped));
        else
            std::printf("INFO %s%s: rot = Strafe on %zu/%zu stepped frames (the latch only: the other frames keep Orient, as today)\n", tag.c_str(), f.name.c_str(),
                        strafeFrames, stepped);
        if (!aShooterMode)
        {
            size_t h2 = 0;
            for (size_t i = 0; i < tail.out.size(); ++i)
                h2 += tail.probe[i].h2Active;
            Check(h2 > 0, tag + f.name + ": the safeguard latched during the reversals", std::to_string(h2) + " frames");
        }
        // cruise back: >= 0.3 s into a reversal episode at >= 1.5 m/s
        std::vector<bool> back(f.fr.size(), false), fwd(f.fr.size(), false);
        size_t backN = 0, backLoopB = 0;
        for (const Episode& e : eps)
            for (size_t i = e.first; i <= e.last; ++i)
                if (f.fr[i].t - f.fr[e.first].t >= 0.3f && SpeedOf(f.fr[i]) >= kReversalMinSpeed && tail.out[i].stepped)
                {
                    back[i] = true;
                    ++backN;
                    backLoopB += Has(ClipName(rt, tail.out[i].clip), "Loop_B");
                }
        for (size_t i = 0; i < f.fr.size(); ++i)
            fwd[i] = SpeedOf(f.fr[i]) >= 3.5f && std::fabs(Wrap(std::atan2(-f.fr[i].vx, f.fr[i].vy) * kRadToDeg - f.fr[i].yawDeg)) < 30.0f;
        if (f.name.find("flick") != std::string::npos)
        {
            Check(backN > 0 && backLoopB * 10 >= backN * 9, tag + f.name + ": cruise back plays a Loop_B / BL / BR clip on >= 90 % of the frames",
                     std::to_string(backLoopB) + "/" + std::to_string(backN));
            size_t nB = 0, nF = 0;
            const double mB = PlantedSlideMask(tail, tail, back, nB), mF = PlantedSlideMask(tail, tail, fwd, nF);
            Check(nB > 5 && nF > 5 && mB <= 1.5 * mF, tag + f.name + ": planted slide in the cruise-back window <= 1.5x the forward cruise window",
                     "back " + Fmt(mB, 3) + " cm (n " + std::to_string(nB) + ") forward " + Fmt(mF, 3) + " cm (n " + std::to_string(nF) + ")");
        }
        // root follows the actor: |actor - visual root| <= 30 deg on >= 95 % of the stepped frames at >= 0.5 m/s, 0.3 s after the speed came up
        size_t rootN = 0, rootOk = 0;
        float lastSlow = -1.0f;
        for (size_t i = 0; i < f.fr.size(); ++i)
        {
            if (SpeedOf(f.fr[i]) < 0.5f)
                lastSlow = f.fr[i].t;
            else if (tail.out[i].stepped && tail.out[i].haveVisual && f.fr[i].t - lastSlow >= 0.3f && (aShooterMode || tail.probe[i].h2Active || f.name.find("arc") == std::string::npos))
            {
                ++rootN;
                rootOk += RootErr(tail, i) <= 30.0f;
            }
        }
        Check(rootN > 0 && rootOk * 100 >= rootN * 95,
              tag + f.name + (aShooterMode || f.name.find("arc") == std::string::npos ? ": |actor - root| <= 30 deg on >= 95 % of the moving frames"
                                                                                       : ": |actor - root| <= 30 deg on >= 95 % of the frames of the H2 sector (latch active; side / diagonal frames stay Orient by decision)"),
              std::to_string(rootOk) + "/" + std::to_string(rootN));
        if (!aShooterMode)
        {
            // no moonwalk in the H2 sector: the planted slide of the latched frames is lower than the raw v2 pose
            std::vector<bool> latched(f.fr.size(), false);
            for (size_t i = 0; i < f.fr.size(); ++i)
                latched[i] = tail.probe[i].h2Active;
            size_t nL = 0, nLr = 0;
            const double mL = PlantedSlideMask(tail, tail, latched, nL), mLr = PlantedSlideMask(tail, raw, latched, nLr);
            Check(nL > 20 && nLr > 20 && mL < mLr, tag + f.name + ": planted foot slide on the latched frames tail < raw (no moonwalk)",
                  "tail " + Fmt(mL, 3) + " cm (n " + std::to_string(nL) + ") raw " + Fmt(mLr, 3) + " cm (n " + std::to_string(nLr) + ")");
        }
        size_t nT = 0, nR = 0;
        const std::vector<bool> all(f.fr.size(), true);
        const double mT = PlantedSlideMask(tail, tail, all, nT), mR = PlantedSlideMask(tail, raw, all, nR);
        Check(nT > 20 && nR > 20 && mT < mR, tag + f.name + ": planted foot slide tail < raw", "tail " + Fmt(mT, 3) + " cm (n " + std::to_string(nT) + ") raw " + Fmt(mR, 3) + " cm (n " + std::to_string(nR) + ")");
        if (f.name.find("arc") != std::string::npos)
        {
            size_t sideN = 0, sideOk = 0;
            for (size_t i = 0; i < f.fr.size(); ++i)
            {
                const float a = std::fabs(StickDegAgainstActor(f.fr[i]));
                if (a >= 70.0f && a <= 110.0f && SpeedOf(f.fr[i]) >= 1.0f && tail.out[i].stepped)
                {
                    ++sideN;
                    const std::string c = ClipName(rt, tail.out[i].clip);
                    sideOk += Has(c, "LL") || Has(c, "LR") || Has(c, "RL") || Has(c, "RR");
                }
            }
            Check(sideN > 0 && sideOk * 100 >= sideN * 60, tag + f.name + ": side sector 70-110 deg plays an LL / LR / RL / RR clip on >= 60 % of the frames",
                     std::to_string(sideOk) + "/" + std::to_string(sideN));
        }
        if (!aShooterMode)
        {
            // the same feed with the safeguard off still repeats the spin: the case discriminates
            motiondriver::SetH2Guard(false);
            const Run off = RunOnFeed(rt, f);
            const auto offSpins = Picks(rt, off, 0, f.fr.size(), IsSpin);
            const size_t episodes = eps.size();
            Check(!eps.empty() && offSpins.size() >= 3 * episodes, tag + f.name + ": without the safeguard the spin is picked again and again (>= 3 per reversal episode)",
                  std::to_string(offSpins.size()) + " picks in " + std::to_string(episodes) + " episodes");
        }
        RestoreDefaults();
    }
}

// A3 F-FLICK: running forward, the camera (= capsule) turns aDeg with the stick held - in one feed (a capsule jump) or 180 deg in 0.25 s. The visual root
// is rate bounded (450 deg/s, 7.5 deg per 60 Hz feed) from the first feed on: it neither snaps nor spins repeatedly.
void FlickCase(Rt& rt, bool aJump, float aDeg)
{
    RestoreDefaults();
    const std::string name = aJump ? "one-feed jump " + std::to_string(static_cast<int>(aDeg)) + " deg" : "180 deg in 0.25 s";
    Gen g("flick " + name);
    g.Idle(1.0f);
    g.Seg(2.0f, 0, 1.0f, 0);
    const size_t j = g.Size();
    g.accel = 25.0f;
    if (aJump)
        g.Seg(0.3f, 0, 1.0f, aDeg, true);
    else
        g.Seg(0.25f, 0, 1.0f, aDeg);
    g.Seg(2.0f, 0, 1.0f, aDeg);
    ShooterModeOn();
    const Run tail = RunSynth(rt, g);
    const Run raw = RunSynth(rt, g, Opts{true, 0, false});
    const auto picks = Picks(rt, tail, j - 3, j + 150, IsSpinOrReface);
    const size_t within = FirstWithin(tail, j, j + 150, 30.0f);
    const float step = MaxVisualStep(tail, j, j + 150);
    const SlideCmp sl = CompareSlide(tail, raw, j, j + 60);
    Check(picks.size() <= 1, "A3 F-FLICK " + name + ": at most one spin / reface pick (never repeated)", std::to_string(picks.size()));
    Check(within != SIZE_MAX && within - j <= 36, "A3 F-FLICK " + name + ": |actor - root| <= 30 deg within 0.6 s of the flick", within == SIZE_MAX ? "never" : Fmt(double(within - j) * kDt) + " s");
    Check(step <= 10.0f, "A3 F-FLICK " + name + ": max visual yaw step <= 10 deg per feed from the first feed (no snap)", Fmt(step) + " deg");
    std::printf("INFO A3 F-FLICK %s: planted slide in the first 1.0 s %s\n", name.c_str(), sl.Text().c_str());
    RestoreDefaults();
}

// A5 F-SPRINT: PSM Sprint, 8.2 m/s along the stick; the CanSprint cone (50 deg) decides the gait and the Sprint databases
void SprintCase(Rt& rt)
{
    for (const float deg : {0.0f, 40.0f, 60.0f, 90.0f, 180.0f})
    {
        RestoreDefaults();
        Gen g("sprint " + std::to_string(static_cast<int>(deg)));
        g.loco = integ::kLocoSprint;
        g.maxSpeed = 8.2f;
        g.Idle(0.5f);
        const size_t s = g.Size();
        g.Seg(2.5f, deg, 1.0f, 0.0f);
        ShooterModeOn();
        const Run r = RunSynth(rt, g);
        size_t n = 0, gaitOk = 0, sprintDb = 0;
        for (size_t i = s + 45; i < g.Size(); ++i)
        {
            ++n;
            const bool cone = deg < 50.0f;
            gaitOk += r.probe[i].gait == (cone ? 2 : 1);
            sprintDb += Has(DbName(rt, r.out[i].db), "Sprint");
        }
        const bool cone = deg < 50.0f;
        const std::string name = "A5 F-SPRINT stick " + std::to_string(static_cast<int>(deg)) + " deg";
        Check(gaitOk * 100 >= n * 95, name + ": gait " + (cone ? "2 (sprint)" : "1 (run)") + " after 0.75 s", std::to_string(gaitOk) + "/" + std::to_string(n));
        if (cone)
            Check(sprintDb * 10 >= n * 9, name + ": Sprint databases on >= 90 % of the frames", std::to_string(sprintDb) + "/" + std::to_string(n));
        else
            Check(sprintDb == 0, name + ": no Sprint database pick", std::to_string(sprintDb) + "/" + std::to_string(n));
        RestoreDefaults();
    }
}

// A7 OFF-ID: Shooter OFF, body turn on: the H2 guard on vs off must give identical output on every frame (the safeguard never acts in the current mode)
bool SameFrame(const FrameOut& a, const FrameOut& b)
{
    return a.stepped == b.stepped && a.clip == b.clip && a.db == b.db && std::memcmp(&a.clipTime, &b.clipTime, sizeof(float)) == 0 &&
           std::memcmp(&a.visualYawDeg, &b.visualYawDeg, sizeof(float)) == 0 && a.hash == b.hash && a.haveVisual == b.haveVisual;
}
void OffIdentityCase(Rt& rt, const FeedFile& aF)
{
    RestoreDefaults();
    std::string why;
    Check(DefaultsInStatus(why), "A7 OFF-ID " + aF.name + ": defaults asserted from Status() before the case", why);
    const Run on = RunOnFeed(rt, aF);
    const std::string btAfter = StatusField(motiondriver::Status(), "btSwitch=");
    Check(btAfter == "1", "A7 OFF-ID " + aF.name + ": the body turn drove during the run (btSwitch=1 on its last Step)", "btSwitch=" + btAfter);
    motiondriver::SetH2Guard(false);
    const Run off = RunOnFeed(rt, aF);
    RestoreDefaults();
    size_t diff = 0, first = SIZE_MAX, stepped = 0;
    for (size_t i = 0; i < on.out.size() && i < off.out.size(); ++i)
    {
        stepped += on.out[i].stepped;
        if (!SameFrame(on.out[i], off.out[i]))
        {
            ++diff;
            if (first == SIZE_MAX)
                first = i;
        }
    }
    Check(stepped > 0 && diff == 0, "A7 OFF-ID " + aF.name + ": guard on == guard off, frame by frame (clip, db, clip time bits, visual yaw bits, pose hash)",
          std::to_string(stepped) + " stepped frames, " + std::to_string(diff) + " differ" + (first == SIZE_MAX ? "" : ", first " + std::to_string(first)));
}

// A11 TRIM (2026-10-30, mm.pelvisYawTrim): a constant pelvis trim edits only the leg-tail frame copy and the root yaw handed to
// the host; it must never feed back into the matcher (same clip, database and clip time bits on every frame), the committed visual
// root differs by at most the trim, and the trim is really applied somewhere (pose hashes differ on forward locomotion frames).
size_t g_trimPoseDiff = 0;
void TrimNoFeedbackCase(Rt& rt, const FeedFile& aF)
{
    RestoreDefaults();
    const Run base = RunOnFeed(rt, aF);
    Check(motiondriver::SetPelvisYawTrim(5.0f) && motiondriver::PelvisYawTrim() == 5.0f, "A11 TRIM " + aF.name + ": SetPelvisYawTrim(5) accepted", "");
    const Run trim = RunOnFeed(rt, aF);
    RestoreDefaults();
    size_t stepped = 0, diff = 0, first = SIZE_MAX, poseDiff = 0;
    float maxVis = 0.0f;
    for (size_t i = 0; i < base.out.size() && i < trim.out.size(); ++i)
    {
        const FrameOut& a = base.out[i];
        const FrameOut& b = trim.out[i];
        stepped += a.stepped;
        const bool same = a.stepped == b.stepped && a.clip == b.clip && a.db == b.db && std::memcmp(&a.clipTime, &b.clipTime, sizeof(float)) == 0;
        if (!same)
        {
            ++diff;
            if (first == SIZE_MAX)
                first = i;
        }
        poseDiff += a.hash != b.hash;
        if (a.haveVisual && b.haveVisual)
            maxVis = std::fmax(maxVis, std::fabs(Wrap(b.visualYawDeg - a.visualYawDeg)));
    }
    g_trimPoseDiff += poseDiff;
    Check(stepped > 0 && diff == 0, "A11 TRIM " + aF.name + ": trim +5 never changes a matcher decision (clip, db, clip time bits)",
          std::to_string(stepped) + " stepped, " + std::to_string(diff) + " differ" + (first == SIZE_MAX ? "" : ", first " + std::to_string(first)) +
              ", pose hash differs on " + std::to_string(poseDiff));
    Check(maxVis <= 5.0f + 1e-3f, "A11 TRIM " + aF.name + ": committed visual root differs by at most the trim", Fmt(maxVis, 3));
}

// A12 TRIM-RUN (2026-10-30 in-game report: pt=5.0/0.00 on a straight keyboard run): W held 6 s from standing, Shooter mode off.
// Empty hands (PSM weapon known, nothing drawn): the trim weight reaches 1 on the run and the reason is "ok". The same run with a weapon
// drawn (the in-game case: a rifle was equipped at 07:50:12, MMWEAPON held=1, feed PSM drawn=1): weight stays 0, reason "drawn".
void TrimRunCase(Rt& rt, bool aDrawn)
{
    RestoreDefaults();
    Gen g(aDrawn ? "trim run, weapon drawn" : "trim run, empty hands");
    g.psm = aDrawn ? kWeaponDrawn : kWeaponKnown;
    g.Idle(1.0f);
    g.Seg(6.0f, 0.0f, 1.0f, 0.0f);
    g.Idle(1.0f);
    motiondriver::SetPelvisYawTrim(5.0f);
    float maxW = 0.0f, runW = 0.0f;
    size_t runFrames = 0;
    std::string why, runWhy;
    const size_t runFrom = static_cast<size_t>(3.0f / kDt), runTo = static_cast<size_t>(7.0f / kDt); // steady part of the run
    Opts o;
    o.onFrame = [&](size_t i) {
        const float w = motiondriver::PelvisYawTrimWeight();
        maxW = std::fmax(maxW, w);
        if (i >= runFrom && i < runTo)
        {
            runW += w;
            ++runFrames;
            runWhy = motiondriver::PelvisYawTrimReason();
        }
    };
    (void)RunSynth(rt, g, o);
    RestoreDefaults();
    const float meanRun = runFrames ? runW / float(runFrames) : 0.0f;
    if (!aDrawn)
        Check(maxW == 1.0f && meanRun > 0.99f && runWhy == "ok", "A12 TRIM-RUN empty hands: straight keyboard run -> trim weight 1, reason ok",
              "max " + Fmt(maxW, 3) + " mean on the run " + Fmt(meanRun, 3) + " reason " + runWhy);
    else
        Check(maxW == 0.0f && runWhy == "drawn", "A12 TRIM-RUN weapon drawn: weight 0, reason drawn (valid gate)",
              "max " + Fmt(maxW, 3) + " reason " + runWhy);
}

// ------------------------------------------------------------------ body yaw offset (2026-10-30, mm.bodyYawOffset)
// Frame hook observer: the real frame hook (WFrame) plus, per feed, the frame handed to the leg tail: the hash of its pose buffer
// (the bone locals of the frame copy), the provided root yaw and the body offset weight / reason of the Step it belongs to.
struct BoRec
{
    bool have = false, provided = false;
    float rootYaw = 0.0f, w = 0.0f; // rad, weight
    uint64_t poseHash = 0;
    std::string why;
};
std::vector<BoRec> g_boRecs;
size_t g_boFeed = SIZE_MAX;
uint64_t HashBytes(const void* aP, size_t aN)
{
    uint64_t h = 1469598103934665603ull; // FNV-1a
    const auto* p = static_cast<const unsigned char*>(aP);
    for (size_t i = 0; i < aN; ++i)
        h = (h ^ p[i]) * 1099511628211ull;
    return h;
}
bool WFrameBo(const int32_t* aBaseGameIndex, uint32_t aBaseBones, motionhost::MotionSetFrame& aOut)
{
    const bool ok = WFrame(aBaseGameIndex, aBaseBones, aOut);
    if (ok && g_boFeed < g_boRecs.size() && aOut.pose != nullptr)
    {
        BoRec& r = g_boRecs[g_boFeed];
        r.have = true;
        r.provided = aOut.providedRoot;
        r.rootYaw = aOut.rootOffsetYaw;
        r.poseHash = HashBytes(aOut.pose, size_t(aOut.numBones) * sizeof(*aOut.pose));
        r.w = motiondriver::BodyYawOffsetWeight();
        r.why = motiondriver::BodyYawOffsetReason();
    }
    return ok;
}
const motionhost::MotionHooks kHooksBo{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &WFrameBo, &motiondriver::OwnsTraversal, nullptr,
                              &motiondriver::TravActive};
struct BoRun
{
    Run run;
    std::vector<BoRec> rec;
};
BoRun RunBo(Rt& rt, const Gen& aG, Opts aO = {})
{
    g_boRecs.assign(aG.Size(), BoRec{});
    g_boFeed = SIZE_MAX;
    aO.hooks = &kHooksBo;
    auto user = aO.onFrame;
    aO.onFrame = [&](size_t i) {
        g_boFeed = i;
        if (user)
            user(i);
    };
    BoRun r;
    r.run = RunSynth(rt, aG, aO);
    g_boFeed = SIZE_MAX;
    r.rec = std::move(g_boRecs);
    g_boRecs.clear();
    return r;
}
BoRun RunBoOnFeed(Rt& rt, const FeedFile& aF)
{
    Gen g("recorded");
    g.f = aF;
    g.act.assign(aF.fr.size(), 0);
    return RunBo(rt, g);
}

// A13 BODY-OFFSET: a constant body yaw offset (8 deg) on the recorded feeds only adds 8 deg x weight to the root yaw handed to the
// leg tail: the frame pose buffer (bone locals) is bit-identical to offset 0, no matcher decision changes, the root is provided on
// the same frames. Offset 0: the body offset code never runs (weight 0, reason off on every frame). With the pelvis trim (+5) on as
// well (aTrim), the trim's pose edit is untouched by the offset (pose bytes identical), the root yaws differ by 8 x weight.
size_t g_boFullFrames = 0;
void BodyOffsetCase(Rt& rt, const FeedFile& aF, bool aTrim)
{
    const std::string tag = "A13 BODY-OFFSET " + aF.name + (aTrim ? " (pelvis trim +5)" : "");
    RestoreDefaults();
    if (aTrim)
        motiondriver::SetPelvisYawTrim(5.0f);
    const BoRun base = RunBoOnFeed(rt, aF);
    Check(motiondriver::SetBodyYawOffset(8.0f) && motiondriver::BodyYawOffset() == 8.0f, tag + ": SetBodyYawOffset(8) accepted", "");
    const BoRun bo = RunBoOnFeed(rt, aF);
    RestoreDefaults();
    size_t stepped = 0, pickDiff = 0, poseDiff = 0, rootDiff = 0, yawBad = 0, framed = 0, full = 0, offRan = 0;
    float worst = 0.0f;
    for (size_t i = 0; i < base.run.out.size() && i < bo.run.out.size(); ++i)
    {
        const FrameOut& a = base.run.out[i];
        const FrameOut& b = bo.run.out[i];
        stepped += a.stepped;
        pickDiff += !(a.stepped == b.stepped && a.clip == b.clip && a.db == b.db && std::memcmp(&a.clipTime, &b.clipTime, sizeof(float)) == 0);
        const BoRec& ra = base.rec[i];
        const BoRec& rb = bo.rec[i];
        offRan += ra.have && (ra.w != 0.0f || ra.why != "off");
        if (ra.have != rb.have)
        {
            ++poseDiff;
            continue;
        }
        if (!ra.have)
            continue;
        ++framed;
        poseDiff += ra.poseHash != rb.poseHash;
        rootDiff += ra.provided != rb.provided;
        if (ra.provided && rb.provided)
        {
            const float d = Wrap((rb.rootYaw - ra.rootYaw) / integ::kDegToRad);
            const float err = std::fabs(d - 8.0f * rb.w);
            worst = std::fmax(worst, err);
            yawBad += err > 1e-3f;
            full += rb.w == 1.0f && std::fabs(d - 8.0f) <= 1e-3f;
        }
        else if (!ra.provided && !rb.provided)
            yawBad += std::memcmp(&ra.rootYaw, &rb.rootYaw, sizeof(float)) != 0;
    }
    g_boFullFrames += full;
    Check(stepped > 0 && pickDiff == 0, tag + ": offset 8 never changes a matcher decision (clip, db, clip time bits)",
          std::to_string(stepped) + " stepped, " + std::to_string(pickDiff) + " differ");
    Check(framed > 0 && poseDiff == 0 && rootDiff == 0, tag + ": leg-tail frame pose bytes bit-identical to offset 0, root provided on the same frames",
          std::to_string(framed) + " frames, pose differs on " + std::to_string(poseDiff) + ", provided differs on " + std::to_string(rootDiff));
    Check(yawBad == 0, tag + ": root yaw differs by exactly 8 deg x weight on every frame (1e-3 deg)", "worst " + Fmt(worst, 5) + " deg, " +
              std::to_string(yawBad) + " bad, " + std::to_string(full) + " frames at weight 1 = +8.000");
    if (!aTrim)
        Check(offRan == 0, tag + ": offset 0 -> the body offset never runs (weight 0, reason off on every frame)", std::to_string(offRan) + " frames ran");
}

// A14 BODY-STATES: where the body offset is active (weight 1, reason ok) and where it fades out. Empty hands: idle, walk, run, crouch
// walk, jump (air), landing run. Weapon drawn (Pistol2H overlay): idle and run with the weapon drawn are active; aiming, a shot burst
// (+ its pose hold) and a reload hold the weight at 0 (reasons aim / wpnact); back to 1 after each.
void BodyStatesCase(Rt& rt)
{
    struct Seg { std::string what; size_t from, to; bool on; const char* why; };
    auto check = [&](const BoRun& r, const std::vector<Seg>& segs) {
        for (const Seg& s : segs)
        {
            // steady part of the segment: from 0.4 s after its start (fades 0.25 s) to its end
            size_t n = 0, have = 0, okN = 0;
            float wMin = 2.0f, wMax = -1.0f;
            std::string why;
            for (size_t i = s.from + static_cast<size_t>(0.4f / kDt); i < s.to && i < r.rec.size(); ++i)
            {
                ++n;
                if (!r.rec[i].have)
                    continue;
                ++have;
                wMin = std::fmin(wMin, r.rec[i].w);
                wMax = std::fmax(wMax, r.rec[i].w);
                why = r.rec[i].why;
                okN += s.on ? (r.rec[i].w == 1.0f && r.rec[i].why == "ok" && r.rec[i].provided)
                            : (r.rec[i].w == 0.0f && r.rec[i].why == std::string(s.why));
            }
            Check(n > 0 && have == n && okN == n,
                  "A14 BODY-STATES " + s.what + (s.on ? ": active (weight 1, reason ok, root provided)" : std::string(": weight 0, reason ") + s.why),
                  std::to_string(okN) + "/" + std::to_string(n) + " frames (" + std::to_string(have) + " with a frame), weight " + Fmt(wMin, 3) +
                      ".." + Fmt(wMax, 3) + ", last reason " + why);
        }
    };
    RestoreDefaults();
    motiondriver::SetBodyYawOffset(8.0f);
    {
        Gen g("body states, empty hands");
        std::vector<Seg> segs;
        auto seg = [&](const char* aWhat, std::function<void()> aBuild, bool aOn, const char* aWhy = "ok") {
            const size_t from = g.Size();
            aBuild();
            segs.push_back({aWhat, from, g.Size(), aOn, aWhy});
        };
        seg("idle", [&] { g.Idle(1.5f); }, true);
        seg("walk", [&] { g.Seg(2.0f, 0.0f, 0.4f, 0.0f); }, true);
        seg("run", [&] { g.Seg(2.0f, 0.0f, 1.0f, 0.0f); }, true);
        seg("run turning left", [&] { g.Seg(2.0f, 0.0f, 1.0f, 120.0f); }, true);
        g.loco = integ::kLocoCrouch;
        seg("crouch walk", [&] { g.Seg(2.0f, 0.0f, 1.0f, 120.0f); }, true);
        seg("crouch idle", [&] { g.Idle(1.5f); }, true);
        g.loco = integ::kLocoStand;
        g.Seg(1.0f, 0.0f, 1.0f, 120.0f);
        g.psm = WithDetail(kWeaponKnown, adapter::kPsmJump);
        seg("jump (air)", [&] { g.Seg(0.9f, 0.0f, 1.0f, 120.0f); }, true);
        g.psm = WithDetail(kWeaponKnown, adapter::kPsmRegularLand);
        g.Seg(0.1f, 0.0f, 1.0f, 120.0f);
        g.psm = kWeaponKnown;
        seg("run after landing", [&] { g.Seg(1.5f, 0.0f, 1.0f, 120.0f); }, true);
        seg("stop, idle", [&] { g.Idle(1.5f); }, true);
        // melee (PSM melee attack bits, the H2 melee definition): reason melee on every v2 frame of the span, the weight never rises and
        // is 0 from 0.4 s into the span (fade 0.25 s); where the v1 melee gate closes v2 there are no frames. Afterwards it fades in again.
        const size_t meleeFrom = g.Size();
        g.psm = kWeaponKnown | integ::kPsmMeleeAttack;
        g.Idle(1.0f);
        g.psm = kWeaponKnown;
        const size_t meleeTo = g.Size();
        seg("idle after melee", [&] { g.Idle(2.0f); }, true);
        // overlay release (v2 skips these frames): the re-entry starts at weight 0 and fades in (<= dt / 0.25 s per frame)
        const size_t relFrom = g.Size();
        g.Seg(1.0f, 0.0f, 1.0f, 120.0f);
        const size_t relTo = g.Size();
        seg("run after a release", [&] { g.Seg(2.0f, 0.0f, 1.0f, 120.0f); }, true);
        Opts o;
        o.releaseFrom = relFrom;
        o.releaseTo = relTo;
        const BoRun r = RunBo(rt, g, o);
        check(r, segs);
        size_t mFrames = 0, mBad = 0, mRise = 0, mLate = 0, mLateNonZero = 0;
        float prevW = -1.0f;
        for (size_t i = meleeFrom; i < meleeTo && i < r.rec.size(); ++i)
        {
            if (!r.rec[i].have)
                continue;
            ++mFrames;
            mBad += r.rec[i].why != "melee";
            mRise += prevW >= 0.0f && r.rec[i].w > prevW;
            prevW = r.rec[i].w;
            if (i >= meleeFrom + static_cast<size_t>(0.4f / kDt))
            {
                ++mLate;
                mLateNonZero += r.rec[i].w != 0.0f;
            }
        }
        Check(mFrames > 0 && mBad == 0 && mRise == 0 && mLateNonZero == 0,
              "A14 BODY-STATES melee: reason melee on every motion frame of the span, the weight never rises, 0 from 0.4 s into the span",
              std::to_string(mFrames) + "/" + std::to_string(meleeTo - meleeFrom) + " frames with a motion frame, " + std::to_string(mBad) +
                  " other reason, " + std::to_string(mRise) + " rises, " + std::to_string(mLateNonZero) + "/" + std::to_string(mLate) +
                  " late frames not 0");
        size_t relFrames = 0, firstAfter = SIZE_MAX, after = 0, afterBad = 0;
        for (size_t i = relFrom; i < relTo && i < r.rec.size(); ++i)
            relFrames += r.rec[i].have;
        float prev = 0.0f, maxRise = 0.0f;
        for (size_t i = relTo; i < r.rec.size() && i < relTo + static_cast<size_t>(0.5f / kDt); ++i)
        {
            if (!r.rec[i].have)
                continue;
            if (firstAfter == SIZE_MAX)
            {
                firstAfter = i;
                prev = 0.0f;
            }
            ++after;
            maxRise = std::fmax(maxRise, r.rec[i].w - prev);
            afterBad += r.rec[i].w < prev;
            prev = r.rec[i].w;
        }
        Check(relFrames == 0 && firstAfter != SIZE_MAX && maxRise <= kDt / 0.25f + 1e-4f && afterBad == 0,
              "A14 BODY-STATES release: no motion frame during the release, the re-entry fades in from 0 (<= dt / 0.25 s per frame)",
              std::to_string(relFrames) + " frames in the release, first after +" +
                  (firstAfter == SIZE_MAX ? std::string("none") : std::to_string(firstAfter - relTo)) + ", max rise " + Fmt(maxRise, 4) +
                  " (bound " + Fmt(kDt / 0.25f + 1e-4f, 4) + "), " + std::to_string(afterBad) + " decreases");
    }
    {
        // Shooter mode (bodyturn.shooterMode: the root held on the actor, strafe locomotion), not aiming: the offset stays active
        Gen g("body states, Shooter mode");
        std::vector<Seg> segs;
        auto seg = [&](const char* aWhat, std::function<void()> aBuild, bool aOn, const char* aWhy = "ok") {
            const size_t from = g.Size();
            aBuild();
            segs.push_back({aWhat, from, g.Size(), aOn, aWhy});
        };
        ShooterModeOn();
        seg("Shooter mode idle", [&] { g.Idle(1.5f); }, true);
        seg("Shooter mode run", [&] { g.Seg(2.0f, 0.0f, 1.0f, 0.0f); }, true);
        seg("Shooter mode strafe left", [&] { g.Seg(2.0f, 90.0f, 1.0f, 0.0f); }, true);
        seg("Shooter mode idle again", [&] { g.Idle(1.5f); }, true);
        check(RunBo(rt, g), segs);
        RestoreDefaults();
        motiondriver::SetBodyYawOffset(8.0f);
    }
    {
        Gen g("body states, weapon drawn");
        g.psm = kWeaponDrawn;
        std::vector<Seg> segs;
        auto seg = [&](const char* aWhat, std::function<void()> aBuild, bool aOn, const char* aWhy = "ok") {
            const size_t from = g.Size();
            aBuild();
            segs.push_back({aWhat, from, g.Size(), aOn, aWhy});
        };
        seg("weapon drawn idle", [&] { g.Idle(1.5f); }, true);
        seg("weapon drawn run", [&] { g.Seg(2.0f, 0.0f, 1.0f, 0.0f); }, true);
        g.Idle(0.5f);
        g.psm = kWeaponDrawn | kUpperAim;
        seg("aiming (ADS)", [&] { g.Idle(1.5f); }, false, "aim");
        g.psm = kWeaponDrawn;
        seg("weapon drawn after aiming", [&] { g.Idle(1.5f); }, true);
        {
            const size_t from = g.Size();
            for (int k = 0; k < 10; ++k)
            {
                g.action = 8;
                g.Idle(0.05f);
                g.action = 0;
                g.Idle(0.05f);
            }
            segs.push_back({"hip-fire burst", from, g.Size(), false, "wpnact"});
        }
        g.Idle(1.5f);
        g.action = 2;
        seg("reload", [&] { g.Idle(1.5f); }, false, "wpnact");
        g.action = 0;
        g.Idle(1.0f);
        seg("weapon drawn after the reload", [&] { g.Idle(1.0f); }, true);
        Opts o;
        o.overlay = 3;
        check(RunBo(rt, g, o), segs);
    }
    RestoreDefaults();
}

// A15 BODY-PUBLISHED: the offset on the PUBLISHED pose (after the v1 leg tail: offset root, FootLocker / stance guard, the v2 apply),
// weight 1: the model rotation of Hips and Head at offset 8 = the one at offset 0 turned about the up axis by +8 deg (twist 8 +- 0.1
// deg, tilt <= 0.1 deg) in steady idle and in a steady straight run; standing, LeftFoot / RightFoot and the foot-to-foot vector too.
Xf ModelXf(const mm::integ::PoseSnapshot& s, const Rig& aR, int32_t aBone, bool& aOk)
{
    int32_t path[64];
    int n = 0;
    aOk = true;
    for (int32_t b = aBone; b >= 0 && n < 64; b = aR.parents[size_t(b)])
    {
        if (static_cast<uint32_t>(b) >= s.numBones)
            aOk = false;
        path[n++] = b;
    }
    Xf g;
    if (aOk)
        for (int i = n - 1; i >= 0; --i)
            g = Mul(g, &s.pose[size_t(path[i]) * 7]);
    return g;
}
void BodyPublishedCase(Rt& rt)
{
    const char* names[4] = {"Hips", "Head", "LeftFoot", "RightFoot"};
    int32_t bone[4] = {-1, -1, -1, -1};
    for (uint32_t b = 0; b < rt.db->NumBones(); ++b)
        for (int k = 0; k < 4; ++k)
            if (rt.db->BoneName(b) == names[k])
                bone[k] = static_cast<int32_t>(b);
    Setup(bone[0] >= 0 && bone[1] >= 0 && bone[2] >= 0 && bone[3] >= 0, "A15 bones Hips / Head / LeftFoot / RightFoot", "");
    Gen g("body published pose");
    g.Idle(2.5f);
    const size_t idleTo = g.Size();
    g.Seg(3.0f, 0.0f, 1.0f, 0.0f);
    const size_t runTo = g.Size();
    std::vector<mm::integ::PoseSnapshot> cap0, cap8;
    RestoreDefaults();
    g_capture = &cap0;
    const BoRun base = RunBo(rt, g);
    motiondriver::SetBodyYawOffset(8.0f);
    g_capture = &cap8;
    const BoRun bo = RunBo(rt, g);
    g_capture = nullptr;
    RestoreDefaults();
    // aFeet: the feet are asserted too (standing: FootLocker holds them where the turned pose put them). In a run a planted foot stays
    // where it was planted while the root moves on: the turned stance foot cannot follow a rotation about the moving root, so the feet
    // are only measured there (twist error, toe slide while in contact vs offset 0) and Hips / Head are asserted.
    auto span = [&](const char* aWhat, size_t aFrom, size_t aTo, bool aFeet) {
        size_t n = 0, slideN[2] = {0, 0};
        float twistErr[4] = {0, 0, 0, 0}, tilt[4] = {0, 0, 0, 0}, worstFeet = 0.0f, slideSum[2] = {0, 0}, slideMax[2] = {0, 0};
        bool fkOk = true;
        for (size_t i = aFrom; i < aTo && i < cap0.size() && i < cap8.size(); ++i)
        {
            if (!(base.run.out[i].published && bo.run.out[i].published && base.run.out[i].stepped && bo.run.out[i].stepped &&
                  bo.rec[i].have && bo.rec[i].w == 1.0f))
                continue;
            ++n;
            float fp[2][2][3] = {};
            for (int k = 0; k < 4; ++k)
            {
                bool okA = false, okB = false;
                const Xf a = ModelXf(cap0[i], *rt.rig, bone[k], okA), b = ModelXf(cap8[i], *rt.rig, bone[k], okB);
                fkOk = fkOk && okA && okB;
                // d = b * conj(a): the model rotation turned from offset 0 to offset 8
                const float* p = b.q;
                const float q[4] = {-a.q[0], -a.q[1], -a.q[2], a.q[3]};
                float d[4] = {p[3] * q[0] + p[0] * q[3] + p[1] * q[2] - p[2] * q[1], p[3] * q[1] - p[0] * q[2] + p[1] * q[3] + p[2] * q[0],
                              p[3] * q[2] + p[0] * q[1] - p[1] * q[0] + p[2] * q[3], p[3] * q[3] - p[0] * q[0] - p[1] * q[1] - p[2] * q[2]};
                if (d[3] < 0.0f)
                    for (float& c : d)
                        c = -c;
                const float tw = 2.0f * std::atan2(d[2], d[3]) / integ::kDegToRad;
                twistErr[k] = std::fmax(twistErr[k], std::fabs(tw - 8.0f));
                tilt[k] = std::fmax(tilt[k], 2.0f * std::asin(std::fmin(1.0f, std::hypot(d[0], d[1]))) / integ::kDegToRad);
                if (k >= 2)
                {
                    std::memcpy(fp[0][k - 2], a.p, sizeof(a.p));
                    std::memcpy(fp[1][k - 2], b.p, sizeof(b.p));
                }
            }
            const float h0 = std::atan2(fp[0][1][1] - fp[0][0][1], fp[0][1][0] - fp[0][0][0]);
            const float h8 = std::atan2(fp[1][1][1] - fp[1][0][1], fp[1][1][0] - fp[1][0][0]);
            worstFeet = std::fmax(worstFeet, std::fabs(Wrap((h8 - h0) / integ::kDegToRad) - 8.0f));
            for (int r = 0; r < 2; ++r)
            {
                const FrameOut& o = r == 0 ? base.run.out[i] : bo.run.out[i];
                for (int side = 0; side < 2; ++side)
                    if (o.slide[side] >= 0.0f)
                    {
                        slideSum[r] += o.slide[side];
                        slideMax[r] = std::fmax(slideMax[r], o.slide[side]);
                        ++slideN[r];
                    }
            }
        }
        std::string d = std::to_string(n) + " frames at weight 1; twist error / tilt (deg):";
        for (int k = 0; k < 4; ++k)
            d += std::string(" ") + names[k] + " " + Fmt(twistErr[k], 4) + "/" + Fmt(tilt[k], 4);
        d += (aFeet ? ", foot-to-foot heading error " + Fmt(worstFeet, 4) : std::string(", foot-to-foot heading not used (feet fore / aft)")) + "; toe slide in contact mean/max (m/frame) offset 0 " +
             Fmt(slideN[0] ? slideSum[0] / float(slideN[0]) : 0.0f, 5) + "/" + Fmt(slideMax[0], 5) + ", offset 8 " +
             Fmt(slideN[1] ? slideSum[1] / float(slideN[1]) : 0.0f, 5) + "/" + Fmt(slideMax[1], 5);
        const bool body = twistErr[0] <= 0.1f && tilt[0] <= 0.1f && twistErr[1] <= 0.1f && tilt[1] <= 0.1f;
        const bool feet = twistErr[2] <= 0.1f && tilt[2] <= 0.1f && twistErr[3] <= 0.1f && tilt[3] <= 0.1f && worstFeet <= 0.1f;
        Check(n > 20 && fkOk && body && (!aFeet || feet),
              std::string("A15 BODY-PUBLISHED ") + aWhat + (aFeet ? ": published Hips / Head / feet turned rigidly by +8 deg about the up axis"
                                                                 : ": published Hips / Head turned by +8 deg about the up axis (feet measured)"),
              d);
    };
    span("steady idle", static_cast<size_t>(1.0f / kDt), idleTo, true);
    span("steady run", idleTo + static_cast<size_t>(1.5f / kDt), runTo, false);
}

// A10 F-AIM: Shooter ON vs OFF give identical output on every aiming / hip-fire pose / reload frame (weapon drawn, Pistol2H)
void AimCase(Rt& rt)
{
    Gen g("f-aim");
    g.psm = kWeaponDrawn;
    std::vector<bool> flag;
    auto mark = [&](size_t from, bool v) {
        flag.resize(g.Size(), false);
        for (size_t i = from; i < g.Size(); ++i)
            flag[i] = v;
    };
    g.Idle(1.0f);
    mark(0, false);
    g.psm = kWeaponDrawn | kUpperAim;
    size_t a = g.Size();
    g.Idle(0.5f);
    g.Seg(1.0f, 90.0f, 1.0f, 0.0f);
    g.Seg(1.0f, 180.0f, 1.0f, 0.0f);
    g.Seg(1.0f, -90.0f, 1.0f, 0.0f);
    g.Idle(0.5f);
    mark(a, true);
    g.psm = kWeaponDrawn;
    a = g.Size();
    g.Idle(1.0f);
    mark(a, false);
    a = g.Size();
    for (int k = 0; k < 10; ++k)
    {
        g.action = 8;
        g.Idle(0.05f);
        g.action = 0;
        g.Idle(0.05f);
    }
    g.Idle(1.0f);
    mark(a, true); // burst + the 0.75 s pose hold
    a = g.Size();
    g.action = 2;
    g.Idle(2.0f);
    g.action = 0;
    g.Idle(0.5f);
    mark(a, true);
    g.Idle(0.5f);
    mark(g.Size() - 30, false);
    RestoreDefaults();
    const Run offRun = RunSynth(rt, g, Opts{false, 3, false});
    ShooterModeOn();
    const Run onRun = RunSynth(rt, g, Opts{false, 3, false});
    RestoreDefaults();
    size_t n = 0, diff = 0, first = SIZE_MAX;
    for (size_t i = 0; i < g.Size(); ++i)
        if (flag[i])
        {
            ++n;
            if (!SameFrame(onRun.out[i], offRun.out[i]))
            {
                ++diff;
                if (first == SIZE_MAX)
                    first = i;
            }
        }
    std::string detail = std::to_string(n) + " flagged frames, " + std::to_string(diff) + " differ";
    if (first != SIZE_MAX)
        detail += ", first " + std::to_string(first) + " ON " + DbName(rt, onRun.out[first].db) + ":" + ClipName(rt, onRun.out[first].clip) + " vs OFF " +
                  DbName(rt, offRun.out[first].db) + ":" + ClipName(rt, offRun.out[first].clip) + " probe fn " + std::to_string(onRun.probe[first].shooterModeActive);
    Check(n > 300 && diff == 0, "A10 F-AIM: ADS, hip fire and reload frames are identical with Shooter on and off", detail);
}

// A16 BARREL-LOOP (aim pitch plan 2026-10-30, seam C): ADS with Pistol2H and a near aim point P (legacy ADS target, actor space), the
// capsule + camera turn 60 deg while aiming so the visual root lags the actor (a non-identity root frame on the live path).
Gen BarrelGen()
{
    Gen g("barrel loop");
    g.psm = kWeaponDrawn | kUpperAim;
    g.Idle(1.5f);
    g.Seg(0.5f, 0.0f, 0.0f, 60.0f);
    g.Idle(2.0f);
    return g;
}
constexpr float kBarrelP[3] = {0.6f, 8.0f, 1.1f}; // actor space (m): 8 m ahead, a little right and below the chest
// aProbe (optional): aProbe[i] = the seam probe of feed i (read before feed i + 1)
Run RunBarrel(Rt& rt, const Gen& aG, std::vector<motiondriver::BarrelProbe>* aProbe = nullptr)
{
    Opts o{false, 3, false};
    o.onFrame = [aProbe](size_t i) {
        if (aProbe && i > 0)
            aProbe->push_back(motiondriver::TestBarrelProbe());
        motiondriver::SetAimTarget(true, true, kBarrelP);
    };
    Run r = RunSynth(rt, aG, o);
    if (aProbe)
        aProbe->push_back(motiondriver::TestBarrelProbe());
    motiondriver::SetAimTarget(false, false, nullptr);
    return r;
}
// FNV-1a over the published pose hashes of a run (frames without a published pose count as a marker)
uint64_t RunDigest(const Run& aR)
{
    uint64_t h = 1469598103934665603ull;
    for (const FrameOut& o : aR.out)
        for (int k = 0; k < 9; ++k)
        {
            const uint64_t v = k < 8 ? (o.published ? o.hash : 0x9e3779b97f4a7c15ull) >> (8 * k) : uint64_t(o.stepped ? 1 : 0);
            h = (h ^ (v & 0xffu)) * 1099511628211ull;
        }
    return h;
}

// The loop-off digest of BarrelGen / RunBarrel recorded on the driver BEFORE seam C (Build/UTPP, 2026-10-30 12:22, the tree with the barrel
// loop in MotionPose but no driver wiring): with mm.aimAtCrosshair off the published poses must stay bit-identical.
// Re-recorded 2026-11-01 (footsink2): the ground locomotion fades of this replay (BlendRequest::groundFade) now hold the ankle height on the
// blend of the layers (lift, pelvis drop); the previous 0xc7812df98003fd02 is reproduced bit for bit with the ground fades never flagged.
// Re-recorded 2026-11-02 (feminine shoulders: per-side weights, 25 mm): the previous 0x409b0b37d893a0ee is reproduced bit for bit (5/5 PASS) by
// a build with -DUTPP_FEM_DEFAULT_LEGACY -DUTPP_FEM_SOFTSTART=0.995f -DUTPP_FEM_MAXREACH=0.999f -DUTPP_FEM_UNARMED_SOFTSTART=0.995f.
// Re-recorded 2026-11-09 (rifle-hand-fix: support-hand twist limit 85 deg, female Pistol2H only): the previous 0x671601c3185257c1 is reproduced
// bit for bit by a build with -DUTPP_SUPPORTHAND_LEGACY; all 240 frames are female Pistol2H, 14 published poses differ, only in the LeftArm /
// LeftHand local rotations (Build/work/pose-integration-20261109/t1f_compare_result.txt).
// Re-recorded 2026-11-11 (task C6) as DATA IDENTITY: the woman test database is now the package database; the previous 0x9190a413cb327fb0
// belongs to the former 800-clip bake and was NOT re-derived on a pre-seam-C build.
constexpr uint64_t kBarrelLoopOffDigest = 0xbe0d68357f289848ull;
void BarrelCase(Rt& rt)
{
    RestoreDefaults();
    // (1) exact seam geometry: a root frame turned 30 deg and shifted (0.35, -0.20) m. The pose-frame point P, put into the actor frame
    // (visualroot::Transform), comes back through InversePoint (the seam) as P, and ComputeAim of the same actor point gives the angles of P
    // from the 1.4 m chest: the sweep angles and the loop target are one point in one frame.
    {
        const visualroot::Frame fr{0.35f, -0.20f, 30.0f * integ::kDegToRad};
        const math::Vec3 pose{0.6f, 8.0f, 1.1f};
        math::Vec3 actor{}, back{};
        visualroot::Aim ra{};
        const bool ok = visualroot::Transform(fr, pose, actor) && visualroot::InversePoint(fr, actor, back) && visualroot::ComputeAim(fr, actor, ra);
        const float err = std::hypot(back.x - pose.x, back.y - pose.y, back.z - pose.z);
        const float wantYaw = std::atan2(-pose.x, pose.y) * kRadToDeg, wantPitch = std::atan2(pose.z - 1.4f, std::hypot(pose.x, pose.y)) * kRadToDeg;
        const float moved = std::hypot(actor.x - pose.x, actor.y - pose.y);
        Check(ok && err <= 1e-5f && std::fabs(ra.yawDeg - wantYaw) <= 1e-4f && std::fabs(ra.pitchDeg - wantPitch) <= 1e-4f && moved > 1.0f,
              "A16 BARREL seam geometry: root frame yaw 30 deg + (0.35, -0.20) m: InversePoint(actor P) = pose-frame P, ComputeAim = angles of P",
              "point error " + Fmt(err, 7) + " m, yaw " + Fmt(ra.yawDeg, 4) + " vs " + Fmt(wantYaw, 4) + ", pitch " + Fmt(ra.pitchDeg, 4) + " vs " +
                  Fmt(wantPitch, 4) + ", actor vs pose " + Fmt(moved, 3) + " m");
    }
    const Gen g = BarrelGen();
    // (2) loop off (the DLL default): the published poses are bit-identical to the driver before seam C; the loop never runs
    std::vector<motiondriver::BarrelProbe> offProbe;
    const Run off = RunBarrel(rt, g, &offProbe);
    const uint64_t offDigest = RunDigest(off);
    size_t offActive = 0;
    for (const auto& p : offProbe)
        offActive += p.active || p.measured || p.loop;
    char dg[64];
    std::snprintf(dg, sizeof dg, "digest 0x%016llx", static_cast<unsigned long long>(offDigest));
    Check(offDigest == kBarrelLoopOffDigest && offActive == 0, "A16 BARREL loop off: published poses bit-identical to the driver before seam C, loop never active",
          std::string(dg) + ", " + std::to_string(offActive) + " frames with the loop flagged");
    // (3) seam C on the live root path of the real driver: the point the evaluator used = InversePoint(root frame, actor P), its angles
    // = the sweep target angles; frames with a turned root (|yaw| >= 5 deg) where the actor point is NOT the pose point make it able to fail
    size_t root = 0, turned = 0, differs = 0, fallback = 0;
    float worstPoint = 0.0f, worstAngle = 0.0f, maxYaw = 0.0f;
    for (size_t i = 0; i < offProbe.size() && i < off.out.size(); ++i)
    {
        const auto& p = offProbe[i];
        if (!off.out[i].stepped || !p.evaluated || !p.haveVisual)
            continue;
        if (!p.rootPath)
        {
            ++fallback;
            worstPoint = std::fmax(worstPoint, std::hypot(p.visual[0] - p.actor[0], p.visual[1] - p.actor[1], p.visual[2] - p.actor[2]));
            continue;
        }
        ++root;
        math::Vec3 want{};
        if (!visualroot::InversePoint(visualroot::Frame{p.frame[0], p.frame[1], p.frame[2]}, math::Vec3{p.actor[0], p.actor[1], p.actor[2]}, want))
        {
            worstPoint = 1e9f;
            continue;
        }
        worstPoint = std::fmax(worstPoint, std::hypot(p.visual[0] - want.x, p.visual[1] - want.y, p.visual[2] - want.z));
        const float yaw = std::atan2(-p.visual[0], p.visual[1]) * kRadToDeg;
        const float pitch = std::atan2(p.visual[2] - 1.4f, std::hypot(p.visual[0], p.visual[1])) * kRadToDeg;
        worstAngle = std::fmax(worstAngle, std::fmax(std::fabs(Wrap(yaw - p.aimYawDeg)), std::fabs(pitch - p.aimPitchDeg)));
        maxYaw = std::fmax(maxYaw, std::fabs(p.frame[2]) * kRadToDeg);
        turned += std::fabs(p.frame[2]) * kRadToDeg >= 5.0f;
        differs += std::hypot(p.actor[0] - want.x, p.actor[1] - want.y) > 0.1f;
    }
    Check(root > 100 && turned >= 10 && differs >= 10 && worstPoint <= 1e-4f && worstAngle <= 1e-3f,
          "A16 BARREL seam C (live root path): the evaluator's aim point = InversePoint(root frame, actor P), its angles = the sweep target angles",
          std::to_string(root) + " root-path frames (" + std::to_string(turned) + " with |root yaw| >= 5 deg, max " + Fmt(maxYaw, 1) + " deg; " +
              std::to_string(differs) + " where the actor point is > 0.1 m off), " + std::to_string(fallback) + " fallback frames; worst point error " +
              Fmt(worstPoint, 6) + " m, worst angle error " + Fmt(worstAngle, 5) + " deg");
    // (4) loop on: the barrel error after the loop converges (MotionPoseTest 10 core criterion <= 1 deg) on the steady ADS windows (before the
    // turn and after it); the pose differs from loop off; switched off again the run is bit-identical to the first loop-off run
    std::vector<motiondriver::BarrelProbe> onProbe;
    motiondriver::SetBarrelLoop(true);
    const Run on = RunBarrel(rt, g, &onProbe);
    motiondriver::SetBarrelLoop(false);
    size_t measured = 0, clean = 0;
    float worstErr = 0.0f, firstErr = 0.0f, dist = 0.0f;
    for (size_t i = 0; i < onProbe.size() && i < on.out.size(); ++i)
    {
        const auto& p = onProbe[i];
        if (!on.out[i].stepped || !p.measured)
            continue;
        if (measured++ == 0)
            firstErr = p.errBeforeDeg;
        const bool steady = (i >= 60 && i < 90) || i >= 180;
        if (!steady || p.excluded != 0)
            continue;
        ++clean;
        worstErr = std::fmax(worstErr, std::fabs(p.errAfterDeg));
        dist = p.targetDist;
    }
    Check(clean >= 60 && worstErr <= 1.0f && RunDigest(on) != offDigest,
          "A16 BARREL loop on: the barrel error after the loop <= 1 deg on the steady ADS frames (the pose differs from loop off)",
          std::to_string(measured) + " measured frames, " + std::to_string(clean) + " steady unexcluded, worst |error| " + Fmt(worstErr, 3) +
              " deg (first measured error " + Fmt(firstErr, 2) + " deg), muzzle -> P " + Fmt(dist, 2) + " m");
    const Run again = RunBarrel(rt, g);
    Check(RunDigest(again) == offDigest, "A16 BARREL loop switched off again: bit-identical to the first loop-off run (nothing leaks)");
    RestoreDefaults();
}

// A-5: the strafe speeds the reds multiplier really produces (sideways 0.70 / backward 0.60, crouch 0.889 / 0.80 of the forward speed): the clips, the
// foot slide and the root lock at the real capsule speed
void StrafeSpeedCase(Rt& rt, int aLoco, float aDeg)
{
    RestoreDefaults();
    const std::string name = std::string(aLoco == integ::kLocoCrouch ? "crouch" : "stand") + " stick " + std::to_string(static_cast<int>(aDeg)) + " deg at the reds speed";
    Gen g("strafe-speed " + name);
    g.loco = aLoco;
    g.speedMul = true;
    g.Idle(aLoco == integ::kLocoCrouch ? 3.5f : 1.0f);
    const size_t s0 = g.Size();
    g.Seg(3.0f, aDeg, 1.0f, 0.0f);
    ShooterModeOn();
    const Run tail = RunSynth(rt, g);
    const Run raw = RunSynth(rt, g, Opts{true, 0, false});
    const size_t w0 = s0 + 60, w1 = g.Size();
    size_t n = 0, fam = 0, rootOk = 0;
    for (size_t i = w0; i < w1; ++i)
    {
        ++n;
        const std::string c = ClipName(rt, tail.out[i].clip);
        fam += aDeg >= 120.0f ? (Has(c, "Loop_B") || Has(c, "Start_B")) : (Has(c, "LL") || Has(c, "LR") || Has(c, "RL") || Has(c, "RR"));
        rootOk += RootErr(tail, i) <= 30.0f;
    }
    const auto spins = Picks(rt, tail, s0, w1, IsSpinOrReface);
    const SlideCmp sl = CompareSlide(tail, raw, w0, w1);
    const double speed = std::hypot(g.f.fr[w1 - 1].vx, g.f.fr[w1 - 1].vy);
    std::printf("INFO strafe speed %s: capsule %s m/s\n", name.c_str(), Fmt(speed).c_str());
    Check(fam * 100 >= n * (aDeg >= 120.0f ? 90 : 60), "strafe speed " + name + (aDeg >= 120.0f ? ": Start_B / Loop_B (B, BL, BR) clips on >= 90 % of the frames" : ": LL / LR / RL / RR clips on >= 60 % of the frames"),
          std::to_string(fam) + "/" + std::to_string(n));
    Check(rootOk * 100 >= n * 95, "strafe speed " + name + ": |actor - root| <= 30 deg on >= 95 % of the frames", std::to_string(rootOk) + "/" + std::to_string(n));
    std::string spinTxt = std::to_string(spins.size());
    for (const size_t sp : spins)
        spinTxt += " @" + std::to_string(sp - s0) + ":" + ClipName(rt, tail.out[sp].clip);
    Check(spins.empty(), "strafe speed " + name + ": no spin / reface pick", spinTxt);
    Check(sl.Ok(), "strafe speed " + name + ": planted foot slide tail < raw", sl.Text());
    RestoreDefaults();
}

// A-1: the speed ratio is answered only while v2 drives third-person locomotion with a fresh feed
void SpeedGateCase(Rt& rt)
{
    RestoreDefaults();
    Gen g("speed-gate");
    g.speedMul = true;
    g.Idle(1.0f);
    g.Seg(1.5f, 180.0f, 1.0f, 0.0f);
    ShooterModeOn();
    const Run r = RunSynth(rt, g);
    (void)r;
    auto mul = [] { return motiondriver::ShooterModeSpeedMul(0.0f, -1.0f, 1); };
    auto feedTail = [&](bool aTpp, int aN) {
        motionhost::FeedInput in = ToInput(g.f.fr.back());
        in.tppActive = aTpp;
        for (int i = 0; i < aN; ++i)
            motionhost::Feed(in);
    };
    const float fresh = mul();
    Check(std::fabs(fresh - 0.6f) < 0.005f && motiondriver::ShooterModeLive(), "A-1 speed gate: backward 0.60 while motion drives TPP (fresh Step)", Fmt(fresh, 3));
    Sleep(320);
    Check(mul() == 1.0f && motiondriver::ShooterModeLive(), "A-1 speed gate: exactly 1.0 when the last motion Step is older than 0.25 s (the live flag stays)", Fmt(mul(), 3));
    feedTail(true, 90);
    const float again = mul();
    feedTail(false, 3);
    const float fpp = mul();
    Check(fpp == 1.0f && std::fabs(again - 0.6f) < 0.005f, "A-1 speed gate: first person (no TPP) gives exactly 1.0 right after a TPP Step gave 0.60", "tpp " + Fmt(again, 3) + " fpp " + Fmt(fpp, 3));
    feedTail(true, 90);
    const float back = mul();
    motiondriver::SetOverlayInputs(0, 1, true, false); // overlay release: v2 skips
    feedTail(true, 3);
    const float released = mul();
    motiondriver::SetOverlayInputs(0, 1, false, false);
    Check(released == 1.0f && std::fabs(back - 0.6f) < 0.005f, "A-1 speed gate: a skipped motion feed (overlay release) gives exactly 1.0", "before " + Fmt(back, 3) + " released " + Fmt(released, 3));
    feedTail(true, 90);
    motiondriver::SetEnabled(false); // fallback: v2 off
    Check(mul() == 1.0f && !motiondriver::ShooterModeLive(), "A-1 speed gate: motion disabled gives exactly 1.0 and the live flag falls", Fmt(mul(), 3));
    motiondriver::SetEnabled(true);
    RestoreDefaults();
}

// A-6: the ratio follows the gait the chooser plays: the walk toggle walks at full stick (motion walk strafe 0.90 side / 0.75 back)
void WalkRatioCase(Rt& rt)
{
    RestoreDefaults();
    Gen g("walk-ratio");
    g.Idle(1.0f);
    g.Seg(1.5f, 90.0f, 1.0f, 0.0f);
    ShooterModeOn();
    const Run r = RunSynth(rt, g);
    (void)r;
    auto feedTail = [&](bool aWalk, float aSpeedScale, int aN) {
        motionhost::FeedInput in = ToInput(g.f.fr.back());
        in.velWorld[0] *= aSpeedScale;
        in.velWorld[1] *= aSpeedScale;
        in.haveExtra = true;
        in.walkToggled = aWalk;
        for (int i = 0; i < aN; ++i)
            motionhost::Feed(in);
    };
    feedTail(false, 1.0f, 60);
    const float run = motiondriver::ShooterModeSpeedMul(-1.0f, 0.0f, 1);
    feedTail(true, 0.45f, 60);
    const float walk = motiondriver::ShooterModeSpeedMul(-1.0f, 0.0f, 1);
    Check(std::fabs(run - 0.70f) < 0.005f && std::fabs(walk - 0.90f) < 0.005f, "A-6 walk ratio: full-stick side step 0.70 running, 0.90 with the walk toggle (the gait the chooser plays)",
          "run " + Fmt(run, 3) + " walk toggled " + Fmt(walk, 3));
    RestoreDefaults();
}

// A-7: a reseed of the root (frame gap > 0.25 s / re-entry after skipped feeds) keeps the held heading of the standing V
void ReseedCase(Rt& rt, bool aGap)
{
    RestoreDefaults();
    const std::string name = aGap ? "frame gap 0.4 s" : "5 skipped motion feeds (overlay release)";
    Gen g("reseed");
    g.Idle(1.5f);
    g.Seg(1.0f, 0, 0, 150.0f);
    g.Idle(0.5f);
    const size_t w0 = g.Size();
    if (aGap)
    {
        Frame fr = g.f.fr.back();
        fr.dt = 0.4f;
        fr.t += 0.4f;
        g.f.fr.push_back(fr);
        g.act.push_back(0);
    }
    g.Idle(0.5f);
    ShooterModeOn();
    Opts o;
    if (!aGap)
    {
        o.releaseFrom = w0;
        o.releaseTo = w0 + 5;
    }
    const Run r = RunSynth(rt, g, o);
    float maxDelta = 0;
    size_t n = 0;
    for (size_t i = w0; i < g.Size(); ++i)
        if (r.out[i].stepped && r.out[i].haveVisual)
        {
            ++n;
            maxDelta = std::fmax(maxDelta, std::fabs(Wrap(r.out[i].visualYawDeg - r.out[w0 - 1].visualYawDeg)));
        }
    Check(n > 10 && maxDelta <= 1.0f, "A-7 reseed " + name + ": the standing V keeps her held heading (|delta| <= 1 deg, offset 150 deg)", "max |delta| " + Fmt(maxDelta, 3) + " deg over " + std::to_string(n) + " frames");
    RestoreDefaults();
}

// A-8: a jump with the camera turned in flight: the root follows the actor in the air at <= 200 deg/s, so the touchdown does not meet a large offset
void AirCase(Rt& rt, float aCamTo)
{
    RestoreDefaults();
    const std::string name = "jump, camera " + std::to_string(static_cast<int>(aCamTo)) + " deg in flight";
    Gen g("air " + name);
    g.Idle(1.0f);
    g.Seg(1.0f, 0, 1.0f, 0);
    const size_t takeoff = g.Size();
    g.psm = WithDetail(kWeaponKnown, adapter::kPsmJump);
    g.Seg(0.35f, 0, 1.0f, aCamTo);
    g.Seg(0.65f, 0, 1.0f, aCamTo);
    const size_t land = g.Size();
    g.psm = WithDetail(kWeaponKnown, adapter::kPsmRegularLand);
    g.Seg(0.1f, 0, 1.0f, aCamTo);
    g.psm = kWeaponKnown;
    g.Seg(2.0f, 0, 1.0f, aCamTo);
    ShooterModeOn();
    const Run r = RunSynth(rt, g);
    const float airStep = MaxVisualStep(r, takeoff + 2, land);
    const float errAtLand = RootErr(r, land - 1);
    const float landStep = MaxVisualStep(r, land, land + 150);
    const auto spins = Picks(rt, r, land - 3, land + 150, IsSpinOrReface);
    size_t airFrames = 0;
    for (size_t i = takeoff; i < land; ++i)
        airFrames += r.probe[i].shooterModeActive && r.probe[i].rotationMode == 1;
    Check(airFrames > 40, "A-8 air " + name + ": the flight frames map as Shooter Strafe", std::to_string(airFrames) + " frames");
    Check(airStep <= 3.5f, "A-8 air " + name + ": the visual root follows at <= 200 deg/s in the air (<= 3.5 deg per feed)", Fmt(airStep) + " deg");
    Check(errAtLand <= 30.0f, "A-8 air " + name + ": |actor - root| <= 30 deg at touchdown", Fmt(errAtLand) + " deg");
    Check(landStep <= 8.0f && spins.size() <= 1, "A-8 air " + name + ": after the landing max step <= 8 deg per feed and at most one spin pick", Fmt(landStep) + " deg, " + std::to_string(spins.size()) + " spin picks");
    RestoreDefaults();
}

// ingame-2: Shooter, a slide with the camera turned away during it, then stick back in the exit window: no spin, the root is not thrown onto the camera
void SlideCase(Rt& rt, float aCamTo)
{
    RestoreDefaults();
    const std::string name = "slide, camera " + std::to_string(static_cast<int>(aCamTo)) + " deg";
    Gen g("shooterMode-" + name);
    g.loco = integ::kLocoSprint;
    g.maxSpeed = 8.2f;
    g.Idle(1.0f);
    g.Seg(1.5f, 0, 1.0f, 0);
    const size_t slideStart = g.Size();
    g.loco = integ::kLocoCrouch;
    g.psm = WithDetail(kWeaponKnown, 5);
    g.brake = 6.0f;
    g.Seg(0.2f, 0, 0, 0);
    g.Seg(0.5f, 0, 0, aCamTo); // looking away while sliding
    g.Seg(0.4f, 0, 0, aCamTo);
    const size_t exitStart = g.Size();
    g.psm = kWeaponKnown;
    g.maxSpeed = 3.85f;
    g.accel = 12.0f;
    g.Seg(0.6f, 180.0f, 1.0f, aCamTo); // stick back against the camera = on along the slide
    g.Seg(2.0f, 180.0f, 1.0f, aCamTo);
    ShooterModeOn();
    const Run r = RunSynth(rt, g, Opts{false, 0, true});
    const std::string st = motiondriver::Status();
    const bool declared = StatusField(st, "slideBundle=") == "1";
    Setup(declared, "slide case " + name + ": the bundle declares the slide section", "slideBundle=" + StatusField(st, "slideBundle="));
    if (!declared)
        return;
    const auto spins = Picks(rt, r, slideStart, g.Size(), IsSpin);
    const float step = MaxVisualStep(r, exitStart - 6, g.Size());
    size_t slideFrames = 0;
    for (size_t i = slideStart; i < exitStart; ++i)
        slideFrames += r.probe[i].rotationMode == 0;
    {
        size_t spinFrames = 0;
        float maxErr = 0;
        for (size_t i = slideStart; i < g.Size(); ++i)
        {
            spinFrames += r.probe[i].shouldSpin && i < exitStart + 36;
            maxErr = std::fmax(maxErr, RootErr(r, i));
        }
        std::printf("INFO slide case %s: chooser spin predicate on %zu frames, max |actor - root| %s deg\n", name.c_str(), spinFrames, Fmt(maxErr).c_str());
        Check(spinFrames == 0, "slide case " + name + ": the chooser spin predicate stays off from the slide through the 0.6 s exit window (root up to 180 deg off)",
              std::to_string(spinFrames) + " frames, max |actor - root| " + Fmt(maxErr) + " deg");
    }
    Check(spins.empty(), "slide case " + name + ": 0 SpinTransition picks from the slide to the end", std::to_string(spins.size()) + " picks");
    Check(step <= 8.0f, "slide case " + name + ": max visual root step <= 8 deg per feed through the exit", Fmt(step) + " deg");
    Check(slideFrames > 20, "slide case " + name + ": the slide frames map as Orient", std::to_string(slideFrames) + " frames");
    RestoreDefaults();
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::printf("usage: shooterMode <motion data dir> <base db> <flick_180_run.feed> <run_arc_left.feed> [--slide <bundle prefix> | --explore <case>]\n");
        return 2;
    }
    std::setvbuf(stdout, nullptr, _IONBF, 0); // progress is visible when the run is cut off
    std::string explore, slidePrefix;
    for (int i = 5; i + 1 < argc; ++i)
    {
        if (std::string(argv[i]) == "--explore")
            explore = argv[i + 1];
        if (std::string(argv[i]) == "--slide")
            slidePrefix = argv[i + 1];
    }
    bool barrelOnly = false; // --barrel: only A16 (aim pitch seam C), the recorded feeds are not needed
    for (int i = 5; i < argc; ++i)
        barrelOnly = barrelOnly || std::string(argv[i]) == "--barrel";
    const std::string data = argv[1];
    g_baseDb.assign(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::Paths pw = DataPaths(data, "woman");
    if (!slidePrefix.empty())
    {
        pw.files.db = slidePrefix + ".utppmm";
        pw.files.chooser = slidePrefix + ".chooser.json";
    }
    std::vector<FeedFile> feeds(2);
    for (int k = 0; k < 2; ++k)
    {
        const bool ok = LoadFeed(argv[3 + k], feeds[static_cast<size_t>(k)]);
        Setup(ok || barrelOnly, "feed loaded", std::string(argv[3 + k]));
    }
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    Setup(RigFromDb(pw.files.db, rig, &db) && rig.bones == 73, "woman motion rig 73 bones", std::to_string(rig.bones));
    if (g_setupErr || !db)
    {
        std::printf("shooterMode driver: setup failed (%d SETUP-ERROR)\n", g_setupErr);
        return 3;
    }
    Rt rt;
    rt.rig = &rig;
    rt.db = db.get();
    rt.metrics.rig = &rig;
    rt.metrics.db = db.get();
    for (uint32_t b = 0; b < db->NumBones(); ++b)
    {
        if (db->BoneName(b) == "LeftToeBase")
            rt.metrics.toe[0] = static_cast<int32_t>(b);
        if (db->BoneName(b) == "RightToeBase")
            rt.metrics.toe[1] = static_cast<int32_t>(b);
    }
    motiondriver::SetPathsOverride(0, pw);
    motionhost::SetMotionHooks(&kHooks);
    RestoreDefaults();

    if (!explore.empty())
    {
        // development aid: print the time series of one synthetic case ("start90", "start180" ...) with the slide comparison
        if (explore == "clips")
        {
            for (uint32_t c = 0; c < db->NumClips(); ++c)
            {
                const std::string n(db->ClipName(c));
                if (Has(n, "Start") || Has(n, "Reface") || Has(n, "Spin") || Has(n, "Pivot") || Has(n, "Turn"))
                    std::printf("clip %u %s len %.2f\n", c, n.c_str(), static_cast<double>(db->Clip(c).numFrames) / static_cast<double>(db->Header().fps));
            }
            return 0;
        }
        if (explore.rfind("speed", 0) == 0) // speed<deg>[c]: the strafe at the reds speed (stand; "c" = crouch), frames after 1 s of motion
        {
            Gen sg("explore-speed");
            sg.speedMul = true;
            if (explore.back() == 'c')
                sg.loco = integ::kLocoCrouch;
            sg.Idle(sg.loco == integ::kLocoCrouch ? 3.5f : 1.0f);
            const size_t s0 = sg.Size();
            sg.Seg(3.0f, static_cast<float>(std::atof(explore.c_str() + 5)), 1.0f, 0.0f);
            ShooterModeOn();
            const Run r = RunSynth(rt, sg);
            PrintTable(rt, r, s0 - 2, s0 + 24, 1);
            return 0;
        }
        float offset = 90.0f;
        if (explore.rfind("start", 0) == 0)
            offset = static_cast<float>(std::atof(explore.c_str() + 5));
        Gen g("explore");
        g.Idle(1.0f);
        g.Seg(1.5f, 0, 0, offset);
        g.Idle(0.5f);
        const size_t startFrame = g.Size();
        g.Seg(2.0f, 0, 1.0f, offset);
        ShooterModeOn();
        const Run r = RunSynth(rt, g);
        const Run raw = RunSynth(rt, g, Opts{true, 0, false});
        std::printf("explore %s: startFrame %zu\n", explore.c_str(), startFrame);
        PrintTable(rt, r, startFrame > 6 ? startFrame - 6 : 0, startFrame + 80, 2);
        for (const size_t span : {size_t(36), size_t(60), size_t(120)})
        {
            const SlideCmp sc = CompareSlide(r, raw, startFrame, startFrame + span);
            std::printf("slide window %zu frames: %s; max visual step %.2f\n", span, sc.Text().c_str(), static_cast<double>(MaxVisualStep(r, startFrame, startFrame + span)));
        }
        RestoreDefaults();
        motiondriver::SetEnabled(false);
        motionhost::SetMotionHooks(nullptr);
        motiondriver::Shutdown();
        motionhost::Shutdown();
        return 0;
    }

    // ---------------------------------------------------------------- F-ORBIT-IDLE, F-ORBIT-START, F-TIP-RELEASE
    if (barrelOnly)
        BarrelCase(rt);
    else if (!slidePrefix.empty())
    {
        SlideCase(rt, 90.0f);
        SlideCase(rt, 180.0f);
    }
    else
    {
        OrbitIdleCase(rt, "stand, unarmed", integ::kLocoStand, false);
        OrbitIdleCase(rt, "crouch, unarmed", integ::kLocoCrouch, false);
        OrbitIdleCase(rt, "stand, Pistol2H drawn", integ::kLocoStand, true);
        OrbitReloadCase(rt);
        OrbitOwnedCase(rt, true);
        OrbitOwnedCase(rt, false);
        for (const int loco : {integ::kLocoStand, integ::kLocoCrouch})
            for (const float off : {90.0f, 180.0f})
                OrbitStartCase(rt, loco, off);
        for (const int loco : {integ::kLocoStand, integ::kLocoCrouch})
            for (const float off : {90.0f, 180.0f})
                for (const bool ads : {true, false})
                    TipReleaseCase(rt, loco, off, ads);
        for (const int loco : {integ::kLocoStand, integ::kLocoCrouch})
            for (const float deg : {90.0f, 135.0f, 180.0f})
                StrafeSpeedCase(rt, loco, deg);
        SpeedGateCase(rt);
        WalkRatioCase(rt);
        ReseedCase(rt, true);
        ReseedCase(rt, false);
        for (const float deg : {90.0f, 180.0f})
            AirCase(rt, deg);
        OpenFeedCase(rt, feeds, true);
        for (const float deg : {45.0f, 90.0f, 180.0f})
            FlickCase(rt, true, deg);
        FlickCase(rt, false, 180.0f);
        SprintCase(rt);
        OpenFeedCase(rt, feeds, false);
        AimCase(rt);
#if !defined(UTPP_SHOOTER_MODE_MUTATION) || UTPP_SHOOTER_MODE_MUTATION == 0
        BarrelCase(rt); // A16 (also alone: --barrel); not in the mutation builds, its pinned digest would kill them for the wrong reason
#endif
        // the CET hip-fire lease (UTPPSetMotionHipLease) excludes the H2 latch on a whole reversal feed; without it the safeguard latches
        {
            std::vector<size_t> h2(2, 0);
            for (int lease = 1; lease >= 0; --lease)
            {
                RestoreDefaults();
                motionhost::SetBodyTurn(false);
                motiondriver::SetHipLease(lease != 0);
                const Run r = RunOnFeed(rt, feeds[0]);
                for (size_t i = 0; i < r.out.size(); ++i)
                    h2[static_cast<size_t>(lease)] += r.probe[i].h2Active;
                motiondriver::SetHipLease(false);
            }
            RestoreDefaults();
            Check(h2[1] == 0 && h2[0] > 0, "H2: the CET hip-fire lease excludes the latch on the whole flick feed (no latched frame), without it the safeguard latches",
                  "lease " + std::to_string(h2[1]) + " frames, no lease " + std::to_string(h2[0]) + " frames");
        }
        // A7: the 6 recorded feeds (next to the flick feed) and the 13 closed-loop variants
        {
            const std::string flickPath = argv[3];
            const size_t at = flickPath.find("flick_180_run");
            std::vector<FeedFile> id;
            for (const char* nm : {"slow_walk_half_stick", "run_arc_left", "flick_180_run", "crouch_walk_turns", "sprint_start_stop", "idle_then_start_sideways"})
            {
                FeedFile ff;
                const std::string path = flickPath.substr(0, at) + nm + ".feed";
                const bool ok = LoadFeed(path, ff);
                Setup(ok, "A7 feed loaded", path);
                if (ok)
                    id.push_back(std::move(ff));
            }
            struct CL
            {
                size_t feed;
                float rate;
                int repeat;
            };
            const CL cls[] = {{0, 360, 1}, {0, 720, 1}, {0, 720, 2}, {1, 360, 1}, {1, 720, 1}, {0, 540, 1}, {0, 1080, 1}, {0, 1440, 1}, {0, 5000, 1},
                              {1, 540, 1}, {1, 1080, 1}, {1, 1440, 1}, {1, 5000, 1}};
            for (const CL& c : cls)
                id.push_back(ClosedLoop(feeds[c.feed], c.rate, c.repeat));
            Setup(id.size() == 6 + 13, "A7: 6 recorded feeds + 13 closed-loop variants", std::to_string(id.size()));
            for (const FeedFile& ff : id)
                OffIdentityCase(rt, ff);
            // A11: the 6 recorded feeds with the pelvis trim
            for (size_t k = 0; k < 6 && k < id.size(); ++k)
                TrimNoFeedbackCase(rt, id[k]);
            Check(g_trimPoseDiff > 0, "A11 TRIM: the trim changes the published pose on some frames (it is applied)", std::to_string(g_trimPoseDiff));
            TrimRunCase(rt, false);
            TrimRunCase(rt, true);
            // A13: the 6 recorded feeds with the body yaw offset (+ one with the pelvis trim on as well), A14: active / inactive states
            for (size_t k = 0; k < 6 && k < id.size(); ++k)
                BodyOffsetCase(rt, id[k], false);
            if (id.size() > 1)
                BodyOffsetCase(rt, id[1], true);
            Check(g_boFullFrames > 0, "A13 BODY-OFFSET: the offset reaches weight 1 (+8.000 deg) on some frames (it is applied)", std::to_string(g_boFullFrames));
            BodyStatesCase(rt);
            BodyPublishedCase(rt);
        }
        // the speed natives: lock-free, exactly 1.0 unless Shooter is live
        {
            RestoreDefaults();
            const float offV = motiondriver::ShooterModeSpeedMul(0.0f, -1.0f, 1);
            ShooterModeOn();
            const bool live = motiondriver::ShooterModeLive();
            const float backV = motiondriver::ShooterModeSpeedMul(0.0f, -1.0f, 1);
            RestoreDefaults();
            const float offAgain = motiondriver::ShooterModeSpeedMul(0.0f, -1.0f, 1);
            Check(offV == 1.0f && offAgain == 1.0f && !motiondriver::ShooterModeLive(), "ShooterModeSpeedMul is exactly 1.0 with Shooter off (also after RestoreDefaults)");
            Check(live && backV >= 0.59f && backV <= 1.0f, "ShooterModeSpeedMul is live while Shooter is on (motion enabled, bundle loaded) and answers a Neutral ratio",
                  "live " + std::to_string(live) + " " + Fmt(backV, 3));
            std::string why;
            Check(DefaultsInStatus(why), "RestoreDefaults: Status shows shooterMode=0 h2Guard=1", why);
        }
    }

    RestoreDefaults();
    motiondriver::SetEnabled(false);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    if (g_setupErr)
    {
        std::printf("shooterMode driver: %d PASS %d FAIL, setup incomplete (%d SETUP-ERROR)\n", g_pass, g_fail, g_setupErr);
        return 3;
    }
    std::printf("shooterMode driver: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
