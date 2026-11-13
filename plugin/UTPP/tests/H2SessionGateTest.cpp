// Shooter mode, acceptance A9: the H2 safeguard on RECORDED GAME SESSIONS, on the current mode.
// Usage: utpp_motion_driver_h2session_test <v2 data dir> <v1 database.utppmm> <session feed (31 columns)> <sidecar> [--no-driver]
//   feed    = the recorded log (MM feed dump), sidecar = tests/data/h2_sidecar_*.txt made by tools/replay/h2_sidecar_from_log.py
//             from the log of the same session (native body turn off intervals, Locomotion active windows, why the native was off, the in-game
//             rotation mode Strafe windows, the weapon action windows; all on the feed time axis, alignment verified by the camTrace body yaw).
// Claim under test: in the CURRENT mode (weapon turn on, body turn driving) the H2 safeguard
// never changes an animation decision outside the frames that were Strafe in game anyway.
//
// A9a  gate level (the replay the offline driver tests cannot do, ToInput drops haveTarget / weapon / air): MotionFeed from ALL 31 feed columns
//      (haveTarget + targetYawDeg, psm -> UpperBody / detail / weapon, onGround / distance to ground / PSM fall, walk), MapContext::
//      bodyTurnSwitchedOff as MotionDriver Step computes it (native off AND the Locomotion blockers are only weapon / disabled / killed / not installed
//      AND tpp AND no melee), hipFireAny from the weapon action windows, MapConfig::visualStrafe from the Strafe windows; MapToChooser with the
//      real MapState. Pass: MapState::h2Active is false on EVERY frame outside the in-game Strafe windows [t0 - 0.02, t1 + 0.15].
//      Stricter variant: the same without the hip-fire / visualStrafe exclusions of the gate (they only exist inside the windows).
//      Evidence variants (INFO, no verdict): arm cause ignored (every off state arms), melee not excluded.
// A9b  full driver (real v1 MMDriver + MotionDriver, 17 columns + haveTarget / air / extra / psm as the game feeds them): the same session with the
//      H2 guard on and off, UTPPSetMMBodyTurn replayed from the sidecar, the Locomotion reasons probe and the weapon action from the sidecar. Pass:
//      the first frame whose decision (stepped, clip, database, clip time, published pose hash) differs between the two runs lies inside an
//      in-game Strafe window (no differing frame at all also passes), and every frame with the probe h2Active is inside a window.
#define main UTPPUnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include "../src/input/LocomotionCore.hpp"

#include <cstdlib>
#include <set>

namespace
{
constexpr double kPre = 0.02, kPost = 0.15; // window margins (spec A9)
constexpr double kHipHoldS = 0.75;          // DLL hip-fire pose hold after the last Shoot (StationaryShotTurn.hpp / HipFirePose)

struct Win
{
    double a = 0, b = 0;
};
struct MaskEv
{
    double t = 0;
    bool weaponTurn = true;
};
struct Sidecar
{
    std::string name, feed;
    std::vector<Win> off, freeMove, strafe, melee, lease; // lease: the CET hip-fire lease (UTPPSetMotionHipLease), an H2 exclusion
    std::vector<std::pair<std::string, Win>> why;
    std::map<int, std::vector<Win>> act;
    std::vector<MaskEv> mask;
};
double ParseT(const std::string& s) { return s == "inf" ? 1.0e30 : std::atof(s.c_str()); }
bool LoadSidecar(const std::string& aPath, Sidecar& s)
{
    std::ifstream in(aPath);
    if (!in)
        return false;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream l(line);
        std::string k;
        l >> k;
        if (k == "session")
            l >> s.name;
        else if (k == "feed")
            l >> s.feed;
        else if (k == "off" || k == "freeMove" || k == "strafe" || k == "melee" || k == "lease")
        {
            std::string a, b;
            l >> a >> b;
            Win w{ParseT(a), ParseT(b)};
            (k == "off" ? s.off : k == "freeMove" ? s.freeMove : k == "strafe" ? s.strafe : k == "lease" ? s.lease : s.melee).push_back(w);
        }
        else if (k == "why")
        {
            std::string lab, a, b;
            l >> lab >> a >> b;
            s.why.push_back({lab, Win{ParseT(a), ParseT(b)}});
        }
        else if (k == "act")
        {
            int n = 0;
            std::string a, b;
            l >> n >> a >> b;
            s.act[n].push_back(Win{ParseT(a), ParseT(b)});
        }
        else if (k == "mask")
        {
            std::string t, hx;
            int wt = 1;
            l >> t >> hx >> wt;
            s.mask.push_back(MaskEv{ParseT(t), wt != 0});
        }
    }
    return !s.off.empty() || !s.strafe.empty() || !s.name.empty();
}
bool InWin(const std::vector<Win>& w, double t, double aPre = 0.0, double aPost = 0.0)
{
    for (const Win& x : w)
        if (t >= x.a - aPre && t <= x.b + aPost)
            return true;
    return false;
}
bool WhyAt(const Sidecar& s, double t, const char* aLabelPrefix)
{
    for (const auto& p : s.why)
        if (p.first.rfind(aLabelPrefix, 0) == 0 && t >= p.second.a && t <= p.second.b)
            return true;
    return false;
}

// ---- 31-column feed (the integration LoadFeed reads 17 and fails on the '-' of the v2-owned frames)
struct Row
{
    double t = 0;
    float dt = 0, vx = 0, vy = 0, vz = 0, mvx = 0, mvy = 0, yaw = 0, cam = 0;
    int loco = 0;
    uint32_t psm = 0;
    bool haveTarget = false;
    float targetYaw = 0, capX = 0, capY = 0, capZ = 0;
    int upperBody = -1, detailed = -1;
    bool onGround = true;
    float distGround = -2.0f;
    int psmFall = 0;
    bool walk = false;
};
float F(const std::string& s) { return s == "-" ? 0.0f : static_cast<float>(std::atof(s.c_str())); }
bool LoadRows(const std::string& aPath, std::vector<Row>& aRows)
{
    std::ifstream in(aPath);
    if (!in)
        return false;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream l(line);
        std::vector<std::string> c;
        std::string x;
        while (l >> x)
            c.push_back(x);
        if (c.size() < 31)
            return false;
        Row r;
        r.t = std::atof(c[0].c_str());
        r.dt = F(c[1]);
        r.vx = F(c[2]);
        r.vy = F(c[3]);
        r.vz = F(c[4]);
        r.mvx = F(c[5]);
        r.mvy = F(c[6]);
        r.yaw = F(c[7]);
        r.cam = F(c[8]);
        r.loco = std::atoi(c[9].c_str());
        r.psm = static_cast<uint32_t>(std::strtoull(c[10].c_str(), nullptr, 10));
        r.haveTarget = c[17] == "1";
        r.targetYaw = F(c[18]);
        r.capX = F(c[19]);
        r.capY = F(c[20]);
        r.capZ = F(c[21]);
        r.upperBody = std::atoi(c[25].c_str());
        r.detailed = c[26] == "-" ? -1 : std::atoi(c[26].c_str());
        r.onGround = c[27] == "1";
        r.distGround = F(c[28]);
        r.psmFall = std::atoi(c[29].c_str());
        r.walk = c[30] == "1";
        aRows.push_back(r);
    }
    return !aRows.empty();
}

double NormAbs(double a)
{
    while (a > 180.0)
        a -= 360.0;
    while (a <= -180.0)
        a += 360.0;
    return std::fabs(a);
}

// what MotionDriver Step reads from the Locomotion core: the blocker bits of this moment, reconstructed from the sidecar.
// Locomotion active (the camera saw it, or the CET guard pause / aim-align state: Locomotion drives, only the native is off) = no blocker.
// Locomotion inactive: kWeapon only when the weapon turn setting was off with a weapon drawn (block mask bit 13), else a blocker that never arms
// (kHold: free look / camera / other module, the stand-in for every other reason the log does not name).
uint32_t ReasonsAt(const Sidecar& s, double t, bool aWeaponDrawn)
{
    if (InWin(s.freeMove, t) || WhyAt(s, t, "off_paused") || WhyAt(s, t, "off_aim"))
        return 0;
    bool weaponTurn = true;
    for (const MaskEv& m : s.mask)
        if (m.t <= t)
            weaponTurn = m.weaponTurn;
    if (!weaponTurn && aWeaponDrawn)
        return utpp::freemove::kWeapon;
    return utpp::freemove::kHold;
}
bool MeleeOf(uint32_t aPsm) { return integ::PsmMeleeBits(aPsm) != 0 || integ::PsmMeleeStateOf(aPsm) >= 5; }

struct Variant
{
    const char* name;
    bool guard = true;
    bool exclusions = true; // MapConfig::visualStrafe from the Strafe windows + MapContext::hipFireAny from the weapon action windows
    bool armCause = true;   // false: every native-off state arms (reasons ignored)
    bool melee = true;      // false: melee frames count as an arming cause too
};
struct GateOut
{
    std::vector<uint8_t> h2, latched;
    std::vector<int> rot;
    std::vector<uint8_t> shouldSpin;
    std::vector<int> gait;
    size_t offFrames = 0, switchedFrames = 0;
};

GateOut RunGate(const std::vector<Row>& rows, const Sidecar& sc, const Variant& v)
{
    GateOut o;
    o.h2.assign(rows.size(), 0);
    o.latched.assign(rows.size(), 0);
    o.rot.assign(rows.size(), 0);
    o.shouldSpin.assign(rows.size(), 0);
    o.gait.assign(rows.size(), 0);
    adapter::MapState ms;
    adapter::MapConfig mc;
    mc.stanceFilterS = adapter::kStanceFilterSeconds;
    mc.stanceHoldInDodge = true;
    mc.stanceCleanEdge = true;
    mc.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
    mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
    mc.shooterMode = false;
    mc.h2Strafe = v.guard;
    const std::vector<Win> empty;
    const auto actIt = sc.act.find(8);
    const std::vector<Win>& act8 = actIt != sc.act.end() ? actIt->second : empty;
    for (size_t i = 0; i < rows.size(); ++i)
    {
        const Row& r = rows[i];
        const float dt = (std::isfinite(r.dt) && r.dt > 0.0f) ? std::fmin(r.dt, 0.25f) : 0.0f;
        const float vel[3] = {r.vx, r.vy, r.vz};
        const float mv[2] = {r.mvx, r.mvy};
        adapter::AirFeed air;
        air.valid = true;
        air.onGround = r.onGround;
        air.distanceToGround = r.distGround;
        air.psmFall = r.psmFall;
        adapter::MotionFeed f = adapter::MakeMotionFeed(vel, mv, dt, r.psm, r.loco, air, r.cam, r.yaw);
        f.wantsToWalk = r.walk;
        f.haveTarget = r.haveTarget && std::isfinite(r.targetYaw);
        f.targetYawDeg = f.haveTarget ? r.targetYaw : 0.0f;
        const bool off = InWin(sc.off, r.t);
        const bool melee = MeleeOf(r.psm);
        const bool armCause = v.armCause ? (ReasonsAt(sc, r.t, f.weaponDrawn) & ~(utpp::freemove::kWeapon | utpp::freemove::kDisabled | utpp::freemove::kKilled |
                                                                                   utpp::freemove::kNotInstalled)) == 0
                                         : true;
        adapter::MapContext x;
        x.rootYawValid = false;
        x.bodyTurnSwitchedOff = off && armCause && (v.melee ? !melee : true);
        mc.visualStrafe = v.exclusions && InWin(sc.strafe, r.t, 0.0, 0.0) && !f.aiming;
        x.hipFireAny = v.exclusions && InWin(act8, r.t, 0.0, kHipHoldS);
        x.hipLease = v.exclusions && InWin(sc.lease, r.t);
        o.offFrames += off;
        o.switchedFrames += x.bodyTurnSwitchedOff;
        const mm::motion::ChooserState cs = adapter::MapToChooser(f, mc, ms, x);
        o.h2[i] = ms.h2Active;
        o.latched[i] = ms.h2Latched;
        o.rot[i] = static_cast<int>(ms.st.rotationMode);
        o.shouldSpin[i] = cs.shouldSpinTransition;
        o.gait[i] = cs.gait;
    }
    return o;
}

std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0)
{
    char buf[256];
    std::snprintf(buf, sizeof buf, aFmt, a, b, c, d);
    return buf;
}

// latched frames outside the in-game Strafe windows (the verdict) and inside them (informational), episodes printed
size_t Report(const char* aName, const GateOut& g, const std::vector<Row>& rows, const Sidecar& sc)
{
    size_t inside = 0, outside = 0;
    double firstOut = -1;
    std::vector<std::pair<double, double>> eps; // outside episodes
    for (size_t i = 0; i < rows.size(); ++i)
    {
        if (!g.h2[i])
            continue;
        if (InWin(sc.strafe, rows[i].t, kPre, kPost))
            ++inside;
        else
        {
            ++outside;
            if (firstOut < 0)
                firstOut = rows[i].t;
            if (!eps.empty() && rows[i].t - eps.back().second < 0.2)
                eps.back().second = rows[i].t;
            else
                eps.push_back({rows[i].t, rows[i].t});
        }
    }
    std::printf("INFO A9a %s: h2Active frames %zu (inside Strafe windows %zu, OUTSIDE %zu)\n", aName, inside + outside, inside, outside);
    for (size_t k = 0; k < eps.size() && k < 12; ++k)
    {
        // t, input angle against the actor, why the native was off, speed
        size_t i = 0;
        while (i < rows.size() && rows[i].t < eps[k].first)
            ++i;
        const Row& r = rows[std::min(i, rows.size() - 1)];
        double iy = r.haveTarget ? r.targetYaw : r.cam - std::atan2(r.mvx, r.mvy) * 57.29577951;
        std::printf("INFO   outside episode feed t %.3f..%.3f: input-vs-actor %.0f deg, speed %.2f m/s, melee %d, freeMove %d, why paused %d aim %d\n", eps[k].first,
                    eps[k].second, NormAbs(iy - r.yaw), std::hypot(r.vx, r.vy), MeleeOf(r.psm) ? 1 : 0, InWin(sc.freeMove, r.t) ? 1 : 0,
                    WhyAt(sc, r.t, "off_paused") ? 1 : 0, WhyAt(sc, r.t, "off_aim") ? 1 : 0);
    }
    return outside;
}

struct Pick
{
    bool stepped = false;
    int32_t clip = -1, db = -1;
    float clipTime = 0;
    uint64_t hash = 0;
    bool h2 = false;
};
motionhost::FeedInput ToInputFull(const Row& r)
{
    motionhost::FeedInput in;
    in.velWorld[0] = r.vx;
    in.velWorld[1] = r.vy;
    in.velWorld[2] = r.vz;
    in.move[0] = r.mvx;
    in.move[1] = r.mvy;
    in.cameraYawDeg = r.cam;
    in.playerYawDeg = r.yaw;
    in.locoState = r.loco;
    in.tppActive = true;
    in.dt = r.dt;
    in.haveTarget = r.haveTarget;
    in.targetYawDeg = r.targetYaw;
    in.psmBlock = r.psm;
    in.haveExtra = true;
    in.walkToggled = r.walk;
    in.capsulePos[0] = r.capX;
    in.capsulePos[1] = r.capY;
    in.capsulePos[2] = r.capZ;
    in.haveAir = true;
    in.onGround = r.onGround;
    in.distToGround = r.distGround;
    in.psmFall = r.psmFall;
    return in;
}
uint32_t g_probeReasons = 0;
uint32_t ReasonsProbe() { return g_probeReasons; }

// Settings back to the shipped defaults before a case; the assertion keeps one case from leaking into the next (identity-3).
bool RestoreDefaults(bool aGuard)
{
    motiondriver::SetShooterMode(false);
    motiondriver::SetH2Guard(aGuard);
    motionhost::SetBodyTurn(true);
    motiondriver::SetWeaponAction(0);
    motiondriver::SetHipLease(false);
    const std::string st = motiondriver::Status();
    return StatusField(st, "shooterMode=") == "0" && StatusField(st, "h2Guard=") == (aGuard ? "1" : "0");
}

std::vector<Pick> RunFull(const Rig& aRig, const std::vector<Row>& rows, const Sidecar& sc, bool aGuard)
{
    std::vector<Pick> out;
    out.reserve(rows.size());
    Check(RestoreDefaults(aGuard), std::string("A9b settings restored (shooterMode=0 h2Guard=") + (aGuard ? "1" : "0") + ")");
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    ResetBase(aRig);
    motiondriver::SetEnabled(false);
    motiondriver::SetEnabled(true);
    motionhost::Feed(ToInputFull(rows.front()));
    motiondriver::WaitLoader();
    ResetBase(aRig);
    motiondriver::SetEnabled(false);
    motiondriver::SetEnabled(true);
    RestoreDefaults(aGuard);
    motiondriver::SetLocomotionReasonsProbe(&ReasonsProbe);
    const std::vector<Win> empty;
    const auto a8 = sc.act.find(8), a2 = sc.act.find(2);
    const std::vector<Win>& act8 = a8 != sc.act.end() ? a8->second : empty;
    const std::vector<Win>& act2 = a2 != sc.act.end() ? a2->second : empty;
    bool bodyTurn = true;
    for (size_t i = 0; i < rows.size(); ++i)
    {
        const Row& r = rows[i];
        const bool off = InWin(sc.off, r.t);
        if (off == bodyTurn) // bodyTurn true while off is true -> switch
        {
            bodyTurn = !off;
            motionhost::SetBodyTurn(bodyTurn);
        }
        g_probeReasons = off ? ReasonsAt(sc, r.t, integ::PsmWeaponDrawn(r.psm)) : 0u;
        motiondriver::SetWeaponAction(InWin(act8, r.t, 0.0, 0.0) ? 8 : (InWin(act2, r.t) ? 2 : 0));
        motiondriver::SetHipLease(InWin(sc.lease, r.t));
        g_log = HookLog{};
        g_havePre = false;
        motionhost::Feed(ToInputFull(r));
        Pick p;
        const bool published = ReadPublished();
        p.hash = published ? SnapHash(g_read) : 0;
        p.stepped = g_log.stepResult;
        const std::string st = motiondriver::Status();
        const std::string pk = StatusField(st, "pick=clip:");
        const size_t at = pk.find('@'), sl = pk.find("/db");
        if (at != std::string::npos && sl != std::string::npos && sl > at)
        {
            p.clip = std::atoi(pk.substr(0, at).c_str());
            p.clipTime = static_cast<float>(std::atof(pk.substr(at + 1, sl - at - 1).c_str()));
            p.db = std::atoi(pk.substr(sl + 3).c_str());
        }
        p.h2 = motiondriver::TestStrafeProbe().h2Active && p.stepped;
        out.push_back(p);
    }
    motiondriver::SetLocomotionReasonsProbe(nullptr);
    RestoreDefaults(true);
    return out;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::printf("usage: h2session <motion data dir> <base db> <session feed> <sidecar> [--no-driver]\n");
        return 2;
    }
    const std::string data = argv[1], feedPath = argv[3], sidePath = argv[4];
    const bool driver = !(argc > 5 && std::string(argv[5]) == "--no-driver");
    g_baseDb.assign(argv[2], argv[2] + std::strlen(argv[2]));

    std::vector<Row> rows;
    Sidecar sc;
    const bool rowsOk = LoadRows(feedPath, rows);
    Check(rowsOk, "feed loaded (31 columns)", feedPath.substr(feedPath.find_last_of("/\\") + 1) + " frames " + std::to_string(rows.size()));
    const bool sideOk = LoadSidecar(sidePath, sc);
    Check(sideOk, "sidecar loaded", sc.name + " off " + std::to_string(sc.off.size()) + " strafe " + std::to_string(sc.strafe.size()) +
          " freeMove " + std::to_string(sc.freeMove.size()) + " act8 " + std::to_string(sc.act.count(8) ? sc.act[8].size() : 0));
    if (g_fail || rows.empty())
    {
        std::printf("h2 session: %d PASS %d FAIL\n", g_pass, g_fail);
        return 1;
    }
    Check(sc.feed == feedPath.substr(feedPath.find_last_of("/\\") + 1), "sidecar belongs to the feed", sc.feed);
    {
        size_t bad = 0;
        for (const Row& r : rows)
            bad += (r.upperBody == 6) != adapter::AimingFromFeed(r.psm); // the decoded column and the psm bits must agree
        Check(bad == 0, "feed column upperBody agrees with the psm bits (AimingFromFeed)", std::to_string(bad) + " frames differ");
    }

    // ------------------------------------------------------------ A9a: gate level
    // the facts the claim rests on (sidecar + feed), printed so a reader sees what the gate had to face
    {
        size_t offMoving = 0, offMoving105 = 0, offMoving105Out = 0, melee = 0;
        for (const Row& r : rows)
        {
            if (!InWin(sc.off, r.t) || !r.onGround || r.upperBody == 6 || std::hypot(r.vx, r.vy) < 0.5f)
                continue;
            ++offMoving;
            melee += MeleeOf(r.psm);
            const float stick = std::hypot(r.mvx, r.mvy);
            if (stick < 0.15f || std::hypot(r.vx, r.vy) < 1.5f)
                continue;
            const double iy = r.haveTarget ? r.targetYaw : r.cam - std::atan2(r.mvx, r.mvy) * 57.29577951;
            if (NormAbs(iy - r.yaw) >= 105.0)
            {
                ++offMoving105;
                offMoving105Out += !InWin(sc.strafe, r.t, kPre, kPost) && !InWin(sc.act.count(8) ? sc.act[8] : std::vector<Win>{}, r.t, kPre, kHipHoldS);
            }
        }
        std::printf("INFO A9a facts %s: native-off frames in the sidecar: ground+moving+not ADS %zu (melee %zu); with input >= %g deg from the actor at >= 1.5 m/s %zu, "
                    "of them outside the Strafe/hip-fire windows %zu\n", sc.name.c_str(), offMoving, melee, static_cast<double>(adapter::kH2EnterDeg), offMoving105, offMoving105Out);
    }
    const Variant vDll{"as the DLL (arm cause + melee + hip-fire exclusions)", true, true, true, true};
    const Variant vNoExcl{"no hip-fire / visualStrafe exclusions", true, false, true, true};
    const Variant vAnyCause{"arm cause ignored (every native-off state arms)", true, true, false, true};
    const Variant vNoMelee{"arm cause ignored, melee not excluded", true, true, false, false};
    const Variant vOff{"guard off", false, true, true, true};
    const GateOut gDll = RunGate(rows, sc, vDll), gNoExcl = RunGate(rows, sc, vNoExcl), gAny = RunGate(rows, sc, vAnyCause), gNoMelee = RunGate(rows, sc, vNoMelee),
                  gOff = RunGate(rows, sc, vOff);
    std::printf("INFO A9a gate input: native-off frames %zu, bodyTurnSwitchedOff frames (as the DLL) %zu, (arm cause ignored) %zu\n", gDll.offFrames, gDll.switchedFrames,
                gAny.switchedFrames);
    size_t outDll = Report(vDll.name, gDll, rows, sc);
    Check(outDll == 0, "A9a h2Active false on every frame outside the in-game Strafe windows (as the DLL)", std::to_string(outDll) + " frames outside");
    size_t outNoExcl = Report(vNoExcl.name, gNoExcl, rows, sc);
    Check(outNoExcl == 0, "A9a ... also without the hip-fire / visualStrafe exclusions of the gate", std::to_string(outNoExcl) + " frames outside");
    const size_t outAny = Report(vAnyCause.name, gAny, rows, sc);
    const size_t outNoMelee = Report(vNoMelee.name, gNoMelee, rows, sc);
    {
        // the replay must exercise the gate, or the verdict above is vacuous: once the exclusions of the DLL are taken away the gate latches
        // (inside the in-game Strafe windows: hip fire / ADS recordings; outside: sessions with melee combat in motion)
        size_t latchedNoExcl = 0, latchedNoMelee = 0;
        for (size_t i = 0; i < rows.size(); ++i)
        {
            latchedNoExcl += gNoExcl.h2[i];
            latchedNoMelee += gNoMelee.h2[i];
        }
        Check(latchedNoExcl + latchedNoMelee > 0, "A9a the replay exercises the gate (it latches once the DLL exclusions are removed)",
              "no exclusions " + std::to_string(latchedNoExcl) + " frames, melee not excluded " + std::to_string(latchedNoMelee) + " frames (" +
                  std::to_string(outAny + outNoMelee) + " outside the windows)");
    }
    {
        size_t on = 0;
        for (uint8_t b : gOff.h2)
            on += b;
        Check(on == 0, "A9a guard off never latches", std::to_string(on));
    }
    {
        // the gate changes only the rotation mode / spin flag; outside the windows the chooser inputs must be identical with the guard on and off
        size_t firstDiff = SIZE_MAX, diffs = 0;
        for (size_t i = 0; i < rows.size(); ++i)
            if (gDll.rot[i] != gOff.rot[i] || gDll.shouldSpin[i] != gOff.shouldSpin[i] || gDll.gait[i] != gOff.gait[i])
            {
                ++diffs;
                if (firstDiff == SIZE_MAX)
                    firstDiff = i;
            }
        const bool ok = firstDiff == SIZE_MAX || InWin(sc.strafe, rows[firstDiff].t, kPre, kPost);
        Check(ok, "A9a chooser input (rotation mode, spin flag, gait) guard on vs off: the first differing frame is inside a Strafe window (or none differs)",
              diffs == 0 ? std::string("no frame differs") : Fmt("%.0f frames differ, first at feed t %.3f", static_cast<double>(diffs), rows[firstDiff].t));
    }

    // ------------------------------------------------------------ A9b: full driver, guard on vs off
    if (driver)
    {
        const motiondriver::Paths pw = DataPaths(data, "woman");
        Rig rig;
        std::unique_ptr<mm::motion::MotionDatabase> db;
        if (!RigFromDb(pw.files.db, rig, &db))
        {
            Check(false, "A9b woman motion rig", pw.files.db);
        }
        else
        {
            motiondriver::SetPathsOverride(0, pw);
            motionhost::SetMotionHooks(&kHooks);
            const std::vector<Pick> on = RunFull(rig, rows, sc, true);
            const std::vector<Pick> off = RunFull(rig, rows, sc, false);
            size_t stepped = 0, h2 = 0, h2Out = 0, diffs = 0, firstDiff = SIZE_MAX;
            for (size_t i = 0; i < rows.size() && i < on.size() && i < off.size(); ++i)
            {
                stepped += on[i].stepped;
                if (on[i].h2)
                {
                    ++h2;
                    h2Out += !InWin(sc.strafe, rows[i].t, kPre, kPost);
                }
                const bool same = on[i].stepped == off[i].stepped && on[i].clip == off[i].clip && on[i].db == off[i].db &&
                                  on[i].clipTime == off[i].clipTime && on[i].hash == off[i].hash;
                if (!same)
                {
                    ++diffs;
                    if (firstDiff == SIZE_MAX)
                        firstDiff = i;
                }
            }
            int32_t lb = -1;
            Check(motiondriver::LoadedBody(lb) && lb == 0 && motiondriver::Reason() == Fallback::None, "A9b bundle loaded, no fallback", adapter::FallbackName(motiondriver::Reason()));
            // v2 hands frames to v1 in some PSM states (release details, gate): only a replay that never ran v2 would be vacuous
            Check(stepped * 2 >= rows.size(), "A9b motion stepped on >= 50 % of the frames", std::to_string(stepped) + "/" + std::to_string(rows.size()));
            std::printf("INFO A9b guard on: probe h2Active frames %zu (outside windows %zu); decisions differing from guard off: %zu frames\n", h2, h2Out, diffs);
            Check(h2Out == 0, "A9b full driver: every probe h2Active frame lies inside an in-game Strafe window", std::to_string(h2Out) + " outside");
            const bool ok = firstDiff == SIZE_MAX || InWin(sc.strafe, rows[firstDiff].t, kPre, kPost);
            Check(ok, "A9b full driver guard on vs off: the first differing decision lies inside an in-game Strafe window (or none differs)",
                  firstDiff == SIZE_MAX ? std::string("no frame differs")
                                        : Fmt("first diff frame %.0f at feed t %.3f", static_cast<double>(firstDiff), rows[firstDiff].t));
            motiondriver::SetEnabled(false);
            motionhost::SetMotionHooks(nullptr);
            motiondriver::Shutdown();
            motionhost::Shutdown();
        }
    }
    std::printf("h2 session %s: %d PASS %d FAIL\n", sc.name.c_str(), g_pass, g_fail);
    return g_fail ? 1 : 0;
}
