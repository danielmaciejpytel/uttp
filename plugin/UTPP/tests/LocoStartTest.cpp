// Start from rest and jump take-off: offline replay of the 2026-10-31 loco diagnostic (findings CR-2 / GW-2 / HIP-3 and JS1).
// Real feed windows (tests/data/loco_start_feeds_20261031_2255.txt, mm.feedDump rows copied verbatim) and synthetic starts go through the
// production stack minus the game: motiondriver::AirFloorStep (the runtime floor rule) -> adapter::MapToChooser (production MapConfig: slide, stance
// filter, walk toggle 1.6 m/s, minimum analog walk 0.82 m/s, gravity 16) -> BuildQueryTrajectory -> ChooserMatcher::Update on the installed
// woman v2 bundle (utpp_motion_woman_knee_twistfix_slide).
//
// Starts (CR-2 / GW-2 / HIP-3): the first stick frame from rest carries capsule velocity 0, so TurnAngle(future, 0) was the world yaw of the
// future and IsPivoting fired at most headings; IsStarting needed |future| >= |vel| + 100 cm/s, which a slow target (crouch ~48 cm/s, the
// analog floor walk ~80-100 cm/s) never reaches. Checks: no IsPivoting on the first moving frames, the first pick of the start is a *_Starts
// row, no *_Pivots pose in the first 0.5 s, the start clip is held; real pivots (run / walk reversal, 90 deg walk turn) still flag
// IsPivoting and pick a *_Pivots row; on every replayed frame the predicates equal the motion formulas wherever the fix does not apply
// (|vel|xy > kHasVelocity for IsPivoting, |future|xy >= kStartingMargin for IsStarting).
// Take-off (JS1): the PSM reports Jump (18) one feed before the capsule launches (onGround 1, vz 0 / -0.39); InAir on that feed searched
// the jump databases on a query without vertical velocity or floor (TimeToLand = no hit 1.52) and re-picked one feed later. Checks: one
// jump pick per take-off within 0.1 s, its TimeToLand < 1, a Jump_F_Start_Stand clip for the standing jump; the movement mode of the
// held replay differs from the unheld one only on the grounded take-off feeds; --jumps: the same on every take-off of the JumpPickTest fixtures.
// The held feed maps as the previous ground frame: no new pick in any database on it (no ground re-pick right before the jump pick) and
// the previous gait / stance / rotation mode, orient and strafe (visualStrafe), incl. the sprint (4) -> 18 take-offs of the --jumps fixtures.
//
// Usage: utpp_mm_locostart_test <loco_start_feeds.txt> <bundle path without extension> [--jumps <jump_feeds.txt>]... [--report]
// Mutations (UTPP_LOCOSTART_MUTATION, src/mm/MotionTrajectory.hpp / MotionAdapter.hpp): 1 IsPivoting without the velocity floor, 2 IsStarting with
// the fixed 100 cm/s margin, 3 no take-off hold, 4 the first hold (mode only, the detail stays 18: Run gait and a ground re-pick on a
// sprint take-off). Each must print a FAIL line of its own check.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../src/MotionDriver.hpp" // AirFloorStep (the runtime floor rule)
#include "../src/mm/MotionAdapter.hpp"

namespace adapter = utpp::mm::adapter;
namespace motion = utpp::mm::motion;
namespace motiontraj = utpp::mm::motiontraj;
namespace chooser = utpp::mm::chooser;

namespace
{
int g_fail = 0, g_checks = 0;
bool g_report = false;
void Check(bool aOk, const std::string& aWhat)
{
    ++g_checks;
    if (!aOk)
        ++g_fail;
    if (!aOk || g_report)
        std::printf("%s %s\n", aOk ? "PASS" : "FAIL", aWhat.c_str());
}

struct Row
{
    double t = 0;
    float dt = 0, vel[3] = {0, 0, 0}, mv[2] = {0, 0}, yaw = 0, cam = 0, target = 0;
    int loco = 0, detailed = -1;
    uint32_t psm = 0;
    bool haveTarget = false;
    int onGround = -1;
    float dist = -1;
    int fall = 0, walk = 0;
};
struct Section
{
    std::string name;
    std::vector<Row> rows;
};

float Num(const std::string& aTok, float aDef = 0.0f) { return aTok == "-" ? aDef : std::strtof(aTok.c_str(), nullptr); }

bool LoadFixture(const char* aPath, std::vector<Section>& aOut)
{
    std::ifstream f(aPath);
    if (!f)
        return false;
    std::string line;
    std::vector<std::string> cols;
    while (std::getline(f, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.rfind("# cols", 0) == 0)
        {
            std::istringstream is(line.substr(6));
            std::string c;
            cols.clear();
            while (is >> c)
                cols.push_back(c);
            continue;
        }
        if (line.empty() || line[0] == '#')
            continue;
        if (line.rfind("@section ", 0) == 0 || line.rfind("@slide ", 0) == 0) // this fixture / the jump fixtures of tests/JumpPickTest.cpp
        {
            std::string name = line.substr(line.find(' ') + 1);
            aOut.push_back({name.substr(0, name.find(' ')), {}});
            continue;
        }
        if (aOut.empty() || cols.empty())
            return false;
        std::istringstream is(line);
        std::vector<std::string> tok;
        std::string t;
        while (is >> t)
            tok.push_back(t);
        if (tok.size() != cols.size())
            return false;
        auto at = [&](const char* aName) -> const std::string& {
            for (size_t i = 0; i < cols.size(); ++i)
                if (cols[i] == aName)
                    return tok[i];
            static const std::string kDash = "-";
            return kDash;
        };
        Row r;
        r.t = std::strtod(at("t").c_str(), nullptr);
        r.dt = Num(at("dt"));
        r.vel[0] = Num(at("vx"));
        r.vel[1] = Num(at("vy"));
        r.vel[2] = Num(at("vz"));
        r.mv[0] = Num(at("mvx"));
        r.mv[1] = Num(at("mvy"));
        r.yaw = Num(at("yawDeg"));
        r.cam = Num(at("camDeg"));
        r.loco = static_cast<int>(Num(at("loco")));
        r.psm = static_cast<uint32_t>(std::strtoull(at("psm").c_str(), nullptr, 10));
        r.haveTarget = at("haveTarget") == "1";
        r.target = Num(at("targetYawDeg"));
        r.detailed = at("detailed") == "-" ? -1 : static_cast<int>(Num(at("detailed")));
        r.onGround = at("onGround") == "-" ? -1 : static_cast<int>(Num(at("onGround")));
        r.dist = Num(at("distGround"), -1.0f);
        r.fall = static_cast<int>(Num(at("psmFall")));
        r.walk = static_cast<int>(Num(at("walk")));
        aOut.back().rows.push_back(r);
    }
    return !aOut.empty();
}

struct Model
{
    motion::MotionDatabase db;
    chooser::Table table;
    adapter::SlideSpec slide;
    adapter::QueryTimes qtimes;
    bool Load(const std::string& aStem, std::string& aErr)
    {
        if (!db.LoadFromFile((aStem + ".utppmm").c_str(), motion::kRigIdentityWoman, aErr))
            return false;
        if (!table.LoadFile((aStem + ".chooser.json").c_str(), aErr))
            return false;
        std::ifstream f(aStem + ".chooser.json", std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        if (!adapter::ParseSlideSpec(ss.str(), slide))
        {
            aErr = "no slide section";
            return false;
        }
        qtimes = adapter::BuildQueryTimes(db);
        return true;
    }
};

struct Options
{
    bool strafe = false;        // the host's hipfire pose hold (mc.visualStrafe) + aim facing
    float takeoffHoldS = -1.0f; // < 0 = production MapConfig default
};

struct Frame
{
    double t = 0;
    int detail = 0, onGround = -1, mode = 0, moveState = 0, gait = 0, stance = 0;
    bool starting = false, pivoting = false, moving = false, strafe = false;
    float stick = 0, ttl = 0, vzCmS = 0;
    motiontraj::V3 vel, fut;
    std::string db, clip;
    float clipTime = 0;
    bool jumped = false;
};

std::vector<Frame> Replay(Model& m, const std::vector<Row>& aRows, const Options& aOpt)
{
    std::vector<Frame> out;
    motion::ChooserMatcher matcher;
    std::string err;
    const motion::MatcherConfig cfg; // production defaults
    if (!matcher.Init(&m.db, &m.table, cfg, err))
    {
        std::printf("FAIL matcher init %s\n", err.c_str());
        ++g_fail;
        return out;
    }
    adapter::MapState map;
    std::vector<motion::TrajSample> traj(adapter::kQueryTrajMax);
    int32_t prevDb = -1;
    utpp::motiondriver::AirFloorState floor;
    for (const Row& r : aRows)
    {
        const float dt = std::fmin(r.dt > 0 ? r.dt : 0.0f, 0.25f);
        adapter::AirFeed air;
        air.valid = r.onGround >= 0;
        air.onGround = r.onGround != 0;
        air.distanceToGround = r.dist;
        air.psmFall = r.fall;
        if (air.valid) // the runtime floor (MotionDriver.cpp Step: dead reckoning + J2 take-off plane seed)
            air.distanceToGround = utpp::motiondriver::AirFloorStep(floor, air.onGround, r.dist, adapter::DetailFromFeed(r.psm, r.loco),
                                                              std::isfinite(r.vel[2]) ? r.vel[2] : 0.0f, dt)
                                       .distance;
        adapter::MotionFeed f = adapter::MakeMotionFeed(r.vel, r.mv, dt, r.psm, r.loco, air, r.cam, r.yaw);
        f.wantsToWalk = r.walk != 0;
        f.haveTarget = r.haveTarget && std::isfinite(r.target);
        f.targetYawDeg = f.haveTarget ? r.target : 0.0f;
        adapter::MapContext ctx;
        ctx.rootYawValid = true;
        ctx.rootYawDeg = r.yaw;
        ctx.aimFacing = aOpt.strafe;
        if (prevDb >= 0)
            ctx.currentDbPivots = adapter::DatabaseHasPivotsTag(m.db.DbName(uint32_t(prevDb)));
        adapter::MapConfig mc; // MotionDriver.cpp Step
        mc.slide = true;
        mc.slideSpec = m.slide;
        mc.slideDeceleration = adapter::kSlideDecelerationMs2;
        mc.slideAirGraceS = adapter::kSlideAirGraceSeconds;
        mc.exitAirGraceS = adapter::kExitAirGraceSeconds;
        mc.stanceFilterS = adapter::kStanceFilterSeconds;
        mc.stanceHoldInDodge = true;
        mc.stanceCleanEdge = true;
        mc.stanceCleanHoldS = adapter::kStanceCleanHoldSeconds;
        mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;
        mc.stanceSpeedBlendTime = matcher.BlendTimeFor(motion::ChooserState{}, motion::ChooserState{});
        mc.gameWalkTogglePoint = 1.6f;   // CET mm.walkSpeed of the session
        adapter::SetAnalogFloor(mc, 0.82f); // CET mm.minWalkSpeed of the session
        mc.gravity = 16.0f;
        mc.visualStrafe = aOpt.strafe;
        if (aOpt.takeoffHoldS >= 0.0f)
            mc.jumpTakeoffHoldS = aOpt.takeoffHoldS;
        const motion::ChooserState cs = adapter::MapToChooser(f, mc, map, ctx);
        const uint32_t nt = adapter::BuildQueryTrajectory(map, m.qtimes, traj.data(), uint32_t(traj.size()));
        if (nt == 0)
            continue;
        const motion::MatchResult res = matcher.Update(dt, traj.data(), nt, cs, nullptr);
        if (res.clip < 0)
            continue;
        prevDb = res.database;
        Frame fr;
        fr.t = r.t;
        fr.detail = adapter::DetailFromFeed(r.psm, r.loco);
        fr.onGround = r.onGround;
        fr.mode = cs.movementMode;
        fr.moveState = cs.movementState;
        fr.gait = cs.gait;
        fr.stance = cs.stance;
        fr.starting = cs.isStarting;
        fr.pivoting = cs.isPivoting;
        fr.moving = map.st.isMoving;
        fr.strafe = map.st.rotationMode == motiontraj::RotationMode::Strafe;
        fr.stick = std::hypot(r.mv[0], r.mv[1]);
        fr.ttl = cs.timeToLand;
        fr.vzCmS = cs.verticalVelocityCmS;
        fr.vel = map.st.velocity;
        fr.fut = map.st.trjFuture;
        fr.db = std::string(m.db.DbName(uint32_t(res.database)));
        fr.clip = std::string(m.db.ClipName(uint32_t(res.clip)));
        fr.clipTime = res.clipTime;
        fr.jumped = res.jumped;
        out.push_back(std::move(fr));
    }
    return out;
}

bool Has(const std::string& a, const char* aPart) { return a.find(aPart) != std::string::npos; }

void Dump(const char* aName, const std::vector<Frame>& aFr, double aFrom, double aTo)
{
    if (!g_report)
        return;
    for (const Frame& f : aFr)
        if (f.t >= aFrom && f.t <= aTo)
            std::printf("  %s t=%.3f det=%d gnd=%d mode=%d ms=%d st=%d pv=%d stance=%d gait=%d |v|=%.1f |f|=%.1f vz=%.0f ttl=%.2f %s%s:%s ct=%.3f\n",
                        aName, f.t, f.detail, f.onGround, f.mode, f.moveState, f.starting, f.pivoting, f.stance, f.gait,
                        motiontraj::SizeXY(f.vel), motiontraj::SizeXY(f.fut), f.vzCmS, f.ttl, f.jumped ? "*" : " ", f.db.c_str(), f.clip.c_str(),
                        f.clipTime);
}

// motion predicates (MotionTrajectory.hpp before the fix) where the fix must not change anything
void CheckPredicateIdentity(const char* aName, const std::vector<Frame>& aFr)
{
    int pivotDiff = 0, startDiff = 0, pivotN = 0, startN = 0;
    for (size_t i = 1; i < aFr.size(); ++i)
    {
        const Frame& f = aFr[i];
        // the database tag the predicate saw = the database played on the previous frame
        const bool prevPivots = adapter::DatabaseHasPivotsTag(aFr[i - 1].db);
        if (motiontraj::SizeXY(f.vel) > motiontraj::kHasVelocity)
        {
            ++pivotN;
            const float thr = f.strafe ? motiontraj::kPivotStrafe : motiontraj::kPivotOrient;
            const bool referenceOnset = std::fabs(motiontraj::TurnAngle(f.fut, f.vel)) >= thr && f.moving;
            pivotDiff += referenceOnset != f.pivoting ? 1 : 0;
        }
        if (motiontraj::SizeXY(f.fut) >= motiontraj::kStartingMargin)
        {
            ++startN;
            const bool referenceOnset = f.moving && motiontraj::SizeXY(f.fut) >= motiontraj::SizeXY(f.vel) + motiontraj::kStartingMargin && !prevPivots;
            startDiff += referenceOnset != f.starting ? 1 : 0;
        }
    }
    Check(pivotDiff == 0 && startDiff == 0, std::string("identity ") + aName + ": predicates equal Neutral outside the fix domain (pivot " +
                                                std::to_string(pivotDiff) + "/" + std::to_string(pivotN) + ", start " +
                                                std::to_string(startDiff) + "/" + std::to_string(startN) + " differ)");
}

// one start from rest: the first stick frame aT0; aWantDb = the Starts database the pick must come from
void CheckStart(const std::string& aName, const std::vector<Frame>& aFr, double aT0, const char* aWantDb)
{
    size_t i0 = aFr.size();
    for (size_t i = 0; i < aFr.size(); ++i)
        if (aFr[i].t >= aT0 - 1e-6)
        {
            i0 = i;
            break;
        }
    if (i0 >= aFr.size())
    {
        Check(false, "start " + aName + ": no frame at the start");
        return;
    }
    Dump(aName.c_str(), aFr, aT0 - 0.04, aT0 + 0.2);
    // no IsPivoting on the first four frames from rest (the zero-velocity frame and the first frames with a velocity heading)
    int pivots = 0;
    for (size_t i = i0; i < aFr.size() && i < i0 + 4; ++i)
        pivots += aFr[i].pivoting ? 1 : 0;
    Check(pivots == 0, "start " + aName + ": IsPivoting on " + std::to_string(pivots) + " of the first 4 moving frames (want 0)");
    // the first pick of the start (the first database switch out of the idle at or after the stick frame)
    size_t ip = aFr.size();
    for (size_t i = i0; i < aFr.size() && aFr[i].t < aT0 + 0.3; ++i)
        if (aFr[i].jumped && !Has(aFr[i].db, "Idle"))
        {
            ip = i;
            break;
        }
    const bool startPick = ip < aFr.size() && Has(aFr[ip].db, aWantDb);
    Check(startPick, "start " + aName + ": first pick " + (ip < aFr.size() ? aFr[ip].db + ":" + aFr[ip].clip : std::string("none")) +
                         " is a " + aWantDb + " row (IsStarting " + (ip < aFr.size() && aFr[ip].starting ? "1" : "0") + ")");
    int pivotPoses = 0;
    for (size_t i = i0; i < aFr.size() && aFr[i].t < aT0 + 0.5; ++i)
        pivotPoses += Has(aFr[i].db, "Pivots") ? 1 : 0;
    Check(pivotPoses == 0, "start " + aName + ": " + std::to_string(pivotPoses) + " frames of a *_Pivots pose in the first 0.5 s (want 0)");
    if (startPick)
    {
        double held = 0;
        for (size_t i = ip + 1; i < aFr.size() && aFr[i].clip == aFr[ip].clip; ++i)
            held = aFr[i].t - aFr[ip].t;
        Check(held >= 0.15, "start " + aName + ": start clip " + aFr[ip].clip + " held " + std::to_string(held) + " s (want >= 0.15)");
    }
}

// synthetic feed: aRestS of rest, then the stick aStick at aStickDeg from the body (0 = forward, camera = player yaw aYaw, the body
// keeps its yaw); the capsule stays at 0 on the first stick frame (the game feed, all three recorded starts) and then accelerates at
// aAccMs2 to aSpeedMs along the stick direction
std::vector<Row> Synthetic(float aYaw, float aStick, float aSpeedMs, float aAccMs2, bool aCrouch, bool aWalk, double aRestS, double aMoveS,
                           float aDt, float aStickDeg = 0.0f)
{
    std::vector<Row> rows;
    const float r = (aYaw + aStickDeg) * 3.14159265f / 180.0f, a = aStickDeg * 3.14159265f / 180.0f;
    const float fx = -std::sin(r), fy = std::cos(r);
    float sp = 0;
    const int n = int(std::lround((aRestS + aMoveS) / aDt)), rest = int(std::lround(aRestS / aDt));
    for (int i = 0; i < n; ++i)
    {
        Row w;
        w.t = i * aDt;
        w.dt = aDt;
        w.yaw = w.cam = aYaw;
        w.onGround = 1;
        w.dist = -2.0f;
        w.walk = aWalk ? 1 : 0;
        w.loco = aCrouch ? 2 : 1; // DetailFromFeed: no detail field in the block -> the v1 loco state (1 Stand / 2 Crouch)
        w.psm = 1627398144u;      // the recorded ground block (idle / crouch rows of the fixture)
        if (i >= rest)
        {
            w.mv[0] = -std::sin(a) * aStick; // camera space (MoveX right, MoveY forward): world forward(aYaw + aStickDeg)
            w.mv[1] = std::cos(a) * aStick;
            if (i > rest)
                sp = std::fmin(aSpeedMs, sp + aAccMs2 * aDt);
        }
        w.vel[0] = fx * sp;
        w.vel[1] = fy * sp;
        rows.push_back(w);
    }
    return rows;
}

// moving steadily at aSpeedMs, then the stick turns by aTurnDeg (camera fixed) and the capsule velocity rotates / reverses toward it at aAccMs2
std::vector<Row> SyntheticTurn(float aYaw, float aStick, float aSpeedMs, float aAccMs2, float aTurnDeg, bool aWalk, double aRunS, double aAfterS,
                               float aDt)
{
    std::vector<Row> rows;
    const float r = aYaw * 3.14159265f / 180.0f;
    float vx = -std::sin(r) * aSpeedMs, vy = std::cos(r) * aSpeedMs;
    const float r2 = (aYaw + aTurnDeg) * 3.14159265f / 180.0f;
    const float tx = -std::sin(r2) * aSpeedMs, ty = std::cos(r2) * aSpeedMs;
    const int n = int(std::lround((aRunS + aAfterS) / aDt)), run = int(std::lround(aRunS / aDt));
    for (int i = 0; i < n; ++i)
    {
        Row w;
        w.t = i * aDt;
        w.dt = aDt;
        w.yaw = w.cam = aYaw;
        w.onGround = 1;
        w.dist = -2.0f;
        w.walk = aWalk ? 1 : 0;
        w.loco = 1;
        w.psm = 1627398144u;
        if (i < run)
            w.mv[1] = aStick;
        else
        {
            const float sr = aTurnDeg * 3.14159265f / 180.0f; // stick rotated by the turn (camera space: right = +x)
            w.mv[0] = -std::sin(sr) * aStick;
            w.mv[1] = std::cos(sr) * aStick;
            const float dx = tx - vx, dy = ty - vy, d = std::hypot(dx, dy), step = aAccMs2 * aDt;
            if (d <= step)
                vx = tx, vy = ty;
            else
                vx += dx / d * step, vy += dy / d * step;
        }
        w.vel[0] = vx;
        w.vel[1] = vy;
        rows.push_back(w);
    }
    return rows;
}

double FirstStick(const std::vector<Row>& aRows)
{
    for (const Row& r : aRows)
        if (std::hypot(r.mv[0], r.mv[1]) > 0.0f)
            return r.t;
    return -1.0;
}

const Section* Find(const std::vector<Section>& aS, const char* aName)
{
    for (const Section& s : aS)
        if (s.name == aName)
            return &s;
    return nullptr;
}

// ---- the held take-off feeds (the hold sets mode 0 where the unheld replay has mode 1, on a grounded Jump 18 feed): the held feed must map
// as the previous ground frame (MapToChooser maps it with the previous PSM detail), so it takes no new pick in ANY database (no ground
// re-pick right before the jump pick) and keeps the previous frame's gait, stance and rotation mode (sprint gait, visualStrafe). Sprint
// take-offs (previous ground detail 4) counted separately: the first hold mapped them to the Run gait and re-picked Sprint_to_Run.
struct HeldFeedStats
{
    int held = 0, sprint = 0, jumped = 0, inputs = 0;
    std::string what; // the first offending feed
};
HeldFeedStats HeldFeeds(const std::vector<Frame>& aOn, const std::vector<Frame>& aOff)
{
    HeldFeedStats h;
    int lastGroundDetail = -1;
    for (size_t i = 0; i < aOn.size() && i < aOff.size(); ++i)
    {
        const Frame& f = aOn[i];
        const bool held = f.mode == 0 && aOff[i].mode != 0 && f.onGround == 1 && f.detail == adapter::kPsmJump;
        if (!held)
        {
            lastGroundDetail = f.detail;
            continue;
        }
        ++h.held;
        h.sprint += lastGroundDetail == adapter::kPsmSprint ? 1 : 0;
        const bool inputs = i > 0 && (f.gait != aOn[i - 1].gait || f.stance != aOn[i - 1].stance || f.strafe != aOn[i - 1].strafe);
        h.jumped += f.jumped ? 1 : 0;
        h.inputs += inputs ? 1 : 0;
        if ((f.jumped || inputs) && h.what.empty())
        {
            char b[320];
            std::snprintf(b, sizeof b, "; first at t=%.3f (previous detail %d gait %d -> %d stance %d -> %d strafe %d -> %d, %s%s:%s)", f.t,
                          lastGroundDetail, i ? aOn[i - 1].gait : -1, f.gait, i ? aOn[i - 1].stance : -1, f.stance, i ? int(aOn[i - 1].strafe) : -1,
                          int(f.strafe), f.jumped ? "jumped " : "", f.db.c_str(), f.clip.c_str());
            h.what = b;
        }
    }
    return h;
}

// ---- jump take-off: one jump-database pick per take-off within 0.1 s, TimeToLand < 1
void CheckTakeoff(Model& m, const Section& s, bool aStanding)
{
    // fixture sanity: the take-off feed = the first jump detail on the ground, recorded `detailed` column 18 = DetailFromFeed
    double t0 = -1;
    for (const Row& r : s.rows)
        if (r.onGround == 1 && adapter::DetailFromFeed(r.psm, r.loco) == adapter::kPsmJump)
        {
            Check(r.detailed == adapter::kPsmJump, "jump " + s.name + ": recorded detail 18 = DetailFromFeed on the take-off feed");
            t0 = r.t;
            break;
        }
    if (t0 < 0)
    {
        Check(false, "jump " + s.name + ": no grounded take-off feed in the fixture");
        return;
    }
    const std::vector<Frame> fr = Replay(m, s.rows, Options{});
    Dump(s.name.c_str(), fr, t0 - 0.02, t0 + 0.12);
    for (const Frame& f : fr) // report: every pick of the flight
        if (g_report && f.jumped && f.t >= t0 - 1e-6 && f.t < t0 + 1.0)
            std::printf("  %s pick t=%.3f mode=%d vz=%.0f ttl=%.2f %s:%s ct=%.3f\n", s.name.c_str(), f.t, f.mode, f.vzCmS, f.ttl, f.db.c_str(),
                        f.clip.c_str(), f.clipTime);
    int picks = 0;
    const Frame* first = nullptr;
    for (const Frame& f : fr)
        if (f.t >= t0 - 1e-6 && f.t < t0 + 0.1 && f.jumped && Has(f.db, "Jumps"))
        {
            ++picks;
            if (!first)
                first = &f;
        }
    Check(picks == 1, "jump " + s.name + ": " + std::to_string(picks) + " jump-database picks within 0.1 s of the take-off feed (want 1)");
    Check(first && first->ttl < 1.0f, "jump " + s.name + ": first take-off pick TimeToLand " + std::to_string(first ? first->ttl : -1.0f) + " (want < 1)");
    if (aStanding)
        Check(first && Has(first->clip, "Jump_F_Start_Stand"),
              "jump " + s.name + ": standing take-off pick " + (first ? first->clip : std::string("none")) + " is a Jump_F_Start_Stand clip");
    else if (first)
        std::printf("jump %s: running take-off pick %s ct %.3f ttl %.2f\n", s.name.c_str(), first->clip.c_str(), first->clipTime, first->ttl);
    // the hold changes the movement mode only on the grounded take-off feed
    Options off;
    off.takeoffHoldS = 0.0f;
    const std::vector<Frame> base = Replay(m, s.rows, off);
    int other = 0, held = 0;
    for (size_t i = 0; i < fr.size() && i < base.size(); ++i)
        if (fr[i].mode != base[i].mode)
        {
            if (fr[i].onGround == 1 && fr[i].detail == adapter::kPsmJump && fr[i].mode == 0)
                ++held;
            else
                ++other;
        }
    Check(fr.size() == base.size() && other == 0 && held == 1,
          "jump " + s.name + ": the hold changes the movement mode on " + std::to_string(held) + " take-off feed(s) and " + std::to_string(other) +
              " other frames (want 1 / 0)");
    const HeldFeedStats h = HeldFeeds(fr, base);
    Check(h.held >= 1 && h.jumped == 0 && h.inputs == 0,
          "jump " + s.name + ": held feeds " + std::to_string(h.held) + ", jumped picks on held feeds " + std::to_string(h.jumped) +
              ", ground inputs changed on held feeds " + std::to_string(h.inputs) + " (want >= 1 / 0 / 0)" + h.what);
}

// ---- the jump fixtures of tests/JumpPickTest.cpp (2026-10-21 slides / jumps, 2026-10-25 running jumps): every take-off the hold acts on
// (grounded Jump 18 feed out of a ground frame, outside the slide context) takes one jump-database pick within 0.1 s, the first with
// TimeToLand < 1; without the hold the same fixtures re-pick (the defect is not specific to the 2026-10-31 session)
int CheckFixtureTakeoffs(Model& m, const char* aPath)
{
    std::vector<Section> secs;
    if (!LoadFixture(aPath, secs))
    {
        Check(false, std::string("jump fixture ") + aPath + " not loaded");
        return 0;
    }
    Options off;
    off.takeoffHoldS = 0.0f;
    Options strafeOn, strafeOff = off; // the host's hipfire pose hold (visualStrafe): the held feed keeps the strafe rotation mode
    strafeOn.strafe = strafeOff.strafe = true;
    int takeoffs = 0, held = 0, doubleOn = 0, doubleOff = 0, longTtlOn = 0, otherModeDiff = 0;
    HeldFeedStats hs[2]; // orient, strafe
    for (const Section& s : secs)
    {
        const std::vector<Frame> on = Replay(m, s.rows, Options{});
        const std::vector<Frame> base = Replay(m, s.rows, off);
        if (on.size() != base.size())
        {
            ++otherModeDiff;
            continue;
        }
        for (int k = 0; k < 2; ++k)
        {
            const HeldFeedStats h = k == 0 ? HeldFeeds(on, base) : HeldFeeds(Replay(m, s.rows, strafeOn), Replay(m, s.rows, strafeOff));
            hs[k].held += h.held;
            hs[k].sprint += h.sprint;
            hs[k].jumped += h.jumped;
            hs[k].inputs += h.inputs;
            if (hs[k].what.empty() && !h.what.empty())
                hs[k].what = " " + s.name + h.what;
        }
        auto picks = [](const std::vector<Frame>& aFr, size_t aFrom, float& aTtl, std::string& aClips) {
            int n = 0;
            aTtl = -1.0f;
            aClips.clear();
            for (size_t k = aFrom; k < aFr.size() && aFr[k].t < aFr[aFrom].t + 0.1 - 1e-6; ++k)
                if (aFr[k].jumped && Has(aFr[k].db, "Jumps"))
                {
                    aTtl = n == 0 ? aFr[k].ttl : aTtl;
                    char b[96];
                    std::snprintf(b, sizeof b, "%s%s@%.2f", n ? " " : "", aFr[k].clip.c_str(), aFr[k].clipTime);
                    aClips += b;
                    ++n;
                }
            return n;
        };
        for (size_t i = 0; i < on.size(); ++i)
        {
            const bool takeoffFeed = on[i].onGround == 1 && on[i].detail == adapter::kPsmJump &&
                                     (i == 0 || !(on[i - 1].onGround == 1 && on[i - 1].detail == adapter::kPsmJump));
            const bool diff = on[i].mode != base[i].mode;
            if (diff && !(takeoffFeed && on[i].mode == 0))
                ++otherModeDiff;
            if (!takeoffFeed)
                continue;
            ++takeoffs;
            if (!diff)
                continue; // slide context / no launch: the hold does not act, unchanged
            ++held;
            float ttlOn = 0, ttlOff = 0;
            std::string clipsOn, clipsOff;
            const int nOn = picks(on, i, ttlOn, clipsOn), nOff = picks(base, i, ttlOff, clipsOff);
            doubleOn += nOn > 1 ? 1 : 0;
            doubleOff += nOff > 1 ? 1 : 0;
            longTtlOn += nOn > 0 && !(ttlOn < 1.0f) ? 1 : 0;
            if (g_report)
                std::printf("  fixture %s take-off t=%.3f: picks %d (ttl %.2f: %s) without the hold %d (ttl %.2f: %s)\n", s.name.c_str(), on[i].t,
                            nOn, ttlOn, clipsOn.c_str(), nOff, ttlOff, clipsOff.c_str());
        }
    }
    std::printf("jump fixture %s: %d grounded take-off feeds, %d held; double picks within 0.1 s %d -> %d, first pick TimeToLand >= 1: %d\n", aPath,
                takeoffs, held, doubleOff, doubleOn, longTtlOn);
    Check(held >= 1 && doubleOn == 0 && longTtlOn == 0 && otherModeDiff == 0,
          std::string("jump fixture ") + aPath + ": every held take-off picks once with TimeToLand < 1, the hold changes no other frame (held " +
              std::to_string(held) + ", double " + std::to_string(doubleOn) + ", ttl >= 1: " + std::to_string(longTtlOn) + ", other frames " +
              std::to_string(otherModeDiff) + ")");
    for (int k = 0; k < 2; ++k)
        Check(hs[k].held >= (k ? 1 : held) && hs[k].held >= 1 && hs[k].jumped == 0 && hs[k].inputs == 0,
              std::string("jump fixture ") + aPath + (k ? " strafe" : " orient") + ": held feeds " + std::to_string(hs[k].held) + " (sprint " +
                  std::to_string(hs[k].sprint) + "), jumped picks on held feeds " + std::to_string(hs[k].jumped) +
                  ", ground inputs changed on held feeds " + std::to_string(hs[k].inputs) + " (want >= " + std::to_string(k ? 1 : std::max(held, 1)) + " / 0 / 0)" +
                  hs[k].what);
    return hs[0].sprint;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::printf("usage: utpp_mm_locostart_test <loco_start_feeds.txt> <bundle stem> [--report]\n");
        return 2;
    }
    std::vector<const char*> jumpFixtures;
    for (int i = 3; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--report") == 0)
            g_report = true;
        else if (std::strcmp(argv[i], "--jumps") == 0 && i + 1 < argc)
            jumpFixtures.push_back(argv[++i]);
    }
    std::vector<Section> sections;
    if (!LoadFixture(argv[1], sections))
    {
        std::printf("SETUP fixture %s not loaded\n", argv[1]);
        return 2;
    }
    Model m;
    std::string err;
    if (!m.Load(argv[2], err))
    {
        std::printf("SETUP bundle %s: %s\n", argv[2], err.c_str());
        return 2;
    }

    // ---- recorded starts from rest
    struct RecStart { const char* section; bool strafe; const char* want; };
    const RecStart rec[] = {
        {"crouch_start", false, "Crouch_Walk_Starts"},
        {"hip_start", true, "Crouch_Walk_Starts"},
        {"gwalk_start_before", false, "Stand_Walk_Starts"},
        {"gwalk_start_after", false, "Stand_Walk_Starts"},
    };
    for (const RecStart& c : rec)
    {
        const Section* s = Find(sections, c.section);
        if (!s)
        {
            Check(false, std::string("fixture section ") + c.section);
            continue;
        }
        Options o;
        o.strafe = c.strafe;
        const std::vector<Frame> fr = Replay(m, s->rows, o);
        CheckStart(std::string("replay ") + c.section, fr, FirstStick(s->rows), c.want);
        CheckPredicateIdentity(c.section, fr);
    }

    // ---- synthetic starts from rest at several world yaws, orient / strafe
    struct SynStart { const char* name; float stick, speed, acc; bool crouch, walk; const char* want; };
    const SynStart syn[] = {
        {"stand analog walk 0.45 (floor 0.82)", 0.4545f, 0.82f, 10.0f, false, false, "Stand_Walk_Starts"},
        {"stand walk toggle 0.45", 0.4545f, 0.82f, 10.0f, false, true, "Stand_Walk_Starts"},
        {"crouch walk 0.18", 0.18f, 0.58f, 12.0f, true, false, "Crouch_Walk_Starts"},
        {"crouch walk 0.45", 0.4545f, 0.58f, 12.0f, true, false, "Crouch_Walk_Starts"},
        {"stand run 1.0", 1.0f, 3.84f, 8.0f, false, false, "Stand_Run_Starts"},
    };
    for (const int hz : {30, 60, 144})
        for (const SynStart& c : syn)
            for (const bool strafe : {false, true})
                for (const float yaw : {0.0f, 90.0f, 180.0f, -155.0f})
                {
                    const std::vector<Row> rows = Synthetic(yaw, c.stick, c.speed, c.acc, c.crouch, c.walk, 0.8, 1.2, 1.0f / float(hz));
                    Options o;
                    o.strafe = strafe;
                    const std::vector<Frame> fr = Replay(m, rows, o);
                    char name[160];
                    std::snprintf(name, sizeof name, "%s %s yaw %.0f %d Hz", c.name, strafe ? "strafe" : "orient", yaw, hz);
                    CheckStart(name, fr, FirstStick(rows), c.want);
                    CheckPredicateIdentity(name, fr);
                }
    // starts with a turn from rest (the stick 90 / 180 deg off the body, orient): a turn start of the *_Starts databases, no pivot pose
    for (const SynStart& c : {syn[0], syn[3]})
        for (const float off : {90.0f, -90.0f, 180.0f})
            for (const float yaw : {0.0f, -155.0f})
            {
                const std::vector<Row> rows = Synthetic(yaw, c.stick, c.speed, c.acc, c.crouch, c.walk, 0.8, 1.2, 1.0f / 60.0f, off);
                const std::vector<Frame> fr = Replay(m, rows, Options{});
                char name[160];
                std::snprintf(name, sizeof name, "%s stick %.0f deg off the body yaw %.0f", c.name, off, yaw);
                CheckStart(name, fr, FirstStick(rows), c.want);
                CheckPredicateIdentity(name, fr);
            }

    // ---- real pivots still flag IsPivoting and pick a *_Pivots row (orient)
    struct SynTurn { const char* name; float stick, speed, acc, turn; bool walk; };
    const SynTurn turns[] = {
        {"run reversal", 1.0f, 3.84f, 10.0f, 180.0f, false},
        {"walk toggle reversal", 1.0f, 1.6f, 8.0f, 180.0f, true},
        {"walk toggle 90 turn", 1.0f, 1.6f, 8.0f, 90.0f, true},
    };
    for (const SynTurn& c : turns)
        for (const float yaw : {0.0f, -155.0f})
        {
            const std::vector<Row> rows = SyntheticTurn(yaw, c.stick, c.speed, c.acc, c.turn, c.walk, 1.5, 1.0, 1.0f / 60.0f);
            const std::vector<Frame> fr = Replay(m, rows, Options{});
            int pv = 0, pvPick = 0;
            for (const Frame& f : fr)
                if (f.t >= 1.5 - 1e-6 && f.t < 1.8)
                {
                    pv += f.pivoting ? 1 : 0;
                    pvPick += f.jumped && Has(f.db, "Pivots") ? 1 : 0;
                }
            char name[160];
            std::snprintf(name, sizeof name, "pivot %s yaw %.0f: IsPivoting frames %d, *_Pivots picks %d (want > 0 / > 0)", c.name, yaw, pv, pvPick);
            Dump(c.name, fr, 1.48, 1.62);
            Check(pv > 0 && pvPick > 0, name);
            CheckPredicateIdentity(c.name, fr);
        }

    // ---- jump take-off (JS1)
    if (const Section* s = Find(sections, "jump_stand"))
        CheckTakeoff(m, *s, true);
    else
        Check(false, "fixture section jump_stand");
    if (const Section* s = Find(sections, "jump_run"))
        CheckTakeoff(m, *s, false);
    else
        Check(false, "fixture section jump_run");
    int sprintTakeoffs = 0;
    for (const char* p : jumpFixtures)
        sprintTakeoffs += CheckFixtureTakeoffs(m, p);
    // sprint (PSM 4) -> Jump 18 take-offs held in the --jumps fixtures (2026-10-25 running jumps: 5, t 58.854 / 60.038 / 61.171 among them)
    Check(jumpFixtures.empty() || sprintTakeoffs >= 5,
          "jump fixtures: " + std::to_string(sprintTakeoffs) + " held sprint (4) -> 18 take-off feeds (want >= 5)");

    std::printf("%s: %d / %d checks failed\n", g_fail ? "FAILED" : "OK", g_fail, g_checks);
    return g_fail ? 1 : 0;
}
