// UTPP B1 strafe S1 core test: StrafeMode timing / gates, StrafeDesiredMotion, ComputeStrafeProfiles on a database.
// usage: utpp_mm_strafe_test <v6.utppmm> [<dense_subset.utppmm>]
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "mm/Strafe.hpp"
#include "mm/BaseMatcher.hpp"

using namespace utpp::mm;
using namespace utpp::mm::strafe;

static int g_fail = 0;
static int g_pass = 0;
static bool g_dump = false; // 3rd argument "dump": per-clip ROOT yaw / loop bins
static bool Near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

static void Expect(const char* aWhat, bool aOk, const std::string& aDetail = {})
{
    if (aOk)
        ++g_pass;
    else
        ++g_fail;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat, aDetail.empty() ? "" : " - ", aDetail.c_str());
}

static uint64_t FileRigHash(const char* aPath)
{
    std::ifstream f(aPath, std::ios::binary);
    char h[32] = {};
    f.read(h, sizeof(h));
    uint64_t r = 0;
    std::memcpy(&r, h + 24, 8);
    return r;
}

static uint8_t DriverClass(std::string_view n)
{
    const uint8_t k = steer::DriverClipClass(n, steer::ClassifyClip(n), true);
    return (k == steer::kClassStanceToCrouch && steer::IsMovingToCrouchName(n)) ? uint8_t(steer::kClassStanceMoveToCrouch) : k;
}

static void TestMode()
{
    const StrafeConfig cfg;
    const float dt = 1.0f / 60.0f;
    StrafeMode m;
    Expect("mode starts Off", m.State() == StrafeState::Off);
    m.Update(true, dt, cfg);
    Expect("enter on first wanted frame", m.State() == StrafeState::Active && m.Entered() && m.Entries() == 1);
    m.Update(true, dt, cfg);
    Expect("Entered is one-shot", !m.Entered());
    m.Update(false, dt, cfg);
    Expect("not wanted -> Exiting", m.State() == StrafeState::Exiting && !m.Exited());
    m.Update(true, dt, cfg);
    Expect("1-frame blip does not exit", m.State() == StrafeState::Active && m.Exits() == 0 && m.Blips() == 1 && !m.Entered());
    int frames = 0;
    while (m.State() != StrafeState::Off && frames < 100)
    {
        m.Update(false, dt, cfg);
        ++frames;
    }
    // exitHold 0.15 s at 60 Hz: Exiting on frame 1 (t = 0), Off once 0.15 s accumulated (frames 2..10 -> 9 * dt = 0.15)
    Expect("exit after exitHold", m.State() == StrafeState::Off && m.Exited() && m.Exits() == 1 && frames >= 9 && frames <= 11,
           "frames=" + std::to_string(frames));
    m.Update(false, dt, cfg);
    Expect("Exited is one-shot", !m.Exited());
    m.Update(true, std::numeric_limits<float>::quiet_NaN(), cfg);
    m.Update(false, std::numeric_limits<float>::quiet_NaN(), cfg);
    m.Update(false, std::numeric_limits<float>::infinity(), cfg);
    Expect("NaN / inf dt never advance the hold", m.State() == StrafeState::Exiting);
    {
        StrafeConfig bad;
        bad.exitHold = std::numeric_limits<float>::quiet_NaN();
        StrafeMode n;
        n.Update(true, 1.0f / 60.0f, bad);
        int nf = 0;
        while (n.State() != StrafeState::Off && nf < 120)
        {
            n.Update(false, 1.0f / 60.0f, bad);
            ++nf;
        }
        Expect("NaN exitHold falls back to 0.15 s (no stuck Exiting)", n.State() == StrafeState::Off && n.Exited() && nf >= 9 && nf <= 11,
               std::to_string(nf));
    }

    // Base turn-class list, checked against an explicit literal list (not against IsStrafeTurnBaseClass itself).
    {
        constexpr uint32_t want = (1u << steer::kClassMoveTurn) | (1u << steer::kClassReface) | (1u << steer::kClassTurnInPlace) |
                                  (1u << steer::kClassTurnWalk) | (1u << steer::kClassTurnRun) | (1u << steer::kClassTurnSprint) |
                                  (1u << steer::kClassArc) | (1u << steer::kClassRefaceWalk) | (1u << steer::kClassRefaceRun) |
                                  (1u << steer::kClassRefaceSprint) | (1u << steer::kClassTurnAgainst) | (1u << steer::kClassSlowTurnStart);
        uint32_t got = 0;
        for (uint8_t c = 0; c < 32; ++c)
            got |= IsStrafeTurnBaseClass(c) ? (1u << c) : 0u;
        Expect("base turn classes = MoveTurn/Turn*/Reface*/TIP/Arc/TurnAgainst/SlowTurnStart; pivots excluded", got == want,
               std::to_string(got) + " vs " + std::to_string(want));
    }

    // Map_theta (design step 4): game point -> DB loop speed in that direction, linear through 0.
    {
        StrafeSpeedTable t;
        const float w[5] = {2.19f, 2.19f, 1.97f, 1.64f, 1.64f};
        for (int b = 0; b < 5; ++b)
            t.speed[0][0][b] = w[b];
        Expect("MapSpeed: game point -> S(90)", Near(t.MapSpeed(0, 0, 90.0f * kDeg, 1.65f, 1.65f), 1.97f, 1e-4f));
        Expect("MapSpeed: half speed -> half S(180)", Near(t.MapSpeed(0, 0, kPi, 0.825f, 1.65f), 0.82f, 1e-4f));
        Expect("MapSpeed: empty table / bad point / NaN -> passthrough or finite",
               Near(t.MapSpeed(1, 1, 0.0f, 1.2f, 1.65f), 1.2f, 1e-6f) && Near(t.MapSpeed(0, 0, 0.0f, 1.2f, 0.0f), 1.2f, 1e-6f) &&
                   Near(t.MapSpeed(0, 0, 0.0f, std::numeric_limits<float>::quiet_NaN(), 1.65f), 0.0f, 1e-6f) &&
                   std::isfinite(t.MapSpeed(0, 0, std::numeric_limits<float>::quiet_NaN(), 1.0f, std::numeric_limits<float>::infinity())));
    }

    // Wanted gates
    const bool avail[2] = {true, true};
    StrafeInput in;
    in.strafeOn = in.tpp = in.upperMaskOn = in.upperMaskOk = true;
    in.upperBody = 6;
    in.maskStates = kMaskStateAim;
    Expect("wanted: aim + weapon", Wanted(in, cfg, avail));
    StrafeInput t = in;
    t.upperBody = 0;
    Expect("not wanted: upperBody != 6", !Wanted(t, cfg, avail));
    // literal CET UTPPSetMMUpperMaskStates values (not the symbolic constant): 7 = weapon in hand, 5 = empty hands
    t = in;
    t.maskStates = 7;
    Expect("wanted: CET mask states 7 (weapon in hand)", Wanted(t, cfg, avail));
    t.maskStates = 2;
    Expect("wanted: mask states bit 1 only", Wanted(t, cfg, avail));
    t.maskStates = 5;
    Expect("not wanted: CET mask states 5 (empty hands)", !Wanted(t, cfg, avail));
    t.maskStates = 0xFFFFFFFDu;
    Expect("not wanted: every mask state bit except bit 1", !Wanted(t, cfg, avail));
    t = in;
    t.maskStates = 0;
    Expect("not wanted: no weapon", !Wanted(t, cfg, avail));
    StrafeConfig un = cfg;
    un.unarmed = true;
    Expect("wanted: unarmed switch", Wanted(t, un, avail));
    t = in;
    t.vz = 1.0f;
    Expect("not wanted: |vz| >= 1", !Wanted(t, cfg, avail));
    t.vz = -0.99f;
    Expect("wanted: |vz| 0.99", Wanted(t, cfg, avail));
    t.vz = std::numeric_limits<float>::quiet_NaN();
    Expect("not wanted: vz NaN", !Wanted(t, cfg, avail));
    bool detailOk = true;
    for (int d : {6, 14, 18, 23, 28})
    {
        t = in;
        t.psmDetail = d;
        detailOk = detailOk && !Wanted(t, cfg, avail);
    }
    for (int d : {0, 5, 7, 13, 29})
    {
        t = in;
        t.psmDetail = d;
        detailOk = detailOk && Wanted(t, cfg, avail);
    }
    Expect("detail gate {6, 14..28}", detailOk);
    t = in;
    t.tpp = false;
    Expect("not wanted: FPP", !Wanted(t, cfg, avail));
    t = in;
    t.upperMaskOk = false;
    Expect("not wanted: upper mask not ok", !Wanted(t, cfg, avail));
    t = in;
    t.stance = kStanceCrouch;
    const bool noCrouch[2] = {true, false};
    Expect("not wanted: stance not available", !Wanted(t, cfg, noCrouch) && Wanted(t, cfg, avail));
    Expect("slow-walk gate bypass while not Off",
           StrafeBypassesSlowWalkGate(StrafeState::Active) && StrafeBypassesSlowWalkGate(StrafeState::Exiting) &&
               !StrafeBypassesSlowWalkGate(StrafeState::Off));
}

static void TestDesired()
{
    bt::MotionParams p;
    p.intentFraction = 1.0f;
    p.stateTargetSpeed = 1.65f;
    p.stopIntent = true;
    float worstDeg = 0.0f, worstSpeed = 0.0f;
    bool facingOk = true;
    for (int a = 0; a <= 360; a += 15)
    {
        bt::MotionFrame f;
        const float th = float(a) * kDeg; // CCW from forward
        f.move[0] = -std::sin(th);
        f.move[1] = std::cos(th);
        f.charVel[0] = 0.3f; // capsule lagging in some other direction
        f.charVel[1] = 0.2f;
        f.bodyYaw = 1.0f;
        f.cameraYaw = -2.0f;
        const bt::DesiredMotion d = StrafeDesiredMotion(f, p, StrafeState::Active);
        const float got = std::atan2(-d.vel[0], d.vel[1]);
        worstDeg = std::fmax(worstDeg, std::fabs(bt::Wrap(got - th)) / kDeg);
        worstSpeed = std::fmax(worstSpeed, std::fabs(std::hypot(d.vel[0], d.vel[1]) - 1.65f));
        facingOk = facingOk && d.facing[0] == 0.0f && d.facing[1] == 1.0f && d.direction && d.turnRate == 0.0f;
    }
    Expect("desired: velocity angle == stick angle (0..360 step 15)", worstDeg < 0.1f, "worst " + std::to_string(worstDeg) + " deg");
    Expect("desired: speed = max(capsule, intent x |stick|)", worstSpeed < 1e-4f);
    Expect("desired: facing (0,1), direction, turnRate 0", facingOk);
    {
        bt::MotionFrame f;
        f.move[1] = 0.5f;
        f.charVel[1] = 1.2f;
        const bt::DesiredMotion d = StrafeDesiredMotion(f, p, StrafeState::Active);
        Expect("desired: capsule faster than intent wins", std::fabs(d.vel[1] - 1.2f) < 1e-5f && d.vel[0] == 0.0f);
    }
    {
        bt::MotionFrame f;
        f.move[0] = 1.0f; // stick right
        f.charVel[0] = 0.0f;
        f.charVel[1] = -1.0f; // capsule still moving backwards
        const bt::DesiredMotion d = StrafeDesiredMotion(f, p, StrafeState::Exiting);
        Expect("desired: Exiting uses the capsule direction", std::fabs(d.vel[0]) < 1e-5f && d.vel[1] < -1.6f && d.facing[1] == 1.0f);
    }
    {
        bt::MotionFrame f;
        f.move[0] = 0.1f;
        f.charVel[0] = 0.7f;
        const bt::DesiredMotion d = StrafeDesiredMotion(f, p, StrafeState::Active);
        Expect("desired: stopIntent with released stick -> 0", d.vel[0] == 0.0f && d.vel[1] == 0.0f);
        bt::MotionParams q = p;
        q.stopIntent = false;
        const bt::DesiredMotion e = StrafeDesiredMotion(f, q, StrafeState::Active);
        Expect("desired: no stopIntent -> capsule velocity", e.vel[0] == 0.7f && e.vel[1] == 0.0f);
    }
    {
        const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
        bool fin = true;
        for (int s = 0; s < 3; ++s)
        {
            bt::MotionFrame f;
            f.move[0] = nan;
            f.move[1] = inf;
            f.charVel[0] = inf;
            f.charVel[1] = nan;
            bt::MotionParams q = p;
            q.stateTargetSpeed = nan;
            const bt::DesiredMotion d = StrafeDesiredMotion(f, q, StrafeState(s));
            fin = fin && std::isfinite(d.vel[0]) && std::isfinite(d.vel[1]) && std::isfinite(d.facing[0]) && std::isfinite(d.facing[1]);
        }
        Expect("desired: NaN / inf inputs -> finite", fin);
    }
    // rate law
    Expect("rate: own speed <= 0.3 -> n/a", LoopOwnSpeedRate(1.0f, 0.3f, 0.5f, 1.5f) < 0.0f && LoopOwnSpeedRate(1.0f, std::nanf(""), 0.5f, 1.5f) < 0.0f);
    Expect("rate: capsule / own", std::fabs(LoopOwnSpeedRate(1.65f, 1.97f, 0.5f, 1.5f) - 1.65f / 1.97f) < 1e-6f);
    Expect("rate: clamped", LoopOwnSpeedRate(5.0f, 1.0f, 0.5f, 1.5f) == 1.5f && LoopOwnSpeedRate(0.0f, 1.0f, 0.5f, 1.5f) == 0.5f);
    StrafeRateSmoother sm;
    sm.Update(1.0f, 1.0f / 60.0f);
    float r = 0.0f;
    for (int i = 0; i < 6; ++i) // 0.1 s
        r = sm.Update(0.5f, 1.0f / 60.0f);
    Expect("rate smoother: halflife 0.1 s", std::fabs(r - 0.75f) < 1e-3f, std::to_string(r));
    Expect("rate smoother: NaN ignored", sm.Update(std::nanf(""), 0.016f) == r);
    Expect("DirBin / DirSector", DirBin(0.0f) == 0 && DirBin(44.0f * kDeg) == 1 && DirBin(-91.0f * kDeg) == 2 && DirBin(kPi) == 4 &&
                                     DirSector(-kPi / 2) == 6 && DirSector(kPi / 4) == 1 && DirSector(-0.1f) == 0);
}

struct DbResult
{
    StrafeProfiles prof, profNoRoot;
    std::vector<uint8_t> classes;
};

static bool LoadAndProfile(const char* aPath, Database& db, DbResult& r)
{
    std::string err;
    if (!db.LoadFromFile(aPath, FileRigHash(aPath), err))
    {
        Expect("load database", false, err);
        return false;
    }
    const uint64_t excl = BaseMatcher::TagMaskContaining(db, "_Lands_") | BaseMatcher::TagMaskContaining(db, "FromTraversal");
    const uint64_t stand = BaseMatcher::TagMaskContaining(db, "Stand_") & ~excl;
    const uint64_t crouch = BaseMatcher::TagMaskContaining(db, "Crouch_") & ~excl;
    r.classes.resize(db.NumClips());
    for (uint32_t c = 0; c < db.NumClips(); ++c)
        r.classes[c] = DriverClass(db.ClipName(c));
    ComputeStrafeProfiles(db, r.classes, stand, crouch, StrafeConfig{}, r.prof);
    // Same masks with the ROOT rule disabled: only the base-class list can make a clip StrafeTurn.
    StrafeConfig noRoot;
    noRoot.rootTurnMaxDeg = 1.0e9f;
    ComputeStrafeProfiles(db, r.classes, stand, crouch, noRoot, r.profNoRoot);
    return true;
}

// Class-rule checks independent of IsStrafeTurnBaseClass: literal turn names, pivots by name + class.
struct ClassRuleCounts
{
    uint32_t pivots = 0, pivotsKept = 0, turns = 0, turnsStrafe = 0, nonTurnChanged = 0;
};
static ClassRuleCounts CheckClassRule(const Database& db, const DbResult& r)
{
    ClassRuleCounts n;
    for (uint32_t c = 0; c < db.NumClips(); ++c)
    {
        const std::string_view nm = db.ClipName(c);
        const uint8_t k = r.classes[c];
        const bool pivotName = nm.find("_Pivot_") != std::string_view::npos && steer::IsPivot(k);
        if (pivotName && r.prof.maxCumYawDeg[c] < 0.05f)
        {
            ++n.pivots;
            // facing-fixed pivots keep their pivot class in both sets, with and without the ROOT rule
            if (r.prof.classes[0][c] == k && r.prof.classes[1][c] == k && r.profNoRoot.classes[0][c] == k && r.profNoRoot.classes[1][c] == k)
                ++n.pivotsKept;
        }
        const bool turnClass = k == steer::kClassMoveTurn || k == steer::kClassTurnWalk || k == steer::kClassTurnRun ||
                               k == steer::kClassTurnSprint || k == steer::kClassReface || k == steer::kClassRefaceWalk ||
                               k == steer::kClassRefaceRun || k == steer::kClassRefaceSprint || k == steer::kClassTurnInPlace ||
                               k == steer::kClassArc || k == steer::kClassSlowTurnStart || k == steer::kClassTurnAgainst;
        if (turnClass)
        {
            ++n.turns;
            if (r.profNoRoot.classes[0][c] == kClassStrafeTurn && r.profNoRoot.classes[1][c] == kClassStrafeTurn)
                ++n.turnsStrafe;
        }
        else if (k != steer::kClassPattern && (r.profNoRoot.classes[0][c] != k || r.profNoRoot.classes[1][c] != k))
            ++n.nonTurnChanged; // ROOT rule off: a non-turn, non-pattern clip must keep its class
    }
    return n;
}

static void PrintTable(const StrafeProfiles& p)
{
    const char* sn[2] = {"stand", "crouch"};
    const char* gn[2] = {"walk", "run"};
    for (int s = 0; s < 2; ++s)
        for (int g = 0; g < 2; ++g)
        {
            std::printf("  table %s %s:", sn[s], gn[g]);
            for (int b = 0; b < kNumBins; ++b)
                std::printf(" %.2f(%u)", double(p.table.speed[s][g][b]), unsigned(p.table.count[s][g][b]));
            std::printf("  sectors=0x%02X (%d) avail=%d\n", p.sectors[s][g], SectorCount(p.sectors[s][g]), int(p.availGait[s][g]));
        }
    std::printf("  facingFixedLoops=%u rootTurners=%u turnClassClips=%u patternsKept walk/run=%u/%u offBand=%u/%u\n", p.facingFixedLoops,
                p.rootTurners, p.turnClassClips, p.patternsKept[0], p.patternsKept[1], p.patternsOffBand[0], p.patternsOffBand[1]);
}


static void TestV6(const char* aPath)
{
    Database db;
    DbResult r;
    if (!LoadAndProfile(aPath, db, r))
        return;
    const StrafeProfiles& p = r.prof;
    PrintTable(p);
    uint32_t patterns0 = 0, pivots0 = 0, motionTurnMin = 0, motionTurns = 0;
    float motionTurnMinDeg = 1e9f;
    bool turnsAreStrafeTurn = true, patternsWalkBand = true;
    for (uint32_t c = 0; c < db.NumClips(); ++c)
    {
        const std::string_view n = db.ClipName(c);
        const float d = p.maxCumYawDeg[c];
        const uint8_t k = r.classes[c];
        if (k == steer::kClassPattern && d < 0.05f)
            ++patterns0;
        if (n.find("_Pivot_") != std::string_view::npos && d < 0.05f)
            ++pivots0;
        const bool motionTurn = n.rfind("M_Neutral", 0) == 0 &&
                              (IsStrafeTurnBaseClass(k) && n.find("_Pivot_") == std::string_view::npos && k != steer::kClassArc);
        if (g_dump && (motionTurn || n.find("SideStep") != std::string_view::npos || p.loopBin[c] >= 0))
            std::printf("  dump %-70.*s k=%2u yaw=%6.1f bin=%d own=%.2f\n", int(n.size()), n.data(), unsigned(k), double(d), int(p.loopBin[c]),
                        double(p.loopOwnSpeed[c]));
        if (motionTurn)
        {
            ++motionTurns;
            motionTurnMinDeg = std::fmin(motionTurnMinDeg, d);
            motionTurnMin += d >= 44.9f ? 1u : 0u; // motion has 045 turns too (design said >= 90)
        }
        if (IsStrafeTurnBaseClass(k))
            turnsAreStrafeTurn = turnsAreStrafeTurn && p.classes[0][c] == kClassStrafeTurn && p.classes[1][c] == kClassStrafeTurn;
        if (n.find("SideStep") != std::string_view::npos)
            turnsAreStrafeTurn = turnsAreStrafeTurn && p.classes[0][c] == kClassStrafeTurn;
        if (k == steer::kClassPattern && d < 10.0f)
        {
            const int sc = steer::SpeedClassOfName(n);
            patternsWalkBand = patternsWalkBand && ((p.classes[0][c] == k) == (sc == 0 || sc == 1)) &&
                               ((p.classes[1][c] == k) == (sc == 0 || sc == 2));
        }
    }
    std::printf("  v6: patterns@0deg=%u pivots@0deg=%u motionTurns=%u (>=44.9 deg: %u, min %.1f)\n",
                patterns0, pivots0, motionTurns, motionTurnMin, double(motionTurnMinDeg));
    Expect("v6: Neutral directional facing-fixed loops (12 walk + 12 run + 10 crouch)", p.facingFixedLoops == 34, std::to_string(p.facingFixedLoops));
    Expect("v6: 60 facing-fixed Pivot_X_Y", pivots0 == 60, std::to_string(pivots0));
    Expect("v6: ~307 patterns at 0.0 deg", patterns0 >= 300 && patterns0 <= 310, std::to_string(patterns0));
    Expect("v6: every Neutral turning clip >= 45 deg", motionTurns > 0 && motionTurnMin == motionTurns, std::to_string(motionTurnMinDeg));
    Expect("v6: turn classes -> StrafeTurn in both sets", turnsAreStrafeTurn);
    {
        const ClassRuleCounts cr = CheckClassRule(db, r);
        std::printf("  v6 class rule: pivots@0=%u kept=%u turnClass=%u strafeTurn(ROOT off)=%u nonTurnChanged=%u\n", cr.pivots, cr.pivotsKept,
                    cr.turns, cr.turnsStrafe, cr.nonTurnChanged);
        Expect("v6: 60 facing-fixed pivots keep their pivot class in both strafe sets", cr.pivots == 60 && cr.pivotsKept == 60,
               std::to_string(cr.pivotsKept) + "/" + std::to_string(cr.pivots));
        Expect("v6: ROOT rule off -> every turn/reface/TIP/Arc/SlowTurnStart/TurnAgainst clip still StrafeTurn",
               cr.turns > 0 && cr.turnsStrafe == cr.turns, std::to_string(cr.turnsStrafe) + "/" + std::to_string(cr.turns));
        Expect("v6: ROOT rule off -> non-turn, non-pattern clips keep their class", cr.nonTurnChanged == 0, std::to_string(cr.nonTurnChanged));
    }
    Expect("v6: pattern band filter (walk set: Walk, jog set: Run)", patternsWalkBand && p.patternsOffBand[0] > 0 && p.patternsOffBand[1] > 0);
    const float walk[5] = {2.19f, 2.19f, 1.97f, 1.64f, 1.64f}, run[5] = {5.46f, 5.46f, 3.83f, 3.28f, 3.28f},
                cr[5] = {2.46f, 2.46f, 2.19f, 1.97f, 1.97f};
    bool tw = true, tr = true, tc = true;
    for (int b = 0; b < 5; ++b)
    {
        tw = tw && Near(p.table.speed[kStanceStand][kGaitWalk][b], walk[b], 0.06f);
        tr = tr && Near(p.table.speed[kStanceStand][kGaitRun][b], run[b], 0.1f);
        tc = tc && Near(p.table.speed[kStanceCrouch][kGaitWalk][b], cr[b], 0.06f);
    }
    Expect("v6: walk speed table 2.19/2.19/1.97/1.64/1.64", tw);
    Expect("v6: run speed table 5.46/5.46/3.83/3.28/3.28", tr);
    Expect("v6: crouch speed table 2.46/2.46/2.19/1.97/1.97", tc);
    Expect("v6: avail stand + crouch", p.avail[kStanceStand] && p.avail[kStanceCrouch]);
    Expect("v6: speed interpolation 22.5 / 67.5 deg", Near(p.table.Speed(0, 0, 67.5f * kDeg), 0.5f * (walk[1] + walk[2]), 0.06f));
    // determinism: a second call gives identical tables
    StrafeProfiles again;
    ComputeStrafeProfiles(db, r.classes, BaseMatcher::TagMaskContaining(db, "Stand_") & ~(BaseMatcher::TagMaskContaining(db, "_Lands_") | BaseMatcher::TagMaskContaining(db, "FromTraversal")),
                          BaseMatcher::TagMaskContaining(db, "Crouch_") & ~(BaseMatcher::TagMaskContaining(db, "_Lands_") | BaseMatcher::TagMaskContaining(db, "FromTraversal")),
                          StrafeConfig{}, again);
    Expect("v6: recompute is deterministic", std::memcmp(&again.table, &p.table, sizeof(p.table)) == 0 && again.classes[0] == p.classes[0] &&
                                                 again.classes[1] == p.classes[1]);
    // S4 jog: the jog profile follows the measured run loop speeds (S(theta) / S(0) within 0.02 in every bin), so every run loop
    // plays at one rate F / S(0) at full stick
    {
        bool ok = p.table.speed[kStanceStand][kGaitRun][0] > 0.0f;
        std::string d;
        for (int b = 0; b < kNumBins && ok; ++b)
        {
            const float ratio = p.table.speed[kStanceStand][kGaitRun][b] / p.table.speed[kStanceStand][kGaitRun][0];
            ok = std::fabs(ratio - JogProfile(45.0f * kDeg * float(b))) <= 0.02f;
            d += std::to_string(ratio) + " ";
        }
        Expect("v6: S4 jog profile = run loop speed ratios (<= 0.02 per bin)", ok, d);
        // the minimum jog speed keeps every run loop at or above the 0.6 rate floor (rate = F / S_run(0) at full stick)
        const float s0 = p.table.speed[kStanceStand][kGaitRun][0];
        Expect("v6: S4 kJogMinF / S_run(0) >= 0.6 (no run loop played faster than the capsule at the minimum jog speed)",
               s0 > 0.0f && kJogMinF / s0 >= 0.6f, std::to_string(kJogMinF / s0));
    }
}

// S4 jog while aiming: profile and target speed helpers
static void TestJog()
{
    const float bins[5] = {1.0f, 1.0f, 0.70f, 0.60f, 0.60f};
    bool ok = true;
    for (int b = 0; b < 5; ++b)
        ok = ok && Near(JogProfile(45.0f * kDeg * float(b)), bins[b], 1e-5f) && Near(JogProfile(-45.0f * kDeg * float(b)), bins[b], 1e-5f);
    Expect("jog: profile 1 / 1 / 0.70 / 0.60 / 0.60 at 0 / 45 / 90 / 135 / 180 deg, symmetric", ok);
    Expect("jog: profile linear between the bin centres (67.5 deg 0.85, 112.5 deg 0.65), clamped beyond 180 deg",
           Near(JogProfile(67.5f * kDeg), 0.85f, 1e-5f) && Near(JogProfile(112.5f * kDeg), 0.65f, 1e-5f) && Near(JogProfile(4.0f), 0.60f, 1e-5f));
    bool mono = true;
    for (int d = 1; d <= 180; ++d)
        mono = mono && JogProfile(float(d) * kDeg) <= JogProfile(float(d - 1) * kDeg) + 1e-6f;
    Expect("jog: profile non-increasing in |theta|", mono);
    // body-relative stick: x right (MoveX), y forward
    Expect("jog: target forward 3.0, left / right 2.10, back 1.80, back-left diagonal 1.80, forward-right diagonal 3.0",
           Near(JogTargetSpeed(3.0f, 0.0f, 1.0f), 3.0f, 1e-5f) && Near(JogTargetSpeed(3.0f, -1.0f, 0.0f), 2.1f, 1e-5f) &&
               Near(JogTargetSpeed(3.0f, 1.0f, 0.0f), 2.1f, 1e-5f) && Near(JogTargetSpeed(3.0f, 0.0f, -1.0f), 1.8f, 1e-5f) &&
               Near(JogTargetSpeed(3.0f, -0.7f, -0.7f), 1.8f, 1e-5f) && Near(JogTargetSpeed(3.0f, 0.7f, 0.7f), 3.0f, 1e-5f));
    Expect("jog: partial stick keeps the direction profile (|stick| 0.5 left = 2.10)", Near(JogTargetSpeed(3.0f, -0.5f, 0.0f), 2.1f, 1e-5f));
    Expect("jog: stick <= 0.15 -> forward F; jogF 0 / negative / NaN -> 0; NaN stick -> forward",
           Near(JogTargetSpeed(3.0f, 0.1f, -0.1f), 3.0f, 1e-6f) && JogTargetSpeed(0.0f, 0.0f, 1.0f) == 0.0f &&
               JogTargetSpeed(-1.0f, 0.0f, 1.0f) == 0.0f && JogTargetSpeed(std::nanf(""), 0.0f, 1.0f) == 0.0f &&
               Near(JogTargetSpeed(3.0f, std::nanf(""), std::nanf("")), 3.0f, 1e-6f));
    Expect("jog: accepted speeds 0 or 3.3..6 (1.65 / 3.0 / 3.29 / 6.01 / -1 / NaN rejected)",
           JogSpeedValid(0.0f) && JogSpeedValid(3.3f) && JogSpeedValid(3.85f) && JogSpeedValid(6.0f) && !JogSpeedValid(1.65f) &&
               !JogSpeedValid(3.0f) && !JogSpeedValid(3.29f) && !JogSpeedValid(6.01f) && !JogSpeedValid(-1.0f) && !JogSpeedValid(std::nanf("")));
    Expect("jog: at the minimum speed the slowest direction (back 0.60 F) stays above the vanilla aim walk 1.65 m/s",
           kJogMinF * 0.60f > 1.65f && JogTargetSpeed(kJogMinF, 0.0f, -1.0f) > 1.65f);
}

static void TestSubset(const char* aPath)
{
    Database db;
    DbResult r;
    if (!LoadAndProfile(aPath, db, r))
        return;
    const StrafeProfiles& p = r.prof;
    std::printf("  subset (%u clips):\n", db.NumClips());
    PrintTable(p);
    bool keep = true, turn = true;
    for (uint32_t c = 0; c < db.NumClips(); ++c)
    {
        const uint8_t k = r.classes[c];
        if (IsStrafeTurnBaseClass(k))
            turn = turn && p.classes[0][c] == kClassStrafeTurn;
        const std::string_view n = db.ClipName(c);
        if ((k == steer::kClassNone && n.find("_Loop") != std::string_view::npos) || steer::IsPivot(k))
            if (n.find("_Pivot_") != std::string_view::npos || n.find("_Loop") != std::string_view::npos)
                keep = keep && (p.maxCumYawDeg[c] >= 10.0f || p.classes[0][c] == k);
    }
    Expect("subset: turn / reface / TIP -> StrafeTurn", turn);
    Expect("subset: Neutral loops keep their class", keep);
    {
        const ClassRuleCounts cr = CheckClassRule(db, r);
        std::printf("  subset class rule: pivots@0=%u kept=%u turnClass=%u strafeTurn(ROOT off)=%u nonTurnChanged=%u\n", cr.pivots,
                    cr.pivotsKept, cr.turns, cr.turnsStrafe, cr.nonTurnChanged);
        Expect("subset: facing-fixed pivots keep their pivot class", cr.pivotsKept == cr.pivots, std::to_string(cr.pivotsKept) + "/" + std::to_string(cr.pivots));
        Expect("subset: ROOT rule off -> turn classes still StrafeTurn", cr.turns > 0 && cr.turnsStrafe == cr.turns,
               std::to_string(cr.turnsStrafe) + "/" + std::to_string(cr.turns));
        Expect("subset: ROOT rule off -> non-turn, non-pattern clips keep their class", cr.nonTurnChanged == 0, std::to_string(cr.nonTurnChanged));
    }
    Expect("subset: has facing-fixed loops", p.facingFixedLoops > 0, std::to_string(p.facingFixedLoops));
}

int main(int argc, char** argv)
{
    g_dump = argc >= 4 && std::string_view(argv[3]) == "dump";
    TestMode();
    TestDesired();
    TestJog();
    if (argc >= 2)
        TestV6(argv[1]);
    if (argc >= 3)
        TestSubset(argv[2]);
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
