// Compile check of the DRAFT adapter src/mm/MotionAdapter.hpp (report 6.10) plus checks of its pure helpers and of MapToChooser
// (ABP Update_States through MotionTrajectory). The adapter itself is exercised only through Init on an empty database (must
// fail closed). Optional --chooser <utpp_motion_*_v2.chooser.json>: the scenario chooser sets are evaluated with the real
// chooser table (Chooser.hpp); without it those set checks are reported as NOT RUN.
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "../src/mm/MotionAdapter.hpp"
#include "MotionFixRealFrames.hpp"
#include "MotionFixAirFrames.hpp"
#include "MotionFixCrouchFrames.hpp"

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const char* aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL %s\n", aWhat);
    }
}

namespace ch = utpp::mm::chooser;
using Set = std::vector<std::string>;

// Chooser databases (names, emission order) for a ChooserState: the same input binding as ChooserMatcher Init / FillInputs
// (ChooserMatcher.hpp l. 158-172 / 191-211), AllResults like the MM node.
Set ChooserSet(const ch::Table& t, const utpp::mm::motion::ChooserState& s)
{
    ch::Inputs in;
    t.ResetInputs(in);
    auto num = [&](const char* k, double v) { if (const int i = t.FindInput(k); i >= 0) t.SetNumber(in, i, v); };
    auto bo = [&](const char* k, bool v) { if (const int i = t.FindInput(k); i >= 0) t.SetBool(in, i, v); };
    auto en = [&](const char* k, int v) { if (const int i = t.FindInput(k); i >= 0) t.SetEnum(in, i, v); };
    en("Gait", s.gait);
    en("Stance", s.stance);
    en("MovementState", s.movementState);
    en("MovementMode", s.movementMode);
    num("Speed2D", s.speed2dCmS);
    num("TrajectoryCollision.TimeToLand", s.timeToLand);
    bo("IsStarting", s.isStarting);
    bo("IsPivoting", s.isPivoting);
    bo("JustLanded_Light", s.justLandedLight);
    bo("JustLanded_Heavy", s.justLandedHeavy);
    bo("JustTraversed", s.justTraversed);
    bo("ShouldTurnInPlace", s.shouldTurnInPlace);
    bo("ShouldSpinTransition", s.shouldSpinTransition);
    num("MMDatabaseLOD", s.databaseLod);
    ch::Output out;
    t.Evaluate(in, ch::Mode::AllResults, out);
    Set names;
    for (uint32_t i = 0; i < out.count; ++i)
        if (out.db[i] >= 0 && out.db[i] < static_cast<int>(t.NumDatabases()))
            names.push_back(t.GetDatabase(static_cast<uint32_t>(out.db[i])).name);
    return names;
}

bool HasSuffix(const Set& aSet, std::string_view aSuffix)
{
    for (const std::string& n : aSet)
        if (n.size() >= aSuffix.size() && n.compare(n.size() - aSuffix.size(), aSuffix.size(), aSuffix) == 0)
            return true;
    return false;
}

std::string Join(const Set& aSet)
{
    std::string s;
    for (const std::string& n : aSet)
        s += (s.empty() ? "" : "+") + n;
    return s.empty() ? std::string("<none>") : s;
}
} // namespace

int main(int argc, char** argv)
{
    using namespace utpp::mm::adapter;
    std::string chooserPath;
    for (int i = 1; i < argc; ++i)
        if (std::string_view(argv[i]) == "--chooser" && i + 1 < argc)
            chooserPath = argv[++i];
    ch::Table table;
    bool haveTable = false;
    if (!chooserPath.empty())
    {
        std::string loadErr;
        haveTable = table.LoadFile(chooserPath.c_str(), loadErr);
        Check(haveTable, "chooser table loads");
        if (!haveTable)
            std::printf("chooser %s: %s\n", chooserPath.c_str(), loadErr.c_str());
    }
    else
        std::printf("chooser-set checks NOT RUN (no --chooser <file>)\n");
    Version v = Version::V1;
    Check(ParseVersion("2", v) && v == Version::V2, "parse 2");
    Check(ParseVersion("base", v) && v == Version::V1, "parse base");
    Check(!ParseVersion("3", v), "parse 3 rejected");

    Check(PathsFor(Body::Woman, "C:\\x").db == "C:\\x\\data\\utpp_motion_woman.utppmm", "woman db path");
    Check(PathsFor(Body::Man, "C:\\x\\").chooser == "C:\\x\\data\\utpp_motion_man.chooser.json", "man chooser path");
    Check(PathsFor(Body::Unknown, "C:\\x").db.empty(), "unknown body no path");
    Check(CNameHash("") == 0xCBF29CE484222325ull, "fnv basis");

    ResolveInputs r;
    Check(Resolve(r) == Fallback::NotRequested, "base requested");
    r.requested = Version::V2;
    Check(Resolve(r) == Fallback::BodyUnknown, "body unknown");
    r.body = Body::Woman;
    Check(Resolve(r) == Fallback::DbMissing, "db missing");
    r.dbFile = r.dbParsed = true;
    r.dbRigIdentity = utpp::mm::motion::kRigIdentityMan;
    Check(Resolve(r) == Fallback::RigMismatch, "man db on woman body");
    r.dbRigIdentity = utpp::mm::motion::kRigIdentityWoman;
    r.topologyMismatches = 1;
    Check(Resolve(r) == Fallback::RigMismatch, "topology mismatch");
    r.topologyMismatches = 0;
    Check(Resolve(r) == Fallback::ChooserMissing, "chooser missing");
    r.chooserFile = r.chooserParsed = r.matcherInit = r.boneMapOk = true;
    Check(Resolve(r) == Fallback::None && Effective(Resolve(r)) == Version::V2, "motion active");
    r.budgetTripped = true;
    Check(Resolve(r) == Fallback::Budget && Effective(Resolve(r)) == Version::V1, "budget -> base");
    Check(std::string(FallbackName(Fallback::RigMismatch)) == "rig-mismatch", "fallback name");

    // ballistic TTL: 2 m drop from rest at 9.81 -> 0.6386 s; jump 6.2 m/s up from the ground -> 1.264 s; no hit -> 1.5
    Check(std::fabs(TimeToLand(2.0f, 0.0f, 9.81f, 1.5f) - 0.6386f) < 1e-3f, "ttl drop 2 m");
    Check(std::fabs(TimeToLand(0.0f, 6.2f, 9.81f, 1.5f) - 1.2640f) < 1e-3f, "ttl jump 6.2");
    Check(TimeToLand(-1.0f, 0.0f, 9.81f, 1.5f) == 1.5f, "ttl no hit");

    // stick -> world: v1 convention (BodyTurn.hpp LegacyDesiredMotion l. 90-92); stick forward + camera yaw 90 -> world (-1, 0)
    {
        float wx = 0, wy = 0;
        StickToWorld(0.0f, 1.0f, 90.0f, wx, wy);
        Check(std::fabs(wx + 1.0f) < 1e-6f && std::fabs(wy) < 1e-6f, "stick fwd cam 90 -> world -X (base convention)");
        StickToWorld(1.0f, 0.0f, 0.0f, wx, wy);
        Check(std::fabs(wx - 1.0f) < 1e-6f && std::fabs(wy) < 1e-6f, "stick right cam 0 -> world +X");
        MotionFeed fa;
        fa.move[1] = 0.5f;
        fa.cameraYawDeg = 90.0f;
        // was: 400 (maxAccel x half stick); motion default FixedSpeed-SingleGait scales the input with Normal2D (CBP l. 5,
        // 1118-1119, IA_Move l. 840-841) -> full MaxAcceleration; VariableSpeed modes keep the raw input
        const auto a = InputAcceleration(fa, 800.0f);
        Check(std::fabs(a.x + 800.0f) < 1e-3f && std::fabs(a.y) < 1e-3f && a.z == 0.0f, "half stick FixedSpeed -> accel = maxAccel (Normal2D)");
        const auto av = InputAcceleration(fa, 800.0f, StickMode::VariableSpeedSingleGait);
        Check(std::fabs(av.x + 400.0f) < 1e-3f && std::fabs(av.y) < 1e-3f, "half stick VariableSpeed -> accel = maxAccel x stick");
        // kStickReleased 0.02 (2026-10-31): the game moves the capsule at |mv| 0.09, so only residual noise counts as released
        fa.move[1] = kStickReleased;
        Check(utpp::mm::motiontraj::IsZero(InputAcceleration(fa, 800.0f)), "released stick (|mv| <= kStickReleased) -> accel 0");
        fa.move[1] = 0.05f;
        Check(kStickReleased == 0.02f && !utpp::mm::motiontraj::IsZero(InputAcceleration(fa, 800.0f)),
              "small tilt |mv| 0.05 is input (kStickReleased 0.02, was 0.1)");
    }
    MapConfig c;
    MapState m;
    MapContext x;
    MotionFeed f;
    f.dt = 1.0f / 60.0f;
    f.psmDetailed = kPsmStand;
    f.velWorld[0] = 4.0f; // 400 cm/s -> run
    f.move[0] = 1.0f;     // stick right, camera 0 -> world +X (along the velocity)
    utpp::mm::motion::ChooserState s = MapToChooser(f, c, m, x);
    // was: Moving from capsule speed alone; ABP IsMoving (pseudo l. 1777-1780) needs Acceleration != 0 -> stick held here
    Check(s.movementMode == 0 && s.gait == 1 && s.movementState == 1 && s.stance == 0, "stand run");
    // Piecewise game -> motion map (MapConfig): 2.4 m/s -> 200 + 0.8 x 300 / 2.25 = 307
    Check(std::fabs(SpeedScaleFor(c, 0, 2.4f) * 240.0f - 306.67f) < 0.05f, "speed map 2.4 m/s -> 307 cm/s");
    // was: gait from speed (300 +- 30 cm/s, [P]); CBP GetDesiredGait (l. 1173-1187) takes it from the input: FixedSpeed-
    // SingleGait -> Run at any speed / stick unless WantsToWalk (Walk) or CanSprint (Sprint)
    c.stickMode = StickMode::FixedSpeedSingleGait; // motion default mode (CBP l. 5); MapConfig default is VariableSpeed-Walk/Run [P]
    f.velWorld[0] = 1.0f;
    f.move[0] = 0.3f;
    s = MapToChooser(f, c, m, x);
    Check(s.gait == 1, "SingleGait: slow + partial stick -> Run (gait from input, not speed)");
    f.wantsToWalk = true;
    s = MapToChooser(f, c, m, x);
    Check(s.gait == 0, "WantsToWalk -> Walk");
    f.wantsToWalk = false;
    f.psmDetailed = kPsmSprint;
    s = MapToChooser(f, c, m, x);
    Check(s.gait == 2, "WantsToSprint (PSM Sprint), orient to movement -> Sprint");
    c.stickMode = StickMode::FixedSpeedWalkRun;
    f.psmDetailed = kPsmStand;
    s = MapToChooser(f, c, m, x);
    Check(s.gait == 0, "Walk/Run mode: |stick| 0.3 < 0.7 -> Walk");
    f.move[0] = 0.8f;
    s = MapToChooser(f, c, m, x);
    Check(s.gait == 1, "Walk/Run mode: |stick| 0.8 >= 0.7 -> Run");
    c.stickMode = MapConfig{}.stickMode; // back to the default (VariableSpeed-Walk/Run [P]) for the scenarios below
    f.move[0] = 1.0f;
    // CanSprint in strafe (aiming): |actor yaw - input yaw| < 50 (CBP l. 1378); input right (+X) = integ yaw -90
    f.psmDetailed = kPsmSprint;
    f.aiming = true;
    f.playerYawDeg = -90.0f;
    s = MapToChooser(f, c, m, x);
    Check(s.gait == 2, "strafe sprint, actor facing the input -> Sprint");
    f.playerYawDeg = 0.0f;
    s = MapToChooser(f, c, m, x);
    Check(s.gait == 1, "strafe sprint, input 90 deg off the actor -> CanSprint false -> Run");
    f.aiming = false;
    f.psmDetailed = kPsmStand;
    f.psmDetailed = kPsmCrouchSprint;
    s = MapToChooser(f, c, m, x);
    Check(s.stance == 0 && s.gait == 2, "crouch sprint (PSM 30) = Stand + Sprint (the motion set-motionfix item 7)");
    Check(PsmSprint(kPsmCrouchSprint) && PsmSprint(kPsmSprint) && !PsmCrouchStance(kPsmCrouchSprint) && PsmCrouchStance(kPsmCrouch),
          "PsmSprint / PsmCrouchStance: CrouchSprint sprints standing");
    // motion TimeToLand is analytic from the fall height of the first floor hit (UE PoseSearchTrajectoryLibrary.cpp l. 941-949:
    // v0/g + sqrt(v0^2 + 2 g FallHeight)/g), not quantised to the 0.1 s samples -> the ballistic value
    {
        // jump 6.2 m/s from the floor on a fresh state (no history: GetSampleAtTime(0) = the hit sample, FallHeight 0)
        MapState mj;
        MotionFeed fj;
        fj.dt = 1.0f / 60.0f;
        fj.psmDetailed = kPsmJump;
        fj.velWorld[2] = 6.2f;
        fj.air.valid = true;
        fj.air.onGround = false;
        fj.air.distanceToGround = 0.0f;
        const auto sj = MapToChooser(fj, c, mj, x);
        Check(sj.movementMode == 1 && std::fabs(sj.timeToLand - 1.264f) < 1e-3f, "jump in air + ttl");
        std::printf("ttl jump 6.2: trajectory %.4f ballistic %.4f\n", static_cast<double>(sj.timeToLand),
                    static_cast<double>(TimeToLand(0.0f, 6.2f, 9.81f, 1.5f)));
    }
    f.psmDetailed = kPsmFall;
    f.velWorld[2] = 0.0f;
    f.air.valid = true;
    f.air.onGround = false;
    f.air.distanceToGround = 2.0f;
    s = MapToChooser(f, c, m, x);
    const float ball = TimeToLand(2.0f, 0.0f, 9.81f, 1.5f);
    Check(s.movementMode == 1 && std::fabs(s.timeToLand - ball) < 1e-3f, "fall 2 m: trajectory ttl = ballistic");
    std::printf("ttl 2 m: trajectory %.4f ballistic %.4f\n", static_cast<double>(s.timeToLand), static_cast<double>(ball));
    f.air.distanceToGround = -1.0f;
    s = MapToChooser(f, c, m, x);
    // was: ttlNoHit 1.5; motion no floor hit -> time of the last trajectory sample (dt + 15 x 0.1)
    Check(std::fabs(s.timeToLand - (f.dt + 1.5f)) < 1e-4f, "air feed no hit -> last sample time");
    f.psmDetailed = kPsmStand; // PSM still Stand on the first air frame: UTPPFeedAir !onGround leads
    f.velWorld[2] = -8.0f;     // falling at 800 cm/s
    s = MapToChooser(f, c, m, x);
    Check(s.movementMode == 1, "air feed off ground counts as air");
    // was: heavy from the PSM HardLand detail; CBP OnLanded (l. 889-890, 918-923) LandVelocity at Falling -> Walking and
    // ABP JustLanded_Heavy |LandVelocity.Z| >= 700: the movementMode transition with the last air vz decides
    f.psmDetailed = kPsmStand;
    f.velWorld[2] = 0.0f;
    f.air.onGround = true;
    s = MapToChooser(f, c, m, x);
    Check(s.movementMode == 0 && s.justLandedHeavy && !s.justLandedLight, "land at vz -8 m/s -> heavy");
    if (haveTable)
    {
        // stick held, 1 m/s -> Stand Walks row 6 (JustLanded_Heavy -> PSD_*_Stand_Walk_Lands_Heavy, choosers_rules.txt)
        const Set ls = ChooserSet(table, s);
        Check(HasSuffix(ls, "_Lands_Heavy") && !HasSuffix(ls, "_Lands_Light"), "heavy landing -> *_Lands_Heavy in the chooser set");
        std::printf("land set: %s\n", Join(ls).c_str());
    }
    {
        int heavyFrames = 1;
        for (int i = 1; i < 30; ++i)
        {
            s = MapToChooser(f, c, m, x);
            if (s.justLandedHeavy)
                ++heavyFrames;
            if (i == 16)
                Check(s.justLandedHeavy, "heavy still true at 0.27 s");
        }
        // RetriggerableDelay 0.3 s at 60 Hz: 18 frames (0 .. 0.283 s), allow the float edge at exactly 0.3 s
        Check(heavyFrames >= 18 && heavyFrames <= 19, "justLandedHeavy for 0.3 s");
        std::printf("justLandedHeavy frames %d (60 Hz)\n", heavyFrames);
    }
    Check(!s.justLandedHeavy, "land flag expires");
    // J1: under the game gravity (16 m/s^2) the heavy threshold is the same 2.5 m drop (894 cm/s, latched at touchdown):
    // -8 m/s (hood / roof jump-off) lands light for the whole window, -10 m/s (3 m drop) heavy
    {
        MapConfig cg = c;
        cg.gravity = 16.0f;
        auto land = [&](float aVz, int& aLight, int& aHeavy, float aGravityAfter) {
            MapState mg;
            MotionFeed fg;
            fg.dt = 1.0f / 60.0f;
            fg.psmDetailed = kPsmFall;
            fg.air.valid = true;
            fg.air.onGround = false;
            fg.air.distanceToGround = -1.0f;
            fg.velWorld[2] = aVz;
            for (int i = 0; i < 3; ++i)
                (void)MapToChooser(fg, cg, mg, x);
            fg.psmDetailed = kPsmStand;
            fg.velWorld[2] = 0.0f;
            fg.air.onGround = true;
            aLight = aHeavy = 0;
            MapConfig ca = cg;
            for (int i = 0; i < 30; ++i)
            {
                const auto sg = MapToChooser(fg, ca, mg, x);
                ca.gravity = aGravityAfter;
                aLight += sg.justLandedLight ? 1 : 0;
                aHeavy += sg.justLandedHeavy ? 1 : 0;
            }
        };
        int light8 = 0, heavy8 = 0, light10 = 0, heavy10 = 0, lightL = 0, heavyL = 0;
        land(-8.0f, light8, heavy8, 16.0f);
        land(-10.0f, light10, heavy10, 16.0f);
        // latched at the touchdown feed: the gravity of the following feeds (9.81 here) does not move it inside the window
        land(-8.0f, lightL, heavyL, 9.81f);
        std::printf("g 16: vz -8 light %d heavy %d, vz -10 light %d heavy %d (60 Hz)\n", light8, heavy8, light10, heavy10);
        Check(light8 >= 18 && light8 <= 19 && heavy8 == 0, "g 16: land at vz -8 m/s -> light for 0.3 s, never heavy");
        Check(heavy10 >= 18 && heavy10 <= 19 && light10 == 0, "g 16: land at vz -10 m/s -> heavy for 0.3 s, never light");
        Check(lightL >= 18 && lightL <= 19 && heavyL == 0, "g 16 latched at touchdown: gravity 9.81 after it keeps vz -8 light");
    }
    // r3 landing: the heavy land of a vehicle jump-out flight (MapContext::jumpOutFlight) keeps the ordinary threshold (the 11.4 m/s impact of the
    // 2026-11-03 test is heavy at 8.9 m/s), but its recovery is capped: ChooserState::jumpOutLandOver rises motiontraj::kJumpOutLandRecoverS after the
    // touchdown and stays up on the ground. An ordinary flight with the same impact and a light jump-out land never raise it; a new flight clears it.
    {
        MapConfig cg = c;
        cg.gravity = 16.0f;
        struct LandRun
        {
            int light = 0, heavy = 0, firstOver = -1, overFrames = 0, overInAir = 0;
        };
        auto land = [&](float aVz, bool aJumpOut) {
            LandRun out;
            MapState mg;
            MapContext xj = x;
            xj.jumpOutFlight = aJumpOut;
            MotionFeed fg;
            fg.dt = 1.0f / 60.0f;
            fg.psmDetailed = kPsmFall;
            fg.air.valid = true;
            fg.air.onGround = false;
            fg.air.distanceToGround = -1.0f;
            fg.velWorld[2] = aVz;
            for (int i = 0; i < 3; ++i)
                (void)MapToChooser(fg, cg, mg, xj);
            fg.psmDetailed = kPsmStand;
            fg.velWorld[2] = 0.0f;
            fg.air.onGround = true;
            for (int i = 0; i < 90; ++i)
            {
                const auto sg = MapToChooser(fg, cg, mg, xj);
                out.light += sg.justLandedLight ? 1 : 0;
                out.heavy += sg.justLandedHeavy ? 1 : 0;
                if (sg.jumpOutLandOver)
                {
                    out.firstOver = out.firstOver < 0 ? i : out.firstOver;
                    ++out.overFrames;
                }
            }
            // a second flight in the same window: the cap flag is gone while airborne
            fg.psmDetailed = kPsmFall;
            fg.air.onGround = false;
            fg.velWorld[2] = 3.0f;
            for (int i = 0; i < 3; ++i)
                out.overInAir += MapToChooser(fg, cg, mg, xj).jumpOutLandOver ? 1 : 0;
            return out;
        };
        const LandRun jo = land(-11.4f, true), plain = land(-11.4f, false), lightJo = land(-8.0f, true);
        std::printf("g 16 jump-out heavy land cap: flag heavy %d light %d firstOver %d overFrames %d overInAir %d; no flag heavy %d over %d; vz -8 flag over %d (60 Hz)\n", jo.heavy,
                    jo.light, jo.firstOver, jo.overFrames, jo.overInAir, plain.heavy, plain.overFrames, lightJo.overFrames);
        Check(jo.heavy >= 18 && jo.heavy <= 19 && jo.light == 0, "g 16 jump-out flight: touchdown at vz -11.4 m/s keeps the ordinary threshold (heavy for 0.3 s, never light)");
        Check(jo.firstOver >= 35 && jo.firstOver <= 37 && jo.overFrames == 90 - jo.firstOver,
              "g 16 jump-out heavy land: the recovery cap rises about 0.6 s after the touchdown and stays up on the ground");
        Check(jo.overInAir == 0, "g 16 jump-out heavy land: the cap flag is gone in the next flight");
        Check(plain.heavy >= 18 && plain.heavy <= 19 && plain.overFrames == 0, "g 16 without the jump-out flag: the same impact is heavy and never capped (unchanged)");
        Check(lightJo.light >= 18 && lightJo.light <= 19 && lightJo.heavy == 0 && lightJo.overFrames == 0, "g 16 jump-out flight: a vz -8 m/s touchdown stays light and is not capped");
    }
    // CAR-1 step 4: PSM detail 0 (workspot / vehicle, no detailed locomotion) on the ground maps to Stand (stance 0), from a fresh
    // state and held after a stand feed
    {
        MapState m0;
        MotionFeed f0;
        f0.dt = 1.0f / 60.0f;
        f0.psmDetailed = 0;
        bool allStand = MapToChooser(f0, c, m0, x).stance == 0;
        f0.psmDetailed = kPsmStand;
        allStand = allStand && MapToChooser(f0, c, m0, x).stance == 0;
        f0.psmDetailed = 0;
        for (int i = 0; i < 30; ++i)
            allStand = allStand && MapToChooser(f0, c, m0, x).stance == 0;
        Check(allStand, "PSM detail 0 + stand -> stance 0");
    }
    // correction 2026-10-13: release set = runtime constant (ladder 10-13, knockdown 29, felled 31); everything else held
    Check(PsmReleasedToGame(kPsmLadderFirst) && PsmReleasedToGame(kPsmLadderLast) && PsmReleasedToGame(kPsmKnockdown) &&
              PsmReleasedToGame(kPsmFelled),
          "released states = runtime constant");
    {
        int held = 0;
        for (int d : {int(kPsmAimWalk), int(kPsmSlide), int(kPsmSlideFall), int(kPsmDodge), int(kPsmClimb), int(kPsmVault), 14, 15, 16,
                      17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, int(kPsmCrouchSprint)})
            held += PsmReleasedToGame(d) ? 0 : 1;
        Check(held == 22, "aimwalk / slide / dodge / climb / vault / air 14-25 / 26-28 / crouch sprint held");
    }
    Check(MotionRuns(0u) && !MotionRuns(utpp::mm::integ::kGatePsmBlock), "motion runs only on effGate 0 (open or held)");
    Check(!PsmAir(kPsmClimb) && PsmTraversal(kPsmClimb), "climb counts as ground");
    {
        // existing psmBlock packing: detail + 1 in bits 16..23 only with bit 6, upper body + 1 in bits 13..15
        using namespace utpp::mm::integ;
        const uint32_t jump = kPsmDetailed | (uint32_t(kPsmJump + 1) << kPsmDetailShift);
        Check(DetailFromFeed(jump, kLocoOther) == kPsmJump, "detail from packed field");
        Check(DetailFromFeed(0u, kLocoCrouch) == kPsmCrouch && DetailFromFeed(0u, kLocoSprint) == kPsmSprint &&
                  DetailFromFeed(0u, kLocoStand) == kPsmStand && DetailFromFeed(0u, kLocoOther) == kPsmStand,
              "detail 1/3/4 from locoState when not packed");
        Check(AimingFromFeed(uint32_t(6 + 1) << kPsmUpperShift) && !AimingFromFeed(uint32_t(5 + 1) << kPsmUpperShift) && !AimingFromFeed(0u),
              "aiming from UpperBody field");
        const float vel[3] = {1.0f, 0.0f, 0.0f};
        const float mv[2] = {0.0f, 1.0f};
        AirFeed a;
        a.valid = true;
        const MotionFeed fv = MakeMotionFeed(vel, mv, 1.0f / 60.0f, jump | (uint32_t(6 + 1) << kPsmUpperShift), kLocoOther, a);
        Check(fv.psmDetailed == kPsmJump && fv.aiming && fv.air.valid && fv.velWorld[0] == 1.0f && fv.move[1] == 1.0f, "MakeMotionFeed");
    }
    {
        // justTraversed from the Climb / Vault exit transition
        MapState mt;
        MotionFeed ft;
        ft.dt = 1.0f / 60.0f;
        ft.psmDetailed = kPsmVault;
        Check(!MapToChooser(ft, c, mt, x).justTraversed, "vault not yet traversed");
        ft.psmDetailed = kPsmStand;
        Check(MapToChooser(ft, c, mt, x).justTraversed, "vault exit -> justTraversed");
        Check(!MapToChooser(ft, c, mt, x).justTraversed, "justTraversed one frame");
    }

    // ---------------------------------------------------------------- scenarios: ABP Update_States through MotionTrajectory
    namespace t2 = utpp::mm::motiontraj;
    auto feed = [](float aVy, float aMy) {
        MotionFeed g;
        g.dt = 1.0f / 60.0f;
        g.psmDetailed = kPsmStand;
        g.velWorld[1] = aVy; // camera yaw 0: stick forward (MoveY) = world +Y (StickToWorld)
        g.move[1] = aMy;
        return g;
    };
    const MapContext x0; // previous result without the Pivots tag; root yaw = actor yaw (ABP l. 1762 else branch)
    {
        // steady run (point 3): game run 3.70..3.85 m/s (run_arc_left range) with the stick held, 60 frames after a 30-frame
        // warm-up -> Moving every frame, no IsStarting (ABP IsStarting: |Trj_FutureVelocity| >= |Velocity| + 100) and no
        // invented deceleration (piecewise speed map lands the game run on the motion Run cap 500), one constant chooser set
        MapState ms;
        utpp::mm::motion::ChooserState cs;
        for (int i = 0; i < 30; ++i)
            cs = MapToChooser(feed(3.85f, 1.0f), c, ms, x0);
        int moving = 0, starting = 0, pivoting = 0, run = 0, setChanges = 0;
        float minGain = 1.0e9f, maxGain = -1.0e9f;
        bool startsOrStops = false, loops = true;
        Set first;
        for (int i = 0; i < 60; ++i)
        {
            const float vRun = 3.775f + 0.075f * std::sin(0.7f * static_cast<float>(i));
            cs = MapToChooser(feed(vRun, 1.0f), c, ms, x0);
            moving += cs.movementState == 1 ? 1 : 0;
            starting += cs.isStarting ? 1 : 0;
            pivoting += cs.isPivoting ? 1 : 0;
            run += cs.gait == 1 ? 1 : 0;
            const float gain = t2::SizeXY(ms.st.trjFuture) - t2::SizeXY(ms.st.velocity);
            minGain = std::fmin(minGain, gain);
            maxGain = std::fmax(maxGain, gain);
            if (haveTable)
            {
                const Set st = ChooserSet(table, cs);
                if (i == 0)
                    first = st;
                else if (st != first)
                    ++setChanges;
                startsOrStops = startsOrStops || HasSuffix(st, "_Starts") || HasSuffix(st, "_Stops");
                loops = loops && HasSuffix(st, "_Run_Loops");
            }
        }
        Check(moving == 60 && starting == 0 && pivoting == 0 && run == 60, "steady run 60 frames: Moving, Run, no IsStarting / IsPivoting");
        Check(minGain > -1.0f && maxGain < t2::kStartingMargin, "steady run: future speed - speed in (-1, +100) cm/s (no invented accel / decel)");
        std::printf("steady run: future - current speed %.1f .. %.1f cm/s\n", static_cast<double>(minGain), static_cast<double>(maxGain));
        if (haveTable)
        {
            Check(setChanges == 0 && loops && !startsOrStops, "steady run: one chooser set with *_Run_Loops, no *_Starts / *_Stops");
            std::printf("steady run set: %s (changes %d)\n", Join(first).c_str(), setChanges);
        }
        // the old uniform x1 scale (diagnosis): the same feed keeps IsStarting true (motion Run cap 500 vs 377..385 cm/s)
        MapConfig u = c;
        u.speedScale = 1.0f;
        u.stickMode = StickMode::FixedSpeedSingleGait; // the a222fdf diagnosis config (motion default mode, CMC modifier)
        u.gameAnalogModifier = false;
        u.cmc = utpp::mm::motiontraj::MotionCmc{};
        MapState mu;
        int startingUniform = 0;
        for (int i = 0; i < 60; ++i)
            startingUniform += MapToChooser(feed(3.775f + 0.075f * std::sin(0.7f * static_cast<float>(i)), 1.0f), u, mu, x0).isStarting ? 1 : 0;
        std::printf("steady run with uniform speedScale 1: IsStarting on %d / 60 frames (piecewise map: %d)\n", startingUniform, starting);
    }
    {
        // finding 7: steady game motion at every measured stick bucket (night MMTRACE medians, sourcepack-gaitspeed-analysis.txt;
        // stand / crouch / PSM sprint) for 60 frames after a 30-frame warm-up -> no IsStarting, no state / gait change, the
        // prediction cap (MakeDerived maxSpeed) equals the mapped steady game speed, |future - current| < 100 cm/s
        // (ABP IsStarting margin); half stick -> Walk gait and Walk_* databases, full stick -> Run.
        struct Bucket
        {
            float stick, speed;
            int32_t psm;
            int gait;
            const char* name;
        };
        const Bucket buckets[] = {
            {0.18f, 0.30f, kPsmStand, 0, "stand 0.18"},  {0.27f, 0.41f, kPsmStand, 0, "stand 0.27"},
            {0.73f, 1.94f, kPsmStand, 1, "stand 0.73"},  {0.82f, 2.48f, kPsmStand, 1, "stand 0.82"},
            {1.00f, 3.84f, kPsmStand, 1, "stand 1.00"},  {0.82f, 1.76f, kPsmCrouch, 1, "crouch 0.82"},
            {1.00f, 8.21f, kPsmSprint, 2, "sprint 1.00"},
        };
        for (const Bucket& b : buckets)
        {
            MapState ms;
            utpp::mm::motion::ChooserState cs;
            auto fb = feed(b.speed, b.stick);
            fb.psmDetailed = b.psm;
            for (int i = 0; i < 30; ++i)
                cs = MapToChooser(fb, c, ms, x0);
            const int stance = b.psm == kPsmCrouch ? 1 : 0;
            const float mapped = b.speed * 100.0f * SpeedScaleFor(c, stance, b.speed);
            const t2::Derived dd = t2::MakeDerived(c.cmc, cs.gait, stance,
                                                   GameAnalogModifier(c, cs.gait, stance, b.psm == kPsmSprint, b.stick), mapped,
                                                   t2::V3{0.0f, 1.0f, 0.0f}, true, false);
            int starting = 0, changes = 0, gaitOk = 0, setChanges = 0;
            float maxAbsGain = 0.0f;
            bool walkDbs = true;
            Set first;
            for (int i = 0; i < 60; ++i)
            {
                cs = MapToChooser(fb, c, ms, x0);
                starting += cs.isStarting ? 1 : 0;
                changes += (ms.st.movementState != ms.st.movementStateLast || ms.st.gait != ms.st.gaitLast ||
                            ms.st.stance != ms.st.stanceLast || cs.movementState != 1)
                               ? 1
                               : 0;
                gaitOk += cs.gait == b.gait ? 1 : 0;
                maxAbsGain = std::fmax(maxAbsGain, std::fabs(t2::SizeXY(ms.st.trjFuture) - t2::SizeXY(ms.st.velocity)));
                if (haveTable)
                {
                    const Set st = ChooserSet(table, cs);
                    if (i == 0)
                        first = st;
                    else if (st != first)
                        ++setChanges;
                    if (b.gait == 0)
                    {
                        bool any = false;
                        for (const std::string& n : st)
                            any = any || n.find("_Walk_") != std::string::npos;
                        walkDbs = walkDbs && any;
                    }
                }
            }
            char what[160];
            std::snprintf(what, sizeof what, "steady %s m/s %.2f: no IsStarting / state change, gait %d", b.name,
                          static_cast<double>(b.speed), b.gait);
            Check(starting == 0 && changes == 0 && gaitOk == 60, what);
            // [P] 2 cm/s: crouch uses the stand curve fraction (0.646 x 2.7 = 1.744 vs 1.76 measured -> 1.3 cm/s mapped)
            std::snprintf(what, sizeof what, "steady %s: prediction cap %.1f == mapped steady speed %.1f cm/s", b.name,
                          static_cast<double>(dd.maxSpeed), static_cast<double>(mapped));
            Check(std::fabs(dd.maxSpeed - mapped) < 2.0f, what);
            std::snprintf(what, sizeof what, "steady %s: |future - current| %.1f < 100 cm/s", b.name, static_cast<double>(maxAbsGain));
            Check(maxAbsGain < t2::kStartingMargin, what);
            if (haveTable)
            {
                std::snprintf(what, sizeof what, "steady %s: one chooser set%s", b.name, b.gait == 0 ? " with Walk_* databases" : "");
                Check(setChanges == 0 && walkDbs, what);
                std::printf("steady %s set: %s\n", b.name, Join(first).c_str());
            }
        }
    }
    {
        // stick released at 4 m/s while the capsule still moves: ABP IsMoving needs Acceleration != 0 (pseudo l. 1777-1780)
        // -> MovementState Idle; chooser "Stand Idles" rows 1-2 (Speed2D >= 20 / >= 100) -> *_Stops
        MapState ms;
        utpp::mm::motion::ChooserState cs;
        for (int i = 0; i < 30; ++i)
            cs = MapToChooser(feed(4.0f, 1.0f), c, ms, x0);
        Check(cs.movementState == 1, "run at 4 m/s with the stick held: Moving");
        cs = MapToChooser(feed(4.0f, 0.0f), c, ms, x0);
        Check(cs.movementState == 0 && cs.speed2dCmS > 100.0f, "stick released at 4 m/s -> Idle while the capsule moves");
        if (haveTable)
        {
            const Set st = ChooserSet(table, cs);
            Check(HasSuffix(st, "_Stops") && !HasSuffix(st, "_Idles"), "stick released at 4 m/s -> *_Stops in the chooser set");
            std::printf("stop set: %s\n", Join(st).c_str());
        }
        int idle = 0;
        for (int i = 1; i <= 20; ++i)
            idle += MapToChooser(feed(4.0f - 0.15f * static_cast<float>(i), 0.0f), c, ms, x0).movementState == 0 ? 1 : 0;
        Check(idle == 20, "decelerating capsule without stick stays Idle (no Moving flicker)");
    }
    {
        // stick from 0 at rest: prediction accelerates (CMC 800 cm/s^2) -> Moving, IsStarting (|Future| >= 0 + 100) -> *_Starts
        MapState ms;
        utpp::mm::motion::ChooserState cs;
        for (int i = 0; i < 30; ++i)
            cs = MapToChooser(feed(0.0f, 0.0f), c, ms, x0);
        Check(cs.movementState == 0 && !cs.isStarting, "at rest: Idle");
        if (haveTable)
            Check(HasSuffix(ChooserSet(table, cs), "_Stand_Idles"), "at rest -> *_Stand_Idles");
        cs = MapToChooser(feed(0.0f, 1.0f), c, ms, x0);
        Check(cs.movementState == 1 && cs.isStarting, "stick from 0 -> Moving + IsStarting");
        if (haveTable)
        {
            const Set st = ChooserSet(table, cs);
            Check(HasSuffix(st, "_Starts"), "stick from 0 -> *_Starts in the chooser set");
            std::printf("start set: %s\n", Join(st).c_str());
        }
    }
    {
        // 180-degree stick flip at run: future velocity reverses -> TurnAngle 180 >= 45 (IsPivoting, OrientToMovement)
        MapState ms;
        utpp::mm::motion::ChooserState cs;
        for (int i = 0; i < 30; ++i)
            cs = MapToChooser(feed(3.85f, 1.0f), c, ms, x0);
        cs = MapToChooser(feed(3.85f, -1.0f), c, ms, x0);
        Check(cs.movementState == 1 && cs.isPivoting && cs.gait == 1 && !cs.isStarting, "180 stick flip at run -> Moving + IsPivoting");
        if (haveTable)
        {
            const Set st = ChooserSet(table, cs);
            Check(HasSuffix(st, "_Run_Pivots"), "180 stick flip at run -> *_Run_Pivots in the chooser set");
            std::printf("pivot set: %s\n", Join(st).c_str());
        }
    }

    // ---------------------------------------------------------------- MM query trajectory (finding 8, BuildQueryTrajectory)
    // Times = PSS_Default trajectory reads (report schema table: -0.05 Pos / 0 Vel+Facing / 0.35 Pos+Facing / 0.70 Pos+Vel+Facing
    // / 1.0 VelDir) incl. the -1/60 velocity partners, built like BuildQueryTimes (same float arithmetic).
    {
        constexpr float h = 1.0f / 60.0f;
        QueryTimes qt;
        for (const float t : {-0.05f, 0.0f - h, 0.0f, 0.35f, 0.70f - h, 0.70f, 1.0f - h, 1.0f})
        {
            uint32_t i = qt.n++;
            while (i > 0 && qt.t[i - 1] > t)
            {
                qt.t[i] = qt.t[i - 1];
                --i;
            }
            qt.t[i] = t;
        }
        utpp::mm::motion::TrajSample q[kQueryTrajMax];
        auto at = [&](uint32_t n, float t) {
            for (uint32_t i = 0; i < n; ++i)
                if (q[i].t == t)
                    return q[i];
            return utpp::mm::motion::TrajSample{-99.0f, 1.0e9f, 1.0e9f, 1.0e9f, 1.0e9f};
        };
        auto near = [](float a, float b, float tol) { return std::fabs(a - b) <= tol; };
        constexpr float kDeg = 3.14159265358979f / 180.0f;
        // [P] 1 mm / 1e-4 rad: float rounding of positions integrated in cm over <= 90 frames (|pos| < 1000 cm, float ulp
        // there 6e-5 cm); every expectation below is exact in real arithmetic.
        constexpr float kPosTol = 1.0e-3f, kYawTol = 1.0e-4f;
        {
            MapState ms;
            utpp::mm::motion::ChooserState cs;
            Check(BuildQueryTrajectory(ms, qt, q, kQueryTrajMax) == 0, "query trajectory: not built before the first update");
            // (a) straight steady run along game yaw 30 (forward (-sin, cos)), stick forward in camera space, actor = camera yaw;
            // game 3.85 m/s -> motion Run cap 500 cm/s (piecewise map), steady (no acceleration left at the cap, StepGround).
            const float psi = 30.0f;
            MotionFeed g = feed(0.0f, 1.0f);
            g.velWorld[0] = -std::sin(psi * kDeg) * 3.85f;
            g.velWorld[1] = std::cos(psi * kDeg) * 3.85f;
            g.cameraYawDeg = psi;
            g.playerYawDeg = psi;
            for (int i = 0; i < 90; ++i)
                cs = MapToChooser(g, c, ms, x0);
            const uint32_t n = BuildQueryTrajectory(ms, qt, q, kQueryTrajMax);
            Check(n == qt.n, "query trajectory: one sample per read time");
            bool sorted = true;
            for (uint32_t i = 1; i < n; ++i)
                sorted = sorted && q[i].t > q[i - 1].t;
            Check(sorted, "query trajectory: sorted by time");
            const auto o = at(n, 0.0f);
            const float vx = -std::sin(psi * kDeg) * 5.0f, vy = std::cos(psi * kDeg) * 5.0f; // m/s, motion space
            bool pos = true, yaw = true;
            for (const float t : {-0.05f, 0.35f, 0.70f, 1.0f})
            {
                const auto smp = at(n, t);
                pos = pos && near(smp.x - o.x, vx * t, kPosTol) && near(smp.y - o.y, vy * t, kPosTol) && near(smp.z - o.z, 0.0f, kPosTol);
            }
            for (uint32_t i = 0; i < n; ++i)
                yaw = yaw && near(utpp::mm::math::WrapAngle(q[i].yaw - psi * kDeg), 0.0f, kYawTol);
            Check(pos, "steady run: query position(t) - position(0) = v t at -0.05 / 0.35 / 0.70 / 1.0 (v = 500 cm/s along yaw 30)");
            Check(yaw, "steady run (OrientToMovement): query facing = velocity direction = game yaw 30 at every time");
            const auto a = at(n, 0.70f), b = at(n, 0.70f - h);
            Check(near((a.x - b.x) * 60.0f, vx, 1.0e-2f) && near((a.y - b.y) * 60.0f, vy, 1.0e-2f),
                  "steady run: query velocity at 0.70 (finite difference 1/60) = v");
            std::printf("query steady run: p(0.35) - p(0) = (%.4f, %.4f) m, expected (%.4f, %.4f)\n",
                        static_cast<double>(at(n, 0.35f).x - o.x), static_cast<double>(at(n, 0.35f).y - o.y),
                        static_cast<double>(vx * 0.35f), static_cast<double>(vy * 0.35f));

            // (c) stop: stick released at the run (capsule still 3.85 m/s on this frame). motion braking without input
            // (MakeDerived: BrakingDeceleration 2000, friction BrakingFriction 0 x factor 1; StepGround = UE
            // StepCharacterMovementGroundPrediction, PSTL l. 125-193, one 0.1 s braking step since friction is 0, stop below
            // BRAKE_TO_STOP 10 or on reversal): prediction steps 500 -> 300 -> 100 -> 0 cm/s; positions are integrated with
            // the velocity at the start of each 0.1 s step (UpdatePrediction l. 474-520): 50 / 80 / 90 cm after the current
            // sample (t = DeltaTime), then constant. Query origin p(0) = last frame's capsule = current - 500 x dt.
            MotionFeed st = g;
            st.move[1] = 0.0f;
            cs = MapToChooser(st, c, ms, x0);
            const uint32_t n2 = BuildQueryTrajectory(ms, qt, q, kQueryTrajMax);
            float vsim = 500.0f, dist = 0.0f, tsim = g.dt;
            // The query read times of this test's schemas between the current sample and 0.35 s do not fall inside the braking
            // steps (no time in (dt + 0.1, dt + 0.2)), so this check can only assert the settled 90 cm at 0.35 / 0.70 / 1.0;
            // the 50 / 80 cm prediction samples are asserted on MotionTrajectory directly in PredicatesTest.
            float expect[3] = {};
            const float times[3] = {0.35f, 0.70f, 1.0f};
            for (int k = 0; k < 15; ++k)
            {
                const float next = dist + vsim * 0.1f;
                for (int j = 0; j < 3; ++j)
                    if (times[j] >= tsim && times[j] < tsim + 0.1f)
                        expect[j] = dist + (next - dist) * (times[j] - tsim) / 0.1f; // GetSampleAtTime lerp
                dist = next;
                vsim = std::fmax(vsim - 2000.0f * 0.1f, 0.0f);
                if (vsim < 10.0f)
                    vsim = 0.0f;
                tsim += 0.1f;
            }
            const auto o2 = at(n2, 0.0f);
            const float back = 500.0f * g.dt; // cm from p(0) to the current sample
            bool conv = true;
            for (int j = 0; j < 3; ++j)
            {
                const auto smp = at(n2, times[j]);
                const float along = ((smp.x - o2.x) * -std::sin(psi * kDeg) + (smp.y - o2.y) * std::cos(psi * kDeg)) * 100.0f; // cm
                conv = conv && near(along, back + expect[j], 0.1f);
                std::printf("query stop: along(%.2f) = %.3f cm, braking model %.3f cm\n", static_cast<double>(times[j]),
                            static_cast<double>(along), static_cast<double>(back + expect[j]));
            }
            Check(conv && near(expect[0], 90.0f, 1.0e-3f) && near(expect[2], 90.0f, 1.0e-3f),
                  "stop: query future positions = braking model (settled 90 cm after the current sample at 0.35 / 0.70 / 1.0; "
                  "the 50 / 80 cm braking samples are checked only in PredicatesTest)");
            const auto e1 = at(n2, 1.0f), e0 = at(n2, 1.0f - h);
            Check(near(e1.x - e0.x, 0.0f, 1.0e-5f) && near(e1.y - e0.y, 0.0f, 1.0e-5f), "stop: query velocity at 1.0 = 0 (stopped)");
            (void)cs;
        }
        {
            // (b) strafe while aiming: stick right, camera = actor yaw 40 (motion strafe: bUseControllerDesiredRotation while aiming,
            // CBP l. 1146-1148, RotationRate -1 on the ground = actor rotates to the control yaw at once, l. 1155-1156); facing from
            // the mesh (PSTL l. 21, 113-116), MaxControllerYawRate Moving 0 -> query facing = control yaw at every time while the
            // velocity points right of it (velocity direction != facing).
            MapState ms;
            const float psi = 40.0f;
            MotionFeed g = feed(0.0f, 0.0f);
            g.move[0] = 1.0f;
            g.aiming = true;
            g.cameraYawDeg = psi;
            g.playerYawDeg = psi;
            g.velWorld[0] = std::cos(psi * kDeg) * 3.5f; // right(psi) = (cos, sin)
            g.velWorld[1] = std::sin(psi * kDeg) * 3.5f;
            for (int i = 0; i < 60; ++i)
                MapToChooser(g, c, ms, x0);
            const uint32_t n = BuildQueryTrajectory(ms, qt, q, kQueryTrajMax);
            bool yaw = n == qt.n;
            for (uint32_t i = 0; i < n; ++i)
                yaw = yaw && near(utpp::mm::math::WrapAngle(q[i].yaw - psi * kDeg), 0.0f, kYawTol);
            Check(yaw, "strafe while aiming: query facing = control yaw 40 at every time");
            const auto a = at(n, 0.35f), o = at(n, 0.0f);
            const float dir = std::atan2(a.y - o.y, a.x - o.x) / kDeg; // atan2 convention: right(40) = 40
            Check(near(dir, psi, 0.5f), "strafe while aiming: query movement direction = right of the control yaw");
            std::printf("query strafe: facing %.3f deg, movement %.3f deg (atan2)\n", static_cast<double>(q[0].yaw / kDeg),
                        static_cast<double>(dir));
        }
    }

    // ---------------------------------------------------------------- MM query parity vs baked DFEA (finding 8)
    // Database next to --chooser (*.chooser.json -> *.utppmm). One recorded row of a forward Run_Loops clip
    // (M_Neutral_Run_Loop_F in PSD_Dense_Stand_Run_Loops, middle row of its entry). The clip root path T(h) on the half-frame
    // grid of FORMAT.txt "Feature values" (ROOT steps in the frame of THIS frame, loops wrap with period N-1, odd half frames
    // = averaged x, y, yaw) gives the clip speed v_c = |T(+1.0 s) - T(0)|. A synthetic feed holds the stick so the motion
    // prediction cap equals v_c (speedScale 1, AnalogInputModifier = |stick| as in the CMC, Run gait) with the capsule at
    // v_c along game yaw 0 (OrientToMovement) -> MotionTrajectory is a constant-velocity line, BuildQueryTrajectory -> query.
    // Checks, all on the raw (un-normalized) trajectory dims of the row's schema, DFEA raw = stored / scale + mean:
    //  (P1) bake parity: the same features computed by ChooserMatcher::BuildQuery on the clip root path = DFEA raw within
    //       eps. Limit: a straight constant-velocity row with constant heading cannot detect a time-offset error
    //       (p(t) - p(0) = v t for any shift), world- vs character-space velocity or facing-rotation errors (all heading dims
    //       are (0, 1), x 0); it checks units (cm -> m), the time grid and the normalization on a real row only. Facing vs
    //       velocity is separated by the strafe query check (b) above;
    //  (P2) query = the constant-velocity model features (x 0, y v_c t, yaw 0) within eps;
    //  (P3) |query - DFEA raw| <= |clip - constant-velocity model| + 2 eps per dim: the tolerance is the clip's own deviation
    //       from a steady straight run (step sway, speed ripple) measured from ROOT, not a free number.
    // [P] eps: 1 mm (positions, m), 1 cm/s (velocities, m/s), 1e-3 (unitless directions / headings): float32 rounding of DFEA
    // (normalized float32) and of positions integrated in cm over 90 frames; it is >= 50x below every compared feature scale.
    if (haveTable)
    {
        namespace motionns = utpp::mm::motion;
        constexpr std::string_view kSuffix = ".chooser.json";
        const bool okName = chooserPath.size() > kSuffix.size() &&
                            chooserPath.compare(chooserPath.size() - kSuffix.size(), kSuffix.size(), kSuffix) == 0;
        Check(okName, "parity: --chooser ends with .chooser.json");
        const std::string dbPath = okName ? chooserPath.substr(0, chooserPath.size() - kSuffix.size()) + ".utppmm" : std::string();
        motionns::MotionDatabase pdb;
        std::string perr;
        const bool man = dbPath.find("_man") != std::string::npos;
        const bool loaded = okName && pdb.LoadFromFile(dbPath.c_str(), man ? motionns::kRigIdentityMan : motionns::kRigIdentityWoman, perr);
        Check(loaded, "parity: database next to the chooser loads");
        if (!loaded)
            std::printf("parity db %s: %s\n", dbPath.c_str(), perr.c_str());
        int32_t pd = -1, pe = -1;
        for (uint32_t d = 0; loaded && d < pdb.NumDatabases(); ++d)
            if (pdb.DbName(d) == "PSD_Dense_Stand_Run_Loops")
            {
                pd = static_cast<int32_t>(d);
                const motionns::DatabaseRecord& D = pdb.Db(d);
                // the motion forward run loop, middle row, checked to run straight forward (heading of the 1 s displacement
                // within 0.5 deg of the root forward +Y); the non-package forward loops drift 3-8 deg and would only widen P3
                for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries && pe < 0; ++e)
                {
                    const motionns::EntryRecord& en = pdb.Entry(e);
                    const motionns::MotionClipRecord& cr = pdb.Clip(en.clip);
                    if (en.numRows == 0 || !(cr.flags & motionns::kClipLoop) || cr.loopPeriod == 0)
                        continue;
                    const uint32_t j = pdb.Row(en.firstRow + en.numRows / 2).frame - cr.firstFrame;
                    float px = 0.0f, py = 0.0f, pyaw = 0.0f;
                    for (uint32_t k = 0; k < 30; ++k)
                    {
                        const auto& sd = pdb.Root(cr.firstFrame + (j + k) % cr.loopPeriod);
                        px += std::cos(pyaw) * sd.dx - std::sin(pyaw) * sd.dy;
                        py += std::sin(pyaw) * sd.dx + std::cos(pyaw) * sd.dy;
                        pyaw += sd.dyaw;
                    }
                    const float sp = std::sqrt(px * px + py * py), hd = std::atan2(-px, py) * 57.29577951f;
                    if (pdb.ClipName(en.clip) == "M_Neutral_Run_Loop_F" && std::fabs(hd) <= 0.5f)
                    {
                        pe = static_cast<int32_t>(e);
                        std::printf("parity entry %s: %.3f m/s heading %.3f deg over 1 s\n", std::string(pdb.ClipName(en.clip)).c_str(),
                                    static_cast<double>(sp), static_cast<double>(hd));
                    }
                }
            }
        Check(pd >= 0 && pe >= 0, "parity: PSD_Dense_Stand_Run_Loops holds M_Neutral_Run_Loop_F running straight forward");
        motionns::ChooserMatcher mm;
        std::string merr;
        const bool mmOk = pd >= 0 && pe >= 0 && mm.Init(&pdb, &table, motionns::MatcherConfig{}, merr);
        Check(mmOk, "parity: matcher init");
        if (mmOk)
        {
            const uint32_t d = static_cast<uint32_t>(pd);
            const motionns::DatabaseRecord& D = pdb.Db(d);
            const motionns::EntryRecord& E = pdb.Entry(static_cast<uint32_t>(pe));
            const motionns::MotionClipRecord& C = pdb.Clip(E.clip);
            const uint32_t row = E.firstRow + E.numRows / 2;
            const uint32_t F = pdb.Row(row).frame;
            const uint32_t P = C.loopPeriod;
            Check((C.flags & motionns::kClipLoop) && P > 0, "parity: the clip is a loop");
            const QueryTimes qtimes = BuildQueryTimes(pdb);
            Check(qtimes.n > 0 && !qtimes.overflow, "parity: query times");
            std::printf("parity query times (%u):", qtimes.n);
            for (uint32_t i = 0; i < qtimes.n; ++i)
                std::printf(" %.4f", static_cast<double>(qtimes.t[i]));
            std::printf("\n");
            // clip root on integer frames k = -kB .. kA relative to F (k = 0 identity)
            constexpr int kB = 24, kA = 64; // kB 8 -> 24: the slide exit schema samples -0.75 s
            float rx[kB + kA + 1] = {}, ry[kB + kA + 1] = {}, ryaw[kB + kA + 1] = {};
            const int j0 = static_cast<int>(F - C.firstFrame);
            auto step = [&](int k) -> const auto& {
                const int mi = ((j0 + k) % static_cast<int>(P) + static_cast<int>(P)) % static_cast<int>(P);
                return pdb.Root(C.firstFrame + static_cast<uint32_t>(mi));
            };
            for (int k = 0; k < kA; ++k) // T(k+1) = T(k) o step(k)
            {
                const auto& sd = step(k);
                const float cy = std::cos(ryaw[kB + k]), sy = std::sin(ryaw[kB + k]);
                rx[kB + k + 1] = rx[kB + k] + cy * sd.dx - sy * sd.dy;
                ry[kB + k + 1] = ry[kB + k] + sy * sd.dx + cy * sd.dy;
                ryaw[kB + k + 1] = ryaw[kB + k] + sd.dyaw;
            }
            for (int k = 0; k > -kB; --k) // T(k-1) = T(k) o step(k-1)^-1
            {
                const auto& sd = step(k - 1);
                const float yw = ryaw[kB + k] - sd.dyaw;
                ryaw[kB + k - 1] = yw;
                rx[kB + k - 1] = rx[kB + k] - (std::cos(yw) * sd.dx - std::sin(yw) * sd.dy);
                ry[kB + k - 1] = ry[kB + k] - (std::sin(yw) * sd.dx + std::cos(yw) * sd.dy);
            }
            auto clipAt = [&](float t) { // half-frame grid (t multiple of 1/60 s); odd half frames averaged
                const int h = static_cast<int>(std::lround(t * 60.0f));
                const int a = (h >= 0 ? h : h - 1) / 2; // floor(h / 2)
                const int b = (h % 2 == 0) ? a : a + 1;
                motionns::TrajSample smp;
                smp.t = t;
                smp.x = 0.5f * (rx[kB + a] + rx[kB + b]);
                smp.y = 0.5f * (ry[kB + a] + ry[kB + b]);
                smp.yaw = 0.5f * (ryaw[kB + a] + ryaw[kB + b]);
                return smp;
            };
            const float tMin = qtimes.t[0], tMax = qtimes.t[qtimes.n - 1];
            const bool inWin = tMin * 60.0f >= -2.0f * kB + 2.0f && tMax * 60.0f <= 2.0f * kA - 2.0f;
            Check(inWin, "parity: query times inside the integrated clip window");
            const motionns::TrajSample c1 = clipAt(1.0f);
            const float vc = std::sqrt(c1.x * c1.x + c1.y * c1.y); // m/s over [0, 1.0]
            std::printf("parity row: db %u, clip %s, frame %u (local %d of %u), v_c %.4f m/s, clip heading drift over 1 s %.3f deg\n",
                        d, std::string(pdb.ClipName(E.clip)).c_str(), F, j0, C.numFrames, static_cast<double>(vc),
                        static_cast<double>(std::atan2(-c1.x, c1.y) * 57.29577951f));
            // prediction cap = forward gait speed (CBP l. 10-13: Run 500, Sprint 700 cm/s) x AnalogInputModifier |stick| (UE
            // PSTL l. 73): Run when v_c / 5 m/s is in [AnalogWalk/RunThreshold, 1], else Sprint (PSM Sprint, CanSprint with
            // OrientToMovement) with stick v_c / 7 m/s. M_Neutral_Run_Loop_F runs above the Run cap -> Sprint.
            const bool runOk = vc / 5.0f >= c.analogWalkRunThreshold && vc <= 5.0f;
            const float stick = runOk ? vc / 5.0f : vc / 7.0f;
            const bool reach = stick >= c.analogWalkRunThreshold && stick <= 1.0f;
            Check(inWin && reach, "parity: clip speed reachable as a Neutral prediction cap (Run / Sprint x |stick|)");
            if (inWin && reach)
            {
                MapConfig pc = c;
                pc.speedScale = 1.0f;          // game m/s x 100 = motion cm/s (no piecewise map)
                pc.gameAnalogModifier = false; // CMC AnalogInputModifier = |stick| (UE l. 73)
                pc.stickMode = StickMode::VariableSpeedWalkRun;
                MapState ms;
                MotionFeed g = feed(vc, stick);
                if (!runOk)
                    g.psmDetailed = kPsmSprint;
                g.cameraYawDeg = 0.0f;
                g.playerYawDeg = 0.0f;
                utpp::mm::motion::ChooserState pcs;
                for (int i = 0; i < 90; ++i)
                    pcs = MapToChooser(g, pc, ms, x0);
                std::printf("parity feed: %.4f m/s, stick %.4f, gait %d, moving %d, starting %d, speed2d %.2f cm/s\n",
                            static_cast<double>(vc), static_cast<double>(stick), pcs.gait, pcs.movementState, pcs.isStarting ? 1 : 0,
                            static_cast<double>(pcs.speed2dCmS));
                Check(pcs.gait == (runOk ? 1 : 2) && pcs.movementState == 1 && !pcs.isStarting, "parity: steady feed (gait, Moving, no IsStarting)");
                motionns::TrajSample qs[kQueryTrajMax], cl[kQueryTrajMax], id[kQueryTrajMax];
                const uint32_t n = BuildQueryTrajectory(ms, qtimes, qs, kQueryTrajMax);
                Check(n == qtimes.n, "parity: query trajectory built");
                for (uint32_t i = 0; i < qtimes.n; ++i)
                {
                    cl[i] = clipAt(qtimes.t[i]);
                    id[i] = motionns::TrajSample{qtimes.t[i], 0.0f, vc * qtimes.t[i], 0.0f, 0.0f};
                }
                const motionns::SchemaRecord& S = pdb.Schema(D.schema);
                std::vector<float> fq(S.dims), fc(S.dims), fi(S.dims);
                auto raw = [&](const motionns::TrajSample* a, std::vector<float>& out) {
                    mm.SetQueryInputs(a, qtimes.n, nullptr);
                    mm.BuildQuery(d, false);
                    for (uint32_t k = 0; k < S.dims; ++k)
                        out[k] = mm.RawQuery()[k];
                };
                raw(qs, fq);
                raw(cl, fc);
                raw(id, fi);
                mm.SetQueryInputs(nullptr, 0, nullptr);
                const float* stored = pdb.Features(d, row - D.firstRow);
                const float* mean = pdb.NormMean(D.norm);
                const float* scale = pdb.NormScale(D.norm);
                bool p1 = true, p2 = true, p3 = true;
                uint32_t dims = 0;
                float worst = 0.0f;
                for (uint32_t ci = S.firstChannel; ci < S.firstChannel + S.numChannels; ++ci)
                {
                    const motionns::ChannelRecord& chn = pdb.Channel(ci);
                    if (!(chn.flags & motionns::kChanTrajectory))
                        continue;
                    const float eps = chn.kind == 1 ? 1.0e-3f : chn.kind == 2 ? 1.0e-2f : 1.0e-3f;
                    for (uint32_t k = 0; k < chn.cardinality; ++k)
                    {
                        const uint32_t o = chn.dataOffset + k;
                        const float fd = stored[o] / scale[o] + mean[o];
                        const float tol = std::fabs(fc[o] - fi[o]) + 2.0f * eps;
                        const bool a1 = std::fabs(fc[o] - fd) <= eps, a2 = std::fabs(fq[o] - fi[o]) <= eps,
                                   a3 = std::fabs(fq[o] - fd) <= tol;
                        p1 = p1 && a1;
                        p2 = p2 && a2;
                        p3 = p3 && a3;
                        worst = std::fmax(worst, std::fabs(fq[o] - fd));
                        ++dims;
                        std::printf("parity %-28s kind %u ts %+.3f to %+.3f [%u]: query %+.5f dfea %+.5f clip %+.5f model %+.5f "
                                    "|q-d| %.5f tol %.5f%s\n",
                                    std::string(pdb.String(chn.labelOff)).c_str(), chn.kind, static_cast<double>(chn.tSample),
                                    static_cast<double>(chn.tOrigin), k, static_cast<double>(fq[o]), static_cast<double>(fd),
                                    static_cast<double>(fc[o]), static_cast<double>(fi[o]), static_cast<double>(std::fabs(fq[o] - fd)),
                                    static_cast<double>(tol), (a1 && a2 && a3) ? "" : " <-");
                    }
                }
                Check(dims > 0, "parity: schema has trajectory dims");
                Check(p1, "parity P1: ChooserMatcher features of the clip root path = baked DFEA (eps)");
                Check(p2, "parity P2: query features = constant-velocity model (eps)");
                Check(p3, "parity P3: |query - DFEA| <= clip deviation from the constant-velocity model + 2 eps, every trajectory dim");
                std::printf("parity: %u trajectory dims, max |query - dfea| %.5f\n", dims, static_cast<double>(worst));
            }
        }
    }
    else
        std::printf("query parity check NOT RUN (no --chooser <file>)\n");

    // ---------------------------------------------------------------- motionfix item 1: Locomotion input direction on real frames
    // In-game v2 test 2026-10-13 (MotionFixRealFrames.hpp): under Locomotion the game sends mv (0, 1) and the run direction only in
    // haveTarget / targetYawDeg. Steady sideways run (target - camera 90 deg), run toward the camera (180 deg) and crouch walk
    // toward the camera: after a 30-frame warm-up on the same frames, every frame must be Moving with no IsPivoting /
    // IsStarting, the chooser set must hold the gait's *_Loops and no *_Pivots, and the predicted facing (OrientToMovement:
    // acceleration yaw) and the future velocity must point along the capsule velocity. The same frames WITHOUT the target
    // (pre-fix input: mv rotated by the camera yaw) must show the reversal (IsPivoting / facing off by > 90 deg): the case
    // discriminates.
    {
        struct Case
        {
            const char* name;
            const motionfix::RealFrame* fr;
            int n;
            const char* loops; // chooser set suffix of the steady gait
        };
        const Case cases[] = {{"side run", motionfix::kSideRun, 90, "_Run_Loops"},
                              {"toward-camera run", motionfix::kTowardRun, 90, "_Run_Loops"},
                              {"crouch walk toward camera", motionfix::kCrouchToward, 90, "_Loops"}};
        auto hasSub = [](const Set& aSet, std::string_view aSub) {
            for (const std::string& n : aSet)
                if (n.find(aSub) != std::string::npos)
                    return true;
            return false;
        };
        for (const Case& k : cases)
        {
            for (int withTarget = 1; withTarget >= 0; --withTarget)
            {
                MapState ms;
                const MapContext xr;
                int checked = 0, moving = 0, pivoting = 0, starting = 0, loopsSet = 0, pivotsSet = 0;
                float worstFacing = 0.0f, worstFuture = 0.0f;
                Set lastSet;
                for (int i = 0; i < k.n; ++i)
                {
                    const motionfix::RealFrame& rf = k.fr[i];
                    const float vel[3] = {rf.vx, rf.vy, rf.vz};
                    const float mv[2] = {rf.mvx, rf.mvy};
                    MotionFeed g = MakeMotionFeed(vel, mv, rf.dt, rf.psm, rf.loco, AirFeed{}, rf.camDeg, rf.yawDeg);
                    if (withTarget)
                    {
                        g.haveTarget = rf.haveTarget;
                        g.targetYawDeg = rf.targetYawDeg;
                    }
                    const utpp::mm::motion::ChooserState cs = MapToChooser(g, c, ms, xr);
                    if (i < 30)
                        continue;
                    ++checked;
                    moving += cs.movementState == 1 ? 1 : 0;
                    pivoting += cs.isPivoting ? 1 : 0;
                    starting += cs.isStarting ? 1 : 0;
                    const float velYaw = t2::YawOf(ms.st.velocity);
                    const float face = ms.traj.Prediction(t2::kPredictionCount - 1).facing;
                    worstFacing = std::fmax(worstFacing, std::fabs(t2::NormalizeAxis(face - velYaw)));
                    worstFuture = std::fmax(worstFuture, std::fabs(t2::NormalizeAxis(t2::YawOf(ms.st.trjFuture) - velYaw)));
                    if (haveTable)
                    {
                        lastSet = ChooserSet(table, cs);
                        loopsSet += HasSuffix(lastSet, k.loops) ? 1 : 0;
                        pivotsSet += hasSub(lastSet, "Pivots") ? 1 : 0;
                    }
                }
                std::printf("motionfix %s %s: %d frames, moving %d, pivoting %d, starting %d, |facing - velocity yaw| max %.1f, "
                            "|future - velocity yaw| max %.1f deg, sets with %s %d / Pivots %d (last %s)\n",
                            k.name, withTarget ? "target" : "no-target(pre-fix)", checked, moving, pivoting, starting,
                            static_cast<double>(worstFacing), static_cast<double>(worstFuture), k.loops, loopsSet, pivotsSet,
                            Join(lastSet).c_str());
                std::string what = std::string("motionfix ") + k.name;
                if (withTarget)
                {
                    Check(checked == 60 && moving == 60 && pivoting == 0 && starting == 0,
                          (what + ": Moving, no IsPivoting / IsStarting on the 60 checked frames").c_str());
                    Check(worstFacing < 30.0f && worstFuture < 30.0f,
                          (what + ": predicted facing and future velocity within 30 deg of the capsule velocity").c_str());
                    if (haveTable)
                        Check(loopsSet == 60 && pivotsSet == 0, (what + ": chooser set holds *_Loops, never *_Pivots").c_str());
                }
                else
                    Check(pivoting > 0 || worstFacing > 90.0f,
                          (what + ": without the target (pre-fix) the camera-rotated mv reverses the input (discrimination)").c_str());
            }
        }
        // InputWorld: target wins over move (magnitude kept), no target -> StickToWorld
        MotionFeed h;
        h.move[1] = 0.5f;
        h.cameraYawDeg = -82.2186432f;
        h.haveTarget = true;
        h.targetYawDeg = 97.7813568f;
        float wx = 0.0f, wy = 0.0f;
        InputWorld(h, wx, wy);
        Check(std::fabs(std::hypot(wx, wy) - 0.5f) < 1e-5f && std::fabs(std::atan2(-wx, wy) / 0.0174532925f - 97.7813568f) < 1e-3f,
              "InputWorld: target yaw, |mv| magnitude");
        h.haveTarget = false;
        InputWorld(h, wx, wy);
        float sx = 0.0f, sy = 0.0f;
        StickToWorld(0.0f, 0.5f, -82.2186432f, sx, sy);
        Check(wx == sx && wy == sy, "InputWorld: no target = StickToWorld");
    }

    // ---------------------------------------------------------------- motionfix item 5: standing jump air picks (real frames)
    // In-game v2 test 2026-10-13 (MotionFixAirFrames.hpp): two standing jumps (capsule Speed2D 0.00-0.04 m/s, PSM Jump 18, 0.72 s
    // flight). Real path: MakeMotionFeed with the dump's air fields (AirFeed valid, onGround, distGround -2 = not probed, psmFall)
    // -> MapToChooser -> BuildQueryTrajectory -> ChooserMatcher on the woman database.
    //  (a) default MatcherConfig (airFallBias 0 = off, measured): after the apex (InAir, Vz <= 0, Speed2D < 0.5 m/s) every
    //      update before touchdown plays the motion stand takeoff M_Neutral_Jump_F_Start_Stand_L/Rfoot (motion has no in-place
    //      jump clip; PSD_Dense_Jumps entries 4 / 5), never the walk / run / off-ledge variants, never Loop_Fall;
    //  (b) the bias mechanism with an explicit addend (-1.0 [P], 2x the PSD_Dense_Jumps continuing_pose_cost_bias -0.5): applied
    //      exactly on the post-apex updates and reaches M_Neutral_Jump_Loop_Fall (the measurement that kept it off: Loop_Fall's
    //      root falls at a constant 9.8 m/s, so it only wins near touchdown and alternates with the stand clip).
    if (haveTable)
    {
        namespace motionns = utpp::mm::motion;
        constexpr std::string_view kSuffix = ".chooser.json";
        const std::string dbPath = chooserPath.substr(0, chooserPath.size() - kSuffix.size()) + ".utppmm";
        motionns::MotionDatabase jdb;
        std::string jerr;
        const bool man = dbPath.find("_man") != std::string::npos;
        const bool loaded = jdb.LoadFromFile(dbPath.c_str(), man ? motionns::kRigIdentityMan : motionns::kRigIdentityWoman, jerr);
        Check(loaded, "motionfix air: database loads");
        const QueryTimes qtimes = loaded ? BuildQueryTimes(jdb) : QueryTimes{};
        struct JCase
        {
            const char* name;
            const motionfixair::AirFrame* fr;
            int n;
        };
        const JCase jcases[] = {{"standing jump A (t 163.65)", motionfixair::kStandJumpA, motionfixair::kStandJumpAN},
                                {"standing jump B (t 1371.56)", motionfixair::kStandJumpB, motionfixair::kStandJumpBN}};
        auto nameOf = [&](int32_t aClip) { return aClip >= 0 ? std::string(jdb.ClipName(static_cast<uint32_t>(aClip))) : std::string("-"); };
        for (const JCase& k : jcases)
        {
            for (int on = 0; loaded && on <= 1; ++on)
            {
                motionns::MatcherConfig mcfg;
                if (on)
                    mcfg.airFallBias = -1.0f;
                motionns::ChooserMatcher mm;
                std::string merr;
                const bool ok = mm.Init(&jdb, &table, mcfg, merr);
                Check(ok, "motionfix air: matcher init");
                if (!ok)
                    break;
                MapState ms;
                const MapContext xr;
                const MapConfig mc;
                int post = 0, postAir = 0, stand = 0, fall = 0, biased = 0, inAir = 0, flips = 0;
                std::string seq, lastName;
                for (int i = 0; i < k.n; ++i)
                {
                    const motionfixair::AirFrame& rf = k.fr[i];
                    const float vel[3] = {rf.vx, rf.vy, rf.vz};
                    const float mv[2] = {rf.mvx, rf.mvy};
                    AirFeed af;
                    af.valid = true;
                    af.onGround = rf.onGround;
                    af.distanceToGround = rf.distGround;
                    af.psmFall = rf.psmFall;
                    MotionFeed g = MakeMotionFeed(vel, mv, rf.dt, rf.psm, rf.loco, af, rf.camDeg, rf.yawDeg);
                    g.haveTarget = rf.haveTarget;
                    g.targetYawDeg = rf.targetYawDeg;
                    const motionns::ChooserState cs = MapToChooser(g, mc, ms, xr);
                    motionns::TrajSample q[kQueryTrajMax];
                    const uint32_t nt = BuildQueryTrajectory(ms, qtimes, q, kQueryTrajMax);
                    if (nt == 0)
                        continue;
                    const motionns::MatchResult& mr = mm.Update(rf.dt, q, nt, cs);
                    const std::string nm = nameOf(mr.clip);
                    inAir += cs.movementMode == 1 ? 1 : 0;
                    biased += mr.airFallBias ? 1 : 0;
                    if (cs.movementMode == 1 && cs.verticalVelocityCmS <= 0.0f && cs.speed2dCmS < motionns::kAirFallMaxSpeedCmS)
                    {
                        ++post;
                        // still airborne in the game (the touchdown update itself may already pick a Lands clip)
                        if (!rf.onGround)
                        {
                            ++postAir;
                            stand += nm.find("M_Neutral_Jump_F_Start_Stand_") == 0 ? 1 : 0;
                            fall += nm == "M_Neutral_Jump_Loop_Fall" ? 1 : 0;
                        }
                    }
                    if (nm != lastName)
                    {
                        flips += cs.movementMode == 1 && !lastName.empty() ? 1 : 0;
                        char b[200];
                        std::snprintf(b, sizeof b, "%s%.2f:%s(%s)", seq.empty() ? "" : " ", static_cast<double>(rf.t), nm.c_str(),
                                      mr.database >= 0 ? std::string(jdb.DbName(static_cast<uint32_t>(mr.database))).c_str() : "-");
                        seq += b;
                        lastName = nm;
                    }
                }
                std::printf("motionfix air %s, airFallBias %.2f: inAir %d, post-apex %d (airborne %d), stand-start %d, Loop_Fall %d, "
                            "biased updates %d, clip changes in air %d; picks %s\n",
                            k.name, static_cast<double>(mcfg.airFallBias), inAir, post, postAir, stand, fall, biased, flips, seq.c_str());
                const std::string what = std::string("motionfix air ") + k.name;
                if (!on)
                {
                    Check(post >= 15 && postAir >= 15 && stand == postAir && fall == 0 && biased == 0,
                          (what + ": default: every airborne post-apex update plays the Neutral stand takeoff, no bias").c_str());
                }
                else
                    Check(post >= 15 && biased == post && biased < inAir && fall > 0,
                          (what + ": bias -1.0: applied exactly on the post-apex updates, reaches Loop_Fall").c_str());
            }
        }
    }

    // ---------------------------------------------------------------- motionfix item 7: pose snaps on real frames
    // In-game v2 test 2026-10-13 (MotionFixAirFrames.hpp kSprintStop / kReTakeoff / kWeaponIdle): the real path MakeMotionFeed (air
    // fields) -> MapToChooser -> BuildQueryTrajectory -> ChooserMatcher on the woman database, with the v2 inertializer
    // request / deficit semantics of MotionAdapter::Update / PoseEvaluator (QuinticInertializer::SetMinDuration(kBlendFloorSeconds))
    // and the MotionDriver landing bypass (SetIgnoreDeficitOnNextBlend on the justLanded edge, consumed by the next request).
    //  (a) sprint stop (car, 8.2 m/s -> 1.2 m/s within two feeds): the first jump of the stop is a PSD_Dense_Stand_Sprint_Stops
    //      clip, also when the two collapse feeds (stick release at 7.99 m/s, 5.19 m/s) are missing from the feed (the
    //      chooser then first sees Idle at 1.18 m/s = Speed2D ~150: Walk / Run_Stops only without the collapse hold); every
    //      jump's effective blend >= kBlendFloorSeconds; the span ends in the Idles set with no frozen clip end.
    //  (b) re-takeoff 0.46 s after a landing: the landing jump keeps its full landing blend (bypass), the takeoff jump and
    //      every other jump last >= kBlendFloorSeconds (the deficit of the landing / re-search blends no longer cuts it to
    //      ~1 frame).
    //  (c) standstill after a stop with the revolver drawn: no update is frozen past the end of a non-looping clip (the clip
    //      end hands over by a search: UE CanAdvance), and from 1.5 s after the stop every pick is in PSD_Dense_Stand_Idles.
    //  (d) the MotionAdapter adapter (offline / replay) has the same floor.
    if (haveTable)
    {
        namespace motionns = utpp::mm::motion;
        constexpr std::string_view kSuffix = ".chooser.json";
        const std::string dbPath = chooserPath.substr(0, chooserPath.size() - kSuffix.size()) + ".utppmm";
        motionns::MotionDatabase sdb;
        std::string serr;
        const bool man = dbPath.find("_man") != std::string::npos;
        const bool loaded = sdb.LoadFromFile(dbPath.c_str(), man ? motionns::kRigIdentityMan : motionns::kRigIdentityWoman, serr);
        Check(loaded, "motionfix snap: database loads");
        const QueryTimes qtimes = loaded ? BuildQueryTimes(sdb) : QueryTimes{};
        const float fps = loaded && sdb.Header().fps > 0.0f ? sdb.Header().fps : 30.0f;
        auto nameOf = [&](int32_t aClip) { return aClip >= 0 ? std::string(sdb.ClipName(static_cast<uint32_t>(aClip))) : std::string("-"); };
        auto dbOf = [&](int32_t aDb) { return aDb >= 0 ? std::string(sdb.DbName(static_cast<uint32_t>(aDb))) : std::string("-"); };
        struct Jump
        {
            float t;
            int32_t clip, db;
            float blendTime, duration;
        };
        struct Replay
        {
            int updates = 0, jumps = 0, frozen = 0, idlesAfter = 0, notIdlesAfter = 0;
            float minDuration = 1.0e9f;
            float collapseSpeedCmS = -1.0f; // chooser Speed2D of the first Idle feed at t >= collapseT
            std::vector<Jump> jumpList;
            std::string seq;
            int32_t lastDb = -1;
            float lastT = 0.0f;
        };
        // skipFrom <= t < skipTo: feeds dropped (a missed stick-release feed); landingBypass: the MotionDriver rule;
        // idlesAfterT: picks at t >= it must be in *_Idles.
        auto replay = [&](const motionfixair::AirFrame* fr, int n, float skipFrom, float skipTo, bool landingBypass, float idlesAfterT,
                          Replay& r, float collapseT = 1.0e9f) {
            motionns::ChooserMatcher mm;
            std::string merr;
            if (!mm.Init(&sdb, &table, motionns::MatcherConfig{}, merr))
                return false;
            utpp::mm::QuinticInertializer in;
            if (!in.Init(sdb.NumBones(), 1))
                return false;
            in.SetMinDuration(kBlendFloorSeconds);
            std::vector<utpp::mm::BoneTransform> raw(sdb.NumBones()), prev(sdb.NumBones()), out(sdb.NumBones());
            bool havePrev = false, armed = false, prevLanded = false;
            int heldAtEnd = 0;
            MapState ms;
            const MapContext xr;
            const MapConfig mc;
            for (int i = 0; i < n; ++i)
            {
                const motionfixair::AirFrame& rf = fr[i];
                if (rf.t >= skipFrom && rf.t < skipTo)
                    continue;
                const float vel[3] = {rf.vx, rf.vy, rf.vz};
                const float mv[2] = {rf.mvx, rf.mvy};
                AirFeed af;
                af.valid = true;
                af.onGround = rf.onGround;
                af.distanceToGround = rf.distGround;
                af.psmFall = rf.psmFall;
                MotionFeed g = MakeMotionFeed(vel, mv, rf.dt, rf.psm, rf.loco, af, rf.camDeg, rf.yawDeg);
                g.haveTarget = rf.haveTarget;
                g.targetYawDeg = rf.targetYawDeg;
                const motionns::ChooserState cs = MapToChooser(g, mc, ms, xr);
                if (rf.t >= collapseT && r.collapseSpeedCmS < 0.0f && cs.movementMode == 0 && cs.movementState == 0)
                    r.collapseSpeedCmS = cs.speed2dCmS;
                motionns::TrajSample q[kQueryTrajMax];
                const uint32_t nt = BuildQueryTrajectory(ms, qtimes, q, kQueryTrajMax);
                if (nt == 0)
                    continue;
                const motionns::MatchResult& mr = mm.Update(rf.dt, q, nt, cs);
                if (mr.clip < 0 || !SamplePose(sdb, mr.clip, mr.clipTime, raw.data(), static_cast<uint32_t>(raw.size())))
                    continue;
                ++r.updates;
                const bool landed = cs.justLandedLight || cs.justLandedHeavy;
                if (landingBypass)
                {
                    if (landed && !prevLanded)
                        armed = true;
                    else if (!landed)
                        armed = false;
                    prevLanded = landed;
                }
                if (mr.blend.valid && mr.blend.blendTime > 0.0f)
                {
                    in.Request(mr.blend.blendTime, armed);
                    armed = false;
                }
                const bool hadPrev = havePrev; // the first pick has no source pose: nothing to blend from
                if (!in.Update(raw.data(), (havePrev && !mr.jumped) ? prev.data() : nullptr, rf.dt, out.data()))
                    out = raw;
                prev = raw;
                havePrev = true;
                const motionns::MotionClipRecord& cl = sdb.Clip(static_cast<uint32_t>(mr.clip));
                const float len = static_cast<float>(cl.numFrames - 1) / fps;
                if (!(cl.flags & motionns::kClipLoop) && mr.clipTime > len + 1e-3f)
                    ++r.frozen; // past the end of a non-looping clip: SamplePose clamps = frozen pose
                // pinned on the last frame for 2 updates in a row (clip time stuck at the end, no hand-over search)
                if (!mr.jumped && !(cl.flags & motionns::kClipLoop) && mr.clipTime >= len - 1e-3f)
                {
                    if (++heldAtEnd >= 2)
                        ++r.frozen;
                }
                else
                    heldAtEnd = 0;
                // UE: a non-looping Stop clip plays to its end (DoNotInterrupt continuing pose), then the search hands over
                if (rf.t >= idlesAfterT)
                {
                    const bool stopPlaying = !mr.jumped && dbOf(mr.database).ends_with("_Stops") && !(cl.flags & motionns::kClipLoop) &&
                                             mr.clipTime <= len + 1e-3f;
                    if (dbOf(mr.database).ends_with("_Idles"))
                        ++r.idlesAfter;
                    else if (!stopPlaying)
                        ++r.notIdlesAfter;
                }
                if (mr.jumped)
                {
                    ++r.jumps;
                    if (hadPrev)
                        r.minDuration = std::fmin(r.minDuration, in.Duration());
                    r.jumpList.push_back({rf.t, mr.clip, mr.database, mr.blend.blendTime, in.Duration()});
                    char b[256];
                    std::snprintf(b, sizeof b, "%s%.3f:%s@%.2f c%.3f/%.3f(%s)b%.2f/%.3f len%.2f", r.seq.empty() ? "" : " ", static_cast<double>(rf.t), nameOf(mr.clip).c_str(), static_cast<double>(mr.clipTime), static_cast<double>(mr.cost), static_cast<double>(mr.continuingCost),
                                  dbOf(mr.database).c_str(), static_cast<double>(mr.blend.blendTime), static_cast<double>(in.Duration()), static_cast<double>(len));
                    r.seq += b;
                }
                r.lastDb = mr.database;
                r.lastT = rf.t;
            }
            return r.updates > 0;
        };
        auto firstJumpAfter = [](const Replay& r, float aT) -> const Jump* {
            for (const Jump& j : r.jumpList)
                if (j.t >= aT)
                    return &j;
            return nullptr;
        };
        const float floorEps = kBlendFloorSeconds - 1e-4f;
        // (a) sprint stop: full feed and with the two collapse feeds missing
        for (int variant = 0; loaded && variant <= 1; ++variant)
        {
            Replay rp;
            const bool ok = replay(motionfixair::kSprintStop, motionfixair::kSprintStopN, variant ? 1503.52f : 0.0f, variant ? 1503.56f : 0.0f, false,
                                   1.0e9f, rp, 1503.52f);
            const Jump* stop = firstJumpAfter(rp, 1503.5f);
            std::printf("motionfix snap sprint stop%s: updates %d, jumps %d, min effective blend %.3f s, frozen %d, first stop jump %s, "
                        "first Idle chooser Speed2D %.1f cm/s; jumps %s\n",
                        variant ? " (collapse feeds missing)" : "", rp.updates, rp.jumps, static_cast<double>(rp.minDuration), rp.frozen,
                        stop ? dbOf(stop->db).c_str() : "-", static_cast<double>(rp.collapseSpeedCmS), rp.seq.c_str());
            const std::string what = std::string("motionfix snap sprint stop") + (variant ? " (collapse feeds missing)" : "");
            // CHT Stand Idles: Speed2D [550, 999] adds PSD_Dense_Stand_Sprint_Stops to the searched set (the cost picks among the sets)
            Check(ok && rp.collapseSpeedCmS >= 550.0f, (what + ": the first Idle feed after the collapse searches Sprint_Stops (Speed2D >= 550)").c_str());
            Check(ok && stop && stop->t < 1503.6f && dbOf(stop->db).ends_with("_Stops"), (what + ": the stop jumps into a Stops clip").c_str());
            Check(ok && rp.jumps >= 2 && rp.minDuration >= floorEps, (what + ": every jump blends >= kBlendFloorSeconds").c_str());
            Check(ok && rp.frozen == 0, (what + ": no update frozen past a non-looping clip end").c_str());
        }
        // (b) re-takeoff after a landing (landing bypass as in MotionDriver)
        if (loaded)
        {
            Replay rp;
            const bool ok = replay(motionfixair::kReTakeoff, motionfixair::kReTakeoffN, 0.0f, 0.0f, true, 1.0e9f, rp);
            const Jump* land = firstJumpAfter(rp, 1499.66f);
            const Jump* take = firstJumpAfter(rp, 1500.13f);
            std::printf("motionfix snap re-takeoff: updates %d, jumps %d, min effective blend %.3f s, landing jump %s b%.2f/%.3f, takeoff jump %s b%.2f/%.3f; "
                        "jumps %s\n",
                        rp.updates, rp.jumps, static_cast<double>(rp.minDuration), land ? dbOf(land->db).c_str() : "-",
                        land ? static_cast<double>(land->blendTime) : 0.0, land ? static_cast<double>(land->duration) : 0.0,
                        take ? dbOf(take->db).c_str() : "-", take ? static_cast<double>(take->blendTime) : 0.0,
                        take ? static_cast<double>(take->duration) : 0.0, rp.seq.c_str());
            Check(ok && land && land->t < 1499.75f && land->duration >= land->blendTime - 1e-4f && land->blendTime >= 0.2f - 1e-4f,
                  "motionfix snap re-takeoff: the landing jump keeps its full landing blend (bypass)");
            Check(ok && take && take->t < 1500.2f && dbOf(take->db).find("Jumps") != std::string::npos && take->duration >= floorEps,
                  "motionfix snap re-takeoff: the takeoff jump blends >= kBlendFloorSeconds");
            Check(ok && rp.jumps >= 3 && rp.minDuration >= floorEps, "motionfix snap re-takeoff: every jump blends >= kBlendFloorSeconds");
        }
        // (c) standstill with the revolver drawn: the stop clip end hands over to the Idles set
        if (loaded)
        {
            Replay rp;
            const bool ok = replay(motionfixair::kWeaponIdle, motionfixair::kWeaponIdleN, 0.0f, 0.0f, false, 74.7f, rp);
            std::printf("motionfix snap weapon idle: updates %d, jumps %d, min effective blend %.3f s, frozen %d, idles after 74.7 %d / not %d, last db %s; "
                        "jumps %s\n",
                        rp.updates, rp.jumps, static_cast<double>(rp.minDuration), rp.frozen, rp.idlesAfter, rp.notIdlesAfter, dbOf(rp.lastDb).c_str(),
                        rp.seq.c_str());
            Check(ok && rp.frozen == 0, "motionfix snap weapon idle: no update frozen past a non-looping clip end");
            Check(ok && rp.notIdlesAfter == 0 && rp.idlesAfter >= 100 && dbOf(rp.lastDb) == "PSD_Dense_Stand_Idles",
                  "motionfix snap weapon idle: from 1.5 s after the stop only the running Stop clip (to its end) or PSD_Dense_Stand_Idles");
            Check(ok && rp.jumps >= 2 && rp.minDuration >= floorEps, "motionfix snap weapon idle: every jump blends >= kBlendFloorSeconds");
        }
        // Mutation 4 closure: force the *actual* matcher across the end of a selected real non-looping
        // Stops clip. Ordinary 60 Hz replay can switch to Idles early and never exercise CanAdvance=false.
        // Re-create a valid query from the recorded stop frame, then advance by its remaining lifetime + 1 frame.
        // Production must search and hand over; mutant 4 returns its previous frozen Last without searching.
        if (loaded)
        {
            motionns::ChooserMatcher mm;
            std::string err;
            const bool initialized = mm.Init(&sdb, &table, motionns::MatcherConfig{}, err);
            MapState ms;
            const MapContext ctx;
            const MapConfig cfg;
            bool selectedStop = false, crossed = false, searchedAfterEnd = false, unfrozen = false, forced = false;
            int32_t stopClip = -1, afterClip = -1;
            float stopTime = -1.0f, stopLength = -1.0f;
            if (initialized)
                for (int i = 0; i < motionfixair::kSprintStopN; ++i)
                {
                    const auto& rf = motionfixair::kSprintStop[i];
                    const float vel[3] = {rf.vx, rf.vy, rf.vz};
                    const float move[2] = {rf.mvx, rf.mvy};
                    AirFeed af;
                    af.valid = true;
                    af.onGround = rf.onGround;
                    af.distanceToGround = rf.distGround;
                    af.psmFall = rf.psmFall;
                    MotionFeed stopFeed = MakeMotionFeed(vel, move, rf.dt, rf.psm, rf.loco, af, rf.camDeg, rf.yawDeg);
                    stopFeed.haveTarget = rf.haveTarget;
                    stopFeed.targetYawDeg = rf.targetYawDeg;
                    const motionns::ChooserState state = MapToChooser(stopFeed, cfg, ms, ctx);
                    motionns::TrajSample q[kQueryTrajMax];
                    const uint32_t count = BuildQueryTrajectory(ms, qtimes, q, kQueryTrajMax);
                    if (!count)
                        continue;
                    const motionns::MatchResult before = mm.Update(rf.dt, q, count, state);
                    if (before.clip < 0 || !dbOf(before.database).ends_with("_Stops"))
                        continue;
                    const auto& clip = sdb.Clip(static_cast<uint32_t>(before.clip));
                    if ((clip.flags & motionns::kClipLoop) != 0 || clip.numFrames < 2)
                        continue;
                    selectedStop = true;
                    stopClip = before.clip;
                    stopTime = before.clipTime;
                    stopLength = float(clip.numFrames - 1) / fps;
                    // the matcher advances the clip by dt * the selected play rate (rate2), so the crossing dt is in real time
                    const float rate = std::fmax(before.playRate, 0.05f);
                    const float crossingDt = std::max(0.0f, stopLength - stopTime) / rate + 2.0f / fps;
                    crossed = crossingDt > 0.0f && stopTime + crossingDt * rate > stopLength + 1.0e-4f;
                    // Explicitly change the interrupt policy on the crossing update. Mutant 4 returns
                    // the stale Last() (searched/jumped may already be true) before evaluating this force.
                    const motionns::Interrupt force = motionns::Interrupt::ForceInterruptAndInvalidateContinuingPose;
                    const motionns::MatchResult after = mm.Update(crossingDt, q, count, state, nullptr, &force);
                    afterClip = after.clip;
                    forced = after.interrupt == force;
                    searchedAfterEnd = forced && after.searched && after.rowsEvaluated > 0;
                    unfrozen = forced && after.jumped && after.clip >= 0 &&
                               !(after.clip == before.clip && std::fabs(after.clipTime - stopLength) <= 1.0e-4f);
                    break;
                }
            std::printf("motionfix mutation4 clip end: selected=%d crossed=%d clip=%d(%s) t=%.4f len=%.4f "
                        "after=%d(%s) searched=%d forced=%d unfrozen=%d\n",
                        selectedStop ? 1 : 0, crossed ? 1 : 0, stopClip, nameOf(stopClip).c_str(),
                        stopTime, stopLength, afterClip, nameOf(afterClip).c_str(),
                        searchedAfterEnd ? 1 : 0, forced ? 1 : 0, unfrozen ? 1 : 0);
            Check(initialized && selectedStop && crossed && forced && searchedAfterEnd && unfrozen,
                  "motionfix mutation4: real non-loop Stops clip crosses its end and matcher searches rather than freezing Last");
        }
        // (e) crouch idle (item 3, MotionFixCrouchFrames.hpp): the female dump span replayed on a motion database (the first woman_motion file
        // plays a non-motion non-package clip there: run with --chooser ...woman_knee*.chooser.json)
        bool hasStandToCrouch = false;
        for (uint32_t ci = 0; loaded && ci < sdb.NumClips(); ++ci)
            hasStandToCrouch = hasStandToCrouch || sdb.ClipName(ci) == "M_Neutral_Transition_Stand_to_Crouch";
        if (loaded && hasStandToCrouch && chooserPath.find("woman_knee_") != std::string::npos)
        {
            Replay rp;
            const bool ok = replay(motionfixair::kCrouchIdle, motionfixair::kCrouchIdleN, 0.0f, 0.0f, false, 1.0e9f, rp);
            std::printf("motionfix crouch idle: updates %d, jumps %d, frozen %d; jumps %s\n", rp.updates, rp.jumps, rp.frozen, rp.seq.c_str());
            int reEntry = 0, firstEntry = 0, idleLoop = 0;
            for (const Jump& j : rp.jumpList)
            {
                const std::string n = nameOf(j.clip);
                if (n == "M_Neutral_Transition_Stand_to_Crouch")
                {
                    if (j.t >= 1409.5f)
                        ++reEntry; // without the m_completedNoReselectClip exclusion the transition is entered again after the crouch walk stop (cost 0.063 vs idle loop 0.226)
                    else
                        ++firstEntry;
                }
                if (n == "M_Neutral_Crouch_Idle_Loop" && j.t >= 1409.5f)
                    ++idleLoop;
            }
            std::printf("motionfix crouch idle: stand->crouch entries before 1409.5 %d, entries after 1409.5 (the crouch stop) %d, Crouch_Idle_Loop picks after 1409.5 %d\n",
                        firstEntry, reEntry, idleLoop);
            Check(ok && firstEntry >= 1, "motionfix crouch idle: the stand->crouch transition still plays when entering the crouch");
            Check(ok && reEntry == 0, "motionfix crouch idle: no stand->crouch transition after the crouch stop clip ended (neither the first entry nor a re-entry)");
            Check(ok && idleLoop >= 1, "motionfix crouch idle: the hand-over reaches M_Neutral_Crouch_Idle_Loop");
        }
        // (d) the MotionAdapter adapter floors its inertializer too
        if (loaded)
        {
            MotionAdapter ad;
            std::string aerr;
            const bool ok = ad.Init(&sdb, &table, motionns::MatcherConfig{}, 1, aerr);
            Check(ok && ad.Inertial().MinDuration() == kBlendFloorSeconds, "motionfix snap: MotionAdapter adapter inertializer floor = kBlendFloorSeconds");
        }
    }

    // Hipfire visual strafe is explicit and never falsifies native ADS or PSM.
    {
        MapConfig cfg;
        MotionFeed f;
        f.dt = 1.0f / 60; f.psmDetailed = kPsmSprint; f.move[1] = 1;
        f.velWorld[1] = 3.85f; f.aiming = false;
        MapState plain, explicitOff, visual;
        MapContext context;
        const auto before = MapToChooser(f, cfg, plain, context);
        cfg.visualStrafe = false;
        const auto off = MapToChooser(f, cfg, explicitOff, context);
        Check(before.gait == off.gait && plain.st.rotationMode == explicitOff.st.rotationMode &&
              before.movementMode == off.movementMode && before.stance == off.stance,
              "visual strafe default false preserves native mapping");
        cfg.visualStrafe = true;
        const auto shot = MapToChooser(f, cfg, visual, context);
        Check(!f.aiming && f.psmDetailed == kPsmSprint &&
              visual.st.rotationMode == utpp::mm::motiontraj::RotationMode::Strafe && shot.gait == 1,
              "hipfire native ADS false selects visual Strafe and caps sprint to Run");
        if (haveTable)
        {
            const auto selected = ChooserSet(table, shot);
            bool noSprint = !selected.empty();
            for (const auto& name : selected) noSprint = noSprint && name.find("Sprint") == std::string::npos;
            Check(noSprint, "hipfire visual Strafe selects available non-Sprint chooser rows");
        }
        f.psmDetailed = kPsmCrouch; MapState crouch;
        const auto duck = MapToChooser(f, cfg, crouch, context);
        Check(duck.stance == 1 && crouch.st.rotationMode == utpp::mm::motiontraj::RotationMode::Strafe && duck.gait != 2,
              "visual strafe preserves crouch stance without Sprint");
        for (int detail : {kPsmJump,kPsmFall,kPsmClimb,kPsmVault,kPsmLadderFirst,kPsmKnockdown,kPsmFelled})
        {
            f.psmDetailed = detail; MapState blocked, control; MapConfig original = cfg; original.visualStrafe = false;
            const auto actual = MapToChooser(f,cfg,blocked,context);
            const auto expected = MapToChooser(f,original,control,context);
            Check(blocked.st.rotationMode == control.st.rotationMode && actual.gait == expected.gait &&
                  actual.movementMode == expected.movementMode,
                  "visual strafe does not change air/traversal/released mapping");
        }
        f.psmDetailed = kPsmSprint; f.air.valid = true; f.air.onGround = false;
        MapState air, airControl; MapConfig normal = cfg; normal.visualStrafe = false;
        auto a = MapToChooser(f,cfg,air,context); auto b = MapToChooser(f,normal,airControl,context);
        Check(air.st.rotationMode == airControl.st.rotationMode && a.gait == b.gait && a.movementMode == 1,
              "live AIR feed blocks visual strafe even when PSM still reports Sprint");
    }

    // ---------------------------------------------------------------- G4: stick -> speed curve (game 2.5 point), G2 walk toggle speed,
    // G5 Tier2 Slow cap, G3 Walk/Run hysteresis (all new MapConfig / MotionFeed fields default to the pre-change behaviour)
    {
        // G4: the 2.5 median |mv| 0.455 -> 0.82 m/s is a point of the curve; the 2.31 points and the confirmed full stick stay
        const float full = kGameStickSpeedMotion[kGameStickPointsMotion - 1];
        Check(kGameStickPointsMotion == 6 && std::fabs(kGameStickMotion[2] - 0.455f) < 1e-6f && std::fabs(kGameStickSpeedMotion[2] - 0.82f) < 1e-6f,
              "G4: stick curve has the 2.5 point 0.455 -> 0.82 m/s");
        Check(std::fabs(GameStickSpeedFraction(0.455f) * full - 0.82f) < 1e-4f, "G4: stick 0.455 -> 0.82 m/s");
        Check(std::fabs(GameStickSpeedFraction(0.27f) * full - 0.41f) < 1e-4f && std::fabs(GameStickSpeedFraction(0.73f) * full - 1.94f) < 1e-4f &&
                  std::fabs(GameStickSpeedFraction(0.82f) * full - 2.48f) < 1e-4f && std::fabs(GameStickSpeedFraction(1.0f) - 1.0f) < 1e-6f,
              "G4: the 2.31 points and the full stick 3.84 m/s are unchanged");
        Check(std::fabs(GameStickSpeedFraction(0.3625f) * full - 0.615f) < 1e-3f, "G4: linear between 0.27 and 0.455");
        bool monotone = true;
        float prev = 0.0f;
        for (int i = 1; i <= 100; ++i)
        {
            const float fr = GameStickSpeedFraction(static_cast<float>(i) / 100.0f);
            monotone = monotone && fr >= prev;
            prev = fr;
        }
        Check(monotone, "G4: the stick curve is monotone");
    }
    {
        // G2: walk toggle speed. Point 0 (default) = identical to the unmodified driver.
        namespace t2 = utpp::mm::motiontraj;
        MapConfig c0; // gameWalkTogglePoint 0
        MotionFeed fw = feed(2.72f, 1.0f);
        fw.wantsToWalk = true;
        Check(!EffectiveMapConfigNeeded(c0, fw), "G2: point 0 -> no effective config (the caller keeps the config)");
        MapState ma, mb;
        bool same = true;
        for (int i = 0; i < 90; ++i)
        {
            const auto a = MapToChooser(fw, c0, ma, x0);
            const auto b = MapToChooserCore(fw, c0, mb, x0);
            same = same && a.gait == b.gait && a.isStarting == b.isStarting && a.movementState == b.movementState &&
                   ma.st.speed2D == mb.st.speed2D && ma.st.trjFuture.x == mb.st.trjFuture.x && ma.st.trjFuture.y == mb.st.trjFuture.y &&
                   ma.st.trjCurrent.x == mb.st.trjCurrent.x && ma.st.trjCurrent.y == mb.st.trjCurrent.y;
        }
        Check(same, "G2: point 0 -> bit-identical to the unwrapped MapToChooser (walk toggle on, 2.72 m/s)");
        // unmodified map: the toggle's 2.72 m/s lands at ~350 cm/s over the Walk cap 200 (the measured defect)
        Check(std::fabs(2.72f * 100.0f * SpeedScaleFor(c0, 0, 2.72f) - 349.3f) < 1.0f, "G2: unmodified map puts 2.72 m/s at ~349 cm/s");

        MapConfig cw;
        cw.gameWalkTogglePoint = 2.0f;
        Check(EffectiveMapConfigNeeded(cw, fw) && WalkMapApplies(cw, fw), "G2: point 2.0 + wantsToWalk on the stand ground -> walk map");
        const MapConfig e = EffectiveMapConfig(cw, fw);
        Check(e.walkMapActive && std::fabs(e.gameStandPoints[0] - 2.0f) < 1e-6f && e.gameStandPoints[1] == cw.gameStandPoints[1] &&
                  e.gameStandPoints[2] == cw.gameStandPoints[2] && e.gameCrouchPoint == cw.gameCrouchPoint,
              "G2: only the first stand point changes");
        Check(std::fabs(2.0f * 100.0f * SpeedScaleFor(e, 0, 2.0f) - 200.0f) < 0.01f, "G2: 2.0 m/s -> 200 cm/s (the Walk cap)");
        Check(std::fabs(2.0f * 100.0f * SpeedScaleFor(c0, 0, 2.0f) - 253.3f) < 0.2f, "G2: without the point 2.0 m/s is 253 cm/s (over the cap)");
        Check(std::fabs(2.72f * 100.0f * SpeedScaleFor(e, 0, 2.72f) - 316.8f) < 0.2f, "G2: 2.72 m/s (unmodified toggle) -> 317 cm/s, continuous above the point");
        Check(std::fabs(GameAnalogModifier(e, 0, 0, false, 1.0f) - 1.0f) < 1e-4f, "G2: full stick -> analog modifier 1 against the Walk cap");
        Check(std::fabs(GameSteadySpeed(e, 0, false, 1.0f) - 2.0f) < 1e-5f && std::fabs(GameSteadySpeed(e, 0, false, 0.455f) - 2.0f * 0.82f / 3.84f) < 1e-4f,
              "G2: steady speed = point x stick fraction");
        // gameCap = the point, consistent with the stop / start prediction (no silently zeroed acceleration, MotionAdapter.hpp gameCap guard)
        {
            const float gameCap = e.gameStandPoints[0];
            auto p = CapsuleStartPrediction(e, 0.0f, 2.0f, gameCap);
            const t2::Derived dd = t2::MakeDerived(e.cmc, 0, 0, GameAnalogModifier(e, 0, 0, false, 1.0f), 2.0f * 100.0f * SpeedScaleFor(e, 0, 2.0f),
                                                   t2::V3{0.0f, 1.0f, 0.0f}, true, false);
            Check(std::fabs(p.QuerySpeed(gameCap) - dd.maxSpeed) < 0.1f && std::fabs(dd.maxSpeed - 200.0f) < 0.1f,
                  "G2: capsule start prediction at the point equals the prediction cap (query speed 200)");
        }
        // not applied: toggle off, aiming, AimWalk, crouch, sprint, air, Tier2 Slow, point out of range
        {
            MotionFeed g = fw;
            g.wantsToWalk = false;
            Check(!WalkMapApplies(cw, g), "G2: toggle off -> no walk map");
            g = fw;
            g.aiming = true;
            Check(!WalkMapApplies(cw, g), "G2: aiming -> no walk map");
            g = fw;
            g.psmDetailed = kPsmAimWalk;
            Check(!WalkMapApplies(cw, g), "G2: AimWalk -> no walk map");
            g = fw;
            g.psmDetailed = kPsmCrouch;
            Check(!WalkMapApplies(cw, g), "G2: crouch -> no walk map");
            g = fw;
            g.psmDetailed = kPsmSprint;
            Check(!WalkMapApplies(cw, g), "G2: sprint -> no walk map");
            g = fw;
            g.air.valid = true;
            g.air.onGround = false;
            Check(!WalkMapApplies(cw, g), "G2: air -> no walk map");
            g = fw;
            g.tier2Slow = true;
            Check(!WalkMapApplies(cw, g), "G2: Tier2 Slow -> no walk map");
            MapConfig bad = cw;
            bad.gameWalkTogglePoint = 3.85f;
            Check(!WalkMapApplies(bad, fw), "G2: point >= the stand run point -> ignored");
            bad.gameWalkTogglePoint = std::nanf("");
            Check(!WalkMapApplies(bad, fw), "G2: NaN point -> ignored");
        }
        // steady walk at the point: no invented starting / stopping; the same speed without the point keeps a decelerating prediction
        {
            auto run = [&](const MapConfig& cfg, float speed, const MotionFeed& base, int& starting, int& gaitOk, float& minGain, float& maxGain) {
                MapState ms;
                MotionFeed g = base;
                g.velWorld[1] = speed;
                utpp::mm::motion::ChooserState cs;
                for (int i = 0; i < 30; ++i)
                    cs = MapToChooser(g, cfg, ms, x0);
                starting = gaitOk = 0;
                minGain = 1.0e9f;
                maxGain = -1.0e9f;
                for (int i = 0; i < 60; ++i)
                {
                    cs = MapToChooser(g, cfg, ms, x0);
                    starting += cs.isStarting ? 1 : 0;
                    gaitOk += cs.gait == 0 ? 1 : 0;
                    const float gain = t2::SizeXY(ms.st.trjFuture) - t2::SizeXY(ms.st.velocity);
                    minGain = std::fmin(minGain, gain);
                    maxGain = std::fmax(maxGain, gain);
                }
            };
            int st1, g1, st0, g0;
            float lo1, hi1, lo0, hi0;
            run(cw, 2.0f, fw, st1, g1, lo1, hi1);
            run(c0, 2.0f, fw, st0, g0, lo0, hi0);
            std::printf("G2 steady walk 2.0 m/s: point 2.0 -> starting %d gait0 %d gain %.1f .. %.1f; point 0 -> starting %d gait0 %d gain %.1f .. %.1f\n", st1, g1,
                        static_cast<double>(lo1), static_cast<double>(hi1), st0, g0, static_cast<double>(lo0), static_cast<double>(hi0));
            Check(st1 == 0 && g1 == 60 && lo1 > -1.0f && hi1 < 1.0f, "G2: steady walk at the point: Walk gait, no IsStarting, future == current speed");
            Check(lo0 < -20.0f, "G2: without the point the same 2.0 m/s walk predicts a deceleration (the old defect)");
        }
    }
    {
        // G5: Tier2 Slow lowers the stand run cap by 1.3 m/s (3.85 - 1.3 = 2.55)
        namespace t2 = utpp::mm::motiontraj;
        MapConfig c0;
        MotionFeed fs = feed(2.55f, 1.0f);
        fs.tier2Slow = true;
        Check(EffectiveMapConfigNeeded(c0, fs) && !WalkMapApplies(c0, fs), "G5: Tier2 Slow -> effective config, no walk map");
        MotionFeed fn = fs;
        fn.tier2Slow = false;
        Check(!EffectiveMapConfigNeeded(c0, fn), "G5: flag off -> no effective config (bit-identical)");
        const MapConfig e = EffectiveMapConfig(c0, fs);
        Check(std::fabs(e.gameRunReductionMs - 1.3f) < 1e-6f && !e.walkMapActive && e.gameStandPoints[0] == c0.gameStandPoints[0],
              "G5: only the run reduction changes");
        Check(std::fabs(GameSteadySpeed(e, 0, false, 1.0f) - 2.55f) < 1e-5f && std::fabs(GameSteadySpeed(c0, 0, false, 1.0f) - 3.85f) < 1e-5f,
              "G5: full-stick steady run 2.55 m/s (3.85 without the flag)");
        // sprint / crouch are not touched
        Check(GameSteadySpeed(e, 0, true, 1.0f) == GameSteadySpeed(c0, 0, true, 1.0f) && GameSteadySpeed(e, 1, false, 1.0f) == GameSteadySpeed(c0, 1, false, 1.0f),
              "G5: sprint and crouch full-stick speeds unchanged");
        const float mod = GameAnalogModifier(e, 1, 0, false, 1.0f);
        Check(std::fabs(mod - 326.67f / 500.0f) < 1e-3f && GameAnalogModifier(c0, 1, 0, false, 1.0f) == 1.0f,
              "G5: analog modifier 0.653 (cap 327 cm/s) with the flag, 1 without");
        {
            const float gameCap = c0.gameStandPoints[1] - e.gameRunReductionMs;
            auto p = CapsuleStartPrediction(e, 0.0f, 2.55f, gameCap);
            const t2::Derived dd = t2::MakeDerived(e.cmc, 1, 0, mod, 2.55f * 100.0f * SpeedScaleFor(e, 0, 2.55f), t2::V3{0.0f, 1.0f, 0.0f}, true, false);
            Check(std::fabs(p.QuerySpeed(gameCap) - dd.maxSpeed) < 0.1f, "G5: capsule start prediction at the lowered cap equals the prediction cap");
        }
        auto run = [&](const MotionFeed& g, int& starting, int& runOk) {
            MapState ms;
            utpp::mm::motion::ChooserState cs;
            for (int i = 0; i < 30; ++i)
                cs = MapToChooser(g, c0, ms, x0);
            starting = runOk = 0;
            for (int i = 0; i < 60; ++i)
            {
                cs = MapToChooser(g, c0, ms, x0);
                starting += cs.isStarting ? 1 : 0;
                runOk += cs.gait == 1 ? 1 : 0;
            }
        };
        int stOn, runOn, stOff, runOff;
        run(fs, stOn, runOn);
        run(fn, stOff, runOff);
        std::printf("G5 steady 2.55 m/s full stick: Tier2 flag -> starting %d run %d; no flag -> starting %d run %d\n", stOn, runOn, stOff, runOff);
        Check(stOn == 0 && runOn == 60, "G5: Tier2 Slow steady run at 2.55 m/s: Run gait, no IsStarting");
        Check(stOff > 0, "G5: without the flag the same 2.55 m/s run keeps IsStarting (the old defect)");
    }
    {
        // G3: Walk/Run hysteresis (MapConfig::gaitHysteresis, default OFF)
        MapConfig c0;
        Check(!c0.gaitHysteresis && c0.gaitHysteresisUp == 0.75f && c0.gaitHysteresisDown == 0.65f, "G3: hysteresis off by default, 0.75 / 0.65");
        auto gait = [](float aStick, int aPrev, bool aWalk = false, bool aSprint = false, StickMode aMode = StickMode::VariableSpeedWalkRun) {
            return GaitFromInput(aSprint, aWalk, aStick, aMode, 0.7f, true, 0.0f, 0.0f, aPrev, aPrev >= 0 ? 0.75f : 0.0f, aPrev >= 0 ? 0.65f : 0.0f);
        };
        Check(gait(0.7f, -1) == 1 && gait(0.69f, -1) == 0, "G3: no hysteresis = the single 0.7 threshold");
        Check(gait(0.74f, 0) == 0 && gait(0.75f, 0) == 1, "G3: Walk -> Run at 0.75");
        Check(gait(0.65f, 1) == 1 && gait(0.649f, 1) == 0 && gait(0.649f, 2) == 0 && gait(0.66f, 2) == 1, "G3: Run (or Sprint) -> Walk below 0.65");
        Check(gait(0.5f, 0, true) == 0 && gait(1.0f, 1, true) == 0 && gait(1.0f, 0, false, true) == 2 && gait(0.1f, 0, false, true) == 1,
              "G3: walk toggle and sprint decide as before");
        Check(gait(0.1f, 0, false, false, StickMode::FixedSpeedSingleGait) == 1 && gait(0.1f, 0, false, false, StickMode::VariableSpeedSingleGait) == 1,
              "G3: SingleGait modes are never hysteretic");
        // through MapToChooser: a stick resting on the threshold (0.68 / 0.72 alternating) flips the gait every frame without it
        auto flips = [&](bool aHyst, float aStartStick) {
            MapConfig c;
            c.gaitHysteresis = aHyst;
            MapState ms;
            utpp::mm::motion::ChooserState cs;
            for (int i = 0; i < 30; ++i)
                cs = MapToChooser(feed(1.9f, aStartStick), c, ms, x0);
            int changes = 0, last = cs.gait;
            for (int i = 0; i < 60; ++i)
            {
                cs = MapToChooser(feed(1.9f, (i & 1) ? 0.72f : 0.68f), c, ms, x0);
                changes += cs.gait != last ? 1 : 0;
                last = cs.gait;
            }
            return changes;
        };
        const int off = flips(false, 0.68f), onRun = flips(true, 1.0f), onWalk = flips(true, 0.3f);
        std::printf("G3 gait changes over 60 frames on a 0.68 / 0.72 stick: off %d, hysteresis from Run %d, from Walk %d\n", off, onRun, onWalk);
        Check(off >= 50, "G3: without hysteresis the gait flips with the stick around 0.7");
        Check(onRun == 0 && onWalk == 0, "G3: with hysteresis the gait holds (Run from Run, Walk from Walk)");
    }
    {
        // G9: aim walk at the vanilla speed (aim jog off) asks for the Walk gait; the flag changes nothing outside AimWalk
        MapConfig c;
        auto gaitOf = [&](int32_t aDetail, bool aVanilla, bool aAiming) {
            MotionFeed g = feed(1.65f, 1.0f);
            g.psmDetailed = aDetail;
            g.aiming = aAiming;
            g.aimWalkVanilla = aVanilla;
            MapState ms;
            utpp::mm::motion::ChooserState cs;
            for (int i = 0; i < 10; ++i)
                cs = MapToChooser(g, c, ms, x0);
            return cs.gait;
        };
        Check(gaitOf(kPsmAimWalk, true, true) == 0 && gaitOf(kPsmAimWalk, false, true) == 1,
              "G9: AimWalk full stick: Walk with the vanilla aim speed, Run without the flag (as before)");
        Check(gaitOf(kPsmStand, true, false) == 1, "G9: the flag does not touch the Stand detail");
    }

    // adapter fails closed on an empty database
    utpp::mm::motion::MotionDatabase db;
    MotionAdapter d;
    std::string err;
    Check(!d.Init(&db, nullptr, utpp::mm::motion::MatcherConfig{}, 2, err) && !d.Ready(), "init fails closed");
    Check(!d.Update(1.0f / 60.0f, nullptr, 0, s), "update without init");

    std::printf("motion_adapter compile test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
