// Slide feed (CET mm.slide): offline replay of REAL game slides through the v2 feed adapter adapter::MapToChooser, header only (no
// database, no game). Fixture tests\data\slide_feeds.txt = 6 slides (PSM LocomotionDetailed 5, 0.5 s before / 1.0 s after) cut by
// a streaming parser from the mm.feedDump logs (the recorded log, 2026-10-13..19; sprint slides of 0.17 .. 3.13 s, one after a
// landing, one male V). Usage: utpp_mm_slide_feed_test <slide_feeds.txt> [<chooser.json with slide> <chooser.json without>]
//
// Asserted: MovementMode / MovementMode_LastFrame / MovementDirection per frame (enter at the first detail-5 frame, the last-frame
// flag on exactly the first post-slide frame, direction by the 45 deg rule with hysteresis), Stance Crouch during the slide, real
// speed passed through, no effect on non-slide frames, the setting OFF (or a bundle without the section) = the pre-slide output,
// the chooser.json "slide" section adapter. The measured query-trajectory quality of the slide braking model is printed and bounded.
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

#include "../src/mm/MotionAdapter.hpp"

namespace adapter = utpp::mm::adapter;
namespace integ = utpp::mm::integ;
namespace motion = utpp::mm::motion;
namespace motiontraj = utpp::mm::motiontraj;

namespace
{
int g_fail = 0;
int g_checks = 0;
void Check(bool aOk, const char* aWhat, double aA = 0.0, double aB = 0.0)
{
    ++g_checks;
    if (!aOk)
    {
        if (++g_fail <= 40) // a systematic failure repeats per frame: the first ones say it
            std::printf("FAIL: %s (%.4f, %.4f)\n", aWhat, aA, aB);
    }
}

struct Row
{
    double t = 0;
    float dt = 0, vel[3] = {0, 0, 0}, mv[2] = {0, 0}, yaw = 0, cam = 0, target = 0;
    int loco = 0;
    uint32_t psm = 0;
    bool haveTarget = false;
    int onGround = -1; // -1 '-' (no air feed)
    float dist = -1;
    int fall = 0, walk = 0;
};
struct Slide
{
    std::string name;
    std::vector<Row> rows;
};

float Num(const std::string& aTok, float aDef = 0.0f) { return aTok == "-" ? aDef : std::strtof(aTok.c_str(), nullptr); }

bool LoadFixture(const char* aPath, std::vector<Slide>& aOut)
{
    std::ifstream f(aPath);
    if (!f)
        return false;
    std::string line;
    std::vector<std::string> cols;
    while (std::getline(f, line))
    {
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
        if (line.rfind("@slide ", 0) == 0)
        {
            aOut.push_back({line.substr(7), {}});
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
        r.onGround = at("onGround") == "-" ? -1 : static_cast<int>(Num(at("onGround")));
        r.dist = Num(at("distGround"), -1.0f);
        r.fall = static_cast<int>(Num(at("psmFall")));
        r.walk = static_cast<int>(Num(at("walk")));
        aOut.back().rows.push_back(r);
    }
    return !aOut.empty();
}

adapter::MotionFeed FeedOf(const Row& r)
{
    adapter::AirFeed air;
    air.valid = r.onGround >= 0;
    air.onGround = r.onGround != 0;
    air.distanceToGround = r.dist;
    air.psmFall = r.fall;
    adapter::MotionFeed f = adapter::MakeMotionFeed(r.vel, r.mv, r.dt, r.psm, r.loco, air, r.cam, r.yaw);
    f.wantsToWalk = r.walk != 0;
    f.haveTarget = r.haveTarget && std::isfinite(r.target);
    f.targetYawDeg = f.haveTarget ? r.target : 0.0f;
    return f;
}

adapter::MapConfig Config(bool aSlide, bool aDeclared = true)
{
    adapter::MapConfig mc;
    mc.stanceSpeedBlendTime = 0.2f;
    mc.slide = aSlide;
    mc.slideSpec.declared = aDeclared;
    mc.slideDeceleration = aSlide ? adapter::kSlideDecelerationMs2 : 0.0f;
    return mc;
}

struct Out
{
    motion::ChooserState s;
    bool detail5 = false;        // the feed's PSM detail
    float speed = 0;             // real planar game speed (m/s)
    float velYaw = 0;            // capsule heading, integ convention (deg)
    float pred05 = 0, pred10 = 0; // predicted motion-mapped distance ahead at +0.5 / +1.0 s (m), from the query trajectory
    float scale = 1;             // mapped / game speed at this frame
    adapter::MapState state;
};

std::vector<Out> Replay(const Slide& aSlide, const adapter::MapConfig& aCfg)
{
    std::vector<Out> out;
    adapter::MapState m;
    adapter::MapContext ctx;
    for (const Row& r : aSlide.rows)
    {
        const adapter::MotionFeed f = FeedOf(r);
        Out o;
        o.s = adapter::MapToChooser(f, aCfg, m, ctx);
        o.detail5 = f.psmDetailed == adapter::kPsmSlide;
        o.speed = std::hypot(r.vel[0], r.vel[1]);
        o.velYaw = o.speed > 1e-4f ? std::atan2(-r.vel[0], r.vel[1]) / integ::kDegToRad : 0.0f;
        const auto a = adapter::QuerySample(m.traj, 0.5f);
        const auto b = adapter::QuerySample(m.traj, 1.0f);
        o.pred05 = std::hypot(a.x, a.y);
        o.pred10 = std::hypot(b.x, b.y);
        o.scale = o.speed > 1e-3f ? adapter::SpeedScaleFor(aCfg, 0, o.speed) : 1.0f;
        o.state = m;
        out.push_back(o);
    }
    return out;
}

bool Same(const motion::ChooserState& a, const motion::ChooserState& b)
{
    // every field a slide-less chooser reads, bitwise for the floats
    auto fb = [](float x, float y) { return std::memcmp(&x, &y, sizeof x) == 0; };
    return a.gait == b.gait && a.stance == b.stance && a.movementState == b.movementState && a.movementMode == b.movementMode &&
           fb(a.speed2dCmS, b.speed2dCmS) && fb(a.timeToLand, b.timeToLand) && a.isStarting == b.isStarting &&
           a.isPivoting == b.isPivoting && a.justLandedLight == b.justLandedLight && a.justLandedHeavy == b.justLandedHeavy &&
           a.justTraversed == b.justTraversed && a.shouldTurnInPlace == b.shouldTurnInPlace &&
           a.shouldSpinTransition == b.shouldSpinTransition && a.databaseLod == b.databaseLod &&
           fb(a.verticalVelocityCmS, b.verticalVelocityCmS) && a.stationaryShotTurn == b.stationaryShotTurn &&
           a.stationaryAimTurn == b.stationaryAimTurn && a.explicitWalk == b.explicitWalk &&
           fb(a.shotTurnYawDeg, b.shotTurnYawDeg) && fb(a.stationaryAimYawDeg, b.stationaryAimYawDeg) &&
           fb(a.stationaryAimTargetYawDeg, b.stationaryAimTargetYawDeg) && a.startTurnDirection == b.startTurnDirection &&
           a.committedStartTurnClip == b.committedStartTurnClip && a.movementModeLast == b.movementModeLast &&
           a.movementDirection == b.movementDirection;
}

// ---- synthetic feed: straight line at aSpeed (m/s), body facing aFacingDeg, heading aHeadingDeg, PSM detail aDetail
Row Synth(double aT, int aDetail, float aSpeed, float aFacingDeg, float aHeadingDeg)
{
    Row r;
    r.t = aT;
    r.dt = 1.0f / 60.0f;
    const float h = aHeadingDeg * integ::kDegToRad;
    r.vel[0] = -std::sin(h) * aSpeed;
    r.vel[1] = std::cos(h) * aSpeed;
    r.mv[1] = 1.0f; // the stick stays forward during a real slide
    r.yaw = aFacingDeg;
    r.cam = aFacingDeg;
    r.loco = aDetail == 5 ? 0 : 3;
    // PSM block packing: the detail only travels with kPsmDetailed (details other than Stand 1 / Crouch 3 / Sprint 4)
    r.psm = (aDetail == 1 || aDetail == 3 || aDetail == 4)
                ? 0u
                : (integ::kPsmDetailed | (static_cast<uint32_t>(aDetail + 1) << integ::kPsmDetailShift));
    if (aDetail == 3)
        r.loco = 2;
    r.onGround = 1;
    return r;
}

Slide SynthSlide(const char* aName, std::initializer_list<std::pair<int, std::pair<float, float>>> aSegments)
{
    // segments: detail, (heading deg, frames as float)
    Slide s{aName, {}};
    double t = 0;
    for (const auto& seg : aSegments)
        for (int i = 0; i < static_cast<int>(seg.second.second); ++i, t += 1.0 / 60.0)
            s.rows.push_back(Synth(t, seg.first, 8.0f, 0.0f, seg.second.first));
    return s;
}

void TestSpec(const char* aWith, const char* aWithout)
{
    adapter::SlideSpec s;
    const std::string good =
        R"({"slide": {"version": 1, "sliding_value": 2, "direction_values": {"Forward": 0, "Other": 1},
            "databases": {"PSD_Slide_FeetOut": 35, "PSD_Slide_KneesOut": 36}}})";
    Check(adapter::ParseSlideSpec(good, s) && s.declared && s.slidingValue == 2 && s.forwardValue == 0 && s.otherValue == 1,
          "spec: complete section declared");
    // the adapter follows the file, not constants
    const std::string moved =
        R"({"slide": {"version": 1, "sliding_value": 6, "direction_values": {"Forward": 1, "Other": 3},
            "databases": {"PSD_Slide_FeetOut": 35, "PSD_Slide_KneesOut": 36}}})";
    Check(adapter::ParseSlideSpec(moved, s) && s.slidingValue == 6 && s.forwardValue == 1 && s.otherValue == 3,
          "spec: values come from the file");
    for (const char* bad : {R"({})", R"({"slide": 3})",
                            R"({"slide": {"version": 2, "sliding_value": 2, "direction_values": {"Forward": 0, "Other": 1},
                               "databases": {"PSD_Slide_FeetOut": 1, "PSD_Slide_KneesOut": 2}}})",
                            R"({"slide": {"version": 1, "sliding_value": 1, "direction_values": {"Forward": 0, "Other": 1},
                               "databases": {"PSD_Slide_FeetOut": 1, "PSD_Slide_KneesOut": 2}}})",
                            R"({"slide": {"version": 1, "sliding_value": 2, "direction_values": {"Forward": 0},
                               "databases": {"PSD_Slide_FeetOut": 1, "PSD_Slide_KneesOut": 2}}})",
                            R"({"slide": {"version": 1, "sliding_value": 2, "direction_values": {"Forward": 1, "Other": 1},
                               "databases": {"PSD_Slide_FeetOut": 1, "PSD_Slide_KneesOut": 2}}})",
                            R"({"slide": {"version": 1, "sliding_value": 2.5, "direction_values": {"Forward": 0, "Other": 1},
                               "databases": {"PSD_Slide_FeetOut": 1, "PSD_Slide_KneesOut": 2}}})",
                            R"({"slide": {"version": 1, "sliding_value": 2, "direction_values": {"Forward": 0, "Other": 1},
                               "databases": {"PSD_Slide_FeetOut": 1}}})",
                            R"({"slide": {"version": 1)", ""})
    {
        s.declared = true;
        Check(!adapter::ParseSlideSpec(bad, s) && !s.declared, "spec: incomplete / invalid section is not declared");
    }
    auto text = [](const char* p) {
        std::ifstream f(p, std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    };
    if (aWith)
    {
        Check(adapter::ParseSlideSpec(text(aWith), s) && s.declared && s.slidingValue == 2 && s.forwardValue == 0 && s.otherValue == 1,
              "spec: real slide bundle chooser.json declares the section (sliding 2, Forward 0, Other 1)");
        std::printf("spec: %s -> declared sliding=%d forward=%d other=%d\n", aWith, s.slidingValue, s.forwardValue, s.otherValue);
    }
    if (aWithout)
        Check(!adapter::ParseSlideSpec(text(aWithout), s) && !s.declared, "spec: real bundle without slide section is not declared");
}

void TestSynthetic()
{
    constexpr int kSlide = 5, kCrouch = 3, kRun = 1;
    // direction: Forward until the heading leaves 45 deg, Other with 5 deg hysteresis, speed gate holds the direction
    {
        const Slide s = SynthSlide("direction", {{kRun, {0.f, 5.f}}, {kSlide, {20.f, 4.f}}, {kSlide, {44.f, 3.f}}, {kSlide, {46.f, 3.f}},
                                                 {kSlide, {43.f, 3.f}}, {kSlide, {41.f, 3.f}}, {kSlide, {39.f, 3.f}},
                                                 {kSlide, {-90.f, 3.f}}, {kSlide, {180.f, 3.f}}, {kSlide, {-30.f, 3.f}},
                                                 {kCrouch, {0.f, 2.f}}});
        const auto on = Replay(s, Config(true));
        auto dirAt = [&](size_t i) { return on[i].s.movementDirection; };
        size_t i = 5;
        Check(dirAt(i) == 0 && dirAt(i + 3) == 0, "direction: 20 deg and 44 deg = Forward");
        i += 4 + 3;
        Check(dirAt(i) == 1 && dirAt(i + 2) == 1, "direction: 46 deg = Other");
        i += 3;
        Check(dirAt(i) == 1 && dirAt(i + 2) == 1, "direction: 43 / 41 deg after Other stays Other (hysteresis)", dirAt(i), dirAt(i + 2));
        i += 3 + 3;
        Check(dirAt(i) == 0, "direction: 39 deg returns to Forward");
        i += 3;
        Check(dirAt(i) == 1 && dirAt(i + 2) == 1, "direction: -90 deg = Other");
        i += 3;
        Check(dirAt(i) == 1, "direction: 180 deg (backwards) = Other");
        i += 3;
        Check(dirAt(i) == 0 && dirAt(i + 2) == 0, "direction: -30 deg = Forward");
        i += 3;
        Check(dirAt(i) == 0 && on[i].s.movementMode == 0, "direction: 0 outside the slide");
        // the direction does not leak outside the slide
        bool leak = false;
        for (size_t k = 0; k < on.size(); ++k)
            leak = leak || (on[k].s.movementMode != 2 && on[k].s.movementDirection != 0);
        Check(!leak, "direction: 0 on every non-slide frame");
    }
    // low speed: the heading is noise, the direction holds
    {
        Slide s = SynthSlide("lowspeed", {{kSlide, {90.f, 3.f}}});
        for (Row& r : s.rows)
            r.vel[0] = r.vel[1] = 0.0f; // stopped capsule
        s.rows.insert(s.rows.begin(), Synth(0, kSlide, 8.0f, 0.0f, 90.0f));
        const auto on = Replay(s, Config(true));
        Check(on[0].s.movementDirection == 1 && on[1].s.movementDirection == 1 && on[3].s.movementDirection == 1,
              "direction: held at speed < 0.5 m/s");
    }
    // enter / exit flags
    {
        const Slide s = SynthSlide("exit", {{kRun, {0.f, 4.f}}, {kSlide, {0.f, 6.f}}, {kCrouch, {0.f, 4.f}}});
        const auto on = Replay(s, Config(true));
        bool ok = true;
        for (size_t i = 0; i < on.size(); ++i)
        {
            const bool slide = i >= 4 && i < 10;
            ok = ok && on[i].s.movementMode == (slide ? 2 : 0) && on[i].s.stance == (slide ? 1 : (i >= 10 ? 1 : 0));
            ok = ok && on[i].s.movementModeLast == (i == 4 ? 0 : (i == 10 ? 2 : on[i - (i ? 1 : 0)].s.movementMode));
        }
        Check(ok, "enter at the first detail-5 frame, Last = Sliding exactly on the first post-slide frame");
        Check(on[9].s.movementModeLast == 2 && on[10].s.movementModeLast == 2 && on[11].s.movementModeLast == 0,
              "Last: Sliding on slide frames 2..n (previous frame) and the first exit frame, OnGround after");
        Check(on[10].s.movementMode == 0 && on[10].s.stance == 1, "exit frame: OnGround, Crouch (PSM Crouch 3)");
    }
    // exit to a standing run: the stance is Stand again, Last still Sliding for that one frame
    {
        const Slide s = SynthSlide("exit-stand", {{kSlide, {0.f, 5.f}}, {kRun, {0.f, 3.f}}});
        const auto on = Replay(s, Config(true));
        Check(on[5].s.movementModeLast == 2 && on[5].s.stance == 0 && on[5].s.movementMode == 0 && on[6].s.movementModeLast == 0,
              "exit to Stand: stance Stand, Last Sliding once");
    }
    // slide after a jump: Last = InAir on the first slide frame
    {
        Slide s = SynthSlide("air", {{18, {0.f, 3.f}}, {kSlide, {0.f, 3.f}}});
        for (int i = 0; i < 3; ++i)
            s.rows[size_t(i)].onGround = 0;
        const auto on = Replay(s, Config(true));
        Check(on[2].s.movementMode == 1 && on[3].s.movementMode == 2 && on[3].s.movementModeLast == 1, "slide after InAir: Last = InAir");
    }
    // airborne feed during PSM 5: not a slide (the capsule left the ground)
    {
        Slide s = SynthSlide("slidefall", {{kSlide, {0.f, 4.f}}});
        for (Row& r : s.rows)
            r.onGround = 0;
        const auto on = Replay(s, Config(true));
        Check(on.back().s.movementMode == 1 && !on.back().state.slideActive, "PSM 5 with the air feed off the ground = InAir, no slide");
    }
    // setting off / bundle without the section: identical to the pre-slide output
    {
        const Slide s = SynthSlide("off", {{kRun, {0.f, 4.f}}, {kSlide, {0.f, 8.f}}, {kCrouch, {0.f, 4.f}}});
        const auto off = Replay(s, Config(false));
        const auto undeclared = Replay(s, Config(true, false));
        bool a = true, b = true;
        for (size_t i = 0; i < off.size(); ++i)
        {
            a = a && off[i].s.movementMode == 0 && off[i].s.movementDirection == 0 && !off[i].state.slideActive;
            b = b && Same(off[i].s, undeclared[i].s) && undeclared[i].pred05 == off[i].pred05;
        }
        Check(a, "OFF: MovementMode never Sliding, Direction 0");
        Check(b, "setting on without a declared section = OFF, bit for bit (state and trajectory)");
    }
}

void TestReplay(const std::vector<Slide>& aSlides)
{
    size_t totalSlide = 0, totalFrames = 0, exits = 0;
    double sumErrOn = 0, sumErrOff = 0, sumRatioOn = 0, sumRatioOff = 0;
    size_t nErr = 0;
    float maxDirAngle = 0, minOtherAngle = 999;
    for (const Slide& sl : aSlides)
    {
        const auto on = Replay(sl, Config(true));
        const auto off = Replay(sl, Config(false));
        Check(on.size() == off.size() && !on.empty(), "replay: same length");
        size_t slideFrames = 0, exitFrames = 0;
        bool seenSlide = false, beforeSlide = true;
        for (size_t i = 0; i < on.size(); ++i)
        {
            const Out& n = on[i];
            const Out& o = off[i];
            const bool isSlide = n.detail5 && n.state.slideActive;
            ++totalFrames;
            // OFF output: never Sliding, never a direction, Last = the previous frame's mode
            Check(o.s.movementMode != 2 && o.s.movementDirection == 0 && !o.state.slideActive, "OFF: no Sliding / direction");
            Check(o.s.movementModeLast == (i ? off[i - 1].s.movementMode : 0), "OFF: Last = previous frame's mode");
            if (n.detail5 && n.state.slideActive != (n.detail5 && n.s.movementMode == 2))
                Check(false, "slide flag consistent with MovementMode");
            if (isSlide)
            {
                ++slideFrames;
                seenSlide = true;
                beforeSlide = false;
                Check(n.s.movementMode == 2, "slide frame: MovementMode Sliding");
                Check(n.s.stance == 1, "slide frame: Stance Crouch");
                {
                    // motion query speed = the slide speed map (MapConfig::slideLoopCmS / slideEntryCmS / slideLowMs / slideHighMs)
                    const adapter::MapConfig mc = Config(true);
                    const float u = std::fmin(1.0f, std::fmax(0.0f, (n.speed - mc.slideLowMs) / (mc.slideHighMs - mc.slideLowMs)));
                    const float want = n.speed < mc.slideLowMs ? mc.slideLoopCmS * n.speed / mc.slideLowMs :
                                                                 mc.slideLoopCmS + (mc.slideEntryCmS - mc.slideLoopCmS) * u;
                    Check(std::fabs(n.s.speed2dCmS - want) < 0.5f && n.s.speed2dCmS > 0.0f, "slide frame: query speed follows the slide speed map",
                          double(n.s.speed2dCmS), double(want));
                }
                Check(n.s.movementState == 1, "slide frame: Moving");
                Check(n.s.movementModeLast == (i ? on[i - 1].s.movementMode : 0), "slide frame: Last = previous frame's mode");
                if (slideFrames == 1)
                    Check(n.s.movementModeLast != 2, "first slide frame: Last is not Sliding yet");
                // direction against an independent measurement (outside the hysteresis band)
                const float dev = std::fabs(motiontraj::NormalizeAxis(n.velYaw - sl.rows[i].yaw));
                if (n.speed > adapter::kSlideDirectionMinSpeed)
                {
                    if (n.s.movementDirection == 0)
                        maxDirAngle = std::max(maxDirAngle, dev);
                    else
                        minOtherAngle = std::min(minOtherAngle, dev);
                    if (dev < 38.0f)
                        Check(n.s.movementDirection == 0, "direction Forward below 38 deg", dev);
                    if (dev > 47.0f)
                        Check(n.s.movementDirection == 1, "direction Other above 47 deg", dev);
                }
            }
            else
            {
                Check(n.s.movementMode != 2, "non-slide frame: not Sliding");
                if (seenSlide && n.s.movementModeLast == 2)
                    ++exitFrames;
                if (!seenSlide)
                    Check(Same(n.s, o.s) && n.pred05 == o.pred05 && n.pred10 == o.pred10 &&
                              std::memcmp(&n.state.st.velocity, &o.state.st.velocity, sizeof n.state.st.velocity) == 0,
                          "before the slide: setting on == setting off bit for bit (state and trajectory)");
                if (!beforeSlide && n.s.movementModeLast != 2)
                    Check(n.s.movementDirection == 0, "after the slide: direction 0");
            }
        }
        Check(slideFrames > 0, "fixture slide has detail-5 frames");
        Check(exitFrames == 1, "exactly one exit frame carries Last = Sliding", double(exitFrames));
        totalSlide += slideFrames;
        exits += exitFrames;

        // query trajectory quality of the slide braking model: mapped distance ahead at +0.5 s vs what the capsule really did
        for (size_t i = 0; i < on.size(); ++i)
        {
            if (!(on[i].detail5 && on[i].state.slideActive))
                continue;
            // actual mapped distance over the next 0.5 s (needs the slide to last that long, so no exit effect pollutes it)
            double t = 0, dist = 0, distOff = 0;
            size_t k = i + 1;
            for (; k < on.size() && t < 0.5; ++k)
            {
                if (!on[k].detail5)
                    break;
                dist += double(on[k].s.speed2dCmS) * 0.01 * double(sl.rows[k].dt); // slide speed map (cm/s -> m)
                distOff += double(off[k].s.speed2dCmS) * 0.01 * double(sl.rows[k].dt); // standard map
                t += sl.rows[k].dt;
            }
            if (t < 0.49)
                continue;
            const double eOn = std::fabs(on[i].pred05 - dist), eOff = std::fabs(off[i].pred05 - distOff);
            sumErrOn += eOn;
            sumErrOff += eOff;
            sumRatioOn += on[i].pred05 / dist;
            sumRatioOff += off[i].pred05 / distOff;
            ++nErr;
        }
    }
    std::printf("replay: %zu slides, %zu frames, %zu slide frames, %zu exit frames; direction Forward up to %.1f deg off the body\n",
                aSlides.size(), totalFrames, totalSlide, exits, double(maxDirAngle));
    if (minOtherAngle < 999)
        std::printf("replay: direction Other from %.1f deg\n", double(minOtherAngle));
    Check(exits == aSlides.size(), "one exit frame per slide", double(exits), double(aSlides.size()));
    if (nErr > 0)
    {
        std::printf("replay: predicted mapped distance at +0.5 s vs actual, %zu slide frames: mean |error| slide model %.3f m, standard "
                    "predictor %.3f m; mean predicted/actual %.3f vs %.3f\n",
                    nErr, sumErrOn / double(nErr), sumErrOff / double(nErr), sumRatioOn / double(nErr), sumRatioOff / double(nErr));
        Check(sumErrOn < sumErrOff, "slide braking trajectory is closer to the real slide than the standard input prediction",
              sumErrOn / double(nErr), sumErrOff / double(nErr));
        Check(std::fabs(sumRatioOn / double(nErr) - 1.0) < 0.25, "slide braking trajectory: predicted +0.5 s distance within 25 % of the real one",
              sumRatioOn / double(nErr));
    }
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mm_slide_feed_test <slide_feeds.txt> [<chooser.json with slide> [<chooser.json without>]]\n");
        return 2;
    }
    std::vector<Slide> slides;
    if (!LoadFixture(argv[1], slides))
    {
        std::printf("FAIL: fixture %s unreadable\n", argv[1]);
        return 1;
    }
    TestSpec(argc > 2 ? argv[2] : nullptr, argc > 3 ? argv[3] : nullptr);
    TestSynthetic();
    TestReplay(slides);
    std::printf("slide feed: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
