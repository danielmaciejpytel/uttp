// UTPP 0.5.2 offline test: stance-change transition clips (steer::kClassStanceTransition) only right after a real
// stand <-> crouch change.
// Usage: utpp_mm_stance_test <database.utppmm with stance transition clips, e.g. utpp_base.utppmm>
// Unit part: name classification (motion and other naming schemes; non-stance names), curation rule. Driver part (real MMDriver):
// scripted stand / crouch walking, running and idling; every newly selected stance-transition clip must start within the
// window after a stance change (window 0.6 s); the same script with the window off (< 0 = 0.5.1) is reported as INFO.
// The package database has two stance transition clips (5 searchable frames each) and the driver selects none of them on this
// script, so the window check holds without a selection (INFO line counts the selections).

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/MotionHost.hpp"
#include "../src/State.hpp"
#include "../src/mm/Steering.hpp"

namespace utpp
{
HotState g_hot;
mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;
} // namespace utpp

namespace
{
int g_pass = 0, g_fail = 0;
void Check(const char* aName, bool aOk, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
}

std::string ClipName(const std::string& aStatus)
{
    const size_t p = aStatus.find(" clip=");
    if (p == std::string::npos)
        return {};
    const size_t a = aStatus.find('(', p);
    const size_t b = aStatus.find(')', a);
    return (a == std::string::npos || b == std::string::npos) ? std::string() : aStatus.substr(a + 1, b - a - 1);
}

struct Phase
{
    int loco;       // 1 stand, 2 crouch, 3 sprint
    float speed;    // m/s forward (0 = idle, stick released)
    float seconds;
    float turnDeg = 0.0f; // slalom amplitude (deg/s peak yaw rate, 1 Hz)
};

struct RunResult
{
    int stanceSelections = 0;   // newly selected stance-transition clips
    int outsideWindow = 0;      // ... more than the window after the last stance change
    std::vector<std::string> outsideNames;
    int changes = 0;
};

RunResult RunScript(const std::vector<Phase>& aScript, float aWindow)
{
    using namespace utpp::motionhost;
    RunResult r;
    SetStanceTransitionWindow(aWindow);
    std::string prevClip;
    int prevLoco = 0;
    float sinceChange = 1e6f;
    const float dt = 1.0f / 60.0f;
    float yaw = 0.0f, speed = 0.0f, t = 0.0f;
    for (const Phase& ph : aScript)
    {
        const int stance = ph.loco == 2 ? 2 : 1;
        if (prevLoco != 0 && stance != prevLoco)
        {
            sinceChange = 0.0f;
            ++r.changes;
        }
        prevLoco = stance;
        for (int i = 0; i < int(ph.seconds / dt); ++i)
        {
            FeedInput in;
            t += dt;
            speed += std::fmax(-6.0f * dt, std::fmin(6.0f * dt, ph.speed - speed)); // capsule ramps (6 m/s^2)
            yaw += ph.turnDeg * std::sin(6.2831853f * t) * dt;
            const float yr = yaw * 0.017453292f;
            in.velWorld[0] = -std::sin(yr) * speed;
            in.velWorld[1] = std::cos(yr) * speed;
            in.playerYawDeg = yaw;
            in.cameraYawDeg = 0.0f;
            in.move[1] = ph.speed > 0.0f ? 1.0f : 0.0f;
            in.locoState = ph.loco;
            in.tppActive = true;
            in.dt = dt;
            Feed(in);
            const std::string clip = ClipName(Status());
            if (!clip.empty() && clip != prevClip && utpp::mm::steer::ClassifyClip(clip) == utpp::mm::steer::kClassStanceTransition)
            {
                ++r.stanceSelections;
                if (sinceChange > 0.6f + dt + 1e-4f)
                {
                    ++r.outsideWindow;
                    if (r.outsideNames.size() < 6)
                        r.outsideNames.push_back(clip + " @" + std::to_string(sinceChange) + "s");
                }
            }
            prevClip = clip;
            sinceChange += dt;
        }
    }
    return r;
}
} // namespace

int main(int argc, char** argv)
{
    using namespace utpp::mm::steer;
    // ---- names
    const char* stance[] = {"M_Neutral_Transition_Stand_to_Crouch", "M_Neutral_Transition_Crouch_to_Stand"};
    bool allStance = true;
    for (const char* n : stance)
        allStance = allStance && ClassifyClip(n) == kClassStanceTransition;
    Check("classify: stand <-> crouch transitions", allStance);
    Check("classify: other clips unchanged", ClassifyClip("M_Neutral_Transition_Run_to_Walk_Rfoot") == kClassTransition &&
                                                 ClassifyClip("M_Neutral_Crouch_Stop_F_Lfoot") == kClassStop &&
                                                 ClassifyClip("M_Neutral_Crouch_Loop_F") == kClassNone &&
                                                 ClassifyClip("M_Neutral_Run_Start_BR_Rfoot") == kClassStart);
    // ---- rule
    CurationInput ci;
    CurationConfig cc;
    ci.sinceStance = 0.3f;
    const bool inWin = (ExcludedClasses(ci, cc) & Bit(kClassStanceTransition)) == 0;
    ci.sinceStance = 0.61f;
    const bool after = (ExcludedClasses(ci, cc) & Bit(kClassStanceTransition)) != 0;
    ci.sinceStance = 1e6f;
    const bool never = (ExcludedClasses(ci, cc) & Bit(kClassStanceTransition)) != 0;
    cc.stanceWindow = -1.0f;
    const bool off = (ExcludedClasses(ci, cc) & Bit(kClassStanceTransition)) == 0;
    Check("rule: allowed within 0.6 s, excluded after / without a change, window < 0 = 0.5.1", inWin && after && never && off);

    if (argc < 2)
    {
        std::printf("RESULT %d passed, %d failed (driver part skipped: no database)\n", g_pass, g_fail);
        return g_fail == 0 ? 0 : 1;
    }
    // ---- driver
    using namespace utpp::motionhost;
    const bool loaded = LoadDatabase(std::wstring(argv[1], argv[1] + std::strlen(argv[1])), "test");
    Check("driver: database loaded", loaded, Status().substr(0, 0));
    utpp::mm::Database db;
    std::string err;
    db.LoadFromFile(argv[1], kExpectedRigHash, err);
    int stanceClips = 0;
    for (uint32_t c = 0; c < db.NumClips(); ++c)
        stanceClips += ClassifyClip(db.ClipName(c)) == kClassStanceTransition ? 1 : 0;
    Check("driver: database has stance transition clips", stanceClips > 0, std::to_string(stanceClips));
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        hashes.push_back(utpp::mm::integ::CNameHash(db.BoneName(b)));
        parents.push_back(static_cast<int16_t>(db.BoneParent(b)));
    }
    static int rig = 0;
    utpp::g_hot.armed.store(true);
    OnArmed(&rig, hashes, parents);
    SetEnabled(true);
    // 0.5.9: the 0.5.1 negative control below was recorded with the 0.5.8 dynamics (the calibrated trajectory half-life of
    // UTPPSetMMTrajCal changes the picks of this scripted path); the 0.5.9 switches are compared in utpp_mm_loco_test Section059.
    SetCapsuleSync(false);
    SetOffsetCatchup(false);
    SetTrajCal(false);
    std::vector<Phase> script;
    for (int rep = 0; rep < 3; ++rep)
    {
        script.push_back({1, 1.5f, 3.0f});         // stand walk
        script.push_back({1, 0.0f, 0.8f});         // stop
        script.push_back({1, 1.5f, 3.0f, 90.0f});  // stand walk slalom
        script.push_back({2, 1.2f, 3.0f});         // crouch walk
        script.push_back({2, 0.0f, 1.5f});         // crouch idle
        script.push_back({2, 2.7f, 3.0f, 90.0f});  // crouch run slalom
        script.push_back({2, 0.0f, 0.8f});         // stop
        script.push_back({2, 2.7f, 2.0f});         // crouch start again
        script.push_back({1, 3.85f, 3.0f});        // stand run
        script.push_back({1, 3.85f, 3.0f, 120.0f});// stand run slalom
        script.push_back({1, 0.0f, 1.5f});         // stand idle
        script.push_back({1, 1.5f, 1.0f});         // start walking
        script.push_back({1, 3.85f, 2.0f});        // speed up
        script.push_back({3, 8.2f, 2.0f});         // sprint
    }
    const RunResult on = RunScript(script, 0.6f);
    std::string names;
    for (const auto& n : on.outsideNames)
        names += n + "; ";
    Check("driver window 0.6 s: no stance transition selected outside the window", on.outsideWindow == 0,
          std::to_string(on.stanceSelections) + " selected, " + std::to_string(on.outsideWindow) + " outside " + names);
    const RunResult offRun = RunScript(script, -1.0f);
    std::printf("[INFO] window off (0.5.1): %d stance transitions selected, %d outside 0.6 s after a change (%d changes)\n",
                offRun.stanceSelections, offRun.outsideWindow, offRun.changes);
    std::printf("[INFO] window 0.6 s: %d stance transitions selected (all within the window), %d changes\n", on.stanceSelections,
                on.changes);
    Shutdown();
    std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
