// Focused real-database regression for a crouch stop selecting standing-entry
// animation seconds after the stance request (retail session 2026-10-19 02:43).
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "../src/mm/ChooserMatcher.hpp"

using namespace utpp::mm;
using namespace utpp::mm::motion;
static int passed = 0, failed = 0;
#define CHECK(c, msg) do { if (c) ++passed; else { ++failed; std::printf("FAIL %s\n", msg); } } while (0)

struct Rig
{
    MotionDatabase db;
    chooser::Table table;
    ChooserMatcher matcher;
    std::vector<TrajSample> trajectory;
    float dt = 1.f / 60.f;
    bool Load(const char* path)
    {
        std::string err, chooserPath(path);
        chooserPath.replace(chooserPath.find_last_of('.'), std::string::npos, ".chooser.json");
        if (!db.LoadFromFile(path, kRigIdentityWoman, err) || !table.LoadFile(chooserPath.c_str(), err) ||
            !matcher.Init(&db, &table, {}, err))
        { std::printf("LOAD %s\n", err.c_str()); return false; }
        return true;
    }
    MatchResult Step(ChooserState state, float speed = 0.f, float step = -1.f, const Interrupt* interrupt = nullptr)
    {
        trajectory.clear();
        for (int i = -60; i <= 96; ++i)
        {
            TrajSample sample;
            sample.t = i / 60.f;
            sample.y = speed * sample.t;
            trajectory.push_back(sample);
        }
        return matcher.Update(step < 0 ? dt : step, trajectory.data(), uint32_t(trajectory.size()), state, nullptr, interrupt);
    }
    bool Transition(const MatchResult& r) const
    { return r.clip >= 0 && steer::ClassifyClip(db.ClipName(uint32_t(r.clip))) == steer::kClassStanceTransition; }
    int Run(ChooserState state, float seconds, float speed = 0.f)
    {
        int picks = 0;
        for (int i = 0; i < int(std::lround(seconds / dt)); ++i)
        {
            const auto r = Step(state, speed);
            if (r.clip < 0) { ++failed; std::printf("FAIL missing pose\n"); break; }
            if (r.jumped && Transition(r)) ++picks;
        }
        return picks;
    }
};

static ChooserState State(int stance = 0, bool moving = false)
{
    ChooserState s;
    s.stance = stance;
    s.movementState = moving ? 1 : 0;
    s.gait = moving ? 1 : 0;
    s.speed2dCmS = moving ? (stance ? 150.f : 400.f) : 0.f;
    return s;
}

int main(int argc, char** argv)
{
    Rig rig;
    if (!rig.Load(argc > 1 ? argv[1] : "Build/Run153919/fixture/utpp_motion_woman.utppmm")) return 2;
    for (const int hz : {30, 60, 144})
    {
        rig.dt = 1.f / hz;
        const auto stand = State(), crouch = State(1), move = State(0, true), crouchMove = State(1, true);
        rig.matcher.Reset();
        CHECK(rig.Run(crouch, 8.f) == 0, "already crouched initialization must not stand then crouch");

        rig.matcher.Reset();
        rig.Run(stand, 1.f);
        int picks = 0, continuationPastWindow = 0;
        for (int f = 0; f < hz * 10; ++f)
        {
            const auto r = rig.Step(crouch);
            picks += r.jumped && rig.Transition(r);
            continuationPastWindow += !r.jumped && rig.Transition(r) && f > hz;
        }
        CHECK(picks == 1, "real idle stance edge admits exactly one transition");
        CHECK(continuationPastWindow > 0, "admitted player continues beyond admission window");
        rig.matcher.Reset(true);
        CHECK(rig.Run(crouch, 4.f) == 0, "Reset(true) cannot replay consumed transition");
        CHECK(rig.Run(stand, 3.f) == 1, "real crouch to stand admits opposite transition");
        CHECK(rig.Run(crouch, 4.f) == 1, "subsequent real stance edge re-arms admission");

        // The observed path: entering crouch while still moving does not select
        // Stand_to_Crouch. Its database appears only after the stop, several
        // seconds later. A completed-clip latch alone cannot prevent this.
        rig.matcher.Reset();
        rig.Run(move, 2.f, 4.f);
        rig.Run(crouchMove, 2.f, 1.5f);
        const int latePicks = rig.Run(crouch, 16.f);
        CHECK(latePicks == 0, "moving stand -> crouch -> stop -> idle has no late stance entry");
        std::printf("Hz=%d moving-to-crouch-stop lateTransitions=%d\n", hz, latePicks);

        rig.matcher.Reset();
        rig.Run(move, 1.f, 4.f);
        rig.Step(crouchMove, 1.5f);
        rig.matcher.Reset(true);
        CHECK(rig.Run(crouch, 4.f) == 0, "yield cancels unconsumed window rather than reopening it");

        rig.matcher.Reset();
        rig.Run(stand, 1.f);
        auto air = crouch;
        air.movementMode = 1;
        rig.Run(air, .3f);
        CHECK(rig.Run(crouch, 4.f) == 0, "air stance change and landing are not a ground stance edge");

        rig.matcher.Reset();
        rig.Run(stand, 1.f);
        auto traversed = crouch;
        traversed.justTraversed = true;
        rig.Step(traversed);
        CHECK(rig.Run(crouch, 4.f) == 0, "traversal handback does not admit a standing-entry animation");

        rig.matcher.Reset();
        rig.Run(stand, 1.f);
        const auto zero = rig.Step(crouch, 0.f, 0.f);
        CHECK(!(zero.jumped && rig.Transition(zero)), "dt0 stance change does not consume a new transition");
        // Ordinary cost search may already prefer crouch idle after the zero-
        // time evaluation. Invalidate its query once to test admission, rather
        // than incorrectly requiring the policy to force a transition choice.
        const Interrupt invalidate = Interrupt::ForceInterruptAndInvalidateContinuingPose;
        const auto afterZero = rig.Step(crouch, 0.f, -1.f, &invalidate);
        CHECK(int(afterZero.jumped && rig.Transition(afterZero)) + rig.Run(crouch, 4.f) == 1,
            "dt0 observed edge remains admissible once when time advances");

        rig.matcher.Reset();
        rig.Run(move, 1.f, 4.f);
        rig.Step(crouchMove, 1.5f);
        rig.Run(crouchMove, .7f, 1.5f);
        const auto expired = rig.Step(crouch, 0.f, -1.f, &invalidate);
        CHECK(!(expired.jumped && rig.Transition(expired)) && rig.Run(crouch, 4.f) == 0,
            "unconsumed stance admission expires even before a stop database appears");

        rig.matcher.Reset();
        rig.Run(stand, 1.f);
        rig.Step(crouch);
        const Interrupt force = Interrupt::ForceInterrupt;
        const auto forced = rig.Step(crouch, 0.f, -1.f, &force);
        CHECK(!(forced.jumped && rig.Transition(forced)), "forced search cannot replay consumed stance entry");

        rig.matcher.Reset();
        rig.Run(stand, 1.f);
        rig.Step(crouch);
        rig.Run(stand, .2f);
        CHECK(rig.Run(crouch, 4.f) == 1, "rapid real reversal permits the new destination exactly once");

        rig.matcher.Reset();
        rig.Run(stand, 1.f);
        const auto gap = rig.Step(crouch, 0.f, 1.f);
        CHECK(!(gap.jumped && rig.Transition(gap)) && rig.Run(crouch, 4.f) == 0,
            "long gap cannot manufacture a fresh stance edge");
    }
    std::printf("CrouchStanceAdmission: %d PASS, %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}
