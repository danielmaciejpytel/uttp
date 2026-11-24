// Speed converter, trajectory prediction (speedconv::kTrajectory, mm/speedconv_traj.hpp): the weight of the converted prediction, the
// prediction from the converted state through a sharp turn, and the hook on a real query trajectory (history, times and heights kept,
// a held loop unchanged). Mutations: 30 = the plain prediction always, 31 = the prediction starts from the game velocity, 32 = the
// current game speed over the whole horizon (the start ramp of the plain prediction lost).
#include "../src/mm/speedconv_traj.hpp"
#include <cmath>
#include <cstdio>

using namespace utpp::mm;
using motiontraj::V3;

static int g_fail = 0;
static void Check(bool aOk, const char* aWhat)
{
    if (!aOk)
    {
        std::printf("FAIL %s\n", aWhat);
        ++g_fail;
    }
}
static float AngleDeg(V3 a, V3 b)
{
    return std::fabs(motiontraj::NormalizeAxis(motiontraj::YawOf(a) - motiontraj::YawOf(b)));
}

int main()
{
    const float dt = 1.0f / 60.0f;
    speedconv::Limits lim;
    lim.friction = 2.0f;
    lim.maxTurnCos = std::cos(150.0f / motiontraj::kRad2Deg);
    const V3 runX{400.0f, 0.0f, 0.0f}, runY{0.0f, 400.0f, 0.0f};

    // weight: 0 when the converted and the game velocity agree, 1 from 5 deg / 25 cm/s, linear below
    Check(speedconv::TrajectoryWeight(runX, runX) == 0.0f, "traj weight: equal velocities keep the plain prediction");
    Check(speedconv::TrajectoryWeight(runX, runY) == 1.0f, "traj weight: a 90 deg divergence is the full converted prediction");
    const float half = speedconv::TrajectoryWeight(runX, motiontraj::RotateYaw(runX, 2.5f));
    Check(std::fabs(half - 0.5f) < 0.01f, "traj weight: 2.5 deg is half");
    Check(std::fabs(speedconv::TrajectoryWeight(runX, V3{412.5f, 0.0f, 0.0f}) - 0.5f) < 0.01f, "traj weight: 12.5 cm/s is half");

    // a run along +x, then the input (and the game velocity) turns to +y
    speedconv::State s;
    for (int i = 0; i < 60; ++i)
        speedconv::Step(s, runX, true, true, dt, lim, false);
    const V3 conv = speedconv::Step(s, runY, true, true, dt, lim, false);
    Check(AngleDeg(conv, runX) < 10.0f, "traj setup: one feed after the turn the converted heading is still near the old one");
    V3 off[motiontraj::kPredictionCount + 1];
    float speed400[motiontraj::kPredictionCount];
    for (float& v : speed400)
        v = 400.0f;
    speedconv::PredictConvertedOffsets(s, runY, runY, 0.0f, speed400, true, lim, false, false, off);
    Check(off[0].x == 0.0f && off[0].y == 0.0f, "traj predict: the first offset is the current position");
    // the first 0.1 s continues along the converted heading (the game heading would be +y: x = 0)
    Check(off[1].x > off[1].y, "traj predict: the prediction starts along the converted heading");
    Check(off[1].x > 20.0f, "traj predict: the first interval moves on along the old heading");
    // the end of the horizon heads along the input (the converter converges)
    const V3 lastStep = off[motiontraj::kPredictionCount] - off[motiontraj::kPredictionCount - 1];
    Check(AngleDeg(lastStep, runY) < 15.0f, "traj predict: the end of the horizon follows the input");
    // the curve is monotonic in heading (no overshoot past the input)
    bool monotonic = true;
    float prevYaw = 0.0f;
    for (int k = 1; k <= motiontraj::kPredictionCount; ++k)
    {
        const V3 st = off[k] - off[k - 1];
        const float yaw = motiontraj::YawOf(st);
        if (yaw < prevYaw - 0.5f || yaw > 90.5f)
            monotonic = false;
        prevYaw = yaw;
    }
    Check(monotonic, "traj predict: the heading turns monotonically to the input without overshoot");
    // the state passed in is not changed
    Check(s.vel.x == conv.x && s.vel.y == conv.y, "traj predict: the converter state is not changed");

    // the hook on a query trajectory
    motiontraj::MotionCmc cmc;
    const motiontraj::Derived d = motiontraj::MakeDerived(cmc, 1, 0, 1.0f, 400.0f, V3{0.0f, 800.0f, 0.0f}, true, false);
    motiontraj::Trajectory plain, hooked;
    speedconv::State hs;
    for (int i = 0; i < 60; ++i)
    {
        const V3 q = speedconv::Step(hs, runX, true, true, dt, lim, false);
        plain.Update(dt, q, V3{800.0f, 0.0f, 0.0f}, d, 980.0f);
        hooked.Update(dt, q, V3{800.0f, 0.0f, 0.0f}, d, 980.0f);
        const float w = speedconv::ApplyConvertedPrediction(hooked, hs, runX, V3{800.0f, 0.0f, 0.0f}, true, lim, false, false, false);
        if (i > 30)
            Check(w == 0.0f, "traj hook: a held run keeps the plain prediction");
    }
    for (int k = 0; k <= motiontraj::kPredictionCount; ++k)
        Check(plain.Prediction(k).pos.x == hooked.Prediction(k).pos.x && plain.Prediction(k).pos.y == hooked.Prediction(k).pos.y,
              "traj hook: a held run is bit for bit the plain prediction");
    const V3 q = speedconv::Step(hs, runY, true, true, dt, lim, false);
    plain.Update(dt, q, V3{0.0f, 800.0f, 0.0f}, d, 980.0f);
    hooked.Update(dt, q, V3{0.0f, 800.0f, 0.0f}, d, 980.0f);
    const float w = speedconv::ApplyConvertedPrediction(hooked, hs, runY, V3{0.0f, 800.0f, 0.0f}, true, lim, false, false, false);
    Check(w == 1.0f, "traj hook: the converted prediction replaces the plain one through a sharp turn");
    // the plain prediction (gait friction 5 in 0.1 s steps) turns sooner than the converted one (friction 2)
    Check(hooked.Prediction(5).pos.x > plain.Prediction(5).pos.x + 10.0f,
          "traj hook: the converted prediction keeps more of the old heading at 0.5 s");
    bool kept = true;
    for (int k = 0; k <= motiontraj::kPredictionCount; ++k)
        kept = kept && hooked.Prediction(k).t == plain.Prediction(k).t && hooked.Prediction(k).pos.z == plain.Prediction(k).pos.z &&
               hooked.Prediction(k).facing == plain.Prediction(k).facing;
    for (int i = 0; i < hooked.NumHistory(); ++i)
        kept = kept && hooked.History(i).pos.x == plain.History(i).pos.x && hooked.History(i).pos.y == plain.History(i).pos.y;
    Check(kept, "traj hook: history, times, heights and facings are kept");
    Check(hooked.Prediction(0).pos.x == plain.Prediction(0).pos.x && hooked.Prediction(0).pos.y == plain.Prediction(0).pos.y,
          "traj hook: the current sample is kept");
    // no input: unchanged
    motiontraj::Trajectory idle = plain;
    Check(speedconv::ApplyConvertedPrediction(idle, hs, runY, V3{}, false, lim, false, false, false) == 0.0f,
          "traj hook: no input keeps the plain prediction");

    // a slow move turning into a speed-up: the converted prediction keeps the speed ramp of the plain prediction (path length at 1.5 s)
    {
        const V3 slowX{150.0f, 0.0f, 0.0f}, slowY{0.0f, 150.0f, 0.0f};
        motiontraj::Trajectory p2, h2;
        speedconv::State s2;
        for (int i = 0; i < 30; ++i)
        {
            const V3 q2 = speedconv::Step(s2, slowX, true, true, dt, lim, false);
            p2.Update(dt, q2, V3{800.0f, 0.0f, 0.0f}, d, 980.0f);
        }
        const V3 q2 = speedconv::Step(s2, slowY, true, true, dt, lim, false);
        p2.Update(dt, q2, V3{0.0f, 800.0f, 0.0f}, d, 980.0f);
        h2 = p2;
        motiontraj::Trajectory g2 = p2;
        const float w2 = speedconv::ApplyConvertedPrediction(h2, s2, slowY, V3{0.0f, 800.0f, 0.0f}, true, lim, false, false, true);
        speedconv::ApplyConvertedPrediction(g2, s2, slowY, V3{0.0f, 800.0f, 0.0f}, true, lim, false, false, false);
        float lp = 0.0f, lh = 0.0f, lg = 0.0f;
        for (int k = 0; k < motiontraj::kPredictionCount; ++k)
        {
            lg += motiontraj::SizeXY(g2.Prediction(k + 1).pos - g2.Prediction(k).pos);
            lp += motiontraj::SizeXY(p2.Prediction(k + 1).pos - p2.Prediction(k).pos);
            lh += motiontraj::SizeXY(h2.Prediction(k + 1).pos - h2.Prediction(k).pos);
        }
        Check(w2 == 1.0f && lp > 1.5f * 150.0f * 1.5f, "traj ramp setup: the plain prediction speeds up through the turn");
        if (!(std::fabs(lh - lp) <= 0.1f * lp))
            std::printf("  path length plain %.1f converted %.1f\n", lp, lh);
        Check(std::fabs(lh - lp) <= 0.1f * lp, "traj ramp: the converted prediction keeps the speed of the plain prediction");
        // without a start forecast the game speed is held over the horizon
        Check(std::fabs(lg - 150.0f * 1.5f) <= 0.05f * 150.0f * 1.5f, "traj hold: without a start forecast the game speed is held");
    }

    // a start: a forecast, or the game speed below 0.9 of the cap; a loop near the cap holds the game speed
    Check(speedconv::TrajectoryRamp(true, 480.0f, 500.0f) && speedconv::TrajectoryRamp(false, 200.0f, 500.0f) &&
              !speedconv::TrajectoryRamp(false, 470.0f, 500.0f),
          "traj ramp choice: starts chase the plain ramp, loops hold the game speed");

    if (g_fail == 0)
        std::printf("speedconv_traj: all checks passed\n");
    return g_fail == 0 ? 0 : 1;
}
