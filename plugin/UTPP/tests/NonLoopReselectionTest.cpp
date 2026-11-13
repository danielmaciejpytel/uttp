// Focused regression: a non-loop clip's excluded search tail must not erase
// locomotion phase when the adapter has no caller PoseHistory. Local motion data.
// Usage: test <database.utppmm> <chooser.json>
#define main FullMatcherSuiteMainNotRun
#include "ChooserMatcherTest.cpp"
#undef main

namespace
{
std::vector<TrajSample> ConstantTrajectory(float speed)
{
    std::vector<TrajSample> result;
    for (int k = -60; k <= 96; ++k)
    {
        TrajSample sample;
        sample.t = k / 60.f;
        sample.y = speed * sample.t;
        result.push_back(sample);
    }
    return result;
}

bool QueryMatchesHistory(ChooserMatcher& matcher, const MotionDatabase& db, const MatchResult& result,
                         const std::vector<TrajSample>& trajectory, const PoseHistory* history)
{
    if (result.database < 0)
        return false;
    const uint32_t dims = db.Db(uint32_t(result.database)).dims;
    const std::vector<float> actual(matcher.RawQuery(), matcher.RawQuery() + dims);
    matcher.SetQueryInputs(trajectory.data(), uint32_t(trajectory.size()), history);
    matcher.BuildQuery(uint32_t(result.database), false);
    for (uint32_t i = 0; i < dims; ++i)
        if (std::fabs(actual[i] - matcher.RawQuery()[i]) > 1e-5f)
            return false;
    return true;
}
}

int main(int argc, char** argv)
{
    if (argc != 3)
        return 2;
    MotionDatabase db;
    chooser::Table table;
    std::string error;
    if (!db.LoadFromFile(argv[1], kRigIdentityWoman, error) || !table.LoadFile(argv[2], error))
    {
        std::printf("load: %s\n", error.c_str());
        return 2;
    }
    constexpr float dt = 1.f / 60;
#ifdef UTPP_NLR_NO_LAST_POSE_MUTATION
    // Mutant (pre-fix behaviour): the last-pose fallback acts only without a caller PoseHistory, and an EMPTY history leaves every pose
    // channel at the database mean, so the excluded non-loop tail loses its phase again and the same transition phase is re-selected
    // ("repeated non-loop phase" / "stable loop wraps" MUST fail). Production code is not touched.
    static const PoseHistory kMainHistory{};
    const PoseHistory* const mainHistory = &kMainHistory;
#else
    const PoseHistory* const mainHistory = nullptr;
#endif
    for (int gait : {1, 2})
    {
        const float speed = gait == 1 ? 5.f : 7.65f;
        const auto trajectory = ConstantTrajectory(speed);
        const ChooserState state = Ground(1, gait, speed);
        ChooserMatcher matcher, edge;
        CHECK(matcher.Init(&db, &table, MatcherConfig{}, error), "init");
        MatchResult initial, previous, edgeResult;
        bool haveEdge = false;
        int repeats = 0, loopWraps = 0, invalid = 0, badBlends = 0;
        for (int frame = 0; frame < 1200; ++frame)
        {
            const MatchResult result = matcher.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, mainHistory);
            if (frame == 0)
                initial = result;
            if (result.clip < 0)
            {
                ++invalid;
                continue;
            }
            const auto& clip = db.Clip(uint32_t(result.clip));
            const bool rewind = previous.clip == result.clip && result.clipTime + .1f < previous.clipTime;
            repeats += rewind && result.jumped && !(clip.flags & motion::kClipLoop);
            loopWraps += rewind && !result.jumped && (clip.flags & motion::kClipLoop);
            badBlends += frame > 0 && result.jumped && (!result.blend.valid || result.blend.blendTime <= 0);
            if (!haveEdge && result.clip == initial.clip)
            {
                const auto& entry = db.Entry(result.entry);
                const uint32_t finalFrame = db.Row(entry.firstRow + entry.numRows - 1).frame;
                const uint32_t nextFrame = clip.firstFrame + uint32_t(std::lround(
                    (result.clipTime + dt * result.playRate) * db.Header().fps));
                if (nextFrame > finalFrame)
                {
                    edge = matcher;
                    edgeResult = result;
                    haveEdge = true;
                }
            }
            previous = result;
        }
        CHECK(initial.clip >= 0 && !(db.Clip(uint32_t(initial.clip)).flags & motion::kClipLoop),
              "gait%d transition remains selectable", gait);
        CHECK(haveEdge, "gait%d reached excluded tail", gait);
        CHECK(repeats == 0, "gait%d repeated non-loop phase=%d", gait, repeats);
        CHECK(loopWraps >= 2, "gait%d stable loop wraps=%d", gait, loopWraps);
        CHECK(invalid == 0 && badBlends == 0, "gait%d invalid=%d badBlends=%d", gait, invalid, badBlends);
        if (!haveEdge)
            continue;

        auto forced = edge;
        const Interrupt force = Interrupt::ForceInterrupt;
        const auto forcedResult = forced.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, nullptr, &force);
        CHECK(forcedResult.clip != edgeResult.clip, "gait%d non-invalidating interrupt keeps phase", gait);

        auto invalidated = edge;
        const Interrupt invalidate = Interrupt::ForceInterruptAndInvalidateContinuingPose;
        const auto invalidatedResult = invalidated.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state,
                                                         nullptr, &invalidate);
        CHECK(QueryMatchesHistory(invalidated, db, invalidatedResult, trajectory, nullptr),
              "gait%d invalidation uses neutral query, not stale pose", gait);

        const Interrupt invalidateOnChange = Interrupt::InterruptOnDatabaseChangeAndInvalidateContinuingPose;
        auto sameDatabase = edge;
        const auto sameResult = sameDatabase.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state,
                                                     nullptr, &invalidateOnChange);
        CHECK(sameResult.clip != edgeResult.clip, "gait%d unchanged database does not invalidate phase", gait);
        auto changedDatabase = edge;
        const auto changedState = Ground(1, gait == 1 ? 2 : 1, speed);
        const auto changedResult = changedDatabase.Update(dt, trajectory.data(), uint32_t(trajectory.size()),
                                                          changedState, nullptr, &invalidateOnChange);
        CHECK(!((changedResult.dbMask >> uint32_t(edgeResult.database)) & 1u) &&
                  QueryMatchesHistory(changedDatabase, db, changedResult, trajectory, nullptr),
              "gait%d leaving database invalidates fallback", gait);

        PoseHistory history;
        history.numSamples = 2;
        history.numBones = 3;
        history.bone[0] = 11;
        history.bone[1] = 12;
        history.bone[2] = 2;
        for (uint32_t sample = 0; sample < history.numSamples; ++sample)
        {
            history.root[sample].t = (float(sample) - 1) * dt;
            history.root[sample].y = history.root[sample].t * speed;
            for (uint32_t bone = 0; bone < history.numBones; ++bone)
            {
                history.gp[sample][bone] = {float(bone) * .2f, .1f * sample, .25f};
                history.gq[sample][bone] = {0, 0, 0, 1};
            }
        }
        auto supplied = edge;
        const auto suppliedResult = supplied.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, &history);
        CHECK(QueryMatchesHistory(supplied, db, suppliedResult, trajectory, &history),
              "gait%d explicit history wins over fallback", gait);

        auto reset = edge;
        reset.Reset();
        const auto resetResult = reset.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state);
        CHECK(resetResult.clip == initial.clip && resetResult.clipTime == initial.clipTime && !resetResult.blend.valid,
              "gait%d reset has no stale fallback", gait);
        std::printf("gait%d: repeats=%d loopWraps=%d invalid=%d badBlends=%d edge=%d@%.5f\n",
                    gait, repeats, loopWraps, invalid, badBlends, edgeResult.clip, edgeResult.clipTime);
    }
    std::printf("NonLoopReselection: PASS=%d FAIL=%d\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
