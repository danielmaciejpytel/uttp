// Vehicle-exit pose seed (mm/ExitPoseHistory.hpp, MotionDriver VehicleExitHistoryLocked): the first v2 search after a vehicle exit re-entry
// reads the pose captured at the exit edge instead of the database mean.
//   A. exact space: the PoseHistory built from the database local pose of a row (the same helper production feeds with the capture)
//      makes BuildQuery reproduce that row's Position / Heading features and zero velocity (FORMAT.txt v2 "Feature values").
//   B. search: with such a history the first search after a reset takes that row (cost ~0) instead of the mean-pose winner.
//   C. without a history (nullptr) the first search after a reset is unchanged (= a freshly initialised matcher).
//   D. fail closed: a capture without the foot chains produces no history for those bones; none at all = false.
// Local motion data. Usage: test <database.utppmm> <chooser.json>
#define main FullMatcherSuiteMainNotRun
#include "ChooserMatcherTest.cpp"
#undef main

#include "../src/mm/ExitPoseHistory.hpp"

namespace
{
std::vector<TrajSample> StillTrajectory()
{
    std::vector<TrajSample> result;
    for (int k = -60; k <= 96; ++k)
    {
        TrajSample sample;
        sample.t = k / 60.f;
        result.push_back(sample);
    }
    return result;
}

// local pose of frame aFrame of the database as the production callback would deliver the capture
bool HistoryOfFrame(const MotionDatabase& db, uint32_t aFrame, PoseHistory& aHist)
{
    const PoseRecord* pose = db.Pose(aFrame);
    return BuildExitPoseHistory(
        db, 2,
        [&](uint32_t i, math::Vec3& p, math::Quat& q) {
            p = math::Vec3{pose[i].px, pose[i].py, pose[i].pz};
            q = math::Quat{pose[i].qx, pose[i].qy, pose[i].qz, pose[i].qw};
            return true;
        },
        TrajSample{}, aHist);
}

int FindDb(const MotionDatabase& db, const char* aName)
{
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
        if (db.DbName(d) == aName)
            return static_cast<int>(d);
    return -1;
}
} // namespace

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
    const int idleDb = FindDb(db, "PSD_Dense_Stand_Idles");
    CHECK(idleDb >= 0, "PSD_Dense_Stand_Idles present");
    if (idleDb < 0)
        return 1;
    const DatabaseRecord& D = db.Db(static_cast<uint32_t>(idleDb));
    const SchemaRecord& S = db.Schema(D.schema);
    const auto trajectory = StillTrajectory();
    const ChooserState state = Ground(0, 0, 0.0f);
    constexpr float dt = 1.f / 60;

    // ---- C. reference: the first search of a fresh matcher, and of a matcher reset after a long run (the re-entry path), no history
    ChooserMatcher fresh;
    CHECK(fresh.Init(&db, &table, MatcherConfig{}, error), "init");
    const MatchResult base = fresh.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, nullptr);
    CHECK(base.clip >= 0 && base.database == idleDb, "mean-pose first pick is in PSD_Dense_Stand_Idles (db %d)", base.database);
    ChooserMatcher rerun;
    CHECK(rerun.Init(&db, &table, MatcherConfig{}, error), "init rerun");
    for (int i = 0; i < 90; ++i)
        rerun.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, nullptr);
    rerun.Reset(true);
    const MatchResult again = rerun.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, nullptr);
    CHECK(again.clip == base.clip && again.localRow == base.localRow && again.clipTime == base.clipTime && again.cost == base.cost,
          "C: no history = unchanged first search after a reset (clip %d/%d row %d/%d cost %.6f/%.6f)", again.clip, base.clip,
          again.localRow, base.localRow, double(again.cost), double(base.cost));

    // ---- A / B over rows of the idle database
    uint32_t tested = 0, exactRows = 0, preferred = 0, differentFrame = 0;
    double worstPose = 0.0, worstVel = 0.0;
    const uint32_t step = std::max(1u, D.numRows / 24u);
    for (uint32_t r = 0; r < D.numRows; r += step)
    {
        const RowRecord& row = db.Row(D.firstRow + r);
        if (row.flags & kRowBlock)
            continue;
        PoseHistory hist;
        if (!HistoryOfFrame(db, row.frame, hist))
        {
            CHECK(false, "history of frame %u", row.frame);
            continue;
        }
        ++tested;
        ChooserMatcher m;
        CHECK(m.Init(&db, &table, MatcherConfig{}, error), "init row");
        m.SetQueryInputs(trajectory.data(), uint32_t(trajectory.size()), &hist);
        m.BuildQuery(static_cast<uint32_t>(idleDb), false);
        const float* mean = db.NormMean(D.norm);
        const float* scale = db.NormScale(D.norm);
        const float* f = db.Features(static_cast<uint32_t>(idleDb), r);
        bool exact = true;
        for (uint32_t c = S.firstChannel; c < S.firstChannel + S.numChannels; ++c)
        {
            const ChannelRecord& ch = db.Channel(c);
            if (ch.flags & kChanTrajectory)
                continue;
            for (uint32_t k = 0; k < ch.cardinality; ++k)
            {
                const uint32_t o = ch.dataOffset + k;
                const double q = m.RawQuery()[o];
                if (ch.kind == 2 || ch.kind == 3) // held pose: no foot velocity
                {
                    worstVel = std::max(worstVel, std::fabs(q));
                    exact = exact && std::fabs(q) < 1e-6;
                }
                else
                {
                    const double want = f[o] / scale[o] + mean[o];
                    worstPose = std::max(worstPose, std::fabs(q - want));
                    exact = exact && std::fabs(q - want) < 2e-3;
                }
            }
        }
        exactRows += exact;
        if (!exact)
            std::printf("  row %u frame %u: pose dims differ from the database row\n", r, row.frame);

        // B: first search after a reset with this history
        ChooserMatcher s;
        CHECK(s.Init(&db, &table, MatcherConfig{}, error), "init search");
        for (int i = 0; i < 5; ++i)
            s.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, nullptr);
        s.Reset(true);
        const MatchResult seeded = s.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, &hist);
        // the winner must be the row itself (or its loop duplicate / a neighbour frame of the same still pose, features within 1e-2), at ~zero cost, and not
        // the mean pick (this bake indexes the idle pose of a single clip, so the pick is told apart by its frame)
        const float* wf = seeded.database == idleDb && seeded.localRow >= 0 ? db.Features(static_cast<uint32_t>(idleDb), uint32_t(seeded.localRow)) : nullptr;
        double featDiff = 1e9;
        if (wf)
        {
            featDiff = 0.0;
            for (uint32_t k = 0; k < D.dims; ++k)
                featDiff = std::max(featDiff, double(std::fabs(wf[k] - f[k])));
        }
        // rows of moving clips (the exit-to-idle clip of the aimotion bakes) have their own trajectory features: the still query does not reproduce
        // them, so B looks at the still idle loop only (E covers the exit clip)
        const bool apart = db.Entry(row.entry).clip == uint32_t(base.clip) && std::llabs(int64_t(row.frame) - int64_t(base.frame)) > 3;
        if (apart)
        {
            ++differentFrame;
            const bool taken = featDiff < 1e-2 && seeded.cost < 0.05f && seeded.frame != base.frame;
            preferred += taken;
            if (!taken)
                std::printf("  row %u frame %u: seeded pick frame %u cost %.4f featDiff %.3e\n", r, row.frame, seeded.frame,
                            double(seeded.cost), featDiff);
        }
    }
    std::printf("rows tested=%u exact=%u worstPoseErr=%.2e worstVel=%.2e | rows apart from the mean pick=%u preferred=%u (mean pick frame %u)\n",
                tested, exactRows, worstPose, worstVel, differentFrame, preferred, base.frame);
    CHECK(tested >= 12, "A: rows tested %u", tested);
    CHECK(exactRows == tested, "A: exact pose dims for %u / %u rows (worst %.2e)", exactRows, tested, worstPose);
    CHECK(differentFrame >= 8, "B: rows apart from the mean pick %u", differentFrame);
    CHECK(preferred == differentFrame, "B: the first search takes the captured pose's row, not the mean pick (%u / %u)", preferred, differentFrame);

    // ---- E. the exit-to-idle clip of the aimotion bakes (VX_Exit_Idle_N10_MB starts on the exit end pose): when the bundle has it, the
    // pose of its first frame as the capture makes the first search pick it - with the mean query it never wins (the field defect).
    for (uint32_t c = 0; c < db.NumClips(); ++c)
    {
        if (db.ClipName(c) != "VX_Exit_Idle_N10_MB")
            continue;
        PoseHistory hist;
        CHECK(HistoryOfFrame(db, db.Clip(c).firstFrame, hist), "E: history of the exit clip start");
        ChooserMatcher s;
        CHECK(s.Init(&db, &table, MatcherConfig{}, error), "init exit clip");
        const MatchResult seeded = s.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, &hist);
        std::printf("VX_Exit_Idle_N10_MB: mean pick clip %d, seeded pick clip %d (%.*s) ct=%.3f cost=%.4f\n", base.clip, seeded.clip,
                    seeded.clip >= 0 ? int(db.ClipName(uint32_t(seeded.clip)).size()) : 0,
                    seeded.clip >= 0 ? db.ClipName(uint32_t(seeded.clip)).data() : "", double(seeded.clipTime), double(seeded.cost));
        CHECK(base.clip != int32_t(c), "E: the mean query never picks the exit clip");
        CHECK(seeded.clip == int32_t(c), "E: the captured exit pose picks the exit clip (got %d)", seeded.clip);
    }

    // ---- F. an authored exit-transition clip (MatcherConfig::exitTransitionPrefix "VX_", exitTransitionTailS) plays out once selected
    // The hold is a per-clip property of the bundle: it only exists when a clip carries the prefix. G proves that nothing else changes.
    uint32_t numVx = 0;
    for (uint32_t c = 0; c < db.NumClips(); ++c)
        numVx += db.ClipName(c).starts_with("VX_");
    std::printf("bundle VX_ clips: %u\n", numVx);
    std::vector<TrajSample> walkTraj;
    for (int k = -60; k <= 96; ++k)
    {
        TrajSample s;
        s.t = k / 60.f;
        s.y = 1.6f * s.t; // 1.6 m/s forward
        walkTraj.push_back(s);
    }
    struct Run { bool seeded = false; float leftAt = -1.0f; float endCt = 0.0f; };
    // aMode 0: still query throughout, 1: walk stick from step 60 (1 s), 2: walking from the first update after the seeded pick
    const auto play = [&](uint32_t aClip, const PoseHistory& aHist, const char* aPrefix, float aTail, int aMode, Run& aRun) {
        const float len = float(db.Clip(aClip).numFrames - 1) / db.Header().fps;
        MatcherConfig cfg;
        cfg.exitTransitionPrefix = aPrefix;
        cfg.exitTransitionTailS = aTail;
        ChooserMatcher m;
        if (!m.Init(&db, &table, cfg, error))
            return;
        aRun.seeded = m.Update(dt, trajectory.data(), uint32_t(trajectory.size()), state, &aHist).clip == int32_t(aClip);
        for (int i = 1; aRun.seeded && i < int((len + 1.0f) * 60.f); ++i)
        {
            const bool walk = aMode == 2 || (aMode == 1 && i >= 60);
            const MatchResult& r = m.Update(dt, walk ? walkTraj.data() : trajectory.data(), uint32_t(trajectory.size()),
                                            walk ? Ground(1, 0, 1.6f) : state, nullptr);
            if (r.clip != int32_t(aClip))
            {
                aRun.leftAt = float(i) / 60.f;
                break;
            }
            aRun.endCt = r.clipTime;
        }
    };
    // the four runs of one clip; aRealPrefix = the clip is a VX_ clip of the bundle (field defect checked against the clip end as well)
    const auto scenario = [&](uint32_t aClip, const char* aPrefix, bool aRealBundleClip) {
        const float len = float(db.Clip(aClip).numFrames - 1) / db.Header().fps;
        const float tail = MatcherConfig{}.exitTransitionTailS;
        PoseHistory hist;
        if (!HistoryOfFrame(db, db.Clip(aClip).firstFrame, hist))
            return false;
        Run off, held, moved, walkStart;
        play(aClip, hist, aPrefix, 0.0f, 0, off);
        play(aClip, hist, aPrefix, tail, 0, held);
        play(aClip, hist, aPrefix, tail, 1, moved);
        play(aClip, hist, aPrefix, tail, 2, walkStart);
        const bool usable = off.seeded && held.seeded && moved.seeded && walkStart.seeded && off.leftAt >= 0.0f && off.endCt < held.endCt - 0.3f;
        if (!aRealBundleClip && !usable)
            return false;
        std::printf("F [%.*s]: len %.3f s; hold off: left at +%.3f s (clip time %.3f); hold on, standing: left at +%.3f s (clip time %.3f); "
                    "stick at +1.0 s: left at +%.3f s (clip time %.3f); walking from the start: left at +%.3f s\n",
                    int(db.ClipName(aClip).size()), db.ClipName(aClip).data(), double(len), double(off.leftAt), double(off.endCt),
                    double(held.leftAt), double(held.endCt), double(moved.leftAt), double(moved.endCt), double(walkStart.leftAt));
        CHECK(off.seeded && held.seeded && moved.seeded && walkStart.seeded, "F: the captured start pose picks the clip");
        CHECK(off.leftAt >= 0.0f && off.endCt < held.endCt - 0.3f, "F: without the hold the clip is replaced early (clip time %.3f)", double(off.endCt));
        CHECK(held.leftAt >= 0.0f && held.endCt >= len - tail - 0.05f && held.endCt <= len,
              "F: standing query keeps the clip to its tail (left at clip time %.3f of %.3f)", double(held.endCt), double(len));
        CHECK(moved.leftAt >= 1.0f - 1e-4f && moved.leftAt <= 1.0f + 0.1f, "F: a walking query leaves the clip within 0.1 s of the stick (left at +%.3f s)",
              double(moved.leftAt));
        CHECK(walkStart.leftAt >= 0.0f && walkStart.leftAt < 0.2f, "F: a clip selected while moving is not held (left at +%.3f s)", double(walkStart.leftAt));
        return true;
    };
    for (uint32_t c = 0; c < db.NumClips(); ++c)
        if (db.ClipName(c) == "VX_Exit_Idle_N10_MB")
            scenario(c, "VX_", true);
    if (numVx == 0)
    {
        // synthetic: no VX_ clip in this bundle - name the full name of a non-loop clip of the idle database as the prefix and seed its first frame
        bool found = false;
        uint32_t candidates = 0;
        for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries && !found; ++e)
        {
            const uint32_t c = db.Entry(e).clip;
            if ((db.Clip(c).flags & utpp::mm::motion::kClipLoop) || db.Clip(c).numFrames < 90)
                continue;
            ++candidates;
            const std::string name(db.ClipName(c));
            found = scenario(c, name.c_str(), false);
        }
        std::printf("F synthetic (clip name as the prefix): %s (%u non-loop idle-database clips of >= 3 s)\n", found ? "ran" : "not feasible", candidates);
    }

    // ---- G. databases without VX_ clips: the hold exists but cannot arm, results equal a matcher with the hold off, step by step
    {
        uint32_t same = 0, total = 0;
        for (int variant = 0; variant < 2; ++variant)
        {
            MatcherConfig offCfg;
            offCfg.exitTransitionTailS = 0.0f;
            ChooserMatcher a, b;
            CHECK(a.Init(&db, &table, MatcherConfig{}, error) && b.Init(&db, &table, offCfg, error), "init G");
            for (int i = 0; i < 600; ++i)
            {
                const bool walk = variant == 1 && (i / 150) % 2 == 1;
                const TrajSample* tr = walk ? walkTraj.data() : trajectory.data();
                const ChooserState st = walk ? Ground(1, 0, 1.6f) : state;
                const MatchResult ra = a.Update(dt, tr, uint32_t(trajectory.size()), st, nullptr);
                const MatchResult rb = b.Update(dt, tr, uint32_t(trajectory.size()), st, nullptr);
                ++total;
                same += ra.clip == rb.clip && ra.database == rb.database && ra.localRow == rb.localRow && ra.clipTime == rb.clipTime &&
                        ra.cost == rb.cost && ra.jumped == rb.jumped && ra.playRate == rb.playRate;
            }
        }
        std::printf("G: hold on vs off, unseeded still / walk runs: %u of %u updates identical (bundle VX_ clips %u)\n", same, total, numVx);
        // a bundle with VX_ clips may legitimately differ once an unseeded run happens to select one; without them it must not
        CHECK(numVx > 0 || same == total, "G: no VX_ clips in the bundle -> hold on == hold off (%u of %u)", same, total);
    }

    // ---- D. fail closed
    {
        PoseHistory h;
        const PoseRecord* pose = db.Pose(db.Row(D.firstRow).frame);
        const bool none = BuildExitPoseHistory(
            db, 2, [&](uint32_t, math::Vec3&, math::Quat&) { return false; }, TrajSample{}, h);
        CHECK(!none && h.numSamples == 0 && h.numBones == 0, "D: no pose at all -> no history");
        // everything but the left leg chain (LeftUpLeg 5, LeftLeg 8, LeftFoot 11 of the shipped rig) -> LeftFoot dropped, the rest kept
        PoseHistory part;
        const bool some = BuildExitPoseHistory(
            db, 2,
            [&](uint32_t i, math::Vec3& p, math::Quat& q) {
                if (db.BoneName(i) == "LeftLeg")
                    return false;
                p = math::Vec3{pose[i].px, pose[i].py, pose[i].pz};
                q = math::Quat{pose[i].qx, pose[i].qy, pose[i].qz, pose[i].qw};
                return true;
            },
            TrajSample{}, part);
        bool hasLeft = false, hasRight = false, hasHips = false;
        for (uint32_t k = 0; k < part.numBones; ++k)
        {
            const std::string_view n = db.BoneName(static_cast<uint32_t>(part.bone[k]));
            hasLeft |= n == "LeftFoot";
            hasRight |= n == "RightFoot";
            hasHips |= n == "Hips";
        }
        CHECK(some && !hasLeft && hasRight && hasHips && part.numSamples == 1, "D: broken left chain drops LeftFoot only (some=%d)", some);
    }
    std::printf("VehExitSeed: PASS=%d FAIL=%d\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
