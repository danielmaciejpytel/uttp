// Live pose history (mm/LivePoseHistory.hpp): the ring of published poses the next search reads as its pose history.
//   A. space: the newest sample equals the vehicle-exit builder's Root-space transforms of the same local pose (shared forward kinematics).
//   B. time base: one push builds nothing; two pushes give two samples at -(dt + dtNow) and -dtNow, oldest first.
//   C. window: at 60 Hz the ring keeps at most kHistMaxSamples samples, all inside kLiveHistoryWindow (+ dtNow).
//   D. root samples come from the query trajectory at the sample times (linear, yaw along the short arc across +-pi).
//   E. fail closed: a pose with another bone count resets the ring; Reset empties it.
//   F. query: a live history of a held frame reproduces that row's current Position features and zero foot velocity.
//   G. the visual root offset carried by the published pose is removed (history in the animation's Root space).
// Local motion data. Usage: test <database.utppmm> <chooser.json>
#define main FullMatcherSuiteMainNotRun
#include "ChooserMatcherTest.cpp"
#undef main

#include "../src/mm/LivePoseHistory.hpp"

namespace
{
struct Local
{
    math::Vec3 p;
    math::Quat q;
};

std::vector<Local> FramePose(const MotionDatabase& db, uint32_t aFrame)
{
    const PoseRecord* pose = db.Pose(aFrame);
    std::vector<Local> out(db.NumBones());
    for (uint32_t i = 0; i < db.NumBones(); ++i)
        out[i] = Local{math::Vec3{pose[i].px, pose[i].py, pose[i].pz}, math::Quat{pose[i].qx, pose[i].qy, pose[i].qz, pose[i].qw}};
    return out;
}

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

float Dist(const math::Vec3& a, const math::Vec3& b) { return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)); }
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
    const uint32_t frame = db.Clip(0).firstFrame + db.Clip(0).numFrames / 2;
    const auto pose = FramePose(db, frame);
    const auto traj = StillTrajectory();
    constexpr float kStep = 1.0f / 60.0f;

    // A
    {
        LivePoseHistory live;
        PoseHistory exitHist, liveHist;
        CHECK(live.Push(db, 2, pose.data(), db.NumBones(), kStep) && live.Push(db, 2, pose.data(), db.NumBones(), kStep), "A: push");
        CHECK(live.Build(kStep, traj.data(), uint32_t(traj.size()), liveHist), "A: build");
        const bool ok = BuildExitPoseHistory(
            db, 2,
            [&](uint32_t i, math::Vec3& p, math::Quat& q) {
                p = pose[i].p;
                q = pose[i].q;
                return true;
            },
            TrajSample{}, exitHist);
        CHECK(ok && exitHist.numBones == liveHist.numBones && liveHist.numBones > 0, "A: same tracked bones (%u / %u)", exitHist.numBones,
              liveHist.numBones);
        float worst = 0.0f;
        for (uint32_t k = 0; ok && k < liveHist.numBones && k < exitHist.numBones; ++k)
        {
            CHECK(liveHist.bone[k] == exitHist.bone[k], "A: bone order %u", k);
            worst = std::max(worst, Dist(liveHist.gp[1][k], exitHist.gp[0][k]));
            const math::Quat a = liveHist.gq[1][k], b = exitHist.gq[0][k];
            worst = std::max(worst, 1.0f - std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w));
        }
        CHECK(worst < 1e-6f, "A: Root-space transforms identical (worst %.2e)", double(worst));
    }
    // B
    {
        LivePoseHistory live;
        PoseHistory h;
        CHECK(live.Push(db, 2, pose.data(), db.NumBones(), 0.0f), "B: first push");
        CHECK(!live.Build(kStep, traj.data(), uint32_t(traj.size()), h) && h.numSamples == 0, "B: one sample builds nothing");
        CHECK(live.Push(db, 2, pose.data(), db.NumBones(), 0.02f), "B: second push");
        CHECK(live.Build(0.015f, traj.data(), uint32_t(traj.size()), h) && h.numSamples == 2, "B: two samples");
        CHECK(std::fabs(h.root[0].t + 0.035f) < 1e-6f && std::fabs(h.root[1].t + 0.015f) < 1e-6f, "B: times %.4f %.4f", double(h.root[0].t),
              double(h.root[1].t));
    }
    // C
    {
        LivePoseHistory live;
        PoseHistory h;
        for (int k = 0; k < 90; ++k)
            live.Push(db, 2, pose.data(), db.NumBones(), kStep);
        CHECK(live.Build(kStep, traj.data(), uint32_t(traj.size()), h), "C: build");
        CHECK(h.numSamples <= kHistMaxSamples && h.numSamples >= 20, "C: samples %u", h.numSamples);
        bool sorted = true;
        for (uint32_t i = 1; i < h.numSamples; ++i)
            sorted &= h.root[i].t > h.root[i - 1].t;
        CHECK(sorted && h.root[0].t >= -(kLiveHistoryWindow + kStep) - 1e-5f, "C: sorted, oldest %.3f s", double(h.root[0].t));
    }
    // D
    {
        TrajSample a, b;
        a.t = -0.2f, a.x = 1.0f, a.yaw = 3.0f;
        b.t = 0.0f, b.x = 3.0f, b.yaw = -3.0f;
        const TrajSample two[2] = {a, b};
        const TrajSample m = LivePoseHistory::TrajAt(two, 2, -0.1f);
        const float wrapped = std::remainder(m.yaw - (3.0f + (6.2831853f - 6.0f) * 0.5f), 6.2831853f);
        CHECK(std::fabs(m.x - 2.0f) < 1e-5f && std::fabs(wrapped) < 1e-4f && m.t == -0.1f, "D: x %.3f yaw %.4f", double(m.x), double(m.yaw));
        CHECK(LivePoseHistory::TrajAt(two, 2, -1.0f).x == 1.0f && LivePoseHistory::TrajAt(two, 2, 1.0f).x == 3.0f, "D: clamped ends");
    }
    // E
    {
        LivePoseHistory live;
        PoseHistory h;
        live.Push(db, 2, pose.data(), db.NumBones(), kStep);
        live.Push(db, 2, pose.data(), db.NumBones(), kStep);
        CHECK(!live.Push(db, 2, pose.data(), db.NumBones() - 1, kStep) && live.NumSamples() == 0, "E: wrong bone count resets");
        live.Push(db, 2, pose.data(), db.NumBones(), kStep);
        live.Push(db, 2, pose.data(), db.NumBones(), kStep);
        live.Reset();
        CHECK(live.NumSamples() == 0 && !live.Build(kStep, traj.data(), uint32_t(traj.size()), h), "E: reset empties");
    }
    // F: a query built from the live history of a held frame reproduces the frame's own Position features (both samples are that pose,
    // the trajectory is still) and gives zero foot velocity
    {
        uint32_t tested = 0, exact = 0;
        for (uint32_t d = 0; d < db.NumDatabases() && tested < 6; ++d)
        {
            const DatabaseRecord& D = db.Db(d);
            if (D.numRows < 40 || std::string_view(db.DbName(d)).find("Idle") == std::string_view::npos)
                continue;
            const SchemaRecord& S = db.Schema(D.schema);
            const uint32_t r = D.numRows / 2;
            const RowRecord& row = db.Row(D.firstRow + r);
            const auto p = FramePose(db, row.frame);
            LivePoseHistory live;
            PoseHistory h;
            live.Push(db, 2, p.data(), db.NumBones(), kStep);
            live.Push(db, 2, p.data(), db.NumBones(), kStep);
            if (!live.Build(kStep, traj.data(), uint32_t(traj.size()), h))
                continue;
            ChooserMatcher m;
            if (!m.Init(&db, &table, MatcherConfig{}, error))
                continue;
            m.SetQueryInputs(traj.data(), uint32_t(traj.size()), &h);
            m.BuildQuery(d, false);
            const float* mean = db.NormMean(D.norm);
            const float* scale = db.NormScale(D.norm);
            const float* f = db.Features(d, r);
            bool ok = true, any = false;
            for (uint32_t c = S.firstChannel; c < S.firstChannel + S.numChannels; ++c)
            {
                const ChannelRecord& ch = db.Channel(c);
                if (ch.flags & kChanTrajectory)
                    continue;
                for (uint32_t k = 0; k < ch.cardinality; ++k)
                {
                    const uint32_t o = ch.dataOffset + k;
                    const double q = m.RawQuery()[o];
                    if (ch.kind == 2 || ch.kind == 3)
                        ok = ok && std::fabs(q) < 1e-6;
                    else if (ch.kind == 1 && ch.tSample == 0.0f && ch.tOrigin == 0.0f)
                    {
                        any = true;
                        ok = ok && std::fabs(q - (f[o] / scale[o] + mean[o])) < 2e-3;
                    }
                }
            }
            tested += any;
            exact += any && ok;
            if (any && !ok)
                std::printf("  database %u row %u: query differs from the row\n", d, r);
        }
        CHECK(tested >= 1 && exact == tested, "F: held-frame queries reproduce the row in %u / %u idle databases", exact, tested);
    }
    // G: the visual root offset on the root's children (as the offset root applies it) is removed again
    {
        const float ox = 0.12f, oy = -0.07f, oyaw = 0.6f;
        const math::Quat qz{0.0f, 0.0f, std::sin(0.5f * oyaw), std::cos(0.5f * oyaw)};
        auto moved = pose;
        for (uint32_t b = 0; b < db.NumBones(); ++b)
            if (db.BoneParent(b) == 0)
            {
                const math::Vec3 r = math::QuatRotate(qz, moved[b].p);
                moved[b].p = math::Vec3{r.x + ox, r.y + oy, r.z};
                moved[b].q = math::QuatMul(qz, moved[b].q);
            }
        LivePoseHistory plain, offset;
        PoseHistory a, b;
        plain.Push(db, 2, pose.data(), db.NumBones(), kStep);
        plain.Push(db, 2, pose.data(), db.NumBones(), kStep);
        offset.Push(db, 2, moved.data(), db.NumBones(), kStep, ox, oy, oyaw);
        offset.Push(db, 2, moved.data(), db.NumBones(), kStep, ox, oy, oyaw);
        CHECK(plain.Build(kStep, traj.data(), uint32_t(traj.size()), a) && offset.Build(kStep, traj.data(), uint32_t(traj.size()), b),
              "G: build");
        float worst = 0.0f;
        for (uint32_t k = 0; k < a.numBones; ++k)
        {
            worst = std::max(worst, Dist(a.gp[1][k], b.gp[1][k]));
            const math::Quat x = a.gq[1][k], y = b.gq[1][k];
            worst = std::max(worst, 1.0f - std::fabs(x.x * y.x + x.y * y.y + x.z * y.z + x.w * y.w));
        }
        CHECK(a.numBones > 0 && worst < 1e-5f, "G: offset removed (worst %.2e)", double(worst));
    }
    std::printf("LivePoseHistory: PASS=%d FAIL=%d\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
