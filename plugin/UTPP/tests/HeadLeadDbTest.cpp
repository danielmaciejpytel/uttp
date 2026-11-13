// Head lead through the RUNTIME path on REAL database poses (mm/HeadLead.hpp + the v2 database + the real bone map + the real
// record blend of the game integration). Guards the in-game report "circling left -> V turns the head slightly to the right"
// (2026-10-19): the unit test on a synthetic rig could not see a wrong reference or a frame mismatch between the database pose
// (aPose of MotionDriver.cpp HeadLeadApplyLocked, 7 floats per database bone) and the game records.
//
//   utpp_headlead_db_test <utpp_motion_woman.utppmm>
//
// 1. The database pose is a game-local transform (PoseRecord): after integ::BuildBoneMap (names -> game rig, parent check) and
//    integ::BlendIntoRecords (weight 1) the 48-byte game records carry exactly the database quaternions; an independent FK on the
//    GAME records (parents of the game rig) gives the same head / Spine3 yaw as the FK on the database chain, and the idle clip
//    has yaw(Head) - yaw(Spine3) == restBias (the constant the correction was tuned with on the live capture).
// 2. Left / right circles of the live captures (actor yaw lags the velocity: travel +39 left / -31 right of the actor forward):
//    run clips FL / FR (and F), several frames each. The driver sequence is reproduced: ResolveDb, Step per feed row, Apply on a
//    fresh copy of the database pose per published frame, then the record blend. In the game-rig frame
//      - a left circle turns the head LEFT (yaw of the face +), a right circle turns it RIGHT (-), by more than 10 deg,
//      - the face ends clearly closer to / further into the run than before and than with the Spine3 law (pathBlend 0),
//      - the twist from Spine3 stays inside maxTwistDeg.
// The database is motion-derived (outside git); the test prints numbers only.
#define _CRT_SECURE_NO_WARNINGS 1
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/mm/GameIntegration.hpp"
#include "../src/mm/HeadLead.hpp"
#include "../src/mm/MotionDatabase.hpp"

using namespace utpp::mm;

namespace
{
int g_fail = 0;
void Check(bool aOk, const char* aWhat, double aA = 0.0, double aB = 0.0)
{
    if (!aOk)
    {
        ++g_fail;
        std::printf("FAIL: %s (%.3f, %.3f)\n", aWhat, aA, aB);
    }
}

constexpr float kDt = 1.0f / 60.0f;
constexpr float kDeg = 180.0f / 3.14159265358979323846f;

// independent quaternion code (xyzw) for the game-record FK
struct Q
{
    double x = 0, y = 0, z = 0, w = 1;
};
Q Mul(const Q& a, const Q& b)
{
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
// direction of the bone-local +Z axis rotated by q, projected on the model XY plane; angle from +X, CCW positive (deg)
double LocalZYaw(const Q& q)
{
    const double vx = 2.0 * (q.x * q.z + q.y * q.w), vy = 2.0 * (q.y * q.z - q.x * q.w);
    return std::atan2(vy, vx) * kDeg;
}
double Wrap(double a)
{
    a = std::fmod(a + 180.0, 360.0);
    if (a < 0.0)
        a += 360.0;
    return a - 180.0;
}

struct GameRig
{
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    std::vector<int32_t> dbParents;
    std::vector<uint64_t> dbHashes;
    integ::BoneMapResult map;
    std::vector<uint8_t> records; // 48 B per bone
    uint32_t n = 0;
};

// FK on the game records: model quaternion of every bone via the game parents
Q ModelQ(const GameRig& aR, uint32_t aBone)
{
    std::vector<uint32_t> chain;
    for (int32_t b = static_cast<int32_t>(aBone); b >= 0; b = aR.parents[static_cast<size_t>(b)])
        chain.push_back(static_cast<uint32_t>(b));
    Q run;
    bool first = true;
    for (size_t i = chain.size(); i-- > 0;)
    {
        const float* rec = reinterpret_cast<const float*>(aR.records.data() + size_t(chain[i]) * integ::kGameBoneStride);
        const float* q = rec + integ::kGameRotOffsetFloats;
        Q l{q[0], q[1], q[2], q[3]};
        const double n2 = l.x * l.x + l.y * l.y + l.z * l.z + l.w * l.w;
        const double k = 1.0 / std::sqrt(n2);
        l = {l.x * k, l.y * k, l.z * k, l.w * k};
        run = first ? l : Mul(run, l);
        first = false;
    }
    return run;
}

// the database pose of one frame in the layout of the snapshot (what StepLocked copies from b.pose)
void DbPose(const motion::MotionDatabase& aDb, uint32_t aFrame, float* aOut)
{
    const motion::PoseRecord* p = aDb.Pose(aFrame);
    for (uint32_t i = 0; i < aDb.NumBones(); ++i)
    {
        float* d = aOut + size_t(i) * 7;
        d[0] = p[i].px; d[1] = p[i].py; d[2] = p[i].pz;
        d[3] = p[i].qx; d[4] = p[i].qy; d[5] = p[i].qz; d[6] = p[i].qw;
    }
}

// publish: snapshot -> game records (the real BlendIntoRecords with weight 1)
void Publish(GameRig& aR, const float* aPose)
{
    integ::PoseSnapshot snap;
    snap.weight = 1.0f;
    snap.numBones = aR.n;
    snap.firstBone = 2;
    for (uint32_t i = 0; i < integ::kMaxMMBones; ++i)
        snap.gameIndex[i] = i < aR.n ? aR.map.gameIndex[i] : -1;
    std::memcpy(snap.pose, aPose, size_t(aR.n) * 7 * sizeof(float));
    for (uint32_t b = 0; b < aR.n; ++b) // game records: identity rotation (the blend aligns the sign to them)
    {
        float* rec = reinterpret_cast<float*>(aR.records.data() + size_t(b) * integ::kGameBoneStride);
        rec[4] = rec[5] = rec[6] = 0.0f;
        rec[7] = 1.0f;
    }
    integ::BlendIntoRecords(aR.records.data(), aR.n, snap, 1.0f);
}

struct Probe
{
    double face = 0.0, spine3 = 0.0; // yaw of the bone-local Z in the game rig frame (face == looking direction, 0 = forward + offset)
};
Probe ReadHead(const GameRig& aR, uint32_t aHead, uint32_t aSpine3)
{
    return {LocalZYaw(ModelQ(aR, aHead)), LocalZYaw(ModelQ(aR, aSpine3))};
}

struct Outcome
{
    double deltaHead = 0.0;   // game-rig face yaw: corrected - uncorrected
    double vsPathBefore = 0.0, vsPathAfter = 0.0;
    double twist = 0.0;       // Wrap(head - Spine3 - twistBias) after
    double corr = 0.0;
};

// aTravRel: travel direction relative to the actor forward (+ left). Left circle +180 deg/s, right -180 deg/s.
Outcome RunCircle(GameRig& aR, const motion::MotionDatabase& aDb, const float* aPose0, const headlead::Params& aP, float aRate, float aTravRel,
                  uint32_t aHead, uint32_t aSpine3)
{
    const headlead::Skeleton sk = headlead::ResolveDb(aDb, aDb.NumBones()); // as HeadLeadApplyLocked
    headlead::HeadLead hl;
    headlead::Signals s;
    s.yawValid = true;
    float heading = 0.0f;
    std::vector<float> pose(size_t(aDb.NumBones()) * 7);
    for (float t = 0; t < 3.0f; t += kDt)
    {
        heading += aRate * kDt;
        s.vx = 3.8f * std::cos(heading * 3.14159265f / 180.0f);
        s.vy = 3.8f * std::sin(heading * 3.14159265f / 180.0f);
        s.yawDeg = heading - 90.0f - aTravRel;
        hl.Step(aP, s, kDt);
        std::memcpy(pose.data(), aPose0, pose.size() * sizeof(float)); // the matcher output of this frame (fresh, uncorrected)
        hl.Apply(aP, sk, pose.data(), aDb.NumBones(), kDt);
    }
    Outcome o;
    Publish(aR, aPose0);
    const Probe before = ReadHead(aR, aHead, aSpine3);
    Publish(aR, pose.data());
    const Probe after = ReadHead(aR, aHead, aSpine3);
    o.deltaHead = Wrap(after.face - before.face);
    o.vsPathBefore = Wrap(before.face - aTravRel);
    o.vsPathAfter = Wrap(after.face - aTravRel);
    o.twist = Wrap(after.face - after.spine3 - aP.twistBiasDeg); // the quantity the twist window bounds
    o.corr = hl.Corr();
    return o;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_headlead_db_test <db.utppmm>\n");
        return 2;
    }
    motion::MotionDatabase db;
    std::string err;
    if (!db.LoadFromFile(argv[1], motion::kRigIdentityWoman, err))
    {
        std::printf("FAIL: database load: %s\n", err.c_str());
        return 1;
    }
    GameRig R;
    R.n = db.NumBones();
    for (uint32_t i = 0; i < R.n; ++i)
    {
        R.dbHashes.push_back(integ::CNameHash(db.BoneName(i)));
        R.dbParents.push_back(db.BoneParent(i));
        R.hashes.push_back(integ::CNameHash(db.BoneName(i))); // the game rig: same names, same order (live capture: gameIndex == db index)
        R.parents.push_back(static_cast<int16_t>(db.BoneParent(i)));
    }
    R.map = integ::BuildBoneMap(R.dbHashes, R.dbParents, R.hashes, R.parents, 2);
    R.records.assign(size_t(R.n) * integ::kGameBoneStride, 0);
    Check(R.map.ok && R.map.mapped == R.n - 2, "bone map ok (all bones above Trajectory mapped, parents consistent)", R.map.mapped, R.n);

    const headlead::Skeleton sk = headlead::ResolveDb(db, db.NumBones());
    Check(sk.ok, "runtime skeleton resolves on the database");
    const uint32_t iHead = static_cast<uint32_t>(sk.head), iS3 = static_cast<uint32_t>(sk.spine3);

    auto clipIndex = [&](const char* aName) -> int {
        for (uint32_t c = 0; c < db.NumClips(); ++c)
            if (db.ClipName(c) == aName)
                return static_cast<int>(c);
        return -1;
    };
    const headlead::Params P;
    std::vector<float> pose(size_t(db.NumBones()) * 7);

    // 1) frame: database chain == game records, idle bias
    {
        const int c = clipIndex("M_Neutral_Stand_Idle_Loop");
        Check(c >= 0, "idle clip present");
        if (c >= 0)
        {
            double sumBias = 0.0;
            int cnt = 0;
            double maxDiff = 0.0;
            const auto& cl = db.Clip(static_cast<uint32_t>(c));
            for (uint32_t k = 0; k < cl.numFrames; k += 20)
            {
                DbPose(db, cl.firstFrame + k, pose.data());
                Publish(R, pose.data());
                const Probe pr = ReadHead(R, iHead, iS3);
                // database-chain FK with the production code (Root first, as HeadLead::Apply does)
                headlead::Quat run, mS, mH;
                for (int i = 0; i < sk.len; ++i)
                {
                    const float* q = pose.data() + size_t(sk.chain[i]) * 7 + 3;
                    headlead::Quat l{q[0], q[1], q[2], q[3]};
                    headlead::Norm(l);
                    run = i == 0 ? l : headlead::Mul(run, l);
                    if (sk.chain[i] == sk.spine3) mS = run;
                    if (sk.chain[i] == sk.head) mH = run;
                }
                maxDiff = std::fmax(maxDiff, std::fmax(std::fabs(Wrap(pr.face - headlead::AxisYaw(mH))), std::fabs(Wrap(pr.spine3 - headlead::AxisYaw(mS)))));
                sumBias += Wrap(pr.face - pr.spine3);
                ++cnt;
            }
            Check(maxDiff < 0.05, "database chain FK == game record FK (head, Spine3 yaw)", maxDiff);
            Check(std::fabs(sumBias / cnt - P.restBiasDeg) < 2.0, "idle yaw(Head) - yaw(Spine3) == restBias in the game rig", sumBias / cnt, P.restBiasDeg);
            std::printf("frame check: db chain vs game records max %.4f deg, idle bias %.2f (restBias %.2f)\n", maxDiff, sumBias / cnt,
                        static_cast<double>(P.restBiasDeg));
        }
    }

    // 2) circles on real run poses
    struct Case
    {
        const char* clip;
        float rate, trav;
        int sign; // + left, - right
    };
    const Case cases[] = {{"M_Neutral_Run_Loop_FL", +180.0f, 39.0f, +1},  {"M_Neutral_Run_Loop_F", +180.0f, 39.0f, +1},
                          {"M_Neutral_Run_Loop_F_L_20", +180.0f, 39.0f, +1}, {"M_Neutral_Run_Loop_RL", +180.0f, 39.0f, +1},
                          {"M_Neutral_Run_Loop_FR", -180.0f, -31.0f, -1}, {"M_Neutral_Run_Loop_F", -180.0f, -31.0f, -1},
                          {"M_Neutral_Run_Loop_F_R_20", -180.0f, -31.0f, -1}};
    headlead::Params legacy = P;
    legacy.pathBlend = 0.0f;
    for (const Case& cs : cases)
    {
        const int c = clipIndex(cs.clip);
        Check(c >= 0, cs.clip);
        if (c < 0)
            continue;
        const auto& cl = db.Clip(static_cast<uint32_t>(c));
        double sumD = 0, sumLegD = 0, sumPathAfter = 0, sumPathLeg = 0, sumPathBefore = 0, maxTwist = 0;
        int cnt = 0;
        for (uint32_t k = 0; k < cl.numFrames; k += std::max<uint32_t>(1u, cl.numFrames / 6))
        {
            DbPose(db, cl.firstFrame + k, pose.data());
            const Outcome n = RunCircle(R, db, pose.data(), P, cs.rate, cs.trav, iHead, iS3);
            const Outcome l = RunCircle(R, db, pose.data(), legacy, cs.rate, cs.trav, iHead, iS3);
            Check(n.deltaHead * cs.sign > 3.0, "game-rig head turn has the sign of the turn (every frame)", n.deltaHead, cs.sign);
            sumD += n.deltaHead; sumLegD += l.deltaHead;
            sumPathAfter += n.vsPathAfter; sumPathLeg += l.vsPathAfter; sumPathBefore += n.vsPathBefore;
            maxTwist = std::fmax(maxTwist, std::fabs(n.twist));
            ++cnt;
        }
        const double d = sumD / cnt, dl = sumLegD / cnt, pa = sumPathAfter / cnt, pl = sumPathLeg / cnt, pb = sumPathBefore / cnt;
        std::printf("%-28s %s: head yaw change %+.1f (Spine3 law %+.1f) | face vs run before %+.1f, Spine3 law %+.1f, path law %+.1f | max twist %.1f\n",
                    cs.clip, cs.sign > 0 ? "left " : "right", d, dl, pb, pl, pa, maxTwist);
        Check(d * cs.sign > 10.0, "mean game-rig head turn > 10 deg in the direction of the turn", d, cs.sign);
        Check(maxTwist <= P.maxTwistDeg + 1.0, "twist from Spine3 stays inside maxTwistDeg", maxTwist, P.maxTwistDeg);
        // the face ends further into the run than before; on the lead (minus deadband) when the twist bound does not hold it back, and
        // never on the wrong side of the run by more than 10 deg when it does (the Spine3 law left it 18..37 deg on the wrong side
        // in the right circles)
        const double lead = std::fmin(static_cast<double>(P.maxLeadDeg), static_cast<double>(P.Gain()) * std::fabs(static_cast<double>(cs.rate)));
        Check((pa - pb) * cs.sign > 10.0, "face moves into the run", pa - pb, cs.sign);
        if (maxTwist < P.maxTwistDeg - 1.0)
            Check(pa * cs.sign > lead - P.deadbandDeg - 2.0, "face reaches the lead of the run (unbounded case)", pa, lead);
        // The twist window is now centred on head-vs-Spine3 == twistBias (1 deg), no longer on the idle pose (the old window reached
        // +73 deg vs Spine3 to the left). A clip that holds the head far to the turn side already (the backward run RL, not one of the
        // live circle loops) is therefore held back by the 50 deg bound: it must still improve by > 10 deg (above) and stay within 20 deg.
        const double wrongSide = maxTwist < P.maxTwistDeg - 1.0 ? 10.0 : 20.0;
        Check(pa * cs.sign > -wrongSide, "face never ends on the wrong side of the run by more than 10 deg (20 when the twist bound holds it)", pa, cs.sign);
        if (cs.sign < 0) // right circle: the Spine3 law leaves the head ~50 deg left of the run, the path law has to fix it
            Check(pl - pa > 10.0, "right circle: path law clearly better than the Spine3 law", pl, pa);
    }
    std::printf(g_fail ? "HeadLeadDbTest FAILED (%d)\n" : "HeadLeadDbTest OK (%d failures)\n", g_fail);
    return g_fail ? 1 : 0;
}
