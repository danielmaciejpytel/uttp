// Ground dodge clips through the motion driver (src/MotionDriver.cpp + src/mm/Dodge.hpp) on recorded dodges.
// Usage: utpp_motion_driver_dodge_test <data dir> <fixture dir>
//   <data dir>: the motion data (utpp_motion_<body>.utppmm + .chooser.json, motion\, traversal\), as utpp_motion_driver_unit_test.
//   <fixture dir>: utpp_dodge_<body>.utpptrav and the recorded feed windows dodge_feed_woman_a.txt, dodge_feed_woman_b.txt,
//   dodge_feed_man.txt (31-column feed dump rows around four ground dodges each). A missing file makes the test FAIL.
// Checks per recording: the pose of a dodge frame is the clip of the sector the driver picked (legs within 3 deg of the clip sample
// once the entry blend is over) and that sector is the impulse direction relative to the drawn heading; single-frame steps at the
// entry and the hand-back stay within 3 cm (hips) / 6 deg (feet, hands) of the clip's own largest step; the same at 30 / 60 / 120 Hz
// (resampled feeds); without the file (missing or invalid) the output is byte-identical to a replay with a missing file, and frames
// before the first dodge are byte-identical with and without it; shown clip changes per second over each dodge stay <= 3.0.
// Prints (INFO): visual - capsule offset with / without the clips, planted-foot slide, hand-back time vs the capsule stop.

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "../src/MotionDriver.hpp"
#include "../src/mm/Dodge.hpp"
#include "../src/mm/MotionDatabase.hpp"
#include "../src/mm/TraversalFile.hpp"
#include "TestFailFast.hpp"

using namespace utpp;
namespace dodge = utpp::mm::dodge;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat.c_str(), aDetail.empty() ? "" : " | ", aDetail.c_str());
    if (!aOk)
        UtppFailFastNote("FAIL " + aWhat + (aDetail.empty() ? "" : " | ") + aDetail);
}
std::string F(const char* aFmt, double a, double b = 0, double c = 0, double d = 0)
{
    char buf[256];
    std::snprintf(buf, sizeof buf, aFmt, a, b, c, d);
    return buf;
}

struct Db
{
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    std::vector<std::string> names;
    uint32_t bones = 0;
};
bool LoadDb(const std::string& aPath, Db& aDb)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (bytes.size() < 32)
        return false;
    uint64_t rig = 0;
    std::memcpy(&rig, bytes.data() + 24, sizeof(rig));
    mm::motion::MotionDatabase db;
    std::string err;
    if (!db.LoadFromMemory(bytes, rig, err))
        return false;
    aDb.bones = db.NumBones();
    for (uint32_t i = 0; i < db.NumBones(); ++i)
    {
        aDb.hashes.push_back(mm::integ::CNameHash(db.BoneName(i)));
        aDb.parents.push_back(static_cast<int16_t>(db.BoneParent(i)));
        aDb.names.emplace_back(db.BoneName(i));
    }
    return true;
}
int BoneIndex(const Db& aDb, const char* aName)
{
    for (uint32_t i = 0; i < aDb.bones; ++i)
        if (aDb.names[i] == aName)
            return static_cast<int>(i);
    return -1;
}

motiondriver::Paths DataPaths(const std::string& aData, const std::string& aBody)
{
    motiondriver::Paths p;
    p.files.db = aData + "\\utpp_motion_" + aBody + ".utppmm";
    p.files.chooser = aData + "\\utpp_motion_" + aBody + ".chooser.json";
    p.files.overlay = aData + "\\motion\\utpp_overlays_" + aBody + ".utppov";
    p.files.style = aData + "\\motion\\utpp_style_weights_" + aBody + ".txt";
    p.traversal = aData + "\\traversal\\utpp_traversal_" + aBody + ".utpptrav";
    return p;
}

struct Row
{
    float t = 0, dt = 0, v[3] = {}, mv[2] = {}, yaw = 0, cam = 0, tgtYaw = 0, cap[3] = {};
    int32_t loco = 0, detail = -1;
    uint32_t psm = 0;
    bool haveTgt = false;
};
std::vector<Row> ReadRows(const std::string& aPath)
{
    std::vector<Row> rows;
    std::ifstream f(aPath);
    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::vector<std::string> c;
        size_t p = 0;
        while (p < line.size())
        {
            const size_t q = line.find(' ', p);
            c.push_back(line.substr(p, q == std::string::npos ? std::string::npos : q - p));
            if (q == std::string::npos)
                break;
            p = q + 1;
        }
        if (c.size() < 31)
            continue;
        Row r;
        r.t = std::strtof(c[0].c_str(), nullptr);
        r.dt = std::strtof(c[1].c_str(), nullptr);
        for (int k = 0; k < 3; ++k)
            r.v[k] = std::strtof(c[2 + k].c_str(), nullptr);
        r.mv[0] = std::strtof(c[5].c_str(), nullptr);
        r.mv[1] = std::strtof(c[6].c_str(), nullptr);
        r.yaw = std::strtof(c[7].c_str(), nullptr);
        r.cam = std::strtof(c[8].c_str(), nullptr);
        r.loco = static_cast<int32_t>(std::strtol(c[9].c_str(), nullptr, 10));
        r.psm = static_cast<uint32_t>(std::strtoul(c[10].c_str(), nullptr, 10));
        r.haveTgt = c[17] == "1";
        r.tgtYaw = std::strtof(c[18].c_str(), nullptr);
        for (int k = 0; k < 3; ++k)
            r.cap[k] = std::strtof(c[19 + k].c_str(), nullptr);
        r.detail = static_cast<int32_t>(std::strtol(c[26].c_str(), nullptr, 10));
        rows.push_back(r);
    }
    return rows;
}
float WrapDeg(float a) { return dodge::WrapDeg(a); }

// Fixed-rate copy of the recording: continuous channels interpolated, discrete ones (PSM, locomotion, target flag) from the row the
// sample time falls in.
std::vector<Row> Resample(const std::vector<Row>& aRows, float aHz)
{
    std::vector<Row> out;
    if (aRows.size() < 2)
        return out;
    const float dt = 1.0f / aHz;
    size_t j = 0;
    for (float t = aRows.front().t; t <= aRows.back().t; t += dt)
    {
        while (j + 1 < aRows.size() && aRows[j + 1].t <= t)
            ++j;
        const Row& a = aRows[j];
        const Row& b = aRows[std::min(j + 1, aRows.size() - 1)];
        const float u = b.t > a.t ? std::clamp((t - a.t) / (b.t - a.t), 0.0f, 1.0f) : 0.0f;
        Row r = a;
        r.t = t;
        r.dt = dt;
        for (int k = 0; k < 3; ++k)
        {
            r.v[k] = a.v[k] + (b.v[k] - a.v[k]) * u;
            r.cap[k] = a.cap[k] + (b.cap[k] - a.cap[k]) * u;
        }
        r.yaw = a.yaw + WrapDeg(b.yaw - a.yaw) * u;
        r.cam = a.cam + WrapDeg(b.cam - a.cam) * u;
        out.push_back(r);
    }
    return out;
}

struct Frame
{
    float t = 0, dt = 0, speed = 0, velYawDeg = 0, yawDeg = 0, rootYawDeg = 0, cap[3] = {};
    bool applied = false, dodgeFeed = false;
    motiondriver::DodgeProbe probe;
    float offset[3] = {}; // visual - capsule (x, y m, yaw rad), capsule axes
    int32_t shown = -1;   // db clip, or -(sector + 2) on a dodge clip frame
    int32_t under = -1;   // db clip of the motion search (also under a dodge clip)
    std::vector<float> pose;
};

mm::integ::PoseSnapshot BaseLikeSnapshot(uint64_t aRig, uint32_t aBones)
{
    mm::integ::PoseSnapshot s{};
    s.weight = 0.75f;
    s.numBones = aBones;
    s.firstBone = motiondriver::kFirstBone;
    s.rig = aRig;
    for (uint32_t i = 0; i < mm::integ::kMaxMMBones; ++i)
        s.gameIndex[i] = (i >= motiondriver::kFirstBone && i < aBones) ? static_cast<int32_t>(i) : -1;
    for (uint32_t i = 0; i < aBones; ++i)
        s.pose[size_t(i) * 7 + 6] = 1.0f;
    return s;
}

int PickClip(const std::string& aStatus)
{
    const size_t p = aStatus.find("pick=clip:");
    return p == std::string::npos ? -1 : std::atoi(aStatus.c_str() + p + 10);
}

constexpr uint64_t kRigPtr = 0x1234567800ull;

motionhost::FeedInput StandFeed()
{
    motionhost::FeedInput f;
    f.locoState = mm::integ::kLocoStand;
    f.tppActive = true;
    f.dt = 1.0f / 60.0f;
    f.haveRootContext = f.nativeRootEnabled = true;
    return f;
}

// Fresh driver session on aPaths (body aBody) with the rig armed; true when the bundle is loaded and the motion layer runs.
bool Session(int32_t aBody, const motiondriver::Paths& aPaths, const Db& aDb)
{
    motiondriver::SetEnabled(false);
    motiondriver::OnDisarmed();
    motiondriver::SetPathsOverride(aBody, aPaths);
    motiondriver::SetRigBody(aBody);
    motiondriver::SetEnabled(true);
    motionhost::FeedInput f = StandFeed();
    for (int i = 0; i < 4; ++i)
    {
        (void)motiondriver::Step(f, 0, f.dt);
        motiondriver::WaitLoader();
    }
    motiondriver::OnArmed(kRigPtr, aDb.hashes, aDb.parents);
    int32_t lb = -1;
    return motiondriver::LoadedBody(lb) && lb == aBody && motiondriver::Reason() == mm::adapter::Fallback::None;
}

std::vector<Frame> Replay(const std::vector<Row>& aRows, const Db& aDb)
{
    std::vector<Frame> out;
    out.reserve(aRows.size());
    uint64_t seq = 0;
    const Row* prev = nullptr;
    for (const Row& r : aRows)
    {
        motionhost::FeedInput f = StandFeed();
        std::memcpy(f.velWorld, r.v, sizeof f.velWorld);
        std::memcpy(f.move, r.mv, sizeof f.move);
        f.cameraYawDeg = r.cam;
        f.playerYawDeg = r.yaw;
        f.locoState = r.loco;
        f.dt = r.dt > 0.0f && r.dt < 0.1f ? r.dt : 1.0f / 60.0f;
        f.haveTarget = r.haveTgt;
        f.targetYawDeg = r.tgtYaw;
        f.psmBlock = r.psm;
        f.haveExtra = true;
        std::memcpy(f.capsulePos, r.cap, sizeof f.capsulePos);
        f.nativeBodyTurnEnabled = true;
        f.rootFeedSequence = ++seq;
        if (prev)
        {
            const float yr = r.yaw * mm::integ::kDegToRad;
            mm::integ::WorldToCharacter(r.cap[0] - prev->cap[0], r.cap[1] - prev->cap[1], yr, f.capsuleDeltaCurrent[0], f.capsuleDeltaCurrent[1]);
            f.capsuleYawDelta = WrapDeg(r.yaw - prev->yaw) * mm::integ::kDegToRad;
        }
        prev = &r;
        const float rootYawBefore = motiondriver::TestStrafeProbe().rootYawDeg;
        (void)motiondriver::Step(f, 0, f.dt);
        Frame fr;
        fr.t = r.t;
        fr.dt = f.dt;
        fr.speed = std::hypot(r.v[0], r.v[1]);
        fr.velYawDeg = std::atan2(-r.v[0], r.v[1]) * 57.29578f;
        fr.yawDeg = r.yaw;
        fr.rootYawDeg = rootYawBefore;
        fr.cap[0] = r.cap[0];
        fr.cap[1] = r.cap[1];
        fr.cap[2] = r.cap[2];
        fr.dodgeFeed = r.detail == 7;
        fr.probe = motiondriver::TestDodgeProbe();
        const motiondriver::RootProbe rp = motiondriver::TestRootProbe();
        std::memcpy(fr.offset, rp.after, sizeof fr.offset);
        mm::integ::PoseSnapshot s = BaseLikeSnapshot(kRigPtr, aDb.bones);
        fr.applied = motiondriver::ApplyToSnapshot(s);
        fr.pose.assign(s.pose, s.pose + size_t(aDb.bones) * 7);
        fr.under = PickClip(motiondriver::Status());
        fr.shown = fr.probe.active ? -(fr.probe.sector + 2) : fr.under;
        out.push_back(std::move(fr));
    }
    return out;
}

// model-space rotation / position of a local pose (database order)
void Model(const float* aPose, const Db& aDb, std::vector<mm::math::Quat>& aQ, std::vector<mm::math::Vec3>& aP)
{
    aQ.assign(aDb.bones, mm::math::Quat{});
    aP.assign(aDb.bones, mm::math::Vec3{});
    for (uint32_t b = motiondriver::kFirstBone; b < aDb.bones; ++b)
    {
        const int p = aDb.parents[b];
        const float* d = aPose + size_t(b) * 7;
        const mm::math::Quat q{d[3], d[4], d[5], d[6]};
        const mm::math::Vec3 t{d[0], d[1], d[2]};
        if (p < 0)
        {
            aQ[b] = q;
            aP[b] = t;
        }
        else
        {
            aQ[b] = mm::math::QuatNormalize(mm::math::QuatMul(aQ[p], q));
            aP[b] = aP[p] + mm::math::QuatRotate(aQ[p], t);
        }
    }
}
float AngleDeg(const mm::math::Quat& a, const mm::math::Quat& b)
{
    const float d = std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * std::acos(std::fmin(1.0f, d)) * 57.29578f;
}
float Dist(const mm::math::Vec3& a, const mm::math::Vec3& b) { return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)); }

struct Bones
{
    int hips = -1, legs[4] = {-1, -1, -1, -1}, ends[4] = {-1, -1, -1, -1}; // ends: LeftFoot RightFoot LeftHand RightHand
    std::vector<int> upper;                                                   // LeftArm RightArm LeftForeArm RightForeArm Head
};

// largest single-frame step of the clip itself at feed step aDt (clip time advances aDt * kRate): hips (m), ends (deg)
void ClipOwnStep(const dodge::DodgeClips& aC, dodge::Sector aS, float aDt, const Db& aDb, const Bones& aB, float& aHips, float& aEnds)
{
    aHips = aEnds = 0.0f;
    std::vector<float> a(size_t(aDb.bones) * 7), b(a.size());
    std::vector<mm::math::Quat> qa, qb;
    std::vector<mm::math::Vec3> pa, pb;
    const float len = aC.length[static_cast<int>(aS)];
    for (float t = 0.0f; t + aDt * dodge::kRate <= len + 1e-4f; t += aDt * dodge::kRate)
    {
        dodge::SampleDodge(aC, aS, t, a.data());
        dodge::SampleDodge(aC, aS, t + aDt * dodge::kRate, b.data());
        Model(a.data(), aDb, qa, pa);
        Model(b.data(), aDb, qb, pb);
        aHips = std::fmax(aHips, Dist(pa[aB.hips], pb[aB.hips]));
        for (int e : aB.ends)
            aEnds = std::fmax(aEnds, AngleDeg(qa[e], qb[e]));
    }
}

struct Episode
{
    size_t start = 0, end = 0; // first clip frame, hand-back frame
    int32_t sector = -1, firstSector = -1; // shown at the end of the settle window / picked on the impulse feed
    float expectRel = 0, actorRel = 0;
    int32_t expected = -1;
};

struct Metrics
{
    int clips = 0, sectorOk = 0, dodges = 0, outcomeOk = 0;
    float offsetEnd = 0, offsetEndBase = 0; // visual - capsule at the end of the hand-back blend (largest over the dodges)
    float legMaxDeg = 0, hipsExcess = -1e9f, endsExcess = -1e9f, hipsEntry = 0, hipsExit = 0, endsEntry = 0, endsExit = 0;
    float hipsHeightEntry = 0, hipsHeightExit = 0; // largest single-frame hips height step (m)
    float hipsHeightExcess = -1e9f;                // largest height step above the shown clip's / motion pose's own (m)
    // hands (model, deg): largest single-frame step over the release + hand-back / the entry window, the same without the clips, excess
    float releaseHands = 0, releaseHandsBase = 0, releaseHandsExcess = -1e9f, entryHands = 0, entryHandsBase = 0, entryHandsExcess = -1e9f;
    uint32_t sectorBits = 0;                       // bit s: a clip of sector s was shown
    float upperMaxDeg = 0;                         // arms / head vs the frame before the clip, clip frames before the release
    int upperFrames = 0;                           // clip frames measured for upperMaxDeg
    float offsetMax = 0, offsetMaxBase = 0, slideCm = 0, changesMax = 0, changesMaxBase = 0, stopLagMax = 0;
    size_t firstDodge = 0;
};

Metrics Analyse(const std::string& aTag, const std::vector<Frame>& aWith, const std::vector<Frame>& aBase, const dodge::DodgeClips& aC,
                const Db& aDb, const Bones& aB, bool aPrint)
{
    Metrics m;
    m.firstDodge = aWith.size();
    for (size_t i = 0; i < aWith.size(); ++i)
        if (aWith[i].dodgeFeed)
        {
            m.firstDodge = i;
            break;
        }
    std::vector<Episode> eps;
    for (size_t i = 0; i < aWith.size(); ++i)
    {
        const Frame& f = aWith[i];
        if (f.probe.active && f.probe.start && (eps.empty() || eps.back().end != 0 || eps.back().sector != f.probe.sector))
        {
            if (!eps.empty() && eps.back().end == 0) // sector change inside the settle window: same episode, new sector
            {
                eps.back().sector = f.probe.sector;
                continue;
            }
            Episode e;
            e.start = i;
            e.sector = e.firstSector = f.probe.sector;
            eps.push_back(e);
        }
        if (!eps.empty() && eps.back().end == 0 && f.probe.handback)
            eps.back().end = i;
    }
    m.clips = static_cast<int>(eps.size());
    for (const Episode& e : eps)
        m.sectorBits |= e.sector >= 0 ? (1u << e.sector) : 0u;
    // every recorded dodge (rising PSM dodge detail): a clip of the impulse sector, or none when it is a forward dodge entered above the
    // walk / run midpoint of the game's standing speeds (the run goes on). Impulse feed = first speed rise of 1.5 m/s or 0.06 s in.
    for (size_t i = 1; i < aWith.size(); ++i)
    {
        if (!aWith[i].dodgeFeed || aWith[i - 1].dodgeFeed)
            continue;
        ++m.dodges;
        float entry = aWith[i - 1].speed;
        size_t j = i;
        if (aWith[i].speed > entry + dodge::kImpulseRiseMps)
            entry = aWith[i - 1].speed;
        else
        {
            entry = aWith[i].speed;
            j = i + 1;
            while (j + 1 < aWith.size() && !(aWith[j].speed > entry + dodge::kImpulseRiseMps) && aWith[j].t - aWith[i].t < dodge::kImpulseWaitS &&
                   aWith[j].dodgeFeed)
                ++j;
        }
        if (j >= aWith.size())
            continue;
        const dodge::Sector s = dodge::ClassifySector(WrapDeg(aWith[j].velYawDeg - aWith[j].rootYawDeg));
        const bool runOn = s == dodge::Sector::Forward && entry >= 0.5f * (1.6f + 3.85f);
        bool shown = false;
        int32_t sec = -1;
        for (size_t k = i; k < aWith.size() && aWith[k].t - aWith[i].t < 0.15f; ++k)
            if (aWith[k].probe.active)
            {
                shown = true;
                sec = aWith[k].probe.sector;
                break;
            }
        const bool ok = runOn ? !shown : (shown && sec == static_cast<int32_t>(s));
        m.outcomeOk += ok ? 1 : 0;
        if (aPrint)
            std::printf("INFO %s dodge at t=%.3f: entry %.2f m/s, impulse sector %s -> %s (%s)\n", aTag.c_str(), aWith[i].t, entry,
                        dodge::SectorName(s), shown ? dodge::SectorName(static_cast<dodge::Sector>(sec)) : "no clip (run goes on)",
                        ok ? "as expected" : "UNEXPECTED");
    }
    std::vector<float> clipPose(size_t(aDb.bones) * 7);
    std::vector<mm::math::Quat> qs, qc, q0, q1;
    std::vector<mm::math::Vec3> ps, pc, p0, p1;
    for (Episode& e : eps)
    {
        if (e.end == 0)
            e.end = aWith.size() - 1;
        // pose = the picked clip (legs, local) after the entry blend
        for (size_t i = e.start; i < e.end; ++i)
        {
            const Frame& f = aWith[i];
            if (!f.probe.active || f.probe.clipTime < dodge::kBlendInS * dodge::kRate + 0.05f)
                continue;
            dodge::SampleDodge(aC, static_cast<dodge::Sector>(f.probe.sector), f.probe.clipTime, clipPose.data());
            for (int l : aB.legs)
            {
                const float* a = &f.pose[size_t(l) * 7 + 3];
                const float* c = &clipPose[size_t(l) * 7 + 3];
                m.legMaxDeg = std::fmax(m.legMaxDeg, AngleDeg({a[0], a[1], a[2], a[3]}, {c[0], c[1], c[2], c[3]}));
            }
        }
        // the upper body keeps the pose shown before the clip: arms and head (local) on the clip frames before the release
        if (e.start > 0 && !aB.upper.empty())
            for (size_t i = e.start; i < e.end; ++i)
            {
                const Frame& f = aWith[i];
                const float len = aC.length[static_cast<int>(f.probe.sector < 0 ? 0 : f.probe.sector)];
                if (!f.probe.active || f.probe.clipTime > len - dodge::kUpperReleaseS * dodge::kRate)
                    continue;
                ++m.upperFrames;
                for (int u : aB.upper)
                {
                    const float* a = &f.pose[size_t(u) * 7 + 3];
                    const float* c = &aWith[e.start - 1].pose[size_t(u) * 7 + 3];
                    m.upperMaxDeg = std::fmax(m.upperMaxDeg, AngleDeg({a[0], a[1], a[2], a[3]}, {c[0], c[1], c[2], c[3]}));
                }
            }
        float ownH = 0, ownE = 0;
        ClipOwnStep(aC, static_cast<dodge::Sector>(e.sector), aWith[e.start].dt, aDb, aB, ownH, ownE);
        // entry: the frame before the clip -> 6 clip frames; exit: the hand-back frame and 0.25 s after it
        auto steps = [&](size_t a0, size_t a1, float& h, float& en, float& hz) {
            h = en = hz = 0;
            for (size_t i = std::max<size_t>(a0, 1); i <= a1 && i < aWith.size(); ++i)
            {
                Model(aWith[i - 1].pose.data(), aDb, q0, p0);
                Model(aWith[i].pose.data(), aDb, q1, p1);
                h = std::fmax(h, Dist(p0[aB.hips], p1[aB.hips]));
                hz = std::fmax(hz, std::fabs(p1[aB.hips].z - p0[aB.hips].z));
                float e1 = 0.0f, ek[4] = {};
                for (int k = 0; k < 4; ++k)
                {
                    ek[k] = AngleDeg(q0[aB.ends[k]], q1[aB.ends[k]]);
                    e1 = std::fmax(e1, ek[k]);
                }
                en = std::fmax(en, e1);
                if (aPrint && std::getenv("UTPP_DODGE_TRACE"))
                {
                    float eb = -1.0f; // the same step without the clips
                    if (i < aBase.size())
                    {
                        Model(aBase[i - 1].pose.data(), aDb, qs, ps);
                        Model(aBase[i].pose.data(), aDb, qc, pc);
                        for (int x : aB.ends)
                            eb = std::fmax(eb, AngleDeg(qs[x], qc[x]));
                    }
                    std::printf("STEP %s t=%.3f dt %.4f active %d start %d handback %d clipT %.3f shown %d under %d hips (%.3f %.3f %.3f) step %.1f cm ends %.1f deg "
                                "(feet %.1f %.1f hands %.1f %.1f) without clips: shown %d ends %.1f deg\n",
                                aTag.c_str(), aWith[i].t, aWith[i].dt, aWith[i].probe.active, aWith[i].probe.start, aWith[i].probe.handback,
                                aWith[i].probe.clipTime, aWith[i].shown, aWith[i].under, p1[aB.hips].x, p1[aB.hips].y, p1[aB.hips].z,
                                Dist(p0[aB.hips], p1[aB.hips]) * 100.0f, e1, ek[0], ek[1], ek[2], ek[3], i < aBase.size() ? aBase[i].shown : -9, eb);
                }
            }
        };
        float hIn, eIn, hOut, eOut, zIn, zOut;
        size_t exitEnd = e.end;
        while (exitEnd + 1 < aWith.size() && aWith[exitEnd + 1].t - aWith[e.end].t <= dodge::kBlendOutS + 0.05f)
            ++exitEnd;
        if (aPrint && std::getenv("UTPP_DODGE_TRACE")) // the whole episode, frame by frame
            steps(e.start, exitEnd, hIn, eIn, zIn);
        const bool print = aPrint;
        aPrint = false;
        steps(e.start, e.start + 6, hIn, eIn, zIn);
        steps(e.end, exitEnd, hOut, eOut, zOut);
        aPrint = print;
        m.hipsHeightEntry = std::fmax(m.hipsHeightEntry, zIn);
        m.hipsHeightExit = std::fmax(m.hipsHeightExit, zOut);
        // hips height step added by the entry / hand-back: each frame's height step minus the larger of the shown clip's own (both
        // frames on the clip) and the motion pose's own (the replay without the clips, same frames)
        auto heightExcess = [&](size_t a0, size_t a1) {
            float x = -1e9f;
            std::vector<float> ca(size_t(aDb.bones) * 7), cb(ca.size());
            for (size_t i = std::max<size_t>(a0, 1); i <= a1 && i < aWith.size(); ++i)
            {
                Model(aWith[i - 1].pose.data(), aDb, q0, p0);
                Model(aWith[i].pose.data(), aDb, q1, p1);
                const float dz = std::fabs(p1[aB.hips].z - p0[aB.hips].z);
                float own = 0.0f;
                const Frame &f0 = aWith[i - 1], &f1 = aWith[i];
                if (f0.probe.active && f1.probe.active && f0.probe.sector == f1.probe.sector && !f1.probe.start)
                {
                    dodge::SampleDodge(aC, static_cast<dodge::Sector>(f0.probe.sector), f0.probe.clipTime, ca.data());
                    dodge::SampleDodge(aC, static_cast<dodge::Sector>(f1.probe.sector), f1.probe.clipTime, cb.data());
                    Model(ca.data(), aDb, qs, ps);
                    Model(cb.data(), aDb, qc, pc);
                    own = std::fabs(pc[aB.hips].z - ps[aB.hips].z);
                }
                if (i < aBase.size())
                {
                    Model(aBase[i - 1].pose.data(), aDb, qs, ps);
                    Model(aBase[i].pose.data(), aDb, qc, pc);
                    own = std::fmax(own, std::fabs(pc[aB.hips].z - ps[aB.hips].z));
                }
                x = std::fmax(x, dz - own);
            }
            return x;
        };
        m.hipsHeightExcess = std::fmax(m.hipsHeightExcess, std::fmax(heightExcess(e.start, e.start + 6), heightExcess(e.end, exitEnd)));
        // hands (model) over the upper-body release and the hand-back: largest single-frame step with the clips against the largest of
        // the replay without the clips over the same frames
        {
            const float len = aC.length[static_cast<int>(e.sector < 0 ? 0 : e.sector)];
            float with = 0.0f, base = 0.0f;
            for (size_t i = e.start + 1; i <= exitEnd && i < aWith.size(); ++i)
            {
                const Frame& f = aWith[i];
                if (f.probe.active && f.probe.clipTime <= len - dodge::kUpperReleaseS * dodge::kRate)
                    continue;
                Model(aWith[i - 1].pose.data(), aDb, q0, p0);
                Model(f.pose.data(), aDb, q1, p1);
                for (int k = 2; k < 4; ++k)
                    with = std::fmax(with, AngleDeg(q0[aB.ends[k]], q1[aB.ends[k]]));
                if (i < aBase.size())
                {
                    Model(aBase[i - 1].pose.data(), aDb, qs, ps);
                    Model(aBase[i].pose.data(), aDb, qc, pc);
                    for (int k = 2; k < 4; ++k)
                        base = std::fmax(base, AngleDeg(qs[aB.ends[k]], qc[aB.ends[k]]));
                }
            }
            m.releaseHands = std::fmax(m.releaseHands, with);
            m.releaseHandsBase = std::fmax(m.releaseHandsBase, base);
            m.releaseHandsExcess = std::fmax(m.releaseHandsExcess, with - base);
        }
        // hands (model) on the entry window: the same against the replay without the clips
        {
            float with = 0.0f, base = 0.0f;
            for (size_t i = e.start + 1; i <= e.start + 6 && i < aWith.size(); ++i)
            {
                Model(aWith[i - 1].pose.data(), aDb, q0, p0);
                Model(aWith[i].pose.data(), aDb, q1, p1);
                for (int k = 2; k < 4; ++k)
                    with = std::fmax(with, AngleDeg(q0[aB.ends[k]], q1[aB.ends[k]]));
                if (i < aBase.size())
                {
                    Model(aBase[i - 1].pose.data(), aDb, qs, ps);
                    Model(aBase[i].pose.data(), aDb, qc, pc);
                    for (int k = 2; k < 4; ++k)
                        base = std::fmax(base, AngleDeg(qs[aB.ends[k]], qc[aB.ends[k]]));
                }
            }
            m.entryHands = std::fmax(m.entryHands, with);
            m.entryHandsBase = std::fmax(m.entryHandsBase, base);
            m.entryHandsExcess = std::fmax(m.entryHandsExcess, with - base);
        }
        m.hipsEntry = std::fmax(m.hipsEntry, hIn);
        m.hipsExit = std::fmax(m.hipsExit, hOut);
        m.endsEntry = std::fmax(m.endsEntry, eIn);
        m.endsExit = std::fmax(m.endsExit, eOut);
        m.hipsExcess = std::fmax(m.hipsExcess, std::fmax(hIn, hOut) - ownH);
        m.endsExcess = std::fmax(m.endsExcess, std::fmax(eIn, eOut) - ownE);
        // sector: impulse direction (the feed of the pick) against the drawn heading before that feed
        const Frame& pf = aWith[e.start];
        e.expectRel = WrapDeg(pf.velYawDeg - pf.rootYawDeg);
        e.actorRel = WrapDeg(pf.velYawDeg - pf.yawDeg);
        e.expected = static_cast<int32_t>(dodge::ClassifySector(e.expectRel));
        m.sectorOk += e.expected == e.firstSector ? 1 : 0; // a later change inside the settle window is DodgePlayer logic (utpp_loco_dodge_test)
        // offset (visual - capsule) over the episode + blend-out, with and without the clips
        for (size_t i = e.start; i <= exitEnd; ++i)
        {
            if (aPrint && std::getenv("UTPP_DODGE_TRACE"))
                std::printf("TRACE %s t=%.3f speed %.2f active %d clipT %.3f offset with (%.3f %.3f %.1f deg) without (%.3f %.3f %.1f deg)\n",
                            aTag.c_str(), aWith[i].t, aWith[i].speed, aWith[i].probe.active, aWith[i].probe.clipTime, aWith[i].offset[0],
                            aWith[i].offset[1], aWith[i].offset[2] * 57.3f, i < aBase.size() ? aBase[i].offset[0] : 0.0f,
                            i < aBase.size() ? aBase[i].offset[1] : 0.0f, i < aBase.size() ? aBase[i].offset[2] * 57.3f : 0.0f);
            m.offsetMax = std::fmax(m.offsetMax, std::hypot(aWith[i].offset[0], aWith[i].offset[1]));
            if (i < aBase.size())
                m.offsetMaxBase = std::fmax(m.offsetMaxBase, std::hypot(aBase[i].offset[0], aBase[i].offset[1]));
        }
        m.offsetEnd = std::fmax(m.offsetEnd, std::hypot(aWith[exitEnd].offset[0], aWith[exitEnd].offset[1]));
        if (exitEnd < aBase.size())
            m.offsetEndBase = std::fmax(m.offsetEndBase, std::hypot(aBase[exitEnd].offset[0], aBase[exitEnd].offset[1]));
        // planted-foot slide: world foot (capsule + visual frame + model) of the lower foot while both frames are below 12 cm
        for (size_t i = e.start + 1; i <= exitEnd; ++i)
        {
            auto world = [&](const Frame& f, int bone, float& z) {
                Model(f.pose.data(), aDb, qs, ps);
                const mm::math::Vec3 p = ps[bone];
                float lx, ly;
                mm::math::Rot2(f.offset[2], p.x, p.y, lx, ly);
                lx += f.offset[0];
                ly += f.offset[1];
                float wx, wy;
                mm::math::Rot2(f.yawDeg * mm::integ::kDegToRad, lx, ly, wx, wy);
                z = p.z;
                return std::array<float, 2>{wx + f.cap[0], wy + f.cap[1]};
            };
            float best = 1e9f;
            for (int k = 0; k < 2; ++k)
            {
                float z0, z1;
                const auto a = world(aWith[i - 1], aB.ends[k], z0);
                const auto b = world(aWith[i], aB.ends[k], z1);
                if (z0 < 0.12f && z1 < 0.12f)
                    best = std::fmin(best, std::hypot(b[0] - a[0], b[1] - a[1]));
            }
            if (best < 1e8f)
                m.slideCm += best * 100.0f;
        }
        // hand-back vs the capsule stop (speed < 0.5 m/s after the dodge)
        size_t stop = e.start;
        while (stop + 1 < aWith.size() && aWith[stop].speed >= 0.5f)
            ++stop;
        m.stopLagMax = std::fmax(m.stopLagMax, std::fabs(aWith[e.end].t - aWith[stop].t));
        // shown clip changes per second over [start - 0.5 s, start + 1.5 s]
        int changes = 0, changesBase = 0;
        float t0 = aWith[e.start].t - 0.5f, t1 = aWith[e.start].t + 1.5f;
        for (size_t i = 1; i < aWith.size(); ++i)
            if (aWith[i].t >= t0 && aWith[i].t <= t1)
            {
                changes += aWith[i].shown != aWith[i - 1].shown ? 1 : 0;
                changesBase += i < aBase.size() && aBase[i].shown != aBase[i - 1].shown ? 1 : 0;
            }
        m.changesMax = std::fmax(m.changesMax, changes / (t1 - t0));
        m.changesMaxBase = std::fmax(m.changesMaxBase, changesBase / (t1 - t0));
        if (aPrint)
            std::printf("INFO %s clip %s at t=%.3f..%.3f: rel to drawn heading %.0f deg (to the actor %.0f) -> expected %s; entry steps hips %.1f cm / ends %.1f deg, "
                        "exit hips %.1f cm / ends %.1f deg (clip own %.1f cm / %.1f deg); hand-back - capsule stop %.3f s\n",
                        aTag.c_str(), dodge::SectorName(static_cast<dodge::Sector>(e.sector)), aWith[e.start].t, aWith[e.end].t,
                        e.expectRel, e.actorRel, dodge::SectorName(static_cast<dodge::Sector>(e.expected)), hIn * 100.0f, eIn,
                        hOut * 100.0f, eOut, ownH * 100.0f, ownE, aWith[e.end].t - aWith[stop].t);
    }
    return m;
}

// Copy of aRows with the first back dodge of aFrames (a clip of sector Back) turned by aTurnDeg: the velocity of the dodge feeds and
// of the 0.6 s from their start is rotated, the capsule path takes the rotated displacements and every later row is shifted by the
// accumulated difference; the body yaw and the stick are kept. Empty when the replay shows no back dodge.
std::vector<Row> TurnImpulse(const std::vector<Row>& aRows, const std::vector<Frame>& aFrames, float aTurnDeg)
{
    size_t start = aRows.size();
    for (size_t i = 0; i < aFrames.size() && i < aRows.size(); ++i)
        if (aFrames[i].probe.active && aFrames[i].probe.start && aFrames[i].probe.sector == static_cast<int32_t>(dodge::Sector::Back))
        {
            start = i;
            break;
        }
    if (start >= aRows.size())
        return {};
    while (start > 0 && aRows[start - 1].detail == 7)
        --start;
    const float c = std::cos(aTurnDeg * mm::integ::kDegToRad), s = std::sin(aTurnDeg * mm::integ::kDegToRad);
    std::vector<Row> out = aRows;
    float shift[2] = {0.0f, 0.0f};
    for (size_t i = start; i < out.size(); ++i)
    {
        if (aRows[i].t - aRows[start].t <= 0.6f || aRows[i].detail == 7)
        {
            const float vx = aRows[i].v[0], vy = aRows[i].v[1];
            out[i].v[0] = c * vx - s * vy;
            out[i].v[1] = s * vx + c * vy;
            if (i > start)
            {
                const float dx = aRows[i].cap[0] - aRows[i - 1].cap[0], dy = aRows[i].cap[1] - aRows[i - 1].cap[1];
                shift[0] += (c * dx - s * dy) - dx;
                shift[1] += (s * dx + c * dy) - dy;
            }
        }
        out[i].cap[0] = aRows[i].cap[0] + shift[0];
        out[i].cap[1] = aRows[i].cap[1] + shift[1];
    }
    return out;
}

// UTPP_DODGE_DUMP=<folder>: one binary file <tag>_<variant>.bin per replay: int32 frames, int32 bones, then per frame 10 floats
// (t, capsule x y z, yaw deg, visual offset x y yaw rad, clip shown, sector) and bones * 7 floats of the local pose.
void Dump(const std::string& aTag, const char* aVariant, const std::vector<Frame>& aFrames, const Db& aDb)
{
    const char* dir = std::getenv("UTPP_DODGE_DUMP");
    if (dir == nullptr || *dir == 0)
        return;
    std::string name = aTag + "_" + aVariant;
    for (char& ch : name)
        if (!(std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '-'))
            ch = '_';
    std::ofstream f(std::string(dir) + "\\" + name + ".bin", std::ios::binary);
    const int32_t n = static_cast<int32_t>(aFrames.size()), nb = static_cast<int32_t>(aDb.bones);
    f.write(reinterpret_cast<const char*>(&n), 4);
    f.write(reinterpret_cast<const char*>(&nb), 4);
    for (const Frame& fr : aFrames)
    {
        const float h[10] = {fr.t, fr.cap[0], fr.cap[1], fr.cap[2], fr.yawDeg, fr.offset[0], fr.offset[1], fr.offset[2],
                             fr.probe.active ? 1.0f : 0.0f, static_cast<float>(fr.probe.sector)};
        f.write(reinterpret_cast<const char*>(h), sizeof h);
        std::vector<float> pv = fr.pose;
        pv.resize(size_t(nb) * 7, 0.0f);
        f.write(reinterpret_cast<const char*>(pv.data()), static_cast<std::streamsize>(pv.size() * sizeof(float)));
    }
}

bool SamePoses(const std::vector<Frame>& a, const std::vector<Frame>& b, size_t aUpTo)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < std::min(aUpTo, a.size()); ++i)
        if (a[i].applied != b[i].applied || a[i].pose != b[i].pose)
            return false;
    return true;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::printf("usage: utpp_motion_driver_dodge_test <data dir> <fixture dir>\n");
        return 2;
    }
    const std::string data = argv[1], fix = argv[2];
    const std::string corrupt = "utpp_dodge_invalid_test.utpptrav"; // written to the working directory (the build folder), removed at the end
    struct Rec
    {
        const char* body;
        int32_t index;
        const char* feed;
    };
    const Rec recs[] = {{"woman", 0, "dodge_feed_woman_a.txt"}, {"woman", 0, "dodge_feed_woman_b.txt"}, {"man", 1, "dodge_feed_man.txt"}};
    std::printf("INFO clip rate %.2f, entry blend %.2f s, hand-back blend %.2f s\n", static_cast<double>(dodge::kRate),
                static_cast<double>(dodge::kBlendInS), static_cast<double>(dodge::kBlendOutS));
    for (const Rec& rec : recs)
    {
        const std::string tag = std::string(rec.feed);
        motiondriver::Paths base = DataPaths(data, rec.body);
        Db db;
        Check(LoadDb(base.files.db, db), tag + ": database bones", base.files.db);
        const std::vector<Row> rows = ReadRows(fix + "\\" + rec.feed);
        Check(rows.size() > 300, tag + ": recorded rows", std::to_string(rows.size()));
        if (db.bones == 0 || rows.size() <= 300)
            continue;
        Bones bn;
        bn.hips = BoneIndex(db, "Hips");
        const char* legs[4] = {"LeftUpLeg", "RightUpLeg", "LeftLeg", "RightLeg"};
        const char* ends[4] = {"LeftFoot", "RightFoot", "LeftHand", "RightHand"};
        bool bonesOk = bn.hips >= 0;
        for (int k = 0; k < 4; ++k)
        {
            bn.legs[k] = BoneIndex(db, legs[k]);
            bn.ends[k] = BoneIndex(db, ends[k]);
            bonesOk = bonesOk && bn.legs[k] >= 0 && bn.ends[k] >= 0;
        }
        for (const char* u : {"LeftArm", "RightArm", "LeftForeArm", "RightForeArm", "Head"})
        {
            bn.upper.push_back(BoneIndex(db, u));
            bonesOk = bonesOk && bn.upper.back() >= 0;
        }
        Check(bonesOk, tag + ": hips / leg / foot / hand / arm / head bones");
        const std::string dodgeFile = fix + "\\utpp_dodge_" + rec.body + ".utpptrav";
        mm::traversal::TraversalFile tf;
        dodge::DodgeClips clips;
        std::string err;
        const bool clipsOk = mm::traversal::LoadTraversalFile(dodgeFile, tf, err) &&
            dodge::BuildDodgeClips(tf, std::string("player_") + rec.body + "_skeleton", db.bones,
                                   [&](uint32_t i) { return std::string_view(db.names[i]); }, clips);
        Check(clipsOk, tag + ": dodge clips load", clipsOk ? dodgeFile : err + clips.err);
        if (!bonesOk || !clipsOk)
            continue;

        motiondriver::Paths missing = base, invalid = base, with = base;
        missing.dodge = fix + "\\does-not-exist.utpptrav";
        invalid.dodge = corrupt;
        with.dodge = dodgeFile;
        {
            std::ofstream bad(corrupt, std::ios::binary);
            bad << "utpp-traversal 2\nRIG player_" << rec.body << "_skeleton 0x0 1\nPOSEBONES 1 Root\nCOUNT 0\n";
        }
        Check(Session(rec.index, missing, db), tag + ": session without the dodge file");
        Check(!motiondriver::TestDodgeProbe().loaded, tag + ": missing file -> clips not loaded");
        const std::vector<Frame> noFile = Replay(rows, db);
        Check(Session(rec.index, invalid, db), tag + ": session with an invalid dodge file");
        Check(!motiondriver::TestDodgeProbe().loaded, tag + ": invalid file -> clips not loaded (fail closed)");
        const std::vector<Frame> badFile = Replay(rows, db);
        Check(SamePoses(noFile, badFile, noFile.size()), tag + ": invalid file = missing file, byte-identical replay");
        Check(Session(rec.index, with, db), tag + ": session with the dodge file");
        Check(motiondriver::TestDodgeProbe().loaded, tag + ": dodge clips loaded by the driver");
        const std::vector<Frame> withFile = Replay(rows, db);
        int applied = 0;
        for (const Frame& f : withFile)
            applied += f.applied ? 1 : 0;
        Check(applied > static_cast<int>(withFile.size()) * 9 / 10, tag + ": motion pose on the replayed feeds", std::to_string(applied));
        const Metrics m = Analyse(tag, withFile, noFile, clips, db, bn, true);
        Check(SamePoses(withFile, noFile, m.firstDodge), tag + ": before the first dodge the output is byte-identical with and without the clips",
              std::to_string(m.firstDodge) + " frames");
        Check(m.dodges == 4 && m.outcomeOk == m.dodges,
              tag + ": every recorded dodge shows the clip of its sector, or none for a forward dodge out of a run",
              std::to_string(m.outcomeOk) + " / " + std::to_string(m.dodges) + " dodges, " + std::to_string(m.clips) + " clips");
        Check(m.sectorOk == m.clips, tag + ": each clip is the sector of the impulse relative to the drawn heading",
              std::to_string(m.sectorOk) + " / " + std::to_string(m.clips));
        Check(m.legMaxDeg < 3.0f, tag + ": legs follow the picked clip after the entry blend", F("max %.2f deg", m.legMaxDeg));
        Check(m.upperFrames > 0 && m.upperMaxDeg < 1.0f, tag + ": arms and head keep the pose shown before the clip",
              F("max %.2f deg over %.0f frames", m.upperMaxDeg, m.upperFrames));
        Dump(tag, "without", noFile, db);
        Dump(tag, "with", withFile, db);
        Check(m.hipsExcess <= 0.03f, tag + ": hips single-frame step at entry / hand-back within 3 cm of the clip's own",
              F("entry %.1f exit %.1f cm, excess %.1f cm", m.hipsEntry * 100, m.hipsExit * 100, m.hipsExcess * 100));
        Check(m.endsExcess <= 6.0f, tag + ": feet / hands single-frame step at entry / hand-back within 6 deg of the clip's own",
              F("entry %.1f exit %.1f deg, excess %.1f deg", m.endsEntry, m.endsExit, m.endsExcess));
        Check(m.changesMax <= 3.0f, tag + ": shown clip changes per second over each dodge <= 3.0", F("%.2f (without the clips %.2f)", m.changesMax, m.changesMaxBase));
        Check(m.hipsHeightExcess <= 0.03f, tag + ": hips height step at entry / hand-back within 3 cm of the shown clip's / motion pose's own",
              F("entry %.1f exit %.1f cm, excess %.1f cm", m.hipsHeightEntry * 100.0f, m.hipsHeightExit * 100.0f, m.hipsHeightExcess * 100.0f));
        std::printf("INFO %s visual - capsule offset over the dodges: max %.1f cm with the clips, %.1f cm without; at the end of the hand-back "
                    "%.1f cm with, %.1f cm without; planted-foot slide %.1f cm; hand-back vs capsule stop max %.3f s\n",
                    tag.c_str(), m.offsetMax * 100.0f, m.offsetMaxBase * 100.0f, m.offsetEnd * 100.0f, m.offsetEndBase * 100.0f, m.slideCm,
                    m.stopLagMax);
        for (float hz : {30.0f, 60.0f, 120.0f})
        {
            const std::vector<Row> rs = Resample(rows, hz);
            Check(Session(rec.index, missing, db), tag + F(" %.0f Hz: session without the file", hz));
            const std::vector<Frame> b = Replay(rs, db);
            Check(Session(rec.index, with, db), tag + F(" %.0f Hz: session with the file", hz));
            const std::vector<Frame> w = Replay(rs, db);
            const Metrics r = Analyse(tag + F(" %.0fHz", hz), w, b, clips, db, bn, std::getenv("UTPP_DODGE_TRACE") != nullptr);
            const std::string t = tag + F(" %.0f Hz", hz);
            Check(r.dodges == 4 && r.outcomeOk == r.dodges && r.sectorOk == r.clips, t + ": clips and sectors",
                  std::to_string(r.outcomeOk) + " / " + std::to_string(r.dodges) + " dodges, sectors " + std::to_string(r.sectorOk) + " / " +
                      std::to_string(r.clips));
            Check(r.legMaxDeg < 3.0f, t + ": legs follow the picked clip", F("max %.2f deg", r.legMaxDeg));
            // a blend moves the pose at a speed, so its single-frame step grows with the frame time: below 60 Hz the limits are per
            // 1/60 s of frame time; a step that does not shrink with the frame time (a cut) still fails at 60 and 120 Hz
            const float perFrame = std::fmax(1.0f, 60.0f / hz);
            Check(r.hipsExcess <= 0.03f * perFrame, t + ": hips entry / hand-back steps within 3 cm per 1/60 s of the clip's own",
                  F("entry %.1f exit %.1f cm (excess %.1f)", r.hipsEntry * 100, r.hipsExit * 100, r.hipsExcess * 100));
            Check(r.endsExcess <= 6.0f * perFrame, t + ": feet / hands entry / hand-back steps within 6 deg per 1/60 s of the clip's own",
                  F("entry %.1f exit %.1f deg (excess %.1f)", r.endsEntry, r.endsExit, r.endsExcess));
            std::printf("INFO %s hands over the release and hand-back: largest step %.1f deg (without the clips %.1f)\n", t.c_str(), r.releaseHands,
                        r.releaseHandsBase);
            // the resampled feed can change clips more often than 3.0 / s without any dodge clip; the dodge must not add to that
            Check(r.changesMax <= std::fmax(3.0f, r.changesMaxBase), t + ": shown clip changes per second <= 3.0, or no more than without the clips",
                  F("%.2f (without the clips %.2f)", r.changesMax, r.changesMaxBase));
            Check(r.hipsHeightExcess <= 0.03f * perFrame, t + ": hips height step at entry / hand-back within 3 cm per 1/60 s of the shown clip's / motion pose's own",
                  F("entry %.1f exit %.1f cm, excess %.1f cm", r.hipsHeightEntry * 100.0f, r.hipsHeightExit * 100.0f, r.hipsHeightExcess * 100.0f));
            std::printf("INFO %s offset max %.1f cm (without %.1f), end %.1f cm (without %.1f), slide %.1f cm, hand-back vs stop %.3f s\n",
                        t.c_str(), r.offsetMax * 100.0f, r.offsetMaxBase * 100.0f, r.offsetEnd * 100.0f, r.offsetEndBase * 100.0f, r.slideCm,
                        r.stopLagMax);
        }
        // a drawn weapon / aiming: the weapon layers act on top of the dodge pose (the held upper body is the base under them); the hands
        // at the entry, over the release and the hand-back step no more than 6 deg beyond the same replay without the clips
        struct Ov
        {
            int32_t pose, aim;
            const char* name;
        };
        for (const Ov& o : {Ov{1, 1, "rifle ready"}, Ov{1, 2, "rifle aiming"}, Ov{3, 2, "pistol two-handed aiming"}})
        {
            const std::string t = tag + " " + o.name;
            Check(Session(rec.index, missing, db), t + ": session without the file");
            motiondriver::SetOverlayInputs(o.pose, o.aim, false, false);
            const std::vector<Frame> b = Replay(rows, db);
            Check(Session(rec.index, with, db), t + ": session with the file");
            motiondriver::SetOverlayInputs(o.pose, o.aim, false, false);
            const std::vector<Frame> w = Replay(rows, db);
            motiondriver::SetOverlayInputs(0, 0, false, false);
            const Metrics r = Analyse(t, w, b, clips, db, bn, std::getenv("UTPP_DODGE_TRACE") != nullptr);
            Check(r.dodges == 4 && r.outcomeOk == r.dodges && r.sectorOk == r.clips, t + ": the same clips and sectors as without the weapon",
                  F("%.0f / %.0f dodges, sectors %.0f / %.0f", r.outcomeOk, r.dodges, r.sectorOk, r.clips));
            // the weapon stance also moves the legs away from the clip (printed: rifle 15 - 19.5 deg, two-handed pistol 4.6 - 5.1 deg)
            std::printf("INFO %s legs vs the clip after the entry blend: max %.2f deg\n", t.c_str(), r.legMaxDeg);
            Check(r.entryHandsExcess <= 6.0f && r.releaseHandsExcess <= 6.0f,
                  t + ": hands at the entry / over the release and hand-back within 6 deg of the replay without the clips",
                  F("entry %.1f (without %.1f) release + hand-back %.1f (without %.1f) deg", r.entryHands, r.entryHandsBase, r.releaseHands,
                    r.releaseHandsBase));
        }
        // outside the gate nothing is written: the same feed with every dodge crouched (Crouch 3 for 0.6 s around each dodge, the dodge
        // feeds stay Dodge 7: the stance holds through them) or in the air (the dodge feeds are DodgeAir 22) shows no clip and the output
        // is byte-identical with and without the clip file
        for (int kind = 0; kind < 2; ++kind)
        {
            std::vector<Row> v = rows;
            auto setDetail = [](Row& aRow, int aDetail) {
                aRow.psm = (aRow.psm & ~(0xFFu << 16)) | (static_cast<uint32_t>(aDetail + 1) << 16);
            };
            for (size_t i = 0; i < v.size(); ++i)
            {
                if (rows[i].detail != 7)
                {
                    bool near = false;
                    for (size_t k = 0; k < rows.size() && !near; ++k)
                        near = rows[k].detail == 7 && std::fabs(rows[k].t - rows[i].t) <= 0.6f;
                    if (kind == 0 && near)
                        setDetail(v[i], 3);
                }
                else if (kind == 1)
                    setDetail(v[i], 22);
            }
            const std::string t = tag + (kind == 0 ? " crouched" : " in the air");
            Check(Session(rec.index, missing, db), t + ": session without the file");
            const std::vector<Frame> b = Replay(v, db);
            Check(Session(rec.index, with, db), t + ": session with the file");
            const uint32_t clips0 = motiondriver::TestDodgeProbe().clips;
            const std::vector<Frame> w = Replay(v, db);
            int shown = 0, applied = 0;
            for (const Frame& f : w)
            {
                shown += f.probe.active ? 1 : 0;
                applied += f.applied ? 1 : 0;
            }
            Check(shown == 0 && motiondriver::TestDodgeProbe().clips == clips0, t + ": no dodge clip is shown",
                  F("%.0f clip frames, %.0f clips started", shown, motiondriver::TestDodgeProbe().clips - clips0));
            Check(applied > static_cast<int>(w.size()) * 9 / 10 && SamePoses(w, b, w.size()),
                  t + ": the output is byte-identical with and without the clip file", F("%.0f / %.0f frames applied", applied, w.size()));
        }
        // side dodges: the recorded back dodge with its impulse turned 90 degrees against the body (left / right), recorded rate
        for (float turnDeg : {90.0f, -90.0f})
        {
            const std::vector<Row> side = TurnImpulse(rows, withFile, turnDeg);
            const std::string t = tag + F(" side %+.0f", turnDeg);
            Check(!side.empty(), t + ": back dodge found in the recording");
            if (side.empty())
                continue;
            Check(Session(rec.index, missing, db), t + ": session without the file");
            const std::vector<Frame> b = Replay(side, db);
            Check(Session(rec.index, with, db), t + ": session with the file");
            const std::vector<Frame> w = Replay(side, db);
            const Metrics r = Analyse(t, w, b, clips, db, bn, true);
            // back (180 deg) turned by +90 = -90 relative to the body = right; by -90 = +90 = left (positive = left)
            const int want = static_cast<int>(turnDeg > 0.0f ? dodge::Sector::Right : dodge::Sector::Left);
            Check(r.dodges == 4 && r.outcomeOk == r.dodges && r.sectorOk == r.clips && (r.sectorBits & (1u << want)) != 0,
                  t + ": the turned dodge shows the " + dodge::SectorName(static_cast<dodge::Sector>(want)) + " clip, the others as recorded",
                  std::to_string(r.outcomeOk) + " / " + std::to_string(r.dodges) + " dodges, sectors " + std::to_string(r.sectorOk) + " / " +
                      std::to_string(r.clips));
            Check(r.legMaxDeg < 3.0f, t + ": legs follow the picked clip", F("max %.2f deg", r.legMaxDeg));
            Check(r.upperFrames > 0 && r.upperMaxDeg < 1.0f, t + ": arms and head keep the pose shown before the clip",
                  F("max %.2f deg over %.0f frames", r.upperMaxDeg, r.upperFrames));
            Check(r.hipsExcess <= 0.03f && r.endsExcess <= 6.0f, t + ": entry / hand-back steps within 3 cm / 6 deg of the clip's own",
                  F("hips entry %.1f exit %.1f cm, ends entry %.1f exit %.1f deg", r.hipsEntry * 100, r.hipsExit * 100, r.endsEntry, r.endsExit));
            Check(r.hipsHeightExcess <= 0.03f, t + ": hips height step at entry / hand-back within 3 cm of the shown clip's / motion pose's own",
                  F("entry %.1f exit %.1f cm, excess %.1f cm", r.hipsHeightEntry * 100.0f, r.hipsHeightExit * 100.0f, r.hipsHeightExcess * 100.0f));
            Check(r.changesMax <= 3.0f, t + ": shown clip changes per second <= 3.0", F("%.2f (without the clips %.2f)", r.changesMax, r.changesMaxBase));
            Dump(t, "without", b, db);
            Dump(t, "with", w, db);
        }
    }
    std::remove(corrupt.c_str());
    motiondriver::SetEnabled(false);
    motiondriver::Shutdown();
    std::printf("%s: %d passed, %d failed\n", g_fail ? "FAILED" : "PASSED", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
