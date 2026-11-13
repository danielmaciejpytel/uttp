// UTPP native get-up (src/ragdoll/GetUpPlayer.hpp, RdLastPose.hpp, src/mm/MotionAdapter.hpp ragdollRecover): offline checks of the pure logic.
// Usage: utpp_ragdoll_getup_test [tests/data dir]   (optional fixtures getup_frame0_<body>.json: frame-0 model joints of the baked clips)
//
// Sections: RdLastPose store, lie classifier, yaw alignment + plan, blend / floor clamp / world anchoring, phase machine, loader
// (utpp_getup_<body>.utpptrav grammar), MapToChooser after a ragdoll (false jump / landing).
// Mutations (build with -DUTPP_GETUP_MUTATION=k, CMake utpp_ragdoll_getup_mut<k>; each MUST fail):
//   1 lateral axis sign swapped (L and R clips), 2 alignment ignores the weights, 3 no floor clamp, 4 plan-time capsule instead of
//   the live one, 6 a smaller descriptor overwrites the full-body pose, 7 no early input / off-anchor hand-off.
//   10 the whole-body lift releases as fast as it rises (the r13c release continuity), 11 no hand-off foot step (planted toes slide).
//   12 no hand step in FloorCorrect (r13d: the fingers of the ragdoll pose stay under the floor at the release).
//   13 the blend has the plain 0.10 m lift cap (seq 9: a 0.21 m crossfade residual leaves the body 0.11 m in the floor).
//   9 (GuTarget) the live sample is trusted at once: a stale pre-teleport sample re-expresses the release pose (CMake: add 9 to the
//   _utpp_gu_mut list, expected output "FAIL release target").
// -DUTPP_RAGDOLL_RECOVER_MUTATION=5: no distance guard in MapToChooser (the ledge case MUST fail).
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "../src/mm/MotionAdapter.hpp"
#include "../src/mm/MiniJson.hpp"
#include "../src/ragdoll/GetUpClips.hpp"
#include "../src/ragdoll/GetUpPlayer.hpp"
#include "../src/ragdoll/RdLastPose.hpp"
#include "TestFailFast.hpp"

using namespace utpp::ragdoll;
namespace adapter = utpp::mm::adapter;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat.c_str(), aDetail.empty() ? "" : " | ", aDetail.c_str());
    if (!aOk)
        UtppFailFastNote(std::string("FAIL ") + aWhat + (aDetail.empty() ? "" : " | ") + aDetail);
}
std::string F(double a, double b = 0.0, double c = 0.0)
{
    char buf[96];
    std::snprintf(buf, sizeof buf, "%.4f %.4f %.4f", a, b, c);
    return buf;
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad = kPi / 180.0;
constexpr uint32_t kN = kGetUpBones, kFirst = 2;

// ---------------------------------------------------------------------------------------------- the 73-bone test rig
std::string ParentName(const std::string& s)
{
    static const std::map<std::string, std::string> kMap = {
        {"Trajectory", "Root"}, {"Hips", "Root"}, {"reference_joint", "Root"}, {"Spine", "Hips"}, {"LeftUpLeg", "Hips"},
        {"RightUpLeg", "Hips"}, {"Spine1", "Spine"}, {"LeftLeg", "LeftUpLeg"}, {"RightLeg", "RightUpLeg"}, {"Spine2", "Spine1"},
        {"LeftFoot", "LeftLeg"}, {"RightFoot", "RightLeg"}, {"Spine3", "Spine2"}, {"LeftShoulder", "Spine3"},
        {"RightShoulder", "Spine3"}, {"Neck", "Spine3"}, {"LeftArm", "LeftShoulder"}, {"RightArm", "RightShoulder"},
        {"Neck1", "Neck"}, {"LeftForeArm", "LeftArm"}, {"RightForeArm", "RightArm"}, {"Head", "Neck1"}, {"LeftHand", "LeftForeArm"},
        {"RightHand", "RightForeArm"}, {"LeftHandEnd", "LeftHand"}, {"WeaponLeft", "LeftHand"}, {"RightHandEnd", "RightHand"},
        {"WeaponRight", "RightHand"}, {"LeftHeel", "LeftFoot"}, {"RightHeel", "RightFoot"}, {"LeftEye", "Head"}, {"RightEye", "Head"},
        {"LeftToeBase", "LeftFoot"}, {"RightToeBase", "RightFoot"}};
    const auto it = kMap.find(s);
    if (it != kMap.end())
        return it->second;
    if (s.rfind("LeftInHand", 0) == 0)
        return "LeftHand";
    if (s.rfind("RightInHand", 0) == 0)
        return "RightHand";
    const char last = s.back();
    if (s.size() > 5 && (s.rfind("LeftHand", 0) == 0 || s.rfind("RightHand", 0) == 0) && last >= '1' && last <= '3')
    {
        if (last == '1') // the game rig: XHand<Finger>1 hangs from the metacarpal XInHand<Finger>
        {
            const bool left = s.rfind("Left", 0) == 0;
            return (left ? "LeftInHand" : "RightInHand") + s.substr(left ? 8 : 9, s.size() - (left ? 8 : 9) - 1);
        }
        std::string p = s;
        p.back() = static_cast<char>(last - 1);
        return p;
    }
    return "Root";
}

std::vector<int32_t> MakeParents()
{
    std::vector<int32_t> par(kN, -1);
    for (uint32_t i = 1; i < kN; ++i)
    {
        const std::string pn = ParentName(kGetUpBoneNames[i]);
        for (uint32_t k = 0; k < kN; ++k)
            if (pn == kGetUpBoneNames[k])
                par[i] = static_cast<int32_t>(k);
    }
    return par;
}

const std::vector<int32_t>& Parents()
{
    static const std::vector<int32_t> p = MakeParents();
    return p;
}
const GuBones& Bones()
{
    static const GuBones b = ResolveGuBones(kN, [](uint32_t i) { return std::string_view(kGetUpBoneNames[i]); });
    return b;
}

using Pose = std::vector<float>;
Pose FromTable(const float* aTable) { return Pose(aTable, aTable + size_t(kN) * 7); }

struct Fk
{
    std::vector<float> p, q;
};
Fk DoFk(const Pose& aLocal)
{
    Fk f;
    f.p.resize(size_t(kN) * 3);
    f.q.resize(size_t(kN) * 4);
    gu::FkModel(Parents().data(), kN, kFirst, aLocal.data(), f.p.data(), f.q.data());
    return f;
}
const float* Joint(const Fk& aFk, int aJ) { return &aFk.p[size_t(Bones().joint[aJ]) * 3]; }
float Dist(const float* a, const float* b) { return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2])); }
Pose MorphPose(const Pose& aA, const Pose& aB, float aU);

// rotates the whole body about its Hips by aR (applied to every root child)
Pose Rotate(Pose aPose, const gu::Q& aR, const float aPivot[3])
{
    for (uint32_t i = kFirst; i < kN; ++i)
        if (gu::RootChild(Parents().data(), i, kFirst))
        {
            float* l = &aPose[size_t(i) * 7];
            const float d[3] = {l[0] - aPivot[0], l[1] - aPivot[1], l[2] - aPivot[2]};
            float r[3];
            gu::Rotate(aR, d, r);
            for (int k = 0; k < 3; ++k)
                l[k] = aPivot[k] + r[k];
            gu::Put(l + 3, gu::Normalize(gu::Mul(aR, gu::Get(l + 3))));
        }
    return aPose;
}
gu::Q AxisAngle(float x, float y, float z, double aAngle)
{
    const float l = std::sqrt(x * x + y * y + z * z);
    const float s = static_cast<float>(std::sin(0.5 * aAngle)) / l;
    return gu::Q{x * s, y * s, z * s, static_cast<float>(std::cos(0.5 * aAngle))};
}

// belly sign (+1 / -1) such that the Back clip's front points up
float CalibratedSign(const Pose& aBack)
{
    const Fk f = DoFk(aBack);
    float n[3];
    BellyNormal(Joint(f, kJHips), Joint(f, kJSpine), Joint(f, kJLUp), Joint(f, kJRUp), n);
    return n[2] >= 0.0f ? 1.0f : -1.0f;
}

// the four clips of a body: B / F from the table, L / R = the Back pose rolled onto the left / right side (truth by geometry:
// the left hip is the lower one for L)
struct ClipSet
{
    Pose local[kGuClipCount];
    float sign = 1.0f;
};
ClipSet MakeClipSet(bool aWoman)
{
    ClipSet s;
    s.local[0] = FromTable(aWoman ? kGetUpWomanBackFrame0 : kGetUpManBackFrame0);
    s.local[1] = FromTable(aWoman ? kGetUpWomanFrontFrame0 : kGetUpManFrontFrame0);
    s.sign = CalibratedSign(s.local[0]);
    const Fk b = DoFk(s.local[0]);
    float ax = Joint(b, kJHead)[0] - Joint(b, kJHips)[0], ay = Joint(b, kJHead)[1] - Joint(b, kJHips)[1];
    if (std::sqrt(ax * ax + ay * ay) < 0.05f)
    {
        ax = 0.0f;
        ay = 1.0f;
    }
    for (int side = 0; side < 2; ++side) // 0: roll +90, 1: roll -90
    {
        const Pose rolled = Rotate(s.local[0], AxisAngle(ax, ay, 0.0f, side == 0 ? kPi / 2 : -kPi / 2), Joint(b, kJHips));
        const Fk r = DoFk(rolled);
        const bool leftLower = Joint(r, kJLUp)[2] < Joint(r, kJRUp)[2];
        s.local[leftLower ? 2 : 3] = rolled;
    }
    return s;
}

GuClipInfo MakeClip(const Pose& aLocal, float aSign)
{
    GuClipInfo c;
    c.frame0Local = aLocal;
    const Fk f = DoFk(aLocal);
    c.frame0Model = f.p;
    c.standSign = aSign;
    LieAxes(Joint(f, kJHips), Joint(f, kJSpine), Joint(f, kJLUp), Joint(f, kJRUp), aSign, c.feat[0], c.feat[1]);
    c.frames = 151;
    c.fps = 30.0f;
    c.lengthS = 5.0f;
    c.handOffS = 2.3f;
    c.interruptS = 1.5f;
    c.ok = true;
    return c;
}

// ---------------------------------------------------------------------------------------------- RdLastPose
void TestLastPose()
{
    RdLastPose lp;
    static uint32_t dst[RdLastPose::kMaxBones];
    static uint8_t data[RdLastPose::kMaxBones][RdLastPose::kStride];
    const auto fill = [&](uint32_t n, float v) {
        for (uint32_t i = 0; i < n; ++i)
        {
            dst[i] = 100 + i;
            float rec[12];
            for (float& r : rec)
                r = v + float(i);
            std::memcpy(data[i], rec, sizeof rec);
        }
    };
    RdLastPose::Snapshot s;
    const int64_t hz = 1000;
    Check(!lp.Read(s, 0, hz, 0.15), "last pose: empty store reads false");
    fill(150, 1.0f);
    Check(lp.Write(0xAB, dst, data, 150, 1000, 619), "last pose: write stores");
    Check(lp.Read(s, 1050, hz, 0.15) && s.n == 150 && s.desc == 0xAB && s.descCount == 619 && s.dst[3] == 103 && s.rec[3][5] == 4.0f &&
              s.ticks == 1000,
          "last pose: read returns the copy (n, desc, dst, record floats)");
    uint32_t pn = 0;
    Check(lp.Peek(1050, hz, 0.15, pn) && pn == 150 && !lp.Peek(1200, hz, 0.15, pn), "last pose: peek follows the age cap");
    Check(!lp.Read(s, 1160, hz, 0.15) && lp.Read(s, 1149, hz, 0.15), "last pose: age cap 0.15 s");
    // smaller descriptor must not overwrite the full-body one
    fill(80, 50.0f);
    const bool wrote = lp.Write(0xCD, dst, data, 80, 1100);
    lp.Read(s, 1100, hz, 0.15);
    Check(!wrote && s.n == 150 && s.desc == 0xAB, "last pose: a smaller n never overwrites the full-body save");
    fill(160, 7.0f);
    Check(lp.Write(0xEF, dst, data, 160, 1200) && lp.Read(s, 1200, hz, 0.15) && s.n == 160 && s.desc == 0xEF,
          "last pose: a larger n replaces it");
    // freeze
    lp.Freeze(true);
    fill(160, 9.0f);
    const bool frozenWrite = lp.Write(0xEF, dst, data, 160, 1300);
    lp.Read(s, 1300, hz, 0.15);
    Check(!frozenWrite && s.ticks == 1200 && s.rec[0][0] == 7.0f, "last pose: frozen writes are ignored (the pre-release pose stays)");
    lp.Clear();
    Check(!lp.Frozen() && !lp.Read(s, 1300, hz, 0.15), "last pose: Clear empties the store and lifts the freeze");
    fill(40, 3.0f);
    Check(lp.Write(0x1, dst, data, 40, 2000) && lp.Read(s, 2000, hz, 0.15) && s.n == 40, "last pose: Clear also resets the largest-n rule");
    Check(!lp.Write(0x1, dst, data, 0, 2001) && !lp.Write(0x1, dst, data, 257, 2001), "last pose: invalid n refused");

    // seqlock: a hammering writer, every successful read is self-consistent (never a mix of two saves)
    RdLastPose hot;
    std::atomic<bool> stop{false};
    std::thread writer([&] {
        static uint32_t d[RdLastPose::kMaxBones];
        static uint8_t dat[RdLastPose::kMaxBones][RdLastPose::kStride];
        for (uint32_t i = 0; i < 150; ++i)
            d[i] = i;
        for (float v = 1.0f; !stop.load(); v += 1.0f)
        {
            float rec[12];
            for (float& r : rec)
                r = v;
            for (uint32_t i = 0; i < 150; ++i)
                std::memcpy(dat[i], rec, sizeof rec);
            hot.Write(1, d, dat, 150, static_cast<int64_t>(v));
            std::this_thread::sleep_for(std::chrono::microseconds(30)); // a write per frame, not a hammering loop
        }
    });
    int ok = 0, torn = 0, failed = 0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(300); ++i)
    {
        RdLastPose::Snapshot r;
        if (!hot.Read(r, 1 << 30, 1 << 30, 1e9))
        {
            failed += ok > 0 ? 1 : 0; // reads before the first write are no failures
            continue;
        }
        bool consistent = true;
        for (uint32_t k = 0; k < r.n && consistent; ++k)
            for (int j = 0; j < 12; ++j)
                consistent = consistent && r.rec[k][j] == r.rec[0][0];
        ok += consistent;
        torn += !consistent;
    }
    stop = true;
    writer.join();
    Check(torn == 0 && ok > 0, "last pose: seqlock - no torn snapshot is ever returned under a hammering writer",
          "ok=" + std::to_string(ok) + " failedReads=" + std::to_string(failed) + " torn=" + std::to_string(torn));
    // the bounded wait: a read that meets a write in flight waits (at most ~200 us) instead of failing, so Plan is not refused spuriously
    Check(failed * 100 <= ok + failed, "last pose: a read that meets a write in flight waits and succeeds (under 1 % failed reads)",
          "ok=" + std::to_string(ok) + " failedReads=" + std::to_string(failed));
    // Clear() serialized with a writer: after a Clear that returns while a writer hammers, the writer's stale publish is discarded
    {
        RdLastPose c;
        std::atomic<bool> go{true};
        std::thread w([&] {
            static uint32_t d[RdLastPose::kMaxBones];
            static uint8_t dat[RdLastPose::kMaxBones][RdLastPose::kStride];
            for (uint32_t i = 0; i < 150; ++i)
                d[i] = i;
            std::memset(dat, 0, sizeof dat);
            while (go.load())
                c.Write(1, d, dat, 150, 1);
        });
        bool ordered = true;
        for (int i = 0; i < 2000; ++i)
        {
            c.Clear();
            uint32_t n = 0;
            // right after Clear the largest-n rule is reset: a smaller write is accepted once, never refused by a stale bestN
            ordered = ordered && !c.Frozen();
            (void)n;
        }
        go = false;
        w.join();
        c.Clear();
        RdLastPose::Snapshot sn;
        Check(ordered && !c.Read(sn, 1, 1000, 1e9) && !c.Frozen(), "last pose: Clear() under a running writer ends empty and unfrozen");
    }
}

// ---------------------------------------------------------------------------------------------- classifier
struct Rng
{
    std::mt19937 g{12345};
    double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(g); }
};

void TestClassifier(const std::string& aDataDir)
{
    Rng rng;
    for (int body = 0; body < 2; ++body)
    {
        const bool woman = body == 0;
        const ClipSet set = MakeClipSet(woman);
        const char* bn = woman ? "woman" : "man";
        Check(!set.local[2].empty() && !set.local[3].empty(), std::string("classifier ") + bn + ": rolled L and R clips built (left / right hip lower)");
        GuClipInfo clips[kGuClipCount];
        for (int c = 0; c < kGuClipCount; ++c)
            clips[c] = MakeClip(set.local[c], set.sign);
        // frame-0 features have the expected signs
        Check(clips[0].feat[0] > 0.8f && clips[1].feat[0] < -0.8f && clips[2].feat[1] < -0.8f && clips[3].feat[1] > 0.8f,
              std::string("classifier ") + bn + ": clip frame-0 features B front+ F front- L lat- R lat+",
              F(clips[0].feat[0], clips[1].feat[0], clips[2].feat[1]) + " " + F(clips[3].feat[1]));
        float feat[kGuClipCount][2];
        for (int c = 0; c < kGuClipCount; ++c)
        {
            feat[c][0] = clips[c].feat[0];
            feat[c][1] = clips[c].feat[1];
        }
        int wrong = 0, total = 0;
        std::string firstBad;
        for (int c = 0; c < kGuClipCount; ++c)
        {
            for (int rep = 0; rep < 25; ++rep)
            {
                const Fk f = DoFk(set.local[c]);
                // random yaw 0..360 and a +-25 deg tilt (roll / pitch) about the hips, then the shifted joints
                gu::Q r = gu::YawQ(static_cast<float>(rng.U(0.0, 2.0 * kPi)));
                r = gu::Mul(r, AxisAngle(1, 0, 0, rng.U(-25.0, 25.0) * kRad));
                r = gu::Mul(r, AxisAngle(0, 1, 0, rng.U(-25.0, 25.0) * kRad));
                const float* h = Joint(f, kJHips);
                float pt[4][3];
                const int ids[4] = {kJHips, kJSpine, kJLUp, kJRUp};
                for (int k = 0; k < 4; ++k)
                {
                    const float* j = Joint(f, ids[k]);
                    const float d[3] = {j[0] - h[0], j[1] - h[1], j[2] - h[2]};
                    float o[3];
                    gu::Rotate(r, d, o);
                    pt[k][0] = o[0] + 3.0f;
                    pt[k][1] = o[1] - 2.0f;
                    pt[k][2] = o[2] + 0.2f;
                }
                float fz, lz;
                const int got = ClassifyLie(pt[0], pt[1], pt[2], pt[3], set.sign, feat, fz, lz);
                ++total;
                if (got != c)
                {
                    ++wrong;
                    if (firstBad.empty())
                        firstBad = std::string("clip ") + kGuClipNames[c] + " -> " + std::to_string(got) + " front/lat " + F(fz, lz);
                }
                const int got2 = ClassifyLie(pt[0], pt[1], pt[2], pt[3], set.sign, kGuCanonicalFeatures, fz, lz);
                if (got2 != c && got == c)
                {
                    ++wrong;
                    if (firstBad.empty())
                        firstBad = std::string("canonical features: clip ") + kGuClipNames[c] + " -> " + std::to_string(got2);
                }
            }
        }
        Check(wrong == 0, std::string("classifier ") + bn + ": B F L R recognised under random yaw and +-25 deg tilt (100 poses)",
              "wrong=" + std::to_string(wrong) + "/" + std::to_string(total) + " " + firstBad);
        // degenerate: all joints equal -> refused
        const float z[3] = {0, 0, 0};
        float fz, lz;
        Check(ClassifyLie(z, z, z, z, 1.0f, feat, fz, lz) < 0, std::string("classifier ") + bn + ": a degenerate pose is refused");
        // A's frame-0 fixtures (optional)
        std::ifstream in(aDataDir + "/getup_frame0_" + bn + ".json", std::ios::binary);
        if (!in)
        {
            std::printf("SKIP classifier %s: no fixture file %s/getup_frame0_%s.json\n", bn, aDataDir.c_str(), bn);
            continue;
        }
        std::stringstream ss;
        ss << in.rdbuf();
        utpp::json::Value doc;
        std::string err;
        if (!utpp::json::Parse(ss.str(), doc, err))
        {
            Check(false, std::string("fixture ") + bn + " parses", err);
            continue;
        }
        const utpp::json::Value* cl = doc.Find("clips");
        int fixtureOk = 0, fixtureBad = 0;
        float fsign = set.sign;
        const char* keys[kGuClipCount] = {"B", "F", "L", "R"};
        for (int pass = 0; pass < 2 && cl; ++pass)
            for (int c = 0; c < kGuClipCount; ++c)
            {
                const utpp::json::Value* clip = cl->Find(keys[c]);
                const utpp::json::Value* fm = clip ? clip->Find("frame0_model") : nullptr;
                if (!fm)
                {
                    if (pass == 0)
                        ++fixtureBad;
                    continue;
                }
                float pt[4][3];
                const char* nm[4] = {"Hips", "Spine", "LeftUpLeg", "RightUpLeg"};
                bool have = true;
                for (int k = 0; k < 4; ++k)
                {
                    const utpp::json::Value* v = fm->Find(nm[k]);
                    have = have && v && v->arr.size() == 3;
                    for (int j = 0; have && j < 3; ++j)
                        pt[k][j] = static_cast<float>(v->arr[size_t(j)].num);
                }
                if (!have)
                {
                    ++fixtureBad;
                    continue;
                }
                float ffz, flz;
                if (pass == 0 && c == 0)
                { // calibrate the sign on B (the clip on the back has its front up)
                    float n[3];
                    BellyNormal(pt[0], pt[1], pt[2], pt[3], n);
                    fsign = n[2] >= 0.0f ? 1.0f : -1.0f;
                }
                if (pass == 0)
                    continue;
                const int got = ClassifyLie(pt[0], pt[1], pt[2], pt[3], fsign, kGuCanonicalFeatures, ffz, flz);
                (got == c ? fixtureOk : fixtureBad)++;
            }
        Check(cl && fixtureOk == 4 && fixtureBad == 0, std::string("classifier ") + bn + ": A's frame-0 fixtures give B F L R",
              "ok=" + std::to_string(fixtureOk) + " bad=" + std::to_string(fixtureBad));
    }
}

// ---------------------------------------------------------------------------------------------- alignment + plan
struct Scene
{
    float anchor[3];
    double yaw;
    float entity[3];
    double entityYaw;
};

// A snapshot for the plan: the clip pose placed at (anchor, yaw), expressed in the entity frame (root children), plus noise
void MakeSnapshot(const Pose& aClip, const Scene& s, Rng& aRng, double aNoise, double aHandOffset, Pose& aOut)
{
    aOut = aClip;
    const float dYaw = static_cast<float>(s.yaw - s.entityYaw);
    const gu::Q qd = gu::YawQ(dYaw);
    const float off[3] = {s.anchor[0] - s.entity[0], s.anchor[1] - s.entity[1], s.anchor[2] - s.entity[2]};
    float loc[3];
    gu::RotZ(static_cast<float>(-s.entityYaw), off, loc);
    for (uint32_t i = kFirst; i < kN; ++i)
    {
        float* l = &aOut[size_t(i) * 7];
        if (gu::RootChild(Parents().data(), i, kFirst))
        {
            float r[3];
            gu::Rotate(qd, l, r);
            for (int k = 0; k < 3; ++k)
                l[k] = r[k] + loc[k];
            gu::Put(l + 3, gu::Normalize(gu::Mul(qd, gu::Get(l + 3))));
        }
        else
            for (int k = 0; k < 3; ++k)
                l[k] += aNoise > 0.0 ? static_cast<float>(aRng.U(-aNoise, aNoise)) : 0.0f;
    }
    if (aHandOffset > 0.0)
        for (const char* hn : {"LeftHand", "RightHand"})
            for (uint32_t i = 0; i < kN; ++i)
                if (std::string(kGetUpBoneNames[i]) == hn)
                {
                    const double a = aRng.U(0.0, 2.0 * kPi);
                    aOut[size_t(i) * 7 + 0] += static_cast<float>(aHandOffset * std::cos(a));
                    aOut[size_t(i) * 7 + 1] += static_cast<float>(aHandOffset * std::sin(a));
                }
}

// the entity follows the ragdoll body: put it at the predicted frame-0 Hips of the clip (anchor + RotZ(yaw) * hips0 XY), plus noise
void EntityAtHips(Scene& s, const Pose& aClip, Rng& aRng, double aNoise)
{
    const Fk f = DoFk(aClip);
    const float h[3] = {Joint(f, kJHips)[0], Joint(f, kJHips)[1], 0.0f};
    float w[3];
    gu::RotZ(static_cast<float>(s.yaw), h, w);
    s.entity[0] = s.anchor[0] + w[0] + (aNoise > 0.0 ? static_cast<float>(aRng.U(-aNoise, aNoise)) : 0.0f);
    s.entity[1] = s.anchor[1] + w[1] + (aNoise > 0.0 ? static_cast<float>(aRng.U(-aNoise, aNoise)) : 0.0f);
}

GuPlanOut RunPlan(const GuClipInfo* aClips, const Pose& aSnap, const Scene& s, bool aRef, float aSign, float aAirZ = std::nanf(""))
{
    static std::vector<uint8_t> have(kN, 1);
    GuPlanIn in;
    in.n = kN;
    in.firstBone = kFirst;
    in.parents = Parents().data();
    in.bones = &Bones();
    in.clips = aClips;
    in.snapLocal = aSnap.data();
    in.have = have.data();
    in.entityPos[0] = s.entity[0];
    in.entityPos[1] = s.entity[1];
    in.entityPos[2] = s.entity[2];
    in.entityYaw = static_cast<float>(s.entityYaw);
    in.haveRefSign = aRef;
    in.refSign = aSign;
    in.airGroundZ = aAirZ;
    return BuildGetUpPlan(in);
}

double YawDiffDeg(double a, double b)
{
    double d = std::fmod(a - b, 2.0 * kPi);
    if (d > kPi) d -= 2.0 * kPi;
    if (d < -kPi) d += 2.0 * kPi;
    return std::fabs(d) / kRad;
}

void TestAlignment()
{
    Rng rng;
    const ClipSet set = MakeClipSet(true);
    GuClipInfo clips[kGuClipCount];
    for (int c = 0; c < kGuClipCount; ++c)
        clips[c] = MakeClip(set.local[c], set.sign);

    // noise-free rigid placement: exact
    {
        Scene s{{1.0f, 2.0f, 0.0f}, 0.7, {1.1f, 1.9f, 0.4f}, -0.4};
        EntityAtHips(s, set.local[0], rng, 0.0);
        Pose snap;
        MakeSnapshot(set.local[0], s, rng, 0.0, 0.0, snap);
        const GuPlanOut o = RunPlan(clips, snap, s, true, set.sign);
        Check(o.code == 0 && YawDiffDeg(o.plan.yaw, s.yaw) < 0.1 && std::fabs(o.plan.anchor[0] - s.anchor[0]) < 0.005 &&
                  std::fabs(o.plan.anchor[1] - s.anchor[1]) < 0.005 && o.plan.rms < 0.01,
              "plan: a rigidly placed clip is recovered exactly", F(o.plan.yaw, o.plan.anchor[0], o.plan.anchor[1]) + " rms " + F(o.plan.rms));
        Check(std::fabs(o.plan.fwd[0] + std::sin(s.yaw)) < 1e-3 && std::fabs(o.plan.fwd[1] - std::cos(s.yaw)) < 1e-3,
              "plan: forward = (-sin yaw, cos yaw)", F(o.plan.fwd[0], o.plan.fwd[1]));
        const GuPlanOut a = RunPlan(clips, snap, s, true, set.sign, 0.25f);
        Check(a.code == 0 && a.plan.anchor[2] == 0.25f && a.groundSrc == 'a' && o.groundSrc == 'j' && std::isfinite(o.plan.anchor[2]),
              "plan: ground z from the air feed when given, else the lowest joint", F(a.plan.anchor[2], o.plan.anchor[2]));
    }
    // random yaw / translation with 2 cm class noise
    int bad = 0, n = 0;
    double worstYaw = 0, worstXY = 0, worstRms = 0;
    for (int rep = 0; rep < 60; ++rep)
    {
        const int c = rep % kGuClipCount;
        Scene s;
        s.anchor[0] = static_cast<float>(rng.U(-50, 50));
        s.anchor[1] = static_cast<float>(rng.U(-50, 50));
        s.anchor[2] = 0.0f;
        s.yaw = rng.U(-kPi, kPi);
        EntityAtHips(s, set.local[c], rng, 0.25);
        s.entity[2] = static_cast<float>(rng.U(0.0, 0.5));
        s.entityYaw = rng.U(-kPi, kPi);
        Pose snap;
        MakeSnapshot(set.local[c], s, rng, 0.006, 0.0, snap);
        const GuPlanOut o = RunPlan(clips, snap, s, true, set.sign);
        ++n;
        const double ey = YawDiffDeg(o.plan.yaw, s.yaw);
        const double exy = std::hypot(o.plan.anchor[0] - s.anchor[0], o.plan.anchor[1] - s.anchor[1]);
        worstYaw = std::fmax(worstYaw, ey);
        worstXY = std::fmax(worstXY, exy);
        worstRms = std::fmax(worstRms, o.plan.rms);
        if (o.code != c || ey > 2.0 || exy > 0.03 || o.plan.rms > 0.05)
            ++bad;
    }
    Check(bad == 0, "plan: 60 random placements with ~2 cm noise: right clip, yaw within 2 deg, XY within 3 cm, rms under 0.05",
          "bad=" + std::to_string(bad) + "/" + std::to_string(n) + " worst yaw " + F(worstYaw) + " xy " + F(worstXY) + " rms " + F(worstRms));
    // thrown-out hands (0.15 m): the hand weights are small, the result still holds the 2 deg / 3 cm bound
    bad = 0;
    worstYaw = worstXY = 0;
    for (int rep = 0; rep < 40; ++rep)
    {
        const int c = rep % 2; // B / F: the hands lie in the plane
        Scene s;
        s.anchor[0] = static_cast<float>(rng.U(-50, 50));
        s.anchor[1] = static_cast<float>(rng.U(-50, 50));
        s.anchor[2] = 0.0f;
        s.yaw = rng.U(-kPi, kPi);
        EntityAtHips(s, set.local[c], rng, 0.0);
        s.entity[2] = 0.0f;
        s.entityYaw = rng.U(-kPi, kPi);
        Pose snap;
        MakeSnapshot(set.local[c], s, rng, 0.0, 0.15, snap);
        const GuPlanOut o = RunPlan(clips, snap, s, true, set.sign);
        const double ey = YawDiffDeg(o.plan.yaw, s.yaw);
        const double exy = std::hypot(o.plan.anchor[0] - s.anchor[0], o.plan.anchor[1] - s.anchor[1]);
        worstYaw = std::fmax(worstYaw, ey);
        worstXY = std::fmax(worstXY, exy);
        if (o.code != c || ey > 2.0 || exy > 0.03)
            ++bad;
    }
    Check(bad == 0, "plan: thrown-out hands are down-weighted (weights 0.5, 0.15 m): yaw within 2 deg, XY within 3 cm",
          "bad=" + std::to_string(bad) + " worst yaw " + F(worstYaw) + " xy " + F(worstXY));
    // refusals
    {
        Scene s{{0.0f, 0.0f, 0.0f}, 0.3, {0.0f, 0.0f, 0.0f}, 0.0};
        Pose snap;
        MakeSnapshot(set.local[0], s, rng, 0.0, 0.0, snap);
        // a snapshot that does not match any clip: the Spine3 .. Head chain thrown 1.5 m away
        Pose bent = snap;
        for (uint32_t i = kFirst; i < kN; ++i)
        {
            const std::string nm = kGetUpBoneNames[i];
            if (nm == "Neck" || nm == "LeftLeg" || nm == "RightLeg")
            {
                bent[size_t(i) * 7 + 0] += 1.5f;
                bent[size_t(i) * 7 + 1] += 1.2f;
            }
        }
        const GuPlanOut m = RunPlan(clips, bent, s, true, set.sign);
        Check(m.code == kGuPlanPoseMismatch, "plan: a 0.7 m+ mismatch is refused (pose-mismatch)", std::string(m.plan.why) + " rms " + F(m.plan.rms));
        Scene farScene = s;
        farScene.entity[0] = 0.9f; // the body is 0.9 m from where the capsule is
        Pose snap2;
        MakeSnapshot(set.local[0], Scene{{0.0f, 0.0f, 0.0f}, 0.3, {0.0f, 0.0f, 0.0f}, 0.0}, rng, 0.0, 0.0, snap2);
        // entity moved: express the same world pose relative to the farScene entity by rebuilding the snapshot there
        Scene far2{{0.0f, 0.0f, 0.0f}, 0.3, {0.0f, 0.0f, 0.0f}, 0.0};
        EntityAtHips(far2, set.local[0], rng, 0.0);
        far2.entity[0] += 0.9f; // the body is 0.9 m from the capsule
        MakeSnapshot(set.local[0], far2, rng, 0.0, 0.0, snap2);
        const GuPlanOut a = RunPlan(clips, snap2, far2, true, set.sign);
        Check(a.code == kGuPlanAnchorFar, "plan: an anchor more than 0.6 m from the entity is refused (anchor-far)", std::string(a.plan.why));
        // missing classification bone
        std::vector<uint8_t> have(kN, 1);
        have[size_t(Bones().joint[kJLUp])] = 0;
        GuPlanIn in;
        in.n = kN;
        in.firstBone = kFirst;
        in.parents = Parents().data();
        in.bones = &Bones();
        in.clips = clips;
        in.snapLocal = snap.data();
        in.have = have.data();
        in.haveRefSign = true;
        in.refSign = set.sign;
        Check(BuildGetUpPlan(in).code == kGuPlanBonesMissing, "plan: a missing hip record is refused (bones-missing)");
        have[size_t(Bones().joint[kJLUp])] = 1;
        in.n = 0;
        Check(BuildGetUpPlan(in).code == kGuPlanUnavailable, "plan: bad input is refused (unavailable)");
    }
    // partly missing records: the filler is the chosen clip's frame 0 and is counted
    {
        Scene s{{2.0f, 1.0f, 0.0f}, -1.0, {2.0f, 1.0f, 0.0f}, 0.2};
        EntityAtHips(s, set.local[1], rng, 0.0);
        Pose snap;
        MakeSnapshot(set.local[1], s, rng, 0.0, 0.0, snap);
        std::vector<uint8_t> have(kN, 1);
        have[size_t(Bones().joint[kJLHand])] = 0; // hand and below
        GuPlanIn in;
        in.n = kN;
        in.firstBone = kFirst;
        in.parents = Parents().data();
        in.bones = &Bones();
        in.clips = clips;
        in.snapLocal = snap.data();
        in.have = have.data();
        in.entityPos[0] = s.entity[0];
        in.entityPos[1] = s.entity[1];
        in.entityYaw = 0.2f;
        in.haveRefSign = true;
        in.refSign = set.sign;
        const GuPlanOut o = BuildGetUpPlan(in);
        Check(o.code == 1 && o.missing == 1, "plan: a bone without a record is filled and counted", "code " + std::to_string(o.code) + " missing " + std::to_string(o.missing));
    }
}

// ---------------------------------------------------------------------------------------------- blend, clamp, anchoring
void TestPose()
{
    Rng rng;
    const ClipSet set = MakeClipSet(true);
    GuClipInfo clips[kGuClipCount];
    for (int c = 0; c < kGuClipCount; ++c)
        clips[c] = MakeClip(set.local[c], set.sign);
    Scene s{{4.0f, -3.0f, 0.1f}, 0.9, {4.1f, -2.9f, 0.5f}, 0.4};
    EntityAtHips(s, set.local[1], rng, 0.0);
    Pose snap;
    MakeSnapshot(set.local[1], s, rng, 0.0, 0.0, snap);
    const GuPlanOut plan = RunPlan(clips, snap, s, true, set.sign);
    const GuShape shape{kN, kFirst, Parents().data()};
    const Pose clip = set.local[plan.code == 1 ? 0 : 1]; // another pose than the snapshot
    GuAnchor anchor;
    std::memcpy(anchor.pos, plan.plan.anchor, sizeof anchor.pos);
    anchor.yaw = plan.plan.yaw;
    GuFrame planFrame;
    std::memcpy(planFrame.cap, s.entity, sizeof planFrame.cap);
    planFrame.yaw = static_cast<float>(s.entityYaw);

    // t = 0: the output is the snapshot
    Pose out(size_t(kN) * 7);
    ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, planFrame, 0.0f, out.data());
    float worst = 0;
    for (size_t i = size_t(kFirst) * 7; i < out.size(); ++i)
    {
        if (i % 7 >= 3) // quaternion sign is free
            continue;
        worst = std::fmax(worst, std::fabs(out[i] - plan.snapLocal[i]));
    }
    float qworst = 0;
    for (uint32_t b = kFirst; b < kN; ++b)
    {
        const float* o = &out[size_t(b) * 7 + 3];
        const float* r = &plan.snapLocal[size_t(b) * 7 + 3];
        const float d = std::fabs(o[0] * r[0] + o[1] * r[1] + o[2] * r[2] + o[3] * r[3]);
        qworst = std::fmax(qworst, 1.0f - d);
    }
    Check(worst < 1e-5f && qworst < 1e-5f, "blend: at t = 0 the output equals the snapshot (1e-5)", F(worst, qworst));
    // w = 1: the clip (non-root bones identical, root children warped)
    ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, planFrame, 1.0f, out.data());
    float diff = 0;
    for (uint32_t b = kFirst; b < kN; ++b)
        if (!gu::RootChild(Parents().data(), b, kFirst))
            for (int k = 0; k < 3; ++k)
                diff = std::fmax(diff, std::fabs(out[size_t(b) * 7 + k] - clip[size_t(b) * 7 + k]));
    Check(diff < 1e-6f, "blend: at t >= blend time the non-root bones are the clip's", F(diff));
    // root children = RotZ(dYaw) p + RotZ(anchor - capsule, -yaw)
    {
        const uint32_t hips = static_cast<uint32_t>(Bones().joint[kJHips]);
        const float dYaw = gu::WrapPi(anchor.yaw - planFrame.yaw);
        const float off[3] = {anchor.pos[0] - planFrame.cap[0], anchor.pos[1] - planFrame.cap[1], anchor.pos[2] - planFrame.cap[2]};
        float l1[3], l2[3];
        gu::RotZ(-planFrame.yaw, off, l1);
        gu::RotZ(dYaw, &clip[size_t(hips) * 7], l2);
        const float e = std::fabs(out[size_t(hips) * 7] - (l1[0] + l2[0])) + std::fabs(out[size_t(hips) * 7 + 1] - (l1[1] + l2[1])) +
                        std::fabs(out[size_t(hips) * 7 + 2] - (l1[2] + l2[2]));
        Check(e < 1e-5f, "blend: the clip's Hips = RotZ(anchor yaw - entity yaw) p + RotZ(anchor - capsule, -entity yaw)", F(e));
    }
    // smoothstep monotone, finite, shortest arc
    {
        bool mono = true;
        float prev = -1;
        for (int i = 0; i <= 100; ++i)
        {
            const float v = gu::Smooth(float(i) / 100.0f);
            mono = mono && v >= prev;
            prev = v;
        }
        Check(mono && gu::Smooth(0) == 0.0f && gu::Smooth(1) == 1.0f && gu::Smooth(2.0f) == 1.0f && gu::Smooth(-1.0f) == 0.0f,
              "blend: smoothstep is monotone and clamped");
        const gu::Q a{0, 0, 0, 1}, b{0, 0, 0.1f, -0.99498744f}; // b is -(rotation by ~11.5 deg): the same rotation as +q
        const gu::Q m = gu::Nlerp(a, b, 0.5f);
        Check(m.w > 0.99f && std::fabs(m.z) < 0.06f, "blend: quaternions take the shortest arc", F(m.z, m.w));
        bool finite = true;
        for (int i = 0; i <= 10; ++i)
        {
            ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, planFrame, float(i) / 10.0f, out.data());
            for (float v : out)
                finite = finite && std::isfinite(v);
        }
        Check(finite, "blend: the output is finite for every weight");
    }
    // world anchoring: the capsule jumps (+0.17 z, +0.3 xy, yaw turn) in the middle of the blend, the world Hips stay put
    {
        const uint32_t hips = static_cast<uint32_t>(Bones().joint[kJHips]);
        double worstMove = 0;
        for (float w : {0.0f, 0.4f, 1.0f})
        {
            GuFrame live = planFrame;
            const auto worldHips = [&](const GuFrame& f, const GuFrame& use) {
                Pose o(size_t(kN) * 7);
                const GuFrame sel = SelectFrame(planFrame, use, true);
                ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, sel, w, o.data());
                float wp[3];
                gu::RotZ(f.yaw, &o[size_t(hips) * 7], wp);
                return std::array<double, 3>{f.cap[0] + wp[0], f.cap[1] + wp[1], f.cap[2] + wp[2]};
            };
            const auto before = worldHips(live, live);
            live.cap[0] += 0.3f;
            live.cap[1] -= 0.1f;
            live.cap[2] += 0.17f;
            live.yaw += 0.35f;
            const auto after = worldHips(live, live);
            worstMove = std::fmax(worstMove, std::sqrt((after[0] - before[0]) * (after[0] - before[0]) + (after[1] - before[1]) * (after[1] - before[1]) +
                                                        (after[2] - before[2]) * (after[2] - before[2])));
        }
        Check(worstMove < 1e-4, "anchoring: a capsule that moves 0.17 m up and 0.3 m sideways does not move the world Hips (1e-4)", F(worstMove));
    }
    // floor clamp: a knee 8 cm under the floor is lifted at the rate limit, never beyond 0.10 m
    {
        ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, planFrame, 1.0f, out.data());
        // ground plane placed so that the deepest clamp joint is exactly 8 cm under it (knee radius included)
        float lowest = 1e9f;
        {
            const Fk f = DoFk(out);
            for (int j = 0; j < kGuClampCount; ++j)
                if (Bones().clamp[j] >= 0)
                    lowest = std::fmin(lowest, f.p[size_t(Bones().clamp[j]) * 3 + 2] - kGuClampRadius[j]);
        }
        const float zg = lowest + 0.08f;
        const float d0 = FloorDeficit(shape, out.data(), Bones(), zg);
        Check(std::fabs(d0 - 0.08f) < 1e-4f, "clamp: the deficit of the deepest joint is measured (8 cm)", F(d0));
        GuLift lift;
        const float dt = 1.0f / 60.0f;
        float maxStep = 0, prev = 0, last = 0;
        Pose lifted = out;
        for (int i = 0; i < 40; ++i)
        {
            const float def = FloorDeficit(shape, out.data(), Bones(), zg);
            last = lift.Step(def, dt);
            maxStep = std::fmax(maxStep, last - prev);
            prev = last;
        }
        AddLift(shape, last, lifted.data());
        const float after = FloorDeficit(shape, lifted.data(), Bones(), zg);
        Check(maxStep <= kGuLiftRateMs * dt + 1e-6f && std::fabs(last - 0.08f) < 1e-4f && after < 0.005f,
              "clamp: an 8 cm deficit is lifted at <= 0.5 m/s per step and ends on the floor", F(maxStep, last, after));
        GuLift big;
        float bl = 0;
        for (int i = 0; i < 120; ++i)
            bl = big.Step(0.30f, dt);
        Check(bl <= kGuLiftMaxM + 1e-6f && bl > kGuLiftMaxM - 1e-4f, "clamp: the lift never exceeds 0.10 m", F(bl));
        GuLift none;
        Check(none.Step(0.0f, dt) == 0.0f && none.Step(-1.0f, dt) == 0.0f && none.Step(std::nanf(""), dt) == 0.0f, "clamp: no deficit, no lift (also NaN)");
    }
    // per-limb floor correction: a hand / foot 8 cm under the floor inside the blend is raised on its own chain
    {
        ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, planFrame, 0.6f, out.data());
        const Pose mixed = out;
        const Fk f0 = DoFk(mixed);
        const int ends[4] = {7, 8, 11, 12}; // clamp indices: LeftHand RightHand LeftFoot RightFoot
        int good = 0, never = 0;
        std::string det;
        for (int e : ends)
        {
            const int32_t bi = Bones().clamp[e];
            const float zg = f0.p[size_t(bi) * 3 + 2] - kGuClampRadius[e] + 0.08f; // this joint 8 cm under the floor
            Pose p = mixed;
            const int acted = FloorCorrect(shape, p.data(), Bones(), zg);
            const Fk f1 = DoFk(p);
            const float after = zg + kGuClampRadius[e] - f1.p[size_t(bi) * 3 + 2];
            good += (acted > 0 && after <= 0.011f) ? 1 : 0;
            never += after <= 0.081f ? 1 : 0;
            det += F(after) + " ; ";
        }
        // the hands reach the floor on their own chain; a foot on a straight shin would need a big swing for a few mm: its swing is capped and the
        // rest is left to the whole-body lift (checked below), so it must at least never get worse
        Check(good >= 2 && never == 4, "limb correction: a hand 8 cm under the floor ends within 1 cm of radius + floor, no hand / foot gets worse (swing cap on the feet)", det);
        // deepest joint 8 cm under, whatever it is: limbs first, then the lift: every clamp joint above radius - 1 cm, bounded steps
        float zgDeep;
        {
            float lo = 1e9f;
            for (int j = 0; j < kGuClampCount; ++j)
                if (Bones().clamp[j] >= 0)
                    lo = std::fmin(lo, f0.p[size_t(Bones().clamp[j]) * 3 + 2] - kGuClampRadius[j]);
            zgDeep = lo + 0.08f;
        }
        GuLift lift;
        std::vector<float> prevPos, prevPosRaw;
        float maxStepFix = 0, maxStepRaw = 0, worstBelow = 0;
        const float dtf = 1.0f / 60.0f;
        for (int i = 0; i <= 36; ++i)
        {
            const float w = gu::Smooth(float(i) / 36.0f);
            Pose p(size_t(kN) * 7);
            ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, planFrame, w, p.data());
            const Pose raw = p;
            if (FloorDeficit(shape, p.data(), Bones(), zgDeep) > 0.002f)
                FloorCorrect(shape, p.data(), Bones(), zgDeep);
            const float lf = lift.Step(FloorDeficit(shape, p.data(), Bones(), zgDeep), dtf);
            AddLift(shape, lf, p.data());
            const Fk fk = DoFk(p), fr = DoFk(raw);
            if (i > 3)
                for (int j = 0; j < kGuClampCount; ++j)
                    if (Bones().clamp[j] >= 0)
                        worstBelow = std::fmax(worstBelow, zgDeep + kGuClampRadius[j] - fk.p[size_t(Bones().clamp[j]) * 3 + 2]);
            if (!prevPos.empty())
                for (uint32_t b = kFirst; b < kN; ++b)
                {
                    maxStepFix = std::fmax(maxStepFix, Dist(&fk.p[size_t(b) * 3], &prevPos[size_t(b) * 3]));
                    maxStepRaw = std::fmax(maxStepRaw, Dist(&fr.p[size_t(b) * 3], &prevPosRaw[size_t(b) * 3]));
                }
            prevPos = fk.p;
            prevPosRaw = fr.p;
        }
        Check(worstBelow <= 0.011f, "limb correction + lift: from frame 4 on no clamp joint is more than 1 cm below radius + floor (8 cm deficit in the blend)", F(worstBelow));
        Check(maxStepFix <= maxStepRaw + 0.06f, "limb correction + lift: the per-frame joint step stays bounded (no pop; at most the blend's own step + 6 cm)",
              F(maxStepFix, maxStepRaw));
    }
    // release continuity (in-game r13c): the whole-body lift rises fast and falls slowly, so a residual deficit that disappears at once (a
    // limb that leaves the floor, a publish gap) does not drop the body; planted toes do not slide while the lift releases. Driver order as in
    // GetUpApplyLocked: FloorDeficit -> FloorCorrect (> 2 mm) -> residual -> GuLift::Step -> AddLift.
    {
        ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, planFrame, 1.0f, out.data());
        const Pose base = out;
        const Fk f0 = DoFk(base);
        float lowAny = 1e9f;
        for (int j = 0; j < kGuClampCount; ++j)
            if (Bones().clamp[j] >= 0)
                lowAny = std::fmin(lowAny, f0.p[size_t(Bones().clamp[j]) * 3 + 2] - kGuClampRadius[j]);
        const int32_t toe[2] = {Bones().clamp[13], Bones().clamp[14]};
        struct Rec
        {
            float hips = 0, below = 0, lift = 0, toeXY[2][2] = {{0, 0}, {0, 0}};
        };
        const auto frame = [&](GuLift& aLift, float aZg, float aDt) {
            Pose p = base;
            const float raw = FloorDeficit(shape, p.data(), Bones(), aZg);
            if (raw > 0.002f)
                FloorCorrect(shape, p.data(), Bones(), aZg);
            const float res = FloorDeficit(shape, p.data(), Bones(), aZg);
            Rec r;
            r.lift = aLift.Step(res, aDt);
            AddLift(shape, r.lift, p.data());
            const Fk fk = DoFk(p);
            r.hips = Joint(fk, kJHips)[2];
            for (int j = 0; j < kGuClampCount; ++j)
                if (Bones().clamp[j] >= 0)
                    r.below = std::fmax(r.below, aZg + kGuClampRadius[j] - fk.p[size_t(Bones().clamp[j]) * 3 + 2]);
            for (int t = 0; t < 2; ++t)
                if (toe[t] >= 0)
                {
                    r.toeXY[t][0] = fk.p[size_t(toe[t]) * 3];
                    r.toeXY[t][1] = fk.p[size_t(toe[t]) * 3 + 1];
                }
            return r;
        };
        const float dt = 1.0f / 60.0f;
        const float noFloor = lowAny - 0.5f;
        // (a) the residual deficit itself (what the lift follows): 3.6 cm (the r13c size) appears at once and vanishes at once. The rise keeps the
        // floor within one frame; the release is <= 1.5 cm per frame and a 0.1 s publish gap (the cap of dtApply) releases at most 2.5 cm
        {
            GuLift lf;
            float maxRelease = 0, worstUnder = 0, prev = 0;
            for (int i = 0; i < 70; ++i)
            {
                const float target = (i >= 5 && i < 35) ? 0.036f : 0.0f;
                const float v = lf.Step(target, dt);
                worstUnder = std::fmax(worstUnder, target - v);
                maxRelease = std::fmax(maxRelease, prev - v);
                prev = v;
            }
            GuLift gap;
            for (int i = 0; i < 20; ++i)
                (void)gap.Step(0.036f, dt);
            const float held = gap.lift, afterGap = gap.Step(0.0f, 0.1f);
            std::printf("INFO lift state: max release step %.4f under target %.4f held %.4f gap drop %.4f\n", static_cast<double>(maxRelease),
                        static_cast<double>(worstUnder), static_cast<double>(held), static_cast<double>(held - afterGap));
            Check(worstUnder <= 0.01f, "release: the lift follows a 3.6 cm deficit that appears at once to within 1 cm (the floor is kept)", F(worstUnder));
            Check(maxRelease <= 0.015f && maxRelease > 0.0f, "release: a deficit that vanishes at once releases the lift at most 1.5 cm per frame (no 3.6 cm drop)", F(maxRelease));
            Check(held - afterGap <= 0.025f + 1e-5f && afterGap > 0.0f, "release: a 0.1 s publish gap releases at most 2.5 cm of the lift", F(held, afterGap));
        }
        // (b) the whole path (limbs first, then the lift) on the clip pose with a 10 cm deficit that appears and vanishes at once: the floor holds
        // while it is there, the hips fall at most 1.5 cm per frame in the release, and the planted toes do not move while the lift releases
        {
            const float zgHi = lowAny + 0.10f;
            GuLift lift;
            const Rec r0 = frame(lift, noFloor, dt);
            float worstBelow = 0, maxRelease = 0, drift = 0, liftAtRelease = 0, prevHips = r0.hips;
            for (int i = 0; i < 70; ++i)
            {
                const bool hi = i >= 5 && i < 35;
                const Rec r = frame(lift, hi ? zgHi : noFloor, dt);
                if (hi && i >= 8)
                    worstBelow = std::fmax(worstBelow, r.below);
                if (i == 35)
                    liftAtRelease = r.lift;
                if (i >= 35)
                {
                    maxRelease = std::fmax(maxRelease, prevHips - r.hips);
                    if (r.lift > 0.0f)
                        for (int t = 0; t < 2; ++t)
                            drift = std::fmax(drift, std::hypot(r.toeXY[t][0] - r0.toeXY[t][0], r.toeXY[t][1] - r0.toeXY[t][1]));
                }
                prevHips = r.hips;
            }
            std::printf("INFO lift path: lift at release %.4f below %.4f hips release step %.4f toe drift %.4f\n", static_cast<double>(liftAtRelease),
                        static_cast<double>(worstBelow), static_cast<double>(maxRelease), static_cast<double>(drift));
            Check(liftAtRelease >= 0.02f, "release: the 10 cm deficit leaves a residual of at least 2 cm for the lift after the limbs (the scenario is meaningful)", F(liftAtRelease));
            Check(worstBelow <= 0.011f, "release: while the 10 cm deficit is there no clamp joint is more than 1 cm below radius + floor", F(worstBelow));
            Check(maxRelease <= 0.015f, "release: the hips fall at most 1.5 cm per frame when the whole deficit vanishes", F(maxRelease));
            Check(drift <= 0.01f, "release: a planted toe moves at most 1 cm (world) while the lift releases", F(drift));
        }
        // (c) a deficit that fades out smoothly (the clip rises away from the floor) through the limb correction: bounded toe step per frame, hips <= 1.5 cm
        {
            GuLift lift;
            float maxToe = 0, maxHips = 0;
            Rec prev = frame(lift, lowAny + 0.05f, dt);
            for (int i = 1; i <= 40; ++i)
            {
                const float def = 0.05f * (1.0f - float(i) / 40.0f);
                const Rec r = frame(lift, lowAny + def, dt);
                for (int t = 0; t < 2; ++t)
                    maxToe = std::fmax(maxToe, std::hypot(r.toeXY[t][0] - prev.toeXY[t][0], r.toeXY[t][1] - prev.toeXY[t][1]));
                maxHips = std::fmax(maxHips, std::fabs(r.hips - prev.hips));
                prev = r;
            }
            Check(maxToe <= 0.01f && maxHips <= 0.015f, "release: a deficit that fades out through limb correction + lift moves a toe at most 1 cm and the hips at most 1.5 cm per frame",
                  F(maxToe, maxHips));
        }
    }
    // adaptive blend: a snapshot far from the clip's frame 0 gets a longer blend and the head never sweeps more than 2.5 cm per frame
    {
        Scene sc{{0.0f, 0.0f, 0.0f}, 0.4, {0.0f, 0.0f, 0.0f}, 0.0};
        const Pose odd = MorphPose(set.local[0], set.local[1], 0.12f); // a body half way between lying on the back and on the front
        EntityAtHips(sc, set.local[0], rng, 0.0);
        Pose snapOdd;
        MakeSnapshot(odd, sc, rng, 0.0, 0.0, snapOdd);
        const GuPlanOut pl = RunPlan(clips, snapOdd, sc, true, set.sign);
        std::printf("INFO adaptive blend scenario: code %d rms %.3f blend %.3f\n", pl.code, static_cast<double>(pl.plan.rms), static_cast<double>(pl.plan.blendInS));
        Check(pl.code >= 0 && pl.plan.blendInS > kGuBlendInS + 0.1f && pl.plan.blendInS <= kGuBlendMaxS + 1e-6f,
              "adaptive blend: a snapshot that differs from frame 0 (3D, heights too) gets a longer blend than 0.3 s, at most 0.6 s", F(pl.plan.rms, pl.plan.blendInS));
        if (pl.code >= 0)
        {
            const Pose& tgt = set.local[pl.code];
            GuAnchor an;
            std::memcpy(an.pos, pl.plan.anchor, sizeof an.pos);
            an.yaw = pl.plan.yaw;
            GuFrame fr0;
            std::memcpy(fr0.cap, sc.entity, sizeof fr0.cap);
            fr0.yaw = static_cast<float>(sc.entityYaw);
            const auto headSweep = [&](float aBlendS) {
                float mx = 0;
                std::vector<float> prev;
                const int steps = static_cast<int>(std::ceil(aBlendS * 60.0f));
                for (int i = 0; i <= steps; ++i)
                {
                    Pose p(size_t(kN) * 7);
                    ComposeBlend(shape, tgt.data(), pl.snapLocal.data(), pl.snapRootWorld.data(), an, fr0, gu::Smooth(float(i) / float(steps)), p.data());
                    const Fk fk = DoFk(p);
                    const float* h = Joint(fk, kJHead);
                    if (!prev.empty())
                        mx = std::fmax(mx, Dist(h, prev.data()));
                    prev.assign(h, h + 3);
                }
                return mx;
            };
            const float adaptive = headSweep(pl.plan.blendInS), fixedShort = headSweep(kGuBlendInS);
            std::printf("INFO head sweep per frame: adaptive %.4f m, fixed 0.30 s %.4f m\n", static_cast<double>(adaptive), static_cast<double>(fixedShort));
            Check(adaptive <= 0.025f && fixedShort > adaptive, "adaptive blend: the head moves at most 2.5 cm per frame during the blend (a fixed 0.3 s blend moves it faster)",
                  F(adaptive, fixedShort));
        }
    }
}

// ---------------------------------------------------------------------------------------------- phases
GuPlan MakePlan()
{
    GuPlan p;
    p.clip = 0;
    p.ok = true;
    p.handOffS = 2.3f;
    p.interruptS = 1.5f;
    p.lengthS = 5.0f;
    p.rms = 0.02f;
    return p;
}

void TestPhases()
{
    const float dt = 1.0f / 60.0f;
    GetUpPlayer pl;
    Check(!pl.Active() && !pl.Start(GuPlan{}, 0.0), "phases: idle; a plan that is not ok is refused");
    GuPlan bad = MakePlan();
    bad.clip = 7;
    Check(!pl.Start(bad, 0.0), "phases: a bad clip index is refused");
    Check(pl.Start(MakePlan(), 10.0) && pl.Phase() == GuPhase::Blend && !pl.Start(MakePlan(), 10.0), "phases: Start -> Blend, a second Start is refused");
    // run on feed time to the clip end
    double wall = 10.0;
    float t = 0;
    bool sawPlay = false, sawHandOff = false, ended = false;
    float blendDoneAt = -1, handOffAt = -1, endedAt = -1;
    std::string reason;
    float wAt0 = pl.BlendW();
    float wMid = -1;
    for (int i = 0; i < 1000 && !ended; ++i)
    {
        wall += dt;
        t += dt;
        const auto r = pl.Advance(dt, false, 0.0f, wall);
        if (std::fabs(t - 0.15f) < dt * 0.6f)
            wMid = pl.BlendW();
        if (r.blendDone)
        {
            sawPlay = true;
            blendDoneAt = t;
        }
        if (r.handOffStarted)
        {
            sawHandOff = true;
            handOffAt = t;
        }
        if (r.ended)
        {
            ended = true;
            endedAt = t;
            reason = r.reason;
        }
    }
    Check(sawPlay && std::fabs(blendDoneAt - kGuBlendInS) < 2 * dt, "phases: Blend -> Play at the blend time", F(blendDoneAt));
    Check(sawHandOff && std::fabs(handOffAt - (2.3f + (1.0f - kGuBlendClipRate) * kGuBlendInS)) < 2 * dt, "phases: Play -> HandOff at the HandOff marker (clip time; the clip runs at half rate during the blend)", F(handOffAt));
    Check(ended && reason == "done" && std::fabs(endedAt - (2.3f + (1.0f - kGuBlendClipRate) * kGuBlendInS + kGuBlendOutS)) < 3 * dt && !pl.Active(), "phases: HandOff -> Idle after the blend-out",
          F(endedAt) + " " + reason);
    Check(wAt0 == 0.0f && std::fabs(wMid - 0.5f) < 0.02f, "phases: blend weight is smoothstep (0 at start, 0.5 at half)", F(wAt0, wMid));
    // input after Interrupt hands off early; before it, it does not
    {
        GetUpPlayer p;
        p.Start(MakePlan(), 0.0);
        double w = 0;
        float tt = 0;
        bool early = false, tooEarly = false;
        const char* why = "";
        for (int i = 0; i < 600 && !early; ++i)
        {
            tt += dt;
            w += dt;
            const auto r = p.Advance(dt, true, 0.0f, w);
            if (r.handOffStarted)
            {
                early = true;
                why = r.reason;
                tooEarly = tt < 1.5f + (1.0f - kGuBlendClipRate) * kGuBlendInS - 2 * dt;
            }
        }
        Check(early && !tooEarly && std::string(why) == "input" && std::fabs(tt - (1.5f + (1.0f - kGuBlendClipRate) * kGuBlendInS)) < 2 * dt,
              "phases: movement input hands off from the Interrupt marker on (not before)", F(tt) + " " + why);
    }
    // off the anchor before Interrupt (the PSM moved the capsule): hand-off "input-interrupt"; not when never armed
    {
        GetUpPlayer p;
        p.Start(MakePlan(), 0.0);
        double w = 0;
        bool hand = false;
        const char* why = "";
        for (int i = 0; i < 40; ++i) // 0.66 s with the capsule ON the anchor (arms the check), then 0.4 m away
            p.Advance(dt, false, 0.01f, w += dt);
        for (int i = 0; i < 5 && !hand; ++i)
        {
            const auto r = p.Advance(dt, false, 0.4f, w += dt);
            if (r.handOffStarted)
            {
                hand = true;
                why = r.reason;
            }
        }
        Check(hand && std::string(why) == "input-interrupt" && p.Phase() == GuPhase::HandOff, "phases: before Interrupt a capsule 0.4 m off the anchor ends the get-up (input-interrupt)", why);
        GetUpPlayer q;
        q.Start(MakePlan(), 0.0);
        w = 0;
        bool falseHand = false;
        for (int i = 0; i < 60; ++i) // the capsule is still on its way to the teleport target: 0.5 m off, never seen near the anchor
            falseHand = falseHand || q.Advance(dt, false, 0.5f, w += dt).handOffStarted;
        Check(!falseHand, "phases: no false off-anchor hand-off before the capsule was ever near the anchor");
    }
    // pause freezes the clock; watchdogs
    {
        GetUpPlayer p;
        p.Start(MakePlan(), 0.0);
        double w = 0;
        for (int i = 0; i < 30; ++i)
            p.Advance(dt, false, 0.0f, w += dt);
        const float t1 = p.ClipTime();
        for (int i = 0; i < 120; ++i)
            p.Advance(dt, false, 0.0f, w += dt, true);
        Check(std::fabs(p.ClipTime() - t1) < 1e-6f && p.Active(), "phases: the menu / photo pause freezes the clip clock", F(t1, p.ClipTime()));
        GetUpPlayer q;
        q.Start(MakePlan(), 0.0);
        bool wdEnded = false;
        std::string why;
        w = 0;
        for (int i = 0; i < 60 && !wdEnded; ++i)
        {
            const auto r = q.Advance(0.0f, false, 0.0f, w += 0.5); // wall clock runs, no feed time
            wdEnded = r.ended;
            why = r.reason ? r.reason : "";
        }
        Check(wdEnded && why == "watchdog-wall" && !q.Active(), "phases: the wall watchdog ends a run without feed time (20 s)", why);
        // time dilation: 12 s of wall time for 0.4 s of feed time does not end the run (the feed-time watchdog is the real limit)
        GetUpPlayer dil;
        dil.Start(MakePlan(), 0.0);
        w = 0;
        for (int i = 0; i < 24; ++i)
            dil.Advance(0.0167f, false, 0.0f, w += 0.5);
        Check(dil.Active(), "phases: 12 s of wall time under time dilation does not abort the get-up", F(dil.ClipTime()));
        GetUpPlayer z;
        GuPlan pl2 = MakePlan();
        pl2.handOffS = 9.0f;
        pl2.interruptS = 8.0f; // never reached: the feed watchdog (length + 1 s) ends it
        pl2.lengthS = 3.0f;
        z.Start(pl2, 0.0);
        wdEnded = false;
        w = 0;
        for (int i = 0; i < 400 && !wdEnded; ++i)
        {
            const auto r = z.Advance(0.1f, false, 0.0f, w += 0.001);
            wdEnded = r.ended;
            why = r.reason ? r.reason : "";
        }
        Check(wdEnded && why == "watchdog-feed", "phases: the feed watchdog (clip length + 1 s) ends a run", why);
        GetUpPlayer a;
        a.Start(MakePlan(), 0.0);
        a.Advance(dt, false, 0.0f, 0.01);
        a.Abort("test");
        Check(!a.Active() && std::string(a.EndReason()) == "test" && a.HandOffLeft() == 0.0f, "phases: Abort ends at once from any phase");
        GetUpPlayer pw;
        pw.Start(MakePlan(), 0.0);
        Check(pw.Watchdog(21.0) && !pw.Active(), "phases: the render-side wall watchdog also ends a run");
    }
}

// ---------------------------------------------------------------------------------------------- loader
std::string Fix(double v)
{
    char b[48];
    std::snprintf(b, sizeof b, "%.6f", v);
    std::string s = b;
    return s == "-0.000000" ? "0.000000" : s;
}

// The Back pose stood up: long axis (Hips -> Head) to +Z, belly to +Y (rotation about the Hips)
Pose MakeStandPose(const ClipSet& aSet)
{
    const Pose& lie = aSet.local[0];
    const Fk f = DoFk(lie);
    float u[3], b[3], n[3];
    for (int k = 0; k < 3; ++k)
        u[k] = Joint(f, kJHead)[k] - Joint(f, kJHips)[k];
    BellyNormal(Joint(f, kJHips), Joint(f, kJSpine), Joint(f, kJLUp), Joint(f, kJRUp), n);
    const auto norm = [](float* v) {
        const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        for (int k = 0; k < 3; ++k)
            v[k] /= l;
    };
    norm(u);
    const float d = n[0] * aSet.sign * u[0] + n[1] * aSet.sign * u[1] + n[2] * aSet.sign * u[2];
    for (int k = 0; k < 3; ++k)
        b[k] = n[k] * aSet.sign - d * u[k];
    norm(b);
    const float s[3] = {u[1] * b[2] - u[2] * b[1], u[2] * b[0] - u[0] * b[2], u[0] * b[1] - u[1] * b[0]};
    const float* cols[3] = {u, b, s};
    const float tcols[3][3] = {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
    double R[3][3] = {};
    for (int k = 0; k < 3; ++k)
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R[i][j] += double(tcols[k][i]) * double(cols[k][j]);
    const double tr = R[0][0] + R[1][1] + R[2][2];
    gu::Q q;
    if (tr > 0.0)
    {
        const double sq = std::sqrt(tr + 1.0) * 2.0;
        q = gu::Q{float((R[2][1] - R[1][2]) / sq), float((R[0][2] - R[2][0]) / sq), float((R[1][0] - R[0][1]) / sq), float(0.25 * sq)};
    }
    else if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
    {
        const double sq = std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0;
        q = gu::Q{float(0.25 * sq), float((R[0][1] + R[1][0]) / sq), float((R[0][2] + R[2][0]) / sq), float((R[2][1] - R[1][2]) / sq)};
    }
    else if (R[1][1] > R[2][2])
    {
        const double sq = std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0;
        q = gu::Q{float((R[0][1] + R[1][0]) / sq), float(0.25 * sq), float((R[1][2] + R[2][1]) / sq), float((R[0][2] - R[2][0]) / sq)};
    }
    else
    {
        const double sq = std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0;
        q = gu::Q{float((R[0][2] + R[2][0]) / sq), float((R[1][2] + R[2][1]) / sq), float(0.25 * sq), float((R[1][0] - R[0][1]) / sq)};
    }
    Pose p = Rotate(lie, gu::Normalize(q), Joint(f, kJHips));
    for (uint32_t i = kFirst; i < kN; ++i) // standing: the root children rise (the host test needs a Hips that moves)
        if (gu::RootChild(Parents().data(), i, kFirst))
            p[size_t(i) * 7 + 2] += 0.85f;
    return p;
}

// linear morph between two local poses (positions lerp, quaternions shortest-arc nlerp)
Pose MorphPose(const Pose& aA, const Pose& aB, float aU)
{
    Pose o(aA.size());
    for (uint32_t i = 0; i < kN; ++i)
    {
        const float* a = &aA[size_t(i) * 7];
        const float* b = &aB[size_t(i) * 7];
        float* d = &o[size_t(i) * 7];
        for (int k = 0; k < 3; ++k)
            d[k] = a[k] + (b[k] - a[k]) * aU;
        gu::Put(d + 3, gu::Nlerp(gu::Get(a + 3), gu::Get(b + 3), aU));
    }
    return o;
}

std::string MakeFile(const char* aRig, int aFrames, bool aHandOff, int aDropBone, const Pose* aPoses /*4 lying poses*/, const Pose* aStand = nullptr, bool aMorph = false)
{
    std::ostringstream o;
    o << "utpp-traversal 2\n";
    o << "RIG " << aRig << " 0x0123456789abcdef 152\n";
    const int nb = int(kN) - (aDropBone >= 0 ? 1 : 0);
    o << "POSEBONES " << nb;
    for (uint32_t i = 0; i < kN; ++i)
        if (int(i) != aDropBone)
            o << ' ' << kGetUpBoneNames[i];
    o << "\nCOUNT 4\n";
    for (int c = 0; c < kGuClipCount; ++c)
    {
        const double fps = 30.0;
        o << "M " << kGuClipNames[c] << " M_ragdoll_getup_stand_" << "BFLR"[c] << ' ' << Fix((aFrames - 1) / fps)
          << " 1.000000 0.300000 SmoothStep 0.300000 SmoothStep 1 -1.000000 Other 30.000000 " << aFrames << " 0\n";
        o << "SEC Default 0.000000 -\n";
        if (aHandOff || c != 1)
            o << "SEC HandOff " << Fix((aFrames - 1) / fps) << " -\n";
        o << "SEC Interrupt " << Fix((aFrames - 1) / fps * 0.7) << " -\n";
        for (int f = 0; f < aFrames; ++f)
            o << "R 0.000000 0.000000 0.000000 0.000000\n";
        o << "C Distance_From_Ledge -\n";
        for (int f = 0; f < aFrames; ++f)
        {
            o << 'P';
            const Pose fp = aStand ? (aMorph ? MorphPose(aPoses[c], *aStand, float(f) / float(aFrames - 1)) : (f >= 4 ? *aStand : aPoses[c])) : aPoses[c];
            for (uint32_t i = 0; i < kN; ++i)
            {
                if (int(i) == aDropBone)
                    continue;
                for (int k = 0; k < 7; ++k)
                {
                    double v = fp[size_t(i) * 7 + k];
                    if (k >= 3) // keep the 1e-4 unit-length bound after the 6-digit rounding
                        v = std::round(v * 1e6) / 1e6;
                    o << ' ' << Fix(v);
                }
            }
            o << '\n';
        }
        o << "E\n";
    }
    o << "END\n";
    return o.str();
}

void TestLoader()
{
    const ClipSet set = MakeClipSet(true);
    Pose poses[kGuClipCount];
    for (int c = 0; c < kGuClipCount; ++c)
        poses[c] = set.local[c];
    const auto name = [](uint32_t i) { return std::string_view(kGetUpBoneNames[i]); };
    const auto build = [&](const std::string& text, GuClips& out, std::string& err) {
        utpp::mm::traversal::TraversalFile f;
        if (!utpp::mm::traversal::ParseTraversalText(text, f, err))
            return false;
        return BuildGetUpClips(f, "player_woman_skeleton", kN, kFirst, Parents().data(), name, out) || (err = out.err, false);
    };
    GuClips ok;
    std::string err;
    const Pose stand = MakeStandPose(set);
    const std::string good = MakeFile("player_woman_skeleton", 20, true, -1, poses, &stand);
    Check(build(good, ok, err) && ok.ok, "loader: a 4-montage file with zero W lines, SEC HandOff / Interrupt and 73 pose bones loads", err);
    if (ok.ok)
    {
        bool shape = true;
        for (int c = 0; c < kGuClipCount; ++c)
            shape = shape && ok.clip[c].ok && ok.clip[c].frames == 20 && std::fabs(ok.clip[c].handOffS - 19.0f / 30.0f) < 1e-4f &&
                    ok.clip[c].frame0Model.size() == size_t(kN) * 3 && ok.clip[c].interruptS < ok.clip[c].handOffS;
        Check(shape && ok.poseMap.size() == kN && ok.poseMap[5] == 5 && ok.bones.jointsOk, "loader: precomputed per-clip data (frames, HandOff, Interrupt, frame-0 joints, pose map)");
        Check(ok.clip[0].feat[0] > 0.8f && ok.clip[1].feat[0] < -0.8f && ok.clip[2].feat[1] < -0.8f && ok.clip[3].feat[1] > 0.8f,
              "loader: frame-0 features of B F L R", F(ok.clip[0].feat[0], ok.clip[1].feat[0], ok.clip[2].feat[1]) + " " + F(ok.clip[3].feat[1]));
    }
    GuClips c1;
    err.clear();
    Check(!build(MakeFile("player_woman_skeleton", 20, false, -1, poses), c1, err) && err.find("HandOff") != std::string::npos,
          "loader: a missing SEC HandOff gives getupOk = false", err);
    err.clear();
    Check(!build(MakeFile("player_woman_skeleton", 20, true, 40, poses), c1, err) && err.find("pose bone missing") != std::string::npos,
          "loader: a wrong bone count (a pose bone missing) gives getupOk = false", err);
    err.clear();
    Check(!build(MakeFile("player_man_skeleton", 20, true, -1, poses), c1, err) && err.find("rig") != std::string::npos,
          "loader: a file of the other body's rig is refused", err);
    err.clear();
    {
        std::string text = good;
        const size_t at = text.find("GetUp_L");
        text.replace(at, 7, "GetUp_X");
        // the montage name is also used in M_.. sequence only after the 2nd token; rename only the first occurrence
        Check(!build(text, c1, err) && err.find("montage missing") != std::string::npos, "loader: a missing montage gives getupOk = false", err);
    }
    err.clear();
    {
        std::string text = good;
        text.pop_back(); // no final LF
        Check(!build(text, c1, err), "loader: a malformed file (no final LF) gives getupOk = false", err);
    }
    // a pose sampled through the pose map reproduces frame 0 of the file
    if (ok.ok)
    {
        utpp::mm::traversal::TraversalFile f;
        std::string e2;
        utpp::mm::traversal::ParseTraversalText(good, f, e2);
        Pose sampled(size_t(kN) * 7);
        SampleGetUpPose(*f.Find("GetUp_B"), f.poseBones.size(), ok.poseMap, 0.0, sampled.data());
        float worst = 0;
        for (size_t i = 0; i < sampled.size(); ++i)
            if (i % 7 < 3)
                worst = std::fmax(worst, std::fabs(sampled[i] - poses[0][i]));
        Check(worst < 1e-5f, "loader: SampleGetUpPose reproduces the baked frame", F(worst));
    }
}

// ---------------------------------------------------------------------------------------------- MapToChooser after a ragdoll
struct RecoverResult
{
    int airFeeds = 0;
    bool anyLanded = false;
};
RecoverResult RunRecover(bool aRecover, float aDist, float aVz, int aPsmDetail = 1)
{
    adapter::MapConfig mc;
    mc.gravity = 16.0f;
    adapter::MapState m;
    adapter::MapContext ctx;
    ctx.ragdollRecover = aRecover;
    const float dt = 1.0f / 60.0f;
    const auto feed = [&](bool aGround, float aD, float aV) {
        const float vel[3] = {0.0f, 0.0f, aV};
        const float mv[2] = {0.0f, 0.0f};
        adapter::AirFeed air;
        air.valid = true;
        air.onGround = aGround;
        air.distanceToGround = aD;
        const uint32_t psm = (uint32_t(aPsmDetail + 1) << 16) | (1u << 6);
        return adapter::MapToChooser(adapter::MakeMotionFeed(vel, mv, dt, psm, 0, air, 0.0f, 0.0f), mc, m, ctx);
    };
    RecoverResult r;
    for (int i = 0; i < 60; ++i)
        feed(true, 0.0f, 0.0f);
    for (int i = 0; i < 20; ++i)
        r.airFeeds += feed(false, aDist, aVz).movementMode == 1 ? 1 : 0;
    for (int i = 0; i < 30; ++i)
    {
        const auto cs = feed(true, 0.0f, 0.0f);
        r.anyLanded = r.anyLanded || cs.justLandedLight || cs.justLandedHeavy || cs.movementMode == 1;
    }
    return r;
}

void TestRecover()
{
    const RecoverResult control = RunRecover(false, 0.17f, -1.5f);
    Check(control.airFeeds == 20 && control.anyLanded, "recover: without ragdollRecover a 0.17 m drop is air and a landing (control)",
          std::to_string(control.airFeeds));
    const RecoverResult fixd = RunRecover(true, 0.17f, -1.5f);
    Check(fixd.airFeeds == 0 && !fixd.anyLanded, "recover: with ragdollRecover a 0.17 m drop (vz -1.5) is ground: mode 0, no justLanded",
          std::to_string(fixd.airFeeds));
    const RecoverResult ledge = RunRecover(true, 1.2f, -1.5f);
    Check(ledge.airFeeds == 20, "recover: a 1.2 m drop stays in the air (a real fall off a ledge)", std::to_string(ledge.airFeeds));
    const RecoverResult fastFall = RunRecover(true, 0.4f, -6.0f);
    Check(fastFall.airFeeds == 20, "recover: the last 0.5 m of a REAL fall (vz -6, 0.4 m above the floor) stays in the air", std::to_string(fastFall.airFeeds));
    const RecoverResult jump = RunRecover(true, 0.17f, 5.0f);
    Check(jump.airFeeds == 20, "recover: a rising jump is never touched", std::to_string(jump.airFeeds));
    const RecoverResult unprobed = RunRecover(true, -1.0f, -1.5f);
    Check(unprobed.airFeeds == 20, "recover: no floor distance (not probed) keeps the air", std::to_string(unprobed.airFeeds));
}
} // namespace

// A's real files (utpp_getup_<body>.utpptrav): the exact files the game loads must pass the loader with the right clip features
// seq 9 (GetUp_F) blend deficit probe on the real clips: a snapshot = the real clip's frame 0 rolled about the body axis (and pitched), put on a flat
// floor; the Begin pre-check loop (9 samples, clip rate 0.5) reports the worst clamp joint before / after the limb correction and its blend weight.
struct GuProbeOut
{
    float raw = 0, res = 0, wRaw = 0, wRes = 0, rms = 0, blend = 0;
    int jRaw = -1, jRes = -1, code = -1;
};
GuProbeOut ProbeBlendDeficit(const utpp::mm::traversal::TravMontage& aM, size_t aPoseBones, const std::vector<int32_t>& aMap, const GuClipInfo* aClips,
                             int aClip, float aSign, double aRollDeg, double aPitchDeg)
{
    GuProbeOut o;
    const GuShape shape{kN, kFirst, Parents().data()};
    Pose pose = aClips[aClip].frame0Local;
    {
        const Fk f = DoFk(pose);
        const float* h = Joint(f, kJHips);
        const float* hd = Joint(f, kJHead);
        float ax = hd[0] - h[0], ay = hd[1] - h[1];
        const float l = std::sqrt(ax * ax + ay * ay);
        ax /= l, ay /= l;
        pose = Rotate(pose, AxisAngle(ax, ay, 0.0f, aRollDeg * kRad), h);
        pose = Rotate(pose, AxisAngle(-ay, ax, 0.0f, aPitchDeg * kRad), h);
        const Fk g = DoFk(pose);
        float lo = 1e9f;
        for (int j = 0; j < kGuClampCount; ++j)
            if (Bones().clamp[j] >= 0)
                lo = std::fmin(lo, g.p[size_t(Bones().clamp[j]) * 3 + 2] - kGuClampRadius[j]);
        for (uint32_t i = kFirst; i < kN; ++i)
            if (gu::RootChild(Parents().data(), i, kFirst))
                pose[size_t(i) * 7 + 2] -= lo; // the lowest clamp joint rests on the floor z = 0
    }
    Rng rng;
    Scene s{{0.0f, 0.0f, 0.0f}, 0.0, {0.0f, 0.0f, 0.0f}, 0.0};
    EntityAtHips(s, pose, rng, 0.0);
    Pose snap;
    MakeSnapshot(pose, s, rng, 0.0, 0.0, snap);
    const GuPlanOut plan = RunPlan(aClips, snap, s, true, aSign, 0.0f);
    o.code = plan.code;
    if (plan.code < 0)
        return o;
    o.rms = plan.plan.rms;
    GuAnchor anchor;
    std::memcpy(anchor.pos, plan.plan.anchor, sizeof anchor.pos);
    anchor.yaw = plan.plan.yaw;
    GuFrame fr;
    std::memcpy(fr.cap, s.entity, sizeof fr.cap);
    fr.yaw = 0.0f;
    const float zg = anchor.pos[2] - fr.cap[2];
    o.blend = std::clamp(plan.plan.blendInS, kGuBlendInS, kGuBlendMaxS);
    Pose clip(size_t(kN) * 7), out(size_t(kN) * 7);
    for (int k = 0; k <= 8; ++k)
    {
        const float t = o.blend * float(k) / 8.0f, w = gu::Smooth(t / o.blend);
        SampleGetUpPose(aM, aPoseBones, aMap, static_cast<double>(t * kGuBlendClipRate), clip.data());
        ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, fr, w, out.data());
        int j = -1;
        const float raw = FloorDeficit(shape, out.data(), Bones(), zg, &j);
        if (raw > o.raw)
            o.raw = raw, o.jRaw = j, o.wRaw = w;
        if (raw > 0.002f)
        {
            (void)FloorCorrect(shape, out.data(), Bones(), zg);
            const float res = FloorDeficit(shape, out.data(), Bones(), zg, &j);
            if (res > o.res)
                o.res = res, o.jRes = j, o.wRes = w;
        }
    }
    return o;
}

void TestRealFiles(const std::string& aDir)
{
    for (int body = 0; body < 2; ++body)
    {
        const bool woman = body == 0;
        const std::string nm = woman ? "woman" : "man";
        const std::string path = aDir + "/utpp_getup_" + nm + ".utpptrav";
        std::ifstream in(path, std::ios::binary);
        if (!in)
        {
            std::printf("SKIP real file %s not present\n", path.c_str());
            continue;
        }
        std::stringstream ss;
        ss << in.rdbuf();
        utpp::mm::traversal::TraversalFile f;
        std::string err;
        const bool parsed = utpp::mm::traversal::ParseTraversalText(ss.str(), f, err);
        Check(parsed, "real file " + nm + ": parses with the traversal grammar", err);
        if (!parsed)
            continue;
        GuClips c;
        const bool ok = BuildGetUpClips(f, woman ? "player_woman_skeleton" : "player_man_skeleton", kN, kFirst, Parents().data(),
                                        [](uint32_t i) { return std::string_view(kGetUpBoneNames[i]); }, c);
        Check(ok, "real file " + nm + ": passes the loader checks (bones, rig, SEC markers)", c.err);
        if (!ok)
            continue;
        Check(c.clip[0].feat[0] > 0.8f && c.clip[1].feat[0] < -0.8f && c.clip[2].feat[1] < -0.8f && c.clip[3].feat[1] > 0.8f,
              "real file " + nm + ": frame-0 features B front+ F front- L lat- R lat+ (standing sign from the real end pose)",
              F(c.clip[0].feat[0], c.clip[1].feat[0], c.clip[2].feat[1]) + " " + F(c.clip[3].feat[1]));
        bool markers = true;
        std::string det;
        for (int k = 0; k < kGuClipCount; ++k)
        {
            markers = markers && c.clip[k].interruptS > 0.0f && c.clip[k].interruptS <= c.clip[k].handOffS && c.clip[k].handOffS > 1.0f;
            det += F(c.clip[k].interruptS, c.clip[k].handOffS, c.clip[k].standSign) + " ; ";
        }
        Check(markers, "real file " + nm + ": 0 < Interrupt <= HandOff", det);
        // the finger gate on the authored frames (clip root on the floor, as anchored in game): how often the hand step acts and how far it turns
        // a hand of the real clips up to the hand-off (an authored palm on the floor may have finger joints just under radius + floor)
        {
            const GuShape shape{kN, kFirst, Parents().data()};
            std::string info;
            float worstTurn = 0.0f;
            for (int k = 0; k < kGuClipCount; ++k)
            {
                const utpp::mm::traversal::TravMontage* m = f.Find(kGuClipNames[k]);
                if (m == nullptr)
                    continue;
                int fired = 0, frames = 0;
                float maxDef = 0.0f, maxTurn = 0.0f, minLow = 1e9f;
                for (float t = 0.0f; t <= c.clip[k].handOffS + 1e-4f; t += 1.0f / 30.0f, ++frames)
                {
                    Pose p(size_t(kN) * 7);
                    SampleGetUpPose(*m, f.poseBones.size(), c.poseMap, static_cast<double>(t), p.data());
                    const float fd = FingerDeficit(shape, p.data(), Bones(), 0.0f);
                    maxDef = std::fmax(maxDef, fd);
                    minLow = std::fmin(minLow, kGuFingerRadius - FingerDeficit(shape, p.data(), Bones(), 1.0f) + 1.0f); // lowest finger joint z
                    if (fd <= 0.002f)
                        continue;
                    ++fired;
                    const Pose before = p;
                    (void)FloorCorrect(shape, p.data(), Bones(), 0.0f);
                    const Fk a = DoFk(p), b = DoFk(before);
                    for (int h = 0; h < 2; ++h)
                    {
                        const float* qa = &a.q[size_t(Bones().clamp[7 + h]) * 4];
                        const float* qb = &b.q[size_t(Bones().clamp[7 + h]) * 4];
                        maxTurn = std::fmax(maxTurn, 2.0f * std::acos(std::fmin(1.0f, std::fabs(qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3]))));
                    }
                }
                worstTurn = std::fmax(worstTurn, maxTurn);
                char buf[160];
                std::snprintf(buf, sizeof buf, "%s fired %d/%d frames, lowest finger joint %.4f m, finger deficit %.4f m, hand turn %.3f rad; ",
                              kGuClipNames[k], fired, frames, static_cast<double>(minLow), static_cast<double>(maxDef), static_cast<double>(maxTurn));
                info += buf;
            }
            std::printf("INFO real file %s hand step on the authored frames: %s\n", nm.c_str(), info.c_str());
        }
        {
            GuClipInfo all[kGuClipCount];
            for (int k = 0; k < kGuClipCount; ++k)
                all[k] = c.clip[k];
            for (int k = 0; k < kGuClipCount; ++k)
            {
                const utpp::mm::traversal::TravMontage* m = f.Find(kGuClipNames[k]);
                if (m == nullptr)
                    continue;
                std::string info;
                for (double pitch : {0.0, 25.0, -25.0})
                    for (double roll : {-60.0, -30.0, 0.0, 30.0, 60.0})
                    {
                        const GuProbeOut o = ProbeBlendDeficit(*m, f.poseBones.size(), c.poseMap, all, k, c.clip[0].standSign, roll, pitch);
                        char buf[200];
                        std::snprintf(buf, sizeof buf, "\n   roll %+4.0f pitch %+4.0f -> plan %d rms %.3f blend %.2f raw %.3f %s@w%.2f res %.3f %s@w%.2f", roll, pitch, o.code,
                                      static_cast<double>(o.rms), static_cast<double>(o.blend), static_cast<double>(o.raw), o.jRaw >= 0 ? kGuClampNames[o.jRaw] : "-",
                                      static_cast<double>(o.wRaw), static_cast<double>(o.res), o.jRes >= 0 ? kGuClampNames[o.jRes] : "-", static_cast<double>(o.wRes));
                        info += buf;
                    }
                std::printf("INFO real file %s blend deficit probe %s (frame 0 rolled / pitched, flat floor):%s\n", nm.c_str(), kGuClipNames[k], info.c_str());
            }
        }
        Check(c.clip[0].standSign == c.clip[1].standSign && c.clip[2].standSign == c.clip[3].standSign && c.clip[0].standSign == c.clip[2].standSign,
              "real file " + nm + ": every clip ends in the same standing belly sign");
        // the Hips of the real frame 0 are far from the clip root (the baked Trajectory is folded in): the plan's anchor-far check uses the predicted Hips
        const size_t hb = static_cast<size_t>(c.bones.joint[kJHips]);
        const float hx = c.clip[0].frame0Model[hb * 3], hy = c.clip[0].frame0Model[hb * 3 + 1];
        std::printf("INFO real file %s: B frame-0 Hips XY from the clip root %.3f %.3f\n", nm.c_str(), static_cast<double>(hx), static_cast<double>(hy));
        // end to end on the real clips: every clip placed at a random pose is classified as itself and aligned, with the entity at the body
        Rng rng;
        GuClipInfo clips[kGuClipCount];
        for (int k = 0; k < kGuClipCount; ++k)
            clips[k] = c.clip[k];
        int bad = 0;
        for (int rep = 0; rep < 40; ++rep)
        {
            const int k = rep % kGuClipCount;
            Scene s;
            s.anchor[0] = static_cast<float>(rng.U(-50, 50));
            s.anchor[1] = static_cast<float>(rng.U(-50, 50));
            s.anchor[2] = 0.0f;
            s.yaw = rng.U(-kPi, kPi);
            s.entity[2] = 0.0f;
            s.entityYaw = rng.U(-kPi, kPi);
            Pose snap;
            EntityAtHips(s, c.clip[k].frame0Local, rng, 0.2);
            MakeSnapshot(c.clip[k].frame0Local, s, rng, 0.006, 0.0, snap);
            const GuPlanOut o = RunPlan(clips, snap, s, true, c.clip[0].standSign);
            const double ey = YawDiffDeg(o.plan.yaw, s.yaw);
            const double exy = std::hypot(o.plan.anchor[0] - s.anchor[0], o.plan.anchor[1] - s.anchor[1]);
            if (o.code != k || ey > 2.0 || exy > 0.03)
                ++bad;
        }
        Check(bad == 0, "real file " + nm + ": each real clip placed at a random pose is classified as itself and aligned (2 deg, 3 cm), entity at the body",
              "bad=" + std::to_string(bad) + "/40");
    }
}

// anchor-far uses the PREDICTED frame-0 Hips (the Root of the baked clips is ~0.4 m from the Hips)
// r13b release continuity: the entity frame of the release (script teleport target) is trusted until a feed sample matches it
void TestReleaseTarget()
{
    GuFrame pre, target, plan;
    pre.cap[0] = 1577.958f; pre.cap[1] = 2254.905f; pre.cap[2] = 182.154f; pre.yaw = -1.2480f;     // r13b: entity before the teleport
    target.cap[0] = 1578.138f; target.cap[1] = 2254.478f; target.cap[2] = 182.0075f; target.yaw = -1.0714f; // 0.46 m / 0.146 m / +10.1 deg
    plan = pre;
    GuTarget t;
    t.Arm(target, pre);
    // no feed sample yet (the gap before the first Step): the target frame
    GuFrame f = t.Resolve(plan, pre, false);
    Check(t.pending && Dist(f.cap, target.cap) < 1e-6f && std::fabs(f.yaw - target.yaw) < 1e-6f, "release target: without a feed sample the target frame is used");
    // a stale pre-teleport sample is not trusted
    t.OnFeed();
    f = t.Resolve(plan, pre, true);
    Check(t.pending && Dist(f.cap, target.cap) < 1e-6f && std::fabs(f.yaw - target.yaw) < 1e-6f, "release target: a stale pre-teleport sample does not move the frame (release target)");
    // a sample within 2 cm / 1 deg of the target ends the wait and is used
    GuFrame closeEnough = target;
    closeEnough.cap[0] += 0.012f;
    closeEnough.yaw += 0.01f;
    t.OnFeed();
    f = t.Resolve(plan, closeEnough, true);
    Check(!t.pending && Dist(f.cap, closeEnough.cap) < 1e-6f, "release target: a sample within 2 cm / 1 deg of the target is adopted (release target)");
    // after that the live sample rules (world anchoring through the capsule)
    GuFrame moved = closeEnough;
    moved.cap[2] += 0.17f;
    f = t.Resolve(plan, moved, true);
    Check(Dist(f.cap, moved.cap) < 1e-6f, "release target: after the match the live sample is the frame (a later capsule move is followed)");
    // a sample somewhere else (push, partial teleport): live at once
    t.Arm(target, pre);
    GuFrame elsewhere = target;
    elsewhere.cap[0] += 0.5f;
    t.OnFeed();
    f = t.Resolve(plan, elsewhere, true);
    Check(!t.pending && Dist(f.cap, elsewhere.cap) < 1e-6f, "release target: a sample that left the pre frame without reaching the target is live (release target)");
    // no teleport at all: bounded wait, then live (the body stays in the world)
    t.Arm(target, pre);
    bool pendingEarly = true;
    for (int i = 0; i < kGuTargetMaxFeeds - 1; ++i)
    {
        t.OnFeed();
        f = t.Resolve(plan, pre, true);
        pendingEarly = pendingEarly && t.pending && Dist(f.cap, target.cap) < 1e-6f;
    }
    t.OnFeed();
    f = t.Resolve(plan, pre, true);
    Check(pendingEarly && !t.pending && Dist(f.cap, pre.cap) < 1e-6f, "release target: no teleport - target for the bounded number of feeds, then the live sample (release target)");
    // disarmed: the old rule (live sample, else plan frame)
    t.Disarm();
    f = t.Resolve(plan, moved, true);
    const GuFrame g = t.Resolve(plan, moved, false);
    Check(Dist(f.cap, moved.cap) < 1e-6f && Dist(g.cap, plan.cap) < 1e-6f, "release target: disarmed it is SelectFrame (live sample, else plan frame)");
}

void TestAnchorFarPredicted()
{
    Rng rng;
    const ClipSet set = MakeClipSet(true);
    GuClipInfo clips[kGuClipCount];
    for (int c = 0; c < kGuClipCount; ++c)
    {
        Pose p = set.local[c];
        for (uint32_t i = kFirst; i < kN; ++i)
            if (gu::RootChild(Parents().data(), i, kFirst))
                p[size_t(i) * 7 + 1] += 0.45f; // the Trajectory offset folded into Hips
        clips[c] = MakeClip(p, set.sign);
    }
    Scene s{{3.0f, 1.0f, 0.0f}, 1.1, {0.0f, 0.0f, 0.0f}, 0.0};
    EntityAtHips(s, clips[0].frame0Local, rng, 0.0);
    Pose snap;
    MakeSnapshot(clips[0].frame0Local, s, rng, 0.0, 0.0, snap);
    const float rootToEntity = std::hypot(s.entity[0] - s.anchor[0], s.entity[1] - s.anchor[1]);
    const GuPlanOut ok = RunPlan(clips, snap, s, true, set.sign);
    Check(ok.code == 0 && rootToEntity > 0.4f, "anchor-far: a body whose clip Root is 0.45 m from its Hips and from the entity is accepted", "root-entity " + F(rootToEntity) + " code " + std::to_string(ok.code));
    Scene farPlaced = s;
    farPlaced.entity[0] = s.entity[0] + 0.9f;
    Pose snap2;
    MakeSnapshot(clips[0].frame0Local, farPlaced, rng, 0.0, 0.0, snap2);
    const GuPlanOut farRes = RunPlan(clips, snap2, farPlaced, true, set.sign);
    Check(farRes.code == kGuPlanAnchorFar, "anchor-far: the entity 0.9 m beyond the placed body is refused", std::string(farRes.plan.why));
}

// ---------------------------------------------------------------------------------------------- hand-off foot step (r13c M3)
// The hand-off blend (held get-up pose -> MM pose, smoothstep over kGuBlendOutS like BlendFromHeldSpan) with a 9 cm stance mismatch: a plain
// blend slides the planted toes; HandOffFootStep lifts the stepping foot instead. M3's definition: toe below the planted height, speed < 0.25 m/s,
// >= 3 frames -> the slip is the largest XY distance from the first frame of such a run.
void TestHandOffStep()
{
    const ClipSet set = MakeClipSet(true);
    const Pose held = MakeStandPose(set);
    const GuShape shape{kN, kFirst, Parents().data()};
    const Fk fh = DoFk(held);
    const int32_t up[2] = {Bones().joint[kJLUp], Bones().joint[kJRUp]};
    const int32_t toe[2] = {Bones().clamp[13], Bones().clamp[14]};
    const float zg = std::fmin(fh.p[size_t(toe[0]) * 3 + 2], fh.p[size_t(toe[1]) * 3 + 2]) - 0.03f; // the lower toe stands 3 cm above the floor
    // the MM stance: both legs swung sideways (about the forward axis) until each toe is about 9 cm from the held one
    Pose mm = held;
    for (int c = 0; c < 2; ++c)
    {
        const bool left = fh.p[size_t(toe[c]) * 3] < fh.p[size_t(Bones().joint[kJHips]) * 3];
        const float ang = left ? 0.10f : -0.10f; // outwards: either way the mismatch is measured below
        const gu::Q dq{0.0f, std::sin(0.5f * ang), 0.0f, std::cos(0.5f * ang)};
        const int32_t par = Parents()[up[c]];
        const gu::Q qm = gu::Normalize(gu::Mul(dq, gu::Get(&fh.q[size_t(up[c]) * 4])));
        const gu::Q qp = gu::Get(&fh.q[size_t(par) * 4]);
        gu::Put(&mm[size_t(up[c]) * 7 + 3], gu::Normalize(gu::Mul(gu::Q{-qp.x, -qp.y, -qp.z, qp.w}, qm)));
        // pitch the leg about the model x axis until the toe stands at the held height again (the MM foot is planted too)
        const float z0 = fh.p[size_t(toe[c]) * 3 + 2];
        const Pose base = mm;
        float lo = -0.4f, hi = 0.4f;
        const auto zAt = [&](float aPhi, Pose* aOut) {
            Pose t = base;
            const Fk f1 = DoFk(base);
            const gu::Q dp{std::sin(0.5f * aPhi), 0.0f, 0.0f, std::cos(0.5f * aPhi)};
            const gu::Q q1 = gu::Normalize(gu::Mul(dp, gu::Get(&f1.q[size_t(up[c]) * 4])));
            gu::Put(&t[size_t(up[c]) * 7 + 3], gu::Normalize(gu::Mul(gu::Q{-qp.x, -qp.y, -qp.z, qp.w}, q1)));
            if (aOut)
                *aOut = t;
            return DoFk(t).p[size_t(toe[c]) * 3 + 2];
        };
        const bool rising = zAt(hi, nullptr) > zAt(lo, nullptr);
        for (int it = 0; it < 40; ++it)
        {
            const float mid = 0.5f * (lo + hi);
            ((zAt(mid, nullptr) < z0) == rising ? lo : hi) = mid;
        }
        (void)zAt(0.5f * (lo + hi), &mm);
    }
    const Fk fm = DoFk(mm);
    float mismatch[2];
    for (int c = 0; c < 2; ++c)
        mismatch[c] = std::hypot(fh.p[size_t(toe[c]) * 3] - fm.p[size_t(toe[c]) * 3], fh.p[size_t(toe[c]) * 3 + 1] - fm.p[size_t(toe[c]) * 3 + 1]);
    Check(mismatch[0] > 0.07f && mismatch[0] < 0.14f && mismatch[1] > 0.07f && mismatch[1] < 0.14f, "hand-off step: the scenario has a 7..14 cm stance mismatch per foot", F(mismatch[0], mismatch[1]));
    const auto blend = [&](float aS) {
        Pose o = held;
        for (uint32_t i = 0; i < kN; ++i)
        {
            const float* a = &held[size_t(i) * 7];
            const float* b = &mm[size_t(i) * 7];
            float* d = &o[size_t(i) * 7];
            for (int k = 0; k < 3; ++k)
                d[k] = a[k] + (b[k] - a[k]) * aS;
            gu::Put(d + 3, gu::Nlerp(gu::Get(a + 3), gu::Get(b + 3), aS));
        }
        return o;
    };
    struct Run
    {
        float slip = 0, floorSlide = 0, hipsStep = 0, floorMargin = 1e9f, maxLift = 0, maxToeZStep = 0, maxToeXyDiff = 0, endErr = 0;
        int stepped = 0;
    };
    const int frames = 18; // 0.3 s at 60 fps
    const float restH[2] = {fh.p[size_t(toe[0]) * 3 + 2] - zg, fh.p[size_t(toe[1]) * 3 + 2] - zg};
    const auto run = [&](bool aStep) {
        Run r;
        std::vector<std::array<float, 3>> prev(2);
        std::vector<std::vector<std::array<float, 3>>> runs(2);
        std::vector<std::array<float, 3>> last(2);
        float prevHips = 0;
        for (int i = 0; i <= frames; ++i)
        {
            const float a = float(i) / float(frames), s = a * a * (3.0f - 2.0f * a);
            Pose p = blend(s);
            const Fk plainFk = DoFk(p);
            if (aStep)
                r.stepped += HandOffFootStep(shape, Bones(), held.data(), mm.data(), s, zg, p.data());
            const Fk fk = DoFk(p);
            const float hips = Joint(fk, kJHips)[2];
            if (i > 0)
                r.hipsStep = std::fmax(r.hipsStep, std::fabs(hips - prevHips));
            prevHips = hips;
            for (int j = 0; j < kGuClampCount; ++j)
                if (Bones().clamp[j] >= 0)
                    r.floorMargin = std::fmin(r.floorMargin, fk.p[size_t(Bones().clamp[j]) * 3 + 2] - zg - kGuClampRadius[j]);
            for (int c = 0; c < 2; ++c)
            {
                const float* t = &fk.p[size_t(toe[c]) * 3];
                const std::array<float, 3> cur{t[0], t[1], t[2] - zg};
                r.maxLift = std::fmax(r.maxLift, cur[2] - restH[c]);
                r.maxToeXyDiff = std::fmax(r.maxToeXyDiff, std::hypot(t[0] - plainFk.p[size_t(toe[c]) * 3], t[1] - plainFk.p[size_t(toe[c]) * 3 + 1]));
                bool planted = false;
                if (i > 0)
                {
                    r.maxToeZStep = std::fmax(r.maxToeZStep, std::fabs(cur[2] - last[c][2]));
                    const float speed = std::sqrt((cur[0] - last[c][0]) * (cur[0] - last[c][0]) + (cur[1] - last[c][1]) * (cur[1] - last[c][1]) +
                                                  (cur[2] - last[c][2]) * (cur[2] - last[c][2])) * 60.0f;
                    planted = cur[2] < restH[c] + 0.003f && speed < 0.25f;
                    if (cur[2] < restH[c] + 0.003f)
                        r.floorSlide += std::hypot(cur[0] - last[c][0], cur[1] - last[c][1]); // path of the toe while it is on the floor
                }
                const auto flush = [&]() {
                    if (runs[c].size() >= 3)
                        for (const auto& q : runs[c])
                            r.slip = std::fmax(r.slip, std::hypot(q[0] - runs[c][0][0], q[1] - runs[c][0][1]));
                    runs[c].clear();
                };
                if (planted)
                    runs[c].push_back(cur);
                else
                    flush();
                if (i == frames)
                {
                    flush();
                    const float* tm = &fm.p[size_t(toe[c]) * 3];
                    r.endErr = std::fmax(r.endErr, std::hypot(t[0] - tm[0], t[1] - tm[1]));
                }
                last[c] = cur;
            }
        }
        return r;
    };
    const Run plain = run(false), step = run(true);
    std::printf("INFO hand-off step: plain floorSlide %.4f slip %.4f hips %.4f floor %.4f | step floorSlide %.4f slip %.4f hips %.4f floor %.4f lift %.4f toeZ/frame %.4f toeXY diff %.4f end %.4f legs %d\n",
                static_cast<double>(plain.floorSlide), static_cast<double>(plain.slip), static_cast<double>(plain.hipsStep), static_cast<double>(plain.floorMargin), static_cast<double>(step.floorSlide), static_cast<double>(step.slip),
                static_cast<double>(step.hipsStep), static_cast<double>(step.floorMargin), static_cast<double>(step.maxLift), static_cast<double>(step.maxToeZStep),
                static_cast<double>(step.maxToeXyDiff), static_cast<double>(step.endErr), step.stepped);
    Check(plain.floorSlide > 0.12f, "hand-off step: the plain blend of a 9 cm stance mismatch drags the two toes over the floor for more than 12 cm in total (the r13c case)", F(plain.floorSlide, plain.slip));
    Check(step.stepped > 0 && step.slip <= 0.015f && step.floorSlide <= 0.03f,
          "hand-off step: with the step a planted toe drifts at most 1.5 cm (M3 definition) and drags over the floor at most 3 cm in total, both feet", F(step.slip, step.floorSlide, step.stepped));
    Check(step.hipsStep <= plain.hipsStep + 1e-6f && step.hipsStep <= 0.01f, "hand-off step: the hips are not touched (no pop)", F(step.hipsStep, plain.hipsStep));
    Check(step.floorMargin >= plain.floorMargin - 1e-4f, "hand-off step: the step only lifts, no joint goes below the plain blend (no floor penetration)", F(step.floorMargin, plain.floorMargin));
    Check(step.maxLift > 0.015f && step.maxLift <= kGuStepLiftM + 0.01f, "hand-off step: the foot is lifted by a few cm (at most the step height)", F(step.maxLift));
    Check(step.maxToeZStep <= 0.01f && step.maxToeXyDiff <= 0.01f, "hand-off step: the toe rises smoothly (<= 1 cm per frame) and its horizontal path is the plain blend (<= 1 cm)", F(step.maxToeZStep, step.maxToeXyDiff));
    Check(step.endErr < 1e-3f, "hand-off step: the blend ends on the MM stance", F(step.endErr));
    // no mismatch, no step: held == MM leaves the blend alone; a foot that is not planted (held toe 20 cm up) is left alone
    {
        Pose p = held;
        const int n0 = HandOffFootStep(shape, Bones(), held.data(), held.data(), 0.5f, zg, p.data());
        Check(n0 == 0 && p == held, "hand-off step: identical stances make no step and do not touch the pose");
        Pose q = blend(0.5f);
        const Pose q0 = q;
        const int n1 = HandOffFootStep(shape, Bones(), held.data(), mm.data(), 0.5f, zg - 0.3f, q.data()); // the floor 30 cm lower: the held foot is airborne
        Check(n1 == 0 && q == q0, "hand-off step: a foot that is not planted (held toe 33 cm above the floor) is not stepped");
    }
}

// r13d M1: the ragdoll's last pose (logged in game) has the right wrist resting on the floor and the fingers bent through it; the get-up
// starts from that pose. The hand step of FloorCorrect keeps every finger joint above the floor from the first frame of the release, moves only
// the hand (no torso pop) and stays continuous while the blend and the deficit change. Driver order as in GetUpApplyLocked.
const float kR13dReleasePose[kN * 7] = {
#include "data/r13d_getup_release_pose.inc"
};
constexpr float kR13dGroundZ = 182.0111f - 182.0931f; // the ray ground at the anchor in the logged model frame

void TestFingerFloor()
{
    const GuShape shape{kN, kFirst, Parents().data()};
    const Pose rel = FromTable(kR13dReleasePose);
    const auto idx = [](const char* aName) {
        for (uint32_t i = 0; i < kN; ++i)
            if (std::string_view(kGetUpBoneNames[i]) == aName)
                return static_cast<int32_t>(i);
        return int32_t(-1);
    };
    const int32_t rHand = idx("RightHand"), rPinky3 = idx("RightHandPinky3"), hips = idx("Hips"), spine3 = idx("Spine3"), head = idx("Head");
    Check(Bones().fingerN[0] == 14 && Bones().fingerN[1] == 14, "hand step: 14 finger joints per hand are resolved (Thumb1-2, four fingers 1-3)",
          std::to_string(Bones().fingerN[0]) + " " + std::to_string(Bones().fingerN[1]));
    {
        const Fk f = DoFk(rel);
        const float wz = f.p[size_t(rHand) * 3 + 2] - kR13dGroundZ, pz = f.p[size_t(rPinky3) * 3 + 2] - kR13dGroundZ;
        Check(std::fabs(wz - 0.0305f) < 0.002f && std::fabs(pz + 0.0569f) < 0.002f,
              "hand step: the logged r13d release pose has the wrist 3 cm up and RightHandPinky3 5.7 cm under the floor", F(wz, pz));
    }
    // lowest margin of the finger joints (kGuFingerRadius) and of the clamp joints (kGuClampRadius; aHandsOnly: the arm joints only) above the floor
    const auto margins = [&](const Pose& aPose, float aZg, bool aHandsOnly, float& aFinger, float& aAll) {
        const Fk f = DoFk(aPose);
        aFinger = aAll = 1e9f;
        for (int j = 0; j < kGuClampCount; ++j)
            if (Bones().clamp[j] >= 0 && (!aHandsOnly || (j >= 3 && j <= 8)))
                aAll = std::fmin(aAll, f.p[size_t(Bones().clamp[j]) * 3 + 2] - aZg - kGuClampRadius[j]);
        for (int h = 0; h < 2; ++h)
            for (int k = 0; k < Bones().fingerN[h]; ++k)
                aFinger = std::fmin(aFinger, f.p[size_t(Bones().finger[h][k]) * 3 + 2] - aZg - kGuFingerRadius);
        aAll = std::fmin(aAll, aFinger);
    };
    // largest step of a finger joint about its wrist between two poses (the shape the hand step changes; the lift and the arm stages move the wrist)
    const auto fingerStep = [&](const Pose& aNow, const Pose& aPrev) {
        const Fk a = DoFk(aNow), b = DoFk(aPrev);
        float mx = 0.0f;
        for (int h = 0; h < 2; ++h)
            for (int k = 0; k < Bones().fingerN[h]; ++k)
            {
                const size_t fb = size_t(Bones().finger[h][k]) * 3, wb = size_t(Bones().clamp[7 + h]) * 3;
                float d[3];
                for (int i = 0; i < 3; ++i)
                    d[i] = (a.p[fb + i] - a.p[wb + i]) - (b.p[fb + i] - b.p[wb + i]);
                mx = std::fmax(mx, std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
            }
        return mx;
    };
    // driver order of GetUpApplyLocked: deficit -> FloorCorrect (gate incl. fingers / an active hand rate) -> residual -> GuLift::Step -> AddLift
    const auto apply = [&](Pose& p, float aZg, GuHandRate& aRate, float aDtHand) {
        if (FloorDeficit(shape, p.data(), Bones(), aZg) > 0.002f || FingerDeficit(shape, p.data(), Bones(), aZg) > 0.002f || aRate.Active())
            (void)FloorCorrect(shape, p.data(), Bones(), aZg, &aRate, aDtHand);
    };

    // 1. the exact correction (the target the rate limit follows): the logged first frame after the release (the release pose with the logged
    // whole-body lift 0.023 m, Begin status maxDeficit) measured RightHandPinky3 at margin -0.027 (metric M1 FAIL); exactly corrected it is on the floor
    {
        Pose p = rel;
        if (FloorDeficit(shape, p.data(), Bones(), kR13dGroundZ) > 0.002f || FingerDeficit(shape, p.data(), Bones(), kR13dGroundZ) > 0.002f)
            (void)FloorCorrect(shape, p.data(), Bones(), kR13dGroundZ);
        AddLift(shape, 0.023f, p.data());
        float fm, am;
        margins(p, kR13dGroundZ, true, fm, am);
        std::printf("INFO r13d logged first frame (lift 0.023), exact hand step: finger margin %.4f, arms %.4f\n", static_cast<double>(fm), static_cast<double>(am));
        Check(fm >= -0.002f, "hand step: exact target, the logged r13d first frame after the release (lift 0.023 m) has every finger joint above radius + floor",
              F(fm, am));
    }
    // the floor rises 1 mm at a time under the logged pose (up to the logged floor): the exact target keeps every finger above it, no jump
    {
        float worst = 1e9f, maxJump = 0;
        std::vector<float> last;
        for (int k = 0; k <= 70; ++k)
        {
            const float zgk = kR13dGroundZ - 0.07f + 0.001f * float(k);
            Pose p = rel;
            if (FloorDeficit(shape, p.data(), Bones(), zgk) > 0.002f || FingerDeficit(shape, p.data(), Bones(), zgk) > 0.002f)
                (void)FloorCorrect(shape, p.data(), Bones(), zgk);
            float fm, am;
            margins(p, zgk, true, fm, am);
            worst = std::fmin(worst, fm);
            const Fk f = DoFk(p);
            std::vector<float> cur;
            for (int k2 = 0; k2 < Bones().fingerN[1]; ++k2)
                cur.insert(cur.end(), &f.p[size_t(Bones().finger[1][k2]) * 3], &f.p[size_t(Bones().finger[1][k2]) * 3] + 3);
            if (!last.empty())
                for (size_t c = 0; c + 2 < cur.size(); c += 3)
                    maxJump = std::fmax(maxJump, Dist(&cur[c], &last[c]));
            last = cur;
        }
        std::printf("INFO r13d hand step sweep: worst finger margin %.4f, max finger move per 1 mm of floor %.4f\n", static_cast<double>(worst),
                    static_cast<double>(maxJump));
        Check(worst >= -0.002f && maxJump <= 0.006f,
              "hand step: exact target, a floor rising 1 mm at a time under the r13d pose keeps every finger above it, no finger jumps over 6 mm",
              F(worst, maxJump));
    }

    // 2. the rate limit on the held release pose (the blend weight is ~0.05 at 0.07 s of a 0.55 s blend): frame 0 = the first apply after Begin
    // (dtApply 0: no lift yet, the hand step gets one nominal 60 fps frame), then the logged lift 0.023 m (the test rig's own residual of this pose
    // is larger than the game's and would hide the fingers). Arms + fingers are judged. 30 / 60 / 144 fps. Every step is normalised to a 60 fps
    // frame (x (1/60) / dt) like metric M2; frame 0 is measured from the ragdoll pose
    for (const float fps : {30.0f, 60.0f, 144.0f})
    {
        const float dt = 1.0f / fps;
        GuHandRate rate;
        Pose prev = rel;
        float worstLate = 1e9f, f0 = 0, maxStep = 0, torso = 0, firstOk = -1.0f;
        for (int i = 0; float(i) * dt <= 0.20f; ++i)
        {
            const float t = float(i) * dt, dtHand = i == 0 ? 1.0f / 60.0f : dt;
            Pose p = rel;
            apply(p, kR13dGroundZ, rate, dtHand);
            const Fk a = DoFk(p), b = DoFk(rel);
            for (int32_t bn : {hips, spine3, head})
                torso = std::fmax(torso, Dist(&a.p[size_t(bn) * 3], &b.p[size_t(bn) * 3]));
            if (i > 0)
                AddLift(shape, 0.023f, p.data());
            float fm, am;
            margins(p, kR13dGroundZ, true, fm, am);
            if (i == 0)
                f0 = fm;
            if (t >= 0.07f - 1e-4f)
                worstLate = std::fmin(worstLate, am);
            if (firstOk < 0.0f && fm >= -0.01f)
                firstOk = t;
            Pose pNoLift = p, prevNoLift = prev; // finger shape about the wrist: the lift moves both alike
            maxStep = std::fmax(maxStep, fingerStep(pNoLift, prevNoLift) * (1.0f / 60.0f) / dtHand);
            prev = p;
        }
        char what[64];
        std::snprintf(what, sizeof what, "%.0f fps", static_cast<double>(fps));
        std::printf("INFO r13d held release pose %s: frame 0 finger margin %.4f, fingers within M1 from %.3f s, worst margin from 0.07 s %.4f, "
                    "finger step about the wrist per 60 fps frame %.4f, torso moved %.5f\n", what, static_cast<double>(f0), static_cast<double>(firstOk),
                    static_cast<double>(worstLate), static_cast<double>(maxStep), static_cast<double>(torso));
        Check(worstLate >= -0.01f, std::string("hand step: held r13d release pose, ") + what +
              ": from 0.07 s on every arm and finger joint within 1 cm of radius + floor (metric M1)", F(worstLate, f0, firstOk));
        Check(maxStep <= 0.02f, std::string("hand step: held r13d release pose, ") + what +
              ": a finger joint moves at most 2 cm about its wrist per 60 fps frame, frame 0 from the ragdoll pose included (no snap)", F(maxStep));
        Check(torso < 1e-5f, std::string("hand step: held r13d release pose, ") + what + ": the hand step moves no torso joint (Hips, Spine3, Head)", F(torso));
        // the floor goes away (e.g. the blend lifts the hand): the applied correction returns to none at the same rate, no snap back
        {
            float back = 0.0f; // largest per-frame change of the right hand's local rotation (rad)
            Pose last = prev;
            bool idle = false;
            for (int i = 1; float(i) * dt <= 0.25f; ++i)
            {
                Pose p = rel;
                apply(p, kR13dGroundZ - 0.5f, rate, dt);
                const float* qa = &p[size_t(rHand) * 7 + 3];
                const float* qb = &last[size_t(rHand) * 7 + 3];
                back = std::fmax(back, 2.0f * std::acos(std::fmin(1.0f, std::fabs(qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3]))));
                last = p;
                idle = !rate.Active();
            }
            Check(idle && back <= kGuHandRateRadS * dt + 1e-3f, std::string("hand step: ") + what +
                  ": without a deficit the correction returns to none within 0.25 s, at the rate limit per frame", F(back, idle ? 1.0 : 0.0));
        }
    }

    // 3. the whole blend (the plan on the logged pose, a test-rig clip), driver order with the rate limit, 30 / 60 / 144 fps
    const ClipSet set = MakeClipSet(true);
    GuClipInfo clips[kGuClipCount];
    for (int c = 0; c < kGuClipCount; ++c)
        clips[c] = MakeClip(set.local[c], set.sign);
    Scene s{{0.0f, 0.0f, kR13dGroundZ}, 0.0, {0.0f, 0.0f, 0.0f}, 0.0};
    const GuPlanOut plan = RunPlan(clips, rel, s, true, set.sign, kR13dGroundZ);
    Check(plan.code >= 0, "hand step: the r13d release pose gets a plan", std::to_string(plan.code));
    if (plan.code < 0)
        return;
    GuAnchor anchor;
    std::memcpy(anchor.pos, plan.plan.anchor, sizeof anchor.pos);
    anchor.pos[2] = kR13dGroundZ;
    anchor.yaw = plan.plan.yaw;
    GuFrame live; // after the teleport: the entity stands at the anchor
    std::memcpy(live.cap, anchor.pos, sizeof live.cap);
    live.yaw = anchor.yaw;
    const float zg = anchor.pos[2] - live.cap[2];
    const Pose& clip = set.local[plan.code];
    const float blendS = std::clamp(plan.plan.blendInS, kGuBlendInS, kGuBlendMaxS);
    for (const float fps : {30.0f, 60.0f, 144.0f})
    {
        const float dt = 1.0f / fps;
        GuLift lift;
        GuHandRate rate;
        Pose prev, prevRaw;
        float worstLate = 1e9f, maxStep = 0, maxStepRaw = 0, torso0 = 0;
        const int frames = static_cast<int>(std::ceil((blendS + 0.1f) / dt));
        for (int i = 0; i <= frames; ++i)
        {
            const float t = std::fmin(float(i) * dt, blendS), dtHand = i == 0 ? 1.0f / 60.0f : dt;
            Pose p(size_t(kN) * 7);
            ComposeBlend(shape, clip.data(), plan.snapLocal.data(), plan.snapRootWorld.data(), anchor, live, gu::Smooth(t / blendS), p.data());
            const Pose raw = p;
            apply(p, zg, rate, dtHand);
            const float lf = lift.Step(FloorDeficit(shape, p.data(), Bones(), zg), i == 0 ? 0.0f : dt, t < blendS ? kGuLiftMaxBlendM : kGuLiftMaxM);
            Pose noLift = p;
            AddLift(shape, lf, p.data());
            if (i == 0)
            {
                const Fk a = DoFk(p), b = DoFk(raw);
                for (int32_t bn : {hips, spine3, head})
                    torso0 = std::fmax(torso0, Dist(&a.p[size_t(bn) * 3], &b.p[size_t(bn) * 3]));
            }
            if (float(i) * dt >= 0.07f - 1e-4f)
            {
                float fm, am;
                margins(p, zg, false, fm, am);
                worstLate = std::fmin(worstLate, am);
            }
            const Pose& from = i == 0 ? raw : prev; // frame 0: from the ragdoll pose (= the uncorrected blend at w = 0)
            const Pose& fromRaw = i == 0 ? raw : prevRaw;
            maxStep = std::fmax(maxStep, fingerStep(noLift, from) * (1.0f / 60.0f) / dtHand);
            maxStepRaw = std::fmax(maxStepRaw, fingerStep(raw, fromRaw) * (1.0f / 60.0f) / dtHand);
            prev = noLift;
            prevRaw = raw;
        }
        char what[64];
        std::snprintf(what, sizeof what, "%.0f fps", static_cast<double>(fps));
        std::printf("INFO r13d blend %.2f s %s: worst margin from 0.07 s %.4f, finger step about the wrist per 60 fps frame %.4f (blend alone %.4f)\n",
                    static_cast<double>(blendS), what, static_cast<double>(worstLate), static_cast<double>(maxStep), static_cast<double>(maxStepRaw));
        Check(torso0 < 1e-5f, std::string("hand step: r13d blend, ") + what + ": frame 0 moves no torso joint (no pop, the gap pose keeps the ragdoll torso)",
              F(torso0));
        Check(worstLate >= -0.01f, std::string("hand step: r13d blend, ") + what + ": from 0.07 s on every clamp and finger joint within 1 cm of radius + floor (M1)",
              F(worstLate));
        Check(maxStep <= maxStepRaw + 0.02f, std::string("hand step: r13d blend, ") + what +
              ": the finger joints move about their wrist at most 2 cm per 60 fps frame more than the blend alone (no snap)", F(maxStep, maxStepRaw));
    }
}

// seq 9 (2026-11-04 23:13:56, GetUp_F): the Begin pre-check predicted and the run logged raw 0.285 / after the limbs 0.209 m on a flat ground plane
// (the plan's lowest-joint ground 1.3 cm above the ray: the ragdoll pose itself was not under it; the residual came from the crossfade), with the
// lift capped at 0.10 m the body stayed up to ~0.11 m in the floor. Reproduction: the same residual profile (rises over ~10 frames to 0.209 m
// at w ~0.5, gone at the blend end) through the full path at 30 / 60 / 144 fps: the blend cap keeps the floor; hips <= 0.06 m per 60 fps frame
// (M2 / M6); after the blend the lift only falls (<= kGuLiftFallMs), also when the deficit is still there at the Blend -> Play switch.
void TestBlendDeficitLift()
{
    const ClipSet set = MakeClipSet(true);
    const GuShape shape{kN, kFirst, Parents().data()};
    const Pose base = set.local[1];
    const Fk f0 = DoFk(base);
    float low = 1e9f; // the lowest torso joint (Hips, Spine3, Head) above radius: the limb pass cannot correct those
    for (int j = 0; j < 3; ++j)
        if (Bones().clamp[j] >= 0)
            low = std::fmin(low, f0.p[size_t(Bones().clamp[j]) * 3 + 2] - kGuClampRadius[j]);
    const float blendS = 0.60f;
    // the floor offset whose residual after the limb pass is the logged 0.209 m (bisection)
    float peakOff = 0.209f;
    {
        float lo = 0.0f, hi = 0.4f;
        for (int it = 0; it < 40; ++it)
        {
            const float mid = 0.5f * (lo + hi);
            Pose p = base;
            (void)FloorCorrect(shape, p.data(), Bones(), low + mid);
            (FloorDeficit(shape, p.data(), Bones(), low + mid) > 0.209f ? hi : lo) = mid;
        }
        peakOff = 0.5f * (lo + hi);
    }
    // the floor above the lowest torso joint: 0 -> peakOff (residual 0.209 m) over 0.15..0.32 s,
    // held to 0.55 s, gone at 0.60 s (aHold: still 0.209 m when the blend ends)
    const auto floorAt = [&](float t, bool aHold) {
        float d = 0.0f;
        if (t >= 0.15f)
            d = peakOff * std::fmin(1.0f, (t - 0.15f) / 0.17f);
        if (!aHold && t >= 0.55f)
            d *= std::fmax(0.0f, 1.0f - (t - 0.55f) / 0.05f);
        return low + d;
    };
    for (const bool hold : {false, true})
        for (const float fps : {30.0f, 60.0f, 144.0f})
        {
            const float dt = 1.0f / fps;
            GuLift lift;
            float worstBelow = 0, maxHips = 0, maxFallAfter = 0, peak = 0, prevHips = 0, residualMax = 0;
            bool have = false;
            for (int i = 0; float(i) * dt <= 1.2f; ++i)
            {
                const float t = float(i) * dt;
                const float zg = floorAt(t, hold);
                Pose p = base;
                if (FloorDeficit(shape, p.data(), Bones(), zg) > 0.002f)
                    (void)FloorCorrect(shape, p.data(), Bones(), zg);
                const float res = FloorDeficit(shape, p.data(), Bones(), zg);
                residualMax = std::fmax(residualMax, res);
                const float before = lift.lift;
                const float lf = lift.Step(res, i == 0 ? 0.0f : dt, t < blendS ? kGuLiftMaxBlendM : kGuLiftMaxM);
                if (t >= blendS)
                    maxFallAfter = std::fmax(maxFallAfter, (before - lf) / dt);
                peak = std::fmax(peak, lf);
                AddLift(shape, lf, p.data());
                const Fk fk = DoFk(p);
                const float hz = Joint(fk, kJHips)[2];
                if (have)
                    maxHips = std::fmax(maxHips, std::fabs(hz - prevHips) * (1.0f / 60.0f) / dt);
                prevHips = hz;
                have = true;
                if (t >= 0.18f && t < blendS && !(t > 0.55f && !hold))
                    for (int j = 0; j < kGuClampCount; ++j)
                        if (Bones().clamp[j] >= 0)
                            worstBelow = std::fmax(worstBelow, zg + kGuClampRadius[j] - fk.p[size_t(Bones().clamp[j]) * 3 + 2]);
            }
            char what[96];
            std::snprintf(what, sizeof what, "%.0f fps%s", static_cast<double>(fps), hold ? ", deficit held past the blend" : "");
            std::printf("INFO seq 9 lift %s: residual %.3f peak lift %.3f worst under the floor %.4f hips step per 60 fps frame %.4f fall after the blend %.3f m/s\n",
                        what, static_cast<double>(residualMax), static_cast<double>(peak), static_cast<double>(worstBelow), static_cast<double>(maxHips),
                        static_cast<double>(maxFallAfter));
            Check(residualMax > 0.2f && residualMax < 0.22f, std::string("blend lift: seq 9 reproduction ") + what +
                  ": a residual of about 0.21 m after the limbs (meaningful)", F(residualMax, peakOff));
            Check(worstBelow <= 0.011f, std::string("blend lift: seq 9 reproduction ") + what +
                  ": during the blend no clamp joint is more than 1 cm below radius + floor (the 0.10 m cap left 0.11 m)", F(worstBelow, peak));
            Check(maxHips <= 0.06f, std::string("blend lift: seq 9 reproduction ") + what + ": the hips move at most 0.06 m per 60 fps frame (M2 / M6)",
                  F(maxHips));
            Check(maxFallAfter <= kGuLiftFallHighMs + 1e-3f, std::string("blend lift: seq 9 reproduction ") + what +
                  ": after the blend the lift falls at most 0.75 m/s toward the 0.10 m cap (no drop at Blend -> Play)", F(maxFallAfter));
        }
}

int main(int argc, char** argv)
{
    const std::string dataDir = argc > 1 ? argv[1] : "";
    const std::string realDir = argc > 2 ? argv[2] : "-";
    TestLastPose();
    TestClassifier(dataDir);
    TestAlignment();
    TestPose();
    TestPhases();
    TestLoader();
    TestAnchorFarPredicted();
    TestReleaseTarget();
    TestHandOffStep();
    TestFingerFloor();
    TestBlendDeficitLift();
    TestRealFiles(realDir);
    TestRecover();
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
