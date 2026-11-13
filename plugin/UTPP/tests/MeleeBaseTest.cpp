// Offline tests of the melee chest base correction (src/mm/MeleeBase.hpp + the generated clip table), not shipped.
// Usage: utpp_melee_base_test   Exit code 0 = all checks passed.
// Rig: the real player rig 0..28 (as tests/MeleeAimTest.cpp) with two proportion sets standing for Female V and Male V. Covers: rig
// resolution (fail closed), the generated table (integrity, round-trip match of every sampled row, noise, interpolation, unknown pose),
// the chest delta (direction of the striking arm preserved for a +-30 deg mismatch of the base about every axis, both rigs), the
// limits (identity inside the knee, C1 soft saturation, asymmetry, what happens at the limit), continuity (weight 0 = bit identity,
// monotone weight ramp, gate ease in / out), the composition with the aim tilt and the lower body guard.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/mm/MeleeAim.hpp"
#include "../src/mm/MeleeBase.hpp"

using namespace utpp::mm;
using namespace utpp::mm::meleebase;
using math::Quat;
using math::Vec3;

static int g_fail = 0;
static int g_pass = 0;

static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}

static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0, double e = 0, double f = 0)
{
    char buf[256];
    std::snprintf(buf, sizeof(buf), aFmt, a, b, c, d, e, f);
    return buf;
}

static constexpr float kR2D = 57.2957795f;
static float AngleDeg(const Quat& a, const Quat& b)
{
    const Quat d = math::QuatMul(math::QuatNormalize(a), math::QuatInv(math::QuatNormalize(b))); // atan2 form: acos is noisy near 0 in float
    return 2.0f * std::atan2(std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z), std::fabs(d.w)) * kR2D;
}
static float AngleDeg(Vec3 a, Vec3 b) { return aim::AngleBetween(a, b) * kR2D; }

// ------------------------------------------------------------------------------------------------ the player rig 0..28

static const char* kNames[] = {"Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1",
                               "LeftLeg", "RightLeg", "Spine2", "LeftFoot", "RightFoot", "Spine3", "LeftShoulder",
                               "RightShoulder", "Neck", "LeftArm", "RightArm", "Neck1", "LeftForeArm", "RightForeArm", "Head",
                               "LeftHand", "RightHand", "LeftHandEnd", "WeaponLeft", "RightHandEnd", "WeaponRight"};
static const int16_t kParents[] = {-1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10, 13, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24, 24};
static constexpr uint32_t kN = 29;
enum : uint32_t
{
    kHips = 2, kSpine = 4, kSpine1 = 7, kSpine2 = 10, kSpine3 = 13, kLShoulder = 14, kRShoulder = 15, kNeck = 16, kLArm = 17, kRArm = 18,
    kNeck1 = 19, kLFore = 20, kRFore = 21, kHead = 22, kLHand = 23, kRHand = 24
};

struct Pose
{
    std::vector<uint8_t> bytes;
};

static void SetLocal(Pose& aPose, uint32_t aIndex, Vec3 aP, Quat aQ)
{
    float* r = reinterpret_cast<float*>(aPose.bytes.data() + size_t(aIndex) * integ::kGameBoneStride);
    r[0] = aP.x, r[1] = aP.y, r[2] = aP.z, r[3] = 0.0f;
    r[4] = aQ.x, r[5] = aQ.y, r[6] = aQ.z, r[7] = aQ.w;
    r[8] = r[9] = r[10] = r[11] = 1.0f;
}
static void SetLocalRot(Pose& aPose, uint32_t aIndex, Quat aQ)
{
    float* r = reinterpret_cast<float*>(aPose.bytes.data() + size_t(aIndex) * integ::kGameBoneStride);
    r[4] = aQ.x, r[5] = aQ.y, r[6] = aQ.z, r[7] = aQ.w;
}
static Quat GetLocalRot(const Pose& aPose, uint32_t aIndex) { return aim::ReadLocal(aPose.bytes.data(), aIndex).q; }

static Quat Axis(Vec3 aAxis, float aDeg) { return aim::AxisAngle(aAxis, aDeg / kR2D); }
static const Vec3 kX{1, 0, 0}, kY{0, 1, 0}, kZ{0, 0, 1};
static Vec3 Norm3(Vec3 v) { return v * (1.0f / aim::Length(v)); }
static Quat Mul(const Quat& a, const Quat& b) { return math::QuatMul(a, b); }

// Proportion set: scale of the bone offsets (Female V ~0.96, Male V ~1.04); the key bones keep different local rotations per body.
static Pose MakePose(float aScale, bool aMale)
{
    Pose p;
    p.bytes.assign(size_t(kN) * integ::kGameBoneStride, 0);
    const Vec3 off[kN] = {{0, 0, 0},     {0, 0, 0},         {0, 0, 0.98f},     {0, 0, 0},          {0, 0, 0.10f},    {-0.10f, 0, -0.05f},
                          {0.10f, 0, -0.05f}, {0, 0, 0.12f}, {0, 0, -0.45f},    {0, 0, -0.45f},     {0, 0, 0.12f},    {0, 0, -0.42f},
                          {0, 0, -0.42f},     {0, 0, 0.14f}, {-0.04f, 0, 0.12f}, {0.04f, 0, 0.12f}, {0, 0, 0.16f},    {-0.12f, 0, 0},
                          {0.12f, 0, 0},      {0, 0, 0.08f}, {0, 0, -0.28f},    {0, 0, -0.28f},     {0, 0, 0.10f},    {0, 0, -0.26f},
                          {0, 0, -0.26f},     {0, 0, -0.08f}, {0, 0.05f, 0},    {0, 0, -0.08f},     {0, 0.05f, 0}};
    for (uint32_t i = 0; i < kN; ++i)
        SetLocal(p, i, Vec3{off[i].x * aScale, off[i].y * aScale, off[i].z * aScale}, Quat{});
    const float t = aMale ? 1.0f : -1.0f; // tiny differences between the bodies' local rotations
    // a boxing guard: the hips lean forward and twist, the spine follows, the arms are raised and bent
    SetLocalRot(p, kHips, Mul(Axis(kZ, 12.0f * t), Axis(kX, -10.0f)));
    SetLocalRot(p, kSpine, Mul(Axis(kX, -6.0f), Axis(kZ, -4.0f * t)));
    SetLocalRot(p, kSpine1, Mul(Axis(kX, -5.0f), Axis(kZ, -6.0f)));
    SetLocalRot(p, kSpine2, Mul(Axis(kX, -3.0f), Axis(kZ, -5.0f * t)));
    SetLocalRot(p, kSpine3, Mul(Axis(kX, 2.0f), Axis(kY, 3.0f * t)));
    SetLocalRot(p, kRShoulder, Axis(kY, 8.0f));
    SetLocalRot(p, kLShoulder, Axis(kY, -8.0f));
    SetLocalRot(p, kRArm, Mul(Axis(kX, -35.0f), Axis(kZ, 20.0f)));
    SetLocalRot(p, kLArm, Mul(Axis(kX, -30.0f), Axis(kZ, -25.0f * t)));
    SetLocalRot(p, kRFore, Axis(kX, -80.0f));
    SetLocalRot(p, kLFore, Axis(kX, -70.0f));
    SetLocalRot(p, kNeck, Axis(kX, 4.0f));
    SetLocalRot(p, kHead, Axis(kX, 5.0f));
    return p;
}

static bool Model(const Pose& aPose, aim::Xf* aOut) { return aim::ComputeModel(aPose.bytes.data(), kN, kParents, kN, aOut); }

static Rig MakeRig()
{
    std::vector<uint64_t> hashes(kN);
    std::vector<int16_t> parents(kParents, kParents + kN);
    for (uint32_t i = 0; i < kN; ++i)
        hashes[i] = integ::CNameHash(kNames[i]);
    return ResolveRig(hashes, parents);
}

// The striking-arm direction (shoulder joint -> wrist) in model space
static Vec3 ArmDir(const aim::Xf* m) { return m[kRHand].p - m[kRArm].p; }

// ------------------------------------------------------------------------------------------------ rig

static void TestRig()
{
    const Rig r = MakeRig();
    Expect("rig: the player rig resolves (chain Root..Spine3 = 6 bones, key bones found, fkCount 14)",
           r.ok == 1 && r.hips == 2 && r.spine == 4 && r.spine1 == 7 && r.spine2 == 10 && r.spine3 == 13 && r.chainLen == 6 &&
               r.chain[0] == 0 && r.chain[5] == 13 && r.fkCount == 14 && r.key[0] == int32_t(kNeck) && r.key[9] == int32_t(kLHand),
           Fmt("ok %g chain %g fk %g", r.ok, r.chainLen, r.fkCount));
    std::vector<uint64_t> hashes(kN);
    std::vector<int16_t> parents(kParents, kParents + kN);
    for (uint32_t i = 0; i < kN; ++i)
        hashes[i] = integ::CNameHash(kNames[i]);
    {
        auto h = hashes;
        h[kSpine2] = 1;
        Expect("rig: a missing Spine2 fails closed", ResolveRig(h, parents).ok == 0);
        h = hashes;
        h[kLHand] = 1;
        Expect("rig: a missing key bone (LeftHand) fails closed", ResolveRig(h, parents).ok == 0);
        auto p = parents;
        p[kSpine2] = int16_t(kSpine);
        Expect("rig: Spine2 not a child of Spine1 fails closed", ResolveRig(hashes, p).ok == 0);
        p = parents;
        p[kSpine3] = int16_t(kSpine3 + 2);
        Expect("rig: non-topological parents fail closed", ResolveRig(hashes, p).ok == 0);
        Expect("rig: empty / mismatched input fails closed", ResolveRig({}, {}).ok == 0 && ResolveRig(hashes, std::vector<int16_t>(3)).ok == 0);
    }
}

// ------------------------------------------------------------------------------------------------ table

static void TestTable()
{
    for (int body = 0; body < 2; ++body)
    {
        const TableData& t = body == 0 ? FemaleTable() : MaleTable();
        const char* nm = body == 0 ? "female" : "male";
        bool unit = true, wpos = true, order = true;
        for (uint32_t i = 0; i < t.frameCount; ++i)
        {
            for (int k = 0; k <= kKeyBones; ++k)
            {
                const Quat q = DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(k) * 4);
                if (std::fabs(math::QuatDot(q, q) - 1.0f) > 2e-3f)
                    unit = false;
                if (q.w < -1e-4f)
                    wpos = false;
            }
            if (i > 0 && t.clipOfFrame[i] < t.clipOfFrame[i - 1])
                order = false;
        }
        Expect((std::string("table ") + nm + ": rows 1821 in 52 clips, unit quaternions, w >= 0, frames of a clip consecutive").c_str(),
               t.frameCount == 1821 && t.clipCount == 52 && unit && wpos && order, Fmt("rows %g clips %g", t.frameCount, t.clipCount));
        bool named = true;
        for (uint32_t c = 0; c < t.clipCount; ++c)
            named = named && t.clipNames[c] && t.clipNames[c][0];
        bool noExcluded = true;
        for (uint32_t c = 0; c < t.clipCount; ++c)
        {
            const std::string s = t.clipNames[c];
            for (const char* ex : {"melee_crouch", "melee_slide", "melee_sprint", "sprint_attack", "crouch_attack", "melee_groundslam"})
                if (s.rfind(ex, 0) == 0)
                    noExcluded = false;
        }
        Expect((std::string("table ") + nm + ": clip names set, the sprint / crouch / slide / air / groundslam clips are not in it").c_str(),
               named && noExcluded);

        // every 7th row: its own key bones find it back (or a row with identical keys and the same chest), rms ~ 0
        bool roundTrip = true;
        float worstRms = 0.0f, worstChest = 0.0f;
        std::string detail;
        for (uint32_t i = 0; i < t.frameCount; i += 7)
        {
            Quat key[kKeyBones];
            for (int k = 0; k < kKeyBones; ++k)
                key[k] = DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(k) * 4);
            Match m;
            const bool ok = FindMatch(t, key, -1, m);
            const Quat want = DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(kKeyBones) * 4);
            const float ca = ok ? AngleDeg(m.chest, want) : 999.0f;
            worstRms = std::max(worstRms, ok ? m.rmsDeg : 999.0f);
            worstChest = std::max(worstChest, ca);
            if (!ok || m.rmsDeg > 0.5f || ca > 3.0f)
            {
                roundTrip = false;
                detail = Fmt("row %g rms %g chest %g", i, ok ? m.rmsDeg : -1.0, ca);
            }
        }
        Expect((std::string("table ") + nm + ": every 7th row is found again (rms < 0.5 deg: int16 quantisation, chest within 3 deg of its own)").c_str(), roundTrip,
               detail.empty() ? Fmt("worst rms %.3f chest %.2f", worstRms, worstChest) : detail);

        // identical key bones in different clips: do the chests agree? (the ambiguity the matcher lives with)
        uint32_t pairs = 0, disagree = 0;
        float worstPair = 0.0f;
        for (uint32_t i = 0; i < t.frameCount; i += 3)
        {
            Quat key[kKeyBones];
            for (int k = 0; k < kKeyBones; ++k)
                key[k] = DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(k) * 4);
            const Quat ci = DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(kKeyBones) * 4);
            for (uint32_t j = i + 1; j < t.frameCount; ++j)
            {
                if (t.clipOfFrame[j] == t.clipOfFrame[i] || KeyDistance(t, j, key, 2e-4f) > 2e-4f)
                    continue;
                ++pairs;
                const float a = AngleDeg(ci, DecodeQuat(t.frames + size_t(j) * kRowInts + size_t(kKeyBones) * 4));
                worstPair = std::max(worstPair, a);
                if (a > 8.0f)
                    ++disagree;
            }
        }
        Expect((std::string("table ") + nm + ": rows with identical key bones in different clips (weighted distance < 2e-4) carry chests within 8 deg of each other (the ambiguity the matcher lives with)").c_str(),
               disagree == 0, Fmt("pairs %g, worst %.2f deg, over 8: %g", pairs, worstPair, disagree));

        // noise: 1.5 deg on every key -> still found, rms ~1.5, chest within 4 deg
        bool noisy = true;
        float worstN = 0.0f;
        for (uint32_t i = 3; i < t.frameCount; i += 53)
        {
            Quat key[kKeyBones];
            for (int k = 0; k < kKeyBones; ++k)
            {
                const Quat q = DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(k) * 4);
                const Vec3 ax = Norm3(Vec3{std::sin(float(k) * 1.7f + 1.0f), std::cos(float(k) * 2.3f), std::sin(float(k) * 0.7f + 2.0f)});
                key[k] = Mul(q, Axis(ax, k % 2 ? -1.5f : 1.5f));
            }
            Match m;
            const bool ok = FindMatch(t, key, int32_t(i) - 2, m);
            const float ca = ok ? AngleDeg(m.chest, DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(kKeyBones) * 4)) : 999.0f;
            worstN = std::max(worstN, ca);
            if (!ok || m.rmsDeg > 3.0f || ca > 5.0f)
                noisy = false;
        }
        Expect((std::string("table ") + nm + ": 1.5 deg of noise on every key bone: still matched (rms < 3), chest within 5 deg").c_str(), noisy,
               Fmt("worst chest %.2f", worstN));

        // a pose that is no fists frame at all
        Quat junk[kKeyBones];
        for (int k = 0; k < kKeyBones; ++k)
            junk[k] = Mul(Axis(kX, 70.0f + 9.0f * float(k)), Axis(kZ, 33.0f * float(k)));
        Match m;
        const bool ok = FindMatch(t, junk, -1, m);
        Expect((std::string("table ") + nm + ": an unknown pose gets no confidence").c_str(), ok && Confidence(m.rmsDeg) == 0.0f,
               Fmt("rms %.1f", m.rmsDeg));
    }
    // the engine's one-frame reference pose (all local rotations identity, arms out to the sides): no confidence, the gate fades the last rotation out
    for (const TableData* t : {&FemaleTable(), &MaleTable()})
    {
        Quat id[kKeyBones];
        Match m;
        const bool ok = FindMatch(*t, id, -1, m);
        Expect("table: the engine's reference pose frame (identity key rotations) gets no confidence", ok && Confidence(m.rmsDeg) < 0.05f, Fmt("rms %.1f", m.rmsDeg));
    }
    // interpolation between two rows of one clip: ~ halfway (t 0.3..0.5), the chest in between
    {
        const TableData& t = FemaleTable();
        // frames 6 / 7 of combo_attack_02 (the arm is moving fast there)
        uint32_t i = 0;
        while (i < t.frameCount && !(std::string(t.clipNames[t.clipOfFrame[i]]) == "combo_attack_02"))
            ++i;
        i += 6;
        Quat key[kKeyBones];
        for (int k = 0; k < kKeyBones; ++k)
            key[k] = math::QuatNlerpShortest(DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(k) * 4),
                                             DecodeQuat(t.frames + size_t(i + 1) * kRowInts + size_t(k) * 4), 0.5f);
        Match m;
        const bool ok = FindMatch(t, key, -1, m);
        const Quat a = DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(kKeyBones) * 4);
        const Quat b = DecodeQuat(t.frames + size_t(i + 1) * kRowInts + size_t(kKeyBones) * 4);
        const Quat mid = math::QuatNlerpShortest(a, b, 0.5f);
        Expect("table: a pose halfway between two rows interpolates (neighbour found, t 0.3..0.5, chest within 1.5 deg of the midpoint)",
               ok && (m.frame == int32_t(i) || m.frame == int32_t(i) + 1) && m.neighbour >= 0 && m.t > 0.3f && m.t <= 0.5f + 1e-4f &&
                   AngleDeg(m.chest, mid) < 1.5f,
               Fmt("t %.2f, chest vs mid %.2f deg; row %g frame %g nb %g chest a-b %.2f", m.t, AngleDeg(m.chest, mid), double(i), double(m.frame), double(m.neighbour), AngleDeg(a, b)));
    }
    Expect("table: confidence 1 up to 5.5 deg, 0 from 10 deg, monotone between",
           Confidence(0.0f) == 1.0f && Confidence(5.5f) == 1.0f && Confidence(10.0f) == 0.0f && Confidence(50.0f) == 0.0f &&
               Confidence(6.0f) > Confidence(8.0f) && Confidence(NAN) == 0.0f);
    Match none;
    Quat bad[kKeyBones];
    bad[0] = Quat{NAN, 0, 0, 1};
    Expect("table: a non-finite key rotation is no match", !FindMatch(FemaleTable(), bad, -1, none) && !none.ok);
}

// ------------------------------------------------------------------------------------------------ direction preserved

// A "clip" pose (hips lean + twist, guard locals) and the same locals on a live base (the hips local replaced by a base rotated by aMis
// about aAxis). The correction must bring Spine3 (and the striking arm) back to the clip's world orientation, exactly (no limits).
static void TestDirection()
{
    const Rig rig = MakeRig();
    for (int body = 0; body < 2; ++body)
    {
        const bool male = body == 1;
        const Pose clip = MakePose(male ? 1.04f : 0.96f, male);
        aim::Xf mc[kN];
        Model(clip, mc);
        const Quat chestClip = mc[kSpine3].q;
        const Vec3 dirClip = ArmDir(mc);
        float worstChest = 0.0f, worstDir = 0.0f, worstPos = 0.0f;
        std::string worst;
        const Vec3 axes[3] = {kX, kY, kZ};
        for (int ax = 0; ax < 3; ++ax)
            for (float mis : {-30.0f, -15.0f, 15.0f, 30.0f})
            {
                Pose live = clip;
                // the live base: Hips and Spine locals changed (pelvis rotated by aMis about the model axis; the spine local keeps the clip's)
                SetLocalRot(live, kHips, Mul(Axis(axes[ax], mis), GetLocalRot(clip, kHips)));
                aim::Xf ml[kN];
                Model(live, ml);
                Quat liveChest;
                const bool okc = LiveChest(live.bytes.data(), kN, rig, liveChest);
                const Quat delta = ChestDelta(chestClip, liveChest);
                const bool oka = ApplyChestRotation(live.bytes.data(), kN, rig, ToRotationVector(delta));
                aim::Xf mf[kN];
                Model(live, mf);
                const float a = AngleDeg(mf[kSpine3].q, chestClip), d = AngleDeg(ArmDir(mf), dirClip);
                // everything under Spine3 keeps the clip's local motion: forearm / hand / head locals untouched
                const float l = AngleDeg(GetLocalRot(live, kRFore), GetLocalRot(clip, kRFore)) + AngleDeg(GetLocalRot(live, kHead), GetLocalRot(clip, kHead));
                // lengths of the chain kept
                const float lenOk = std::fabs(aim::Length(mf[kSpine3].p - mf[kSpine2].p) - aim::Length(mc[kSpine3].p - mc[kSpine2].p));
                if (!okc || !oka || a > worstChest)
                    worstChest = okc && oka ? a : 999.0f;
                if (d > worstDir)
                    worstDir = d;
                worstPos = std::max(worstPos, lenOk);
                if (!okc || !oka || a > 0.05f || d > 0.05f || l > 1e-3f)
                    worst = Fmt("axis %g mis %g: chest %.3f dir %.3f locals %.4f", ax, mis, a, d, l);
            }
        Expect((std::string("direction ") + (male ? "male" : "female") + " rig: base off by +-15 / +-30 deg about X / Y / Z -> chest and striking arm back to the clip's orientation (<= 0.05 deg), arm locals untouched").c_str(),
               worst.empty(), worst.empty() ? Fmt("worst chest %.4f arm %.4f", worstChest, worstDir) : worst);
        Expect((std::string("direction ") + (male ? "male" : "female") + " rig: bone lengths kept").c_str(), worstPos < 1e-5f, Fmt("%.2e", worstPos));
    }
    // a base that already is the clip's: no rotation at all
    {
        const Rig rig2 = MakeRig();
        const Pose clip = MakePose(1.0f, false);
        aim::Xf mc[kN];
        Model(clip, mc);
        Pose live = clip;
        Quat lc;
        LiveChest(live.bytes.data(), kN, rig2, lc);
        const Quat delta = ChestDelta(mc[kSpine3].q, lc);
        Expect("direction: the live base equals the clip's -> the delta is the identity (< 0.01 deg)", AngleDeg(delta, Quat{}) < 0.01f);
    }
    // only Spine1..Spine3 turn: nothing else in the records changes
    {
        Pose p = MakePose(1.0f, false);
        const Pose before = p;
        const Rig rig2 = MakeRig();
        ApplyChestRotation(p.bytes.data(), kN, rig2, Vec3{0.2f, -0.1f, 0.3f});
        bool onlySpine = true;
        for (uint32_t b = 0; b < kN; ++b)
            if (std::memcmp(p.bytes.data() + b * integ::kGameBoneStride, before.bytes.data() + b * integ::kGameBoneStride, integ::kGameBoneStride) != 0 &&
                b != kSpine1 && b != kSpine2 && b != kSpine3)
                onlySpine = false;
        Expect("apply: only the Spine1 / Spine2 / Spine3 records change (Hips, Spine, legs, clavicles, arms, neck, head bitwise)", onlySpine);
    }
}

// ------------------------------------------------------------------------------------------------ limits

static void TestLimits()
{
    const Limits L;
    uint32_t f = 0;
    // identity inside the knee: pitch -25 (fwd limit 35, knee 28), back 12 (limit 20, knee 16), yaw 30 (45, 36), roll 10 (20, 16)
    {
        const Quat q = FromEuler(Euler{20.0f / kR2D, -20.0f / kR2D, 8.0f / kR2D});
        const Quat o = LimitRotation(q, L, &f);
        Expect("limits: inside the knee (yaw 20, forward 20, roll 8; total < 36) the rotation passes unchanged, no flag", AngleDeg(o, q) < 0.02f && f == 0,
               Fmt("diff %.4f flags %g", AngleDeg(o, q), f));
    }
    // saturation
    {
        for (float fw : {40.0f, 60.0f, 85.0f})
        {
            const Quat q = FromEuler(Euler{0.0f, -fw / kR2D, 0.0f});
            const Euler e = ToEuler(LimitRotation(q, L, &f));
            Expect((std::string("limits: forward bend ") + std::to_string(int(fw)) + " deg is held at <= 35 and > 28 (flag forward)").c_str(),
                   -e.pitch * kR2D <= 35.0f + 1e-3f && -e.pitch * kR2D > 28.0f && (f & kLimForward) != 0, Fmt("%.2f", -e.pitch * kR2D));
        }
        const Euler eb = ToEuler(LimitRotation(FromEuler(Euler{0.0f, 50.0f / kR2D, 0.0f}), L, &f));
        Expect("limits: backward bend 50 -> <= 20 (flag back), stricter than forward", eb.pitch * kR2D <= 20.0f + 1e-3f && eb.pitch * kR2D > 16.0f && (f & kLimBack) != 0,
               Fmt("%.2f", eb.pitch * kR2D));
        const Euler ey = ToEuler(LimitRotation(FromEuler(Euler{-80.0f / kR2D, 0.0f, 0.0f}), L, &f));
        Expect("limits: twist 80 -> <= 45 both ways (flag yaw)", -ey.yaw * kR2D <= 45.0f + 1e-3f && -ey.yaw * kR2D > 36.0f && (f & kLimYaw) != 0, Fmt("%.2f", ey.yaw * kR2D));
        const Euler er = ToEuler(LimitRotation(FromEuler(Euler{0.0f, 0.0f, 60.0f / kR2D}), L, &f));
        Expect("limits: roll 60 -> <= 20 (flag roll)", er.roll * kR2D <= 20.0f + 1e-3f && er.roll * kR2D > 16.0f && (f & kLimRoll) != 0, Fmt("%.2f", er.roll * kR2D));
    }
    // C1: the soft saturation is monotone, slope <= 1, continuous slope at the knee
    {
        bool mono = true, slope = true;
        float prev = 0.0f;
        for (float x = 0.0f; x <= 120.0f; x += 0.05f)
        {
            const float y = SoftLimit(x, 35.0f);
            if (y + 1e-5f < prev)
                mono = false;
            if (x > 0.0f && (y - prev) / 0.05f > 1.0001f)
                slope = false;
            prev = y;
        }
        const float knee = kKnee * 35.0f, h = 0.01f;
        const float sBelow = (SoftLimit(knee, 35.0f) - SoftLimit(knee - h, 35.0f)) / h;
        const float sAbove = (SoftLimit(knee + h, 35.0f) - SoftLimit(knee, 35.0f)) / h;
        Expect("limits: soft saturation monotone, slope <= 1, no kink at the knee (slopes 1.0 / ~1.0), never above the limit",
               mono && slope && std::fabs(sBelow - sAbove) < 0.01f && SoftLimit(500.0f, 35.0f) <= 35.0f && SoftLimit(-500.0f, 35.0f) >= -35.0f &&
                   SoftLimit(-10.0f, 35.0f) == -10.0f,
               Fmt("slope below %.4f above %.4f", sBelow, sAbove));
    }
    // euler round trip
    {
        float worst = 0.0f;
        for (float y : {-40.0f, 0.0f, 25.0f})
            for (float p : {-35.0f, -10.0f, 15.0f})
                for (float r : {-18.0f, 0.0f, 12.0f})
                {
                    const Quat q = FromEuler(Euler{y / kR2D, p / kR2D, r / kR2D});
                    const Euler e = ToEuler(q);
                    worst = std::max({worst, std::fabs(e.yaw * kR2D - y), std::fabs(e.pitch * kR2D - p), std::fabs(e.roll * kR2D - r)});
                }
        Expect("limits: euler decomposition round-trips (<= 0.01 deg)", worst < 0.01f, Fmt("%.4f", worst));
        // geometry: forward bend moves the head toward +Y
        const Vec3 up = math::QuatRotate(FromEuler(Euler{0.0f, -20.0f / kR2D, 0.0f}), kZ);
        Expect("limits: negative pitch bends the chest forward (+Y), positive back", up.y > 0.3f && math::QuatRotate(FromEuler(Euler{0.0f, 20.0f / kR2D, 0.0f}), kZ).y < -0.3f);
    }
    {
        // bend 30 + twist 40 + roll 15 are each inside their own limit (knee-ish) but together exceed 45 deg: the total cap holds the size, keeps the axis
        const Quat q = FromEuler(Euler{-40.0f / kR2D, -30.0f / kR2D, 15.0f / kR2D});
        const Quat o = LimitRotation(q, L, &f);
        const float before = aim::Length(ToRotationVector(q)) * kR2D, after = aim::Length(ToRotationVector(o)) * kR2D;
        const Vec3 a1 = ToRotationVector(q), a2 = ToRotationVector(o);
        Expect("limits: the total turn is capped at 45 deg (soft), same axis, flag total",
               after <= 45.0f + 1e-3f && after < before && (f & kLimTotal) != 0 && AngleDeg(a1, a2) < 3.0f, Fmt("%.1f -> %.1f deg, axis moved %.2f", before, after, AngleDeg(a1, a2)));
    }
    Expect("limits: non-finite rotation -> identity, no flags", AngleDeg(LimitRotation(Quat{NAN, 0, 0, 1}, L, &f), Quat{}) < 1e-6f && f == 0);
    // what happens at the limit: the residual stays and is what the log reports; end to end on the rig the applied chest turns by the limited angle
    {
        const Rig rig = MakeRig();
        Pose p = MakePose(1.0f, true);
        aim::Xf m0[kN];
        Model(p, m0);
        const Quat raw = Axis(kX, -70.0f); // asks for a 70 deg forward bend
        const Quat lim = LimitRotation(raw, L, &f);
        ApplyChestRotation(p.bytes.data(), kN, rig, ToRotationVector(lim));
        aim::Xf m1[kN];
        Model(p, m1);
        const float turned = AngleDeg(m1[kSpine3].q, m0[kSpine3].q);
        Expect("limits: asked 70 deg forward -> the chest turns by 28..35 deg (the flag is set, the residual stays), never the 70", turned > 28.0f && turned <= 35.01f && (f & kLimForward) != 0,
               Fmt("%.2f", turned));
    }
}

// ------------------------------------------------------------------------------------------------ continuity

static void TestContinuity()
{
    const Rig rig = MakeRig();
    const Pose base = MakePose(1.0f, false);
    const Vec3 rot = ToRotationVector(Mul(Axis(kX, -22.0f), Axis(kZ, 17.0f)));
    // weight 0: nothing written (bitwise)
    {
        Pose p = base;
        Gate g;
        const Vec3 a = g.Step(true, 1.0f, rot, 0.0f, 0.016f);
        const bool ok = ApplyChestRotation(p.bytes.data(), kN, rig, a);
        Expect("continuity: weight 0 -> zero rotation and the records are bitwise untouched", ok && std::memcmp(p.bytes.data(), base.bytes.data(), p.bytes.size()) == 0);
    }
    // monotone weight ramp 0 -> 1 (the melee mask fade): Spine3 turn grows monotonically, steps proportional, ends at the full rotation
    {
        float prev = -1.0f, maxStep = 0.0f;
        bool mono = true;
        aim::Xf m0[kN];
        Model(base, m0);
        for (int i = 0; i <= 20; ++i)
        {
            const float w = float(i) / 20.0f;
            Pose p = base;
            Gate g;
            g.Step(true, 1.0f, rot, w, 0.016f);
            ApplyChestRotation(p.bytes.data(), kN, rig, g.Step(true, 1.0f, rot, w, 0.016f));
            aim::Xf m[kN];
            Model(p, m);
            const float a = AngleDeg(m[kSpine3].q, m0[kSpine3].q);
            if (a + 1e-4f < prev)
                mono = false;
            if (prev >= 0.0f)
                maxStep = std::max(maxStep, a - prev);
            prev = a;
        }
        const float full = aim::Length(rot) * kR2D;
        Expect("continuity: weight ramp 0 -> 1 turns the chest monotonically up to the full rotation without a jump (step <= 1.2 x full / 20)",
               mono && std::fabs(prev - full) < 0.05f && maxStep <= 1.2f * full / 20.0f, Fmt("full %.2f got %.2f maxStep %.3f", full, prev, maxStep));
    }
    // gate: the first call snaps, a lost match eases out over ~80 ms (no step bigger than the exponential), regained eases in over ~25 ms
    {
        Gate g;
        Vec3 a = g.Step(true, 1.0f, rot, 1.0f, 0.0f);
        const bool snapped = std::fabs(aim::Length(a) - aim::Length(rot)) < 1e-5f;
        float prev = aim::Length(a), maxDrop = 0.0f;
        bool mono = true;
        for (int i = 0; i < 12; ++i) // 12 x 8.3 ms = 100 ms
        {
            a = g.Step(false, 0.0f, Vec3{}, 1.0f, 0.0083f);
            const float v = aim::Length(a);
            if (v > prev + 1e-6f)
                mono = false;
            maxDrop = std::max(maxDrop, (prev - v) / aim::Length(rot));
            prev = v;
        }
        const float after100 = prev / aim::Length(rot);
        for (int i = 0; i < 10; ++i)
            a = g.Step(true, 1.0f, rot, 1.0f, 0.0083f); // regained
        const float regained = aim::Length(a) / aim::Length(rot);
        Expect("continuity: gate snaps on the first call, a lost match fades out smoothly (no step > 12 %% of the rotation per 8 ms, ~30 %% left after 100 ms), a regained one eases back in",
               snapped && mono && maxDrop < 0.12f && after100 > 0.15f && after100 < 0.7f && regained > 0.95f,
               Fmt("maxDrop %.3f after100 %.3f regained %.3f", maxDrop, after100, regained));
        Gate g2;
        g2.Step(true, 1.0f, rot, 1.0f, 0.0f);
        const Vec3 lowConf = g2.Step(true, 0.5f, rot, 1.0f, 0.1f);
        Expect("continuity: a lower match confidence scales the rotation (gate follows the confidence)", aim::Length(lowConf) < aim::Length(rot) && aim::Length(lowConf) > 0.0f);
        g2.Reset();
        Expect("continuity: Reset clears (first call snaps again, NaN weight / dt are safe)",
               aim::Length(g2.Step(true, 1.0f, rot, NAN, NAN)) == 0.0f && g2.Value() == 1.0f);
    }
}

// ------------------------------------------------------------------------------------------------ stickiness, rate limit, hold

static void TestStickyAndSmoothing()
{
    // sticky row choice: two rows of different clips with (nearly) identical keys; with the hint in clip A the choice stays in A
    const TableData& t = FemaleTable();
    uint32_t ia = 0, ib = 0;
    bool found = false;
    for (uint32_t i = 100; i < t.frameCount && !found; ++i)
    {
        Quat key[kKeyBones];
        for (int k = 0; k < kKeyBones; ++k)
            key[k] = DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(k) * 4);
        for (uint32_t j = 0; j < i && !found; ++j)
            if (t.clipOfFrame[j] != t.clipOfFrame[i] && i - j > 10 && KeyDistance(t, j, key, 2e-5f) <= 2e-5f)
            {
                ia = i;
                ib = j;
                found = true;
            }
    }
    Quat keyA[kKeyBones];
    for (int k = 0; k < kKeyBones; ++k)
        keyA[k] = DecodeQuat(t.frames + size_t(ia) * kRowInts + size_t(k) * 4);
    Match m;
    FindMatch(t, keyA, int32_t(ib), m); // hint = the lookalike in the other clip B
    const bool stayedB = m.clip == t.clipOfFrame[ib];
    FindMatch(t, keyA, int32_t(ia), m);
    const bool stayedA = m.clip == t.clipOfFrame[ia];
    Expect("sticky: two clips with the same key bones - the hinted clip is kept (no A-B-A flipping)", found && stayedA && stayedB, Fmt("found %g", found ? 1.0 : 0.0));
    // a pose far from the hinted clip: the clip that really matches wins
    uint32_t far = 0;
    for (uint32_t i = 0; i < t.frameCount; ++i)
    {
        Quat k2[kKeyBones];
        for (int k = 0; k < kKeyBones; ++k)
            k2[k] = DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(k) * 4);
        if (t.clipOfFrame[i] != t.clipOfFrame[ib] && KeyDistance(t, ib, k2, 1e9f) > 0.02f)
        {
            far = i;
            break;
        }
    }
    Quat keyF[kKeyBones];
    for (int k = 0; k < kKeyBones; ++k)
        keyF[k] = DecodeQuat(t.frames + size_t(far) * kRowInts + size_t(k) * 4);
    FindMatch(t, keyF, int32_t(ib), m);
    Expect("sticky: a pose far from the hinted clip switches to the clip that really matches", m.clip == t.clipOfFrame[far] && m.rmsDeg < 1.0f, Fmt("rms %.2f", m.rmsDeg));

    // rate limit + smoothing of the applied rotation (frame-rate independent): the target jumps by 30 deg
    for (float fps : {60.0f, 120.0f, 30.0f})
    {
        Gate g;
        const Vec3 r0 = ToRotationVector(Axis(kX, -10.0f)), r1 = ToRotationVector(Axis(kX, -40.0f));
        g.Step(true, 1.0f, r0, 1.0f, 0.0f);
        const float dt = 1.0f / fps;
        float prev = aim::Length(r0), maxStep = 0.0f, t90 = -1.0f;
        for (int i = 1; i <= int(fps); ++i)
        {
            const float len = aim::Length(g.Step(true, 1.0f, r1, 1.0f, dt));
            maxStep = std::max(maxStep, std::fabs(len - prev));
            prev = len;
            if (t90 < 0.0f && len >= aim::Length(r0) + 0.9f * (aim::Length(r1) - aim::Length(r0)))
                t90 = float(i) * dt;
        }
        Expect(Fmt("rate limit @ %.0f fps: a 30 deg jump of the target moves the applied rotation <= 480 deg/s (+3 %%) and arrives (90 %% by < 0.2 s)", double(fps)).c_str(),
               maxStep / kDegF <= 480.0f * dt * 1.03f && t90 > 0.0f && t90 < 0.2f, Fmt("max step %.2f deg (limit %.2f), 90%% at %.3f s", maxStep / kDegF, 480.0f * dt, t90));
    }
    // a lost match keeps the last rotation at full strength for ~3 frames, then fades
    {
        Gate g;
        const Vec3 r = ToRotationVector(Axis(kX, -25.0f));
        g.Step(true, 1.0f, r, 1.0f, 0.0f);
        float first3 = 1.0f, at12 = 1.0f;
        for (int i = 1; i <= 12; ++i)
        {
            const float v = aim::Length(g.Step(false, 0.0f, Vec3{}, 1.0f, 1.0f / 60.0f)) / aim::Length(r);
            if (i <= 3)
                first3 = std::min(first3, v);
            if (i == 12)
                at12 = v;
        }
        Expect("hold: a lost match (the engine reference pose frame) keeps the rotation at full strength for 3 frames at 60 fps, then fades (< 90 % after 12)",
               first3 > 0.999f && at12 < 0.9f && at12 > 0.1f, Fmt("min of first 3 %.3f, after 12 frames %.3f", first3, at12));
    }
}

// ------------------------------------------------------------------------------------------------ composition with the tilt

static void TestTiltComposition()
{
    const Rig rig = MakeRig();
    // the aim tilt's rig (Spine1..Neck): same player rig
    std::vector<uint64_t> hashes(kN);
    std::vector<int16_t> parents(kParents, kParents + kN);
    for (uint32_t i = 0; i < kN; ++i)
        hashes[i] = integ::CNameHash(kNames[i]);
    const meleeaim::Rig tiltRig = meleeaim::ResolveRig(hashes, parents);
    for (int body = 0; body < 2; ++body)
    {
        const bool male = body == 1;
        const Pose clip = MakePose(male ? 1.04f : 0.96f, male);
        aim::Xf mc[kN];
        Model(clip, mc);
        // reference: the clip's own base + tilt
        Pose ref = clip;
        meleeaim::ApplyTilt(ref.bytes.data(), kN, tiltRig, 12.0f / kR2D, -9.0f / kR2D);
        aim::Xf mr[kN];
        Model(ref, mr);
        // mismatched base: correct first, then tilt
        Pose live = clip;
        SetLocalRot(live, kHips, Mul(Axis(kX, 24.0f), Mul(Axis(kZ, -20.0f), GetLocalRot(clip, kHips))));
        Quat lc;
        LiveChest(live.bytes.data(), kN, rig, lc);
        ApplyChestRotation(live.bytes.data(), kN, rig, ToRotationVector(ChestDelta(mc[kSpine3].q, lc)));
        meleeaim::ApplyTilt(live.bytes.data(), kN, tiltRig, 12.0f / kR2D, -9.0f / kR2D);
        aim::Xf ml[kN];
        Model(live, ml);
        const float d = AngleDeg(ArmDir(ml), ArmDir(mr));
        const float h = AngleDeg(ml[kHead].q, mr[kHead].q);
        Expect((std::string("tilt: ") + (male ? "male" : "female") + " rig: base correction, then the aim tilt on top = the tilt on the clip's own base (arm direction <= 0.2 deg, head <= 0.2 deg)").c_str(),
               d < 0.2f && h < 0.2f, Fmt("arm %.3f head %.3f", d, h));
        // the order matters in principle (rotations do not commute): the reverse order is NOT the same, which is why the correction runs first
        Pose rev = clip;
        SetLocalRot(rev, kHips, Mul(Axis(kX, 24.0f), Mul(Axis(kZ, -20.0f), GetLocalRot(clip, kHips))));
        meleeaim::ApplyTilt(rev.bytes.data(), kN, tiltRig, 12.0f / kR2D, -9.0f / kR2D);
        Quat lc2;
        LiveChest(rev.bytes.data(), kN, rig, lc2);
        ApplyChestRotation(rev.bytes.data(), kN, rig, ToRotationVector(ChestDelta(mc[kSpine3].q, lc2)));
        aim::Xf mv[kN];
        Model(rev, mv);
        // tilt first then the correction lands the chest on the clip's chest and so drops the tilt: arm direction ~ the clip's, not the tilted one
        Expect((std::string("tilt: ") + (male ? "male" : "female") + " rig: correction AFTER the tilt would cancel the tilt (so the runtime runs it before)").c_str(),
               AngleDeg(ArmDir(mv), ArmDir(mc)) < 0.5f && AngleDeg(ArmDir(mv), ArmDir(mr)) > 5.0f,
               Fmt("vs clip %.2f vs tilted %.2f", AngleDeg(ArmDir(mv), ArmDir(mc)), AngleDeg(ArmDir(mv), ArmDir(mr))));
    }
}

// ------------------------------------------------------------------------------------------------ lower body guard

static void TestGuard()
{
    Expect("guard: Female V (engine hips 0.94..0.99, MM 0.93..0.99) stays on the engine, Male V (engine 0.356 vs MM 0.93) goes to MM",
           !KeepLowerOnMM(false, 0.99f, 0.93f) && !KeepLowerOnMM(false, 0.94f, 0.99f) && !KeepLowerOnMM(false, 0.93f, 0.93f) &&
               KeepLowerOnMM(false, 0.356f, 0.93f) && KeepLowerOnMM(false, 0.356f, 0.99f));
    {
        // hysteresis: enter above 0.30, leave below 0.20; between the two the previous decision holds (no snap between engine and MM legs)
        bool k = false;
        int flips = 0;
        bool prev = k;
        float dz = 0.0f;
        for (int i = 0; i < 400; ++i) // dz wanders 0 -> 0.5 -> 0 through the dead band, then hovers around 0.25 for a long time
        {
            dz = i < 100 ? 0.005f * float(i) : (i < 200 ? 0.5f - 0.005f * float(i - 100) : 0.25f + 0.04f * std::sin(0.7f * float(i)));
            k = KeepLowerOnMM(k, 0.93f - dz, 0.93f);
            if (k != prev)
                ++flips;
            prev = k;
        }
        Expect("guard: hysteresis 0.30 / 0.20 - one enter and one leave over a long wander, no flicker while it hovers at 0.25 (0.21..0.29)",
               flips == 2, Fmt("flips %g", flips));
        Expect("guard: a non-finite input keeps the previous decision",
               KeepLowerOnMM(true, NAN, 0.9f) && !KeepLowerOnMM(false, 0.3f, INFINITY) && KeepLowerOnMM(true, 0.9f, 0.93f) == false);
    }
    // BlendIntoRecords: lowerScale 0 leaves the lower bones on the game pose; aLowerOnMM writes them (MM), the default is untouched
    {
        integ::PoseSnapshot snap{};
        snap.weight = 1.0f;
        snap.numBones = 8;
        snap.firstBone = 2;
        snap.upperScale = 0.0f;
        snap.lowerScale = 0.0f;
        for (uint32_t b = 0; b < integ::kMaxMMBones; ++b)
            snap.gameIndex[b] = b < 8 ? int32_t(b) : -1;
        for (uint32_t b = 0; b < 8; ++b)
        {
            float* p = &snap.pose[size_t(b) * 7];
            p[0] = 0.1f * float(b), p[1] = 0.2f, p[2] = 0.93f, p[3] = 0.0f, p[4] = 0.0f, p[5] = 0.0f, p[6] = 1.0f;
        }
        struct Bone { float p[4]; float q[4]; float s[4]; };
        std::vector<Bone> a(8), b(8);
        for (auto* v : {&a, &b})
            for (uint32_t i = 0; i < 8; ++i)
                (*v)[i] = Bone{{0.0f, 0.0f, 0.356f, 0.0f}, {0.1f, 0.0f, 0.0f, 0.995f}, {1, 1, 1, 1}};
        const uint32_t n0 = integ::BlendIntoRecords(reinterpret_cast<uint8_t*>(a.data()), 8, snap, 1.0f);
        const uint32_t n1 = integ::BlendIntoRecords(reinterpret_cast<uint8_t*>(b.data()), 8, snap, 1.0f, 1.0f);
        Expect("guard: full body (lowerScale 0) leaves the game pose; the guard flag writes the MM pose (Hips z 0.93) instead; default argument = old behaviour",
               n0 == 0 && a[2].p[2] == 0.356f && n1 == 6 && std::fabs(b[2].p[2] - 0.93f) < 1e-6f && b[0].p[2] == 0.356f && b[1].p[2] == 0.356f,
               Fmt("n0 %g n1 %g hips %.3f", n0, n1, b[2].p[2]));
    }
}

// ------------------------------------------------------------------------------------------------ lead / game pose keys

// The row of the Female table with the largest chest step to the next row of the same clip, with at least 3 rows after it in the clip
static uint32_t FastChestRow(const TableData& t)
{
    uint32_t best = 0;
    float bestStep = 0.0f;
    for (uint32_t i = 0; i + 3 < t.frameCount; ++i)
    {
        if (t.clipOfFrame[i] != t.clipOfFrame[i + 3])
            continue;
        const float s = AngleDeg(DecodeQuat(t.frames + size_t(i) * kRowInts + size_t(kKeyBones) * 4), DecodeQuat(t.frames + size_t(i + 1) * kRowInts + size_t(kKeyBones) * 4));
        if (s > bestStep)
        {
            bestStep = s;
            best = i;
        }
    }
    return best;
}

static void TestLeadAndGameKeys()
{
    const TableData& t = FemaleTable();
    auto chestOf = [&](uint32_t aRow) { return math::QuatNormalize(DecodeQuat(t.frames + size_t(aRow) * kRowInts + size_t(kKeyBones) * 4)); };
    auto keysOf = [&](uint32_t aRow, Quat* aKey) {
        for (int k = 0; k < kKeyBones; ++k)
            aKey[k] = DecodeQuat(t.frames + size_t(aRow) * kRowInts + size_t(k) * 4);
    };
    // 1. the lead: for an exact row the aimed chest is kChestLeadRows rows further along the same clip (whole row: the next row's chest)
    {
        const uint32_t row = FastChestRow(t);
        Quat key[kKeyBones];
        keysOf(row, key);
        Match m;
        FindMatch(t, key, -1, m);
        const float toLead = AngleDeg(m.chestLead, chestOf(m.frame + 1)), toNow = AngleDeg(m.chestLead, m.chest);
        Expect("lead: an exact row aims at the chest kChestLeadRows (1.0) rows later in the clip, not at the chest of the matched time",
               m.ok && m.frame == int32_t(row) && toLead < 1.5f && toNow > 4.0f, Fmt("row %g: lead vs row+1 %.2f deg, lead vs now %.2f deg", double(row), toLead, toNow));
    }
    // 2. the lead stops at the last row of the clip (it never reads the next clip's rows)
    {
        uint32_t last = 0;
        for (uint32_t i = 0; i + 1 < t.frameCount; ++i)
            if (t.clipOfFrame[i] != t.clipOfFrame[i + 1] && i > 300)
            {
                last = i;
                break;
            }
        Quat key[kKeyBones];
        keysOf(last, key);
        Match m;
        FindMatch(t, key, -1, m);
        Expect("lead: at the last row of a clip the aimed chest is the clip's own last chest (held, no reading into the next clip)",
               m.ok && AngleDeg(m.chestLead, chestOf(last)) < 0.5f, Fmt("row %g: lead vs last row %.3f deg", double(last), AngleDeg(m.chestLead, chestOf(last))));
    }
    // 3. the match is made on the keys of the game pose, the chest on the live records: while the mask blends in the live arms are a mix with
    //    the MM pose (here 60 % identity) and match no row; with the game pose's keys the layer finds the row and applies the correction
    {
        const Rig rig = MakeRig();
        const uint32_t row = FastChestRow(t) + 1;
        Quat gameKey[kKeyBones];
        keysOf(row, gameKey);
        Pose live = MakePose(1.0f, false);
        for (int k = 0; k < kKeyBones; ++k)
            SetLocalRot(live, uint32_t(rig.key[k]), math::QuatNlerpShortest(gameKey[k], Quat{}, 0.6f));
        // the base of the live chest differs from the clip's: Hips turned by 25 deg about X
        SetLocalRot(live, kHips, Mul(Axis(kX, 25.0f), GetLocalRot(live, kHips)));
        auto run = [&](const Quat* aEngineKeys, StepInfo& aInfo, std::vector<uint8_t>& aBytes) {
            Hints hints;
            Gate gate;
            aBytes = live.bytes;
            CorrectChest(aBytes.data(), kN, rig, 0, hints, gate, 1.0f, 0.0f, &aInfo, aEngineKeys);
        };
        StepInfo withKeys, without;
        std::vector<uint8_t> bytesWith, bytesWithout;
        run(gameKey, withKeys, bytesWith);
        run(nullptr, without, bytesWithout);
        Expect("game pose keys: the blended live arms match no row (rms > 10 deg: no correction), the game pose's keys match the row (rms < 1) and turn the chest",
               !without.matched && without.rmsDeg > 10.0f && withKeys.matched && withKeys.rmsDeg < 1.0f && aim::Length(withKeys.applied) > 0.1f && aim::Length(without.applied) < 1e-6f,
               Fmt("without: matched %g rms %.1f applied %.3f | with: matched %g rms %.2f applied %.1f deg", without.matched ? 1.0 : 0.0, without.rmsDeg,
                   aim::Length(without.applied) * kR2D, withKeys.matched ? 1.0 : 0.0, withKeys.rmsDeg, aim::Length(withKeys.applied) * kR2D));
        // the records' key bones are not touched by the match itself (only Spine1..3 turn)
        bool onlySpine = true;
        for (uint32_t b = 0; b < kN; ++b)
            if (std::memcmp(bytesWith.data() + b * integ::kGameBoneStride, live.bytes.data() + b * integ::kGameBoneStride, integ::kGameBoneStride) != 0 && b != kSpine1 && b != kSpine2 && b != kSpine3)
                onlySpine = false;
        Expect("game pose keys: only Spine1 / Spine2 / Spine3 change in the records", onlySpine);
    }
}

int main()
{
    TestRig();
    TestTable();
    TestDirection();
    TestLimits();
    TestContinuity();
    TestStickyAndSmoothing();
    TestTiltComposition();
    TestGuard();
    TestLeadAndGameKeys();
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
