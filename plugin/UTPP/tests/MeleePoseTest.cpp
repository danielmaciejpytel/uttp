// Melee full pose (mm/MeleePose.hpp): data file parsing and synthetic replays per weapon. The clip's upper body (key bones, played at
// 60 fps between the 30 fps rows) runs on a foreign lower body; the layer must write the clip's own lower body time-synchronised with
// the clip, continuously at entry, between strikes and at the hand-back, and hold over a one-frame reference pose.
// Reference pose guard (mm/MeleePoseGuard.hpp): synthetic rigs and the recorded engine poses of <dir>/melee_pose_guard_female.bin.
// usage: utpp_melee_pose_test <dir with utpp_melee_pose_female.bin / utpp_melee_pose_male.bin / melee_pose_guard_female.bin> [<fixture> ...]
// (further fixtures by their magic: "MPG1" reference pose guard, "MPB1" borders and clip choice on recorded frames)
#define _CRT_SECURE_NO_WARNINGS
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "../src/mm/MeleePose.hpp"
#include "../src/mm/MeleePoseGuard.hpp"

using namespace utpp::mm;
using meleepose::LowerPose;
using meleepose::PoseTables;

namespace
{
int g_fail = 0;
void Check(bool aOk, const char* aWhat, double aValue = 0.0)
{
    if (!aOk)
    {
        ++g_fail;
        std::printf("FAIL %s (%.3f)\n", aWhat, aValue);
    }
}

std::vector<uint8_t> ReadFile(const std::string& aPath)
{
    std::ifstream f(aPath, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

float AngleDeg(const math::Quat& aA, const math::Quat& aB)
{
    float d = std::fabs(math::QuatDot(math::QuatNormalize(aA), math::QuatNormalize(aB)));
    d = d > 1.0f ? 1.0f : d;
    return 2.0f * std::acos(d) * 57.2957795f;
}

// key locals of the clip at a fractional absolute row (nlerp between rows, clamped to the clip)
void ClipKeys(const PoseTables& aT, uint32_t aClip, float aRow, math::Quat* aKey)
{
    const float first = float(aT.clipFirstRow[aClip]), last = first + float(aT.clipRowCount[aClip] - 1);
    const float p = aRow < first ? first : (aRow > last ? last : aRow);
    const uint32_t i0 = uint32_t(p), i1 = float(i0) + 1.0f <= last ? i0 + 1 : i0;
    for (int k = 0; k < meleebase::kKeyBones; ++k)
    {
        const math::Quat a = meleebase::DecodeQuat(aT.keys + size_t(i0) * meleebase::kRowInts + k * 4);
        const math::Quat b = meleebase::DecodeQuat(aT.keys + size_t(i1) * meleebase::kRowInts + k * 4);
        aKey[k] = math::QuatNlerpShortest(math::QuatNormalize(a), math::QuatNormalize(b), p - float(i0));
    }
}

struct Records
{
    std::vector<uint8_t> bytes = std::vector<uint8_t>(size_t(meleepose::kLowerBones) * integ::kGameBoneStride, 0);
    void Set(const LowerPose& aP)
    {
        for (int k = 0; k < meleepose::kLowerBones; ++k)
        {
            float* r = reinterpret_cast<float*>(bytes.data() + size_t(k) * integ::kGameBoneStride);
            if (k == 0)
            {
                r[0] = aP.hips.x;
                r[1] = aP.hips.y;
                r[2] = aP.hips.z;
            }
            float* q = r + integ::kGameRotOffsetFloats;
            q[0] = aP.q[k].x;
            q[1] = aP.q[k].y;
            q[2] = aP.q[k].z;
            q[3] = aP.q[k].w;
        }
    }
    LowerPose Get() const
    {
        LowerPose p;
        for (int k = 0; k < meleepose::kLowerBones; ++k)
        {
            const aim::Xf l = aim::ReadLocal(bytes.data(), uint32_t(k));
            p.q[k] = l.q;
            if (k == 0)
                p.hips = l.p;
        }
        return p;
    }
};

struct Diff
{
    float rotDeg = 0.0f, hipsM = 0.0f;
};
Diff PoseDiff(const LowerPose& aA, const LowerPose& aB)
{
    Diff d;
    for (int k = 0; k < meleepose::kLowerBones; ++k)
        d.rotDeg = std::fmax(d.rotDeg, AngleDeg(aA.q[k], aB.q[k]));
    d.hipsM = aim::Length(aA.hips - aB.hips);
    return d;
}

bool IsStandingStrike(const char* aName)
{
    const std::string n(aName);
    return (n.find("attack") != std::string::npos) && n.find("deflect") == std::string::npos && n.find("hit") == std::string::npos &&
        n.find("finisher") == std::string::npos;
}

// the engine's lower body: the first row of the table (a standing guard pose) leaning back 8 deg with the hips 3 cm higher - a standing
// base that is not the clip's (set per table in main)
LowerPose g_foreign;
LowerPose ForeignLower() { return g_foreign; }
void SetForeign(const PoseTables& aT)
{
    g_foreign = meleepose::RowPose(aT, 0);
    g_foreign.q[0] = math::QuatNormalize(math::QuatMul(aim::AxisAngle(math::Vec3{1.0f, 0.0f, 0.0f}, 8.0f * aim::kDegF), g_foreign.q[0]));
    g_foreign.hips.z += 0.03f;
}

struct Totals
{
    int strikes = 0;
    float worstRot = 0.0f, worstHips = 0.0f;
};

// limits of the synthetic replay (measured: vs clip <= 7.4 deg / 17.8 mm, entry <= 15.4 deg, between strikes <= 18.2 deg, with
// reference-pose frames the same steps as without; the engine-base run without the layer: 66..127 deg off)
constexpr float kVsClipDeg = 8.0f;
constexpr float kVsClipHipsM = 0.02f;
constexpr float kEaseStepDeg = 16.0f;
constexpr float kBetweenStepDeg = 20.0f;

struct Run
{
    float sameRot = 0.0f, sameHips = 0.0f;   // matched the played clip: written lower body vs the clip at the played time
    float twinRot = 0.0f, twinHips = 0.0f;   // matched another clip with the same upper body (reported, not judged)
    int sameFrames = 0, twinFrames = 0;
    float stepWithin = 0.0f, ownStep = 0.0f;  // largest single-frame step inside strikes / the clip's own largest step there
    float stepBetween = 0.0f, stepEntry = 0.0f, stepExit = 0.0f, stepHipsWithin = 0.0f, ownStepHips = 0.0f;
    int heldFrames = 0, relocks = 0, strikes = 0;
};

// Plays the strikes of one weapon back to back at 60 fps, each from a start offset of 0..2 rows; the upper body crosses from one strike
// to the next over kCross frames (the game's transition blend); the full-body share eases in over 0.15 s at the start and out over
// 0.15 s at the end. aSeamEvery > 0: every aSeamEvery-th frame inside a strike is replaced by the reference pose (identity keys).
Run ReplayWeapon(const PoseTables& aT, const meleepose::Weapon& aW, int aSeamEvery = 0, bool aWrite = true)
{
    constexpr int kCross = 6;
    const float dt = 1.0f / 60.0f;
    meleepose::LowerRig rig;
    for (int k = 0; k < meleepose::kLowerBones; ++k)
        rig.bone[k] = k;
    rig.ok = 1;
    meleepose::State st;
    Records rec;
    Run run;
    LowerPose prevOut = ForeignLower();
    bool havePrev = false;
    std::vector<uint32_t> strikes;
    for (uint32_t c = 0; c < aW.clipCount; ++c)
        if (IsStandingStrike(aT.clipNames[aW.firstClip + c]))
            strikes.push_back(aW.firstClip + c);
    struct Frame
    {
        uint32_t clip;
        float row;
        int inStrike; // frames since the strike started
        bool crossing;
    };
    std::vector<Frame> frames;
    for (size_t s = 0; s < strikes.size(); ++s)
    {
        const uint32_t clip = strikes[s];
        const float first = float(aT.clipFirstRow[clip]), len = float(aT.clipRowCount[clip] - 1), start = float(s % 3);
        const int n = int((len - start) / aT.rowsPerSecond / dt);
        for (int f = 0; f <= n; ++f)
            frames.push_back(Frame{clip, first + start + float(f) * dt * aT.rowsPerSecond, f, s > 0 && f < kCross});
    }
    const int total = int(frames.size()), ease = 9;
    run.strikes = int(strikes.size());
    math::Quat prevKey[meleebase::kKeyBones];
    int sinceRelock = 1000;
    for (int i = 0; i < total; ++i)
    {
        const Frame& fr = frames[size_t(i)];
        math::Quat key[meleebase::kKeyBones];
        ClipKeys(aT, fr.clip, fr.row, key);
        if (fr.crossing)
        {
            // previous strike's last pose -> this strike over kCross frames
            const Frame& last = frames[size_t(i - fr.inStrike - 1)];
            math::Quat from[meleebase::kKeyBones];
            ClipKeys(aT, last.clip, last.row, from);
            for (int k = 0; k < meleebase::kKeyBones; ++k)
                key[k] = math::QuatNlerpShortest(from[k], key[k], float(fr.inStrike + 1) / float(kCross + 1));
        }
        const bool seam = aSeamEvery > 0 && !fr.crossing && fr.inStrike > 8 && (fr.inStrike % aSeamEvery) == 0;
        if (seam)
            for (auto& q : key)
                q = math::Quat{};
        for (int k = 0; k < meleebase::kKeyBones; ++k)
            prevKey[k] = key[k];
        const float share = std::fmin(1.0f, float(i) / float(ease)) * std::fmin(1.0f, float(total - 1 - i) / float(ease));
        rec.Set(ForeignLower());
        LowerPose pose;
        meleepose::StepInfo si;
        const float w = meleepose::Step(st, aT, &aW, key, share, havePrev ? dt : 0.0f, pose, &si);
        if (w > 0.0f && aWrite)
            meleepose::WriteLower(rec.bytes.data(), uint32_t(meleepose::kLowerBones), rig, pose, w);
        const LowerPose out = rec.Get();
        if (si.held)
            ++run.heldFrames;
        sinceRelock = si.relocked ? 0 : sinceRelock + 1;
        const bool settled = sinceRelock > int(meleepose::kRelockBlendSeconds / dt + 0.5f); // the relock cross-fade is over
        if (si.relocked && havePrev && i > ease)
            ++run.relocks;
        if (havePrev)
        {
            const Diff step = PoseDiff(out, prevOut);
            if (i <= ease)
                run.stepEntry = std::fmax(run.stepEntry, step.rotDeg);
            else if (i >= total - 1 - ease)
                run.stepExit = std::fmax(run.stepExit, step.rotDeg);
            else if (fr.crossing || fr.inStrike < kCross + 4 || !settled)
                run.stepBetween = std::fmax(run.stepBetween, step.rotDeg);
            else
            {
                run.stepWithin = std::fmax(run.stepWithin, step.rotDeg);
                run.stepHipsWithin = std::fmax(run.stepHipsWithin, step.hipsM);
                const Diff own = PoseDiff(meleepose::SamplePose(aT, fr.clip, fr.row), meleepose::SamplePose(aT, fr.clip, fr.row - dt * aT.rowsPerSecond));
                run.ownStep = std::fmax(run.ownStep, own.rotDeg);
                run.ownStepHips = std::fmax(run.ownStepHips, own.hipsM);
            }
        }
        prevOut = out;
        havePrev = true;
        // against the clip at the played time: full share, past the crossing, the matcher on a clip
        if (share >= 1.0f && !fr.crossing && fr.inStrike >= kCross + 4 && si.matched && settled)
        {
            const Diff d = PoseDiff(out, meleepose::SamplePose(aT, fr.clip, fr.row));
            const bool same = si.clip == int32_t(fr.clip);
            if (same)
            {
                ++run.sameFrames;
                run.sameRot = std::fmax(run.sameRot, d.rotDeg);
                run.sameHips = std::fmax(run.sameHips, d.hipsM);
            }
            else
            {
                ++run.twinFrames;
                run.twinRot = std::fmax(run.twinRot, d.rotDeg);
                run.twinHips = std::fmax(run.twinHips, d.hipsM);
            }
        }
    }
    return run;
}

void JudgeWeapon(const PoseTables& aT, const meleepose::Weapon& aW, const char* aBody, const char* aWeapon, Totals& aTot)
{
    const Run r = ReplayWeapon(aT, aW);
    std::printf("%-6s %-22s same clip: %4d fr rot %4.1f deg hips %5.1f mm | twin: %4d fr rot %4.1f hips %5.1f mm | step within %4.1f (clip own %4.1f) "
                "hips %4.1f (own %4.1f) mm, between strikes / relock %4.1f, entry %4.1f, exit %4.1f deg, relocks %d (strikes %d)\n",
                aBody, aWeapon, r.sameFrames, r.sameRot, r.sameHips * 1000.0f, r.twinFrames, r.twinRot, r.twinHips * 1000.0f, r.stepWithin, r.ownStep,
                r.stepHipsWithin * 1000.0f, r.ownStepHips * 1000.0f, r.stepBetween, r.stepEntry, r.stepExit, r.relocks, r.strikes);
    aTot.strikes += 1;
    aTot.worstRot = std::fmax(aTot.worstRot, r.sameRot);
    aTot.worstHips = std::fmax(aTot.worstHips, r.sameHips);
    // most frames are matched on the played clip (the rest: sibling clips whose upper body is the same, see the report)
    Check(r.sameFrames > 5 * r.twinFrames && r.sameFrames > 300, "frames matched on the played clip", r.sameFrames);
    // the lower body follows the played clip at the played time (clock tracking error; the cross-fades after a relock are judged as steps)
    Check(r.sameRot < kVsClipDeg, "lower body rotation vs the played clip (deg)", r.sameRot);
    Check(r.sameHips < kVsClipHipsM, "hips position vs the played clip (m)", r.sameHips);
    // the old behaviour (nothing written: the engine's standing base) is far off the clip
    const Run old = ReplayWeapon(aT, aW, 0, false);
    Check(old.sameRot > 2.0f * kVsClipDeg, "without the layer the lower body is off the clip (deg)", old.sameRot);
    // inside a strike the written lower body moves no faster than the clip itself (+ margin for the clock tracking)
    Check(r.stepWithin <= r.ownStep + 3.0f, "step inside strikes vs the clip's own step (deg)", r.stepWithin - r.ownStep);
    Check(r.stepHipsWithin <= r.ownStepHips + 0.02f, "hips step inside strikes vs the clip's own step (m)", r.stepHipsWithin - r.ownStepHips);
    // entry / exit / between strikes: eased (share fade over 9 frames, relock cross-fade over 0.1 s)
    Check(r.stepEntry < kEaseStepDeg, "entry step (deg)", r.stepEntry);
    Check(r.stepExit < kEaseStepDeg, "exit step (deg)", r.stepExit);
    Check(r.stepBetween < kBetweenStepDeg, "step between strikes / at a relock (deg)", r.stepBetween);
    // a one-frame reference pose every 12th frame: held (no dip to the engine base), steps stay in the range of the between-strike ones
    const Run s = ReplayWeapon(aT, aW, 12);
    Check(s.heldFrames > 0, "seam frames held", s.heldFrames);
    Check(s.stepWithin < kBetweenStepDeg, "seam: step inside strikes with reference-pose frames (deg)", s.stepWithin);
    std::printf("       old (no layer): vs played clip rot %.1f deg hips %.1f mm | with seams: step within %.1f deg, held %d\n", old.sameRot,
                old.sameHips * 1000.0f, s.stepWithin, s.heldFrames);
}

void TestParse(const PoseTables& aT)
{
    Check(aT.ok, "parse");
    for (const char* v : {"w_melee_fists", "w_melee_knife", "w_melee_katana", "w_melee_one_hand_blunt", "w_melee_baton"})
        Check(aT.Find(integ::CNameHash(v)) != nullptr, v);
    Check(aT.Find(integ::CNameHash("w_melee_unknown")) == nullptr, "unknown weapon");
    // a truncated / corrupted file fails closed
    std::vector<uint8_t> b = aT.bytes;
    b.resize(b.size() / 2);
    PoseTables bad;
    Check(!meleepose::ParseTables(b, bad) && !bad.ok, "truncated file rejected");
    b = aT.bytes;
    b[0] = 'X';
    Check(!meleepose::ParseTables(b, bad) && !bad.ok, "bad magic rejected");
}

// share 0 (switch off / standing gate closed): nothing is written and the state resets
void TestOff(const PoseTables& aT, const meleepose::Weapon& aW)
{
    meleepose::State st;
    math::Quat key[meleebase::kKeyBones];
    ClipKeys(aT, aW.firstClip, float(aT.clipFirstRow[aW.firstClip]), key);
    LowerPose pose;
    Check(meleepose::Step(st, aT, &aW, key, 0.0f, 0.016f, pose) == 0.0f, "share 0 writes nothing");
    Check(meleepose::Step(st, aT, nullptr, key, 1.0f, 0.016f, pose) == 0.0f, "no weapon section writes nothing");
}

// ------------------------------------------------------------------------------------------------ reference pose guard

void SetRec(std::vector<uint8_t>& aB, uint32_t aBone, const math::Vec3& aP, const math::Quat& aQ)
{
    float* r = reinterpret_cast<float*>(aB.data() + size_t(aBone) * integ::kGameBoneStride);
    r[0] = aP.x;
    r[1] = aP.y;
    r[2] = aP.z;
    r[3] = 1.0f;
    r[4] = aQ.x;
    r[5] = aQ.y;
    r[6] = aQ.z;
    r[7] = aQ.w;
    r[8] = r[9] = r[10] = 1.0f;
    r[11] = 0.0f;
}

math::Quat AxisAngle(float aX, float aY, float aZ, float aDeg)
{
    const float h = aDeg * 0.5f / 57.2957795f, s = std::sin(h), n = std::sqrt(aX * aX + aY * aY + aZ * aZ);
    return math::Quat{aX / n * s, aY / n * s, aZ / n * s, std::cos(h)};
}

// synthetic rig: Root, Hips, Spine, LeftArm, RightArm (under Spine), LeftUpLeg (under Hips), Head (under Spine)
void TestGuardSynthetic()
{
    const char* names[] = {"Root", "Hips", "Spine", "LeftArm", "RightArm", "LeftUpLeg", "Head"};
    const int16_t parents[] = {-1, 0, 1, 2, 2, 1, 2};
    std::vector<uint64_t> h;
    for (const char* n : names)
        h.push_back(integ::CNameHash(n));
    const meleepose::GuardRig rig = meleepose::ResolveGuardRig(h, parents);
    Check(rig.ok == 1 && rig.count == 4, "guard rig = Spine subtree (Spine, both arms, Head)", rig.count);
    // fail closed: no Spine / a cycle
    std::vector<uint64_t> h2 = h;
    h2[2] = integ::CNameHash("Spine9");
    Check(meleepose::ResolveGuardRig(h2, parents).ok == 0, "guard rig without Spine rejected");
    const int16_t cyc[] = {-1, 6, 1, 2, 2, 1, 1};
    Check(meleepose::ResolveGuardRig(h, cyc).ok == 0, "guard rig with a parent cycle rejected");

    const uint32_t nb = 7;
    auto pose = [&](float aLeftDeg, float aRightDeg, float aLegDeg) {
        std::vector<uint8_t> b(size_t(nb) * integ::kGameBoneStride, 0);
        for (uint32_t k = 0; k < nb; ++k)
            SetRec(b, k, math::Vec3{0.0f, 0.1f * float(k), 0.0f}, math::Quat{0, 0, 0, 1});
        SetRec(b, 3, math::Vec3{0.17f, 0, 0}, AxisAngle(0, 0, 1, aLeftDeg));
        SetRec(b, 4, math::Vec3{0.17f, 0, 0}, AxisAngle(0, 1, 1, aRightDeg));
        SetRec(b, 5, math::Vec3{0, 0, 0}, AxisAngle(1, 0, 0, aLegDeg));
        return b;
    };
    // reference frame: both arms the same local (here: identity), the leg elsewhere
    auto refPose = [&](float aLegDeg) {
        std::vector<uint8_t> b = pose(0.0f, 0.0f, aLegDeg);
        SetRec(b, 3, math::Vec3{0.17f, 0, 0}, AxisAngle(0, 0, 1, -22.6f));
        SetRec(b, 4, math::Vec3{0.17f, 0, 0}, AxisAngle(0, 0, 1, -22.6f));
        SetRec(b, 6, math::Vec3{0, 0.3f, 0}, math::Quat{0, 0, 0, 1});
        return b;
    };
    meleepose::GuardState st;
    std::vector<uint8_t> good = pose(40.0f, 30.0f, 5.0f);
    Check(meleepose::GuardStep(st, rig, good.data(), nb, true, 0, 0.0f) == meleepose::GuardResult::Stored, "normal frame stored");
    std::vector<uint8_t> ref = refPose(7.0f);
    Check(meleepose::GuardStep(st, rig, ref.data(), nb, true, 16, 0.016f) == meleepose::GuardResult::Held, "reference frame held");
    bool same = true;
    for (uint32_t k : {2u, 3u, 4u, 6u})
        same = same && std::memcmp(ref.data() + k * integ::kGameBoneStride, good.data() + k * integ::kGameBoneStride, integ::kGameBoneStride) == 0;
    Check(same, "held frame = last good pose of the Spine subtree");
    Check(AngleDeg(meleepose::GuardRot(ref.data(), 5), AxisAngle(1, 0, 0, 7.0f)) < 1e-3f, "legs of the held frame untouched");
    std::vector<uint8_t> ref2 = refPose(7.0f);
    Check(meleepose::GuardStep(st, rig, ref2.data(), nb, true, 33, 0.033f) == meleepose::GuardResult::Held, "second reference frame held");
    std::vector<uint8_t> ref3 = refPose(7.0f);
    Check(meleepose::GuardStep(st, rig, ref3.data(), nb, true, 50, 0.050f) == meleepose::GuardResult::Passed, "third reference frame passes (cap)");
    Check(meleepose::GuardRot(ref3.data(), 3).w != meleepose::GuardRot(good.data(), 3).w, "passed frame unchanged");
    // a normal frame resets the cap; a stale stored pose is never used
    std::vector<uint8_t> good2 = pose(41.0f, 31.0f, 5.0f);
    Check(meleepose::GuardStep(st, rig, good2.data(), nb, true, 66, 0.1f) == meleepose::GuardResult::Stored, "normal frame after the cap stored");
    std::vector<uint8_t> refStale = refPose(7.0f);
    Check(meleepose::GuardStep(st, rig, refStale.data(), nb, true, 200, 0.134f) == meleepose::GuardResult::Passed, "stale stored pose not used");
    // a pose that really rests on the signature without a step (both arms already equal in the stored pose) is left alone
    meleepose::GuardState st2;
    std::vector<uint8_t> sym = refPose(0.0f);
    Check(meleepose::GuardStep(st2, rig, sym.data(), nb, true, 0, 0.0f) == meleepose::GuardResult::Passed, "signature without stored pose passes");
    std::vector<uint8_t> symGood = pose(-22.0f, 30.0f, 0.0f);
    meleepose::GuardStep(st2, rig, symGood.data(), nb, true, 0, 0.0f);
    std::vector<uint8_t> sym2 = refPose(0.0f);
    SetRec(sym2, 4, math::Vec3{0.17f, 0, 0}, AxisAngle(0, 0, 1, -22.4f)); // within the signature, but neither arm moved 20 deg
    SetRec(symGood, 4, math::Vec3{0.17f, 0, 0}, AxisAngle(0, 0, 1, -10.0f));
    meleepose::GuardStep(st2, rig, symGood.data(), nb, true, 16, 0.016f);
    const meleepose::GuardResult rSym = meleepose::GuardStep(st2, rig, sym2.data(), nb, true, 33, 0.016f);
    Check(rSym == meleepose::GuardResult::Passed, "signature without a 20 deg step passes", double(int(rSym)));
    // inactive: nothing stored, the state is cleared
    std::vector<uint8_t> ref4 = refPose(7.0f);
    Check(meleepose::GuardStep(st, rig, ref4.data(), nb, false, 300, 0.0f) == meleepose::GuardResult::Off && !st.have, "inactive clears");
}

// Recorded engine poses ("MPG1", tools/melee/melee_pose_guard_fixture.py): windows around every reference-pose frame of a melee run plus
// windows around hard cuts and fast strikes (negative controls). Every replaceable reference frame is held, no other frame changes,
// no reference frame reaches the output, and the single-frame hand turns above 100 deg drop to the cuts between two clips.
struct GuardFixtureResult
{
    int frames = 0, signatureIn = 0, held = 0, passed = 0, signatureOut = 0, changedOther = 0, popsIn = 0, popsOut = 0;
};

GuardFixtureResult RunGuardFixture(const std::string& aPath, bool aVerbose)
{
    GuardFixtureResult res;
    const std::vector<uint8_t> in = ReadFile(aPath);
    if (in.size() < 12 || std::memcmp(in.data(), "MPG1", 4) != 0)
    {
        Check(false, ("guard fixture missing or bad: " + aPath).c_str());
        return res;
    }
    size_t o = 4;
    auto rd = [&](void* aDst, size_t aN) {
        if (o + aN > in.size())
            throw 1;
        std::memcpy(aDst, in.data() + o, aN);
        o += aN;
    };
    try
    {
        int32_t nb = 0;
        rd(&nb, 4);
        std::vector<int16_t> par(static_cast<size_t>(nb));
        std::vector<std::string> names(static_cast<size_t>(nb));
        std::vector<uint64_t> hashes(static_cast<size_t>(nb));
        for (int32_t i = 0; i < nb; ++i)
        {
            char s[33] = {};
            rd(&par[size_t(i)], 2);
            rd(s, 32);
            names[size_t(i)] = s;
            hashes[size_t(i)] = integ::CNameHash(names[size_t(i)]);
        }
        int32_t body = 0, nwin = 0;
        rd(&body, 4);
        rd(&nwin, 4);
        const meleepose::GuardRig rig = meleepose::ResolveGuardRig(hashes, par);
        Check(rig.ok == 1, "guard rig of the recorded catalogue");
        int32_t hand[2] = {-1, -1};
        for (int32_t i = 0; i < nb; ++i)
        {
            if (names[size_t(i)] == "LeftHand")
                hand[0] = i;
            if (names[size_t(i)] == "RightHand")
                hand[1] = i;
        }
        // model rotation of a bone (chain through the catalogue parents)
        auto modelRot = [&](const std::vector<uint8_t>& aB, int32_t aBone) {
            math::Quat q{0, 0, 0, 1};
            for (int32_t k = aBone, n = 0; k >= 0 && n < 64; k = par[size_t(k)], ++n)
                q = math::QuatMul(math::QuatNormalize(meleepose::GuardRot(aB.data(), uint32_t(k))), q);
            return q;
        };
        std::vector<int16_t> P(size_t(nb) * 3), Q(size_t(nb) * 4);
        for (int32_t w = 0; w < nwin; ++w)
        {
            char label[33] = {};
            int32_t nf = 0;
            rd(label, 32);
            rd(&nf, 4);
            meleepose::GuardState st;
            double t = 0.0;
            std::vector<uint8_t> prevIn, prevOut;
            int winHeld = 0, winPopsIn = 0, winPopsOut = 0;
            for (int32_t f = 0; f < nf; ++f)
            {
                float dt = 0.0f;
                rd(&dt, 4);
                rd(P.data(), P.size() * 2);
                rd(Q.data(), Q.size() * 2);
                t += double(dt);
                std::vector<uint8_t> b(size_t(nb) * integ::kGameBoneStride, 0);
                for (int32_t i = 0; i < nb; ++i)
                    SetRec(b, uint32_t(i),
                           math::Vec3{P[size_t(i) * 3] * 1e-4f, P[size_t(i) * 3 + 1] * 1e-4f, P[size_t(i) * 3 + 2] * 1e-4f},
                           math::Quat{Q[size_t(i) * 4] / 32767.0f, Q[size_t(i) * 4 + 1] / 32767.0f, Q[size_t(i) * 4 + 2] / 32767.0f,
                                      Q[size_t(i) * 4 + 3] / 32767.0f});
                const std::vector<uint8_t> before = b;
                const bool sig = meleepose::GuardSignature(b.data(), rig);
                const int64_t now = int64_t(t * 1e6);
                const float age = st.have ? float(double(now - st.ticks) * 1e-6) : 1e9f;
                const meleepose::GuardResult r = meleepose::GuardStep(st, rig, b.data(), uint32_t(nb), true, now, age);
                ++res.frames;
                res.signatureIn += sig ? 1 : 0;
                res.held += r == meleepose::GuardResult::Held ? 1 : 0;
                winHeld += r == meleepose::GuardResult::Held ? 1 : 0;
                res.passed += (sig && r != meleepose::GuardResult::Held) ? 1 : 0;
                res.signatureOut += meleepose::GuardSignature(b.data(), rig) ? 1 : 0;
                if (!sig && std::memcmp(before.data(), b.data(), b.size()) != 0)
                    ++res.changedOther;
                if (!prevIn.empty() && hand[0] >= 0 && hand[1] >= 0)
                {
                    float stepIn = 0.0f, stepOut = 0.0f;
                    for (int s = 0; s < 2; ++s)
                    {
                        stepIn = std::fmax(stepIn, AngleDeg(modelRot(before, hand[s]), modelRot(prevIn, hand[s])));
                        stepOut = std::fmax(stepOut, AngleDeg(modelRot(b, hand[s]), modelRot(prevOut, hand[s])));
                    }
                    winPopsIn += stepIn > 100.0f ? 1 : 0;
                    winPopsOut += stepOut > 100.0f ? 1 : 0;
                }
                prevIn = before;
                prevOut = b;
            }
            res.popsIn += winPopsIn;
            res.popsOut += winPopsOut;
            if (aVerbose)
                std::printf("  guard window %-22s frames %3d held %d  hand steps >100 deg: %d -> %d\n", label, nf, winHeld, winPopsIn, winPopsOut);
        }
    }
    catch (int)
    {
        Check(false, ("guard fixture truncated: " + aPath).c_str());
    }
    std::printf("guard %s: frames %d, reference frames in %d, held %d, passed %d, reference frames out %d, other frames changed %d, "
                "hand steps >100 deg %d -> %d\n",
                aPath.c_str(), res.frames, res.signatureIn, res.held, res.passed, res.signatureOut, res.changedOther, res.popsIn, res.popsOut);
    return res;
}

void TestGuardRecorded(const std::string& aPath)
{
    const GuardFixtureResult r = RunGuardFixture(aPath, true);
    Check(r.signatureIn >= 29, "recorded reference frames present", r.signatureIn);
    Check(r.held == r.signatureIn && r.passed == 0 && r.signatureOut == 0, "every recorded reference frame held", r.held);
    Check(r.changedOther == 0, "no other recorded frame changed", r.changedOther);
    // what stays: the cuts between two clips whose poses differ (no reference frame) and the jump from the held pose to the next clip
    Check(r.popsOut * 2 < r.popsIn, "hand steps above 100 deg at least halved", r.popsOut);
}
// ------------------------------------------------------------------------------------------------ borders and clip choice on recorded frames
//
// "MPB1" (tools/melee/melee_pose_border_fixture.py): full-body episodes of a melee run, both recorded stages (stage 0 = the engine pose,
// stage 1 = the pose shown, with the layer as it ran in that session) and the layer's fade per frame.
//   * borders: BorderStep runs on the shown lower body as the written pose. Per foot the ground travel (horizontal path while the ankle is
//     within kLiftHeight of its lowest height in the window) is measured in the entry (0.35 s from the start), at full weight and in the
//     hand-back (from the end on), and the vertical Hips speed in the entry and the hand-back; recorded (as shown) against after the
//     borders. Judged on the windows where the capsule stands.
//   * clip choice: Step runs on the engine keys of stage 0 with the window's weapon section, without the held state and the base
//     preference (free clock) and with them (held clock); reported per window: clip changes, frames on _hit / _deflect clips, and the ground
//     travel of the clip's own feet at full weight.

// ------------------------------------------------------------------------------------------------ model of the runtime call
//
// One call of the runtime (mm/MeleePose.cpp Apply) on plain data: the clip clock and match (Step) on the key bones of the game pose, the write (WriteLower, weight x the full-body fade) and the borders (BorderStep) on the records.
// With the fade closed the state starts over and only the borders' tail runs (BorderStep without a write).

namespace layermodel
{
using namespace meleepose;
struct LayerState
{
    State step;
    BorderState border;
    bool live = false; // the clip clock ran on the previous call (false: the next call is the first of an episode, dt = 0)
};

struct LayerCallInfo
{
    StepInfo step;
    BorderInfo border;
    LowerPose pose{};
    float weight = 0.0f;    // Step's weight (before the fade)
    float written = 0.0f;   // weight written into the records (0 = nothing written)
    bool border0 = false;   // the borders ran this call
};

// aFade = 1 - lowerScale (0 = the standing gate is closed); aDt = seconds since the previous call; aKey may be null (no key bones).
inline void LayerCall(LayerState& aS, const PoseTables& aT, const Weapon* aWeapon, const Quat* aKey, float aFade, float aDt, bool aHoldState,
                      bool aCapsuleMoving, uint8_t* aBones, uint32_t aCount, const LowerRig& aRig, LayerCallInfo* aInfo = nullptr)
{
    LayerCallInfo info;
    const float fade = std::isfinite(aFade) ? aim::Clampf(aFade, 0.0f, 1.0f) : 0.0f;
    if (!(fade > 0.0f))
    {
        aS.step = State{};
        aS.live = false;
        if (aS.border.active)
            info.border0 = BorderStep(aS.border, aBones, aCount, aRig, nullptr, false, 0.0f, aDt, &info.border, aCapsuleMoving);
        if (aInfo)
            *aInfo = info;
        return;
    }
    const float dt = aS.live ? aDt : 0.0f;
    aS.live = true;
    info.weight = Step(aS.step, aT, aWeapon, aKey, 1.0f, dt, info.pose, &info.step, aHoldState);
    const float wEff = info.weight * fade;
    LowerModel pre;
    const bool havePre = wEff > 0.0f && !aS.border.active && ComputeLowerModel(aBones, aCount, aRig, pre);
    bool wrote = false;
    if (wEff > 0.0f && WriteLower(aBones, aCount, aRig, info.pose, wEff))
    {
        wrote = true;
        info.written = wEff;
    }
    info.border0 = BorderStep(aS.border, aBones, aCount, aRig, havePre ? &pre : nullptr, wrote, wEff, aDt, &info.border, aCapsuleMoving);
    if (aInfo)
        *aInfo = info;
}
} // namespace layermodel

// ------------------------------------------------------------------------------------------------ switch blend (synthetic)

// Upper bones 0..10 in kUpperNames order; RightArm (8) cuts by aCutDeg at frame 10 of 30 at 60 fps. Returns the largest single-frame
// RightArm step and the frames the blend wrote.
struct SwitchSynth
{
    float maxStep = 0.0f;
    int written = 0, changedAfter = 0;
};

SwitchSynth RunSwitchSynth(float aCutDeg, float aSeconds, bool aSwitchFlag = true)
{
    std::vector<uint64_t> hashes;
    for (const char* n : meleepose::kUpperNames)
        hashes.push_back(integ::CNameHash(n));
    const meleepose::UpperRig rig = meleepose::ResolveUpperRig(hashes);
    meleepose::SwitchBlendState st;
    SwitchSynth r;
    math::Quat prev{0.0f, 0.0f, 0.0f, 1.0f};
    const float dt = 1.0f / 60.0f;
    for (int f = 0; f < 30; ++f)
    {
        std::vector<uint8_t> b(size_t(meleepose::kUpperBones) * integ::kGameBoneStride, 0);
        // the clip turns the arm slowly (1 deg per frame), the cut adds aCutDeg from frame 10 on
        const math::Quat in = AxisAngle(0, 0, 1, float(f) + (f >= 10 ? aCutDeg : 0.0f));
        for (uint32_t k = 0; k < uint32_t(meleepose::kUpperBones); ++k)
            SetRec(b, k, math::Vec3{}, k == 8 ? in : math::Quat{0.0f, 0.0f, 0.0f, 1.0f});
        const std::vector<uint8_t> before = b;
        const bool wrote = meleepose::SwitchBlendStep(st, b.data(), uint32_t(meleepose::kUpperBones), rig, aSwitchFlag && f == 10, dt, aSeconds);
        r.written += wrote ? 1 : 0;
        if (!wrote && b != before)
            ++r.changedAfter;
        const math::Quat out = aim::ReadLocal(b.data(), 8).q;
        if (f > 0)
        {
            const float d = std::fabs(math::QuatDot(math::QuatNormalize(prev), math::QuatNormalize(out)));
            r.maxStep = std::fmax(r.maxStep, 2.0f * std::acos(d > 1.0f ? 1.0f : d) * 57.2957795f);
        }
        prev = out;
    }
    return r;
}

void TestSwitchBlendSynthetic()
{
    const SwitchSynth cut = RunSwitchSynth(150.0f, meleepose::kSwitchBlendSeconds);
    std::printf("switch blend synthetic: 150 deg cut -> largest step %.1f deg, %d frames written\n", cut.maxStep, cut.written);
    // 0.1 s at 60 fps: the cut spread over 6 frames (150 / 6 = 25 deg) plus the clip's own 1 deg per frame
    Check(cut.maxStep <= 150.0f / 6.0f + 2.0f, "switch blend: a 150 deg cut spread over 0.1 s (deg)", cut.maxStep);
    Check(cut.written >= 5 && cut.written <= 6, "switch blend: written on the frames of 0.1 s only", cut.written);
    Check(cut.changedAfter == 0, "switch blend: records untouched outside the blend", cut.changedAfter);
    const SwitchSynth small = RunSwitchSynth(20.0f, meleepose::kSwitchBlendSeconds);
    Check(small.written == 0, "switch blend: a relock below kSwitchMinDeg is left alone", small.written);
    const SwitchSynth noFlag = RunSwitchSynth(150.0f, meleepose::kSwitchBlendSeconds, false);
    Check(noFlag.written == 0 && noFlag.maxStep >= 150.0f, "switch blend: no relock, no blend (the cut stays)", noFlag.maxStep);
    // mutation: a blend that never runs leaves the cut, the first check catches it
    const SwitchSynth mut = RunSwitchSynth(150.0f, 0.0f);
    Check(mut.maxStep > 150.0f / 6.0f + 2.0f, "switch blend: the step check catches a blend that never runs", mut.maxStep);
    // fail closed: no rig
    meleepose::SwitchBlendState st;
    std::vector<uint8_t> b(size_t(meleepose::kUpperBones) * integ::kGameBoneStride, 0);
    Check(!meleepose::SwitchBlendStep(st, b.data(), uint32_t(meleepose::kUpperBones), meleepose::UpperRig{}, true, 0.016f) && !st.have,
          "switch blend: bad rig writes nothing");
}

// Clip switches of the upper body in one window (report, appended to aPath): the clip the layer's matcher follows on the engine keys of
// stage 0 (free clock); at every frame its clip changes, the single-frame step of both hands (orientation and position in the Spine frame)
// and of the Hips (position and orientation in the model frame), before the mod writes (stage 0) and as shown (stage 1).
utpp::mm::BoneTransform ModelOf(const std::vector<uint8_t>& aRec, const std::vector<int16_t>& aPar, int32_t aBone)
{
    int32_t chain[64];
    int n = 0;
    for (int32_t b = aBone; b >= 0 && n < 64; b = aPar[size_t(b)])
        chain[n++] = b;
    utpp::mm::BoneTransform m{math::Vec3{}, math::Quat{0.0f, 0.0f, 0.0f, 1.0f}};
    for (int i = n - 1; i >= 0; --i)
    {
        const aim::Xf l = aim::ReadLocal(aRec.data(), uint32_t(chain[i]));
        m.p = m.p + math::QuatRotate(m.q, l.p);
        m.q = math::QuatNormalize(math::QuatMul(m.q, l.q));
    }
    return m;
}

float AngleBetweenDeg(const math::Quat& aA, const math::Quat& aB)
{
    float d = std::fabs(math::QuatDot(math::QuatNormalize(aA), math::QuatNormalize(aB)));
    d = d > 1.0f ? 1.0f : d;
    return 2.0f * std::acos(d) * 57.2957795f;
}

struct SwitchPose
{
    utpp::mm::BoneTransform hand[2]; // in the Spine frame
    utpp::mm::BoneTransform hips;    // model frame
};

SwitchPose SwitchPoseOf(const std::vector<uint8_t>& aRec, const std::vector<int16_t>& aPar, const int32_t* aBones)
{
    SwitchPose p;
    const utpp::mm::BoneTransform spine = ModelOf(aRec, aPar, aBones[0]);
    const math::Quat inv = math::QuatInv(spine.q);
    for (int s = 0; s < 2; ++s)
    {
        const utpp::mm::BoneTransform h = ModelOf(aRec, aPar, aBones[1 + s]);
        p.hand[s] = utpp::mm::BoneTransform{math::QuatRotate(inv, h.p - spine.p), math::QuatNormalize(math::QuatMul(inv, h.q))};
    }
    p.hips = ModelOf(aRec, aPar, aBones[3]);
    return p;
}

void SwitchTable(const char* aPath, const char* aLabel, const PoseTables& aT, const meleepose::Weapon& aW, const std::vector<uint64_t>& aHashes,
                 const std::vector<int16_t>& aPar, const meleebase::Rig& aKeyRig, const std::vector<std::vector<uint8_t>>& aS0,
                 const std::vector<std::vector<uint8_t>>& aS1, const std::vector<float>& aDt, const std::vector<float>& aFade, bool aHold)
{
    int32_t bones[4] = {-1, -1, -1, -1};
    const char* names[4] = {"Spine", "LeftHand", "RightHand", "Hips"};
    for (int k = 0; k < 4; ++k)
        for (size_t i = 0; i < aHashes.size(); ++i)
            if (aHashes[i] == integ::CNameHash(names[k]))
                bones[k] = int32_t(i);
    if (bones[0] < 0 || bones[1] < 0 || bones[2] < 0 || bones[3] < 0)
        return;
    FILE* f = std::fopen(aPath, "a");
    if (!f)
        return;
    const uint32_t nb = uint32_t(aHashes.size());
    meleepose::State st;
    int32_t lastClip = -1;
    float lastRow = 0.0f, tm = 0.0f;
    for (size_t i = 0; i < aS0.size(); ++i)
    {
        tm += i > 0 ? aDt[i] : 0.0f;
        math::Quat key[meleebase::kKeyBones];
        const bool haveKeys = meleebase::ReadKeys(aS0[i].data(), nb, aKeyRig, key);
        LowerPose pose;
        meleepose::StepInfo si;
        meleepose::Step(st, aT, &aW, haveKeys ? key : nullptr, 1.0f, i > 0 ? aDt[i] : 0.0f, pose, &si, aHold, true);
        if (si.matched && si.clip >= 0 && lastClip >= 0 && si.clip != lastClip && i > 0)
        {
            float step[2][2][2] = {}; // stage, hand, (deg, cm)
            float hips[2][2] = {};    // stage, (cm, deg)
            for (int s = 0; s < 2; ++s)
            {
                const std::vector<std::vector<uint8_t>>& st2 = s == 0 ? aS0 : aS1;
                const SwitchPose a = SwitchPoseOf(st2[i - 1], aPar, bones), b = SwitchPoseOf(st2[i], aPar, bones);
                for (int h = 0; h < 2; ++h)
                {
                    step[s][h][0] = AngleBetweenDeg(a.hand[h].q, b.hand[h].q);
                    const math::Vec3 d = b.hand[h].p - a.hand[h].p;
                    step[s][h][1] = std::sqrt(math::Dot(d, d)) * 100.0f;
                }
                const math::Vec3 dh = b.hips.p - a.hips.p;
                hips[s][0] = std::sqrt(math::Dot(dh, dh)) * 100.0f;
                hips[s][1] = AngleBetweenDeg(a.hips.q, b.hips.q);
            }
            std::fprintf(f, "%-22s t %5.2f s fade %.2f | %-28s fr %5.1f -> %-28s fr %5.1f | hands L/R deg cm s0 %5.1f %4.1f / %5.1f %4.1f  s1 %5.1f %4.1f / "
                            "%5.1f %4.1f | hips cm deg s0 %4.1f %5.1f  s1 %4.1f %5.1f\n",
                         aLabel, tm, aFade[i], aT.clipNames[size_t(lastClip)], lastRow - float(aT.clipFirstRow[size_t(lastClip)]),
                         aT.clipNames[size_t(si.clip)], si.clockRow - float(aT.clipFirstRow[size_t(si.clip)]), step[0][0][0], step[0][0][1],
                         step[0][1][0], step[0][1][1], step[1][0][0], step[1][0][1], step[1][1][0], step[1][1][1], hips[0][0], hips[0][1], hips[1][0],
                         hips[1][1]);
        }
        if (si.clip >= 0)
        {
            lastClip = si.clip;
            lastRow = si.clockRow;
        }
    }
    std::fclose(f);
}

// Switch blend on a recorded window: the layer model on stage 0 with the blend off and on (aSeconds; 0 = a blend that never runs, the
// mutation of the check). Per clip switch: the largest single-frame step of the hands (orientation in the Spine frame, deg) on the switch
// frame with the blend off, and over the blend frames with it on; the frames outside the blends must be byte-identical.
struct SwitchBlendReplay
{
    int switches = 0, bigCuts = 0, bigCutsHalved = 0, blendFrames = 0, changedOutside = 0;
    float worstOff = 0.0f, worstOn = 0.0f;
    float worstRise = 0.0f; // largest increase of the hand step by a blend on a switch below 90 deg (deg)
};

SwitchBlendReplay ReplaySwitchBlend(const char* aLabel, const PoseTables& aT, const meleepose::Weapon& aW, const std::vector<uint64_t>& aHashes,
                                    const std::vector<int16_t>& aPar, const meleebase::Rig& aKeyRig, const meleepose::LowerRig& aLower,
                                    const std::vector<std::vector<uint8_t>>& aS0, const std::vector<float>& aDt, const std::vector<float>& aFade,
                                    bool aHold, float aSeconds, bool aPrint)
{
    SwitchBlendReplay r;
    int32_t bones[4] = {-1, -1, -1, -1};
    const char* names[4] = {"Spine", "LeftHand", "RightHand", "Hips"};
    for (int k = 0; k < 4; ++k)
        for (size_t i = 0; i < aHashes.size(); ++i)
            if (aHashes[i] == integ::CNameHash(names[k]))
                bones[k] = int32_t(i);
    const meleepose::UpperRig upper = meleepose::ResolveUpperRig(aHashes);
    if (bones[0] < 0 || bones[1] < 0 || bones[2] < 0 || bones[3] < 0 || !upper.ok)
        return r;
    const uint32_t nb = uint32_t(aHashes.size());
    const size_t n = aS0.size();
    std::vector<std::vector<uint8_t>> out[2];
    std::vector<int32_t> switchAt, switchFrom, switchTo;
    std::vector<uint8_t> blending(n, 0);
    // the runtime's first melee step: the reference pose guard on the engine pose (the melee mask is on through the window)
    const meleepose::GuardRig grig = meleepose::ResolveGuardRig(aHashes, aPar);
    std::vector<double> tm(n, 0.0);
    for (size_t f = 1; f < n; ++f)
        tm[f] = tm[f - 1] + double(aDt[f]);
    for (int v = 0; v < 2; ++v)
    {
        layermodel::LayerState ls;
        meleepose::SwitchBlendState sb;
        const std::unique_ptr<meleepose::GuardState> gs = std::make_unique<meleepose::GuardState>();
        for (size_t f = 0; f < n; ++f)
        {
            std::vector<uint8_t> b = aS0[f];
            const float age = gs->have ? float(tm[f] - tm[size_t(gs->ticks)]) : 1e9f;
            meleepose::GuardStep(*gs, grig, b.data(), nb, true, int64_t(f), age);
            math::Quat key[meleebase::kKeyBones];
            const bool haveKeys = meleebase::ReadKeys(b.data(), nb, aKeyRig, key);
            const int32_t clipBefore = ls.step.clip;
            layermodel::LayerCallInfo li;
            layermodel::LayerCall(ls, aT, &aW, haveKeys ? key : nullptr, aFade[f], aDt[f], aHold, false, b.data(), nb, aLower, &li);
            const bool sw = li.step.relocked && clipBefore >= 0 && aFade[f] > 0.0f;
            if (v == 0 && sw)
            {
                switchAt.push_back(int32_t(f));
                switchFrom.push_back(clipBefore);
                switchTo.push_back(li.step.clip);
            }
            if (v == 1)
            {
                if (aFade[f] > 0.0f)
                    blending[f] = meleepose::SwitchBlendStep(sb, b.data(), nb, upper, sw, f > 0 ? aDt[f] : 0.0f, aSeconds) ? 1 : 0;
                else
                    sb = meleepose::SwitchBlendState{}; // the runtime resets with the layer
            }
            out[v].push_back(std::move(b));
        }
    }
    auto handStep = [&](int aV, size_t aF) {
        const SwitchPose a = SwitchPoseOf(out[aV][aF - 1], aPar, bones), b = SwitchPoseOf(out[aV][aF], aPar, bones);
        return std::fmax(AngleBetweenDeg(a.hand[0].q, b.hand[0].q), AngleBetweenDeg(a.hand[1].q, b.hand[1].q));
    };
    if (const char* tr = std::getenv("UTPP_SWITCH_TRACE"); tr && aPrint && std::strcmp(tr, aLabel) == 0)
        for (size_t f = 1; f < n; ++f)
            std::printf("    sb f%3zu blend %d hand step off %6.1f on %6.1f\n", f, blending[f], handStep(0, f), handStep(1, f));
    for (size_t f = 0; f < n; ++f)
    {
        if (blending[f])
            ++r.blendFrames;
        else if (out[0][f] != out[1][f])
            ++r.changedOutside;
    }
    for (size_t i = 0; i < switchAt.size(); ++i)
    {
        const int32_t s = switchAt[i];
        if (s <= 0)
            continue;
        ++r.switches;
        // the switch frame and the blend frames after it, in both runs (the clip's own motion counts on both sides)
        float off = handStep(0, size_t(s)), on = handStep(1, size_t(s));
        int frames = 0;
        for (size_t f = size_t(s); f < n && blending[f]; ++f, ++frames)
            if (f + 1 < n)
            {
                off = std::fmax(off, handStep(0, f + 1));
                on = std::fmax(on, handStep(1, f + 1));
            }
        r.worstOff = std::fmax(r.worstOff, off);
        r.worstOn = std::fmax(r.worstOn, on);
        if (off <= 90.0f && frames > 0)
            r.worstRise = std::fmax(r.worstRise, on - off);
        if (off > 90.0f)
        {
            ++r.bigCuts;
            if (on <= 0.5f * off)
                ++r.bigCutsHalved;
        }
        if (aPrint)
            std::printf("  switch blend %-20s frame %3d %-26s -> %-26s hand step %5.1f -> %5.1f deg over %d blend frames\n", aLabel, s,
                        aT.clipNames[size_t(switchFrom[i])], aT.clipNames[size_t(switchTo[i])], off, on, frames);
    }
    return r;
}

struct TravelMetrics
{
    float entry = 0.0f, full = 0.0f, handback = 0.0f, hipsSpeed = 0.0f;
};

// aPhase per frame: 0 none, 1 entry, 2 full weight, 3 hand-back; aFade per frame: 1 = the layer's fade is below full (the Hips speed
// is judged there; at full weight the clip's own Hips motion passes)
TravelMetrics MeasureTravel(const std::vector<math::Vec3>& aL, const std::vector<math::Vec3>& aR, const std::vector<float>& aHipsZ,
                            const std::vector<float>& aDt, const std::vector<uint8_t>& aPhase, const std::vector<uint8_t>& aFade)
{
    TravelMetrics m;
    const size_t n = aL.size();
    for (int s = 0; s < 2; ++s)
    {
        const std::vector<math::Vec3>& a = s == 0 ? aL : aR;
        float lowest = 1e9f;
        for (const math::Vec3& p : a)
            lowest = std::fmin(lowest, p.z);
        float seg = 0.0f;
        for (size_t i = 1; i < n; ++i)
        {
            const bool ground = a[i].z <= lowest + meleepose::kLiftHeight && a[i - 1].z <= lowest + meleepose::kLiftHeight;
            if (!ground || aPhase[i] == 0 || aPhase[i] != aPhase[i - 1])
            {
                seg = 0.0f;
                continue;
            }
            seg += meleepose::Horizontal(a[i], a[i - 1]);
            float& slot = aPhase[i] == 1 ? m.entry : (aPhase[i] == 2 ? m.full : m.handback);
            slot = std::fmax(slot, seg);
        }
    }
    for (size_t i = 1; i < n; ++i)
        if (aFade[i] && aFade[i - 1] && aDt[i] > 0.0f)
            m.hipsSpeed = std::fmax(m.hipsSpeed, std::fabs(aHipsZ[i] - aHipsZ[i - 1]) / aDt[i]);
    return m;
}

bool IsVariant(const char* aName)
{
    const std::string n(aName);
    auto ends = [&](const char* aSuf) {
        const size_t k = std::strlen(aSuf);
        return n.size() > k && n.compare(n.size() - k, k, aSuf) == 0;
    };
    return ends("_hit") || ends("_deflect");
}

struct ChoiceMetrics
{
    int changes = 0, variantFrames = 0, fullFrames = 0;
    int lowFrames = 0; // frames the full-body fade is full but the layer's own weight is below 0.5 (the legs go back to the engine pose)
    float footTravel = 0.0f;
};

void RunBorderFixture(const std::string& aPath, const PoseTables* aTables)
{
    const std::vector<uint8_t> in = ReadFile(aPath);
    if (in.size() < 12 || std::memcmp(in.data(), "MPB1", 4) != 0)
    {
        Check(false, ("border fixture missing or bad: " + aPath).c_str());
        return;
    }
    size_t o = 4;
    auto rd = [&](void* aDst, size_t aN) {
        if (o + aN > in.size())
            throw 1;
        std::memcpy(aDst, in.data() + o, aN);
        o += aN;
    };
    try
    {
        int32_t nb = 0;
        rd(&nb, 4);
        std::vector<int16_t> par(static_cast<size_t>(nb));
        std::vector<uint64_t> hashes(static_cast<size_t>(nb));
        for (int32_t i = 0; i < nb; ++i)
        {
            char s[33] = {};
            rd(&par[size_t(i)], 2);
            rd(s, 32);
            hashes[size_t(i)] = integ::CNameHash(s);
        }
        int32_t body = 0, nwin = 0;
        rd(&body, 4);
        rd(&nwin, 4);
        const PoseTables& t = aTables[body == 1 ? 1 : 0];
        const meleepose::LowerRig lower = meleepose::ResolveLowerRig(hashes, par);
        const meleebase::Rig keyRig = meleebase::ResolveRig(hashes, par);
        Check(lower.ok == 1 && keyRig.ok == 1, "border fixture: lower and key rig of the recorded catalogue");
        if (!lower.ok || !keyRig.ok)
            return;
        std::vector<int16_t> P(size_t(nb) * 3), Q(size_t(nb) * 4);
        auto fill = [&](std::vector<uint8_t>& aB) {
            for (int32_t i = 0; i < nb; ++i)
                SetRec(aB, uint32_t(i), math::Vec3{P[size_t(i) * 3] * 1e-4f, P[size_t(i) * 3 + 1] * 1e-4f, P[size_t(i) * 3 + 2] * 1e-4f},
                       math::QuatNormalize(math::Quat{Q[size_t(i) * 4] / 32767.0f, Q[size_t(i) * 4 + 1] / 32767.0f,
                                                      Q[size_t(i) * 4 + 2] / 32767.0f, Q[size_t(i) * 4 + 3] / 32767.0f}));
        };
        int judged = 0, beforeOver = 0;
        SwitchBlendReplay sbTot, sbMut;
        float worstAfter = 0.0f, worstHipsAfter = 0.0f;
        std::printf("border fixture %s (body %d)\n", aPath.c_str(), body);
        std::printf("  window             capsule | ground travel cm entry / full / hand-back, Hips speed m/s: recorded -> borders | "
                    "clip choice changes / variant frames / frames below half weight / clip feet travel cm: free clock -> held clock\n");
        for (int32_t w = 0; w < nwin; ++w)
        {
            char label[33] = {}, weapon[33] = {};
            int32_t hold = 0, nf = 0;
            float cap = 0.0f, on = 0.0f, off = 0.0f;
            rd(label, 32);
            rd(weapon, 32);
            rd(&hold, 4);
            rd(&cap, 4);
            rd(&on, 4);
            rd(&off, 4);
            rd(&nf, 4);
            const meleepose::Weapon* wp = t.Find(integ::CNameHash(weapon));
            const size_t frames = static_cast<size_t>(nf);
            std::vector<std::vector<uint8_t>> s0(frames), s1(frames);
            std::vector<float> dt(frames), weight(frames);
            std::vector<int32_t> writing(frames);
            std::vector<uint8_t> phase(size_t(nf), 0);
            double tm = 0.0;
            for (int32_t f = 0; f < nf; ++f)
            {
                rd(&dt[size_t(f)], 4);
                rd(&weight[size_t(f)], 4);
                rd(&writing[size_t(f)], 4);
                tm += f > 0 ? double(dt[size_t(f)]) : 0.0;
                phase[size_t(f)] = tm < on ? 0 : (tm < on + 0.35 ? 1 : (tm < off ? 2 : 3));
                for (std::vector<uint8_t>* st : {&s0[size_t(f)], &s1[size_t(f)]})
                {
                    rd(P.data(), P.size() * 2);
                    rd(Q.data(), Q.size() * 2);
                    st->assign(size_t(nb) * integ::kGameBoneStride, 0);
                    fill(*st);
                }
            }
            // borders: recorded (stage 1) against after the borders on the same written pose
            std::vector<math::Vec3> l0, r0, l1, r1;
            std::vector<float> h0, h1;
            meleepose::BorderState bs;
            for (int32_t f = 0; f < nf; ++f)
            {
                meleepose::LowerModel m;
                meleepose::ComputeLowerModel(s1[size_t(f)].data(), uint32_t(nb), lower, m);
                l0.push_back(m.leg[0][2].p);
                r0.push_back(m.leg[1][2].p);
                h0.push_back(m.hipsLocalZ);
                std::vector<uint8_t> b = s1[size_t(f)];
                meleepose::LowerModel pre;
                const bool havePre = f > 0 && meleepose::ComputeLowerModel(s1[size_t(f) - 1].data(), uint32_t(nb), lower, pre);
                meleepose::BorderStep(bs, b.data(), uint32_t(nb), lower, havePre ? &pre : nullptr, writing[size_t(f)] != 0, weight[size_t(f)],
                                      dt[size_t(f)]);
                meleepose::ComputeLowerModel(b.data(), uint32_t(nb), lower, m);
                l1.push_back(m.leg[0][2].p);
                r1.push_back(m.leg[1][2].p);
                h1.push_back(m.hipsLocalZ);
            }
            std::vector<uint8_t> fadeMask(frames, 0);
            for (size_t f = 0; f < frames; ++f)
                fadeMask[f] = (phase[f] == 1 || phase[f] == 3) && weight[f] < 0.999f ? 1 : 0;
            const TravelMetrics before = MeasureTravel(l0, r0, h0, dt, phase, fadeMask), after = MeasureTravel(l1, r1, h1, dt, phase, fadeMask);
            // the layer as the game runs it (LayerCall: match on the engine keys, write, borders) on the engine pose of stage 0 with the
            // modelled fade; the state is carried across the windows of one fixture only through the window's own frames
            std::vector<math::Vec3> lm, rm;
            std::vector<float> hm, gm;
            std::vector<uint8_t> modelMask(frames, 0);
            if (wp)
            {
                layermodel::LayerState ls;
                const char* trace = std::getenv("UTPP_BORDER_TRACE");
                const bool tr = trace && std::strcmp(trace, label) == 0;
                for (int32_t f = 0; f < nf; ++f)
                {
                    std::vector<uint8_t> b = s0[size_t(f)];
                    math::Quat key[meleebase::kKeyBones];
                    const bool haveKeys = meleebase::ReadKeys(b.data(), uint32_t(nb), keyRig, key);
                    layermodel::LayerCallInfo li;
                    layermodel::LayerCall(ls, t, wp, haveKeys ? key : nullptr, weight[size_t(f)], dt[size_t(f)], hold != 0, false, b.data(),
                                         uint32_t(nb), lower, &li);
                    meleepose::LowerModel m;
                    meleepose::ComputeLowerModel(b.data(), uint32_t(nb), lower, m);
                    lm.push_back(m.leg[0][2].p);
                    rm.push_back(m.leg[1][2].p);
                    hm.push_back(m.hipsLocalZ);
                    gm.push_back(li.written);
                    modelMask[size_t(f)] = (phase[size_t(f)] == 1 || phase[size_t(f)] == 3) && li.written < 0.999f ? 1 : 0;
                    if (tr)
                    {
                        meleepose::LowerModel m0;
                        meleepose::ComputeLowerModel(s0[size_t(f)].data(), uint32_t(nb), lower, m0);
                        std::printf("    f%3d dt %.4f ph%d fade %.2f gate %.2f w %.2f rl%d m%d bact%d pl%d feet %d%d rel %.2f %.2f floor %.3f | hipsZ s0 %.3f s1 %.3f model %.3f | L s0 %6.3f %6.3f %6.3f "
                                    "s1 %6.3f %6.3f %6.3f model %6.3f %6.3f %6.3f | R s0 %6.3f %6.3f %6.3f s1 %6.3f %6.3f %6.3f model %6.3f %6.3f %6.3f\n",
                                    f, dt[size_t(f)], phase[size_t(f)], weight[size_t(f)], li.step.gate, li.written, li.step.relocked ? 1 : 0, li.step.matched ? 1 : 0,
                                    ls.border.active ? 1 : 0, li.border.planted, int(ls.border.foot[0].mode), int(ls.border.foot[1].mode),
                                    ls.border.foot[0].release, ls.border.foot[1].release, ls.border.floorZ, m0.hipsLocalZ, h0[size_t(f)], m.hipsLocalZ, m0.leg[0][2].p.x,
                                    m0.leg[0][2].p.y, m0.leg[0][2].p.z, l0[size_t(f)].x, l0[size_t(f)].y, l0[size_t(f)].z, m.leg[0][2].p.x,
                                    m.leg[0][2].p.y, m.leg[0][2].p.z, m0.leg[1][2].p.x, m0.leg[1][2].p.y, m0.leg[1][2].p.z, r0[size_t(f)].x,
                                    r0[size_t(f)].y, r0[size_t(f)].z, m.leg[1][2].p.x, m.leg[1][2].p.y, m.leg[1][2].p.z);
                    }
                }
            }
            if (wp)
            {
                const SwitchBlendReplay a = ReplaySwitchBlend(label, t, *wp, hashes, par, keyRig, lower, s0, dt, weight, hold != 0,
                                                              meleepose::kSwitchBlendSeconds, true);
                const SwitchBlendReplay m = ReplaySwitchBlend(label, t, *wp, hashes, par, keyRig, lower, s0, dt, weight, hold != 0, 0.0f, false);
                for (int v = 0; v < 2; ++v)
                {
                    SwitchBlendReplay& d = v == 0 ? sbTot : sbMut;
                    const SwitchBlendReplay& s = v == 0 ? a : m;
                    d.switches += s.switches;
                    d.bigCuts += s.bigCuts;
                    d.bigCutsHalved += s.bigCutsHalved;
                    d.blendFrames += s.blendFrames;
                    d.changedOutside += s.changedOutside;
                    d.worstOff = std::fmax(d.worstOff, s.worstOff);
                    d.worstOn = std::fmax(d.worstOn, s.worstOn);
                    d.worstRise = std::fmax(d.worstRise, s.worstRise);
                }
            }
            if (const char* sw = std::getenv("UTPP_SWITCH_TABLE"); sw && wp)
                SwitchTable(sw, label, t, *wp, hashes, par, keyRig, s0, s1, dt, weight, hold != 0);
            const TravelMetrics model = wp ? MeasureTravel(lm, rm, hm, dt, phase, modelMask) : TravelMetrics{};
            std::printf("  %-16s model of the game on stage 0: %5.1f %5.1f %5.1f %4.2f\n", label, model.entry * 100.0f, model.full * 100.0f,
                        model.handback * 100.0f, model.hipsSpeed);
            // clip choice: free clock (no held state, variants taken as matched) and held clock
            ChoiceMetrics cm[2];
            for (int v = 0; v < 2 && wp; ++v)
            {
                meleepose::State st;
                int32_t lastClip = -1;
                std::vector<math::Vec3> cl, cr;
                std::vector<float> ch, cdt;
                std::vector<uint8_t> cph;
                for (int32_t f = 0; f < nf; ++f)
                {
                    math::Quat key[meleebase::kKeyBones];
                    const bool haveKeys = meleebase::ReadKeys(s0[size_t(f)].data(), uint32_t(nb), keyRig, key);
                    LowerPose pose;
                    meleepose::StepInfo si;
                    const float wgt = meleepose::Step(st, t, wp, haveKeys ? key : nullptr, writing[size_t(f)] ? 1.0f : 0.0f, dt[size_t(f)], pose,
                                                      &si, v == 1 && hold != 0, v == 1);
                    if (weight[size_t(f)] >= 0.999f && wgt < 0.5f)
                        ++cm[v].lowFrames;
                    if (!(wgt > 0.99f) || weight[size_t(f)] < 0.999f || si.clip < 0)
                        continue;
                    ++cm[v].fullFrames;
                    if (lastClip >= 0 && si.clip != lastClip)
                        ++cm[v].changes;
                    lastClip = si.clip;
                    if (IsVariant(t.clipNames[size_t(si.clip)]))
                        ++cm[v].variantFrames;
                    std::vector<uint8_t> b = s0[size_t(f)];
                    meleepose::WriteLower(b.data(), uint32_t(nb), lower, pose, 1.0f);
                    meleepose::LowerModel m;
                    meleepose::ComputeLowerModel(b.data(), uint32_t(nb), lower, m);
                    cl.push_back(m.leg[0][2].p);
                    cr.push_back(m.leg[1][2].p);
                    ch.push_back(m.hipsLocalZ);
                    cdt.push_back(dt[size_t(f)]);
                    cph.push_back(2);
                }
                if (!cl.empty())
                    cm[v].footTravel = MeasureTravel(cl, cr, ch, cdt, cph, std::vector<uint8_t>(cph.size(), 0)).full;
            }
            const bool stands = cap < 0.05f;
            std::printf("  %-16s %5.2f m%s | %5.1f %5.1f %5.1f %4.2f -> %5.1f %5.1f %5.1f %4.2f | %d %d %d %5.1f -> %d %d %d %5.1f%s\n", label, cap,
                        stands ? " " : "*", before.entry * 100.0f, before.full * 100.0f, before.handback * 100.0f, before.hipsSpeed,
                        after.entry * 100.0f, after.full * 100.0f, after.handback * 100.0f, after.hipsSpeed, cm[0].changes, cm[0].variantFrames, cm[0].lowFrames,
                        cm[0].footTravel * 100.0f, cm[1].changes, cm[1].variantFrames, cm[1].lowFrames, cm[1].footTravel * 100.0f, hold ? " (held state)" : "");
            if (!stands)
                continue; // the plants live in the character frame: judged where the capsule stands
            ++judged;
            if (std::fmax(before.entry, std::fmax(before.full, before.handback)) > 0.03f || before.hipsSpeed > meleepose::kHipsSpeed + 0.02f)
                ++beforeOver;
            worstAfter = std::fmax(worstAfter, std::fmax(after.entry, std::fmax(after.full, after.handback)));
            worstHipsAfter = std::fmax(worstHipsAfter, after.hipsSpeed);
            if (wp)
            {
                Check(cm[1].variantFrames <= cm[0].variantFrames, "clip choice: no more _hit / _deflect frames than the free clock", cm[1].variantFrames);
                if (hold)
                {
                    Check(cm[1].footTravel <= cm[0].footTravel + 0.005f, "clip choice: held state, clip feet travel not above the free clock (m)",
                          cm[1].footTravel);
                    Check(cm[1].lowFrames <= cm[0].lowFrames, "clip choice: held state, frames below half weight not above the free clock",
                          cm[1].lowFrames);
                }
            }
        }
        std::printf("  (* capsule moved: reported only)\n");
        std::printf("  switch blend: %d switches, %d cuts above 90 deg, %d halved, worst hand step %.1f -> %.1f deg, %d blend frames, %d other "
                    "frames changed; blend that never runs: %d of %d halved\n",
                    sbTot.switches, sbTot.bigCuts, sbTot.bigCutsHalved, sbTot.worstOff, sbTot.worstOn, sbTot.blendFrames, sbTot.changedOutside,
                    sbMut.bigCutsHalved, sbMut.bigCuts);
        Check(sbTot.changedOutside == 0, "switch blend: frames outside the blends unchanged", sbTot.changedOutside);
        // a strike moves the hands by up to about 15 deg per frame: a blend on a smaller switch may shift that motion by less than this
        Check(sbTot.worstRise <= 10.0f, "switch blend: switches below 90 deg, hand step raised by at most 10 deg", sbTot.worstRise);
        if (sbTot.bigCuts > 0)
        {
            Check(sbTot.bigCutsHalved == sbTot.bigCuts, "switch blend: every hand cut above 90 deg at least halved", sbTot.bigCuts - sbTot.bigCutsHalved);
            Check(sbMut.bigCutsHalved < sbMut.bigCuts, "switch blend: the check catches a blend that never runs", sbMut.bigCutsHalved);
        }
        Check(judged > 0, "border fixture: windows with a standing capsule", judged);
        Check(beforeOver > 0, "border fixture: the recording shows the slides / Hips speed the borders remove", beforeOver);
        Check(worstAfter <= 0.03f, "borders: foot ground travel at entry, full weight and hand-back <= 3 cm (m)", worstAfter);
        Check(worstHipsAfter <= meleepose::kHipsSpeed + 0.02f, "borders: Hips vertical speed in entry and hand-back (m/s)", worstHipsAfter);
    }
    catch (int)
    {
        Check(false, ("border fixture truncated: " + aPath).c_str());
    }
}

} // namespace

int main(int aArgc, char** aArgv)
{
    if (aArgc < 2)
    {
        std::printf("usage: utpp_melee_pose_test <data dir>\n");
        return 2;
    }
    const std::string dir = aArgv[1];
    Totals tot;
    for (const char* body : {"female", "male"})
    {
        PoseTables t;
        const bool ok = meleepose::ParseTables(ReadFile(dir + "/utpp_melee_pose_" + body + ".bin"), t);
        Check(ok, "load");
        if (!ok)
            continue;
        TestParse(t);
        SetForeign(t);
        const char* names[] = {"w_melee_fists", "w_melee_knife", "w_melee_katana", "w_melee_one_hand_blunt", "w_melee_baton"};
        for (const char* n : names)
        {
            const meleepose::Weapon* w = t.Find(integ::CNameHash(n));
            if (!w)
                continue;
            JudgeWeapon(t, *w, body, n, tot);
            TestOff(t, *w);
        }
    }
    std::printf("weapon runs %d  worst vs the played clip: rot %.2f deg hips %.2f mm\n", tot.strikes, tot.worstRot, tot.worstHips * 1000.0f);
    // the lower body follows the clip (the old behaviour leaves the engine's base: tens of degrees, hips several cm off)
    Check(tot.strikes == 10, "weapon runs (5 weapons x 2 bodies)", tot.strikes);
    TestGuardSynthetic();
    TestSwitchBlendSynthetic();
    TestGuardRecorded(dir + "/melee_pose_guard_female.bin");
    // further recordings by magic: guard fixtures (e.g. a whole run cut with --all; reported, judged only for frames that must not change)
    // and border / clip choice fixtures
    PoseTables both[2];
    meleepose::ParseTables(ReadFile(dir + "/utpp_melee_pose_female.bin"), both[0]);
    meleepose::ParseTables(ReadFile(dir + "/utpp_melee_pose_male.bin"), both[1]);
    for (int i = 2; i < aArgc; ++i)
    {
        const std::vector<uint8_t> head = ReadFile(aArgv[i]);
        if (head.size() >= 4 && std::memcmp(head.data(), "MPB1", 4) == 0)
        {
            RunBorderFixture(aArgv[i], both);
            continue;
        }
        const GuardFixtureResult r = RunGuardFixture(aArgv[i], true);
        std::printf("extra guard fixture %s: frames %d, reference frames in %d, replaced %d, passed %d, out %d, other frames changed %d, "
                    "hand steps >100 deg %d -> %d\n",
                    aArgv[i], r.frames, r.signatureIn, r.held, r.passed, r.signatureOut, r.changedOther, r.popsIn, r.popsOut);
        Check(r.changedOther == 0 && r.signatureOut <= r.passed, "extra guard fixture: only reference frames change", r.changedOther);
    }
    std::printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
