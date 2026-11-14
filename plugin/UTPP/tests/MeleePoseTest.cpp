// Melee full pose (mm/MeleePose.hpp): data file parsing and synthetic replays per weapon. The clip's upper body (key bones, played at
// 60 fps between the 30 fps rows) runs on a foreign lower body; the layer must write the clip's own lower body time-synchronised with
// the clip, continuously at entry, between strikes and at the hand-back, and hold over a one-frame reference pose.
// Reference pose guard (mm/MeleePoseGuard.hpp): synthetic rigs and the recorded engine poses of <dir>/melee_pose_guard_female.bin.
// usage: utpp_melee_pose_test <dir with utpp_melee_pose_female.bin / utpp_melee_pose_male.bin / melee_pose_guard_female.bin> [<fixture> ...]
// (further fixtures by their magic: "MPG1" reference pose guard, "MPB1" borders and clip choice on recorded frames)
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
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
