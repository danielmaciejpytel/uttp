// Melee full pose (mm/MeleePose.hpp): data file parsing and synthetic replays per weapon. The clip's upper body (key bones, played at
// 60 fps between the 30 fps rows) runs on a foreign lower body; the layer must write the clip's own lower body time-synchronised with
// the clip, continuously at entry, between strikes and at the hand-back, and hold over a one-frame reference pose.
// usage: utpp_melee_pose_test <dir with utpp_melee_pose_female.bin / utpp_melee_pose_male.bin>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../src/mm/MeleePose.hpp"

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
    std::printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
