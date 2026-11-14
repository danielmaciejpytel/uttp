// Melee full pose on recorded in-game fists sequences, through the production glue (MeleeBaseReplay.hpp with UTPP_REPLAY_FULL_POSE:
// OnArmed, LowerBodyKeepWeight, CaptureEngineKeys, BlendIntoRecords, meleepose::Apply, meleebase::Apply in the order of FkHook.cpp).
// Each fixture is replayed with the layer off (the chest correction alone) and on. Measured on the frames where the full pose writes
// at full weight: the chest (Spine3) orientation against the clip at the layer's clock and the feet against the clip's own; the Hips
// height against the clip outside a relock cross-fade and against the decaying jump inside one; on every frame the single-frame steps of the Hips (against
// the off run or the clip's own step at the played rows), both feet and the chest.
// usage: utpp_melee_pose_replay_test <pose table dir> <fixture.bin> [<fixture.bin> ...]
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../src/mm/MeleePose.hpp"
#include "MeleeBaseReplay.hpp"

using namespace utpp::mm;
using utpp::replay::Replay;

namespace
{
int g_fail = 0;
// the borders' Hips offset decays at kHipsSpeed: an offset of about 7 cm is gone within this time at full weight
constexpr float kSettleSeconds = 0.25f;

void Check(bool aOk, const char* aWhat, double aValue)
{
    std::printf("[%s] %s -- %.3f\n", aOk ? "PASS" : "FAIL", aWhat, aValue);
    if (!aOk)
        ++g_fail;
}

float AngleDeg(const math::Quat& aA, const math::Quat& aB)
{
    float d = std::fabs(math::QuatDot(math::QuatNormalize(aA), math::QuatNormalize(aB)));
    d = d > 1.0f ? 1.0f : d;
    return 2.0f * std::acos(d) * 57.2957795f;
}

struct Result
{
    int frames = 0, full = 0, held = 0;
    float hipsErrMax = 0.0f, chestErrMax = 0.0f, chestErrSum = 0.0f;
    float stepHips = 0.0f, stepFoot = 0.0f, stepChest = 0.0f;
    float stepFootJudged = 0.0f; // largest foot step between two consecutive judged frames (full weight)
    float footZMin = 1e9f, footZMax = -1e9f;
    float ownFootStep = 0.0f; // the clip's own largest foot step between judged frames (the rig's bone lengths, the clip's rotations)
    float hipsOffsetMax = 0.0f;   // largest Hips height offset of the borders (shown - written)
    float fadeHipsSpeed = 0.0f;   // fastest vertical Hips speed (shown) on frames inside the full-body fades
    float settledOffsetMax = 0.0f; // largest Hips height offset at full weight once the fade in has been over for kSettleSeconds
    int settledFrames = 0;
    std::vector<float> stepHipsAt; // per frame: the Hips step against the previous frame (0 at a window start)
    std::vector<float> ownHipsAt;  // per frame (layer on): the clip's own Hips step between the played (clip, clock) of this and the previous frame
    int xfadeFrames = 0;           // frames at full weight inside a relock cross-fade (the clip's share below 1)
    float xfadeErrMax = 0.0f, xfadeExcessMax = -1.0f;
};

struct RefFrame
{
    int32_t clip = -1;
    float clock = 0.0f;
    bool judged = false;
    bool fullShare = false; // the clip at full share (outside a relock cross-fade): the Hips height is judged against the clip itself
};

// model position of a foot (Hips -> UpLeg -> Leg -> Foot) for a clip lower pose on the rig's own bone translations
math::Vec3 ClipFoot(const Replay& aR, const meleepose::LowerPose& aP, bool aLeft)
{
    const char* names[3] = {aLeft ? "LeftUpLeg" : "RightUpLeg", aLeft ? "LeftLeg" : "RightLeg", aLeft ? "LeftFoot" : "RightFoot"};
    const int q[3] = {aLeft ? 2 : 5, aLeft ? 3 : 6, aLeft ? 4 : 7};
    aim::Xf x{aP.hips, aP.q[0]};
    for (int k = 0; k < 3; ++k)
    {
        const aim::Xf l = aim::ReadLocal(aR.records.data(), uint32_t(aR.Find(names[k])));
        x = aim::Mul(x, aim::Xf{l.p, aP.q[q[k]]});
    }
    return x.p;
}

// aFullPose: the layer on; it records the judged frames and the layer's clip / clock in aRef, the off run is judged on the same frames
Result Run(const std::string& aFixture, bool aFullPose, const meleepose::PoseTables& aT, std::vector<RefFrame>& aRef)
{
    Result r;
    Replay rp;
    if (!rp.Load(aFixture))
    {
        std::printf("cannot load %s\n", aFixture.c_str());
        ++g_fail;
        return r;
    }
    utpp::meleepose::SetEnabled(aFullPose);
    utpp::meleepose::SetWeapon(0);
    rp.Arm();
    const int hips = rp.Find("Hips"), lf = rp.Find("LeftFoot"), rf = rp.Find("RightFoot"), chest = rp.Find("Spine3");
    bool havePrev = false;
    std::vector<aim::Xf> prev, prev2; // prev2: the frame before this one (prev is overwritten before the judged block)
    if (aFullPose)
        aRef.assign(rp.frames.size(), RefFrame{});
    RefFrame lastJudged;
    float lastHipsZ = 0.0f;
    float fullFor = 0.0f; // time at full weight (recorded lowerScale < 0.01 and the layer's own weight full) since the last fade frame
    int32_t prevClip = -1;
    float prevClock = 0.0f, lastWrittenZ = NAN, lastClipShare = 1.0f, jumpZ = NAN;
    for (size_t i = 0; i < rp.frames.size(); ++i)
    {
        rp.Step(i);
        {
            const float hzShown = aim::ReadLocal(rp.records.data(), uint32_t(hips)).p.z;
            const float ls = rp.frames[i].lowerScale;
            if (rp.frames[i].dt > 0.0f && i > 0 && ls > 0.01f && ls < 0.99f)
                r.fadeHipsSpeed = std::fmax(r.fadeHipsSpeed, std::fabs(hzShown - lastHipsZ) / rp.frames[i].dt);
            lastHipsZ = hzShown;
            if (aFullPose)
            {
                const utpp::meleepose::LastStep st = utpp::meleepose::GetLastStep();
                fullFor = ls < 0.01f && st.weight >= 0.999f ? fullFor + std::fmax(rp.frames[i].dt, 0.0f) : 0.0f;
                const float off = std::fabs(st.hipsOffset);
                r.hipsOffsetMax = std::fmax(r.hipsOffsetMax, off);
                if (fullFor > kSettleSeconds)
                {
                    r.settledOffsetMax = std::fmax(r.settledOffsetMax, off);
                    ++r.settledFrames;
                }
            }
        }
        const std::vector<aim::Xf> m = rp.Model();
        ++r.frames;
        float stepH = 0.0f, ownH = 0.0f;
        if (aFullPose)
        {
            const utpp::meleepose::LastStep ls = utpp::meleepose::GetLastStep();
            const bool have = ls.clip >= 0 && size_t(ls.clip) < aT.clipFirstRow.size();
            if (have && prevClip >= 0 && rp.frames[i].dt >= 0.0f)
                ownH = aim::Length(meleepose::SamplePose(aT, uint32_t(ls.clip), ls.clockRow).hips - meleepose::SamplePose(aT, uint32_t(prevClip), prevClock).hips);
            prevClip = have ? ls.clip : -1;
            prevClock = ls.clockRow;
        }
        if (havePrev && rp.frames[i].dt >= 0.0f)
        {
            stepH = aim::Length(m[size_t(hips)].p - prev[size_t(hips)].p);
            r.stepHips = std::fmax(r.stepHips, stepH);
            r.stepFoot = std::fmax(r.stepFoot, std::fmax(aim::Length(m[size_t(lf)].p - prev[size_t(lf)].p), aim::Length(m[size_t(rf)].p - prev[size_t(rf)].p)));
            r.stepChest = std::fmax(r.stepChest, AngleDeg(m[size_t(chest)].q, prev[size_t(chest)].q));
        }
        r.stepHipsAt.push_back(stepH);
        r.ownHipsAt.push_back(ownH);
        prev2 = prev.empty() ? m : prev;
        prev = m;
        havePrev = true;
        if (aFullPose)
        {
            const utpp::meleepose::LastStep ls = utpp::meleepose::GetLastStep();
            if (ls.held)
                ++r.held;
            // judged where the full pose writes at full weight; the Hips height against the clip only outside a relock cross-fade
            const bool full = rp.frames[i].lowerScale < 0.01f && ls.clip >= 0 && size_t(ls.clip) < aT.clipFirstRow.size() && ls.weight > 0.99f && ls.matched;
            aRef[i].judged = full;
            aRef[i].fullShare = full && ls.clipShare > 0.99f;
            aRef[i].clip = ls.clip;
            aRef[i].clock = ls.clockRow;
            // a relock cross-fade carries the difference between the pose written before the relock and the clip at the relock and lets it
            // decay: the Hips height stays within (1 - the clip's share) x that jump (+ 0.02 m) of the clip as it plays
            const float written = aim::ReadLocal(rp.records.data(), uint32_t(hips)).p.z - ls.hipsOffset;
            if (rp.frames[i].dt < 0.0f)
            {
                lastWrittenZ = NAN;
                lastClipShare = 1.0f;
                jumpZ = NAN;
            }
            const float clipZ = ls.clip >= 0 && size_t(ls.clip) < aT.clipFirstRow.size() ? meleepose::SamplePose(aT, uint32_t(ls.clip), ls.clockRow).hips.z : NAN;
            if (ls.clipShare < lastClipShare - 1e-4f || (ls.clipShare < 0.999f && std::isnan(jumpZ)))
                jumpZ = std::fabs(clipZ - lastWrittenZ); // a relock started (the clip's share dropped): the jump it found
            if (full && ls.clipShare <= 0.99f && !std::isnan(jumpZ))
            {
                const float err = std::fabs(written - clipZ);
                ++r.xfadeFrames;
                r.xfadeErrMax = std::fmax(r.xfadeErrMax, err);
                r.xfadeExcessMax = std::fmax(r.xfadeExcessMax, err - ((1.0f - ls.clipShare) * jumpZ + 0.02f));
            }
            lastWrittenZ = written;
            lastClipShare = ls.clipShare;
        }
        if (i < aRef.size() && aRef[i].judged)
        {
            const RefFrame& ls = aRef[i];
            ++r.full;
            const meleepose::LowerPose clip = meleepose::SamplePose(aT, uint32_t(ls.clip), ls.clock);
            if (lastJudged.judged && i > 0 && aRef[i - 1].judged)
            {
                const meleepose::LowerPose was = meleepose::SamplePose(aT, uint32_t(lastJudged.clip), lastJudged.clock);
                r.ownFootStep = std::fmax(r.ownFootStep, std::fmax(aim::Length(ClipFoot(rp, clip, true) - ClipFoot(rp, was, true)),
                                                                   aim::Length(ClipFoot(rp, clip, false) - ClipFoot(rp, was, false))));
            }
            if (i > 0 && aRef[i - 1].judged)
                r.stepFootJudged = std::fmax(r.stepFootJudged, std::fmax(aim::Length(m[size_t(lf)].p - prev2[size_t(lf)].p),
                                                                         aim::Length(m[size_t(rf)].p - prev2[size_t(rf)].p)));
            lastJudged = ls;
            // the layer's written Hips height (the shown height minus the borders' offset, which eases a height change in the fades)
            const float hz = aim::ReadLocal(rp.records.data(), uint32_t(hips)).p.z - (aFullPose ? utpp::meleepose::GetLastStep().hipsOffset : 0.0f);
            if (aRef[i].fullShare)
                r.hipsErrMax = std::fmax(r.hipsErrMax, std::fabs(hz - clip.hips.z));
            // the clip's chest (model orientation) at the clock row
            const uint32_t i0 = uint32_t(ls.clock);
            const math::Quat c = math::QuatNormalize(meleebase::DecodeQuat(aT.keys + size_t(i0) * meleebase::kRowInts + meleebase::kKeyBones * 4));
            const float ce = AngleDeg(m[size_t(chest)].q, c);
            r.chestErrMax = std::fmax(r.chestErrMax, ce);
            r.chestErrSum += ce;
            r.footZMin = std::fmin(r.footZMin, std::fmin(m[size_t(lf)].p.z, m[size_t(rf)].p.z));
            r.footZMax = std::fmax(r.footZMax, std::fmax(m[size_t(lf)].p.z, m[size_t(rf)].p.z));
        }
    }
    return r;
}

std::vector<uint8_t> ReadFile(const std::string& aPath)
{
    std::ifstream f(aPath, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
} // namespace

int main(int aArgc, char** aArgv)
{
    if (aArgc < 3)
    {
        std::printf("usage: utpp_melee_pose_replay_test <pose table dir> <fixture.bin> ...\n");
        return 2;
    }
    const std::string dir = std::string(aArgv[1]) + "/";
    utpp::meleepose::Load(std::wstring(dir.begin(), dir.end()));
    meleepose::PoseTables tables[2];
    meleepose::ParseTables(ReadFile(dir + "utpp_melee_pose_female.bin"), tables[0]);
    meleepose::ParseTables(ReadFile(dir + "utpp_melee_pose_male.bin"), tables[1]);
    for (int a = 2; a < aArgc; ++a)
    {
        Replay probe;
        if (!probe.Load(aArgv[a]))
        {
            Check(false, "fixture loads", 0.0);
            continue;
        }
        const meleepose::PoseTables& t = tables[probe.body == 1 ? 1 : 0];
        std::vector<RefFrame> ref;
        const Result on = Run(aArgv[a], true, t, ref);
        const Result off = Run(aArgv[a], false, t, ref);
        std::printf("%s (body %d, %d frames)\n", aArgv[a], probe.body, on.frames);
        std::printf("  off: judged %d | hips z vs clip max %.0f mm | chest vs clip max %.1f mean %.1f deg | steps hips %.0f mm feet %.0f mm chest %.1f deg\n",
                    off.full, off.hipsErrMax * 1000.0f, off.chestErrMax, off.full ? off.chestErrSum / float(off.full) : 0.0f, off.stepHips * 1000.0f,
                    off.stepFoot * 1000.0f, off.stepChest);
        std::printf("  on:  judged %d (held %d) | hips z vs clip max %.0f mm | chest vs clip max %.1f mean %.1f deg | steps hips %.0f mm feet %.0f mm chest %.1f deg | feet z %.0f..%.0f mm, clip's own foot step %.0f mm\n",
                    on.full, on.held, on.hipsErrMax * 1000.0f, on.chestErrMax, on.full ? on.chestErrSum / float(on.full) : 0.0f, on.stepHips * 1000.0f,
                    on.stepFoot * 1000.0f, on.stepChest, on.footZMin * 1000.0f, on.footZMax * 1000.0f, on.ownFootStep * 1000.0f);
        Check(on.full > 20, "frames written at full weight", on.full);
        Check(on.hipsErrMax < 0.02f, "hips height vs the clip at the clock, frames writing the clip at full share (m)", on.hipsErrMax);
        std::printf("  relock cross-fades: %d frames at full weight, hips height vs the clip max %.0f mm\n", on.xfadeFrames, on.xfadeErrMax * 1000.0f);
        Check(on.xfadeExcessMax <= 0.0f, "hips height in a relock cross-fade: within (1 - the clip's share) x the jump at the relock + 0.02 m (excess, m)",
              on.xfadeExcessMax);
        // inside the full-body fades the shown Hips height moves at most at the borders' speed limit
        std::printf("  borders: Hips offset max %.0f mm | Hips vertical speed in the fades: on %.2f m/s, off %.2f m/s\n", on.hipsOffsetMax * 1000.0f,
                    on.fadeHipsSpeed, off.fadeHipsSpeed);
        Check(on.fadeHipsSpeed <= meleepose::kHipsSpeed + 0.02f, "borders: Hips vertical speed in the fades (m/s)", on.fadeHipsSpeed);
        std::printf("  borders: Hips offset at full weight %.2f s after a fade: max %.1f mm over %d frames\n", kSettleSeconds,
                    on.settledOffsetMax * 1000.0f, on.settledFrames);
        Check(on.settledFrames > 0 && on.settledOffsetMax <= 0.001f, "borders: Hips offset gone at full weight after a fade (m)",
              on.settledOffsetMax);
        Check(on.chestErrMax < off.chestErrMax, "chest vs the clip: full pose below the chest correction alone (max deg)", on.chestErrMax);
        // continuity: no step beyond the off run's own (+ 2 cm / 5 deg for the clip's own faster legs)
        // the Hips may move as fast as the off run or as the clip itself between the rows the layer played (+ 2 cm), frame by frame
        float hipsExcess = -1.0f, hipsRef = 0.0f, hipsOn = 0.0f;
        size_t hipsAt = 0;
        for (size_t i = 0; i < on.stepHipsAt.size() && i < off.stepHipsAt.size(); ++i)
        {
            const float allowed = std::fmax(off.stepHipsAt[i], on.ownHipsAt[i]);
            if (on.stepHipsAt[i] - allowed > hipsExcess)
            {
                hipsExcess = on.stepHipsAt[i] - allowed;
                hipsRef = allowed;
                hipsOn = on.stepHipsAt[i];
                hipsAt = i;
            }
        }
        std::printf("  hips: largest step %.0f mm (off run %.0f mm); worst frame %d against its reference: step %.0f mm, reference %.0f mm\n",
                    on.stepHips * 1000.0f, off.stepHips * 1000.0f, int(hipsAt), hipsOn * 1000.0f, hipsRef * 1000.0f);
        Check(hipsExcess <= 0.02f, "hips step vs the off run or the clip's own step at the played rows, per frame (m)", hipsExcess);
        // feet at full weight: no faster than the clip's own feet (+ 1 cm). The fade in / out (the recorded lowerScale) moves the feet
        // from the locomotion stance to the clip's and back without pinning them: reported, not judged
        Check(on.stepFootJudged <= on.ownFootStep + 0.01f, "largest foot step at full weight vs the clip's own (m)", on.stepFootJudged - on.ownFootStep);
        std::printf("  feet: largest step anywhere %.0f mm (off run %.0f mm), at full weight %.0f mm\n", on.stepFoot * 1000.0f, off.stepFoot * 1000.0f,
                    on.stepFootJudged * 1000.0f);
        Check(on.stepChest <= off.stepChest + 5.0f, "largest chest step vs the off run (deg)", on.stepChest - off.stepChest);
    }
    const utpp::meleepose::Counters c = utpp::meleepose::GetCounters();
    std::printf("layer cost: mean %.2f us, max %.2f us over %llu calls (debug-free release build of the test)\n", c.meanMicros, c.maxMicros,
                static_cast<unsigned long long>(c.calls));
    std::printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
