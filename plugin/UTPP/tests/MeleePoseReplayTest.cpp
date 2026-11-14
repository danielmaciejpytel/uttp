// Melee full pose on recorded in-game fists sequences, through the production glue (MeleeBaseReplay.hpp with UTPP_REPLAY_FULL_POSE:
// OnArmed, LowerBodyKeepWeight, CaptureEngineKeys, BlendIntoRecords, meleepose::Apply, meleebase::Apply in the order of FkHook.cpp).
// Each fixture is replayed with the layer off (the chest correction alone) and on. Measured on the frames where the full pose writes
// at full weight: Hips height and the chest (Spine3) orientation against the clip at the layer's clock; on every frame the largest
// single-frame step of the Hips, both feet and the chest.
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
};

struct RefFrame
{
    int32_t clip = -1;
    float clock = 0.0f;
    bool judged = false;
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
    for (size_t i = 0; i < rp.frames.size(); ++i)
    {
        rp.Step(i);
        const std::vector<aim::Xf> m = rp.Model();
        ++r.frames;
        if (havePrev && rp.frames[i].dt >= 0.0f)
        {
            r.stepHips = std::fmax(r.stepHips, aim::Length(m[size_t(hips)].p - prev[size_t(hips)].p));
            r.stepFoot = std::fmax(r.stepFoot, std::fmax(aim::Length(m[size_t(lf)].p - prev[size_t(lf)].p), aim::Length(m[size_t(rf)].p - prev[size_t(rf)].p)));
            r.stepChest = std::fmax(r.stepChest, AngleDeg(m[size_t(chest)].q, prev[size_t(chest)].q));
        }
        prev2 = prev.empty() ? m : prev;
        prev = m;
        havePrev = true;
        if (aFullPose)
        {
            const utpp::meleepose::LastStep ls = utpp::meleepose::GetLastStep();
            if (ls.held)
                ++r.held;
            // judged where the full pose writes at full weight
            aRef[i].judged = rp.frames[i].lowerScale < 0.01f && ls.clip >= 0 && size_t(ls.clip) < aT.clipFirstRow.size() && ls.weight > 0.99f && ls.matched;
            aRef[i].clip = ls.clip;
            aRef[i].clock = ls.clockRow;
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
            const float hz = aim::ReadLocal(rp.records.data(), uint32_t(hips)).p.z;
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
        Check(on.hipsErrMax < 0.02f, "hips height vs the clip at the clock (m)", on.hipsErrMax);
        Check(on.chestErrMax < off.chestErrMax, "chest vs the clip: full pose below the chest correction alone (max deg)", on.chestErrMax);
        // continuity: no step beyond the off run's own (+ 2 cm / 5 deg for the clip's own faster legs)
        Check(on.stepHips <= off.stepHips + 0.02f, "largest hips step vs the off run (m)", on.stepHips - off.stepHips);
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
