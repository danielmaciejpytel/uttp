// Offline test of the PRODUCTION glue of the melee chest base correction (src/mm/MeleeBase.cpp through tests/MeleeBaseReplay.hpp: OnArmed,
// SetArmedBody, LowerBodyKeepWeight / LowerBodyReset, BlendIntoRecords, Apply in the order of FkHook.cpp) on two small recorded excerpts
// (tests/data/melee_base_male.bin / melee_base_female.bin: one standing combo + one walking window of the 2026-11-12 23:3x pose recordings, made
// by tools/melee/fist_base_fixture.py). Not shipped. Usage: utpp_melee_base_glue_test <tests/data dir>
// Covers: ONLY the armed body's table is used (Male recording: no Female row, and the reverse), the punch direction against the clip's
// reference at the matched time (p95), the jitter of the applied rotation (frame to frame, A-B-A clip alternations), the flag as an eased
// target (a flip is no jump), body unknown = layer off, the lower body guard (Male: legs on MM, Female: engine legs untouched) and its ease
// when the decision flips mid-episode.
#define _CRT_SECURE_NO_WARNINGS
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "MeleeBaseReplay.hpp"

using namespace utpp;
using namespace utpp::mm;

static int g_fail = 0, g_pass = 0;
static void Expect(const std::string& aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName.c_str(), aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}
static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0, double e = 0, double f = 0, double g = 0, double h = 0)
{
    char buf[400];
    std::snprintf(buf, sizeof(buf), aFmt, a, b, c, d, e, f, g, h);
    return buf;
}
static float Pct(std::vector<float> v, float p)
{
    if (v.empty())
        return 0.0f;
    std::sort(v.begin(), v.end());
    return v[size_t(p * float(v.size() - 1))];
}

static float Elevation(const replay::Replay& rp, int side)
{
    const std::vector<aim::Xf> m = rp.Model();
    const int arm = rp.Find(side == 0 ? "RightArm" : "LeftArm"), hand = rp.Find(side == 0 ? "RightHand" : "LeftHand");
    const math::Vec3 v = m[size_t(hand)].p - m[size_t(arm)].p;
    return std::asin(v.z / aim::Length(v)) * 57.2957795f;
}
// the elevation of the unmodified engine records of frame aIndex
static float RawElevation(replay::Replay& rp, size_t aIndex, int side)
{
    const replay::Frame& f = rp.frames[aIndex];
    std::vector<uint8_t> save = rp.records;
    for (int k = 0; k < rp.nb; ++k)
    {
        float* r = reinterpret_cast<float*>(rp.records.data() + size_t(k) * integ::kGameBoneStride);
        for (int c = 0; c < 3; ++c)
            r[c] = f.P[size_t(k) * 3 + size_t(c)];
        for (int c = 0; c < 4; ++c)
            r[4 + c] = f.Q[size_t(k) * 4 + size_t(c)];
    }
    const float e = Elevation(rp, side);
    rp.records = save;
    return e;
}
static float HipsZ(const replay::Replay& rp)
{
    return aim::ReadLocal(rp.records.data(), uint32_t(rp.Find("Hips"))).p.z;
}

static void TestBody(const std::string& aDir, int aBody)
{
    const char* label = aBody == 1 ? "male" : "female";
    replay::Replay rp;
    if (!rp.Load(aDir + (aBody == 1 ? "/melee_base_male.bin" : "/melee_base_female.bin")))
    {
        Expect(std::string(label) + ": fixture loads", false, aDir);
        return;
    }
    Expect(std::string(label) + ": fixture is the " + label + " recording", rp.body == aBody && rp.nb >= 20 && rp.frames.size() >= 100,
           Fmt("body %g bones %g frames %g", rp.body, rp.nb, double(rp.frames.size())));
    rp.Arm();
    const utpp::meleebase::Counters c0 = utpp::meleebase::GetCounters();
    std::vector<float> errAfter, errBefore, jit, keepDz;
    std::vector<int32_t> clips;
    std::vector<math::Vec3> applied;
    std::vector<float> dts;
    float prevHips = NAN;
    bool keptOnMM = true, engineLegs = true;
    int standFrames = 0;
    for (size_t i = 0; i < rp.frames.size(); ++i)
    {
        rp.Step(i);
        const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
        const replay::Frame& f = rp.frames[i];
        if (!std::isnan(f.refElev))
        {
            errAfter.push_back(std::fabs(Elevation(rp, f.side) - f.refElev));
            errBefore.push_back(std::fabs(RawElevation(rp, i, f.side) - f.refElev));
        }
        if (ls.matched)
            clips.push_back(ls.clip);
        const math::Vec3 a{ls.applied[0], ls.applied[1], ls.applied[2]};
        if (i > 0 && f.dt >= 0.0f && ls.matched && !applied.empty())
        {
            jit.push_back(aim::Length(a - applied.back()) * 57.2957795f);
            dts.push_back(f.dt);
        }
        applied.push_back(a);
        if (f.lowerScale < 0.3f && f.dt >= 0.0f)
        {
            ++standFrames;
            const float hz = HipsZ(rp);
            if (aBody == 1 && !(std::fabs(hz - 0.93f) < 0.12f))
                keptOnMM = false; // Male: the legs stay on MM (Hips z ~0.93 m, not the engine's 0.36)
            if (aBody == 0 && std::fabs(hz - f.P[size_t(rp.Find("Hips")) * 3 + 2]) > 1e-4f && f.lowerScale == 0.0f)
                engineLegs = false; // Female: the engine legs pass through untouched
        }
        prevHips = HipsZ(rp);
    }
    (void)prevHips;
    const utpp::meleebase::Counters c1 = utpp::meleebase::GetCounters();
    const uint64_t dF = c1.matchedFemale - c0.matchedFemale, dM = c1.matchedMale - c0.matchedMale;
    Expect(std::string(label) + ": production path uses ONLY the armed body's table (" + (aBody == 1 ? "Male rows only" : "Female rows only") + ", >= 20 matched frames)",
           (aBody == 1 ? (dF == 0 && dM >= 20) : (dM == 0 && dF >= 20)), Fmt("female rows %g male rows %g", double(dF), double(dM)));
    Expect(std::string(label) + ": " + std::to_string(errAfter.size()) + " punch-extension frames with a reference: elevation error vs the clip p95 <= 7 deg (after), > 10 deg before",
           errAfter.size() >= 15 && Pct(errAfter, 0.95f) <= 7.0f && Pct(errBefore, 0.95f) > 10.0f,
           Fmt("after p95 %.1f p50 %.1f | before p95 %.1f", Pct(errAfter, 0.95f), Pct(errAfter, 0.5f), Pct(errBefore, 0.95f)));
    // jitter
    // noise = second difference of the applied rotation (real clip motion is smooth; a flip or jump between rows shows here)
    std::vector<float> noise;
    for (size_t k = 2; k < applied.size(); ++k)
        if (rp.frames[k].dt >= 0.0f && rp.frames[k - 1].dt >= 0.0f && aim::Length(applied[k]) > 1e-6f && aim::Length(applied[k - 1]) > 1e-6f && aim::Length(applied[k - 2]) > 1e-6f)
            noise.push_back(aim::Length(applied[k] - applied[k - 1] * 2.0f + applied[k - 2]) * 57.2957795f);
    float worstRate = 0.0f;
    for (size_t k = 0; k < jit.size(); ++k)
        worstRate = std::max(worstRate, jit[k] / std::max(dts[k], 1e-3f));
    int aba = 0;
    for (size_t k = 2; k < clips.size(); ++k)
        if (clips[k] == clips[k - 2] && clips[k] != clips[k - 1])
            ++aba;
    Expect(std::string(label) + ": applied rotation is smooth (noise = 2nd difference p95 <= 6 deg (real punches curve: ~5; flipping between rows gave ~6 on the long recordings AND steps of 13 / 20 deg), no step above the 480 deg/s rate limit + 5 %; before: ~6 deg p95, steps up to 13 / 20 deg)",
           !noise.empty() && Pct(noise, 0.95f) <= 6.0f && worstRate <= 480.0f * 1.05f + 20.0f,
           Fmt("1st diff p50 %.2f p95 %.2f max %.2f deg/frame | noise p50 %.2f p95 %.2f max %.2f | worst rate %.0f deg/s", Pct(jit, 0.5f), Pct(jit, 0.95f), Pct(jit, 1.0f), Pct(noise, 0.5f), Pct(noise, 0.95f), Pct(noise, 1.0f), worstRate));
    Expect(std::string(label) + ": the chosen clip does not flip A-B-A (<= 6 alternations)", aba <= 6, Fmt("%g", aba));
    if (standFrames > 10)
    {
        if (aBody == 1)
            Expect("male: standing full body - Hips / Spine / legs stay on MM (Hips z ~0.93 m, the engine's would be 0.36)", keptOnMM, Fmt("%.0f standing frames", double(standFrames)));
        else
            Expect("female: standing full body - the engine legs pass through untouched (no guard)", engineLegs, Fmt("%.0f standing frames", double(standFrames)));
    }

    // the flag as an eased target: meleeBase 0 for frames 30..60 of the first window (a flip must not jump)
    {
        replay::Replay r2;
        r2.Load(aDir + (aBody == 1 ? "/melee_base_male.bin" : "/melee_base_female.bin"));
        r2.Arm();
        std::vector<float> len, base;
        {
            replay::Replay r0;
            r0.Load(aDir + (aBody == 1 ? "/melee_base_male.bin" : "/melee_base_female.bin"));
            r0.Arm();
            for (size_t i = 0; i < 110 && i < r0.frames.size(); ++i)
            {
                r0.Step(i);
                const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
                base.push_back(aim::Length(math::Vec3{ls.applied[0], ls.applied[1], ls.applied[2]}) * 57.2957795f);
            }
            r2.Arm();
        }
        float lenBefore = 0, lenInside = 0, lenAfter = 0;
        for (size_t i = 0; i < 110 && i < r2.frames.size(); ++i)
        {
            r2.Step(i, (i >= 30 && i < 60) ? 0 : -1);
            const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
            len.push_back(aim::Length(math::Vec3{ls.applied[0], ls.applied[1], ls.applied[2]}) * 57.2957795f);
            if (i == 29)
                lenBefore = len.back();
            if (i == 58)
                lenInside = len.back();
            if (i == 100)
                lenAfter = len.back();
        }
        float maxStep = 0.0f;
        for (size_t k = 1; k < len.size(); ++k)
            maxStep = std::max(maxStep, std::fabs(len[k] - len[k - 1]));
        Expect(std::string(label) + ": the fists / standing flag is an eased target - flipping it off and on moves the correction by <= 9 deg per frame (a jump was ~25-45 deg), reaches ~0 and recovers",
               maxStep <= 9.0f && lenBefore > 8.0f && lenInside < 0.25f * lenBefore && lenAfter > 0.8f * base[100],
               Fmt("max step %.2f deg, before %.1f, inside %.1f, after %.1f (un-flipped %.1f)", maxStep, lenBefore, lenInside, lenAfter, base[100]));
    }
    // body unknown: the layer is off (and says so), the records get no chest rotation
    {
        replay::Replay r3;
        r3.Load(aDir + (aBody == 1 ? "/melee_base_male.bin" : "/melee_base_female.bin"));
        r3.Arm();
        utpp::mm::meleebase::SetArmedBody(-1);
        const utpp::meleebase::Counters b0 = utpp::meleebase::GetCounters();
        float maxApplied = 0;
        for (size_t i = 0; i < 60 && i < r3.frames.size(); ++i)
        {
            r3.Step(i);
            const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
            maxApplied = std::max(maxApplied, aim::Length(math::Vec3{ls.applied[0], ls.applied[1], ls.applied[2]}));
        }
        const utpp::meleebase::Counters b1 = utpp::meleebase::GetCounters();
        Expect(std::string(label) + ": armed body unknown (-1) -> no table scan, no rotation, counted as noBody", b1.matched == b0.matched && b1.noBody > b0.noBody && maxApplied < 1e-6f,
               Fmt("noBody +%g applied %.4f", double(b1.noBody - b0.noBody), maxApplied));
        utpp::mm::meleebase::SetArmedBody(aBody);
    }
    // the lower body guard flips mid-episode (the engine hips come back to 0.85 m for a stretch, MM's stay at 0.93): the legs move
    // over ~0.2 s (hysteresis + ease), not in one frame
    if (aBody == 1)
    {
        replay::Replay r4;
        r4.Load(aDir + "/melee_base_male.bin");
        r4.Arm();
        float prev = NAN, maxDz = 0.0f, lo = 9, hi = -9;
        size_t first = 0;
        while (first < r4.frames.size() && !(r4.frames[first].lowerScale < 0.05f))
            ++first;
        for (size_t i = 0; i < r4.frames.size() && i < first + 70; ++i)
        {
            const bool flipped = i >= first + 25; // and it stays: the engine hips are 0.85 m from there on
            r4.Step(i, -1, NAN, flipped ? 0.85f : NAN);
            if (i >= first)
            {
                const float hz = HipsZ(r4);
                if (!std::isnan(prev))
                    maxDz = std::max(maxDz, std::fabs(hz - prev));
                prev = hz;
                lo = std::min(lo, hz);
                hi = std::max(hi, hz);
            }
        }
        Expect("male: guard decision flipping mid-episode - Hips z moves by <= 0.03 m per frame (0.14 m apart: a single-frame switch would move it 0.14)",
               first < r4.frames.size() && (hi - lo) > 0.10f && maxDz <= 0.03f, Fmt("range %.2f max step %.3f", hi - lo, maxDz));
    }
}

// The first punch of a series: the one that lands while the melee upper-body mask is still blending in (0.15 s, the records are a mix of the MM pose
// and the game pose) or right after it. Two recorded windows per body (tests/data/melee_base_blend_*.bin, "MBO3": engine records raw + the MM pose of
// the blended upper bones, tools/melee/fist_base_blend_fixture.py; 2026-11-13 runs) replayed through the production path with the real blend.
// Before the fix the table was matched on the mixed records (rms 10..23 deg until ~0.1 s: no correction), the correction then climbed from zero
// while the first punch landed (female: +10 deg above the clip, male: +10 / +13 deg at the cross, wrist above the head).
static void TestBlend(const std::string& aDir, int aBody)
{
    const char* label = aBody == 1 ? "male" : "female";
    replay::Replay rp;
    if (!rp.Load(aDir + (aBody == 1 ? "/melee_base_blend_male.bin" : "/melee_base_blend_female.bin")))
    {
        Expect(std::string(label) + ": blend fixture loads", false, aDir);
        return;
    }
    Expect(std::string(label) + ": blend fixture is the " + label + " recording with the MM pose of the blended upper bones",
           rp.body == aBody && rp.upper.size() >= 10 && rp.frames.size() >= 60, Fmt("body %g upper bones %g frames %g", rp.body, double(rp.upper.size()), double(rp.frames.size())));
    rp.Arm();
    int blendFrames = 0, blendMatched = 0, ref = 0;
    std::vector<float> errAfter, errBefore, firstAfter;
    int inWindow = 0;
    for (size_t i = 0; i < rp.frames.size(); ++i)
    {
        const replay::Frame& f = rp.frames[i];
        if (f.dt < 0.0f)
            inWindow = 0; // a new window (hard reset of the layer)
        rp.Step(i);
        const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
        if (f.meleeWeight > 0.15f && f.meleeWeight < 0.9f)
        {
            ++blendFrames;
            blendMatched += ls.matched ? 1 : 0;
        }
        if (!std::isnan(f.refElev))
        {
            const float after = std::fabs(Elevation(rp, f.side) - f.refElev);
            errAfter.push_back(after);
            errBefore.push_back(std::fabs(RawElevation(rp, i, f.side) - f.refElev));
            if (++inWindow <= 4)
                firstAfter.push_back(after); // the first stretched-arm frames after the blend: the first punch
            ++ref;
        }
    }
    Expect(std::string(label) + ": while the mask blends in the table is matched on the game pose (>= 90 % of the " + std::to_string(blendFrames) +
               " blend frames matched; matching the mixed records gave ~20 %)",
           blendFrames >= 8 && blendMatched >= int(0.9f * float(blendFrames)), Fmt("%g of %g matched", blendMatched, blendFrames));
    Expect(std::string(label) + ": first punch of each window (first 4 stretched-arm frames, " + std::to_string(ref) + " frames with a reference): elevation error vs the clip <= 6 deg (old code: ~11 deg)",
           firstAfter.size() >= 6 && Pct(firstAfter, 1.0f) <= 6.0f && Pct(errAfter, 0.95f) <= 7.0f && Pct(errBefore, 0.95f) > 10.0f,
           Fmt("first-punch max %.1f deg | all frames p95 %.1f p50 %.1f | engine pose p95 %.1f", Pct(firstAfter, 1.0f), Pct(errAfter, 0.95f), Pct(errAfter, 0.5f), Pct(errBefore, 0.95f)));
}

int main(int argc, char** argv)
{
    const std::string dir = argc > 1 ? argv[1] : "tests/data";
    TestBody(dir, 1);
    TestBody(dir, 0);
    TestBlend(dir, 1);
    TestBlend(dir, 0);
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
