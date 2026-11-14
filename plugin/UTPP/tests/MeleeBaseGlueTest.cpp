// Offline test of the PRODUCTION glue of the melee chest base correction (src/mm/MeleeBase.cpp through tests/MeleeBaseReplay.hpp: OnArmed,
// SetArmedBody, LowerBodyKeepWeight / LowerBodyReset, BlendIntoRecords, the melee full pose layer, Apply in the order of FkHook.cpp) on two small
// recorded excerpts (tests/data/melee_base_male.bin / melee_base_female.bin: one standing combo window + one walking window cut from a pose
// recording of each body by tools/melee/fist_base_fixture.py, which documents the recording and the windows it needs). Not shipped.
// Usage: utpp_melee_base_glue_test <tests/data dir> <melee pose table dir>
// The checks give their verdict on the shipped default: the melee full pose layer ON (melee.fullPose, src/mm/MeleePose.cpp, with its pose
// tables) in front of the chest correction. The same checks are then run with the layer OFF (the chest correction alone, a developer-only
// fallback) and printed as information lines without a verdict, except the checks of code that only runs there (the lower body guard on
// standing frames, the flag flip in a standing window): those are verdicts named "fallback: ...".
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

#ifndef UTPP_REPLAY_FULL_POSE
#error "build with UTPP_REPLAY_FULL_POSE: the test drives the shipped default (melee full pose layer + chest correction)"
#endif
#include "MeleeBaseReplay.hpp"

using namespace utpp;
using namespace utpp::mm;

static int g_fail = 0, g_pass = 0;
// false: the fallback run (melee.fullPose off, the chest correction alone) - measured and printed as information, no verdict
static bool g_verdict = true;
static void Expect(const std::string& aName, bool aOk, const std::string& aDetail = {})
{
    if (!g_verdict)
    {
        std::printf("[INFO fallback, no verdict: %s] %s%s%s\n", aOk ? "within" : "outside", aName.c_str(), aDetail.empty() ? "" : " -- ", aDetail.c_str());
        return;
    }
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName.c_str(), aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}
// a verdict check of the fallback configuration (melee full pose off): counted on the fallback pass as well
static void ExpectFallback(const std::string& aName, bool aOk, const std::string& aDetail = {})
{
    const bool v = g_verdict;
    g_verdict = true;
    Expect("fallback: " + aName, aOk, aDetail);
    g_verdict = v;
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

// The punch reference of a frame is the clip's shoulder -> wrist elevation of the striking arm at the row the chest correction matched
// (stored in the fixture as refElev). The matched row of the run is kept per frame to check that reference for single-frame jumps.
struct RefRow
{
    int32_t clip = -1;
    float row = NAN;       // fractional table row (frame + t x (neighbour - frame)), NaN = no match
    bool newWindow = false; // the frame starts a window (hard reset)
};
static RefRow MatchedRow(const replay::Frame& aF)
{
    const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
    RefRow r;
    r.newWindow = aF.dt < 0.0f;
    if (ls.matched && ls.frame >= 0)
    {
        r.clip = ls.clip;
        r.row = float(ls.frame) + (ls.neighbour >= 0 ? ls.t * float(ls.neighbour - ls.frame) : 0.0f);
    }
    return r;
}
// shoulder -> wrist vector of the clip at table row aRow (model space of the clip: its Spine3 orientation and the shoulder / arm / forearm
// locals of the row, the rig's own forearm and hand offsets)
static math::Vec3 ClipArm(const replay::Replay& rp, const utpp::mm::meleebase::TableData& aT, uint32_t aRow, int aSide)
{
    const int16_t* row = aT.frames + size_t(aRow) * size_t(utpp::mm::meleebase::kRowInts);
    const int k0 = aSide == 0 ? 2 : 6; // RightShoulder / LeftShoulder in kKeyNames order, then Arm, ForeArm, Hand
    const math::Quat chest = math::QuatNormalize(utpp::mm::meleebase::DecodeQuat(row + size_t(utpp::mm::meleebase::kKeyBones) * 4u));
    const math::Quat arm = math::QuatNormalize(math::QuatMul(math::QuatMul(chest, utpp::mm::meleebase::DecodeQuat(row + size_t(k0) * 4u)),
                                                             utpp::mm::meleebase::DecodeQuat(row + size_t(k0 + 1) * 4u)));
    const math::Quat fore = math::QuatNormalize(math::QuatMul(arm, utpp::mm::meleebase::DecodeQuat(row + size_t(k0 + 2) * 4u)));
    const math::Vec3 pFore = aim::ReadLocal(rp.records.data(), uint32_t(rp.Find(aSide == 0 ? "RightForeArm" : "LeftForeArm"))).p;
    const math::Vec3 pHand = aim::ReadLocal(rp.records.data(), uint32_t(rp.Find(aSide == 0 ? "RightHand" : "LeftHand"))).p;
    return math::QuatRotate(arm, pFore) + math::QuatRotate(fore, pHand);
}
static float ClipElevation(const replay::Replay& rp, const utpp::mm::meleebase::TableData& aT, float aRow, int aSide)
{
    const uint32_t i0 = uint32_t(aRow);
    const uint32_t i1 = i0 + 1 < aT.frameCount ? i0 + 1 : i0;
    const float t = aRow - float(i0);
    const math::Vec3 v = ClipArm(rp, aT, i0, aSide) * (1.0f - t) + ClipArm(rp, aT, i1, aSide) * t;
    return std::asin(v.z / aim::Length(v)) * 57.2957795f;
}
// Reference rule, applied to every frame with a reference: when the matched row jumps for ONE frame - the frames before and after it are
// in the same window, on the same clip, within kRefAgreeRows of each other, and the frame's row is more than kRefJumpRows from their mean
// (or on another clip) - the frame's reference is the clip elevation at the mean of its neighbours' rows (the row consistent with them).
// Returns the number of references changed; aCheckMax = the largest difference between the elevation computed here at the run's own row
// and the fixture's reference on the unchanged frames (both must be the same reference).
constexpr float kRefAgreeRows = 3.0f;
constexpr float kRefJumpRows = 3.0f;
static int FixReferences(const replay::Replay& rp, const utpp::mm::meleebase::TableData& aT, const std::vector<RefRow>& aRows, std::vector<float>& aRef,
                         float& aCheckMax, int& aCompared)
{
    int changed = 0;
    aCheckMax = 0.0f;
    aCompared = 0;
    for (size_t i = 0; i < aRows.size() && i < aRef.size(); ++i)
    {
        if (std::isnan(aRef[i]) || std::isnan(aRows[i].row))
            continue;
        const replay::Frame& f = rp.frames[i];
        if (i > 0 && i + 1 < aRows.size() && !aRows[i].newWindow && !aRows[i + 1].newWindow)
        {
            const RefRow &a = aRows[i - 1], &b = aRows[i + 1];
            if (!std::isnan(a.row) && !std::isnan(b.row) && a.clip == b.clip && std::fabs(b.row - a.row) <= kRefAgreeRows)
            {
                const float mid = 0.5f * (a.row + b.row);
                if (aRows[i].clip != a.clip || std::fabs(aRows[i].row - mid) > kRefJumpRows)
                {
                    const float was = aRef[i];
                    aRef[i] = ClipElevation(rp, aT, mid, f.side);
                    std::printf("  reference of frame %d: matched row %.2f between %.2f and %.2f -> row %.2f, elevation %.1f -> %.1f deg\n", int(i),
                                aRows[i].row, a.row, b.row, mid, was, aRef[i]);
                    ++changed;
                    continue;
                }
            }
        }
        aCheckMax = std::max(aCheckMax, std::fabs(ClipElevation(rp, aT, aRows[i].row, f.side) - aRef[i]));
        ++aCompared;
    }
    return changed;
}
static void ReportReferences(const char* aLabel, const char* aWhat, int aChanged, float aCheckMax, int aCompared)
{
    Expect(std::string(aLabel) + ": " + aWhat + "punch reference = the clip at the matched row: recomputed here at the run's own row it equals the fixture's (max diff <= 1 deg); single-frame row jumps replaced by the row of the neighbours",
           aCompared >= 15 && aCheckMax <= 1.0f, Fmt("max diff %.3f deg over %g frames | %g reference(s) changed", aCheckMax, double(aCompared), double(aChanged)));
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
    const utpp::meleepose::Counters pose0 = utpp::meleepose::GetCounters();
    std::vector<float> errAfter, errBefore, jit, keepDz;
    std::vector<int32_t> clips;
    std::vector<math::Vec3> applied;
    std::vector<float> dts;
    std::vector<RefRow> rows;
    std::vector<float> elevAfter(rp.frames.size(), NAN), elevBefore(rp.frames.size(), NAN), ref(rp.frames.size(), NAN);
    float prevHips = NAN;
    bool keptOnMM = true, engineLegs = true;
    int standFrames = 0, poseStandFrames = 0;
    size_t walkStart = 0; // first frame of the walking window (the second hard reset)
    float walkLowerMin = 1.0f;
    for (size_t i = 0; i < rp.frames.size(); ++i)
    {
        rp.Step(i);
        const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
        const replay::Frame& f = rp.frames[i];
        rows.push_back(MatchedRow(f));
        ref[i] = f.refElev;
        if (!std::isnan(f.refElev))
        {
            elevAfter[i] = Elevation(rp, f.side);
            elevBefore[i] = RawElevation(rp, i, f.side);
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
        if (i > 0 && f.dt < 0.0f && walkStart == 0)
            walkStart = i;
        if (walkStart != 0)
            walkLowerMin = std::min(walkLowerMin, f.lowerScale);
        if (f.lowerScale < 0.3f && f.dt >= 0.0f && utpp::meleepose::GetLastStep().weight > 0.0f && walkStart == 0)
            ++poseStandFrames;
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
    {
        float checkMax = 0.0f;
        int compared = 0;
        const int changed = FixReferences(rp, aBody == 1 ? utpp::mm::meleebase::MaleTable() : utpp::mm::meleebase::FemaleTable(), rows, ref, checkMax, compared);
        for (size_t i = 0; i < ref.size(); ++i)
            if (!std::isnan(ref[i]))
            {
                errAfter.push_back(std::fabs(elevAfter[i] - ref[i]));
                errBefore.push_back(std::fabs(elevBefore[i] - ref[i]));
            }
        ReportReferences(label, "", changed, checkMax, compared);
    }
    const utpp::meleebase::Counters c1 = utpp::meleebase::GetCounters();
    const uint64_t poseWritten = utpp::meleepose::GetCounters().written - pose0.written;
    if (g_verdict)
        Expect(std::string(label) + ": default path - the melee full pose layer wrote the lower body (>= 20 frames; 0 = its tables did not load)",
               poseWritten >= 20, Fmt("%g frames written", double(poseWritten)));
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
    // The lower body guard acts on the standing full body only (lowerScale < 0.3). On the default path the melee full pose writes the clip's
    // lower body on those frames (its own checks: utpp_melee_pose_test / utpp_melee_pose_replay_test); the walking window has no standing
    // frame (lowerScale stays on locomotion), so the guard is judged on the fallback configuration, where it is the code that runs.
    if (g_verdict)
        std::printf("  %s: standing full body guard - not observable on the default path: the full pose wrote %d of the %d standing frames, the walking "
                    "window's lowest lowerScale is %.2f (>= 0.3: no standing frame); judged as \"fallback: ...\"\n",
                    label, poseStandFrames, standFrames, walkLowerMin);
    else if (aBody == 1)
        ExpectFallback("male: standing full body - Hips / Spine / legs stay on MM (Hips z ~0.93 m, the engine's would be 0.36)", standFrames > 10 && keptOnMM,
                       Fmt("%.0f standing frames", double(standFrames)));
    else
        ExpectFallback("female: standing full body - the engine legs pass through untouched (no guard)", standFrames > 10 && engineLegs,
                       Fmt("%.0f standing frames", double(standFrames)));

    // the flag as an eased target (a flip must not jump). The chest correction is what the flag eases: on the default path it runs in the
    // walking window (the full pose is off while V moves), so the flag is flipped off there for kFlipFrames frames; on the fallback pass
    // the standing window carries it (frames 30..60), as a check of the fallback configuration.
    {
        const std::string file = aDir + (aBody == 1 ? "/melee_base_male.bin" : "/melee_base_female.bin");
        const bool walk = g_verdict;
        const size_t s0 = walk ? walkStart : 0;
        const size_t flipFrom = s0 + (walk ? 15 : 30), flipTo = s0 + (walk ? 35 : 60);
        const size_t atBefore = flipFrom - 1, atInside = flipTo - 2, atAfter = walk ? rp.frames.size() - 2 : 100;
        const size_t end = walk ? rp.frames.size() : 110;
        std::vector<float> len(end, 0.0f), base(end, 0.0f);
        {
            replay::Replay r0;
            r0.Load(file);
            r0.Arm();
            for (size_t i = 0; i < end && i < r0.frames.size(); ++i)
            {
                r0.Step(i);
                const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
                base[i] = aim::Length(math::Vec3{ls.applied[0], ls.applied[1], ls.applied[2]}) * 57.2957795f;
            }
        }
        replay::Replay r2;
        r2.Load(file);
        r2.Arm();
        float maxStep = 0.0f;
        for (size_t i = 0; i < end && i < r2.frames.size(); ++i)
        {
            r2.Step(i, (i >= flipFrom && i < flipTo) ? 0 : -1);
            const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
            len[i] = aim::Length(math::Vec3{ls.applied[0], ls.applied[1], ls.applied[2]}) * 57.2957795f;
            if (i > s0)
                maxStep = std::max(maxStep, std::fabs(len[i] - len[i - 1]));
        }
        const bool ok = (!walk || walkStart > 0) && atAfter < end && maxStep <= 9.0f && len[atBefore] > 8.0f && len[atInside] < 0.25f * len[atBefore] &&
                        len[atAfter] > 0.8f * base[atAfter];
        const std::string name = std::string(label) + (walk ? ": walking window" : ": standing window") +
                                 " - the fists / standing flag is an eased target - flipping it off and on moves the correction by <= 9 deg per frame (a jump was ~25-45 deg), reaches ~0 and recovers";
        const std::string detail = Fmt("max step %.2f deg, before %.1f, inside %.1f, after %.1f (un-flipped %.1f)", maxStep, len[atBefore], len[atInside],
                                       len[atAfter], base[atAfter]);
        if (walk)
            Expect(name, ok, detail);
        else
            ExpectFallback(name, ok, detail);
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
    // the lower body guard flips mid-episode: the engine hips sit at the engine's base pose height (0.36 m, more than kGuardEnterM below
    // MM's: the legs are kept on MM) and come back to 0.85 m from first + 25 on (within kGuardLeaveM of MM's: the decision flips to the engine
    // legs). The legs move over ~0.2 s (hysteresis + ease), not in one frame. Measured as the difference to the same run without the flip,
    // so the clip's own Hips motion (a lunge of ~4 cm per frame) is not counted as the flip's. On the default path the melee full pose writes
    // the standing legs at full weight: the flip must not show there at all; the ease itself is judged on the fallback configuration.
    if (aBody == 1)
    {
        size_t first = 0;
        std::vector<float> hz[2];
        for (int flip = 0; flip < 2; ++flip)
        {
            replay::Replay r4;
            r4.Load(aDir + "/melee_base_male.bin");
            r4.Arm();
            while (flip == 0 && first < r4.frames.size() && !(r4.frames[first].lowerScale < 0.05f))
                ++first;
            for (size_t i = 0; i < r4.frames.size() && i < first + 70; ++i)
            {
                const bool flipped = flip == 1 && i >= first + 25; // and it stays: the engine hips are 0.85 m from there on
                r4.Step(i, -1, NAN, flipped ? 0.85f : 0.36f);
                if (i >= first)
                    hz[flip].push_back(HipsZ(r4));
            }
        }
        float maxDz = 0.0f, lo = 9, hi = -9;
        for (size_t k = 0; k < hz[0].size() && k < hz[1].size(); ++k)
        {
            const float d = hz[1][k] - hz[0][k];
            if (k > 0)
                maxDz = std::max(maxDz, std::fabs(d - (hz[1][k - 1] - hz[0][k - 1])));
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
        const bool ran = hz[0].size() >= 60 && hz[1].size() == hz[0].size();
        if (g_verdict)
            Expect("male: guard decision flipping mid-episode under the full pose - the flip moves Hips z by <= 0.03 m per frame (against the same run without the flip)",
                   ran && maxDz <= 0.03f, Fmt("flip effect range %.2f m, max step %.3f m", hi - lo, maxDz));
        else
            ExpectFallback("male: guard decision flipping mid-episode - Hips z moves by <= 0.03 m per frame (0.14 m apart: a single-frame switch would move it 0.14; against the same run without the flip)",
                           ran && (hi - lo) > 0.10f && maxDz <= 0.03f, Fmt("flip effect range %.2f m, max step %.3f m", hi - lo, maxDz));
    }
}

// The first punch of a series: the one that lands while the melee upper-body mask is still blending in (0.15 s, the records are a mix of the MM pose
// and the game pose) or right after it. Two recorded windows per body (tests/data/melee_base_blend_*.bin, "MBO3": engine records raw + the MM pose of
// the blended upper bones, cut by tools/melee/fist_base_fixture.py) replayed through the production path with the real blend.
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
    std::vector<RefRow> rows;
    std::vector<float> elevAfter(rp.frames.size(), NAN), elevBefore(rp.frames.size(), NAN), refs(rp.frames.size(), NAN);
    for (size_t i = 0; i < rp.frames.size(); ++i)
    {
        const replay::Frame& f = rp.frames[i];
        rp.Step(i);
        const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
        rows.push_back(MatchedRow(f));
        refs[i] = f.refElev;
        if (f.meleeWeight > 0.15f && f.meleeWeight < 0.9f)
        {
            ++blendFrames;
            blendMatched += ls.matched ? 1 : 0;
        }
        if (!std::isnan(f.refElev))
        {
            elevAfter[i] = Elevation(rp, f.side);
            elevBefore[i] = RawElevation(rp, i, f.side);
        }
    }
    {
        float checkMax = 0.0f;
        int compared = 0;
        const int changed = FixReferences(rp, aBody == 1 ? utpp::mm::meleebase::MaleTable() : utpp::mm::meleebase::FemaleTable(), rows, refs, checkMax, compared);
        ReportReferences(label, "blend fixture: ", changed, checkMax, compared);
    }
    int inWindow = 0;
    for (size_t i = 0; i < rp.frames.size(); ++i)
    {
        if (rp.frames[i].dt < 0.0f)
            inWindow = 0; // a new window (hard reset of the layer)
        if (std::isnan(refs[i]))
            continue;
        const float after = std::fabs(elevAfter[i] - refs[i]);
        errAfter.push_back(after);
        errBefore.push_back(std::fabs(elevBefore[i] - refs[i]));
        if (++inWindow <= 4)
            firstAfter.push_back(after); // the first stretched-arm frames after the blend: the first punch
        ++ref;
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
    if (argc < 3)
    {
        std::printf("usage: utpp_melee_base_glue_test <tests/data dir> <melee pose table dir>\n");
        return 1;
    }
    // the shipped default: the melee full pose layer (melee.fullPose on) in front of the chest correction, with its pose tables
    const std::string tables = std::string(argv[2]) + "/";
    utpp::meleepose::Load(std::wstring(tables.begin(), tables.end()));
    utpp::meleepose::SetWeapon(0);
    for (int pass = 0; pass < 2; ++pass)
    {
        g_verdict = pass == 0;
        utpp::meleepose::SetEnabled(g_verdict);
        std::printf(g_verdict ? "== default path: melee full pose ON + chest correction (verdicts)\n"
                              : "\n== fallback path: melee full pose OFF, the chest correction alone (information only, no verdict)\n");
        TestBody(dir, 1);
        TestBody(dir, 0);
        TestBlend(dir, 1);
        TestBlend(dir, 0);
    }
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
