// ragdoll R0 probe (src/ragdoll/GetUpProbe.hpp): offline checks of the pose composition and the probe lifetime.
// Usage: utpp_ragdoll_getup_probe_test [woman_motion.utppmm man_motion.utppmm]
//   The optional databases check that the embedded get-up table (src/ragdoll/GetUpFrame0.hpp) matches their bone order and composes.
// Mutations (build with -DUTPP_RAGDOLL_PROBE_MUTATION=k, tests/CMake utpp_ragdoll_getup_probe_mut1/2): 1 wrong axis -> "FAIL compose:
// offset on +Y only"; 2 the hold never expires -> "FAIL hold expires".
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "../src/ragdoll/GetUpClips.hpp"
#include "../src/ragdoll/GetUpProbe.hpp"

#ifdef UTPP_RAGDOLL_PROBE_DB
#include "../src/mm/MotionDatabase.hpp"
#endif

using namespace utpp::ragdoll;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat.c_str(), aDetail.empty() ? "" : " | ", aDetail.c_str());
}

constexpr uint32_t kN = 10, kFirst = 2;
// bones 0 Root, 1 Pelvis-less helper, 2 Hips (root child, parent 1), 3 Spine (2), 4 Head (3), 5 Prop (root child, parent 0),
// 6 Arm (3), 7 Hand (6), 8 Leg (2), 9 Foot (8)
const int32_t kParents[kN] = {-1, 0, 1, 2, 3, 0, 3, 6, 2, 8};

std::vector<float> MakeSrc()
{
    std::vector<float> v(size_t(kN) * 7);
    for (uint32_t i = 0; i < kN; ++i)
    {
        float* p = &v[size_t(i) * 7];
        p[0] = 0.01f * float(i) + 0.003f;
        p[1] = -0.02f * float(i) + 0.05f;
        p[2] = 0.1f * float(i) + 0.9f;
        const float a = 0.1f * float(i + 1);
        const float l = std::sqrt(1.0f + a * a + 2.0f * a * a);
        p[3] = a / l; p[4] = a / l; p[5] = a / l; p[6] = 1.0f / l;
    }
    return v;
}

bool SameBits(const float* a, const float* b, size_t n) { return std::memcmp(a, b, n * sizeof(float)) == 0; }

void TestArgs()
{
    ProbeArgs a = SanitizeProbeArgs(1.2f, 3.0f);
    Check(a.ok && a.offsetM == 1.2f && a.durationS == 3.0f, "args: in range kept");
    a = SanitizeProbeArgs(99.0f, 99.0f);
    Check(a.ok && a.offsetM == kProbeMaxOffsetM && a.durationS == kProbeMaxDurationS, "args: offset and duration clamped high");
    a = SanitizeProbeArgs(-4.0f, 0.1f);
    Check(a.ok && a.offsetM == 0.0f && a.durationS == kProbeMinDurationS, "args: negative offset -> 0, duration clamped to 0.5 s");
    const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
    Check(!SanitizeProbeArgs(nan, 1.0f).ok && !SanitizeProbeArgs(1.0f, nan).ok && !SanitizeProbeArgs(inf, 1.0f).ok &&
              !SanitizeProbeArgs(1.0f, -inf).ok,
          "args: non-finite refused");
    GetUpProbe p;
    Check(!p.Start(nan, 1.0f, 0.0, 3) && !p.Start(1.0f, 1.0f, 0.0, -1) && !p.Start(1.0f, 1.0f, nan, 3) && !p.Active(),
          "args: refused Start leaves the probe idle");
}

void TestCompose()
{
    const std::vector<float> src = MakeSrc();
    std::vector<float> dst(src.size(), -7.0f); // sentinel
    const uint32_t moved = ComposeProbePose(src.data(), kParents, kN, kFirst, 1.2f, dst.data());
    Check(moved == 2, "compose: root children counted (Hips + Prop)", std::to_string(moved));

    bool axis = true, quat = true, others = true;
    for (uint32_t i = kFirst; i < kN; ++i)
    {
        const float* s = &src[size_t(i) * 7];
        const float* d = &dst[size_t(i) * 7];
        const bool root = kParents[i] < int32_t(kFirst);
        if (root)
        {
            axis = axis && d[0] == s[0] && std::fabs(d[1] - (s[1] + 1.2f)) < 1e-6f && d[2] == s[2];
            quat = quat && SameBits(d + 3, s + 3, 4);
        }
        else
            others = others && SameBits(d, s, 7);
    }
    Check(axis, "compose: offset on +Y only (x, z unchanged, y + 1.2)");
    Check(quat, "compose: root child rotations unchanged");
    Check(others, "compose: every other bone bit-identical to the clip frame");

    bool below = true;
    for (uint32_t i = 0; i < kFirst; ++i)
        for (int k = 0; k < 7; ++k)
            below = below && dst[size_t(i) * 7 + k] == -7.0f;
    Check(below, "compose: bones below firstBone not written");

    std::vector<float> zero(src.size(), -7.0f);
    (void)ComposeProbePose(src.data(), kParents, kN, kFirst, 0.0f, zero.data());
    bool same = true;
    for (uint32_t i = kFirst; i < kN; ++i)
        same = same && SameBits(&zero[size_t(i) * 7], &src[size_t(i) * 7], 7);
    Check(same, "compose: offset 0 = the clip pose itself (control run)");

    std::vector<float> d3(src.size(), 0.0f);
    (void)ComposeProbePose(src.data(), kParents, kN, kFirst, 3.0f, d3.data());
    Check(std::fabs((d3[size_t(kFirst) * 7 + 1] - src[size_t(kFirst) * 7 + 1]) - 3.0f) < 1e-5f, "compose: 3 m offset");
}

void TestLifetime()
{
    GetUpProbe p;
    Check(!p.Active() && p.ComposeOffsetM() == 0.0f, "idle: inactive, no offset");
    const GetUpProbe::TickResult idle = p.Advance(0.1f, 1.0);
    Check(!idle.logDue && !idle.ended && !idle.handBackStarted && !p.Active(), "idle: Advance does nothing");

    Check(p.Start(1.2f, 3.0f, 100.0, 7) && p.Holding() && p.Clip() == 7, "start: holding");
    Check(p.ComposeOffsetM() == 1.2f, "start: compose offset = requested");
    int logs = 0, step = 0, handBackAt = -1, endedAt = -1;
    ProbeEnd endReason = ProbeEnd::None;
    double wall = 100.0;
    for (; step < 100 && endedAt < 0; ++step)
    {
        wall += 0.125;
        const GetUpProbe::TickResult r = p.Advance(0.125f, wall);
        logs += r.logDue ? 1 : 0;
        if (r.handBackStarted)
            handBackAt = step + 1;
        if (r.ended)
        {
            endedAt = step + 1;
            endReason = r.reason;
        }
    }
    Check(handBackAt == 24, "hold expires: hand-back starts after 3.0 s of feed time", std::to_string(handBackAt));
    Check(endedAt > handBackAt && endedAt - handBackAt <= 2 && endReason == ProbeEnd::Expired,
          "hold expires: probe ends right after the 0.12 s hand-back", std::to_string(endedAt));
    Check(logs == 5, "log: one status tick per 0.5 s of hold (0.5 .. 2.5)", std::to_string(logs));
    Check(!p.Active() && p.ComposeOffsetM() == 0.0f && p.Clip() == -1 && p.OffsetM() == 0.0f, "hand-back done: state fully cleared");
    Check(p.Runs() == 1, "runs counter");

    // hand-back phase publishes the normal pose (no offset) while the blend runs
    GetUpProbe q;
    (void)q.Start(3.0f, 0.5f, 0.0, 1);
    (void)q.Advance(0.25f, 0.1);
    const GetUpProbe::TickResult h = q.Advance(0.25f, 0.2);
    Check(h.handBackStarted && q.HandingBack() && q.ComposeOffsetM() == 0.0f && q.HandBackLeftS() > 0.0f,
          "hand-back: pose offset released, blend span running");

    // restart replaces a running probe cleanly
    GetUpProbe r;
    (void)r.Start(1.2f, 2.0f, 0.0, 1);
    (void)r.Advance(0.5f, 0.1);
    (void)r.Start(3.0f, 1.0f, 0.2, 2);
    Check(r.Holding() && r.ElapsedS() == 0.0f && r.OffsetM() == 3.0f && r.Clip() == 2 && r.Runs() == 2, "restart: fresh run");

    // one long frame counts for at most 0.25 s of feed time
    GetUpProbe s;
    (void)s.Start(1.0f, 2.0f, 0.0, 1);
    (void)s.Advance(10.0f, 0.1);
    Check(s.ElapsedS() == kProbeMaxStepS, "clock: one frame counts at most 0.25 s");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    (void)s.Advance(nan, 0.2);
    (void)s.Advance(-1.0f, 0.3);
    Check(s.ElapsedS() == kProbeMaxStepS, "clock: NaN / negative dt ignored");
}

void TestAbortAndWatchdog()
{
    GetUpProbe p;
    (void)p.Start(1.2f, 3.0f, 10.0, 4);
    (void)p.Advance(0.5f, 10.5);
    p.Abort();
    Check(!p.Active() && p.ComposeOffsetM() == 0.0f && p.Clip() == -1 && p.ElapsedS() == 0.0f && p.HandBackLeftS() == 0.0f,
          "abort: state cleared at once (no hand-back)");
    const GetUpProbe::TickResult t = p.Advance(0.1f, 11.0);
    Check(!t.logDue && !t.ended && !t.handBackStarted, "abort: later ticks do nothing");

    GetUpProbe h;
    (void)h.Start(1.2f, 3.0f, 0.0, 4);
    (void)h.Advance(0.25f, 0.1);
    h.Abort();
    (void)h.Start(1.2f, 3.0f, 1.0, 4);
    h.Abort();
    Check(!h.Active(), "abort: during hand-back or hold both clear");
    GetUpProbe hb;
    (void)hb.Start(1.0f, 0.5f, 0.0, 4);
    (void)hb.Advance(0.25f, 0.1);
    (void)hb.Advance(0.25f, 0.2);
    Check(hb.HandingBack(), "abort: setup in hand-back");
    hb.Abort();
    Check(!hb.Active() && hb.HandBackLeftS() == 0.0f, "abort: hand-back cleared");

    GetUpProbe w;
    (void)w.Start(3.0f, 5.0f, 100.0, 4);
    Check(!w.Watchdog(100.0) && !w.Watchdog(105.0) && w.Active(), "watchdog: quiet before 5.25 s wall");
    Check(w.Watchdog(105.3) && !w.Active() && w.ComposeOffsetM() == 0.0f, "watchdog: ends a run with no feed at all");
    GetUpProbe w2;
    (void)w2.Start(3.0f, 5.0f, 100.0, 4);
    (void)w2.Advance(0.01f, 105.0);
    const GetUpProbe::TickResult r = w2.Advance(0.01f, 106.0);
    Check(r.ended && r.reason == ProbeEnd::Watchdog && !w2.Active(), "watchdog: stalled feed time (tiny dt) still ends via wall clock");
    GetUpProbe w3;
    (void)w3.Start(1.0f, 2.0f, 100.0, 4);
    Check(w3.Watchdog(std::numeric_limits<double>::quiet_NaN()) && !w3.Active(), "watchdog: NaN clock fails closed");
    GetUpProbe w4;
    Check(!w4.Watchdog(1e9), "watchdog: idle probe unaffected");
}

void TestTable(bool aWoman, const char* aLabel)
{
    const std::string L = std::string("table ") + aLabel + ": ";
    for (int32_t clip = 0; clip < kGetUpClipCount; ++clip)
    {
        const float* t = GetUpFrame0(aWoman, clip);
        Check(t != nullptr, L + GetUpClipName(clip) + " present");
        if (t == nullptr)
            continue;
        bool finite = true, unit = true;
        for (uint32_t i = 0; i < kGetUpBones; ++i)
        {
            const float* p = t + size_t(i) * 7;
            for (int k = 0; k < 7; ++k)
                finite = finite && std::isfinite(p[k]);
            const float l = std::sqrt(p[3] * p[3] + p[4] * p[4] + p[5] * p[5] + p[6] * p[6]);
            unit = unit && std::fabs(l - 1.0f) < 1e-3f;
        }
        Check(finite && unit, L + GetUpClipName(clip) + ": finite, unit quaternions");
        const float hipsZ = t[2 * 7 + 2];
        Check(hipsZ > 0.05f && hipsZ < 0.35f, L + GetUpClipName(clip) + ": hips lying low (frame 0)", std::to_string(hipsZ) + " m");
        // composes like any pose: Hips (bone 2, parent Root 0) is a root child and moves 1.2 m on +Y, the spine is untouched
        int32_t parents[kGetUpBones];
        parents[0] = -1;
        parents[1] = 0;
        parents[2] = 0;
        parents[3] = 0;
        for (uint32_t i = 4; i < kGetUpBones; ++i)
            parents[i] = 2; // everything else below Hips for this check (the real parents are in the DB check below)
        std::vector<float> dst(size_t(kGetUpBones) * 7, 0.0f);
        const uint32_t moved = ComposeProbePose(t, parents, kGetUpBones, 2, 1.2f, dst.data());
        Check(moved == 2 && std::fabs(dst[2 * 7 + 1] - (t[2 * 7 + 1] + 1.2f)) < 1e-5f && dst[4 * 7 + 1] == t[4 * 7 + 1],
              L + GetUpClipName(clip) + ": table composes (Hips and reference_joint displaced, Spine untouched)");
    }
    Check(GetUpFrame0(aWoman, 0) != GetUpFrame0(aWoman, 1) && std::memcmp(GetUpFrame0(aWoman, 0), GetUpFrame0(aWoman, 1), 73 * 7 * sizeof(float)) != 0,
          L + "Back and Front differ");
}

#ifdef UTPP_RAGDOLL_PROBE_DB
void TestDatabase(const char* aPath, uint64_t aRig, const char* aLabel)
{
    using namespace utpp::mm::motion;
    MotionDatabase db;
    std::string err;
    if (!db.LoadFromFile(aPath, aRig, err))
    {
        Check(false, std::string("db ") + aLabel + ": load", err);
        return;
    }
    Check(GetUpBoneOrderMatches(db.NumBones(), [&](uint32_t i) { return db.BoneName(i); }),
          std::string("db ") + aLabel + ": bone order = the get-up table's (guard the driver checks at Start)");
    const uint32_t n = db.NumBones();
    std::vector<int32_t> parents(n);
    for (uint32_t i = 0; i < n; ++i)
        parents[i] = db.BoneParent(i);
    const float* src = GetUpFrame0(std::string(aLabel) == "woman", 0);
    std::vector<float> dst(size_t(n) * 7, 0.0f);
    const uint32_t moved = ComposeProbePose(src, parents.data(), n, 2, 1.2f, dst.data());
    int hips = -1;
    for (uint32_t i = 2; i < n; ++i)
        if (db.BoneName(i) == "Hips")
            hips = static_cast<int>(i);
    Check(moved >= 1 && hips >= 0 && parents[hips] < 2, std::string("db ") + aLabel + ": Hips is a root child", std::to_string(moved) + " root children");
    if (hips < 0)
        return;
    const float* s = src + size_t(hips) * 7;
    const float* d = &dst[size_t(hips) * 7];
    Check(std::fabs(d[1] - s[1] - 1.2f) < 1e-5f && d[0] == s[0] && d[2] == s[2],
          std::string("db ") + aLabel + ": real frame 0 hips displaced 1.2 m on +Y only");
    bool others = true;
    for (uint32_t i = 2; i < n; ++i)
        if (parents[i] >= 2)
            others = others && SameBits(&dst[size_t(i) * 7], src + size_t(i) * 7, 7);
    Check(others, std::string("db ") + aLabel + ": all non-root bones of the real clip unchanged");
}
#endif
} // namespace

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    TestArgs();
    TestCompose();
    TestLifetime();
    TestAbortAndWatchdog();
    TestTable(true, "woman");
    TestTable(false, "man");
#ifdef UTPP_RAGDOLL_PROBE_DB
    if (argc >= 3)
    {
        TestDatabase(argv[1], utpp::mm::motion::kRigIdentityWoman, "woman");
        TestDatabase(argv[2], utpp::mm::motion::kRigIdentityMan, "man");
    }
#endif
    std::printf("ragdoll getup probe: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
