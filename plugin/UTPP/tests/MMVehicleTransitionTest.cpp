// Offline test of mm/VehicleTransition.hpp (mount pop CAR-5 / MOTO-3, exit hand-in MOTO-6 / CAR-1 step 5).
// Synthetic 12-bone database mapped onto a 20-record game pose (db b -> game b + 3), 60 Hz frames, 1 MHz ticks.
//   (a) cut frame: max per-bone jump <= 15 deg while |A - B| > 40 deg (and over the whole transition);
//   (b) monotonic decay of the offsets, < 1 deg at 0.20 s;
//   (c) after 0.20 s no write (records bit-exact game), only the Hips while a root offset runs, nothing after 0.30 s;
//   (d) root offset = -(world jump) on the cut frame (world-continuous Hips), 0 at 0.30 s, clamps 1.2 m / 120 deg;
//   (e) bike case, game hips +0.45 m and bent legs: hips height offset 0, leg chains on the game pose;
//   (f) exit: BeginExit returns the fresh capture, no written frame within 0.3 s at the squat pelvis height Q;
//   (g) a rig change aborts; (h) a perspective denial: no write, no capture, transition ended;
//   (i) on foot (no Begin* / SetVehicleCapture): nothing touched; (r) bone roles from names;
//   (j) Mapping owner: helper mapping jobs of the same frame never write the transition again (hold / offsets / root-only);
//   (k) production roles path (roles from the game names of the frozen snapshot's map): a bent leg chain goes to the game pose;
//   (l) vehicle hold veto: utpp_camera withdraws the pose-write permission 5-10 ms after the mount cut (recording 2026-10-28
//       05:05:24: one hold frame, then the raw game pose); with the mounted TPP flag the hold + decaying offsets run through the
//       denial (no capture), without it (FPP) the denial still ends the transition.
//   (s) seat pose switch (recording 2026-10-28 11:49:37, bike mount 11:55:08.141): the game's one-frame swap from the hunched
//       first-person enter end to the third-person seated pose (Hips +9 cm, Spine 17 deg / Spine3 29 deg at 11:55:09.879) is
//       handed in from the previous frame's pose (no step on the swap frame, small steps after starting at zero speed, game pose
//       bit-exact after kSeatSeconds); the enter clip's own motion and the one-frame neck / spine spike of the seating frame never
//       trigger it;
//       FPP (no mounted TPP flag), an exit and a second swap in the same mount write nothing. Review 2026-10-28: a frame hitch on
//       the enter clip (8.7120 -> 8.8112, 99 ms, Hips 21 cm) never fires (steps compared as rates), the swap still fires at 60 /
//       30 fps and after a 50 ms hitch; a sustained FPP before the swap and an FPP switch during the hand-in write nothing in FPP.
//   (p) mounted pop filter (review 2026-10-28): the recorded bike exit window (11:50:48.00-48.79, 66 / 86 / 71 deg one-frame pops)
//       at 60 / 30 / 20 fps and with hitches: 3 pops, smoothed; ordinary motion through a 3+ frame hitch never fires; sustained
//       fast motion at 20 fps never freezes the body > 0.1 s; a one-frame spike and back is re-anchored (no overshoot).
// Mutations (UTPP_VEHTRANS_MUTATION): 1 = blend off -> (a)(b) fail; 2 = root sign flipped -> (d) fails; 3 = no hips zero ->
// (e) fails; 4 = no main-job filter -> (j) fails; 5 = no roles from the game rig -> (k) fails; 7 = no seat pose switch
// guard -> (s) fails; 8 = raw per-frame steps (no frame-gap rates, pop re-fire without the freeze guard) -> (p)(s) fail;
// 9 = the seat guard follows its TPP latch only (no current mounted-TPP check) -> (s) fails. The last line lists the failed
// groups ("failed groups: [a][b]").
// Recording 2026-10-28 19-55-01 (bike mount / exit, the recorded log):
//   (t) T1 captures through the vehicle hold's denial (none reached the recorded edges); a first-person ride writes / captures nothing;
//   (u) T2 the unmount swap (Hips 1.011 -> 0.337 m, then a squat, vehicle bit still set): shown Hips never drops > 5 cm, steps
//       <= 3 cm / 15 deg per frame, hold through the denied frames after the edge, MM from the first allowed frame; a break without
//       an edge is blended back; the seated ride never latches;
//   (v) T3 a third-person exit under the vehicle hold without a capture Fills; (w) T4 standing entry: the 39 deg knee keeps its offset, no leg pop, the legs
//   keep moving through the held enter frame 0; (x) T5 car seat entry: the 25 deg rule; (y) T6 the sub-threshold slide (67 deg /
//   0.30 s) is followed by the root offset; (z) T7 the translation-only flash is smoothed by the pop filter's Hips term.
// Mutations: 10 -> (t), 11 -> (u), 12 -> (v), 13 -> (x), 14 -> (w), 15 -> (y), 16 -> (z).
// Recording 2026-11-02 02:01-02:50 (car doors): (E) hold cap - see the group comment at TestDoorCap (replay of the 30 measured rides through
//   the cap the door script computes; rides 4 / 9 / 30 clear the door, all other rides and bikes bit-identical, arrival order, no pop).
//   Mutation 22 (cap ignored) -> (E).
// Recording 2026-11-01 07-50-12 (car entry spin: ride 3 rootPeakYaw 63.4 deg, V parallel to the car, the workspot pivots the entity 75 deg
// in 0.28 s) and 07-47-01 (bike ride 2, 88.4 deg):
//   (D) replays of the logged entity paths (feed 60 Hz / frames 60 Hz and feed 30 Hz / frames 120 Hz) as CAR entries (BeginMount aCar): the yaw align
//       latch (> 30 deg held) with the per-frame blend: the shown yaw is monotonic toward the entry pose, no step above 1.35 x the entity's own
//       pivot step, no yaw offset after latch + kAlignSeconds (the old hold released it late over the tail while the clip turned back);
//       car ride 2 (8.7 deg) keeps the hold. Bikes (aCar false, the default) keep the hold bit-identically: the 85 deg pivot path and the c9
//       feed (1.9 deg steps, 211-228 deg/s) as a bike reproduce the shown yaw / root offset series of the 1dbef1a header (FNV-1a golden).
//       Mutation 20 (the old hold) -> (D).
// Recording 2026-10-28 23-05-01 (c9 clips, ride 2 mount 23:05:21.737, feed + body trace):
//   (A) T8 the planar slide (0.76 m / -10 deg in 0.28 s, entity still, then the clip turns back 10 deg over 0.7 s): the visual Hips
//       stays still during the slide; the root offset then moves it <= 3.0 cm per 1/60 s on top of the entity (time decay only for
//       the planar offset; the old yaw-progress factor: 3.6 cm per frame under planted feet; new 2.8), the offset ends bit-exact by the
//       tail (1.05 s, unchanged). Mutation 17 -> (A).
// Recording 2026-10-29 00-29-45 (c9b, enter_r, both mounts: a left-leg jerk at mount + 1.19 s, identical in both rides):
//   (B) T9 standing entry whose game legs hold frame 0 past the leg decay (legMoveAt=-1, recorded +0.33 s), then a fast clip-like leg
//       swing (10 deg per frame) through the root tail, the epoch expiry and beyond: from the end of the leg decay every leg record is
//       the game's own (bit-exact), only the Hips carries the root offset, its step against the game stays <= 0.5 cm / 0.5 deg per
//       frame through the expiry, nothing is written after it. Mutation 18 (leg offsets kept until the legs move) -> (B).
// Recording 2026-10-29 01-28-54 (first bike mount of the session, a one-frame upper-body pop on the seating edge, 44.617):
//   (C) replay of the recorded live game pose (s0, tests/vehtrans_neckpop_fixture.inc) in the production call order: the early
//       mapping job (FkSide under the vehicle hold's denial) on a pose without the pop, then the FK hook (NeckGuard) on the recorded
//       pose, plus a repeated FK call per frame. Ride 1 fires the seated neck cut once on the pop frame (seating edge = the frozen
//       frame 44.601), the shown Head steps <= 5 cm and Neck1 <= 8 deg per frame, the Head stays within 5 cm of the straight line
//       edge - 50 ms .. edge + 300 ms, no overshoot against the pop > 10 deg, nothing written before the pop or after
//       kSeatNeckSeconds, the repeated call writes nothing more; ride 2 (no pop), the enter clip without the pop and an FPP ride fire
//       nothing and write nothing; no FK-stage call during the mount transition. Mutation 19 (no neck guard) -> (C).
// Report 2026-11-03 (car entry from the driver side: V slides toward the rear of the car and through its side while sitting down):
//   (F) car glide limit (VehicleTransition.hpp GLIDE LIMIT) - replay of the same 34 logged rides without a door cap: the visible root (measured at the feed
//       instants, from 0.06 s on) stays below 3.05 m/s (the old hold: 6-16 m/s on the jump path), the held offset closes on the entity at <= 1.15 x kGlideSpeedMps
//       (2.7 m/s where the time budget binds: the 2.2 m Outlaw slide), the release is smooth (no growth, no snap at the end, offset 0 and epoch over by
//       kGlideEndSeconds), a mount with a door cap and a bike are bit-identical with and without the limit. Group E replays the pre-glide hold (g_glideOff).
//       Mutation 26 (no glide limit) -> (F).
// Review r2 (findings 17 / 20): (G) a door cap that arrives after the glide started is judged against the glided release (jump gate, cumulative skip / tail,
//       re-base bounded by the cap), and the glided cumulative tail keeps the offset fixed in the world after 1.05 s. Mutations 27-30 -> (G).
#include "../src/mm/VehicleTransition.hpp"
#include "TestFailFast.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

namespace
{
using namespace utpp::mm;
namespace vt = utpp::mm::vehtrans;

constexpr int64_t kTps = 1000000;
constexpr int64_t kFrame = 16667; // 60 Hz
constexpr uint32_t kGameCount = 20;
constexpr uint32_t kDb = 12;
constexpr uint32_t kFirst = 2;
constexpr uint64_t kRig = 0x1234;
constexpr uint64_t kOtherRig = 0x5678;
constexpr float kDeg = 3.14159265358979323846f / 180.0f;
enum Bone : uint32_t { Root, Traj, Hips, Spine, LUpLeg, LLeg, RUpLeg, RLeg, Spine1, LArm, RArm, Head };
const char* const kNames[kDb] = {"Root", "Trajectory", "Hips", "Spine", "LeftUpLeg", "LeftLeg",
                                 "RightUpLeg", "RightLeg", "Spine1", "LeftArm", "RightArm", "Head"};

int g_failures = 0;
std::set<char> g_failedGroups;
void Check(char aGroup, bool aOk, const std::string& aWhat)
{
    if (aOk)
        return;
    ++g_failures;
    g_failedGroups.insert(aGroup);
    std::printf("FAIL (%c) %s\n", aGroup, aWhat.c_str());
    UtppFailFastNote(std::string("FAIL (") + static_cast<char>(aGroup) + ") " + aWhat.c_str());
}

struct Q4 { float x = 0, y = 0, z = 0, w = 1; };
Q4 AxisAngle(float ax, float ay, float az, float deg)
{
    const float n = std::sqrt(ax * ax + ay * ay + az * az);
    const float h = 0.5f * deg * kDeg;
    const float s = std::sin(h) / n;
    return {ax * s, ay * s, az * s, std::cos(h)};
}
Q4 Mul(const Q4& a, const Q4& b)
{
    float o[4];
    const float qa[4] = {a.x, a.y, a.z, a.w}, qb[4] = {b.x, b.y, b.z, b.w};
    vt::detail::QMul(qa, qb, o);
    return {o[0], o[1], o[2], o[3]};
}
float AngleDeg(const Q4& a, const Q4& b)
{
    const float qa[4] = {a.x, a.y, a.z, a.w}, qb[4] = {b.x, b.y, b.z, b.w};
    return vt::detail::QAngle(qa, qb) / kDeg;
}
// same angle in double through atan2 (2 acos(|dot|) in float resolves only ~0.05-0.1 deg near 0): for "the same pose" checks
double AngleDegFine(const Q4& a, const Q4& b)
{
    // r = a^-1 b
    const double ax = -a.x, ay = -a.y, az = -a.z, aw = a.w;
    const double x = aw * b.x + ax * b.w + ay * b.z - az * b.y;
    const double y = aw * b.y - ax * b.z + ay * b.w + az * b.x;
    const double z = aw * b.z + ax * b.y - ay * b.x + az * b.w;
    const double w = aw * b.w - ax * b.x - ay * b.y - az * b.z;
    return 2.0 * std::atan2(std::sqrt(x * x + y * y + z * z), std::fabs(w)) * 180.0 / 3.14159265358979323846;
}

struct Pose
{
    std::array<std::array<float, 3>, kDb> p{};
    std::array<Q4, kDb> q{};
};
Pose GamePoseB() // upright entry pose
{
    Pose g;
    for (uint32_t b = 0; b < kDb; ++b)
        g.p[b] = {0.0f, 0.0f, 0.12f};
    g.p[Hips] = {0.0f, 0.0f, 1.0f};
    return g;
}
// car seat entry pose: B with the Hips low (a car seat is bent and low: below vt::kStandHipsZ, the 25 deg leg rule and the
// per-sample root jump path of a car mount)
Pose CarSeatPoseB()
{
    Pose g = GamePoseB();
    g.p[Hips] = {0.0f, 0.0f, 0.55f};
    return g;
}
Pose MmPoseA() // run stride; arms 60 deg, legs < 25 deg from B
{
    Pose a = GamePoseB();
    a.p[Hips] = {0.02f, 0.05f, 0.95f};
    a.q[Hips] = AxisAngle(1, 0, 0, 8);
    a.q[Spine] = AxisAngle(1, 0, 0, 12);
    a.q[LUpLeg] = AxisAngle(1, 0, 0, 20);
    a.q[LLeg] = AxisAngle(1, 0, 0, -15);
    a.q[RUpLeg] = AxisAngle(1, 0, 0, -20);
    a.q[RLeg] = AxisAngle(1, 0, 0, -10);
    a.q[Spine1] = AxisAngle(1, 0, 0, 10);
    a.q[LArm] = AxisAngle(0, 1, 0, 60);
    a.q[RArm] = AxisAngle(0, 1, 0, -60);
    a.q[Head] = AxisAngle(0, 0, 1, 45);
    return a;
}

using Records = std::vector<uint8_t>;
float* Rec(Records& r, uint32_t aDbBone) { return reinterpret_cast<float*>(r.data() + size_t(aDbBone + 3) * 48); }
const float* Rec(const Records& r, uint32_t aDbBone) { return reinterpret_cast<const float*>(r.data() + size_t(aDbBone + 3) * 48); }
Records MakeRecords(const Pose& g)
{
    Records r(size_t(kGameCount) * 48, 0);
    for (uint32_t i = 0; i < kGameCount; ++i)
    {
        float* f = reinterpret_cast<float*>(r.data() + size_t(i) * 48);
        f[0] = 0.01f * float(i); f[1] = 0.5f; f[2] = -0.25f; f[3] = 0.0f;
        f[4] = 0.0f; f[5] = 0.0f; f[6] = 0.0f; f[7] = 1.0f;
        f[8] = 1.0f; f[9] = 1.0f; f[10] = 1.0f; f[11] = 0.0f;
    }
    for (uint32_t b = 0; b < kDb; ++b)
    {
        float* f = Rec(r, b);
        f[0] = g.p[b][0]; f[1] = g.p[b][1]; f[2] = g.p[b][2];
        f[4] = g.q[b].x; f[5] = g.q[b].y; f[6] = g.q[b].z; f[7] = g.q[b].w;
    }
    return r;
}
Q4 RecQ(const Records& r, uint32_t b) { const float* f = Rec(r, b); return {f[4], f[5], f[6], f[7]}; }

integ::PoseSnapshot MakeSnap(const Pose& m, float aWeight)
{
    integ::PoseSnapshot s;
    s.weight = aWeight;
    s.numBones = kDb;
    s.firstBone = kFirst;
    s.rig = kRig;
    s.sequence = 1;
    for (uint32_t b = 0; b < integ::kMaxMMBones; ++b)
        s.gameIndex[b] = b < kDb ? int32_t(b + 3) : -1;
    for (uint32_t b = 0; b < kDb; ++b)
    {
        float* d = &s.pose[size_t(b) * 7];
        d[0] = m.p[b][0]; d[1] = m.p[b][1]; d[2] = m.p[b][2];
        d[3] = m.q[b].x; d[4] = m.q[b].y; d[5] = m.q[b].z; d[6] = m.q[b].w;
    }
    for (uint32_t b : {Spine1, LArm, RArm, Head})
        s.upperBits[b >> 5] |= 1u << (b & 31u);
    s.upperScale = 1.0f;
    return s;
}

int64_t g_clock = 1000 * kTps;
int64_t NextEpochTime() { g_clock += 10 * kTps; return g_clock; }

// world rotation / position of the Hips for an entity yaw / position (Z-up, CCW yaw)
Q4 HipsWorldQ(const Records& r, float aYawDeg) { return Mul(AxisAngle(0, 0, 1, aYawDeg), RecQ(r, Hips)); }
void HipsWorldP(const Records& r, const float aPos[3], float aYawDeg, float aOut[3])
{
    const float* f = Rec(r, Hips);
    float wx = 0, wy = 0;
    vt::detail::CharacterToWorld(f[0], f[1], aYawDeg * kDeg, wx, wy);
    aOut[0] = aPos[0] + wx; aOut[1] = aPos[1] + wy; aOut[2] = aPos[2] + f[2];
}

void SetRoles()
{
    vt::SetBoneRoles(vt::BuildBoneRoles(kDb, kFirst, [](uint32_t b) { return std::string_view(kNames[b]); }));
}

// ------------------------------------------------------------------------------------------------ (i) on foot
void TestOnFoot()
{
    Check('i', !vt::Wanted() && !vt::MountActive(), "fresh state wants nothing");
    const Records game = MakeRecords(GamePoseB());
    Records r = game;
    const int64_t t = NextEpochTime();
    Check('i', vt::FkSide(r.data(), kGameCount, kRig, t, true) == 0, "FkSide returns 0 on foot");
    Check('i', vt::ApplyIfActive(r.data(), kGameCount, kRig, t) == 0, "ApplyIfActive returns 0 on foot");
    Check('i', !vt::CaptureIfWanted(r.data(), kGameCount, kRig, t), "no capture on foot");
    Check('i', std::memcmp(r.data(), game.data(), game.size()) == 0, "records byte-identical on foot");
    integ::PoseSnapshot s = MakeSnap(MmPoseA(), 1.0f);
    const integ::PoseSnapshot before = s;
    Check('i', !vt::HandInBlend(s, t) && std::memcmp(&s, &before, sizeof s) == 0, "HandInBlend leaves the snapshot alone");
    integ::PoseSnapshot out;
    Check('i', !vt::BeginExit(t, &out), "BeginExit without a capture is false");
    Check('i', vt::g_counters.holdWrites.load() == 0 && vt::g_counters.offsetWrites.load() == 0 &&
                   vt::g_counters.captures.load() == 0, "no counters on foot");
}

// ------------------------------------------------------------------------------------------------ (r) roles
void TestRoles()
{
    const vt::BoneRoles r = vt::BuildBoneRoles(kDb, kFirst, [](uint32_t b) { return std::string_view(kNames[b]); });
    Check('r', r.valid == 1 && r.hips == int32_t(Hips), "hips found");
    Check('r', r.chain[LUpLeg] == vt::kChainLeftLeg && r.chain[LLeg] == vt::kChainLeftLeg &&
                   r.chain[RUpLeg] == vt::kChainRightLeg && r.chain[RLeg] == vt::kChainRightLeg, "leg chains");
    Check('r', r.chain[Spine] == vt::kChainNone && r.chain[LArm] == vt::kChainNone && r.chain[Hips] == vt::kChainNone,
          "no chain for spine / arms / hips");
}

// ------------------------------------------------------------------------------------------------ (a)(b)(c)(d) car mount
void TestCarMount()
{
    SetRoles();
    const Pose a = MmPoseA(), b = CarSeatPoseB();
    const integ::PoseSnapshot snapA = MakeSnap(a, 1.0f);
    const Records game = MakeRecords(b);
    const float p0[3] = {10.0f, 20.0f, 0.0f}, yaw0 = 30.0f;
    const float p1[3] = {10.8f, 20.0f, 0.0f}, yaw1 = 100.0f; // workspot jump: 0.8 m, +70 deg
    // previous (normal MM) frame
    Records prev = game;
    integ::BlendIntoRecords(prev.data(), kGameCount, snapA, 1.0f);
    float prevHipsW[3];
    HipsWorldP(prev, p0, yaw0, prevHipsW);
    Q4 prevQ[kDb];
    for (uint32_t k = kFirst; k < kDb; ++k)
        prevQ[k] = k == Hips ? HipsWorldQ(prev, yaw0) : RecQ(prev, k);
    {
        float maxCut = 0.0f;
        for (uint32_t k = kFirst; k < kDb; ++k)
            maxCut = std::fmax(maxCut, k == Hips ? AngleDeg(prevQ[k], HipsWorldQ(game, yaw1)) : AngleDeg(prevQ[k], RecQ(game, k)));
        Check('a', maxCut > 40.0f, "fixture: |A-B| > 40 deg (" + std::to_string(maxCut) + ")");
    }

    const int64_t t0 = NextEpochTime();
    vt::BeginMount(snapA, p0, yaw0, t0);
    vt::UpdateWorld(p1, yaw1, t0);
    float ro[3] = {};
    {
        float ex = 0, ey = 0;
        integ::WorldToCharacter(p0[0] - p1[0], p0[1] - p1[1], yaw1 * kDeg, ex, ey);
        const bool have = vt::RootOffset(t0, ro);
        Check('d', have && std::fabs(ro[0] - ex) < 1e-4f && std::fabs(ro[1] - ey) < 1e-4f && std::fabs(ro[2] - (yaw0 - yaw1)) < 1e-3f,
              "root offset = -(world jump) on the cut frame (" + std::to_string(ro[0]) + "," + std::to_string(ro[1]) + "," +
                  std::to_string(ro[2]) + ")");
    }

    float maxJumpAll = 0.0f, cutJump = -1.0f, maxHipsStep = 0.0f;
    float lastOff[kDb];
    for (float& v : lastOff) v = 1e9f;
    bool monotonic = true, decayStartedFull = false, below1 = true, onlyHipsLate = true, nothingAfter = true;
    for (int k = 0; k <= 30; ++k)
    {
        const int64_t t = t0 + k * kFrame;
        const float ts = float(k * kFrame) / float(kTps);
        Records r = game;
        const uint32_t n = vt::FkSide(r.data(), kGameCount, kRig, t, true);
        if (k == 0)
        {
            // a second animation job 0.2 ms later in the same frame writes the same frozen pose (only the root offset moved by
            // ~1e-6) and does not advance the hold (checked by (b): the offsets start on frame kMountHoldFrames)
            Records r2 = game;
            vt::FkSide(r2.data(), kGameCount, kRig, t + 200, true);
            bool same = true;
            for (uint32_t j = 0; j < kGameCount * 12; ++j)
                same = same && std::fabs(reinterpret_cast<const float*>(r.data())[j] - reinterpret_cast<const float*>(r2.data())[j]) < 1e-5f;
            Check('a', same, "repeated job in the cut frame writes the same pose");
            float w[3];
            HipsWorldP(r, p1, yaw1, w);
            Check('d', std::fabs(w[0] - prevHipsW[0]) < 1e-4f && std::fabs(w[1] - prevHipsW[1]) < 1e-4f,
                  "Hips stay where they were in the world on the cut frame");
        }
        float frameMax = 0.0f;
        for (uint32_t j = kFirst; j < kDb; ++j)
        {
            const Q4 cur = j == Hips ? HipsWorldQ(r, yaw1) : RecQ(r, j);
            frameMax = std::fmax(frameMax, AngleDeg(cur, prevQ[j]));
            prevQ[j] = cur;
        }
        float hw[3];
        HipsWorldP(r, p1, yaw1, hw);
        maxHipsStep = std::fmax(maxHipsStep, std::hypot(hw[0] - prevHipsW[0], hw[1] - prevHipsW[1]));
        for (int c = 0; c < 3; ++c)
            prevHipsW[c] = hw[c];
        if (k == 0)
            cutJump = frameMax;
        maxJumpAll = std::fmax(maxJumpAll, frameMax);
        // (b) offsets of the non-root bones relative to the live game pose
        if (k >= int(vt::kMountHoldFrames))
        {
            for (uint32_t j : {Spine, Spine1, LArm, RArm, Head, LUpLeg})
            {
                const float off = AngleDeg(RecQ(r, j), RecQ(game, j));
                if (k == int(vt::kMountHoldFrames) && (j == LArm || j == RArm) && off > 40.0f)
                    decayStartedFull = true;
                if (off > lastOff[j] + 1e-3f)
                    monotonic = false;
                lastOff[j] = off;
                if (ts >= vt::kMountSeconds && off >= 1.0f)
                    below1 = false;
            }
        }
        // (c) after 0.20 s only the Hips (root offset) may differ; after 0.30 s nothing
        if (ts >= vt::kMountSeconds)
        {
            for (uint32_t j = 0; j < kGameCount; ++j)
                if (j != Hips + 3 && std::memcmp(r.data() + j * 48, game.data() + j * 48, 48) != 0)
                    onlyHipsLate = false;
        }
        if (ts >= vt::kRootSeconds + 1e-6f && (n != 0 || std::memcmp(r.data(), game.data(), game.size()) != 0))
            nothingAfter = false;
        if (k == 18) // 0.30 s
        {
            float z[3] = {};
            Check('d', !vt::RootOffset(t, z) && z[0] == 0.0f && z[1] == 0.0f && z[2] == 0.0f, "root offset 0 at 0.30 s");
        }
    }
    std::printf("car mount: cut-frame jump %.3f deg, max jump %.3f deg, max hips world step %.4f m\n", double(cutJump),
                double(maxJumpAll), double(maxHipsStep));
    Check('a', cutJump >= 0.0f && cutJump <= 15.0f, "cut-frame max per-bone jump <= 15 deg (" + std::to_string(cutJump) + ")");
    Check('a', maxJumpAll <= 15.0f, "max per-bone jump over the transition <= 15 deg (" + std::to_string(maxJumpAll) + ")");
    Check('a', maxHipsStep < 0.10f, "Hips world step per frame < 0.10 m (" + std::to_string(maxHipsStep) + ")");
    Check('b', decayStartedFull, "offsets start at the full A-B difference (> 40 deg) after the hold");
    Check('b', monotonic, "offsets decay monotonically");
    Check('b', below1, "offsets < 1 deg at 0.20 s");
    Check('c', onlyHipsLate, "after 0.20 s only the Hips record (root offset) differs from the game pose");
    Check('c', nothingAfter, "after 0.30 s no write (records bit-exact game)");
    Check('c', !vt::MountActive(), "transition epoch expired");

    // (c) without a world jump nothing is written from 0.20 s on
    {
        const int64_t s0 = NextEpochTime();
        vt::BeginMount(snapA, p0, yaw0, s0);
        bool ok = true, wrote = false;
        for (int k = 0; k <= 20; ++k)
        {
            const int64_t t = s0 + k * kFrame;
            vt::UpdateWorld(p0, yaw0, t);
            Records r = game;
            const uint32_t n = vt::FkSide(r.data(), kGameCount, kRig, t, true);
            if (k < 12 && n > 0)
                wrote = true;
            if (float(k * kFrame) / float(kTps) >= vt::kMountSeconds &&
                (n != 0 || std::memcmp(r.data(), game.data(), game.size()) != 0))
                ok = false;
        }
        UTPP_MUT_IF (vt::kMutation != 1)
            Check('c', wrote, "no-jump mount writes during the blend");
        Check('c', ok, "no-jump mount: after 0.20 s no write, records bit-exact");
        Check('c', !vt::MountActive(), "no-jump mount: epoch expired");
    }
    // (d) clamps
    {
        const int64_t s0 = NextEpochTime();
        vt::BeginMount(snapA, p0, 0.0f, s0);
        const float far[3] = {p0[0] + 3.0f, p0[1], 0.0f};
        vt::UpdateWorld(far, 150.0f, s0 + kFrame);
        float o[3] = {};
        const bool have = vt::RootOffset(s0 + kFrame, o);
        const float d = std::hypot(o[0], o[1]);
        Check('d', have && std::fabs(d - vt::kRootMaxDistance) < 1e-4f && std::fabs(std::fabs(o[2]) - vt::kRootMaxYawDeg) < 1e-3f,
              "root offset clamped to 1.2 m / 120 deg (" + std::to_string(d) + ", " + std::to_string(o[2]) + ")");
        Check('d', have && o[2] < 0.0f, "clamped yaw keeps the sign of -(jump)");
        // a small step (< 0.15 m, < 15 deg) is motion, not a jump
        vt::Abort();
        const int64_t s1 = NextEpochTime();
        vt::BeginMount(snapA, p0, 0.0f, s1);
        const float near[3] = {p0[0] + 0.1f, p0[1], 0.0f};
        vt::UpdateWorld(near, 10.0f, s1 + kFrame);
        Check('d', !vt::RootOffset(s1 + kFrame, o), "small step is not compensated");
        vt::Abort();
    }
}

// ------------------------------------------------------------------------------------------------ (e) bike mount
void TestBikeMount()
{
    SetRoles();
    const Pose a = MmPoseA();
    Pose bike = GamePoseB();
    bike.p[Hips] = {0.0f, -0.10f, 1.45f}; // seated on the bike: +0.45 m
    bike.q[LUpLeg] = AxisAngle(1, 0, 0, 80);
    bike.q[LLeg] = AxisAngle(1, 0, 0, -90);
    bike.q[RUpLeg] = AxisAngle(1, 0, 0, 80);
    bike.q[RLeg] = AxisAngle(1, 0, 0, -90);
    const integ::PoseSnapshot snapA = MakeSnap(a, 1.0f);
    const Records game = MakeRecords(bike);
    const float p0[3] = {0.0f, 0.0f, 0.0f};
    const float p1[3] = {0.3f, 0.2f, 0.45f}; // the jump has a vertical part
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(snapA, p0, 0.0f, t0);
    vt::UpdateWorld(p1, 0.0f, t0 + kFrame / 2);
    bool hipsZ = true, legsGame = true, holdMm = true;
    for (int k = 0; k <= 18; ++k)
    {
        const int64_t t = t0 + k * kFrame;
        Records r = game;
        vt::FkSide(r.data(), kGameCount, kRig, t, true);
        const float z = Rec(r, Hips)[2];
        if (k < int(vt::kMountHoldFrames))
        {
            if (std::fabs(z - a.p[Hips][2]) > 1e-5f)
                holdMm = false; // the hold shows the frozen MM pose
            continue;
        }
        if (std::fabs(z - bike.p[Hips][2]) > 1e-6f)
        {
            hipsZ = false;
            std::printf("  bike frame %d hips z %.4f (game %.4f)\n", k, double(z), double(bike.p[Hips][2]));
        }
        for (uint32_t j : {LUpLeg, LLeg, RUpLeg, RLeg})
            if (std::memcmp(Rec(r, j) + 4, Rec(game, j) + 4, 16) != 0)
                legsGame = false;
    }
    Check('e', hipsZ, "hips height offset 0 after the hold (game hips +0.45 m)");
    Check('e', legsGame, "leg chains with >= 25 deg difference stay on the game pose");
    Check('e', vt::kMutation == 1 || holdMm, "hold frames show the frozen MM hips");
    vt::Abort();
}

// ------------------------------------------------------------------------------------------------ (f) exit hand-in
void TestExit()
{
    SetRoles();
    Pose standS = GamePoseB(); // standing end of the exit clip
    standS.p[Hips] = {0.0f, 0.0f, 0.97f};
    standS.q[LArm] = AxisAngle(0, 1, 0, 20);
    Pose squatQ = GamePoseB(); // live game pose after the unmount
    squatQ.p[Hips] = {0.0f, 0.1f, 0.55f};
    squatQ.q[LUpLeg] = AxisAngle(1, 0, 0, 100);
    squatQ.q[RUpLeg] = AxisAngle(1, 0, 0, 100);
    squatQ.q[LArm] = AxisAngle(0, 1, 0, -70);
    Pose mmM = GamePoseB(); // MM standing
    mmM.p[Hips] = {0.0f, 0.02f, 0.98f};
    mmM.q[LArm] = AxisAngle(0, 1, 0, 50);
    const float p0[3] = {0, 0, 0};

    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
    vt::SetVehicleCapture(true, kRig);
    const Records gameS = MakeRecords(standS);
    int64_t t = t0;
    for (int k = 0; k < 60; ++k, t += kFrame)
    {
        Records r = gameS;
        vt::FkSide(r.data(), kGameCount, kRig, t, true);
    }
    Check('f', vt::Wanted() && !vt::MountActive(), "riding: capture on, mount transition over");
    const int64_t tE = t;
    integ::PoseSnapshot cap;
    const bool handed = vt::BeginExit(tE, &cap);
    Check('f', handed, "BeginExit returns the fresh capture");
    Check('f', handed && std::fabs(cap.pose[Hips * 7 + 2] - standS.p[Hips][2]) < 1e-6f && cap.weight == 1.0f && cap.rig == kRig,
          "capture = live standing pose S");
    Check('f', !vt::Wanted(), "BeginExit ends the capture");
    const Records gameQ = MakeRecords(squatQ);
    const integ::PoseSnapshot snapM = MakeSnap(mmM, 1.0f);
    bool neverQ = true, firstIsS = false, endsOnM = true;
    float minDz = 1e9f;
    for (int k = 0; k <= 18; ++k)
    {
        const int64_t tk = tE + k * kFrame;
        const float ts = float(k * kFrame) / float(kTps);
        integ::PoseSnapshot s = snapM;
        const bool active = vt::HandInBlend(s, tk);
        Records r = gameQ;
        integ::BlendIntoRecords(r.data(), kGameCount, s, 1.0f); // MMDriver: GateFader::Fill -> weight 1 on the edge frame
        const float z = Rec(r, Hips)[2];
        minDz = std::fmin(minDz, std::fabs(z - squatQ.p[Hips][2]));
        if (std::fabs(z - squatQ.p[Hips][2]) < 0.30f)
            neverQ = false;
        if (k == 0)
            firstIsS = active && std::fabs(z - standS.p[Hips][2]) < 1e-5f && AngleDeg(RecQ(r, LArm), standS.q[LArm]) < 0.1f;
        if (ts >= vt::kHandInSeconds && (active || std::memcmp(&s, &snapM, sizeof s) != 0))
            endsOnM = false;
    }
    Check('f', neverQ, "no frame within 0.3 s near the squat pelvis height Q (min |z-Q| " + std::to_string(minDz) + ")");
    Check('f', firstIsS, "edge frame = captured pose S (no pop)");
    Check('f', endsOnM, "hand-in done by 0.25 s (snapshot = MM)");
    // stale capture
    {
        const int64_t s0 = NextEpochTime();
        vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, s0);
        vt::SetVehicleCapture(true, kRig);
        Records r = gameS;
        vt::FkSide(r.data(), kGameCount, kRig, s0 + kTps, true);
        integ::PoseSnapshot o;
        Check('f', !vt::BeginExit(s0 + kTps + kTps / 5, &o), "capture older than 0.10 s is not handed in");
        integ::PoseSnapshot s = snapM;
        Check('f', !vt::HandInBlend(s, s0 + kTps + kTps / 5), "no hand-in after a stale capture");
    }
    // a capture of another rig is never taken
    {
        const int64_t s0 = NextEpochTime();
        vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, s0);
        vt::SetVehicleCapture(true, kRig);
        Records r = gameS;
        const uint64_t before = vt::g_counters.captures.load();
        vt::CaptureIfWanted(r.data(), kGameCount, kOtherRig, s0 + kTps);
        Check('f', vt::g_counters.captures.load() == before, "no capture for another rig");
        Check('f', std::memcmp(r.data(), gameS.data(), gameS.size()) == 0, "capture never writes");
        vt::Abort();
    }
}

// ------------------------------------------------------------------------------------------------ (g) rig change
void TestRigChange()
{
    SetRoles();
    const Records game = MakeRecords(GamePoseB());
    const float p0[3] = {0, 0, 0};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
    Records r = game;
    Check('g', vt::FkSide(r.data(), kGameCount, kOtherRig, t0, true) == 0, "other rig: no write");
    Check('g', std::memcmp(r.data(), game.data(), game.size()) == 0, "other rig: records untouched");
    Check('g', !vt::MountActive(), "rig change aborts the transition");
    Check('g', vt::FkSide(r.data(), kGameCount, kRig, t0 + kFrame, true) == 0 &&
                   std::memcmp(r.data(), game.data(), game.size()) == 0, "aborted transition does not resume");
}

// ------------------------------------------------------------------------------------------------ (h) perspective denied
void TestPerspective()
{
    SetRoles();
    const Records game = MakeRecords(GamePoseB());
    const float p0[3] = {0, 0, 0};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
    vt::SetVehicleCapture(true, kRig);
    const uint64_t caps = vt::g_counters.captures.load();
    Records r = game;
    Check('h', vt::FkSide(r.data(), kGameCount, kRig, t0, false) == 0, "denied: no write");
    Check('h', std::memcmp(r.data(), game.data(), game.size()) == 0, "denied: records untouched");
    Check('h', vt::g_counters.captures.load() == caps, "denied: no capture");
    Check('h', !vt::MountActive(), "denied: transition ended");
    Check('h', vt::FkSide(r.data(), kGameCount, kRig, t0 + kFrame, true) == 0 &&
                   std::memcmp(r.data(), game.data(), game.size()) == 0, "allowed again: the ended transition does not resume");
    vt::Abort();
    Check('h', !vt::Wanted(), "Abort clears everything");
}

// ------------------------------------------------------------------------------------------------ (l) vehicle hold veto
void TestVehicleVeto()
{
    SetRoles();
    const Records game = MakeRecords(GamePoseB());
    const float p0[3] = {0, 0, 0};
    {
        const int64_t t0 = NextEpochTime();
        vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
        vt::SetVehicleCapture(true, kRig);
        vt::SetMountTpp(true);
        bool wroteEarly = true, decayStartedFull = false, monotonic = true, active5 = false, capOnWrite = false, capAfter = false;
        float last = 1e9f, offEnd = 1e9f;
        for (int k = 0; k <= 30; ++k)
        {
            Records r = game;
            const uint64_t capsK = vt::g_counters.captures.load();
            const uint32_t n = vt::FkSide(r.data(), kGameCount, kRig, t0 + k * kFrame, false); // the veto is on from the cut
            const bool captured = vt::g_counters.captures.load() != capsK;
            if (vt::MountActive() && n > 0 && captured)
                capOnWrite = true;
            if (!vt::MountActive() && captured)
                capAfter = true;
            const float off = AngleDeg(RecQ(r, LArm), RecQ(game, LArm));
            if (k < 8)
                wroteEarly = wroteEarly && n > 0;
            if (k == int(vt::kMountHoldFrames) && off > 40.0f)
                decayStartedFull = true;
            if (k >= int(vt::kMountHoldFrames))
            {
                if (off > last + 1e-3f)
                    monotonic = false;
                last = off;
            }
            if (k == 5)
                active5 = vt::MountActive();
            if (float(k * kFrame) / float(kTps) >= vt::kMountSeconds && offEnd > off)
                offEnd = off;
        }
        Check('l', wroteEarly, "vehicle veto: the hold / offset frames are written through the denial");
        Check('l', decayStartedFull, "vehicle veto: the arms start from the MM pose (> 40 deg) after the hold frames");
        Check('l', monotonic && offEnd < 1.0f, "vehicle veto: monotonic decay to < 1 deg at 0.20 s");
        Check('l', active5, "vehicle veto: the transition is still running 0.08 s after the cut");
        Check('l', !capOnWrite, "vehicle veto: no capture on the frames the transition writes");
        Check('t', vt::kMutation == 1 || capAfter, "T1 vehicle veto: the ride capture runs through the denial once the transition expired");
        Check('l', !vt::MountActive(), "vehicle veto: the transition expires normally");
        vt::Abort();
    }
    {
        // FPP (no mounted TPP flag, or it was switched off): the denial ends the transition at once
        const int64_t t0 = NextEpochTime();
        vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
        vt::SetVehicleCapture(true, kRig);
        vt::SetMountTpp(true);
        Records r = game;
        Check('l', vt::FkSide(r.data(), kGameCount, kRig, t0, true) > 0, "fixture: the allowed cut frame writes the hold");
        vt::SetMountTpp(false);
        Records r2 = game;
        Check('l', vt::FkSide(r2.data(), kGameCount, kRig, t0 + kFrame, false) == 0 &&
                       std::memcmp(r2.data(), game.data(), game.size()) == 0 && !vt::MountActive(),
              "mounted TPP flag off: the denial ends the transition and writes nothing");
        vt::Abort();
        Check('l', !vt::Wanted(), "Abort clears everything");
    }
}

// ------------------------------------------------------------------------------------------------ (j) helper mapping jobs
// Mapping owner (PoseHook ApplyEarlyMapping): per animation frame the main mapping job (fresh game records, main anchors, every
// core bone covered) and then helper jobs that rewrite only helper destinations, so the core records still hold the transition
// write of the main job. A helper job must leave them alone in the hold, the offset and the root-only phase.
uint64_t GameHash(int32_t aGi)
{
    return aGi >= 3 && aGi < int32_t(kDb + 3) ? integ::CNameHash(kNames[aGi - 3]) : 0ull;
}
void TestHelperJobs()
{
    SetRoles();
    const Pose a = MmPoseA(), b = CarSeatPoseB();
    const integ::PoseSnapshot snapA = MakeSnap(a, 1.0f);
    const Records game = MakeRecords(b);
    // main source: the 12 database names (game 3..14) + the five main anchors (game 15..19); helper sources: no anchors, or the
    // anchors with only the helper destinations
    std::vector<uint64_t> mainNames, helperNames, anchorHelperNames;
    std::vector<uint32_t> mainPairs, helperPairs, anchorHelperPairs;
    for (uint32_t k = 0; k < kDb; ++k)
    {
        mainNames.push_back(integ::CNameHash(kNames[k]));
        mainPairs.push_back(k);
        mainPairs.push_back(k + 3);
    }
    const char* const anchors[5] = {"reference_joint", "WeaponLeft", "WeaponRight", "LeftEye", "RightEye"};
    for (uint32_t k = 0; k < 5; ++k)
    {
        mainNames.push_back(integ::CNameHash(anchors[k]));
        mainPairs.push_back(kDb + k);
        mainPairs.push_back(15 + k);
        anchorHelperNames.push_back(integ::CNameHash(anchors[k]));
        anchorHelperPairs.push_back(k);
        anchorHelperPairs.push_back(15 + k);
    }
    for (uint32_t k = 0; k < 3; ++k)
    {
        helperNames.push_back(integ::CNameHash(std::string("helper_") + char('a' + k)));
        helperPairs.push_back(k);
        helperPairs.push_back(k); // game 0..2: helper destinations
        anchorHelperNames.push_back(integ::CNameHash(std::string("helper_") + char('a' + k)));
        anchorHelperPairs.push_back(5 + k);
        anchorHelperPairs.push_back(k);
    }
    Check('j', vt::MainMappingJob(mainNames, mainPairs, kGameCount, snapA), "the main job is recognised");
    Check('j', !vt::MainMappingJob(helperNames, helperPairs, kGameCount, snapA), "a helper job without the main anchors is not");
    Check('j', !vt::MainMappingJob(anchorHelperNames, anchorHelperPairs, kGameCount, snapA),
          "a job that does not cover every core bone is not");
    Check('j', vt::FkSideMapping(nullptr, 0, kRig, NextEpochTime(), true, mainNames, mainPairs) == 0 && !vt::Wanted(),
          "on foot: FkSideMapping does nothing");

    const float p0[3] = {10.0f, 20.0f, 0.0f}, yaw0 = 30.0f;
    const float p1[3] = {10.8f, 20.0f, 0.0f}, yaw1 = 100.0f;
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(snapA, p0, yaw0, t0);
    vt::UpdateWorld(p1, yaw1, t0);
    const uint64_t hold0 = vt::g_counters.holdWrites.load(), off0 = vt::g_counters.offsetWrites.load(),
                   root0 = vt::g_counters.rootOnlyWrites.load();
    bool helperUntouched = true, helperZero = true, mainLikeFk = true;
    int holdF = 0, offF = 0, rootF = 0;
    for (int k = 0; k <= 20; ++k)
    {
        const int64_t t = t0 + k * kFrame;
        const uint64_t h = vt::g_counters.holdWrites.load(), o = vt::g_counters.offsetWrites.load(),
                       ro = vt::g_counters.rootOnlyWrites.load();
        Records r = game;
        vt::FkSideMapping(r.data(), kGameCount, kRig, t, true, mainNames, mainPairs);
        holdF += vt::g_counters.holdWrites.load() > h ? 1 : 0;
        offF += vt::g_counters.offsetWrites.load() > o ? 1 : 0;
        rootF += vt::g_counters.rootOnlyWrites.load() > ro ? 1 : 0;
        // the same frame through FkSide on fresh records: identical (the main job path is FkSide)
        Records f = game;
        vt::FkSide(f.data(), kGameCount, kRig, t, true);
        for (uint32_t j = 0; j < kGameCount * 12; ++j)
            mainLikeFk = mainLikeFk && std::fabs(reinterpret_cast<const float*>(r.data())[j] - reinterpret_cast<const float*>(f.data())[j]) < 1e-5f;
        for (int helper = 0; helper < 2; ++helper)
        {
            const Records before = r;
            const uint32_t n = helper == 0 ? vt::FkSideMapping(r.data(), kGameCount, kRig, t + 300, true, helperNames, helperPairs)
                                           : vt::FkSideMapping(r.data(), kGameCount, kRig, t + 400, true, anchorHelperNames,
                                                               anchorHelperPairs);
            helperZero = helperZero && n == 0;
            if (std::memcmp(r.data(), before.data(), before.size()) != 0)
            {
                helperUntouched = false;
                std::printf("  helper job %d wrote the already written records on frame %d (%.3f s)\n", helper, k,
                            double(k * kFrame) / double(kTps));
            }
        }
    }
    std::printf("helper jobs: main job frames hold %d offsets %d root-only %d\n", holdF, offF, rootF);
    Check('j', vt::kMutation == 1 || (holdF >= 1 && offF >= 3), "fixture: hold and offset phases ran on the main job");
    Check('j', rootF >= 3, "fixture: the root-only phase ran on the main job");
    Check('j', (vt::kMutation == 1 || (vt::g_counters.holdWrites.load() > hold0 && vt::g_counters.offsetWrites.load() > off0)) &&
                   vt::g_counters.rootOnlyWrites.load() > root0, "fixture: transition counters advanced");
    Check('j', mainLikeFk, "the main mapping job writes exactly what FkSide writes");
    Check('j', helperZero, "helper jobs return 0");
    Check('j', helperUntouched, "helper jobs leave the already written core records untouched (hold, offsets, root-only)");
    vt::Abort();
}

// ------------------------------------------------------------------------------------------------ (k) production roles path
// What MMDriver does at the mount: roles of the frozen snapshot from the game names of its own map (BuildBoneRolesFromGame),
// then BeginMount. Seated legs: UpLeg 40 deg off the MM pose, Leg only 10 deg -> the whole chain goes to the game pose (the
// per-bone fallback without roles would keep the 10 deg shin offset on the seated thigh).
void TestProductionRoles()
{
    const Pose a = MmPoseA();
    Pose seat = CarSeatPoseB();
    seat.q[LUpLeg] = AxisAngle(1, 0, 0, 60);  // MM 20  -> 40 deg
    seat.q[LLeg] = AxisAngle(1, 0, 0, -5);    // MM -15 -> 10 deg
    seat.q[RUpLeg] = AxisAngle(1, 0, 0, 20);  // MM -20 -> 40 deg
    seat.q[RLeg] = AxisAngle(1, 0, 0, 0);     // MM -10 -> 10 deg
    const integ::PoseSnapshot snapA = MakeSnap(a, 1.0f);
    const Records game = MakeRecords(seat);
    const vt::BoneRoles roles = vt::BuildBoneRolesFromGame(snapA, GameHash);
    Check('k', roles.valid == 1 && roles.hips == int32_t(Hips), "roles from the game names: Hips found");
    Check('k', roles.chain[LUpLeg] == vt::kChainLeftLeg && roles.chain[LLeg] == vt::kChainLeftLeg &&
                   roles.chain[RUpLeg] == vt::kChainRightLeg && roles.chain[RLeg] == vt::kChainRightLeg &&
                   roles.chain[Spine] == vt::kChainNone && roles.chain[LArm] == vt::kChainNone,
          "roles from the game names: leg chains, nothing else");
    const float p0[3] = {0, 0, 0};
    // r = 0: production (roles from the game names); r = 1: no roles at all (the per-bone fallback, pinned)
    bool chainGame[2] = {true, true}, shinKept[2] = {false, false}, armOffset[2] = {false, false};
    for (int run = 0; run < 2; ++run)
    {
        vt::SetBoneRoles(run == 0 ? vt::BuildBoneRolesFromGame(snapA, GameHash) : vt::BoneRoles{});
        const int64_t t0 = NextEpochTime();
        vt::BeginMount(snapA, p0, 0.0f, t0);
        for (int k = 0; k <= 11; ++k)
        {
            const int64_t t = t0 + k * kFrame;
            Records r = game;
            vt::FkSide(r.data(), kGameCount, kRig, t, true);
            if (k < int(vt::kMountHoldFrames))
                continue;
            for (uint32_t j : {LUpLeg, LLeg, RUpLeg, RLeg})
                if (std::memcmp(Rec(r, j) + 4, Rec(game, j) + 4, 16) != 0)
                    chainGame[run] = false;
            if (k == int(vt::kMountHoldFrames) && AngleDeg(RecQ(r, LLeg), RecQ(game, LLeg)) > 5.0f)
                shinKept[run] = true;
            if (k == int(vt::kMountHoldFrames) && AngleDeg(RecQ(r, LArm), RecQ(game, LArm)) > 40.0f)
                armOffset[run] = true;
        }
        vt::Abort();
    }
    Check('k', chainGame[0], "production roles: a leg chain with one bone >= 25 deg off stays on the game pose (thigh and shin)");
    Check('k', vt::kMutation == 1 || armOffset[0], "production roles: the upper body keeps its offsets");
    Check('k', vt::kMutation == 1 || (!chainGame[1] && shinKept[1]),
          "fallback without roles (pinned): per bone, the 10 deg shin keeps its offset");
    SetRoles();
}
// ------------------------------------------------------------------------------------------------ (s) seat pose switch
// Recorded live game pose (bodytrace before_mm, local p.xyz q.xyzw) of Hips, Spine, Spine1, Spine3, Head, LeftLeg at the recorded
// times (seconds after 11:55:00): enter clip motion 8.70-8.81, the one-frame neck / spine spike of the seating frame 9.561, the
// swap 9.8788. From the recorded log (bt_mount.csv rows).
struct SeatFrame
{
    float t;
    float b[6][7];
};
const SeatFrame kSeatRec[] = {
    {8.6952f, {{-0.234508f, -0.153895f, 0.581896f, -0.690737f, 0.173388f, -0.691305f, -0.122137f}, {0.086011f, -0.000000f, 0.000000f, -0.016218f, -0.002234f, -0.108320f, 0.993981f}, {0.053405f, -0.000000f, 0.000000f, -0.012888f, 0.007959f, 0.077366f, 0.996888f}, {0.087406f, -0.000000f, -0.000000f, -0.017514f, 0.005928f, -0.212830f, 0.976915f}, {0.073008f, 0.000000f, 0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, -0.000000f, 0.000000f, -0.000019f, -0.000024f, -0.108649f, 0.994080f}}},
    {8.7120f, {{-0.244243f, -0.143385f, 0.547532f, -0.688921f, 0.181155f, -0.688584f, -0.135735f}, {0.086011f, 0.000000f, -0.000000f, -0.017527f, -0.002977f, -0.102701f, 0.994554f}, {0.052520f, 0.000000f, -0.000000f, -0.013267f, 0.009075f, 0.076783f, 0.996918f}, {0.085432f, 0.000000f, -0.000000f, -0.019462f, 0.008362f, -0.226507f, 0.973780f}, {0.073007f, 0.000000f, 0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, -0.000000f, 0.000000f, -0.000022f, -0.000021f, -0.000002f, 1.000000f}}},
    {8.7295f, {{-0.253249f, -0.132834f, 0.513129f, -0.686970f, 0.189101f, -0.685490f, -0.149717f}, {0.086011f, 0.000000f, 0.000000f, -0.018784f, -0.003543f, -0.096507f, 0.995149f}, {0.051620f, 0.000000f, 0.000000f, -0.013698f, 0.010258f, 0.076419f, 0.996929f}, {0.083449f, 0.000000f, 0.000000f, -0.021083f, 0.010481f, -0.240869f, 0.970272f}, {0.073007f, -0.000000f, 0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, 0.000000f, 0.000000f, -0.000024f, -0.000019f, 0.000000f, 1.000000f}}},
    {8.7503f, {{-0.263641f, -0.119694f, 0.470590f, -0.684119f, 0.199846f, -0.681115f, -0.167708f}, {0.086011f, -0.000000f, 0.000000f, -0.020005f, -0.003855f, -0.088646f, 0.995855f}, {0.050537f, 0.000000f, 0.000000f, -0.014099f, 0.011486f, 0.076302f, 0.996919f}, {0.081270f, 0.000000f, -0.000000f, -0.022341f, 0.012155f, -0.258647f, 0.965637f}, {0.073007f, 0.000000f, -0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, -0.000000f, 0.000000f, -0.000021f, -0.000022f, -0.226329f, 0.974051f}}},
    {8.7621f, {{-0.269261f, -0.112099f, 0.446254f, -0.682193f, 0.206611f, -0.678304f, -0.178406f}, {0.086011f, 0.000000f, -0.000000f, -0.020509f, -0.003817f, -0.084123f, 0.996237f}, {0.049954f, 0.000000f, 0.000000f, -0.014254f, 0.012033f, 0.076516f, 0.996894f}, {0.080295f, 0.000000f, -0.000000f, -0.022625f, 0.012532f, -0.268665f, 0.962887f}, {0.073007f, -0.000000f, -0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, -0.000000f, -0.000000f, -0.000021f, -0.000022f, -0.277425f, 0.960747f}}},
    {8.7867f, {{-0.280490f, -0.096515f, 0.395663f, -0.677520f, 0.221684f, -0.671692f, -0.201627f}, {0.086011f, -0.000000f, -0.000000f, -0.020688f, -0.002945f, -0.075244f, 0.996946f}, {0.048879f, 0.000000f, -0.000000f, -0.014299f, 0.012720f, 0.076459f, 0.996889f}, {0.078122f, 0.000000f, 0.000000f, -0.021978f, 0.011591f, -0.288717f, 0.957092f}, {0.073007f, 0.000000f, -0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, 0.000000f, -0.000000f, -0.000022f, -0.000021f, -0.310648f, 0.950525f}}},
    {8.7948f, {{-0.283957f, -0.091608f, 0.379508f, -0.675795f, 0.226834f, -0.669328f, -0.209398f}, {0.086011f, 0.000000f, -0.000000f, -0.020445f, -0.002393f, -0.072602f, 0.997149f}, {0.048588f, 0.000000f, -0.000000f, -0.014220f, 0.012797f, 0.076252f, 0.996905f}, {0.077398f, -0.000000f, -0.000000f, -0.021365f, 0.010721f, -0.294871f, 0.955238f}, {0.073007f, 0.000000f, -0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, -0.000000f, -0.000000f, -0.000021f, -0.000021f, -0.333853f, 0.942625f}}},
    {8.8112f, {{-0.290814f, -0.082077f, 0.347337f, -0.672389f, 0.236674f, -0.664473f, -0.224398f}, {0.086011f, -0.000000f, 0.000000f, -0.019880f, -0.001263f, -0.067567f, 0.997516f}, {0.048098f, 0.000000f, -0.000000f, -0.014028f, 0.012829f, 0.075336f, 0.996977f}, {0.075927f, 0.000000f, -0.000000f, -0.020039f, 0.008795f, -0.306704f, 0.951554f}, {0.073007f, -0.000000f, -0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, 0.000000f, 0.000000f, -0.000021f, -0.000021f, -0.378622f, 0.925552f}}},
    {9.5277f, {{0.000006f, -0.096176f, 0.004426f, -0.706838f, 0.018957f, -0.706869f, 0.018891f}, {0.086011f, 0.000000f, 0.000000f, -0.000022f, -0.000021f, -0.115723f, 0.993282f}, {0.039036f, 0.000000f, 0.000000f, -0.000029f, -0.000022f, -0.012161f, 0.999926f}, {0.041663f, 0.000000f, 0.000000f, 0.000035f, -0.000011f, -0.258385f, 0.966042f}, {0.073007f, -0.000000f, 0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, 0.000000f, -0.000000f, -0.000018f, -0.000018f, -0.920974f, 0.389624f}}},
    {9.5445f, {{-0.000000f, -0.092875f, 0.003726f, -0.706815f, 0.019984f, -0.706835f, 0.019919f}, {0.086011f, 0.000000f, -0.000000f, -0.000022f, -0.000022f, -0.111661f, 0.993746f}, {0.039037f, -0.000000f, 0.000000f, -0.000022f, -0.000022f, -0.008692f, 0.999962f}, {0.041829f, 0.000000f, -0.000000f, -0.000024f, -0.000021f, -0.268616f, 0.963248f}, {0.073007f, 0.000000f, -0.000000f, -0.000019f, -0.000024f, 0.099453f, 0.995042f}, {0.431106f, -0.000000f, -0.000000f, -0.000018f, -0.000018f, -0.922136f, 0.386865f}}},
    {9.5611f, {{-0.000001f, -0.090646f, 0.002954f, -0.706790f, 0.021063f, -0.706798f, 0.021000f}, {0.086011f, 0.000000f, -0.000000f, -0.000022f, -0.000022f, -0.108221f, 0.994127f}, {0.039030f, -0.000000f, -0.000000f, 0.143777f, 0.061521f, -0.005331f, 0.987682f}, {0.041998f, -0.000000f, 0.000000f, 0.117993f, 0.102626f, -0.274668f, 0.948738f}, {0.073007f, -0.000000f, 0.000000f, -0.236112f, -0.199400f, 0.094557f, 0.946335f}, {0.431106f, -0.000000f, -0.000000f, -0.000018f, -0.000018f, -0.923412f, 0.383810f}}},
    {9.5794f, {{-0.000001f, -0.087811f, 0.001835f, -0.706756f, 0.022307f, -0.706756f, 0.022252f}, {0.086011f, -0.000000f, -0.000000f, -0.000022f, -0.000022f, -0.103651f, 0.994614f}, {0.039013f, -0.000000f, -0.000000f, -0.000063f, -0.000017f, -0.037167f, 0.999309f}, {0.042201f, 0.000000f, -0.000000f, -0.000066f, -0.000027f, -0.321994f, 0.946742f}, {0.073007f, -0.000000f, 0.000000f, -0.000076f, -0.000030f, 0.099359f, 0.995052f}, {0.431105f, 0.000000f, 0.000000f, -0.000018f, -0.000018f, -0.924825f, 0.380393f}}},
    {9.5951f, {{-0.000000f, -0.085392f, 0.000840f, -0.706728f, 0.023189f, -0.706726f, 0.023142f}, {0.086011f, 0.000000f, -0.000000f, -0.000022f, -0.000022f, -0.100390f, 0.994948f}, {0.038991f, 0.000000f, -0.000000f, -0.000064f, -0.000016f, -0.033598f, 0.999436f}, {0.042391f, -0.000000f, -0.000000f, -0.000067f, -0.000026f, -0.329972f, 0.943991f}, {0.073007f, 0.000000f, 0.000000f, -0.000080f, -0.000030f, 0.099324f, 0.995055f}, {0.431106f, -0.000000f, 0.000000f, -0.000018f, -0.000018f, -0.926072f, 0.377346f}}},
    {9.6132f, {{-0.000000f, -0.082979f, -0.000125f, -0.706691f, 0.024300f, -0.706689f, 0.024267f}, {0.086011f, 0.000000f, -0.000000f, -0.000022f, -0.000021f, -0.096147f, 0.995367f}, {0.038960f, 0.000000f, -0.000000f, -0.000030f, -0.000023f, -0.029743f, 0.999558f}, {0.042615f, -0.000000f, 0.000000f, -0.000032f, -0.000023f, -0.338373f, 0.941012f}, {0.073007f, 0.000000f, -0.000000f, -0.000014f, -0.000024f, 0.099286f, 0.995059f}, {0.431106f, 0.000000f, 0.000000f, -0.000018f, -0.000018f, -0.927300f, 0.374318f}}},
    {9.7949f, {{-0.000000f, -0.066624f, -0.006462f, -0.706392f, 0.031813f, -0.706392f, 0.031768f}, {0.086011f, -0.000000f, -0.000000f, -0.000022f, -0.000022f, -0.069015f, 0.997616f}, {0.038479f, 0.000000f, 0.000000f, -0.000034f, -0.000021f, -0.002415f, 0.999997f}, {0.044921f, 0.000000f, 0.000000f, -0.000037f, -0.000022f, -0.388990f, 0.921242f}, {0.073007f, 0.000000f, -0.000000f, -0.000024f, -0.000024f, 0.099321f, 0.995055f}, {0.431106f, -0.000000f, 0.000000f, -0.000018f, -0.000018f, -0.932811f, 0.360367f}}},
    {9.8119f, {{-0.000000f, -0.065912f, -0.006737f, -0.706380f, 0.032079f, -0.706380f, 0.032046f}, {0.086011f, 0.000000f, -0.000000f, -0.000022f, -0.000022f, -0.068109f, 0.997678f}, {0.038440f, -0.000000f, 0.000000f, -0.000033f, -0.000021f, -0.000761f, 1.000000f}, {0.045094f, -0.000000f, -0.000000f, -0.000035f, -0.000021f, -0.390448f, 0.920625f}, {0.073007f, 0.000000f, 0.000000f, -0.000024f, -0.000024f, 0.099311f, 0.995056f}, {0.431106f, -0.000000f, -0.000000f, -0.000018f, -0.000018f, -0.932947f, 0.360013f}}},
    {9.8284f, {{-0.000000f, -0.065421f, -0.006928f, -0.706369f, 0.032305f, -0.706369f, 0.032286f}, {0.086011f, 0.000000f, 0.000000f, -0.000022f, -0.000022f, -0.067529f, 0.997717f}, {0.038407f, -0.000000f, 0.000000f, -0.000032f, -0.000021f, 0.000554f, 1.000000f}, {0.045243f, -0.000000f, -0.000000f, -0.000034f, -0.000022f, -0.391282f, 0.920271f}, {0.073007f, 0.000000f, 0.000000f, -0.000024f, -0.000025f, 0.099305f, 0.995057f}, {0.431106f, -0.000000f, -0.000000f, -0.000018f, -0.000018f, -0.933031f, 0.359796f}}},
    {9.8452f, {{-0.000000f, -0.065067f, -0.007065f, -0.706359f, 0.032514f, -0.706359f, 0.032511f}, {0.086011f, -0.000000f, -0.000000f, -0.000022f, -0.000022f, -0.066927f, 0.997758f}, {0.038377f, 0.000000f, -0.000000f, -0.000031f, -0.000021f, 0.001692f, 0.999999f}, {0.045385f, 0.000000f, -0.000000f, -0.000033f, -0.000022f, -0.391679f, 0.920102f}, {0.073007f, 0.000000f, 0.000000f, -0.000024f, -0.000024f, 0.099298f, 0.995058f}, {0.431106f, -0.000000f, -0.000000f, -0.000018f, -0.000018f, -0.933085f, 0.359656f}}},
    {9.8626f, {{-0.000000f, -0.064829f, -0.007157f, -0.706351f, 0.032684f, -0.706351f, 0.032695f}, {0.086011f, 0.000000f, -0.000000f, -0.000022f, -0.000022f, -0.066416f, 0.997792f}, {0.038354f, -0.000000f, -0.000000f, -0.000030f, -0.000022f, 0.002659f, 0.999997f}, {0.045509f, -0.000000f, 0.000000f, -0.000032f, -0.000022f, -0.391739f, 0.920077f}, {0.073007f, 0.000000f, -0.000000f, -0.000024f, -0.000024f, 0.099293f, 0.995058f}, {0.431106f, -0.000000f, 0.000000f, -0.000018f, -0.000018f, -0.933116f, 0.359575f}}},
    {9.8788f, {{0.006771f, -0.082098f, 0.082289f, -0.707125f, -0.001339f, -0.707044f, 0.007787f}, {0.086011f, 0.000000f, -0.000000f, -0.007663f, 0.010332f, -0.214192f, 0.976707f}, {0.061500f, -0.000000f, -0.000000f, -0.011155f, -0.001059f, -0.024473f, 0.999638f}, {0.099803f, 0.000000f, -0.000000f, -0.003493f, -0.012092f, -0.152902f, 0.988162f}, {0.073007f, 0.000000f, -0.000000f, 0.000001f, -0.000591f, 0.239261f, 0.970955f}, {0.431106f, -0.000000f, -0.000000f, -0.000051f, -0.000027f, -0.550011f, 0.835158f}}},
    {9.8956f, {{0.006586f, -0.082054f, 0.082268f, -0.707024f, -0.001147f, -0.707148f, 0.007558f}, {0.086011f, 0.000000f, -0.000000f, -0.007594f, 0.010158f, -0.214240f, 0.976699f}, {0.061501f, 0.000000f, 0.000000f, -0.011040f, -0.001093f, -0.024603f, 0.999636f}, {0.099803f, 0.000000f, -0.000000f, -0.003410f, -0.011828f, -0.153042f, 0.988143f}, {0.073007f, -0.000000f, 0.000000f, 0.000236f, -0.001478f, 0.243460f, 0.969910f}, {0.431106f, 0.000000f, 0.000000f, -0.000051f, -0.000027f, -0.541587f, 0.840645f}}},
    {9.9123f, {{0.006284f, -0.082002f, 0.082238f, -0.706847f, -0.000777f, -0.707329f, 0.007166f}, {0.086011f, -0.000000f, 0.000000f, -0.007500f, 0.009939f, -0.214304f, 0.976688f}, {0.061501f, 0.000000f, 0.000000f, -0.010900f, -0.001142f, -0.024817f, 0.999632f}, {0.099803f, -0.000000f, -0.000000f, -0.003329f, -0.011517f, -0.153251f, 0.988115f}, {0.073007f, 0.000000f, -0.000000f, 0.000629f, -0.002538f, 0.245456f, 0.969404f}, {0.431106f, 0.000000f, -0.000000f, -0.000051f, -0.000027f, -0.540449f, 0.841377f}}},
    {9.9287f, {{0.005813f, -0.081952f, 0.082195f, -0.706614f, -0.000287f, -0.707568f, 0.006653f}, {0.086011f, 0.000000f, -0.000000f, -0.007369f, 0.009671f, -0.214389f, 0.976673f}, {0.061502f, 0.000000f, -0.000000f, -0.010718f, -0.001201f, -0.025129f, 0.999626f}, {0.099802f, -0.000000f, 0.000000f, -0.003251f, -0.011181f, -0.153526f, 0.988076f}, {0.073007f, 0.000000f, -0.000000f, 0.001086f, -0.003621f, 0.245840f, 0.969303f}, {0.431106f, 0.000000f, -0.000000f, -0.000051f, -0.000027f, -0.540546f, 0.841314f}}},
    {9.9452f, {{0.005361f, -0.081911f, 0.082155f, -0.706420f, 0.000052f, -0.707764f, 0.006304f}, {0.086011f, -0.000000f, 0.000000f, -0.007179f, 0.009339f, -0.214504f, 0.976652f}, {0.061503f, 0.000000f, 0.000000f, -0.010467f, -0.001267f, -0.025551f, 0.999618f}, {0.099802f, -0.000000f, 0.000000f, -0.003168f, -0.010818f, -0.153859f, 0.988029f}, {0.073007f, -0.000000f, 0.000000f, 0.001533f, -0.004621f, 0.245367f, 0.969418f}, {0.431106f, -0.000000f, -0.000000f, -0.000051f, -0.000027f, -0.539095f, 0.842245f}}},
    {9.9619f, {{0.004795f, -0.081859f, 0.082105f, -0.705999f, 0.001018f, -0.708193f, 0.005254f}, {0.086011f, -0.000000f, -0.000000f, -0.006939f, 0.008915f, -0.214679f, 0.976619f}, {0.061503f, -0.000000f, 0.000000f, -0.010146f, -0.001358f, -0.026110f, 0.999607f}, {0.099801f, -0.000000f, -0.000000f, -0.003106f, -0.010455f, -0.154260f, 0.987970f}, {0.073007f, -0.000000f, -0.000000f, 0.001910f, -0.005526f, 0.244431f, 0.969649f}, {0.431106f, -0.000000f, -0.000000f, -0.000080f, -0.000034f, -0.536917f, 0.843635f}}},
};
constexpr float kSeatMountAt = 8.141f; // MountEvent (log 11:55:08.141)
constexpr float kSeatSwapAt = 9.8788f;

Pose SeatPose(const SeatFrame& f)
{
    Pose g = GamePoseB();
    // recorded Spine3 goes to the database Spine1 slot (both torso bones), LeftLeg to LLeg; recorded Spine1 is not used
    const uint32_t slot[6] = {Hips, Spine, 0xFFu, Spine1, Head, LLeg};
    for (uint32_t i = 0; i < 6; ++i)
    {
        if (slot[i] == 0xFFu)
            continue;
        g.p[slot[i]] = {f.b[i][0], f.b[i][1], f.b[i][2]};
        g.q[slot[i]] = {f.b[i][3], f.b[i][4], f.b[i][5], f.b[i][6]};
    }
    return g;
}
float Dist3(const float* a, const float* b)
{
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

// One replay of the recorded frames (+ 0.6 s holding the last recorded pose) through the vehicle hold's denial path.
struct SeatRun
{
    bool untouchedBefore = true, swapFrameIsPrev = false, exactAfter = true, anyWrite = false, armedAfter = true, wroteFpp = false;
    float maxHipsStep = 0.0f, maxTorsoStep = 0.0f, swapHipsStepGame = 0.0f;
    float firstHipsStep = -1.0f; // shown Hips step on the first frame after the swap frame (hand-in start speed)
    float fireAt = -1.0f, fireHipsCm = 0.0f, fireTorsoDeg = 0.0f; // last fire (g_seatLast*), seconds after the mount
    uint64_t fires = 0;
};
// aStride 2 = 30 fps (every other recorded frame, the swap frame kept); aDropTpp = the TPP flag off from the seating frame
// until the vehicle TPP decision (recording 09.578 'FPP <- vehicle' .. 09.910 'TPP <- vehicle'), the swap at 9.879 inside it.
// aSkip = recorded frame indices left out (a frame hitch); aFppFrom >= 0 = the player switches to FPP at that recorded time and
// stays there (the mounted TPP flag off from then on).
SeatRun ReplaySeat(bool aMountTpp, bool aExitBeforeSwap, bool aSecondSwap, int aStride = 1, bool aDropTpp = false,
                   const std::set<size_t>& aSkip = std::set<size_t>{}, float aFppFrom = -1.0f)
{
    SeatRun out;
    const float p0[3] = {0, 0, 0};
    const int64_t t0 = NextEpochTime();
    const uint64_t fires0 = vt::g_counters.seatFires.load();
    vt::g_seatLastAt.store(-1.0f);
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
    vt::SetVehicleCapture(true, kRig);
    vt::SetMountTpp(aMountTpp);
    std::vector<std::pair<float, Pose>> frames;
    const size_t nAll = sizeof kSeatRec / sizeof kSeatRec[0];
    for (size_t i = (aStride == 2 ? 1u : 0u); i < nAll; i += size_t(aStride))
        if (!aSkip.count(i))
            frames.emplace_back(kSeatRec[i].t, SeatPose(kSeatRec[i]));
    const size_t nRec = sizeof kSeatRec / sizeof kSeatRec[0];
    const Pose last = SeatPose(kSeatRec[nRec - 1]);
    const float tLast = kSeatRec[nRec - 1].t;
    for (int k = aStride; k <= 36; k += aStride)
        frames.emplace_back(tLast + float(k) / 60.0f, last);
    if (aSecondSwap) // the hunched pose for one frame and back, 0.6 s later in the same mount: never handed in again
    {
        const Pose pre = SeatPose(kSeatRec[18]);
        const float t2 = frames.back().first;
        frames.emplace_back(t2 + 1.0f / 60.0f, pre);
        for (int k = 2; k <= 30; ++k)
            frames.emplace_back(t2 + float(k) / 60.0f, last);
    }
    Records prevShown;
    int64_t fireT = 0;
    bool exited = false;
    for (size_t i = 0; i < frames.size(); ++i)
    {
        const float tr = frames[i].first;
        const int64_t t = t0 + int64_t(double(tr - kSeatMountAt) * double(kTps));
        if (aExitBeforeSwap && !exited && tr > 9.70f)
        {
            integ::PoseSnapshot cap;
            vt::BeginExit(t, &cap);
            exited = true;
        }
        if (aDropTpp && aMountTpp)
            vt::SetMountTpp(!(tr >= 9.578f && tr < 9.910f));
        const bool fpp = aFppFrom >= 0.0f && tr >= aFppFrom;
        if (fpp)
            vt::SetMountTpp(false);
        const Records game = MakeRecords(frames[i].second);
        Records r = game;
        const uint32_t n = vt::FkSide(r.data(), kGameCount, kRig, t, false); // vehicle hold: permission withdrawn
        const bool same = std::memcmp(r.data(), game.data(), game.size()) == 0;
        if (!same || n)
            out.anyWrite = true;
        if (fpp && (!same || n))
        {
            out.wroteFpp = true;
            std::printf("  seat: write at %.4f in FPP (switched at %.3f)\n", double(tr), double(aFppFrom));
        }
        const bool beforeSwap = tr < kSeatSwapAt - 0.004f;
        if (beforeSwap && (!same || n))
        {
            out.untouchedBefore = false;
            std::printf("  seat: write at %.4f before the swap (n %u)\n", double(tr), n);
        }
        if (std::fabs(tr - kSeatSwapAt) < 0.004f && !prevShown.empty())
        {
            out.swapHipsStepGame = Dist3(Rec(game, Hips), Rec(prevShown, Hips));
            out.swapFrameIsPrev = Dist3(Rec(r, Hips), Rec(prevShown, Hips)) < 1e-4f &&
                                  AngleDegFine(RecQ(r, Spine1), RecQ(prevShown, Spine1)) < 0.05 &&
                                  AngleDegFine(RecQ(r, Spine), RecQ(prevShown, Spine)) < 0.05;
            fireT = t;
        }
        if (!beforeSwap && !prevShown.empty() && tr < 10.4f)
        {
            if (fireT && t > fireT && out.firstHipsStep < 0.0f)
                out.firstHipsStep = Dist3(Rec(r, Hips), Rec(prevShown, Hips));
            out.maxHipsStep = std::fmax(out.maxHipsStep, Dist3(Rec(r, Hips), Rec(prevShown, Hips)));
            out.maxTorsoStep = std::fmax(out.maxTorsoStep, std::fmax(AngleDeg(RecQ(r, Spine1), RecQ(prevShown, Spine1)),
                                                                     AngleDeg(RecQ(r, Spine), RecQ(prevShown, Spine))));
        }
        if (fireT && float(t - fireT) / float(kTps) >= vt::kSeatSeconds && !same)
        {
            out.exactAfter = false;
            std::printf("  seat: write %.3f s after the swap\n", double(float(t - fireT) / float(kTps)));
        }
        prevShown = r;
    }
    out.fires = vt::g_counters.seatFires.load() - fires0;
    out.fireAt = vt::g_seatLastAt.load();
    out.fireHipsCm = vt::g_seatLastHipsCm.load();
    out.fireTorsoDeg = vt::g_seatLastTorsoDeg.load();
    out.armedAfter = vt::SeatGuardArmed();
    vt::Abort();
    return out;
}

void TestSeatSwitch()
{
    // production roles (game names of the frozen map): Spine / Spine1 are the torso bones of the test database
    const vt::BoneRoles roles = vt::BuildBoneRolesFromGame(MakeSnap(MmPoseA(), 1.0f), GameHash);
    Check('s', roles.torso[Spine] == 1 && roles.torso[Spine1] == 1 && roles.torso[Hips] == 0 && roles.torso[Head] == 0 &&
                   roles.torso[LArm] == 0, "torso roles: Spine / Spine1 only");
    vt::SetBoneRoles(roles);
    const SeatRun tpp = ReplaySeat(true, false, false);
    std::printf("  seat (TPP): game Hips step on the swap %.1f cm; shown max Hips step %.2f cm, max torso step %.2f deg, fires %llu\n",
                double(tpp.swapHipsStepGame * 100.0f), double(tpp.maxHipsStep * 100.0f), double(tpp.maxTorsoStep),
                static_cast<unsigned long long>(tpp.fires));
    Check('s', tpp.swapHipsStepGame > 0.08f, "fixture: the recorded swap moves the game Hips > 8 cm in one frame");
    Check('s', tpp.untouchedBefore, "no write before the swap (enter clip motion, seating-frame spike, mount transition expiry)");
    Check('s', tpp.fires == 1, "the swap fires the guard once");
    Check('s', tpp.swapFrameIsPrev, "swap frame shows the previous frame's pose (no step)");
    Check('s', tpp.maxHipsStep <= 0.025f && tpp.maxTorsoStep <= 6.0f, "hand-in steps <= 2.5 cm (Hips) / 6 deg (torso) per frame");
    Check('s', tpp.exactAfter, "game pose bit-exact from kSeatSeconds after the swap");
    // hand-in starts at zero speed (1 - smootherstep): the first frame after the swap moves the Hips by the game motion only
    // (~0.03 cm in the recording); the old exp2 / smoothstep decay moved 11 % of the 9.1 cm step (~1 cm) on that frame
    std::printf("  seat (TPP): first hand-in frame Hips step %.3f cm\n", double(tpp.firstHipsStep * 100.0f));
    Check('s', tpp.firstHipsStep >= 0.0f && tpp.firstHipsStep <= 0.002f, "hand-in starts smoothly: first Hips step after the swap <= 0.2 cm");
    Check('s', !tpp.armedAfter, "guard disarmed after its hand-in (once per mount)");
    const SeatRun fpp = ReplaySeat(false, false, false);
    Check('s', !fpp.anyWrite && fpp.fires == 0, "FPP mount (no mounted TPP flag): nothing written");
    const SeatRun ex = ReplaySeat(true, true, false);
    Check('s', !ex.anyWrite && ex.fires == 0, "exit before the swap: nothing written");
    const SeatRun twice = ReplaySeat(true, false, true);
    const SeatRun drop = ReplaySeat(true, false, false, 1, true);
    Check('s', drop.fires == 1 && drop.swapFrameIsPrev && drop.maxHipsStep <= 0.025f && drop.exactAfter,
          "TPP flag off over the seating frame's FPP decision: the latched guard still hands the swap in");
    const SeatRun f30 = ReplaySeat(true, false, false, 2);
    std::printf("  seat (30 fps): game Hips step on the swap %.1f cm; shown max Hips step %.2f cm, max torso step %.2f deg, fires %llu\n",
                double(f30.swapHipsStepGame * 100.0f), double(f30.maxHipsStep * 100.0f), double(f30.maxTorsoStep),
                static_cast<unsigned long long>(f30.fires));
    Check('s', f30.untouchedBefore && f30.fires == 1 && f30.swapFrameIsPrev, "30 fps: no write on the clip motion, the swap fires");
    Check('s', f30.maxHipsStep <= 0.05f && f30.maxTorsoStep <= 12.0f && f30.exactAfter, "30 fps: hand-in steps <= 5 cm / 12 deg");
    std::printf("  seat (TPP): fire %.3f s after the mount (Hips %.1f cm, torso %.1f deg); 30 fps fire %.3f s (Hips %.1f cm, torso %.1f deg)\n",
                double(tpp.fireAt), double(tpp.fireHipsCm), double(tpp.fireTorsoDeg), double(f30.fireAt), double(f30.fireHipsCm),
                double(f30.fireTorsoDeg));
    {
        // review 2026-10-28: a frame hitch on the enter clip, 8.7120 -> 8.8112 (99 ms: Hips 21.4 cm, Spine3 9.5 deg, 5.8x the
        // previous step) is ordinary motion; the per-frame comparison fired there (0.67 s after the mount) and spent the one-shot guard
        const SeatRun h = ReplaySeat(true, false, false, 1, false, {2, 3, 4, 5, 6});
        std::printf("  seat (hitch 8.7120 -> 8.8112): fires %llu, last fire %.3f s after the mount (Hips %.1f cm, torso %.1f deg), swap "
                    "frame = previous %d, shown max Hips step %.2f cm\n",
                    static_cast<unsigned long long>(h.fires), double(h.fireAt), double(h.fireHipsCm), double(h.fireTorsoDeg),
                    h.swapFrameIsPrev ? 1 : 0, double(h.maxHipsStep * 100.0f));
        Check('s', h.untouchedBefore, "hitch on the enter clip (99 ms, Hips 21 cm): no write before the swap");
        Check('s', h.fires == 1 && h.fireAt > kSeatSwapAt - kSeatMountAt - 0.01f && h.fireAt < kSeatSwapAt - kSeatMountAt + 0.01f &&
                       h.swapFrameIsPrev && h.exactAfter,
              "hitch on the enter clip: the guard is still armed and hands in the real swap at 1.74 s");
        // a 50 ms hitch right before the swap (9.8284 -> 9.8788): a per-1/60-s normalisation would see 3.0 cm Hips (< 4 cm)
        const SeatRun hs = ReplaySeat(true, false, false, 1, false, {17, 18});
        std::printf("  seat (hitch 9.8284 -> 9.8788): fires %llu, fire %.3f s (Hips %.1f cm, torso %.1f deg), shown max Hips step %.2f cm, "
                    "torso %.2f deg, swap frame = previous %d, untouched before %d, exact after %d\n",
                    static_cast<unsigned long long>(hs.fires), double(hs.fireAt), double(hs.fireHipsCm), double(hs.fireTorsoDeg),
                    double(hs.maxHipsStep * 100.0f), double(hs.maxTorsoStep), hs.swapFrameIsPrev ? 1 : 0, hs.untouchedBefore ? 1 : 0,
                    hs.exactAfter ? 1 : 0);
        Check('s', hs.untouchedBefore && hs.fires == 1 && hs.swapFrameIsPrev && hs.maxHipsStep <= 0.025f && hs.maxTorsoStep <= 6.0f &&
                       hs.exactAfter,
              "50 ms hitch right before the swap: the swap is still handed in (steps <= 2.5 cm / 6 deg)");
        // the player switches to FPP inside the watch window: FPP stays vanilla
        const SeatRun fs = ReplaySeat(true, false, false, 1, false, {}, 8.75f);
        std::printf("  seat (FPP from 8.75 (first FPP frame 8.7503), sustained): fires %llu, any write %d\n", static_cast<unsigned long long>(fs.fires),
                    fs.anyWrite ? 1 : 0);
        Check('s', !fs.anyWrite && fs.fires == 0, "FPP switch 0.61 s after the mount, sustained (the swap 1.13 s later, past kSeatFppGraceSeconds): nothing written, no fire");
        const SeatRun fh = ReplaySeat(true, false, false, 1, false, {}, 9.93f);
        std::printf("  seat (FPP from 9.93, during the hand-in): fires %llu, write in FPP %d, armed after %d\n",
                    static_cast<unsigned long long>(fh.fires), fh.wroteFpp ? 1 : 0, fh.armedAfter ? 1 : 0);
        Check('s', fh.fires == 1 && fh.swapFrameIsPrev && !fh.wroteFpp && !fh.armedAfter,
              "FPP switch during the hand-in: no write from the first FPP frame on, guard done");
    }
    {
        // first-person mount: the guard's Wanted() term stays off once the mount transition is over (FPP rides as before)
        const float p0[3] = {0, 0, 0};
        const int64_t t0 = NextEpochTime();
        vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
        vt::SetVehicleCapture(false, kRig);
        vt::SetMountTpp(false);
        const Records game = MakeRecords(GamePoseB());
        for (int k = 0; k <= 30; ++k)
        {
            Records r = game;
            vt::FkSide(r.data(), kGameCount, kRig, t0 + k * kFrame, false);
        }
        Check('s', !vt::Wanted(), "FPP mount: nothing wanted after the mount transition (seat guard needs the TPP latch)");
        vt::Abort();
    }
    Check('s', twice.fires == (vt::kMutation == 7 ? 0u : 1u) && twice.exactAfter, "a second swap in the same mount is not handed in");
    Check('s', !vt::Wanted(), "Abort clears the guard");
    SetRoles();
}

// ------------------------------------------------------------------------------------------------ (p) mounted pop filter
// Recorded live game pose (bodytrace, s1 = local q.xyzw) of Hips, Spine, LeftUpLeg, LeftLeg, RightUpLeg, RightLeg, Spine3, LeftArm,
// RightArm, Head over the bike exit window 11:50:48.00-48.79 (seconds after 11:50:00; MM at weight 0, the body is the game's): one-frame
// pops 48.110 RightLeg 66 deg, 48.596 LeftArm 86 deg, 48.707 LeftArm 71 deg, ordinary motion <= ~8 deg per frame. From
// the recorded log (the rows popsim.py replays).
struct PopFrame
{
    float t;
    float q[10][4];
};
const PopFrame kPopRec[] = {
    {48.0095f, {{-0.703513f, -0.002214f, -0.710677f, 0.001347f}, {0.000641f, 0.007923f, -0.206545f, 0.978405f}, {0.277375f, 0.005466f, -0.855211f, 0.437776f}, {-0.000046f, -0.000006f, -0.924662f, 0.380789f}, {-0.125836f, 0.237626f, 0.354798f, 0.895443f}, {-0.000050f, -0.000027f, -0.542034f, 0.840356f}, {0.006900f, -0.015033f, -0.162034f, 0.986647f}, {-0.363445f, 0.492794f, -0.387683f, 0.689032f}, {-0.408084f, 0.457259f, -0.440248f, 0.656173f}, {0.003128f, -0.006331f, 0.253057f, 0.967426f}}},
    {48.0255f, {{-0.703365f, -0.002118f, -0.710824f, 0.001265f}, {0.000625f, 0.008453f, -0.206170f, 0.978480f}, {0.277695f, 0.005252f, -0.855189f, 0.437618f}, {-0.000046f, -0.000006f, -0.924624f, 0.380881f}, {-0.125835f, 0.237375f, 0.354716f, 0.895542f}, {-0.000050f, -0.000027f, -0.542068f, 0.840334f}, {0.007333f, -0.016203f, -0.162671f, 0.986520f}, {-0.363520f, 0.493206f, -0.387709f, 0.688682f}, {-0.408344f, 0.456511f, -0.441214f, 0.655883f}, {0.003158f, -0.006320f, 0.253056f, 0.967426f}}},
    {48.0425f, {{-0.703212f, -0.002109f, -0.710976f, 0.001170f}, {0.000638f, 0.009028f, -0.206016f, 0.978507f}, {0.277820f, 0.004838f, -0.855149f, 0.437622f}, {-0.000046f, -0.000006f, -0.924620f, 0.380892f}, {-0.125825f, 0.237101f, 0.354616f, 0.895655f}, {-0.000051f, -0.000027f, -0.542053f, 0.840344f}, {0.007728f, -0.017089f, -0.162814f, 0.986478f}, {-0.363279f, 0.493846f, -0.387420f, 0.688513f}, {-0.408316f, 0.456065f, -0.441780f, 0.655830f}, {0.003178f, -0.006304f, 0.253055f, 0.967426f}}},
    {48.0592f, {{-0.703048f, -0.002254f, -0.711138f, 0.001044f}, {0.000648f, 0.009268f, -0.205781f, 0.978554f}, {0.277921f, 0.004465f, -0.855143f, 0.437573f}, {-0.000046f, -0.000006f, -0.924589f, 0.380966f}, {-0.125898f, 0.236801f, 0.354701f, 0.895691f}, {-0.000051f, -0.000027f, -0.542068f, 0.840334f}, {0.007918f, -0.017471f, -0.162975f, 0.986444f}, {-0.363301f, 0.495012f, -0.386692f, 0.688073f}, {-0.408143f, 0.455826f, -0.442155f, 0.655852f}, {0.003189f, -0.006286f, 0.253056f, 0.967426f}}},
    {48.0770f, {{-0.702875f, -0.002521f, -0.711309f, 0.000873f}, {0.000664f, 0.009430f, -0.205514f, 0.978608f}, {0.277965f, 0.004034f, -0.855101f, 0.437632f}, {-0.000046f, -0.000006f, -0.924558f, 0.381042f}, {-0.126033f, 0.236490f, 0.354877f, 0.895684f}, {-0.000051f, -0.000027f, -0.542086f, 0.840323f}, {0.008048f, -0.017722f, -0.163107f, 0.986416f}, {-0.363135f, 0.496361f, -0.385732f, 0.687728f}, {-0.407937f, 0.455718f, -0.442346f, 0.655926f}, {0.003196f, -0.006265f, 0.253057f, 0.967426f}}},
    {48.0928f, {{-0.702719f, -0.002807f, -0.711461f, 0.000732f}, {0.000678f, 0.009550f, -0.205265f, 0.978660f}, {0.278043f, 0.003684f, -0.855029f, 0.437726f}, {-0.000046f, -0.000006f, -0.924534f, 0.381100f}, {-0.126193f, 0.236222f, 0.355036f, 0.895669f}, {-0.000050f, -0.000027f, -0.542103f, 0.840312f}, {0.008182f, -0.017974f, -0.163248f, 0.986387f}, {-0.362598f, 0.497235f, -0.384978f, 0.687803f}, {-0.407942f, 0.455859f, -0.442323f, 0.655840f}, {0.003203f, -0.006244f, 0.253059f, 0.967425f}}},
    {48.1097f, {{-0.704785f, -0.057089f, -0.704817f, -0.057023f}, {-0.000022f, -0.000022f, -0.157960f, 0.987445f}, {0.230615f, -0.003847f, -0.776288f, 0.586668f}, {-0.000018f, -0.000018f, -0.913462f, 0.406924f}, {-0.014663f, 0.250685f, 0.548447f, 0.797589f}, {-0.000018f, -0.000018f, -0.912847f, 0.408302f}, {-0.000021f, -0.000022f, -0.245804f, 0.969320f}, {-0.446028f, 0.465681f, -0.580669f, 0.497014f}, {-0.399925f, 0.451430f, -0.596884f, 0.529151f}, {-0.000030f, -0.000051f, 0.471014f, 0.882126f}}},
    {48.1303f, {{-0.703986f, -0.058342f, -0.705698f, -0.054685f}, {-0.000832f, -0.001210f, -0.158976f, 0.987281f}, {0.229475f, -0.006881f, -0.775921f, 0.587572f}, {-0.000018f, -0.000018f, -0.913257f, 0.407384f}, {-0.020573f, 0.250015f, 0.563531f, 0.787085f}, {-0.000018f, -0.000018f, -0.913121f, 0.407689f}, {-0.000563f, -0.000021f, -0.244527f, 0.969642f}, {-0.443640f, 0.465716f, -0.578794f, 0.501287f}, {-0.398469f, 0.452026f, -0.593411f, 0.533628f}, {-0.000030f, -0.000050f, 0.468866f, 0.883269f}}},
    {48.1431f, {{-0.702451f, -0.060710f, -0.707364f, -0.050126f}, {-0.002386f, -0.003485f, -0.160939f, 0.986955f}, {0.227311f, -0.012915f, -0.775101f, 0.589391f}, {-0.000018f, -0.000018f, -0.913009f, 0.407940f}, {-0.032196f, 0.248759f, 0.587815f, 0.769126f}, {-0.000018f, -0.000018f, -0.912850f, 0.408296f}, {-0.001619f, -0.000021f, -0.242037f, 0.970266f}, {-0.439554f, 0.465211f, -0.575607f, 0.508966f}, {-0.395859f, 0.452838f, -0.586969f, 0.541942f}, {-0.000030f, -0.000050f, 0.464763f, 0.885435f}}},
    {48.1594f, {{-0.699530f, -0.065102f, -0.710417f, -0.041552f}, {-0.005297f, -0.007722f, -0.164596f, 0.986317f}, {0.223165f, -0.024124f, -0.773612f, 0.592571f}, {-0.000018f, -0.000018f, -0.912492f, 0.409094f}, {-0.055381f, 0.246140f, 0.625801f, 0.738052f}, {-0.000018f, -0.000018f, -0.910473f, 0.413568f}, {-0.003603f, 0.000011f, -0.237374f, 0.971412f}, {-0.434324f, 0.462352f, -0.570581f, 0.521566f}, {-0.391583f, 0.453634f, -0.575347f, 0.556646f}, {-0.000030f, -0.000050f, 0.457116f, 0.889407f}}},
    {48.1757f, {{-0.695557f, -0.070884f, -0.714340f, -0.029890f}, {-0.009178f, -0.013387f, -0.169554f, 0.985387f}, {0.217393f, -0.039067f, -0.771889f, 0.596155f}, {-0.000018f, -0.000018f, -0.911522f, 0.411251f}, {-0.086842f, 0.242075f, 0.663215f, 0.702854f}, {-0.000018f, -0.000018f, -0.904344f, 0.426803f}, {-0.006296f, -0.000024f, -0.230970f, 0.972941f}, {-0.425922f, 0.458444f, -0.564843f, 0.537932f}, {-0.386764f, 0.453280f, -0.560376f, 0.575265f}, {-0.000031f, -0.000050f, 0.446842f, 0.894613f}}},
    {48.1932f, {{-0.690590f, -0.077813f, -0.718884f, -0.015395f}, {-0.013902f, -0.020305f, -0.175708f, 0.984135f}, {0.210035f, -0.057483f, -0.769713f, 0.600103f}, {-0.000018f, -0.000018f, -0.910407f, 0.413714f}, {-0.125832f, 0.237159f, 0.692237f, 0.669873f}, {-0.000018f, -0.000018f, -0.893331f, 0.449399f}, {-0.009649f, -0.000205f, -0.222892f, 0.974795f}, {-0.419618f, 0.450150f, -0.559629f, 0.555068f}, {-0.381976f, 0.451077f, -0.542586f, 0.596845f}, {-0.000031f, -0.000050f, 0.434209f, 0.900812f}}},
    {48.2091f, {{-0.685092f, -0.085127f, -0.723465f, 0.000646f}, {-0.018972f, -0.027781f, -0.182477f, 0.982634f}, {0.201702f, -0.077278f, -0.767714f, 0.603291f}, {-0.000018f, -0.000018f, -0.908816f, 0.417198f}, {-0.165241f, 0.232758f, 0.708614f, 0.645279f}, {-0.000019f, -0.000019f, -0.879002f, 0.476818f}, {-0.013323f, -0.000536f, -0.213873f, 0.976770f}, {-0.413723f, 0.439294f, -0.555456f, 0.572121f}, {-0.377801f, 0.446673f, -0.523692f, 0.619271f}, {-0.000032f, -0.000050f, 0.420460f, 0.907311f}}},
    {48.2260f, {{-0.678436f, -0.093508f, -0.728407f, 0.020118f}, {-0.024889f, -0.036595f, -0.190635f, 0.980663f}, {0.191468f, -0.100756f, -0.765497f, 0.605973f}, {-0.000018f, -0.000018f, -0.906835f, 0.421486f}, {-0.206138f, 0.228859f, 0.713464f, 0.629365f}, {-0.000019f, -0.000019f, -0.861383f, 0.507956f}, {-0.017738f, -0.001056f, -0.202816f, 0.979056f}, {-0.409384f, 0.423540f, -0.551727f, 0.590437f}, {-0.373835f, 0.439018f, -0.501516f, 0.644975f}, {-0.000032f, -0.000050f, 0.404118f, 0.914707f}}},
    {48.2453f, {{-0.670104f, -0.103295f, -0.733705f, 0.044367f}, {-0.031984f, -0.047270f, -0.200696f, 0.977990f}, {0.178407f, -0.129036f, -0.763098f, 0.607620f}, {-0.000018f, -0.000018f, -0.904105f, 0.427311f}, {-0.245695f, 0.230157f, 0.709652f, 0.618915f}, {-0.000019f, -0.000019f, -0.840984f, 0.541059f}, {-0.023077f, -0.002048f, -0.188995f, 0.981705f}, {-0.407406f, 0.401085f, -0.547491f, 0.611069f}, {-0.369925f, 0.426526f, -0.474387f, 0.675417f}, {-0.000033f, -0.000050f, 0.384290f, 0.923212f}}},
    {48.2642f, {{-0.660877f, -0.113274f, -0.738494f, 0.070967f}, {-0.039463f, -0.058650f, -0.211640f, 0.974788f}, {0.163931f, -0.159148f, -0.761033f, 0.607146f}, {-0.000018f, -0.000018f, -0.900770f, 0.434296f}, {-0.273276f, 0.241302f, 0.700054f, 0.614018f}, {-0.000019f, -0.000019f, -0.823109f, 0.567883f}, {-0.028722f, -0.003608f, -0.173737f, 0.984366f}, {-0.405823f, 0.373396f, -0.546193f, 0.630520f}, {-0.366179f, 0.409473f, -0.444776f, 0.707403f}, {-0.000033f, -0.000050f, 0.365572f, 0.930783f}}},
    {48.2760f, {{-0.654708f, -0.119473f, -0.741097f, 0.088648f}, {-0.044262f, -0.066025f, -0.218779f, 0.972531f}, {0.154204f, -0.178587f, -0.759891f, 0.605718f}, {-0.000018f, -0.000018f, -0.898385f, 0.439208f}, {-0.286652f, 0.252022f, 0.692662f, 0.611993f}, {-0.000019f, -0.000019f, -0.812965f, 0.582312f}, {-0.032290f, -0.004968f, -0.163767f, 0.985958f}, {-0.407859f, 0.354360f, -0.541508f, 0.644088f}, {-0.363715f, 0.396735f, -0.425022f, 0.727783f}, {-0.000034f, -0.000049f, 0.348926f, 0.937150f}}},
    {48.2932f, {{-0.645550f, -0.128005f, -0.744116f, 0.114767f}, {-0.051087f, -0.076635f, -0.229140f, 0.969026f}, {0.139783f, -0.206538f, -0.758595f, 0.601944f}, {-0.000018f, -0.000018f, -0.894541f, 0.446986f}, {-0.299740f, 0.272930f, 0.681278f, 0.609530f}, {-0.000019f, -0.000019f, -0.799733f, 0.600356f}, {-0.037301f, -0.007449f, -0.149281f, 0.988063f}, {-0.408670f, 0.324188f, -0.538750f, 0.661543f}, {-0.359624f, 0.376035f, -0.395739f, 0.756743f}, {-0.000034f, -0.000049f, 0.330703f, 0.943735f}}},
    {48.3100f, {{-0.636034f, -0.136091f, -0.746245f, 0.141626f}, {-0.057876f, -0.087281f, -0.239438f, 0.965247f}, {0.124942f, -0.234213f, -0.758028f, 0.595758f}, {-0.000018f, -0.000018f, -0.890120f, 0.455727f}, {-0.310359f, 0.295503f, 0.668714f, 0.607600f}, {-0.000019f, -0.000019f, -0.788897f, 0.614526f}, {-0.042042f, -0.010603f, -0.135033f, 0.989892f}, {-0.411209f, 0.292570f, -0.531913f, 0.679984f}, {-0.354560f, 0.353541f, -0.365424f, 0.784705f}, {-0.000035f, -0.000049f, 0.308536f, 0.951213f}}},
    {48.3261f, {{-0.626877f, -0.143162f, -0.747356f, 0.167299f}, {-0.064162f, -0.097237f, -0.248905f, 0.961496f}, {0.110718f, -0.259842f, -0.757817f, 0.588164f}, {-0.000018f, -0.000018f, -0.885511f, 0.464618f}, {-0.320952f, 0.315346f, 0.656043f, 0.605933f}, {-0.000019f, -0.000019f, -0.782023f, 0.623250f}, {-0.046083f, -0.014280f, -0.122201f, 0.991332f}, {-0.412961f, 0.261863f, -0.523366f, 0.697839f}, {-0.348526f, 0.331443f, -0.336562f, 0.809568f}, {-0.000036f, -0.000049f, 0.287790f, 0.957694f}}},
    {48.3480f, {{-0.614207f, -0.151846f, -0.747493f, 0.202352f}, {-0.072562f, -0.110500f, -0.260888f, 0.956275f}, {0.091523f, -0.293597f, -0.758371f, 0.574716f}, {-0.000019f, -0.000019f, -0.878461f, 0.477815f}, {-0.334691f, 0.341477f, 0.640331f, 0.601126f}, {-0.000019f, -0.000020f, -0.778278f, 0.627920f}, {-0.050853f, -0.020206f, -0.106607f, 0.992794f}, {-0.412343f, 0.220423f, -0.508159f, 0.723299f}, {-0.337959f, 0.302016f, -0.298006f, 0.840097f}, {-0.000037f, -0.000048f, 0.261832f, 0.965114f}}},
    {48.3600f, {{-0.607343f, -0.156019f, -0.746906f, 0.221187f}, {-0.076999f, -0.117465f, -0.266794f, 0.953464f}, {0.081286f, -0.311100f, -0.759027f, 0.566116f}, {-0.000019f, -0.000019f, -0.874259f, 0.485460f}, {-0.342091f, 0.354929f, 0.633481f, 0.596407f}, {-0.000019f, -0.000019f, -0.779579f, 0.626303f}, {-0.052992f, -0.023873f, -0.099357f, 0.993353f}, {-0.410740f, 0.198607f, -0.497730f, 0.737640f}, {-0.331083f, 0.287078f, -0.278168f, 0.854747f}, {-0.000037f, -0.000048f, 0.248768f, 0.968563f}}},
    {48.3775f, {{-0.597876f, -0.161215f, -0.745386f, 0.246888f}, {-0.083078f, -0.126779f, -0.273939f, 0.949728f}, {0.067527f, -0.334436f, -0.760334f, 0.552707f}, {-0.000019f, -0.000019f, -0.867922f, 0.496700f}, {-0.349742f, 0.374486f, 0.625614f, 0.588258f}, {-0.000019f, -0.000019f, -0.783473f, 0.621425f}, {-0.055581f, -0.029222f, -0.091296f, 0.993842f}, {-0.403008f, 0.170867f, -0.481122f, 0.759547f}, {-0.319992f, 0.268386f, -0.252673f, 0.872772f}, {-0.000037f, -0.000048f, 0.233909f, 0.972258f}}},
    {48.3932f, {{-0.589366f, -0.165322f, -0.743299f, 0.269858f}, {-0.088546f, -0.134884f, -0.279219f, 0.946574f}, {0.055431f, -0.354596f, -0.762090f, 0.538895f}, {-0.000019f, -0.000019f, -0.861548f, 0.507676f}, {-0.352391f, 0.394451f, 0.620396f, 0.579083f}, {-0.000019f, -0.000019f, -0.788546f, 0.614976f}, {-0.057506f, -0.034376f, -0.086379f, 0.994007f}, {-0.390315f, 0.149835f, -0.459929f, 0.783370f}, {-0.308424f, 0.253979f, -0.232124f, 0.886842f}, {-0.000038f, -0.000048f, 0.219210f, 0.975678f}}},
    {48.4104f, {{-0.580944f, -0.168875f, -0.740592f, 0.292418f}, {-0.094127f, -0.142605f, -0.282911f, 0.943804f}, {0.043801f, -0.373847f, -0.764377f, 0.523496f}, {-0.000019f, -0.000019f, -0.854386f, 0.519639f}, {-0.352028f, 0.415242f, 0.616341f, 0.569012f}, {-0.000019f, -0.000019f, -0.794504f, 0.607258f}, {-0.059297f, -0.039514f, -0.084348f, 0.993885f}, {-0.370358f, 0.131860f, -0.435500f, 0.809807f}, {-0.294969f, 0.242483f, -0.214849f, 0.898908f}, {-0.000038f, -0.000047f, 0.205663f, 0.978623f}}},
    {48.4259f, {{-0.573779f, -0.171468f, -0.737751f, 0.311608f}, {-0.099268f, -0.148944f, -0.283880f, 0.942005f}, {0.034226f, -0.389755f, -0.766970f, 0.508603f}, {-0.000019f, -0.000019f, -0.847306f, 0.531105f}, {-0.349122f, 0.433534f, 0.613765f, 0.559870f}, {-0.000019f, -0.000019f, -0.800161f, 0.599786f}, {-0.060996f, -0.043610f, -0.086517f, 0.993425f}, {-0.344275f, 0.118978f, -0.412057f, 0.835181f}, {-0.283335f, 0.234489f, -0.203809f, 0.907302f}, {-0.000038f, -0.000047f, 0.195136f, 0.980776f}}},
    {48.4430f, {{-0.567174f, -0.173477f, -0.734671f, 0.329360f}, {-0.104324f, -0.154450f, -0.283489f, 0.940689f}, {0.025666f, -0.404019f, -0.770170f, 0.492897f}, {-0.000019f, -0.000019f, -0.839454f, 0.543431f}, {-0.344633f, 0.450427f, 0.611920f, 0.551269f}, {-0.000019f, -0.000019f, -0.805666f, 0.592370f}, {-0.062635f, -0.047376f, -0.090780f, 0.992770f}, {-0.312439f, 0.108126f, -0.390002f, 0.859412f}, {-0.266595f, 0.232468f, -0.196950f, 0.914383f}, {-0.000039f, -0.000047f, 0.186734f, 0.982411f}}},
    {48.4624f, {{-0.561258f, -0.174868f, -0.731432f, 0.345570f}, {-0.109332f, -0.158866f, -0.281955f, 0.939846f}, {0.018381f, -0.416528f, -0.774426f, 0.475848f}, {-0.000019f, -0.000019f, -0.830314f, 0.557296f}, {-0.338977f, 0.465069f, 0.610871f, 0.543730f}, {-0.000019f, -0.000019f, -0.810896f, 0.585191f}, {-0.064277f, -0.050813f, -0.096642f, 0.991941f}, {-0.274656f, 0.098234f, -0.371545f, 0.881402f}, {-0.249378f, 0.232552f, -0.195861f, 0.919439f}, {-0.000039f, -0.000047f, 0.180676f, 0.983543f}}},
    {48.4758f, {{-0.558077f, -0.175339f, -0.729397f, 0.354664f}, {-0.112329f, -0.160835f, -0.280942f, 0.939461f}, {0.014631f, -0.423193f, -0.778016f, 0.464096f}, {-0.000019f, -0.000019f, -0.823361f, 0.567519f}, {-0.335721f, 0.472036f, 0.610470f, 0.540185f}, {-0.000019f, -0.000019f, -0.813872f, 0.581045f}, {-0.065108f, -0.053048f, -0.099982f, 0.991439f}, {-0.248236f, 0.092629f, -0.363363f, 0.893178f}, {-0.235987f, 0.235290f, -0.196916f, 0.922048f}, {-0.000039f, -0.000047f, 0.179964f, 0.983673f}}},
    {48.4926f, {{-0.555924f, -0.175746f, -0.727104f, 0.362465f}, {-0.115033f, -0.161707f, -0.281072f, 0.938944f}, {0.011740f, -0.427988f, -0.783465f, 0.450412f}, {-0.000019f, -0.000019f, -0.814397f, 0.580308f}, {-0.334398f, 0.475749f, 0.608803f, 0.539629f}, {-0.000019f, -0.000019f, -0.814483f, 0.580187f}, {-0.065366f, -0.055894f, -0.100678f, 0.991195f}, {-0.223110f, 0.088699f, -0.361891f, 0.900771f}, {-0.220850f, 0.240148f, -0.199894f, 0.923903f}, {-0.000039f, -0.000047f, 0.184193f, 0.982890f}}},
    {48.5096f, {{-0.555139f, -0.174521f, -0.726583f, 0.365295f}, {-0.116368f, -0.160650f, -0.281316f, 0.938889f}, {0.011888f, -0.429651f, -0.788529f, 0.439865f}, {-0.000019f, -0.000019f, -0.806033f, 0.591871f}, {-0.335038f, 0.472242f, 0.610505f, 0.540389f}, {-0.000019f, -0.000019f, -0.817688f, 0.575661f}, {-0.065209f, -0.057838f, -0.099741f, 0.991188f}, {-0.200018f, 0.085133f, -0.363455f, 0.905895f}, {-0.204869f, 0.246797f, -0.204747f, 0.924769f}, {-0.000038f, -0.000047f, 0.195161f, 0.980771f}}},
    {48.5257f, {{-0.555866f, -0.171078f, -0.729238f, 0.360495f}, {-0.115437f, -0.157578f, -0.281003f, 0.939617f}, {0.016100f, -0.426967f, -0.791086f, 0.437747f}, {-0.000019f, -0.000019f, -0.802061f, 0.597242f}, {-0.338031f, 0.459323f, 0.618655f, 0.540392f}, {-0.000019f, -0.000019f, -0.825846f, 0.563896f}, {-0.064894f, -0.057466f, -0.097929f, 0.991411f}, {-0.183235f, 0.081388f, -0.366770f, 0.908450f}, {-0.191152f, 0.253919f, -0.210403f, 0.924509f}, {-0.000038f, -0.000048f, 0.211247f, 0.977433f}}},
    {48.5426f, {{-0.557621f, -0.164314f, -0.735576f, 0.347832f}, {-0.112131f, -0.152338f, -0.279624f, 0.941292f}, {0.025325f, -0.420773f, -0.790618f, 0.444108f}, {-0.000019f, -0.000019f, -0.802471f, 0.596691f}, {-0.343414f, 0.435919f, 0.633133f, 0.539616f}, {-0.000019f, -0.000019f, -0.840168f, 0.542326f}, {-0.064391f, -0.054630f, -0.096257f, 0.991768f}, {-0.165762f, 0.077561f, -0.371622f, 0.910167f}, {-0.176706f, 0.262657f, -0.217904f, 0.923203f}, {-0.000037f, -0.000048f, 0.233070f, 0.972460f}}},
    {48.5591f, {{-0.559621f, -0.153994f, -0.745627f, 0.327338f}, {-0.106391f, -0.145454f, -0.276615f, 0.943932f}, {0.039575f, -0.411329f, -0.785738f, 0.460281f}, {-0.000019f, -0.000019f, -0.808895f, 0.587953f}, {-0.352501f, 0.401327f, 0.653803f, 0.535931f}, {-0.000019f, -0.000019f, -0.860441f, 0.509550f}, {-0.063975f, -0.048806f, -0.096087f, 0.992115f}, {-0.147948f, 0.072261f, -0.376953f, 0.911480f}, {-0.164108f, 0.271700f, -0.225539f, 0.921075f}, {-0.000037f, -0.000048f, 0.257227f, 0.966351f}}},
    {48.5782f, {{-0.562015f, -0.137508f, -0.760345f, 0.295137f}, {-0.096997f, -0.135532f, -0.271496f, 0.947899f}, {0.063174f, -0.396967f, -0.776282f, 0.485605f}, {-0.000019f, -0.000019f, -0.820350f, 0.571861f}, {-0.368106f, 0.345302f, 0.678219f, 0.534119f}, {-0.000018f, -0.000018f, -0.884950f, 0.465686f}, {-0.062939f, -0.039466f, -0.097496f, 0.992459f}, {-0.125132f, 0.068879f, -0.384074f, 0.912187f}, {-0.150057f, 0.281639f, -0.234358f, 0.918280f}, {-0.000036f, -0.000048f, 0.288973f, 0.957337f}}},
    {48.5959f, {{-0.633574f, 0.029595f, -0.770389f, -0.064877f}, {-0.036018f, -0.094484f, -0.137132f, 0.985378f}, {0.199268f, 0.001769f, -0.537605f, 0.819311f}, {-0.000014f, -0.000017f, -0.975787f, 0.218724f}, {-0.101084f, -0.206408f, -0.721228f, -0.653458f}, {-0.000014f, -0.000016f, -0.980526f, 0.196389f}, {-0.042153f, 0.214285f, -0.383415f, 0.897384f}, {-0.701052f, -0.060924f, -0.508812f, 0.495908f}, {-0.532135f, 0.404175f, -0.546076f, 0.505249f}, {-0.000019f, -0.000023f, 0.099453f, 0.995042f}}},
    {48.6104f, {{-0.629305f, 0.025232f, -0.775215f, -0.048777f}, {-0.041165f, -0.098867f, -0.125169f, 0.986338f}, {0.194439f, -0.017177f, -0.556684f, 0.807466f}, {-0.000014f, -0.000017f, -0.971678f, 0.236307f}, {-0.061418f, -0.194415f, -0.741368f, -0.639378f}, {-0.000014f, -0.000017f, -0.980162f, 0.198196f}, {-0.047080f, 0.224948f, -0.401291f, 0.886650f}, {-0.678578f, -0.126631f, -0.516278f, 0.506906f}, {-0.492249f, 0.416181f, -0.546671f, 0.534448f}, {-0.000019f, -0.000024f, 0.099453f, 0.995042f}}},
    {48.6265f, {{-0.624728f, 0.023505f, -0.779561f, -0.038042f}, {-0.045897f, -0.103966f, -0.114001f, 0.986959f}, {0.190786f, -0.029129f, -0.572998f, 0.796508f}, {-0.000014f, -0.000017f, -0.967957f, 0.251114f}, {-0.019342f, -0.175830f, -0.752467f, -0.634431f}, {-0.000014f, -0.000017f, -0.979829f, 0.199839f}, {-0.051150f, 0.236963f, -0.416823f, 0.876066f}, {-0.663284f, -0.162338f, -0.497677f, 0.534807f}, {-0.480393f, 0.419484f, -0.540439f, 0.548800f}, {-0.000019f, -0.000024f, 0.099453f, 0.995042f}}},
    {48.6425f, {{-0.619681f, 0.024376f, -0.783835f, -0.031684f}, {-0.050356f, -0.110072f, -0.103296f, 0.987258f}, {0.189517f, -0.036966f, -0.586686f, 0.786458f}, {-0.000014f, -0.000017f, -0.964511f, 0.264042f}, {0.022910f, -0.150657f, -0.755300f, -0.637417f}, {-0.000014f, -0.000017f, -0.979369f, 0.202081f}, {-0.054556f, 0.251050f, -0.430989f, 0.865012f}, {-0.645855f, -0.197433f, -0.477385f, 0.562135f}, {-0.469018f, 0.422797f, -0.535552f, 0.560758f}, {-0.000019f, -0.000024f, 0.099453f, 0.995042f}}},
    {48.6593f, {{-0.613860f, 0.028357f, -0.788353f, -0.029506f}, {-0.054519f, -0.117564f, -0.092972f, 0.987199f}, {0.192169f, -0.042570f, -0.598003f, 0.776950f}, {-0.000015f, -0.000018f, -0.961212f, 0.275811f}, {0.061663f, -0.119263f, -0.750660f, -0.646903f}, {-0.000014f, -0.000017f, -0.978511f, 0.206194f}, {-0.057496f, 0.267743f, -0.444576f, 0.852854f}, {-0.626434f, -0.232443f, -0.453673f, 0.589688f}, {-0.457632f, 0.426357f, -0.532352f, 0.570433f}, {-0.000019f, -0.000024f, 0.099453f, 0.995042f}}},
    {48.6762f, {{-0.607609f, 0.035378f, -0.792821f, -0.031544f}, {-0.057923f, -0.125859f, -0.084050f, 0.986783f}, {0.198765f, -0.045854f, -0.606137f, 0.768757f}, {-0.000015f, -0.000018f, -0.958417f, 0.285370f}, {0.091041f, -0.086135f, -0.740235f, -0.660563f}, {-0.000014f, -0.000017f, -0.977209f, 0.212281f}, {-0.059959f, 0.284988f, -0.456848f, 0.840521f}, {-0.607836f, -0.265112f, -0.428369f, 0.613801f}, {-0.447365f, 0.430592f, -0.530279f, 0.577286f}, {-0.000019f, -0.000024f, 0.099453f, 0.995042f}}},
    {48.7067f, {{-0.617457f, 0.059558f, -0.783278f, -0.040919f}, {0.009754f, -0.011137f, -0.173353f, 0.984749f}, {0.294760f, -0.160194f, -0.665770f, 0.666487f}, {-0.000015f, -0.000017f, -0.945512f, 0.325588f}, {-0.102672f, 0.059036f, 0.646734f, 0.753464f}, {-0.000015f, -0.000017f, -0.964457f, 0.264239f}, {-0.007707f, 0.004458f, -0.221521f, 0.975115f}, {-0.296604f, 0.223196f, -0.488306f, 0.789789f}, {-0.257036f, 0.443099f, -0.370651f, 0.774735f}, {-0.000028f, -0.000046f, 0.433052f, 0.901369f}}},
    {48.7134f, {{-0.614355f, 0.063545f, -0.785243f, -0.043851f}, {0.011133f, -0.011255f, -0.173716f, 0.984668f}, {0.298983f, -0.160821f, -0.663877f, 0.666343f}, {-0.000015f, -0.000017f, -0.945208f, 0.326469f}, {-0.104001f, 0.050549f, 0.645997f, 0.754530f}, {-0.000015f, -0.000017f, -0.964119f, 0.265469f}, {-0.007275f, 0.005566f, -0.220451f, 0.975355f}, {-0.291249f, 0.217450f, -0.485095f, 0.795344f}, {-0.251421f, 0.442634f, -0.365815f, 0.779129f}, {-0.000027f, -0.000047f, 0.436009f, 0.899942f}}},
    {48.7317f, {{-0.605647f, 0.075808f, -0.790344f, -0.052923f}, {0.015423f, -0.011281f, -0.174590f, 0.984456f}, {0.310060f, -0.160041f, -0.658343f, 0.666959f}, {-0.000016f, -0.000018f, -0.944728f, 0.327856f}, {-0.102595f, 0.031030f, 0.644019f, 0.757464f}, {-0.000015f, -0.000017f, -0.963218f, 0.268721f}, {-0.005556f, 0.008343f, -0.217695f, 0.975965f}, {-0.277787f, 0.199290f, -0.477576f, 0.809345f}, {-0.235914f, 0.441564f, -0.352793f, 0.790508f}, {-0.000027f, -0.000047f, 0.443574f, 0.896238f}}},
    {48.7455f, {{-0.597743f, 0.087804f, -0.794496f, -0.061398f}, {0.019496f, -0.010949f, -0.175629f, 0.984202f}, {0.318320f, -0.156826f, -0.652421f, 0.669645f}, {-0.000016f, -0.000018f, -0.944681f, 0.327991f}, {-0.096123f, 0.017965f, 0.641971f, 0.760468f}, {-0.000015f, -0.000017f, -0.962416f, 0.271579f}, {-0.003499f, 0.010156f, -0.214841f, 0.976590f}, {-0.266800f, 0.179033f, -0.473471f, 0.820116f}, {-0.220566f, 0.442911f, -0.341280f, 0.799192f}, {-0.000028f, -0.000048f, 0.449883f, 0.893087f}}},
    {48.7633f, {{-0.591423f, 0.098009f, -0.797543f, -0.067364f}, {0.022582f, -0.010453f, -0.177337f, 0.983836f}, {0.321336f, -0.151866f, -0.646057f, 0.675493f}, {-0.000016f, -0.000018f, -0.945038f, 0.326960f}, {-0.085970f, 0.013187f, 0.640150f, 0.763311f}, {-0.000016f, -0.000017f, -0.961778f, 0.273832f}, {-0.001402f, 0.010397f, -0.211274f, 0.977371f}, {-0.258131f, 0.156951f, -0.473875f, 0.827150f}, {-0.205537f, 0.445378f, -0.331362f, 0.805973f}, {-0.000028f, -0.000048f, 0.454123f, 0.890939f}}},
    {48.7795f, {{-0.586921f, 0.105773f, -0.799575f, -0.070816f}, {0.024567f, -0.009851f, -0.179301f, 0.983438f}, {0.320202f, -0.146084f, -0.639926f, 0.683099f}, {-0.000016f, -0.000018f, -0.945525f, 0.325549f}, {-0.075091f, 0.013999f, 0.638537f, 0.765791f}, {-0.000016f, -0.000017f, -0.961291f, 0.275536f}, {0.000439f, 0.009527f, -0.207654f, 0.978156f}, {-0.251919f, 0.135356f, -0.477717f, 0.830664f}, {-0.192215f, 0.447850f, -0.323265f, 0.811161f}, {-0.000028f, -0.000048f, 0.457008f, 0.889462f}}},
};

Pose PopPose(const PopFrame& f)
{
    Pose g = GamePoseB();
    const uint32_t slot[10] = {Hips, Spine, LUpLeg, LLeg, RUpLeg, RLeg, Spine1, LArm, RArm, Head};
    for (uint32_t i = 0; i < 10; ++i)
        g.q[slot[i]] = {f.q[i][0], f.q[i][1], f.q[i][2], f.q[i][3]};
    return g;
}

struct PopRes
{
    uint64_t fires = 0;
    float maxShownStep = 0.0f, maxGameStep = 0.0f, maxFrozen = 0.0f;
    bool anyWrite = false, exactEnd = true;
};
// The ride after the mount transition: the pop filter on (vehicle and TPP), the animation thread (FkSide, permission as the vehicle
// hold leaves it: withdrawn) every given frame; aFrames = (seconds from the scenario start, live game pose). Shown step = largest
// per-bone rotation between consecutive shown frames; frozen = a frame whose game pose steps > kPopDeg while the shown pose moves
// < 1 deg (maxFrozen = longest run of such frames, seconds). The last pose is held 0.4 s after the scenario (exactEnd: records
// bit-exact game from 0.3 s after the last fire).
PopRes RunPop(const std::vector<std::pair<float, Pose>>& aFrames)
{
    PopRes out;
    SetRoles();
    const float p0[3] = {0, 0, 0};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
    vt::SetVehicleCapture(true, kRig);
    vt::SetMountTpp(true); // production: SetMountTpp and SetPopFilter both follow vehicle && TPP (the Hips never move: no seat fire)
    vt::SetPopFilter(true);
    const Records first = MakeRecords(aFrames.front().second);
    for (int k = 0; k <= 24; ++k) // the mount transition (0.2 s hold / offsets, root 0.3 s) runs out on the first pose
    {
        Records r = first;
        vt::FkSide(r.data(), kGameCount, kRig, t0 + k * kFrame, false);
    }
    const int64_t tS = t0 + 30 * kFrame;
    std::vector<std::pair<float, Pose>> frames = aFrames;
    const float tEnd = aFrames.back().first;
    for (int k = 1; k <= 24; ++k)
        frames.emplace_back(tEnd + float(k) / 60.0f, aFrames.back().second);
    const uint64_t pops0 = vt::g_counters.pops.load();
    Records prevShown, prevGame;
    float frozenRun = 0.0f, prevT = 0.0f, lastFire = -1.0f;
    for (size_t i = 0; i < frames.size(); ++i)
    {
        const float ts = frames[i].first;
        const int64_t t = tS + int64_t(double(ts) * double(kTps));
        const uint64_t before = vt::g_counters.pops.load();
        const Records game = MakeRecords(frames[i].second);
        Records r = game;
        const uint32_t n = vt::FkSide(r.data(), kGameCount, kRig, t, false);
        const bool same = std::memcmp(r.data(), game.data(), game.size()) == 0;
        if (vt::g_counters.pops.load() != before)
            lastFire = ts;
        if (!same || n)
            out.anyWrite = true;
        if (lastFire >= 0.0f && ts - lastFire >= 0.3f && !same)
            out.exactEnd = false;
        if (!prevShown.empty())
        {
            float shown = 0.0f, raw = 0.0f;
            for (uint32_t j = kFirst; j < kDb; ++j)
            {
                shown = std::fmax(shown, AngleDeg(RecQ(r, j), RecQ(prevShown, j)));
                raw = std::fmax(raw, AngleDeg(RecQ(game, j), RecQ(prevGame, j)));
            }
            out.maxShownStep = std::fmax(out.maxShownStep, shown);
            out.maxGameStep = std::fmax(out.maxGameStep, raw);
            if (raw > vt::kPopDeg && shown < 1.0f)
            {
                frozenRun += ts - prevT;
                out.maxFrozen = std::fmax(out.maxFrozen, frozenRun);
            }
            else
                frozenRun = 0.0f;
        }
        prevShown = r;
        prevGame = game;
        prevT = ts;
    }
    out.fires = vt::g_counters.pops.load() - pops0;
    vt::Abort();
    return out;
}

// LArm about Y by aDeg (the arm swing of the exit clip)
Pose ArmPose(float aDeg)
{
    Pose g = GamePoseB();
    g.q[LArm] = AxisAngle(0, 1, 0, aDeg);
    return g;
}

void TestPopFilter()
{
    const size_t nRec = sizeof kPopRec / sizeof kPopRec[0];
    auto recorded = [&](size_t aStride, const std::set<size_t>& aSkip) {
        std::vector<std::pair<float, Pose>> f;
        for (size_t i = 0; i < nRec; i += aStride)
            if (!aSkip.count(i))
                f.emplace_back(kPopRec[i].t - kPopRec[0].t, PopPose(kPopRec[i]));
        return f;
    };
    struct Variant
    {
        const char* name;
        size_t stride;
        std::set<size_t> skip;
        float maxShown;
    };
    // recorded at 60 / 30 / 20 fps, a 4-frame hitch in the exit motion, a 3-frame hitch right before the 86 deg pop
    const Variant variants[] = {{"60 fps", 1, {}, 15.0f},
                                {"30 fps", 2, {}, 25.0f},
                                {"20 fps", 3, {}, 35.0f},
                                {"hitch 48.43-48.48", 1, {25, 26, 27, 28}, 25.0f},
                                {"hitch before the 86 deg pop", 1, {32, 33, 34}, 25.0f}};
    for (const Variant& v : variants)
    {
        const PopRes r = RunPop(recorded(v.stride, v.skip));
        std::printf("  pops (recorded exit, %s): fires %llu, game step max %.1f deg, shown step max %.1f deg, exact 0.3 s after the last "
                    "fire %d\n",
                    v.name, static_cast<unsigned long long>(r.fires), double(r.maxGameStep), double(r.maxShownStep), r.exactEnd ? 1 : 0);
        Check('p', r.maxGameStep >= 60.0f, std::string("fixture: the recorded pops step >= 60 deg (") + v.name + ")");
        Check('p', r.fires == 3 && r.maxShownStep <= v.maxShown && r.exactEnd,
              std::string("recorded 66 / 86 / 71 deg pops: 3 fires, shown step <= ") + std::to_string(v.maxShown) + " deg, game pose " +
                  "bit-exact after them (" + v.name + ": fires " + std::to_string(r.fires) + ", shown " + std::to_string(r.maxShownStep) + ")");
    }
    // ordinary exit motion (10 deg per frame at 60 fps) through a 3-frame and a 5-frame hitch (40 / 60 deg in one step)
    {
        std::vector<std::pair<float, Pose>> f;
        for (int k = 0; k <= 60; ++k)
            if (!(k >= 20 && k <= 22) && !(k >= 40 && k <= 44))
                f.emplace_back(float(k) / 60.0f, ArmPose(10.0f * float(k)));
        const PopRes r = RunPop(f);
        std::printf("  pops (10 deg/frame motion, 3- and 5-frame hitches): fires %llu, game step max %.1f deg, any write %d\n",
                    static_cast<unsigned long long>(r.fires), double(r.maxGameStep), r.anyWrite ? 1 : 0);
        Check('p', r.maxGameStep >= 59.0f, "fixture: the hitches step >= 60 deg");
        Check('p', r.fires == 0 && !r.anyWrite, "ordinary motion through a frame hitch is not a pop (no fire, records untouched)");
    }
    // sustained fast motion at 20 fps from rest: 40 deg per frame for 1 s
    {
        std::vector<std::pair<float, Pose>> f;
        for (int k = 0; k <= 4; ++k)
            f.emplace_back(float(k) / 20.0f, ArmPose(0.0f));
        for (int k = 1; k <= 20; ++k)
            f.emplace_back(float(4 + k) / 20.0f, ArmPose(40.0f * float(k)));
        const PopRes r = RunPop(f);
        std::printf("  pops (40 deg/frame at 20 fps for 1 s): fires %llu, longest freeze %.3f s, shown step max %.1f deg\n",
                    static_cast<unsigned long long>(r.fires), double(r.maxFrozen), double(r.maxShownStep));
        Check('p', r.maxFrozen <= 0.1f, "sustained fast motion never freezes the shown body > 0.1 s (" + std::to_string(r.maxFrozen) + " s)");
    }
    // a one-frame spike and back (66 deg): re-anchored on the way back, no overshoot
    {
        std::vector<std::pair<float, Pose>> f;
        for (int k = 0; k <= 30; ++k)
        {
            Pose g = GamePoseB();
            if (k == 10)
                g.q[RLeg] = AxisAngle(1, 0, 0, 66.0f);
            f.emplace_back(float(k) / 60.0f, g);
        }
        const PopRes r = RunPop(f);
        std::printf("  pops (66 deg one-frame spike and back): fires %llu, shown step max %.1f deg\n", static_cast<unsigned long long>(r.fires),
                    double(r.maxShownStep));
        Check('p', r.fires == 2 && r.maxShownStep <= 15.0f && r.exactEnd, "one-frame spike and back: 2 fires, shown step <= 15 deg");
    }
    SetRoles();
}

// ------------------------------------------------------------------------------------------------ 2026-10-28 19-55-01 (T1-T7)
// Shown records of one animation frame as the FK / mapping caller produces them: FkSide, then (non-zero: the transition / hold
// records stay) the MM blend at weight 1 when the permission allows and MM is published (aMm != nullptr).
Records ShownFrame(const Records& aGame, int64_t aT, bool aAllows, const integ::PoseSnapshot* aMm, uint32_t* aN = nullptr)
{
    Records r = aGame;
    const uint32_t n = vt::FkSide(r.data(), kGameCount, kRig, aT, aAllows);
    if (aN)
        *aN = n;
    if (!n && aAllows && aMm)
        integ::BlendIntoRecords(r.data(), kGameCount, *aMm, 1.0f);
    return r;
}
float HipsDist(const Records& a, const Records& b)
{
    const float* x = Rec(a, Hips);
    const float* y = Rec(b, Hips);
    return std::sqrt((x[0] - y[0]) * (x[0] - y[0]) + (x[1] - y[1]) * (x[1] - y[1]) + (x[2] - y[2]) * (x[2] - y[2]));
}
float MaxRotStep(const Records& a, const Records& b)
{
    float m = 0.0f;
    for (uint32_t j = kFirst; j < kDb; ++j)
        m = std::fmax(m, AngleDeg(RecQ(a, j), RecQ(b, j)));
    return m;
}
// the ride after a mount transition on the given pose: vehicle hold (mounted TPP, permission withdrawn), capture, pop filter
int64_t RideOn(const Pose& aPose, int aFrames)
{
    SetRoles();
    const float p0[3] = {0, 0, 0};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
    vt::SetVehicleCapture(true, kRig);
    vt::SetMountTpp(true);
    vt::SetPopFilter(true);
    const Records g = MakeRecords(aPose);
    for (int k = 0; k < aFrames; ++k)
    {
        Records r = g;
        vt::FkSide(r.data(), kGameCount, kRig, t0 + k * kFrame, false);
    }
    return t0 + aFrames * kFrame;
}
// MMDriver's edge feed: the vehicle feed flags drop (SetVehicleCapture / SetMountTpp / SetPopFilter follow the vehicle bit), then
// BeginExit
void EdgeFlagsOff()
{
    vt::SetVehicleCapture(false, kRig);
    vt::SetMountTpp(false);
    vt::SetPopFilter(false);
}

// T1: captures under the vehicle hold's denial (the whole recorded ride ran under it: no capture reached the edge)
void TestDeniedCapture()
{
    Pose stand = GamePoseB();
    stand.p[Hips] = {0.0f, 0.0f, 1.011f};
    stand.q[LArm] = AxisAngle(0, 1, 0, 20);
    const uint64_t caps0 = vt::g_counters.captures.load();
    const int64_t tE = RideOn(stand, 80);
    const uint64_t caps = vt::g_counters.captures.load() - caps0;
    EdgeFlagsOff();
    integ::PoseSnapshot cap;
    const bool handed = vt::BeginExit(tE, &cap);
    std::printf("  T1 denied ride: captures %llu, handed %d\n", static_cast<unsigned long long>(caps), handed ? 1 : 0);
    Check('t', caps >= 30, "T1 captures run through the vehicle hold's denial (" + std::to_string(caps) + ")");
    Check('t', handed && std::fabs(cap.pose[Hips * 7 + 2] - 1.011f) < 1e-5f, "T1 BeginExit hands in the denied-frame capture");
    vt::Abort();
    // first person ride (MMDriver: capture / mounted-TPP flags off): the denial writes and captures nothing, no latch
    {
        const float p0[3] = {0, 0, 0};
        const int64_t t0 = NextEpochTime();
        vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0);
        vt::SetVehicleCapture(false, kRig);
        vt::SetMountTpp(false);
        Pose sunk = stand;
        sunk.p[Hips] = {0.0f, 0.0f, 0.337f};
        const uint64_t c0 = vt::g_counters.captures.load(), b0 = vt::g_counters.exitBreaks.load();
        bool untouched = true;
        for (int k = 0; k < 40; ++k)
        {
            const Records g = MakeRecords(k < 30 ? stand : sunk);
            Records r = g;
            const uint32_t n = vt::FkSide(r.data(), kGameCount, kRig, t0 + k * kFrame, false);
            untouched = untouched && n == 0 && std::memcmp(r.data(), g.data(), g.size()) == 0;
        }
        Check('t', untouched && vt::g_counters.captures.load() == c0 && vt::g_counters.exitBreaks.load() == b0 && !vt::ExitHoldActive(),
              "T1 first person ride: no write, no capture, no latch");
        vt::Abort();
    }
}

// T2: the recorded unmount (19:55:17.251 standing end of the exit clip, Hips 1.011; 17.268 Hips 0.337 with straight legs; 17.286 /
// 17.304 squat, knees 125 deg, forearms 100-122 deg; vehicle bit still set; edge feed 17.304, permission back 17.317)
void TestExitLatch()
{
    Pose stand = GamePoseB();
    stand.p[Hips] = {0.0f, 0.0f, 1.011f};
    stand.q[LArm] = AxisAngle(0, 1, 0, 20);
    Pose sunk = stand;
    sunk.p[Hips] = {0.0f, 0.0f, 0.337f};
    Pose squat = sunk;
    squat.q[LUpLeg] = AxisAngle(1, 0, 0, 125);
    squat.q[RUpLeg] = AxisAngle(1, 0, 0, 125);
    squat.q[LLeg] = AxisAngle(1, 0, 0, -120);
    squat.q[RLeg] = AxisAngle(1, 0, 0, -120);
    squat.q[LArm] = AxisAngle(0, 1, 0, -100);
    squat.q[RArm] = AxisAngle(0, 1, 0, 110);
    Pose mmM = GamePoseB(); // MM stand idle (Hips 0.989)
    mmM.p[Hips] = {0.0f, 0.02f, 0.989f};
    mmM.q[LArm] = AxisAngle(0, 1, 0, 45);
    const integ::PoseSnapshot snapM = MakeSnap(mmM, 1.0f);
    for (int variant = 0; variant < 2; ++variant) // 0: edge on the 3rd swap frame (recorded); 1: edge on the 2nd
    {
        const int64_t tR = RideOn(stand, 60);
        const uint64_t b0 = vt::g_counters.exitBreaks.load();
        Records prev = ShownFrame(MakeRecords(stand), tR, false, nullptr);
        float minZ = 1e9f, maxHipsStep = 0.0f, maxRot = 0.0f;
        bool handed = false, holdEndsAllowed = false, endsOnM = true;
        const int edgeK = variant == 0 ? 3 : 2;
        integ::PoseSnapshot cap;
        for (int k = 1; k <= 30; ++k)
        {
            const int64_t t = tR + k * kFrame;
            const Pose& g = k == 1 ? sunk : squat; // the game pose after the swap stays squatting (worst case)
            if (k == edgeK)
            {
                EdgeFlagsOff();
                handed = vt::BeginExit(t - 1000, &cap);
            }
            // MMDriver publishes MM (weight 1, hand-in) on the edge feed and from then on
            integ::PoseSnapshot s = snapM;
            const bool mmOut = k >= edgeK && handed;
            if (mmOut)
            {
                vt::HandInBlend(s, t - 900);
                vt::MarkExitHandInPublished();
            }
            const bool allows = k >= edgeK + 2; // the permission comes back 1-2 frames after the edge (recorded 13 ms)
            uint32_t n = 0;
            const Records r = ShownFrame(MakeRecords(g), t, allows, mmOut ? &s : nullptr, &n);
            if (allows && n == 0 && !holdEndsAllowed)
                holdEndsAllowed = true;
            minZ = std::fmin(minZ, Rec(r, Hips)[2]);
            maxHipsStep = std::fmax(maxHipsStep, HipsDist(r, prev));
            maxRot = std::fmax(maxRot, MaxRotStep(r, prev));
            if (k >= edgeK + 2 + 16 && (vt::HandInActive() || vt::ExitHoldActive()))
                endsOnM = false;
            prev = r;
        }
        std::printf("  T2 exit latch (edge on swap frame %d): breaks %llu handed %d, shown Hips min %.3f m, max Hips step %.2f cm, max "
                    "bone step %.1f deg\n",
                    edgeK, static_cast<unsigned long long>(vt::g_counters.exitBreaks.load() - b0), handed ? 1 : 0, double(minZ),
                    double(maxHipsStep * 100.0f), double(maxRot));
        Check('u', handed && std::fabs(cap.pose[Hips * 7 + 2] - 1.011f) < 1e-5f, "T2 the edge hands in the standing capture (not the swap)");
        Check('u', minZ >= 1.011f - 0.05f, "T2 shown Hips never drops > 5 cm (min " + std::to_string(minZ) + ")");
        Check('u', maxHipsStep <= 0.03f && maxRot <= 15.0f,
              "T2 shown Hips step <= 3 cm, bone step <= 15 deg per frame (" + std::to_string(maxHipsStep) + " m, " + std::to_string(maxRot) + " deg)");
        Check('u', holdEndsAllowed && endsOnM, "T2 the hold ends on the first allowed frame, MM alone afterwards");
        vt::Abort();
    }
    // a standing break without an edge (no unmount follows): held kCaptureMaxAgeSeconds, then blended back to the live pose
    {
        Pose low = stand;
        low.p[Hips] = {0.0f, 0.0f, 1.011f - 0.30f};
        const int64_t tR = RideOn(stand, 60);
        Records prev = ShownFrame(MakeRecords(stand), tR, false, nullptr);
        float maxHipsStep = 0.0f;
        bool exactEnd = true;
        const uint64_t r0 = vt::g_counters.exitHoldReleases.load();
        for (int k = 1; k <= 60; ++k)
        {
            const int64_t t = tR + k * kFrame;
            const Records g = MakeRecords(low);
            const Records r = ShownFrame(g, t, false, nullptr);
            maxHipsStep = std::fmax(maxHipsStep, HipsDist(r, prev));
            if (float(k * kFrame) / float(kTps) > vt::kCaptureMaxAgeSeconds + vt::kLatchReleaseSeconds + 0.04f &&
                std::memcmp(r.data(), g.data(), g.size()) != 0)
                exactEnd = false;
            prev = r;
        }
        std::printf("  T2 break without an edge: releases %llu, max Hips step %.2f cm, exact after the release %d\n",
                    static_cast<unsigned long long>(vt::g_counters.exitHoldReleases.load() - r0), double(maxHipsStep * 100.0f), exactEnd ? 1 : 0);
        Check('u', vt::g_counters.exitHoldReleases.load() - r0 == 1 && maxHipsStep <= 0.03f && exactEnd && !vt::ExitHoldActive(),
              "T2 a break without an edge is released through a blend (Hips step <= 3 cm), then the live pose bit-exact");
        vt::Abort();
    }
    // the seated ride (Hips ~0.1 m) never latches: a 23.9 cm one-frame Hips step there is the pop filter's
    {
        Pose seat = GamePoseB();
        seat.p[Hips] = {0.0f, -0.13f, 0.08f};
        Pose flash = seat;
        flash.p[Hips] = {0.23f, -0.13f, 0.023f};
        const int64_t tR = RideOn(seat, 60);
        const uint64_t b0 = vt::g_counters.exitBreaks.load();
        for (int k = 1; k <= 12; ++k)
            ShownFrame(MakeRecords(k >= 1 && k <= 6 ? flash : seat), tR + k * kFrame, false, nullptr);
        Check('u', vt::g_counters.exitBreaks.load() == b0, "T2 the seated ride never latches");
        vt::Abort();
    }
    // jump-out from the seated ride (loco fixture drop: Hips -0.40 m, legs 30 / upper body 60 deg): no latch, BeginExit hands in
    // the capture of the launch frame = the pose shown there (the pop filter's output on the drop, not a held ride pose)
    {
        Pose seat = GamePoseB();
        seat.p[Hips] = {0.0f, -0.13f, 0.08f};
        Pose fly = seat;
        fly.p[Hips] = {0.0f, -0.13f, 0.08f - 0.40f};
        for (uint32_t j : {LUpLeg, LLeg, RUpLeg, RLeg})
            fly.q[j] = AxisAngle(1, 0, 0, 30.0f);
        for (uint32_t j : {Spine1, LArm, RArm, Head})
            fly.q[j] = AxisAngle(1, 0, 0, 60.0f);
        const int64_t tR = RideOn(seat, 60);
        const uint64_t b0 = vt::g_counters.exitBreaks.load();
        Records shown;
        for (int k = 1; k <= 3; ++k)
            shown = ShownFrame(MakeRecords(fly), tR + k * kFrame, false, nullptr);
        EdgeFlagsOff();
        integ::PoseSnapshot cap;
        const bool handed = vt::BeginExit(tR + 3 * kFrame + 1000, &cap);
        const Q4 cq{cap.pose[LArm * 7 + 3], cap.pose[LArm * 7 + 4], cap.pose[LArm * 7 + 5], cap.pose[LArm * 7 + 6]};
        Check('u', vt::g_counters.exitBreaks.load() == b0 && handed && std::fabs(cap.pose[Hips * 7 + 2] - Rec(shown, Hips)[2]) < 1e-5f &&
                       AngleDeg(cq, RecQ(shown, LArm)) < 0.1f && !vt::ExitHoldActive(),
              "T2 jump-out from the seated ride: no latch, the capture of the shown launch frame is handed in");
        vt::Abort();
        // the same with the permission allowed (no vehicle hold): the capture stays the live game pose before the guards (unchanged
        // allowed path) - the raw flight pose is handed in
        const int64_t tA = RideOn(seat, 60);
        const uint64_t bA = vt::g_counters.exitBreaks.load();
        for (int k = 1; k <= 3; ++k)
            ShownFrame(MakeRecords(fly), tA + k * kFrame, true, nullptr);
        EdgeFlagsOff();
        integ::PoseSnapshot capA;
        const bool handedA = vt::BeginExit(tA + 3 * kFrame + 1000, &capA);
        const Q4 cqA{capA.pose[LArm * 7 + 3], capA.pose[LArm * 7 + 4], capA.pose[LArm * 7 + 5], capA.pose[LArm * 7 + 6]};
        Check('u', vt::g_counters.exitBreaks.load() == bA && handedA && std::fabs(capA.pose[Hips * 7 + 2] - fly.p[Hips][2]) < 1e-5f &&
                       AngleDeg(cqA, fly.q[LArm]) < 0.1f,
              "T2 jump-out, allowed frames: no latch, the raw flight pose capture is handed in (allowed capture path unchanged)");
        vt::Abort();
    }
    // allowed frames on the standing unmount swap: the swap frame is not captured, the standing capture is held and handed in
    {
        Pose st = GamePoseB();
        st.p[Hips] = {0.0f, 0.0f, 1.011f};
        Pose sk = st;
        sk.p[Hips] = {0.0f, 0.0f, 0.337f};
        const int64_t tA = RideOn(st, 60);
        ShownFrame(MakeRecords(st), tA, true, nullptr);
        const Records r1 = ShownFrame(MakeRecords(sk), tA + kFrame, true, nullptr);
        EdgeFlagsOff();
        integ::PoseSnapshot capS;
        const bool handedS = vt::BeginExit(tA + kFrame + 1000, &capS);
        Check('u', handedS && std::fabs(capS.pose[Hips * 7 + 2] - 1.011f) < 1e-5f && std::fabs(Rec(r1, Hips)[2] - 1.011f) < 1e-5f,
              "T2 allowed frames: the swap frame shows and hands in the standing capture");
        vt::Abort();
    }
}

// T3: a third-person exit without a capture snaps MM in (Fill), not a fade through the game pose
void TestExitFill()
{
    Check('v', vt::ExitWantsFill(false, true, false, true), "T3 third-person ride under the vehicle hold, no capture: Fill");
    Check('v', !vt::ExitWantsFill(true, true, false, true), "T3 handed in: the hand-in path (no second Fill)");
    Check('v', !vt::ExitWantsFill(false, false, false, true), "T3 first-person ride: the old fade");
    Check('v', !vt::ExitWantsFill(false, true, true, true), "T3 jump-out: the live exit capture path");
    Check('v', !vt::ExitWantsFill(false, true, false, false), "T3 ride with the permission allowed: the normal reopen (unchanged)");
    // no capture at all (e.g. the capture flag never came on): BeginExit false -> Fill
    SetRoles();
    const float p0[3] = {0, 0, 0};
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, NextEpochTime());
    integ::PoseSnapshot o;
    const bool handed = vt::BeginExit(NextEpochTime(), &o);
    Check('v', !handed && vt::ExitWantsFill(handed, true, false, true), "T3 BeginExit without a capture under the vehicle hold -> Fill");
    vt::Abort();
}

// T4 / T5: the leg rule at the first post-hold frame. Bike: standing third-person entry frame 0 (Hips 1.004 m), knee 39 deg off
// the MM stride (recorded 35-39 deg); the game legs hold frame 0 for 0.30 s (workspot slide), then the clip moves 3 deg / frame.
// Car: the same legs on a low seat pose keep the 25 deg rule.
struct LegRun
{
    bool firstKept = false, dropped = false;
    float firstStep = 0.0f, maxStep = 0.0f, endOff = 0.0f;
    int maxSameRun = 0;
    float cap = 0.0f;
    uint32_t legsDropped = 0, legKept = 0;
};
LegRun RunLegs(float aHipsZ)
{
    LegRun out;
    SetRoles();
    const Pose a = MmPoseA();
    Pose g0 = GamePoseB();
    g0.p[Hips] = {0.0f, 0.0f, aHipsZ};
    g0.q[LUpLeg] = AxisAngle(1, 0, 0, 20);
    g0.q[LLeg] = AxisAngle(1, 0, 0, -15 + 39); // knee 39 deg off the MM stride
    g0.q[RUpLeg] = AxisAngle(1, 0, 0, -20);
    g0.q[RLeg] = AxisAngle(1, 0, 0, -10 - 30);
    const float p0[3] = {0, 0, 0};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(a, 1.0f), p0, 0.0f, t0);
    vt::SetMountTpp(true);
    Records prev;
    int sameRun = 0;
    const int moveAt = 18; // 0.30 s
    for (int k = 0; k <= 40; ++k)
    {
        Pose g = g0;
        if (k > moveAt)
            g.q[LLeg] = AxisAngle(1, 0, 0, -15 + 39 + 3.0f * float(k - moveAt));
        const Records game = MakeRecords(g);
        Records r = game;
        vt::FkSide(r.data(), kGameCount, kRig, t0 + k * kFrame, false); // vehicle hold veto from the cut
        if (!prev.empty())
        {
            float step = 0.0f;
            for (uint32_t j : {LUpLeg, LLeg, RUpLeg, RLeg})
                step = std::fmax(step, AngleDeg(RecQ(r, j), RecQ(prev, j)));
            if (k == int(vt::kMountHoldFrames))
                out.firstStep = step;
            if (k >= int(vt::kMountHoldFrames))
                out.maxStep = std::fmax(out.maxStep, step);
            if (k > int(vt::kMountHoldFrames) && k <= moveAt + 4)
            {
                sameRun = step < 1e-4f ? sameRun + 1 : 0;
                out.maxSameRun = std::max(out.maxSameRun, sameRun);
            }
        }
        if (k == int(vt::kMountHoldFrames))
        {
            out.firstKept = AngleDeg(RecQ(r, LLeg), RecQ(game, LLeg)) > 30.0f;
            out.dropped = std::memcmp(Rec(r, LLeg) + 4, Rec(game, LLeg) + 4, 16) == 0;
        }
        if (k == moveAt + 1 + int(vt::kLegFinishSeconds * 60.0f) + 1)
            out.endOff = AngleDeg(RecQ(r, LLeg), RecQ(game, LLeg));
        prev = r;
    }
    out.cap = vt::g_mountLegCapDeg.load();
    out.legsDropped = vt::g_mountLegsDropped.load();
    out.legKept = vt::g_mountLegOffsetBones.load();
    vt::Abort();
    return out;
}
void TestStandingLegs()
{
    const LegRun bike = RunLegs(1.004f);
    std::printf("  T4 standing entry legs: cap %.0f deg, kept %u dropped %u, first post-hold step %.2f deg, max step %.2f deg, longest still "
                "run %d frames, offset %.3f deg after the finish\n",
                double(bike.cap), bike.legKept, bike.legsDropped, double(bike.firstStep), double(bike.maxStep), bike.maxSameRun,
                double(bike.endOff));
    Check('w', bike.firstKept && bike.legsDropped == 0 && bike.cap == vt::kLegMaxOffsetStandingDeg,
          "T4 standing entry: the 39 deg knee keeps its offset (60 deg cap)");
    Check('w', bike.firstStep < 5.0f && bike.maxStep <= 8.0f,
          "T4 no leg pop: first post-hold step < 5 deg, <= 8 deg per frame (" + std::to_string(bike.firstStep) + ", " +
              std::to_string(bike.maxStep) + ")");
    Check('w', bike.maxSameRun < 5, "T4 the legs keep moving through the held enter frame 0 (no run of >= 5 still frames)");
    Check('w', bike.endOff < 0.05f, "T4 the leg offsets end within kLegFinishSeconds after the game legs move");
    const LegRun car = RunLegs(0.55f);
    std::printf("  T5 car seat entry legs: cap %.0f deg, kept %u dropped %u\n", double(car.cap), car.legKept, car.legsDropped);
    Check('x', car.dropped && car.cap == vt::kLegMaxOffsetDeg && car.legsDropped > 0,
          "T5 car seat entry: the 25 deg rule (bent leg chain on the game pose after the hold)");
}

// T6: the recorded slide (sub-threshold entity samples: +12.9 / +3.8 deg, then to +67 deg over 0.30 s, the clip's root unwinding
// it over 0.6 s) on a standing entry (not a car: BeginMount default): the visual root follows the cumulative change
float WorldYawDeg(const Records& r, float aEntityYawDeg)
{
    const Q4 w = HipsWorldQ(r, aEntityYawDeg);
    // forward (0, 1, 0) of the Hips in the world
    const float x = 2.0f * (w.x * w.y - w.w * w.z);
    const float y = 1.0f - 2.0f * (w.x * w.x + w.z * w.z);
    return std::atan2(-x, y) / kDeg;
}
void TestRootTracking()
{
    SetRoles();
    const Pose a = MmPoseA();
    Pose g = GamePoseB();
    g.p[Hips] = {0.0f, 0.0f, 1.004f};
    const Records game = MakeRecords(g);
    const float yaw0 = -8.4f;
    const float p0[3] = {5.0f, 5.0f, 0.0f};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(a, 1.0f), p0, yaw0, t0);
    vt::SetMountTpp(true);
    float visStart = 0.0f, maxDev = 0.0f, maxStep = 0.0f, prevVis = 0.0f, rootMax = 0.0f, entityPeak = 0.0f;
    bool engaged = false, exact = true;
    for (int k = 0; k <= 80; ++k)
    {
        const int64_t t = t0 + k * kFrame;
        const float s = float(k) / 18.0f;
        float yaw = yaw0, slide = 0.0f;
        if (k <= 18)
        {
            yaw = yaw0 + 67.0f * vt::detail::Smoothstep(s);
            slide = 0.25f * vt::detail::Smoothstep(s);
        }
        else
        {
            yaw = yaw0 + 67.0f * (1.0f - vt::detail::Smoothstep(float(k - 18) / 36.0f)); // the clip unwinds over 0.6 s
            slide = 0.25f;
        }
        entityPeak = std::fmax(entityPeak, yaw - yaw0);
        const float pos[3] = {p0[0] + slide, p0[1], 0.0f};
        vt::UpdateWorld(pos, yaw, t);
        Records r = game;
        vt::FkSide(r.data(), kGameCount, kRig, t, false);
        float ro[3] = {};
        if (vt::RootOffset(t, ro))
        {
            engaged = true;
            rootMax = std::fmax(rootMax, std::fabs(ro[2]));
        }
        const float vis = WorldYawDeg(r, yaw);
        if (k == 0)
            visStart = vis;
        else
            maxStep = std::fmax(maxStep, std::fabs(vt::detail::WrapPi((vis - prevVis) * kDeg) / kDeg));
        maxDev = std::fmax(maxDev, std::fabs(vt::detail::WrapPi((vis - visStart) * kDeg) / kDeg));
        prevVis = vis;
        if (float(k * kFrame) / float(kTps) > vt::kRootTrackSeconds + vt::kRootTailSeconds + 0.02f &&
            std::memcmp(r.data(), game.data(), game.size()) != 0)
            exact = false;
    }
    std::printf("  T6 root tracking: entity peak %.1f deg, root offset max %.1f deg, visual Hips yaw: max deviation %.1f deg, max step %.2f "
                "deg/frame, engaged at %.3f s, exact after %d\n",
                double(entityPeak), double(rootMax), double(maxDev), double(maxStep), double(vt::g_rootEngageAt.load()), exact ? 1 : 0);
    Check('y', engaged && rootMax > 40.0f, "T6 sub-threshold slide samples produce a root offset (" + std::to_string(rootMax) + " deg)");
    Check('y', maxDev <= 20.0f && maxStep <= 6.0f,
          "T6 the visual Hips yaw stays within 20 deg of the start, <= 6 deg per frame (" + std::to_string(maxDev) + ", " +
              std::to_string(maxStep) + ")");
    Check('y', exact && !vt::MountActive(), "T6 the root offset ends by kRootTrackSeconds + kRootTailSeconds (records bit-exact)");
    vt::Abort();
}


// ------------------------------------------------------------------------------------------------ (D) yaw align (car entry spin)
// Replays of the entity paths of recording 2026-11-01 (the recorded log car ride 3 / ride 2,
// utpp-2026-11-01-07-47-01.log bike ride 2): t = seconds since BeginMount, yaw = entity yaw (deg, unwrapped; the mountdiag samples
// s1-s3 of each ride, then the 100 ms camTrace body= samples, linearly interpolated), x = planar distance from the mount position (m).
struct YawPt { float t, yaw, x; };
const std::vector<YawPt> kCarRide3 = {
    {0.000f, -159.1f, 0.00f}, {0.015f, -159.1f, 0.00f}, {0.034f, -171.2f, 0.02f}, {0.049f, -177.9f, 0.03f}, {0.093f, -190.0f, 0.06f},
    {0.194f, -220.3f, 0.11f}, {0.294f, -234.3f, 0.15f}, {0.394f, -234.3f, 0.15f}, {0.494f, -232.2f, 0.15f}, {0.595f, -226.9f, 0.15f},
    {0.694f, -220.1f, 0.15f}, {0.795f, -211.5f, 0.15f}, {0.895f, -201.8f, 0.15f}, {0.995f, -192.0f, 0.15f}, {1.097f, -182.6f, 0.15f},
    {1.207f, -173.3f, 0.15f}, {1.342f, -167.3f, 0.15f}, {1.391f, -160.2f, 0.15f}, {1.493f, -155.1f, 0.15f}, {1.593f, -153.9f, 0.15f},
    {1.709f, -154.0f, 0.15f}, {2.000f, -154.0f, 0.15f}};
const std::vector<YawPt> kCarRide2 = { // V refaced 90 deg and stood at the door: +9 deg at the workspot, then the clip's turn
    {0.000f, 115.7f, 0.00f}, {0.019f, 115.7f, 0.00f}, {0.036f, 117.2f, 0.12f}, {0.053f, 118.0f, 0.17f}, {0.130f, 119.4f, 0.30f},
    {0.197f, 122.7f, 0.40f}, {0.298f, 124.3f, 0.50f}, {0.400f, 124.9f, 0.53f}, {0.504f, 127.7f, 0.53f}, {0.625f, 133.4f, 0.53f},
    {0.726f, 140.0f, 0.53f}, {0.826f, 150.6f, 0.53f}, {0.892f, 158.0f, 0.53f}, {1.013f, 166.7f, 0.53f}, {1.112f, 177.1f, 0.53f},
    {1.212f, 185.4f, 0.53f}, {1.294f, 192.7f, 0.53f}, {2.000f, 206.0f, 0.53f}};
const std::vector<YawPt> kBikeRide2 = { // sportbike 07:48:31: the entity pivots +99 deg and stays (no unwinding), rootPeakYaw 88.4 deg
    {0.000f, 115.1f, 0.00f}, {0.010f, 115.1f, 0.00f}, {0.028f, 126.5f, 0.10f}, {0.043f, 135.2f, 0.17f}, {0.090f, 152.3f, 0.35f},
    {0.190f, 194.1f, 0.60f}, {0.290f, 213.7f, 0.74f}, {0.390f, 214.6f, 0.74f}, {0.491f, 213.5f, 0.74f}, {0.591f, 211.7f, 0.74f},
    {0.723f, 209.6f, 0.74f}, {0.840f, 207.3f, 0.74f}, {0.890f, 205.7f, 0.74f}, {0.990f, 205.0f, 0.74f}, {1.090f, 204.9f, 0.74f},
    {2.000f, 204.9f, 0.74f}};
const std::vector<YawPt> kBikeC9 = { // c9 bike (10-29 logs): 1.9 deg in 9 ms (211-228 deg/s), +12.9 deg in total, 0.2 m planar
    {0.000f, 0.0f, 0.00f}, {0.009f, 1.9f, 0.02f}, {0.018f, 2.4f, 0.04f}, {0.200f, 9.0f, 0.20f}, {0.450f, 12.9f, 0.25f}, {2.000f, 3.8f, 0.25f}};

float PtAt(const std::vector<YawPt>& p, float t, bool aYaw)
{
    if (t <= p.front().t)
        return aYaw ? p.front().yaw : p.front().x;
    for (size_t i = 1; i < p.size(); ++i)
        if (t <= p[i].t)
        {
            const float w = (t - p[i - 1].t) / (p[i].t - p[i - 1].t);
            return aYaw ? p[i - 1].yaw + (p[i].yaw - p[i - 1].yaw) * w : p[i - 1].x + (p[i].x - p[i - 1].x) * w;
        }
    return aYaw ? p.back().yaw : p.back().x;
}
float WrapDeg(float a) { return vt::detail::WrapPi(a * kDeg) / kDeg; }

struct AlignRun
{
    bool engaged = false, align = false, exact = false;
    float engageAt = -1.0f, alignAt = -1.0f, peakYaw = 0.0f;
    float maxStep = 0.0f;       // shown yaw, deg per 1/60 s, from the engage frame on (before it the shown yaw is the entity's own)
    float maxStepAll = 0.0f;    // the same over the whole replay
    float entMaxStep = 0.0f;    // the entity's own fastest turn from the engage frame on (the vanilla pivot), deg per 1/60 s
    float monoViol = 0.0f;      // largest step against the turn direction until the blend is over (deg per 1/60 s)
    int reversals = 0;          // direction changes of the shown yaw (> 0.02 deg per 1/60 s)
    float tailOff = 0.0f;       // largest |shown - entity| after the blend (or after the tracking window without an align), deg
    float offAtEnd = 0.0f;      // |shown - entity| at alignAt + kAlignSeconds
    float totalTurn = 0.0f;     // entity yaw at the end of the blend minus the start (deg)
    uint64_t hash = 0xcbf29ce484222325ull; // FNV-1a over the bits of the shown yaw and the root offset of every animation frame
    int stepped = 0;            // animation frames inside the blend whose root yaw offset equals the previous frame's (a per-feed step)
};
// aFeedHz: UpdateWorld cadence, aAnimHz: FkSide cadence (the animation frames between two feeds see the live entity and the offset of
// the last feed). Standing entry: the Hips height of the game entry frame (car 0.973 m, bike 0.999-1.004 m).
AlignRun RunAlign(const std::vector<YawPt>& aPts, float aHipsZ, int aFeedHz, int aAnimHz, bool aCar, float aCapSec = -1.0f)
{
    SetRoles();
    const Pose a = MmPoseA();
    Pose g = GamePoseB();
    g.p[Hips] = {0.0f, 0.0f, aHipsZ};
    const Records game = MakeRecords(g);
    const float p0[3] = {50.0f, 50.0f, 0.0f};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(a, 1.0f), p0, WrapDeg(aPts.front().yaw), t0, aCar);
    vt::SetMountTpp(true);
    if (aCapSec >= 0.0f)
        vt::SetHoldCap(aCapSec, t0 + 1000); // group E: a cap on a bike / this path must change nothing
    AlignRun run;
    std::vector<float> shown, tAt, roYaw;
    const int frames = int(2.2f * float(aAnimHz));
    int nextFeed = 1;
    bool exactAll = true;
    for (int j = 0; j <= frames; ++j)
    {
        const float tj = float(j) / float(aAnimHz);
        const int64_t ticks = t0 + int64_t(double(tj) * double(kTps));
        while (float(nextFeed) / float(aFeedHz) <= tj + 1e-5f)
        {
            const float tf = float(nextFeed) / float(aFeedHz);
            const float pos[3] = {p0[0] + PtAt(aPts, tf, false), p0[1], 0.0f};
            vt::UpdateWorld(pos, WrapDeg(PtAt(aPts, tf, true)), t0 + int64_t(double(tf) * double(kTps)));
            ++nextFeed;
        }
        const float ey = PtAt(aPts, tj, true);
        Records r = game;
        vt::FkSide(r.data(), kGameCount, kRig, ticks, false);
        const float off = vt::detail::WrapPi((WorldYawDeg(r, WrapDeg(ey)) - WrapDeg(ey)) * kDeg) / kDeg;
        shown.push_back(ey + off);
        tAt.push_back(tj);
        float ro[3] = {};
        roYaw.push_back(vt::RootOffset(ticks, ro) ? ro[2] : 0.0f);
        for (float v : {shown.back(), ro[0], ro[1], ro[2]})
        {
            uint32_t bits = 0;
            std::memcpy(&bits, &v, sizeof bits);
            for (int b = 0; b < 4; ++b)
                run.hash = (run.hash ^ ((bits >> (8 * b)) & 0xffu)) * 0x100000001b3ull;
        }
        if (tj > vt::kRootTrackSeconds + vt::kRootTailSeconds + 0.02f && std::memcmp(r.data(), game.data(), game.size()) != 0)
            exactAll = false;
    }
    run.engageAt = vt::g_rootEngageAt.load();
    run.alignAt = vt::g_rootAlignAt.load();
    run.engaged = run.engageAt >= 0.0f;
    run.align = run.alignAt >= 0.0f;
    run.peakYaw = vt::g_rootPeakYawDeg.load();
    run.exact = exactAll && !vt::MountActive();
    const float blendEnd = run.align ? run.alignAt + vt::kAlignSeconds : vt::kRootTrackSeconds;
    run.totalTurn = PtAt(aPts, blendEnd, true) - aPts.front().yaw;
    const float dir = run.totalTurn < 0.0f ? -1.0f : 1.0f;
    const float scale = float(aAnimHz) / 60.0f;
    int lastSign = 0;
    for (size_t j = 1; j < shown.size(); ++j)
    {
        const float d = (shown[j] - shown[j - 1]) * scale; // deg per 1/60 s
        run.maxStepAll = std::fmax(run.maxStepAll, std::fabs(d));
        if (tAt[j] >= run.engageAt)
        {
            run.maxStep = std::fmax(run.maxStep, std::fabs(d));
            run.entMaxStep = std::fmax(run.entMaxStep, std::fabs(PtAt(aPts, tAt[j], true) - PtAt(aPts, tAt[j - 1], true)) * scale);
        }
        if (tAt[j] <= blendEnd)
            run.monoViol = std::fmax(run.monoViol, -d * dir);
        if (std::fabs(d) > 0.02f)
        {
            const int sg = d < 0.0f ? -1 : 1;
            if (lastSign != 0 && sg != lastSign)
                ++run.reversals;
            lastSign = sg;
        }
        if (run.align && tAt[j] > run.alignAt + 0.03f && tAt[j] < blendEnd - 0.03f && std::fabs(roYaw[j] - roYaw[j - 1]) < 1e-4f)
            ++run.stepped;
        const float offNow = std::fabs(shown[j] - PtAt(aPts, tAt[j], true));
        if (run.align && std::fabs(tAt[j] - blendEnd) <= 0.5f / float(aAnimHz))
            run.offAtEnd = offNow;
        if (tAt[j] >= blendEnd + 0.02f && tAt[j] <= vt::kRootTrackSeconds + vt::kRootTailSeconds + 0.02f)
            run.tailOff = std::fmax(run.tailOff, offNow);
    }
    vt::Abort();
    return run;
}
void PrintAlign(const char* aWhat, const AlignRun& r)
{
    std::printf("  %s: engageAt %.3f alignAt %.3f peakYaw %.1f deg, turn %.1f deg, max step %.2f deg/frame (entity %.2f, whole replay %.2f), against-turn %.2f deg, "
                "reversals %d, offset at blend end %.3f deg, tail offset max %.3f deg, stepped frames %d, exact %d\n",
                aWhat, double(r.engageAt), double(r.alignAt), double(r.peakYaw), double(r.totalTurn), double(r.maxStep),
                double(r.entMaxStep), double(r.maxStepAll), double(r.monoViol), r.reversals, double(r.offAtEnd), double(r.tailOff), r.stepped, r.exact ? 1 : 0);
}
void CheckAligned(const char* aName, const AlignRun& r, float aMinPeak)
{
    const std::string n = aName;
    Check('D', r.engaged && r.peakYaw >= aMinPeak,
          n + ": tracking engaged, held yaw peak >= " + std::to_string(aMinPeak) + " (" + std::to_string(r.peakYaw) + ")");
    Check('D', r.align && r.alignAt > 0.0f && r.alignAt < 0.25f, n + ": the yaw align latched early (" + std::to_string(r.alignAt) + " s)");
    Check('D', r.monoViol <= 0.3f,
          n + ": shown yaw monotonic toward the entry pose until the blend is over (largest step against " + std::to_string(r.monoViol) + " deg)");
    Check('D', r.maxStep <= 1.35f * r.entMaxStep + 0.3f,
          n + ": the blend never turns V faster than 1.35 x the entity's own pivot (" + std::to_string(r.maxStep) + " vs " +
              std::to_string(r.entMaxStep) + " deg per 1/60 s)");
    Check('D', r.offAtEnd < 0.5f && r.tailOff < 0.5f,
          n + ": no late release - no yaw offset after latch + kAlignSeconds (" + std::to_string(r.offAtEnd) + ", " + std::to_string(r.tailOff) + " deg)");
    Check('D', r.reversals <= 1,
          n + ": at most one direction change (onto the entry pose, then the clip's turn into the seat): " + std::to_string(r.reversals));
    Check('D', r.exact, n + ": records bit-exact after the tracking window, epoch ended");
}
void TestYawAlign()
{
    // car ride 3: V parallel to the car, the workspot pivots the entity 75 deg in 0.28 s (rootPeakYaw 63.4 deg held before)
    const AlignRun c3 = RunAlign(kCarRide3, 0.973f, 60, 60, true);
    PrintAlign("D1 car ride 3 (feed 60 Hz, frames 60 Hz)", c3);
    CheckAligned("D1 car ride 3", c3, 60.0f);
    // the same with a slow feed and fast animation frames: the blend is evaluated per animation frame, not stepped per feed
    const AlignRun c3b = RunAlign(kCarRide3, 0.973f, 30, 120, true);
    PrintAlign("D2 car ride 3 (feed 30 Hz, frames 120 Hz)", c3b);
    Check('D', c3b.align && c3b.stepped == 0 && c3b.tailOff < 0.5f && c3b.exact,
          "D2 car ride 3 with a 30 Hz feed and 120 Hz animation frames: aligned, the yaw offset changes on every animation frame of the "
          "blend (stepped frames " + std::to_string(c3b.stepped) + "), no late release");
    // the sportbike ride 2 path (07:47: the entity pivots +99 deg and stays - the old hold released ~88 deg over the tail) as a CAR
    // entry: aligned like ride 3 (the same geometry on a car, e.g. an 85 deg workspot pivot)
    const AlignRun b2c = RunAlign(kBikeRide2, 0.999f, 60, 60, true);
    PrintAlign("D3 85 deg pivot path as a car", b2c);
    CheckAligned("D3 85 deg pivot path as a car", b2c, 70.0f);
    // the same path as a BIKE (BeginMount default / aCar false): the validated hold, bit-identical to the code before the yaw align
    // (golden = FNV-1a of the shown yaw and the root offset of every frame, taken from the 1dbef1a header with the same replay)
    const AlignRun b2 = RunAlign(kBikeRide2, 0.999f, 60, 60, false);
    PrintAlign("D6 85 deg pivot path as a bike (hold)", b2);
    std::printf("  D6 bike hash %016llx\n", static_cast<unsigned long long>(b2.hash));
    Check('D', !b2.align && b2.peakYaw > 70.0f && b2.tailOff > 50.0f,
          "D6 bike: the 85 deg pivot keeps the cumulative hold (no yaw align, the late tail release as before)");
    Check('D', b2.hash == 0x124c1508744dadfeull, "D6 bike: shown yaw / root offset series bit-identical to the code before the yaw align");
    // car ride 2: V stood at the door, +9 deg at the workspot: below kAlignYawDeg, the hold of the standing entry as before
    const AlignRun c2 = RunAlign(kCarRide2, 0.973f, 60, 60, true);
    PrintAlign("D4 car ride 2 (8.7 deg held before)", c2);
    Check('D', c2.engaged && !c2.align && c2.alignAt < 0.0f, "D4 car ride 2: tracking engaged, no yaw align (peak below kAlignYawDeg)");
    Check('D', c2.peakYaw > 4.0f && c2.peakYaw < 12.0f, "D4 car ride 2: held yaw peak about 8.7 deg (" + std::to_string(c2.peakYaw) + ")");
    Check('D', c2.engageAt > 0.04f && c2.engageAt < 0.07f,
          "D4 car ride 2: engaged on the planar distance at about 0.05 s (" + std::to_string(c2.engageAt) + ")");
    Check('D', c2.exact, "D4 car ride 2: records bit-exact after the tracking window");
    // c9 bike: 1.9 deg per 9 ms (211-228 deg/s) stays a cumulative engage (distance 0.15 m), no yaw align
    const AlignRun c9 = RunAlign(kBikeC9, 1.004f, 120, 60, false);
    PrintAlign("D5 c9 bike (12.9 deg)", c9);
    std::printf("  D5 c9 hash %016llx\n", static_cast<unsigned long long>(c9.hash));
    Check('D', c9.engaged && !c9.align && c9.engageAt > 0.1f && c9.exact,
          "D5 c9 bike: engaged on the cumulative change only, no yaw align (engaged at " + std::to_string(c9.engageAt) + " s)");
    Check('D', c9.hash == 0x6ab0b42e10ddd37dull, "D5 c9 bike: series bit-identical to the code before the yaw align");
}

// T7: the recorded mid-exit flash (19:55:15.804 / 15.903): a translation-only Hips step of 23.9 cm sideways (z -5.7 cm) for 6
// frames and back, at a low Hips (0.30 m, mid-exit)
void TestPopHips()
{
    Pose base = GamePoseB();
    base.p[Hips] = {0.0f, -0.13f, 0.30f};
    Pose flash = base;
    flash.p[Hips] = {0.0f + 0.227f, -0.13f + 0.075f, 0.30f - 0.057f};
    const int64_t tR = RideOn(base, 40);
    const uint64_t hp0 = vt::g_counters.hipsPops.load();
    Records prev = ShownFrame(MakeRecords(base), tR, false, nullptr);
    float maxHips = 0.0f, maxRot = 0.0f;
    bool exactEnd = true;
    for (int k = 1; k <= 40; ++k)
    {
        const int64_t t = tR + k * kFrame;
        const Records g = MakeRecords(k >= 2 && k <= 7 ? flash : base);
        const Records r = ShownFrame(g, t, false, nullptr);
        maxHips = std::fmax(maxHips, HipsDist(r, prev));
        maxRot = std::fmax(maxRot, MaxRotStep(r, prev));
        if (k >= 8 + 19 && std::memcmp(r.data(), g.data(), g.size()) != 0)
            exactEnd = false;
        prev = r;
    }
    const uint64_t fires = vt::g_counters.hipsPops.load() - hp0;
    std::printf("  T7 translation flash: Hips fires %llu, shown max Hips step %.2f cm, max bone step %.2f deg, exact after %d\n",
                static_cast<unsigned long long>(fires), double(maxHips * 100.0f), double(maxRot), exactEnd ? 1 : 0);
    Check('z', fires == 2 && maxHips <= 0.03f && maxRot <= 10.0f && exactEnd,
          "T7 translation-only flash: 2 Hips fires, shown <= 3 cm / 10 deg per frame, game pose bit-exact after (" +
              std::to_string(maxHips) + " m)");
    vt::Abort();
    // a translation fire never suppresses the rotation pop of the next frame (recorded F0 Hips 23.9 cm, F1 Spine 49 deg)
    {
        const int64_t t1 = RideOn(base, 40);
        const uint64_t p0 = vt::g_counters.pops.load(), h0 = vt::g_counters.hipsPops.load();
        ShownFrame(MakeRecords(base), t1, false, nullptr);
        ShownFrame(MakeRecords(flash), t1 + kFrame, false, nullptr);
        Pose f1 = flash;
        f1.q[Spine1] = AxisAngle(0, 0, 1, 49.0f);
        ShownFrame(MakeRecords(f1), t1 + 2 * kFrame, false, nullptr);
        const uint64_t rot = (vt::g_counters.pops.load() - p0) - (vt::g_counters.hipsPops.load() - h0);
        Check('z', vt::g_counters.hipsPops.load() - h0 == 1 && rot == 1, "T7 Hips fire on F0 and the rotation pop on F1 both fire");
        vt::Abort();
    }
}
// T8: c9 ride 2 (feed 45.13-45.41): entity +0.76 m along x and -9.9 deg in 0.28 s (smoothstep), held to 0.35 s, then the clip
// turns it back +9.6 deg over 0.70 s and drifts it +0.38 m over 0.85 s (its own root motion). The game Hips stays at the root
// (the clip's own Hips motion is not part of this check), so the visual Hips world motion is the entity motion plus the root offset.
void TestPlanarRootDecay()
{
    SetRoles();
    const Pose a = MmPoseA();
    Pose g = GamePoseB();
    g.p[Hips] = {0.0f, 0.0f, 1.004f};
    const Records game = MakeRecords(g);
    const float yaw0 = -8.4f;
    const float p0[3] = {1574.75f, 2253.40f, 182.0f};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(a, 1.0f), p0, yaw0, t0);
    vt::SetMountTpp(true);
    float prevW[2] = {}, slideMax = 0.0f, afterMax = 0.0f, entityMax = 0.0f, prevE[2] = {};
    bool exact = true;
    for (int k = 0; k <= 96; ++k)
    {
        const int64_t t = t0 + k * kFrame;
        const float ts = float(k) / 60.0f;
        float x = p0[0], yaw = yaw0;
        if (ts <= 0.28f)
        {
            x += 0.76f * vt::detail::Smoothstep(ts / 0.28f);
            yaw += -9.9f * vt::detail::Smoothstep(ts / 0.28f);
        }
        else
        {
            x += 0.76f + (ts > 0.35f ? 0.38f * vt::detail::Smoothstep((ts - 0.35f) / 0.85f) : 0.0f);
            yaw += -9.9f + (ts > 0.35f ? 9.6f * vt::detail::Smoothstep((ts - 0.35f) / 0.70f) : 0.0f);
        }
        const float pos[3] = {x, p0[1], p0[2]};
        vt::UpdateWorld(pos, yaw, t);
        Records r = game;
        vt::FkSide(r.data(), kGameCount, kRig, t, false);
        const float* h = Rec(r, Hips);
        float wx = 0.0f, wy = 0.0f;
        vt::detail::CharacterToWorld(h[0], h[1], yaw * kDeg, wx, wy);
        const float w[2] = {x + wx, p0[1] + wy};
        if (k > 0)
        {
            const float step = std::hypot(w[0] - prevW[0], w[1] - prevW[1]);
            const float est = std::hypot(x - prevE[0], p0[1] - prevE[1]);
            // the root offset's own world motion (visual Hips minus the entity): what slides the body under its planted feet
            const float off = std::hypot((w[0] - x) - (prevW[0] - prevE[0]), (w[1] - p0[1]) - (prevW[1] - prevE[1]));
            if (ts <= 0.40f && k > int(vt::kMountHoldFrames) + 2)
                slideMax = std::fmax(slideMax, step);
            if (ts > 0.40f)
            {
                afterMax = std::fmax(afterMax, off);
                entityMax = std::fmax(entityMax, est);
            }
        }
        prevW[0] = w[0]; prevW[1] = w[1];
        prevE[0] = x; prevE[1] = p0[1];
        if (ts > vt::kRootTrackSeconds + vt::kRootTailSeconds + 0.02f && std::memcmp(r.data(), game.data(), game.size()) != 0)
            exact = false;
    }
    std::printf("  T8 planar root decay: engaged %.3f s, peak %.2f m, visual Hips step: slide %.2f cm, offset after %.2f cm (entity %.2f cm) "
                "per frame, exact after %d\n",
                double(vt::g_rootEngageAt.load()), double(vt::g_rootPeakDistM.load()), double(slideMax * 100.0f),
                double(afterMax * 100.0f), double(entityMax * 100.0f), exact ? 1 : 0);
    Check('A', vt::g_rootPeakDistM.load() > 0.5f && slideMax <= 0.03f,
          "T8 the visual Hips holds through the slide (<= 3 cm per frame after the engage, " + std::to_string(slideMax) + " m)");
    Check('A', afterMax <= 0.030f,
          "T8 the planar offset leaves at <= 3.0 cm per frame in the world (entity motion excluded, " + std::to_string(afterMax * 100.0f) + " cm)");
    Check('A', exact && !vt::MountActive(), "T8 the root offset ends by kRootTrackSeconds + kRootTailSeconds (records bit-exact)");
    vt::Abort();
}
// T9: the epoch never touches a leg after the leg decay (recording 2026-10-29 00:30:05.585 / 20.090: the jerk is not an epoch write)
void TestLegsUntouchedAfterDecay()
{
    SetRoles();
    const Pose a = MmPoseA();
    Pose g0 = GamePoseB();
    g0.p[Hips] = {0.0f, 0.0f, 1.004f};
    g0.q[LLeg] = AxisAngle(1, 0, 0, -15 + 39); // standing entry: knee 39 deg off the MM stride (kept by the 60 deg cap)
    const float yaw0 = -8.4f;
    const float p0[3] = {1576.7f, 2253.1f, 182.0f};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(a, 1.0f), p0, yaw0, t0);
    vt::SetMountTpp(true);
    const int moveAt = 21; // game legs on frame 0 until 0.35 s (after the 0.28 s leg decay: legMoveAt stays -1)
    bool legsExact = true, otherExact = true, lateWrite = false, ended = false;
    int legCheckedFrames = 0;
    float maxHipsStepCm = 0.0f, maxHipsRotDeg = 0.0f;
    Records prevGame, prevOut;
    for (int k = 0; k <= 96; ++k)
    {
        const int64_t t = t0 + k * kFrame;
        const float ts = float(k) / 60.0f;
        // entity: 0.21 m / +6 deg slide in 0.28 s (recorded ride 1), then still
        const float s = vt::detail::Smoothstep(ts / 0.28f);
        const float pos[3] = {p0[0] - 0.21f * s, p0[1], p0[2]};
        vt::UpdateWorld(pos, yaw0 + 6.0f * s, t);
        Pose g = g0;
        if (k > moveAt)
        {
            const float u = float(k - moveAt);
            g.q[LUpLeg] = AxisAngle(1, 0, 0, 10.0f * u);           // the swing: 10 deg per frame
            g.q[LLeg] = AxisAngle(1, 0, 0, -15 + 39 + 6.0f * u);
            g.p[Hips] = {0.0f, 0.0f, 1.004f - 0.006f * u};          // seating: 0.6 cm per frame
            g.q[Spine] = AxisAngle(1, 0, 0, 0.8f * u);
        }
        const Records game = MakeRecords(g);
        Records r = game;
        const uint32_t n = vt::FkSide(r.data(), kGameCount, kRig, t, false);
        if (!vt::MountActive())
            ended = true;
        if (ended && (n != 0 || std::memcmp(r.data(), game.data(), game.size()) != 0))
            lateWrite = true;
        // from the end of the leg decay (hold frames + kLegDecaySeconds + one frame): the legs are the game's, every other non-Hips
        // bone too
        if (ts > float(vt::kMountHoldFrames) / 60.0f + vt::kLegDecaySeconds + 1.0f / 60.0f)
        {
            ++legCheckedFrames;
            for (uint32_t j : {LUpLeg, LLeg, RUpLeg, RLeg})
                if (std::memcmp(Rec(r, j), Rec(game, j), 48) != 0)
                    legsExact = false;
            for (uint32_t j : {Spine, Spine1, LArm, RArm, Head})
                if (std::memcmp(Rec(r, j), Rec(game, j), 48) != 0)
                    otherExact = false;
            if (!prevOut.empty())
            {
                // the Hips against the game's own Hips motion: the change of the offset per frame (continuity through the expiry)
                const float* h = Rec(r, Hips);
                const float* hg = Rec(game, Hips);
                const float* ph = Rec(prevOut, Hips);
                const float* phg = Rec(prevGame, Hips);
                float d = 0.0f;
                for (int c = 0; c < 3; ++c)
                    d += ((h[c] - hg[c]) - (ph[c] - phg[c])) * ((h[c] - hg[c]) - (ph[c] - phg[c]));
                maxHipsStepCm = std::fmax(maxHipsStepCm, std::sqrt(d) * 100.0f);
                const float rotNow = AngleDeg(RecQ(r, Hips), RecQ(game, Hips));
                const float rotPrev = AngleDeg(RecQ(prevOut, Hips), RecQ(prevGame, Hips));
                maxHipsRotDeg = std::fmax(maxHipsRotDeg, std::fabs(rotNow - rotPrev));
            }
        }
        prevGame = game;
        prevOut = r;
    }
    std::printf("  T9 legs after the leg decay: checked %d frames, legs exact %d, other bones exact %d, legMoveAt %.3f, Hips offset change max "
                "%.2f cm / %.2f deg per frame, epoch ended %d, written after the end %d\n",
                legCheckedFrames, legsExact ? 1 : 0, otherExact ? 1 : 0, double(vt::g_mountLegMoveAt.load()), double(maxHipsStepCm),
                double(maxHipsRotDeg), ended ? 1 : 0, lateWrite ? 1 : 0);
    Check('B', legCheckedFrames > 40 && legsExact && otherExact,
          "T9 after the leg decay the epoch writes no leg (and no bone but the Hips): records bit-exact the game's");
    Check('B', maxHipsStepCm <= 0.5f && maxHipsRotDeg <= 0.5f,
          "T9 the Hips root offset changes <= 0.5 cm / 0.5 deg per frame through the expiry (" + std::to_string(maxHipsStepCm) + " cm, " +
              std::to_string(maxHipsRotDeg) + " deg)");
    Check('B', ended && !lateWrite, "T9 the epoch ends by kRootTrackSeconds + kRootTailSeconds and nothing is written after it");
    vt::Abort();
}

// ------------------------------------------------------------------------------------------------ (C) seated neck cut
// Recorded live game pose (stage s0) of the 16 spine / upper-body bones, recording 2026-10-29 01:28:54: ride 1 (mount cut 43.205,
// seating edge 44.616, frozen frame 44.601, pop 44.617) and ride 2 (mount 57.690, edge 59.105, frozen frame 59.090, no pop).
#include "vehtrans_neckpop_fixture.inc"
constexpr uint32_t kNkFirst = 2;             // database: Root, Trajectory, then the recorded bones
constexpr uint32_t kNkDb = kNkFirst + kNeckBones;
constexpr uint32_t kNkGameCount = kNkDb + 5; // db b -> game b + 3
enum NkBone : uint32_t { NkHips = 0, NkSpine = 1, NkSpine1 = 2, NkNeck = 5, NkNeck1 = 6, NkHead = 7, NkLArm = 9, NkRHand = 15 };
uint64_t NeckGameHash(int32_t aGi)
{
    const int32_t i = aGi - 3 - int32_t(kNkFirst);
    return (i >= 0 && i < int32_t(kNeckBones)) ? integ::CNameHash(kNeckBoneNames[i]) : 0ull;
}
integ::PoseSnapshot NeckSnap(const NeckFrame& f)
{
    integ::PoseSnapshot s;
    s.weight = 1.0f;
    s.numBones = kNkDb;
    s.firstBone = kNkFirst;
    s.rig = kRig;
    s.sequence = 1;
    for (uint32_t b = 0; b < integ::kMaxMMBones; ++b)
        s.gameIndex[b] = b < kNkDb ? int32_t(b + 3) : -1;
    for (uint32_t i = 0; i < kNeckBones; ++i)
        for (int k = 0; k < 7; ++k)
            s.pose[size_t(kNkFirst + i) * 7 + k] = f.b[i][k];
    s.upperScale = 1.0f;
    return s;
}
Records NeckRecords(const NeckFrame& f)
{
    Records r(size_t(kNkGameCount) * 48, 0);
    for (uint32_t g = 0; g < kNkGameCount; ++g)
    {
        float* v = reinterpret_cast<float*>(r.data() + size_t(g) * 48);
        v[0] = 0.01f * float(g); v[1] = 0.5f; v[2] = -0.25f; v[3] = 0.0f;
        v[4] = 0.0f; v[5] = 0.0f; v[6] = 0.0f; v[7] = 1.0f;
        v[8] = 1.0f; v[9] = 1.0f; v[10] = 1.0f; v[11] = 0.0f;
    }
    for (uint32_t i = 0; i < kNeckBones; ++i)
    {
        float* v = reinterpret_cast<float*>(r.data() + size_t(kNkFirst + i + 3) * 48);
        v[0] = f.b[i][0]; v[1] = f.b[i][1]; v[2] = f.b[i][2];
        v[4] = f.b[i][3]; v[5] = f.b[i][4]; v[6] = f.b[i][5]; v[7] = f.b[i][6];
    }
    return r;
}
const float* NkRec(const Records& r, uint32_t i) { return reinterpret_cast<const float*>(r.data() + size_t(kNkFirst + i + 3) * 48); }
Q4 NkQ(const Records& r, uint32_t i) { const float* v = NkRec(r, i); return {v[4], v[5], v[6], v[7]}; }
void QRot(const Q4& q, const float v[3], float o[3])
{
    const float tx = 2.0f * (q.y * v[2] - q.z * v[1]);
    const float ty = 2.0f * (q.z * v[0] - q.x * v[2]);
    const float tz = 2.0f * (q.x * v[1] - q.y * v[0]);
    o[0] = v[0] + q.w * tx + (q.y * tz - q.z * ty);
    o[1] = v[1] + q.w * ty + (q.z * tx - q.x * tz);
    o[2] = v[2] + q.w * tz + (q.x * ty - q.y * tx);
}
// model space (FK along the recorded parent chain, Hips = root) of one recorded bone
void NkModel(const Records& r, uint32_t i, float aP[3], Q4& aQ)
{
    const int p = kNeckParent[i];
    const float* v = NkRec(r, i);
    const Q4 lq{v[4], v[5], v[6], v[7]};
    if (p < 0)
    {
        aP[0] = v[0]; aP[1] = v[1]; aP[2] = v[2];
        aQ = lq;
        return;
    }
    float pp[3];
    Q4 pq;
    NkModel(r, uint32_t(p), pp, pq);
    float rv[3];
    QRot(pq, v, rv);
    for (int k = 0; k < 3; ++k)
        aP[k] = pp[k] + rv[k];
    aQ = Mul(pq, lq);
}
// Head position relative to the Hips (model space), m
void NkHeadRel(const Records& r, float aOut[3])
{
    float hp[3], h[3];
    Q4 q;
    NkModel(r, NkHips, hp, q);
    NkModel(r, NkHead, h, q);
    for (int k = 0; k < 3; ++k)
        aOut[k] = h[k] - hp[k];
}
// rotation vector (axis x angle, deg) of a * b^-1
void RotVecDeg(const Q4& a, const Q4& b, float aOut[3])
{
    Q4 d = Mul(a, Q4{-b.x, -b.y, -b.z, b.w});
    if (d.w < 0.0f)
        d = {-d.x, -d.y, -d.z, -d.w};
    const float s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    const float ang = 2.0f * std::atan2(s, d.w) / kDeg;
    const float k = s > 1e-9f ? ang / s : 0.0f;
    aOut[0] = d.x * k; aOut[1] = d.y * k; aOut[2] = d.z * k;
}
Q4 Slerp(const Q4& a, const Q4& b, float u)
{
    const float qa[4] = {a.x, a.y, a.z, a.w};
    float qb[4] = {b.x, b.y, b.z, b.w};
    if (qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3] < 0.0f)
        for (float& v : qb)
            v = -v;
    const float aInv[4] = {-qa[0], -qa[1], -qa[2], qa[3]};
    float rel[4], p[4], o[4];
    vt::detail::QMul(aInv, qb, rel);
    vt::detail::QNormalize(rel);
    vt::detail::QPow(rel, u, p);
    vt::detail::QMul(qa, p, o);
    vt::detail::QNormalize(o);
    return {o[0], o[1], o[2], o[3]};
}

bool g_neckVerbose = false; // test argument -v: per-frame lines of the neck replays
struct NeckRun
{
    uint64_t fires = 0, seatWrites = 0; // the NECK guard's fires / writes (neckFires / neckWrites counters)
    uint64_t seatGuardFires = 0, seatGuardWrites = 0; // the seat pose switch guard's counters during the same replay (must stay 0)
    bool seatFieldsUntouched = true; // a neck fire leaves the seat guard's last-fire fields (g_seatLast*) alone
    int neckFired = 0;
    float neckHipsCm = -1.0f, neckTorsoDeg = -1.0f;
    float neckStepDeg = 0.0f, edgeAt = -1.0f, fireAt = -1.0f;
    bool untouchedBefore = true; // nothing written before the pop frame (enter clip, frozen frame)
    bool exactAfter = true;      // nothing written from kSeatNeckSeconds after the pop on
    bool popFrameIsPrev = false; // the pop frame shows the previous frame's neck chain
    bool repeatUnchanged = true; // a second FK-stage call on the same frame changes nothing
    bool mapStageUntouched = true; // the mapping-stage guards never wrote
    uint64_t neckCalls = 0;
    float maxHeadStepCm = 0.0f, maxNeck1StepDeg = 0.0f, gameMaxHeadStepCm = 0.0f, gameMaxNeck1StepDeg = 0.0f;
    float maxLineDevCm = 0.0f, gameLineDevCm = 0.0f, maxOppositeDeg = 0.0f, maxAlongDeg = 0.0f;
};
// One replay in the production call order of a Mapping-owner frame (2026-10-29 18:22:46): first the early mapping job (FkSideMapping
// -> FkSide under the vehicle hold's denial: mount transition, seat guard, pop filter) on the game pose of the mapping stage, which
// does not carry the pop (aMap; nullptr = the same recorded frame), then the FK hook (FkHook ApplyMotionMatching -> NeckGuard) on the
// recorded pose the FK pass receives (s0); the output is the s1 pose. The game rewrites the pose between both stages, so the mapping
// stage's writes never reach the FK stage. Each frame also gets a second FK-stage call 0.5 ms later on the written records.
// aEdge = the logged seating edge (straight-line window edge - 50 ms .. edge + 300 ms), aPopAt = the pop frame (< 0 = none). aStride 2 =
// 30 fps (every other frame, the pop frame and the frame before it kept).
NeckRun ReplayNeck(const NeckFrame* aFr, size_t aN, double aMountAt, double aEdge, double aPopAt, int aStride = 1,
                   const NeckFrame* aMap = nullptr, bool aTpp = true)
{
    NeckRun out;
    vt::SetBoneRoles(vt::BuildBoneRolesFromGame(NeckSnap(aFr[0]), NeckGameHash));
    const float p0[3] = {0, 0, 0};
    const int64_t t0 = NextEpochTime();
    const uint64_t fires0 = vt::g_counters.neckFires.load(), writes0 = vt::g_counters.neckWrites.load();
    const uint64_t sFires0 = vt::g_counters.seatFires.load(), sWrites0 = vt::g_counters.seatWrites.load();
    vt::g_seatLastHipsCm.store(-7.0f);
    vt::g_seatLastTorsoDeg.store(-7.0f);
    vt::g_seatLastAt.store(-7.0f);
    const uint64_t calls0 = vt::g_neckCalls.load();
    vt::BeginMount(NeckSnap(aFr[0]), p0, 0.0f, t0);
    vt::SetVehicleCapture(true, kRig);
    vt::SetMountTpp(aTpp);
    vt::SetPopFilter(aTpp);
    std::vector<size_t> idx;
    for (size_t i = 0; i < aN; ++i)
    {
        const bool popOrBefore = aPopAt >= 0.0 && (std::fabs(aFr[i].t - aPopAt) < 0.004 ||
                                                   (i + 1 < aN && std::fabs(aFr[i + 1].t - aPopAt) < 0.004));
        if (aStride == 1 || (i % size_t(aStride)) == 0 || popOrBefore)
            if (idx.empty() || aFr[i].t - aFr[idx.back()].t > 0.004)
                idx.push_back(i);
    }
    struct Shown
    {
        double t;
        float head[3], gameHead[3];
        Q4 headQ, gameHeadQ;
    };
    std::vector<Shown> shown;
    Records prevR, prevGame;
    double prevT = 0.0;
    for (size_t i : idx)
    {
        const double tr = aFr[i].t;
        const int64_t t = t0 + int64_t((tr - aMountAt) * double(kTps));
        // mapping stage (its own copy of the pose; the game rewrites the upper body after it)
        const Records mapGame = NeckRecords(aMap ? aMap[i] : aFr[i]);
        Records mapR = mapGame;
        vt::FkSide(mapR.data(), kNkGameCount, kRig, t, false);
        if (std::memcmp(mapR.data(), mapGame.data(), mapGame.size()) != 0)
            out.mapStageUntouched = false;
        // FK stage
        const Records game = NeckRecords(aFr[i]);
        Records r = game;
        vt::NeckGuard(r.data(), kNkGameCount, kRig, t + 1000);
        const Records once = r;
        vt::NeckGuard(r.data(), kNkGameCount, kRig, t + 1500);
        if (std::memcmp(r.data(), once.data(), once.size()) != 0)
            out.repeatUnchanged = false;
        const bool same = std::memcmp(r.data(), game.data(), game.size()) == 0;
        const bool isPop = aPopAt >= 0.0 && std::fabs(tr - aPopAt) < 0.004;
        if (!same && (aPopAt < 0.0 || tr < aPopAt - 0.004))
        {
            out.untouchedBefore = false;
            std::printf("  neck: write at %.4f before the pop\n", tr);
        }
        if (!same && aPopAt >= 0.0 && tr > aPopAt + double(vt::kSeatNeckSeconds) + 0.001)
            out.exactAfter = false;
        Shown s{};
        s.t = tr;
        NkHeadRel(r, s.head);
        NkHeadRel(game, s.gameHead);
        float mp[3];
        NkModel(r, NkHead, mp, s.headQ);
        NkModel(game, NkHead, mp, s.gameHeadQ);
        if (!prevR.empty() && tr - prevT < 0.05)
        {
            out.maxHeadStepCm = std::fmax(out.maxHeadStepCm, 100.0f * Dist3(s.head, shown.back().head));
            out.gameMaxHeadStepCm = std::fmax(out.gameMaxHeadStepCm, 100.0f * Dist3(s.gameHead, shown.back().gameHead));
            out.maxNeck1StepDeg = std::fmax(out.maxNeck1StepDeg, AngleDeg(NkQ(r, NkNeck1), NkQ(prevR, NkNeck1)));
            out.gameMaxNeck1StepDeg = std::fmax(out.gameMaxNeck1StepDeg, AngleDeg(NkQ(game, NkNeck1), NkQ(prevGame, NkNeck1)));
        }
        if (isPop && !prevR.empty())
            out.popFrameIsPrev = AngleDegFine(NkQ(r, NkNeck1), NkQ(prevR, NkNeck1)) < 0.05 &&
                                 AngleDegFine(NkQ(r, NkHead), NkQ(prevR, NkHead)) < 0.05 &&
                                 AngleDegFine(NkQ(r, NkNeck), NkQ(prevR, NkNeck)) < 0.05;
        shown.push_back(s);
        prevR = r;
        prevGame = game;
        prevT = tr;
    }
    // straight line from edge - 50 ms to edge + 300 ms (Hips-relative Head position; model-space Head rotation for the overshoot)
    size_t ia = shown.size(), ib = shown.size();
    for (size_t k = 0; k < shown.size(); ++k)
    {
        if (ia == shown.size() && shown[k].t >= aEdge - 0.05)
            ia = k;
        if (shown[k].t <= aEdge + 0.30)
            ib = k;
    }
    if (ia < ib && ib < shown.size())
    {
        const Shown& A = shown[ia];
        const Shown& B = shown[ib];
        // pop direction: the game's Head rotation on the pop frame against the line
        float popAxis[3] = {0, 0, 0};
        for (size_t k = ia; k <= ib; ++k)
            if (aPopAt >= 0.0 && std::fabs(shown[k].t - aPopAt) < 0.004)
            {
                const float u = float((shown[k].t - A.t) / (B.t - A.t));
                RotVecDeg(shown[k].gameHeadQ, Slerp(A.headQ, B.headQ, u), popAxis);
            }
        const float pl = std::sqrt(popAxis[0] * popAxis[0] + popAxis[1] * popAxis[1] + popAxis[2] * popAxis[2]);
        for (size_t k = ia; k <= ib; ++k)
        {
            const float u = float((shown[k].t - A.t) / (B.t - A.t));
            float lp[3], lg[3];
            for (int c = 0; c < 3; ++c)
            {
                lp[c] = A.head[c] + (B.head[c] - A.head[c]) * u;
                lg[c] = A.gameHead[c] + (B.gameHead[c] - A.gameHead[c]) * u;
            }
            out.maxLineDevCm = std::fmax(out.maxLineDevCm, 100.0f * Dist3(shown[k].head, lp));
            out.gameLineDevCm = std::fmax(out.gameLineDevCm, 100.0f * Dist3(shown[k].gameHead, lg));
            if (pl > 1e-3f)
            {
                float v[3];
                RotVecDeg(shown[k].headQ, Slerp(A.headQ, B.headQ, u), v);
                const float proj = (v[0] * popAxis[0] + v[1] * popAxis[1] + v[2] * popAxis[2]) / pl;
                out.maxOppositeDeg = std::fmax(out.maxOppositeDeg, -proj);
                out.maxAlongDeg = std::fmax(out.maxAlongDeg, proj);
                if (g_neckVerbose)
                {
                    float vg[3];
                    RotVecDeg(shown[k].gameHeadQ, Slerp(A.headQ, B.headQ, u), vg);
                    const float projG = (vg[0] * popAxis[0] + vg[1] * popAxis[1] + vg[2] * popAxis[2]) / pl;
                    std::printf("    %.4f along the pop: shown %+.1f deg game %+.1f deg; head line dev %.1f cm\n", shown[k].t,
                                double(proj), double(projG), double(100.0f * Dist3(shown[k].head, lp)));
                }
            }
        }
    }
    out.fires = vt::g_counters.neckFires.load() - fires0;
    out.seatWrites = vt::g_counters.neckWrites.load() - writes0;
    out.seatGuardFires = vt::g_counters.seatFires.load() - sFires0;
    out.seatGuardWrites = vt::g_counters.seatWrites.load() - sWrites0;
    out.seatFieldsUntouched = vt::g_seatLastHipsCm.load() == -7.0f && vt::g_seatLastTorsoDeg.load() == -7.0f && vt::g_seatLastAt.load() == -7.0f;
    out.neckHipsCm = vt::g_neckLastHipsCm.load();
    out.neckTorsoDeg = vt::g_neckLastTorsoDeg.load();
    out.neckFired = vt::g_seatNeckFired.load();
    out.neckStepDeg = vt::g_seatNeckStepDeg.load();
    out.edgeAt = vt::g_seatNeckEdgeAt.load();
    out.fireAt = out.fires ? vt::g_neckLastAt.load() : -1.0f;
    out.neckCalls = vt::g_neckCalls.load() - calls0;
    vt::SetPopFilter(false);
    vt::Abort();
    SetRoles();
    return out;
}
void PrintNeck(const char* aWhat, const NeckRun& n)
{
    std::printf("  neck %s: fires %llu (neck %d, step %.1f deg, at %.3f s, edge %.3f s), seat writes %llu; shown max Head step %.1f cm "
                "(game %.1f), Neck1 %.1f deg (game %.1f); line dev %.1f cm (game %.1f); along the pop %.1f deg, opposite %.1f deg; "
                "untouched before %d, pop frame = previous %d, exact after %d, repeat call unchanged %d, mapping stage untouched %d, "
                "FK-stage calls %llu\n",
                aWhat, static_cast<unsigned long long>(n.fires), n.neckFired, double(n.neckStepDeg), double(n.fireAt), double(n.edgeAt),
                static_cast<unsigned long long>(n.seatWrites), double(n.maxHeadStepCm), double(n.gameMaxHeadStepCm),
                double(n.maxNeck1StepDeg), double(n.gameMaxNeck1StepDeg), double(n.maxLineDevCm), double(n.gameLineDevCm),
                double(n.maxAlongDeg), double(n.maxOppositeDeg), n.untouchedBefore ? 1 : 0, n.popFrameIsPrev ? 1 : 0,
                n.exactAfter ? 1 : 0, n.repeatUnchanged ? 1 : 0, n.mapStageUntouched ? 1 : 0,
                static_cast<unsigned long long>(n.neckCalls));
}
void TestNeckPop()
{
    const vt::BoneRoles roles = vt::BuildBoneRolesFromGame(NeckSnap(kNeckRide1[0]), NeckGameHash);
    uint32_t nNeck = 0, nUpper = 0;
    for (uint32_t b = 0; b < integ::kMaxMMBones; ++b)
    {
        nNeck += roles.neck[b];
        nUpper += roles.upper[b];
    }
    Check('C', nNeck == 3 && roles.neck[kNkFirst + NkNeck] && roles.neck[kNkFirst + NkNeck1] && roles.neck[kNkFirst + NkHead] &&
                   nUpper == 14 && !roles.upper[kNkFirst + NkHips] && !roles.upper[kNkFirst + NkSpine] &&
                   roles.upper[kNkFirst + NkSpine1] && roles.upper[kNkFirst + NkLArm] && roles.upper[kNkFirst + NkRHand],
          "neck roles Neck / Neck1 / Head; upper roles Spine1..Head, shoulders, arms, forearms, hands (14)");
    const vt::BoneRoles byName = vt::BuildBoneRoles(kNeckBones, 0, [](uint32_t b) { return std::string_view(kNeckBoneNames[b]); });
    Check('C', byName.neck[NkNeck] && byName.neck[NkHead] && !byName.neck[NkSpine1] && byName.upper[NkSpine1] && !byName.upper[NkSpine],
          "roles by database names: the same neck / upper sets");
    const size_t n1 = sizeof kNeckRide1 / sizeof kNeckRide1[0], n2 = sizeof kNeckRide2 / sizeof kNeckRide2[0];
    // ride 1 without the pop: the pop frames (44.617 .. 44.849) replaced by a smooth blend (smoothstep in time, slerp per bone) from
    // the frozen frame's pose to the recorded seated pose of 44.866 - the mapping-stage pose of ride 1 and the enter clip case
    std::vector<NeckFrame> clip(kNeckRide1, kNeckRide1 + n1);
    {
        size_t frozen = 0, settled = 0;
        for (size_t i = 0; i < clip.size(); ++i)
        {
            if (std::fabs(clip[i].t - 44.6006) < 0.004)
                frozen = i;
            if (std::fabs(clip[i].t - 44.8661) < 0.004)
                settled = i;
        }
        const NeckFrame a = clip[frozen], z = clip[settled];
        for (size_t i = frozen + 1; i < settled; ++i)
        {
            const float u = vt::detail::Smoothstep(float((clip[i].t - a.t) / (z.t - a.t)));
            for (uint32_t j = 0; j < kNeckBones; ++j)
            {
                for (int k = 0; k < 3; ++k)
                    clip[i].b[j][k] = a.b[j][k] + (z.b[j][k] - a.b[j][k]) * u;
                const Q4 q = Slerp(Q4{a.b[j][3], a.b[j][4], a.b[j][5], a.b[j][6]}, Q4{z.b[j][3], z.b[j][4], z.b[j][5], z.b[j][6]}, u);
                clip[i].b[j][3] = q.x; clip[i].b[j][4] = q.y; clip[i].b[j][5] = q.z; clip[i].b[j][6] = q.w;
            }
        }
    }
    // no FK-stage guard while the mount transition runs (epoch active); the guard is armed by BeginMount
    {
        vt::SetBoneRoles(vt::BuildBoneRolesFromGame(NeckSnap(kNeckRide1[0]), NeckGameHash));
        const float p0[3] = {0, 0, 0};
        const int64_t t0 = NextEpochTime();
        vt::BeginMount(NeckSnap(kNeckRide1[0]), p0, 0.0f, t0);
        vt::SetMountTpp(true);
        const uint64_t calls0 = vt::g_neckCalls.load();
        Records r = NeckRecords(kNeckRide1[0]);
        const Records before = r;
        const uint32_t w = vt::NeckGuard(r.data(), kNkGameCount, kRig, t0 + kFrame);
        Check('C', vt::MountActive() && w == 0 && vt::g_neckCalls.load() == calls0 && std::memcmp(r.data(), before.data(), r.size()) == 0,
              "NeckGuard stays out while the mount transition runs");
        vt::Abort();
        SetRoles();
    }
    const NeckRun r1 = ReplayNeck(kNeckRide1, n1, 43.205, 44.616, 44.6172, 1, clip.data());
    PrintNeck("ride 1 (pop 44.617 at the FK stage only)", r1);
    Check('C', r1.gameMaxNeck1StepDeg > 30.0f && r1.gameMaxHeadStepCm > 20.0f, "fixture: the recorded pop (Neck1 > 30 deg, Head > 20 cm)");
    Check('C', r1.mapStageUntouched, "ride 1: the mapping-stage guards write nothing (the pop is not in their pose)");
    Check('C', r1.neckCalls > 40, "ride 1: the FK-stage guard runs on the ride frames after the mount transition");
    Check('C', r1.fires == 1 && r1.neckFired == 1, "ride 1: the neck guard fires once");
    // 2026-11-01: the neck cut counts apart from the seat pose switch guard (the ride line's 'fires' summed both and the neck's Hips
    // step overwrote the seat guard's field): the neck fire shows in neckFires / neckLast*, the seat guard stays at 0 / untouched
    Check('C', r1.seatGuardFires == 0 && r1.seatGuardWrites == 0 && r1.seatFieldsUntouched,
          "ride 1: a neck fire does not count as a seat guard fire, does not count its writes and leaves g_seatLast* alone");
    Check('C', r1.fireAt > 1.38f && r1.fireAt < 1.60f && r1.neckHipsCm >= 0.0f && r1.neckTorsoDeg >= 0.0f,
          "ride 1: the neck fire has its own time / Hips step / torso step fields (g_neckLast*)");
    Check('C', r1.edgeAt > 1.38f && r1.edgeAt < 1.41f, "ride 1: seating edge = the frozen frame 44.601 (1.396 s after the mount)");
    Check('C', r1.untouchedBefore && r1.popFrameIsPrev, "ride 1: nothing written before the pop; the pop frame shows the previous neck chain");
    Check('C', r1.maxHeadStepCm <= 5.0f && r1.maxNeck1StepDeg <= 8.0f,
          "ride 1: Head step <= 5 cm and Neck1 <= 8 deg per frame (" + std::to_string(r1.maxHeadStepCm) + " cm, " +
              std::to_string(r1.maxNeck1StepDeg) + " deg)");
    Check('C', r1.maxLineDevCm <= 5.0f,
          "ride 1: Head within 5 cm of the straight line edge - 50 ms .. edge + 300 ms (" + std::to_string(r1.maxLineDevCm) + " cm)");
    Check('C', r1.maxOppositeDeg <= 10.0f,
          "ride 1: no overshoot against the pop direction > 10 deg (" + std::to_string(r1.maxOppositeDeg) + " deg)");
    Check('C', r1.exactAfter && r1.repeatUnchanged, "ride 1: nothing written after kSeatNeckSeconds; a repeated FK call writes nothing more");
    // the mapping stage sees the pop too (its pop filter fires there; the FK stage still gets the game's pose)
    const NeckRun r1m = ReplayNeck(kNeckRide1, n1, 43.205, 44.616, 44.6172);
    PrintNeck("ride 1, pop also at the mapping stage", r1m);
    Check('C', r1m.fires == 1 && r1m.popFrameIsPrev && r1m.maxHeadStepCm <= 5.0f && r1m.maxNeck1StepDeg <= 8.0f &&
                   r1m.maxLineDevCm <= 5.0f && r1m.maxOppositeDeg <= 10.0f && r1m.exactAfter && r1m.repeatUnchanged,
          "ride 1 with the pop also at the mapping stage: the FK-stage result holds the limits");
    const NeckRun r2 = ReplayNeck(kNeckRide2, n2, 57.690, 59.105, -1.0);
    PrintNeck("ride 2 (no pop)", r2);
    Check('C', r2.fires == 0 && r2.neckFired == 0 && r2.seatWrites == 0 && r2.seatGuardFires == 0 && r2.seatGuardWrites == 0 &&
                   r2.untouchedBefore && r2.neckCalls > 40,
          "ride 2 (seating edge, frozen frame 59.090, no pop): the guard runs, no fire, nothing written");
    Check('C', r2.edgeAt > 1.39f && r2.edgeAt < 1.41f, "ride 2: seating edge = the frozen frame 59.090");
    const NeckRun rc = ReplayNeck(clip.data(), clip.size(), 43.205, 44.616, -1.0);
    PrintNeck("enter clip without the pop", rc);
    Check('C', rc.fires == 0 && rc.seatWrites == 0 && rc.untouchedBefore, "enter clip + seating edge without the pop: no fire, nothing written");
    const NeckRun rf = ReplayNeck(kNeckRide1, n1, 43.205, 44.616, 44.6172, 1, clip.data(), false);
    PrintNeck("ride 1 in FPP", rf);
    Check('C', rf.fires == 0 && rf.seatWrites == 0 && rf.untouchedBefore, "first-person ride: no fire, nothing written");
    // 30 fps (every other frame; the frozen frame and the pop frame kept)
    const NeckRun r30 = ReplayNeck(kNeckRide1, n1, 43.205, 44.616, 44.6172, 2, clip.data());
    PrintNeck("ride 1 at 30 fps", r30);
    Check('C', r30.fires == 1 && r30.maxNeck1StepDeg <= 16.0f && r30.maxOppositeDeg <= 10.0f && r30.repeatUnchanged,
          "ride 1 at 30 fps: fires, Neck1 <= 16 deg per 1/30 s, no overshoot > 10 deg");
}

// ------------------------------------------------------------------------------------------------ (E) door hold cap
// Recording 2026-11-02 02:01-02:50 (car doors, 30 rides, the recorded log): the standing-entry hold keeps the
// rendered root at the pre-mount spot while the door opens; in hella ride 4 and bratsk ride 30 (columbus ride 9 by the model) that spot lies
// in the swept sector. The fixture (tests/vehtrans_doorcap_fixture.inc, generated by gen_vehtrans_doorcap_fixture.py) holds the entity root
// of every ride (vehicle frame, 'vtpp' samples), the door angle against time of the door-data frames, and a copy of the generated arcs
// (UTPPCarDoorArcOf). The replay feeds the entity at 50 Hz (linear between the logged samples; 50 Hz reproduces the logged hold / jump mode of 29 of 30 rides,
// 60 Hz+ turns jump rides 14 / 16 / 23 / 26 / 27 into holds) and renders at 100 Hz through the real
// BeginMount / SetHoldCap / UpdateWorld / FkSide / RootOffset code; the vehicle is static, the entity yaw is constant 0 (the logged yaw is
// not in the series: the yaw align path is covered by group D, not here).
#include "vehtrans_doorcap_fixture.inc"

struct ArcFx { bool valid = false; float hx = 0, hy = 0, length = 0, closedDeg = 0, openDeg = 0, delay = 0, dur = 0; };
ArcFx ArcOfSeat(const char* aSeat)
{
    ArcFx a;
    for (const DoorArcFx& d : kDoorArcs)
        if (std::strcmp(d.seat, aSeat) == 0)
        {
            a.valid = true; a.hx = d.hx; a.hy = d.hy; a.length = d.length; a.closedDeg = d.closedDeg; a.openDeg = d.openDeg;
            a.delay = d.delay; a.dur = d.dur;
        }
    return a;
}

// MIRROR of UTPPDoorCapOf (scripts/utpp_vehicle_door.reds): change both together.
struct CapInfo { int why = 0; float r = 0, theta = 0, tEdge = -1.0f, cap = -1.0f; };
CapInfo RedsCap(const ArcFx& arc, float px, float py)
{
    CapInfo info;
    if (!arc.valid) { info.why = 1; return info; }
    const float body = 0.20f;
    const float dx = px - arc.hx, dy = py - arc.hy;
    const float r = std::sqrt(dx * dx + dy * dy);
    info.r = r;
    if (r > arc.length + body) { info.why = 2; return info; }
    const float cd = arc.closedDeg * kDeg;
    const float ax = std::cos(cd), ay = std::sin(cd);
    const float dot = ax * dx + ay * dy, cross = ax * dy - ay * dx;
    const float c = std::fmax(-1.0f, std::fmin(1.0f, dot / std::fmax(r, 0.0001f)));
    float ang = std::acos(c) / kDeg;
    if (cross < 0.0f) ang = -ang;
    const float th = ang * (arc.openDeg < 0.0f ? -1.0f : 1.0f);
    info.theta = th;
    const float delta = std::asin(std::fmin(1.0f, body / std::fmax(r, body))) / kDeg;
    const float op = std::fabs(arc.openDeg);
    if (th + delta < 0.0f || th - delta >= op) { info.why = 3; return info; }
    const float lo = std::fmax(th - delta, 0.0f);
    const float s = std::fmax(0.0f, std::fmin(1.0f, lo / op));
    info.tEdge = arc.delay + arc.dur * (0.5f - std::sin(std::asin(1.0f - 2.0f * s) / 3.0f));
    if (info.tEdge > 0.75f) { info.why = 4; return info; }
    info.cap = std::fmax(info.tEdge - 0.05f, 0.0f);
    return info;
}

// distance (m) from a point to the door panel (hinge -> rear edge) at an opening angle (deg from the closed door, magnitude)
float DoorDist(const ArcFx& arc, float aAngleDeg, float px, float py)
{
    const float sgn = arc.openDeg < 0.0f ? -1.0f : 1.0f;
    const float a = (arc.closedDeg + sgn * aAngleDeg) * kDeg;
    const float vx = std::cos(a) * arc.length, vy = std::sin(a) * arc.length;
    const float dx = px - arc.hx, dy = py - arc.hy;
    float u = (dx * vx + dy * vy) / (arc.length * arc.length);
    u = std::fmax(0.0f, std::fmin(1.0f, u));
    return std::hypot(dx - u * vx, dy - u * vy);
}
float InterpT(const float* aT, int aN, const float* aV, int aStride, int aComp, float aTime)
{
    if (aTime <= aT[0]) return aV[aComp];
    for (int i = 1; i < aN; ++i)
        if (aTime <= aT[i])
        {
            const float w = (aTime - aT[i - 1]) / (aT[i] - aT[i - 1]);
            return aV[(i - 1) * aStride + aComp] + (aV[i * aStride + aComp] - aV[(i - 1) * aStride + aComp]) * w;
        }
    return aV[(aN - 1) * aStride + aComp];
}
float DoorAngleAt(const DoorRideFx& r, float aTime)
{
    const float step = 0.02f;
    if (aTime <= 0.0f) return r.deg[0];
    const int i = int(aTime / step);
    if (i >= r.nDeg - 1) return r.deg[r.nDeg - 1];
    const float w = (aTime - float(i) * step) / step;
    return r.deg[i] + (r.deg[i + 1] - r.deg[i]) * w;
}

struct DoorRun
{
    std::vector<float> t, vx, vy, ex, ey; // per animation frame: time, visible root, entity root
    std::vector<float> rox, roy, hx, hy;  // per animation frame: RootOffset xy, and the RENDERED Hips record xy after FkSide
    uint64_t hash = 0xcbf29ce484222325ull;
    int capState = 0, capExtra = 0;
    float capStart = -1.0f, capTail = 0.0f, engageAt = -1.0f, jumpAt = -1.0f, peakDist = 0.0f, jumpPeak = 0.0f, jumpEnd = -1.0f;
    float offsetPop = 0.0f; // largest per-frame change of (visible - entity)
    float peakSpeed = 0.0f; // fastest visible root (m/s)
    float entPeakSpeed = 0.0f; // fastest entity root, i.e. the vanilla slide itself (m/s)
    float travel = 0.0f;
    bool epochEnded = false; // the mount transition has expired by the end of the replay
    std::vector<float> ft, fvx, fvy, fex, fey; // per feed instant (UpdateWorld sample, plus t = 0): time, visible root, entity root
};
// aCapSec < 0: no cap call. aCapAt: seconds after BeginMount of the SetHoldCap call (< 0 = before BeginMount).
DoorRun RunDoorRide(const DoorRideFx& r, bool aCar, float aCapSec, float aCapAt, float aCap2Sec = -1.0f, float aCap2At = 0.0f)
{
    SetRoles();
    Pose g = GamePoseB();
    g.p[Hips] = {0.0f, 0.0f, 0.973f};
    const Records game = MakeRecords(g);
    const float p0[3] = {r.xy[0], r.xy[1], 0.0f};
    const int64_t t0 = NextEpochTime();
    if (aCapSec >= 0.0f && aCapAt < 0.0f)
        vt::SetHoldCap(aCapSec, t0 + int64_t(double(aCapAt) * double(kTps)));
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0, aCar);
    vt::SetMountTpp(true);
    DoorRun run;
    // 50 Hz feed reproduces the logged hold / jump mode of 29 of the 30 door2 rides (ride 3 engaged by yaw); the door3 rides (id >= 100) were fed at
    // 55-70 Hz (mountdiag +9 / +27 / +41 ms): 60 Hz keeps their hold path (a 50 Hz step of the emperor's 8 m/s slide is a jump)
    const int feedHz = r.ride >= 100 ? 60 : 50, animHz = 100;
    int nextFeed = 1;
    bool capSent = aCapSec < 0.0f || aCapAt < 0.0f;
    bool cap2Sent = aCap2Sec < 0.0f;
    float prevVx = 0, prevVy = 0, prevOx = 0, prevOy = 0, prevEx = 0, prevEy = 0;
    const int frames = int(1.6f * float(animHz));
    auto feedSample = [&](float aTf) {
        float ro[3] = {};
        const bool on = vt::RootOffset(t0 + int64_t(double(aTf) * double(kTps)), ro);
        const float ex = InterpT(r.t, r.n, r.xy, 2, 0, aTf), ey = InterpT(r.t, r.n, r.xy, 2, 1, aTf);
        run.ft.push_back(aTf); run.fex.push_back(ex); run.fey.push_back(ey);
        run.fvx.push_back(ex + (on ? ro[0] : 0.0f)); run.fvy.push_back(ey + (on ? ro[1] : 0.0f));
    };
    feedSample(0.0f);
    for (int j = 0; j <= frames; ++j)
    {
        const float tj = float(j) / float(animHz);
        const int64_t ticks = t0 + int64_t(double(tj) * double(kTps));
        if (!capSent && tj >= aCapAt - 1e-6f)
        {
            vt::SetHoldCap(aCapSec, ticks);
            capSent = true;
        }
        if (!cap2Sent && tj >= aCap2At - 1e-6f)
        {
            vt::SetHoldCap(aCap2Sec, ticks);
            cap2Sent = true;
        }
        while (float(nextFeed) / float(feedHz) <= tj + 1e-5f)
        {
            const float tf = float(nextFeed) / float(feedHz);
            const float pos[3] = {InterpT(r.t, r.n, r.xy, 2, 0, tf), InterpT(r.t, r.n, r.xy, 2, 1, tf), 0.0f};
            vt::UpdateWorld(pos, 0.0f, t0 + int64_t(double(tf) * double(kTps)));
            feedSample(tf);
            ++nextFeed;
        }
        Records rec = game;
        vt::FkSide(rec.data(), kGameCount, kRig, ticks, false);
        float ro[3] = {};
        const bool on = vt::RootOffset(ticks, ro);
        const float ex = InterpT(r.t, r.n, r.xy, 2, 0, tj), ey = InterpT(r.t, r.n, r.xy, 2, 1, tj);
        const float vx = ex + (on ? ro[0] : 0.0f), vy = ey + (on ? ro[1] : 0.0f);
        for (float v : {on ? 1.0f : 0.0f, ro[0], ro[1], ro[2]})
        {
            uint32_t bits = 0;
            std::memcpy(&bits, &v, sizeof bits);
            for (int b = 0; b < 4; ++b)
                run.hash = (run.hash ^ ((bits >> (8 * b)) & 0xffu)) * 0x100000001b3ull;
        }
        // the rendered bones (every record FkSide left, the Hips with the root offset applied) are part of the identity hash
        for (size_t i = 0; i < rec.size(); ++i)
            run.hash = (run.hash ^ rec[i]) * 0x100000001b3ull;
        run.rox.push_back(on ? ro[0] : 0.0f); run.roy.push_back(on ? ro[1] : 0.0f);
        run.hx.push_back(Rec(rec, Hips)[0]); run.hy.push_back(Rec(rec, Hips)[1]);
        run.t.push_back(tj); run.vx.push_back(vx); run.vy.push_back(vy); run.ex.push_back(ex); run.ey.push_back(ey);
        if (j > 0)
        {
            run.offsetPop = std::fmax(run.offsetPop, std::hypot((vx - ex) - prevOx, (vy - ey) - prevOy));
            const float step = std::hypot(vx - prevVx, vy - prevVy);
            run.peakSpeed = std::fmax(run.peakSpeed, step * float(animHz));
            run.entPeakSpeed = std::fmax(run.entPeakSpeed, std::hypot(ex - prevEx, ey - prevEy) * float(animHz));
            run.travel += step;
        }
        prevVx = vx; prevVy = vy; prevOx = vx - ex; prevOy = vy - ey; prevEx = ex; prevEy = ey;
    }
    run.capState = vt::g_capState.load();
    run.capExtra = vt::g_capSecondIgnored.load();
    run.capStart = vt::g_capStartAt.load();
    run.capTail = vt::g_capTailUsed.load();
    run.engageAt = vt::g_rootEngageAt.load();
    run.peakDist = vt::g_rootPeakDistM.load();
    run.jumpAt = vt::g_jumpEngageAt.load();
    run.jumpPeak = vt::g_jumpPeakDistM.load();
    run.jumpEnd = vt::g_jumpReleaseEnd.load();
    run.epochEnded = !vt::MountActive();
    vt::Abort();
    return run;
}
// Glide statistics of a replay: what the player sees of the root hold, measured at the FEED instants (the published offset is constant
// between two feeds, so a 100 Hz render of a 50 Hz feed would add the entity's own slide to every in-between frame and measure the raw
// slide, not the hold). Speeds are planar world speeds of the visible root between consecutive feed instants, while the hold runs.
// The first kGlideGateFrom seconds are reported apart: before the first re-base / the cumulative engage (0.15 m of entity change) the visible
// root IS the entity (up to two feed intervals, <= 0.2 m).
constexpr float kGlideGateFrom = 0.06f;
struct GlideStats
{
    float peakVis = 0.0f, peakVisAt = 0.0f; // fastest visible root from kGlideGateFrom on (m/s) and when
    float peakEarly = 0.0f;                 // fastest visible root before kGlideGateFrom (the pre-engage frames)
    float entAtPeak = 0.0f;                 // the entity's own speed over the feed interval of the peak (clip walk / vanilla slide)
    float shrink = 0.0f;                    // fastest closing of |visible - entity| over >= 0.1 s of feed instants (m/s): the release speed
    float endAt = 0.0f;                     // last time the visible root differs from the entity by > 1 cm
    float peakOff = 0.0f;                   // largest |visible - entity| (m)
    float endStep = 0.0f;                   // visible root step (m) over the last feed interval before the offset ends, against the entity's step (continuity)
    float endSkew = 0.0f;                   // |visible step - entity step| (m) over the feed interval in which the offset ends
};
GlideStats Glide(const DoorRun& r)
{
    GlideStats g;
    for (size_t i = 0; i < r.t.size(); ++i)
    {
        const float d = std::hypot(r.vx[i] - r.ex[i], r.vy[i] - r.ey[i]);
        g.peakOff = std::fmax(g.peakOff, d);
        if (d > 0.01f)
            g.endAt = r.t[i];
    }
    const size_t n = r.ft.size();
    auto off = [&](size_t k) { return std::hypot(r.fvx[k] - r.fex[k], r.fvy[k] - r.fey[k]); };
    for (size_t k = 1; k < n; ++k)
    {
        if (r.ft[k - 1] > g.endAt)
            break;
        const float dt = r.ft[k] - r.ft[k - 1];
        const float vv = std::hypot(r.fvx[k] - r.fvx[k - 1], r.fvy[k] - r.fvy[k - 1]) / dt;
        const float ev = std::hypot(r.fex[k] - r.fex[k - 1], r.fey[k] - r.fey[k - 1]) / dt;
        if (r.ft[k] < kGlideGateFrom)
            g.peakEarly = std::fmax(g.peakEarly, vv);
        else if (vv > g.peakVis)
        {
            g.peakVis = vv; g.peakVisAt = r.ft[k]; g.entAtPeak = ev;
        }
        g.endSkew = std::hypot((r.fvx[k] - r.fvx[k - 1]) - (r.fex[k] - r.fex[k - 1]), (r.fvy[k] - r.fvy[k - 1]) - (r.fey[k] - r.fey[k - 1]));
        g.endStep = std::hypot(r.fvx[k] - r.fvx[k - 1], r.fvy[k] - r.fvy[k - 1]);
        for (size_t m = k + 1; m < n && r.ft[m] - r.ft[k - 1] < 0.35f; ++m)
            if (r.ft[m] - r.ft[k - 1] >= 0.10f)
            {
                g.shrink = std::fmax(g.shrink, (off(k - 1) - off(m)) / (r.ft[m] - r.ft[k - 1]));
                break;
            }
    }
    return g;
}
// smallest distance of the visible root (aVisible) or the entity root to the door panel over the opening window, and the time of it
float MinDoorDist(const DoorRideFx& r, const ArcFx& arc, const DoorRun& run, bool aVisible, float* aAt = nullptr)
{
    float best = 99.0f, at = 0.0f;
    for (size_t i = 0; i < run.t.size(); ++i)
    {
        if (run.t[i] < arc.delay || run.t[i] > 1.0f)
            continue;
        const float d = DoorDist(arc, DoorAngleAt(r, run.t[i]), aVisible ? run.vx[i] : run.ex[i], aVisible ? run.vy[i] : run.ey[i]);
        if (d < best) { best = d; at = run.t[i]; }
    }
    if (aAt) *aAt = at;
    return best;
}
// the same against the relative criterion: visible root >= min(0.20, entity root's own distance - 0.02) at every frame; returns the worst margin
float WorstRelative(const DoorRideFx& r, const ArcFx& arc, const DoorRun& run)
{
    float worst = 99.0f;
    for (size_t i = 0; i < run.t.size(); ++i)
    {
        if (run.t[i] < arc.delay || run.t[i] > 1.0f)
            continue;
        const float a = DoorAngleAt(r, run.t[i]);
        const float dv = DoorDist(arc, a, run.vx[i], run.vy[i]), de = DoorDist(arc, a, run.ex[i], run.ey[i]);
        worst = std::fmin(worst, dv - (std::fmin(0.20f, de) - 0.02f));
    }
    return worst;
}
const DoorRideFx* FindRide(int aRide)
{
    for (const DoorRideFx& r : kDoorRides)
        if (r.ride == aRide)
            return &r;
    return nullptr;
}
bool SameSeries(const DoorRun& a, const DoorRun& b, float aUntil)
{
    for (size_t i = 0; i < a.t.size() && i < b.t.size(); ++i)
    {
        if (a.t[i] >= aUntil)
            break;
        if (a.vx[i] != b.vx[i] || a.vy[i] != b.vy[i])
            return false;
    }
    return true;
}

// The rendered root goes through ApplyIfActive, not RootOffset (which has no runtime caller): the capped-vs-base difference of the
// rendered Hips xy must equal the RootOffset difference. Returns the largest deviation; aSwing = the largest rendered difference.
float RenderedVsOffset(const DoorRun& a, const DoorRun& b, float* aSwing = nullptr)
{
    float worst = 0.0f, swing = 0.0f;
    for (size_t i = 0; i < a.t.size() && i < b.t.size(); ++i)
    {
        const float dhx = a.hx[i] - b.hx[i], dhy = a.hy[i] - b.hy[i];
        worst = std::fmax(worst, std::fmax(std::fabs(dhx - (a.rox[i] - b.rox[i])), std::fabs(dhy - (a.roy[i] - b.roy[i]))));
        swing = std::fmax(swing, std::hypot(dhx, dhy));
    }
    if (aSwing) *aSwing = swing;
    return worst;
}

// Speed at which the visible root moves AGAINST the entity (the camera follows the entity: utpp_camera pivot = player:GetWorldPosition()):
// the fastest SHRINK of |visible - entity| over aWindow seconds, divided by aWindow (m/s) = how fast the held body closes on the entity. aWindow 1/30 = one frame of the 30 fps recording.
float OffsetRate(const DoorRun& r, float aWindow)
{
    float best = 0.0f;
    auto offAt = [&](float aT, float& ox, float& oy) {
        const float f = aT * 100.0f;
        int i = int(f);
        if (i < 0) i = 0;
        if (i >= int(r.t.size()) - 1) i = int(r.t.size()) - 2;
        const float w = f - float(i);
        ox = (r.vx[i] - r.ex[i]) * (1.0f - w) + (r.vx[i + 1] - r.ex[i + 1]) * w;
        oy = (r.vy[i] - r.ey[i]) * (1.0f - w) + (r.vy[i + 1] - r.ey[i + 1]) * w;
    };
    for (float t = 0.0f; t + aWindow <= 1.15f; t += 0.005f)
    {
        float ax, ay, bx, by;
        offAt(t, ax, ay);
        offAt(t + aWindow, bx, by);
        best = std::fmax(best, (std::hypot(ax, ay) - std::hypot(bx, by)) / aWindow); // only the SHRINKING of the offset (the release)
    }
    return best;
}

// Jump path of the hold cap with synthetic entity samples (30 Hz feed, the entity jumps aJump metres along x at the first sample, then
// yaw-only jump samples). Reads the real RootOffset (the cap's planar factor) and the cap diagnostics.
struct JumpCapRun
{
    int state = 0;
    float releaseEnd = -1.0f, tail = 0.0f, shownAt027 = 0.0f, maxGrow = 0.0f, maxStep = 0.0f, firstMag = 0.0f;
};
JumpCapRun RunJumpCap(float aJump, float aCapSec, const std::vector<float>& aYawSamplesDeg)
{
    SetRoles();
    const float p0[3] = {0.0f, 0.0f, 0.0f};
    const int64_t t0 = NextEpochTime();
    if (aCapSec >= 0.0f)
        vt::SetHoldCap(aCapSec, t0 - kTps / 100);
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0, true);
    vt::SetMountTpp(true);
    JumpCapRun run;
    float prev = 0.0f;
    for (size_t k = 0; k < aYawSamplesDeg.size() + 1; ++k)
    {
        const float tf = float(k + 1) / 30.0f;
        const int64_t tick = t0 + int64_t(double(tf) * double(kTps));
        float before[3] = {};
        if (!vt::RootOffset(tick, before)) { before[0] = before[1] = 0.0f; }
        const float pos[3] = {aJump, 0.0f, 0.0f};
        vt::UpdateWorld(pos, k == 0 ? 0.0f : aYawSamplesDeg[k - 1], tick);
        float after[3] = {};
        if (!vt::RootOffset(tick, after)) { after[0] = after[1] = 0.0f; }
        if (k == 0)
            run.firstMag = std::hypot(after[0], after[1]);
        else // a later jump sample rebases the offset onto the visible root: the planar length must not step
            run.maxStep = std::fmax(run.maxStep, std::fabs(std::hypot(after[0], after[1]) - std::hypot(before[0], before[1])));
    }
    for (int j = 0; j <= 60; ++j)
    {
        const float tj = float(j) / 100.0f;
        float ro[3] = {};
        if (!vt::RootOffset(t0 + int64_t(double(tj) * double(kTps)), ro)) { ro[0] = ro[1] = 0.0f; }
        const float m = std::hypot(ro[0], ro[1]);
        if (j > 10) // after the last feed (0.1 s)
            run.maxGrow = std::fmax(run.maxGrow, m - prev);
        prev = m;
        if (j == 27)
            run.shownAt027 = m;
    }
    run.state = vt::g_capState.load();
    run.releaseEnd = vt::g_jumpReleaseEnd.load();
    run.tail = vt::g_capTailUsed.load();
    vt::Abort();
    return run;
}

void TestDoorCap()
{
    // group E was written for the hold before the glide limit (the cap acts on that hold, 'no cap' is its reference): replayed with the limit off
    // (group F below runs the same fixtures with it on and shows that every ride with a cap keeps this hold bit-identically)
    vt::g_glideOff.store(true);
    Check('E', sizeof(kDoorRides) / sizeof(kDoorRides[0]) == 34 && sizeof(kDoorArcs) / sizeof(kDoorArcs[0]) >= 10, "E0 fixture: 30 rides + the car-door3 rides 109 / 129 / 131 / 116, the generated arcs");
    std::set<int> applied;
    std::vector<int> jumpRides;
    std::printf("  E1 hold cap per ride (cap = what scripts/utpp_vehicle_door.reds computes; why 0 = cap sent): ride seat why anchor r theta tEdge cap -> state start tail | path | min door dist uncapped -> capped | same\n");
    for (const DoorRideFx& r : kDoorRides)
    {
        const ArcFx arc = ArcOfSeat(r.seat);
        const CapInfo ci = RedsCap(arc, r.xy[0], r.xy[1]);
        const DoorRun base = RunDoorRide(r, true, -1.0f, 0.0f);
        const DoorRun cap = RunDoorRide(r, true, ci.cap, 0.0f);
        const bool fired = cap.capState == 2 || cap.capState == 3;
        if (fired) applied.insert(r.ride);
        const bool jump = base.jumpAt >= 0.0f;
        if (jump) jumpRides.push_back(r.ride);
        const bool same = cap.hash == base.hash;
        float mb = 0, mc = 0;
        if (arc.valid) { mb = MinDoorDist(r, arc, base, true); mc = MinDoorDist(r, arc, cap, true); }
        std::printf("    %2d %-12s %-5.5s why %d (%.2f,%.2f) r %.2f th %5.1f tEdge %5.3f cap %6.3f -> state %d start %.3f tail %.3f | %s peak %.2f | %.3f -> %.3f | %s\n",
                    r.ride, r.vehicle, r.seat, ci.why, double(r.xy[0]), double(r.xy[1]), double(ci.r), double(ci.theta), double(ci.tEdge), double(ci.cap),
                    cap.capState, double(cap.capStart), double(cap.capTail), jump ? "jump" : (base.engageAt >= 0.0f ? "hold" : "none"),
                    double(jump ? base.jumpPeak : base.peakDist), double(mb), double(mc), same ? "identical" : "CHANGED");
        if (r.ride != 4 && r.ride != 9 && r.ride != 25 && r.ride != 30 && r.ride != 109 && r.ride != 129 && r.ride != 131 && r.ride != 116)
            Check('E', same && !fired, "E1 ride " + std::to_string(r.ride) + ": the cap does not act and the output is bit-identical to no cap (rendered bones included)");
        float swing = 0.0f;
        const float dev = RenderedVsOffset(cap, base, &swing);
        Check('E', dev < 1e-4f, "E1 ride " + std::to_string(r.ride) + ": rendered Hips difference equals the RootOffset difference (" + std::to_string(dev) + ")");
        if (fired)
            Check('E', swing > 0.01f, "E1 ride " + std::to_string(r.ride) + ": the cap changes the RENDERED Hips (" + std::to_string(swing) + " m)");
    }
    Check('E', applied == std::set<int>({4, 9, 25, 30, 109, 129, 131, 116}), "E1 the cap acts on exactly rides {4, 9, 25, 30, 109, 129, 131, 116}");

    // E2: clearance of the visible root from the door panel over the opening, rides 4, 9 and 25 absolute, ride 30 relative to its own entity
    for (int id : {4, 9, 25, 30, 109, 129, 131, 116})
    {
        const DoorRideFx& r = *FindRide(id);
        const ArcFx arc = ArcOfSeat(r.seat);
        const CapInfo ci = RedsCap(arc, r.xy[0], r.xy[1]);
        const DoorRun base = RunDoorRide(r, true, -1.0f, 0.0f);
        const DoorRun cap = RunDoorRide(r, true, ci.cap, 0.0f);
        float atB = 0, atC = 0;
        const float mb = MinDoorDist(r, arc, base, true, &atB), mc = MinDoorDist(r, arc, cap, true, &atC), me = MinDoorDist(r, arc, cap, false);
        const float relB = WorstRelative(r, arc, base), relC = WorstRelative(r, arc, cap);
        std::printf("  E2 ride %d: visible root to door panel min %.3f m at %.2f s uncapped, %.3f m at %.2f s capped (entity root alone %.3f m); relative margin %.3f -> %.3f; "
                    "visible peak speed %.1f -> %.1f m/s (entity slide %.1f m/s), path %.2f -> %.2f m; offset-vs-entity rate (1/30 s) %.1f -> %.1f m/s\n",
                    id, double(mb), double(atB), double(mc), double(atC), double(me), double(relB), double(relC), double(base.peakSpeed),
                    double(cap.peakSpeed), double(cap.entPeakSpeed), double(base.travel), double(cap.travel), double(OffsetRate(base, 1.0f / 30.0f)), double(OffsetRate(cap, 1.0f / 30.0f)));
        // rides 30 and 129 (bratsk door): the clip start sits on the arc rim, judged against the entity's own distance. Ride 116 (emperor passenger,
        // recorded clean in game): the uncapped hold only comes within 0.19 m of the panel, so it has no 'into the door' check
        const bool relRide = id == 30 || id == 129;
        if (!relRide)
        {
            if (id != 116)
                Check('E', mb < 0.10f, "E2 ride " + std::to_string(id) + ": the uncapped hold carries the visible root into the door (" + std::to_string(mb) + " m)");
            Check('E', mc >= 0.20f, "E2 ride " + std::to_string(id) + ": capped visible root stays >= 0.20 m from the door panel (" + std::to_string(mc) + " m)");
        }
        else
        {
            // ride 30's clip start sits on the arc rim (decision.md 4.2: the entity root alone grazes the panel), so the hold is judged
            // against the entity root's own distance: the visible root is never closer than min(0.20, entity distance) - 0.02
            Check('E', relB < -0.05f, "E2 ride " + std::to_string(id) + ": the uncapped hold intrudes beyond the entity root's own path (" + std::to_string(relB) + ")");
            Check('E', relC >= 0.0f, "E2 ride " + std::to_string(id) + ": capped visible root never closer than min(0.20, entity distance) - 0.02 (" + std::to_string(relC) + ")");
        }
        Check('E', RenderedVsOffset(cap, base) < 1e-4f, "E2 ride " + std::to_string(id) + ": rendered Hips difference equals the RootOffset difference");
        if (id != 25)
        {
            // the camera follows the ENTITY (utpp_camera pivot = player:GetWorldPosition()), so the pop the player sees is how fast the held body
            // closes on the entity (offset shrink rate) and how fast the body itself moves compared to the vanilla slide. Columbus ride 109 (game
            // 2026-11-02 11:16:39, a one-frame jump of V and the framing): tracking the anchor through the release (mutation 25 = the code
            // before the fix) gave 4.5 m/s and 1.6x the slide there; the frozen release offset gives 3.0 m/s and 1.2x. Replay of all rides capped in
            // that run, before -> after: shrink 109 4.5 -> 3.0, 129 (bratsk door) 3.7 -> 2.0, 131 (bratsk passenger) 3.8 -> 2.3, 116 (emperor
            // passenger) 5.9 -> 2.5 m/s; hella ride 4 6.5 -> 3.5, ride 9 5.3 -> 3.1. NOTE: the metric is a model of what the player sees, not proof:
            // the emperor passenger ride (5.9 m/s before) was rated clean in game, so the threshold only keeps every capped ride at or below the
            // values of the fixed code with a margin; it does not separate 'pops' from 'does not pop' by itself.
            const float rate = OffsetRate(cap, 1.0f / 30.0f);
            Check('E', rate < 4.0f, "E2 ride " + std::to_string(id) + ": the held body closes on the entity at less than 4 m/s over one 30 fps frame (" + std::to_string(rate) + " m/s)");
            Check('E', cap.peakSpeed < 1.3f * cap.entPeakSpeed, "E2 ride " + std::to_string(id) + ": the capped body moves at most 1.3x the vanilla slide speed (" + std::to_string(cap.peakSpeed) + " vs " + std::to_string(cap.entPeakSpeed) + " m/s)");
        }
        if (id != 25)
            Check('E', cap.peakSpeed < 14.0f, "E2 ride " + std::to_string(id) + ": the released root does not skate faster than the vanilla slide itself (" + std::to_string(cap.peakSpeed) + " m/s)");
        else
            // ride 25 is a jump-path ride: the 1.2 m jump offset is released over the 0.15 s minimum tail, so the visible root briefly
            // outruns the entity's own slide (16.3 vs 13.9 m/s, uncapped 13.3) - the accepted price of 0.21 m door clearance instead of
            // 0.09 m. Bounded so a longer jump or a shorter tail cannot make it worse unnoticed.
            Check('E', cap.peakSpeed < 1.2f * cap.entPeakSpeed, "E2 ride 25: the jump-path release stays within 1.2x the vanilla slide speed (" + std::to_string(cap.peakSpeed) + " vs " + std::to_string(cap.entPeakSpeed) + " m/s)");
    }

    // E3: arrival order. before BeginMount, with it, and up to 0.1 s after it: applied on the next sample, never retroactively
    for (int id : {4, 9, 30, 109, 129, 131, 116})
    {
        const DoorRideFx& r = *FindRide(id);
        const ArcFx arc = ArcOfSeat(r.seat);
        const CapInfo ci = RedsCap(arc, r.xy[0], r.xy[1]);
        const DoorRun base = RunDoorRide(r, true, -1.0f, 0.0f);
        for (float at : {-0.05f, 0.0f, 0.05f, 0.10f})
        {
            const DoorRun c = RunDoorRide(r, true, ci.cap, at);
            const bool same = SameSeries(c, base, c.capStart - 1e-4f);
            std::printf("  E3 ride %d cap %.3f arrives %+.2f s: state %d start %.3f tail %.3f, series identical to no cap before the start %d, largest offset step %.3f m (no cap %.3f)\n",
                        id, double(ci.cap), double(at), c.capState, double(c.capStart), double(c.capTail), same ? 1 : 0, double(c.offsetPop), double(base.offsetPop));
            Check('E', c.capState == 2, "E3 ride " + std::to_string(id) + ": cap arriving at " + std::to_string(at) + " s is applied");
            Check('E', at < 0.0f || c.capStart >= at - 1e-4f, "E3 ride " + std::to_string(id) + ": a cap arriving at " + std::to_string(at) + " s starts after its arrival (no retroactive release)");
            Check('E', same, "E3 ride " + std::to_string(id) + ": before the release starts the shown root is bit-identical to no cap (arrival " + std::to_string(at) + " s)");
            Check('E', RenderedVsOffset(c, base) < 1e-4f, "E3 ride " + std::to_string(id) + ": rendered Hips difference equals the RootOffset difference (arrival " + std::to_string(at) + " s)");
            Check('E', c.offsetPop <= base.offsetPop + 0.02f, "E3 ride " + std::to_string(id) + ": no pop - largest per-frame change of the root offset " + std::to_string(c.offsetPop) + " vs no cap " + std::to_string(base.offsetPop));
        }
    }

    // E3b: a second cap for the same mount (a doubled MountingEvent) is ignored - a new curve would take the released root back to the anchor
    {
        const DoorRideFx& r = *FindRide(4);
        const ArcFx arc = ArcOfSeat(r.seat);
        const CapInfo ci = RedsCap(arc, r.xy[0], r.xy[1]);
        const DoorRun base = RunDoorRide(r, true, -1.0f, 0.0f);
        const DoorRun one = RunDoorRide(r, true, ci.cap, 0.0f);
        const DoorRun two = RunDoorRide(r, true, ci.cap, 0.0f, ci.cap + 0.10f, 0.15f);
        std::printf("  E3b ride 4: second cap at +0.15 s: ignored %d, state %d, largest offset step %.3f m (one cap %.3f, no cap %.3f), series identical to one cap %d\n",
                    two.capExtra, two.capState, double(two.offsetPop), double(one.offsetPop), double(base.offsetPop), two.hash == one.hash ? 1 : 0);
        Check('E', two.capExtra == 1 && two.capState == 2, "E3b a second cap for an already capped mount is counted and ignored");
        Check('E', two.offsetPop <= base.offsetPop + 0.02f && two.hash == one.hash, "E3b the second cap does not pop (and changes nothing)");
    }

    // E4: too late (0.30 s after BeginMount) and stale (left over for the next mount) caps change nothing; a negative cap is no cap
    {
        const DoorRideFx& r = *FindRide(4);
        const ArcFx arc = ArcOfSeat(r.seat);
        const CapInfo ci = RedsCap(arc, r.xy[0], r.xy[1]);
        const DoorRun base = RunDoorRide(r, true, -1.0f, 0.0f);
        const DoorRun late = RunDoorRide(r, true, ci.cap, 0.30f);
        Check('E', late.hash == base.hash && late.capState == 8, "E4 a cap 0.30 s into a running mount is not applied and recorded as late on this ride (state " + std::to_string(late.capState) + ")");
        const DoorRun next = RunDoorRide(r, true, -1.0f, 0.0f); // the late cap is NOT kept for the next mount
        Check('E', next.hash == base.hash && next.capState == 0, "E4 a late cap is not carried over to the next mount (state " + std::to_string(next.capState) + ")");
        vt::SetHoldCap(ci.cap, g_clock + 1); // no mount running (RunDoorRide aborts its mount): pending ...
        const DoorRun stale = RunDoorRide(r, true, -1.0f, 0.0f); // ... and this BeginMount is 10 s later
        Check('E', stale.hash == base.hash && stale.capState == 6, "E4 a stale pending cap is dropped by the next BeginMount (state " + std::to_string(stale.capState) + ")");
        vt::SetHoldCap(ci.cap, g_clock + 5);  // pending for the next mount ...
        vt::SetHoldCap(-1.0f, g_clock + 6);   // ... and withdrawn (< 0 = no cap)
        const DoorRun after = RunDoorRide(r, true, -1.0f, 0.0f);
        Check('E', after.hash == base.hash && after.capState == 0, "E4 a withdrawn cap (< 0) leaves nothing pending (state " + std::to_string(after.capState) + ")");
    }

    // E5: bikes keep the validated hold bit-identically: the c9 bike path (group D golden hash), and ride 4's path as a bike
    {
        const AlignRun c9 = RunAlign(kBikeC9, 1.004f, 120, 60, false, 0.2f);
        Check('E', c9.hash == 0x6ab0b42e10ddd37dull && vt::g_capState.load() == 5, "E5 c9 bike with a cap: series bit-identical to group D5's golden (state " + std::to_string(vt::g_capState.load()) + " = refused)");
        const DoorRideFx& r = *FindRide(4);
        const ArcFx arc = ArcOfSeat(r.seat);
        const CapInfo ci = RedsCap(arc, r.xy[0], r.xy[1]);
        const DoorRun bikeBase = RunDoorRide(r, false, -1.0f, 0.0f);
        const DoorRun bike = RunDoorRide(r, false, ci.cap, 0.0f);
        Check('E', bike.hash == bikeBase.hash && bike.capState == 5, "E5 ride 4's path as a bike with a cap: bit-identical, cap refused (state " + std::to_string(bike.capState) + ")");
    }
    // E6: the jump path decides once per mount. A first jump sample 0.65 m away with a cap of 0.22 s is capped (state 3); the next
    // jump samples (yaw only, inside the watch window) rebase the offset onto the capped visible root (0.65 m * 0.92 = 0.60 m, below the
    // magnitude gate) and must neither drop the cap nor restart the natural 0.3 s decay: the release still ends by max(cap, sample + 0.15 s).
    {
        const JumpCapRun none = RunJumpCap(0.65f, -1.0f, {20.0f, 40.0f});
        const JumpCapRun two = RunJumpCap(0.65f, 0.22f, {20.0f});
        const JumpCapRun three = RunJumpCap(0.65f, 0.22f, {20.0f, 40.0f});
        std::printf("  E6 jump cap, 0.65 m, cap 0.22 s: state %d end %.3f tail %.3f shown@0.27 %.4f (2 samples) | state %d end %.3f tail %.3f shown@0.27 %.4f step %.5f grow %.5f (3 samples) | no cap shown@0.27 %.3f\n",
                    two.state, double(two.releaseEnd), double(two.tail), double(two.shownAt027), three.state, double(three.releaseEnd), double(three.tail),
                    double(three.shownAt027), double(three.maxStep), double(three.maxGrow), double(none.shownAt027));
        Check('E', two.firstMag > 0.6f && two.state == 3, "E6 the first jump sample applies the cap on the jump path (state " + std::to_string(two.state) + ")");
        Check('E', std::fabs(two.releaseEnd - 0.22f) < 3e-3f, "E6 one yaw-only jump sample later the release still ends by the cap (" + std::to_string(two.releaseEnd) + ")");
        Check('E', three.state == 3, "E6 the cap survives two later jump samples (state " + std::to_string(three.state) + ")");
        Check('E', std::fabs(three.releaseEnd - 0.25f) < 3e-3f, "E6 the release ends by max(cap, last sample + min release) = 0.25 s (" + std::to_string(three.releaseEnd) + ")");
        Check('E', two.shownAt027 < 1e-4f && three.shownAt027 < 1e-4f, "E6 the capped root is released by 0.27 s; no cap leaves " + std::to_string(none.shownAt027) + " m");
        Check('E', none.shownAt027 > 0.005f, "E6 control: without the cap the natural decay still shows an offset at 0.27 s");
        Check('E', three.maxStep < 1e-4f && three.maxGrow < 1e-4f, "E6 no pop at the rebase and no regrowth after it");
        // a first sample below the magnitude gate is not capped (state 1, natural 0.3 s decay from the sample)
        const JumpCapRun small = RunJumpCap(0.45f, 0.22f, {});
        Check('E', small.state == 1 && std::fabs(small.releaseEnd - (1.0f / 30.0f + vt::kRootSeconds)) < 3e-3f,
              "E6 a first jump sample of 0.45 m is not capped (state " + std::to_string(small.state) + ", end " + std::to_string(small.releaseEnd) + ")");
    }
    std::printf("  E1 jump-path rides (first step > 0.15 m inside the watch window): ");
    for (int id : jumpRides) std::printf("%d ", id);
    std::printf("\n");
    vt::g_glideOff.store(false);
}

// ------------------------------------------------------------------------------------------------ (F) car glide limit
// Report 2026-11-03 (car entries from the driver side: V slides toward the rear and through the side of the car). The vanilla workspot slides the
// entity to the clip start (1.4-2.2 m behind the pre-mount spot of a driver) with a sin ease over ~0.30 s: 7-11 m/s. Replay of the same 34 logged rides as
// group E without a door cap (the parallel / behind / passenger approaches the door script sends no cap for), the visible root measured at the FEED
// instants (see GlideStats) while the hold runs, with the glide limit off (the hold before it) and on:
//   F1 the visible root never outruns kGlideVisibleMax (or the clip's own walk) from kGlideGateFrom on (the old hold: 6-16 m/s on the jump path);
//   F2 the held offset closes on the entity at no more than 1.15 x kGlideSpeedMps (offsets the time budget binds: kGlideBudgetMax, the 2.2 m Outlaw slide);
//   F3 the release is smooth (C1): the offset never grows once the tracking is over, its last 5 cm / 1 cm close in steps of <= 12 / 5 mm per 1/100 s, and
//      it is 0 and the epoch over kGlideEndSeconds after the mount;
//   F4 a mount WITH a door cap (every ride the door script sends a cap for) and a bike replay the pre-glide hold bit-identically.
// Mutation 26 (no glide limit) -> F1 / F2.
constexpr float kGlideVisibleMax = 2.75f; // m/s: the visible root's speed from kGlideGateFrom on, unless that is the clip's own walk (<= 1.1 x the entity's speed there: 2.3-2.8 m/s)
constexpr float kGlideBudgetMax = 2.90f;  // m/s: the closing speed when the time budget (offset 0 by kGlideEndSeconds) binds - the 2.2 m Outlaw slide (1.5 x 2.25 m / 1.15 s = 2.8)
void CheckGlideRide(const DoorRideFx& r, const char* aLabel)
{
    vt::g_glideOff.store(true);
    const DoorRun old = RunDoorRide(r, true, -1.0f, 0.0f);
    vt::g_glideOff.store(false);
    const DoorRun now = RunDoorRide(r, true, -1.0f, 0.0f);
    const GlideStats go = Glide(old), gn = Glide(now);
    std::printf("  F ride %3d %-9s %-4s visible peak %5.2f -> %5.2f m/s (first %.2f s: %5.2f -> %5.2f), release closes %5.2f -> %5.2f m/s, peak offset %.2f -> %.2f m, offset end %.2f -> %.2f s\n",
                r.ride, r.vehicle, aLabel, double(go.peakVis), double(gn.peakVis), double(kGlideGateFrom), double(go.peakEarly), double(gn.peakEarly), double(go.shrink),
                double(gn.shrink), double(go.peakOff), double(gn.peakOff), double(go.endAt), double(gn.endAt));
    const std::string id = "F ride " + std::to_string(r.ride) + " (" + aLabel + "): ";
    Check('F', gn.peakVis <= kGlideVisibleMax || gn.peakVis <= 1.1f * gn.entAtPeak,
          id + "the visible root never outruns " + std::to_string(kGlideVisibleMax) + " m/s (or the clip's own walk) while the hold runs (" + std::to_string(gn.peakVis) + ", entity " + std::to_string(gn.entAtPeak) + ")");
    const float limit = gn.peakOff <= 1.3f ? 1.15f * vt::kGlideSpeedMps : kGlideBudgetMax;
    Check('F', gn.shrink <= limit, id + "the held offset closes on the entity at <= " + std::to_string(limit) + " m/s (" + std::to_string(gn.shrink) + ")");
    // F3: smooth release. From 0.5 s on (the tracking / follow windows are over: the offset is a pure decay) the offset (distance visible root - entity)
    // never grows (1 mm per frame tolerance), the last 5 cm close in steps of <= 1.2 cm per 1/100 s and the last 1 cm in steps of <= 5 mm (the smoothstep's
    // speed has fallen to ~1 m/s and then to 0 at the end: C1, no snap) ...
    float grow = 0.0f, step5 = 0.0f, step1 = 0.0f, prevD = 1e9f;
    for (size_t i = 0; i < now.t.size(); ++i)
    {
        if (now.t[i] < 0.5f)
            continue;
        const float d = std::hypot(now.vx[i] - now.ex[i], now.vy[i] - now.ey[i]);
        if (prevD < 1e8f)
        {
            grow = std::fmax(grow, d - prevD);
            if (prevD < 0.05f)
                step5 = std::fmax(step5, std::fabs(d - prevD));
            if (prevD < 0.01f)
                step1 = std::fmax(step1, std::fabs(d - prevD));
        }
        prevD = d;
    }
    std::printf("    smooth release: growth %.4f m, last 5 cm step %.4f m, last 1 cm step %.4f m per 1/100 s\n", double(grow), double(step5), double(step1));
    Check('F', grow <= 0.001f, id + "the offset never grows once the tracking is over (" + std::to_string(grow) + " m)");
    Check('F', step5 <= 0.012f && step1 <= 0.005f, id + "no snap at the end of the release: last 5 cm in steps of " + std::to_string(step5) + " m, last 1 cm " + std::to_string(step1) + " m per 1/100 s");
    float endOff = 0.0f;
    for (size_t i = 0; i < now.t.size(); ++i)
        if (now.t[i] >= vt::kGlideEndSeconds + 0.03f)
            endOff = std::fmax(endOff, std::hypot(now.vx[i] - now.ex[i], now.vy[i] - now.ey[i]));
    Check('F', endOff < 1e-4f, id + "the final offset is 0 from kGlideEndSeconds on (" + std::to_string(endOff) + " m)");
    Check('F', now.epochEnded, id + "the mount transition has expired by the end of the replay");
}


// Synthetic slide with a pivot: the entity slides aDist m along -y and turns aYawDeg over 0.30 s (sin ease, the vanilla workspot), 50 Hz feed, 100 Hz render.
// Returns the largest change of the SHOWN yaw (entity yaw + yaw offset) per 1/100 s in deg/s, and the last time the yaw offset is above 1 deg.
struct YawRun { float maxRate = 0.0f, endAt = 0.0f, planarEnd = 0.0f, peakOff = 0.0f, peakVis = 0.0f; };
YawRun RunSlideYaw(float aDist, float aYawDeg, bool aGlideOff)
{
    vt::g_glideOff.store(aGlideOff);
    SetRoles();
    Pose g = GamePoseB();
    g.p[Hips] = {0.0f, 0.0f, 0.973f};
    const Records game = MakeRecords(g);
    const float p0[3] = {-1.85f, 0.5f, 0.0f};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0, true);
    vt::SetMountTpp(true);
    YawRun run;
    auto ease = [](float t) { return std::sin(1.5707963f * std::fmin(std::fmax(t / 0.30f, 0.0f), 1.0f)); };
    int nextFeed = 1;
    float prevShown = 0.0f, pvx = p0[0], pvy = p0[1];
    for (int j = 0; j <= 160; ++j)
    {
        const float tj = float(j) / 100.0f;
        const int64_t ticks = t0 + int64_t(double(tj) * double(kTps));
        while (float(nextFeed) / 50.0f <= tj + 1e-5f)
        {
            const float tf = float(nextFeed) / 50.0f;
            const float pos[3] = {p0[0], p0[1] - aDist * ease(tf), 0.0f};
            vt::UpdateWorld(pos, aYawDeg * ease(tf), t0 + int64_t(double(tf) * double(kTps)));
            {   // the visible root (entity + the character-frame offset turned by the entity's yaw) at the feed instant
                float fo[3] = {};
                const bool fon = vt::RootOffset(t0 + int64_t(double(tf) * double(kTps)), fo);
                const float a = aYawDeg * ease(tf) * kDeg;
                const float vx = pos[0] + (fon ? std::cos(a) * fo[0] - std::sin(a) * fo[1] : 0.0f), vy = pos[1] + (fon ? std::sin(a) * fo[0] + std::cos(a) * fo[1] : 0.0f);
                if (tf >= kGlideGateFrom)
                    run.peakVis = std::fmax(run.peakVis, std::hypot(vx - pvx, vy - pvy) * 50.0f);
                pvx = vx; pvy = vy;
            }
            ++nextFeed;
        }
        Records rec = game;
        vt::FkSide(rec.data(), kGameCount, kRig, ticks, false);
        float ro[3] = {};
        const bool on = vt::RootOffset(ticks, ro);
        const float shown = aYawDeg * ease(tj) + (on ? ro[2] : 0.0f);
        if (j > 0)
            run.maxRate = std::fmax(run.maxRate, std::fabs(shown - prevShown) * 100.0f);
        prevShown = shown;
        if (on && std::fabs(ro[2]) > 1.0f)
            run.endAt = tj;
        if (on && std::hypot(ro[0], ro[1]) > 0.01f)
            run.planarEnd = tj;
        run.peakOff = std::fmax(run.peakOff, std::hypot(ro[0], ro[1]));
    }
    vt::Abort();
    vt::g_glideOff.store(false);
    return run;
}


// The animation thread may read Control right after a feed published it with a rootTicks newer than its own aNow (the game thread republishes the jump path at every
// slide-follow feed): the hold must not drop to 0 for that frame. Largest difference of the shown planar offset between aNow = feed - 0.5 ms and aNow = feed over the
// follow samples (0.16 .. 0.30 s) of a 2.2 m slide, 50 Hz feed.
float RunSlideRace(bool aGlideOff)
{
    vt::g_glideOff.store(aGlideOff);
    SetRoles();
    const float p0[3] = {-1.85f, 0.5f, 0.0f};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0, true);
    vt::SetMountTpp(true);
    float worst = 0.0f;
    for (int k = 1; k <= 20; ++k)
    {
        const float tf = float(k) / 50.0f;
        const float pos[3] = {p0[0], p0[1] - 2.2f * std::sin(1.5707963f * std::fmin(tf / 0.30f, 1.0f)), 0.0f};
        const int64_t tick = t0 + int64_t(double(tf) * double(kTps));
        vt::UpdateWorld(pos, 0.0f, tick);
        float before[3] = {}, after[3] = {};
        const bool b = vt::RootOffset(tick - 500, before), a = vt::RootOffset(tick, after);
        if (tf >= 0.16f && tf <= 0.30f)
            worst = std::fmax(worst, std::hypot((b ? before[0] : 0.0f) - (a ? after[0] : 0.0f), (b ? before[1] : 0.0f) - (a ? after[1] : 0.0f)));
    }
    vt::Abort();
    vt::g_glideOff.store(false);
    return worst;
}

void TestGlide()
{
    std::printf("  F glide limit per ride (no door cap): visible peak from %.2f s, release closing speed (>= 0.1 s windows), pre-glide hold -> glide limit\n", double(kGlideGateFrom));
    for (const DoorRideFx& r : kDoorRides)
        CheckGlideRide(r, ArcOfSeat(r.seat).valid ? "arc" : "-");
    // the three named cases: the Outlaw door slide (2.2 m in 0.30 s, ride 25), the Hella door slide (1.45 m, ride 4), the Emperor parallel hold (0.97 m, ride 12)
    const struct { int ride; float minOffset; float maxOffset; } named[] = {{25, 2.0f, 2.5f}, {4, 1.0f, 1.3f}, {12, 0.9f, 1.2f}};
    for (const auto& n : named)
    {
        vt::g_glideOff.store(false);
        const DoorRun run = RunDoorRide(*FindRide(n.ride), true, -1.0f, 0.0f);
        const GlideStats g = Glide(run);
        Check('F', g.peakOff >= n.minOffset && g.peakOff <= n.maxOffset,
              "F named ride " + std::to_string(n.ride) + ": the hold now carries the whole slide (peak offset " + std::to_string(g.peakOff) + " m; the old clamp stopped at 1.2 m)");
    }
    // F4: everything the limit must leave alone
    int capRides = 0;
    for (const DoorRideFx& r : kDoorRides)
    {
        const ArcFx arc = ArcOfSeat(r.seat);
        const CapInfo ci = RedsCap(arc, r.xy[0], r.xy[1]);
        if (ci.cap >= 0.0f)
        {
            ++capRides;
            vt::g_glideOff.store(true);
            const DoorRun old = RunDoorRide(r, true, ci.cap, 0.0f);
            vt::g_glideOff.store(false);
            const DoorRun now = RunDoorRide(r, true, ci.cap, 0.0f);
            Check('F', old.hash == now.hash, "F4 ride " + std::to_string(r.ride) + ": a mount with a door cap replays the pre-glide hold bit-identically");
        }
        vt::g_glideOff.store(true);
        const DoorRun bikeOld = RunDoorRide(r, false, -1.0f, 0.0f);
        vt::g_glideOff.store(false);
        const DoorRun bikeNow = RunDoorRide(r, false, -1.0f, 0.0f);
        Check('F', bikeOld.hash == bikeNow.hash, "F4 ride " + std::to_string(r.ride) + ": a bike mount is bit-identical with and without the limit");
    }

    // F5 door distance: the visible root of a mount WITHOUT a cap (the door cannot reach its pre-mount spot: why 2 / 3) comes no closer to the door panel than before
    for (const DoorRideFx& r : kDoorRides)
    {
        const ArcFx arc = ArcOfSeat(r.seat);
        if (!arc.valid || RedsCap(arc, r.xy[0], r.xy[1]).cap >= 0.0f)
            continue;
        vt::g_glideOff.store(true);
        const DoorRun old = RunDoorRide(r, true, -1.0f, 0.0f);
        vt::g_glideOff.store(false);
        const DoorRun now = RunDoorRide(r, true, -1.0f, 0.0f);
        const float dOld = MinDoorDist(r, arc, old, true), dNow = MinDoorDist(r, arc, now, true);
        std::printf("  F5 ride %3d door panel distance of the visible root %.3f -> %.3f m\n", r.ride, double(dOld), double(dNow));
        Check('F', dNow >= dOld - 0.03f || dNow >= 0.25f, "F5 ride " + std::to_string(r.ride) + ": the glided visible root comes no closer to the door panel than before (or stays >= 0.25 m: the cap's clearance is 0.20 m) (" + std::to_string(dOld) + " -> " + std::to_string(dNow) + " m)");
    }
    // F6 yaw: a 2.2 m slide with a 75 deg pivot (the entity turns toward the entry slot as in the recorded car entries): the shown yaw is no faster than before
    {
        const YawRun off = RunSlideYaw(2.2f, 75.0f, true), on = RunSlideYaw(2.2f, 75.0f, false);
        std::printf("  F6 2.2 m / 75 deg slide: shown yaw rate %.0f -> %.0f deg/s, yaw offset ends %.2f -> %.2f s, planar offset ends %.2f -> %.2f s, peak planar offset %.2f -> %.2f m\n",
                    double(off.maxRate), double(on.maxRate), double(off.endAt), double(on.endAt), double(off.planarEnd), double(on.planarEnd), double(off.peakOff), double(on.peakOff));
        std::printf("  F6 visible root peak speed at the feed instants (from 0.06 s): %.2f -> %.2f m/s\n", double(off.peakVis), double(on.peakVis));
        Check('F', on.peakVis <= off.peakVis, "F6 the visible root is no faster with the glide limit under a pivot (" + std::to_string(off.peakVis) + " -> " + std::to_string(on.peakVis) + " m/s)");
        Check('F', on.maxRate <= off.maxRate * 1.05f + 5.0f, "F6 the shown yaw is no faster with the glide limit (" + std::to_string(off.maxRate) + " -> " + std::to_string(on.maxRate) + " deg/s)");
        Check('F', on.endAt <= off.endAt + 0.05f, "F6 the yaw offset is released no later than before (" + std::to_string(off.endAt) + " -> " + std::to_string(on.endAt) + " s)");
        Check('F', on.peakOff > off.peakOff + 0.3f, "F6 control: the glided hold carries more of the slide (" + std::to_string(off.peakOff) + " -> " + std::to_string(on.peakOff) + " m)");
    }
    // F8 cross-thread race (see RunSlideRace): no one-frame drop of the hold on a follow feed
    {
        const float w = RunSlideRace(false);
        std::printf("  F8 follow feed read 0.5 ms early: largest change of the shown planar offset %.4f m\n", double(w));
        Check('F', w < 0.005f, "F8 an animation thread reading Control just before a follow feed's publish does not see the hold drop (" + std::to_string(w) + " m)");
    }
    // F7 a cap that arrives while the limit is active (0.05 / 0.10 s after BeginMount; the door script sends it at the MountingEvent): the planar clamp stays, so the
    // hold is not cut from 2.2 m to 1.2 m in one sample, and the cap is applied (its release then runs on the cap's own clock)
    for (float at : {0.05f, 0.10f})
    {
        const DoorRideFx& r = *FindRide(25);
        const CapInfo ci = RedsCap(ArcOfSeat(r.seat), r.xy[0], r.xy[1]);
        vt::g_glideOff.store(false);
        const DoorRun c = RunDoorRide(r, true, ci.cap, at);
        float worstDrop = 0.0f, prevOff = -1.0f;
        for (size_t i = 0; i < c.t.size(); ++i)
        {
            const float o = std::hypot(c.vx[i] - c.ex[i], c.vy[i] - c.ey[i]);
            if (c.t[i] >= at && c.t[i] <= at + 0.08f && prevOff >= 0.0f)
                worstDrop = std::fmax(worstDrop, prevOff - o);
            prevOff = o;
        }
        std::printf("  F7 ride 25, cap arrives at +%.2f s: state %d, largest one-frame drop of the offset in the next 0.08 s %.3f m, peak offset %.2f m\n", double(at), c.capState, double(worstDrop), double(c.jumpPeak));
        Check('F', c.capState == 3 || c.capState == 2, "F7 a cap arriving at +" + std::to_string(at) + " s is applied (state " + std::to_string(c.capState) + ")");
        Check('F', worstDrop < 0.12f, "F7 a cap arriving at +" + std::to_string(at) + " s does not cut the hold in one frame (" + std::to_string(worstDrop) + " m)");
    }
    Check('F', capRides >= 15, "F4 the cap rides were replayed (" + std::to_string(capRides) + ")");
}

// ------------------------------------------------------------------------------------------------ (G) review r2 findings 17 / 20: late door caps and the glided tail
// 17  A car mount without a cap glides: the planar release runs over Control::planarS (up to kGlideEndSeconds) and the clamp stays at kGlideMaxDistance. A door cap
//     that arrives after the glide started (up to kHoldCapMaxAgeSeconds after BeginMount) is judged against that release, not against kRootSeconds / kRootTailSeconds:
//     G1 jump path, cap 0.5 s at +0.20 s (no further jump sample): the cap is applied, the visible root is on the entity from the cap on;
//     G2 jump path, cap 0.9 s at +0.10 s with a re-base sample after it: the release is bounded by the cap and speed-limited (the old code released the 2 m hold in 0.30 s);
//     G3 cumulative path (synthetic standing-entry slide), cap 1.2 s at +0.20 s: applied (the old code skipped it for capSec >= 1.05 s), the offset is gone by the cap;
//        a cap behind the end of the glided release (1.5 s) is skipped as before (control).
// 20  The glided planar tail of the cumulative path (to 1.5 s) keeps being re-published from the world-fixed peak: G4 an entity that turns 25 deg between 1.05 and 1.5 s
//     does not swing the shown root sideways.
// Mutations 27 (jump gate against kRootSeconds) -> G1; 28 (re-base after the cap drops the speed-limited release) -> G2; 29 (cumulative skip / clamp against
// kRootTailSeconds) -> G3; 30 (trackOn window kRootTrackSeconds + kRootTailSeconds) -> G4.
struct SynthRun
{
    std::vector<float> t, off, wx, wy; // per feed instant: time, |published planar offset|, shown offset in the world frame (entity yaw applied)
    int capState = 0;
    float planarS = 0.0f;
};
// Synthetic standing-entry slide of a car (Hips at the game entry height, so the cumulative path is available): the entity slides aDist m along -y in aSlideS
// (sin ease, 50 Hz feed, steps below kJumpDistance), and its yaw turns linearly from 0 to aYawTo between aYaw0 and aYaw1 seconds.
SynthRun RunSynthSlide(float aDist, float aSlideS, float aYawTo, float aYaw0, float aYaw1, float aCapSec, float aCapAt)
{
    SetRoles();
    Pose g = GamePoseB();
    g.p[Hips] = {0.0f, 0.0f, 0.973f};
    const Records game = MakeRecords(g);
    const float p0[3] = {-1.85f, 0.5f, 0.0f};
    const int64_t t0 = NextEpochTime();
    vt::BeginMount(MakeSnap(MmPoseA(), 1.0f), p0, 0.0f, t0, true);
    vt::SetMountTpp(true);
    SynthRun run;
    const auto yawAt = [&](float t) { return aYawTo * std::fmin(std::fmax((t - aYaw0) / (aYaw1 - aYaw0), 0.0f), 1.0f); };
    int nextFeed = 1;
    bool capSent = aCapSec < 0.0f;
    for (int j = 0; j <= 170; ++j)
    {
        const float tj = float(j) / 100.0f;
        const int64_t ticks = t0 + int64_t(double(tj) * double(kTps));
        if (!capSent && tj >= aCapAt - 1e-6f)
        {
            vt::SetHoldCap(aCapSec, ticks);
            capSent = true;
        }
        while (float(nextFeed) / 50.0f <= tj + 1e-5f)
        {
            const float tf = float(nextFeed) / 50.0f;
            const float pos[3] = {p0[0], p0[1] - aDist * std::sin(1.5707963f * std::fmin(tf / aSlideS, 1.0f)), 0.0f};
            const int64_t tick = t0 + int64_t(double(tf) * double(kTps));
            vt::UpdateWorld(pos, yawAt(tf), tick);
            float ro[3] = {};
            const bool on = vt::RootOffset(tick, ro);
            const float a = yawAt(tf) * kDeg;
            run.t.push_back(tf);
            run.off.push_back(on ? std::hypot(ro[0], ro[1]) : 0.0f);
            run.wx.push_back(on ? std::cos(a) * ro[0] - std::sin(a) * ro[1] : 0.0f);
            run.wy.push_back(on ? std::sin(a) * ro[0] + std::cos(a) * ro[1] : 0.0f);
            ++nextFeed;
        }
        Records rec = game;
        vt::FkSide(rec.data(), kGameCount, kRig, ticks, false);
    }
    run.capState = vt::g_capState.load();
    run.planarS = vt::g_glidePlanarS.load();
    vt::Abort();
    return run;
}
float OffsetAfter(const DoorRun& r, float aFrom)
{
    float worst = 0.0f;
    for (size_t i = 0; i < r.t.size(); ++i)
        if (r.t[i] >= aFrom)
            worst = std::fmax(worst, std::hypot(r.vx[i] - r.ex[i], r.vy[i] - r.ey[i]));
    return worst;
}
float OffsetAt(const DoorRun& r, float aTime)
{
    for (size_t i = 0; i < r.t.size(); ++i)
        if (r.t[i] >= aTime - 1e-6f)
            return std::hypot(r.vx[i] - r.ex[i], r.vy[i] - r.ey[i]);
    return 0.0f;
}
float PeakClosing(const DoorRun& r, float aFrom)
{
    float peak = 0.0f;
    for (size_t i = 1; i < r.t.size(); ++i)
        if (r.t[i] >= aFrom)
            peak = std::fmax(peak, (std::hypot(r.vx[i - 1] - r.ex[i - 1], r.vy[i - 1] - r.ey[i - 1]) - std::hypot(r.vx[i] - r.ex[i], r.vy[i] - r.ey[i])) /
                                       (r.t[i] - r.t[i - 1]));
    return peak;
}
float OffsetAfter(const SynthRun& r, float aFrom)
{
    float worst = 0.0f;
    for (size_t i = 0; i < r.t.size(); ++i)
        if (r.t[i] >= aFrom)
            worst = std::fmax(worst, r.off[i]);
    return worst;
}
float OffsetAt(const SynthRun& r, float aTime)
{
    for (size_t i = 0; i < r.t.size(); ++i)
        if (r.t[i] >= aTime - 1e-6f)
            return r.off[i];
    return 0.0f;
}

void TestGlideLateCap()
{
    vt::g_glideOff.store(false);
    const DoorRideFx& r25 = *FindRide(25);
    // G1
    {
        const DoorRun c = RunDoorRide(r25, true, 0.5f, 0.20f);
        const float peak = OffsetAt(c, 0.20f), late = OffsetAfter(c, 0.52f);
        std::printf("  G1 ride 25, cap 0.50 s at +0.20 s: state %d, offset at the cap's arrival %.2f m, after the cap %.4f m, peak closing %.1f m/s\n", c.capState, double(peak), double(late),
                    double(PeakClosing(c, 0.2f)));
        Check('G', peak > 1.0f, "G1 fixture: the glided hold still carries more than 1 m when the cap arrives (" + std::to_string(peak) + " m)");
        Check('G', c.capState == 3, "G1 a late cap is applied against the glided release (state " + std::to_string(c.capState) + ")");
        Check('G', late < 0.01f, "G1 the visible root is back on the entity by the cap (" + std::to_string(late) + " m after 0.52 s)");
    }
    // G2
    {
        const DoorRun c = RunDoorRide(r25, true, 0.9f, 0.10f);
        const float at50 = OffsetAt(c, 0.50f), late = OffsetAfter(c, 0.92f), closing = PeakClosing(c, 0.15f);
        std::printf("  G2 ride 25, cap 0.90 s at +0.10 s: state %d, offset at 0.50 s %.2f m, after the cap %.4f m, peak closing %.1f m/s\n", c.capState, double(at50), double(late), double(closing));
        Check('G', late < 0.01f, "G2 the visible root is back on the entity by the cap (" + std::to_string(late) + " m after 0.92 s)");
        Check('G', closing < 6.0f && at50 > 0.3f, "G2 the release is speed-limited, bounded by the cap and not the old 0.30 s one (peak closing " + std::to_string(closing) + " m/s, offset at 0.50 s " + std::to_string(at50) + " m)");
    }
    // G3
    {
        const SynthRun c = RunSynthSlide(2.0f, 0.45f, 0.0f, 1.0f, 1.5f, 1.2f, 0.20f);
        const SynthRun ctl = RunSynthSlide(2.0f, 0.45f, 0.0f, 1.0f, 1.5f, 1.3f, 0.20f);
        const float natural = OffsetAt(c, 1.0f), late = OffsetAfter(c, 1.22f);
        std::printf("  G3 cumulative slide 2 m, planar release %.2f s, cap 1.20 s at +0.20 s: state %d, offset at 1.00 s %.2f m, after the cap %.4f m; cap 1.30 s: state %d, offset at 1.00 s %.2f m\n",
                    double(c.planarS), c.capState, double(natural), double(late), ctl.capState, double(OffsetAt(ctl, 1.0f)));
        Check('G', c.planarS > 0.7f && natural > 0.3f, "G3 fixture: a glided release longer than kRootTailSeconds is still running at 1.0 s (planarS " + std::to_string(c.planarS) + " s, offset " + std::to_string(natural) + " m)");
        Check('G', c.capState == 2, "G3 a cap behind kRootTrackSeconds + kRootTailSeconds but inside the glided release is applied (state " + std::to_string(c.capState) + ")");
        Check('G', late < 0.01f, "G3 the glided hold does not outlast the cap (" + std::to_string(late) + " m after 1.22 s)");
        Check('G', ctl.capState == 1 && OffsetAt(ctl, 1.0f) > 0.1f, "G3 control: a cap behind the end of the glided release is skipped (state " + std::to_string(ctl.capState) + ")");
    }
    // G4
    {
        const SynthRun s = RunSynthSlide(2.0f, 0.45f, 25.0f, 1.05f, 1.5f, -1.0f, 0.0f);
        float rx = 0.0f, ry = 0.0f;
        for (size_t i = 0; i < s.t.size(); ++i)
            if (s.t[i] >= 1.0f - 1e-6f)
            {
                const float n = std::hypot(s.wx[i], s.wy[i]);
                rx = s.wx[i] / n; ry = s.wy[i] / n;
                break;
            }
        float lateral = 0.0f;
        for (size_t i = 0; i < s.t.size(); ++i)
            if (s.t[i] >= 1.06f && s.t[i] <= 1.5f)
                lateral = std::fmax(lateral, std::fabs(rx * s.wy[i] - ry * s.wx[i]));
        std::printf("  G4 cumulative slide 2 m, planar release %.2f s, entity turns 25 deg in 1.05..1.5 s: offset at 1.06 s %.2f m, largest sideways swing of the shown root %.3f m\n", double(s.planarS),
                    double(OffsetAt(s, 1.06f)), double(lateral));
        Check('G', s.planarS > 0.9f && OffsetAt(s, 1.06f) > 0.3f, "G4 fixture: the glided planar release outlasts kRootTrackSeconds + kRootTailSeconds (planarS " + std::to_string(s.planarS) + " s)");
        Check('G', lateral < 0.02f, "G4 the held offset stays fixed in the world while the entity turns after 1.05 s (" + std::to_string(lateral) + " m sideways)");
    }
}
} // namespace

int main(int argc, char** argv)
{
    g_neckVerbose = argc > 1 && std::strcmp(argv[1], "-v") == 0;
    vt::g_ticksPerSecond.store(kTps);
    TestOnFoot();
    TestRoles();
    TestCarMount();
    TestBikeMount();
    TestExit();
    TestRigChange();
    TestPerspective();
    TestVehicleVeto();
    TestHelperJobs();
    TestProductionRoles();
    TestSeatSwitch();
    TestPopFilter();
    TestDeniedCapture();
    TestExitLatch();
    TestExitFill();
    TestStandingLegs();
    TestRootTracking();
    TestYawAlign();
    TestPopHips();
    TestPlanarRootDecay();
    TestLegsUntouchedAfterDecay();
    TestNeckPop();
    TestDoorCap();
    TestGlide();
    TestGlideLateCap();
    std::string groups;
    for (char g : g_failedGroups)
        groups += std::string("[") + g + "]";
    std::printf("vehtrans mutation=%d failures=%d\nfailed groups: %s\n", static_cast<int>(vt::kMutation), g_failures, groups.c_str());
    return g_failures ? 1 : 0;
}
