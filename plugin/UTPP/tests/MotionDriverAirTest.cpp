// air (commit C prep): v2 air consumer through the REAL v1 MMDriver (src/MotionHost.cpp + src/MotionDriver.cpp, no RED4ext).
// Usage: utpp_motion_driver_air_test <data dir> <v1 database.utppmm>
//   data dir = <root>\Data (utpp_motion_woman.utppmm + .chooser.json, overlay\, traversal\; motion-derived, outside git).
// Synthetic 60 Hz feeds (world gravity = the game's: 19.2 m/s^2 rising, 16 m/s^2 falling = air::AirConfig gUpDefault / gDownDefault, which
// is what the runtime AirState starts from and hands to v2 since e4fdd84 "Connect learned air gravity"; before it the feeds used the
// constant 9.81 of adapter::MapConfig::gravity; probe every 2nd air feed, others -2 = not probed) of the runtime
// interface (MotionDriver.hpp SetAirFeed: Main.cpp passes the UTPPFeedAir fields right before motionhost::Feed) with the runtime's
// AirOwns span (airborne + landing; the runtime AirState's span, used here only to count feeds): standing jump, running jump,
// drop 1.2 m (light), drop 4 m (heavy), hood jump-off (run jump off a 1.0 m hood, ~7.6 m/s: light at the game gravity; vehicle batch J1).
// Heavy = |landing vz| >= 700 sqrt(kG / 9.81) cm/s (a 2.5 m drop; 894 cm/s at kG 16). v2 owns air (motionfix item 5: the AirOwns yield was removed).
// Checks:
//   A chooser MovementMode InAir exactly from the first airborne feed to the last (drop: the PSM still says Stand on the first
//     airborne feed, only the AirFeed makes it InAir), OnGround again on the touchdown feed
//   B picks: every air feed after the entry blend (from the 2nd airborne feed) plays a frame of a Jumps database; every feed of
//     the justLanded window plays a Lands database of the right weight (Lands_Light / Lands_Heavy)
//   C TimeToLand (trajectory floor from distanceToGround, dead-reckoned on not-probed feeds) within 0.05 s of the analytic
//     time to land on every descending air feed with <= 1.4 s left
//   D justLandedLight / Heavy from the landing vz (gravity-scaled threshold, 894 cm/s at kG 16) for 0.3 s (18 feeds at 60 Hz), never both, false before
//   E foot contacts handed to the v1 leg tail are 0 on every air feed (the source CONT of the picks is not 0 on all of them)
//   F landing continuity: max per-bone rotation step of the published pose over touchdown .. +0.4 s <= the largest
//     consecutive-frame step inside the source clips of the Jumps / Lands databases
//   G root vertical (RTZ feature vs capsule): residual reported, finite, the animation root integral moved during the jump
//   H v2 owns air: v2 steps every feed of the AirOwns span; status says air:owns=v2
//   I mm.motionDriver off: published snapshots with SetAirFeed calls == the same feed without hooks (child processes, v1 bit-identical)
//   J (next) runtime path of main 22d4062: air fields in FeedInput, forwarded only by motiondriver::FeedAirFromInput (the
//     Main.cpp call site): every v2 Step around the drop consumed the air feed, InAir exactly on
//     the airborne feeds; the runtime AirState owns the same drop; the landing deficit bypass is consumed once, by the touchdown feed
//   V (vehicle batch, CAR-1 / J4) vehicle exit: v2 steps every feed of the v1 exit window (no skip 'gate'), chooser stance Stand through
//     the exit animation (detail 0, latched exit workspot); unmount + ballistic arc vz0 +6: InAir Jumps from the 2nd airborne feed, light landing.
//     Mutations UTPP_VEHEXIT_MUTATION=2 (utpp_motion_driver_air_vehexit_mut2) and UTPP_TRAJV2_HEAVY_MUTATION=1 (utpp_motion_driver_air_heavy_mut1) fail V / hood.
//   K (vehicle batch, J2) runtime-faithful probing (MakeRuntimeProbe: no probe on grounded / rising feeds, take-off feed PSM Jump
//     18 on the ground): flat run jump and standing jump TimeToLand within 0.1 s on the rising feeds and no Jumps_Far chooser
//     database while the true time to land < 1.0 s (A-G also checked on those feeds); drop guard: a 1.5 m walk-off never uses the
//     take-off plane as its floor
// Mutations (build with -DUTPP_MOTION_DRIVER_AIR_MUTATION=k): 1 air feed dropped, 4 air contacts kept, 5 landing flags not forwarded,
// 6 no dead reckoning, 7 RTZ integral off, 8 landing deficit bypass off, 9 FeedAirFromInput drops onGround, 10 FeedAirFromInput
// forwards nothing, 2 (motionfix item 5) v2 yields airborne feeds again, 12 (J2) no take-off plane seed (fails K), 13 (J2) the
// seed is also used for a walk-off (fails the K drop guard). Each must FAIL this test (3, 11 tested the removed SetMotionOwnsAir
// switch). 12 and 13 are built as utpp_motion_driver_air_mut12 / mut13 (tests/motiondriverair.cmake).

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/MotionHost.hpp"
#include "../src/MotionDriver.hpp"
#include "../src/State.hpp"
#include "../src/mm/MotionDatabase.hpp"
#include "TestFailFast.hpp"

namespace utpp
{
HotState g_hot;
mm::integ::SeqlockBuffer<mm::integ::PoseSnapshot> g_mmPose;
} // namespace utpp

using namespace utpp;
using namespace utpp::mm;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat.c_str(), aDetail.empty() ? "" : " | ", aDetail.c_str());
    if (!aOk)
        UtppFailFastNote(std::string("FAIL ") + aWhat + (aDetail.empty() ? "" : " | ") + aDetail);
}
std::string F(double v, int p = 3)
{
    char b[64];
    std::snprintf(b, sizeof b, "%.*f", p, v);
    return b;
}

constexpr float kDt = 1.0f / 60.0f;
// World gravity of the synthetic feeds = the game's (see the header). The runtime AirState starts from exactly these values and v2 plans the
// landing (TimeToLand, check C) with the gravity AirState has learned; a world with another g (the old 9.81) makes the first jump's
// time-to-land miss by ~0.1 s while the online estimate converges, which is a property of the synthetic world, not of the code.
constexpr float kGUp = 19.2f; // air::AirConfig::gUpDefault
constexpr float kG = 16.0f;   // air::AirConfig::gDownDefault
// J1 (vehicle batch): the motion heavy-landing rule is a 2.5 m drop (700 cm/s at 9.81 m/s^2); at the game's fall gravity the same drop lands
// at 700 * sqrt(g / 9.81) cm/s (kG 16 -> 894 cm/s). Computed here independently of motiontraj::HeavyLandThresholdCmS (checked equal below), so
// the mutation UTPP_TRAJV2_HEAVY_MUTATION=1 (threshold stays 700) is caught by the scenario expectations, not hidden by them.
float HeavyLandCmSAtG() { return 700.0f * std::sqrt(kG / 9.81f); }

// ------------------------------------------------------------------ synthetic feed
struct Sf
{
    float vx = 0, vy = 0, vz = 0, mvy = 0;
    int detail = -1; // PSM LocomotionDetailed (-1 = Stand via locoState, no detailed bit)
    int loco = 1;
    bool airFeed = true, onGround = true, airOwns = false;
    float dist = -2.0f;
    int fall = 0;
    // truth
    bool airborne = false;
    float ttlTrue = -1.0f; // s to touchdown (airborne feeds)
    bool descending = false;
    float distTrue = -1.0f; // m above the landing level (airborne feeds)
    uint32_t psmExtra = 0;  // vehicle batch: extra psmBlock bits (kPsmVehicle 0x1 during a ride, kPsmWorkspot 0x8 for the exit workspot)
};
uint32_t PsmBlock(int aDetail) { return aDetail < 0 ? 0u : (integ::kPsmDetailed | (uint32_t(aDetail + 1) << integ::kPsmDetailShift)); }

struct Scenario
{
    std::string name;
    std::vector<Sf> fr;
    size_t takeoff = 0, touchdown = 0; // first airborne feed, first ground feed after it
    bool heavy = false;
    bool psmLeadsLate = false; // drop: PSM still Stand on the first airborne feed
    bool psmJumpOnGround = false; // runtime-faithful jump: the take-off feed (last ground feed) already says PSM Jump 18
};

// aRun: planar speed (m/s, forward +y); aVz0 jump velocity (0 = drop from aDrop m)
Scenario Make(const std::string& aName, float aRun, float aVz0, float aDrop)
{
    Scenario s;
    s.name = aName;
    const int ground = 60;
    for (int i = 0; i < ground; ++i)
    {
        Sf f;
        f.vy = aRun;
        f.mvy = aRun > 0 ? 1.0f : 0.0f;
        s.fr.push_back(f);
    }
    s.takeoff = s.fr.size();
    // Two-part ballistic flight: rising with kGUp until the apex (aVz0 > 0), falling with kG afterwards (a drop starts at its apex);
    // landing when z = -aDrop, z measured from the take-off level.
    const float tApex = aVz0 > 0.0f ? aVz0 / kGUp : 0.0f;
    const float hApex = (aVz0 > 0.0f ? aVz0 * aVz0 / (2.0f * kGUp) : 0.0f) + aDrop; // apex height above the landing level
    const float tLand = tApex + std::sqrt(2.0f * hApex / kG);
    const int nAir = static_cast<int>(std::floor(tLand / kDt)); // airborne feeds at t = (k+1) dt <= tLand
    for (int k = 0; k < nAir; ++k)
    {
        const float t = (k + 1) * kDt;
        Sf f;
        f.vy = aRun;
        f.mvy = aRun > 0 ? 1.0f : 0.0f;
        const bool rising = t <= tApex;
        f.vz = rising ? aVz0 - kGUp * t : -kG * (t - tApex);
        const float z = rising ? aVz0 * t - 0.5f * kGUp * t * t : (hApex - aDrop) - 0.5f * kG * (t - tApex) * (t - tApex);
        f.airborne = true;
        f.onGround = false;
        f.airOwns = true;
        f.loco = 0;
        f.descending = f.vz < 0.0f;
        f.detail = aVz0 > 0.0f && f.vz > 0.0f ? 18 : 14; // Jump while rising, Fall after
        if (aVz0 <= 0.0f && k == 0)
        {
            f.detail = -1; // PSM still Stand (the runtime AirState leads the PSM by a frame)
            f.loco = 1;
            s.psmLeadsLate = true;
        }
        f.dist = (k % 2 == 0) ? (z + aDrop) : -2.0f; // probe every 2nd feed
        f.distTrue = z + aDrop;
        f.fall = f.descending ? 1 : 0;
        f.ttlTrue = tLand - t;
        s.fr.push_back(f);
    }
    s.touchdown = s.fr.size();
    const float landVz = -kG * (tLand - tApex);
    s.heavy = std::fabs(landVz) * 100.0f >= HeavyLandCmSAtG();
    for (int i = 0; i < 60; ++i)
    {
        Sf f;
        f.vy = aRun;
        f.mvy = aRun > 0 ? 1.0f : 0.0f;
        if (i < 9)
        {
            f.detail = s.heavy ? 24 : 23; // HardLand / RegularLand for 0.15 s
            f.loco = 0;
            f.airOwns = true;             // runtime Recover phase still owns
            f.fall = s.heavy ? 2 : 1;
        }
        f.dist = 0.0f;
        s.fr.push_back(f);
    }
    return s;
}

// J2 (vehicle batch): runtime-faithful probe mode of the same flight. The runtime probes the ground only on an airborne feed whose
// previous feed was airborne and not rising, every 2nd feed (MotionHost.cpp AirWantsProbe: g_airLastAirborne, g_airLastVz <= 0,
// g_feedCalls parity); grounded and rising feeds are never probed (-2). A jump's take-off feed still touches the ground with PSM
// Jump 18 (tests/data/jump_feeds_20261021_1213.txt: every take-off reads ground detail, ground detail, 18, then airborne 18).
Scenario MakeRuntimeProbe(const std::string& aName, float aRun, float aVz0, float aDrop)
{
    Scenario s = Make(aName, aRun, aVz0, aDrop);
    for (size_t i = 0; i < s.takeoff; ++i)
        s.fr[i].dist = -2.0f;
    if (aVz0 > 0.0f && s.takeoff > 0)
    {
        s.fr[s.takeoff - 1].detail = 18;
        s.fr[s.takeoff - 1].loco = 0;
        s.psmJumpOnGround = true;
    }
    for (size_t i = s.takeoff; i < s.touchdown; ++i)
    {
        const Sf& prev = s.fr[i - 1];
        const bool wants = prev.airborne && prev.vz <= 0.0f && ((i - s.takeoff) % 2 == 1);
        s.fr[i].dist = wants ? s.fr[i].distTrue : -2.0f;
    }
    for (size_t i = s.touchdown; i < s.fr.size(); ++i)
        s.fr[i].dist = -2.0f;
    return s;
}

// Vehicle batch (CAR-1 / CAR-2 / J4 through v2): a 1 s ride (kPsmVehicle, PSM detail 0, loco 0 - the v1 gate releases the body to the
// game), then the vehicle-bit falling edge. aWindow = the exit animation inside the v1 exit window (PSM detail 0 = NotInBase, loco 0,
// aWs: the exit workspot latched at the edge) for aWindowS seconds, then Stand. aArcVz0 > 0: the unmount launches a ballistic arc
// instead (vz0, g = kG 16, PSM detail 0, runtime-faithful probing) that lands on the take-off level, RegularLand 23 for 0.15 s, Stand.
Scenario MakeVehicleExit(const std::string& aName, float aWindowS, bool aWs, float aArcVz0)
{
    Scenario s;
    s.name = aName;
    for (int i = 0; i < 60; ++i)
    {
        Sf f;
        f.detail = 0;
        f.loco = 0;
        f.psmExtra = 0x1u; // kPsmVehicle
        s.fr.push_back(f);
    }
    s.takeoff = s.fr.size(); // = the vehicle-bit falling edge
    if (aArcVz0 > 0.0f)
    {
        const float tLand = 2.0f * aArcVz0 / kG;
        const int nAir = static_cast<int>(std::floor(tLand / kDt));
        for (int k = 0; k < nAir; ++k)
        {
            const float t = (k + 1) * kDt;
            Sf f;
            f.vy = 2.0f;
            f.vz = aArcVz0 - kG * t;
            f.detail = 0;
            f.loco = 0;
            f.psmExtra = aWs ? 0x8u : 0u;
            f.airborne = true;
            f.onGround = false;
            f.airOwns = true;
            f.descending = f.vz < 0.0f;
            f.distTrue = aArcVz0 * t - 0.5f * kG * t * t;
            f.fall = f.descending ? 1 : 0;
            f.ttlTrue = tLand - t;
            const Sf& prev = s.fr.back();
            const bool wants = prev.airborne && prev.vz <= 0.0f && (k % 2 == 1);
            f.dist = wants ? std::fmax(0.0f, f.distTrue) : -2.0f;
            s.fr.push_back(f);
        }
        s.touchdown = s.fr.size();
        s.heavy = std::fabs(aArcVz0 - kG * tLand) * 100.0f >= HeavyLandCmSAtG();
        for (int i = 0; i < 9; ++i)
        {
            Sf f;
            f.vy = 1.0f;
            f.detail = 23;
            f.loco = 0;
            f.airOwns = true;
            f.fall = 1;
            s.fr.push_back(f);
        }
    }
    else
    {
        const int n = static_cast<int>(std::lround(aWindowS / kDt));
        for (int i = 0; i < n; ++i)
        {
            Sf f;
            f.detail = 0;
            f.loco = 0;
            f.psmExtra = aWs ? 0x8u : 0u;
            s.fr.push_back(f);
        }
        s.touchdown = s.fr.size(); // = the end of the exit animation
    }
    for (int i = 0; i < 60; ++i)
        s.fr.push_back(Sf{});
    return s;
}

motionhost::FeedInput ToInput(const Sf& f)
{
    motionhost::FeedInput in;
    in.velWorld[0] = f.vx;
    in.velWorld[1] = f.vy;
    in.velWorld[2] = f.vz;
    in.move[1] = f.mvy;
    in.cameraYawDeg = 0;
    in.playerYawDeg = 0;
    in.locoState = f.loco;
    in.tppActive = true;
    in.dt = kDt;
    in.psmBlock = PsmBlock(f.detail) | f.psmExtra;
    return in;
}

// ------------------------------------------------------------------ rig / data
struct Rig
{
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
};
std::unique_ptr<motion::MotionDatabase> LoadMotion(const std::string& aPath, Rig& aRig)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return nullptr;
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (bytes.size() < 32)
        return nullptr;
    uint64_t rig = 0;
    std::memcpy(&rig, bytes.data() + 24, sizeof(rig));
    auto db = std::make_unique<motion::MotionDatabase>();
    std::string err;
    if (!db->LoadFromMemory(bytes, rig, err))
        return nullptr;
    for (uint32_t i = 0; i < db->NumBones(); ++i)
    {
        aRig.hashes.push_back(integ::CNameHash(db->BoneName(i)));
        aRig.parents.push_back(static_cast<int16_t>(db->BoneParent(i)));
    }
    return db;
}
motiondriver::Paths DataPaths(const std::string& aData, const char* aBody)
{
    const std::string b(aBody);
    motiondriver::Paths p;
    p.files.db = aData + "\\utpp_motion_" + b + ".utppmm";
    p.files.chooser = aData + "\\utpp_motion_" + b + ".chooser.json";
    p.files.overlay = aData + "\\motion\\utpp_overlays_" + b + ".utppov";
    p.files.style = aData + "\\motion\\utpp_style_weights_" + b + ".txt";
    p.traversal = aData + "\\traversal\\utpp_traversal_" + b + ".utpptrav";
    return p;
}

// ------------------------------------------------------------------ observed hooks
bool g_stepped = false, g_framed = false;
float g_contact[2] = {0, 0};
bool WStep(const motionhost::FeedInput& aRaw, uint32_t aEffGate, float aDt)
{
    g_stepped = motiondriver::Step(aRaw, aEffGate, aDt);
    return g_stepped;
}
bool WFrame(const int32_t* aIdx, uint32_t aBones, motionhost::MotionSetFrame& aOut)
{
    g_framed = motiondriver::Frame(aIdx, aBones, aOut);
    if (g_framed)
    {
        g_contact[0] = aOut.contact[0];
        g_contact[1] = aOut.contact[1];
    }
    return g_framed;
}
const motionhost::MotionHooks kHooks{&WStep, &motiondriver::ApplyToSnapshot, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &WFrame};
// diagnostics only (UTPP_AIR_RAW=1): no frame hook -> the raw v2 pose is published without the v1 leg tail
const motionhost::MotionHooks kHooksRaw{&WStep, &motiondriver::ApplyToSnapshot, &motiondriver::OnArmed, &motiondriver::OnDisarmed, nullptr};
bool EnvSet(const char* aName)
{
    char b[8];
    return GetEnvironmentVariableA(aName, b, sizeof b) > 0;
}

std::wstring g_baseDb;
uint8_t g_rigDummy[16];
bool ResetBase(const Rig& aRig)
{
    motionhost::SetEnabled(false);
    motionhost::OnDisarmed();
    if (!motionhost::LoadDatabase(g_baseDb, "default"))
        return false;
    motionhost::SetCommandDtFromFeed(true);
    g_hot.armed.store(true, std::memory_order_release);
    motionhost::OnArmed(g_rigDummy, aRig.hashes, aRig.parents);
    motionhost::SetEnabled(true);
    return true;
}

integ::PoseSnapshot g_read, g_prev;
// per-bone rotation step (deg) between two snapshots, bone index = snapshot bone (= v2 database bone while v2 publishes)
void BoneSteps(const integ::PoseSnapshot& a, const integ::PoseSnapshot& b, std::vector<float>& aOut)
{
    aOut.assign(integ::kMaxMMBones, 0.0f);
    if (a.rig != b.rig)
        return;
    const uint32_t n = std::min({a.numBones, b.numBones, integ::kMaxMMBones});
    for (uint32_t i = std::max(a.firstBone, b.firstBone); i < n; ++i)
    {
        if (a.gameIndex[i] < 0 || a.gameIndex[i] != b.gameIndex[i])
            continue;
        const float* p = &a.pose[size_t(i) * 7 + 3];
        const float* q = &b.pose[size_t(i) * 7 + 3];
        const float d = std::fabs(p[0] * q[0] + p[1] * q[1] + p[2] * q[2] + p[3] * q[3]);
        aOut[i] = 2.0f * std::acos(std::min(1.0f, d)) * 57.2957795f;
    }
}
float MaxAngleDeg(const integ::PoseSnapshot& a, const integ::PoseSnapshot& b)
{
    if (a.rig != b.rig)
        return 0.0f;
    const uint32_t n = std::min({a.numBones, b.numBones, integ::kMaxMMBones});
    float m = 0.0f;
    for (uint32_t i = std::max(a.firstBone, b.firstBone); i < n; ++i)
    {
        if (a.gameIndex[i] < 0 || a.gameIndex[i] != b.gameIndex[i])
            continue;
        const float* p = &a.pose[size_t(i) * 7 + 3];
        const float* q = &b.pose[size_t(i) * 7 + 3];
        const float d = std::fabs(p[0] * q[0] + p[1] * q[1] + p[2] * q[2] + p[3] * q[3]);
        m = std::max(m, 2.0f * std::acos(std::min(1.0f, d)) * 57.2957795f);
    }
    return m;
}
uint64_t SnapHash(const integ::PoseSnapshot& aS)
{
    integ::PoseSnapshot s = aS;
    s.publishTicks = 0;
    s.sequence = 0;
    const auto* p = reinterpret_cast<const uint8_t*>(&s);
    uint64_t h = 1469598103934665603ull;
    for (size_t i = 0; i < sizeof(s); ++i)
        h = (h ^ p[i]) * 1099511628211ull;
    return h;
}

std::string StatusField(const std::string& aStatus, const char* aKey)
{
    const size_t p = aStatus.find(aKey);
    if (p == std::string::npos)
        return {};
    const size_t s = p + std::strlen(aKey);
    const size_t e = aStatus.find(' ', s);
    return aStatus.substr(s, e == std::string::npos ? std::string::npos : e - s);
}

struct Out
{
    bool stepped = false, framed = false, published = false;
    float contact[2] = {0, 0};
    int clip = -1, db = -1;
    float clipTime = 0, step = 0;
    std::string skip; // status skip= of that feed (the reason of the last v2 skip)
    std::vector<float> steps; // per bone (deg), empty = no previous published pose
    motiondriver::AirDiag d;
};

// one feed: air fields (runtime order: SetAirFeed, then Feed)
Out FeedOne(const Sf& f, bool aSendAir)
{
    if (aSendAir && f.airFeed)
        motiondriver::SetAirFeed(f.onGround, f.dist, f.fall);
    g_stepped = g_framed = false;
    motionhost::Feed(ToInput(f));
    Out o;
    o.stepped = g_stepped;
    o.framed = g_framed;
    o.contact[0] = g_contact[0];
    o.contact[1] = g_contact[1];
    o.published = g_mmPose.Read(g_read);
    o.d = motiondriver::AirDiagnostics();
    const std::string st = motiondriver::Status();
    o.skip = StatusField(st, "skip=");
    const std::string pk = StatusField(st, "pick=clip:");
    const size_t at = pk.find('@'), sl = pk.find("/db");
    if (at != std::string::npos && sl != std::string::npos && sl > at)
    {
        o.clip = std::atoi(pk.substr(0, at).c_str());
        o.clipTime = static_cast<float>(std::atof(pk.substr(at + 1, sl - at - 1).c_str()));
        o.db = std::atoi(pk.substr(sl + 3).c_str());
    }
    return o;
}

std::vector<Out> Run(const Scenario& s, bool aSendAir)
{
    std::vector<Out> out;
    bool havePrev = false;
    for (const Sf& f : s.fr)
    {
        Out o = FeedOne(f, aSendAir);
        if (o.published && havePrev)
        {
            o.step = MaxAngleDeg(g_read, g_prev);
            BoneSteps(g_read, g_prev, o.steps);
        }
        if (o.published)
        {
            g_prev = g_read;
            havePrev = true;
        }
        out.push_back(o);
    }
    return out;
}

float ContactAt(const motion::MotionDatabase& db, int aClip, float aClipTime, int aSide)
{
    if (aClip < 0 || static_cast<uint32_t>(aClip) >= db.NumClips())
        return 0.0f;
    const auto& cl = db.Clip(static_cast<uint32_t>(aClip));
    const float fps = db.Header().fps > 0.0f ? db.Header().fps : 30.0f;
    const float local = std::fmax(0.0f, aClipTime * fps);
    const uint32_t f0 = cl.firstFrame + std::min(static_cast<uint32_t>(local), cl.numFrames - 1);
    return aSide ? db.Contact(f0).r : db.Contact(f0).l;
}

// largest consecutive-frame bone rotation step inside the clips of the databases whose name contains aKey
std::vector<float> SourceStepBound(const motion::MotionDatabase& db, const char* aKey1, const char* aKey2)
{
    uint64_t mask = 0;
    for (uint32_t d = 0; d < db.NumDatabases() && d < 64; ++d)
    {
        const std::string_view n = db.DbName(d);
        if (n.find(aKey1) != std::string_view::npos || n.find(aKey2) != std::string_view::npos)
            mask |= 1ull << d;
    }
    const uint32_t nb = db.NumBones();
    std::vector<float> m(nb, 0.0f);
    for (uint32_t c = 0; c < db.NumClips(); ++c)
    {
        const auto& cl = db.Clip(c);
        if ((cl.dbMask & mask) == 0)
            continue;
        for (uint32_t f = cl.firstFrame; f + 1 < cl.firstFrame + cl.numFrames; ++f)
        {
            const auto a = db.Pose(f), b = db.Pose(f + 1);
            for (uint32_t i = 2; i < nb; ++i)
            {
                const float* p = &a[i].qx;
                const float* q = &b[i].qx;
                const float d = std::fabs(p[0] * q[0] + p[1] * q[1] + p[2] * q[2] + p[3] * q[3]);
                m[i] = std::max(m[i], 2.0f * std::acos(std::min(1.0f, d)) * 57.2957795f);
            }
        }
    }
    return m;
}

void Prime(const Scenario& s)
{
    // re-entry: v2 off for one feed then on (matcher, trajectory and air episode restart), 60 ground feeds follow in the scenario
    motiondriver::SetEnabled(false);
    (void)FeedOne(s.fr.front(), true);
    motiondriver::SetEnabled(true);
}

std::string DbN(const motion::MotionDatabase& db, int d) { return d >= 0 && uint32_t(d) < db.NumDatabases() ? std::string(db.DbName(uint32_t(d))) : "-"; }

void CheckScenario(const Scenario& s, const std::vector<Out>& r, const motion::MotionDatabase& db, const std::vector<float>& aBound)
{
    const std::string n = "[" + s.name + "] ";
    // A
    {
        int bad = -1;
        // runtime-faithful jump: the PSM Jump 18 of the take-off feed (the capsule has not left the ground) is held OnGround (JS1,
        // adapter::MapConfig::jumpTakeoffHoldS, tests/LocoStartTest.cpp): InAir from the launch feed on, like every other flight
        const size_t airFrom = s.takeoff;
        for (size_t i = s.takeoff - 5; i < s.touchdown + 3 && bad < 0; ++i)
        {
            const bool air = i >= airFrom && i < s.touchdown;
            if (!r[i].stepped || (r[i].d.movementMode == 1) != air)
                bad = static_cast<int>(i);
        }
        Check(bad < 0, n + "A MovementMode InAir exactly on the airborne feeds " + std::to_string(airFrom) + ".." +
                           std::to_string(s.touchdown - 1) + (s.psmLeadsLate ? " (first one from the AirFeed only, PSM Stand)" : "") +
                           (s.psmJumpOnGround ? " (the grounded PSM Jump take-off feed held OnGround)" : ""),
              bad < 0 ? "" : "feed " + std::to_string(bad) + " stepped=" + std::to_string(r[size_t(bad)].stepped) + " mode=" +
                                 std::to_string(r[size_t(bad)].d.movementMode));
    }
    // B
    {
        int bad = -1, airN = 0;
        for (size_t i = s.takeoff + 1; i < s.touchdown && bad < 0; ++i, ++airN)
            if (DbN(db, r[i].db).find("Jumps") == std::string::npos)
                bad = static_cast<int>(i);
        Check(bad < 0 && airN > 0, n + "B air picks from Jumps databases (" + std::to_string(airN) + " feeds)",
              bad < 0 ? "first " + DbN(db, r[s.takeoff + 1].db) : "feed " + std::to_string(bad) + " db " + DbN(db, r[size_t(bad)].db));
        const std::string want = s.heavy ? "Lands_Heavy" : "Lands_Light";
        int badL = -1, landN = 0;
        for (size_t i = s.touchdown; i < r.size() && r[i].d.justLandedLight != r[i].d.justLandedHeavy && badL < 0; ++i, ++landN)
            if (DbN(db, r[i].db).find(want) == std::string::npos)
                badL = static_cast<int>(i);
        Check(badL < 0 && landN > 0, n + "B landing picks from " + want + " databases (" + std::to_string(landN) + " feeds)",
              badL < 0 ? DbN(db, r[s.touchdown].db) : "feed " + std::to_string(badL) + " db " + DbN(db, r[size_t(badL)].db));
    }
    // C
    {
        float worst = 0.0f;
        int n2 = 0, nNp = 0;
        for (size_t i = s.takeoff; i < s.touchdown; ++i)
        {
            const Sf& f = s.fr[i];
            if (!f.descending || f.ttlTrue > 1.4f)
                continue;
            worst = std::max(worst, std::fabs(r[i].d.timeToLand - f.ttlTrue));
            ++n2;
            nNp += f.dist < -1.5f ? 1 : 0;
        }
        Check(n2 > 0 && nNp > 0 && worst <= 0.05f, n + "C TimeToLand from the trajectory floor within 0.05 s (" + std::to_string(n2) +
                                                     " descending feeds, " + std::to_string(nNp) + " not probed)", "max err " + F(worst));
    }
    // D
    {
        int lightN = 0, heavyN = 0, bad = -1;
        for (size_t i = s.takeoff; i < r.size(); ++i)
        {
            const bool l = r[i].d.justLandedLight, h = r[i].d.justLandedHeavy;
            if (l && h)
                bad = static_cast<int>(i);
            if ((l || h) && i < s.touchdown)
                bad = static_cast<int>(i);
            lightN += l;
            heavyN += h;
        }
        const int want = 18; // 0.3 s at 60 Hz (sinceLand < 0.3)
        const bool ok = bad < 0 && (s.heavy ? (heavyN == want && lightN == 0) : (lightN == want && heavyN == 0)) &&
                        (s.heavy ? r[s.touchdown].d.justLandedHeavy : r[s.touchdown].d.justLandedLight);
        Check(ok, n + "D justLanded" + (s.heavy ? "Heavy" : "Light") + " on touchdown for 0.3 s (18 feeds)",
              "light " + std::to_string(lightN) + " heavy " + std::to_string(heavyN) + " landVz " + F(r[s.touchdown].d.landVzCmS, 1) +
                  " cm/s bad " + std::to_string(bad));
    }
    // E
    {
        int framed = 0, srcNonZero = 0, bad = -1;
        for (size_t i = s.takeoff; i < s.touchdown; ++i)
        {
            if (!r[i].framed)
                continue;
            ++framed;
            if (ContactAt(db, r[i].clip, r[i].clipTime, 0) > 0.0f || ContactAt(db, r[i].clip, r[i].clipTime, 1) > 0.0f)
                ++srcNonZero;
            if (r[i].contact[0] != 0.0f || r[i].contact[1] != 0.0f)
                bad = static_cast<int>(i);
        }
        Check(framed == int(s.touchdown - s.takeoff) && bad < 0, n + "E leg-tail contacts 0 on every air feed (" + std::to_string(framed) +
                                                                   " tail feeds, source CONT > 0 on " + std::to_string(srcNonZero) + ")",
              "bad " + std::to_string(bad));
    }
    // F
    {
        // per bone: the published step over touchdown .. +0.4 s (60 Hz feeds) <= that bone's largest consecutive-frame step in the
        // source Jumps / Lands clips (30 fps frames, i.e. twice the feed interval)
        float worst = 0.0f, m = 0.0f, ratio = 0.0f;
        int wb = -1, nSteps = 0, rb = -1, rf = -1;
        // A published rotation step is measured as 2 acos(dot) of two float32 quaternions; the smallest non-zero value of that measure
        // is one ULP below 1.0: 2 acos(1 - 2^-24) = 0.0396 deg. A bone that is static in every source clip (bound 0, e.g. RightHeel of
        // the Jumps / Lands clips: 0.0000 deg between the clips of the two databases) shows exactly that after the blend stack's weighted sum and
        // renormalisation of identical quaternions since the live outgoing-clip crossfade (bca1b73). The allowance is that single
        // quantum (0.05 deg is below the 0.0792 deg of two of them) and applies ONLY to a bone whose source bound is exactly 0; every
        // bone with a source step above 0 (even 0.04 deg, e.g. LeftHeel) is judged against its own bound exactly as before.
        constexpr float kStepQuantumDeg = 0.05f;
        for (size_t i = s.touchdown; i < s.touchdown + 24 && i < r.size(); ++i)
        {
            m = std::max(m, r[i].step);
            nSteps += r[i].steps.empty() ? 0 : 1;
            for (size_t b = 2; b < r[i].steps.size() && b < aBound.size(); ++b)
            {
                const float ex = r[i].steps[b] - (aBound[b] > 0.0f ? aBound[b] : kStepQuantumDeg);
                if (aBound[b] >= 1.0f && r[i].steps[b] / aBound[b] > ratio)
                {
                    ratio = r[i].steps[b] / aBound[b];
                    rb = static_cast<int>(b);
                    rf = static_cast<int>(i - s.touchdown);
                }
                if (wb < 0 || ex > worst)
                {
                    worst = ex;
                    wb = static_cast<int>(b);
                }
            }
        }
        if (EnvSet("UTPP_AIR_DEBUG"))
            for (size_t i = s.touchdown - 3; i < s.touchdown + 24 && i < r.size(); ++i)
                std::printf("  dbg %s feed %zu step %.2f clip %d @%.3f db %s framed %d jl %d%d\n", s.name.c_str(), i, r[i].step, r[i].clip,
                            r[i].clipTime, DbN(db, r[i].db).c_str(), r[i].framed ? 1 : 0, r[i].d.justLandedLight ? 1 : 0,
                            r[i].d.justLandedHeavy ? 1 : 0);
        Check(nSteps == 24 && wb >= 0 && worst <= 0.0f,n + "F landing continuity: per-bone step touchdown..+0.4 s <= the bone's source clip step",
              "max step " + F(m, 2) + " deg, worst bone " + (wb >= 0 ? std::string(db.BoneName(uint32_t(wb))) : "-") + " excess " + F(worst, 2) +
                  " deg (bound " + (wb >= 0 ? F(aBound[size_t(wb)], 2) : "-") + ")");
        // verifier info 3: the bound above is a 30 fps frame step (twice the 60 Hz feed interval), lenient by 2x. Tight form: the
        // published 60 Hz step <= 0.55 x that bound (half = the source clip's own step per 60 Hz feed, +10% for the decaying
        // inertialization offset); bones with a source step < 1 deg skipped. Fix: max 0.50 (source motion after the blend).
        // Mutation 8 (no deficit bypass) used to show up here (1.80 run_jump / 1.34 drop_2m on the touchdown feed); since the live crossfade of
        // the outgoing clips (bca1b73) this ratio no longer reacts to it and the bypass-consumption checks below detect it.
        Check(rb >= 0 && ratio <= 0.55f, n + "F landing continuity at 60 Hz: per-bone step touchdown..+0.4 s <= 0.55 x the source clip 30 fps step",
              "max ratio " + F(ratio, 3) + " bone " + (rb >= 0 ? std::string(db.BoneName(uint32_t(rb))) : "-") + " at touchdown +" +
                  std::to_string(rf) + " feeds");
        // the deficit bypass behind F (verifier minor 2): consumed exactly once, by the touchdown feed's blend request; never
        // armed outside the 0.3 s justLanded window
        int consumed = 0, consumedAt = -1, armedOutside = -1;
        for (size_t i = s.takeoff; i < s.touchdown + 24 && i < r.size(); ++i)
        {
            if (r[i].d.landBypassConsumed)
            {
                ++consumed;
                consumedAt = static_cast<int>(i);
            }
            if (r[i].d.landBypassArmed && (i < s.touchdown || i >= s.touchdown + 18))
                armedOutside = static_cast<int>(i);
        }
        Check(consumed == 1 && consumedAt == static_cast<int>(s.touchdown) && armedOutside < 0,
              n + "F landing blend deficit bypass consumed once, by the touchdown feed",
              "consumed " + std::to_string(consumed) + " at feed " + std::to_string(consumedAt) + " (touchdown " +
                  std::to_string(s.touchdown) + "), armed outside the window at " + std::to_string(armedOutside));
    }
    // G
    {
        const motiondriver::AirDiag& d = r[s.touchdown - 1].d;
        const bool ok = std::isfinite(d.residual) && std::isfinite(d.maxResidual) && std::fabs(d.animDz) > 0.01f;
        Check(ok, n + "G root vertical RTZ vs capsule reported (residual = anim - capsule, m)",
              "animDz " + F(d.animDz) + " capsuleDz " + F(d.capsuleDz) + " residual " + F(d.residual) + " max " + F(d.maxResidual));
        std::printf("RESIDUAL %s animDz=%.3f capsuleDz=%.3f residual=%.3f maxAbs=%.3f\n", s.name.c_str(), d.animDz, d.capsuleDz, d.residual,
                    d.maxResidual);
    }
}

// ------------------------------------------------------------------ I: child processes (fresh v1 state)
int ChildMain(const std::string& aMode, const std::string& aData, const std::string& aBaseStack, const std::string& aOut)
{
    g_baseDb.assign(aBaseStack.begin(), aBaseStack.end());
    Rig rw;
    const motiondriver::Paths pw = DataPaths(aData, "woman");
    auto db = LoadMotion(pw.files.db, rw);
    if (!db)
        return 3;
    motiondriver::SetPathsOverride(0, pw);
    motiondriver::SetRigBody(0);
    const bool hooks = aMode == "airoff";
    motionhost::SetMotionHooks(hooks ? &kHooks : nullptr);
    if (!ResetBase(rw))
        return 4;
    const Scenario s = Make("drop_4m", 3.8f, 0.0f, 4.0f);
    FILE* o = nullptr;
    if (fopen_s(&o, aOut.c_str(), "w") != 0 || !o)
        return 5;
    for (const Sf& f : s.fr)
    {
        if (hooks)
            motiondriver::SetAirFeed(f.onGround, f.dist, f.fall);
        motionhost::Feed(ToInput(f));
        const bool pub = g_mmPose.Read(g_read);
        std::fprintf(o, "%llu\n", pub ? static_cast<unsigned long long>(SnapHash(g_read)) : 0ull);
    }
    std::fclose(o);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    return 0;
}
bool RunChild(const char* aMode, const std::string& aData, const std::string& aBaseStack, std::vector<std::string>& aLines)
{
    char exe[MAX_PATH];
    GetModuleFileNameA(nullptr, exe, MAX_PATH);
    char tmp[MAX_PATH];
    GetTempPathA(MAX_PATH, tmp);
    const std::string out = std::string(tmp) + "utpp_air_child_" + aMode + "_" + std::to_string(GetCurrentProcessId()) + ".txt";
    std::string cmd = "\"" + std::string(exe) + "\" --child " + aMode + " \"" + aData + "\" \"" + aBaseStack + "\" \"" + out + "\"";
    STARTUPINFOA si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    std::vector<char> buf(cmd.begin(), cmd.end());
    buf.push_back(0);
    if (!CreateProcessA(nullptr, buf.data(), nullptr, nullptr, FALSE, 0, nullptr, nullptr, &si, &pi))
        return false;
    WaitForSingleObject(pi.hProcess, 300000);
    DWORD code = 1;
    GetExitCodeProcess(pi.hProcess, &code);
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    std::ifstream in(out);
    std::string l;
    while (std::getline(in, l))
        aLines.push_back(l);
    in.close();
    DeleteFileA(out.c_str());
    return code == 0 && !aLines.empty();
}
} // namespace

int main(int argc, char** argv)
{
    if (argc == 6 && std::string(argv[1]) == "--child")
        return ChildMain(argv[2], argv[3], argv[4], argv[5]);
    if (argc < 3)
    {
        std::printf("usage: utpp_motion_driver_air_test <data dir> <base database.utppmm>\n");
        return 2;
    }
    std::printf("air mutation build: %d\n", UTPP_MOTION_DRIVER_AIR_MUTATION_TEST);
    const std::string data = argv[1], v1 = argv[2];
    g_baseDb.assign(v1.begin(), v1.end());
    Rig rw;
    const motiondriver::Paths pw = DataPaths(data, "woman");
    auto db = LoadMotion(pw.files.db, rw);
    Check(db != nullptr, "woman motion database loaded", pw.files.db);
    Check(std::ifstream(v1).good(), "base database present", v1);
    if (!db || g_fail)
    {
        std::printf("RESULT pass=%d fail=%d\n", g_pass, g_fail);
        return 1;
    }
    const std::vector<float> bound = SourceStepBound(*db, "Jumps", "Lands_");
    std::printf("source clip step bound (Jumps + Lands databases, consecutive frames): max over bones %.2f deg\n",
                bound.empty() ? 0.0 : static_cast<double>(*std::max_element(bound.begin(), bound.end())));

    motiondriver::SetPathsOverride(0, pw);
    motiondriver::SetRigBody(0);
    motionhost::SetMotionHooks(EnvSet("UTPP_AIR_RAW") ? &kHooksRaw : &kHooks);
    Check(ResetBase(rw), "base armed");
    // load the bundle (the first enabled Step starts the worker)
    motiondriver::SetEnabled(true);
    {
        Sf f;
        (void)motiondriver::Step(ToInput(f), 1u, kDt);
        motiondriver::WaitLoader();
    }
    Check(motiondriver::Status().find(" air:owns=motion ") != std::string::npos, "H status: motion owns air (air:owns=motion)");

    // hood_jump_off (vehicle batch, J1): a run jump (vz0 5.66 m/s = the recorded CP take-off) off a 1.0 m car hood lands at ~7.7 m/s:
    // Heavy under the unscaled 700 cm/s rule (the all-fours landing of the recording), Light at the game gravity
    const Scenario sc[] = {Make("stand_jump", 0.0f, 4.5f, 0.0f), Make("run_jump", 3.8f, 4.5f, 0.0f), Make("drop_1m2", 3.8f, 0.0f, 1.2f),
                           Make("drop_4m", 3.8f, 0.0f, 4.0f), Make("hood_jump_off", 3.8f, 5.66f, 1.0f)};
    Check(std::fabs(motiontraj::HeavyLandThresholdCmS(kG) - HeavyLandCmSAtG()) <= 0.5f,
          "J1 precondition: motiontraj::HeavyLandThresholdCmS(kG 16) == 700 sqrt(16 / 9.81) (2.5 m drop)",
          F(motiontraj::HeavyLandThresholdCmS(kG), 1) + " vs " + F(HeavyLandCmSAtG(), 1));
    Check(!sc[0].heavy && !sc[1].heavy && !sc[2].heavy && sc[3].heavy && !sc[4].heavy,
          "scenario landing weights light, light, light, heavy, light (hood jump-off)");
    for (const Scenario& s : sc)
    {
        Prime(s);
        const std::vector<Out> r = Run(s, true);
        size_t ownFeeds = 0, own = 0;
        for (size_t i = 0; i < r.size(); ++i)
            if (s.fr[i].airOwns)
            {
                ++ownFeeds;
                own += r[i].stepped ? 1 : 0;
            }
        Check(ownFeeds > 0 && own == ownFeeds, "[" + s.name + "] H motion owns air: motion steps every feed of the AirOwns span",
              std::to_string(own) + "/" + std::to_string(ownFeeds));
        CheckScenario(s, r, *db, bound);
    }
    // K (vehicle batch, J2): runtime-faithful probing (no probe on grounded / rising feeds). A flat run jump and a standing jump
    // (vz0 5.66 m/s = the recorded CP take-off) know their floor from the take-off plane seed: on every rising airborne feed after
    // the first, |TimeToLand - true| <= 0.1 s, and the chooser never offers PSD_Dense_Jumps_Far (TTL >= 1.2 s rows) while the true
    // time to land is below 1.0 s. Drop guard: a walk-off from a 1.5 m ledge (PSM Stand -> Fall 14, no jump detail) keeps today's
    // behaviour: no seeded floor, every floor used is a probe hit (dead-reckoned) within 0.1 m of the truth or none, and C holds.
    // Mutation 12 (no seed) fails K, mutation 13 (seed also for drops) fails the drop guard.
    {
        uint64_t farMask = 0;
        for (uint32_t d = 0; d < db->NumDatabases() && d < 64; ++d)
            if (std::string_view(db->DbName(d)).find("Jumps_Far") != std::string_view::npos)
                farMask |= 1ull << d;
        Check(farMask != 0, "K precondition: the database has a Jumps_Far database");
        for (const Scenario& s : {MakeRuntimeProbe("rt_run_jump", 4.0f, 5.66f, 0.0f), MakeRuntimeProbe("rt_stand_jump", 0.0f, 5.66f, 0.0f)})
        {
            Prime(s);
            const std::vector<Out> r = Run(s, true);
            float worst = 0.0f;
            int rising = 0, worstAt = -1, farAt = -1, seeded = 0;
            for (size_t i = s.takeoff; i < s.touchdown; ++i)
            {
                const Sf& f = s.fr[i];
                if (f.ttlTrue < 1.0f && (r[i].d.chooserDbMask & farMask) != 0 && farAt < 0)
                    farAt = static_cast<int>(i);
                seeded += r[i].d.takeoffSeed ? 1 : 0;
                if (i == s.takeoff || f.vz <= 0.0f)
                    continue;
                ++rising;
                const float e = std::fabs(r[i].d.timeToLand - f.ttlTrue);
                if (e > worst)
                {
                    worst = e;
                    worstAt = static_cast<int>(i);
                }
            }
            Check(rising > 0 && worst <= 0.1f, "[" + s.name + "] K TimeToLand within 0.1 s on the rising feeds (take-off plane seed)",
                  std::to_string(rising) + " rising feeds, max err " + F(worst) + " at feed " + std::to_string(worstAt) +
                      (worstAt >= 0 ? " (ttl " + F(r[size_t(worstAt)].d.timeToLand) + " true " + F(s.fr[size_t(worstAt)].ttlTrue) + ")" : "") +
                      ", seeded feeds " + std::to_string(seeded));
            Check(farAt < 0, "[" + s.name + "] K no Jumps_Far in the chooser databases while the true time to land < 1.0 s",
                  farAt < 0 ? "" : "feed " + std::to_string(farAt) + " ttl " + F(r[size_t(farAt)].d.timeToLand) + " true " + F(s.fr[size_t(farAt)].ttlTrue));
            CheckScenario(s, r, *db, bound); // A-G on the runtime-faithful feeds
        }
        {
            const Scenario s = MakeRuntimeProbe("rt_drop_1m5", 3.8f, 0.0f, 1.5f);
            Prime(s);
            const std::vector<Out> r = Run(s, true);
            int bad = -1, seeded = 0, none = 0;
            for (size_t i = s.takeoff; i < s.touchdown; ++i)
            {
                const float used = r[i].d.distanceUsed;
                seeded += r[i].d.takeoffSeed ? 1 : 0;
                none += used < 0.0f ? 1 : 0;
                if (bad < 0 && (r[i].d.takeoffSeed || (used >= 0.0f && std::fabs(used - s.fr[i].distTrue) > 0.1f)))
                    bad = static_cast<int>(i);
            }
            Check(bad < 0 && seeded == 0 && none > 0, "[" + s.name + "] K drop guard: a walk-off never uses the take-off plane as its floor",
                  "first bad feed " + std::to_string(bad) + (bad >= 0 ? " used " + F(r[size_t(bad)].d.distanceUsed) + " true " + F(s.fr[size_t(bad)].distTrue) : "") +
                      ", seeded " + std::to_string(seeded) + ", no-floor feeds " + std::to_string(none));
            // (no CheckScenario: C needs a floor on every descending feed, which a walk-off has only from its first probe hit - today's
            // behaviour, kept on purpose)
        }
    }
    // V (vehicle batch, CAR-1 / CAR-2 / J4 through v2): after the vehicle-bit falling edge the v1 exit window holds the body on MM, so
    // v2 must step every feed of the exit animation (no skip 'gate': the exit workspot latched at the edge and the NotInBase detail 0
    // are held, not released) and the chooser stays in the Stand stance on the ground (CAR-1 step 4: detail 0 -> Stand, no crouch
    // flash). The unmount with a ballistic arc (vz0 +6 m/s, 0.75 s, J4) is owned by v2 air: from the 2nd airborne feed InAir with
    // Jumps rows, then a light landing (Lands_Light) at ~6 m/s. Mutation UTPP_VEHEXIT_MUTATION=2 (the window ends on the post-hold
    // gate: the latched workspot releases) fails the workspot case.
    {
        for (const Scenario& s : {MakeVehicleExit("exit_ws_1s3", 1.30f, true, 0.0f), MakeVehicleExit("exit_bike_0s35", 0.35f, false, 0.0f)})
        {
            Prime(s);
            const std::vector<Out> r = Run(s, true);
            int notStepped = -1, gateSkips = 0, crouch = -1, air = -1, rideStepped = 0;
            std::string skip;
            for (size_t i = 0; i < s.takeoff; ++i)
                rideStepped += r[i].stepped ? 1 : 0;
            for (size_t i = s.takeoff; i < r.size(); ++i)
            {
                if (!r[i].stepped)
                {
                    notStepped = notStepped < 0 ? static_cast<int>(i) : notStepped;
                    gateSkips += r[i].skip == "gate" ? 1 : 0;
                    skip = r[i].skip;
                    continue;
                }
                if (r[i].d.stance != 0 && crouch < 0)
                    crouch = static_cast<int>(i);
                if (r[i].d.movementMode != 0 && air < 0)
                    air = static_cast<int>(i);
            }
            const std::string d = "edge feed " + std::to_string(s.takeoff) + ", exit animation until " + std::to_string(s.touchdown) +
                                  ": first not stepped " + std::to_string(notStepped) + " (skip '" + skip + "', gate skips " +
                                  std::to_string(gateSkips) + "), first non-Stand stance " + std::to_string(crouch) + ", first non-ground mode " +
                                  std::to_string(air) + ", ride feeds stepped " + std::to_string(rideStepped) + "/" + std::to_string(s.takeoff);
            Check(notStepped < 0 && gateSkips == 0, "[" + s.name + "] V motion steps every feed of the exit window and after it (no skip 'gate')", d);
            Check(notStepped < 0 && crouch < 0 && air < 0, "[" + s.name + "] V CAR-1 step 4: chooser stance Stand on the ground through the exit (detail 0 / latched workspot)", d);
        }
        {
            const Scenario s = MakeVehicleExit("exit_arc_vz6", 0.0f, false, 6.0f);
            Prime(s);
            const std::vector<Out> r = Run(s, true);
            int bad = -1, airN = 0;
            for (size_t i = s.takeoff + 1; i < s.touchdown && bad < 0; ++i, ++airN)
                if (!r[i].stepped || r[i].d.movementMode != 1 || DbN(*db, r[i].db).find("Jumps") == std::string::npos)
                    bad = static_cast<int>(i);
            int landN = 0, badL = -1, heavyN = 0;
            for (size_t i = s.touchdown; i < r.size(); ++i)
            {
                heavyN += r[i].d.justLandedHeavy ? 1 : 0;
                if (!r[i].d.justLandedLight)
                    continue;
                ++landN;
                if (badL < 0 && DbN(*db, r[i].db).find("Lands_Light") == std::string::npos)
                    badL = static_cast<int>(i);
            }
            const std::string d = "airborne feeds " + std::to_string(s.touchdown - s.takeoff) + ", first bad " + std::to_string(bad) +
                                  (bad >= 0 ? " (stepped " + std::to_string(r[size_t(bad)].stepped) + " mode " + std::to_string(r[size_t(bad)].d.movementMode) +
                                                  " db " + DbN(*db, r[size_t(bad)].db) + " skip '" + r[size_t(bad)].skip + "')"
                                            : std::string()) +
                                  "; landing: light feeds " + std::to_string(landN) + " heavy " + std::to_string(heavyN) + " first non-Lands_Light " +
                                  std::to_string(badL) + " landVz " + F(r[s.touchdown].d.landVzCmS, 1) + " cm/s";
            Check(!s.heavy && bad < 0 && airN > 0, "[" + s.name + "] V J4 unmount + ballistic arc: motion InAir with Jumps rows from the 2nd airborne feed", d);
            Check(r[s.touchdown].d.justLandedLight && landN == 18 && heavyN == 0 && badL < 0,
                  "[" + s.name + "] V J4 unmount + ballistic arc: a light landing (Lands_Light, 18 feeds, never heavy)", d);
        }
    }
    // J (next): the real runtime path of main 22d4062. The air fields travel in FeedInput (UTPPFeedAir -> in.haveAir /
    // onGround / distToGround / psmFall) and reach v2 only through motiondriver::FeedAirFromInput (the Main.cpp call site right
    // before motionhost::Feed). The drop's first
    // airborne feed still says Stand in the PSM, so only the forwarded air feed makes it InAir.
    {
        const Scenario& s = sc[2];
        Prime(s);
        int bad = -1, notStepped = 0, noFeed = 0, runtimeOwns = 0, consumed = 0, consumedAt = -1;
        for (size_t i = 0; i < s.fr.size(); ++i)
        {
            const Sf& f = s.fr[i];
            motionhost::FeedInput in = ToInput(f);
            in.haveAir = f.airFeed;
            in.onGround = f.onGround;
            in.distToGround = f.dist;
            in.psmFall = f.fall;
            motiondriver::FeedAirFromInput(in);
            g_stepped = false;
            motionhost::Feed(in);
            runtimeOwns += motionhost::AirOwnsForTest() ? 1 : 0;
            if (g_stepped && motiondriver::AirDiagnostics().landBypassConsumed)
            {
                ++consumed;
                consumedAt = static_cast<int>(i);
            }
            if (i + 5 < s.takeoff || i >= s.touchdown + 3)
                continue;
            const motiondriver::AirDiag d = motiondriver::AirDiagnostics();
            notStepped += g_stepped ? 0 : 1;
            noFeed += g_stepped && !d.feedValid ? 1 : 0;
            const bool air = i >= s.takeoff && i < s.touchdown;
            if (bad < 0 && g_stepped && (d.movementMode == 1) != air)
                bad = static_cast<int>(i);
        }
        char d[200];
        std::snprintf(d, sizeof d, "takeoff %zu touchdown %zu: first wrong mode feed %d, not stepped %d, stepped without air feed %d, runtime AirOwns feeds %d",
                      s.takeoff, s.touchdown, bad, notStepped, noFeed, runtimeOwns);
        Check(runtimeOwns > 0, "J precondition: the runtime AirState (base) owns the drop from the same FeedInput air fields", d);
        Check(notStepped == 0 && noFeed == 0, "J FeedAirFromInput: every motion Step around the drop consumed the forwarded UTPPFeedAir fields", d);
        Check(notStepped == 0 && bad < 0, "J MovementMode InAir exactly on the airborne feeds (first one: PSM still Stand) through the runtime call site", d);
        // verifier minor (next): the landing deficit bypass is reachable on this path
        Check(consumed == 1 && consumedAt == static_cast<int>(s.touchdown),
              "J landing blend deficit bypass consumed once, by the touchdown feed, on the runtime path",
              "consumed " + std::to_string(consumed) + " at feed " + std::to_string(consumedAt) + " (touchdown " + std::to_string(s.touchdown) + ")");
    }
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();

    // I: v1 bit-identical with mm.motionDriver off (child processes)
    {
        std::vector<std::string> a, b, c;
        const bool ok = RunChild("nohooks", data, v1, a) && RunChild("nohooks", data, v1, b) && RunChild("airoff", data, v1, c);
        size_t pub = 0;
        for (const std::string& l : a)
            pub += l != "0";
        Check(ok && a == b && pub > 0, "I precondition: two runs without hooks identical", std::to_string(pub) + " published");
        size_t diff = 0;
        for (size_t i = 0; i < std::min(a.size(), c.size()); ++i)
            diff += a[i] != c[i];
        Check(ok && a.size() == c.size() && diff == 0, "I mm.motionDriver off with SetAirFeed: published snapshots == no hooks",
              std::to_string(diff) + " differing feeds of " + std::to_string(c.size()));
    }
    std::printf("RESULT pass=%d fail=%d\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
