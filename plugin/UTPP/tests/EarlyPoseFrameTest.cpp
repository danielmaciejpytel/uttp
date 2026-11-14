// One whole frame of each pose owner through the real hooks, at unit level: the main mapping job (MappingDetour -> early write of the
// Mapping owner), a deformer mapping job (native helper copies), then the FK call (FkDetour -> FK-side layers -> skin helper Evaluate).
// The game is replaced by fake rig / mapping / descriptor memory and fake original functions; the bone data is the recorded woman fixture
// (Build/test-data/body_helper_woman) with its approved helper bundle.
// Every frame kind runs under the Mapping owner and, as the control, under the FK owner; the skin helper recompute must see the same
// core writers (coverage row), take the same route and leave the same records under both owners:
//   normal locomotion, vehicle transition (weight 0), vault episode (window open, live motion write), reload with the contact solve (authored Pistol2H clip and
//   rifle game clip), a frame whose main job is not admitted, a frame without a main job, and a main job on another worker thread.
// usage: EarlyPoseFrameTest <woman helper fixture dir>
#include "../src/BodyHelperRuntime.hpp"
#include "../src/EarlyPoseRuntime.hpp"
#include "../src/FkHook.hpp"
#include "../src/PoseHook.hpp"
#include "../src/State.hpp"
#include "../src/mm/CoreWriteRecord.hpp"
#include "../src/mm/GameIntegration.hpp"
#include "../src/mm/NativeReloadContactsRuntime.hpp"
#include "../src/mm/VaultTorso.hpp"
#include "../src/mm/VehicleTransition.hpp"
#include "../src/ragdoll/RagdollSlotProbe.hpp"

#include <Windows.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// The ragdoll slot probe needs the game; its two FK-side entry points are inert here (no ragdoll episode runs in this test).
namespace utpp::ragdollslot
{
ragdoll::RdKeep& Keep() { static ragdoll::RdKeep keep; return keep; }
void OnPlayerFk(const uint8_t*, void*, int64_t) {}
} // namespace utpp::ragdollslot

using namespace utpp;
namespace fs = std::filesystem;
namespace br = utpp::bodyhelperruntime;
namespace cw = utpp::mm::corewrite;
namespace er = utpp::earlyruntime;
namespace vt = utpp::mm::vehtrans;
namespace rcr = utpp::mm::reload_contacts_runtime;

namespace
{
int g_checks = 0, g_failures = 0;
void check(bool ok, const std::string& label)
{
    ++g_checks;
    if (!ok) { ++g_failures; std::printf("FAIL %s\n", label.c_str()); }
}
int64_t Now() { LARGE_INTEGER q{}; QueryPerformanceCounter(&q); return q.QuadPart; }
int64_t Hz() { LARGE_INTEGER q{}; QueryPerformanceFrequency(&q); return q.QuadPart; }

// ---- fixture: recorded merged woman rig (names, parents) + one recorded frame (12 floats per bone) + the helper bundle files
struct Fixture
{
    std::vector<uint64_t> names; std::vector<int16_t> parents; std::vector<float> bones;
    std::string program, binding;
};
std::string ReadAll(const fs::path& p) { std::ifstream i(p, std::ios::binary); std::ostringstream o; o << i.rdbuf(); return o.str(); }
bool LoadFixture(const std::string& dir, Fixture& fx)
{
    std::ifstream in(dir + "/frames.txt");
    if (!in) return false;
    std::string tag, label; uint32_t rig = 0, count = 0; int version = 0;
    in >> tag >> version >> tag >> rig;
    fx.names.resize(rig); fx.parents.resize(rig); fx.bones.assign(size_t(rig) * 12, 0.f);
    for (uint32_t i = 0; i < rig; ++i)
    {
        uint32_t idx = 0; std::string hash, name; int parent = 0;
        in >> tag >> idx >> hash >> parent >> name;
        fx.names[i] = std::strtoull(hash.c_str(), nullptr, 16); fx.parents[i] = int16_t(parent);
    }
    bool take = false;
    while (in >> tag)
    {
        if (tag == "FRAME") { in >> label >> count; take = label == "stand_armed.s1"; }
        else if (tag == "R")
        {
            uint32_t idx = 0; in >> idx; float v[12];
            for (auto& x : v) in >> x;
            if (take && idx < rig) std::memcpy(&fx.bones[size_t(idx) * 12], v, sizeof v);
        }
    }
    fx.program = ReadAll(fs::path(dir) / "program.ir"); fx.binding = ReadAll(fs::path(dir) / "binding.dat");
    return !fx.program.empty() && !fx.binding.empty();
}

// ---- fake game memory (only the fields the hooks read: see PoseHook.cpp / FkHook.cpp / State.hpp)
struct NameArray { const uint64_t* entries; uint32_t capacity; uint32_t size; }; // DynArray<CName> layout
void PutNames(uint8_t* aBase, size_t aOff, const std::vector<uint64_t>& aNames)
{
    const NameArray a{aNames.data(), uint32_t(aNames.size()), uint32_t(aNames.size())};
    std::memcpy(aBase + aOff, &a, sizeof a);
}
template <class T> void Put(uint8_t* aBase, size_t aOff, T aValue) { std::memcpy(aBase + aOff, &aValue, sizeof aValue); }

alignas(16) uint8_t g_metaRig[0x40]{};   // merged rig: names @0x20 (the player rig pointer of the hooks)
alignas(16) uint8_t g_desc[0x40]{};      // output descriptor: bones @0x00, count @0x0C, tag @0x18
constexpr uint64_t kTag = 0xABCDEF01ull;
std::vector<float> g_bones;              // the descriptor's bone buffer
Fixture g_fx;

// One mapping job of the frame: a source rig (names @0x50), a pair table (pairs @0x40, per-LOD counts @0x50), the context (table @0x30,
// descriptor @0x38), the graph (source rig @0x40; the test keeps its job pointer @0x00) and the source pose (bones @0x30).
struct Job
{
    std::vector<uint64_t> names; std::vector<uint32_t> pairs; std::vector<float> source;
    uint8_t rig[0x60]{}; uint8_t table[0x60]{}; uint8_t context[0x40]{}; uint8_t graph[0x48]{}; uint8_t sourceDesc[0x40]{};
    uint16_t counts[8]{};
    void Wire()
    {
        PutNames(rig, 0x50, names);
        Put<const uint32_t*>(table, 0x40, pairs.data());
        for (auto& c : counts) c = uint16_t(pairs.size() / 2);
        Put<const uint16_t*>(table, 0x50, counts);
        Put<const uint8_t*>(context, 0x30, table);
        Put<uint8_t*>(context, 0x38, g_desc);
        Put<Job*>(graph, 0x00, this);
        Put<const uint8_t*>(graph, 0x40, rig);
        Put<const float*>(sourceDesc, 0x30, source.data());
    }
};
// The game's merge: every pair copies the source record to the destination record.
void* FakeMapping(void* aGraph, void*, void*, unsigned int)
{
    Job* j = nullptr; std::memcpy(&j, aGraph, sizeof j);
    for (size_t i = 0; i + 1 < j->pairs.size(); i += 2)
        std::memcpy(&g_bones[size_t(j->pairs[i + 1]) * 12], &j->source[size_t(j->pairs[i]) * 12], 48);
    return nullptr;
}
uint64_t g_fkCalls = 0;
void* FakeFk(void*, void*) { ++g_fkCalls; return nullptr; }
void RunJob(Job& aJob) { MappingDetour(aJob.graph, aJob.context, aJob.sourceDesc, 0); }

// ---- the motion pose: core bones of the merged rig (not helpers), the fixture pose with the left upper arm turned by 8 degrees
const char* const kCore[] = {"Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck", "Neck1", "Head", "LeftShoulder", "LeftArm",
                             "LeftForeArm", "LeftHand", "WeaponLeft", "RightShoulder", "RightArm", "RightForeArm", "RightHand",
                             "WeaponRight", "LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"};
const char* const kAnchorsOnly[] = {"reference_joint", "LeftEye", "RightEye"}; // with WeaponLeft / WeaponRight above: the 5 main anchors
std::vector<uint32_t> g_coreDst;   // destination index per core bone
std::vector<int> g_coreParent;     // database parent per core bone (-1 = root of the subset)
std::vector<uint64_t> g_catalog;   // required helper catalog of the bundle (receipt + forbidden early names)
int32_t Find(uint64_t aHash)
{
    for (size_t i = 0; i < g_fx.names.size(); ++i) if (g_fx.names[i] == aHash) return int32_t(i);
    return -1;
}
void RotateZ(float* q, float deg)
{
    const float h = deg * 3.14159265f / 360.f; const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float s = std::sin(h), c = std::cos(h);
    q[0] = c * x + s * y; q[1] = c * y - s * x; q[2] = c * z + s * w; q[3] = c * w - s * z;
}
mm::integ::PoseSnapshot MotionPose(float aWeight, int64_t aTicks)
{
    mm::integ::PoseSnapshot s;
    s.weight = aWeight; s.numBones = uint32_t(g_coreDst.size()); s.firstBone = 0;
    s.source = mm::integ::kPoseSourceDriver; s.rig = reinterpret_cast<uint64_t>(g_metaRig); s.publishTicks = aTicks; s.sequence = 1;
    s.bodyType = 0; // woman
    for (uint32_t b = 0; b < mm::integ::kMaxMMBones; ++b) s.gameIndex[b] = -1;
    for (uint32_t b = 0; b < s.numBones; ++b)
    {
        const float* r = &g_fx.bones[size_t(g_coreDst[b]) * 12];
        float* d = &s.pose[size_t(b) * 7];
        d[0] = r[0]; d[1] = r[1]; d[2] = r[2]; d[3] = r[4]; d[4] = r[5]; d[5] = r[6]; d[6] = r[7];
        if (std::string(kCore[b]) == "LeftArm") RotateZ(d + 3, 8.f);
        s.gameIndex[b] = int32_t(g_coreDst[b]);
        if (b >= 2) s.upperBits[b >> 5] |= 1u << (b & 31u); // upper body = everything above Spine (arms, chest, head, weapons)
    }
    return s;
}

// ---- jobs: the main job (core + anchors, the game's pass-0 pose = the fixture) and the deformer job (native helper copies)
Job g_main, g_helper;
bool BuildJobs()
{
    for (const char* n : kCore)
    {
        const int32_t d = Find(mm::integ::CNameHash(n));
        if (d < 0) { std::printf("core bone %s not in the fixture rig\n", n); return false; }
        g_main.names.push_back(mm::integ::CNameHash(n)); g_coreDst.push_back(uint32_t(d));
    }
    for (const char* n : kAnchorsOnly)
    {
        if (Find(mm::integ::CNameHash(n)) < 0) { std::printf("anchor %s not in the fixture rig\n", n); return false; }
        g_main.names.push_back(mm::integ::CNameHash(n));
    }
    for (size_t i = 0; i < g_main.names.size(); ++i)
    {
        const int32_t d = Find(g_main.names[i]);
        g_main.pairs.push_back(uint32_t(i)); g_main.pairs.push_back(uint32_t(d));
        g_main.source.insert(g_main.source.end(), &g_fx.bones[size_t(d) * 12], &g_fx.bones[size_t(d) * 12] + 12);
    }
    for (size_t b = 0; b < g_coreDst.size(); ++b)
    {
        int p = g_fx.parents[g_coreDst[b]], found = -1;
        while (p >= 0 && found < 0)
        {
            for (size_t k = 0; k < b; ++k) if (int(g_coreDst[k]) == p) found = int(k);
            p = g_fx.parents[size_t(p)];
        }
        g_coreParent.push_back(found);
    }
    for (uint64_t h : g_catalog)
    {
        const int32_t d = Find(h);
        if (d < 0) { std::printf("catalog helper %016llx not in the rig\n", static_cast<unsigned long long>(h)); return false; }
        if (std::find(g_main.names.begin(), g_main.names.end(), h) != g_main.names.end())
        { std::printf("catalog helper %016llx is a core bone of the test\n", static_cast<unsigned long long>(h)); return false; }
        g_helper.pairs.push_back(uint32_t(g_helper.names.size())); g_helper.pairs.push_back(uint32_t(d));
        g_helper.names.push_back(h);
        g_helper.source.insert(g_helper.source.end(), &g_fx.bones[size_t(d) * 12], &g_fx.bones[size_t(d) * 12] + 12);
    }
    g_main.Wire(); g_helper.Wire();
    return true;
}

// ---- arming (disarmed -> owner -> receipt plan, helper plans, early policy -> armed), as the plugin does at arm time
// The owner goes through the plugin's arm-path choice (ArmOwnerForRig); aEarlyCatalog replaces the early helper catalog (nullptr = the
// bundle's catalog), aChoice receives the choice. True = everything armed and the requested owner took the rig.
er::ArmChoice g_lastChoice;
bool Arm(er::Owner aOwner, const fs::path& aWork, const std::vector<uint64_t>* aEarlyCatalog = nullptr)
{
    g_hot.armed.store(false);
    er::Disarm(); shadowcopy::Disarm();
    if (!er::SelectOwner(aOwner)) return false;
    g_bones = g_fx.bones;
    Put<float*>(g_desc, 0x00, g_bones.data()); Put<uint32_t>(g_desc, 0x0C, uint32_t(g_fx.names.size())); Put<uint64_t>(g_desc, 0x18, kTag);
    PutNames(g_metaRig, 0x20, g_fx.names);
    const uint64_t rig = reinterpret_cast<uint64_t>(g_metaRig);
    g_hot.playerDesc.store(reinterpret_cast<uint64_t>(g_desc)); g_hot.playerRig.store(rig);
    g_hot.playerBones.store(uint32_t(g_fx.names.size())); g_hot.trackA.store(kTag); g_hot.trackB.store(0);
    g_hot.matchMode.store(0); g_hot.fault.store(false); g_hot.mmActive.store(true);
    g_hot.perspectiveWriteMaxAge.store(Hz() * 30); g_hot.mmStaleTicks.store(0);
    bool ok = br::Load(aWork.wstring()) && br::ApplyEnabled();
    ok = posediag::PrepareCycleCensus(rig, g_fx.names, g_catalog) && ok;
    ok = br::PrepareArm(rig, g_fx.parents, g_fx.names, g_catalog) && ok;
    ok = er::ArmOwnerForRig(true, rig, uint32_t(g_fx.names.size()), kTag, 0, g_fx.names, aEarlyCatalog ? *aEarlyCatalog : g_catalog,
                            g_lastChoice) && ok;
    ok = ok && g_lastChoice.owner == aOwner && !g_lastChoice.fellBack;
    shadowcopy::Arm(rig, g_fx.names, g_fx.parents);
    mm::vaulttorso::rt::OnArmed(g_metaRig, g_fx.names, g_fx.parents);
    g_hot.armed.store(true);
    return ok;
}

// ---- one frame: the game pose, the motion pose, the main job (optional, optionally on another thread), the deformer job, the FK call
struct FrameSpec
{
    float weight = 1.f; bool mmActive = true; bool mainJob = true; bool mainOnOtherThread = false; bool rigMismatch = false;
    bool vault = false; int32_t overlay = 0; bool reloadEligible = false; bool authored = false;
    bool endTransitionBeforeFk = false; // the game thread ends the vehicle transition between the mapping jobs and the FK call
    bool firstPerson = false;           // the camera withdraws the third-person write window
};
struct FrameOut
{
    uint32_t record = 0;                     // core write record the helper evaluation saw
    std::array<uint64_t, br::kWriterRows> rowCalls{};   // coverage delta per writer row (any outcome)
    std::array<uint64_t, size_t(br::Outcome::Count)> outcome{};   // coverage delta per outcome (all rows)
    std::vector<float> bones;
    uint64_t earlyApplied = 0, mmApplied = 0, rcOwnerOk = 0, rcCaptured = 0, rcFinish = 0;
};
FrameOut RunFrame(const FrameSpec& f)
{
    FrameOut out;
    g_bones = g_fx.bones; // the pose the game evaluates before the merge of this frame
    const auto cov0 = br::ReadCoverage();
    const auto e0 = er::ReadCounters();
    const uint64_t mm0 = g_hot.mmAppliedCalls.value.load(), rc0 = g_hot.rcOwnerOk.value.load(), cap0 = g_hot.rcCaptured.value.load(),
                   fin0 = g_hot.rcFinishWritten.value.load();
    const int64_t t = Now();
    g_hot.mmActive.store(f.mmActive);
    g_hot.perspectiveWrite.Publish(!f.firstPerson, t);
    auto snap = MotionPose(f.weight, t);
    if (f.rigMismatch) snap.rig += 8;
    snap.traceOverlay = f.overlay; snap.reloadContactsEligible = f.reloadEligible; snap.reloadAuthoredClip = f.authored && f.reloadEligible;
    snap.reloadContactsWeight = f.reloadEligible ? 1.f : 0.f; snap.traceAction = 2;
    g_mmPose.Publish(snap);
    if (f.vault)
    {
        mm::vaulttorso::rt::g_foot.store(true); mm::vaulttorso::rt::g_weight.store(1.f); mm::vaulttorso::rt::g_pubTicks.store(t);
    }
    if (f.mainJob)
    {
        if (f.mainOnOtherThread) { std::thread worker([] { RunJob(g_main); }); worker.join(); }
        else RunJob(g_main);
    }
    RunJob(g_helper);
    if (f.endTransitionBeforeFk) vt::Abort();
    FkDetour(g_desc, g_metaRig);
    out.record = cw::Current();
    if (f.vault) { mm::vaulttorso::rt::g_weight.store(0.f); mm::vaulttorso::rt::g_foot.store(false); }
    const auto cov1 = br::ReadCoverage();
    for (size_t w = 0; w < br::kWriterRows; ++w)
        for (size_t o = 0; o < size_t(br::Outcome::Count); ++o)
        {
            const uint64_t d = cov1.byWriter[w][o] - cov0.byWriter[w][o];
            out.rowCalls[w] += d; out.outcome[o] += d;
        }
    out.bones = g_bones;
    out.earlyApplied = er::ReadCounters().applied - e0.applied;
    out.mmApplied = g_hot.mmAppliedCalls.value.load() - mm0;
    out.rcOwnerOk = g_hot.rcOwnerOk.value.load() - rc0;
    out.rcCaptured = g_hot.rcCaptured.value.load() - cap0;
    out.rcFinish = g_hot.rcFinishWritten.value.load() - fin0;
    return out;
}
std::string Row(const FrameOut& o)
{
    std::string s;
    for (size_t w = 0; w < br::kWriterRows; ++w)
        if (o.rowCalls[w]) s += std::string(s.empty() ? "" : "+") + (w == br::kNoWriter ? "none" : cw::kWriterNames[w]);
    s += " ->";
    for (size_t k = 0; k < size_t(br::Outcome::Count); ++k)
        if (o.outcome[k]) s += std::string(" ") + br::kOutcomeNames[k];
    return s;
}
uint32_t Writers(const FrameOut& o) { return o.record & cw::kWriterMask; }
double MaxDiff(const std::vector<float>& a, const std::vector<float>& b)
{
    double m = 0;
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) m = std::fmax(m, std::fabs(double(a[i]) - double(b[i])));
    return a.size() == b.size() ? m : 1e9;
}

// ---- the frame kinds, run in one sequence per owner (the same sequence for both)
struct Run
{
    FrameOut normal, normalAgain, notAdmitted, noMain, vehicleRef, vehicleWritten, vault, reloadAuthored, reloadRifle, otherThread,
             afterOtherThread, lateFk, noMainAfterSkip, vehicleEnded;
    bool reloadConfig = false;
};
Run RunAll(er::Owner aOwner, const fs::path& aWork)
{
    Run r;
    const std::string L = aOwner == er::Owner::Mapping ? "Mapping: " : "FK: ";
    check(Arm(aOwner, aWork), L + "arm (bundle, receipt plan, helper plans" + (aOwner == er::Owner::Mapping ? ", early policy)" : ")"));
    check(er::CurrentOwner() == aOwner, L + "owner selected");
    check(shadowcopy::Status() == shadowcopy::ArmStatus::Ready,
          L + "the recorded merged rig's shadow skeleton has the expected shape (" + shadowcopy::StatusName() + ")");
    r.normal = RunFrame({});
    r.normalAgain = RunFrame({});
    // the main job is there but not admitted (motion pose of another rig): nothing writes under either owner
    { FrameSpec f; f.rigMismatch = true; r.notAdmitted = RunFrame(f); }
    // no main job at all in this frame (after a written frame): nothing of the previous frame may be counted
    RunFrame({});
    { FrameSpec f; f.mainJob = false; r.noMain = RunFrame(f); }
    // a cycle whose jobs wrote but whose FK call is late (no new main job in between): the next FK call consumes that cycle's receipt,
    // so it carries that cycle's writers once; the FK call after it has none
    g_bones = g_fx.bones; g_mmPose.Publish(MotionPose(1.f, Now())); RunJob(g_main); RunJob(g_helper);
    { FrameSpec f; f.mainJob = false; r.lateFk = RunFrame(f); r.noMainAfterSkip = RunFrame(f); }
    // vehicle transition at motion weight 0, vehicle camera in third person: a reference frame (window open, nothing written), then a
    // frame the mount transition writes
    {
        vt::Abort();
        vt::SetBoneRoles(vt::BuildBoneRoles(uint32_t(g_coreDst.size()), 0, [](uint32_t b) { return std::string_view(kCore[b]); }));
        vt::SetVehicleCapture(true, reinterpret_cast<uint64_t>(g_metaRig));
        vt::SetMountTpp(true);
        FrameSpec f; f.weight = 0.f; f.mmActive = false;
        r.vehicleRef = RunFrame(f);
        const float p0[3] = {10.f, 20.f, 0.f}, p1[3] = {10.8f, 20.f, 0.f};
        const int64_t t0 = Now();
        vt::BeginMount(MotionPose(1.f, t0), p0, 30.f, t0);
        vt::UpdateWorld(p1, 100.f, t0);
        r.vehicleWritten = RunFrame(f);
        // the transition writes again and the game thread ends it before the FK call of that cycle
        { FrameSpec e = f; e.endTransitionBeforeFk = true; r.vehicleEnded = RunFrame(e); }
        vt::Abort(); vt::SetVehicleCapture(false, 0); vt::SetMountTpp(false);
    }
    // vault limiter episode running (weight 1, third person: the weight-0 writer window is open) while the motion pose is still written;
    // the standing fixture chest is above the floor, so the limiter itself corrects nothing (no vault writer bit on either owner)
    { FrameSpec f; f.vault = true; r.vault = RunFrame(f); }
    // reloads with the contact solve: the authored Pistol2H clip and a rifle game clip
    {
        rcr::Prepared p;
        r.reloadConfig = rcr::FillPrepared(p, reinterpret_cast<uint64_t>(g_metaRig), uint32_t(g_coreDst.size()),
                                           MotionPose(1.f, 0).gameIndex, [](uint32_t i) { return g_coreParent[i]; },
                                           [](uint32_t i) { return std::string_view(kCore[i]); });
        rcr::g_config.Publish(r.reloadConfig ? p : rcr::Prepared{});
        FrameSpec f; f.reloadEligible = true;
        f.overlay = int32_t(mm::ov::PoseOverlay::Pistol2H); f.authored = true; r.reloadAuthored = RunFrame(f);
        f.overlay = int32_t(mm::ov::PoseOverlay::Rifle); f.authored = false; r.reloadRifle = RunFrame(f);
        rcr::g_config.Publish(rcr::Prepared{});
    }
    // the main job on another worker thread than the FK call
    { FrameSpec f; f.mainOnOtherThread = true; r.otherThread = RunFrame(f); }
    r.afterOtherThread = RunFrame({});
    g_hot.armed.store(false);
    return r;
}

uint64_t Fnv(const std::vector<float>& aBones)
{
    uint64_t h = 0xcbf29ce484222325ull;
    const auto* b = reinterpret_cast<const uint8_t*>(aBones.data());
    for (size_t i = 0; i < aBones.size() * sizeof(float); ++i) { h ^= b[i]; h *= 0x100000001b3ull; }
    return h;
}
void Report(const char* aName, const FrameOut& m, const FrameOut& k)
{
    std::printf("  %-18s Mapping: writers 0x%02x %-28s | FK: writers 0x%02x %-28s record 0x%08x bones %016llx\n", aName, Writers(m),
                Row(m).c_str(), Writers(k), Row(k).c_str(), k.record, static_cast<unsigned long long>(Fnv(k.bones)));
}
double MaxDiffMain(const std::vector<float>& a, const std::vector<float>& b);
// The helper evaluation of the Mapping frame saw the writers of the FK frame and ended with the same outcome.
void Same(const char* aName, const FrameOut& m, const FrameOut& k, bool aBones)
{
    const std::string L = std::string(aName) + ": ";
    check(Writers(m) == Writers(k), L + "the helper evaluation sees the same core writers under both owners");
    check(m.rowCalls == k.rowCalls && m.outcome == k.outcome, L + "same coverage row and outcome under both owners (" + Row(m) + " vs " + Row(k) + ")");
    if (aBones)
        check(MaxDiffMain(m.bones, k.bones) < 1e-6, L + "the frame leaves the same records under both owners, shadow skeleton apart (max diff " +
              std::to_string(MaxDiffMain(m.bones, k.bones)) + ")");
}

// ---- shadow skeleton: the fixture rig with the game's 26 shadow bones appended, parents as in the game's shadow rig (shadow_X under
// shadow_(parent of X), shadow_Hips under the hips' parent, shadow_ToeBase under shadow_Foot)
constexpr const char* kShadowMain[24] = {
    "Hips", "Spine", "Spine1", "Spine2", "Spine3", "LeftUpLeg", "RightUpLeg", "LeftLeg", "RightLeg", "LeftFoot", "RightFoot",
    "LeftShoulder", "RightShoulder", "Neck", "Neck1", "Head", "LeftArm", "RightArm", "LeftForeArm", "RightForeArm", "LeftHand",
    "RightHand", "WeaponLeft", "WeaponRight"};
struct ShadowRig { std::vector<int32_t> main, shadow; int32_t toe[2]{}, heel[2]{}, shadowToe[2]{}; };
int32_t FindIn(const Fixture& fx, const std::string& aName)
{
    const uint64_t h = mm::integ::CNameHash(aName);
    for (size_t i = 0; i < fx.names.size(); ++i) if (fx.names[i] == h) return int32_t(i);
    return -1;
}
// The recorded merged rig carries the shadow skeleton (its records are not part of the recorded frame, so they start at 0).
bool FindShadow(const Fixture& fx, ShadowRig& s)
{
    for (const char* n : kShadowMain)
    {
        s.main.push_back(FindIn(fx, n)); s.shadow.push_back(FindIn(fx, std::string("shadow_") + n));
        if (s.main.back() < 0 || s.shadow.back() < 0) return false;
    }
    const char* sides[2] = {"Left", "Right"};
    for (int k = 0; k < 2; ++k)
    {
        s.toe[k] = FindIn(fx, std::string(sides[k]) + "ToeBase"); s.heel[k] = FindIn(fx, std::string(sides[k]) + "Heel");
        s.shadowToe[k] = FindIn(fx, std::string("shadow_") + sides[k] + "ToeBase");
        if (s.toe[k] < 0 || s.heel[k] < 0 || s.shadowToe[k] < 0) return false;
    }
    return true;
}
ShadowRig g_shadow;
bool IsShadow(size_t aBone)
{
    if (std::find(g_shadow.shadow.begin(), g_shadow.shadow.end(), int32_t(aBone)) != g_shadow.shadow.end()) return true;
    return int32_t(aBone) == g_shadow.shadowToe[0] || int32_t(aBone) == g_shadow.shadowToe[1];
}
// Records outside the shadow skeleton. The test has no game shadow pass (it runs after the early owner's write in the game), so under
// the Mapping owner the shadow records keep their start values while the FK owner sets them; they are checked on their own.
double MaxDiffMain(const std::vector<float>& a, const std::vector<float>& b)
{
    if (a.size() != b.size()) return 1e9;
    double m = 0;
    for (size_t i = 0; i < a.size(); ++i)
        if (!IsShadow(i / 12)) m = std::fmax(m, std::fabs(double(a[i]) - double(b[i])));
    return m;
}
double RecordDiff(const std::vector<float>& a, int32_t ia, const std::vector<float>& b, int32_t ib)
{
    double m = 0;
    for (int c = 0; c < 12; ++c) m = std::fmax(m, std::fabs(double(a[size_t(ia) * 12 + c]) - double(b[size_t(ib) * 12 + c])));
    return m;
}
// local(Heel) * local(ToeBase) in double (translation, rotation, scale), compared with the shadow toe record
double ToeDiff(const std::vector<float>& b, int32_t aHeel, int32_t aToe, int32_t aShadowToe)
{
    const float* h = &b[size_t(aHeel) * 12]; const float* t = &b[size_t(aToe) * 12]; const float* s = &b[size_t(aShadowToe) * 12];
    const double hx = h[4], hy = h[5], hz = h[6], hw = h[7];
    const double v[3] = {double(h[8]) * t[0], double(h[9]) * t[1], double(h[10]) * t[2]};
    // rotate v by (hx, hy, hz, hw) through the rotation matrix
    const double r[3] = {
        (1 - 2 * (hy * hy + hz * hz)) * v[0] + 2 * (hx * hy - hz * hw) * v[1] + 2 * (hx * hz + hy * hw) * v[2],
        2 * (hx * hy + hz * hw) * v[0] + (1 - 2 * (hx * hx + hz * hz)) * v[1] + 2 * (hy * hz - hx * hw) * v[2],
        2 * (hx * hz - hy * hw) * v[0] + 2 * (hy * hz + hx * hw) * v[1] + (1 - 2 * (hx * hx + hy * hy)) * v[2]};
    const double tx = t[4], ty = t[5], tz = t[6], tw = t[7];
    const double q[4] = {hw * tx + hx * tw + hy * tz - hz * ty, hw * ty - hx * tz + hy * tw + hz * tx,
                         hw * tz + hx * ty - hy * tx + hz * tw, hw * tw - hx * tx - hy * ty - hz * tz};
    const double want[11] = {h[0] + r[0], h[1] + r[1], h[2] + r[2], double(t[3]), q[0], q[1], q[2], q[3],
                             double(h[8]) * t[8], double(h[9]) * t[9], double(h[10]) * t[10]};
    double m = 0;
    for (int c = 0; c < 11; ++c) m = std::fmax(m, std::fabs(want[c] - double(s[c])));
    return m;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) { std::printf("usage: EarlyPoseFrameTest <woman helper fixture dir>\n"); return 2; }
    check(LoadFixture(argv[1], g_fx), "woman fixture and bundle load");
    if (g_failures) return 1;
    const fs::path work = fs::temp_directory_path() / ("utpp_early_pose_frame_test_" + std::to_string(GetCurrentProcessId()));
    {
        std::error_code ec; fs::remove_all(work, ec);
        fs::create_directories(work / "skin_helpers" / "woman");
        std::ofstream(work / "skin_helpers" / "mode.txt", std::ios::binary) << "APPLY\n";
        std::ofstream(work / "skin_helpers" / "woman" / "program.ir", std::ios::binary) << g_fx.program;
        std::ofstream(work / "skin_helpers" / "woman" / "binding.dat", std::ios::binary) << g_fx.binding;
    }
    check(br::Load(work.wstring()) && br::ApplyEnabled(), "APPLY bundle loads");
    g_catalog = br::RequiredCatalog();
    check(!g_catalog.empty() && BuildJobs(), "jobs built from the fixture rig");
    check(FindShadow(g_fx, g_shadow), "the fixture rig holds the 24 shadow bones and both shadow toes");
    if (g_failures) return 1;
    *MappingOriginalSlot() = &FakeMapping;
    *FkOriginalSlot() = &FakeFk;
    std::printf("fixture rig %zu bones, %zu core bones, %zu required helpers\n", g_fx.names.size(), g_coreDst.size(), g_catalog.size());

    const Run m = RunAll(er::Owner::Mapping, work);
    const Run k = RunAll(er::Owner::FK, work);
    std::printf("frame kinds (core writers seen by the helper evaluation, coverage row -> outcome):\n");
    Report("normal", m.normal, k.normal); Report("normal again", m.normalAgain, k.normalAgain);
    Report("not admitted", m.notAdmitted, k.notAdmitted); Report("no main job", m.noMain, k.noMain);
    Report("vehicle reference", m.vehicleRef, k.vehicleRef); Report("vehicle written", m.vehicleWritten, k.vehicleWritten);
    Report("vehicle ended", m.vehicleEnded, k.vehicleEnded);
    Report("vault", m.vault, k.vault); Report("reload authored", m.reloadAuthored, k.reloadAuthored);
    Report("reload rifle", m.reloadRifle, k.reloadRifle); Report("other thread", m.otherThread, k.otherThread);
    Report("after other thread", m.afterOtherThread, k.afterOtherThread);
    Report("late FK", m.lateFk, k.lateFk);
    Report("FK after late FK", m.noMainAfterSkip, k.noMainAfterSkip);
    std::printf("reload config %d/%d; Mapping rcOwnerOk %llu/%llu captured %llu/%llu finish %llu/%llu; FK rcOwnerOk %llu/%llu\n",
                m.reloadConfig, k.reloadConfig, (unsigned long long)m.reloadAuthored.rcOwnerOk, (unsigned long long)m.reloadRifle.rcOwnerOk,
                (unsigned long long)m.reloadAuthored.rcCaptured, (unsigned long long)m.reloadRifle.rcCaptured,
                (unsigned long long)m.reloadAuthored.rcFinish, (unsigned long long)m.reloadRifle.rcFinish,
                (unsigned long long)k.reloadAuthored.rcOwnerOk, (unsigned long long)k.reloadRifle.rcOwnerOk);

    // normal locomotion frame: one early write under Mapping, one FK-side write under FK; the helpers are recomputed on the published path
    check(m.normal.earlyApplied == 1 && m.normal.mmApplied == 1 && k.normal.earlyApplied == 0 && k.normal.mmApplied == 1,
          "normal: exactly one motion write per frame (Mapping: early job, FK: FK call)");
    check(Writers(k.normal) == cw::Bit(cw::kDriver) && k.normal.outcome[size_t(br::Outcome::Committed)] == 1,
          "normal (FK control): driver writer, helpers committed");
    Same("normal", m.normal, k.normal, true);
    Same("normal again", m.normalAgain, k.normalAgain, true);
    check(MaxDiff(m.normal.bones, g_fx.bones) > 1e-4, "normal: the motion pose changed the records (the frame is not a no-op)");
    // frames without an admitted early write: no writer of an earlier frame may be counted
    check(Writers(m.notAdmitted) == 0 && Writers(k.notAdmitted) == 0, "not admitted: no core writer under either owner");
    Same("not admitted", m.notAdmitted, k.notAdmitted, true);
    check(Writers(m.noMain) == 0 && m.noMain.rowCalls[br::kNoWriter] == 1,
          "no main job (Mapping): no core writer of the previous frame leaks into this frame");
    check(Writers(k.noMain) == cw::Bit(cw::kDriver), "no main job (FK control): the FK call writes the motion pose");
    check(Writers(m.lateFk) == cw::Bit(cw::kDriver), "late FK call (Mapping): it consumes the pending cycle and its writer");
    check(Writers(m.noMainAfterSkip) == 0 && m.noMainAfterSkip.rowCalls[br::kNoWriter] == 1,
          "FK call after the late one (Mapping): the consumed cycle's writers are not counted again");
    // vehicle transition: the reference frame is measured, the written frame takes the weight-0 writer route on both owners
    Same("vehicle reference", m.vehicleRef, k.vehicleRef, false);
    check(k.vehicleWritten.record & cw::Bit(cw::kVehicleTransition), "vehicle written (FK control): the transition wrote core bones");
    Same("vehicle written", m.vehicleWritten, k.vehicleWritten, false);
    // Mapping only (the FK owner writes nothing once the transition has ended): the main job wrote the transition inside the window it
    // opened, so the helpers follow that write on the weight-0 route even though the window is closed by the time of the FK call
    check(Writers(k.vehicleEnded) == 0, "vehicle ended (FK control): nothing written after the end");
    check(Writers(m.vehicleEnded) == cw::Bit(cw::kVehicleTransition) && (m.vehicleEnded.record & cw::kZeroWindow) &&
          m.vehicleEnded.outcome[size_t(br::Outcome::Committed)] == 1,
          "vehicle ended (Mapping): the transition write of the main job is recomputed on the weight-0 route (" + Row(m.vehicleEnded) + ")");
    // vault with a live motion write: the published path on both owners
    check((k.vault.record & cw::kZeroWindow) && (k.vault.record & cw::Bit(cw::kDriver)), "vault (FK control): window open and the motion pose written");
    Same("vault", m.vault, k.vault, true);
    // reloads: the contact solve is admitted under Mapping only (authored and rifle game clip); the helpers see the motion writer on both
    check(m.reloadAuthored.rcOwnerOk == 1 && m.reloadRifle.rcOwnerOk == 1, "reload (Mapping): the contact solve admits the authored and the rifle clip");
    check(k.reloadAuthored.rcOwnerOk == 0, "reload (FK control): the authored clip is never corrected under FK");
    Same("reload authored", m.reloadAuthored, k.reloadAuthored, false);
    Same("reload rifle", m.reloadRifle, k.reloadRifle, false);
    // main job on another thread: the mapping-cycle receipt of the FK thread has no main job, so the helper evaluation is refused on
    // both owners (same outcome); under Mapping the writers travel with that receipt, so this call has none, and nothing of the other
    // thread's job reaches the next frame of this thread
    check(m.otherThread.earlyApplied == 1, "other thread (Mapping): the early job wrote");
    check(m.otherThread.outcome == k.otherThread.outcome && m.otherThread.outcome[size_t(br::Outcome::Receipt)] == 1,
          "other thread: the helper evaluation is refused on the receipt under both owners");
    check(Writers(m.otherThread) == 0 && m.otherThread.rowCalls[br::kNoWriter] == 1,
          "other thread (Mapping): no writer is credited to a call outside the job's cycle");
    Same("after other thread", m.afterOtherThread, k.afterOtherThread, true);

    // ---- health check of the Mapping owner (the periodic warning): quiet while jobs are admitted or the motion pose is legitimately
    // refused (weight 0, first person); reported when jobs arrive and none is admitted
    {
        er::Counters a{}, b{};
        b.calls = 10; b.notReady = 10;
        check(er::MappingStalled(a, b), "health: jobs without an admitted one are reported");
        b.applied = 1; check(!er::MappingStalled(a, b), "health: one admitted job is healthy");
        b.applied = 0; b.poseRejected = 1; check(!er::MappingStalled(a, b), "health: a pose rejection (weight 0 / stale) stays quiet");
        b.poseRejected = 0; b.perspectiveRejected = 1; check(!er::MappingStalled(a, b), "health: a first-person rejection stays quiet");
        check(!er::MappingStalled(a, a), "health: no jobs, nothing to report");
        check(Arm(er::Owner::Mapping, work), "health: Mapping armed");
        const auto c0 = er::ReadCounters(); RunFrame({});
        const auto c1 = er::ReadCounters(); { FrameSpec f; f.weight = 0.f; RunFrame(f); }
        const auto c2 = er::ReadCounters(); { FrameSpec f; f.firstPerson = true; RunFrame(f); }
        const auto c3 = er::ReadCounters();
        er::Disarm(); RunFrame({}); // the policy is gone while armed: every job is refused as not ready
        const auto c4 = er::ReadCounters();
        check(!er::MappingStalled(c0, c1) && c1.applied == c0.applied + 1, "health: a written frame is healthy");
        check(!er::MappingStalled(c1, c2), "health: a weight-0 frame stays quiet");
        check(!er::MappingStalled(c2, c3), "health: a first-person frame stays quiet");
        check(er::MappingStalled(c3, c4) && c4.notReady > c3.notReady, "health: a frame whose job is no longer admitted is reported");
        g_hot.armed.store(false);
    }

    // ---- early arm step not ready for a rig (a catalog helper missing from the rig): the FK owner takes that rig and writes the pose;
    // the next ready rig returns to the early owner
    {
        const uint64_t rig = reinterpret_cast<uint64_t>(g_metaRig);
        std::vector<uint64_t> wide = g_catalog;
        wide.push_back(mm::integ::CNameHash("utpp_test_bone_absent_from_rig"));
        Arm(er::Owner::Mapping, work, &wide);
        check(g_lastChoice.fellBack && g_lastChoice.owner == er::Owner::FK && g_lastChoice.missingHelpers == 1 &&
              g_lastChoice.missingAnchors == 0, "fallback: a missing helper hands the rig to the FK owner (1 helper missing)");
        check(er::CurrentOwner() == er::Owner::FK && er::RequestedOwner() == er::Owner::Mapping,
              "fallback: FK writes this rig, the early owner stays requested");
        const FrameOut f1 = RunFrame({});
        check(f1.earlyApplied == 0 && f1.mmApplied == 1 && Writers(f1) == cw::Bit(cw::kDriver) && MaxDiff(f1.bones, g_fx.bones) > 1e-4,
              "fallback: the FK call writes the motion pose (no silent loss)");
        er::ArmChoice refused;
        check(!er::ArmOwnerForRig(true, rig, uint32_t(g_fx.names.size()), kTag, 0, g_fx.names, g_catalog, refused) &&
              er::CurrentOwner() == er::Owner::FK, "fallback: the owner does not change while armed");
        check(Arm(er::Owner::Mapping, work) && er::CurrentOwner() == er::Owner::Mapping,
              "fallback: arming a ready rig returns to the early owner");
        const FrameOut f2 = RunFrame({});
        check(f2.earlyApplied == 1 && f2.mmApplied == 1, "fallback: the early job writes again after the return");
        check(MaxDiffMain(f1.bones, f2.bones) < 1e-6, "fallback: the FK-written frame equals the early-written frame (shadow skeleton apart)");
        g_hot.armed.store(false);
        er::ArmChoice noHook;
        check(er::ArmOwnerForRig(false, rig, uint32_t(g_fx.names.size()), kTag, 0, g_fx.names, g_catalog, noHook) && noHook.fellBack &&
              noHook.missingHelpers == 0 && er::CurrentOwner() == er::Owner::FK, "fallback: no mapping hook hands the rig to the FK owner");
        er::Disarm();
    }

    // ---- shadow skeleton under the FK owner: set from the written main bones after the write, untouched otherwise
    {
        const ShadowRig& s = g_shadow;
        Arm(er::Owner::FK, work);
        check(shadowcopy::Status() == shadowcopy::ArmStatus::Ready, std::string("shadow: copy armed (") + shadowcopy::StatusName() + ")");
        const int32_t leftArm = FindIn(g_fx, "LeftArm");
        const FrameOut w = RunFrame({});
        double pairDiff = 0, toeDiff = 0;
        for (size_t j = 0; j < s.main.size(); ++j) pairDiff = std::fmax(pairDiff, RecordDiff(w.bones, s.shadow[j], w.bones, s.main[j]));
        for (int side = 0; side < 2; ++side) toeDiff = std::fmax(toeDiff, ToeDiff(w.bones, s.heel[side], s.toe[side], s.shadowToe[side]));
        check(RecordDiff(w.bones, leftArm, g_fx.bones, leftArm) > 1e-4, "shadow: the motion pose moved LeftArm");
        check(pairDiff == 0.0, "shadow: the 24 shadow bones equal their main bones after the FK write (max diff " + std::to_string(pairDiff) + ")");
        check(toeDiff < 1e-5, "shadow: shadow_ToeBase = Heel * ToeBase (max diff " + std::to_string(toeDiff) + ")");
        auto untouched = [&](const FrameOut& o) {
            double d = 0;
            for (size_t j = 0; j < s.shadow.size(); ++j) d = std::fmax(d, RecordDiff(o.bones, s.shadow[j], g_fx.bones, s.shadow[j]));
            for (int side = 0; side < 2; ++side) d = std::fmax(d, RecordDiff(o.bones, s.shadowToe[side], g_fx.bones, s.shadowToe[side]));
            return d == 0.0;
        };
        { FrameSpec f; f.weight = 0.f; check(untouched(RunFrame(f)), "shadow: weight 0 leaves the game's shadow pose"); }
        { FrameSpec f; f.firstPerson = true; check(untouched(RunFrame(f)), "shadow: first person leaves the game's shadow pose"); }
        Arm(er::Owner::Mapping, work);
        check(shadowcopy::Status() == shadowcopy::ArmStatus::Ready && er::CurrentOwner() == er::Owner::Mapping,
              "shadow: Mapping owner armed on the shadow rig");
        check(untouched(RunFrame({})), "shadow: the Mapping owner never sets the shadow bones (the game's pass runs after its write)");
        g_hot.armed.store(false);
        // the game's reference pose: Heel and ToeBase of the shadow rig give its shadow_ToeBase
        Arm(er::Owner::FK, work);
        std::vector<float> ref = g_fx.bones;
        const float heel[12] = {0.0610666685f, -2.84217088e-16f, 0.f, 0.f, 3.94609158e-18f, -9.86522895e-19f, 0.879211128f, 0.476432562f, 1, 1, 1, 1};
        const float toe[12] = {0.171654835f, 1.24344979e-16f, 1.79412039e-15f, 0.f, 2.19228608e-18f, 3.50765773e-18f, 0.147201985f, 0.989106476f,
                               0.99999994f, 0.99999994f, 1, 1};
        const float want[8] = {-0.032661017f, 0.143807173f, 0.f, 0.f, 0.f, 0.f, 0.939765215f, 0.341820896f};
        std::memcpy(&ref[size_t(s.heel[0]) * 12], heel, sizeof heel); std::memcpy(&ref[size_t(s.toe[0]) * 12], toe, sizeof toe);
        const uint32_t n = shadowcopy::Apply(reinterpret_cast<uint8_t*>(ref.data()), uint32_t(g_fx.names.size()), reinterpret_cast<uint64_t>(g_metaRig));
        double refDiff = 0;
        for (int c = 0; c < 8; ++c) refDiff = std::fmax(refDiff, std::fabs(double(ref[size_t(s.shadowToe[0]) * 12 + c]) - want[c]));
        check(n == 26 && refDiff < 1e-6, "shadow: the reference Heel and ToeBase give the game's reference shadow_LeftToeBase (max diff " +
              std::to_string(refDiff) + ")");
        check(shadowcopy::Apply(reinterpret_cast<uint8_t*>(ref.data()), uint32_t(g_fx.names.size()) - 1, reinterpret_cast<uint64_t>(g_metaRig)) == 0,
              "shadow: a pose of another bone count is not touched");
        g_hot.armed.store(false);
        // a rig whose shadow toe hangs under another bone is refused as a whole
        Fixture bent = g_fx; bent.parents[size_t(s.shadowToe[1])] = int16_t(s.heel[1]);
        check(shadowcopy::Arm(reinterpret_cast<uint64_t>(g_metaRig), bent.names, bent.parents) == 0 &&
              shadowcopy::Status() == shadowcopy::ArmStatus::ShapeMismatch, "shadow: a rig of another shape is refused");
        // a rig without the shadow skeleton: nothing to copy
        Fixture plain = g_fx;
        for (int32_t i : s.shadow) plain.names[size_t(i)] = 0x1000u + uint64_t(i);
        for (int32_t i : s.shadowToe) plain.names[size_t(i)] = 0x1000u + uint64_t(i);
        check(shadowcopy::Arm(reinterpret_cast<uint64_t>(g_metaRig), plain.names, plain.parents) == 0 &&
              shadowcopy::Status() == shadowcopy::ArmStatus::NoShadowBones, "shadow: a rig without shadow bones is left alone");
        shadowcopy::Disarm();
    }

    std::error_code ec; fs::remove_all(work, ec);
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
