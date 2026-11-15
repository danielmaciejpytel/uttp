// Vault shoulder limiter (src/mm/VaultShoulder.hpp): car vault released to the game animation, every weapon state, REAL data of the
// recording 2026-11-10 13:00 (the vault torso fixture, Build/test-data/vaulttorso/vaulttorso_20261110.txt: bones 0..28 of every frame of
// each window + the travRelease timeline; 12 released car vaults, 8 with a rifle and 4 unarmed, plus two mantles and one released climb).
//
// The vaults go through the PRODUCTION chain: rt::Feed (game-thread gate / fade / exit hold, fed with the PSM inputs of the frame) and
// rt::FkPass (the FK-stage pass of FkHook.cpp, without any MM-active condition) with synthetic QPC ticks. Without the fix the game's car
// vault squeezes the shoulder joints to 19 - 22 cm (band 32.0 - 40.3 cm). Mutation builds MUST report FAIL lines: 1 = the pass compiled out,
// 2 = the gate needs a ranged weapon, 3 = no exit hold, 4 = no upper-arm compensation, 5 = the FK pass needs MM active.
// Argument: the fixture file.
#include "../src/mm/VaultShoulder.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

using namespace utpp::mm;
namespace vs = utpp::mm::vaultshoulder;
namespace vt = utpp::mm::vaulttorso;

namespace
{
int passed = 0, failed = 0;
void Check(const std::string& name, bool ok, const std::string& detail = "")
{
    std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : " | ", detail.c_str());
    ok ? ++passed : ++failed;
}
std::string F2(double v)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%.2f", v);
    return b;
}

constexpr uint32_t kBones = 29; // Root .. WeaponRight (fixture)
constexpr uint32_t kStride = integ::kGameBoneStride;
constexpr float kDeg = 57.2957795f;
constexpr int64_t kTps = 1000000; // synthetic QPC frequency
constexpr int kSpine3 = 13, kLClav = 14, kRClav = 15, kLArm = 17, kRArm = 18, kLHand = 23, kRHand = 24;
constexpr float kBandMaxM = 0.4031f; // upper band edge of the width measure (both bodies)
constexpr float kGapMinM = 0.0249f;  // minimum distance of the clavicle joints along the lateral axis (measure)

struct Frame
{
    float t = 0.0f;
    float rec[kBones][7] = {};
};
struct Vault
{
    std::string label;
    int weapon = 0, detail = 0, release = 0;
    float on = 0.0f, off = 0.0f;
    std::vector<Frame> frames;
};
struct Fixture
{
    std::vector<std::string> names;
    std::vector<int16_t> parents;
    std::vector<Vault> vaults;
};

bool Load(const char* aPath, Fixture& aOut)
{
    std::ifstream f(aPath);
    if (!f)
        return false;
    std::string line;
    Vault* cur = nullptr;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream is(line);
        std::string tag;
        is >> tag;
        if (tag == "bone")
        {
            int i, p;
            std::string n;
            is >> i >> n >> p;
            aOut.names.push_back(n);
            aOut.parents.push_back(static_cast<int16_t>(p));
        }
        else if (tag == "vault")
        {
            aOut.vaults.emplace_back();
            cur = &aOut.vaults.back();
            std::string kv;
            is >> cur->label;
            while (is >> kv)
            {
                const size_t eq = kv.find('=');
                const std::string k = kv.substr(0, eq);
                const double v = std::atof(kv.c_str() + eq + 1);
                if (k == "weapon") cur->weapon = int(v);
                else if (k == "detail") cur->detail = int(v);
                else if (k == "release") cur->release = int(v);
                else if (k == "on") cur->on = float(v);
                else if (k == "off") cur->off = float(v);
            }
        }
        else if (tag == "f" && cur)
        {
            Frame fr;
            is >> fr.t;
            for (uint32_t b = 0; b < kBones; ++b)
                for (int k = 0; k < 7; ++k)
                    is >> fr.rec[b][k];
            if (!is)
                return false;
            cur->frames.push_back(fr);
        }
    }
    return aOut.names.size() == kBones && !aOut.vaults.empty();
}

// 48-byte game records: pos @0, quat xyzw @0x10, scale @0x20
void ToRecords(const Frame& aF, uint8_t* aBuf)
{
    std::memset(aBuf, 0, kBones * kStride);
    for (uint32_t b = 0; b < kBones; ++b)
    {
        float* r = reinterpret_cast<float*>(aBuf + b * kStride);
        r[0] = aF.rec[b][0]; r[1] = aF.rec[b][1]; r[2] = aF.rec[b][2]; r[3] = 1.0f;
        r[4] = aF.rec[b][3]; r[5] = aF.rec[b][4]; r[6] = aF.rec[b][5]; r[7] = aF.rec[b][6];
        r[8] = r[9] = r[10] = r[11] = 1.0f;
    }
}

// angle of a^-1 b from its vector part (double; acos of a dot near 1 loses the small angles in float)
float AngleDeg(const math::Quat& a, const math::Quat& b)
{
    const double ax = a.x, ay = a.y, az = a.z, aw = a.w, bx = b.x, by = b.y, bz = b.z, bw = b.w;
    const double w = aw * bw + ax * bx + ay * by + az * bz;
    const double x = aw * bx - ax * bw - ay * bz + az * by;
    const double y = aw * by + ax * bz - ay * bw - az * bx;
    const double z = aw * bz - ax * by + ay * bx - az * bw;
    return float(2.0 * std::atan2(std::sqrt(x * x + y * y + z * z), std::fabs(w)) * 57.29577951308232);
}

struct Measure
{
    float width = 0.0f, gap = 0.0f;   // m, shoulder joints / clavicle joints along the lateral axis of the Spine3 frame
    math::Vec3 joint[2];              // model space shoulder joints (LeftArm, RightArm)
    math::Vec3 jointS3[2];            // the same in the Spine3 frame (the shoulder's own motion, without the body's)
    math::Vec3 hand[2];               // model space hands
    math::Quat armInS3[2];            // upper-arm rotation in the Spine3 frame
};
bool Measure_(const uint8_t* aBuf, const std::vector<int16_t>& aParents, Measure& aM)
{
    aim::Xf model[aim::kMaxFkBones];
    if (!aim::ComputeModel(aBuf, kBones, aParents.data(), kBones, model))
        return false;
    const math::Quat inv3 = math::QuatInv(model[kSpine3].q);
    auto inS3 = [&](math::Vec3 p) { return math::QuatRotate(inv3, p - model[kSpine3].p); };
    aM.width = inS3(model[kRArm].p).z - inS3(model[kLArm].p).z;
    aM.gap = inS3(model[kRClav].p).z - inS3(model[kLClav].p).z;
    aM.joint[0] = model[kLArm].p; aM.joint[1] = model[kRArm].p;
    aM.jointS3[0] = inS3(model[kLArm].p); aM.jointS3[1] = inS3(model[kRArm].p);
    aM.hand[0] = model[kLHand].p; aM.hand[1] = model[kRHand].p;
    aM.armInS3[0] = math::QuatNormalize(math::QuatMul(inv3, model[kLArm].q));
    aM.armInS3[1] = math::QuatNormalize(math::QuatMul(inv3, model[kRArm].q));
    return true;
}

struct Run
{
    std::vector<Measure> before, after;
    std::vector<std::vector<uint8_t>> bytesBefore, bytesAfter;
    std::vector<float> weight, t;
    int applied = 0, saturated = 0;
    bool otherBonesTouched = false, armTranslationTouched = false;
    float maxArmTurnDeg = 0.0f;  // upper-arm rotation change in the Spine3 frame
    float maxHandVsJoint = 0.0f; // |hand shift - shoulder joint shift| (m)
    float maxHandShift = 0.0f;   // m
    float handVert = 0.0f, handHoriz = 0.0f; // the largest hand shift split into model up (Z) and horizontal parts (m)
    float maxHandVert = 0.0f;    // largest vertical hand shift of any frame (m)
};

using Scenario = std::function<void(const Vault&, const Frame&, vt::GateInput&, bool&)>;

// The production chain of one vault, frame by frame with synthetic QPC ticks (MM weight 0 all along).
Run Simulate(const Vault& aV, const Fixture& aFx, const std::vector<uint64_t>& aHashes, const Scenario& aScn = Scenario())
{
    Run run;
    int dummy = 0;
    vs::rt::OnDisarmed();
    vs::rt::OnArmed(&dummy, aHashes, aFx.parents);
#if UTPP_VAULTSHOULDER_MUTATION == 5
    vs::rt::g_mmActiveMirror.store(false);
#endif
    float prevT = aV.frames.empty() ? 0.0f : aV.frames.front().t;
    for (const Frame& fr : aV.frames)
    {
        std::vector<uint8_t> buf(kBones * kStride);
        ToRecords(fr, buf.data());
        vt::GateInput gi;
        gi.travRelease = aV.release != 0 && fr.t >= aV.on && fr.t < aV.off;
        gi.psmDetail = gi.travRelease ? aV.detail : -1;
        gi.weaponDrawn = aV.weapon != 0;
        gi.vehicleVaultRanged = aV.weapon != 0 && aV.detail == 9;
        // the motion driver's vehicle vault event: planned at the release, cleared when the PSM leaves the event's action
        bool vehicleVault = aV.release != 0 && aV.detail == 9 && fr.t >= aV.on && fr.t < aV.off;
        if (aScn)
            aScn(aV, fr, gi, vehicleVault);
        const int64_t now = static_cast<int64_t>((fr.t + 10.0) * double(kTps));
        vs::rt::Feed(gi, vehicleVault, fr.t - prevT, now, kTps);
        prevT = fr.t;
        Measure mb, ma;
        Measure_(buf.data(), aFx.parents, mb);
        std::vector<uint8_t> orig = buf;
        const float w = vs::rt::g_weight.load();
        const uint64_t a0 = vs::rt::g_applied.load(), s0 = vs::rt::g_saturated.load();
        vs::rt::FkPass(buf.data(), kBones, &dummy, now, kTps, true);
        if (vs::rt::g_applied.load() > a0)
            ++run.applied;
        if (vs::rt::g_saturated.load() > s0)
            ++run.saturated;
        Measure_(buf.data(), aFx.parents, ma);
        if (std::getenv("UTPP_VS_VERBOSE") && (ma.width < vs::kBandMinM || std::strcmp(std::getenv("UTPP_VS_VERBOSE"), aV.label.c_str()) == 0))
            std::printf("    %s t=%.3f off=%.3f w=%.2f width %.2f -> %.2f cm ref %.2f | L %.1f %.1f %.1f -> %.1f %.1f %.1f | R %.1f %.1f %.1f -> %.1f %.1f %.1f\n",
                        aV.label.c_str(), double(fr.t), double(aV.off), double(w), double(mb.width * 100), double(ma.width * 100),
                        double((vs::rt::g_ref.half[0] + vs::rt::g_ref.half[1]) * 100), double(mb.jointS3[0].x * 100), double(mb.jointS3[0].y * 100),
                        double(mb.jointS3[0].z * 100), double(ma.jointS3[0].x * 100), double(ma.jointS3[0].y * 100), double(ma.jointS3[0].z * 100),
                        double(mb.jointS3[1].x * 100), double(mb.jointS3[1].y * 100), double(mb.jointS3[1].z * 100), double(ma.jointS3[1].x * 100),
                        double(ma.jointS3[1].y * 100), double(ma.jointS3[1].z * 100));
        for (uint32_t b = 0; b < kBones; ++b)
        {
            const bool clav = b == uint32_t(kLClav) || b == uint32_t(kRClav), arm = b == uint32_t(kLArm) || b == uint32_t(kRArm);
            const uint8_t* p0 = orig.data() + b * kStride;
            const uint8_t* p1 = buf.data() + b * kStride;
            if (!clav && !arm && std::memcmp(p0, p1, kStride) != 0)
                run.otherBonesTouched = true;
            if (arm && (std::memcmp(p0, p1, 16) != 0 || std::memcmp(p0 + 32, p1 + 32, 16) != 0))
                run.armTranslationTouched = true; // only the rotation of the upper arm may change
            if (clav && std::memcmp(p0 + 32, p1 + 32, 16) != 0)
                run.otherBonesTouched = true;     // scale untouched
        }
        for (int k = 0; k < 2; ++k)
        {
            run.maxArmTurnDeg = std::fmax(run.maxArmTurnDeg, AngleDeg(mb.armInS3[k], ma.armInS3[k]));
            const float js = aim::Length(ma.joint[k] - mb.joint[k]), hs = aim::Length(ma.hand[k] - mb.hand[k]);
            const math::Vec3 dh = ma.hand[k] - mb.hand[k];
            run.maxHandVert = std::fmax(run.maxHandVert, std::fabs(dh.z));
            if (hs > run.maxHandShift)
            {
                run.handVert = dh.z;
                run.handHoriz = std::sqrt(dh.x * dh.x + dh.y * dh.y);
            }
            run.maxHandShift = std::fmax(run.maxHandShift, hs);
            run.maxHandVsJoint = std::fmax(run.maxHandVsJoint, std::fabs(hs - js));
        }
        run.before.push_back(mb);
        run.after.push_back(ma);
        run.bytesBefore.push_back(orig);
        run.bytesAfter.push_back(buf);
        run.weight.push_back(w);
        run.t.push_back(fr.t);
    }
    vs::rt::OnDisarmed();
    return run;
}

float MinWidth(const std::vector<Measure>& m) { float v = 1e9f; for (const Measure& x : m) v = std::fmin(v, x.width); return v; }
float MaxWidth(const std::vector<Measure>& m) { float v = -1e9f; for (const Measure& x : m) v = std::fmax(v, x.width); return v; }
float MinGap(const std::vector<Measure>& m) { float v = 1e9f; for (const Measure& x : m) v = std::fmin(v, x.gap); return v; }
int Below(const std::vector<Measure>& m, float aLo) { int n = 0; for (const Measure& x : m) n += x.width < aLo ? 1 : 0; return n; }
// largest per-frame displacement of a shoulder joint in the Spine3 frame (m) and largest per-frame change of the width
float MaxJointStep(const std::vector<Measure>& m)
{
    float v = 0.0f;
    for (size_t i = 1; i < m.size(); ++i)
        for (int k = 0; k < 2; ++k)
            v = std::fmax(v, aim::Length(m[i].jointS3[k] - m[i - 1].jointS3[k]));
    return v;
}
float MaxWidthStep(const std::vector<Measure>& m)
{
    float v = 0.0f;
    for (size_t i = 1; i < m.size(); ++i)
        v = std::fmax(v, std::fabs(m[i].width - m[i - 1].width));
    return v;
}
bool Identical(const Run& r)
{
    for (size_t i = 0; i < r.bytesBefore.size(); ++i)
        if (r.bytesBefore[i] != r.bytesAfter[i])
            return false;
    return true;
}
bool IsCarVault(const Vault& v) { return v.detail == 9 && v.release != 0; }
// lowest width from the release on until the weight is back to 0 after the release ended (the released vault and its exit episode)
float EpisodeMin(const Run& r, const Vault& v, const std::vector<Measure>& m)
{
    float lo = 1e9f;
    for (size_t k = 0; k < r.t.size(); ++k)
    {
        if (r.t[k] < v.on)
            continue;
        if (r.t[k] >= v.off && r.weight[k] == 0.0f)
            break;
        lo = std::fmin(lo, m[k].width);
    }
    return lo;
}
} // namespace

int main(int argc, char** argv)
{
    // ---- gate truth table (pure): on for a released vault 9 over a vehicle, on foot, in every weapon state
    {
        vt::GateInput g;
        g.travRelease = true;
        g.psmDetail = 9;
        Check("gate: released vehicle vault 9, unarmed = on", vs::Wanted(g, true));
        vt::GateInput m = g;
        m.weaponDrawn = true; // melee / fists: weapon drawn, no ranged overlay
        Check("gate: released vehicle vault 9, melee weapon drawn (no ranged overlay) = on", vs::Wanted(m, true));
        vt::GateInput r = m;
        r.vehicleVaultRanged = true;
        Check("gate: released vehicle vault 9, ranged weapon = on", vs::Wanted(r, true));
        Check("gate: off when the last event is not a vehicle vault v2 rejected", !vs::Wanted(r, false));
        vt::GateInput c = g;
        c.psmDetail = 8;
        Check("gate: off for a climb / mantle (detail 8)", !vs::Wanted(c, true));
        vt::GateInput d = g;
        d.travRelease = false;
        Check("gate: off without the travRelease reason (v2 owns the traversal)", !vs::Wanted(d, true));
        vt::GateInput h = g;
        h.psmReleaseBit = true;
        vt::GateInput k = g;
        k.otherRelease = true;
        Check("gate: off with a vehicle / scene / workspot / takedown / carry / vitals / swimming bit or another release reason",
              !vs::Wanted(h, true) && !vs::Wanted(k, true));
        vt::GateInput t = g;
        t.travRelease = false;
        t.psmDetail = -1;
        Check("hold: allowed in the tail of an episode on plain locomotion, also unarmed", vs::HoldAllowed(t, true) && !vs::HoldAllowed(t, false));
        bool follow = false;
        for (int det : {3, 5, 8, 9, 10, 13, 21, 29, 30, 31})
        {
            vt::GateInput f = t;
            f.psmDetail = det;
            follow = follow || vs::HoldAllowed(f, true);
        }
        vt::GateInput mb = t;
        mb.psmReleaseBit = true;
        Check("hold: never on a crouch / slide / mantle / ladder / knockdown or under a mount / scene bit", !follow && !vs::HoldAllowed(mb, true));
    }
    // ---- soft floor and fade
    {
        bool mono = true, above = true;
        float prev = vs::Limit(0.10f);
        for (float w = 0.10f; w <= 0.45f; w += 0.0005f)
        {
            const float l = vs::Limit(w);
            mono = mono && l >= prev - 1e-6f;
            above = above && l >= w - 1e-6f && l >= vs::kBandMinM && (w < vs::kFloorM || l == w);
            prev = l;
        }
        Check("limit: monotone, never narrows, identity at and above the floor, never below the band (32.02 cm)", mono && above);
        vs::Fade f;
        float w = 0.0f;
        int n = 0;
        while (w < 1.0f && n < 100)
        {
            w = f.Step(1.0f / 60.0f, true, false);
            ++n;
        }
        Check("fade: 0 -> 1 in 0.06 s (4 frames at 60 Hz)", n >= 3 && n <= 5, "frames " + std::to_string(n));
        int hn = 0;
        while (f.Weight() >= 1.0f && hn < 200)
        {
            f.Step(1.0f / 60.0f, false, true);
            ++hn;
        }
        Check("fade: the hold ends after at most 0.6 s", hn >= 30 && hn <= 40, "frames " + std::to_string(hn));
        int dn = 0;
        while (f.Weight() > 0.0f && dn < 100)
        {
            f.Step(1.0f / 60.0f, false, false);
            ++dn;
        }
        Check("fade: 1 -> 0 in 0.15 s without the hold", dn >= 8 && dn <= 11, "frames " + std::to_string(dn));
    }

    if (argc < 2)
    {
        std::printf("FAIL missing fixture argument\n");
        return 1;
    }
    Fixture fx;
    const bool loaded = Load(argv[1], fx);
    Check("fixture loads (29 bones, vault windows)", loaded, argv[1]);
    if (!loaded)
        return 1;
    std::vector<uint64_t> hashes;
    for (const std::string& n : fx.names)
        hashes.push_back(integ::CNameHash(n));
    const vs::Rig rig = vs::ResolveRig(hashes, fx.parents);
    Check("rig: Spine3, shoulders and upper arms resolved", rig.ok == 1 && rig.spine3 == kSpine3 && rig.clav[0] == kLClav && rig.clav[1] == kRClav &&
                                                              rig.arm[0] == kLArm && rig.arm[1] == kRArm && rig.count == 19);
    {
        std::vector<uint64_t> bad = hashes;
        bad[kRClav] = 1;
        std::vector<int16_t> par = fx.parents;
        par[kLArm] = kSpine3;
        Check("rig: fail closed (a missing bone, a broken chain)", vs::ResolveRig(bad, fx.parents).ok == 0 && vs::ResolveRig(hashes, par).ok == 0);
    }

    // ---- runtime wiring on the worst frame of unarmed_vault_a
    const Vault* uvA = nullptr;
    for (const Vault& v : fx.vaults)
        if (v.label == "unarmed_vault_a")
            uvA = &v;
    Check("fixture has unarmed_vault_a", uvA != nullptr);
    if (uvA)
    {
        std::vector<uint8_t> buf(kBones * kStride);
        size_t worst = 0;
        float lo = 1e9f;
        for (size_t i = 0; i < uvA->frames.size(); ++i)
        {
            ToRecords(uvA->frames[i], buf.data());
            Measure m;
            Measure_(buf.data(), fx.parents, m);
            if (m.width < lo)
            {
                lo = m.width;
                worst = i;
            }
        }
        int dummy = 0, other = 0;
        vs::rt::OnDisarmed();
        vs::rt::OnArmed(&dummy, hashes, fx.parents);
#if UTPP_VAULTSHOULDER_MUTATION == 5
        vs::rt::g_mmActiveMirror.store(false);
#endif
        vt::GateInput g;
        g.travRelease = true;
        g.psmDetail = 9;
        const int64_t now = 5000000;
        for (int i = 0; i < 10; ++i)
            vs::rt::Feed(g, true, 0.1f, now, kTps);
        // the reference of the episode: the first frame of the vault (V's own pose when the release starts)
        ToRecords(uvA->frames.front(), buf.data());
        vs::rt::FkPass(buf.data(), kBones, &dummy, now, kTps, true);
        ToRecords(uvA->frames[worst], buf.data());
        std::vector<uint8_t> orig = buf;
        Check("runtime: a descriptor of another rig is left alone", !vs::rt::FkPass(buf.data(), kBones, &other, now, kTps, true) && buf == orig);
        Check("runtime: a stale published weight is not applied", !vs::rt::FkPass(buf.data(), kBones, &dummy, now + 10 * kTps, kTps, true) && buf == orig);
        Check("runtime: without the perspective permission (FPP) nothing is written", !vs::rt::FkPass(buf.data(), kBones, &dummy, now, kTps, false) && buf == orig);
        const bool ok = vs::rt::FkPass(buf.data(), kBones, &dummy, now, kTps, true);
        Measure m;
        Measure_(buf.data(), fx.parents, m);
        Check("runtime: unarmed worst frame at MM weight 0 is limited into the band", ok && m.width >= vs::kBandMinM && lo < 0.20f,
              F2(lo * 100.0) + " -> " + F2(m.width * 100.0) + " cm");
        vt::GateInput mount = g;
        mount.psmReleaseBit = true;
        vs::rt::Feed(mount, true, 1.0f / 60.0f, now + 16000, kTps);
        ToRecords(uvA->frames[worst], buf.data());
        orig = buf;
        Check("runtime: a mount / scene bit drops the weight at once (nothing written)",
              !vs::rt::FkPass(buf.data(), kBones, &dummy, now + 16000, kTps, true) && buf == orig && vs::rt::g_weight.load() == 0.0f);
        vs::rt::OnDisarmed();
        Check("runtime: disarmed = nothing applied", !vs::rt::FkPass(buf.data(), kBones, &dummy, now, kTps, true) && buf == orig);
    }

    // ---- every window through the production chain
    std::vector<Run> runs(fx.vaults.size());
    for (size_t i = 0; i < fx.vaults.size(); ++i)
        runs[i] = Simulate(fx.vaults[i], fx, hashes);
    std::printf("label                  weapon | width min cm before -> after | frames < 32.02 before -> after | max width cm after | clav gap min cm"
                " before -> after | max joint step cm before -> after | max width step cm before -> after | max hand shift cm | applied saturated"
                " | episode width min cm before -> after | largest hand shift: vertical / horizontal cm, largest vertical cm\n");
    int carVaults = 0;
    bool problem = true, band = true, upper = true, entry = true, exitOk = true, other = true, armT = true, armRot = true, handRigid = true, steps = true;
    float worstHand = 0.0f, worstArmTurn = 0.0f, worstStepGain = -1e9f, worstMin = 1e9f, outsideMin = 1e9f;
    bool afterEp = true;
    std::string outsideAt;
    for (size_t i = 0; i < fx.vaults.size(); ++i)
    {
        const Vault& v = fx.vaults[i];
        const Run& r = runs[i];
        std::printf("  %-22s %d | %6.2f -> %6.2f | %3d -> %3d | %6.2f | %5.2f -> %5.2f | %5.2f -> %5.2f | %5.2f -> %5.2f | %5.2f | %d %d | %6.2f -> %6.2f"
                    " | %+6.2f / %5.2f, %5.2f\n",
                    v.label.c_str(), v.weapon,
                    double(MinWidth(r.before) * 100), double(MinWidth(r.after) * 100), Below(r.before, vs::kBandMinM), Below(r.after, vs::kBandMinM),
                    double(MaxWidth(r.after) * 100), double(MinGap(r.before) * 100), double(MinGap(r.after) * 100), double(MaxJointStep(r.before) * 100),
                    double(MaxJointStep(r.after) * 100), double(MaxWidthStep(r.before) * 100), double(MaxWidthStep(r.after) * 100),
                    double(r.maxHandShift * 100), r.applied, r.saturated, double(EpisodeMin(r, v, r.before) * 100), double(EpisodeMin(r, v, r.after) * 100),
                    double(r.handVert * 100), double(r.handHoriz * 100), double(r.maxHandVert * 100));
        if (!IsCarVault(v))
            continue;
        ++carVaults;
        problem = problem && MinWidth(r.before) < 0.23f;
        // the released vault and its episode: from the release on until the weight is back to 0 (exit hold and fade included). The frames
        // after the episode are the game's following locomotion; they must be byte-identical (in game the motion pose fades back in there)
        bool ended = false;
        for (size_t k = 0; k < r.t.size(); ++k)
        {
            if (r.t[k] < v.on)
                continue;
            ended = ended || (r.t[k] >= v.off && r.weight[k] == 0.0f);
            if (!ended)
            {
                band = band && r.after[k].width >= vs::kBandMinM;
                worstMin = std::fmin(worstMin, r.after[k].width);
            }
            else
            {
                afterEp = afterEp && r.bytesBefore[k] == r.bytesAfter[k];
                if (r.after[k].width < outsideMin)
                {
                    outsideMin = r.after[k].width;
                    outsideAt = v.label + " +" + F2(r.t[k] - v.off) + " s after the release";
                }
            }
        }
        upper = upper && MaxWidth(r.after) <= std::fmax(kBandMaxM, MaxWidth(r.before));
        for (size_t k = 0; k < r.t.size(); ++k)
            if (r.t[k] < v.on && r.bytesBefore[k] != r.bytesAfter[k])
                entry = false;
        exitOk = exitOk && r.bytesBefore.back() == r.bytesAfter.back();
        other = other && !r.otherBonesTouched;
        armT = armT && !r.armTranslationTouched;
        armRot = armRot && r.maxArmTurnDeg <= 0.05f;
        handRigid = handRigid && r.maxHandVsJoint <= 0.0005f;
        worstHand = std::fmax(worstHand, r.maxHandShift);
        worstArmTurn = std::fmax(worstArmTurn, r.maxArmTurnDeg);
        // the limiter never makes a per-frame step of the shoulder joints or of the width larger than the clip's own largest step
        const float gain = std::fmax(MaxJointStep(r.after) - MaxJointStep(r.before), MaxWidthStep(r.after) - MaxWidthStep(r.before));
        worstStepGain = std::fmax(worstStepGain, gain);
        steps = steps && MaxJointStep(r.after) <= MaxJointStep(r.before) + 0.002f && MaxWidthStep(r.after) <= MaxWidthStep(r.before) + 0.002f;
    }
    Check("12 car vaults (8 rifle, 4 unarmed): the fixture has the recorded problem (width below 23 cm without the fix)", carVaults == 12 && problem);
    Check("12 car vaults: the shoulder width stays inside the band on every frame of the released vault and its exit (>= 32.02 cm)", band,
          "lowest " + F2(worstMin * 100.0) + " cm");
    Check("12 car vaults: after the episode the frames are the game's (byte-identical)", afterEp,
          "lowest game width there " + F2(outsideMin * 100.0) + " cm (" + outsideAt + ")");
    Check("12 car vaults: the limiter never widens past the upper band edge (40.31 cm) or the clip's own maximum", upper);
    Check("12 car vaults: before the release the records are the game's (byte-identical)", entry);
    Check("12 car vaults: after the fade the last frame is the game's again (byte-identical)", exitOk);
    Check("12 car vaults: only the shoulder records and the upper-arm rotations are written", other && armT);
    Check("12 car vaults: the upper arms keep their rotation in the Spine3 frame (<= 0.05 deg)", armRot, "worst " + F2(worstArmTurn) + " deg");
    Check("12 car vaults: the hands translate with the shoulder joints only (hand shift = joint shift within 0.5 mm)", handRigid,
          "largest hand shift " + F2(worstHand * 100.0) + " cm");
    Check("12 car vaults: no per-frame step of a shoulder joint (Spine3 frame) or of the width above the clip's own largest step + 2 mm", steps,
          "worst gain " + F2(worstStepGain * 100.0) + " cm");

    // ---- everything the gate keeps off is byte-identical
    for (size_t i = 0; i < fx.vaults.size(); ++i)
    {
        const Vault& v = fx.vaults[i];
        if (IsCarVault(v))
            continue;
        Check("gate off: " + v.label + " (detail " + std::to_string(v.detail) + ", release " + std::to_string(v.release) + ") byte-identical",
              Identical(runs[i]) && runs[i].applied == 0);
    }

    // ---- scenarios on a real vault
    const Vault* rvB = nullptr;
    for (const Vault& v : fx.vaults)
        if (v.label == "rifle_vault_b")
            rvB = &v;
    if (rvB && uvA)
    {
        // melee in hand: weapon drawn, no ranged overlay - limited the same way
        {
            const Run r = Simulate(*rvB, fx, hashes, [](const Vault&, const Frame&, vt::GateInput& gi, bool&) { gi.vehicleVaultRanged = false; });
            Check("scenario: a melee weapon in hand (no ranged overlay) - inside the band on every frame", Below(r.after, vs::kBandMinM) == 0 && r.applied > 0);
        }
        // a mount bit in the middle: the weight is 0 at once, the records are the game's from there on
        {
            const float cut = 0.50f;
            const Run r = Simulate(*uvA, fx, hashes, [cut](const Vault&, const Frame& fr, vt::GateInput& gi, bool&) {
                if (fr.t >= cut)
                    gi.psmReleaseBit = true;
            });
            bool before = false, after = true;
            for (size_t k = 0; k < r.t.size(); ++k)
            {
                if (r.t[k] >= 0.40f && r.t[k] < cut && r.bytesBefore[k] != r.bytesAfter[k])
                    before = true;
                if (r.t[k] >= cut)
                    after = after && r.bytesBefore[k] == r.bytesAfter[k] && r.weight[k] == 0.0f;
            }
            Check("scenario: a mount / scene bit in the middle of the vault - limited before it, untouched after it", before && after);
        }
        // the PSM goes to a crouch / mantle right after the release: no hold, gone within 0.2 s
        for (int next : {3, 8})
        {
            const float off = uvA->off;
            const Run r = Simulate(*uvA, fx, hashes, [off, next](const Vault&, const Frame& fr, vt::GateInput& gi, bool&) {
                if (fr.t >= off)
                    gi.psmDetail = next;
            });
            bool gone = true;
            for (size_t k = 0; k < r.t.size(); ++k)
                if (r.t[k] >= off + 0.2f && r.bytesBefore[k] != r.bytesAfter[k])
                    gone = false;
            Check(std::string("scenario: after the release the PSM turns to ") + (next == 3 ? "a crouch" : "a mantle") + " - no hold, gone within 0.2 s", gone);
        }
        // the release ends while the clip is at its narrowest (forced at the lowest frame): the exit hold keeps the band
        {
            const Run r = Simulate(*uvA, fx, hashes, [](const Vault&, const Frame& fr, vt::GateInput& gi, bool& vv) {
                if (fr.t >= 0.45f)
                {
                    gi.travRelease = false;
                    gi.psmDetail = -1;
                    vv = false;
                }
            });
            Check("scenario: the release ends in the middle of the squeeze - the exit hold keeps the band", Below(r.after, vs::kBandMinM) == 0 &&
                                                                                                             MinWidth(r.before) < 0.20f,
                  "min " + F2(MinWidth(r.before) * 100.0) + " -> " + F2(MinWidth(r.after) * 100.0) + " cm");
        }
        // weight 0 = byte-identical
        {
            bool same = true;
            std::vector<uint8_t> buf(kBones * kStride), orig;
            vs::Ref ref;
            ToRecords(uvA->frames.front(), buf.data());
            vs::Latch(buf.data(), kBones, rig, ref);
            for (const Frame& fr : uvA->frames)
            {
                ToRecords(fr, buf.data());
                orig = buf;
                vs::Apply(buf.data(), kBones, rig, ref, 0.0f);
                same = same && buf == orig;
            }
            Check("weight 0: the limiter writes nothing (every frame byte-identical)", same);
        }
        // V's own pose narrower than the floor (a strong shoulder narrowing setting): the limiter never widens past it. The reference is
        // latched from a recorded frame of about 28 - 30 cm; every frame narrower than it is widened toward it, never beyond it.
        {
            std::vector<uint8_t> buf(kBones * kStride);
            vs::Ref ref;
            bool haveRef = false;
            float refW = 0.0f;
            for (const Frame& fr : uvA->frames)
            {
                ToRecords(fr, buf.data());
                Measure m;
                Measure_(buf.data(), fx.parents, m);
                if (!haveRef && fr.t > 0.3f && m.width < 0.30f && m.width > 0.28f)
                {
                    haveRef = vs::Latch(buf.data(), kBones, rig, ref);
                    refW = m.width;
                }
            }
            bool never = haveRef;
            int widened = 0;
            for (const Frame& fr : uvA->frames)
            {
                ToRecords(fr, buf.data());
                Measure mb, ma;
                Measure_(buf.data(), fx.parents, mb);
                vs::Apply(buf.data(), kBones, rig, ref, 1.0f);
                Measure_(buf.data(), fx.parents, ma);
                // per side: a shoulder joint never ends farther from the mid plane than in V's own pose (or than the game put it)
                for (int k = 0; k < 2; ++k)
                {
                    const float sign = k == 0 ? -1.0f : 1.0f;
                    never = never && sign * ma.jointS3[k].z <= std::fmax(sign * mb.jointS3[k].z, ref.half[k]) + 0.001f;
                }
                widened += ma.width > mb.width + 0.001f ? 1 : 0;
            }
            Check("reference: a narrow own pose (" + F2(refW * 100.0) + " cm) is never exceeded on either side (widened frames " +
                      std::to_string(widened) + ")",
                  never && widened > 0);
        }
    }
    std::printf("vault shoulder: %d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}
