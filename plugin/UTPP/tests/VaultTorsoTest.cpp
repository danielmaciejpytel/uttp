// Vault torso limiter (src/mm/VaultTorso.hpp): hood slide with a weapon in hand, REAL data of the recording 2026-11-10 13:00
// (the recorded log, DLL log utpp-2026-11-10-13-00-01.log; fixture = Build\test-data\vaulttorso\
// vaulttorso_20261110.txt made by tools/analysis/vaulttorso_fixture.py: bones 0..28 of every frame of each window + the travRelease timeline).
//
// The vaults go through the PRODUCTION chain: rt::Feed (the game-thread gate / fade / exit hold, fed with the PSM inputs of the frame) and
// rt::FkPass (the FK-stage pass of FkHook.cpp, without any MM-active condition: the vault is the game's pose, MM weight 0) with synthetic QPC
// ticks. Without the fix the game's vault with a rifle leaves the chest below the horizontal (Spine1 -> Spine3 elevation -29 .. -37 deg on six
// walking-speed vaults). Mutation builds MUST report FAIL lines: 1 = the pass compiled out (the build before), 2 = the FK gate needs MM active
// (the first version: inert at MM weight 0), 3 = no exit hold, 4 = the foot-only condition ignored.
// Argument: the fixture file.
#include "../src/mm/VaultTorso.hpp"

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
namespace vt = utpp::mm::vaulttorso;

namespace
{
int passed = 0, failed = 0;
void Check(const std::string& name, bool ok, const std::string& detail = "")
{
    std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : " | ", detail.c_str());
    ok ? ++passed : ++failed;
}

constexpr uint32_t kBones = 29; // Root .. WeaponRight (fixture)
constexpr uint32_t kStride = integ::kGameBoneStride;
constexpr float kDeg = 57.2957795f;
constexpr int64_t kTps = 1000000; // synthetic QPC frequency
constexpr int kNeck = 16, kHead = 22, kWeapon = 28;

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

float El(math::Vec3 d) { return std::atan2(d.z, std::sqrt(d.x * d.x + d.y * d.y)) * kDeg; }

struct Measure
{
    float chest = 0.0f, total = 0.0f;           // elevation above the horizontal (deg): Spine1 -> Spine3, Hips -> Spine3
    float chestPitch = 0.0f, totalPitch = 0.0f; // plane pitch atan2(y, z) of the same vectors (deg, backward < 0), the round-1 report convention
    float head = 0.0f;                          // elevation of Spine3 -> Head (deg)
    float weaponRot[4] = {0, 0, 0, 1};          // model rotation of WeaponRight (quaternion)
};
bool Measure_(const uint8_t* aBuf, const std::vector<int16_t>& aParents, const vt::Rig& aRig, Measure& aM)
{
    aim::Xf model[aim::kMaxFkBones];
    if (!aim::ComputeModel(aBuf, kBones, aParents.data(), kBones, model))
        return false;
    auto pi = [](math::Vec3 d) { return std::atan2(d.y, d.z) * kDeg; };
    const math::Vec3 hips = model[aRig.hips].p, s1 = model[aRig.joint[1]].p, s3 = model[aRig.joint[3]].p;
    aM.chest = El(s3 - s1);
    aM.total = El(s3 - hips);
    aM.chestPitch = pi(s3 - s1);
    aM.totalPitch = pi(s3 - hips);
    aM.head = El(model[kHead].p - s3);
    const math::Quat q = model[kWeapon].q;
    aM.weaponRot[0] = q.x; aM.weaponRot[1] = q.y; aM.weaponRot[2] = q.z; aM.weaponRot[3] = q.w;
    return true;
}
float AngleDeg(const float* a, const float* b)
{
    const float d = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    return 2.0f * std::acos(std::fmin(1.0f, d)) * kDeg;
}

struct Run
{
    std::vector<Measure> before, after;
    std::vector<std::vector<uint8_t>> bytesBefore, bytesAfter;
    std::vector<float> weight, t;
    int saturated = 0, applied = 0;
    float maxAxisZ = 0.0f, maxJointDeg = 0.0f;
    bool otherBonesTouched = false;
};

// Per-frame PSM inputs of a scenario (default: the recorded timeline).
using Scenario = std::function<void(const Vault&, const Frame&, vt::GateInput&)>;

// The production chain of one vault, frame by frame with synthetic QPC ticks: rt::Feed(gate inputs) on the game thread, then rt::FkPass on the
// frame's records (MM weight 0 all along: no MM-active condition exists any more; mutation 2 puts it back and clears it here).
Run Simulate(const Vault& aV, const Fixture& aFx, const vt::Rig& aRig, const std::vector<uint64_t>& aHashes, const Scenario& aScn = Scenario())
{
    const bool verbose = std::getenv("UTPP_VT_VERBOSE") != nullptr;
    Run run;
    int dummy = 0;
    vt::rt::OnDisarmed();
    vt::rt::OnArmed(&dummy, aHashes, aFx.parents);
#if UTPP_VAULTTORSO_MUTATION == 2
    vt::rt::g_mmActiveMirror.store(false); // MM weight 0 / mmActive false during the release
#endif
    float prevT = aV.frames.empty() ? 0.0f : aV.frames.front().t;
    for (const Frame& fr : aV.frames)
    {
        std::vector<uint8_t> buf(kBones * kStride);
        ToRecords(fr, buf.data());
        vt::GateInput gi;
        gi.travRelease = aV.release != 0 && fr.t >= aV.on && fr.t < aV.off;
        gi.psmDetail = gi.travRelease ? aV.detail : -1; // after the release the PSM is back on plain locomotion
        gi.weaponDrawn = aV.weapon != 0;
        gi.vehicleVaultRanged = aV.weapon != 0 && aV.detail == 9;
        if (aScn)
            aScn(aV, fr, gi);
        const int64_t now = static_cast<int64_t>((fr.t + 10.0) * double(kTps));
        vt::rt::Feed(gi, fr.t - prevT, now, kTps);
        prevT = fr.t;
        Measure mb;
        Measure_(buf.data(), aFx.parents, aRig, mb);
        std::vector<uint8_t> orig = buf;
        const float w = vt::rt::g_weight.load();
        const uint64_t appliedBefore = vt::rt::g_applied.load();
        const bool did = vt::rt::FkPass(buf.data(), kBones, &dummy, now, kTps, true);
        if (did && vt::rt::g_applied.load() > appliedBefore)
        {
            ++run.applied;
            if (vt::rt::g_saturated.load() > uint64_t(run.saturated))
            {
                run.saturated = int(vt::rt::g_saturated.load());
                if (verbose)
                    std::printf("    saturated %s t=%.3f w=%.2f\n", aV.label.c_str(), double(fr.t), double(w));
            }
            // only the local records of Spine, Spine1, Spine2 are written (Spine3 takes no share, everything else is the game's)
            for (uint32_t b = 0; b < kBones; ++b)
            {
                const bool written = b == uint32_t(aRig.joint[0]) || b == uint32_t(aRig.joint[1]) || b == uint32_t(aRig.joint[2]);
                if (!written && std::memcmp(orig.data() + b * kStride, buf.data() + b * kStride, kStride) != 0)
                    run.otherBonesTouched = true;
                if (written)
                {
                    const float* q0 = reinterpret_cast<const float*>(orig.data() + b * kStride) + 4;
                    const float* q1 = reinterpret_cast<const float*>(buf.data() + b * kStride) + 4;
                    run.maxJointDeg = std::fmax(run.maxJointDeg, AngleDeg(q0, q1));
                }
            }
        }
        Measure ma;
        Measure_(buf.data(), aFx.parents, aRig, ma);
        if (verbose && fr.t >= aV.off - 0.05f && fr.t <= aV.off + 0.25f)
            std::printf("    %s t=%.3f off=%.3f w=%.2f chest %.1f -> %.1f\n", aV.label.c_str(), double(fr.t), double(aV.off), double(w), double(mb.chest), double(ma.chest));
        run.before.push_back(mb);
        run.after.push_back(ma);
        run.bytesBefore.push_back(orig);
        run.bytesAfter.push_back(buf);
        run.weight.push_back(w);
        run.t.push_back(fr.t);
    }
    vt::rt::OnDisarmed();
    return run;
}

float MinOf(const std::vector<Measure>& m, float Measure::*f)
{
    float v = 1e9f;
    for (const Measure& x : m)
        v = std::fmin(v, x.*f);
    return v;
}
float MaxStep(const std::vector<Measure>& m, float Measure::*f, const std::vector<float>& t, float aFrom, float aTo)
{
    float v = 0.0f;
    for (size_t i = 1; i < m.size(); ++i)
        if (t[i] >= aFrom && t[i] <= aTo)
            v = std::fmax(v, std::fabs(m[i].*f - m[i - 1].*f));
    return v;
}
bool Identical(const Run& r)
{
    for (size_t i = 0; i < r.bytesBefore.size(); ++i)
        if (r.bytesBefore[i] != r.bytesAfter[i])
            return false;
    return true;
}
} // namespace

int main(int argc, char** argv)
{
    // ---- gate truth table (pure): on only for a released vault 9 over a vehicle with a ranged weapon drawn, on foot
    {
        vt::GateInput g;
        g.travRelease = g.weaponDrawn = g.vehicleVaultRanged = true;
        g.psmDetail = 9;
        Check("gate: released vehicle vault 9, ranged weapon drawn = on", vt::Wanted(g));
        vt::GateInput a = g;
        a.weaponDrawn = false;
        vt::GateInput b = g;
        b.vehicleVaultRanged = false; // unarmed / melee / fists / not a vehicle vault / v2 played it
        vt::GateInput c = g;
        c.psmDetail = 8; // a mantle / climb
        vt::GateInput d = g;
        d.travRelease = false; // v2 owns the traversal (no release)
        vt::GateInput e = g;
        e.psmDetail = -1;
        vt::GateInput h = g;
        h.psmReleaseBit = true; // a vehicle / scene / workspot / takedown / carry / vitals / swimming bit
        vt::GateInput k = g;
        k.otherRelease = true;
        Check("gate: off without the weapon-drawn bit", !vt::Wanted(a));
        Check("gate: off when the event is not a vehicle vault with a ranged overlay (unarmed, melee, v2-owned vault)", !vt::Wanted(b));
        Check("gate: off for a climb / mantle (detail 8)", !vt::Wanted(c));
        Check("gate: off without the travRelease reason", !vt::Wanted(d));
        Check("gate: off outside any PSM detail", !vt::Wanted(e));
        Check("gate: off when a vehicle / scene / workspot / takedown / carry / vitals / swimming bit is set (not foot-only)", !vt::Wanted(h) && !vt::FootOnly(h));
        Check("gate: off with any other release reason", !vt::Wanted(k) && !vt::FootOnly(k));
        // the exit hold: only the same episode, weapon still drawn, plain locomotion
        vt::GateInput t = g;
        t.travRelease = false;
        t.psmDetail = -1;
        Check("hold: allowed in the tail of an episode on plain locomotion", vt::HoldAllowed(t, true) && vt::HoldAllowed(t, true));
        Check("hold: not without the latched episode", !vt::HoldAllowed(t, false));
        bool follow = false;
        for (int det : {3, 5, 8, 9, 10, 13, 21, 29, 30, 31})
        {
            vt::GateInput f = t;
            f.psmDetail = det; // crouch, slide, climb, vault, ladders, hover jump, knockdown, crouch sprint, felled
            follow = follow || vt::HoldAllowed(f, true);
        }
        Check("hold: never on a crouch / slide / mantle / ladder / knockdown (the limiter does not rotate what follows)", !follow);
        vt::GateInput m = t;
        m.psmReleaseBit = true;
        vt::GateInput w = t;
        w.weaponDrawn = false;
        Check("hold: not under a mount / scene / death bit, not after the weapon is holstered", !vt::HoldAllowed(m, true) && !vt::HoldAllowed(w, true));
    }
    // ---- fade
    {
        vt::Fade f;
        float w = 0.0f;
        int n = 0;
        while (w < 1.0f && n < 100)
        {
            w = f.Step(1.0f / 60.0f, true, false);
            ++n;
        }
        Check("fade: 0 -> 1 in 0.15 s (9 frames at 60 Hz)", n >= 9 && n <= 10, "frames " + std::to_string(n));
        const float held = f.Step(1.0f / 60.0f, false, true);
        Check("fade: the exit hold keeps the weight", held == 1.0f);
        int hn = 0;
        while (f.Weight() >= 1.0f && hn < 200)
        {
            f.Step(1.0f / 60.0f, false, true);
            ++hn;
        }
        Check("fade: the hold ends after at most 0.6 s even if the pose never recovers", hn >= 30 && hn <= 40, "frames " + std::to_string(hn));
        vt::Fade g;
        for (int i = 0; i < 20; ++i)
            g.Step(1.0f / 60.0f, true, false);
        int dn = 0;
        while (g.Weight() > 0.0f && dn < 100)
        {
            g.Step(1.0f / 60.0f, false, false);
            ++dn;
        }
        Check("fade: 1 -> 0 in 0.15 s without the hold", dn >= 9 && dn <= 11, "frames " + std::to_string(dn));
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
    const vt::Rig rig = vt::ResolveRig(hashes, fx.parents);
    Check("rig: Hips, Spine .. Spine3 resolved from the recorded rig", rig.ok == 1 && rig.hips == 2 && rig.joint[0] == 4 && rig.joint[3] == 13 && rig.fkCount == 14);
    {
        std::vector<uint64_t> bad = hashes;
        bad[13] = 1; // Spine3 missing
        Check("rig: fail closed (a missing bone -> ok 0, nothing is applied)", vt::ResolveRig(bad, fx.parents).ok == 0);
        std::vector<int16_t> par = fx.parents;
        par[13] = 7; // Spine3 not a child of Spine2
        Check("rig: fail closed (broken chain)", vt::ResolveRig(hashes, par).ok == 0);
    }
    const Vault* rvA = nullptr;
    for (const Vault& v : fx.vaults)
        if (v.label == "rifle_vault_a")
            rvA = &v;
    Check("fixture has rifle_vault_a", rvA != nullptr);
    // ---- runtime wiring (rt::Feed / FkPass with synthetic ticks)
    if (rvA)
    {
        size_t worst = 0;
        float lo = 1e9f;
        std::vector<uint8_t> buf(kBones * kStride);
        for (size_t i = 0; i < rvA->frames.size(); ++i)
        {
            ToRecords(rvA->frames[i], buf.data());
            Measure m;
            Measure_(buf.data(), fx.parents, rig, m);
            if (m.chest < lo)
            {
                lo = m.chest;
                worst = i;
            }
        }
        int dummy = 0, other = 0;
        vt::rt::OnDisarmed();
        vt::rt::OnArmed(&dummy, hashes, fx.parents);
#if UTPP_VAULTTORSO_MUTATION == 2
        vt::rt::g_mmActiveMirror.store(false);
#endif
        vt::GateInput g;
        g.travRelease = g.weaponDrawn = g.vehicleVaultRanged = true;
        g.psmDetail = 9;
        const int64_t now = 5000000;
        for (int i = 0; i < 13; ++i)
            vt::rt::Feed(g, 0.1f, now, kTps);
        ToRecords(rvA->frames[worst], buf.data());
        std::vector<uint8_t> orig = buf;
        const bool wrongRig = vt::rt::FkPass(buf.data(), kBones, &other, now, kTps, true);
        Check("runtime: a descriptor of another rig is left alone", !wrongRig && buf == orig);
        const bool stale = vt::rt::FkPass(buf.data(), kBones, &dummy, now + 10 * kTps, kTps, true);
        Check("runtime: a stale published weight (the feed stopped) is not applied", !stale && buf == orig);
        const bool noPersp = vt::rt::FkPass(buf.data(), kBones, &dummy, now, kTps, false);
        Check("runtime: without the perspective permission (FPP) nothing is written", !noPersp && buf == orig);
        const bool ok = vt::rt::FkPass(buf.data(), kBones, &dummy, now, kTps, true);
        Measure m;
        Measure_(buf.data(), fx.parents, rig, m);
        Check("runtime: the armed rig with a fresh weight is limited at MM weight 0 (worst rifle frame: chest leaves the horizontal)",
              ok && m.chest > 3.0f && lo < 0.0f, "chest " + std::to_string(lo) + " -> " + std::to_string(m.chest));
        // the foot-only condition breaks (a vehicle mount bit): the weight drops at once, nothing is written
        vt::GateInput mount = g;
        mount.psmReleaseBit = true;
        vt::rt::Feed(mount, 1.0f / 60.0f, now + 16000, kTps);
        ToRecords(rvA->frames[worst], buf.data());
        orig = buf;
        const bool afterMount = vt::rt::FkPass(buf.data(), kBones, &dummy, now + 16000, kTps, true);
        Check("runtime: a mount / scene bit drops the weight at once (no fade, nothing written)",
              !afterMount && buf == orig && vt::rt::g_weight.load() == 0.0f && !vt::rt::Active(now + 16000, kTps));
        vt::rt::OnDisarmed();
        ToRecords(rvA->frames[worst], buf.data());
        Check("runtime: disarmed = nothing applied and the state is forgotten", !vt::rt::FkPass(buf.data(), kBones, &dummy, now, kTps, true) &&
                                                                                    vt::rt::g_weight.load() == 0.0f);
    }

    // ---- real vaults through the production chain
    std::vector<Run> runs(fx.vaults.size());
    for (size_t i = 0; i < fx.vaults.size(); ++i)
        runs[i] = Simulate(fx.vaults[i], fx, rig, hashes);
    float envChest = 0.0f, envTotal = 0.0f;
    int envN = 0;
    for (size_t i = 0; i < fx.vaults.size(); ++i)
        if (fx.vaults[i].label == "unarmed_vault_a" || fx.vaults[i].label == "unarmed_vault_b" || fx.vaults[i].label == "unarmed_vault_c")
        {
            envChest += MinOf(runs[i].before, &Measure::chest);
            envTotal += MinOf(runs[i].before, &Measure::total);
            ++envN;
        }
    envChest /= float(envN ? envN : 1);
    envTotal /= float(envN ? envN : 1);
    std::printf("no-weapon clip (3 vaults, minima, for reference - NOT the target): chest (Spine1 -> Spine3) %.1f deg, Hips -> Spine3 %.1f deg above the horizontal\n",
                double(envChest), double(envTotal));

    for (size_t i = 0; i < fx.vaults.size(); ++i)
    {
        const Vault& v = fx.vaults[i];
        const Run& r = runs[i];
        const float c0 = MinOf(r.before, &Measure::chest), c1 = MinOf(r.after, &Measure::chest);
        const float t0 = MinOf(r.before, &Measure::total), t1 = MinOf(r.after, &Measure::total);
        const float p0 = MinOf(r.before, &Measure::totalPitch), p1 = MinOf(r.after, &Measure::totalPitch);
        const float q0 = MinOf(r.before, &Measure::chestPitch), q1 = MinOf(r.after, &Measure::chestPitch);
        const float s0 = MaxStep(r.before, &Measure::chest, r.t, -9.0f, 9.0f), s1 = MaxStep(r.after, &Measure::chest, r.t, -9.0f, 9.0f);
        const float u0 = MaxStep(r.before, &Measure::total, r.t, -9.0f, 9.0f), u1 = MaxStep(r.after, &Measure::total, r.t, -9.0f, 9.0f);
        std::printf("  %-22s weapon=%d detail=%d | chest min %6.1f -> %6.1f | Hips>Spine3 min %6.1f -> %6.1f | pitch(Y-Z) chest %7.1f -> %7.1f hips>sp3 %7.1f -> %7.1f"
                    " | max step chest %.1f -> %.1f hips>sp3 %.1f -> %.1f | applied %d saturated %d\n",
                    v.label.c_str(), v.weapon, v.detail, double(c0), double(c1), double(t0), double(t1), double(q0), double(q1), double(p0),
                    double(p1), double(s0), double(s1), double(u0), double(u1), r.applied, r.saturated);
    }

    bool jointOk = true, allSixOk = true, noSat = true, contOk = true, otherOk = true, entryOk = true, exitOk = true, beforeBad = true, headOk = true,
         weaponOk = true, exitHoldOk = true;
    float worstHeadRatio = 0.0f, worstWeaponRatio = 0.0f;
    int rifleBad = 0;
    for (size_t i = 0; i < fx.vaults.size(); ++i)
    {
        const Vault& v = fx.vaults[i];
        const Run& r = runs[i];
        const bool bad = v.label.rfind("rifle_vault_", 0) == 0 && v.label != "rifle_vault_run" && v.label != "rifle_vault_stand";
        if (!bad)
            continue;
        ++rifleBad;
        const float c0 = MinOf(r.before, &Measure::chest), c1 = MinOf(r.after, &Measure::chest);
        const float t1 = MinOf(r.after, &Measure::total);
        beforeBad = beforeBad && c0 < 0.0f; // the recorded problem: the chest hangs below the horizontal
        allSixOk = allSixOk && c1 >= 4.0f && t1 - MinOf(r.before, &Measure::total) >= 18.0f;
        noSat = noSat && r.saturated == 0;
        const float lim = 2.0f;
        contOk = contOk && MaxStep(r.after, &Measure::chest, r.t, -9.0f, 9.0f) <= MaxStep(r.before, &Measure::chest, r.t, -9.0f, 9.0f) + lim &&
                 MaxStep(r.after, &Measure::total, r.t, -9.0f, 9.0f) <= MaxStep(r.before, &Measure::total, r.t, -9.0f, 9.0f) + lim &&
                 MaxStep(r.after, &Measure::chest, r.t, -0.1f, 0.4f) <= MaxStep(r.before, &Measure::chest, r.t, -0.1f, 0.4f) + lim &&
                 MaxStep(r.after, &Measure::total, r.t, v.off - 0.3f, v.off + 0.5f) <= MaxStep(r.before, &Measure::total, r.t, v.off - 0.3f, v.off + 0.5f) + lim &&
                 MaxStep(r.after, &Measure::chest, r.t, v.off - 0.3f, v.off + 0.5f) <= MaxStep(r.before, &Measure::chest, r.t, v.off - 0.3f, v.off + 0.5f) + lim;
        otherOk = otherOk && !r.otherBonesTouched;
        for (size_t k = 0; k < r.t.size(); ++k)
        {
            if (r.t[k] < 0.0f && r.bytesBefore[k] != r.bytesAfter[k])
                entryOk = false;
            // exit hold: after the release the clip is still recovering; the limited chest never dips below what the limiter would give
            if (r.t[k] >= v.off && r.t[k] <= v.off + 0.12f && r.before[k].chest < vt::kFloorDeg - vt::kKneeDeg - 4.0f &&
                r.after[k].chest < vt::kFloorDeg - vt::kKneeDeg - 1.0f)
                exitHoldOk = false;
            // shoulders / head / arms / weapon follow the chest: their pitch change is bounded by the targeted correction
            const float dc = r.after[k].chest - r.before[k].chest;
            if (dc > 3.0f)
            {
                const float dh = std::fabs(r.after[k].head - r.before[k].head);
                const float dw = AngleDeg(r.before[k].weaponRot, r.after[k].weaponRot);
                worstHeadRatio = std::fmax(worstHeadRatio, dh / dc);
                worstWeaponRatio = std::fmax(worstWeaponRatio, dw / dc);
                headOk = headOk && dh <= 1.2f * dc + 2.0f;
                weaponOk = weaponOk && dw <= 1.25f * dc + 3.0f;
            }
        }
        if (r.bytesBefore.back() != r.bytesAfter.back())
            exitOk = false;
        // the largest turn of a joint: share * T, T read back from the chest correction (the chest vector turns by about 0.87 T)
        jointOk = jointOk && r.maxJointDeg <= 25.0f;
    }
    std::printf("head / weapon pitch change per degree of chest correction (worst frame): head %.2f weapon %.2f\n", double(worstHeadRatio), double(worstWeaponRatio));
    Check("six rifle vaults: the fixture has the recorded problem (chest below the horizontal without the fix)", rifleBad == 6 && beforeBad);
    Check("six rifle vaults: the chest no longer hangs below the horizontal (minimum >= +4 deg, was -29 .. -37) and Hips > Spine3 gains >= 18 deg", allSixOk);
    Check("six rifle vaults: the target elevation is never out of reach (no solver saturation)", noSat);
    Check("six rifle vaults: no per-frame step above the clip's own maximum + 2 deg (whole window, entry and exit)", contOk);
    Check("six rifle vaults: before the release the records are the game's (entry continuity: byte-identical)", entryOk);
    Check("six rifle vaults: after the fade the last frame is the game's again (exit: byte-identical)", exitOk);
    Check("six rifle vaults: the exit hold keeps the chest limited in the 0.12 s after the release while the clip is still below the floor", exitHoldOk);
    Check("six rifle vaults: only Spine, Spine1, Spine2 are written (hips, legs, root, Spine3 and everything above untouched)", otherOk);
    Check("six rifle vaults: head pitch change <= 1.2 x the chest correction + 2 deg (Spine3 takes no share)", headOk);
    Check("six rifle vaults: weapon (WeaponRight) turn <= 1.25 x the chest correction + 3 deg (the weapon follows the chest)", weaponOk);
    Check("six rifle vaults: no spine joint is turned by more than 25 deg (anatomical bound, no folded torso on skin)", jointOk);

    // the other armed vaults (run / standing): their chest never hangs below the horizontal, the limiter leaves them alone
    for (size_t i = 0; i < fx.vaults.size(); ++i)
    {
        const Vault& v = fx.vaults[i];
        if (v.label != "rifle_vault_run" && v.label != "rifle_vault_stand")
            continue;
        const Run& r = runs[i];
        Check("rifle vault " + v.label.substr(12) + ": a vault whose chest never hangs below the horizontal is left alone (byte-identical)", Identical(r) && r.applied == 0);
    }

    // ---- everything the gate keeps off is byte-identical (through the same chain, not through the limiter)
    for (size_t i = 0; i < fx.vaults.size(); ++i)
    {
        const Vault& v = fx.vaults[i];
        const bool off = v.label.rfind("unarmed_", 0) == 0 || v.label == "rifle_mantle_motion" || v.label == "rifle_climb_released";
        if (!off)
            continue;
        Check("gate off: " + v.label + " (weapon " + std::to_string(v.weapon) + ", detail " + std::to_string(v.detail) + ", release " +
                  std::to_string(v.release) + ") frames are byte-identical",
              Identical(runs[i]) && runs[i].applied == 0);
    }

    // ---- scenarios of the production gate on a real rifle vault (rifle_vault_b)
    const Vault* rvB = nullptr;
    for (const Vault& v : fx.vaults)
        if (v.label == "rifle_vault_b")
            rvB = &v;
    if (rvB)
    {
        // (1) a vehicle mount bit appears in the middle of the slide: the weight is 0 at once and the records are the game's from there on
        {
            const float cut = 0.70f; // the chest is below the floor from about +0.55 s
            const Run r = Simulate(*rvB, fx, rig, hashes, [cut](const Vault&, const Frame& fr, vt::GateInput& gi) {
                if (fr.t >= cut)
                    gi.psmReleaseBit = true;
            });
            bool before = false, after = true, zero = true;
            for (size_t k = 0; k < r.t.size(); ++k)
            {
                if (r.t[k] >= 0.55f && r.t[k] < cut && r.bytesBefore[k] != r.bytesAfter[k])
                    before = true;
                if (r.t[k] >= cut)
                {
                    after = after && r.bytesBefore[k] == r.bytesAfter[k];
                    zero = zero && r.weight[k] == 0.0f;
                }
            }
            Check("scenario: a mount / scene bit in the middle of the slide - the limiter works before it, drops at once after it (weight 0, records untouched)",
                  before && after && zero);
        }
        // (2) the PSM goes to a crouch / mantle right after the release: the hold ends, the weight fades in 0.15 s and nothing is rotated afterwards
        for (int next : {3, 8})
        {
            const float off = rvB->off;
            const Run r = Simulate(*rvB, fx, rig, hashes, [off, next](const Vault&, const Frame& fr, vt::GateInput& gi) {
                if (fr.t >= off)
                    gi.psmDetail = next;
            });
            bool gone = true;
            for (size_t k = 0; k < r.t.size(); ++k)
                if (r.t[k] >= off + 0.2f && r.bytesBefore[k] != r.bytesAfter[k])
                    gone = false;
            Check(std::string("scenario: after the release the PSM turns to ") + (next == 3 ? "a crouch" : "a mantle") +
                      " - no hold, the limiter is gone within 0.2 s and rotates nothing that follows",
                  gone);
        }
        // (2b) the release ends while the game's clip is still below the floor (here forced 0.1 s before the lowest point): the exit hold keeps
        // the limiter on until the chest is back above the floor - without it the weight would fade over 0.15 s and the chest would dip
        {
            const Run r = Simulate(*rvB, fx, rig, hashes, [](const Vault&, const Frame& fr, vt::GateInput& gi) {
                if (fr.t >= 0.55f)
                {
                    gi.travRelease = false; // the PSM is already back on plain locomotion, the v2 event flag is still set
                    gi.psmDetail = -1;
                }
            });
            float lowest = 1e9f, lowestBefore = 1e9f;
            for (size_t k = 0; k < r.t.size(); ++k)
                if (r.t[k] >= 0.55f && r.t[k] <= 0.80f)
                {
                    lowest = std::fmin(lowest, r.after[k].chest);
                    lowestBefore = std::fmin(lowestBefore, r.before[k].chest);
                }
            Check("scenario: the release ends while the clip is still below the floor - the exit hold keeps the chest at >= +3 deg",
                  lowest >= 3.0f && lowestBefore < -20.0f, "chest min " + std::to_string(lowestBefore) + " -> " + std::to_string(lowest));
        }
        // (3) the weapon is holstered right after the release: no hold
        {
            const float off = rvB->off;
            const Run r = Simulate(*rvB, fx, rig, hashes, [off](const Vault&, const Frame& fr, vt::GateInput& gi) {
                if (fr.t >= off)
                    gi.weaponDrawn = false;
            });
            bool gone = true;
            for (size_t k = 0; k < r.t.size(); ++k)
                if (r.t[k] >= off + 0.2f && r.bytesBefore[k] != r.bytesAfter[k])
                    gone = false;
            Check("scenario: the weapon is holstered after the release - no hold", gone);
        }
        // weight 0 = byte-identical, even for the worst frame
        {
            bool same = true;
            std::vector<uint8_t> buf(kBones * kStride), orig;
            for (const Frame& fr : rvB->frames)
            {
                ToRecords(fr, buf.data());
                orig = buf;
                vt::Apply(buf.data(), kBones, rig, 0.0f);
                same = same && buf == orig;
            }
            Check("weight 0: the limiter writes nothing (every frame of a rifle vault byte-identical)", same);
        }
    }
    std::printf("vault torso: %d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}
