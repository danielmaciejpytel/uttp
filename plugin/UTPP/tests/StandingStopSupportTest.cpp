// Ordinary standing stop support: preserve a reachable planted toe without
// freezing authored swing or bypassing the existing drift/reach/slip guards.
// The database supplies real rig topology; bent legs isolate the policy from
// matcher/clip selection. These are skeleton tests, not terrain/sole validation.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#ifdef UTPP_FOOTLOCK_HEADER
#include UTPP_FOOTLOCK_HEADER
#else
#include "../src/mm/FootLocker.hpp"
#endif

using namespace utpp::mm;
static int passed = 0, failed = 0;
static void Check(bool ok, const char* name)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
    ok ? ++passed : ++failed;
}
template<class T> static void Policy(T& f, bool stop, bool idle)
{
    // The same test can expose the recorded defect against the saved old header.
    if constexpr (requires { f.SetStandingStopSupport(stop, idle); })
        f.SetStandingStopSupport(stop, idle);
}
static float Planar(math::Vec3 a, math::Vec3 b)
{ return std::hypot(a.x - b.x, a.y - b.y); }
static math::Quat X(float degrees)
{ return legik::AxisAngle({1, 0, 0}, degrees * math::kPi / 180.f); }

struct Fixture
{
    FootLocker f;
    std::vector<BoneTransform> pose;
    int hips = -1;
    explicit Fixture(const Database& db, FootLockConfig cfg = {}, float bend = 35.f)
        : pose(db.NumBones())
    {
        if (!f.Init(db, cfg)) return;
        f.SetReplant(false, true);
        f.SetPoleCarry(true);
        f.SetSlip(.05f, .02f);
        for (uint32_t i = 0; i < db.NumBones(); ++i)
            if (db.BoneName(i) == "Hips") hips = static_cast<int>(i);
        if (hips < 0) return;
        pose[size_t(hips)].p = {0, 0, 1};
        for (int side = 0; side < 2; ++side)
        {
            pose[size_t(f.ChainBone(side, 0))] = {{side == 0 ? .1f : -.1f, 0, 0}, X(bend)};
            pose[size_t(f.ChainBone(side, 1))] = {{0, 0, -.45f}, X(-2 * bend)};
            pose[size_t(f.ChainBone(side, 2))] = {{0, 0, -.45f}, X(bend)};
            if (f.LockBone(side) != f.ChainBone(side, 2))
                pose[size_t(f.LockBone(side))].p = {0, .14f, -.04f};
        }
    }
    void Step(float dt, float velocity = 0, float left = 1, float right = 1)
    { f.Update(dt, 0, velocity, 0, left, right, pose); }
    void Establish(int hz)
    { for (int i = 0; i < hz / 3; ++i) Step(1.f / hz); }
    void Residual(float distance, float dt)
    { Step(dt, -distance / dt); }
    math::Vec3 World(int side) const
    {
        auto p = f.ModelSpace(f.Output(), f.LockBone(side)).p;
        float x = 0, y = 0;
        f.CharacterToWorld(p, x, y);
        return {x, y, p.z};
    }
    bool Protected() const
    {
        for (size_t i = 0; i < pose.size(); ++i)
        {
            bool leg = false;
            for (int side = 0; side < 2; ++side)
                for (int link = 0; link < 3; ++link)
                    leg |= int(i) == f.ChainBone(side, link);
            const auto& out = f.Output()[i];
            if (!math::IsFinite(out.p) || !math::IsFinite(out.q)) return false;
            if (std::memcmp(&pose[i].p, &out.p, sizeof(math::Vec3)) != 0) return false;
            if (!leg && std::memcmp(&pose[i], &out, sizeof(BoneTransform)) != 0) return false;
        }
        return true;
    }
};

static void StopAndRelease(const Database& db, int hz)
{
    const float dt = 1.f / hz;
    Fixture held(db), normal(db);
    held.Establish(hz); normal.Establish(hz);
    held.Residual(.03f, dt); normal.Residual(.03f, dt);
    const auto planted = held.World(0), original = normal.World(0);
    Policy(held.f, true, false);
    bool guarded = true;
    float maxDrift = 0;
    for (int i = 0; i < hz / 2; ++i)
    {
        if (i > hz / 5) Policy(held.f, false, true); // stop naturally reaches idle
        held.Step(dt); normal.Step(dt);
        maxDrift = std::max(maxDrift, Planar(planted, held.World(0)));
        guarded &= held.Protected() && held.f.Diagnostics().foot[0].locked &&
                   !held.f.Diagnostics().foot[0].replant && !held.f.Diagnostics().foot[0].clamped;
    }
    const float baselineDrift = Planar(original, normal.World(0));
    std::printf("hz=%d stop support=%.4fmm ordinary settle=%.4fmm\n",
                hz, maxDrift * 1000, baselineDrift * 1000);
    Check(baselineDrift > .015f, "fixture exposes cosmetic drift under an established support");
    Check(maxDrift < .001f, "ordinary stop through idle keeps safe planted world toe below 1mm drift");
    Check(guarded, "support preserves lengths, upper body and reach without standing replant");
    const auto beforeZero = held.World(0);
    held.Step(0);
    Check(Planar(beforeZero, held.World(0)) < .000001f, "zero dt does not advance the planted correction");

    Policy(held.f, false, false); // driver restart/ineligibility contract
    const auto released = held.World(0);
    for (int i = 0; i < hz / 4; ++i) held.Step(dt);
    Check(Planar(released, held.World(0)) > .015f, "restart or ineligibility restores ordinary settling");

    Policy(held.f, true, false);
    for (int i = 0; i < hz / 3; ++i)
    {
        held.pose[size_t(held.f.ChainBone(0, 0))].q = X(35.f + 12.f * std::sin(i * dt * 6.f));
        held.Step(dt, 0, 0, 1); // natural left toe-off, right support remains
    }
    Check(!held.f.Diagnostics().foot[0].locked && held.f.Diagnostics().foot[0].weight == 0,
          "authored toe-off fully releases the swinging leg during the stop episode");
    bool swingExact = true;
    for (int link = 0; link < 3; ++link)
    {
        const auto bone = size_t(held.f.ChainBone(0, link));
        swingExact &= std::memcmp(&held.pose[bone], &held.f.Output()[bone], sizeof(BoneTransform)) == 0;
    }
    Check(swingExact && held.f.Diagnostics().foot[1].locked,
          "swing passes through unchanged while the other foot remains supported");
    Policy(held.f, false, false); // air releases the policy AND authored contacts
    for (int i = 0; i < hz / 3; ++i) held.Step(dt, .5f, 0, 0);
    Check(!held.f.Diagnostics().foot[0].locked && !held.f.Diagnostics().foot[1].locked &&
          std::memcmp(held.pose.data(), held.f.Output().data(), held.pose.size() * sizeof(BoneTransform)) == 0,
          "air contract clears both contacts and returns the complete authored pose");
}

static void EligibilityAndSafety(const Database& db, int hz)
{
    const float dt = 1.f / hz;
    Fixture idle(db), normal(db);
    idle.Establish(hz); normal.Establish(hz);
    Policy(idle.f, false, true); // a fresh idle must not create a stop episode
    idle.Residual(.03f, dt); normal.Residual(.03f, dt);
    for (int i = 0; i < hz / 3; ++i) { idle.Step(dt); normal.Step(dt); }
    Check(Planar(idle.World(0), normal.World(0)) < .000001f,
          "idle alone never acquires standing stop support");
    Policy(idle.f, true, false);
    idle.f.Reset(); normal.f.Reset();
    Policy(idle.f, false, true);
    idle.Establish(hz); normal.Establish(hz);
    idle.Residual(.03f, dt); normal.Residual(.03f, dt);
    for (int i = 0; i < hz / 4; ++i) { idle.Step(dt); normal.Step(dt); }
    Check(Planar(idle.World(0), normal.World(0)) < .000001f,
          "reset clears the previous stop episode even when the next state is idle");
    idle.f.Reset(); Policy(idle.f, true, false);
    for (int i = 0; i < hz / 4; ++i) idle.Step(dt, 0, 0, 0);
    Check(!idle.f.Diagnostics().foot[0].locked && !idle.f.Diagnostics().foot[1].locked,
          "stop eligibility never manufactures missing contacts");

    FootLockConfig noSettle; noSettle.settleSpeed = 0;
    Fixture slipping(db, noSettle);
    slipping.Establish(hz); slipping.Residual(.09f, dt);
    Policy(slipping.f, true, false);
    const auto excessStart = slipping.World(0);
    for (int i = 0; i < hz / 3; ++i) slipping.Step(dt);
    Check(Planar(excessStart, slipping.World(0)) > .005f && slipping.f.Diagnostics().slips > 0 &&
          slipping.f.Diagnostics().foot[0].locked && slipping.Protected(),
          "necessary soft-slip remains active above the configured safe offset");

    Fixture large(db), largeNormal(db);
    large.f.SetSlip(0, .02f); largeNormal.f.SetSlip(0, .02f);
    large.Establish(hz); largeNormal.Establish(hz);
    large.Residual(.12f, dt); largeNormal.Residual(.12f, dt);
    Policy(large.f, true, false);
    large.Step(dt); largeNormal.Step(dt);
    Check(Planar(large.World(0), largeNormal.World(0)) < .000001f,
          "large incompatible support retains the ordinary safety policy with slip disabled");
    large.Residual(.5f, dt);
    Check(!large.f.Diagnostics().foot[0].locked && large.f.Diagnostics().driftUnlocks > 0,
          "maxDrift still releases an invalid planted target");

    Fixture reach(db, {}, 2.f), reachNormal(db, {}, 2.f);
    reach.Establish(hz); reachNormal.Establish(hz);
    reach.Residual(.04f, dt); reachNormal.Residual(.04f, dt);
    Policy(reach.f, true, false);
    reach.Step(dt); reachNormal.Step(dt);
    Check(Planar(reach.World(0), reachNormal.World(0)) < .00001f && reach.Protected(),
          "near-straight unsafe reach keeps ordinary handling and fixed bone lengths");
}

int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    Database db; std::string error;
    if (!db.LoadFromFile(argv[1], 0x72CB314EB01F562Dull, error))
    { std::printf("database: %s\n", error.c_str()); return 2; }
    Fixture sanity(db);
    if (!sanity.f.Ready() || sanity.hips < 0) return 2;
    for (int hz : {30, 60, 144})
    {
        StopAndRelease(db, hz);
        EligibilityAndSafety(db, hz);
    }
    std::printf("SUMMARY %d passed %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
