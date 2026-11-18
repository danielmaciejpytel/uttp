// Pelvis stage of the foot locker (mm/PelvisPlacement.hpp): the solver alone (reach bound, horizontal bound, rate-independent spring,
// fade when not allowed) and inside the foot locker on the real rig topology (a stretched locked foot becomes reachable, a pose
// without a stretched foot is left alone, the stage off is bit for bit the leg tail without it). 30, 60 and 144 Hz.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/mm/FootLocker.hpp"
#include "../src/mm/PelvisPlacement.hpp"

using namespace utpp::mm;
static int passed = 0, failed = 0;
static void Check(bool ok, const char* name, int hz = 0)
{
    std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name, hz ? " @Hz=" : "", hz ? std::to_string(hz).c_str() : "");
    ok ? ++passed : ++failed;
}
static float Len(math::Vec3 v) { return std::sqrt(math::Dot(v, v)); }
static math::Quat X(float degrees) { return legik::AxisAngle({1, 0, 0}, degrees * math::kPi / 180.f); }

// two vertical legs of 0.9 m under a hip at 0.95 m, ankles at 0.05 m
static void Legs(PelvisLeg (&l)[2])
{
    for (int s = 0; s < 2; ++s)
    {
        l[s].hip = {s == 0 ? .1f : -.1f, 0, .95f};
        l[s].ankle = {s == 0 ? .1f : -.1f, 0, .08f};
        l[s].target = l[s].ankle;
        l[s].length = .9f;
        l[s].footLength = .15f;
    }
}

static void Solver()
{
    PelvisPlacementConfig c;
    PelvisLeg l[2];
    Legs(l);
    const math::Vec3 rest = PelvisPlacement::TargetOffset(c, l);
    Check(Len(rest) < 1e-6f, "solver: both feet at their animated ankles -> no offset (no bob on flat ground)");
    // left foot held 0.3 m behind: beyond reach
    l[0].target = l[0].ankle + math::Vec3{0, -.3f, 0};
    for (float heel : {0.0f, 0.5f})
    {
        c.heelLift = heel;
        const math::Vec3 o = PelvisPlacement::TargetOffset(c, l);
        // reach bound target (heel share moved towards the animated ankle)
        math::Vec3 t = l[0].target;
        t.y += std::fmin(.3f, l[0].footLength) * heel;
        const float d = Len(t - (l[0].hip + o));
        Check(o.z < 0.0f && d <= c.reach * l[0].length + 1e-4f, heel > 0 ? "solver: stretched leg reachable at reach x length (heel share 0.5)"
                                                                            : "solver: stretched leg reachable at reach x length (heel share 0)");
    }
    c.heelLift = .5f;
    const math::Vec3 o = PelvisPlacement::TargetOffset(c, l);
    Check(std::fabs(o.y - c.rebalance * .5f * -.3f) < 1e-5f, "solver: horizontal rebalance = weight x mean planar foot offset");
    l[0].target = l[0].ankle + math::Vec3{0, -2.0f, 0};
    l[1].target = l[1].ankle + math::Vec3{0, -2.0f, 0};
    const math::Vec3 far = PelvisPlacement::TargetOffset(c, l);
    Check(std::hypot(far.x, far.y) <= c.maxHorizontal + 1e-6f, "solver: horizontal offset capped at maxHorizontal");
    Check(far.z >= -c.maxDrop - 1e-6f, "solver: drop bounded by maxDrop");
    l[0].hip.x = NAN;
    Check(Len(PelvisPlacement::TargetOffset(c, l)) == 0.0f, "solver: non-finite input -> no offset");
}

// spring: same response at a fixed time at any update rate, no overshoot, fade to zero when not allowed
static void Spring()
{
    PelvisLeg l[2];
    Legs(l);
    l[0].target = l[0].ankle + math::Vec3{0, -.3f, 0};
    const float goal = PelvisPlacement::TargetOffset(PelvisPlacementConfig{}, l).z;
    float at02[3] = {};
    int k = 0;
    for (int hz : {30, 60, 144})
    {
        PelvisPlacement p;
        const float dt = 1.0f / static_cast<float>(hz);
        bool monotonic = true, bounded = true;
        float prev = 0.0f;
        const int n02 = static_cast<int>(std::lround(0.2f * static_cast<float>(hz)));
        for (int i = 0; i < hz; ++i)
        {
            const float z = p.Update(dt, l, true).z;
            monotonic &= z <= prev + 1e-6f;
            bounded &= z >= goal - 1e-5f;
            prev = z;
            if (i + 1 == n02)
                at02[k] = z;
        }
        Check(monotonic && bounded, "spring: step response monotonic, no overshoot", hz);
        Check(std::fabs(prev - goal) < 1e-3f, "spring: settles on the target within 1 s", hz);
        bool fade = true;
        prev = p.Offset().z;
        for (int i = 0; i < hz; ++i)
        {
            const float z = p.Update(dt, l, false).z;
            fade &= z >= prev - 1e-6f && z <= 1e-6f;
            prev = z;
        }
        Check(fade && std::fabs(prev) < 1e-3f, "spring: not allowed -> fades back to zero without overshoot", hz);
        ++k;
    }
    Check(std::fabs(at02[0] - at02[1]) < 1e-4f && std::fabs(at02[1] - at02[2]) < 1e-4f,
          "spring: offset at 0.2 s independent of the update rate (30 / 60 / 144 Hz)");
}

struct Rig
{
    FootLocker f;
    std::vector<BoneTransform> pose;
    int hips = -1;
    Rig(const Database& db, bool aPelvis, float bend)
        : pose(db.NumBones())
    {
        FootLockConfig cfg;
        cfg.settleSpeed = 0.0f; // the lock holds still while the capsule stands
        if (!f.Init(db, cfg))
            return;
        f.SetReplant(false, false);
        f.SetPoleCarry(true);
        f.SetSlip(0.0f, .02f);
        if (aPelvis)
        {
            PelvisPlacementConfig pc;
            pc.heelLift = 0.0f; // exact reach bound: the leg IK here keeps the foot orientation
            f.SetPelvisPlacement(true, pc);
            f.SetPelvisAllowed(true);
        }
        for (uint32_t i = 0; i < db.NumBones(); ++i)
            if (db.BoneName(i) == "Hips")
                hips = static_cast<int>(i);
        if (hips < 0)
            return;
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
    void Step(float dt, float vy, float l = 1, float r = 1) { f.Update(dt, 0, vy, 0, l, r, pose); }
    float Ratio(int side) const
    {
        const auto o = f.Output();
        const math::Vec3 a = f.ModelSpace(o, f.ChainBone(side, 0)).p, b = f.ModelSpace(o, f.ChainBone(side, 1)).p,
                         c = f.ModelSpace(o, f.ChainBone(side, 2)).p;
        return Len(c - a) / (Len(b - a) + Len(c - b));
    }
    float LockError(int side, math::Vec3 aWorld) const
    {
        const math::Vec3 p = f.ModelSpace(f.Output(), f.LockBone(side)).p;
        float x = 0, y = 0;
        f.CharacterToWorld(p, x, y);
        return std::hypot(x - aWorld.x, y - aWorld.y);
    }
};

// both feet locked, the capsule walks 12 cm away (both locked toes 12 cm behind their animated toes), then stands
static void Stretch(const Database& db, int hz)
{
    const float dt = 1.0f / static_cast<float>(hz);
    Rig off(db, false, 8.0f), on(db, true, 8.0f);
    for (Rig* r : {&off, &on})
        for (int i = 0; i < hz / 3; ++i)
            r->Step(dt, 0);
    math::Vec3 lockW{};
    {
        const math::Vec3 p = on.f.ModelSpace(on.f.Output(), on.f.LockBone(0)).p;
        on.f.CharacterToWorld(p, lockW.x, lockW.y);
    }
    const int move = static_cast<int>(std::lround(0.2f * static_cast<float>(hz)));
    for (Rig* r : {&off, &on})
        for (int i = 0; i < move; ++i)
            r->Step(dt, .12f / (static_cast<float>(move) * dt));
    const uint64_t c0off = off.f.Diagnostics().clamps, c0on = on.f.Diagnostics().clamps;
    for (int i = 0; i < hz / 2; ++i)
    {
        off.Step(dt, 0);
        on.Step(dt, 0);
    }
    const uint64_t holdOff = off.f.Diagnostics().clamps - c0off, holdOn = on.f.Diagnostics().clamps - c0on;
    std::printf("  stretch @%d Hz: clamps while standing off %llu on %llu | ratio off %.4f on %.4f | pelvis z %.3f m | lock error on %.4f m\n", hz,
                static_cast<unsigned long long>(holdOff), static_cast<unsigned long long>(holdOn), static_cast<double>(off.Ratio(0)),
                static_cast<double>(on.Ratio(0)), static_cast<double>(on.f.Diagnostics().pelvisOffset.z), static_cast<double>(on.LockError(0, lockW)));
    Check(holdOff > 0, "rig: without the pelvis stage the stretched lock clamps", hz);
    Check(holdOn * 10 <= holdOff * 3, "rig: pelvis stage removes >= 70 % of the clamps", hz);
    Check(on.Ratio(0) <= 0.9951f && on.Ratio(1) <= 0.9951f, "rig: settled legs within 0.995 of their length", hz);
    Check(on.LockError(0, lockW) < 0.005f, "rig: locked toe stays on its lock point", hz);
    Check(on.f.Diagnostics().pelvisOffset.z < -0.001f && on.f.Diagnostics().pelvisOffset.z > -0.10f, "rig: pelvis drops a few cm", hz);
}

// bent legs, feet locked and standing: no offset; then the stage switched off returns the old output bit for bit
static void Neutral(const Database& db, int hz)
{
    const float dt = 1.0f / static_cast<float>(hz);
    Rig ref(db, false, 25.0f), on(db, true, 25.0f), toggled(db, true, 25.0f);
    toggled.f.SetPelvisPlacement(false);
    float maxOff = 0.0f;
    bool same = true, sameToggled = true;
    for (int i = 0; i < hz; ++i)
    {
        const float vy = i < hz / 2 ? 0.0f : .05f; // a small capsule drift (lock offsets of a few cm)
        ref.Step(dt, vy);
        on.Step(dt, vy);
        toggled.Step(dt, vy);
        maxOff = std::fmax(maxOff, Len(on.f.Diagnostics().pelvisOffset));
        if (maxOff == 0.0f) // while no offset has been applied the output is the leg tail without the stage
            same &= std::memcmp(ref.f.Output().data(), on.f.Output().data(), ref.f.Output().size_bytes()) == 0;
        sameToggled &= std::memcmp(ref.f.Output().data(), toggled.f.Output().data(), ref.f.Output().size_bytes()) == 0;
    }
    std::printf("  neutral @%d Hz: max pelvis offset %.4f m\n", hz, static_cast<double>(maxOff));
    Check(maxOff < 0.02f, "rig: bent legs with small lock offsets -> pelvis offset < 2 cm", hz);
    Check(same, "rig: zero pelvis offset -> output identical to the leg tail without the stage", hz);
    Check(sameToggled, "rig: stage switched off -> output bit for bit the leg tail without it", hz);
    // airborne / not allowed: the offset fades to zero
    on.f.SetPelvisAllowed(false);
    for (int i = 0; i < hz; ++i)
        on.Step(dt, 0);
    Check(Len(on.f.Diagnostics().pelvisOffset) < 1e-3f, "rig: placement not allowed -> offset faded out", hz);
}

int main(int argc, char** argv)
{
    if (argc != 2)
        return 2;
    Database db;
    std::string error;
    if (!db.LoadFromFile(argv[1], 0x72CB314EB01F562Dull, error))
    {
        std::printf("database: %s\n", error.c_str());
        return 2;
    }
    Solver();
    Spring();
    for (int hz : {30, 60, 144})
    {
        Stretch(db, hz);
        Neutral(db, hz);
    }
    std::printf("%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
