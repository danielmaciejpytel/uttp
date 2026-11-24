// Speed converter strafe travel (speedconv::StanceLimits, MapConfig::speedConverter bit speedconv::kStanceTravel): the limits of a
// strafing feed and a strafe reversal through MapToChooser. No data files.
#include "../src/mm/MotionAdapter.hpp"
#include <cmath>
#include <cstdio>

using namespace utpp::mm;
using namespace utpp::mm::adapter;
using motiontraj::V3;

static int passed = 0, failed = 0;
static void Check(const char* label, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", label);
    ok ? ++passed : ++failed;
}

// Walk forward (world +y) with the facing held at yaw 0 (aiming strafes), then the stick and the capsule velocity flip to backward at
// once (the game turns the capsule within a feed). Returns the query velocity of the first 0.1 s ahead, aAfter feeds after the flip.
struct Run
{
    V3 current;
    float rate = 1.0f;
};
static Run StrafeReversal(int aConverter, bool aAiming, int aAfter, int32_t aDetailed = kPsmStand, float aSpeed = 1.8f)
{
    const float dt = 1.0f / 60.0f;
    const speedconv::Preset p = speedconv::PresetFor(speedconv::kPresetStronger);
    MapConfig mc;
    mc.speedConverter = aConverter;
    mc.speedConverterFriction = p.friction;
    mc.speedConverterMaxTurnDeg = p.maxTurnDeg;
    mc.strafeWhileAiming = true;
    MapState ms;
    Run out;
    const int before = 90;
    for (int i = 0; i < before + aAfter; ++i)
    {
        const float s = i >= before ? -1.0f : 1.0f;
        const float sp = std::fmin(1.0f, float(i) / 30.0f) * aSpeed;
        const float vel[3] = {0.0f, s * sp, 0.0f}, mv[2] = {0.0f, s};
        MotionFeed f = MakeMotionFeed(vel, mv, dt, 0u, 0, AirFeed{}, 0.0f, 0.0f);
        f.psmDetailed = aDetailed;
        f.aiming = aAiming;
        MapContext ctx;
        ctx.rootYawValid = true;
        const motion::ChooserState cs = MapToChooser(f, mc, ms, ctx);
        out.rate = cs.rateSpeedScale;
    }
    out.current = ms.traj.VelocityBetween(0.0f, 0.1f);
    return out;
}

int main()
{
    using namespace speedconv;
    Limits base;
    base.friction = 2.0f;
    base.maxTurnCos = std::cos(150.0f / motiontraj::kRad2Deg);
    const int stronger = PresetFor(kPresetStronger).bits;
    {
        const Limits l = StanceLimits(base, stronger, true, 5.0f);
        Check("stance limits: without the bit the limits are unchanged", l.friction == base.friction && l.maxTurnCos == base.maxTurnCos);
        const Limits o = StanceLimits(base, stronger | kStanceTravel, false, 5.0f);
        Check("stance limits: orient to movement keeps the limits", o.friction == base.friction && o.maxTurnCos == base.maxTurnCos);
        const Limits s = StanceLimits(base, stronger | kStanceTravel, true, 5.0f);
        Check("stance limits: strafing converts every reversal", s.maxTurnCos == -1.0f);
        Check("stance limits: strafing turns at the reference ground friction", s.friction == 5.0f);
        const Limits k = StanceLimits(base, stronger | kStanceTravel, true, 1.0f);
        Check("stance limits: a higher converter friction is kept", k.friction == 2.0f);
        const Limits n = StanceLimits(base, stronger | kStanceTravel, true, std::nanf(""));
        Check("stance limits: a non-finite ground friction keeps the friction", n.friction == 2.0f && n.maxTurnCos == -1.0f);
        Check("stance limits: the other limits are kept",
              s.acceleration == base.acceleration && s.brakingInput == base.brakingInput && s.stallRatio == base.stallRatio);
    }
    {
        // Step: a reversal of the game velocity at a held input; the strafe limits slow down along the old direction first
        State a, b;
        const Limits keep = base;
        const Limits strafe = StanceLimits(base, stronger | kStanceTravel, true, 5.0f);
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 60; ++i)
        {
            Step(a, V3{0.0f, 180.0f, 0.0f}, true, true, dt, keep, false);
            Step(b, V3{0.0f, 180.0f, 0.0f}, true, true, dt, strafe, false);
        }
        const V3 ra = Step(a, V3{0.0f, -180.0f, 0.0f}, true, true, dt, keep, false);
        const V3 rb = Step(b, V3{0.0f, -180.0f, 0.0f}, true, true, dt, strafe, false);
        Check("step: a reversal keeps the game velocity beyond the max turn", std::fabs(ra.y + 180.0f) < 1.0e-3f);
        Check("step: a strafe reversal is converted (still along the old direction)", rb.y > 100.0f);
        int feeds = 0;
        V3 v = rb;
        while (feeds < 120 && std::fabs(v.y + 180.0f) > 1.0f)
        {
            v = Step(b, V3{0.0f, -180.0f, 0.0f}, true, true, dt, strafe, false);
            ++feeds;
        }
        Check("step: a strafe reversal reaches the game velocity within 0.6 s", feeds <= 36);
        Check("step: the strafe reversal takes longer than one feed", feeds >= 6);
    }
    {
        // MapToChooser: the hook passes the strafe rotation mode and the bit
        const int on = stronger | kStanceTravel;
        const Run aimOn = StrafeReversal(on, true, 4), aimOff = StrafeReversal(stronger, true, 4), walkOn = StrafeReversal(on, false, 4);
        const Run walkOff = StrafeReversal(stronger, false, 4);
        std::printf("INFO strafe reversal query vy (cm/s): bit on %.1f, off %.1f; orient on %.1f, off %.1f\n", aimOn.current.y,
                    aimOff.current.y, walkOn.current.y, walkOff.current.y);
        Check("driver: without the bit a strafe reversal queries the game velocity", aimOff.current.y < -100.0f);
        Check("driver: with the bit a strafe reversal slows down along the old direction first", aimOn.current.y > aimOff.current.y + 80.0f);
        Check("driver: orient to movement is the same with and without the bit",
              std::fabs(walkOn.current.y - walkOff.current.y) < 1.0e-4f && std::fabs(walkOn.current.x - walkOff.current.x) < 1.0e-4f);
        const Run crouchOn = StrafeReversal(on, true, 4, kPsmCrouch, 1.2f), crouchOff = StrafeReversal(stronger, true, 4, kPsmCrouch, 1.2f);
        std::printf("INFO crouch strafe reversal query vy (cm/s): bit on %.1f, off %.1f\n", crouchOn.current.y, crouchOff.current.y);
        Check("driver: a crouched strafe reversal is converted too", crouchOn.current.y > crouchOff.current.y + 40.0f);
        const Run late = StrafeReversal(on, true, 60), lateOff = StrafeReversal(stronger, true, 60);
        Check("driver: one second after the reversal the query is the game velocity again",
              std::fabs(late.current.y - lateOff.current.y) < 0.05f * std::fabs(lateOff.current.y) + 1.0f);
    }
    std::printf("%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
