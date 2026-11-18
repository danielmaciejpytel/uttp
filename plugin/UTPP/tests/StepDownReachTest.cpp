// Step-down reach for deeper steps (MapConfig::stepDownReach) and the chooser time to land at the reference gravity
// (motion::ReferenceTimeToLand, MatcherConfig::farJumpGravityScale).
//
// Feed profile: a recorded 0.30-0.31 m drop from standing (12 airborne feeds at ~60 Hz, game gravity ~16 m/s^2, the floor probed on every second
// feed and missed in between, the PSM Fall detail on the last airborne feed, back on the ground on the next feed). With the reach off
// it turns InAir on its last airborne feed (the fall-loop feed before the land clip); with the reach on it stays on the ground. A drop that
// is deeper than the reach, a drop that lasts longer than the reach time, and a floor that is never probed still turn InAir.
#include <cmath>
#include <cstdio>
#include <vector>
#include "../src/mm/MotionAdapter.hpp"
#include "../src/mm/ChooserMatcher.hpp"

using namespace utpp::mm;
using namespace utpp::mm::motion;
static int passed = 0, failed = 0;
#define CHECK(c, ...) do { if (c) ++passed; else { ++failed; std::printf("FAIL " __VA_ARGS__); std::printf("\n"); } } while (0)

struct Row
{
    float dt, vz, dist; // s, m/s, m (-1 = no floor known)
    bool onGround;
    int psm;
};

// recorded rows (vz, dt; a probe hit on every second feed, a miss (-1) in between and on the PSM Fall feed; recorded depth 0.30-0.31 m:
// the vz-integrated drop plus the last probe distance came to 0.303 m)
static std::vector<Row> RecordedDrop(float aDepthScale = 1.0f, bool aProbe = true, bool aPsmFall = true)
{
    const float dts[] = {0.0163f, 0.0167f, 0.0174f, 0.0157f, 0.0176f, 0.0165f, 0.0160f, 0.0167f, 0.0173f, 0.0168f};
    const float vzs[] = {-0.15f, -0.41f, -0.68f, -0.96f, -1.21f, -1.49f, -1.75f, -2.01f, -2.28f, -2.55f};
    std::vector<Row> rows;
    for (int i = 0; i < 20; ++i)
        rows.push_back({1.0f / 60.0f, 0.0f, 0.0f, true, adapter::kPsmStand});
    const float depth = 0.31f * aDepthScale;
    float fallen = 0.0f;
    for (int i = 0; i < 10; ++i)
    {
        fallen -= vzs[i] * dts[i];
        const float dist = aProbe && i % 2 == 0 && i != 9 ? std::fmax(depth - fallen, 0.0f) : -1.0f;
        rows.push_back({dts[i], vzs[i], dist, false, i == 9 && aPsmFall ? int(adapter::kPsmFall) : int(adapter::kPsmStand)});
    }
    rows.push_back({0.033f, -3.08f, 0.0f, true, adapter::kPsmStand});
    for (int i = 0; i < 10; ++i)
        rows.push_back({1.0f / 60.0f, 0.0f, 0.0f, true, adapter::kPsmStand});
    return rows;
}

// movementMode per feed; mc as the runtime builds it (grace 0.12 s, step reach 0.30 m)
static std::vector<int> Modes(const std::vector<Row>& aRows, bool aReach)
{
    adapter::MapConfig mc;
    mc.groundDropGraceS = adapter::kGroundDropGraceSeconds;
    mc.groundDropStepM = adapter::kGroundDropStepM;
    mc.gravity = 16.0f;
    mc.stepDownReach = aReach;
    adapter::MapState ms;
    std::vector<int> out;
    for (const Row& r : aRows)
    {
        const float vel[3] = {0.0f, 0.0f, r.vz}, mv[2] = {0.0f, 0.0f};
        adapter::AirFeed air;
        air.valid = true;
        air.onGround = r.onGround;
        air.distanceToGround = r.dist;
        adapter::MotionFeed f = adapter::MakeMotionFeed(vel, mv, r.dt, 0u, 0, air, 0.0f, 0.0f);
        f.psmDetailed = r.psm;
        adapter::MapContext ctx;
        ctx.rootYawValid = true;
        const ChooserState cs = adapter::MapToChooser(f, mc, ms, ctx);
        out.push_back(cs.movementMode);
    }
    return out;
}

static int AirFeeds(const std::vector<int>& aModes)
{
    int n = 0;
    for (int m : aModes)
        n += m == 1 ? 1 : 0;
    return n;
}

int main()
{
    // identity of the defaults: the flag is off unless the build asks for it
    CHECK(adapter::MapConfig{}.stepDownReach == adapter::kStepDownReachDefault, "MapConfig default follows kStepDownReachDefault");
    CHECK(MatcherConfig{}.farJumpGravityScale == kFarJumpGravityScaleDefault, "MatcherConfig default follows kFarJumpGravityScaleDefault");

    const std::vector<Row> drop = RecordedDrop();
    const std::vector<int> off = Modes(drop, false), on = Modes(drop, true);
    CHECK(AirFeeds(off) >= 1, "reach off: the recorded 0.31 m drop turns InAir (%d air feeds)", AirFeeds(off));
    CHECK(AirFeeds(on) == 0, "reach on: the recorded 0.31 m drop stays on the ground (%d air feeds)", AirFeeds(on));
    // 0.43 m without the PSM Fall detail (inside the reach depth; with it the floor is 0.13 m away on the last feed, beyond the PSM Fall reach)
    CHECK(AirFeeds(Modes(RecordedDrop(1.4f, true, false), true)) == 0, "reach on: a 0.43 m drop of the same duration stays on the ground");
    // 0.62 m: deeper than kStepDownReachM
    CHECK(AirFeeds(Modes(RecordedDrop(2.0f, true, false), true)) >= 1, "reach on: a 0.62 m drop turns InAir");
    CHECK(AirFeeds(Modes(RecordedDrop(1.4f), true)) >= 1, "reach on: a 0.43 m drop whose PSM Fall feed is 0.13 m above the floor turns InAir");
    // never probed: no floor, no reach
    CHECK(AirFeeds(Modes(RecordedDrop(1.0f, false), true)) >= 1, "reach on: a drop with no probe hit turns InAir");
    // a drop that lasts longer than kStepDownReachMaxS (same feeds, every dt doubled)
    {
        std::vector<Row> slow = RecordedDrop();
        for (Row& r : slow)
            if (!r.onGround)
                r.dt *= 2.0f;
        CHECK(AirFeeds(Modes(slow, true)) >= 1, "reach on: a drop that lasts past kStepDownReachMaxS turns InAir");
    }
    // a probe miss (-1) on the PSM Fall feed after a hit: the last hit of the same contact loss is dead-reckoned
    {
        std::vector<Row> miss = RecordedDrop();
        bool hit = false;
        for (Row& r : miss)
            if (!r.onGround)
            {
                if (hit)
                    r.dist = -1.0f;
                hit = hit || r.dist >= 0.0f;
            }
        CHECK(AirFeeds(Modes(miss, true)) == 0, "reach on: probe misses after the first hit keep its floor (dead-reckoned)");
        CHECK(AirFeeds(Modes(miss, false)) >= 1, "reach off: the same feeds turn InAir as before");
    }

    // chooser time to land at the reference gravity
    CHECK(std::fabs(ReferenceTimeToLand(1.0f, kChooserReferenceGravity) - 1.0f) < 1e-6f, "reference gravity: unchanged");
    CHECK(ReferenceTimeToLand(0.7f, 0.0f) == 0.7f, "unknown gravity: unchanged");
    {
        // the far-jump threshold (1.2 s) acts at 1.2 * sqrt(9.81 / g) of game flight time
        const float g = 16.0f, thr = 1.2f * std::sqrt(kChooserReferenceGravity / g);
        CHECK(ReferenceTimeToLand(thr - 0.01f, g) < 1.2f && ReferenceTimeToLand(thr + 0.01f, g) >= 1.2f,
              "16 m/s^2: the 1.2 s threshold maps to %.3f s of flight", thr);
        CHECK(std::fabs(thr - 0.9397f) < 1e-3f, "16 m/s^2 threshold 0.94 s (%.4f)", thr);
        // a same-height drop: sqrt(2h/g) at the game gravity reads as sqrt(2h/g_ref)
        const float h = 2.0f;
        CHECK(std::fabs(ReferenceTimeToLand(std::sqrt(2.0f * h / g), g) - std::sqrt(2.0f * h / kChooserReferenceGravity)) < 1e-4f,
              "a drop of the same height reads the reference flight time");
    }

    std::printf("%s step_down_reach: %d passed, %d failed\n", failed ? "FAIL" : "PASS", passed, failed);
    return failed ? 1 : 0;
}
