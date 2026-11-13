// World-frame covariance of capsule translation; the spring is part of the oracle.
#include "../src/mm/OffsetRoot.hpp"
#include <cstdio>

using namespace utpp::mm;
static int passed = 0, failed = 0;
static void Check(const char* name, bool ok)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
    ok ? ++passed : ++failed;
}

int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    Database db;
    std::string error;
    if (!db.LoadFromFile(argv[1], 0x72CB314EB01F562Dull, error)) return 2;
    const float noAnimation[2] = {0, 0};
    OffsetRootConfig cfg;
    cfg.maxDistance = 1.0f;
    cfg.maxYaw = 1.6f;
    for (float degrees : {-179.0f, -90.0f, -15.0f, 0.0f, 2.863479f, 15.0f, 90.0f, 179.0f})
    {
        const float yaw = degrees * math::kPi / 180.0f, dt = 0.02f;
        const float worldVelocity[2] = {3.0f, -4.0f};
        float currentVelocity[2];
        math::Rot2(-yaw, worldVelocity[0], worldVelocity[1], currentVelocity[0], currentVelocity[1]);
        OffsetRoot straight, turned;
        if (!straight.Init(db, 2, cfg) || !turned.Init(db, 2, cfg)) return 2;
        straight.Update(dt, worldVelocity, 0, noAnimation, 0, true);
        turned.Update(dt, currentVelocity, yaw, noAnimation, 0, true);
        float worldX, worldY;
        math::Rot2(yaw, turned.X(), turned.Y(), worldX, worldY);
        const float e = std::hypot(worldX - straight.X(), worldY - straight.Y());
        std::printf("angle=%.6f extraWorldDriftMM=%.6f\n", degrees, e * 1000);
        Check("same world capsule motion has same world root response during turn", e < 2e-6f);

        // A later pure turn must still carry both offset and spring velocity
        // into the new frame; correcting capsule translation must preserve this.
        OffsetRoot seed, seedTurn;
        if (!seed.Init(db, 2, cfg) || !seedTurn.Init(db, 2, cfg)) return 2;
        seed.Update(dt, worldVelocity, 0, noAnimation, 0, true);
        seedTurn.Update(dt, worldVelocity, 0, noAnimation, 0, true);
        seed.Update(dt, noAnimation, 0, noAnimation, 0, true);
        seedTurn.Update(dt, noAnimation, yaw, noAnimation, 0, true);
        math::Rot2(yaw, seedTurn.X(), seedTurn.Y(), worldX, worldY);
        Check("pure turn preserves existing world offset decay", std::hypot(worldX - seed.X(), worldY - seed.Y()) < 2e-6f);
        const float beforeX = seedTurn.X(), beforeY = seedTurn.Y();
        seedTurn.Update(0, currentVelocity, yaw, noAnimation, 0, true);
        Check("zero dt leaves offset unchanged", seedTurn.X() == beforeX && seedTurn.Y() == beforeY);
    }
    std::printf("frame focused: %d PASS %d FAIL\n", passed, failed);
    return failed ? 1 : 0;
}
