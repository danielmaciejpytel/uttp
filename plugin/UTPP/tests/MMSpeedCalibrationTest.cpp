// UTPP 0.5.2 offline test: per-database speed calibration (mm/SpeedCalibration.hpp).
// Usage: utpp_mm_speedcal_test <motion_subset.utppmm> [motion_full.utppmm]
// motion: derived loop speeds equal the 0.5.1 hard-coded values (2.19 / 5.46 / 7.65 / 2.46) within 0.01 m/s, so the motion
// database keeps its behaviour. Sidecar parsing: overrides, explicit points, rejection keeps the derived values.

#include <cmath>
#include <cstdio>
#include <string>

#include "../src/BaseDatabase.hpp"
#include "../src/mm/BaseMatcher.hpp"
#include "../src/mm/SpeedCalibration.hpp"

using namespace utpp::mm;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(const char* aName, bool aOk, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
}
bool Near(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

// Same state masks as MMDriver::ComputeStateTagMasksLocked (exclude lands on).
SpeedCalibration Derive(const Database& aDb)
{
    const uint64_t excl = BaseMatcher::TagMaskContaining(aDb, "_Lands_") | BaseMatcher::TagMaskContaining(aDb, "FromTraversal");
    return DeriveSpeedCalibration(aDb, BaseMatcher::TagMaskContaining(aDb, "Stand_") & ~excl,
                                  BaseMatcher::TagMaskContaining(aDb, "Crouch_") & ~excl);
}

bool Increasing(const SpeedPoints& m)
{
    for (int i = 1; i < m.count; ++i)
        if (!(m.db[i] > m.db[i - 1]) || !(m.game[i] > m.game[i - 1]))
            return false;
    return m.count > 0 && m.db[0] > 0.1f;
}

bool Load(const char* aPath, Database& aDb)
{
    std::string err;
    if (!aDb.LoadFromFile(aPath, 0x72CB314EB01F562Dull, err))
    {
        std::printf("[INFO] %s not loaded: %s\n", aPath, err.c_str());
        return false;
    }
    return true;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mm_speedcal_test <motion_subset.utppmm> [motion_full.utppmm]\n");
        return 2;
    }
    // name matcher
    Check("loop name matcher", IsClassLoopName("M_Neutral_Walk_Loop_F", "Walk", true) &&
                                   IsClassLoopName("M_Neutral_Walk_Loop_F_L_20", "Walk", true) &&
                                   IsClassLoopName("X_Walk_Loop_F_Walking", "Walk", true) &&
                                   !IsClassLoopName("M_Neutral_Walk_Loop_FL", "Walk", true) &&
                                   !IsClassLoopName("M_Neutral_Walk_Loop_B", "Walk", true) &&
                                   IsClassLoopName("M_Neutral_Walk_Loop_B", "Walk", false) &&
                                   !IsClassLoopName("M_Neutral_Run_Loop_F", "Walk", true) &&
                                   !IsClassLoopName("X_Run_Arc_L_Loop_RunArcLeft", "Run", false));

    Database subset;
    if (Load(argv[1], subset))
    {
        const SpeedCalibration c = Derive(subset);
        std::printf("[INFO] subset: %s\n", c.Summary().c_str());
        Check("Neutral subset: walk / run measured from forward loops = 0.5.1 values (+-0.01)",
              c.cls[kSpeedWalk].source == 'm' && c.cls[kSpeedRun].source == 'm' && Near(c.cls[kSpeedWalk].speed, 2.19f, 0.01f) &&
                  Near(c.cls[kSpeedRun].speed, 5.46f, 0.01f),
              c.Summary());
        Check("Neutral subset: no sprint / crouch loops -> fallback / default", c.cls[kSpeedSprint].source == 'f' &&
                                                                                 c.cls[kSpeedCrouch].source == 'd');
        Check("Neutral subset: stand map increasing, game points unchanged", Increasing(c.stand) && c.stand.game[0] == 1.6f &&
                                                                              c.stand.game[1] == 3.85f && c.stand.game[2] == 8.2f);
        // sidecar
        SpeedCalibration s = c;
        Check("sidecar: class overrides", ApplySpeedSidecar(s, R"({"walk": 2.0, "run": 5.0, "sprint": 7.0, "crouch": 2.2})") &&
                                              s.sidecar && s.stand.db[0] == 2.0f && s.stand.db[1] == 5.0f && s.stand.db[2] == 7.0f &&
                                              s.crouch.db[0] == 2.2f && s.cls[0].source == 'o',
              s.Summary());
        s = c;
        Check("sidecar: explicit points", ApplySpeedSidecar(s, R"({"stand": [[1.5, 1.8], [4.0, 4.4]], "crouchPoints": [[2.5, 1.6]]})") &&
                                              s.stand.count == 2 && s.stand.game[0] == 1.5f && s.stand.db[1] == 4.4f &&
                                              s.crouch.count == 1 && s.crouch.db[0] == 1.6f,
              s.Summary());
        s = c;
        Check("sidecar: syntax error rejected, derived kept", !ApplySpeedSidecar(s, "{\"walk\": 2.0,") && !s.sidecar &&
                                                                  s.stand.db[0] == c.stand.db[0] && !s.sidecarError.empty(),
              s.sidecarError);
        s = c;
        Check("sidecar: decreasing points rejected", !ApplySpeedSidecar(s, R"({"stand": [[1.6, 3.0], [3.85, 2.0]]})") &&
                                                         s.stand.db[1] == c.stand.db[1],
              s.sidecarError);
        s = c;
        Check("sidecar: out of range rejected", !ApplySpeedSidecar(s, R"({"run": 50})") && s.cls[1].speed == c.cls[1].speed);
        s = c;
        Check("sidecar: unknown keys ignored (no change)", ApplySpeedSidecar(s, R"({"note": "x"})") && s.stand.db[0] == c.stand.db[0] &&
                                                               s.stand.db[2] == c.stand.db[2]);
        s = c;
        Check("sidecar: non-number value rejected", !ApplySpeedSidecar(s, R"({"walk": null})") && s.cls[0].speed == c.cls[0].speed);
    }

    if (argc > 2)
    {
        Database full;
        if (Load(argv[2], full))
        {
            const SpeedCalibration c = Derive(full);
            std::printf("[INFO] Neutral full: %s\n", c.Summary().c_str());
            Check("Neutral full: all four classes measured = 0.5.1 values (+-0.01)",
                  c.cls[0].source == 'm' && c.cls[1].source == 'm' && c.cls[2].source == 'm' && c.cls[3].source == 'm' &&
                      Near(c.stand.db[0], 2.19f, 0.01f) && Near(c.stand.db[1], 5.46f, 0.01f) &&
                      Near(c.stand.db[2], 7.65f, 0.01f) && Near(c.crouch.db[0], 2.46f, 0.01f),
                  c.Summary());
            // rate / query scale at the game points vs the 0.5.1 map: relative difference
            const SpeedPoints old{3, {1.6f, 3.85f, 8.2f}, {2.19f, 5.46f, 7.65f}};
            float worst = 0.0f;
            for (float v = 0.5f; v <= 9.0f; v += 0.05f)
                worst = std::fmax(worst, std::fabs(MapSpeedPoints(c.stand, v) / MapSpeedPoints(old, v) - 1.0f));
            Check("Neutral full: mapped db speed within 0.25 % of 0.5.1 over 0.5..9 m/s", worst < 0.0025f,
                  std::to_string(worst * 100.0f) + " %");
            // one family: the per-clip rate factors stay 1 (0.5.2 behaviour)
            const uint64_t excl = BaseMatcher::TagMaskContaining(full, "_Lands_") | BaseMatcher::TagMaskContaining(full, "FromTraversal");
            const uint64_t sm = BaseMatcher::TagMaskContaining(full, "Stand_") & ~excl, cm = BaseMatcher::TagMaskContaining(full, "Crouch_") & ~excl;
            const FamilyRates fr = ComputeFamilyRates(full, sm, cm, c);
            float worstFactor = 0.0f;
            for (const float f : fr.clipFactor)
                worstFactor = std::fmax(worstFactor, std::fabs(f - 1.0f));
            Check("Neutral full: one family, every per-clip rate factor within 0.1 % of 1", fr.families == 1 && worstFactor < 0.001f,
                  fr.Summary() + ", worst " + std::to_string(worstFactor * 100.0f) + " %");
        }
    }
    std::printf("RESULT %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
