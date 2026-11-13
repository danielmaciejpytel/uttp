// UTPP ragdoll S0 bench (offline, standalone; NOT part of the CMake build and compiled to nothing unless UTPP_RAGDOLL_BENCH is set,
// so it can never add a main() to the DLL). Build (MSVC, from a vcvars64 shell, output outside the tree):
//   cl /nologo /EHsc /O2 /std:c++20 /W4 /DUTPP_RAGDOLL_BENCH RagdollSimBench.cpp /Fe:RagdollSimBench.exe
// Scenario: standing pose hit by a "car": uniform 9 m/s forward + 3 m/s up, pelvis extra 12 m/s, 4 s at 120 Hz.
// Reports solver-only cost per step (NOT the whole per-frame cost: script raycasts, 73-bone pose-out and publish are not included),
// max bone-length error, deepest pre-projection ground penetration, final pelvis / head height, time to settle (< 0.15 m/s).
#if defined(UTPP_RAGDOLL_BENCH)
#include "RagdollSim.hpp"

#include <chrono>
#include <cstdio>

using namespace utpp::ragdoll;

int main()
{
    Sim sim;
    sim.Build();
    V3 w[kCount];
    for (int i = 0; i < kCount; ++i)
        w[i] = sim.Rest(i) + V3{0, 0, 0.98f}; // standing: pelvis 0.98 m above the floor
    sim.SetPose(w);
    sim.SetUniformVelocity({9.0f, 0.0f, 3.0f});
    sim.SetVelocityAt(kHips, {12.0f, 1.5f, 3.0f});
    const int steps = 120 * 4;
    double worstUs = 0, totalUs = 0, worstErr = 0, settleT = -1;
    sim.ResetPenetration();
    for (int s = 0; s < steps; ++s)
    {
        const auto t0 = std::chrono::steady_clock::now();
        sim.Step();
        const auto t1 = std::chrono::steady_clock::now();
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        totalUs += us;
        worstUs = std::max(worstUs, us);
        worstErr = std::max(worstErr, static_cast<double>(sim.MaxBoneError()));
        if (settleT < 0 && s > 30 && sim.KineticSpeedMax() < 0.15f)
            settleT = s / 120.0;
    }
    std::printf("steps=%d solver avg=%.2f us/step worst=%.2f us/step (2 steps per 60 fps frame = %.1f us)\n", steps, totalUs / steps,
                worstUs, 2.0 * totalUs / steps);
    std::printf("maxBoneErr=%.4f m prePenetrationMax=%.4f m pelvis=(%.2f,%.2f,%.3f) head z=%.3f settle=%.2f s\n", worstErr,
                sim.MaxPrePenetration(), sim.Pos(kHips).x, sim.Pos(kHips).y, sim.Pos(kHips).z, sim.Pos(kHead).z, settleT);
    return 0;
}
#endif
