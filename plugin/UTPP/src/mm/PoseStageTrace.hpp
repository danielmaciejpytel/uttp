#pragma once
// Test-only record of the order in which the pose layers touch the published pose (tests/LayerOrderDriverTest.cpp).
// Compiled in only with UTPP_POSE_STAGE_TRACE; without it every macro is empty (no code, no state, the build is unchanged).
// Stages are marked where they change the pose: the overlay layering (ComposeLayers), the feminine posture passes, the
// inertialized overlay switch, the aim offset, the barrel pitch loop, the support grip / stock, the shot (recoil) delta, the grip
// IK and, in the driver, the hand-back of the upper body to the game (mm.aimGameUpper) on the snapshot.
#if defined(UTPP_POSE_STAGE_TRACE)
#include <cstdint>

namespace utpp::mm::stagetrace
{
enum class Stage : uint8_t
{
    Layering = 1,
    Feminine,
    OverlaySwitch,
    AimOffset,
    BarrelLoop,
    SupportGrip,
    ShootAdditive,
    GripIK,
    GameUpper,
};
struct Trace
{
    static constexpr int kMax = 32;
    Stage s[kMax] = {};
    int n = 0;
};
inline Trace& Get()
{
    static Trace t;
    return t;
}
inline void Begin() { Get().n = 0; }
inline void Mark(Stage aStage)
{
    Trace& t = Get();
    if (t.n < Trace::kMax)
        t.s[t.n++] = aStage;
}
} // namespace utpp::mm::stagetrace

#define UTPP_STAGE_TRACE_BEGIN() ::utpp::mm::stagetrace::Begin()
#define UTPP_STAGE_TRACE(aStage) ::utpp::mm::stagetrace::Mark(::utpp::mm::stagetrace::Stage::aStage)
#else
#define UTPP_STAGE_TRACE_BEGIN() ((void)0)
#define UTPP_STAGE_TRACE(aStage) ((void)0)
#endif
