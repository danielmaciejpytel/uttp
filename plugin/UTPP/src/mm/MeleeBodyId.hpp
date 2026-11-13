#pragma once
// The body (0 Female V, 1 Male V) the runtime was told about, for the melee chest base correction (mm/MeleeBase.hpp): it picks ONE clip table,
// there is no scan of the other body. Set by motiondriver::SetRigBody (the script utpp_motion.reds sends UTPPSetMMRigBody from the player's
// resolved gender on every body change / arm); -1 = not told yet (the correction stays off). Header-only so that the motion driver and the
// offline tests need no link dependency on MeleeBase.cpp.

#include <atomic>
#include <cstdint>

namespace utpp::mm::meleebase
{
inline std::atomic<int32_t> g_armedBody{-1};
inline void SetArmedBody(int32_t aBody) { g_armedBody.store((aBody == 0 || aBody == 1) ? aBody : -1, std::memory_order_relaxed); }
inline int32_t ArmedBody() { return g_armedBody.load(std::memory_order_relaxed); }
} // namespace utpp::mm::meleebase
