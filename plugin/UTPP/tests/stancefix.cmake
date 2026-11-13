# Mutation builds MUST report an assertion failure: they pass only when the output contains a line starting "FAIL "
# (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error - exit 2, missing data - cannot count as a killed mutant).
# ---- fix batch 2 (stance / camera): offline tests of the P1 stance changes. Included from plugin/UTPP/CMakeLists.txt
# (include(tests/stancefix.cmake OPTIONAL)); expects UTPP_WARNINGS.
# P1 stance latency: a clean Stand <-> Crouch edge reaches the chooser on its own feed (MapConfig::stanceCleanEdge), the flicker keeps the
# 0.15 s filter. Synthetic sequences + replay of the 2026-10-21 12:13 feed dump windows (tests/data/feed_recording_20261021_1213.txt, no motion data).
# Mutation 1 = no bypass, 3 = no crouch-entry speed limit, 4 = no min-hold of the previous raw value; each MUST fail.
set(_utpp_p1_feeds "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/feed_recording_20261021_1213.txt")
add_executable(utpp_p1_stance_latency_test tests/P1StanceLatencyTest.cpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp)
target_compile_features(utpp_p1_stance_latency_test PRIVATE cxx_std_20)
target_compile_options(utpp_p1_stance_latency_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_p1_stance_latency_test COMMAND utpp_p1_stance_latency_test "${_utpp_p1_feeds}")
foreach(_m 1 3 4)
  add_executable(utpp_p1_stance_latency_mut${_m} tests/P1StanceLatencyTest.cpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp)
  target_compile_features(utpp_p1_stance_latency_mut${_m} PRIVATE cxx_std_20)
  target_compile_options(utpp_p1_stance_latency_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_p1_stance_latency_mut${_m} PRIVATE UTPP_P1_MUTATION=${_m})
  add_test(NAME utpp_p1_stance_latency_mut${_m} COMMAND utpp_p1_stance_latency_mut${_m} "${_utpp_p1_feeds}")
  set_tests_properties(utpp_p1_stance_latency_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()
# P1 B2: the legs / pelvis regions of the overlay layering take the stance alpha of the base poses (OverlayMachine::LowerBodyKeys), so the armed
# hips no longer dip 12 cm and rise 9.5 cm, also not when a move / aim event follows inside the 1 s of the base fade, while hands / arms keep the
# pose crossfade. Needs the woman overlay file (motion-derived, outside git). Mutation 2 = no separate stance alpha (the pre-P1 pop), 5 = the retired
# B1 (slow hands), 6 = PreserveIdlePistolLowerBody (female V, Pistol1H Relaxed) reads the pose crossfade instead of the B2 keys; each MUST fail.
# The shipped woman overlay (E7E5) carries the idle-pistol curve patch (legs / pelvis layering 0 in Pose_Pistol1H_Stand_Relax_Idle), so the
# exception of mutation 6 cannot be seen on it: the exception block also runs on the unpatched copy (the B307 file E7E5 was made from), a
# fixture in Build/test-data/overlay (not shipped data).
set(UTPP_OVERLAY_UNPATCHED_FIXTURE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/test-data/overlay/utpp_overlays_woman_unpatched.utppov")
if(DEFINED UTPP_OVERLAY_WOMAN_FILE AND EXISTS "${UTPP_OVERLAY_WOMAN_FILE}" AND EXISTS "${UTPP_OVERLAY_UNPATCHED_FIXTURE}")
  add_executable(utpp_p1_overlay_stance_sync_test tests/OverlayStanceSyncTest.cpp src/mm/Overlay.hpp src/mm/OverlayFile.hpp src/mm/Additive.hpp)
  target_compile_features(utpp_p1_overlay_stance_sync_test PRIVATE cxx_std_20)
  target_compile_options(utpp_p1_overlay_stance_sync_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  add_test(NAME utpp_p1_overlay_stance_sync_test COMMAND utpp_p1_overlay_stance_sync_test "${UTPP_OVERLAY_WOMAN_FILE}" "${UTPP_OVERLAY_UNPATCHED_FIXTURE}")
  foreach(_m 2 5 6)
    add_executable(utpp_p1_overlay_stance_sync_mut${_m} tests/OverlayStanceSyncTest.cpp src/mm/Overlay.hpp src/mm/OverlayFile.hpp src/mm/Additive.hpp)
    target_compile_features(utpp_p1_overlay_stance_sync_mut${_m} PRIVATE cxx_std_20)
    target_compile_options(utpp_p1_overlay_stance_sync_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_p1_overlay_stance_sync_mut${_m} PRIVATE UTPP_P1_MUTATION=${_m})
    add_test(NAME utpp_p1_overlay_stance_sync_mut${_m} COMMAND utpp_p1_overlay_stance_sync_mut${_m} "${UTPP_OVERLAY_WOMAN_FILE}" "${UTPP_OVERLAY_UNPATCHED_FIXTURE}")
    set_tests_properties(utpp_p1_overlay_stance_sync_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
  endforeach()
endif()
