# ---- port: MotionDriver integration test through the real v1 MMDriver (src/MotionHost.cpp + src/MotionDriver.cpp, no RED4ext).
# Short grounded start prediction, cancellation, and bounded support preservation.
foreach(_start_stop_test CapsuleAcceleration CancelledStart StartStopSupport)
    add_executable(utpp_${_start_stop_test}_test tests/${_start_stop_test}Test.cpp)
    target_compile_features(utpp_${_start_stop_test}_test PRIVATE cxx_std_20)
    target_compile_options(utpp_${_start_stop_test}_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_${_start_stop_test}_test PRIVATE bcrypt)
endforeach()
add_test(NAME utpp_CapsuleAcceleration_test COMMAND utpp_CapsuleAcceleration_test)
add_test(NAME utpp_CancelledStart_test COMMAND utpp_CancelledStart_test
         "${_utpp_mdmotion_data}/utpp_motion_woman.utppmm")
# The test finds its three clips by name (their indices differ between the 710 / 726 / 800 clip bakes). --baseline runs the stack WITHOUT the
# cancelled-start fix (cancelledGroundStart off): that pre-fix behaviour MUST still fail the fix checks (the test keeps detecting a missing fix).
add_test(NAME utpp_CancelledStart_baseline_mut COMMAND utpp_CancelledStart_test
         "${_utpp_mdmotion_data}/utpp_motion_woman.utppmm" --baseline)
set_tests_properties(utpp_CancelledStart_baseline_mut PROPERTIES PASS_REGULAR_EXPRESSION
                     "FAIL cancelled start outgoing removed within 0.2 seconds.*FAIL hold reports previous displayed right support")
add_test(NAME utpp_StartStopSupport_test COMMAND utpp_StartStopSupport_test "${UTPP_MMDB_TEST_FILE}")

# Crouch stance-entry legality and contact-correction continuity regressions.
foreach(_crouch_test CrouchStanceAdmission CrouchFootContinuity)
    add_executable(utpp_${_crouch_test}_test tests/${_crouch_test}Test.cpp)
    target_compile_features(utpp_${_crouch_test}_test PRIVATE cxx_std_20)
    target_compile_options(utpp_${_crouch_test}_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_${_crouch_test}_test PRIVATE bcrypt)
endforeach()
add_test(NAME utpp_CrouchStanceAdmission_test COMMAND utpp_CrouchStanceAdmission_test
         "${_utpp_mdmotion_data}/utpp_motion_woman.utppmm"
         "${_utpp_mdmotion_data}/utpp_motion_woman.chooser.json")
add_test(NAME utpp_CrouchFootContinuity_test COMMAND utpp_CrouchFootContinuity_test "${UTPP_MMDB_TEST_FILE}")

# Included from tests/motiondriver.cmake (UTPP-MOTION SEAM line); expects UTPP_WARNINGS, UTPP_MMDB_TEST_FILE and _utpp_mdmotion_data_native.
# Arguments: <data dir> (v2 files, motion-derived, outside git) <v1 database> <feeds from <root>\Build\test-data\replay>. Missing files FAIL.
set(_utpp_mdmotioni_replay "${_utpp_mdmotion_data}/../Build/test-data/replay")
set(_utpp_mdmotioni_feeds)
foreach(_s slow_walk_half_stick run_arc_left flick_180_run crouch_walk_turns sprint_start_stop idle_then_start_sideways)
    file(TO_NATIVE_PATH "${_utpp_mdmotioni_replay}/20261007-175300_${_s}.feed" _f)
    list(APPEND _utpp_mdmotioni_feeds "${_f}")
endforeach()

# Current-frame root transaction, orientation retargeting and host application contracts.
foreach(_graph_test MotionRootFrame MotionOrientationPose MotionOrientationFeet PoseRootHook ProvidedRootFrame)
    add_executable(utpp_${_graph_test}_test tests/${_graph_test}Test.cpp)
    target_compile_features(utpp_${_graph_test}_test PRIVATE cxx_std_20)
    target_compile_options(utpp_${_graph_test}_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_${_graph_test}_test PRIVATE bcrypt)
endforeach()
add_test(NAME utpp_MotionRootFrame_test COMMAND utpp_MotionRootFrame_test)
# Root frame relative to the drawn capsule (the coming tick's turn); the mutant ignores it and must FAIL.
add_executable(utpp_MotionRootFrameAhead_test tests/MotionRootFrameAheadTest.cpp)
target_compile_features(utpp_MotionRootFrameAhead_test PRIVATE cxx_std_20)
target_compile_options(utpp_MotionRootFrameAhead_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_MotionRootFrameAhead_test COMMAND utpp_MotionRootFrameAhead_test)
add_executable(utpp_MotionRootFrameAhead_mut1 tests/MotionRootFrameAheadTest.cpp)
target_compile_features(utpp_MotionRootFrameAhead_mut1 PRIVATE cxx_std_20)
target_compile_options(utpp_MotionRootFrameAhead_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_MotionRootFrameAhead_mut1 PRIVATE UTPP_ROOT_AHEAD_MUTATION=1)
add_test(NAME utpp_MotionRootFrameAhead_mut1 COMMAND utpp_MotionRootFrameAhead_mut1)
set_tests_properties(utpp_MotionRootFrameAhead_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
add_test(NAME utpp_MotionOrientationPose_test COMMAND utpp_MotionOrientationPose_test)
add_test(NAME utpp_MotionOrientationFeet_test COMMAND utpp_MotionOrientationFeet_test "${UTPP_MOTION_DB_TEST_FILE}")
add_test(NAME utpp_PoseRootHook_test COMMAND utpp_PoseRootHook_test
         "${_utpp_mdmotion_data}/utpp_motion_woman.utppmm"
         "${_utpp_mdmotion_data}/motion/utpp_overlays_woman.utppov")
add_test(NAME utpp_ProvidedRootFrame_test COMMAND utpp_ProvidedRootFrame_test "${UTPP_MMDB_PACK_TEST_FILE}")
add_executable(utpp_CameraTargetRoot_test tests/CameraTargetRootTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
target_compile_features(utpp_CameraTargetRoot_test PRIVATE cxx_std_20)
target_compile_options(utpp_CameraTargetRoot_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_CameraTargetRoot_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
target_link_libraries(utpp_CameraTargetRoot_test PRIVATE bcrypt)
add_test(NAME utpp_CameraTargetRoot_test COMMAND utpp_CameraTargetRoot_test "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
# Rest turn-in-place (MotionAdapter restTurn): the mutant never admits it and MUST fail the "rest TIP" checks of the real-host test.
add_executable(utpp_CameraTargetRoot_mut_resttip tests/CameraTargetRootTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
target_compile_features(utpp_CameraTargetRoot_mut_resttip PRIVATE cxx_std_20)
target_compile_options(utpp_CameraTargetRoot_mut_resttip PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_CameraTargetRoot_mut_resttip PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV UTPP_REST_TURN_MUTATION=1)
target_link_libraries(utpp_CameraTargetRoot_mut_resttip PRIVATE bcrypt)
add_test(NAME utpp_CameraTargetRoot_mut_resttip COMMAND utpp_CameraTargetRoot_mut_resttip "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
set_tests_properties(utpp_CameraTargetRoot_mut_resttip PROPERTIES PASS_REGULAR_EXPRESSION "FAIL rest TIP")
# Rest TIP source / aim exclusions: UTPP_REST_OWNER_MUTATION=1 (every offset counts as owned) must fail the "foreign" cases,
# UTPP_REST_TURN_MUTATION=3 (aim intent does not exclude it) the "aim intent" cases, UTPP_REST_TURN_MUTATION=2 (the rest TIP
# counts as an aim start) the slide-exit hold checks.
foreach(_rm "OWNER_MUTATION=1;foreign" "OWNER_MUTATION=2;is played: the last lease step" "OWNER_MUTATION=3;(is played: the last lease step|fallback reaches the 40deg knee)" "TURN_MUTATION=3;aim intent" "TURN_MUTATION=2;does not release")
    list(GET _rm 0 _def)
    list(GET _rm 1 _what)
    string(REGEX REPLACE "[^A-Za-z0-9]" "" _tag "${_def}")
    add_executable(utpp_CameraTargetRoot_mut_${_tag} tests/CameraTargetRootTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_compile_features(utpp_CameraTargetRoot_mut_${_tag} PRIVATE cxx_std_20)
    target_compile_options(utpp_CameraTargetRoot_mut_${_tag} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_CameraTargetRoot_mut_${_tag} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV UTPP_REST_${_def})
    target_link_libraries(utpp_CameraTargetRoot_mut_${_tag} PRIVATE bcrypt)
    add_test(NAME utpp_CameraTargetRoot_mut_${_tag} COMMAND utpp_CameraTargetRoot_mut_${_tag} "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
    set_tests_properties(utpp_CameraTargetRoot_mut_${_tag} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL .*${_what}")
endforeach()
add_executable(utpp_CapsuleBraking_test tests/CapsuleBrakingTest.cpp)
target_compile_features(utpp_CapsuleBraking_test PRIVATE cxx_std_20)
target_compile_options(utpp_CapsuleBraking_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_CapsuleBraking_test COMMAND utpp_CapsuleBraking_test)
add_executable(utpp_CapsuleBrakingHost_test tests/CapsuleBrakingHostTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
target_compile_features(utpp_CapsuleBrakingHost_test PRIVATE cxx_std_20)
target_compile_options(utpp_CapsuleBrakingHost_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_CapsuleBrakingHost_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
target_link_libraries(utpp_CapsuleBrakingHost_test PRIVATE bcrypt)
# The v1 database must hold the Run/Crouch/Sprint _Stop_F clips: the legacy slowdown (motionhost::DecelTarget under v1) is their capsule stop profile.
# The dense subset (UTPP_MMDB_TEST_FILE, 2026-10-06) only has Walk_Stop_F, so the profile is 0 and the test cannot run meaningfully; the game's
# v1 bake (PREFERRED_DATABASE in cet/utpp_core/init.lua) is used, falling back to the subset only when it is absent (the test then fails with the cause).
set(_utpp_capbrake_v1db "${_utpp_mdmotion_data}/utpp_base.utppmm")
if(NOT EXISTS "${_utpp_capbrake_v1db}")
    set(_utpp_capbrake_v1db "${UTPP_MMDB_TEST_FILE}")
endif()
add_test(NAME utpp_CapsuleBrakingHost_test COMMAND utpp_CapsuleBrakingHost_test "${_utpp_mdmotion_data_native}" "${_utpp_capbrake_v1db}")
# Negative control: a v1 database without the *_Run_Stop_F profile (the dense subset) MUST be reported by the fixture check.
add_test(NAME utpp_CapsuleBrakingHost_wrongdb_mut COMMAND utpp_CapsuleBrakingHost_test "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
set_tests_properties(utpp_CapsuleBrakingHost_wrongdb_mut PROPERTIES PASS_REGULAR_EXPRESSION "FAIL fixture reproduces legacy slowdown under base ownership")

add_executable(utpp_RefaceStartPolicy_test tests/RefaceStartPolicyTest.cpp)
target_compile_features(utpp_RefaceStartPolicy_test PRIVATE cxx_std_20)
target_compile_options(utpp_RefaceStartPolicy_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_RefaceStartPolicy_test COMMAND utpp_RefaceStartPolicy_test)

# Explicit recording/fixture arguments are required; recordings are not repository assets.
add_executable(utpp_RefaceStartDirection_test EXCLUDE_FROM_ALL tests/RefaceStartDirectionTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
target_compile_features(utpp_RefaceStartDirection_test PRIVATE cxx_std_20)
target_compile_options(utpp_RefaceStartDirection_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_RefaceStartDirection_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
target_link_libraries(utpp_RefaceStartDirection_test PRIVATE bcrypt)
add_executable(utpp_motion_driver_integ_test tests/MotionDriverIntegrationTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/MotionDriver.cpp
               src/MotionDriver.hpp src/State.hpp src/mm/MotionAdapter.hpp src/mm/MotionPose.hpp src/mm/GameIntegration.hpp)
target_link_libraries(utpp_motion_driver_integ_test PRIVATE bcrypt)
target_compile_options(utpp_motion_driver_integ_test PRIVATE ${UTPP_WARNINGS} /EHsc)
# UTPP_MOTION_DRIVER_TEST_TRAV: motiondriver::TestForceTraversal (check 8i, one montage through the real path; never in the DLL)
target_compile_definitions(utpp_motion_driver_integ_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
target_compile_features(utpp_motion_driver_integ_test PRIVATE cxx_std_20)
# The suite runs as three ctest entries (same executable, trailing "--shard <i>/<n>", see main() of tests/MotionDriverIntegrationTest.cpp):
# shard 0 (this name) = section 1, every in-process section (shared process-global state, so they stay together) and a third of the
# independent tail children; shards 1 / 2 = the other two thirds of the tail children (7e - 7i, pistol aim exits, weapon upper / grip cases).
# Together they run exactly the checks of the former single run. The names end in integ_test (quick-set exclusion) and carry "slow".
add_test(NAME utpp_motion_driver_integ_test COMMAND utpp_motion_driver_integ_test "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}"
         ${_utpp_mdmotioni_feeds} --shard 0/3)
foreach(_shard 1 2)
    add_test(NAME utpp_motion_driver_integ_s${_shard}_integ_test COMMAND utpp_motion_driver_integ_test "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}"
             ${_utpp_mdmotioni_feeds} --shard ${_shard}/3)
    set_tests_properties(utpp_motion_driver_integ_s${_shard}_integ_test PROPERTIES LABELS slow)
endforeach()
# removed 2026-11-05 (review R14), which cheaper test kills the same site: utpp_mm_vehexit_loco_mut4

# Focused gravity handoff through the same driver seam; no full integration suite is run.
add_executable(utpp_motion_driver_gravity_test tests/MotionDriverGravityTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
target_link_libraries(utpp_motion_driver_gravity_test PRIVATE bcrypt)
target_compile_options(utpp_motion_driver_gravity_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_motion_driver_gravity_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
target_compile_features(utpp_motion_driver_gravity_test PRIVATE cxx_std_20)
add_test(NAME utpp_motion_driver_gravity_test COMMAND utpp_motion_driver_gravity_test "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")

add_executable(utpp_stance_calibration_test tests/StanceCalibrationTest.cpp)
target_compile_options(utpp_stance_calibration_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_stance_calibration_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
target_compile_features(utpp_stance_calibration_test PRIVATE cxx_std_20)
add_test(NAME utpp_stance_calibration_test COMMAND utpp_stance_calibration_test)

add_executable(utpp_offset_root_frame_test tests/OffsetRootFrameTest.cpp)
target_compile_options(utpp_offset_root_frame_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_offset_root_frame_test PRIVATE cxx_std_20)
add_test(NAME utpp_offset_root_frame_test COMMAND utpp_offset_root_frame_test "${UTPP_MMDB_TEST_FILE}")

# utpp_mm_steering_test, check "keyboard run 90: max lateral offset >= 3 cm below 0.3.6 and < 21 cm" (re-baselined 2026-10-22 after 4bfa6e8
# fixed the offset-root integration, see the comment in tests/MMSteeringTest.cpp). The mutant drops the 0.3.7 root warp + heading lean from the
# 0.3.9 feature set: the lateral offset returns to the 25 cm clamp and exactly that check MUST fail (the regex pins it, so another failure cannot count).
add_executable(utpp_mm_steering_mut_nowarplean tests/MMSteeringTest.cpp)
target_compile_options(utpp_mm_steering_mut_nowarplean PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_mm_steering_mut_nowarplean PRIVATE UTPP_STEER_LATERAL_MUTATION=1)
add_test(NAME utpp_mm_steering_mut_nowarplean COMMAND utpp_mm_steering_mut_nowarplean "${UTPP_MMDB_TEST_FILE}")
set_tests_properties(utpp_mm_steering_mut_nowarplean PROPERTIES PASS_REGULAR_EXPRESSION "FAIL\\] subset: keyboard run 90: max lateral offset")

# utpp_mm_chooser_matcher_test oracle (re-baselined 2026-10-22): ExpectedExcluded knows the stance-entry admission of f973894 (stand<->crouch entry clips
# are not selectable at dt 0). Mutation 5 (UTPP_MOTION_SNAP_MUTATION, entry clips always admitted) selects the transition start in the crouch/idle probes
# and MUST fail them; the regex pins the probe messages ("idle pose, crouch set ..."), which only the stance-entry rule of the oracle can produce here.
add_executable(utpp_mm_chooser_matcher_mut5 tests/ChooserMatcherTest.cpp)
target_compile_options(utpp_mm_chooser_matcher_mut5 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_mm_chooser_matcher_mut5 PRIVATE UTPP_MOTION_SNAP_MUTATION=5)
add_test(NAME utpp_mm_chooser_matcher_mut5 COMMAND utpp_mm_chooser_matcher_mut5 "${UTPP_MOTION_DB_TEST_FILE}" "${UTPP_MOTION_DB_MAN_TEST_FILE}")
set_tests_properties(utpp_mm_chooser_matcher_mut5 PROPERTIES PASS_REGULAR_EXPRESSION "idle pose, crouch set")

# Narrow ordinary-MM playback checks; these do not run the full matcher suite.
foreach(_rate_test MotionPlayRate MotionMatcherPlayRateFocused)
    add_executable(utpp_${_rate_test}_test tests/${_rate_test}Test.cpp)
    target_compile_options(utpp_${_rate_test}_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_${_rate_test}_test PRIVATE cxx_std_20)
endforeach()
target_link_libraries(utpp_MotionPlayRate_test PRIVATE bcrypt)
add_test(NAME utpp_MotionPlayRate_test COMMAND utpp_MotionPlayRate_test
         "${_utpp_mdmotion_data}/utpp_motion_woman.utppmm")
add_test(NAME utpp_MotionMatcherPlayRateFocused_test COMMAND utpp_MotionMatcherPlayRateFocused_test
         "${_utpp_mdmotion_data}/utpp_motion_woman.utppmm"
         "${_utpp_mdmotion_data}/utpp_motion_woman.chooser.json")

# Preserve pose-query phase at excluded non-loop tails when caller history is absent.
add_executable(utpp_NonLoopReselection_test tests/NonLoopReselectionTest.cpp)
target_compile_options(utpp_NonLoopReselection_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_NonLoopReselection_test PRIVATE cxx_std_20)
# Data base: the twistfix slide bundle installed in the game (the pass of 2026-10-17 was measured on it); falls back to the
# package pair (Data/utpp_motion_woman) if absent. Same pattern as jumppick.cmake.
set(_utpp_nlr_base "${_utpp_mdmotion_data}/utpp_motion_woman_knee_twistfix_slide")
if(NOT (EXISTS "${_utpp_nlr_base}.utppmm" AND EXISTS "${_utpp_nlr_base}.chooser.json"))
    set(_utpp_nlr_base "${_utpp_mdmotion_data}/utpp_motion_woman")
endif()
add_test(NAME utpp_NonLoopReselection_test COMMAND utpp_NonLoopReselection_test
         "${_utpp_nlr_base}.utppmm"
         "${_utpp_nlr_base}.chooser.json")
# Mutant = the behaviour before the fix (no last-pose fallback: an empty caller PoseHistory keeps every pose channel at the database mean);
# the phase repeats of the original defect MUST be reported (the regex pins the primary check).
add_executable(utpp_NonLoopReselection_mut_nolastpose tests/NonLoopReselectionTest.cpp)
target_compile_options(utpp_NonLoopReselection_mut_nolastpose PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_NonLoopReselection_mut_nolastpose PRIVATE cxx_std_20)
target_compile_definitions(utpp_NonLoopReselection_mut_nolastpose PRIVATE UTPP_NLR_NO_LAST_POSE_MUTATION=1)
add_test(NAME utpp_NonLoopReselection_mut_nolastpose COMMAND utpp_NonLoopReselection_mut_nolastpose
         "${_utpp_nlr_base}.utppmm"
         "${_utpp_nlr_base}.chooser.json")
set_tests_properties(utpp_NonLoopReselection_mut_nolastpose PROPERTIES PASS_REGULAR_EXPRESSION "gait1 repeated non-loop phase")

# Vehicle-exit pose seed: the first v2 search after the re-entry reads the captured exit pose (PoseHistory) instead of the database mean.
# Same data base as the non-loop test (a bundle without the VX_* clips is enough: the test uses rows of PSD_Dense_Stand_Idles).
add_executable(utpp_mm_vehexit_seed_test tests/VehExitSeedTest.cpp)
target_compile_options(utpp_mm_vehexit_seed_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_vehexit_seed_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_vehexit_seed_test COMMAND utpp_mm_vehexit_seed_test
         "${_utpp_nlr_base}.utppmm"
         "${_utpp_nlr_base}.chooser.json")

# Actual-data live outgoing-clip blend and PoseEvaluator raw/montage bridge contracts.
foreach(_crossfade_test MotionBlendStack PoseLiveCrossfade)
    add_executable(utpp_${_crossfade_test}_test tests/${_crossfade_test}Test.cpp)
    target_compile_options(utpp_${_crossfade_test}_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_${_crossfade_test}_test PRIVATE cxx_std_20)
    target_link_libraries(utpp_${_crossfade_test}_test PRIVATE bcrypt)
endforeach()
add_test(NAME utpp_MotionBlendStack_test COMMAND utpp_MotionBlendStack_test
         "${_utpp_mdmotion_data}/utpp_motion_woman.utppmm")
add_test(NAME utpp_PoseLiveCrossfade_test COMMAND utpp_PoseLiveCrossfade_test
         "${_utpp_mdmotion_data}/utpp_motion_woman.utppmm"
         "${_utpp_mdmotion_data}/motion/utpp_overlays_woman.utppov")

# Planar native-root semantics are an offline candidate, not enabled in the game.
add_executable(utpp_MotionOffsetRoot_test tests/MotionOffsetRootTest.cpp)
target_compile_options(utpp_MotionOffsetRoot_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_MotionOffsetRoot_test PRIVATE cxx_std_20)
add_test(NAME utpp_MotionOffsetRoot_test COMMAND utpp_MotionOffsetRoot_test)
add_executable(utpp_MotionOffsetRoot_release_mut1 tests/MotionOffsetRootTest.cpp)
target_compile_options(utpp_MotionOffsetRoot_release_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_MotionOffsetRoot_release_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_MotionOffsetRoot_release_mut1 PRIVATE UTPP_ROOT_RELEASE_MUTATION=1)
add_test(NAME utpp_MotionOffsetRoot_release_mut1 COMMAND utpp_MotionOffsetRoot_release_mut1)
set_tests_properties(utpp_MotionOffsetRoot_release_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")

# P4 (recording 2026-10-21 12:15): a reload on the spot carries the hidden root yaw offset instead of shedding it in 0.3 s.
# Replays the recorded windows through motionRoot::OffsetRoot; the mutant compiles the pre-fix policy (mm/RootMontage.hpp) and MUST fail.
add_executable(utpp_reload_root_hold_test tests/ReloadRootHoldTest.cpp src/mm/RootMontage.hpp src/mm/MotionOffsetRoot.hpp)
target_compile_options(utpp_reload_root_hold_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_root_hold_test PRIVATE cxx_std_20)
add_test(NAME utpp_reload_root_hold_test COMMAND utpp_reload_root_hold_test)
add_executable(utpp_reload_root_hold_mut1 tests/ReloadRootHoldTest.cpp src/mm/RootMontage.hpp src/mm/MotionOffsetRoot.hpp)
target_compile_options(utpp_reload_root_hold_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_root_hold_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_reload_root_hold_mut1 PRIVATE UTPP_ROOT_MONTAGE_MUTATION=1)
add_test(NAME utpp_reload_root_hold_mut1 COMMAND utpp_reload_root_hold_mut1)
set_tests_properties(utpp_reload_root_hold_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
add_executable(utpp_reload_root_hold_mut2 tests/ReloadRootHoldTest.cpp src/mm/RootMontage.hpp src/mm/MotionOffsetRoot.hpp)
target_compile_options(utpp_reload_root_hold_mut2 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_root_hold_mut2 PRIVATE cxx_std_20)
target_compile_definitions(utpp_reload_root_hold_mut2 PRIVATE UTPP_ROOT_MONTAGE_MUTATION=2)
add_test(NAME utpp_reload_root_hold_mut2 COMMAND utpp_reload_root_hold_mut2)
set_tests_properties(utpp_reload_root_hold_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")

# P4 follow-up: a hidden yaw offset beyond the turn-in-place angle is folded back at rest (MotionOffsetRoot rest recentering),
# replayed on the 20 recorded stationary reloads. The mutant never recentres (REST 1) and MUST fail.
add_executable(utpp_rest_offset_collapse_test tests/RestOffsetCollapseTest.cpp src/mm/RootMontage.hpp src/mm/MotionOffsetRoot.hpp)
target_compile_options(utpp_rest_offset_collapse_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_rest_offset_collapse_test PRIVATE cxx_std_20)
add_test(NAME utpp_rest_offset_collapse_test COMMAND utpp_rest_offset_collapse_test)
add_executable(utpp_rest_offset_collapse_mut_REST tests/RestOffsetCollapseTest.cpp src/mm/RootMontage.hpp src/mm/MotionOffsetRoot.hpp)
target_compile_options(utpp_rest_offset_collapse_mut_REST PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_rest_offset_collapse_mut_REST PRIVATE cxx_std_20)
target_compile_definitions(utpp_rest_offset_collapse_mut_REST PRIVATE UTPP_ROOT_REST_MUTATION=1)
add_test(NAME utpp_rest_offset_collapse_mut_REST COMMAND utpp_rest_offset_collapse_mut_REST)
set_tests_properties(utpp_rest_offset_collapse_mut_REST PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
add_executable(utpp_MotionOffsetRoot_rest_mut1 tests/MotionOffsetRootTest.cpp)
target_compile_options(utpp_MotionOffsetRoot_rest_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_MotionOffsetRoot_rest_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_MotionOffsetRoot_rest_mut1 PRIVATE UTPP_ROOT_REST_MUTATION=1)
add_test(NAME utpp_MotionOffsetRoot_rest_mut1 COMMAND utpp_MotionOffsetRoot_rest_mut1)
set_tests_properties(utpp_MotionOffsetRoot_rest_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# Translation continuity (MotionOffsetRoot Step::offsetAccel): the mutant ignores the bound and MUST fail.
add_executable(utpp_MotionOffsetRoot_continuity_mut1 tests/MotionOffsetRootTest.cpp)
target_compile_options(utpp_MotionOffsetRoot_continuity_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_MotionOffsetRoot_continuity_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_MotionOffsetRoot_continuity_mut1 PRIVATE UTPP_ROOT_CONTINUITY_MUTATION=1)
add_test(NAME utpp_MotionOffsetRoot_continuity_mut1 COMMAND utpp_MotionOffsetRoot_continuity_mut1)
set_tests_properties(utpp_MotionOffsetRoot_continuity_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")

# P5 FIX C: the stationary shot-turn goal is dropped once the camera left the shot by > 45 deg (recording 2026-10-21 12:15).
# The mutant never abandons the goal and MUST fail.
add_executable(utpp_stationary_shot_turn_test tests/StationaryShotTurnTest.cpp src/mm/StationaryShotTurn.hpp)
target_compile_options(utpp_stationary_shot_turn_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_stationary_shot_turn_test PRIVATE cxx_std_20)
add_test(NAME utpp_stationary_shot_turn_test COMMAND utpp_stationary_shot_turn_test)
add_executable(utpp_stationary_shot_turn_mut1 tests/StationaryShotTurnTest.cpp src/mm/StationaryShotTurn.hpp)
target_compile_options(utpp_stationary_shot_turn_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_stationary_shot_turn_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_stationary_shot_turn_mut1 PRIVATE UTPP_SHOTTURN_MUTATION=1)
add_test(NAME utpp_stationary_shot_turn_mut1 COMMAND utpp_stationary_shot_turn_mut1)
set_tests_properties(utpp_stationary_shot_turn_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
add_executable(utpp_stationary_shot_turn_mut2 tests/StationaryShotTurnTest.cpp src/mm/StationaryShotTurn.hpp)
target_compile_options(utpp_stationary_shot_turn_mut2 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_stationary_shot_turn_mut2 PRIVATE cxx_std_20)
target_compile_definitions(utpp_stationary_shot_turn_mut2 PRIVATE UTPP_SHOTTURN_MUTATION=2)
add_test(NAME utpp_stationary_shot_turn_mut2 COMMAND utpp_stationary_shot_turn_mut2)
set_tests_properties(utpp_stationary_shot_turn_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# R3 (recording 2026-10-23 21:08): a new shot turn needs 50 deg, a passed goal ends it. Mutation 3 (pre-fix: > 5 deg, passed goal kept)
# MUST fail. The sustained hipfire pose: mutation 1 of UTPP_HIPFIRE_SUSTAIN_MUTATION (every shot isolated, 0.75 s) MUST fail.
add_executable(utpp_stationary_shot_turn_mut3 tests/StationaryShotTurnTest.cpp src/mm/StationaryShotTurn.hpp)
target_compile_options(utpp_stationary_shot_turn_mut3 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_stationary_shot_turn_mut3 PRIVATE cxx_std_20)
target_compile_definitions(utpp_stationary_shot_turn_mut3 PRIVATE UTPP_SHOTTURN_MUTATION=3)
add_test(NAME utpp_stationary_shot_turn_mut3 COMMAND utpp_stationary_shot_turn_mut3)
set_tests_properties(utpp_stationary_shot_turn_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# Review L1: mutation 4 (no 180 deg side hysteresis) MUST fail. Review M1 / L5: sustain mutations 2 (sprint keeps the 2 s hold) and
# 3 (the vehicle exit window counts as a shot) MUST fail.
add_executable(utpp_stationary_shot_turn_mut4 tests/StationaryShotTurnTest.cpp src/mm/StationaryShotTurn.hpp)
target_compile_options(utpp_stationary_shot_turn_mut4 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_stationary_shot_turn_mut4 PRIVATE cxx_std_20)
target_compile_definitions(utpp_stationary_shot_turn_mut4 PRIVATE UTPP_SHOTTURN_MUTATION=4)
add_test(NAME utpp_stationary_shot_turn_mut4 COMMAND utpp_stationary_shot_turn_mut4)
set_tests_properties(utpp_stationary_shot_turn_mut4 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# Combat probe 2026-10-24: mutation 4 (the holds survive the air / landing) MUST fail. 2026-10-28: mutation 5 (the jump-out window holds the pose through the landing) MUST fail.
# r5 air: mutation 6 (the release is evaluated on the launch feed too) and 7 (the window holds the pose through the flight, pre-r5) MUST fail.
foreach(_m 0 1 2 3 4 5 6 7)
    if(_m EQUAL 0)
        set(_tgt utpp_hipfire_sustain_test)
    else()
        set(_tgt utpp_hipfire_sustain_mut${_m})
    endif()
    add_executable(${_tgt} tests/HipFireSustainTest.cpp src/mm/HipFirePose.hpp)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    add_test(NAME ${_tgt} COMMAND ${_tgt})
    if(NOT _m EQUAL 0)
        target_compile_definitions(${_tgt} PRIVATE UTPP_HIPFIRE_SUSTAIN_MUTATION=${_m})
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
    endif()
endforeach()

# Offline graph prerequisites; these helpers do not enable a retail root policy.
add_executable(utpp_MotionSteering_test tests/MotionSteeringTest.cpp)
target_compile_features(utpp_MotionSteering_test PRIVATE cxx_std_20)
target_compile_options(utpp_MotionSteering_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_MotionSteering_test COMMAND utpp_MotionSteering_test)

add_executable(utpp_MotionRootTrack_test tests/MotionRootTrackTest.cpp)
target_compile_features(utpp_MotionRootTrack_test PRIVATE cxx_std_20)
target_compile_options(utpp_MotionRootTrack_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_link_libraries(utpp_MotionRootTrack_test PRIVATE bcrypt)
add_test(NAME utpp_MotionRootTrack_test COMMAND utpp_MotionRootTrack_test "${UTPP_MOTION_DB_TEST_FILE}")

foreach(_root_test MotionOrientationWarp VisualRootFrame)
    add_executable(utpp_${_root_test}_test tests/${_root_test}Test.cpp)
    target_compile_features(utpp_${_root_test}_test PRIVATE cxx_std_20)
    target_compile_options(utpp_${_root_test}_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    add_test(NAME utpp_${_root_test}_test COMMAND utpp_${_root_test}_test)
endforeach()

# Selection churn guard (2026-11-11): clip changes per second of the real driver on slices of the 2026-11-11 10:29 game feed dump (the harness windows that failed
# motion_transitions_per_s <= 3.0) and the release / stop latency check; src/mm/ChooserMatcher.hpp MatcherConfig "SELECTION CHURN GUARD", MotionAdapter.hpp
# MapConfig::gaitReturnHoldS. Run-time mutants (src/MutationSelect.hpp, family JUMPPICK): 24 no loop dwell, 25 no young event clip bias, 26 no dodge hold,
# 27 no gait return hold, 28 no start upshift hold (MatcherConfig::startUpshiftHoldS), 29 reface starts held 0.35 s (check C), 30 no start restart / stalled dwell (check D),
# 31 the gait return hold's rise test per feed instead of per second (check F, uneven frame times), 32 no turn commit (MatcherConfig::turnCommit, check G),
# 33 no heading catch-up (MatcherConfig::headingCatchUpS, check K: keyboard-like direction keys);
# each must print a FAIL line of the churn checks. Arguments: <data dir> <v1 database> <slice dir>.
file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tests/data" _utpp_churn_slices)
add_executable(utpp_mm_loco_churn_rt tests/MMChurnTest.cpp src/MotionHost.cpp src/MotionDriver.cpp src/MutationSelect.hpp)
target_compile_features(utpp_mm_loco_churn_rt PRIVATE cxx_std_20)
target_compile_options(utpp_mm_loco_churn_rt PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_mm_loco_churn_rt PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV UTPP_RUNTIME_MUTATION_JUMPPICK)
target_link_libraries(utpp_mm_loco_churn_rt PRIVATE bcrypt)
add_test(NAME utpp_mm_loco_churn_test COMMAND utpp_mm_loco_churn_rt "${_utpp_mdmotion_data_native}" "${_utpp_capbrake_v1db}" "${_utpp_churn_slices}")
foreach(_m 24 25 26 27 28 29 30 31 32 33)
    add_test(NAME utpp_mm_loco_churn_mut${_m} COMMAND utpp_mm_loco_churn_rt "${_utpp_mdmotion_data_native}" "${_utpp_capbrake_v1db}" "${_utpp_churn_slices}")
    if(_m EQUAL 32)
        set(_utpp_churn_fail "FAIL turn played through")
    elseif(_m EQUAL 29)
        set(_utpp_churn_fail "FAIL reface swap not delayed")
    elseif(_m EQUAL 30)
        set(_utpp_churn_fail "FAIL start restart")
    elseif(_m EQUAL 33)
        set(_utpp_churn_fail "FAIL changes per heading step")
    elseif(_m EQUAL 31)
        set(_utpp_churn_fail "FAIL gait return hold (does not flicker|lasts as long) under uneven frame times")
    else()
        set(_utpp_churn_fail "FAIL churn")
    endif()
    set_tests_properties(utpp_mm_loco_churn_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "${_utpp_churn_fail}" ENVIRONMENT "UTPP_MUTATION_JUMPPICK=${_m}")
endforeach()
