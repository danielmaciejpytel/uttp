# ---- port: offline v2 per-frame pose pipeline (src/mm/MotionPose.hpp) test on the real data (motion-derived, outside git).
# Included from plugin/UTPP/CMakeLists.txt (include(tests/motion_pose.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Paths default to the main checkout layout (<root>/Data); a missing file makes the test FAIL (load check).
set(_utpp_pmotion_data "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data")
set(_utpp_pmotion_wdb "${_utpp_pmotion_data}/utpp_motion_woman.utppmm")
set(_utpp_pmotion_mdb "${_utpp_pmotion_data}/utpp_motion_man.utppmm")
set(_utpp_pmotion_wov "${_utpp_pmotion_data}/motion/utpp_overlays_woman.utppov")
set(_utpp_pmotion_mov "${_utpp_pmotion_data}/motion/utpp_overlays_man.utppov")
# the build scripts pass the v2 / overlay files with -D: reuse them (style weights sit next to the .utppov)
if(UTPP_MOTION_DB_TEST_FILE)
    set(_utpp_pmotion_wdb "${UTPP_MOTION_DB_TEST_FILE}")
endif()
if(UTPP_MOTION_DB_MAN_TEST_FILE)
    set(_utpp_pmotion_mdb "${UTPP_MOTION_DB_MAN_TEST_FILE}")
endif()
if(UTPP_OVERLAY_WOMAN_FILE)
    set(_utpp_pmotion_wov "${UTPP_OVERLAY_WOMAN_FILE}")
endif()
if(UTPP_OVERLAY_MAN_FILE)
    set(_utpp_pmotion_mov "${UTPP_OVERLAY_MAN_FILE}")
endif()
get_filename_component(_utpp_pmotion_wovdir "${_utpp_pmotion_wov}" DIRECTORY)
get_filename_component(_utpp_pmotion_movdir "${_utpp_pmotion_mov}" DIRECTORY)
set(UTPP_POSEV2_TEST_ARGS
    "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt"
    "${_utpp_pmotion_mdb}" "${_utpp_pmotion_mov}" "${_utpp_pmotion_movdir}/utpp_style_weights_man.txt"
    CACHE STRING "MotionPoseTest arguments: woman db, ov, style, man db, ov, style")
add_executable(utpp_motion_pose_test tests/MotionPoseTest.cpp src/mm/MotionPose.hpp src/mm/MotionAdapter.hpp src/mm/Overlay.hpp
               src/mm/OverlayFile.hpp src/mm/GripIK.hpp src/mm/IdleBreaks.hpp src/mm/Inertializer.hpp
               src/mm/BarrelPitchLoop.hpp)
target_compile_options(utpp_motion_pose_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_motion_pose_test PRIVATE cxx_std_20)
add_test(NAME utpp_motion_pose_test COMMAND utpp_motion_pose_test ${UTPP_POSEV2_TEST_ARGS})
# Fix batch 2 (AO) mutations of the same test (UTPP_POSEV2_AO_MUTATION): 5 = no spine yaw continuity (7f), 6 = no latch (7g),
# 7 = handback does not end the latch (7h), 11 = the pitch of a returning target is followed at once (7i),
# 18 = the barrel pitch loop never applies (src/mm/BarrelPitchLoop.hpp, test 10). Each build MUST report a [FAIL] line.
# removed 2026-11-05 (review R8), which cheaper test kills the same site: utpp_mm_aospineyaw_mut5, _mut6, _mut11
foreach(_m 7 18)
  add_executable(utpp_motion_pose_mut${_m} tests/MotionPoseTest.cpp src/mm/MotionPose.hpp src/mm/MotionAdapter.hpp src/mm/Overlay.hpp
                 src/mm/OverlayFile.hpp src/mm/GripIK.hpp src/mm/IdleBreaks.hpp src/mm/Inertializer.hpp)
  target_compile_options(utpp_motion_pose_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_motion_pose_mut${_m} PRIVATE cxx_std_20)
  target_compile_definitions(utpp_motion_pose_mut${_m} PRIVATE UTPP_POSEV2_AO_MUTATION=${_m})
  add_test(NAME utpp_motion_pose_mut${_m} COMMAND utpp_motion_pose_mut${_m} ${UTPP_POSEV2_TEST_ARGS})
  set_tests_properties(utpp_motion_pose_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "\\[FAIL\\]")
endforeach()

# Pure body-model -> selected magazine skin leaf conversion, no runtime writes.
add_executable(utpp_mm_magazine_target_test tests/MagazinePoseTargetTest.cpp)
target_compile_options(utpp_mm_magazine_target_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_magazine_target_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_magazine_target_test COMMAND utpp_mm_magazine_target_test)

# Focused feminine/contact regressions use the same actual assets as MotionPose.
foreach(_case IN ITEMS FeminineShouldersAdaptive FemininePoseSwitch FeminineUnarmedContinuity)
  add_executable(utpp_${_case}_test tests/${_case}Test.cpp)
  target_compile_options(utpp_${_case}_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  add_test(NAME utpp_${_case}_test COMMAND utpp_${_case}_test
    "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt")
endforeach()
# Feminine shoulder width consistency (2026-11-02): every clip x weapon pose through the real evaluator; narrowing step bound, reach
# limits as smooth per-side weights, contacts unchanged. Mutation 1 = atomic pass (legacy fail-closed), 2 = no rise limiter:
# each MUST report a FAIL line.
add_executable(utpp_FeminineWidthConsistency_test tests/FeminineWidthConsistencyTest.cpp)
target_compile_options(utpp_FeminineWidthConsistency_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_FeminineWidthConsistency_test COMMAND utpp_FeminineWidthConsistency_test
  "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt")
foreach(_m 1 2)
  add_executable(utpp_FeminineWidthConsistency_mut${_m} tests/FeminineWidthConsistencyTest.cpp)
  target_compile_options(utpp_FeminineWidthConsistency_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_FeminineWidthConsistency_mut${_m} PRIVATE UTPP_FEMSHOULDER_MUTATION=${_m})
  add_test(NAME utpp_FeminineWidthConsistency_mut${_m} COMMAND utpp_FeminineWidthConsistency_mut${_m}
    "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt")
  set_tests_properties(utpp_FeminineWidthConsistency_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()
# REACH DRIFT (pose-integration 2026-11-09): the same gates at the widest setting (5 cm per side = Mod Settings shoulder width -5), where
# the sockets may follow the hands; mutation 3 = no drift (the lateral-only pass, which snapped back from 3.25 cm) MUST report a FAIL line.
add_test(NAME utpp_FeminineWidthConsistency_5cm_test COMMAND utpp_FeminineWidthConsistency_test
  "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt" check 0.05)
add_executable(utpp_FeminineWidthConsistency_mut3 tests/FeminineWidthConsistencyTest.cpp)
target_compile_options(utpp_FeminineWidthConsistency_mut3 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_FeminineWidthConsistency_mut3 PRIVATE UTPP_FEMSHOULDER_MUTATION=3)
add_test(NAME utpp_FeminineWidthConsistency_mut3 COMMAND utpp_FeminineWidthConsistency_mut3
  "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt" check 0.05)
set_tests_properties(utpp_FeminineWidthConsistency_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# RETURN HOLD (dll-shoulder-brake 2026-11-09): steady aim while running at 5 cm per side must not pulse in step; mutation 4 = no hold
# (the shoulder gave the posture back between strides) MUST report a FAIL line.
add_executable(utpp_FeminineWidthConsistency_mut4 tests/FeminineWidthConsistencyTest.cpp)
target_compile_options(utpp_FeminineWidthConsistency_mut4 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_FeminineWidthConsistency_mut4 PRIVATE UTPP_FEMSHOULDER_MUTATION=4)
add_test(NAME utpp_FeminineWidthConsistency_mut4 COMMAND utpp_FeminineWidthConsistency_mut4
  "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt" check 0.05)
set_tests_properties(utpp_FeminineWidthConsistency_mut4 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
add_executable(utpp_native_reload_contacts_runtime_test tests/NativeReloadContactsRuntimeTest.cpp)
target_compile_options(utpp_native_reload_contacts_runtime_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_native_reload_contacts_runtime_test COMMAND utpp_native_reload_contacts_runtime_test
  "${_utpp_pmotion_wov}")

# Fix batch 2 (AO) x reload contacts: the aim offset goes down with the reload hand-over (motionpose::AoReloadScale). Real data and the 53
# reload episodes of the recording 2026-10-21 12:15. Mutation 8 (the AO ignores the hand-over) MUST fail.
set(_utpp_ao_reload "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/recording_1215_aim_offset_reload.csv")
add_executable(utpp_mm_aoreload_test tests/AimOffsetReloadTest.cpp src/mm/MotionPose.hpp src/mm/ReloadContactFade.hpp)
target_compile_options(utpp_mm_aoreload_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_aoreload_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_aoreload_test COMMAND utpp_mm_aoreload_test ${UTPP_POSEV2_TEST_ARGS} "${_utpp_ao_reload}")
add_executable(utpp_mm_aoreload_mut8 tests/AimOffsetReloadTest.cpp src/mm/MotionPose.hpp src/mm/ReloadContactFade.hpp)
target_compile_options(utpp_mm_aoreload_mut8 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_aoreload_mut8 PRIVATE cxx_std_20)
target_compile_definitions(utpp_mm_aoreload_mut8 PRIVATE UTPP_POSEV2_AO_MUTATION=8)
add_test(NAME utpp_mm_aoreload_mut8 COMMAND utpp_mm_aoreload_mut8 ${UTPP_POSEV2_TEST_ARGS} "${_utpp_ao_reload}")
set_tests_properties(utpp_mm_aoreload_mut8 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
add_executable(utpp_mm_aoreload_mut14 tests/AimOffsetReloadTest.cpp src/mm/MotionPose.hpp src/mm/ReloadContactFade.hpp)
target_compile_options(utpp_mm_aoreload_mut14 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_aoreload_mut14 PRIVATE cxx_std_20)
target_compile_definitions(utpp_mm_aoreload_mut14 PRIVATE UTPP_POSEV2_AO_MUTATION=14)
add_test(NAME utpp_mm_aoreload_mut14 COMMAND utpp_mm_aoreload_mut14 ${UTPP_POSEV2_TEST_ARGS} "${_utpp_ao_reload}")
set_tests_properties(utpp_mm_aoreload_mut14 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
add_executable(utpp_mm_aoreload_mut17 tests/AimOffsetReloadTest.cpp src/mm/MotionPose.hpp src/mm/ReloadContactFade.hpp)
target_compile_options(utpp_mm_aoreload_mut17 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_aoreload_mut17 PRIVATE cxx_std_20)
target_compile_definitions(utpp_mm_aoreload_mut17 PRIVATE UTPP_POSEV2_AO_MUTATION=17)
add_test(NAME utpp_mm_aoreload_mut17 COMMAND utpp_mm_aoreload_mut17 ${UTPP_POSEV2_TEST_ARGS} "${_utpp_ao_reload}")
set_tests_properties(utpp_mm_aoreload_mut17 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")

# Fix batch 2 (AO): continuity + latch of the aim-offset spine yaw (src/mm/AimSpineYawFilter.hpp). Angles only, no game data;
# the replay fixture is the AO input of the recording 2026-10-21 12:15. Mutation 5 = no continuity, 6 = no latch: both
# builds MUST report a FAIL line (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error cannot count as a kill).
set(_utpp_ao_replay "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/recording_1215_aim_offset_replay.csv")
add_executable(utpp_mm_aospineyaw_test tests/AimSpineYawFilterTest.cpp src/mm/AimSpineYawFilter.hpp src/mm/Overlay.hpp)
target_compile_options(utpp_mm_aospineyaw_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_aospineyaw_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_aospineyaw_test COMMAND utpp_mm_aospineyaw_test "${_utpp_ao_replay}")
# 5 = no continuity, 6 = no latch, 9 = a returning target is followed at once, 10 = every live frame rate limited,
# 11 = the pitch of a returning target is followed at once, 12 = a restart (weapon class, long frame) resets the filter,
# 15 = a constant 45 deg jump size (low frame rates limit the camera again)
foreach(_m 5 6 9 10 11 12 15)
  add_executable(utpp_mm_aospineyaw_mut${_m} tests/AimSpineYawFilterTest.cpp src/mm/AimSpineYawFilter.hpp src/mm/Overlay.hpp)
  target_compile_options(utpp_mm_aospineyaw_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_mm_aospineyaw_mut${_m} PRIVATE cxx_std_20)
  target_compile_definitions(utpp_mm_aospineyaw_mut${_m} PRIVATE UTPP_POSEV2_AO_MUTATION=${_m})
  add_test(NAME utpp_mm_aospineyaw_mut${_m} COMMAND utpp_mm_aospineyaw_mut${_m} "${_utpp_ao_replay}")
  set_tests_properties(utpp_mm_aospineyaw_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()
