# ---- fix2/hip: slide hipfire recoil (P8) and weapon-pose legs in the air (P6 K1). Included from tests/motiondriver.cmake (after motion_pose.cmake and
# motiondriverinteg.cmake); expects UTPP_WARNINGS, _utpp_mdmotion_data_native, UTPP_MMDB_TEST_FILE and the _utpp_pmotion_* paths of motion_pose.cmake.
# Mutation builds MUST report an assertion failure: they pass only when the output contains a line starting "FAIL "
# (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error cannot count as a killed mutant).

# P6 K1: FadeLowerBodyInAir / AirLegsFade (unit + synthetic ComposeLayers + real woman data through the PoseEvaluator).
# Mutation 1 = FadeLowerBodyInAir does nothing, 2 = no hold after the landing, 3 = MotionPose does not apply airW, 4 = no per-frame cap on the rise, 5 = fall 0.25 s.
if(DEFINED _utpp_pmotion_wov AND DEFINED _utpp_pmotion_wdb AND DEFINED _utpp_pmotion_wovdir)
    set(_utpp_airlegs_args "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt")
else()
    set(_utpp_airlegs_args)
endif()
add_executable(utpp_air_legs_fade_test tests/AirLegsFadeTest.cpp src/mm/MotionPose.hpp src/mm/Overlay.hpp)
target_compile_options(utpp_air_legs_fade_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_air_legs_fade_test PRIVATE cxx_std_20)
add_test(NAME utpp_air_legs_fade_test COMMAND utpp_air_legs_fade_test ${_utpp_airlegs_args})
foreach(_m 1 2 3 4 5)
    add_executable(utpp_air_legs_fade_mut${_m} tests/AirLegsFadeTest.cpp src/mm/MotionPose.hpp src/mm/Overlay.hpp)
    target_compile_options(utpp_air_legs_fade_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_air_legs_fade_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_air_legs_fade_mut${_m} PRIVATE UTPP_AIRLEGS_MUTATION=${_m})
    add_test(NAME utpp_air_legs_fade_mut${_m} COMMAND utpp_air_legs_fade_mut${_m} ${_utpp_airlegs_args})
    set_tests_properties(utpp_air_legs_fade_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()

# P6 K1 driver wiring (airW only from chooser MovementMode 1, never from the slide; reset by a skip; handed over, not reset, by a traversal
# started from the air): the slide host test built with mutation 22 (the slide also fades the legs), 23 (a skip keeps the weight) and
# 24 (reset on the first traversal montage frame) MUST each fail. r9 jump-out arms (Status joArms=): 26 (the jump-out window never reaches the
# arm fade), 27 (every flight fades the arms), 28 (aim / shot / reload in the flight fade the arms too) and 29 (r11: only native ADS keeps the arms, a hip shot
# and a reload fade them) MUST each fail too.
foreach(_m 22 23 24 26 27 28 29)
    add_executable(utpp_motion_driver_slide_host_mut${_m} tests/MotionDriverSlideHostTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_compile_features(utpp_motion_driver_slide_host_mut${_m} PRIVATE cxx_std_20)
    target_compile_options(utpp_motion_driver_slide_host_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_motion_driver_slide_host_mut${_m} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE
                               UTPP_MOTION_DRIVER_TEST_TRAV UTPP_MOTION_DRIVER_FIX_MUTATION=${_m})
    target_link_libraries(utpp_motion_driver_slide_host_mut${_m} PRIVATE bcrypt)
    add_test(NAME utpp_motion_driver_slide_host_mut${_m} COMMAND utpp_motion_driver_slide_host_mut${_m} "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
    set_tests_properties(utpp_motion_driver_slide_host_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()

# P8: hipfire in the v2 slide keeps the authored weapon recoil (MotionDriver.cpp authoredDomain). Parent mode runs the
# v2-nue-recoil-hipfire-*-slide cases of the integration test in child processes; mutations 21 (slide outside the domain) and 25 (shot pulse expires on wall time alone: a late first Step loses the recoil) MUST fail.
foreach(_m "" 21 25)
    if(_m STREQUAL "")
        set(_tgt utpp_motion_driver_slide_recoil_test)
    else()
        set(_tgt utpp_motion_driver_slide_recoil_mut${_m})
    endif()
    add_executable(${_tgt} tests/MotionDriverSlideRecoilTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(${_tgt} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
    if(NOT _m STREQUAL "")
        target_compile_definitions(${_tgt} PRIVATE UTPP_MOTION_DRIVER_FIX_MUTATION=${_m})
    endif()
    target_link_libraries(${_tgt} PRIVATE bcrypt)
    add_test(NAME ${_tgt} COMMAND ${_tgt} "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
    if(NOT _m STREQUAL "")
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
    endif()
endforeach()

# r9 jump-out arms (Overlay.hpp FadeArmsInJumpOutFlight / JumpOutArmsFade): the arm / hand layers of the weapon pose follow the jump clip in a vehicle
# jump-out flight. Mutation 1 = the fade does nothing, 2 = the arms snap back at the touchdown, 3 = no per-frame cap on the rise, 4 = MotionPose does not apply jumpOutArmW (needs the data args),
# 5 = no per-frame cap on the fall, 6 = the hand IK weights are not faded (needs the data args).
add_executable(utpp_jumpout_arms_fade_test tests/JumpOutArmsFadeTest.cpp src/mm/MotionPose.hpp src/mm/Overlay.hpp)
target_compile_options(utpp_jumpout_arms_fade_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_jumpout_arms_fade_test PRIVATE cxx_std_20)
add_test(NAME utpp_jumpout_arms_fade_test COMMAND utpp_jumpout_arms_fade_test ${_utpp_airlegs_args})
foreach(_m 1 2 3 4 5 6)
    add_executable(utpp_jumpout_arms_fade_mut${_m} tests/JumpOutArmsFadeTest.cpp src/mm/MotionPose.hpp src/mm/Overlay.hpp)
    target_compile_options(utpp_jumpout_arms_fade_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_jumpout_arms_fade_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_jumpout_arms_fade_mut${_m} PRIVATE UTPP_JOARMS_MUTATION=${_m})
    add_test(NAME utpp_jumpout_arms_fade_mut${_m} COMMAND utpp_jumpout_arms_fade_mut${_m} ${_utpp_airlegs_args})
    set_tests_properties(utpp_jumpout_arms_fade_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()
