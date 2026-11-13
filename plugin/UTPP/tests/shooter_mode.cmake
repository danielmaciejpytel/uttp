# ---- Shooter mode + H2 safeguard (feat/shooterMode-dll): mm/MotionAdapter.hpp MapConfig::shooterMode / h2Strafe, MotionDriver Step (root hold / lock), natives.
# Included from plugin/UTPP/CMakeLists.txt after tests/hipfireraise.cmake; expects UTPP_WARNINGS. Mutation builds (-DUTPP_SHOOTER_MODE_MUTATION=k,
# src/mm/MotionAdapter.hpp / MotionDriver.cpp) MUST report an assertion failure: they pass only when the output contains a line starting "FAIL "
# (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error cannot count as a killed mutant).

# A1: adapter level, no data (MotionAdapter.hpp only). Mutations: 1 the shooterMode term ignored, 2 no spin suppression, 5 the H2 gate ignores the
# body-turn switch, 6 the H2 clear hold is never reset, 7 CanSprint cone bypassed, 8 the raw hip-fire state is no exclusion, 9 spin
# suppression follows the windowed shooterModeActive (slide window) instead of the setting.
foreach(_m 0 1 2 5 6 7 8 9 12)
    if(_m EQUAL 0)
        set(_tgt utpp_ShooterModeStrafeMap_test)
    else()
        set(_tgt utpp_ShooterModeStrafeMap_mut${_m})
    endif()
    add_executable(${_tgt} tests/ShooterModeStrafeMapTest.cpp src/mm/MotionAdapter.hpp src/mm/MotionTrajectory.hpp)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    add_test(NAME ${_tgt} COMMAND ${_tgt})
    if(_m GREATER 0)
        target_compile_definitions(${_tgt} PRIVATE UTPP_SHOOTER_MODE_MUTATION=${_m})
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
    endif()
endforeach()

# A2-A7, A10 + Shooter idle orbit / start / tip release + the slide case: the REAL v1 MMDriver + MotionDriver (tests/MotionDriverShooterModeTest.cpp
# #includes the integration test for its helpers). motion-derived data lives outside git: the targets exist only where it does.
# Mutations: 3 the standing V turns behind the camera (rest turn-in-place) instead of holding, 4 no root lock while moving, 5 the H2 gate ignores the
# body-turn switch (A7), 10 the root lock has no rate bound, 11 the orbit does not reset the ownership of a hidden offset; with the slide bundle
# (--slide): 2 no spin suppression, 9 spin suppression follows the windowed shooterModeActive; 12 the CET hip-fire lease is no H2 exclusion, 13 the speed
# ratio is answered in first person / on skipped feeds, 15 a root reseed drops the held heading, 16 the root does not follow the actor in the air,
# 17 the speed ratio takes the gait from the stick size instead of the gait the chooser plays.
# A setup problem (missing data, a feed that does not load) prints SETUP-ERROR and exits with code 3, never "FAIL ": a mutation build is only
# killed by a real assertion.
if(DEFINED _utpp_mdmotion_data_native AND UTPP_MMDB_TEST_FILE)
    file(TO_NATIVE_PATH "${_utpp_mdmotioni_replay}/20261007-175300_flick_180_run.feed" _utpp_fn_flick)
    file(TO_NATIVE_PATH "${_utpp_mdmotioni_replay}/20261007-175300_run_arc_left.feed" _utpp_fn_arc)
    set(_utpp_fn_args "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}" "${_utpp_fn_flick}" "${_utpp_fn_arc}")
    set(_utpp_fn_slide_prefix "${_utpp_mdmotion_data}/utpp_motion_woman_knee_twistfix_slide")
    # removed 2026-11-05 (review R6, R7), which cheaper test kills the same site: utpp_ShooterModeStrafeMap_mut5, _mut12, _mut2, _mut9
    foreach(_m 0 3 4 10 11 13 15 16 17)
        if(_m EQUAL 0)
            set(_tgt utpp_motion_driver_shooter_mode_test)
        else()
            set(_tgt utpp_motion_driver_shooter_mode_mut${_m})
        endif()
        # tests/MotionDriverIntegrationTest.cpp is #included by the test (its main renamed), not compiled on its own
        add_executable(${_tgt} tests/MotionDriverShooterModeTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
        target_link_libraries(${_tgt} PRIVATE bcrypt)
        target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
        target_compile_definitions(${_tgt} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
        target_compile_features(${_tgt} PRIVATE cxx_std_20)
        if(_m GREATER 0)
            target_compile_definitions(${_tgt} PRIVATE UTPP_SHOOTER_MODE_MUTATION=${_m})
        endif()
        if(_m EQUAL 2 OR _m EQUAL 9)
            set(_name ${_tgt}_slide)
            set(_extra --slide "${_utpp_fn_slide_prefix}")
        else()
            set(_name ${_tgt})
            set(_extra)
        endif()
        if(NOT (_m EQUAL 2 OR _m EQUAL 9) OR EXISTS "${_utpp_fn_slide_prefix}.utppmm")
            add_test(NAME ${_name} COMMAND ${_tgt} ${_utpp_fn_args} ${_extra})
            if(_m GREATER 0)
                set_tests_properties(${_name} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
            endif()
        endif()
    endforeach()
    if(EXISTS "${_utpp_fn_slide_prefix}.utppmm")
        add_test(NAME utpp_motion_driver_shooter_mode_slide_test COMMAND utpp_motion_driver_shooter_mode_test ${_utpp_fn_args} --slide "${_utpp_fn_slide_prefix}")
    endif()
    # A16 alone (aim pitch plan 2026-10-30 seam C: the barrel loop target in the pose frame, loop off bit-identical, loop on converges): no
    # recorded feed needed, so it runs where the replay feeds are missing too (the full test above also runs A16).
    add_test(NAME utpp_motion_driver_barrel_test COMMAND utpp_motion_driver_shooter_mode_test ${_utpp_fn_args} --barrel)
endif()
