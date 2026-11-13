# ---- air (commit C prep): v2 air consumer test through the real v1 MMDriver (src/MotionHost.cpp + src/MotionDriver.cpp).
# Included from tests/motiondriver.cmake; expects UTPP_WARNINGS, UTPP_MMDB_TEST_FILE and _utpp_mdmotion_data_native.
# Arguments: <data dir> (v2 files, motion-derived, outside git) <v1 database>. Missing files FAIL.
# UTPP_MOTION_DRIVER_AIR_MUTATION (cache, default 0): mutation builds of this target only (tests/MotionDriverAirTest.cpp header lists 1..7);
# the plugin and every other target always compile MotionDriver.cpp with the production value 0.
set(UTPP_MOTION_DRIVER_AIR_MUTATION 0 CACHE STRING "the motion set-air test mutation (0 = production code)")
add_executable(utpp_motion_driver_air_test tests/MotionDriverAirTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/MotionDriver.cpp
               src/MotionDriver.hpp src/State.hpp src/mm/MotionAdapter.hpp src/mm/MotionPose.hpp src/mm/GameIntegration.hpp)
target_link_libraries(utpp_motion_driver_air_test PRIVATE bcrypt)
target_compile_options(utpp_motion_driver_air_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_motion_driver_air_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE
                           UTPP_MOTION_DRIVER_AIR_MUTATION=${UTPP_MOTION_DRIVER_AIR_MUTATION} UTPP_MOTION_DRIVER_AIR_MUTATION_TEST=${UTPP_MOTION_DRIVER_AIR_MUTATION})
target_compile_features(utpp_motion_driver_air_test PRIVATE cxx_std_20)
add_test(NAME utpp_motion_driver_air_test COMMAND utpp_motion_driver_air_test "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
# Vehicle batch (J2): mutation targets of the take-off plane seed (src/MotionDriver.cpp AirMut 12 = no seed, 13 = seed also for a
# walk-off). They pass only when the output carries the failure of the check the mutation must break (K / K drop guard).
set(_utpp_air_mut_expect12 "FAIL \\[rt_(run|stand)_jump\\] K TimeToLand")
set(_utpp_air_mut_expect13 "FAIL \\[rt_drop_1m5\\] K drop guard")
# J1 (W4, vehicle batch): UTPP_TRAJV2_HEAVY_MUTATION=1 (src/mm/MotionTrajectory.hpp: the heavy-landing threshold stays 700 at every
# gravity) must fail the hood jump-off landing (Heavy instead of Light).
add_executable(utpp_motion_driver_air_heavy_mut1 tests/MotionDriverAirTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/MotionDriver.cpp
               src/MotionDriver.hpp src/State.hpp src/mm/MotionAdapter.hpp src/mm/MotionPose.hpp src/mm/GameIntegration.hpp)
target_link_libraries(utpp_motion_driver_air_heavy_mut1 PRIVATE bcrypt)
target_compile_options(utpp_motion_driver_air_heavy_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_motion_driver_air_heavy_mut1 PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE
                           UTPP_MOTION_DRIVER_AIR_MUTATION=0 UTPP_MOTION_DRIVER_AIR_MUTATION_TEST=0 UTPP_TRAJV2_HEAVY_MUTATION=1)
target_compile_features(utpp_motion_driver_air_heavy_mut1 PRIVATE cxx_std_20)
add_test(NAME utpp_motion_driver_air_heavy_mut1 COMMAND utpp_motion_driver_air_heavy_mut1 "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
set_tests_properties(utpp_motion_driver_air_heavy_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL \\[hood_jump_off\\] D justLandedLight")
# CAR-1 (W1, vehicle batch): UTPP_VEHEXIT_MUTATION=2 (src/MotionHost.cpp: the exit window ends on the post-hold effective gate, so the
# exit workspot latched at the unmount releases) must fail the v2 exit-window check (v2 skips 'gate' during the exit animation).
add_executable(utpp_motion_driver_air_vehexit_mut2 tests/MotionDriverAirTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/MotionDriver.cpp
               src/MotionDriver.hpp src/State.hpp src/mm/MotionAdapter.hpp src/mm/MotionPose.hpp src/mm/GameIntegration.hpp)
target_link_libraries(utpp_motion_driver_air_vehexit_mut2 PRIVATE bcrypt)
target_compile_options(utpp_motion_driver_air_vehexit_mut2 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_motion_driver_air_vehexit_mut2 PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE
                           UTPP_MOTION_DRIVER_AIR_MUTATION=0 UTPP_MOTION_DRIVER_AIR_MUTATION_TEST=0 UTPP_VEHEXIT_MUTATION=2)
target_compile_features(utpp_motion_driver_air_vehexit_mut2 PRIVATE cxx_std_20)
add_test(NAME utpp_motion_driver_air_vehexit_mut2 COMMAND utpp_motion_driver_air_vehexit_mut2 "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
set_tests_properties(utpp_motion_driver_air_vehexit_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL \\[exit_ws_1s3\\] V motion steps")
foreach(_m 12 13)
  add_executable(utpp_motion_driver_air_mut${_m} tests/MotionDriverAirTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/MotionDriver.cpp
                 src/MotionDriver.hpp src/State.hpp src/mm/MotionAdapter.hpp src/mm/MotionPose.hpp src/mm/GameIntegration.hpp)
  target_link_libraries(utpp_motion_driver_air_mut${_m} PRIVATE bcrypt)
  target_compile_options(utpp_motion_driver_air_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_motion_driver_air_mut${_m} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE
                             UTPP_MOTION_DRIVER_AIR_MUTATION=${_m} UTPP_MOTION_DRIVER_AIR_MUTATION_TEST=${_m})
  target_compile_features(utpp_motion_driver_air_mut${_m} PRIVATE cxx_std_20)
  add_test(NAME utpp_motion_driver_air_mut${_m} COMMAND utpp_motion_driver_air_mut${_m} "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
  set_tests_properties(utpp_motion_driver_air_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "${_utpp_air_mut_expect${_m}}")
endforeach()
