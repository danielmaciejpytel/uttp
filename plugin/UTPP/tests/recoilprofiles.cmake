# ---- fix3/recoil-profiles (step B of P9): authored recoil per weapon family. Included from tests/hipfireweapons.cmake; expects
# UTPP_WARNINGS, _utpp_mdmotion_data_native and UTPP_MMDB_TEST_FILE. The sampler files live in <data>/recoil (outside git, written by
# tools/analysis/export_nue_native_recoil_curves.py --profiles).
# Mutation builds / tampered inputs MUST report a failure ("FAIL " in the output): PASS_REGULAR_EXPRESSION ignores the exit code.

# Table + data + reds/exporter sync. The reds are the only place that maps a weapon record to an id; a drifted copy has to fail.
add_executable(utpp_recoil_profiles_test tests/RecoilProfilesTest.cpp src/mm/AuthoredRecoil.hpp src/mm/RecoilProfiles.hpp)
target_compile_features(utpp_recoil_profiles_test PRIVATE cxx_std_20)
target_compile_options(utpp_recoil_profiles_test PRIVATE ${UTPP_WARNINGS} /EHsc)
set(_utpp_rp_recoil_dir "${_utpp_mdmotion_data_native}\\recoil")
set(_utpp_rp_reds "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_motion.reds")
set(_utpp_rp_exporter "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/analysis/export_nue_native_recoil_curves.py")
add_test(NAME utpp_recoil_profiles_test COMMAND utpp_recoil_profiles_test "${_utpp_rp_recoil_dir}" "${_utpp_rp_reds}" "${_utpp_rp_exporter}")

# Mutation of the input: the Ajax line of the reds returns another id -> the sync check fails.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_utpp_rp_reds}")
file(READ "${_utpp_rp_reds}" _utpp_rp_text)
string(REPLACE "Equals(family, n\"Ajax\") { return 12; }" "Equals(family, n\"Ajax\") { return 13; }" _utpp_rp_text "${_utpp_rp_text}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/recoil_profiles_tampered.reds" "${_utpp_rp_text}")
add_test(NAME utpp_recoil_profiles_mut_reds COMMAND utpp_recoil_profiles_test "${_utpp_rp_recoil_dir}"
         "${CMAKE_CURRENT_BINARY_DIR}/recoil_profiles_tampered.reds" "${_utpp_rp_exporter}")
set_tests_properties(utpp_recoil_profiles_mut_reds PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")

# Primitive of the player/clip (src/mm/AuthoredRecoil.hpp), registered here for the first time: synthetic fixtures, no data dir.
add_executable(utpp_authored_recoil_test tests/AuthoredRecoilTest.cpp src/mm/AuthoredRecoil.hpp)
target_compile_features(utpp_authored_recoil_test PRIVATE cxx_std_20)
target_compile_options(utpp_authored_recoil_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_authored_recoil_test COMMAND utpp_authored_recoil_test "${CMAKE_CURRENT_BINARY_DIR}/authored_recoil_fixture")

# Driver level: SetRecoilProfile / UTPP_MOTION_DRIVER_RECOIL_PROFILE_MUTATION (profile ignored = the pre-profile build, only the Nue has one)
# MUST fail the profile cases of utpp_motion_driver_hipfire_weapons_test (MotionDriverHipFireWeaponsTest.cpp).
add_executable(utpp_motion_driver_recoil_profile_mut1 tests/MotionDriverHipFireWeaponsTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
target_compile_features(utpp_motion_driver_recoil_profile_mut1 PRIVATE cxx_std_20)
target_compile_options(utpp_motion_driver_recoil_profile_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_motion_driver_recoil_profile_mut1 PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV
                           UTPP_MOTION_DRIVER_RECOIL_PROFILE_MUTATION=1)
target_link_libraries(utpp_motion_driver_recoil_profile_mut1 PRIVATE bcrypt)
add_test(NAME utpp_motion_driver_recoil_profile_mut1 COMMAND utpp_motion_driver_recoil_profile_mut1 "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
set_tests_properties(utpp_motion_driver_recoil_profile_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")

# ---- review round (Masamune / revolver bursts, reload and hip <-> ADS exits). Every mutation MUST report a failure ("FAIL ").
# Player level: the series limits are not passed (legacy accumulation), and a clip switch deletes the running offset.
foreach(_m limits switch)
    string(TOUPPER ${_m} _M)
    add_executable(utpp_authored_recoil_mut_${_m} tests/AuthoredRecoilTest.cpp src/mm/AuthoredRecoil.hpp)
    target_compile_features(utpp_authored_recoil_mut_${_m} PRIVATE cxx_std_20)
    target_compile_options(utpp_authored_recoil_mut_${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_authored_recoil_mut_${_m} PRIVATE UTPP_RECOIL_${_M}_MUTATION=1)
    add_test(NAME utpp_authored_recoil_mut_${_m} COMMAND utpp_authored_recoil_mut_${_m} "${CMAKE_CURRENT_BINARY_DIR}/authored_recoil_fixture_${_m}")
    set_tests_properties(utpp_authored_recoil_mut_${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()

# Profile level: LimitsOf returns the legacy mode for every profile -> the burst checks of the profile test fail on the clamp / steps.
add_executable(utpp_recoil_profiles_mut_series tests/RecoilProfilesTest.cpp src/mm/AuthoredRecoil.hpp src/mm/RecoilProfiles.hpp)
target_compile_features(utpp_recoil_profiles_mut_series PRIVATE cxx_std_20)
target_compile_options(utpp_recoil_profiles_mut_series PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_recoil_profiles_mut_series PRIVATE UTPP_RECOIL_SERIES_MUTATION=1)
add_test(NAME utpp_recoil_profiles_mut_series COMMAND utpp_recoil_profiles_mut_series "${_utpp_rp_recoil_dir}" "${_utpp_rp_reds}" "${_utpp_rp_exporter}")
set_tests_properties(utpp_recoil_profiles_mut_series PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")

# Driver level (child cases of MotionDriverHipFireWeaponsTest.cpp): the kick is deleted at once on a reload start / domain exit (exit
# mutation), a clip switch deletes the offset (switch mutation), the profiles accumulate like the Nue (series mutation).
# removed 2026-11-05 (review R2), which cheaper test kills the same site: utpp_authored_recoil_mut_switch, utpp_recoil_profiles_mut_series
foreach(_m "recoil_exit:UTPP_MOTION_DRIVER_RECOIL_EXIT_MUTATION")
    string(REPLACE ":" ";" _parts "${_m}")
    list(GET _parts 0 _tag)
    list(GET _parts 1 _def)
    set(_tgt utpp_motion_driver_${_tag}_mut1)
    add_executable(${_tgt} tests/MotionDriverHipFireWeaponsTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(${_tgt} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV ${_def}=1)
    target_link_libraries(${_tgt} PRIVATE bcrypt)
    add_test(NAME ${_tgt} COMMAND ${_tgt} "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
    set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()

# Reach of the hand IK for every profile of the table, hip + ADS (MotionDriverRecoilReachTest.cpp). Mutation 1 of RecoilProfiles.hpp
# (UTPP_RECOIL_SERIES_MUTATION: every profile accumulates like the Nue) must fail the bounded-step / cap asserts of the burst cases.
# removed 2026-11-05 (review R3), which cheaper test kills the same site: utpp_recoil_profiles_mut_series
foreach(_m "")
    if(_m STREQUAL "")
        set(_tgt utpp_motion_driver_recoil_reach_test)
    else()
        set(_tgt utpp_motion_driver_recoil_reach_mut${_m})
    endif()
    add_executable(${_tgt} tests/MotionDriverRecoilReachTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(${_tgt} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
    if(NOT _m STREQUAL "")
        target_compile_definitions(${_tgt} PRIVATE UTPP_RECOIL_SERIES_MUTATION=${_m})
    endif()
    target_link_libraries(${_tgt} PRIVATE bcrypt)
    add_test(NAME ${_tgt} COMMAND ${_tgt} "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
    if(NOT _m STREQUAL "")
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
    endif()
endforeach()
