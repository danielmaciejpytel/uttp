# ---- fix3/hipfire-weapons (P9): hipfire with a weapon that has no recoil route keeps the authored Aim arms. Included from
# plugin/UTPP/CMakeLists.txt (after motiondriver.cmake); expects UTPP_WARNINGS, _utpp_mdmotion_data_native and UTPP_MMDB_TEST_FILE.
# Mutation builds MUST report an assertion failure: they pass only when the output contains a line starting "FAIL "
# (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error cannot count as a killed mutant).

# Driver level: the hipfire-* cases of the integration test in child processes. Mutation 1 (UTPP_MOTION_DRIVER_HIPFIRE_ROUTE_MUTATION: the
# route gate is always open = the pre-P9 native first-burst fallback for every weapon) MUST fail the rifle / non-Nue pistol cases;
# mutation 3 (the legacy native FK stance, Pistol2H standing without a profile, is a route again = the Overture revolver hand-over of
# recording 2026-10-25) MUST fail hipfire-pistol2h-other-still and the hipfire-route-* cases. (Old mutation 2, route follows every
# frame, is gone: with the route = the authored domain the live gate no longer flips inside a shot in any harness scenario.)
foreach(_m "" 1 3)
    if(_m STREQUAL "")
        set(_tgt utpp_motion_driver_hipfire_weapons_test)
    else()
        set(_tgt utpp_motion_driver_hipfire_weapons_mut${_m})
    endif()
    add_executable(${_tgt} tests/MotionDriverHipFireWeaponsTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(${_tgt} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
    if(NOT _m STREQUAL "")
        target_compile_definitions(${_tgt} PRIVATE UTPP_MOTION_DRIVER_HIPFIRE_ROUTE_MUTATION=${_m})
    endif()
    target_link_libraries(${_tgt} PRIVATE bcrypt)
    add_test(NAME ${_tgt} COMMAND ${_tgt} "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
    if(NOT _m STREQUAL "")
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
    endif()
endforeach()

# Policy level: HipFireNativeActionOwner (HipFirePose.hpp) with transferRoute. Mutation 1 (UTPP_HIPFIRE_ROUTE_MUTATION: the route is
# always true) MUST fail the new asserts of the overlay test (synthetic part, no data argument) and the warm-up test (exit code 10).
add_executable(utpp_mm_overlay_hipfire_mut1 tests/OverlayTest.cpp src/mm/Additive.hpp src/mm/Overlay.hpp src/mm/OverlayFile.hpp
               src/mm/HipFirePose.hpp)
target_compile_options(utpp_mm_overlay_hipfire_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_overlay_hipfire_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_mm_overlay_hipfire_mut1 PRIVATE UTPP_HIPFIRE_ROUTE_MUTATION=1)
add_test(NAME utpp_mm_overlay_hipfire_mut1 COMMAND utpp_mm_overlay_hipfire_mut1)
set_tests_properties(utpp_mm_overlay_hipfire_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL: hipfire P9")

add_executable(utpp_mm_hipfire_warmup_mut1 tests/HipFireWarmupTest.cpp)
target_compile_options(utpp_mm_hipfire_warmup_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_hipfire_warmup_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_mm_hipfire_warmup_mut1 PRIVATE UTPP_HIPFIRE_ROUTE_MUTATION=1)
add_test(NAME utpp_mm_hipfire_warmup_mut1 COMMAND utpp_mm_hipfire_warmup_mut1)
set_tests_properties(utpp_mm_hipfire_warmup_mut1 PROPERTIES WILL_FAIL TRUE)

# P9 C: the hip raise out of Ready (OverlayTimes::hipFireReadyToAiming, OverlayInputs::hipFireRaise). Mutation 1
# (UTPP_OVL_HIPFIRE_MUTATION: the hip shot keeps the 0.2 s readyToAiming) MUST fail the machine checks of the overlay test and the
# first-shot raise bound of the driver-level hipfire cases.
add_executable(utpp_mm_overlay_hipraise_mut1 tests/OverlayTest.cpp src/mm/Additive.hpp src/mm/Overlay.hpp src/mm/OverlayFile.hpp
               src/mm/HipFirePose.hpp)
target_compile_options(utpp_mm_overlay_hipraise_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_overlay_hipraise_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_mm_overlay_hipraise_mut1 PRIVATE UTPP_OVL_HIPFIRE_MUTATION=1)
add_test(NAME utpp_mm_overlay_hipraise_mut1 COMMAND utpp_mm_overlay_hipraise_mut1)
set_tests_properties(utpp_mm_overlay_hipraise_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL: hipfire P9 C")

# removed 2026-11-05 (review R1), which cheaper test kills the same site: utpp_mm_overlay_hipraise_mut1

# P9 A (review): a stance / move key change inside the hip raise must not restart the hands fade. Mutation 2 (UTPP_OVL_HIPFIRE_MUTATION=2)
# = no guard: the hands reach Aim 0.18 - 0.22 s after the shot while the AO alpha is full after 0.1 s; the overlay test MUST fail.
add_executable(utpp_mm_overlay_hipraise_mut2 tests/OverlayTest.cpp src/mm/Additive.hpp src/mm/Overlay.hpp src/mm/OverlayFile.hpp
               src/mm/HipFirePose.hpp)
target_compile_options(utpp_mm_overlay_hipraise_mut2 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_overlay_hipraise_mut2 PRIVATE cxx_std_20)
target_compile_definitions(utpp_mm_overlay_hipraise_mut2 PRIVATE UTPP_OVL_HIPFIRE_MUTATION=2)
add_test(NAME utpp_mm_overlay_hipraise_mut2 COMMAND utpp_mm_overlay_hipraise_mut2)
set_tests_properties(utpp_mm_overlay_hipraise_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL: hipfire P9 A")

# P9 B (review 3): the AO alpha of the hip shot takes the raise with the hands (0.1 s; 600 deg/s at a 60 deg yaw is under AoJumpDeg(dt)).
# Mutation 3 (UTPP_OVL_HIPFIRE_MUTATION=3) = the AO alpha trails the hands (0.2 s): the overlay test and the driver-level hipfire cases MUST fail.
add_executable(utpp_mm_overlay_hipraise_mut3 tests/OverlayTest.cpp src/mm/Additive.hpp src/mm/Overlay.hpp src/mm/OverlayFile.hpp
               src/mm/HipFirePose.hpp)
target_compile_options(utpp_mm_overlay_hipraise_mut3 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_overlay_hipraise_mut3 PRIVATE cxx_std_20)
target_compile_definitions(utpp_mm_overlay_hipraise_mut3 PRIVATE UTPP_OVL_HIPFIRE_MUTATION=3)
add_test(NAME utpp_mm_overlay_hipraise_mut3 COMMAND utpp_mm_overlay_hipraise_mut3)
set_tests_properties(utpp_mm_overlay_hipraise_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL: hipfire P9 B")

# removed 2026-11-05 (review R1), which cheaper test kills the same site: utpp_mm_overlay_hipraise_mut3

include(${CMAKE_CURRENT_SOURCE_DIR}/tests/recoilprofiles.cmake OPTIONAL) # fix batch 3 step B: authored recoil profiles per weapon family

# Male V parity of the shared weapon-local recoil profiles (W5 2026-11-06): the shipped <data>/recoil files against the profiles exported from
# the male pma_* animsets with the same settings (<data>/recoil_male; export_nue_native_recoil_curves.py --source-dir <male glb> --source-prefix pma).
# Reuses the utpp_authored_recoil_test executable (defined by the include above); skipped when the male export is absent.
if(TARGET utpp_authored_recoil_test AND EXISTS "${_utpp_mdmotion_data}/recoil_male")
    add_test(NAME utpp_authored_recoil_male_parity_test COMMAND utpp_authored_recoil_test "${CMAKE_CURRENT_BINARY_DIR}/authored_recoil_fixture_male"
             "${_utpp_mdmotion_data_native}\\recoil" "${_utpp_mdmotion_data_native}\\recoil_male")
endif()
