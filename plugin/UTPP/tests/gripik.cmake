# ---- port stage 10.6: weapon grip hand IK (src/mm/GripIK.hpp) offline test on the real CP rig poses.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/gripik.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Uses the same baked .utppov files as tests/overlay.cmake (UTPP_OVERLAY_WOMAN_FILE / UTPP_OVERLAY_MAN_FILE, motion-derived,
# outside git); a missing file makes the test FAIL (load check), no synthetic fallback.
foreach(_utpp_gik_var UTPP_OVERLAY_WOMAN_FILE UTPP_OVERLAY_MAN_FILE)
  if(NOT DEFINED ${_utpp_gik_var})
    set(${_utpp_gik_var} "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/overlay/${_utpp_gik_var}-NOT-SET")
  endif()
endforeach()
add_executable(utpp_mm_gripik_test tests/GripIKTest.cpp src/mm/GripIK.hpp src/mm/Additive.hpp src/mm/Overlay.hpp
               src/mm/OverlayFile.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_mm_gripik_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_gripik_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_gripik_test COMMAND utpp_mm_gripik_test "${UTPP_OVERLAY_WOMAN_FILE}" "${UTPP_OVERLAY_MAN_FILE}")

# Native shoot transfer (src/mm/NativeShootTransfer.hpp): chest-relative weapon motion + post-solver arm activation fade (synthetic, no data files).
add_executable(utpp_mm_native_shoot_chest_motion_test tests/NativeShootChestMotionTest.cpp src/mm/NativeShootTransfer.hpp src/mm/GripIK.hpp
               src/mm/Overlay.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_mm_native_shoot_chest_motion_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_native_shoot_chest_motion_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_native_shoot_chest_motion_test COMMAND utpp_mm_native_shoot_chest_motion_test)
