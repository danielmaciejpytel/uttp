# ---- male V upper-body correction (src/mm/MaleBodyCorrection.hpp): world-preserving Spine3 / clavicle / upper-arm rotations by weapon family.
# Included from plugin/UTPP/CMakeLists.txt after motion_pose.cmake (uses its _utpp_pmotion_mov path = male overlay file). Mutation builds MUST print a "FAIL " line:
# 1 = children not compensated, 2 = crouch bit ignored in the key -> family map, 3 = family weights not clamped.
set(_utpp_mbc_args "${_utpp_pmotion_mov}")
add_executable(utpp_male_body_correction_test tests/MaleBodyCorrectionTest.cpp src/mm/MaleBodyCorrection.hpp)
target_compile_options(utpp_male_body_correction_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_male_body_correction_test PRIVATE cxx_std_20)
add_test(NAME utpp_male_body_correction_test COMMAND utpp_male_body_correction_test ${_utpp_mbc_args})
foreach(_m 1 2 3)
    add_executable(utpp_male_body_correction_mut${_m} tests/MaleBodyCorrectionTest.cpp src/mm/MaleBodyCorrection.hpp)
    target_compile_options(utpp_male_body_correction_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_male_body_correction_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_male_body_correction_mut${_m} PRIVATE UTPP_MALECORR_MUTATION=${_m})
    add_test(NAME utpp_male_body_correction_mut${_m} COMMAND utpp_male_body_correction_mut${_m} ${_utpp_mbc_args})
    set_tests_properties(utpp_male_body_correction_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()
