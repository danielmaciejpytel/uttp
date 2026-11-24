# ---- speed converter strafe travel (mm/speedconv_stance.hpp, MapConfig::speedConverter bit 512; off by default) ----
# Included from plugin/UTPP/CMakeLists.txt; expects UTPP_WARNINGS to be defined.
add_executable(utpp_motion_adapter_unit_speedconv_stance_test tests/SpeedConv_stanceTest.cpp src/mm/MotionAdapter.hpp
               src/mm/SpeedConverter.hpp src/mm/speedconv_stance.hpp)
target_compile_options(utpp_motion_adapter_unit_speedconv_stance_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_motion_adapter_unit_speedconv_stance_test PRIVATE cxx_std_20)
add_test(NAME utpp_motion_adapter_unit_speedconv_stance_test COMMAND utpp_motion_adapter_unit_speedconv_stance_test)
# mutations: 40 the strafe limits are never applied, 41 reversals keep the game velocity, 42 the turn friction stays the converter's
# (each must FAIL its check)
foreach(_utpp_scs_mut IN ITEMS 40 41 42)
  set(_utpp_scs_t utpp_motion_adapter_unit_speedconv_stance_test_mut${_utpp_scs_mut})
  add_executable(${_utpp_scs_t} tests/SpeedConv_stanceTest.cpp src/mm/MotionAdapter.hpp src/mm/SpeedConverter.hpp
                 src/mm/speedconv_stance.hpp)
  target_compile_options(${_utpp_scs_t} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(${_utpp_scs_t} PRIVATE cxx_std_20)
  target_compile_definitions(${_utpp_scs_t} PRIVATE UTPP_SPEED_CONVERTER_MUTATION=${_utpp_scs_mut})
  add_test(NAME ${_utpp_scs_t} COMMAND ${_utpp_scs_t})
endforeach()
set_tests_properties(utpp_motion_adapter_unit_speedconv_stance_test_mut40 PROPERTIES PASS_REGULAR_EXPRESSION
                     "FAIL driver: with the bit a strafe reversal slows down")
set_tests_properties(utpp_motion_adapter_unit_speedconv_stance_test_mut41 PROPERTIES PASS_REGULAR_EXPRESSION
                     "FAIL stance limits: strafing converts every reversal")
set_tests_properties(utpp_motion_adapter_unit_speedconv_stance_test_mut42 PROPERTIES PASS_REGULAR_EXPRESSION
                     "FAIL stance limits: strafing turns at the reference ground friction")
