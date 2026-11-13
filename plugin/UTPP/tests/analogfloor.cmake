# ---- Minimum analog walk speed (CET mm.minWalkSpeed): adapter::AnalogWalkFloorMul / AnalogFloorStickFor and MapToChooser with the floor
# on / off (identity at and above the floor stick, predicted cap = the reachable lifted speed).
# Included from plugin/UTPP/CMakeLists.txt (include(tests/analogfloor.cmake OPTIONAL)); expects UTPP_WARNINGS. No data files.
add_executable(utpp_mm_analogfloor_test tests/AnalogWalkFloorTest.cpp src/mm/MotionAdapter.hpp)
target_compile_options(utpp_mm_analogfloor_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_analogfloor_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_analogfloor_test COMMAND utpp_mm_analogfloor_test)
# Mutations (UTPP_ANALOGFLOOR_MUTATION=N), each must fail its own check:
#   1 = no 1.03 threshold: the |mv| 0.45 cluster keeps ~1.01;
#   2 = no driver lift (acceleration and cap of the raw stick): the |mv| 0.09 prediction stays at the raw stick's speed;
#   3 = no x4 cap in the driver lift: the |mv| 0.04 prediction runs ahead of the capsule to the floor;
#   4 = the stand-run fraction and cap under the walk toggle: the walk |mv| 0.15 prediction stays at ~0.47 m/s;
#   5 = the stand-run cap 4 under the walk toggle: the walk |mv| 0.09 prediction stays under the floor.
foreach(_utpp_af_mut 1 2 3 4 5)
    add_executable(utpp_mm_analogfloor_mut${_utpp_af_mut} tests/AnalogWalkFloorTest.cpp src/mm/MotionAdapter.hpp)
    target_compile_options(utpp_mm_analogfloor_mut${_utpp_af_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_mm_analogfloor_mut${_utpp_af_mut} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_mm_analogfloor_mut${_utpp_af_mut} PRIVATE UTPP_ANALOGFLOOR_MUTATION=${_utpp_af_mut})
    add_test(NAME utpp_mm_analogfloor_mut${_utpp_af_mut} COMMAND utpp_mm_analogfloor_mut${_utpp_af_mut})
endforeach()
set_tests_properties(utpp_mm_analogfloor_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL \\|mv\\| 0\\.45 \\(measured cluster")
set_tests_properties(utpp_mm_analogfloor_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL stand \\|mv\\| 0\\.090 floor: predicted")
set_tests_properties(utpp_mm_analogfloor_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL stand \\|mv\\| 0\\.040 floor: predicted")
set_tests_properties(utpp_mm_analogfloor_mut4 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL walk \\|mv\\| 0\\.150 floor: predicted")
set_tests_properties(utpp_mm_analogfloor_mut5 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL walk \\|mv\\| 0\\.090 floor: predicted")
