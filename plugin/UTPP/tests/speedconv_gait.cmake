# ---- Speed converter, gait ramp (speedconv::GaitStep, MapConfig::speedConverter bit kGaitRamp): ramp across a gait change, hand-off
# at the game speed / after kGaitRampMaxS / on an inactive feed, held speeds unchanged. No data files.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/speedconv_gait.cmake OPTIONAL)); expects UTPP_WARNINGS.
add_executable(utpp_mm_loco_speedconv_gait_test tests/SpeedConv_gaitTest.cpp src/mm/speedconv_gait.hpp)
target_compile_options(utpp_mm_loco_speedconv_gait_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_loco_speedconv_gait_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_loco_speedconv_gait_test COMMAND utpp_mm_loco_speedconv_gait_test)
# Mutations (UTPP_SPEED_CONVERTER_MUTATION=N), each must fail its own check:
#   20 = no gait ramp: the first walk -> run feed is already at the run speed;
#   21 = a slow-down ramps at the acceleration: the sprint release brakes too fast;
#   22 = no time hand-off: a game speed that keeps moving away holds the ramp;
#   23 = a ramp also starts inside a speed change: a gait change during a start ramps;
#   24 = no confirmation: the walk gait the stick passes on a stop ramps.
foreach(_utpp_scg_mut 20 21 22 23 24)
    add_executable(utpp_mm_loco_speedconv_gait_mut${_utpp_scg_mut} tests/SpeedConv_gaitTest.cpp src/mm/speedconv_gait.hpp)
    target_compile_options(utpp_mm_loco_speedconv_gait_mut${_utpp_scg_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_mm_loco_speedconv_gait_mut${_utpp_scg_mut} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_mm_loco_speedconv_gait_mut${_utpp_scg_mut} PRIVATE UTPP_SPEED_CONVERTER_MUTATION=${_utpp_scg_mut})
    add_test(NAME utpp_mm_loco_speedconv_gait_mut${_utpp_scg_mut} COMMAND utpp_mm_loco_speedconv_gait_mut${_utpp_scg_mut})
endforeach()
set_tests_properties(utpp_mm_loco_speedconv_gait_mut20 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL walk->run ramps after the confirmation")
set_tests_properties(utpp_mm_loco_speedconv_gait_mut21 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL sprint release brakes at the input braking")
set_tests_properties(utpp_mm_loco_speedconv_gait_mut22 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL time hand-off")
set_tests_properties(utpp_mm_loco_speedconv_gait_mut23 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL no ramp inside a start")
set_tests_properties(utpp_mm_loco_speedconv_gait_mut24 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL stop walk gait not ramped")
