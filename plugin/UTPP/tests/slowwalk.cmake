# ---- G1: established slow walk (analog half stick, walk toggle) on the real v2 database picks the Walk_Loop_* clips, not the Run_to_Walk transition.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/slowwalk.cmake OPTIONAL)); expects UTPP_WARNINGS. Data outside git: the installed bake
# (motion-derived, the knee/twistfix/slide bundle), same lookup as jumppick.cmake.
set(_utpp_sw_data "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data")
if(NOT EXISTS "${_utpp_sw_data}/utpp_motion_woman_knee_twistfix_slide.utppmm" AND UTPP_MOTION_DB_TEST_FILE)
    get_filename_component(_utpp_sw_data "${UTPP_MOTION_DB_TEST_FILE}" DIRECTORY)
endif()
if(EXISTS "${_utpp_sw_data}/utpp_motion_woman_knee_twistfix_slide.utppmm"
   AND EXISTS "${_utpp_sw_data}/utpp_motion_woman_knee_twistfix_slide.chooser.json")
    add_executable(utpp_mm_slowwalk_test tests/SlowWalkLoopTest.cpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp src/mm/MotionPlayRate.hpp)
    target_compile_features(utpp_mm_slowwalk_test PRIVATE cxx_std_20)
    target_compile_options(utpp_mm_slowwalk_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_mm_slowwalk_test PRIVATE bcrypt)
    add_test(NAME utpp_mm_slowwalk_test COMMAND utpp_mm_slowwalk_test "${_utpp_sw_data}/utpp_motion_woman_knee_twistfix_slide.utppmm")
    # Mutation (UTPP_STICK_RELEASED_MUTATION=1: kStickReleased back at 0.1): the |mv| 0.09 small tilt counts as released and stays in
    # Idles, the minimum analog walk check must print FAIL.
    add_executable(utpp_mm_slowwalk_mut1 tests/SlowWalkLoopTest.cpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp src/mm/MotionPlayRate.hpp)
    target_compile_features(utpp_mm_slowwalk_mut1 PRIVATE cxx_std_20)
    target_compile_options(utpp_mm_slowwalk_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_mm_slowwalk_mut1 PRIVATE UTPP_STICK_RELEASED_MUTATION=1)
    target_link_libraries(utpp_mm_slowwalk_mut1 PRIVATE bcrypt)
    add_test(NAME utpp_mm_slowwalk_mut1 COMMAND utpp_mm_slowwalk_mut1 "${_utpp_sw_data}/utpp_motion_woman_knee_twistfix_slide.utppmm")
    set_tests_properties(utpp_mm_slowwalk_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL Hz=[0-9]+ stand \\|mv\\| 0.09")
endif()
