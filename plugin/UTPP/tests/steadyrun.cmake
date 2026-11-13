# ---- G10: steady Run gait between the walk cap and the run loops (analog stick 0.7-0.82) on the real v2 database plays Walk_Loop_* / Run_Loop_* clips, no pose reset.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/steadyrun.cmake OPTIONAL)); expects UTPP_WARNINGS. Data outside git: the installed bake
# (motion-derived, the knee/twistfix/slide bundle), same lookup as slowwalk.cmake.
set(_utpp_sr_data "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data")
if(NOT EXISTS "${_utpp_sr_data}/utpp_motion_woman_knee_twistfix_slide.utppmm" AND UTPP_MOTION_DB_TEST_FILE)
    get_filename_component(_utpp_sr_data "${UTPP_MOTION_DB_TEST_FILE}" DIRECTORY)
endif()
if(EXISTS "${_utpp_sr_data}/utpp_motion_woman_knee_twistfix_slide.utppmm"
   AND EXISTS "${_utpp_sr_data}/utpp_motion_woman_knee_twistfix_slide.chooser.json")
    add_executable(utpp_mm_steadyrun_test tests/SteadyRunLoopTest.cpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp src/mm/MotionPlayRate.hpp)
    target_compile_features(utpp_mm_steadyrun_test PRIVATE cxx_std_20)
    target_compile_options(utpp_mm_steadyrun_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_mm_steadyrun_test PRIVATE bcrypt)
    add_test(NAME utpp_mm_steadyrun_test COMMAND utpp_mm_steadyrun_test "${_utpp_sr_data}/utpp_motion_woman_knee_twistfix_slide.utppmm")
endif()
