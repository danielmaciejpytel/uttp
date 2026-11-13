# ---- Locomotion matcher fixes (recording 2026-10-31 22:55, the recorded log): the real diag_b game feed
# (tests/data/loco_feeds_20261031_2255.txt) through MapToChooser + ChooserMatcher + BlendStack on the installed slide bundle.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/locomatcher.cmake OPTIONAL)); expects UTPP_WARNINGS. Data outside git: the installed
# bake (motion-derived, the knee/twistfix/slide bundle), same lookup as jumppick.cmake.
set(_utpp_lm_data "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data")
if(NOT EXISTS "${_utpp_lm_data}/utpp_motion_woman_knee_twistfix_slide.utppmm" AND UTPP_MOTION_DB_TEST_FILE)
    get_filename_component(_utpp_lm_data "${UTPP_MOTION_DB_TEST_FILE}" DIRECTORY)
endif()
if(EXISTS "${_utpp_lm_data}/utpp_motion_woman_knee_twistfix_slide.utppmm"
   AND EXISTS "${_utpp_lm_data}/utpp_motion_woman_knee_twistfix_slide.chooser.json")
    add_executable(utpp_mm_loco_matcher_test tests/LocoMatcherTest.cpp src/mm/MotionBlendStack.hpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp src/mm/MotionPlayRate.hpp)
    target_compile_features(utpp_mm_loco_matcher_test PRIVATE cxx_std_20)
    target_compile_options(utpp_mm_loco_matcher_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_mm_loco_matcher_test PRIVATE bcrypt)
    add_test(NAME utpp_mm_loco_matcher_test COMMAND utpp_mm_loco_matcher_test
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/loco_feeds_20261031_2255.txt"
             "${_utpp_lm_data}/utpp_motion_woman_knee_twistfix_slide")
    # Mutations (UTPP_LOCOMATCHER_MUTATION=N in src/mm/ChooserMatcher.hpp), each must print its own FAIL line (PASS_REGULAR_EXPRESSION ignores the
    # exit code, so a setup error cannot count as a killed mutant):
    #   1 = the crouch walk loops keep the ordinary 0.85 bound (CR-3);
    #   2 = no slide entry cap: the slide starts at Into 0.93 s (RC2; off in production, tested with the cap set to 0.6);
    #   3 = the entry cap leaves the slide Loop clips in (RC2);
    #   4 = no Into -> Loop handover: the mirrored Into re-jump mid-slide (RC3);
    #   5 = no hold of a selected Crouch -> Stand transition (CR-4).
    #   6 = the walk Start tail keeps competing outside the chooser set (ST-1, src/mm/ChooserMatcher.hpp MatcherConfig::slowWalkStartTail).
    #   7 = no stop commit: a selected Stop clip competes on cost from its next update on (STOP, MatcherConfig::stopCommitTailS).
    set(_utpp_lm_expect_1 "FAIL CR-3 slow crouch walk: crouch loop rate at the 0[.]5 floor")
    set(_utpp_lm_expect_2 "FAIL RC2 cap slide entry plays an Into clip at most 0[.]6 s in")
    set(_utpp_lm_expect_3 "FAIL RC2 cap slide entry plays an Into clip at most 0[.]6 s in")
    set(_utpp_lm_expect_4 "FAIL RC3 prod mid-slide: no re-pick of an Into clip")
    set(_utpp_lm_expect_5 "FAIL CR-4 a selected Crouch_to_Stand is still playing")
    set(_utpp_lm_expect_6 "FAIL ST-1 slow stand walk 99[.]0-101[.]3 s plays a Walk_Loop_")
    set(_utpp_lm_expect_7 "FAIL STOP a selected Stop clip plays on while the stick stays released")
    foreach(_utpp_lm_mut 1 2 3 4 5 6 7)
        add_executable(utpp_mm_loco_matcher_mut${_utpp_lm_mut} tests/LocoMatcherTest.cpp src/mm/MotionBlendStack.hpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp src/mm/MotionPlayRate.hpp)
        target_compile_features(utpp_mm_loco_matcher_mut${_utpp_lm_mut} PRIVATE cxx_std_20)
        target_compile_options(utpp_mm_loco_matcher_mut${_utpp_lm_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
        target_compile_definitions(utpp_mm_loco_matcher_mut${_utpp_lm_mut} PRIVATE UTPP_LOCOMATCHER_MUTATION=${_utpp_lm_mut})
        target_link_libraries(utpp_mm_loco_matcher_mut${_utpp_lm_mut} PRIVATE bcrypt)
        add_test(NAME utpp_mm_loco_matcher_mut${_utpp_lm_mut} COMMAND utpp_mm_loco_matcher_mut${_utpp_lm_mut}
                 "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/loco_feeds_20261031_2255.txt"
                 "${_utpp_lm_data}/utpp_motion_woman_knee_twistfix_slide")
        set_tests_properties(utpp_mm_loco_matcher_mut${_utpp_lm_mut} PROPERTIES PASS_REGULAR_EXPRESSION "${_utpp_lm_expect_${_utpp_lm_mut}}")
    endforeach()
endif()
