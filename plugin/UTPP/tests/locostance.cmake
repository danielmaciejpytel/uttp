# ---- Locomotion stance fixes (recording 2026-10-31 22:55, the recorded log): the real diag_b game feed
# (tests/data/loco_feeds_20261031_2255.txt) through MapToChooser + ChooserMatcher + PoseEvaluator (live base, Pistol2H overlay) on the installed
# slide bundle; the composed pose across the stance edges (RC1 run -> crouch, CR-4 crouch -> stand).
# Included from plugin/UTPP/CMakeLists.txt (include(tests/locostance.cmake OPTIONAL)); expects UTPP_WARNINGS. Data outside git: the installed
# bake (motion-derived, the knee/twistfix/slide bundle) and the woman overlay file, same lookup as locomatcher.cmake / motion_pose.cmake.
set(_utpp_ls_data "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data")
if(NOT EXISTS "${_utpp_ls_data}/utpp_motion_woman_knee_twistfix_slide.utppmm" AND UTPP_MOTION_DB_TEST_FILE)
    get_filename_component(_utpp_ls_data "${UTPP_MOTION_DB_TEST_FILE}" DIRECTORY)
endif()
set(_utpp_ls_ov "${_utpp_ls_data}/motion/utpp_overlays_woman.utppov")
set(_utpp_ls_style "${_utpp_ls_data}/motion/utpp_style_weights_woman.txt")
if(EXISTS "${_utpp_ls_data}/utpp_motion_woman_knee_twistfix_slide.utppmm"
   AND EXISTS "${_utpp_ls_data}/utpp_motion_woman_knee_twistfix_slide.chooser.json"
   AND EXISTS "${_utpp_ls_ov}" AND EXISTS "${_utpp_ls_style}")
    set(_utpp_ls_args "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/loco_feeds_20261031_2255.txt"
        "${_utpp_ls_data}/utpp_motion_woman_knee_twistfix_slide" "${_utpp_ls_ov}" "${_utpp_ls_style}")
    add_executable(utpp_mm_loco_stance_test tests/LocoStanceTest.cpp src/mm/Overlay.hpp src/mm/MotionPose.hpp src/mm/ChooserMatcher.hpp)
    target_compile_features(utpp_mm_loco_stance_test PRIVATE cxx_std_20)
    target_compile_options(utpp_mm_loco_stance_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_mm_loco_stance_test PRIVATE bcrypt)
    add_test(NAME utpp_mm_loco_stance_test COMMAND utpp_mm_loco_stance_test ${_utpp_ls_args})
    # Mutations (UTPP_LOCOSTANCE_MUTATION=N in src/mm/Overlay.hpp + src/mm/MotionPose.hpp), each must print its own FAIL line
    # (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error cannot count as a killed mutant):
    #   1 = the spine / head regions keep the pose crossfade (pose and layer weights) on a stance change (CR-4, pre-fix);
    #   2 = one spine / head inner on the stance alpha of B (OPLower + B-synced curves): the cross term keeps the torso upright running into a crouch;
    #   3 = a group without an overlay pose (holster / draw inside the fade) drops the spine / head to the crossfade path for that frame.
    set(_utpp_ls_expect_1 "FAIL CR-4 crouch -> stand")
    set(_utpp_ls_expect_2 "FAIL RC1 stand -> crouch at [0-9.]+ [(]moving[)]: the overlay keeps")
    set(_utpp_ls_expect_3 "FAIL CR-4 holster / draw inside the crouch -> stand fade")
    foreach(_utpp_ls_mut 1 2 3)
        add_executable(utpp_mm_loco_stance_mut${_utpp_ls_mut} tests/LocoStanceTest.cpp src/mm/Overlay.hpp src/mm/MotionPose.hpp src/mm/ChooserMatcher.hpp)
        target_compile_features(utpp_mm_loco_stance_mut${_utpp_ls_mut} PRIVATE cxx_std_20)
        target_compile_options(utpp_mm_loco_stance_mut${_utpp_ls_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
        target_compile_definitions(utpp_mm_loco_stance_mut${_utpp_ls_mut} PRIVATE UTPP_LOCOSTANCE_MUTATION=${_utpp_ls_mut})
        target_link_libraries(utpp_mm_loco_stance_mut${_utpp_ls_mut} PRIVATE bcrypt)
        add_test(NAME utpp_mm_loco_stance_mut${_utpp_ls_mut} COMMAND utpp_mm_loco_stance_mut${_utpp_ls_mut} ${_utpp_ls_args})
        set_tests_properties(utpp_mm_loco_stance_mut${_utpp_ls_mut} PROPERTIES PASS_REGULAR_EXPRESSION "${_utpp_ls_expect_${_utpp_ls_mut}}")
    endforeach()
endif()
