# ---- Start from rest and jump take-off (2026-10-31 loco diagnostic, findings CR-2 / GW-2 / HIP-3 and JS1): real feed windows
# (tests/data/loco_start_feeds_20261031_2255.txt) and synthetic starts / pivots through AirFloorStep + MapToChooser + ChooserMatcher on the
# installed woman v2 bundle. Included from plugin/UTPP/CMakeLists.txt (include(tests/locostart.cmake OPTIONAL)); expects UTPP_WARNINGS and the
# motion-derived data dir (outside git), same lookup as jumppick.cmake.
set(_utpp_ls_data "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data")
if(NOT EXISTS "${_utpp_ls_data}/utpp_motion_woman_knee_twistfix_slide.utppmm" AND UTPP_MOTION_DB_TEST_FILE)
    get_filename_component(_utpp_ls_data "${UTPP_MOTION_DB_TEST_FILE}" DIRECTORY)
endif()
if(EXISTS "${_utpp_ls_data}/utpp_motion_woman_knee_twistfix_slide.utppmm"
   AND EXISTS "${_utpp_ls_data}/utpp_motion_woman_knee_twistfix_slide.chooser.json")
    add_executable(utpp_mm_locostart_test tests/LocoStartTest.cpp src/mm/MotionAdapter.hpp src/mm/MotionTrajectory.hpp src/MotionDriver.hpp)
    target_compile_features(utpp_mm_locostart_test PRIVATE cxx_std_20)
    target_compile_options(utpp_mm_locostart_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_mm_locostart_test PRIVATE bcrypt)
    add_test(NAME utpp_mm_locostart_test COMMAND utpp_mm_locostart_test
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/loco_start_feeds_20261031_2255.txt"
             "${_utpp_ls_data}/utpp_motion_woman_knee_twistfix_slide"
             --jumps "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261021_1213.txt"
             --jumps "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261025_1341.txt")
    # Mutations (UTPP_LOCOSTART_MUTATION, src/mm/MotionTrajectory.hpp / MotionAdapter.hpp), each must print the FAIL line of its own check
    # (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error - exit 2, missing data - cannot count as a killed mutant):
    #   1 = IsPivoting without the velocity floor: a start from rest at a world heading >= 45 / 30 deg flags IsPivoting;
    #   2 = IsStarting with the fixed 100 cm/s margin: the slow crouch / analog walk start is no start (no *_Starts pick);
    #   3 = no take-off hold: the grounded Jump 18 feed searches the jump databases with TimeToLand 1.52 and the impulse feed re-picks;
    #   4 = the first take-off hold (mode only, the detail stays 18): a sprint (4) -> 18 take-off maps to the Run gait, the matcher interrupts
    #       and picks a ground clip (Sprint_to_Run) on the held feed, right before the jump pick.
    foreach(_m 1 2 3 4)
        add_executable(utpp_mm_locostart_mut${_m} tests/LocoStartTest.cpp src/mm/MotionAdapter.hpp src/mm/MotionTrajectory.hpp src/MotionDriver.hpp)
        target_compile_features(utpp_mm_locostart_mut${_m} PRIVATE cxx_std_20)
        target_compile_options(utpp_mm_locostart_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
        target_compile_definitions(utpp_mm_locostart_mut${_m} PRIVATE UTPP_LOCOSTART_MUTATION=${_m})
        target_link_libraries(utpp_mm_locostart_mut${_m} PRIVATE bcrypt)
        add_test(NAME utpp_mm_locostart_mut${_m} COMMAND utpp_mm_locostart_mut${_m}
                 "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/loco_start_feeds_20261031_2255.txt"
                 "${_utpp_ls_data}/utpp_motion_woman_knee_twistfix_slide"
                 --jumps "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261021_1213.txt"
                 --jumps "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261025_1341.txt")
    endforeach()
    set_tests_properties(utpp_mm_locostart_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL start [^\n]*: IsPivoting on [1-9]")
    set_tests_properties(utpp_mm_locostart_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL start replay crouch_start: first pick [^\n]* is a Crouch_Walk_Starts row")
    set_tests_properties(utpp_mm_locostart_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL jump jump_stand: [2-9] jump-database picks")
    set_tests_properties(utpp_mm_locostart_mut4 PROPERTIES PASS_REGULAR_EXPRESSION
                         "FAIL jump fixture [^\n]*20261025_1341[.]txt orient: held feeds [0-9]+ [(]sprint [1-9][0-9]*[)], jumped picks on held feeds [1-9]")
endif()
