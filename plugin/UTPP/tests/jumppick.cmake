# ---- fix batch 3 (jump): offline replay of the jump pick / quick slide combinations (P6 K2, N3, N9). Included from plugin/UTPP/CMakeLists.txt
# (include(tests/jumppick.cmake OPTIONAL)); expects UTPP_WARNINGS and the motion-derived data dir (outside git, like the slide exit replay).
# Real feed windows of the 2026-10-21 12:13 session (tests/data/jump_feeds_20261021_1213.txt) through MapToChooser + ChooserMatcher + BlendStack on
# the installed slide bundle.
set(_utpp_jp_data "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data")
if(NOT EXISTS "${_utpp_jp_data}/utpp_motion_woman_knee_twistfix_slide.utppmm" AND UTPP_MOTION_DB_TEST_FILE)
    get_filename_component(_utpp_jp_data "${UTPP_MOTION_DB_TEST_FILE}" DIRECTORY) # any other worktree: the build script's data dir
endif()
if(EXISTS "${_utpp_jp_data}/utpp_motion_woman_knee_twistfix_slide.utppmm"
   AND EXISTS "${_utpp_jp_data}/utpp_motion_woman_knee_twistfix_slide.chooser.json")
    add_executable(utpp_mm_jump_pick_test tests/JumpPickTest.cpp src/mm/MotionBlendStack.hpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp)
    target_compile_features(utpp_mm_jump_pick_test PRIVATE cxx_std_20)
    target_compile_options(utpp_mm_jump_pick_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_mm_jump_pick_test PRIVATE bcrypt)
    add_test(NAME utpp_mm_jump_pick_test COMMAND utpp_mm_jump_pick_test
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261021_1213.txt"
             "${_utpp_jp_data}/utpp_motion_woman_knee_twistfix_slide"
             --running "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261025_1341.txt"
             --flicker "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/air_ground_flicker_feeds.txt")
    # TR-dll male parity: the same fixtures (the chooser inputs are body-independent) on the man slide bundle, --rig man = the man skeleton identity.
    if(EXISTS "${_utpp_jp_data}/utpp_motion_man_knee_slide.utppmm" AND EXISTS "${_utpp_jp_data}/utpp_motion_man_knee_slide.chooser.json")
        add_test(NAME utpp_mm_jump_pick_man_test COMMAND utpp_mm_jump_pick_test
                 "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261021_1213.txt"
                 "${_utpp_jp_data}/utpp_motion_man_knee_slide"
                 --rig man --running "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261025_1341.txt"
                 --flicker "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/air_ground_flicker_feeds.txt")
        # With --rig man one limit is looser (JumpPickTest.cpp): the JO1 control >= 8 frames (measured 9; the man bundle does not reproduce the
        # woman log). The slide family switch rotation has the same bound on both bodies (<= 70 deg per 50 ms). The man jump data is still not
        # retargeted with the twistfix clips.
        # UTPP_FOOTSINK_MUTATION=12 (src/mm/MotionBlendStack.hpp): the leg bones of a FootOut <-> KneesOut fade keep their shorter fade; the man
        # bundle must then break the family switch bound (the woman bundle stays under it, so this mutant runs on the man bundle).
        add_executable(utpp_mm_jump_pick_man_footsink_mut12 tests/JumpPickTest.cpp src/mm/MotionBlendStack.hpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp)
        target_compile_features(utpp_mm_jump_pick_man_footsink_mut12 PRIVATE cxx_std_20)
        target_compile_options(utpp_mm_jump_pick_man_footsink_mut12 PRIVATE ${UTPP_WARNINGS} /EHsc)
        target_compile_definitions(utpp_mm_jump_pick_man_footsink_mut12 PRIVATE UTPP_FOOTSINK_MUTATION=12)
        target_link_libraries(utpp_mm_jump_pick_man_footsink_mut12 PRIVATE bcrypt)
        add_test(NAME utpp_mm_jump_pick_man_footsink_mut12 COMMAND utpp_mm_jump_pick_man_footsink_mut12
                 "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261021_1213.txt"
                 "${_utpp_jp_data}/utpp_motion_man_knee_slide"
                 --rig man --running "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261025_1341.txt")
        set_tests_properties(utpp_mm_jump_pick_man_footsink_mut12 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL slide family switch: worst rotation per 50 ms")
    endif()
    # Mutation builds MUST report an assertion failure: they pass only when the output contains a line starting "FAIL "
    # (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error - exit 2, missing data - cannot count as a killed mutant).
    # UTPP_JUMPPICK_MUTATION (src/mm/ChooserMatcher.hpp): 1 B / LL / RL Start rows stay selectable in a standing jump, 2 no take-off
    # row block before touchdown, 3 the standing-jump rules also act on jumps in motion, 4 no 0.3 s fade into the air after a slide, 5 the
    # turn-in-place out of the rising slide exit clip keeps the 0.1 s fade, 6 the FootOut <-> KneesOut switch keeps the 0.2 s fade, 7 the run-off
    # clips (F_Off_*) stay selectable on the take-off update of a standing jump, 8 the standing-jump decision is taken again on every update (a
    # steering in the air switches the rules off), 9 the long turn-in-place fade is gated by the age of the exit clip (hold + 0.2 s), not by its rise,
    # 10 the decision of the first InAir update also holds over a second take-off in the air (double jump / air dash), 11 Matcher::Reset keeps the
    # decision of the flight, 12 (fix5/jump, --running fixture 2026-10-25 13:41) a jump in motion may continue the F Off clip picked on its
    # take-off update through the rise (only new F Off picks are blocked), 13 every flight counts as a game jump (walk-offs off a ramp lose the
    # F Off rows too), 14 the F Off block of a jump in motion acts only on its first airborne update, not through the rise.
    # 15 (fix/jumpout-cam, synthetic jump-out flights JO1 / JO2 of the 2026-11-03 16:28 log) the B / LL / RL Start rows stay selectable in a jump-out flight,
    # 16 no fall-loop hand-over from the apex of a jump-out flight (the Start clip is held / continued through the descent), 17 every flight counts as a
    # jump-out flight (the rules leak into jumps and walk-offs: the flag-off control of the synthetic flights no longer reproduces the log),
    # 18 no contact hold in the chooser mapping (MotionAdapter.hpp MapConfig::contactHoldS: the two-feed ground contact of jump-out B reaches the chooser as a landing).
    # 19 (review r2 finding 15) a re-jump inside a flight rebuilds the jump-out latch from the window flag (the window closed mid-flight, the rest of the flight
    # loses jumpOutNoBack), 20 (review r2 finding 16) the jump-out flag stays on after the jump-out touchdown for the rest of the exit window (a walk-off or jump in
    # the same window is taken for a jump-out flight).
    # 21 (r3 landing) no recovery cap for the heavy land of a jump-out flight (the heavy land clip plays on 0.65 s after the touchdown).
    # 22 (r5 air) the production matcher configuration (motiondriver::ProductionMatcherConfig) has the jump-out fall-loop hand-over off again (the jump-start clip is held through the flight).
    # 23 (r5 air) only databases that index a fall clip themselves drop their Start / Off rows from the apex of a jump-out flight (the Far database keeps its Start clip over open ground).
    # Run-time mutants (src/MutationSelect.hpp): every mutation site is a plain run-time branch, so ONE executable reads the mutation number from
    # UTPP_MUTATION_JUMPPICK and ctest runs it once per mutation (UTPP_MUTATION_JUMPPICK unset = the base test).
    add_executable(utpp_mm_jump_pick_rt tests/JumpPickTest.cpp src/mm/MotionBlendStack.hpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp src/MutationSelect.hpp)
    target_compile_features(utpp_mm_jump_pick_rt PRIVATE cxx_std_20)
    target_compile_options(utpp_mm_jump_pick_rt PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_mm_jump_pick_rt PRIVATE UTPP_RUNTIME_MUTATION_JUMPPICK)
    target_link_libraries(utpp_mm_jump_pick_rt PRIVATE bcrypt)
    foreach(_m 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23)
        add_test(NAME utpp_mm_jump_pick_mut${_m} COMMAND utpp_mm_jump_pick_rt
                 "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261021_1213.txt"
                 "${_utpp_jp_data}/utpp_motion_woman_knee_twistfix_slide"
                 --running "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261025_1341.txt")
        set_tests_properties(utpp_mm_jump_pick_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL " ENVIRONMENT "UTPP_MUTATION_JUMPPICK=${_m}")
    endforeach()
    # 40 (src/mm/MotionAdapter.hpp) no step-down reach of the ground-drop grace (MapConfig::groundDropStepM): the 22 / 24 cm run-down drops of the
    # --flicker fixture (run_down_c / run_down_d) turn InAir and pick the jump / land clips again.
    add_test(NAME utpp_mm_jump_pick_mut40 COMMAND utpp_mm_jump_pick_rt
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261021_1213.txt"
             "${_utpp_jp_data}/utpp_motion_woman_knee_twistfix_slide"
             --running "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/jump_feeds_20261025_1341.txt"
             --flicker "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/air_ground_flicker_feeds.txt")
    set_tests_properties(utpp_mm_jump_pick_mut40 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL run_down_c: running down the step keeps the ground clips"
                         ENVIRONMENT "UTPP_MUTATION_JUMPPICK=40")
endif()
