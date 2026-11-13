# ---- Foot height keeping of the live crossfade (BlendStack::KeepFootHeight; recording 2026-11-01, V's feet sink to the ankles for ~0.5 s while rising
# from crouch to stand). The real diag_b game feed through the production base stack (tests/LocoStanceTest.cpp, FS-* checks): the base and composed
# ankle heights across the three crouch -> stand edges, the knee bend direction, bit identity on single-layer frames.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/footsink.cmake OPTIONAL)) after locostance.cmake, whose data lookup and test arguments it
# reuses. Mutations (UTPP_FOOTSINK_MUTATION=N in src/mm/MotionBlendStack.hpp, listed below) must each print their own FAIL line
# (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error cannot count as a killed mutant).
if(DEFINED _utpp_ls_args)
    # 1 = no height preservation; 2 = the gate follows the stance permission at once (drops 1 -> 0 when the stance layer completes under a newer
    # gait fade); 3 = a collapsed tail keeps the stance flag of the group's base and grants the full gate; 4 = the matcher never flags a ground
    # locomotion fade (walk -> stop / crouched gait sink stay); 5 = the matcher flags every ground-mode fade (slide / jump / air / traversal
    # included); 6 = no pelvis drop (the stand -> crouch and crouch stop floats stay); 7 = no lowering through the legs
    # 8 = nothing vetoes a flagged layer (a take-off / slide entry pick lands inside a flagged ground fade and the lift goes on under it);
    # 9 = a replacing pick takes its flags from the matcher alone (a run pick replacing a landing head over the air layer is flagged);
    # 10 = a replacing pick inherits the stance flag without the ground condition (an air pick replacing a stance pick keeps the lowering).
    # (The bound of the pelvis drop - the Hips fold of the legs' own timing - is never reached on the recorded feed or on a uniform blend: the
    # floats stay under the room it leaves. FS-16 checks that the Hips never end below it; an unbounded drop has no killable mutant here.)
    set(_utpp_fs_expect_1 "FAIL FS-1 crouch -> stand at [0-9.]+ [(]unarmed[)]: the base ankles stay on the blend")
    set(_utpp_fs_expect_2 "FAIL FS-8 stance fade ending under the newer 80.172 gait fade")
    set(_utpp_fs_expect_3 "FAIL FS-10 an old stance layer collapsed into the tail")
    set(_utpp_fs_expect_4 "FAIL FS-12 A run -> stop edge 95.951 [(]armed[)]: the base ankles stay on the blend")
    set(_utpp_fs_expect_5 "FAIL FS-7 ")
    set(_utpp_fs_expect_6 "FAIL FS-13 C ")
    set(_utpp_fs_expect_7 "FAIL FS-13 C ")
    set(_utpp_fs_expect_8 "FAIL FS-17 take-off pick [(]unflagged[)] inside a flagged ground fade")
    set(_utpp_fs_expect_9 "FAIL FS-19 run pick replacing a landing head")
    set(_utpp_fs_expect_10 "FAIL FS-20 air pick replacing a stance pick")
    # 11 = no ankle floor (BlendStack::KeepAnklesAboveLayers, loco-fix 2026-11-09): the sprint -> slide entry sink stays
    set(_utpp_fs_expect_11 "FAIL FS-23 A ")
    foreach(_utpp_fs_mut 1 2 3 4 5 6 7 8 9 10 11)
        add_executable(utpp_mm_footsink_mut${_utpp_fs_mut} tests/LocoStanceTest.cpp src/mm/Overlay.hpp src/mm/MotionPose.hpp src/mm/ChooserMatcher.hpp)
        target_compile_features(utpp_mm_footsink_mut${_utpp_fs_mut} PRIVATE cxx_std_20)
        target_compile_options(utpp_mm_footsink_mut${_utpp_fs_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
        target_compile_definitions(utpp_mm_footsink_mut${_utpp_fs_mut} PRIVATE UTPP_FOOTSINK_MUTATION=${_utpp_fs_mut})
        target_link_libraries(utpp_mm_footsink_mut${_utpp_fs_mut} PRIVATE bcrypt)
        add_test(NAME utpp_mm_footsink_mut${_utpp_fs_mut} COMMAND utpp_mm_footsink_mut${_utpp_fs_mut} ${_utpp_ls_args})
        set_tests_properties(utpp_mm_footsink_mut${_utpp_fs_mut} PROPERTIES PASS_REGULAR_EXPRESSION "${_utpp_fs_expect_${_utpp_fs_mut}}")
    endforeach()
endif()
# FootLocker blend-out sink bound (FootLocker::kBlendOutSinkMax, loco-fix 2026-11-09): UTPP_FLSINK_MUTATION=1 removes it, the mixed 40 s run
# of tests/MMFootLockTest.cpp (check d2) must then report an ankle more than 3 cm under the animated one.
if(UTPP_MMDB_TEST_FILE)
    add_executable(utpp_mm_footlock_flsink_mut1 tests/MMFootLockTest.cpp src/BaseDatabase.hpp src/mm/MMMath.hpp src/mm/BaseTrajectory.hpp
                   src/mm/Inertializer.hpp src/mm/BaseMatcher.hpp src/mm/FootLocker.hpp)
    target_compile_options(utpp_mm_footlock_flsink_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_mm_footlock_flsink_mut1 PRIVATE UTPP_FLSINK_MUTATION=1)
    add_test(NAME utpp_mm_footlock_flsink_mut1 COMMAND utpp_mm_footlock_flsink_mut1 "${UTPP_MMDB_TEST_FILE}")
    set_tests_properties(utpp_mm_footlock_flsink_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "\\[FAIL\\] [(]d2[)] mixed 40 s: the blend-out keeps the ankle")
endif()
# FootLocker standing-stop swing-contact release (FootLocker::StopSwingContact, loco-stop-knees 2026-11-09): UTPP_FLSTOP_MUTATION=1 removes
# it, the walk stop with a false swing contact of tests/MMFootLockTest.cpp (check s1) must then report the swing foot pulled off its spacing.
if(UTPP_MMDB_TEST_FILE)
    add_executable(utpp_mm_footlock_flstop_mut1 tests/MMFootLockTest.cpp src/BaseDatabase.hpp src/mm/MMMath.hpp src/mm/BaseTrajectory.hpp
                   src/mm/Inertializer.hpp src/mm/BaseMatcher.hpp src/mm/FootLocker.hpp)
    target_compile_options(utpp_mm_footlock_flstop_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_mm_footlock_flstop_mut1 PRIVATE UTPP_FLSTOP_MUTATION=1)
    add_test(NAME utpp_mm_footlock_flstop_mut1 COMMAND utpp_mm_footlock_flstop_mut1 "${UTPP_MMDB_TEST_FILE}")
    set_tests_properties(utpp_mm_footlock_flstop_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "\\[FAIL\\] [(]s1[)] walk stop, false swing contact: the foot spacing")
endif()
