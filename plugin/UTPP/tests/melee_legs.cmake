# ---- melee full pose borders: the knee hinge of the leg solve (MeleePose.hpp SolveHingeLeg) ----
# Mutation of the synthetic crouch attack entry of tests/MeleePoseTest.cpp (TestKneeHingeSynthetic): UTPP_MELEE_LEGS_MUTATION=1 keeps the
# two-bone solve as it is wherever the Hips height limit folds a leg; the run must print the KH-1 FAIL line (PASS_REGULAR_EXPRESSION
# ignores the exit code, so a setup error cannot count as a killed mutant).
# Included from plugin/UTPP/CMakeLists.txt after the melee full pose block (include(tests/melee_legs.cmake OPTIONAL)).
if(TARGET utpp_melee_pose_test)
    add_executable(utpp_melee_pose_mut_knee_hinge tests/MeleePoseTest.cpp src/mm/MeleePose.hpp src/mm/MeleeBase.hpp src/mm/MeleeBaseData.hpp
                   src/mm/AimOffset.hpp src/mm/GameIntegration.hpp src/mm/MMMath.hpp)
    target_compile_definitions(utpp_melee_pose_mut_knee_hinge PRIVATE UTPP_MELEE_LEGS_MUTATION=1)
    target_compile_options(utpp_melee_pose_mut_knee_hinge PRIVATE ${UTPP_WARNINGS} /EHsc)
    add_test(NAME utpp_melee_pose_mut_knee_hinge COMMAND utpp_melee_pose_mut_knee_hinge
             "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/test-data/melee_pose")
    set_tests_properties(utpp_melee_pose_mut_knee_hinge PROPERTIES PASS_REGULAR_EXPRESSION "FAIL KH-1 crouch attack entry")
    # first-person girdle hold (MeleePoseGuard.hpp GirdleStep, tests/MeleePoseTest.cpp TestGirdleHold): UTPP_MELEE_EDGES_MUTATION=2 never
    # allows the hold; the run must print the GH-1 FAIL line (the recorded first-person frames reach the output).
    add_executable(utpp_melee_pose_mut_girdle_hold tests/MeleePoseTest.cpp src/mm/MeleePose.hpp src/mm/MeleePoseGuard.hpp src/mm/MeleeBase.hpp
                   src/mm/MeleeBaseData.hpp src/mm/AimOffset.hpp src/mm/GameIntegration.hpp src/mm/MMMath.hpp)
    target_compile_definitions(utpp_melee_pose_mut_girdle_hold PRIVATE UTPP_MELEE_EDGES_MUTATION=2)
    target_compile_options(utpp_melee_pose_mut_girdle_hold PRIVATE ${UTPP_WARNINGS} /EHsc)
    add_test(NAME utpp_melee_pose_mut_girdle_hold COMMAND utpp_melee_pose_mut_girdle_hold
             "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/test-data/melee_pose")
    set_tests_properties(utpp_melee_pose_mut_girdle_hold PROPERTIES PASS_REGULAR_EXPRESSION "FAIL GH-1 first-person girdle")
endif()

# ---- strike out of a sprint: the upper mask waits for the attack state (MeleeFullBody.hpp SprintHoldDefersMask) ----
# Mutation of tests/MeleeFullBodyTest.cpp TestSprintHold: UTPP_MELEE_EDGES_MUTATION=1 raises the mask on the hold state as before; the run
# must print the SH-1 FAIL line (recorded sprint attack feeds: the mask shows the game's sprint clip).
if(TARGET utpp_melee_fullbody_test)
    add_executable(utpp_melee_fullbody_mut_sprint_hold tests/MeleeFullBodyTest.cpp src/mm/MeleeFullBody.hpp src/mm/GameIntegration.hpp)
    target_compile_definitions(utpp_melee_fullbody_mut_sprint_hold PRIVATE UTPP_MELEE_EDGES_MUTATION=1)
    target_compile_options(utpp_melee_fullbody_mut_sprint_hold PRIVATE ${UTPP_WARNINGS} /EHsc)
    add_test(NAME utpp_melee_fullbody_mut_sprint_hold COMMAND utpp_melee_fullbody_mut_sprint_hold)
    set_tests_properties(utpp_melee_fullbody_mut_sprint_hold PROPERTIES PASS_REGULAR_EXPRESSION "FAIL SH-1 sprint hold")
    # sprint attack lunge (GameIntegration.hpp DecideUpperMask, UpperMaskInput::meleeLunge; TestSprintLunge): UTPP_MELEE_EDGES_MUTATION=3
    # lets over-speed close the melee mask as before; the run must print the SL-1 FAIL line.
    add_executable(utpp_melee_fullbody_mut_sprint_lunge tests/MeleeFullBodyTest.cpp src/mm/MeleeFullBody.hpp src/mm/GameIntegration.hpp)
    target_compile_definitions(utpp_melee_fullbody_mut_sprint_lunge PRIVATE UTPP_MELEE_EDGES_MUTATION=3)
    target_compile_options(utpp_melee_fullbody_mut_sprint_lunge PRIVATE ${UTPP_WARNINGS} /EHsc)
    add_test(NAME utpp_melee_fullbody_mut_sprint_lunge COMMAND utpp_melee_fullbody_mut_sprint_lunge)
    set_tests_properties(utpp_melee_fullbody_mut_sprint_lunge PROPERTIES PASS_REGULAR_EXPRESSION "FAIL SL-1 sprint attack lunge")
endif()
