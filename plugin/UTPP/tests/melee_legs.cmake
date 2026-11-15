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
endif()
