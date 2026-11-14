# ---- rifle-grip-stock 2026-11-10: support hand on the handguard + stock in the shoulder of the long guns (src/mm/SupportGrip.hpp, MotionPose.hpp),
# real data of both bodies. Included from plugin/UTPP/CMakeLists.txt after motion_pose.cmake (uses its _utpp_pmotion_* paths).
# Mutation build MUST print a "FAIL " line: 1 = support grip / stock pass compiled out of PoseEvaluator (the build before).
set(_utpp_sgrip_args "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt"
                     "${_utpp_pmotion_mdb}" "${_utpp_pmotion_mov}" "${_utpp_pmotion_movdir}/utpp_style_weights_man.txt")
add_executable(utpp_supportgrip_test tests/SupportGripTest.cpp src/mm/SupportGrip.hpp src/mm/MotionPose.hpp)
target_compile_options(utpp_supportgrip_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_supportgrip_test PRIVATE cxx_std_20)
add_test(NAME utpp_supportgrip_test COMMAND utpp_supportgrip_test ${_utpp_sgrip_args})
add_executable(utpp_supportgrip_mut1 tests/SupportGripTest.cpp src/mm/SupportGrip.hpp src/mm/MotionPose.hpp)
target_compile_options(utpp_supportgrip_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_supportgrip_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_supportgrip_mut1 PRIVATE UTPP_SUPPORTGRIP_MUTATION=1)
add_test(NAME utpp_supportgrip_mut1 COMMAND utpp_supportgrip_mut1 ${_utpp_sgrip_args})
set_tests_properties(utpp_supportgrip_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# Mutation 2 = the wrist-bend term of the support-grip weight compiled out (the build before); must print a "FAIL " line.
add_executable(utpp_supportgrip_mut2 tests/SupportGripTest.cpp src/mm/SupportGrip.hpp src/mm/MotionPose.hpp)
target_compile_options(utpp_supportgrip_mut2 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_supportgrip_mut2 PRIVATE cxx_std_20)
target_compile_definitions(utpp_supportgrip_mut2 PRIVATE UTPP_SUPPORTGRIP_MUTATION=2)
add_test(NAME utpp_supportgrip_mut2 COMMAND utpp_supportgrip_mut2 ${_utpp_sgrip_args})
set_tests_properties(utpp_supportgrip_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
