# ---- forearm roll convention (src/mm/ForearmTwist.hpp): unit checks on the real woman skeleton + PoseEvaluator key 36.
# Included from plugin/UTPP/CMakeLists.txt after motion_pose.cmake (uses its _utpp_pmotion_* paths). Mutation builds MUST print a "FAIL " line:
# 1 = pass not called by PoseEvaluator (MotionPose.hpp), 2 = forearm children not compensated.
set(_utpp_fatw_args "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt")
add_executable(utpp_forearm_twist_test tests/ForearmTwistTest.cpp src/mm/ForearmTwist.hpp src/mm/MotionPose.hpp)
target_compile_options(utpp_forearm_twist_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_forearm_twist_test PRIVATE cxx_std_20)
add_test(NAME utpp_forearm_twist_test COMMAND utpp_forearm_twist_test ${_utpp_fatw_args})
foreach(_m 1 2)
    add_executable(utpp_forearm_twist_mut${_m} tests/ForearmTwistTest.cpp src/mm/ForearmTwist.hpp src/mm/MotionPose.hpp)
    target_compile_options(utpp_forearm_twist_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_forearm_twist_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_forearm_twist_mut${_m} PRIVATE UTPP_FOREARMTWIST_MUTATION=${_m})
    add_test(NAME utpp_forearm_twist_mut${_m} COMMAND utpp_forearm_twist_mut${_m} ${_utpp_fatw_args})
    set_tests_properties(utpp_forearm_twist_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()
