# Focused ownership/admission checks; owned retail semantic fixture outside Git.
set(UTPP_EARLY_SEMANTIC_CSV "${CMAKE_CURRENT_SOURCE_DIR}/../../../Research/logs/game/semantic-gate-runtime-20261015/utpp-pose-semantics-4329588874878.csv" CACHE FILEPATH "Validated owned semantic census fixture")
add_executable(utpp_early_pose_runtime_test tests/EarlyPoseRuntimeTest.cpp src/EarlyPoseRuntime.cpp src/EarlyPoseRuntime.hpp)
target_compile_features(utpp_early_pose_runtime_test PRIVATE cxx_std_20)
target_compile_options(utpp_early_pose_runtime_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_early_pose_runtime_test COMMAND utpp_early_pose_runtime_test "${UTPP_EARLY_SEMANTIC_CSV}")
