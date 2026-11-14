# Focused ownership/admission checks; owned retail semantic fixture outside Git.
set(UTPP_EARLY_SEMANTIC_CSV "${CMAKE_CURRENT_SOURCE_DIR}/../../../Research/logs/game/semantic-gate-runtime-20261015/utpp-pose-semantics-4329588874878.csv" CACHE FILEPATH "Validated owned semantic census fixture")
add_executable(utpp_early_pose_runtime_test tests/EarlyPoseRuntimeTest.cpp src/EarlyPoseRuntime.cpp src/EarlyPoseRuntime.hpp)
target_compile_features(utpp_early_pose_runtime_test PRIVATE cxx_std_20)
target_compile_options(utpp_early_pose_runtime_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_early_pose_runtime_test COMMAND utpp_early_pose_runtime_test "${UTPP_EARLY_SEMANTIC_CSV}")
# One whole frame of each pose owner through the real hooks (main mapping job, deformer job, FK call, skin helper recompute) on the
# woman helper fixture (recorded frames outside Git, under Build/test-data; the test exists only where it does).
set(_utpp_early_frame_fixture "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/test-data/body_helper_woman")
if(EXISTS "${_utpp_early_frame_fixture}/frames.txt")
  add_executable(utpp_early_pose_frame_test tests/EarlyPoseFrameTest.cpp src/FkHook.cpp src/PoseHook.cpp src/EarlyPoseRuntime.cpp
    src/BodyHelperRuntime.cpp src/mm/AimOffset.cpp src/mm/MeleeAim.cpp src/mm/MeleeBase.cpp src/mm/MeleeBaseData.cpp src/mm/MeleePose.cpp
    src/mm/HeadLook.cpp)
  target_compile_features(utpp_early_pose_frame_test PRIVATE cxx_std_20)
  target_compile_options(utpp_early_pose_frame_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_early_pose_frame_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
  target_link_libraries(utpp_early_pose_frame_test PRIVATE utpp_red4ext_sdk bcrypt)
  add_test(NAME utpp_early_pose_frame_test COMMAND utpp_early_pose_frame_test "${_utpp_early_frame_fixture}")
endif()
