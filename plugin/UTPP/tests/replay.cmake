# ---- port report 7 stage 10.1 p.4: offline replay of recorded game feeds through the v2 stack (MotionAdapter draft).
# Included from plugin/UTPP/CMakeLists.txt (include(tests/replay.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Data (outside git): woman v2 .utppmm (+ .chooser.json) and Build/test-data/replay/*.feed from tools/replay/extract_feeds.py.
# Run by the worktree build script (no ctest entry: the feed list lives in the build script).
add_executable(utpp_motion_replay EXCLUDE_FROM_ALL tests/FeedReplayMotion.cpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp src/mm/Inertializer.hpp
               src/mm/MotionDatabase.hpp src/mm/Chooser.hpp src/mm/MMMath.hpp src/mm/MotionPose.hpp src/mm/Overlay.hpp
               src/mm/OverlayFile.hpp src/mm/GripIK.hpp src/mm/IdleBreaks.hpp)
target_compile_options(utpp_motion_replay PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_motion_replay PRIVATE cxx_std_20)

# Stage replay of a recorded game feed dump through the real driver and leg tail (tools/analysis/replay_pose_stages.cpp): the local pose
# of every database bone per feed, before and after the leg tail. A tool, no ctest entry; used by tools/analysis/pose_stage_pops.py.
add_executable(utpp_replay_pose_stages EXCLUDE_FROM_ALL ../../tools/analysis/replay_pose_stages.cpp src/MotionHost.cpp src/MotionDriver.cpp)
target_link_libraries(utpp_replay_pose_stages PRIVATE bcrypt)
target_compile_options(utpp_replay_pose_stages PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_replay_pose_stages PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
target_compile_features(utpp_replay_pose_stages PRIVATE cxx_std_20)
