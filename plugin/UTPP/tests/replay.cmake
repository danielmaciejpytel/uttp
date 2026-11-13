# ---- port report 7 stage 10.1 p.4: offline replay of recorded game feeds through the v2 stack (MotionAdapter draft).
# Included from plugin/UTPP/CMakeLists.txt (include(tests/replay.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Data (outside git): woman v2 .utppmm (+ .chooser.json) and Build/test-data/replay/*.feed from tools/replay/extract_feeds.py.
# Run by the worktree build script (no ctest entry: the feed list lives in the build script).
add_executable(utpp_motion_replay EXCLUDE_FROM_ALL tests/FeedReplayMotion.cpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp src/mm/Inertializer.hpp
               src/mm/MotionDatabase.hpp src/mm/Chooser.hpp src/mm/MMMath.hpp src/mm/MotionPose.hpp src/mm/Overlay.hpp
               src/mm/OverlayFile.hpp src/mm/GripIK.hpp src/mm/IdleBreaks.hpp)
target_compile_options(utpp_motion_replay PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_motion_replay PRIVATE cxx_std_20)
