# ---- port report 6.10: compile check of the DRAFT adapter src/mm/MotionAdapter.hpp (not included by the DLL).
# Included from plugin/UTPP/CMakeLists.txt (include(tests/motion_adapter.cmake OPTIONAL)); expects UTPP_WARNINGS. No data files.
add_executable(utpp_motion_driver_compile_test tests/MotionAdapterCompileTest.cpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp
               src/mm/Inertializer.hpp src/mm/MotionDatabase.hpp src/mm/Chooser.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_motion_driver_compile_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_motion_driver_compile_test PRIVATE cxx_std_20)
add_test(NAME utpp_motion_driver_compile_test COMMAND utpp_motion_driver_compile_test)
# J1 mutation (UTPP_TRAJV2_HEAVY_MUTATION=1: the heavy-landing threshold stays 700 at every gravity): the g 16 landing case must
# print FAIL.
add_executable(utpp_motion_driver_compile_heavy_mut1 tests/MotionAdapterCompileTest.cpp src/mm/MotionAdapter.hpp src/mm/MotionTrajectory.hpp)
target_compile_options(utpp_motion_driver_compile_heavy_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_motion_driver_compile_heavy_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_motion_driver_compile_heavy_mut1 PRIVATE UTPP_TRAJV2_HEAVY_MUTATION=1)
add_test(NAME utpp_motion_driver_compile_heavy_mut1 COMMAND utpp_motion_driver_compile_heavy_mut1)
set_tests_properties(utpp_motion_driver_compile_heavy_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# removed 2026-11-05 (review R13), which cheaper test kills the same site: utpp_mm_jump_pick_mut21

# Query-vs-bake parity on non-straight rows (start / pivot / strafe / stop, woman + man). Needs the motion-derived databases
# (outside git): run utpp_mm_query_parity_test --db <utpp_motion_woman.utppmm> --db <utpp_motion_man.utppmm>. Registered
# with ctest from the build scripts' -DUTPP_MOTION_DB_TEST_FILE / -DUTPP_MOTION_DB_MAN_TEST_FILE (same files as motion_pose.cmake), else
# from UTPP_DATA_DIR; without either it is not registered (without --db the executable fails). the worktree build script also
# runs it directly.
add_executable(utpp_mm_query_parity_test tests/QueryParityTest.cpp src/mm/MotionAdapter.hpp src/mm/MotionTrajectory.hpp src/mm/ChooserMatcher.hpp
               src/mm/MotionDatabase.hpp src/mm/Chooser.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_mm_query_parity_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_query_parity_test PRIVATE cxx_std_20)
# The databases here are the PLAIN motion bundles (Build/test-data/motion/utpp_motion_<body>_noslide): the shipped databases carry the
# slide-exit schema, whose history reads (-0.75 s) lie beyond the 30-sample trajectory history this test feeds (the check "query times
# inside the feed history" states it). Parity therefore runs on the fixtures for both bodies; the shipped databases only have to load
# and build their matcher (--load-only, which also prints their deepest read).
if(UTPP_MOTION_DB_TEST_FILE AND UTPP_MOTION_DB_MAN_TEST_FILE)
    add_test(NAME utpp_mm_query_parity_test COMMAND utpp_mm_query_parity_test
             --db "${UTPP_MOTION_FIXTURE_DIR}/utpp_motion_woman_noslide.utppmm" --db "${UTPP_MOTION_FIXTURE_DIR}/utpp_motion_man_noslide.utppmm"
             --load-only "${UTPP_MOTION_DB_TEST_FILE}" --load-only "${UTPP_MOTION_DB_MAN_TEST_FILE}")
elseif(DEFINED UTPP_DATA_DIR)
    add_test(NAME utpp_mm_query_parity_test COMMAND utpp_mm_query_parity_test
             --db "${UTPP_MOTION_FIXTURE_DIR}/utpp_motion_woman_noslide.utppmm" --db "${UTPP_MOTION_FIXTURE_DIR}/utpp_motion_man_noslide.utppmm"
             --load-only "${UTPP_DATA_DIR}/utpp_motion_woman.utppmm" --load-only "${UTPP_DATA_DIR}/utpp_motion_man.utppmm")
endif()
