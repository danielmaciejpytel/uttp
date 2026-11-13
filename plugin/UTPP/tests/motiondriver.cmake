# ---- port: MotionDriver unit test in isolation (src/MotionDriver.cpp without MotionHost.cpp / RED4ext; report 6.10).
# Included from plugin/UTPP/CMakeLists.txt (include(tests/motiondriver.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Argument: the data dir (<root>/Data, motion-derived, outside git); main checkout or worktree layout like motion_pose.cmake. The dir
# is derived from -DUTPP_MOTION_DB_TEST_FILE when the build script passes it. A missing data file makes the test FAIL.
set(_utpp_mdmotion_data "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data")
if(UTPP_MOTION_DB_TEST_FILE)
    get_filename_component(_utpp_mdmotion_data "${UTPP_MOTION_DB_TEST_FILE}" DIRECTORY)
endif()
file(TO_NATIVE_PATH "${_utpp_mdmotion_data}" _utpp_mdmotion_data_native)
add_executable(utpp_motion_driver_unit_test tests/MotionDriverTest.cpp src/MotionDriver.cpp src/MotionDriver.hpp src/MotionHost.hpp
               src/mm/MotionAdapter.hpp src/mm/MotionPose.hpp src/mm/ChooserMatcher.hpp src/mm/MotionDatabase.hpp src/mm/Chooser.hpp
               src/mm/OverlayFile.hpp src/mm/TraversalFile.hpp src/mm/GameIntegration.hpp)
target_compile_options(utpp_motion_driver_unit_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_motion_driver_unit_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UTPP_MOTION_DRIVER_TEST_FAULTS) # test-only throw (fault path)
target_compile_features(utpp_motion_driver_unit_test PRIVATE cxx_std_20)
add_test(NAME utpp_motion_driver_unit_test COMMAND utpp_motion_driver_unit_test "${_utpp_mdmotion_data_native}")
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/motiondriverinteg.cmake OPTIONAL) # UTPP-MOTION SEAM: MotionDriver integration test (real MMDriver)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/motiondriverair.cmake OPTIONAL) # air: v2 air consumer test (real MMDriver)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/fix2hip.cmake OPTIONAL) # fix2/hip: slide hipfire recoil + weapon-pose legs in the air
