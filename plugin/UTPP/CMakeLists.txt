cmake_minimum_required(VERSION 3.21)
project(UTPP VERSION 0.1.1 LANGUAGES CXX RC)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# RED4ext.SDK is consumed READ-ONLY in header-only mode. add_subdirectory() is deliberately not used:
# the SDK's CMakeLists runs configure_file() into its own include directory.
set(UTPP_RED4EXT_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Tools/repos/RED4ext.SDK"
    CACHE PATH "Path to the RED4ext.SDK checkout (read-only)")
if(NOT EXISTS "${UTPP_RED4EXT_SDK_DIR}/include/RED4ext/RED4ext.hpp")
  message(FATAL_ERROR "RED4ext.SDK not found at ${UTPP_RED4EXT_SDK_DIR}")
endif()

add_library(utpp_red4ext_sdk INTERFACE)
target_include_directories(utpp_red4ext_sdk SYSTEM INTERFACE
  "${UTPP_RED4EXT_SDK_DIR}/include"
  "${UTPP_RED4EXT_SDK_DIR}/vendor/D3D12MemAlloc")
target_compile_definitions(utpp_red4ext_sdk INTERFACE RED4EXT_HEADER_ONLY)

set(UTPP_WARNINGS /W4 /permissive- /utf-8)

# ---- Plugin ----
add_library(UTPP SHARED
  src/Main.cpp
  src/FkHook.cpp
  src/Arming.cpp
  src/MotionHost.cpp
  src/MotionHost.hpp
  src/MotionDriver.cpp # UTPP-MOTION SEAM
  src/MotionDriverNatives.cpp # UTPP-MOTION SEAM
  src/MotionDriver.hpp # UTPP-MOTION SEAM
  src/BaseDatabase.hpp
  src/mm/GameIntegration.hpp
  src/mm/NativeReloadClavicleReach.hpp
  src/mm/BaseMatcher.hpp
  src/mm/FootLocker.hpp
  src/mm/BodyTurn.hpp
  src/mm/ChestAlign.hpp
  src/mm/SpeedCalibration.hpp
  src/mm/SpeedBands.hpp
  src/FkHook.hpp
  src/Arming.hpp
  src/State.hpp
  src/QuatMath.hpp)
# Version resource + header from the one project version above (FileVersion / ProductVersion in UTPP.dll, status line, RED4ext version).
configure_file(src/version.rc.in "${CMAKE_CURRENT_BINARY_DIR}/utpp_version.rc" @ONLY)
configure_file(src/utpp_version.h.in "${CMAKE_CURRENT_BINARY_DIR}/utpp_version.h" @ONLY)
target_sources(UTPP PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/utpp_version.rc")
target_include_directories(UTPP PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(UTPP PRIVATE utpp_red4ext_sdk version bcrypt Cabinet) # bcrypt: SHA-256 of databases (0.5.1); Cabinet: block compression of the compact pose capture
target_compile_definitions(UTPP PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
# Compiler flags for C++ only (the resource compiler does not take them).
# The test-only hooks of MotionHost.cpp (MatcherPoseForTest, AirOwnsForTest, FeedDumpFlushForTest) are compiled out of the DLL only.
target_compile_definitions(UTPP PRIVATE UTPP_PLUGIN_DLL)
foreach(_utpp_flag ${UTPP_WARNINGS} /EHsc)
  target_compile_options(UTPP PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${_utpp_flag}>)
endforeach()
# Keep linker PDBs; embedded compiler debug info avoids the local C1902 PDB-manager failure.
option(UTPP_EMBEDDED_DEBUG_INFO "Use /Z7 instead of compiler PDB /Zi" OFF)
if(UTPP_EMBEDDED_DEBUG_INFO)
  target_compile_options(UTPP PRIVATE $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>>:/Z7>)
else()
  target_compile_options(UTPP PRIVATE $<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:CXX>>:/Zi>)
endif()
target_link_options(UTPP PRIVATE /INCREMENTAL:NO $<$<CONFIG:Release>:/DEBUG /OPT:REF /OPT:ICF>)
# Release hygiene: the PDB path embedded in the DLL debug directory is only the file name (no local build path), for every
# configuration (RelWithDebInfo as well as Release).
target_link_options(UTPP PRIVATE /PDBALTPATH:%_PDB%)
set_target_properties(UTPP PROPERTIES OUTPUT_NAME "UTPP")

# ---- Offline quaternion self-test (not shipped) ----
add_executable(utpp_quat_selftest tests/QuatSelfTest.cpp)
target_compile_options(utpp_quat_selftest PRIVATE ${UTPP_WARNINGS})
enable_testing()
add_executable(utpp_native_reload_guide_turn_test tests/NativeReloadGuideTurnTest.cpp)
target_include_directories(utpp_native_reload_guide_turn_test PRIVATE src)
target_compile_options(utpp_native_reload_guide_turn_test PRIVATE ${UTPP_WARNINGS} /EHsc)
# The overlay data sits in <root>/Data: three levels up from Source/plugin/UTPP, four from a worktree below Build/wt-*.
set(_utpp_gt_wov "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/motion/utpp_overlays_woman.utppov")
if(NOT EXISTS "${_utpp_gt_wov}")
  set(_utpp_gt_wov "${CMAKE_CURRENT_SOURCE_DIR}/../../../../Data/motion/utpp_overlays_woman.utppov")
endif()
add_test(NAME utpp_native_reload_guide_turn_test COMMAND utpp_native_reload_guide_turn_test
  "${_utpp_gt_wov}")
add_executable(utpp_pose_trace_test tests/PoseTraceTest.cpp)
target_compile_options(utpp_pose_trace_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_pose_trace_test COMMAND utpp_pose_trace_test "${CMAKE_CURRENT_BINARY_DIR}/pose-trace-test")
add_test(NAME utpp_quat_selftest COMMAND utpp_quat_selftest)

# ---- Offline motion matching database reader/search self-test (not shipped) ----
set(UTPP_MMDB_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_base_dense_subset.utppmm"
    CACHE FILEPATH "Baked .utppmm used by utpp_mmdb_selftest")
add_executable(utpp_mmdb_selftest tests/MMDatabaseSelfTest.cpp src/BaseDatabase.hpp)
target_compile_options(utpp_mmdb_selftest PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mmdb_selftest COMMAND utpp_mmdb_selftest "${UTPP_MMDB_TEST_FILE}")

# ---- Offline motion matching runtime simulation test (Phase 4a, not shipped) ----
add_executable(utpp_mm_sim_test tests/MMSimTest.cpp src/BaseDatabase.hpp
  src/mm/MMMath.hpp src/mm/BaseTrajectory.hpp src/mm/Inertializer.hpp src/mm/BaseMatcher.hpp)
target_compile_options(utpp_mm_sim_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_sim_test COMMAND utpp_mm_sim_test "${UTPP_MMDB_TEST_FILE}")

# ---- Offline Phase 4.5 foot lock / leg IK test (not shipped) ----
add_executable(utpp_mm_footlock_test tests/MMFootLockTest.cpp src/BaseDatabase.hpp
  src/mm/MMMath.hpp src/mm/BaseTrajectory.hpp src/mm/Inertializer.hpp src/mm/BaseMatcher.hpp src/mm/FootLocker.hpp)
target_compile_options(utpp_mm_footlock_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_footlock_test COMMAND utpp_mm_footlock_test "${UTPP_MMDB_TEST_FILE}")

add_executable(utpp_standing_stop_support_test tests/StandingStopSupportTest.cpp src/BaseDatabase.hpp
  src/mm/FootLocker.hpp)
target_compile_features(utpp_standing_stop_support_test PRIVATE cxx_std_20)
target_compile_options(utpp_standing_stop_support_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_standing_stop_support_test COMMAND utpp_standing_stop_support_test "${UTPP_MMDB_TEST_FILE}")

add_executable(utpp_locomotion_signal_blend_test tests/LocomotionSignalBlendTest.cpp)
target_compile_options(utpp_locomotion_signal_blend_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_locomotion_signal_blend_test COMMAND utpp_locomotion_signal_blend_test)

# ---- Offline Phase 5.1 body-yaw-from-animation test (180 deg reversal fixed-time turn vs UTPP command; not shipped) ----
add_executable(utpp_mm_bodyturn_test tests/MMBodyTurnTest.cpp src/BaseDatabase.hpp
  src/mm/MMMath.hpp src/mm/BaseTrajectory.hpp src/mm/Inertializer.hpp src/mm/BaseMatcher.hpp src/mm/BodyTurn.hpp)
target_compile_options(utpp_mm_bodyturn_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_bodyturn_test COMMAND utpp_mm_bodyturn_test "${UTPP_MMDB_TEST_FILE}")

# ---- Offline Phase 5.3 offset root + acceleration matching test (start from idle; not shipped) ----
add_executable(utpp_mm_offsetroot_test tests/MMOffsetRootTest.cpp src/BaseDatabase.hpp src/mm/MMMath.hpp
  src/mm/BaseTrajectory.hpp src/mm/Inertializer.hpp src/mm/BaseMatcher.hpp src/mm/FootLocker.hpp src/mm/OffsetRoot.hpp
  src/mm/PlaybackRate.hpp src/mm/BodyTurn.hpp src/mm/Steering.hpp src/mm/StanceGuard.hpp)
target_compile_options(utpp_mm_offsetroot_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_offsetroot_test COMMAND utpp_mm_offsetroot_test "${UTPP_MMDB_TEST_FILE}")

# ---- Offline Phase 5.3c steering stability test (circle / slalom / slow walk / regressions; not shipped) ----
add_executable(utpp_mm_steering_test tests/MMSteeringTest.cpp src/BaseDatabase.hpp src/mm/MMMath.hpp
  src/mm/BaseTrajectory.hpp src/mm/Inertializer.hpp src/mm/BaseMatcher.hpp src/mm/FootLocker.hpp src/mm/OffsetRoot.hpp
  src/mm/PlaybackRate.hpp src/mm/BodyTurn.hpp src/mm/Steering.hpp src/mm/StanceGuard.hpp)
target_compile_options(utpp_mm_steering_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_steering_test COMMAND utpp_mm_steering_test "${UTPP_MMDB_TEST_FILE}")

# ---- Offline B2 strafe S1 core test (StrafeMode / StrafeDesiredMotion / ComputeStrafeProfiles; not shipped) ----
set(UTPP_MMDB_PACK_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_base.utppmm"
    CACHE FILEPATH "Package motion set .utppmm used by the offline strafe / chest-align / stance / loco / vehexit / provided-root-frame tests (override for isolated worktrees)")
add_executable(utpp_mm_strafe_test tests/MMStrafeTest.cpp src/BaseDatabase.hpp src/mm/MMMath.hpp src/mm/BodyTurn.hpp
  src/mm/Steering.hpp src/mm/Strafe.hpp src/mm/BaseMatcher.hpp)
target_include_directories(utpp_mm_strafe_test PRIVATE src)
target_compile_options(utpp_mm_strafe_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_strafe_test COMMAND utpp_mm_strafe_test "${UTPP_MMDB_PACK_TEST_FILE}"
  "${UTPP_MMDB_TEST_FILE}")

# ---- Offline B2 strafe S2 chest align test (mm/ChestAlign.hpp; v6 clip names; not shipped) ----
add_executable(utpp_mm_chestalign_test tests/MMChestAlignTest.cpp src/BaseDatabase.hpp src/mm/MMMath.hpp src/mm/ChestAlign.hpp)
target_include_directories(utpp_mm_chestalign_test PRIVATE src)
target_compile_options(utpp_mm_chestalign_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_chestalign_test COMMAND utpp_mm_chestalign_test "${UTPP_MMDB_PACK_TEST_FILE}")

# ---- Offline 0.5.1 runtime database switching test (real MMDriver, no game; not shipped) ----
set(UTPP_MMDB_ALT_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_base_dense_subset_posture.utppmm"
    CACHE FILEPATH "Second baked .utppmm (same rig) used by utpp_mm_dbswitch_test")
add_executable(utpp_mm_dbswitch_test tests/MMDbSwitchTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/BaseDatabase.hpp
  src/State.hpp)
target_link_libraries(utpp_mm_dbswitch_test PRIVATE bcrypt)
target_compile_definitions(utpp_mm_dbswitch_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_compile_options(utpp_mm_dbswitch_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_dbswitch_test COMMAND utpp_mm_dbswitch_test "${UTPP_MMDB_TEST_FILE}" "${UTPP_MMDB_ALT_TEST_FILE}")

# ---- Offline 0.5.2 per-database speed calibration test (not shipped) ----
set(UTPP_MMDB_FULL_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_base_dense_full_posture.utppmm"
    CACHE FILEPATH "Full motion set .utppmm (calibration must equal the 0.5.1 hard-coded values)")
add_executable(utpp_mm_speedcal_test tests/MMSpeedCalibrationTest.cpp src/mm/SpeedCalibration.hpp src/BaseDatabase.hpp)
target_compile_options(utpp_mm_speedcal_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_speedcal_test COMMAND utpp_mm_speedcal_test "${UTPP_MMDB_TEST_FILE}" "${UTPP_MMDB_FULL_TEST_FILE}")

# ---- Offline 0.5.2 stance-change transition curation test (real MMDriver; not shipped) ----
add_executable(utpp_mm_stance_test tests/MMStanceTransitionTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/mm/Steering.hpp)
target_link_libraries(utpp_mm_stance_test PRIVATE bcrypt)
target_compile_definitions(utpp_mm_stance_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_compile_options(utpp_mm_stance_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_stance_test COMMAND utpp_mm_stance_test "${UTPP_MMDB_PACK_TEST_FILE}")

# ---- Offline 0.5.3 gamepad-scenario regression test (real MMDriver, stick-driven game model; not shipped) ----
add_executable(utpp_mm_loco_test tests/MMLocoScenarioTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/mm/Steering.hpp
  src/mm/SpeedCalibration.hpp src/mm/PlaybackRate.hpp src/mm/SpeedBands.hpp)
target_link_libraries(utpp_mm_loco_test PRIVATE bcrypt)
target_compile_definitions(utpp_mm_loco_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_compile_options(utpp_mm_loco_test PRIVATE ${UTPP_WARNINGS} /EHsc)
# The suite is one process of ~400 s (almost all of it LoadDatabase, ~0.75 s per measured run). ctest runs it as parallel shards:
# "--units=<unit>,..." (tests/MMLocoScenarioTest.cpp) selects units of the full run; together the shards run every unit and every
# knee cell exactly once (no --units = the whole single-process run, still available by hand). Keep the utpp_mm_loco_test_ prefix.
# "air" and "airclips" stay together (airclips reads the toe bones SectionAir sets, one process owns the air_notable\ scratch folder next
# to the exe). Timing per shard: UTPP_LOCO_TIMING=1 prints [TIME] lines.
set(UTPP_LOCO_SHARDS
  "knee_j|knee.v6a"
  "knee_k|knee.v6b"
  "knee_g|knee.v7a"
  "knee_h|knee.v7b"
  "knee_v6ma|knee.v6ma"
  "knee_v6mb|knee.v6mb"
  "air|air,airclips"
  "vehexit|vehexit"
  "airsteer_strafe|airsteer,c0c1a,strafe"
  "rest|main,057,058,059,060,062,063,c1b,idle")
foreach(_shard IN LISTS UTPP_LOCO_SHARDS)
  string(REPLACE "|" ";" _shard_parts "${_shard}")
  list(GET _shard_parts 0 _shard_name)
  list(GET _shard_parts 1 _shard_units)
  add_test(NAME utpp_mm_loco_test_${_shard_name} COMMAND utpp_mm_loco_test "${UTPP_MMDB_PACK_TEST_FILE}" "--units=${_shard_units}")
endforeach()
# longest shards first under ctest -j (no cost history yet for the new names): air ~70 s, vehexit ~70 s, the rest 40-60 s
set_tests_properties(utpp_mm_loco_test_air PROPERTIES COST 90)
set_tests_properties(utpp_mm_loco_test_vehexit PROPERTIES COST 80)
set_tests_properties(utpp_mm_loco_test_airsteer_strafe PROPERTIES COST 60)
set_tests_properties(utpp_mm_loco_test_rest PROPERTIES COST 55)

# ---- Offline Phase 4b integration test (yaw convention, gating, bone map, seqlock, blend; not shipped) ----
add_executable(utpp_mm_integration_test tests/MMIntegrationTest.cpp src/mm/GameIntegration.hpp)
target_compile_options(utpp_mm_integration_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_integration_test COMMAND utpp_mm_integration_test "${UTPP_MMDB_TEST_FILE}")

# ---- Staging package (Build only; nothing is installed into the game) ----
set(UTPP_PACKAGE_DIR "${CMAKE_BINARY_DIR}/package")
# Database staged into the package (default: full motion Dense bake; tests keep using UTPP_MMDB_TEST_FILE).
set(UTPP_PACKAGE_DB "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_base_dense_full_posture.utppmm"
    CACHE FILEPATH "Baked .utppmm staged as package utpp.utppmm")
# 0.5.1: optional extra databases (list of .utppmm paths) staged as red4ext/plugins/UTPP/data/<file name>; each one is
# selectable at runtime (UTPPLoadDatabase / UTPP tab "Animation database"). Default empty (package size).
set(UTPP_PACKAGE_EXTRA_DBS "" CACHE STRING "Extra .utppmm files staged into the package db folder (;-separated)")
set(UTPP_PACKAGE_DB_COMMANDS "")
foreach(extra_db IN LISTS UTPP_PACKAGE_EXTRA_DBS)
  list(APPEND UTPP_PACKAGE_DB_COMMANDS COMMAND ${CMAKE_COMMAND} -E copy_if_different "${extra_db}"
       "${UTPP_PACKAGE_DIR}/red4ext/plugins/UTPP/data/")
endforeach()
add_custom_target(utpp_package ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/red4ext/plugins/UTPP"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:UTPP>" "${UTPP_PACKAGE_DIR}/red4ext/plugins/UTPP/"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp.reds"
          "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP/"
  # Mod Settings page of the utpp_core registry (generated by tools/settings/gen_modsettings.py) + its CET bridge
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_modsettings.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_modsettings_bridge.reds" "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP/"
  # Shoulder-swap key (2026-11-09): Mod Settings rows + Input Loader action (default L, pad: hold R3 0.5 s)
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_shoulder_input.reds"
          "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP/"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/r6/input"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/input/utpp_shoulder.xml" "${UTPP_PACKAGE_DIR}/r6/input/"
  # Phase 5.1 CET body-turn module (runtime, no compile dependency:
  # redscript 0.5.31 cannot @wrapMethod a class compiled from source in the same run).
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_bodyturn"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_bodyturn/init.lua"
          "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_bodyturn/"
  # Phase 5.4 (0.3.5) vehicle camera handoff (CET module, runtime).
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_vehiclecam"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_vehiclecam/init.lua"
          "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_vehiclecam/"
  # Motion matching database, loaded from <dll dir>/utpp.utppmm (motion-derived).
  # 0.5.0 Phase 5N: shared UTPP CET module (settings tab, head look, loot view)
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_core"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_core/init.lua"
          "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_core/"
  # Phase 5N step 4b: automatic perspective rules
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_perspective"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_perspective/init.lua"
          "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_perspective/"
  # Phase 5N step 3b: body visibility (TPPRepresentation + generated FPP/TPP part table)
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_body"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_body/init.lua"
          "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_body/body_parts.lua"
          "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_body/"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/red4ext/plugins/UTPP/data"
  ${UTPP_PACKAGE_DB_COMMANDS}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${UTPP_PACKAGE_DB}"
          "${UTPP_PACKAGE_DIR}/red4ext/plugins/UTPP/data/utpp_base.utppmm"
  DEPENDS UTPP "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp.reds" "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_bodyturn/init.lua"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_modsettings.reds" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_modsettings_bridge.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_shoulder_input.reds" "${CMAKE_CURRENT_SOURCE_DIR}/input/utpp_shoulder.xml"
          "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_vehiclecam/init.lua"
          "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_core/init.lua" "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_perspective/init.lua"
          "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_body/init.lua" "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_body/body_parts.lua"
          "${UTPP_PACKAGE_DB}"
  VERBATIM)

# ---- Phase 5N step 4a: native Locomotion (self-contained block appended for a trivial merge) ----
target_sources(UTPP PRIVATE src/input/Locomotion.cpp src/input/Locomotion.hpp src/input/LocomotionCore.hpp)
add_executable(utpp_locomotion_test tests/LocomotionTest.cpp src/input/LocomotionCore.hpp)
target_compile_options(utpp_locomotion_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_locomotion_test COMMAND utpp_locomotion_test "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_locomotion.reds")
add_custom_target(utpp_package_locomotion ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_locomotion.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_bodyturn.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_weapon_restriction.reds"
          "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP/"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_locomotion.reds" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_weapon_restriction.reds"
  VERBATIM)

# ---- vehicle approach walk: move override of the Locomotion core + planner (pure headers) and the script pair ----
target_sources(UTPP PRIVATE src/input/ApproachCore.hpp src/input/NativeInputCore.hpp)
add_executable(utpp_locomotion_override_test tests/LocomotionOverrideTest.cpp src/input/LocomotionCore.hpp
               tests/ref/LocomotionCoreReference.hpp)
target_compile_options(utpp_locomotion_override_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_locomotion_override_test COMMAND utpp_locomotion_override_test)
add_executable(utpp_native_input_test tests/NativeInputTest.cpp src/input/NativeInputCore.hpp)
target_compile_options(utpp_native_input_test PRIVATE ${UTPP_WARNINGS} /EHsc)
# also proves the byte patterns against the real 2.5 exe when it is extracted next to the project
set(UTPP_EXE250 "${CMAKE_CURRENT_SOURCE_DIR}/../../../Extracted/reverse/exe250/Cyberpunk2077.exe" CACHE FILEPATH
    "Extracted 2.5 Cyberpunk2077.exe for the byte-pattern proof (optional; the check is skipped when the file is missing)")
if(EXISTS "${UTPP_EXE250}")
  add_test(NAME utpp_native_input_test COMMAND utpp_native_input_test "${UTPP_EXE250}")
else()
  add_test(NAME utpp_native_input_test COMMAND utpp_native_input_test)
endif()
add_executable(utpp_approach_test tests/ApproachTest.cpp src/input/ApproachCore.hpp)
target_compile_options(utpp_approach_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_approach_test COMMAND utpp_approach_test
         "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/anim/vehicle-entry/entry_approach.json")
# mutation guards (each must FAIL its named check): 1 no facing phase, 2 stick up to 0.85, 3 no stall handling, 4 no turn-first hold, 5 stepped stick,
# 6 stale minD / passed after a facing release and a per-entry facing budget, 7 stick straight at the waypoint (no lead limit / heading latch),
# 8 arrival ignores the residual to the spot, 9 old planner (run-up U-detour, no facing-drift compensation), 10 old position-phase end (momentum lead 0.2 s,
# radius 0.12 m), 11 old carry model / settle point, 12 no facing steering toward the spot, 13 no lead reset on a facing release, 14 no own budget for the
# walk back after a release, 15 no stall boost during the facing turn, 16 input abort against the settle-point bearing in the facing phase,
# 18 old arrival gate 0.15 m, 19 old plan cost of the predicted residual, 20 no aim past the clip yaw during the facing turn, 21 facing release measured
# from the settle point as well, 22 no direction guard of the facing aim near a half turn (r11), 23 the heading latch is taken before the turn-first hold (r11: a
# short leg latches the bearing of the start), 24 fixed 'passed' growth of 0.08 m (r11: no adaptation to the lateral miss)
# r13c: 25 the pre-r13c arrival (gate 0.10 m, old carry model, no lateral settle offset, skip distance 0.35 m: mounts 0.04-0.15 m from the spot), 26 no fade-out of the steering lean
# at the end of the facing budget (a V that does not translate stays off the clip yaw), 27 no clip-heading exemption of the input abort within exemptM of the spot
# r13d: 28 the creep of an aligned V short of the spot keeps the 0.2 facing stick (the engine stops an aligned body: no creep), 29 the linear carry model also for small turns,
# 30 no undershoot of the walk's end (settleBackM)
# r13e: 31 no self-consistent settle points of a close start (the sport1 door start walks straight at the spot and the facing turn carries V past it), 32 no online carry
# adaptation from the measured body turn gain (the slowest turner of the ride plant variants ends past the spot), 33 no undershoot of a last leg parallel to the clip heading
set(_ap_mut_re_1 "FAIL: body that does not turn: gives up after")
set(_ap_mut_re_2 "FAIL: slow plant: walk stick absolute bound 0.68")
set(_ap_mut_re_3 "FAIL: low-stick stall plant")
set(_ap_mut_re_4 "FAIL: facing away from the leg: turn-first hold ran")
set(_ap_mut_re_5 "FAIL: stick rises at most")
set(_ap_mut_re_6 "FAIL: facing drift plant: no facing loop")
set(_ap_mut_re_7 "FAIL: logged rides: the stick leads the body by at most 75 deg")
set(_ap_mut_re_8 "FAIL: logged rides: arrived within 0.16 m of the spot")
set(_ap_mut_re_9 "FAIL: logged rides: path at most 1.2 x the direct distance")
set(_ap_mut_re_10 "FAIL: r5a rides, Locomotion off")
set(_ap_mut_re_11 "FAIL: r13d carry model")
set(_ap_mut_re_12 "FAIL: facing steering")
set(_ap_mut_re_13 "FAIL: facing drift plant: the walk back keeps the stick lead limit")
set(_ap_mut_re_14 "FAIL: facing release, body never follows")
set(_ap_mut_re_15 "FAIL: logged rides: arrived within 0.16 m of the spot")
set(_ap_mut_re_16 "FAIL: facing carry past the settle point: a stick pushed along the clip heading does not abort")
set(_ap_mut_re_18 "FAIL: arrival gate")
set(_ap_mut_re_19 "FAIL: arrival creep: no start takes more than 0.3 s longer")
set(_ap_mut_re_20 "FAIL: r5a rides: all 6 logged starts")
set(_ap_mut_re_21 "FAIL: facing release: a carry 0.15 m past the spot")
set(_ap_mut_re_22 "FAIL: half turn: the facing turn keeps one direction")
set(_ap_mut_re_23 "FAIL: logged rides: arrived within 0.16 m")
set(_ap_mut_re_24 "FAIL: lateral miss of the walk's end")
set(_ap_mut_re_25 "FAIL: arrival creep: every logged r6a start")
set(_ap_mut_re_26 "FAIL: final facing / walk stick range")
set(_ap_mut_re_27 "FAIL: r5a rides: a stick pushed toward the car")
set(_ap_mut_re_28 "FAIL: r13d creep")
set(_ap_mut_re_29 "FAIL: r13d carry model")
set(_ap_mut_re_30 "FAIL: r13d undershoot")
set(_ap_mut_re_31 "FAIL: r13c rides: no systematic miss of one start")
set(_ap_mut_re_32 "FAIL: r13c rides: at most 5 ")
set(_ap_mut_re_33 "FAIL: r13e parallel last leg")
# run-time mutants (src/MutationSelect.hpp): ONE executable reads the mutation number from UTPP_MUTATION_APPROACH (every mutation site is a plain run-time
# expression of ApproachCore.hpp); ctest runs it once per mutation (UTPP_MUTATION_APPROACH unset = the base test)
add_executable(utpp_approach_test_rt tests/ApproachTest.cpp src/input/ApproachCore.hpp src/MutationSelect.hpp)
target_compile_options(utpp_approach_test_rt PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_approach_test_rt PRIVATE UTPP_RUNTIME_MUTATION_APPROACH)
foreach(_ap_mut 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33)
  add_test(NAME utpp_approach_test_mut${_ap_mut} COMMAND utpp_approach_test_rt
           "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/anim/vehicle-entry/entry_approach.json")
  set_tests_properties(utpp_approach_test_mut${_ap_mut} PROPERTIES PASS_REGULAR_EXPRESSION "${_ap_mut_re_${_ap_mut}}"
                       ENVIRONMENT "UTPP_MUTATION_APPROACH=${_ap_mut}")
endforeach()
add_custom_target(utpp_package_vehicle_approach ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_vehicle_approach.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_vehicle_approach_table.reds"
          "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP/"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_vehicle_approach.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_vehicle_approach_table.reds"
  VERBATIM)

# ---- Phase 5N step 5: LootRange + HeadGuard (self-contained block appended for a trivial merge) ----
target_sources(UTPP PRIVATE src/world/LootHead.cpp src/world/LootRange.cpp src/render/HeadGuard.cpp
  src/world/LootHead.hpp src/world/LootHeadLog.hpp src/world/LootRange.hpp src/world/LootRangeCore.hpp
  src/render/HeadGuard.hpp src/render/HeadGuardCore.hpp
  src/world/InteractRange.cpp src/world/InteractRange.hpp src/world/InteractRangeCore.hpp)
add_executable(utpp_loothead_test tests/LootHeadTest.cpp src/world/LootRangeCore.hpp src/world/InteractRangeCore.hpp
  src/render/HeadGuardCore.hpp)
target_compile_options(utpp_loothead_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_loothead_test COMMAND utpp_loothead_test)
add_executable(utpp_headguard_site_test tests/HeadGuardSiteTest.cpp src/render/HeadGuardCore.hpp)
target_compile_options(utpp_headguard_site_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_custom_target(utpp_package_loothead ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_loothead.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_body.reds" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_itemplane.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_crosshair.reds" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_reticle.reds"
          "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP/"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_loothead"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_loothead/init.lua"
          "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_loothead/"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_loothead.reds" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_body.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_itemplane.reds" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_crosshair.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_reticle.reds"
          "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_loothead/init.lua"
  VERBATIM)

# ---- Phase 5N step 3c: own camera (optional package; installed only for the camera test) ----
set(UTPP_ARCHIVE_STAGE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/UTPP/archive_stage" CACHE PATH "Staged utpp_camera.archive(.xl)")
set(UTPP_CAMERA_PACKAGE_DIR "${CMAKE_BINARY_DIR}/package_camera")
add_custom_target(utpp_package_camera ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_CAMERA_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_camera"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_camera/init.lua"
          "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_camera/camera_math.lua"
          "${UTPP_CAMERA_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_camera/"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_CAMERA_PACKAGE_DIR}/archive/pc/mod"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${UTPP_ARCHIVE_STAGE}/utpp_camera.archive"
          "${UTPP_ARCHIVE_STAGE}/utpp_camera.archive.xl" "${UTPP_CAMERA_PACKAGE_DIR}/archive/pc/mod/"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_camera/init.lua" "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_camera/camera_math.lua"
  VERBATIM)

# ---- 2026-10-30 aim pitch step 0: CET test probe (TEST INSTALLS ONLY, never a release: tools/release/make_release.py rejects
# utpp_probe). Own package dir like the camera package; inert until GetMod("utpp_probe").AimProbe(true) (tools/gametest night.py
# --set aim_probe). Install: add bin/x64/plugins/cyber_engine_tweaks/mods/utpp_probe/init.lua of this dir to the test stage.
set(UTPP_PROBE_PACKAGE_DIR "${CMAKE_BINARY_DIR}/package_probe")
add_custom_target(utpp_package_probe ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PROBE_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_probe"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_probe/init.lua"
          "${UTPP_PROBE_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_probe/"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_probe/init.lua"
  VERBATIM)

# ---- 0.6.3 aim offset (self-contained block appended for a trivial merge) ----
target_sources(UTPP PRIVATE src/mm/AimOffset.cpp src/mm/AimOffset.hpp src/mm/AimOffsetRuntime.hpp)
add_executable(utpp_aimoffset_test tests/AimOffsetTest.cpp src/mm/AimOffset.hpp src/mm/GameIntegration.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_aimoffset_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_aimoffset_test COMMAND utpp_aimoffset_test)
# rebuild.cmd runs a fixed list of test executables; this one runs as a post-build step (a failure fails the build).
add_custom_command(TARGET utpp_aimoffset_test POST_BUILD COMMAND utpp_aimoffset_test VERBATIM)

# ---- dual-wield D0: psmBlock dual flag (bit 31) round trip + behaviour identity of the legacy consumers; mut1 / mut2 must fail ----
add_executable(utpp_dualwield_psm_test tests/DualWieldPsmTest.cpp src/mm/GameIntegration.hpp src/mm/AimOffset.hpp)
target_compile_options(utpp_dualwield_psm_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_dualwield_psm_test COMMAND utpp_dualwield_psm_test)
foreach(_dw_mut 1 2)
  add_executable(utpp_dualwield_psm_test_mut${_dw_mut} tests/DualWieldPsmTest.cpp src/mm/GameIntegration.hpp src/mm/AimOffset.hpp)
  target_compile_options(utpp_dualwield_psm_test_mut${_dw_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_dualwield_psm_test_mut${_dw_mut} PRIVATE UTPP_DUALWIELD_MUTATION=${_dw_mut})
  add_test(NAME utpp_dualwield_psm_test_mut${_dw_mut} COMMAND utpp_dualwield_psm_test_mut${_dw_mut})
  set_tests_properties(utpp_dualwield_psm_test_mut${_dw_mut} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL B")
endforeach()

# ---- 0.6.4 item render plane (TPP: RPl_Weapon from the animation parameter broadcast -> RPl_Scene) ----
target_sources(UTPP PRIVATE src/render/ItemPlane.cpp src/render/ItemPlane.hpp)


# ---- 0.6.5 game look write (the game's FPP look = the UTPP world camera direction; input/GameLook.cpp) ----
target_sources(UTPP PRIVATE src/input/GameLook.cpp src/input/GameLook.hpp src/input/GameLookCore.hpp)
add_executable(utpp_gamelook_test tests/GameLookTest.cpp src/input/GameLookCore.hpp)
target_compile_options(utpp_gamelook_test PRIVATE ${UTPP_WARNINGS} /EHsc)
set(UTPP_EXE_COPY "${CMAKE_CURRENT_SOURCE_DIR}/../../../Extracted/reverse/exe250/Cyberpunk2077.exe" CACHE FILEPATH "Copy of the 2.50 game executable (offline site checks)")
add_test(NAME utpp_gamelook_test COMMAND utpp_gamelook_test "${UTPP_EXE_COPY}")
# 2026-10-23: HeadGuard site expectations updated to the 2.5 executable; registered as a test on the same exe copy
add_test(NAME utpp_headguard_site_test COMMAND utpp_headguard_site_test "${UTPP_EXE_COPY}")
# rebuild.cmd runs a fixed list of test executables; this one runs as a post-build step (a failure fails the build).
add_custom_command(TARGET utpp_gamelook_test POST_BUILD COMMAND utpp_gamelook_test "${UTPP_EXE_COPY}" VERBATIM)

# ---- batch 1b feed dump (mm/FeedDump.hpp, header-only, used by MotionHost.cpp; self-contained block appended for a trivial merge) ----
target_sources(UTPP PRIVATE src/mm/FeedDump.hpp)
add_executable(utpp_mm_feeddump_test tests/MMFeedDumpTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/mm/FeedDump.hpp)
target_link_libraries(utpp_mm_feeddump_test PRIVATE bcrypt)
target_compile_definitions(utpp_mm_feeddump_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_compile_options(utpp_mm_feeddump_test PRIVATE ${UTPP_WARNINGS} /EHsc)
set(UTPP_FEEDDUMP_TEST_DIR "${CMAKE_BINARY_DIR}/feeddump_test")
add_test(NAME utpp_mm_feeddump_test COMMAND utpp_mm_feeddump_test "${UTPP_FEEDDUMP_TEST_DIR}" "${UTPP_MMDB_TEST_FILE}")
# rebuild.cmd runs a fixed list of test executables; this one runs as a post-build step (a failure fails the build).
add_custom_command(TARGET utpp_mm_feeddump_test POST_BUILD COMMAND utpp_mm_feeddump_test "${UTPP_FEEDDUMP_TEST_DIR}" "${UTPP_MMDB_TEST_FILE}"
                   VERBATIM)

# ---- AIR HOLD stage (2026-10-13): mm/AirState.hpp air phase machine (header-only, used by MotionHost.cpp); self-contained block ----
target_sources(UTPP PRIVATE src/mm/AirState.hpp)
add_executable(utpp_mm_airstate_test tests/MMAirStateTest.cpp src/mm/AirState.hpp src/mm/GameIntegration.hpp)
target_include_directories(utpp_mm_airstate_test PRIVATE src)
target_compile_options(utpp_mm_airstate_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_airstate_test COMMAND utpp_mm_airstate_test)

# ---- AIR CLIPS stage (2026-10-13): mm/AirTable.hpp .airz root_z sidecar + air clip metadata (header-only); self-contained block ----
target_sources(UTPP PRIVATE src/mm/AirTable.hpp)
target_sources(utpp_mm_sim_test PRIVATE src/mm/AirTable.hpp)
add_executable(utpp_mm_airtable_test tests/MMAirTableTest.cpp src/mm/AirTable.hpp src/BaseDatabase.hpp)
target_include_directories(utpp_mm_airtable_test PRIVATE src)
target_compile_options(utpp_mm_airtable_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_airtable_test COMMAND utpp_mm_airtable_test
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_base.utppmm" "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_base.airz")

# UTPP-MOTION SEAM begin: port v2 test targets (additive; no v1 target above is changed)
# Test fixtures (plain motion bundles without the slide section; not shipped data) live in Build/test-data/motion, not in Data.
set(UTPP_MOTION_FIXTURE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/test-data/motion"
    CACHE PATH "Folder with utpp_motion_<body>_noslide.utppmm / .chooser.json (plain bundles for the slide and query parity tests)")
# ---- port stage 10.1: data-driven chooser (UE Chooser semantics; tables from a motion-derived data file outside git)
set(UTPP_CHOOSER_TEST_TABLE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/sourcepack_chooser_table.json"
    CACHE FILEPATH "utpp-chooser-table JSON (Source/tools/mmbake/export_chooser_table.py)")
set(UTPP_CHOOSER_TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Research/inventory"
    CACHE PATH "Folder with chooser_oracle.json and chooser_vectors.json")
add_executable(utpp_mm_chooser_test tests/MMChooserTest.cpp src/mm/Chooser.hpp src/mm/MiniJson.hpp)
target_compile_options(utpp_mm_chooser_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_chooser_test COMMAND utpp_mm_chooser_test "${UTPP_CHOOSER_TEST_TABLE}"
         "${UTPP_CHOOSER_TEST_DIR}/chooser_oracle.json" "${UTPP_CHOOSER_TEST_DIR}/chooser_vectors.json")

# ---- slide: optional slide model of the chooser (motion; slide chooser json + base json + hand-written oracle)
add_executable(utpp_mm_chooser_slide_test tests/MMChooserSlideTest.cpp src/mm/Chooser.hpp src/mm/MiniJson.hpp)
target_compile_options(utpp_mm_chooser_slide_test PRIVATE ${UTPP_WARNINGS} /EHsc)

# ---- blend-time inertialization (QuinticInertializer)
add_executable(utpp_mm_inertializer_test tests/InertializerTest.cpp src/mm/Inertializer.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_mm_inertializer_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_inertializer_test COMMAND utpp_mm_inertializer_test)

# ---- port stage 10.1: .utppmm VERSION 2 reader self-test (motion-derived database outside git)
set(UTPP_MOTION_DB_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_motion_woman.utppmm"
    CACHE FILEPATH "Baked .utppmm motion (mmbake_motion.py) used by utpp_motion_db_test")
add_executable(utpp_motion_db_test tests/MotionDatabaseTest.cpp src/mm/MotionDatabase.hpp src/mm/MiniJson.hpp src/BaseDatabase.hpp)
target_compile_options(utpp_motion_db_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_motion_db_test COMMAND utpp_motion_db_test "${UTPP_MOTION_DB_TEST_FILE}")

# ---- port stage 10.1: ChooserMatcher (motion pose selection on MotionDatabase + Chooser; databases outside git)
set(UTPP_MOTION_DB_MAN_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_motion_man.utppmm"
    CACHE FILEPATH "Baked .utppmm motion for the man rig (utpp_mm_chooser_matcher_test)")
add_executable(utpp_mm_chooser_matcher_test tests/ChooserMatcherTest.cpp src/mm/ChooserMatcher.hpp src/mm/MotionDatabase.hpp
    src/mm/Chooser.hpp src/mm/MiniJson.hpp src/mm/MMMath.hpp src/BaseDatabase.hpp)
target_compile_options(utpp_mm_chooser_matcher_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_chooser_matcher_test COMMAND utpp_mm_chooser_matcher_test "${UTPP_MOTION_DB_TEST_FILE}" "${UTPP_MOTION_DB_MAN_TEST_FILE}")

# Overlay runtime tests (owned by the overlay work; optional until that file exists).
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/overlay.cmake OPTIONAL)

# Montage / blend space / DeadBlending runtime tests (motion report 4.2 #32-#34; optional until that file exists).
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/montage.cmake OPTIONAL)
# Report 6.10: compile check of the DRAFT v2 driver adapter (src/mm/MotionAdapter.hpp, not in the DLL).
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/motion_adapter.cmake OPTIONAL)
# Report 3.14 / stage 10.8: traversal decision table, warp solver, CP obstacle mapping.
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/traversal.cmake OPTIONAL)
# motion idle-break player test (report 2.5.1 / 3.18, stage 10.9; optional until that file exists).
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/idlebreaks.cmake OPTIONAL)
# stage 10.8 part 2: .utpptrav reader (src/mm/TraversalFile.hpp) + SolveWarp on baked Hurdle / Mantle / Vault montages.
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/travfile.cmake OPTIONAL)
# motion lean / head-turn pose pack (utpp.lean/1) reader + evaluator (src/mm/LeanPack.hpp); data outside git.
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/lean.cmake OPTIONAL)
# motion lean layer runtime (src/mm/LeanRuntime.hpp, CET mm.lean): unit test + offline pose replay; data outside git.
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/leanruntime.cmake OPTIONAL)
# stage 10.8 prep: MotionTraversal (game climb / vault start -> motion montage, warp to the game ledge, hand-back to MM).
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/motion_traversal.cmake OPTIONAL)
# stage 10.1 p.4: offline replay of recorded game feeds through MotionAdapter (utpp_motion_replay; data outside git).
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/replay.cmake OPTIONAL)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/jumppick.cmake OPTIONAL) # fix3/jump: jump pick + quick slide combinations (P6 K2, N3)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/slowwalk.cmake OPTIONAL) # G1: established slow walk plays the Walk_Loop clips
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/steadyrun.cmake OPTIONAL) # G10: steady Run gait (stick 0.7-0.82) plays Walk_Loop / Run_Loop clips, no pose reset
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/analogfloor.cmake OPTIONAL) # minimum analog walk speed: floor multiplier, floor stick, modifier identity
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/locostart.cmake OPTIONAL) # start from rest (no pivot without a velocity heading, slow start margin) + jump take-off hold
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/locohip.cmake OPTIONAL) # hipfire raise out of Relaxed, isolated hold / re-raise, crouch move B fade (HIP-1, HIP-2, CR-5)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/itemplane.cmake OPTIONAL) # item render plane core: policy, registry re-apply, outside writes, prune, cap
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/locomatcher.cmake OPTIONAL) # crouch walk rate floor, run/sprint -> crouch, slide entry / mid-slide, crouch -> stand (recording 2026-10-31 22:55)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/locostance.cmake OPTIONAL) # composed pose across the stance edges: overlay upper body on the stance alpha of B (RC1, CR-4; recording 2026-10-31 22:55)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/footsink.cmake OPTIONAL) # foot height keeping of the live crossfade: feet stay on the layers' height through crouch -> stand (needs locostance.cmake above)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/ovlegs.cmake OPTIONAL) # armed lower-body layering through the stance fades: composed ankles stay on the base pose's height (needs locostance.cmake above)
# stage 10.6 grip IK: two-bone hand IK to the weapon grip after the overlay layering (src/mm/GripIK.hpp, report 3.13 / 5.5).
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/gripik.cmake OPTIONAL)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/predicates.cmake OPTIONAL)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/motion_pose.cmake OPTIONAL)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/clavlimit.cmake OPTIONAL) # mesh-space arm clavicle limit
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/forearmtwist.cmake OPTIONAL) # forearm roll moved into the hand (game convention)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/supportgrip.cmake OPTIONAL) # rifle grip on the handguard + stock in the shoulder (rifle-grip-stock 2026-11-10)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/malecorrection.cmake OPTIONAL) # male V upper-body deformation correction (world-preserving chest / clavicle / arm rotations)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/bodytracecont.cmake OPTIONAL) # continuous helper-bone capture
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/posecapture.cmake OPTIONAL) # compact pose capture (format, session layer, writer)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/diagreport.cmake OPTIONAL) # diagnostic report file + relative paths in logs
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/motiondriver.cmake OPTIONAL) # UTPP-MOTION SEAM: MotionDriver unit test
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/reloadfix.cmake OPTIONAL) # fix batch 1: reload-path policy tests
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/magtap.cmake OPTIONAL) # magazine top-up gesture (refill without a reload state)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/stancefix.cmake OPTIONAL) # fix batch 2: stance latency (P1) tests
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/hipfireweapons.cmake OPTIONAL) # fix batch 3: hipfire with weapons that have no recoil route (P9)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/hipfireraise.cmake OPTIONAL) # fix batch 3: key changes inside / after the hip raise (P9 review 3)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/shooter_mode.cmake OPTIONAL) # feat/shooterMode-dll: Shooter mode + H2 safeguard (strafe rotation mode, root hold / lock)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/vehexit.cmake OPTIONAL) # vehicle batch: exit gate / traversal release tests
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/vehtrans.cmake OPTIONAL) # vehicle batch: mount/exit pose transition tests
# UTPP-MOTION SEAM review minor 7: the v2 glue next to the DLL in the staging package - the DLL always registers the mm.motionDriver
# UTPP-MOTION SEAM natives (MotionDriverNatives.cpp), so its script (scripts/utpp_motion.reds) and CET mod (cet/utpp_motion) are staged with it
# UTPP-MOTION SEAM (utpp_package builds / copies the DLL first). Additive target: utpp_package itself is unchanged. The v2 data folder
# UTPP-MOTION SEAM (<dll dir>/v2, motion-derived) is never staged here (the local stage script, local tests only).
add_custom_target(utpp_package_motion ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_motion.reds"
          "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP/"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_motion"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_motion/init.lua"
          "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_motion/"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/utpp_motion.reds" "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_motion/init.lua"
  VERBATIM) # UTPP-MOTION SEAM
add_dependencies(utpp_package_motion utpp_package) # UTPP-MOTION SEAM DLL staged first
# UTPP-MOTION SEAM end: port v2 test targets
add_executable(utpp_body_deformer_trace_test tests/BodyDeformerTraceTest.cpp src/mm/BodyDeformerTrace.hpp)
target_compile_features(utpp_body_deformer_trace_test PRIVATE cxx_std_20)
target_compile_options(utpp_body_deformer_trace_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_body_deformer_trace_test COMMAND utpp_body_deformer_trace_test)
add_executable(utpp_body_deformer_tracks_test tests/BodyDeformerTracksTest.cpp src/mm/BodyDeformerOffline.hpp)
target_compile_features(utpp_body_deformer_tracks_test PRIVATE cxx_std_20)
target_compile_options(utpp_body_deformer_tracks_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_body_deformer_tracks_test COMMAND utpp_body_deformer_tracks_test)
add_executable(utpp_body_deformer_rotation_test tests/BodyDeformerRotationTest.cpp src/mm/BodyDeformerOffline.hpp)
target_compile_features(utpp_body_deformer_rotation_test PRIVATE cxx_std_20)
target_compile_options(utpp_body_deformer_rotation_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_body_deformer_rotation_test COMMAND utpp_body_deformer_rotation_test)
add_executable(utpp_body_helper_patch_test tests/BodyHelperPatchTest.cpp src/mm/BodyHelperPatch.hpp)
target_compile_features(utpp_body_helper_patch_test PRIVATE cxx_std_20)
target_compile_options(utpp_body_helper_patch_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_body_helper_patch_test COMMAND utpp_body_helper_patch_test)
target_sources(UTPP PRIVATE src/PoseHook.cpp src/PoseHook.hpp src/mm/BodyDeformerTrace.hpp)
target_sources(UTPP PRIVATE src/BodyTraceRuntime.cpp src/BodyTraceRuntime.hpp src/mm/BodyTraceContinuous.hpp) # UTPPBodyPoseTraceContinuous
target_sources(UTPP PRIVATE src/PoseCaptureRuntime.cpp src/PoseCaptureRuntime.hpp src/PoseCaptureCompression.hpp src/mm/PoseCaptureFormat.hpp src/mm/PoseCaptureSession.hpp src/mm/CaptureOwner.hpp) # UTPPPoseCaptureStart / Stop / Status
target_sources(UTPP PRIVATE src/DiagReportRuntime.cpp src/DiagReportRuntime.hpp src/DiagReport.hpp src/PathScrub.hpp) # UTPPDiagWriteReport + path scrub of every log line
target_sources(UTPP PRIVATE src/EarlyPoseRuntime.cpp src/EarlyPoseRuntime.hpp)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/earlyposeruntime.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/shooter_mode_evidence.cmake OPTIONAL) # shooterMode mode: evidence tools (A8 decision dump, A9 H2 session gate replay)
foreach(probe IN ITEMS SemanticPoseDiscovery PerspectiveWriteGate MappingByteReceipt MappingCycleReceipt)
  add_executable(utpp_${probe}_test tests/${probe}Test.cpp)
  target_compile_features(utpp_${probe}_test PRIVATE cxx_std_20)
  target_compile_options(utpp_${probe}_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  add_test(NAME utpp_${probe}_test COMMAND utpp_${probe}_test)
endforeach()
add_executable(utpp_body_helper_candidate_test tests/BodyHelperCandidateTest.cpp src/mm/BodyHelperCandidate.hpp)
target_compile_features(utpp_body_helper_candidate_test PRIVATE cxx_std_20)
target_compile_options(utpp_body_helper_candidate_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_body_helper_candidate_test COMMAND utpp_body_helper_candidate_test)

target_sources(UTPP PRIVATE src/BodyHelperRuntime.cpp src/BodyHelperRuntime.hpp src/mm/BodyHelperAdmission.hpp)

add_executable(utpp_body_helper_commit_test tests/BodyHelperCommitTest.cpp src/mm/BodyHelperCommit.hpp)
target_compile_features(utpp_body_helper_commit_test PRIVATE cxx_std_20)
target_compile_options(utpp_body_helper_commit_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_body_helper_commit_test COMMAND utpp_body_helper_commit_test)

# Pure admission / hot-gate rules of the helper recompute path (which bundle may write, for which body, under which guards).
add_executable(utpp_body_helper_admission_test tests/BodyHelperAdmissionTest.cpp src/mm/BodyHelperAdmission.hpp)
target_compile_features(utpp_body_helper_admission_test PRIVATE cxx_std_20)
target_compile_options(utpp_body_helper_admission_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_body_helper_admission_test COMMAND utpp_body_helper_admission_test)

# Player helper bundles (one per body) on recorded frames (fixtures derived from local recordings: not shipped, live in Build/test-data; the tests exist only where they do).
set(UTPP_HELPER_BUNDLE_FIXTURE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/test-data/body_helper_man" CACHE PATH "Man helper bundle + recorded-frame fixture")
set(UTPP_HELPER_BUNDLE_FIXTURE_DIR_WOMAN "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/test-data/body_helper_woman" CACHE PATH "Woman helper bundle + recorded-frame fixture")
if(EXISTS "${UTPP_HELPER_BUNDLE_FIXTURE_DIR}/frames.txt" AND EXISTS "${UTPP_HELPER_BUNDLE_FIXTURE_DIR_WOMAN}/frames.txt")
  add_executable(utpp_body_helper_bundle_test tests/BodyHelperBundleTest.cpp src/mm/BodyHelperCandidate.hpp src/mm/BodyHelperBindingFile.hpp src/mm/BodyHelperCommit.hpp src/mm/BodyHelperAdmission.hpp)
  target_compile_features(utpp_body_helper_bundle_test PRIVATE cxx_std_20)
  target_compile_options(utpp_body_helper_bundle_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_link_libraries(utpp_body_helper_bundle_test PRIVATE bcrypt)
  add_test(NAME utpp_body_helper_bundle_test COMMAND utpp_body_helper_bundle_test "${UTPP_HELPER_BUNDLE_FIXTURE_DIR}" man)
  add_test(NAME utpp_body_helper_bundle_woman_test COMMAND utpp_body_helper_bundle_test "${UTPP_HELPER_BUNDLE_FIXTURE_DIR_WOMAN}" woman)
  # The real BodyHelperRuntime (Load admission per body folder, PrepareArm, hot gate, Evaluate + Commit) on both fixtures, with the negative controls.
  add_executable(utpp_body_helper_runtime_test tests/BodyHelperRuntimeTest.cpp src/BodyHelperRuntime.cpp src/BodyHelperRuntime.hpp src/EarlyPoseRuntime.cpp src/EarlyPoseRuntime.hpp)
  target_compile_features(utpp_body_helper_runtime_test PRIVATE cxx_std_20)
  target_compile_options(utpp_body_helper_runtime_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_link_libraries(utpp_body_helper_runtime_test PRIVATE bcrypt)
  add_test(NAME utpp_body_helper_runtime_test COMMAND utpp_body_helper_runtime_test "${UTPP_HELPER_BUNDLE_FIXTURE_DIR}" "${UTPP_HELPER_BUNDLE_FIXTURE_DIR_WOMAN}")
  # Cost of Build+Commit for each plan (same thread_local scratch as the hook). Ceiling on the MEDIAN per call: 90 us = about 2x the
  # median measured on 2026-11-12 (44 us with 40 outputs / 281 ops), so a real slowdown of the hot path fails while a loaded machine does not.
  add_test(NAME utpp_body_helper_bundle_timing_test COMMAND utpp_body_helper_bundle_test "${UTPP_HELPER_BUNDLE_FIXTURE_DIR}" man --bench 20000 90)
  add_test(NAME utpp_body_helper_bundle_woman_timing_test COMMAND utpp_body_helper_bundle_test "${UTPP_HELPER_BUNDLE_FIXTURE_DIR_WOMAN}" woman --bench 20000 90)
  set_tests_properties(utpp_body_helper_bundle_timing_test utpp_body_helper_bundle_woman_timing_test PROPERTIES LABELS timing RUN_SERIAL TRUE)
else()
  message(WARNING "UTPP: helper bundle fixtures not found (${UTPP_HELPER_BUNDLE_FIXTURE_DIR}/frames.txt and ${UTPP_HELPER_BUNDLE_FIXTURE_DIR_WOMAN}/frames.txt): "
                  "utpp_body_helper_bundle_test, utpp_body_helper_bundle_woman_test, utpp_body_helper_runtime_test and the two timing tests are NOT built or run. "
                  "Recreate them with tools/analysis/make_helper_bundle_fixture.py or set UTPP_HELPER_BUNDLE_FIXTURE_DIR[_WOMAN].")
endif()

# ---- opt-in head lead (mm/HeadLead.hpp): unit test (sign / symmetry / bias removal / fades / NaN) and the offline film replay tool
add_executable(utpp_headlead_test tests/HeadLeadTest.cpp src/mm/HeadLead.hpp)
target_compile_features(utpp_headlead_test PRIVATE cxx_std_20)
target_compile_options(utpp_headlead_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_headlead_test COMMAND utpp_headlead_test)
add_executable(utpp_headlead_replay EXCLUDE_FROM_ALL tests/HeadLeadReplay.cpp src/mm/HeadLead.hpp)
target_compile_features(utpp_headlead_replay PRIVATE cxx_std_20)
target_compile_options(utpp_headlead_replay PRIVATE ${UTPP_WARNINGS} /EHsc)
# head lead on REAL run poses of the v2 database through ResolveDb + the real bone map + the real record blend (motion-derived database
# outside git: the test exists only where the file is)
set(_utpp_hl_db "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_motion_woman.utppmm")
if(UTPP_MOTION_DB_TEST_FILE)
    set(_utpp_hl_db "${UTPP_MOTION_DB_TEST_FILE}")
endif()
if(EXISTS "${_utpp_hl_db}")
    add_executable(utpp_headlead_db_test tests/HeadLeadDbTest.cpp src/mm/HeadLead.hpp src/mm/MotionDatabase.hpp src/mm/GameIntegration.hpp)
    target_compile_features(utpp_headlead_db_test PRIVATE cxx_std_20)
    target_compile_options(utpp_headlead_db_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    add_test(NAME utpp_headlead_db_test COMMAND utpp_headlead_db_test "${_utpp_hl_db}")
endif()

# ---- opt-in orientation warp (mm/OrientWarp.hpp, CET mm.orientWarp): unit test (conventions, gates = exactly 0, circles left /
# right, mirror symmetry, bounds, rate limits, Apply, NaN) and, where the motion-derived database exists, the same on its real run loops;
# plus the offline replay tool of the 2026-10-19 captures (scratch parser reduces the 3-stage pose csv, see tests/OrientWarpReplay.cpp)
add_executable(utpp_orientwarp_test tests/OrientWarpTest.cpp src/mm/OrientWarp.hpp src/mm/HeadLead.hpp src/mm/MotionDatabase.hpp)
target_compile_features(utpp_orientwarp_test PRIVATE cxx_std_20)
target_compile_options(utpp_orientwarp_test PRIVATE ${UTPP_WARNINGS} /EHsc)
if(EXISTS "${_utpp_hl_db}")
    add_test(NAME utpp_orientwarp_test COMMAND utpp_orientwarp_test "${_utpp_hl_db}")
else()
    add_test(NAME utpp_orientwarp_test COMMAND utpp_orientwarp_test)
endif()
add_executable(utpp_orientwarp_replay EXCLUDE_FROM_ALL tests/OrientWarpReplay.cpp src/mm/OrientWarp.hpp src/mm/HeadLead.hpp)
target_compile_features(utpp_orientwarp_replay PRIVATE cxx_std_20)
target_compile_options(utpp_orientwarp_replay PRIVATE ${UTPP_WARNINGS} /EHsc)

# ---- v2 slide feed (CET mm.slide): offline replay of real game slides through adapter::MapToChooser (header only, fixture
# tests/data/slide_feeds.txt) + the chooser.json "slide" adapter; and the same setting through the real MotionDriver (bundle
# detection, OwnsSlide, no contacts on slide frames, slide databases picked; needs the motion-derived data dir like
# utpp_motion_driver_unit_test). The real chooser files are optional arguments of the feed test (outside git).
add_executable(utpp_mm_slide_feed_test tests/SlideFeedTest.cpp src/mm/MotionAdapter.hpp src/mm/ChooserMatcher.hpp)
target_compile_features(utpp_mm_slide_feed_test PRIVATE cxx_std_20)
target_compile_options(utpp_mm_slide_feed_test PRIVATE ${UTPP_WARNINGS} /EHsc)
set(_utpp_slide_feed_args "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/slide_feeds.txt")
if(DEFINED _utpp_mdmotion_data AND EXISTS "${_utpp_mdmotion_data}/utpp_motion_woman_knee_slide.chooser.json"
   AND EXISTS "${UTPP_MOTION_FIXTURE_DIR}/utpp_motion_woman_noslide.chooser.json")
    list(APPEND _utpp_slide_feed_args "${_utpp_mdmotion_data}/utpp_motion_woman_knee_slide.chooser.json"
         "${UTPP_MOTION_FIXTURE_DIR}/utpp_motion_woman_noslide.chooser.json")
endif()
add_test(NAME utpp_mm_slide_feed_test COMMAND utpp_mm_slide_feed_test ${_utpp_slide_feed_args})
# slide EXIT replay (fix/slide-exit): real feeds around slides (tests/data/slide_exit_feeds.txt, 2026-10-20 / 10-21 sessions)
# through MapToChooser + ChooserMatcher + the live crossfade BlendStack on the installed twistfix slide bundle: pose-blend
# rotation rate on slide exits, exit-clip hold, jump picks after a slide, stance flicker. Needs the motion-derived data dir.
if(DEFINED _utpp_mdmotion_data AND EXISTS "${_utpp_mdmotion_data}/utpp_motion_woman_knee_twistfix_slide.utppmm"
   AND EXISTS "${_utpp_mdmotion_data}/utpp_motion_woman_knee_twistfix_slide.chooser.json")
    add_executable(utpp_mm_slide_exit_test tests/SlideExitTest.cpp src/mm/MotionBlendStack.hpp src/mm/MotionAdapter.hpp
                   src/mm/ChooserMatcher.hpp)
    target_compile_features(utpp_mm_slide_exit_test PRIVATE cxx_std_20)
    target_compile_options(utpp_mm_slide_exit_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_link_libraries(utpp_mm_slide_exit_test PRIVATE bcrypt)
    add_test(NAME utpp_mm_slide_exit_test COMMAND utpp_mm_slide_exit_test
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/slide_exit_feeds.txt"
             "${_utpp_mdmotion_data}/utpp_motion_woman_knee_twistfix_slide")
endif()
if(DEFINED _utpp_mdmotion_data_native)
    add_executable(utpp_motion_driver_slide_test tests/MotionDriverSlideTest.cpp src/MotionDriver.cpp src/MotionDriver.hpp src/MotionHost.hpp
                   src/mm/MotionAdapter.hpp src/mm/MotionPose.hpp src/mm/ChooserMatcher.hpp src/mm/MotionDatabase.hpp src/mm/Chooser.hpp
                   src/mm/OverlayFile.hpp src/mm/TraversalFile.hpp src/mm/GameIntegration.hpp)
    target_compile_options(utpp_motion_driver_slide_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_motion_driver_slide_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    target_compile_features(utpp_motion_driver_slide_test PRIVATE cxx_std_20)
    file(TO_NATIVE_PATH "${UTPP_MOTION_FIXTURE_DIR}" _utpp_fixture_native)
    add_test(NAME utpp_motion_driver_slide_test COMMAND utpp_motion_driver_slide_test "${_utpp_mdmotion_data_native}"
             --plain-dir "${_utpp_fixture_native}")
    # 2026-11-05: the driver-level mutants utpp_reload_rifle_slide_driver_mut4
    # (UTPP_RELOAD_RIFLE_MUTATION=4) and utpp_slide_carry_driver_mut8 (UTPP_RELOAD_ENVELOPE_MUTATION=8) were removed: the same sites
    # (ReloadContactFade.hpp) are killed by utpp_reload_rifle_contacts_mut4 / utpp_reload_slide_rollhold_mut8.
    # the same setting through the real v1 host (MotionHost.cpp Feed): body-yaw command out of the slide, no weight pop
    add_executable(utpp_motion_driver_slide_host_test tests/MotionDriverSlideHostTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_compile_features(utpp_motion_driver_slide_host_test PRIVATE cxx_std_20)
    target_compile_options(utpp_motion_driver_slide_host_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_motion_driver_slide_host_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
    target_link_libraries(utpp_motion_driver_slide_host_test PRIVATE bcrypt)
    add_test(NAME utpp_motion_driver_slide_host_test COMMAND utpp_motion_driver_slide_host_test "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}")
endif()

# ---- vehicle exit guard of the player's TPP representation (world/TppRepGuard.cpp; self-contained block appended for a trivial merge) ----
target_sources(UTPP PRIVATE src/world/TppRepGuard.cpp src/world/TppRepGuard.hpp src/world/TppRepGuardCore.hpp)
add_executable(utpp_tpprepguard_test tests/TppRepGuardTest.cpp src/world/TppRepGuardCore.hpp)
target_include_directories(utpp_tpprepguard_test PRIVATE src)
target_compile_options(utpp_tpprepguard_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_tpprepguard_test COMMAND utpp_tpprepguard_test "${UTPP_EXE_COPY}")

# ---- ragdoll R0 probe (src/ragdoll/GetUpProbe.hpp; glue in MotionDriver.cpp "ragdoll R0 probe"): pose composition + lifetime (offline) and,
# with the baked v2 databases, the presence of the get-up clip the driver publishes. Mutations: 1 wrong displacement axis, 2 the hold never expires
# (the separate prototype in src/ragdoll/RagdollSim.hpp / RagdollSimBench.cpp is not part of this build).
add_executable(utpp_ragdoll_getup_probe_test tests/RagdollGetUpProbeTest.cpp src/ragdoll/GetUpProbe.hpp src/mm/MotionDatabase.hpp)
target_compile_features(utpp_ragdoll_getup_probe_test PRIVATE cxx_std_20)
target_compile_options(utpp_ragdoll_getup_probe_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_ragdoll_getup_probe_test PRIVATE UTPP_RAGDOLL_PROBE_DB)
add_test(NAME utpp_ragdoll_getup_probe_test COMMAND utpp_ragdoll_getup_probe_test "${UTPP_MOTION_DB_TEST_FILE}" "${UTPP_MOTION_DB_MAN_TEST_FILE}")
foreach(_utpp_rag_mut IN ITEMS 1 2)
  add_executable(utpp_ragdoll_getup_probe_mut${_utpp_rag_mut} tests/RagdollGetUpProbeTest.cpp src/ragdoll/GetUpProbe.hpp)
  target_compile_features(utpp_ragdoll_getup_probe_mut${_utpp_rag_mut} PRIVATE cxx_std_20)
  target_compile_options(utpp_ragdoll_getup_probe_mut${_utpp_rag_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_ragdoll_getup_probe_mut${_utpp_rag_mut} PRIVATE UTPP_RAGDOLL_PROBE_MUTATION=${_utpp_rag_mut})
  add_test(NAME utpp_ragdoll_getup_probe_mut${_utpp_rag_mut} COMMAND utpp_ragdoll_getup_probe_mut${_utpp_rag_mut})
endforeach()
set_tests_properties(utpp_ragdoll_getup_probe_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL compose: offset on [+]Y only")
set_tests_properties(utpp_ragdoll_getup_probe_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL hold expires")

# ---- ragdoll R1f probe: read-only pose slot dump (ragdoll/RagdollSlotProbe.cpp) ----
target_sources(UTPP PRIVATE src/ragdoll/RagdollSlotProbe.cpp src/ragdoll/RagdollSlotProbe.hpp)

# ---- ragdoll production core: keep store (src/ragdoll/RdKeep.hpp) and episode / settle / pelvis telemetry (src/ragdoll/RagdollEpisode.hpp) ----
# Mutations: keep 1 serial check gone, 2 FK never closes the evaluation, 3 no age cap; episode 1 blend-out timer starts at Release,
# 2 Abort leaves the owner flag, 3 a late impact does not re-arm the settle hold.
target_sources(UTPP PRIVATE src/ragdoll/RdKeep.hpp src/ragdoll/RagdollEpisode.hpp src/ragdoll/RagdollGuards.hpp)
add_executable(utpp_ragdoll_keep_test tests/RagdollKeepTest.cpp src/ragdoll/RdKeep.hpp)
target_compile_features(utpp_ragdoll_keep_test PRIVATE cxx_std_20)
target_compile_options(utpp_ragdoll_keep_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_ragdoll_keep_test COMMAND utpp_ragdoll_keep_test)
add_executable(utpp_ragdoll_episode_test tests/RagdollEpisodeTest.cpp src/ragdoll/RagdollEpisode.hpp src/ragdoll/RagdollGuards.hpp)
target_compile_features(utpp_ragdoll_episode_test PRIVATE cxx_std_20)
target_compile_options(utpp_ragdoll_episode_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_ragdoll_episode_test COMMAND utpp_ragdoll_episode_test)
foreach(_utpp_rk_mut IN ITEMS 1 2 3)
  add_executable(utpp_ragdoll_keep_mut${_utpp_rk_mut} tests/RagdollKeepTest.cpp src/ragdoll/RdKeep.hpp)
  target_compile_features(utpp_ragdoll_keep_mut${_utpp_rk_mut} PRIVATE cxx_std_20)
  target_compile_options(utpp_ragdoll_keep_mut${_utpp_rk_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_ragdoll_keep_mut${_utpp_rk_mut} PRIVATE UTPP_RAGDOLL_KEEP_MUTATION=${_utpp_rk_mut})
  add_test(NAME utpp_ragdoll_keep_mut${_utpp_rk_mut} COMMAND utpp_ragdoll_keep_mut${_utpp_rk_mut})
  add_executable(utpp_ragdoll_episode_mut${_utpp_rk_mut} tests/RagdollEpisodeTest.cpp src/ragdoll/RagdollEpisode.hpp)
  target_compile_features(utpp_ragdoll_episode_mut${_utpp_rk_mut} PRIVATE cxx_std_20)
  target_compile_options(utpp_ragdoll_episode_mut${_utpp_rk_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_ragdoll_episode_mut${_utpp_rk_mut} PRIVATE UTPP_RAGDOLL_EPISODE_MUTATION=${_utpp_rk_mut})
  add_test(NAME utpp_ragdoll_episode_mut${_utpp_rk_mut} COMMAND utpp_ragdoll_episode_mut${_utpp_rk_mut})
endforeach()
set_tests_properties(utpp_ragdoll_keep_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL stale serial")
set_tests_properties(utpp_ragdoll_keep_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL close evaluation")
set_tests_properties(utpp_ragdoll_keep_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL age cap")
set_tests_properties(utpp_ragdoll_episode_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL blend-out timer starts at the gate edge")
set_tests_properties(utpp_ragdoll_episode_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL abort clears the owner flag")
set_tests_properties(utpp_ragdoll_episode_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL late impact re-arms the settle hold")
# r2 native review fixes: keep 4 aged open slot never reclaimed, 5 poisoned slot stays valid; episode 4 forced blend-out bounces back to
# Active, 5 a missing slot reading clears the Abort lockout, 6 the slot cache binds any component, 7 pelvis indices shared by all rigs.
foreach(_utpp_rk_mut IN ITEMS 4 5)
  add_executable(utpp_ragdoll_keep_mut${_utpp_rk_mut} tests/RagdollKeepTest.cpp src/ragdoll/RdKeep.hpp)
  target_compile_features(utpp_ragdoll_keep_mut${_utpp_rk_mut} PRIVATE cxx_std_20)
  target_compile_options(utpp_ragdoll_keep_mut${_utpp_rk_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_ragdoll_keep_mut${_utpp_rk_mut} PRIVATE UTPP_RAGDOLL_KEEP_MUTATION=${_utpp_rk_mut})
  add_test(NAME utpp_ragdoll_keep_mut${_utpp_rk_mut} COMMAND utpp_ragdoll_keep_mut${_utpp_rk_mut})
endforeach()
foreach(_utpp_rk_mut IN ITEMS 4 5 6 7 8)
  add_executable(utpp_ragdoll_episode_mut${_utpp_rk_mut} tests/RagdollEpisodeTest.cpp src/ragdoll/RagdollEpisode.hpp src/ragdoll/RagdollGuards.hpp)
  target_compile_features(utpp_ragdoll_episode_mut${_utpp_rk_mut} PRIVATE cxx_std_20)
  target_compile_options(utpp_ragdoll_episode_mut${_utpp_rk_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_ragdoll_episode_mut${_utpp_rk_mut} PRIVATE UTPP_RAGDOLL_EPISODE_MUTATION=${_utpp_rk_mut})
  add_test(NAME utpp_ragdoll_episode_mut${_utpp_rk_mut} COMMAND utpp_ragdoll_episode_mut${_utpp_rk_mut})
endforeach()
set_tests_properties(utpp_ragdoll_keep_mut4 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL aged slot is reclaimed")
set_tests_properties(utpp_ragdoll_keep_mut5 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL a poisoned slot is reusable at once")
set_tests_properties(utpp_ragdoll_episode_mut4 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL forced blend-out ends")
set_tests_properties(utpp_ragdoll_episode_mut5 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL abort lockout survives an unknown gate")
set_tests_properties(utpp_ragdoll_episode_mut6 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL slot cache binds only the known player")
set_tests_properties(utpp_ragdoll_episode_mut7 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL pelvis indices are per rig")
set_tests_properties(utpp_ragdoll_episode_mut8 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL hand-off to the get-up keeps the stuck gate")

# ---- ragdoll native get-up (src/ragdoll/GetUpPlayer.hpp pure logic, RdLastPose.hpp store, MotionAdapter.hpp MapContext::ragdollRecover; glue in
# MotionDriver.cpp): classifier, alignment + plan, blend / floor clamp / world anchoring, phase machine, loader grammar, false jump / landing
# after a ragdoll. Optional fixtures tests/data/getup_frame0_<body>.json (frame-0 model joints of the baked clips) are read when present.
# Mutations (each MUST fail): 1 lateral sign swapped, 2 alignment ignores the weights, 3 no floor clamp, 4 plan-time capsule, 6 a smaller
# descriptor overwrites the full-body pose, 7 no early hand-off on input, 12 no hand step (r13d: ragdoll fingers stay under the floor),
# 13 the blend keeps the plain 0.10 m lift cap (seq 9);
# UTPP_RAGDOLL_RECOVER_MUTATION 5 no distance guard (ledge fall).
target_sources(UTPP PRIVATE src/ragdoll/GetUpPlayer.hpp src/ragdoll/RdLastPose.hpp)
add_executable(utpp_ragdoll_getup_test tests/RagdollGetUpTest.cpp src/ragdoll/GetUpPlayer.hpp src/ragdoll/RdLastPose.hpp src/mm/MotionAdapter.hpp)
target_compile_features(utpp_ragdoll_getup_test PRIVATE cxx_std_20)
target_compile_options(utpp_ragdoll_getup_test PRIVATE ${UTPP_WARNINGS} /EHsc)
if(DEFINED _utpp_mdmotion_data_native)
  set(_utpp_gu_real "${_utpp_mdmotion_data_native}/getup")
else()
  set(_utpp_gu_real "-")
endif()
add_test(NAME utpp_ragdoll_getup_test COMMAND utpp_ragdoll_getup_test "${CMAKE_CURRENT_SOURCE_DIR}/tests/data" "${_utpp_gu_real}")
foreach(_utpp_gu_mut IN ITEMS 1 2 3 4 6 7 9 10 11 12 13)
  add_executable(utpp_ragdoll_getup_mut${_utpp_gu_mut} tests/RagdollGetUpTest.cpp src/ragdoll/GetUpPlayer.hpp src/ragdoll/RdLastPose.hpp src/mm/MotionAdapter.hpp)
  target_compile_features(utpp_ragdoll_getup_mut${_utpp_gu_mut} PRIVATE cxx_std_20)
  target_compile_options(utpp_ragdoll_getup_mut${_utpp_gu_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_ragdoll_getup_mut${_utpp_gu_mut} PRIVATE UTPP_GETUP_MUTATION=${_utpp_gu_mut})
  add_test(NAME utpp_ragdoll_getup_mut${_utpp_gu_mut} COMMAND utpp_ragdoll_getup_mut${_utpp_gu_mut} "${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
endforeach()
add_executable(utpp_ragdoll_getup_mut5 tests/RagdollGetUpTest.cpp src/ragdoll/GetUpPlayer.hpp src/ragdoll/RdLastPose.hpp src/mm/MotionAdapter.hpp)
target_compile_features(utpp_ragdoll_getup_mut5 PRIVATE cxx_std_20)
target_compile_options(utpp_ragdoll_getup_mut5 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_ragdoll_getup_mut5 PRIVATE UTPP_RAGDOLL_RECOVER_MUTATION=5)
add_test(NAME utpp_ragdoll_getup_mut5 COMMAND utpp_ragdoll_getup_mut5 "${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
set_tests_properties(utpp_ragdoll_getup_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL classifier")
set_tests_properties(utpp_ragdoll_getup_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL plan: thrown-out hands")
set_tests_properties(utpp_ragdoll_getup_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL clamp")
set_tests_properties(utpp_ragdoll_getup_mut4 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL anchoring")
set_tests_properties(utpp_ragdoll_getup_mut5 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL recover: a 1.2 m drop")
set_tests_properties(utpp_ragdoll_getup_mut6 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL last pose: a smaller n")
set_tests_properties(utpp_ragdoll_getup_mut7 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL phases")
set_tests_properties(utpp_ragdoll_getup_mut9 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL release target")
set_tests_properties(utpp_ragdoll_getup_mut10 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL release: a deficit that vanishes at once")
set_tests_properties(utpp_ragdoll_getup_mut11 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL hand-off step: with the step")
set_tests_properties(utpp_ragdoll_getup_mut12 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL hand step: held r13d release pose, 60 fps: from 0.07 s")
set_tests_properties(utpp_ragdoll_getup_mut13 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL blend lift: seq 9 reproduction 60 fps: during the blend")

# the native get-up through the real v1 host (MotionHost.cpp Feed + MotionDriver.cpp glue): Plan / Begin on the woman bundle with a synthetic
# utpp_getup_woman.utpptrav, first-feed gate / weight, phases, world anchoring under a capsule jump, abort tail. Needs the motion-derived data dir.
if(DEFINED _utpp_mdmotion_data_native)
    add_executable(utpp_ragdoll_getup_host_test tests/RagdollGetUpHostTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_compile_features(utpp_ragdoll_getup_host_test PRIVATE cxx_std_20)
    target_compile_options(utpp_ragdoll_getup_host_test PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_ragdoll_getup_host_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
    target_link_libraries(utpp_ragdoll_getup_host_test PRIVATE bcrypt)
    add_test(NAME utpp_ragdoll_getup_host_test COMMAND utpp_ragdoll_getup_host_test "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}" "${_utpp_mdmotion_data_native}/getup")
    # release continuity (r13b) mutations, each MUST fail: 1 the v2 gap write is off (the game pose shows in the first publish after Begin),
    # 2 the gap publish ends the first-Step grace (a 0.2 s hitch aborts the run), 3 no v1 publish after Begin (MotionHost.cpp PublishGetUpGap)
    # 9 (GetUpPlayer.hpp GuTarget): a stale pre-teleport feed sample is trusted
    foreach(_utpp_gap_mut IN ITEMS 1 2 3)
        add_executable(utpp_ragdoll_getup_host_gap${_utpp_gap_mut} tests/RagdollGetUpHostTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
        target_compile_features(utpp_ragdoll_getup_host_gap${_utpp_gap_mut} PRIVATE cxx_std_20)
        target_compile_options(utpp_ragdoll_getup_host_gap${_utpp_gap_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
        target_compile_definitions(utpp_ragdoll_getup_host_gap${_utpp_gap_mut} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV UTPP_MOTION_DRIVER_GAP_MUTATION=${_utpp_gap_mut})
        target_link_libraries(utpp_ragdoll_getup_host_gap${_utpp_gap_mut} PRIVATE bcrypt)
        add_test(NAME utpp_ragdoll_getup_host_gap${_utpp_gap_mut} COMMAND utpp_ragdoll_getup_host_gap${_utpp_gap_mut} "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}" "${_utpp_mdmotion_data_native}/getup")
    endforeach()
    # utpp_ragdoll_getup_host_stale (UTPP_GETUP_MUTATION=9) removed 2026-11-05 (review R10): GetUpPlayer.hpp site killed by utpp_ragdoll_getup_mut9
    set_tests_properties(utpp_ragdoll_getup_host_gap1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL the real base publish right after Begin")
    set_tests_properties(utpp_ragdoll_getup_host_gap2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL a 0.2 s hitch")
    set_tests_properties(utpp_ragdoll_getup_host_gap3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL the real base publish right after Begin")
endif()

# review fix: the ragdollRecover guard has a downward-speed condition (the last 0.5 m of a real fall stays air); mutation 8 removes it
add_executable(utpp_ragdoll_getup_mut8 tests/RagdollGetUpTest.cpp src/ragdoll/GetUpPlayer.hpp src/ragdoll/RdLastPose.hpp src/mm/MotionAdapter.hpp)
target_compile_features(utpp_ragdoll_getup_mut8 PRIVATE cxx_std_20)
target_compile_options(utpp_ragdoll_getup_mut8 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_ragdoll_getup_mut8 PRIVATE UTPP_RAGDOLL_RECOVER_MUTATION=8)
add_test(NAME utpp_ragdoll_getup_mut8 COMMAND utpp_ragdoll_getup_mut8 "${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
set_tests_properties(utpp_ragdoll_getup_mut8 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL recover: the last 0.5 m")

# ---- melee full body + melee aim tilt (2026-11-09; self-contained block) ----
# MeleeFullBody.hpp: lower-body handover decision (MMDriver Feed); MeleeAim.hpp/.cpp: spine pitch / yaw layer in the FK detour.
target_sources(UTPP PRIVATE src/mm/MeleeAim.cpp src/mm/MeleeAim.hpp src/mm/MeleeAimRuntime.hpp src/mm/MeleeFullBody.hpp)
add_executable(utpp_melee_fullbody_test tests/MeleeFullBodyTest.cpp src/mm/MeleeFullBody.hpp src/mm/GameIntegration.hpp)
target_compile_options(utpp_melee_fullbody_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_melee_fullbody_test COMMAND utpp_melee_fullbody_test)
add_executable(utpp_melee_aim_test tests/MeleeAimTest.cpp src/mm/MeleeAim.hpp src/mm/AimOffset.hpp src/mm/GameIntegration.hpp
               src/mm/MMMath.hpp)
target_compile_options(utpp_melee_aim_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_melee_aim_test COMMAND utpp_melee_aim_test)

# ---- melee chest base correction (2026-11-13; self-contained block) ----
# MeleeBase.hpp: clip-frame match + chest rotation back to the authored orientation (core); MeleeBase.cpp: FK detour runtime;
# MeleeBaseData.cpp: GENERATED fists clip table (tools/melee/fist_base_table.py). utpp_melee_base_offline replays recorded poses
# through the production glue (tools/melee/fist_base_offline.py); utpp_melee_base_glue_test drives the same glue with recorded fixtures
# (tests/data/melee_base_*.bin, made by tools/melee/fist_base_fixture.py): built only on request.
target_sources(UTPP PRIVATE src/mm/MeleeBase.cpp src/mm/MeleeBase.hpp src/mm/MeleeBaseRuntime.hpp src/mm/MeleeBaseData.cpp
               src/mm/MeleeBaseData.hpp)
add_executable(utpp_melee_base_test tests/MeleeBaseTest.cpp src/mm/MeleeBaseData.cpp src/mm/MeleeBase.hpp src/mm/MeleeBaseData.hpp
               src/mm/MeleeAim.hpp src/mm/AimOffset.hpp src/mm/GameIntegration.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_melee_base_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_melee_base_test COMMAND utpp_melee_base_test)
add_executable(utpp_melee_base_glue_test tests/MeleeBaseGlueTest.cpp tests/MeleeBaseReplay.hpp src/mm/MeleeBase.cpp src/mm/MeleeBaseData.cpp
               src/mm/MeleeBase.hpp src/mm/MeleeBaseData.hpp src/mm/MeleeBodyId.hpp src/mm/MeleeBaseRuntime.hpp src/mm/MeleeAim.hpp
               src/mm/AimOffset.hpp src/mm/GameIntegration.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_melee_base_glue_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_melee_base_glue_test COMMAND utpp_melee_base_glue_test "${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
add_executable(utpp_melee_base_offline EXCLUDE_FROM_ALL tests/MeleeBaseOffline.cpp tests/MeleeBaseReplay.hpp src/mm/MeleeBase.cpp
               src/mm/MeleeBaseData.cpp src/mm/MeleeBase.hpp src/mm/MeleeBaseData.hpp src/mm/MeleeBodyId.hpp src/mm/MeleeBaseRuntime.hpp
               src/mm/MeleeAim.hpp src/mm/AimOffset.hpp src/mm/GameIntegration.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_melee_base_offline PRIVATE ${UTPP_WARNINGS} /EHsc)
# ---- head look layer: Spine3 / Neck / Neck1 / Head and the eyes toward the target of utpp_core (self-contained block) ----
# HeadLook.hpp: gaze, spring, distribution, limits, eyes (header-only core); HeadLook.cpp: FK detour glue (after the melee tilt).
target_sources(UTPP PRIVATE src/mm/HeadLook.cpp src/mm/HeadLook.hpp src/mm/HeadLookRuntime.hpp)
add_executable(utpp_headlook_test tests/HeadLookTest.cpp src/mm/HeadLook.cpp src/mm/HeadLookRuntime.hpp src/mm/HeadLook.hpp src/mm/AimOffset.hpp src/mm/GameIntegration.hpp
               src/mm/MMMath.hpp)
target_compile_options(utpp_headlook_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_headlook_test COMMAND utpp_headlook_test)
# replay on recorded in-game poses (fixture exported by tools/analysis/head_look_replay.py; the executable is also the tool of that script)
add_executable(utpp_headlook_replay tests/HeadLookReplay.cpp src/mm/HeadLook.hpp src/mm/AimOffset.hpp src/mm/GameIntegration.hpp
               src/mm/MMMath.hpp)
target_compile_options(utpp_headlook_replay PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_headlook_replay_test COMMAND utpp_headlook_replay "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/headlook_replay_frames.txt")

# ---- staging package: the rest of the runtime scripts + the camera CET module (2026-11-09; self-contained block) ----
# The targets above stage only part of the scripts, and the package did not compile on its own: utpp.reds calls utpp_ragdoll.reds
# (UTPPRgBusy / UTPPRgKnockdownFeed), utpp_ragdoll.reds calls utpp_vehicle_exit.reds (UTPPExitSpotFree), utpp_vehicle_approach.reds
# calls utpp_vehicle_anim.reds. Added: the rest of the release core list (tools/release/make_release.py CODE), the melee script
# (scripts/utpp_melee.reds) and the camera CET module (shipped as core by the release; package_camera keeps its own copy with the
# camera archive). Not staged here: probes and test profiles (scripts/utpp_*_probe.reds, cet/utpp_*_probe, cet/utpp_probe,
# cet/utpp_local_test_profile: test installs only); the optional authored reload (scripts/utpp_reload_gate.reds + cet/utpp_reload_gate:
# it assumes utpp_nue_reload.archive is installed, make_release.py ships it as its own group); scripts/utpp_vehicle_door.reds +
# utpp_vehicle_door_timing.reds (door hold for the car entry clips of utpp_vehicle_car.archive, not part of the release yet).
set(_utpp_pkg_rt_reds)
foreach(_n utpp_authored_recoil utpp_gamelook_exit utpp_melee utpp_ragdoll utpp_vehicle_anim utpp_vehicle_exit)
  list(APPEND _utpp_pkg_rt_reds "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_n}.reds")
endforeach()
set(_utpp_pkg_rt_camera "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_camera/init.lua" "${CMAKE_CURRENT_SOURCE_DIR}/cet/utpp_camera/camera_math.lua")
add_custom_target(utpp_package_runtime ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_utpp_pkg_rt_reds} "${UTPP_PACKAGE_DIR}/r6/scripts/UTPP/"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_camera"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_utpp_pkg_rt_camera}
          "${UTPP_PACKAGE_DIR}/bin/x64/plugins/cyber_engine_tweaks/mods/utpp_camera/"
  DEPENDS ${_utpp_pkg_rt_reds} ${_utpp_pkg_rt_camera}
  VERBATIM)
add_dependencies(utpp_package_runtime utpp_package) # package folders and the DLL staged first

# ---- Test-only host plugin UTPPTestHost (background pad input for automated test runs; test installs only) ----
# Never part of a release package and not in the default build: build it explicitly (target UTPPTestHost).
add_library(UTPPTestHost SHARED EXCLUDE_FROM_ALL testhost/TestHost.cpp testhost/TestHostCore.hpp)
target_link_libraries(UTPPTestHost PRIVATE utpp_red4ext_sdk version)
target_compile_definitions(UTPPTestHost PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
target_compile_options(UTPPTestHost PRIVATE ${UTPP_WARNINGS} /EHsc)
target_link_options(UTPPTestHost PRIVATE /INCREMENTAL:NO /PDBALTPATH:%_PDB%)
set_target_properties(UTPPTestHost PROPERTIES OUTPUT_NAME "UTPPTestHost")
add_executable(utpp_testhost_site_test tests/TestHostSiteTest.cpp testhost/TestHostCore.hpp)
target_compile_options(utpp_testhost_site_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_testhost_site_test COMMAND utpp_testhost_site_test "${UTPP_EXE_COPY}")

# ---- ctest labels (area / mutation / base / slow); keep this LAST so every test above is labelled ----
include(${CMAKE_CURRENT_SOURCE_DIR}/tests/labels.cmake)
