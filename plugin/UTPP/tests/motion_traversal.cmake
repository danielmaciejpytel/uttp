# ---- port stage 10.8 prep: MotionTraversal (src/mm/MotionTraversal.hpp) - game climb / vault start -> motion montage.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/motion_traversal.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Data (motion-derived, outside git) is passed at run time by the worktree build script:
#   Data/traversal/traversal_table.json, Data/traversal/utpp_traversal_{woman,man}.utpptrav
add_executable(utpp_motion_traversal_test tests/MotionTraversalTest.cpp src/mm/MotionTraversal.hpp src/mm/TraversalFile.hpp
               src/mm/BaseTraversal.hpp src/mm/MiniJson.hpp)
target_compile_options(utpp_motion_traversal_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_motion_traversal_test PRIVATE cxx_std_20)
# Registered with ctest like travfile.cmake: data paths resolved by CMake (Data/traversal next to the main checkout or the
# worktree, or the UTPP_TRAV_* cache variables of travfile.cmake when that file is included first); a missing file FAILS.
if(NOT DEFINED _utpp_trav_dir)
  set(_utpp_trav_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/traversal")
endif()
set(UTPP_TRAV_TABLE_FILE "${_utpp_trav_dir}/traversal_table.json"
    CACHE FILEPATH "Neutral traversal decision table (CHT_TraversalAnims export)")
if(NOT DEFINED UTPP_TRAV_WOMAN_FILE)
  set(UTPP_TRAV_WOMAN_FILE "${_utpp_trav_dir}/utpp_traversal_woman.utpptrav"
      CACHE FILEPATH "Baked traversal montages, woman rig (travbake.py)")
endif()
if(NOT DEFINED UTPP_TRAV_MAN_FILE)
  set(UTPP_TRAV_MAN_FILE "${_utpp_trav_dir}/utpp_traversal_man.utpptrav"
      CACHE FILEPATH "Baked traversal montages, man rig (travbake.py)")
endif()
set(_utpp_motion_traversal_test_args)
foreach(_utpp_trav_var UTPP_TRAV_TABLE_FILE UTPP_TRAV_WOMAN_FILE UTPP_TRAV_MAN_FILE)
  set(_utpp_trav_path "${${_utpp_trav_var}}")
  if(_utpp_trav_path STREQUAL "")
    set(_utpp_trav_path "${_utpp_trav_var}-NOT-SET")
  endif()
  if(NOT EXISTS "${_utpp_trav_path}")
    message(WARNING "${_utpp_trav_var} = '${_utpp_trav_path}' does not exist: utpp_motion_traversal_test will FAIL")
  endif()
  list(APPEND _utpp_motion_traversal_test_args "${_utpp_trav_path}")
endforeach()
add_test(NAME utpp_motion_traversal_test COMMAND utpp_motion_traversal_test ${_utpp_motion_traversal_test_args})
# Vehicle batch (TRAV-1 B / C): mutation builds of src/mm/MotionTraversal.hpp (UTPP_TRAVV2_MUTATION 1 = no vehicle rule, 2 = no
# depth rule). They pass only when the output contains the assertion failure of the rule they remove, so a setup error or an
# unrelated failure cannot count as a killed mutant.
set(_utpp_travmotion_mut_expect1 "FAIL: [a-z]+ \\(5\\) vault over a vehicle")
set(_utpp_travmotion_mut_expect2 "FAIL: [a-z]+ \\(6\\) vault with obstacleEnd 1.60")
foreach(_m 1 2)
  add_executable(utpp_motion_traversal_mut${_m} tests/MotionTraversalTest.cpp src/mm/MotionTraversal.hpp src/mm/TraversalFile.hpp
                 src/mm/BaseTraversal.hpp src/mm/MiniJson.hpp)
  target_compile_options(utpp_motion_traversal_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_motion_traversal_mut${_m} PRIVATE cxx_std_20)
  target_compile_definitions(utpp_motion_traversal_mut${_m} PRIVATE UTPP_TRAVV2_MUTATION=${_m})
  add_test(NAME utpp_motion_traversal_mut${_m} COMMAND utpp_motion_traversal_mut${_m} ${_utpp_motion_traversal_test_args})
  set_tests_properties(utpp_motion_traversal_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "${_utpp_travmotion_mut_expect${_m}}")
endforeach()

# TR-dll male parity: the native get-up through the real v1 host on the MAN bundle (same exe as utpp_ragdoll_getup_host_test, CMakeLists.txt; the
# target is defined after this file is included, which add_test(NAME ... COMMAND <target>) resolves at generate time). Data dir = <root>/Data (the
# parent of traversal/), as tests/motiondriver.cmake derives it; the test needs the man v2 bundle and the man get-up file.
get_filename_component(_utpp_tr_data "${_utpp_trav_dir}/.." ABSOLUTE)
if(UTPP_MOTION_DB_TEST_FILE)
  get_filename_component(_utpp_tr_data "${UTPP_MOTION_DB_TEST_FILE}" DIRECTORY)
endif()
file(TO_NATIVE_PATH "${_utpp_tr_data}" _utpp_tr_data_native)
add_test(NAME utpp_ragdoll_getup_host_man_test COMMAND utpp_ragdoll_getup_host_test "${_utpp_tr_data_native}" "${UTPP_MMDB_TEST_FILE}"
         "${_utpp_tr_data_native}/getup" --body man)
# TR-dll male parity: the slide driver test on the man slide bundle (utpp_motion_man_knee_slide) and the plain man bundle; same exe as
# utpp_motion_driver_slide_test (CMakeLists.txt, target resolved at generate time).
file(TO_NATIVE_PATH "${UTPP_MOTION_FIXTURE_DIR}" _utpp_fixture_native)
add_test(NAME utpp_motion_driver_slide_man_test COMMAND utpp_motion_driver_slide_test "${_utpp_tr_data_native}" --body man
         --plain-dir "${_utpp_fixture_native}")
