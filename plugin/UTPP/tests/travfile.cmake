# ---- port stage 10.8 part 2: .utpptrav reader (src/mm/TraversalFile.hpp) + SolveWarp on real montages.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/travfile.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Data files are motion-derived (outside git): tools/mmbake/travbake.py -> Data/traversal/utpp_traversal_{woman,man}.utpptrav.
# Defaults: <root>/Data (worktrees below Build/ see it through the Build/Data junction);
# a missing file makes the test FAIL (no synthetic fallback).
set(_utpp_trav_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/traversal")
set(UTPP_TRAV_WOMAN_FILE "${_utpp_trav_dir}/utpp_traversal_woman.utpptrav"
    CACHE FILEPATH "Baked traversal montages, woman rig (travbake.py)")
set(UTPP_TRAV_MAN_FILE "${_utpp_trav_dir}/utpp_traversal_man.utpptrav"
    CACHE FILEPATH "Baked traversal montages, man rig (travbake.py)")
# trav: FORMAT_TRAVERSAL version 2 files (travbake.py --format 2 --variant knee), arguments 3 / 4
set(UTPP_TRAV_WOMAN_MOTION_FILE "${_utpp_trav_dir}/utpp_traversal_woman_motion.utpptrav"
    CACHE FILEPATH "Baked traversal montages + poses (format 2), woman rig (travbake.py --format 2)")
set(UTPP_TRAV_MAN_MOTION_FILE "${_utpp_trav_dir}/utpp_traversal_man_motion.utpptrav"
    CACHE FILEPATH "Baked traversal montages + poses (format 2), man rig (travbake.py --format 2)")
set(_utpp_travfile_test_args)
foreach(_utpp_trav_var UTPP_TRAV_WOMAN_FILE UTPP_TRAV_MAN_FILE UTPP_TRAV_WOMAN_MOTION_FILE UTPP_TRAV_MAN_MOTION_FILE)
  set(_utpp_trav_path "${${_utpp_trav_var}}")
  if(_utpp_trav_path STREQUAL "")
    set(_utpp_trav_path "${_utpp_trav_var}-NOT-SET")
  endif()
  if(NOT EXISTS "${_utpp_trav_path}")
    message(WARNING "${_utpp_trav_var} = '${_utpp_trav_path}' does not exist: utpp_mm_travfile_test will FAIL")
  endif()
  list(APPEND _utpp_travfile_test_args "${_utpp_trav_path}")
endforeach()
add_executable(utpp_mm_travfile_test tests/TraversalFileTest.cpp src/mm/TraversalFile.hpp src/mm/BaseTraversal.hpp)
target_compile_options(utpp_mm_travfile_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_travfile_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_travfile_test COMMAND utpp_mm_travfile_test ${_utpp_travfile_test_args})
