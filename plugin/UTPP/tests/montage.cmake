# ---- port report 4.2 #32-#34: blend space / DeadBlending / montage runtime offline test.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/montage.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Data files are motion-derived (outside git): tools/mmbake/export_blendspaces.py -> Data/anim/blendspaces.txt and
# Data/anim/montages.txt. Defaults: <root>/Data (worktrees below Build/ see it through the Build/Data junction); a missing file makes the test FAIL (no synthetic fallback).
set(_utpp_anim_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/anim")
set(UTPP_ANIM_BS_FILE "${_utpp_anim_dir}/blendspaces.txt"
    CACHE FILEPATH "Exported blend spaces + UE reference weights (export_blendspaces.py)")
set(UTPP_ANIM_MONTAGE_FILE "${_utpp_anim_dir}/montages.txt"
    CACHE FILEPATH "Exported traversal montages + root tracks (export_blendspaces.py)")
set(_utpp_montage_test_args)
foreach(_utpp_anim_var UTPP_ANIM_BS_FILE UTPP_ANIM_MONTAGE_FILE)
  set(_utpp_anim_path "${${_utpp_anim_var}}")
  if(_utpp_anim_path STREQUAL "")
    set(_utpp_anim_path "${_utpp_anim_var}-NOT-SET")
  endif()
  if(NOT EXISTS "${_utpp_anim_path}")
    message(WARNING "${_utpp_anim_var} = '${_utpp_anim_path}' does not exist: utpp_mm_montage_test will FAIL")
  endif()
  list(APPEND _utpp_montage_test_args "${_utpp_anim_path}")
endforeach()
add_executable(utpp_mm_montage_test tests/MontageTest.cpp src/mm/BlendSpace.hpp src/mm/DeadBlending.hpp src/mm/Montage.hpp)
target_compile_options(utpp_mm_montage_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_montage_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_montage_test COMMAND utpp_mm_montage_test ${_utpp_montage_test_args})
