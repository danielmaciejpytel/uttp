# ---- port stage 10.6: ALS layering runtime (Additive.hpp, Overlay.hpp, OverlayFile.hpp) offline test.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/overlay.cmake)); expects UTPP_WARNINGS to be defined.
# Data files are motion-derived (outside git). Both .utppov paths are always passed to the test: a missing or unset file
# makes the test FAIL (load check), it never falls back to a synthetic-only run. The defaults resolve from the main
# checkout layout (Source/plugin/UTPP -> <root>/Data); other checkouts (worktree Build/wt-<name>) pass the real paths
# with -DUTPP_OVERLAY_WOMAN_FILE / -DUTPP_OVERLAY_MAN_FILE (the worktree build script).
set(UTPP_OVERLAY_WOMAN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/motion/utpp_overlays_woman.utppov"
    CACHE FILEPATH "Baked .utppov (overlaybake.py) for the woman rig")
set(UTPP_OVERLAY_MAN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/motion/utpp_overlays_man.utppov"
    CACHE FILEPATH "Baked .utppov (overlaybake.py) for the man rig")
set(_utpp_overlay_test_args)
foreach(_utpp_ov_var UTPP_OVERLAY_WOMAN_FILE UTPP_OVERLAY_MAN_FILE)
  set(_utpp_ov_path "${${_utpp_ov_var}}")
  if(_utpp_ov_path STREQUAL "")
    set(_utpp_ov_path "${_utpp_ov_var}-NOT-SET")  # keeps the argument position; the test load check fails on it
  endif()
  if(NOT EXISTS "${_utpp_ov_path}")
    message(WARNING "${_utpp_ov_var} = '${_utpp_ov_path}' does not exist: utpp_mm_overlay_test will FAIL "
                    "(pass -D${_utpp_ov_var}=<path to the baked .utppov>)")
  endif()
  list(APPEND _utpp_overlay_test_args "${_utpp_ov_path}")
endforeach()
add_executable(utpp_mm_overlay_test tests/OverlayTest.cpp src/mm/Additive.hpp src/mm/Overlay.hpp src/mm/OverlayFile.hpp
               src/mm/MMMath.hpp)
target_compile_options(utpp_mm_overlay_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_overlay_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_overlay_test COMMAND utpp_mm_overlay_test ${_utpp_overlay_test_args})

# Bounded first-shot capture grace must not flash native FPP arms before transfer.
add_executable(utpp_mm_hipfire_warmup_test tests/HipFireWarmupTest.cpp)
target_compile_options(utpp_mm_hipfire_warmup_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_hipfire_warmup_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_hipfire_warmup_test COMMAND utpp_mm_hipfire_warmup_test)
