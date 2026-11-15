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

# Knee hinge of the legs region (tests/OverlayLegsHingeTest.cpp): stop / start frames of both motion databases composed with the
# weapon pose sets in the three legs forms; the motion databases sit one folder above the overlay files. A missing file fails it.
# Comparison tool for now, built only on request (EXCLUDE_FROM_ALL, no ctest entry): the runtime keeps the Mesh form and the Hinge
# form does not pass OVLEGS-HINGE-2 (settled stance) yet. Run: utpp_mm_overlay_legs_hinge_test <woman.utppmm> <woman.utppov>
# <man.utppmm> <man.utppov> [--all]. Mutation 1 (UTPP_OVLEGS_HINGE_MUTATION=1 in src/mm/Overlay.hpp) = the Mesh form whatever the
# input asks for; it must print "FAIL OVLEGS-HINGE-1 man Rifle Relax". Register both with add_test once a form is chosen.
get_filename_component(_utpp_olh_wdir "${UTPP_OVERLAY_WOMAN_FILE}" DIRECTORY)
get_filename_component(_utpp_olh_mdir "${UTPP_OVERLAY_MAN_FILE}" DIRECTORY)
set(_utpp_olh_args "${_utpp_olh_wdir}/../utpp_motion_woman.utppmm" "${UTPP_OVERLAY_WOMAN_FILE}"
    "${_utpp_olh_mdir}/../utpp_motion_man.utppmm" "${UTPP_OVERLAY_MAN_FILE}")
foreach(_utpp_olh_mut 0 1)
  if(_utpp_olh_mut EQUAL 0)
    set(_utpp_olh_name utpp_mm_overlay_legs_hinge_test)
  else()
    set(_utpp_olh_name utpp_mm_overlay_legs_hinge_mut${_utpp_olh_mut})
  endif()
  add_executable(${_utpp_olh_name} EXCLUDE_FROM_ALL tests/OverlayLegsHingeTest.cpp src/mm/Additive.hpp src/mm/Overlay.hpp
                 src/mm/OverlayFile.hpp src/mm/MotionDatabase.hpp)
  target_compile_options(${_utpp_olh_name} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(${_utpp_olh_name} PRIVATE cxx_std_20)
  if(NOT _utpp_olh_mut EQUAL 0)
    target_compile_definitions(${_utpp_olh_name} PRIVATE UTPP_OVLEGS_HINGE_MUTATION=${_utpp_olh_mut})
  endif()
endforeach()

# Swing gate of the legs layer (tests/OverlayLegsSwingTest.cpp, src/mm/Overlay.hpp LegSwingGate): standing stop / start clips of both
# motion databases at 60 Hz through the overlay crossfade of the weapon pose sets, ungated against gated. Same data files as above.
# Mutation 1 (UTPP_OVLEGS_SWING_MUTATION=1: no gate) must print a FAIL line.
foreach(_utpp_ols_mut 0 1)
  if(_utpp_ols_mut EQUAL 0)
    set(_utpp_ols_name utpp_mm_overlay_legs_swing_test)
  else()
    set(_utpp_ols_name utpp_mm_overlay_legs_swing_mut${_utpp_ols_mut})
  endif()
  add_executable(${_utpp_ols_name} tests/OverlayLegsSwingTest.cpp src/mm/Additive.hpp src/mm/Overlay.hpp src/mm/OverlayFile.hpp
                 src/mm/MotionDatabase.hpp)
  target_compile_options(${_utpp_ols_name} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(${_utpp_ols_name} PRIVATE cxx_std_20)
  add_test(NAME ${_utpp_ols_name} COMMAND ${_utpp_ols_name} ${_utpp_olh_args})
  if(NOT _utpp_ols_mut EQUAL 0)
    target_compile_definitions(${_utpp_ols_name} PRIVATE UTPP_OVLEGS_SWING_MUTATION=${_utpp_ols_mut})
    set_tests_properties(${_utpp_ols_name} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL OVLEGS-SWING-1")
  endif()
endforeach()

# Bounded first-shot capture grace must not flash native FPP arms before transfer.
add_executable(utpp_mm_hipfire_warmup_test tests/HipFireWarmupTest.cpp)
target_compile_options(utpp_mm_hipfire_warmup_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_hipfire_warmup_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_hipfire_warmup_test COMMAND utpp_mm_hipfire_warmup_test)
