# ---- port stage 10.9: motion idle-break player (src/mm/IdleBreaks.hpp) offline test.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/idlebreaks.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Data is motion-derived (outside git): the extracted chooser rules, the retargeted clips (woman + man) and the v2
# databases (name listing only). Defaults resolve from the main checkout layout (Source/plugin/UTPP -> <root>);
# the worktree build passes the real paths with -D (the worktree build script runs the test explicitly as well).
set(_utpp_ib_root "${CMAKE_CURRENT_SOURCE_DIR}/../../..")              # main checkout Source/plugin/UTPP
if(NOT EXISTS "${_utpp_ib_root}/Extracted/sourcepack/sourcepack/pseudo/choosers_rules.txt")
  set(_utpp_ib_root "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")          # worktree Build/wt-<name>/plugin/UTPP
endif()
set(UTPP_IDLEBREAKS_ROOT "${_utpp_ib_root}" CACHE PATH "Project root for utpp_mm_idlebreaks_test data")
if(NOT EXISTS "${UTPP_IDLEBREAKS_ROOT}/Extracted/sourcepack/sourcepack/pseudo/choosers_rules.txt")
  message(WARNING "UTPP_IDLEBREAKS_ROOT = '${UTPP_IDLEBREAKS_ROOT}' has no Extracted/sourcepack/sourcepack: utpp_mm_idlebreaks_test will FAIL")
endif()
add_executable(utpp_mm_idlebreaks_test tests/IdleBreaksTest.cpp src/mm/IdleBreaks.hpp src/mm/MiniJson.hpp)
target_compile_options(utpp_mm_idlebreaks_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_idlebreaks_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_idlebreaks_test COMMAND utpp_mm_idlebreaks_test
         --rules "${UTPP_IDLEBREAKS_ROOT}/Extracted/sourcepack/sourcepack/pseudo/choosers_rules.txt"
         --clips "${UTPP_IDLEBREAKS_ROOT}/Animations/retarget/out_sourcepack_woman"
         --clips "${UTPP_IDLEBREAKS_ROOT}/Animations/retarget/out_sourcepack_man"
         --db "${UTPP_IDLEBREAKS_ROOT}/Data/utpp_motion_woman.utppmm"
         --db "${UTPP_IDLEBREAKS_ROOT}/Data/utpp_motion_man.utppmm")
