# motion lateral-acceleration lean layer runtime (src/mm/LeanRuntime.hpp, CET mm.lean): unit test on the real packs + overlays and the
# offline pose replay tool. Included from plugin/UTPP/CMakeLists.txt after tests/lean.cmake (it reuses UTPP_LEAN_WOMAN_BIN / UTPP_LEAN_MAN_BIN).
# The packs and overlays are motion-derived (outside git, Data\): a missing file makes the test FAIL (no synthetic fallback).
set(UTPP_LEAN_OVERLAY_WOMAN "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/motion/utpp_overlays_woman.utppov"
    CACHE FILEPATH "Overlay file of the woman rig (bone table check of the lean pack)")
set(UTPP_LEAN_OVERLAY_MAN "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/motion/utpp_overlays_man.utppov"
    CACHE FILEPATH "Overlay file of the man rig")
add_executable(utpp_leanruntime_test tests/LeanRuntimeTest.cpp src/mm/LeanRuntime.hpp src/mm/LeanPack.hpp src/mm/BlendSpace.hpp
               src/mm/Additive.hpp src/mm/OverlayFile.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_leanruntime_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_leanruntime_test PRIVATE cxx_std_20)
add_test(NAME utpp_leanruntime_test COMMAND utpp_leanruntime_test "${UTPP_LEAN_WOMAN_BIN}" "${UTPP_LEAN_MAN_BIN}"
         "${UTPP_LEAN_OVERLAY_WOMAN}" "${UTPP_LEAN_OVERLAY_MAN}")
add_executable(utpp_lean_replay EXCLUDE_FROM_ALL tests/LeanReplay.cpp src/mm/LeanRuntime.hpp src/mm/OrientWarp.hpp src/mm/HeadLead.hpp)
target_compile_options(utpp_lean_replay PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_lean_replay PRIVATE cxx_std_20)
