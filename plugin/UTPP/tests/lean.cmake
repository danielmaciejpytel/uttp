# motion lean / head-turn pose pack "utpp.lean/1": reader + blend space evaluator offline test (src/mm/LeanPack.hpp).
# Included from plugin/UTPP/CMakeLists.txt (include(tests/lean.cmake)); expects UTPP_WARNINGS to be defined.
# The packs are motion-derived (outside git), baked by Source/tools/mmbake/leanbake.py into <root>/Data. A missing file
# makes the test FAIL (load check); there is no synthetic fallback. Worktree builds pass the real paths with
# -DUTPP_LEAN_WOMAN_BIN / -DUTPP_LEAN_WOMAN_JSON / -DUTPP_LEAN_MAN_BIN.
set(UTPP_LEAN_WOMAN_BIN "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_lean_woman.bin"
    CACHE FILEPATH "Baked lean pack (leanbake.py) for the woman rig")
set(UTPP_LEAN_WOMAN_JSON "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_lean_woman.json"
    CACHE FILEPATH "JSON header of the woman lean pack")
set(UTPP_LEAN_MAN_BIN "${CMAKE_CURRENT_SOURCE_DIR}/../../../Data/utpp_lean_man.bin"
    CACHE FILEPATH "Baked lean pack (leanbake.py) for the man rig")
foreach(_utpp_lean_var UTPP_LEAN_WOMAN_BIN UTPP_LEAN_WOMAN_JSON UTPP_LEAN_MAN_BIN)
  if(NOT EXISTS "${${_utpp_lean_var}}")
    message(WARNING "${_utpp_lean_var} = '${${_utpp_lean_var}}' does not exist: utpp_mm_lean_pack_test will FAIL "
                    "(pass -D${_utpp_lean_var}=<path>)")
  endif()
endforeach()
add_executable(utpp_mm_lean_pack_test tests/LeanPackTest.cpp src/mm/LeanPack.hpp src/mm/BlendSpace.hpp
               src/mm/Additive.hpp src/mm/MiniJson.hpp src/mm/MMMath.hpp)
target_compile_options(utpp_mm_lean_pack_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_lean_pack_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_lean_pack_test COMMAND utpp_mm_lean_pack_test "${UTPP_LEAN_WOMAN_BIN}" "${UTPP_LEAN_WOMAN_JSON}" "${UTPP_LEAN_MAN_BIN}")
