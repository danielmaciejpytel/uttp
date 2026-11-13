# ---- fix3/hipfire-weapons review 3: a key change inside / after the hip raise (P9 A2) - mutation builds of the overlay test. Included from
# plugin/UTPP/CMakeLists.txt after hipfireweapons.cmake; expects UTPP_WARNINGS. Mutation builds MUST report an assertion failure: they pass
# only when the output contains a "FAIL: hipfire P9 A2" line (PASS_REGULAR_EXPRESSION ignores the exit code).
# UTPP_OVL_HIPFIRE_MUTATION: 4 = the hands guard of a running fade also applies to the fades after the raise (the 0.75 s hold of the hip pose:
# the second key change inside the 0.2 s fade takes its remainder, a whole pose switch in one frame); 5 = no floor (a key change in the last
# frames of the raise snaps in one frame).
foreach(_m 4 5)
    add_executable(utpp_mm_overlay_hipraise_mut${_m} tests/OverlayTest.cpp src/mm/Additive.hpp src/mm/Overlay.hpp src/mm/OverlayFile.hpp
                   src/mm/HipFirePose.hpp)
    target_compile_options(utpp_mm_overlay_hipraise_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_mm_overlay_hipraise_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_mm_overlay_hipraise_mut${_m} PRIVATE UTPP_OVL_HIPFIRE_MUTATION=${_m})
    add_test(NAME utpp_mm_overlay_hipraise_mut${_m} COMMAND utpp_mm_overlay_hipraise_mut${_m})
    set_tests_properties(utpp_mm_overlay_hipraise_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL: hipfire P9 A2")
endforeach()
