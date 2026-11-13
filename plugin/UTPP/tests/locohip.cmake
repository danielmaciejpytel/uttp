# ---- locomotion probe 2026-10-31, hipfire / crouch-stop jerks (HIP-1, HIP-2, CR-5): OverlayMachine + HipFirePoseHold offline test.
# Included from plugin/UTPP/CMakeLists.txt; expects UTPP_WARNINGS. Mutation builds (UTPP_LOCOHIP_MUTATION) MUST print a "FAIL: lochip"
# line (PASS_REGULAR_EXPRESSION ignores the exit code): 1 = the hip shot out of Relaxed chains Relaxed -> Ready -> Aiming (0.2 + 0.1 s),
# 2 = no re-raise from a partial Ready (the readyWeightToAim gate also inside an Aiming -> Ready fade), 3 = moveSwitch (0.1 s) for B in
# the crouch, 4 = the 0.75 s isolated hold, 5 = the sprint cut of the isolated hold also ends the vehicle exit-window tail.
foreach(_m 0 1 2 3 4 5)
    if(_m EQUAL 0)
        set(_tgt utpp_mm_locohip_test)
    else()
        set(_tgt utpp_mm_locohip_mut${_m})
    endif()
    add_executable(${_tgt} tests/LocoHipTest.cpp src/mm/Overlay.hpp src/mm/HipFirePose.hpp)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    add_test(NAME ${_tgt} COMMAND ${_tgt})
    if(NOT _m EQUAL 0)
        target_compile_definitions(${_tgt} PRIVATE UTPP_LOCOHIP_MUTATION=${_m})
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL: lochip")
    endif()
endforeach()
