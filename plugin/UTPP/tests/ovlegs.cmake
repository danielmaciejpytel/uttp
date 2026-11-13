# ---- Armed lower-body layering through the stance fades (ov::ComposeLayers legs / pelvis per stance side + PoseEvaluator::KeepLowerFootHeight;
# recording 2026-10-31 22:55: the composed Pistol2H ankles sank 3 cm below the base pose on crouch -> stand). The real diag_b game feed through
# the production stack (tests/LocoStanceTest.cpp, OVLEGS-* checks): composed vs base ankle height across the stance edges, no ankle step,
# knees forward, foot lift off outside the fades.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/ovlegs.cmake OPTIONAL)) after locostance.cmake, whose data lookup and test arguments it
# reuses. Mutations (UTPP_OVLEGS_MUTATION=N in src/mm/Overlay.hpp + src/mm/MotionPose.hpp) must each print their own FAIL line
# (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error cannot count as a killed mutant):
#   1 = legs / pelvis on the single OPLower inner and no foot lift (pre-fix: 3.0 cm sink at 69.236);
#   2 = legs / pelvis per stance side but no foot lift (2.5 cm of the stand side's nonlinearity left at 69.236).
if(DEFINED _utpp_ls_args)
    set(_utpp_ol_expect_1 "FAIL OVLEGS-1 armed edge 69[.]236: the composed ankles stay within 1[.]5 cm")
    set(_utpp_ol_expect_2 "FAIL OVLEGS-1 armed edge 69[.]236: the composed ankles stay within 1[.]5 cm")
# removed 2026-11-05 (review R12), which cheaper test kills the same site: utpp_mm_ovlegs_mut2
    foreach(_utpp_ol_mut 2)
        add_executable(utpp_mm_ovlegs_mut${_utpp_ol_mut} tests/LocoStanceTest.cpp src/mm/Overlay.hpp src/mm/MotionPose.hpp src/mm/ChooserMatcher.hpp)
        target_compile_features(utpp_mm_ovlegs_mut${_utpp_ol_mut} PRIVATE cxx_std_20)
        target_compile_options(utpp_mm_ovlegs_mut${_utpp_ol_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
        target_compile_definitions(utpp_mm_ovlegs_mut${_utpp_ol_mut} PRIVATE UTPP_OVLEGS_MUTATION=${_utpp_ol_mut})
        target_link_libraries(utpp_mm_ovlegs_mut${_utpp_ol_mut} PRIVATE bcrypt)
        add_test(NAME utpp_mm_ovlegs_mut${_utpp_ol_mut} COMMAND utpp_mm_ovlegs_mut${_utpp_ol_mut} ${_utpp_ls_args})
        set_tests_properties(utpp_mm_ovlegs_mut${_utpp_ol_mut} PROPERTIES PASS_REGULAR_EXPRESSION "${_utpp_ol_expect_${_utpp_ol_mut}}")
    endforeach()
endif()
