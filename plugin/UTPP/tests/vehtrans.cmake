# ---- vehicle batch: mount / exit pose transition (src/mm/VehicleTransition.hpp, header-only) offline test.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/vehtrans.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Mutations UTPP_VEHTRANS_MUTATION MUST fail the named groups: 1 = mount blend off -> (a)(b); 2 = root offset sign flipped -> (d);
# 3 = hips height offset kept -> (e); 4 = no main mapping job filter (helper jobs write again) -> (j); 5 = no bone roles from the
# game rig (per-bone 25 deg fallback) -> (k); 6 = a perspective denial always ends the mount transition (vehicle hold veto) -> (l); 7 = no seat pose switch guard -> (s);
# 8 = raw per-frame steps in the seat guard / pop filter (no frame-gap rates, pop re-fire without the freeze guard) -> (p)(s);
# 9 = the seat guard follows its TPP latch only (no current mounted-TPP check) -> (s); recording 2026-10-28 19-55-01: 10 = no
# capture under the perspective denial -> (t); 11 = no exit latch -> (u); 12 = no Fill for a third-person exit without a capture
# -> (v); 13 = the standing leg cap without its gate -> (x); 14 = no standing entry path -> (w); 15 = no cumulative root tracking
# -> (y); 16 = no Hips translation term in the pop filter -> (z); recording 2026-10-28 23-05-01: 17 = the planar root offset
# decays with the root-yaw progress -> (A); recording 2026-10-29 00-29-45: 18 = the leg offsets kept until the legs move -> (B);
# recording 2026-10-29 01-28-54: 19 = no seated neck cut branch in the seat guard -> (C);
# recording 2026-11-01 07-50-12: 20 = the old cumulative yaw hold (no rate engage, no yaw align) -> (D);
# 21 = the neck fire counted as a seat guard fire (shared counter / fields of the ride line, 2026-11-01) -> (C);
# recording 2026-11-02 02:01-02:50 (car doors): 22 = the per-mount hold cap (SetHoldCap) is ignored -> (E);
# 23 = the rendered Hips root offset ignores the cap (natural factor; RootOffset still reports it) -> (E);
# 24 = the jump path asks its cap gates again at every jump sample (a working cap is dropped) -> (E);
# 25 = the capped cumulative release keeps tracking the anchor instead of freezing the planar offset at the release start -> (E);
# 2026-11-03 (car entry glide limit, VehicleTransition.hpp GLIDE LIMIT): 26 = no glide limit (1.2 m clamp, no slide follow, fixed 0.30 / 0.60 s releases) -> (F).
# review r2 (findings 17 / 20, late door caps against the glided release, glided cumulative tail): 27 = the jump cap gate against kRootSeconds, 28 = a re-base after the
# cap drops the speed-limited release, 29 = the cumulative skip / clamp against kRootTailSeconds, 30 = the world-fixed re-publish window ends at 1.05 s -> (G).
# The test prints "failed groups: [a][b]..." as its last line. Each mutation test expects the FIRST FAIL line of its group(s) (the text the test
# prints for it; a mutant with two groups: both, in output order), not the final summary, so the early exit of tests/TestFailFast.hpp can stop it at that line.
# Mutants are RUN-TIME: one executable (utpp_mm_vehtrans_rt) reads UTPP_MUTATION_VEHTRANS (src/MutationSelect.hpp), ctest runs it once per mutation.
add_executable(utpp_mm_vehtrans_test tests/MMVehicleTransitionTest.cpp src/mm/VehicleTransition.hpp)
target_compile_features(utpp_mm_vehtrans_test PRIVATE cxx_std_20)
target_compile_options(utpp_mm_vehtrans_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_mm_vehtrans_test COMMAND utpp_mm_vehtrans_test)
add_executable(utpp_mm_vehtrans_rt tests/MMVehicleTransitionTest.cpp src/mm/VehicleTransition.hpp src/MutationSelect.hpp)
target_compile_features(utpp_mm_vehtrans_rt PRIVATE cxx_std_20)
target_compile_options(utpp_mm_vehtrans_rt PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_mm_vehtrans_rt PRIVATE UTPP_RUNTIME_MUTATION_VEHTRANS)
set(_vt_mut_re_1 "FAIL .a. cut-frame max per-bone jump .= 15 deg.*FAIL .b. offsets start at the full A-B difference .. 40 deg. after the")
set(_vt_mut_re_2 "FAIL .d. root offset = -.world jump. on the cut frame")
set(_vt_mut_re_3 "FAIL .e. hips height offset 0 after the hold .game hips .0.45 m.")
set(_vt_mut_re_4 "FAIL .j. helper jobs return 0")
set(_vt_mut_re_5 "FAIL .k. roles from the game names: Hips found")
set(_vt_mut_re_6 "FAIL .l. vehicle veto: the hold / offset frames are written through the")
set(_vt_mut_re_7 "FAIL .s. the swap fires the guard once")
set(_vt_mut_re_8 "FAIL .s. hitch on the enter clip .99 ms. Hips 21 cm.: no write before.*FAIL .p. ordinary motion through a frame hitch is not a pop .no fire.")
set(_vt_mut_re_9 "FAIL .s. FPP switch 0.61 s after the mount. sustained .the swap 1.13 s")
set(_vt_mut_re_10 "FAIL .t. T1 vehicle veto: the ride capture runs through the denial once")
set(_vt_mut_re_11 "FAIL .u. T2 the edge hands in the standing capture .not the swap.")
set(_vt_mut_re_12 "FAIL .v. T3 third-person ride under the vehicle hold. no capture: Fill")
set(_vt_mut_re_13 "FAIL .x. T5 car seat entry: the 25 deg rule .bent leg chain on the game")
set(_vt_mut_re_14 "FAIL .w. T4 standing entry: the 39 deg knee keeps its offset")
set(_vt_mut_re_15 "FAIL .y. T6 sub-threshold slide samples produce a root offset")
set(_vt_mut_re_16 "FAIL .z. T7 translation-only flash: 2 Hips fires. shown .= 3 cm / 10")
set(_vt_mut_re_17 "FAIL .A. T8 the planar offset leaves at .= 3.0 cm per frame in the")
set(_vt_mut_re_18 "FAIL .B. T9 after the leg decay the epoch writes no leg .and no bone")
set(_vt_mut_re_19 "FAIL .C. ride 1: the FK-stage guard runs on the ride frames after the")
set(_vt_mut_re_20 "FAIL .D. D1 car ride 3: the yaw align latched early")
set(_vt_mut_re_21 "FAIL .C. ride 1: a neck fire does not count as a seat guard fire. does")
set(_vt_mut_re_22 "FAIL .E. E1 the cap acts on exactly rides .4. 9. 25. 30. 109. 129. 131.")
set(_vt_mut_re_23 "FAIL .E. E1 ride [0-9]+: rendered Hips difference equals the RootOffset")
set(_vt_mut_re_24 "FAIL .E. E6 the first jump sample applies the cap on the jump path")
set(_vt_mut_re_25 "FAIL .E. E2 ride [0-9]+: the held body closes on the entity at less than 4")
set(_vt_mut_re_26 "FAIL .F. F ride [0-9]+ .arc.: the held offset closes on the entity at .=")
set(_vt_mut_re_27 "FAIL .G. G1 a late cap is applied against the glided release")
set(_vt_mut_re_28 "FAIL .G. G2 the release is speed-limited. bounded by the cap and not")
set(_vt_mut_re_29 "FAIL .G. G3 a cap behind kRootTrackSeconds . kRootTailSeconds but")
set(_vt_mut_re_30 "FAIL .G. G4 the held offset stays fixed in the world while the entity")
foreach(_m 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30)
    add_test(NAME utpp_mm_vehtrans_mut${_m} COMMAND utpp_mm_vehtrans_rt)
    set_tests_properties(utpp_mm_vehtrans_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "${_vt_mut_re_${_m}}"
                         ENVIRONMENT "UTPP_MUTATION_VEHTRANS=${_m}")
endforeach()
