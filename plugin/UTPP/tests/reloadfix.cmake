# Mutation builds MUST report an assertion failure: they pass only when the output contains a line starting "FAIL "
# (PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error - exit 2, missing data - cannot count as a killed mutant).
# ---- fix batch 1 (reload): offline tests of the reload-path policies. Included from plugin/UTPP/CMakeLists.txt
# (include(tests/reloadfix.cmake OPTIONAL)); expects UTPP_WARNINGS. No data files.
# Upper-body handback envelope (aim-interrupting reload ramp + overlay aim hold). The mutation build restores the old
# .12s ramp and MUST fail (WILL_FAIL).
add_executable(utpp_reload_upper_envelope_test tests/ReloadUpperEnvelopeTest.cpp src/mm/ReloadContactFade.hpp)
target_compile_options(utpp_reload_upper_envelope_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_upper_envelope_test PRIVATE cxx_std_20)
add_test(NAME utpp_reload_upper_envelope_test COMMAND utpp_reload_upper_envelope_test)
add_executable(utpp_reload_upper_envelope_mut1 tests/ReloadUpperEnvelopeTest.cpp src/mm/ReloadContactFade.hpp)
target_compile_options(utpp_reload_upper_envelope_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_upper_envelope_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_reload_upper_envelope_mut1 PRIVATE UTPP_RELOAD_ENVELOPE_MUTATION=1)
add_test(NAME utpp_reload_upper_envelope_mut1 COMMAND utpp_reload_upper_envelope_mut1)
set_tests_properties(utpp_reload_upper_envelope_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# Mutation 3 = pistol non-ADS entry back to .12 s and MUST fail.
add_executable(utpp_reload_upper_envelope_mut3 tests/ReloadUpperEnvelopeTest.cpp src/mm/ReloadContactFade.hpp)
target_compile_options(utpp_reload_upper_envelope_mut3 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_upper_envelope_mut3 PRIVATE cxx_std_20)
target_compile_definitions(utpp_reload_upper_envelope_mut3 PRIVATE UTPP_RELOAD_ENVELOPE_MUTATION=3)
add_test(NAME utpp_reload_upper_envelope_mut3 COMMAND utpp_reload_upper_envelope_mut3)
set_tests_properties(utpp_reload_upper_envelope_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# Slide roll hold (reload started in the rolled phase of a slide) and the torso counter-roll of a slide reload (SlideDeRoll).
# Mutation 2 disables the hold, mutation 4 the de-roll (weight), mutation 5 only the de-pitch; all MUST fail.
add_executable(utpp_reload_slide_rollhold_test tests/ReloadSlideRollHoldTest.cpp src/mm/ReloadContactFade.hpp src/mm/SlideDeRoll.hpp)
target_compile_options(utpp_reload_slide_rollhold_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_slide_rollhold_test PRIVATE cxx_std_20)
add_test(NAME utpp_reload_slide_rollhold_test COMMAND utpp_reload_slide_rollhold_test)
add_executable(utpp_reload_slide_rollhold_mut4 tests/ReloadSlideRollHoldTest.cpp src/mm/ReloadContactFade.hpp src/mm/SlideDeRoll.hpp)
target_compile_options(utpp_reload_slide_rollhold_mut4 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_slide_rollhold_mut4 PRIVATE cxx_std_20)
target_compile_definitions(utpp_reload_slide_rollhold_mut4 PRIVATE UTPP_RELOAD_ENVELOPE_MUTATION=4)
add_test(NAME utpp_reload_slide_rollhold_mut4 COMMAND utpp_reload_slide_rollhold_mut4)
set_tests_properties(utpp_reload_slide_rollhold_mut4 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
add_executable(utpp_reload_slide_rollhold_mut5 tests/ReloadSlideRollHoldTest.cpp src/mm/ReloadContactFade.hpp src/mm/SlideDeRoll.hpp)
target_compile_options(utpp_reload_slide_rollhold_mut5 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_slide_rollhold_mut5 PRIVATE cxx_std_20)
target_compile_definitions(utpp_reload_slide_rollhold_mut5 PRIVATE UTPP_RELOAD_ENVELOPE_MUTATION=5)
add_test(NAME utpp_reload_slide_rollhold_mut5 COMMAND utpp_reload_slide_rollhold_mut5)
set_tests_properties(utpp_reload_slide_rollhold_mut5 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# Mutation 6 keeps the slide torso uprighting pistol-only (no rifle de-roll), 7 disables the rifle slide-exit latch, 8 disables the
# rifle carry uprighting (slide without a reload); all MUST fail.
foreach(_m 6 7 8)
  add_executable(utpp_reload_slide_rollhold_mut${_m} tests/ReloadSlideRollHoldTest.cpp src/mm/ReloadContactFade.hpp src/mm/SlideDeRoll.hpp)
  target_compile_options(utpp_reload_slide_rollhold_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_reload_slide_rollhold_mut${_m} PRIVATE cxx_std_20)
  target_compile_definitions(utpp_reload_slide_rollhold_mut${_m} PRIVATE UTPP_RELOAD_ENVELOPE_MUTATION=${_m})
  add_test(NAME utpp_reload_slide_rollhold_mut${_m} COMMAND utpp_reload_slide_rollhold_mut${_m})
  set_tests_properties(utpp_reload_slide_rollhold_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()
add_executable(utpp_reload_slide_rollhold_mut2 tests/ReloadSlideRollHoldTest.cpp src/mm/ReloadContactFade.hpp src/mm/SlideDeRoll.hpp)
target_compile_options(utpp_reload_slide_rollhold_mut2 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_slide_rollhold_mut2 PRIVATE cxx_std_20)
target_compile_definitions(utpp_reload_slide_rollhold_mut2 PRIVATE UTPP_RELOAD_ENVELOPE_MUTATION=2)
add_test(NAME utpp_reload_slide_rollhold_mut2 COMMAND utpp_reload_slide_rollhold_mut2)
set_tests_properties(utpp_reload_slide_rollhold_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# Folded native hand: authored reload contact solve below the default GripIK fold clamp. Needs the woman overlay file
# (_utpp_pmotion_wov from motion_pose.cmake). Mutation 3 restores the default clamp and MUST fail.
if(DEFINED _utpp_pmotion_wov)
  add_executable(utpp_reload_folded_arm_test tests/ReloadFoldedArmTest.cpp)
  target_compile_options(utpp_reload_folded_arm_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_reload_folded_arm_test PRIVATE cxx_std_20)
  add_test(NAME utpp_reload_folded_arm_test COMMAND utpp_reload_folded_arm_test "${_utpp_pmotion_wov}")
  add_executable(utpp_reload_folded_arm_mut3 tests/ReloadFoldedArmTest.cpp)
  target_compile_options(utpp_reload_folded_arm_mut3 PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_reload_folded_arm_mut3 PRIVATE cxx_std_20)
  target_compile_definitions(utpp_reload_folded_arm_mut3 PRIVATE UTPP_RELOAD_CONTACT_MUTATION=3)
  add_test(NAME utpp_reload_folded_arm_mut3 COMMAND utpp_reload_folded_arm_mut3 "${_utpp_pmotion_wov}")
  set_tests_properties(utpp_reload_folded_arm_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endif()
# Diverged native body (squatting native pelvis / weapon inside the head): torso transport + placement guard.
# Mutation 4 = no divergence handling, 5 = no placement guard, 7 = placement factor not time limited, 8 = no fade across rejected frames,
# 13 = head clearance with a fixed band instead of relative to the previous output, 14 = HoldOnly without the snapshot
# validation, 16 = a perspective denial that finds the state busy leaves no reset request.
# 19 = no correction fade around the anchor <-> transport switch (4 = no divergence handling; the shoulder anchor stays on).
if(DEFINED _utpp_pmotion_wov)
  add_executable(utpp_reload_diverged_native_test tests/ReloadDivergedNativeTest.cpp)
  target_compile_options(utpp_reload_diverged_native_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_reload_diverged_native_test PRIVATE cxx_std_20)
  add_test(NAME utpp_reload_diverged_native_test COMMAND utpp_reload_diverged_native_test "${_utpp_pmotion_wov}")
  foreach(_m 4 5 7 8 13 14 16 19)
    add_executable(utpp_reload_diverged_native_mut${_m} tests/ReloadDivergedNativeTest.cpp)
    target_compile_options(utpp_reload_diverged_native_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_reload_diverged_native_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_reload_diverged_native_mut${_m} PRIVATE UTPP_RELOAD_CONTACT_MUTATION=${_m})
    add_test(NAME utpp_reload_diverged_native_mut${_m} COMMAND utpp_reload_diverged_native_mut${_m} "${_utpp_pmotion_wov}")
    set_tests_properties(utpp_reload_diverged_native_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
  endforeach()
endif()
# Capped guide turn continuous across the +-pi wrap (needs no data). Mutation 9 = hard clamp.
add_executable(utpp_reload_guide_clamp_test tests/ReloadGuideClampTest.cpp)
target_compile_options(utpp_reload_guide_clamp_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_guide_clamp_test PRIVATE cxx_std_20)
add_test(NAME utpp_reload_guide_clamp_test COMMAND utpp_reload_guide_clamp_test)
add_executable(utpp_reload_guide_clamp_mut9 tests/ReloadGuideClampTest.cpp)
target_compile_options(utpp_reload_guide_clamp_mut9 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_guide_clamp_mut9 PRIVATE cxx_std_20)
target_compile_definitions(utpp_reload_guide_clamp_mut9 PRIVATE UTPP_RELOAD_CONTACT_MUTATION=9)
add_test(NAME utpp_reload_guide_clamp_mut9 COMMAND utpp_reload_guide_clamp_mut9)
set_tests_properties(utpp_reload_guide_clamp_mut9 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
# Upper-arm continuity (joint speed limit, near-antiparallel transport, elbow plane across the +-pi wrap of the guide turn).
# Mutation 10 = plane from the transported authored elbow, 11 = no joint speed limit, 12 = blend weight ignores the wrap.
# Needs the woman overlay file.
if(DEFINED _utpp_pmotion_wov)
  add_executable(utpp_reload_arm_slew_test tests/ReloadArmSlewTest.cpp)
  target_compile_options(utpp_reload_arm_slew_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_reload_arm_slew_test PRIVATE cxx_std_20)
  add_test(NAME utpp_reload_arm_slew_test COMMAND utpp_reload_arm_slew_test "${_utpp_pmotion_wov}")
  foreach(_m 10 11 12 23)
    add_executable(utpp_reload_arm_slew_mut${_m} tests/ReloadArmSlewTest.cpp)
    target_compile_options(utpp_reload_arm_slew_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_reload_arm_slew_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_reload_arm_slew_mut${_m} PRIVATE UTPP_RELOAD_CONTACT_MUTATION=${_m})
    add_test(NAME utpp_reload_arm_slew_mut${_m} COMMAND utpp_reload_arm_slew_mut${_m} "${_utpp_pmotion_wov}")
    set_tests_properties(utpp_reload_arm_slew_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
  endforeach()
endif()
# Shoulder anchor of the native contact targets (fix batch 3, P3 RC2 / N6): the native body stands 6.3cm higher / 8.7cm forward of the MM body, the
# native choreography must keep its shoulder-to-hand vectors and its weapon height over the head on the MM body. Needs the woman overlay file.
# Mutation 17 = absolute native placement, 18 = the old +-2cm band of the left-hand-under-the-hips guard (cuts the correction at the belt),
# 21 = anchor computed on the records before the masked shoulder / arm channels are restored.
if(DEFINED _utpp_pmotion_wov)
  add_executable(utpp_reload_shoulder_anchor_test tests/ReloadShoulderAnchorTest.cpp)
  target_compile_options(utpp_reload_shoulder_anchor_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_reload_shoulder_anchor_test PRIVATE cxx_std_20)
  add_test(NAME utpp_reload_shoulder_anchor_test COMMAND utpp_reload_shoulder_anchor_test "${_utpp_pmotion_wov}")
  foreach(_m 17 18 21)
    add_executable(utpp_reload_shoulder_anchor_mut${_m} tests/ReloadShoulderAnchorTest.cpp)
    target_compile_options(utpp_reload_shoulder_anchor_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_reload_shoulder_anchor_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_reload_shoulder_anchor_mut${_m} PRIVATE UTPP_RELOAD_CONTACT_MUTATION=${_m})
    add_test(NAME utpp_reload_shoulder_anchor_mut${_m} COMMAND utpp_reload_shoulder_anchor_mut${_m} "${_utpp_pmotion_wov}")
    set_tests_properties(utpp_reload_shoulder_anchor_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
  endforeach()
endif()
# Reload-contact episode stance: the authored-reload episode latches its stationary decision on the reload edge and reads the PSM stance
# of that feed (ChooserState::stanceRaw), not the 0.15 s debounced stance. Mutation 1 reads cs.stance and MUST fail the 12d4 checks of the
# driver unit test (same sources and data directory as utpp_motion_driver_unit_test from motiondriver.cmake; only "FAIL 12d4" counts as killed).
if(TARGET utpp_motion_driver_unit_test)
  add_executable(utpp_reload_episode_stance_mut1 tests/MotionDriverTest.cpp src/MotionDriver.cpp)
  target_compile_options(utpp_reload_episode_stance_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_reload_episode_stance_mut1 PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UTPP_MOTION_DRIVER_TEST_FAULTS UTPP_RELOAD_EPISODE_STANCE_MUTATION=1)
  target_compile_features(utpp_reload_episode_stance_mut1 PRIVATE cxx_std_20)
  add_test(NAME utpp_reload_episode_stance_mut1 COMMAND utpp_reload_episode_stance_mut1 "${_utpp_mdmotion_data_native}")
  set_tests_properties(utpp_reload_episode_stance_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL 12d4")
endif()
# Rifle reloads (overlay Rifle, no authored clip): eligibility, chest-frame transport of the native targets, pistol path unchanged.
# Needs the woman overlay file. Mutation (UTPP_RELOAD_RIFLE_MUTATION) 1 = rifle path off, 2 = no forced chest-frame transport,
# 3 = rifles keep the stationary-only pose rule (no crouch / moving rifle reloads), 4 = rifle slide frames excluded (e00bb09),
# 5 = no stock clearance of the carried rifle, 6 = stock clearance shift applied raw (no rate limit), 7 = no left arm torso clearance,
# 8 = no left hand smoothing, 9 = the stock shift ignores the left arm, 10 = left arm clearance offset applied raw (no rate limit),
# 11 = left hand follower starts at the blended hand (no glide on the ramp-in), 12 = no weapon follower, 14 = old exit fade, 15 = no follower reset at weight 0, 16 = no contact catch-up, 17 = episode starts from the MM snapshot, 18 = old residual clearance rise, 19 = old contact catch-up speed, 20 = no start from the observed published pose, 21 = rifle ramp joint slew.
if(DEFINED _utpp_pmotion_wov)
  add_executable(utpp_reload_rifle_contacts_test tests/ReloadRifleContactsTest.cpp)
  target_compile_options(utpp_reload_rifle_contacts_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_reload_rifle_contacts_test PRIVATE cxx_std_20)
  add_test(NAME utpp_reload_rifle_contacts_test COMMAND utpp_reload_rifle_contacts_test "${_utpp_pmotion_wov}")
  foreach(_m 1 2 3 4 5 6 7 8 9 10 11 12 14 15 16 17 18 19 20 21)
    add_executable(utpp_reload_rifle_contacts_mut${_m} tests/ReloadRifleContactsTest.cpp)
    target_compile_options(utpp_reload_rifle_contacts_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_reload_rifle_contacts_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_reload_rifle_contacts_mut${_m} PRIVATE UTPP_RELOAD_RIFLE_MUTATION=${_m})
    add_test(NAME utpp_reload_rifle_contacts_mut${_m} COMMAND utpp_reload_rifle_contacts_mut${_m} "${_utpp_pmotion_wov}")
    set_tests_properties(utpp_reload_rifle_contacts_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
  endforeach()
endif()
# removed 2026-11-05 (review R5), which cheaper test kills the same site: utpp_reload_rifle_contacts_mut1, _mut3, _mut4
# Reload contact admission per pose hook owner (reload_contacts_runtime::Admit): the default build and the build with the Rifle overlay
# listed for the FK owner (UTPP_RELOAD_FK_OWNER_RIFLE=1). No data files.
add_executable(utpp_reload_owner_admit_test tests/ReloadOwnerAdmitTest.cpp)
target_compile_options(utpp_reload_owner_admit_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_owner_admit_test PRIVATE cxx_std_20)
add_test(NAME utpp_reload_owner_admit_test COMMAND utpp_reload_owner_admit_test)
add_executable(utpp_reload_owner_admit_rifle_test tests/ReloadOwnerAdmitTest.cpp)
target_compile_options(utpp_reload_owner_admit_rifle_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_reload_owner_admit_rifle_test PRIVATE cxx_std_20)
target_compile_definitions(utpp_reload_owner_admit_rifle_test PRIVATE UTPP_RELOAD_FK_OWNER_RIFLE=1)
add_test(NAME utpp_reload_owner_admit_rifle_test COMMAND utpp_reload_owner_admit_rifle_test)
