# ---- vehicle batch W1 (feat/veh-dll-exit): vehicle exit gate + traversal release mutation builds.
# Included OPTIONAL from plugin/UTPP/CMakeLists.txt; expects UTPP_WARNINGS and UTPP_MMDB_PACK_TEST_FILE / UTPP_MMDB_TEST_FILE.
# The production targets (UTPP, utpp_mm_loco_test, utpp_mm_integration_test, utpp_mm_airstate_test) always compile with
# UTPP_VEHEXIT_MUTATION undefined. Each mutation target below is built from the same sources with UTPP_VEHEXIT_MUTATION=<n> and
# MUST report an assertion failure: it passes only when the output contains a failed check ("[FAIL]" / "FAIL "); the
# PASS_REGULAR_EXPRESSION ignores the exit code, so a setup error without a failed check does not count as a killed mutant.
# Mutations (src/mm/GameIntegration.hpp, src/MotionHost.cpp, src/mm/AirState.hpp):
#   1  CAR-1: no exit workspot latch exemption (ExitReleaseMask)       -> loco SectionVehExit T1 / T4 / CAR-2, integration table
#   2  CAR-1: the exit window ends on the post-hold effective gate      -> loco SectionVehExit T1 (the latched workspot releases)
#   3  CAR-1 / MOTO-6: TPP exit window keeps the game upper body       -> loco SectionVehExit T2 + Section062 exit, integration table
#   4  TRAV-1: TravReleases ignores aMotionActive (pre-TRAV-1 semantics)    -> loco SectionAir 'climb release ... TRAV-1'
#   5  CAR-2: no vehicle-exit planar speed clamp in AirState            -> utpp_mm_airstate_test TestVehicleExitSpeedClamp
#   6  P2: no vehtrans wiring in MMDriver (no BeginMount / capture / hand-in) -> loco SectionVehExit P2 mount / exit
#   7  review: no ride latch (an abort mid-ride restarts the mount transition) -> loco SectionVehExit P2 ride
#   8  review: every re-arm aborts the hand-in and resets the weight            -> loco SectionVehExit P2 exit + re-arm
#   9  review: no bone roles from the game rig (per-bone 25 deg fallback)        -> loco SectionVehExit P2 mount legs
#  10  2026-10-28: no bike jump-out release (JumpOutLatch never fires)           -> loco SectionVehExit JUMPOUT, integration 'jump-out'
#  11  2026-10-28: jump-out without the ride / arm window guard                  -> integration 'jump-out' negatives
#  12  2026-10-28: no live exit capture at the jump-out launch (no hand-in)      -> loco SectionVehExit JUMPOUT live capture
#  13  2026-10-28: no rotation step cap in the jump-out window                 -> loco SectionVehExit JUMPOUT (jumpOutSlewFrames)
#  14  2026-10-28: the hand-in start does not restart the step limiter history   -> loco SectionVehExit JUMPOUT live capture (visible rotation)
#  15  2026-10-28: the mounted-TPP flag only from the vehicle bit (not at MountEvent) -> loco SectionVehExit mount order
#  16  2026-10-28: no early re-arm probe after a stall of the player's animation calls -> utpp_rearm_probe_mut16 (RearmProbeTest)
#  17  2026-10-28: no mounted pop filter (the game's one-frame jumps of the seated / exit pose stay) -> loco SectionVehExit ride pops
#  18  2026-10-28: a same-rig re-arm in the jump-out flight keeps MM at once (no capture / hand-in) -> loco SectionVehExit JUMPOUT re-arm in the flight
#  19  fix/jumpout-cam: a short contact in a fast fall ends the air episode (no false-touch recovery in AirState) -> utpp_mm_airstate_test TestFalseTouchInFastFall
#  20  review r2: the false-touch take-back for every landing, not only a vehicle exit episode        -> utpp_mm_airstate_test TestFalseTouchGate
#  21  review r2: the take-back reads the exit flag of the feed only (no episode latch)                -> utpp_mm_airstate_test TestFalseTouchGate
#  22  r3 landing: the closed exit window ends the jump-out state at once (no hold to the confirmed touchdown) -> loco SectionVehExit JUMPOUT false contact
#  23  r3 landing: the v2 step gets the closed exit window only (MotionStepLocked drops the held jump-out state)  -> loco SectionVehExit JUMPOUT false contact, v2 side
#  24  r3 landing: the stale Tick path clears the jump-out state with the closed exit window                  -> loco SectionVehExit JUMPOUT feed dropout
#  25  r3b landing: Tick never feeds the touchdown when the landing state gives no locomotion update            -> loco SectionVehExit JUMPOUT landing dropout
#  26  r3b landing: the synthetic landing keeps the Fall detail (0x610F2040) of the last air feed               -> loco SectionVehExit JUMPOUT landing dropout after Fall-detail rows
#  27  r4 landing: the continuation is measured in wall time instead of game sim time (a pause invents landing feeds) -> loco SectionVehExit JUMPOUT pause (sim clock frozen)
#  29  r4 landing: synthetic landing feeds run the exit window down / close it                                  -> loco SectionVehExit JUMPOUT landing dropout keeps the exit window
#  30  r6 launch: no pinned hold of the edge capture at the jump-out launch (the denied frame after the edge shows the raw game pose) -> loco SectionVehExit JUMPOUT launch (r6)
if(UTPP_MMDB_PACK_TEST_FILE)
    # loco scenario mutations: the section is selected by environment (UTPP_LOCO_ONLY*), so a mutant runs only the checks it targets.
    # Run-time mutants (src/MutationSelect.hpp): ONE executable (every mutation site of the loco family is a plain run-time branch) reads the number from
    # UTPP_MUTATION_VEHEXIT and ctest runs it once per mutation. Mutations 1, 3 and 10 have no loco test: the table mutants utpp_mm_vehexit_integ_mut1 / _mut3 /
    # _mut10 below kill their sites (review R4), so the loco mutants 1 / 3 / 10 are dropped.
    set(_tgt utpp_mm_vehexit_loco_rt)
    add_executable(${_tgt} tests/MMLocoScenarioTest.cpp src/MotionHost.cpp src/MotionHost.hpp src/mm/GameIntegration.hpp src/mm/AirState.hpp src/MutationSelect.hpp)
    target_link_libraries(${_tgt} PRIVATE bcrypt)
    target_compile_definitions(${_tgt} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_RUNTIME_MUTATION_VEHEXIT)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    # own output folder: the loco test writes scratch copies next to its exe (air_notable\); a shared folder would race with
    # utpp_mm_loco_test under ctest -j (only the mutation 4 run, the air section, writes there)
    set_target_properties(${_tgt} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/vehexit_mut_rt")
    foreach(_m 2 4 6 7 8 9 12 13 14 15 17 18 22 23 24 25 26 27 29 30)
        set(_tgt utpp_mm_vehexit_loco_mut${_m})
        add_test(NAME ${_tgt} COMMAND utpp_mm_vehexit_loco_rt "${UTPP_MMDB_PACK_TEST_FILE}")
        if(_m EQUAL 4)
            set(_env "UTPP_LOCO_ONLYAIR=1")
        else()
            set(_env "UTPP_LOCO_ONLYVEHEXIT=1")
        endif()
        set_tests_properties(${_tgt} PROPERTIES ENVIRONMENT "${_env};UTPP_MUTATION_VEHEXIT=${_m}" PASS_REGULAR_EXPRESSION "\\[FAIL\\]")
    endforeach()
    # mutation 4 must be killed by its own check (TRAV-1), not by any failed check of the air section
    set_tests_properties(utpp_mm_vehexit_loco_mut4 PROPERTIES PASS_REGULAR_EXPRESSION "\\[FAIL\\] climb release .*TRAV-1")
    # mutation 30 must be killed by its own check (the loco mutants pass on any failed check; a loaded machine can fail unrelated real-time ones)
    set_tests_properties(utpp_mm_vehexit_loco_mut30 PROPERTIES PASS_REGULAR_EXPRESSION "\\[FAIL\\] vehexit .*JUMPOUT launch \\(r6")
endif()
# table mutations of the pure rules (no database needed for the checked part)
foreach(_m 1 3)
    set(_tgt utpp_mm_vehexit_integ_mut${_m})
    add_executable(${_tgt} tests/MMIntegrationTest.cpp src/mm/GameIntegration.hpp)
    target_compile_definitions(${_tgt} PRIVATE UTPP_VEHEXIT_MUTATION=${_m})
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    add_test(NAME ${_tgt} COMMAND ${_tgt} "${UTPP_MMDB_TEST_FILE}")
    set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "\\[FAIL\\] (exit window|upper mask)")
endforeach()
# 2026-10-28 bike jump-out latch (GameIntegration.hpp JumpOutLatch)
foreach(_m 10 11)
    set(_tgt utpp_mm_vehexit_integ_mut${_m})
    add_executable(${_tgt} tests/MMIntegrationTest.cpp src/mm/GameIntegration.hpp)
    target_compile_definitions(${_tgt} PRIVATE UTPP_VEHEXIT_MUTATION=${_m})
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    add_test(NAME ${_tgt} COMMAND ${_tgt} "${UTPP_MMDB_TEST_FILE}")
    set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "\\[FAIL\\] jump-out")
endforeach()
set(_tgt utpp_mm_vehexit_airstate_mut5)
add_executable(${_tgt} tests/MMAirStateTest.cpp src/mm/AirState.hpp src/mm/GameIntegration.hpp)
target_include_directories(${_tgt} PRIVATE src)
target_compile_definitions(${_tgt} PRIVATE UTPP_VEHEXIT_MUTATION=5)
target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME ${_tgt} COMMAND ${_tgt})
set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL .*vehicle exit clamp")
# fix/jumpout-cam (log 2026-11-03 17:34:36 jump-out B): mutation 19 = a short contact in a fast fall ends the air episode again (Recover -> HandBack)
set(_tgt utpp_mm_vehexit_airstate_mut19)
add_executable(${_tgt} tests/MMAirStateTest.cpp src/mm/AirState.hpp src/mm/GameIntegration.hpp)
target_include_directories(${_tgt} PRIVATE src)
target_compile_definitions(${_tgt} PRIVATE UTPP_VEHEXIT_MUTATION=19)
target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME ${_tgt} COMMAND ${_tgt})
set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL .*false touch")
# review r2 finding 14: the take-back only in an episode of a vehicle exit. 20 = the take-back for every landing (ordinary jumps / downhill landings too),
# 21 = the exit flag of the feed only (no latch: the window closes before the landing)
foreach(_m 20 21)
    set(_tgt utpp_mm_vehexit_airstate_mut${_m})
    add_executable(${_tgt} tests/MMAirStateTest.cpp src/mm/AirState.hpp src/mm/GameIntegration.hpp)
    target_include_directories(${_tgt} PRIVATE src)
    target_compile_definitions(${_tgt} PRIVATE UTPP_VEHEXIT_MUTATION=${_m})
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    add_test(NAME ${_tgt} COMMAND ${_tgt})
    set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL .*false touch gate")
endforeach()
# 2026-10-28 bike jump-out: rotation step cap of the published pose (src/mm/PoseStepLimit.hpp); mutation 1 (no cap) MUST fail.
foreach(_m 0 1)
    if(_m EQUAL 0)
        set(_tgt utpp_pose_step_limit_test)
    else()
        set(_tgt utpp_pose_step_limit_mut${_m})
    endif()
    add_executable(${_tgt} tests/PoseStepLimitTest.cpp src/mm/PoseStepLimit.hpp)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    add_test(NAME ${_tgt} COMMAND ${_tgt})
    if(NOT _m EQUAL 0)
        target_compile_definitions(${_tgt} PRIVATE UTPP_STEPLIMIT_MUTATION=${_m})
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
    endif()
endforeach()
# 2026-10-28 bike jump-out (recording 11-49-37): early re-arm probe after the game replaced the player's animated object
# (src/mm/RearmProbe.hpp); mutation 16 (no stall probe, only the 1 s poll) MUST fail.
foreach(_m 0 16)
    if(_m EQUAL 0)
        set(_tgt utpp_rearm_probe_test)
    else()
        set(_tgt utpp_rearm_probe_mut${_m})
    endif()
    add_executable(${_tgt} tests/RearmProbeTest.cpp src/mm/RearmProbe.hpp)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    add_test(NAME ${_tgt} COMMAND ${_tgt})
    if(NOT _m EQUAL 0)
        target_compile_definitions(${_tgt} PRIVATE UTPP_VEHEXIT_MUTATION=${_m})
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
    endif()
endforeach()
# 2026-11-10 hood slide with a weapon (src/mm/VaultTorso.hpp): the game's own vault over a car shows the weapon layer's rigid spine (chest below the
# horizontal); the FK detour limits the elevation of Hips -> Spine3. Real captured poses of the recording (fixture made by
# tools/analysis/vaulttorso_fixture.py, in Build/test-data/vaulttorso); mutations MUST fail: 1 the pass compiled out (the build before), 2 the FK gate needs MM active (first version), 3 no exit hold, 4 foot-only ignored.
set(_utpp_vaulttorso_fixture "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/test-data/vaulttorso/vaulttorso_20261110.txt")
foreach(_m 0 1 2 3 4)
    if(_m EQUAL 0)
        set(_tgt utpp_vehexit_vaulttorso_test)
    else()
        set(_tgt utpp_vehexit_vaulttorso_mut${_m})
    endif()
    add_executable(${_tgt} tests/VaultTorsoTest.cpp src/mm/VaultTorso.hpp)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    add_test(NAME ${_tgt} COMMAND ${_tgt} ${_utpp_vaulttorso_fixture})
    if(NOT _m EQUAL 0)
        target_compile_definitions(${_tgt} PRIVATE UTPP_VAULTTORSO_MUTATION=${_m})
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
    endif()
endforeach()
# offline comparison film tool of the same limiter (tools/films/vaulttorso_film.py); not a test, build explicitly: ninja utpp_vaulttorso_film_tool
add_executable(utpp_vaulttorso_film_tool EXCLUDE_FROM_ALL tests/VaultTorsoFilmTool.cpp src/mm/VaultTorso.hpp)
target_compile_options(utpp_vaulttorso_film_tool PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_vaulttorso_film_tool PRIVATE cxx_std_20)
