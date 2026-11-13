# ---- Shooter mode: evidence tools. Included from plugin/UTPP/CMakeLists.txt (OPTIONAL) after tests/earlyposeruntime.cmake;
# expects UTPP_WARNINGS, UTPP_MMDB_TEST_FILE and _utpp_mdmotion_data_native (motion-derived data, outside git: the targets exist only where it is).
#
# A8  utpp_motion_driver_decisiondump: NOT a ctest test. A tool that writes one line per frame of the real v1 MMDriver + MotionDriver
#     (stepped, clip, db, clip time, visual root yaw, published pose hash). Built twice (baseline 723aee0 and the feature branch with the
#     setting off); the files must be byte-identical. It uses only API that already exists at 723aee0.
#     Usage: utpp_motion_driver_decisiondump <out.txt> <data dir> <v1 db> [--closed13 <flick_180_run.feed> <run_arc_left.feed>] [feed ...]
# A9  utpp_motion_driver_h2session_test: the H2 safeguard replayed on recorded game sessions, at gate level (MapToChooser with the real MapState,
#     MotionFeed from all 31 feed columns) and through the full driver with the guard on and off. The sidecar of each session (tests/data/
#     h2_sidecar_*.txt, tools/replay/h2_sidecar_from_log.py) carries what the feed lacks: native body turn off windows, Locomotion state, the
#     in-game Strafe windows, the weapon action. Needs the new fields of the Shooter branch (MapState::h2Active, MapContext::
#     bodyTurnSwitchedOff, motiondriver::TestStrafeProbe), so the target exists only when MotionAdapter.hpp has them. The session feeds live in
#     Research/logs/game (outside git): a test is registered per session whose feed exists.
if(DEFINED _utpp_mdmotion_data_native AND UTPP_MMDB_TEST_FILE)
    add_executable(utpp_motion_driver_decisiondump EXCLUDE_FROM_ALL tests/MotionDriverDecisionDump.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_link_libraries(utpp_motion_driver_decisiondump PRIVATE bcrypt)
    target_compile_options(utpp_motion_driver_decisiondump PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_motion_driver_decisiondump PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
    target_compile_features(utpp_motion_driver_decisiondump PRIVATE cxx_std_20)

    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/mm/MotionAdapter.hpp" _utpp_fe_drmotion)
    string(FIND "${_utpp_fe_drmotion}" "bodyTurnSwitchedOff" _utpp_fe_has_h2)
    if(NOT _utpp_fe_has_h2 EQUAL -1)
        add_executable(utpp_motion_driver_h2session_test tests/H2SessionGateTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
        target_link_libraries(utpp_motion_driver_h2session_test PRIVATE bcrypt)
        target_compile_options(utpp_motion_driver_h2session_test PRIVATE ${UTPP_WARNINGS} /EHsc)
        target_compile_definitions(utpp_motion_driver_h2session_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
        target_compile_features(utpp_motion_driver_h2session_test PRIVATE cxx_std_20)
        set(_utpp_fe_logs "${_utpp_mdmotion_data}/../Research/logs/game")
        # <session label> <feed file>: 12:13 = the current mode (hip fire, ADS); 01:16 = body turn on, weapon turn on, melee + free look + stagger;
        # 10-15 02:28 / 00:09 = recordings with melee combat in motion (the arm cause and the melee exclusion matter there)
        foreach(_s "20261021_1213|utpp-feed-2026-10-21-12-13-28.txt" "20261021_0116|utpp-feed-2026-10-21-01-16-02.txt"
                   "20261015_0228|utpp-feed-2026-10-15-02-28-51.txt" "20261015_0009|utpp-feed-2026-10-15-00-09-19.txt")
            string(REPLACE "|" ";" _p "${_s}")
            list(GET _p 0 _lbl)
            list(GET _p 1 _feed)
            if(EXISTS "${_utpp_fe_logs}/${_feed}" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/h2_sidecar_${_lbl}.txt")
                file(TO_NATIVE_PATH "${_utpp_fe_logs}/${_feed}" _feed_native)
                file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/h2_sidecar_${_lbl}.txt" _side_native)
                add_test(NAME utpp_motion_driver_h2session_${_lbl} COMMAND utpp_motion_driver_h2session_test
                         "${_utpp_mdmotion_data_native}" "${UTPP_MMDB_TEST_FILE}" "${_feed_native}" "${_side_native}")
            endif()
        endforeach()
    endif()
endif()
