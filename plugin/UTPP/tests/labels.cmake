# ctest labels: area labels from the test name + "mutation" / "base" from the test's expectation, "slow" from a fixed list.
# Included LAST from CMakeLists.txt (every add_test above it is labelled). Usage: ctest -L vehicle, ctest -L weapon -LE mutation,
# ctest -LE "mutation|slow" (quick set). Build\build-main.cmd area <label> / affected wraps this.
# A test can carry several area labels. A test that matches no area gets "misc" (extend the table below when that happens).
set(_utpp_label_areas
  # label     regex on the test name (without the utpp_ prefix)
  vehicle   "vehexit|vehtrans|approach|jumpout|tpprepguard|integ_vehexit|air_vehexit"
  weapon    "hipfire|hipraise|recoil|barrel|aimoffset|magtap|magazine|stationary_shot|gripik|supportgrip|dualwield|itemplane|rearm|overlay_hip|aoreload|shooter_mode_slide|native_shoot"
  reload    "reload|aoreload|magtap"
  loco      "standing_stop|offsetroot|loco|locomotion|stance|strafe|steering|slowwalk|steadyrun|speedcal|Start|Stop|Capsule|Crouch|footlock|footsink|jump|air|slide|travers|travfile|matcher|chooser|inertializer|PlayRate|RootMotion|NonLoop|Reface|idlebreaks|montage|lean|shooter_mode|ShooterModeStrafe|h2session|motion_adapter_integ|mm_sim|gravity|predicates|motion_adapter_compile|motion_adapter_unit|dbswitch|feeddump|query_parity|mmdb|BlendStack|mm_integration|analogfloor|mm_loco|motion_pose|overlay|ovlegs|p1_"
  body      "body_|Feminine|clavicle|forearm|supportgrip|shoulder|deform|pose_|Pose|Primary|offset_root|OffsetRoot|RootFrame|root_frame|chestalign|bodyturn|aospineyaw|headlead|headguard|loothead|orientwarp|Mapping|SemanticPose|quat|rest_offset|early_pose|stance_calibration|arm_slew|folded_arm|upper_envelope"
  ragdoll   "ragdoll"
  camera    "Camera|gamelook|headlead|PerspectiveWriteGate|tpprepguard"
  input     "native_input|p1_stance_latency|dualwield_psm"
  melee     "melee"
)
set(_utpp_label_slow "^utpp_(mm_loco_test(_.*)?|motion_driver_integ_test|motion_driver_hipfire_weapons_.*|motion_driver_hipraise_.*|motion_driver_recoil_(profile|series|exit|switch|reach)_.*|mm_vehexit_loco_.*|motion_driver_shooter_mode_.*|CameraTargetRoot_.*|FeminineWidthConsistency_.*|reload_rifle_domain_.*|mm_motion_pose_.*)$")

get_property(_utpp_all_tests DIRECTORY PROPERTY TESTS)
list(LENGTH _utpp_label_areas _utpp_la_n)
math(EXPR _utpp_la_last "${_utpp_la_n} - 1")
foreach(_t IN LISTS _utpp_all_tests)
  string(REGEX REPLACE "^utpp_" "" _short "${_t}")
  set(_labels)
  foreach(_i RANGE 0 ${_utpp_la_last} 2)
    math(EXPR _j "${_i} + 1")
    list(GET _utpp_label_areas ${_i} _lab)
    list(GET _utpp_label_areas ${_j} _rx)
    if(_short MATCHES "${_rx}")
      list(APPEND _labels ${_lab})
    endif()
  endforeach()
  if(NOT _labels)
    list(APPEND _labels misc)
  endif()
  # mutation = a test that must observe a FAIL line (PASS_REGULAR_EXPRESSION "FAIL ...") or must fail (WILL_FAIL)
  get_test_property(${_t} PASS_REGULAR_EXPRESSION _pre)
  get_test_property(${_t} WILL_FAIL _wf)
  if((_pre AND _pre MATCHES "FAIL") OR _wf OR _t MATCHES "_mut[A-Za-z0-9_]*$")
    list(APPEND _labels mutation)
  else()
    list(APPEND _labels base)
  endif()
  # mutation early exit (tests/TestFailFast.hpp): the test may stop at the first FAIL output that already satisfies its expectation
  if(_pre AND _pre MATCHES "FAIL")
    string(REPLACE ";" "|" _stop "${_pre}")   # a list of expressions = any of them
    set_property(TEST ${_t} APPEND PROPERTY ENVIRONMENT "UTPP_TEST_STOP_ON=${_stop}")
  elseif(_wf)
    set_property(TEST ${_t} APPEND PROPERTY ENVIRONMENT "UTPP_TEST_STOP_ON=FAIL")
  endif()
  # tests that run their scenarios as child processes (pool in tests/MotionDriverIntegrationTest.cpp): 4 children, and ctest counts
  # 4 slots for them, so -j 16 does not oversubscribe the CPU (the mod's worker thread then lags and frame-exact checks such as
  # hipfire-rifle-profile at 144 Hz fail under load only, seen 2026-11-05)
  if(_t MATCHES "^utpp_motion_driver_(integ|hipfire|hipraise|recoil_(profile|series|exit|switch|reach))")
    set_property(TEST ${_t} APPEND PROPERTY ENVIRONMENT "UTPP_TEST_CHILD_JOBS=4")
    set_property(TEST ${_t} PROPERTY PROCESSORS 4)
  endif()
  if(_t MATCHES "${_utpp_label_slow}")
    list(APPEND _labels slow)
  endif()
  set_property(TEST ${_t} APPEND PROPERTY LABELS ${_labels})
endforeach()
