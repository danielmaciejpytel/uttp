# ---- magazine top-up gesture (src/mm/MagTap.hpp + MotionDriver.cpp MagTapStepLocked, native UTPPFeedMotionMagTap).
# Included from plugin/UTPP/CMakeLists.txt (include(tests/magtap.cmake OPTIONAL)) after tests/motiondriver.cmake; expects UTPP_WARNINGS.
# Header test: no data. Driver test: the data dir of motiondriver.cmake (_utpp_mdmotion_data_native; motion-derived, outside git).
# Mutations (UTPP_MAGTAP_MUTATION): 1 = trigger ignored, 2 = naive restart of the clock on a second trigger, 4 = no wait for the slide
# de-roll (SlideDefer); each MUST print a FAIL.
# Driver-only mutation 3 = the CET mm.shinobiBonusReload setting (SetMagTapEnabled) ignored in FeedMagTap.
add_executable(utpp_magtap_test tests/MagTapTest.cpp src/mm/MagTap.hpp)
target_compile_options(utpp_magtap_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_magtap_test PRIVATE cxx_std_20)
add_test(NAME utpp_magtap_test COMMAND utpp_magtap_test)
foreach(_m 1 2 4)
  add_executable(utpp_magtap_mut${_m} tests/MagTapTest.cpp src/mm/MagTap.hpp)
  target_compile_options(utpp_magtap_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_features(utpp_magtap_mut${_m} PRIVATE cxx_std_20)
  target_compile_definitions(utpp_magtap_mut${_m} PRIVATE UTPP_MAGTAP_MUTATION=${_m})
  add_test(NAME utpp_magtap_mut${_m} COMMAND utpp_magtap_mut${_m})
  set_tests_properties(utpp_magtap_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()
if(DEFINED _utpp_mdmotion_data_native)
  add_executable(utpp_magtap_driver_test tests/MagTapDriverTest.cpp src/MotionDriver.cpp src/MotionDriver.hpp src/mm/MagTap.hpp)
  target_compile_options(utpp_magtap_driver_test PRIVATE ${UTPP_WARNINGS} /EHsc)
  target_compile_definitions(utpp_magtap_driver_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UTPP_MOTION_DRIVER_TEST_FAULTS)
  target_compile_features(utpp_magtap_driver_test PRIVATE cxx_std_20)
  add_test(NAME utpp_magtap_driver_test COMMAND utpp_magtap_driver_test "${_utpp_mdmotion_data_native}")
  # mutation 1 MUST fail "B ... the gesture plays", mutation 2 MUST fail "E second request in the fall", mutation 3 (driver only:
  # the CET mm.shinobiBonusReload setting ignored in FeedMagTap) MUST fail "I off by default", mutation 4 (no slide wait) MUST fail
  # "F rifle on slide frames: the start waits"
# removed 2026-11-05 (review R9), which cheaper test kills the same site: utpp_magtap_mut1, _mut2, _mut4
  foreach(_m 3)
    add_executable(utpp_magtap_driver_mut${_m} tests/MagTapDriverTest.cpp src/MotionDriver.cpp)
    target_compile_options(utpp_magtap_driver_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_magtap_driver_mut${_m} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UTPP_MOTION_DRIVER_TEST_FAULTS UTPP_MAGTAP_MUTATION=${_m})
    target_compile_features(utpp_magtap_driver_mut${_m} PRIVATE cxx_std_20)
    add_test(NAME utpp_magtap_driver_mut${_m} COMMAND utpp_magtap_driver_mut${_m} "${_utpp_mdmotion_data_native}")
  endforeach()
  set_tests_properties(utpp_magtap_driver_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL I off by default")
endif()
