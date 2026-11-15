# ---- dual wield recoil per arm through the real driver (tests/DualWieldDriverTest.cpp). Included from plugin/UTPP/CMakeLists.txt
# (after motiondriver.cmake); expects UTPP_WARNINGS and _utpp_mdmotion_data_native.
# Mutations (UTPP_DUALWIELD_KICK_MUTATION) MUST report an assertion failure: 1 = the left weapon's kick is never applied to the left arm
# (fails the left kick checks), 2 = a left shot is no shot event of the Shoot state (the right weapon's wait expires and the native
# fallback takes the arms in hip fire: fails the upperScale check). They pass only when the output contains a line starting "FAIL "
# (a setup error cannot count as a killed mutant).
foreach(_m "" 1 2)
    if(_m STREQUAL "")
        set(_tgt utpp_dualwield_driver_test)
    else()
        set(_tgt utpp_dualwield_driver_mut${_m})
    endif()
    add_executable(${_tgt} tests/DualWieldDriverTest.cpp src/MotionHost.cpp src/MotionDriver.cpp)
    target_compile_features(${_tgt} PRIVATE cxx_std_20)
    target_compile_options(${_tgt} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(${_tgt} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE UTPP_MOTION_DRIVER_TEST_TRAV)
    if(NOT _m STREQUAL "")
        target_compile_definitions(${_tgt} PRIVATE UTPP_DUALWIELD_KICK_MUTATION=${_m})
    endif()
    target_link_libraries(${_tgt} PRIVATE bcrypt)
    add_test(NAME ${_tgt} COMMAND ${_tgt} "${_utpp_mdmotion_data_native}")
    if(NOT _m STREQUAL "")
        set_tests_properties(${_tgt} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
    endif()
endforeach()
