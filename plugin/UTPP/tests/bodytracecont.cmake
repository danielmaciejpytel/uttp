# ---- continuous capture of all helper bones (src/mm/BodyTraceContinuous.hpp): formatter, Capture::Refill, controller state machine.
# Mutations (UTPP_BTC_MUTATION): 1 = no refill, 2 = dropped frames not accumulated, 3 = identical map = new rig, 4 = no size cap. Each MUST print a "FAIL " line.
add_executable(utpp_body_trace_continuous_test tests/BodyTraceContinuousTest.cpp src/mm/BodyTraceContinuous.hpp src/mm/BodyDeformerTrace.hpp)
target_compile_features(utpp_body_trace_continuous_test PRIVATE cxx_std_20)
target_compile_options(utpp_body_trace_continuous_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_body_trace_continuous_test COMMAND utpp_body_trace_continuous_test)
foreach(_m 1 2 3 4)
    add_executable(utpp_body_trace_continuous_mut${_m} tests/BodyTraceContinuousTest.cpp src/mm/BodyTraceContinuous.hpp src/mm/BodyDeformerTrace.hpp)
    target_compile_features(utpp_body_trace_continuous_mut${_m} PRIVATE cxx_std_20)
    target_compile_options(utpp_body_trace_continuous_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_body_trace_continuous_mut${_m} PRIVATE UTPP_BTC_MUTATION=${_m})
    add_test(NAME utpp_body_trace_continuous_mut${_m} COMMAND utpp_body_trace_continuous_mut${_m})
    set_tests_properties(utpp_body_trace_continuous_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()

# End-to-end: the production writer (src/BodyTraceRuntime.cpp) with the real rings, a producer thread playing the FK hook, real CSV files in
# <build>/bodytrace-runtime-test (left there for inspection).
add_executable(utpp_body_trace_runtime_test tests/BodyTraceRuntimeTest.cpp src/BodyTraceRuntime.cpp src/BodyTraceRuntime.hpp
               src/mm/BodyTraceContinuous.hpp src/mm/BodyDeformerTrace.hpp)
target_compile_features(utpp_body_trace_runtime_test PRIVATE cxx_std_20)
target_compile_options(utpp_body_trace_runtime_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_body_trace_runtime_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
add_test(NAME utpp_body_trace_runtime_test COMMAND utpp_body_trace_runtime_test "${CMAKE_BINARY_DIR}/bodytrace-runtime-test")
