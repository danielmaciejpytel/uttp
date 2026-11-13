# ---- compact pose capture (src/mm/PoseCaptureFormat.hpp, src/mm/PoseCaptureSession.hpp): block format, session layer, production writer.
# Mutations (UTPP_PCF_MUTATION): 1 = 8-bit quantization (format test), 2 = old segments never removed (session test). Each MUST print a "FAIL " line.
set(_utpp_pose_slice "${CMAKE_CURRENT_SOURCE_DIR}/../../../Build/test-data/posecapture/utpp-bodytrace-slice.csv")

add_executable(utpp_pose_capture_format_test tests/PoseCaptureFormatTest.cpp src/mm/PoseCaptureFormat.hpp src/mm/BodyDeformerTrace.hpp tests/PoseCaptureFixture.hpp)
target_compile_features(utpp_pose_capture_format_test PRIVATE cxx_std_20)
target_compile_options(utpp_pose_capture_format_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_pose_capture_format_test COMMAND utpp_pose_capture_format_test "${_utpp_pose_slice}")

add_executable(utpp_pose_capture_format_mut1 tests/PoseCaptureFormatTest.cpp src/mm/PoseCaptureFormat.hpp src/mm/BodyDeformerTrace.hpp tests/PoseCaptureFixture.hpp)
target_compile_features(utpp_pose_capture_format_mut1 PRIVATE cxx_std_20)
target_compile_options(utpp_pose_capture_format_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_pose_capture_format_mut1 PRIVATE UTPP_PCF_MUTATION=1)
add_test(NAME utpp_pose_capture_format_mut1 COMMAND utpp_pose_capture_format_mut1 "${_utpp_pose_slice}")
set_tests_properties(utpp_pose_capture_format_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")

add_executable(utpp_pose_capture_session_test tests/PoseCaptureSessionTest.cpp src/mm/PoseCaptureSession.hpp src/mm/PoseCaptureFormat.hpp
               src/mm/BodyTraceContinuous.hpp src/mm/BodyDeformerTrace.hpp)
target_compile_features(utpp_pose_capture_session_test PRIVATE cxx_std_20)
target_compile_options(utpp_pose_capture_session_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_pose_capture_session_test COMMAND utpp_pose_capture_session_test)

add_executable(utpp_pose_capture_session_mut2 tests/PoseCaptureSessionTest.cpp src/mm/PoseCaptureSession.hpp src/mm/PoseCaptureFormat.hpp
               src/mm/BodyTraceContinuous.hpp src/mm/BodyDeformerTrace.hpp)
target_compile_features(utpp_pose_capture_session_mut2 PRIVATE cxx_std_20)
target_compile_options(utpp_pose_capture_session_mut2 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_pose_capture_session_mut2 PRIVATE UTPP_PCF_MUTATION=2)
add_test(NAME utpp_pose_capture_session_mut2 COMMAND utpp_pose_capture_session_mut2)
set_tests_properties(utpp_pose_capture_session_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")

# Measurement / golden-file tool (bench: size and time per algorithm on a recording; encode: the slice as a compact file for the Python converter).
add_executable(utpp_pose_capture_tool tests/PoseCaptureTool.cpp src/PoseCaptureCompression.hpp src/mm/PoseCaptureFormat.hpp tests/PoseCaptureFixture.hpp)
target_compile_features(utpp_pose_capture_tool PRIVATE cxx_std_20)
target_compile_options(utpp_pose_capture_tool PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_pose_capture_tool PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
target_link_libraries(utpp_pose_capture_tool PRIVATE Cabinet)

# End-to-end: the production writer (src/PoseCaptureRuntime.cpp + src/BodyTraceRuntime.cpp for the coexistence check) with the real rings, a producer
# thread playing the FK hook, real files, real Windows compression; files are left in <build>/pose-capture-runtime-test.
add_executable(utpp_pose_capture_runtime_test tests/PoseCaptureRuntimeTest.cpp src/PoseCaptureRuntime.cpp src/PoseCaptureRuntime.hpp src/BodyTraceRuntime.cpp
               src/BodyTraceRuntime.hpp src/PoseCaptureCompression.hpp src/mm/PoseCaptureSession.hpp src/mm/PoseCaptureFormat.hpp src/mm/CaptureOwner.hpp
               src/mm/BodyTraceContinuous.hpp src/mm/BodyDeformerTrace.hpp)
target_compile_features(utpp_pose_capture_runtime_test PRIVATE cxx_std_20)
target_compile_options(utpp_pose_capture_runtime_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_pose_capture_runtime_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
target_link_libraries(utpp_pose_capture_runtime_test PRIVATE Cabinet)
add_test(NAME utpp_pose_capture_runtime_test COMMAND utpp_pose_capture_runtime_test "${CMAKE_BINARY_DIR}/pose-capture-runtime-test")

# Cross-check C++ writer -> Python converter: the tool writes the slice as compact files (XPRESS-Huffman and LZMS blocks of 50 frames), the converter
# test (tools/analysis/test_pose_capture_to_csv.py, UTPP_POSE_GOLDEN) converts them and compares with the slice CSV.
set(_utpp_pose_python "${CMAKE_CURRENT_SOURCE_DIR}/../../../Tools/ai-motion/envs/kimodo/Scripts/python.exe")
set(_utpp_pose_converter_test "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/analysis/test_pose_capture_to_csv.py")
if(EXISTS "${_utpp_pose_python}" AND EXISTS "${_utpp_pose_slice}" AND EXISTS "${_utpp_pose_converter_test}")
    foreach(_alg xpress lzms)
        add_test(NAME utpp_pose_capture_golden_${_alg} COMMAND utpp_pose_capture_tool encode "${_utpp_pose_slice}" "${CMAKE_BINARY_DIR}/pose-capture-golden/golden-${_alg}.utpppose" ${_alg} 0 50)
        set_tests_properties(utpp_pose_capture_golden_${_alg} PROPERTIES FIXTURES_SETUP utpp_pose_capture_golden)
    endforeach()
    add_test(NAME utpp_pose_capture_converter_test COMMAND "${_utpp_pose_python}" -I "${_utpp_pose_converter_test}")
    set_tests_properties(utpp_pose_capture_converter_test PROPERTIES FIXTURES_REQUIRED utpp_pose_capture_golden
                         ENVIRONMENT "UTPP_POSE_GOLDEN=${CMAKE_BINARY_DIR}/pose-capture-golden" TIMEOUT 300)
endif()
