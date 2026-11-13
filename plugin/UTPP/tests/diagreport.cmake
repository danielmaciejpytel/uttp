# ---- diagnostic report file (src/DiagReport.hpp text logic, src/DiagReportRuntime.cpp writer) and the relative-path helpers of src/CommonUtil.hpp.
add_executable(utpp_diag_report_test tests/DiagReportTest.cpp src/DiagReportRuntime.cpp src/DiagReportRuntime.hpp src/DiagReport.hpp src/CommonUtil.hpp)
target_compile_features(utpp_diag_report_test PRIVATE cxx_std_20)
target_compile_options(utpp_diag_report_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_definitions(utpp_diag_report_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
target_link_libraries(utpp_diag_report_test PRIVATE bcrypt)
add_test(NAME utpp_diag_report_test COMMAND utpp_diag_report_test "${CMAKE_BINARY_DIR}/diag-report-test")
