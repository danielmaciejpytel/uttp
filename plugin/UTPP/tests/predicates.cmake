# ---- port report 3.3 / 3.4: motion trajectory + ABP predicates (src/mm/MotionTrajectory.hpp). No data files.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/predicates.cmake OPTIONAL)); expects UTPP_WARNINGS.
add_executable(utpp_mm_predicates_test tests/PredicatesTest.cpp src/mm/MotionTrajectory.hpp)
target_compile_options(utpp_mm_predicates_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_predicates_test PRIVATE cxx_std_20)
add_test(NAME utpp_mm_predicates_test COMMAND utpp_mm_predicates_test)
# J1 mutation (UTPP_TRAJV2_HEAVY_MUTATION=1: the heavy-landing threshold stays 700 at every gravity): the gravity-scaled cases
# must print FAIL.
add_executable(utpp_mm_predicates_heavy_mut1 tests/PredicatesTest.cpp src/mm/MotionTrajectory.hpp)
target_compile_options(utpp_mm_predicates_heavy_mut1 PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_predicates_heavy_mut1 PRIVATE cxx_std_20)
target_compile_definitions(utpp_mm_predicates_heavy_mut1 PRIVATE UTPP_TRAJV2_HEAVY_MUTATION=1)
add_test(NAME utpp_mm_predicates_heavy_mut1 COMMAND utpp_mm_predicates_heavy_mut1)
set_tests_properties(utpp_mm_predicates_heavy_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
