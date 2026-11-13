# ---- port stage 10.8: traversal runtime (src/mm/BaseTraversal.hpp) offline test.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/traversal.cmake OPTIONAL)); expects UTPP_WARNINGS.
# Data (motion-derived, outside git) is passed at run time by the worktree build script:
#   Data/traversal/traversal_table.json, Data/traversal/traversal_vectors.txt, Research/inventory/traversal_oracle.json
add_executable(utpp_mm_traversal_test EXCLUDE_FROM_ALL tests/TraversalTest.cpp src/mm/BaseTraversal.hpp src/mm/MiniJson.hpp)
target_compile_options(utpp_mm_traversal_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_mm_traversal_test PRIVATE cxx_std_20)
