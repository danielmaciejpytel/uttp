# ---- Item render plane core (src/render/ItemPlaneCore.hpp): policy table, force -> vanilla restore, outside-broadcast writes kept,
# pruning, cap (+ reclaim of dead entries at the cap), (instance, reference block) key. Pure core, no game types, no data files.
# Included from plugin/UTPP/CMakeLists.txt (include(tests/itemplane.cmake OPTIONAL)); expects UTPP_WARNINGS.
add_executable(utpp_itemplane_core_test tests/ItemPlaneCoreTest.cpp src/render/ItemPlaneCore.hpp)
target_compile_options(utpp_itemplane_core_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_itemplane_core_test PRIVATE cxx_std_20)
add_test(NAME utpp_itemplane_core_test COMMAND utpp_itemplane_core_test)
# Mutations (UTPP_ITEMPLANE_MUTATION=N, list in ItemPlaneCore.hpp), each must fail its own check:
#   1 = no policy gate; 2 = no inside-broadcast gate; 3 = forcing rewrites every plane; 4 = re-apply rewrites outside-written
#   entries; 5 = prune keeps dead entries; 6 = cap + 1; 7 = registry keeps the written plane; 8 = broadcast write keeps the
#   outside mark; 9 = outside write registers; 10 = key = instance only; 11 = the anim-param gate ignores the handler depth
#   (queued entAnimParamsEvent, NG1); 12 = every outside write marks outsideLast (no agreement check, SPECIAL-2);
#   13 = no reclaim at the cap (dead entries fill the table, new components dropped); 14 = reclaim ignores the rate limit.
foreach(_utpp_ip_mut 1 2 3 4 5 6 7 8 9 10 11 12 13 14)
    add_executable(utpp_itemplane_core_mut${_utpp_ip_mut} tests/ItemPlaneCoreTest.cpp src/render/ItemPlaneCore.hpp)
    target_compile_options(utpp_itemplane_core_mut${_utpp_ip_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_itemplane_core_mut${_utpp_ip_mut} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_itemplane_core_mut${_utpp_ip_mut} PRIVATE UTPP_ITEMPLANE_MUTATION=${_utpp_ip_mut})
    add_test(NAME utpp_itemplane_core_mut${_utpp_ip_mut} COMMAND utpp_itemplane_core_mut${_utpp_ip_mut})
endforeach()
set_tests_properties(utpp_itemplane_core_mut1 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL policy off: broadcast on, requested 2")
set_tests_properties(utpp_itemplane_core_mut2 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL outside broadcast: override on, requested 2")
set_tests_properties(utpp_itemplane_core_mut3 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL policy force: requested plane 1")
set_tests_properties(utpp_itemplane_core_mut4 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL outside-written entry rewritten")
set_tests_properties(utpp_itemplane_core_mut5 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL prune: size")
set_tests_properties(utpp_itemplane_core_mut6 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL cap: max")
set_tests_properties(utpp_itemplane_core_mut7 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL restore after a forced broadcast write")
set_tests_properties(utpp_itemplane_core_mut8 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL broadcast write clears the outside mark")
set_tests_properties(utpp_itemplane_core_mut9 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL outside write registered a component")
set_tests_properties(utpp_itemplane_core_mut10 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL reused address aliases the old entry")
set_tests_properties(utpp_itemplane_core_mut11 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL handler path")
set_tests_properties(utpp_itemplane_core_mut12 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL agree rows")
set_tests_properties(utpp_itemplane_core_mut13 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL reclaim at the cap: record")
set_tests_properties(utpp_itemplane_core_mut14 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL reclaim rate limit")
