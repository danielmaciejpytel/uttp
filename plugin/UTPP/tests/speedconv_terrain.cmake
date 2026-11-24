# ---- Speed converter, terrain part (src/mm/speedconv_terrain.hpp, speedconv::kStepHold): stair step dips of the game speed held in the query.
# Included from plugin/UTPP/CMakeLists.txt; expects UTPP_WARNINGS. Header only, no data.
add_executable(utpp_speedconv_terrain_test tests/SpeedConv_terrainTest.cpp src/mm/speedconv_terrain.hpp src/mm/SpeedConverter.hpp)
target_compile_features(utpp_speedconv_terrain_test PRIVATE cxx_std_20)
target_compile_options(utpp_speedconv_terrain_test PRIVATE ${UTPP_WARNINGS} /EHsc)
add_test(NAME utpp_speedconv_terrain_test COMMAND utpp_speedconv_terrain_test)
# Mutations: 50 = every dip is held (a stop / wall too), 51 = the hold never ends; each must print its FAIL line.
foreach(_utpp_sct_mut 50 51)
    add_executable(utpp_speedconv_terrain_mut${_utpp_sct_mut} tests/SpeedConv_terrainTest.cpp src/mm/speedconv_terrain.hpp src/mm/SpeedConverter.hpp)
    target_compile_features(utpp_speedconv_terrain_mut${_utpp_sct_mut} PRIVATE cxx_std_20)
    target_compile_options(utpp_speedconv_terrain_mut${_utpp_sct_mut} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_definitions(utpp_speedconv_terrain_mut${_utpp_sct_mut} PRIVATE UTPP_SPEED_CONVERTER_MUTATION=${_utpp_sct_mut})
    add_test(NAME utpp_speedconv_terrain_mut${_utpp_sct_mut} COMMAND utpp_speedconv_terrain_mut${_utpp_sct_mut})
endforeach()
set_tests_properties(utpp_speedconv_terrain_mut50 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL no_step_dip_game_speed")
set_tests_properties(utpp_speedconv_terrain_mut51 PROPERTIES PASS_REGULAR_EXPRESSION "FAIL release_reaches_game_speed")
