# ---- mesh-space arm clavicle limit (src/mm/MeshArmClavicleLimit.hpp): unit + synthetic ComposeLayers + real woman data (PoseEvaluator).
# Included from plugin/UTPP/CMakeLists.txt after motion_pose.cmake (uses its _utpp_pmotion_* paths). Mutation builds MUST print a "FAIL " line:
# 1 = no limit, 2 = children of the clavicle not compensated, 3 = no shortest-arc (hemisphere) fix, 4 = the transient bound never engages (section I).
if(DEFINED _utpp_pmotion_wov AND DEFINED _utpp_pmotion_wdb AND DEFINED _utpp_pmotion_wovdir)
    set(_utpp_clavlimit_args "${_utpp_pmotion_wdb}" "${_utpp_pmotion_wov}" "${_utpp_pmotion_wovdir}/utpp_style_weights_woman.txt")
else()
    set(_utpp_clavlimit_args)
endif()
add_executable(utpp_clavicle_limit_test tests/ClavicleLimitTest.cpp src/mm/MeshArmClavicleLimit.hpp src/mm/MotionPose.hpp)
target_compile_options(utpp_clavicle_limit_test PRIVATE ${UTPP_WARNINGS} /EHsc)
target_compile_features(utpp_clavicle_limit_test PRIVATE cxx_std_20)
add_test(NAME utpp_clavicle_limit_test COMMAND utpp_clavicle_limit_test ${_utpp_clavlimit_args})
foreach(_m 1 2 3 4)
    add_executable(utpp_clavicle_limit_mut${_m} tests/ClavicleLimitTest.cpp src/mm/MeshArmClavicleLimit.hpp src/mm/MotionPose.hpp)
    target_compile_options(utpp_clavicle_limit_mut${_m} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(utpp_clavicle_limit_mut${_m} PRIVATE cxx_std_20)
    target_compile_definitions(utpp_clavicle_limit_mut${_m} PRIVATE UTPP_CLAVLIMIT_MUTATION=${_m})
    add_test(NAME utpp_clavicle_limit_mut${_m} COMMAND utpp_clavicle_limit_mut${_m} ${_utpp_clavlimit_args})
    set_tests_properties(utpp_clavicle_limit_mut${_m} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL ")
endforeach()

# Transient bound of the clavicle (tests/ClavicleTransientTest.cpp, ClavicleTransientBound): stop / start clips of both motion databases
# through the weapon pose crossfade. The motion databases sit one folder above the overlay files (overlay.cmake). Mutation builds run
# the rifle sets only and must print a "FAIL CTB-" line: 4 = the bound never engages, 5 = released in one frame.
get_filename_component(_utpp_ctb_wdir "${UTPP_OVERLAY_WOMAN_FILE}" DIRECTORY)
get_filename_component(_utpp_ctb_mdir "${UTPP_OVERLAY_MAN_FILE}" DIRECTORY)
set(_utpp_ctb_args "${_utpp_ctb_wdir}/../utpp_motion_woman.utppmm" "${UTPP_OVERLAY_WOMAN_FILE}"
    "${_utpp_ctb_mdir}/../utpp_motion_man.utppmm" "${UTPP_OVERLAY_MAN_FILE}")
foreach(_m 0 4 5)
    if(_m EQUAL 0)
        set(_utpp_ctb_name utpp_clavicle_transient_test)
    else()
        set(_utpp_ctb_name utpp_clavicle_transient_mut${_m})
    endif()
    add_executable(${_utpp_ctb_name} tests/ClavicleTransientTest.cpp src/mm/MeshArmClavicleLimit.hpp src/mm/Overlay.hpp
                   src/mm/OverlayFile.hpp src/mm/MotionDatabase.hpp)
    target_compile_options(${_utpp_ctb_name} PRIVATE ${UTPP_WARNINGS} /EHsc)
    target_compile_features(${_utpp_ctb_name} PRIVATE cxx_std_20)
    add_test(NAME ${_utpp_ctb_name} COMMAND ${_utpp_ctb_name} ${_utpp_ctb_args})
    if(NOT _m EQUAL 0)
        target_compile_definitions(${_utpp_ctb_name} PRIVATE UTPP_CLAVLIMIT_MUTATION=${_m})
        set_tests_properties(${_utpp_ctb_name} PROPERTIES PASS_REGULAR_EXPRESSION "FAIL CTB-")
    endif()
endforeach()
