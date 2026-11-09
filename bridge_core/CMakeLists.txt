cmake_minimum_required(VERSION 3.22)

project(GTContactBridgeCore VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)

add_library(gt_contact_bridge_core STATIC
    src/GTContactFrameCollector.cpp
)

target_include_directories(gt_contact_bridge_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(gt_contact_bridge_core PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(gt_contact_bridge_core PRIVATE /W4 /WX /permissive- /EHsc)
else()
    target_compile_options(gt_contact_bridge_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(BUILD_TESTING)
    add_executable(gt_contact_bridge_core_tests
        tests/GTContactFrameCollectorTests.cpp
    )
    target_link_libraries(gt_contact_bridge_core_tests PRIVATE gt_contact_bridge_core)
    target_compile_features(gt_contact_bridge_core_tests PRIVATE cxx_std_20)
    if(MSVC)
        target_compile_options(gt_contact_bridge_core_tests PRIVATE /W4 /WX /permissive- /EHsc)
    else()
        target_compile_options(gt_contact_bridge_core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME gt_contact_bridge_core_tests COMMAND gt_contact_bridge_core_tests)

    add_executable(gt_contact_abi_c_compile tests/abi_c_compile.c)
    target_include_directories(gt_contact_abi_c_compile PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    set_target_properties(gt_contact_abi_c_compile PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES)
    add_test(NAME gt_contact_abi_c_compile COMMAND gt_contact_abi_c_compile)
endif()
