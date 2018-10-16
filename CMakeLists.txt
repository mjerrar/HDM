cmake_minimum_required(VERSION 3.2.1)
project (HDM)

# Add project cmake modules to path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(PCL 1.2 REQUIRED)



# We probably don't want this to run on every build.
option(COVERAGE "Generate Coverage Data" ON)

if (COVERAGE)
    include(CodeCoverage)
    set(LCOV_REMOVE_EXTRA "'vendor/*'")
    SETUP_TARGET_FOR_COVERAGE(code_coverage test/cpp-test coverage)
    set(COVERAGE_SRCS app/main.cpp include/main.h)

    SET(CMAKE_CXX_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
    SET(CMAKE_C_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage")
    SET(CMAKE_EXE_LINKER_FLAGS "--coverage -fprofile-arcs -ftest-coverage")
else()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -g")
endif()

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(app)
add_subdirectory(test)
add_subdirectory(vendor/googletest/googletest)




