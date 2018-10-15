set(GTEST_SHUFFLE 1)

find_package(PCL 1.2 REQUIRED)

add_executable(
    cpp-test
    main.cpp
    test.cpp
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS} ../app)
target_link_libraries(cpp-test PUBLIC gtest ${PCL_LIBRARIES})
