add_executable(shell-app main.cpp voxel.cpp passthrough.cpp)
# Link the PCL_LIBS with binary
target_link_libraries( shell-app ${PCL_LIBRARIES} )


include_directories(

    ${CMAKE_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    
)
