cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(supervoxel_plane_segmentation)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable (main
    main.cpp
    region_manager.cpp
    loader.cpp
    utils.cpp
    configuration.cpp
)

target_link_libraries (main
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)