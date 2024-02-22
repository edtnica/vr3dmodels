cmake_minimum_required(VERSION 3.16.3 FATAL_ERROR)

project(visualizer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (visualizer visualizer.cpp)
target_link_libraries (visualizer ${PCL_LIBRARIES})