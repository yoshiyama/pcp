cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl-matrix_transform4pcd)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (matrix_transform4pcd matrix_transform4pcd.cpp)
target_link_libraries (matrix_transform4pcd ${PCL_LIBRARIES})
