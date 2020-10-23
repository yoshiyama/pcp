cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(eigen_matrix)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (eigen_matrix eigen_matrix.cpp)
target_link_libraries (eigen_matrix ${PCL_LIBRARIES})
