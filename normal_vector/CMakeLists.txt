cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(normal_vector)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable (normal_vector normal_vector.cpp)
<<<<<<< HEAD
target_link_libraries (normal_vector ${PCL_LIBRARIES})
=======
target_link_libraries (normal_vector ${PCL_LIBRARIES})
>>>>>>> 55d748024fe63e77fe1ef275868aede65146ef8d
