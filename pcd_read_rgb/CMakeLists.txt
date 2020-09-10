cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(basic pcd_read_rgb)
project(${basic})

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (${basic} ${basic}.cpp)
target_link_libraries (${basic} ${PCL_LIBRARIES})
