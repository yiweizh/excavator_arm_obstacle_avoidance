cmake_minimum_required(VERSION 3.5.1)
project(capstone)
# Find librealsense2 installed package
find_package(realsense2 REQUIRED)
# Find libpcl-dev Point Cloud Library 1.7.2
find_package( PCL 1.7 REQUIRED)
# Find boost 
find_package( Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${Boost_INCLUDE_DIR})
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_subdirectory(segment)
add_subdirectory(visualizer)
