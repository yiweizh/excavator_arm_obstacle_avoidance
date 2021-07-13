cmake_minimum_required(VERSION 3.5.1)
project(capstone)
# Find librealsense2 installed package
find_package(realsense2 REQUIRED)
# Find libpcl-dev Point Cloud Library 1.7.2
find_package( PCL REQUIRED)
# find_library(PCL_LIBRARY_DIRS PATHS "/usr/local/lib" NO_DEFAULT_PATH)
# Find boost 
find_package( Boost REQUIRED)
find_package(OpenGL REQUIRED)
set(DEPENDENCIES realsense2 glfw ${OPENGL_LIBRARIES})
include_directories(${PCL_INCLUDE_DIRS})
message("PCL Include Directory: ${PCL_INCLUDE_DIRS}")
include_directories(${realsense2_INCLUDE_DIRS})
message(STATUS "realsense dirs ${realsense2_INCLUDE_DIRS}")
link_directories(${PCL_LIBRARY_DIRS})
message(STATUS "pcl library ${PCL_LIBRARY_DIRS}")
add_definitions(${PCL_DEFINITIONS})
include_directories(${Boost_INCLUDE_DIR})
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_subdirectory(segment)
add_subdirectory(visualizer)
add_subdirectory(pc_saver)
add_subdirectory(pc_fusion)
