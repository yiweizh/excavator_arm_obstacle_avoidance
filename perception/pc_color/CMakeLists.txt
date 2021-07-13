# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(pc_color)

# find_package(OpenCV REQUIRED)
# find_package(Curses REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${CURSES_INCLUDE_DIRS})
# message(${OpenCV_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} pc_color.cpp)
    
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${CURSES_LIBRARIES})
# set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER pc_saver)
