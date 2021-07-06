cmake_minimum_required(VERSION 3.5.1)
project(segment)
# Add the application sources to the target
add_executable(${PROJECT_NAME} pcl_iterRANSAC.cpp)

# Link librealsense2 to the target
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})