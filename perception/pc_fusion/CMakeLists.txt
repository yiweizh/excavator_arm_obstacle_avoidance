cmake_minimum_required(VERSION 3.5.1)
project(pc_fusion)
add_executable(${PROJECT_NAME} pc_fusion.cpp)


# target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})