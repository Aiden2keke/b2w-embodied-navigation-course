cmake_minimum_required(VERSION 3.16)
project(nav2_controller_guard_regression)
find_package(ament_cmake REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Compile the exact production method, not a handwritten copy of the guard.
execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/extract_method.py
  ${CONTROLLER_SOURCE} ${CMAKE_CURRENT_BINARY_DIR}/goal_method.inc
  COMMAND_ERROR_IS_FATAL ANY)
add_executable(regression regression.cpp)
target_include_directories(regression PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
ament_target_dependencies(regression nav2_controller nav_2d_utils tf2_ros)
find_library(GOAL_CHECKER simple_goal_checker
  HINTS "${nav2_controller_DIR}/../../../lib" REQUIRED)
target_link_libraries(regression ${GOAL_CHECKER})
enable_testing()
add_test(NAME endpoint_transform_guard COMMAND regression)
