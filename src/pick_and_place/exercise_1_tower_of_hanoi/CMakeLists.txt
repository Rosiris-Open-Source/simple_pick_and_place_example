cmake_minimum_required(VERSION 3.20)
project(exercise_1_tower_of_hanoi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(PACKAGE_INCLUDE_DEPS 
  control_msgs
  generate_parameter_library
  geometry_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  rosiris_manipulation_interfaces
  rviz_visual_tools
  tf2
  transform_manager
)
foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

# parameters for tower of hanoi node
generate_parameter_library(
  tower_of_hanoi_parameters 
  src/tower_of_hanoi_parameters.yaml 
)

# tower of hanoi node
add_executable(tower_of_hanoi src/tower_of_hanoi.cpp)

target_include_directories(tower_of_hanoi PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(tower_of_hanoi 
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_visual_tools::moveit_visual_tools
  rclcpp::rclcpp
  rviz_visual_tools::rviz_visual_tools
  transform_manager::transform_manager
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${rosiris_manipulation_interfaces_TARGETS}
  tower_of_hanoi_parameters
)

install(TARGETS tower_of_hanoi
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz scenarios
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
