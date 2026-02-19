cmake_minimum_required(VERSION 3.20)
project(exercise_1_tower_of_hanoi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(PACKAGE_INCLUDE_DEPS 
  control_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
)
foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(tower_of_hanoi src/tower_of_hanoi.cpp)
target_link_libraries(tower_of_hanoi 
  moveit_ros_planning_interface::moveit_move_group_interface  
  rclcpp::rclcpp
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

install(TARGETS tower_of_hanoi
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch moveit_config_xml rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
