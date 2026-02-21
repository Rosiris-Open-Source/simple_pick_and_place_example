cmake_minimum_required(VERSION 3.20)
project(rosiris_manipulation_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp
  rosiris_manipulation_interfaces
  shape_msgs
  std_srvs
  tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(planning_scene_manager_component SHARED
  src/planning_scene_manager.cpp
)
target_include_directories(planning_scene_manager_component PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(planning_scene_manager_component  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS 
  planning_scene_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(planning_scene_manager_node
  src/planning_scene_manager_node.cpp
) 

target_link_libraries(planning_scene_manager_node
  planning_scene_manager_component
)

ament_target_dependencies(planning_scene_manager_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS
  planning_scene_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Python modules
ament_python_install_package(rosiris_manipulation_utils)

install(PROGRAMS
  rosiris_manipulation_utils/scenario_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
