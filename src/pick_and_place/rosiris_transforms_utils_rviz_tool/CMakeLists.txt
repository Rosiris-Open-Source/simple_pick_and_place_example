cmake_minimum_required(VERSION 3.8)
project(rosiris_transforms_utils_rviz_tool)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

set(plugin_headers
  include/rosiris_transforms_utils_rviz_tool/rosiris_transforms_utils_rviz_tool.hpp
)

add_library(${PROJECT_NAME} SHARED
  ${plugin_headers}
  src/rosiris_transforms_utils_rviz_tool.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  ${visualization_msgs_TARGETS}
  Qt5::Widgets
)

# Export the plugin
pluginlib_export_plugin_description_file(
  rviz_common plugin_description.xml
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()