cmake_minimum_required(VERSION 3.20)
project(exercise_0_moveit2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
