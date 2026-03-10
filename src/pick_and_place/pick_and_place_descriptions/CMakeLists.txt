cmake_minimum_required(VERSION 3.20)
project(pick_and_place_descriptions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY grippers/ 
  DESTINATION share/${PROJECT_NAME}/grippers
)

install(
  DIRECTORY robots/ 
  DESTINATION share/${PROJECT_NAME}/robots
)

install(
  DIRECTORY tower_of_hanoi/
  DESTINATION share/${PROJECT_NAME}/tower_of_hanoi
)

ament_package()
