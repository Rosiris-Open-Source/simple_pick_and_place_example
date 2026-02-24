cmake_minimum_required(VERSION 3.20)
project(rosiris_manipulation_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  moveit_msgs
  std_msgs
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(msg_files
  msg/CollisionEntry.msg
  msg/CollisionMatrixUpdate.msg
  msg/CollisionObject.msg
  msg/CollisionObjectPoseUpdate.msg
  msg/ServiceErrorCode.msg
  msg/ServiceResult.msg
)

set(srv_files
  srv/AddCollisionObjects.srv
  srv/AttachCollisionObject.srv
  srv/DetachCollisionObject.srv
  srv/GetAttachedCollisionObjectIds.srv
  srv/GetCollisionObjectIds.srv
  srv/LoadScenario.srv
  srv/MoveCollisionObjects.srv
  srv/RemoveCollisionObjects.srv
  srv/ResetScenario.srv
  srv/UpdateAllowedCollisions.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces ${THIS_PACKAGE_INCLUDE_DEPENDS}
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
