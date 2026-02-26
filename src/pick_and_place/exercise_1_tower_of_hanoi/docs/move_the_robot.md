1. explain planners

  // 5. Move Linear (Post-grasp 5cm +y)
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose post_grasp = move_group.getCurrentPose().pose;
  post_grasp.position.y += 0.05;
  waypoints.push_back(post_grasp);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction =
      move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  if (fraction > 0.9) {
    move_group.execute(trajectory);
  }


vs

PILZ:
  // configure to move lin
  configurePlannerPilzLin(move_group);

  // 3. Move to Grasp Point
  grasp_pose.pose.position.y = 0.0;
  move_group.setPoseTarget(grasp_pose);
  RCLCPP_INFO(this->get_logger(), "Planning to grasp pose...");
  success = move_group.plan(my_plan);

  if (success == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
    move_group.execute(my_plan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed! Error code: %d",
                 success.val);
    return;
  }


vs

OMPL:
  configurePlannerOmpl(move_group);

  geometry_msgs::msg::PoseStamped place_pose;
  place_pose.header.frame_id = "desk_1_place_location_middle";
  place_pose.pose.position.z = 0.5; // 5cm offset
  double roll = deg2rad(90.0);
  double pitch = deg2rad(90.0);
  double yaw = 0.0;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  place_pose.pose.orientation = tf2::toMsg(q);

  move_group.setPoseTarget(place_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_place;
  RCLCPP_INFO(this->get_logger(), "Planning to place pose...");
  success = move_group.plan(my_plan_place);

  if (success == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
    move_group.execute(my_plan_place);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed! Error code: %d",
                 success.val);
    return;
  }
