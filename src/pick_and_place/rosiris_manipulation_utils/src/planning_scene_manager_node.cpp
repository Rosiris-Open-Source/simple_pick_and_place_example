#include <rclcpp/rclcpp.hpp>
#include "rosiris_manipulation_utils/planning_scene_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  auto node = std::make_shared<rosiris_manipulation_utils::PlanningSceneManager>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}