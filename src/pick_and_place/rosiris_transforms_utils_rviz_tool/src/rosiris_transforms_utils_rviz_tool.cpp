#include "rosiris_transforms_utils_rviz_tool/rosiris_transforms_utils_rviz_tool.hpp"

#include <rviz_common/display_context.hpp>

namespace rosiris_transforms_utils_rviz_tool {
HelloWorldPanel::HelloWorldPanel(QWidget *parent) : rviz_common::Panel(parent) {
  QVBoxLayout *layout = new QVBoxLayout;
  QPushButton *button = new QPushButton("Display Hello World");
  layout->addWidget(button);
  setLayout(layout);

  connect(button, &QPushButton::clicked, this,
          &HelloWorldPanel::onButtonClicked);
}

void HelloWorldPanel::onInitialize() {
  // Access the hidden node created by RViz
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "hello_world_marker", 10);
}

void HelloWorldPanel::onButtonClicked() {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id =
      "map"; // Ensure this frame exists in your RViz config
  marker.header.stamp = node_->now();
  marker.ns = "hello_world";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.z = 1.0; // Float it slightly
  marker.scale.z = 0.5;         // Text height
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.text = "Hello World";

  marker_pub_->publish(marker);
}
} // namespace rosiris_transforms_utils_rviz_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rosiris_transforms_utils_rviz_tool::HelloWorldPanel,
                       rviz_common::Panel)