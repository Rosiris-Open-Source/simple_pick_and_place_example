#ifndef ROSIRIS_TRANSFORMS_UTILS_RVIZ_TOOL_HPP_
#define ROSIRIS_TRANSFORMS_UTILS_RVIZ_TOOL_HPP_

#include <QPushButton>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rosiris_transforms_utils_rviz_tool {
class HelloWorldPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit HelloWorldPanel(QWidget *parent = nullptr);
  virtual void onInitialize() override;

protected Q_SLOTS:
  void onButtonClicked();

protected:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Node::SharedPtr node_;
};
} // namespace rosiris_transforms_utils_rviz_tool

#endif