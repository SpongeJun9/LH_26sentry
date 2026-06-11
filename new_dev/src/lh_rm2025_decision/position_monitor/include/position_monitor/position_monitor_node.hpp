#ifndef POSITION_MONITOR_NODE_HPP
#define POSITION_MONITOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace position_monitor
{

class PositionMonitorNode : public rclcpp::Node
{
public:
  explicit PositionMonitorNode(const rclcpp::NodeOptions & options);
  ~PositionMonitorNode() = default;

private:
  void timerCallback();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::string target_frame_;
  std::string source_frame_;
  double publish_rate_;
};

}  // namespace position_monitor

#endif  // POSITION_MONITOR_NODE_HPP
