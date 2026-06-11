#include "position_monitor/position_monitor_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace position_monitor
{

PositionMonitorNode::PositionMonitorNode(const rclcpp::NodeOptions & options)
: Node("position_monitor", options)
{
  // 声明参数
  target_frame_ = this->declare_parameter("target_frame", "map");
  source_frame_ = this->declare_parameter("source_frame", "base_link");
  publish_rate_ = this->declare_parameter("publish_rate", 10.0);

  // 初始化 TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建发布器
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "current_pose", 10);

  // 创建定时器
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&PositionMonitorNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Position Monitor Node started!");
  RCLCPP_INFO(this->get_logger(), "Monitoring transform: %s -> %s at %.1f Hz",
              target_frame_.c_str(), source_frame_.c_str(), publish_rate_);
}

void PositionMonitorNode::timerCallback()
{
  try
  {
    // 获取变换
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

    // 转换为 PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = transform_stamped.header;
    pose_msg.pose.position.x = transform_stamped.transform.translation.x;
    pose_msg.pose.position.y = transform_stamped.transform.translation.y;
    pose_msg.pose.position.z = transform_stamped.transform.translation.z;
    pose_msg.pose.orientation = transform_stamped.transform.rotation;

    // 发布位置
    pose_pub_->publish(pose_msg);

    // 直接打印坐标
    RCLCPP_INFO(this->get_logger(), "当前位置: x=%.3f, y=%.3f, z=%.3f",
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "无法获取变换: %s", ex.what());
  }
}

}  // namespace position_monitor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(position_monitor::PositionMonitorNode)
