#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "core_global/msg/decision_msg.hpp"
using std::placeholders::_1;

namespace dr
{
  class TestNavNode : public rclcpp::Node
  {
  public:
    explicit TestNavNode(const rclcpp::NodeOptions & options)
    : Node("test_nav_node", options)
    {
      RCLCPP_INFO(this->get_logger(), "test_nav_node 启动");
      goal_timeout_s_ = this->declare_parameter<double>("goal_timeout_s", 120.0);
      pub_dec_ = this->create_publisher<core_global::msg::DecisionMsg>("/decision_msg", 10);
      dec_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&TestNavNode::pubMsg, this));
      initGoals();
      sub_status_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_through_poses/_action/status",
        10,
        std::bind(&TestNavNode::statusCallback, this, _1));
      client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
        this, "navigate_through_poses");
      if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "等待导航动作服务器失败");
        state_ = State::ERROR;
        return;
      }
      RCLCPP_INFO(this->get_logger(), "已连接至导航动作服务器");
      sendSinglePoseGoal(goal1_);
    }

  private:
    enum class State { IDLE, SENDING, WAITING_PAUSE, DONE, ERROR };

    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr client_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_status_;
    rclcpp::Publisher<core_global::msg::DecisionMsg>::SharedPtr pub_dec_;
    rclcpp::TimerBase::SharedPtr dec_timer_;

    geometry_msgs::msg::PoseStamped goal1_;
    geometry_msgs::msg::PoseStamped goal2_;

    rclcpp::TimerBase::SharedPtr pause_timer_;
    rclcpp::Time goal_start_time_;
    double goal_timeout_s_{120.0};

    int stage_{0};
    State state_{State::IDLE};

    void initGoals()
    {
      auto now = this->get_clock()->now();
      goal1_.header.stamp = now;
      goal1_.header.frame_id = "map";
      goal1_.pose.position.x = 2.719;
      goal1_.pose.position.y = 4.938;
      goal1_.pose.position.z = 0.0;
      goal1_.pose.orientation.x = 0.0;
      goal1_.pose.orientation.y = 0.0;
      goal1_.pose.orientation.z = 0.0;
      goal1_.pose.orientation.w = 1.0;

      goal2_.header.stamp = now;
      goal2_.header.frame_id = "map";
      goal2_.pose.position.x = 0.0;
      goal2_.pose.position.y = 0.0;
      goal2_.pose.position.z = 0.0;
      goal2_.pose.orientation.x = 0.0;
      goal2_.pose.orientation.y = 0.0;
      goal2_.pose.orientation.z = 0.0;
      goal2_.pose.orientation.w = 1.0;
    }

    void sendSinglePoseGoal(const geometry_msgs::msg::PoseStamped & pose)
    {
      nav2_msgs::action::NavigateThroughPoses::Goal goal_msg;
      goal_msg.poses.clear();
      goal_msg.poses.push_back(pose);
      if (!client_->wait_for_action_server(std::chrono::milliseconds(100))) {
        RCLCPP_ERROR(this->get_logger(), "导航动作服务器不可用");
        state_ = State::ERROR;
        return;
      }
      auto options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
      (void)client_->async_send_goal(goal_msg, options);
      goal_start_time_ = this->get_clock()->now();
      state_ = State::SENDING;
      RCLCPP_INFO(this->get_logger(), "已发送目标点: (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);
    }

    void statusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
      if (msg->status_list.empty()) {
        return;
      }
      const auto status = msg->status_list.back().status;
      if (state_ == State::SENDING) {
        if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "导航完成，阶段: %d", stage_);
          if (stage_ == 0) {
            stage_ = 1;
            RCLCPP_INFO(this->get_logger(), "等待2秒后发送下一个目标");
            pause_timer_ = this->create_wall_timer(
              std::chrono::seconds(2),
              [this]() {
                pause_timer_->cancel();
                pause_timer_.reset();
                sendSinglePoseGoal(goal2_);
                state_ = State::SENDING;
              }
            );
            state_ = State::WAITING_PAUSE;
          } else {
            state_ = State::DONE;
            RCLCPP_INFO(this->get_logger(), "两个点导航已完成");
          }
        } else if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
          RCLCPP_WARN(this->get_logger(), "导航失败，取消目标");
          client_->async_cancel_all_goals();
          state_ = State::ERROR;
        } else {
          const double elapsed = (this->get_clock()->now() - goal_start_time_).seconds();
          if (elapsed > goal_timeout_s_) {
            RCLCPP_WARN(this->get_logger(), "导航超时(>%.0f s)，取消目标", goal_timeout_s_);
            client_->async_cancel_all_goals();
            state_ = State::ERROR;
          }
        }
      }
    }

    void pubMsg()
    {
      core_global::msg::DecisionMsg decision_msg;
      decision_msg.header.frame_id = "decision";
      decision_msg.header.stamp = rclcpp::Clock().now();
      decision_msg.id = 0;
      decision_msg.mode = 0;
      decision_msg.if_spin = false;
      decision_msg.spin_speed = 0;
      decision_msg.if_super_cat = 1;
      pub_dec_->publish(decision_msg);
    }
  };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dr::TestNavNode)
