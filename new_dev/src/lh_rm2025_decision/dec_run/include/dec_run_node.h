#ifndef DEC_MAIN_NODE_H
#define DEC_MAIN_NODE_H
#include "rclcpp/rclcpp.hpp"
#include "core_global/msg/serial_msg.hpp"
#include "core_global/msg/decision_msg.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include "dec_run_typedef.h"
#include <thread>
#include "dec_run_list.hpp"
using std::placeholders::_1;

namespace dr
{
    class DecRunNode : public rclcpp::Node
    {
    private:
        // ---function---
        void init();
        void run();
        void getListParams();
        bool decisionOnce();
        std::shared_ptr<dr::Policy> choose_policy();
        void processSerialMsg(const core_global::msg::SerialMsg::SharedPtr serial_msg);
        void checkNavState();
        void checkGameState();
        void makeNavGoal(); // 添加决策路径点至缓冲区
        void sendNavGoal(); // 根据状态发布位置缓冲区中的元素
        void pubMsg();
        void initTwoPointSequence();
        void twoPointSequenceTick();
        void sendSinglePoseGoal(const geometry_msgs::msg::PoseStamped & pose);
        // ---function_callback---
        void serialCallback(const core_global::msg::SerialMsg::SharedPtr serial_msg);
        // void visionCallback(const core_global::msg::SerialMsg::SharedPtr serial_msg);
        void nav2FeedBackCallBack(const nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr nav2_fb_msg);
        void nav2GoalStatusCallBack(const action_msgs::msg::GoalStatusArray::SharedPtr nav2_goal_msg);
        // ---variable---

        rclcpp::Subscription<core_global::msg::SerialMsg>::SharedPtr sub_serial_;
        // rclcpp::Subscription<core_global::msg::VisionMsg>::SharedPtr sub_vision_;
        rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr sub_nav_feedback_;
        rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::SharedPtr sub_nav_goal_status_;
        // pub
        rclcpp::Publisher<core_global::msg::DecisionMsg>::SharedPtr pub_dec_;
        // action
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr client_nav_through_poses_action_;

        std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_; // 目标点队列缓冲区
        nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;

        std::unique_ptr<dr::DecRunList> list_;

        enum dr::State state_;
        enum dr::VisionRegion vision_region_;

        dr::NavInfo nav_info_;
        dr::VisionInfo vision_info_;
        dr::GameInfo game_info_;
        dr::PolicyInfo policy_info_;

        // std::thread run_thread_;
        // two-point sequence
        bool two_point_sequence_enabled_{false};
        int two_point_seq_stage_{0};
        rclcpp::TimerBase::SharedPtr two_point_timer_;
        rclcpp::TimerBase::SharedPtr two_point_pause_timer_;
        geometry_msgs::msg::PoseStamped seq_pose1_;
        geometry_msgs::msg::PoseStamped seq_pose2_;
        rclcpp::Time seq_goal_start_time_;
        double seq_goal_timeout_s_{120.0};

    public:
        explicit DecRunNode(const rclcpp::NodeOptions &options);
        ~DecRunNode() override;
    };
}

#endif
