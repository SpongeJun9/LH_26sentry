#ifndef CORE_SERIAL_NODE_H_
#define CORE_SERIAL_NODE_H_
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
// STD
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "core_global/msg/target.hpp"
#include "core_global/msg/time_info.hpp"
#include "core_global/msg/vision_msg.hpp"
#include "core_global/msg/watch_dog_msg.hpp"
#include "core_global/msg/serial_msg.hpp"
#include "core_global/msg/decision_msg.hpp"
#include "core_serial/core_serial_typedef.hpp"

namespace cs
{
    class CoreSerialNode : public rclcpp::Node
    {
    public:
        explicit CoreSerialNode(const rclcpp::NodeOptions &options);
        ~CoreSerialNode() override;

    private:
        void getSerialParams();
        void navSendCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
        void decisionSendCallback(const core_global::msg::DecisionMsg::SharedPtr decision_msg);
        void visionSendCallback(const core_global::msg::Target::ConstSharedPtr msg,
                                const core_global::msg::TimeInfo::ConstSharedPtr time_info);

        void reopenSerialPort();
        void receiveDataThread();
        // serial port
        std::unique_ptr<IoContext> owned_ctx_;
        std::string device_name_;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

        void setParam(const rclcpp::Parameter &param);
        void resetTracker();
        void changeTarget();

        // Param client to set detect_colr
        using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
        bool initial_set_param_ = false;
        int decision_send_cout = 0;
        int previous_receive_policy = -1;
        uint8_t previous_receive_color_ = 0;
        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        ResultFuturePtr set_param_future_;

        // Service client to reset tracker
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
        // Service client to change target
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_target_client_;
        // Aimimg point receiving from serial port for visualization
        visualization_msgs::msg::Marker aiming_point_;
        // Broadcast tf from odom to gimbal_link
        double timestamp_offset_ = 0;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        // rclcpp::Subscription<core_global::msg::Target>::SharedPtr aim_sub_;

        message_filters::Subscriber<core_global::msg::Target> aim_sub_;
        message_filters::Subscriber<core_global::msg::TimeInfo> aim_time_info_sub_;

        typedef message_filters::sync_policies::ApproximateTime<
            core_global::msg::Target, core_global::msg::TimeInfo>
            aim_syncpolicy;
        typedef message_filters::Synchronizer<aim_syncpolicy> AimSync;
        std::shared_ptr<AimSync> aim_sync_;

        // For debug usage
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

        std::thread receive_thread_;

        // Task message
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;

        // Time message
        rclcpp::Publisher<core_global::msg::TimeInfo>::SharedPtr aim_time_info_pub_;

        // --- send call back
        // void decisionCallback(const core_global::msg::DecisionMsg::SharedPtr decision_msg) const;
        // void watchDogCallback(const core_global::msg::WatchDogMsg::SharedPtr watch_dog_msg) const;
        // void receiveThread();

        // // nav订阅者
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_;
        // // vision订阅者
        // rclcpp::Subscription<core_global::msg::VisionMsg>::SharedPtr sub_vision_;
        // // decision订阅者
        rclcpp::Subscription<core_global::msg::DecisionMsg>::SharedPtr sub_decision_;
        // // watchdog订阅者
        // rclcpp::Subscription<core_global::msg::WatchDogMsg>::SharedPtr sub_watch_dog_;
        // // 发布者
        rclcpp::Publisher<core_global::msg::SerialMsg>::SharedPtr pub_serial_;
        // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    };
}

#endif