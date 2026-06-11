// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// STD
#include <array>

#include "core_serial/core_serial_node.h"
#include "core_serial/crc.hpp"

using std::placeholders::_1; // 占位符
using std::placeholders::_2;

std::array<double, 9> covariance_li = {{-1, 0, 0, 0, 0, 0, 0, 0, 0}};
std::array<double, 9> covariance_or = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}};
std::array<double, 9> covariance_an = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}};

namespace cs
{
  CoreSerialNode::CoreSerialNode(const rclcpp::NodeOptions &options)
      : Node("core_serial_node", options),
        owned_ctx_{new IoContext(2)},
        serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
  {
    RCLCPP_INFO(get_logger(), "Start CoreSerialNode!");
    getSerialParams();

    // TF broadcaster
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create Publisher
    task_pub_ = this->create_publisher<std_msgs::msg::String>("/task_mode", 10);
    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
    aim_time_info_pub_ =
        this->create_publisher<core_global::msg::TimeInfo>("/time_info/aim", 10);
    pub_serial_ = this->create_publisher<core_global::msg::SerialMsg>("/serial_msg", 10);

    sub_nav_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&CoreSerialNode::navSendCallback, this, _1));
    sub_decision_ = this->create_subscription<core_global::msg::DecisionMsg>("/decision_msg", 10, std::bind(&CoreSerialNode::decisionSendCallback, this, _1));
    // Detect parameter client
    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

    // Tracker reset service client
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

    // Target change service cilent
    change_target_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/change");

    try
    {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open())
      {
        serial_driver_->port()->open();
        receive_thread_ = std::thread(&CoreSerialNode::receiveDataThread, this);
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(
          get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
      throw ex;
    }

    aiming_point_.header.frame_id = "odom_v";
    aiming_point_.ns = "aiming_point";
    aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
    aiming_point_.action = visualization_msgs::msg::Marker::ADD;
    aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
    aiming_point_.color.r = 1.0;
    aiming_point_.color.g = 1.0;
    aiming_point_.color.b = 1.0;
    aiming_point_.color.a = 1.0;
    aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // Create Subscription
    // aim_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    //   "/tracker/target", rclcpp::SensorDataQoS(),
    //   std::bind(&core_global::sendArmorData, this, std::placeholders::_1));
    aim_sub_.subscribe(this, "/tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    aim_time_info_sub_.subscribe(this, "/time_info/aim");
    // rune_sub_.subscribe(this, "/tracker/rune");

    aim_sync_ = std::make_unique<AimSync>(aim_syncpolicy(500), aim_sub_, aim_time_info_sub_);
    aim_sync_->registerCallback(
        std::bind(&CoreSerialNode::visionSendCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  CoreSerialNode::~CoreSerialNode()
  {
    if (receive_thread_.joinable())
    {
      receive_thread_.join();
    }

    if (serial_driver_->port()->is_open())
    {
      serial_driver_->port()->close();
    }

    if (owned_ctx_)
    {
      owned_ctx_->waitForExit();
    }
  }

  void CoreSerialNode::visionSendCallback(
      const core_global::msg::Target::ConstSharedPtr msg,
      const core_global::msg::TimeInfo::ConstSharedPtr time_info)
  {
    const static std::map<std::string, uint8_t> id_unit8_map{{"", 0}, {"outpost", 0}, {"1", 1}, {"1", 1}, {"2", 2}, {"3", 3}, {"4", 4}, {"5", 5}, {"guard", 6}, {"base", 7}};

    try
    {
      cs::VisionSendPacket vision_packet;
      vision_packet.state = msg->tracking ? 1 : 0;
      vision_packet.id = id_unit8_map.at(msg->id);
      vision_packet.armors_num = msg->armors_num;
      vision_packet.x = msg->position.x;
      vision_packet.y = msg->position.y;
      vision_packet.z = msg->position.z;
      vision_packet.yaw = msg->yaw;
      vision_packet.vx = msg->velocity.x;
      vision_packet.vy = msg->velocity.y;
      vision_packet.vz = msg->velocity.z;
      vision_packet.v_yaw = msg->v_yaw;
      vision_packet.r1 = msg->radius_1;
      vision_packet.r2 = msg->radius_2;
      vision_packet.dz = msg->dz;
      // 20240329 ZY: Eliminate communication latency
      vision_packet.t_offset = (this->now().seconds() / 1000.0) - time_info->time;
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&vision_packet), sizeof(vision_packet));

      std::vector<uint8_t> data = toVector(vision_packet);

      serial_driver_->port()->send(data);

      std_msgs::msg::Float64 latency;
      latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
      RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
      latency_pub_->publish(latency);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
      reopenSerialPort();
    }
  }
  void CoreSerialNode::navSendCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
  {
    try
    {
      RCLCPP_INFO(get_logger(), "sensen");
      cs::NavSend nav_packet;
      nav_packet.x = cmd_vel->linear.x;
      nav_packet.y = cmd_vel->linear.y;
      std::cout << cmd_vel->linear.x << "y:" << cmd_vel->linear.y << std::endl;

      std::vector<uint8_t> data = toVector(nav_packet);

      serial_driver_->port()->send(data);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
      reopenSerialPort();
    }
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("id"), cmd_vel->linear.x);
  }
  void CoreSerialNode::decisionSendCallback(const core_global::msg::DecisionMsg::SharedPtr decision_msg)
  {
    try
    {
      if (decision_msg->id != previous_receive_policy)
      {
        this->decision_send_cout = 0;
      }
      if (this->decision_send_cout < 20)
      {
        cs::DecSend dec_packet;
        dec_packet.if_spin = decision_msg->if_spin;
        dec_packet.spin_speed = decision_msg->spin_speed;
        dec_packet.if_super_cat = decision_msg->if_super_cat;

        std::vector<uint8_t> data = toVector(dec_packet);
        this->decision_send_cout++;
        serial_driver_->port()->send(data);
      }
      this->previous_receive_policy = decision_msg->id;
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
      reopenSerialPort();
    }
  }
  void CoreSerialNode::receiveDataThread()
  {
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    data.reserve(sizeof(ReceivePacket));

    while (rclcpp::ok())
    {
      try
      {
        serial_driver_->port()->receive(header);

        if (header[0] == 0x5A)
        {
          data.resize(sizeof(ReceivePacket) - 1);
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          serial_driver_->port()->receive(data);
          data.insert(data.begin(), header[0]);
          ReceivePacket packet = fromVector(data);
          bool crc_ok =
              crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
          if (crc_ok)
          {
            core_global::msg::SerialMsg msg;
            msg.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
            msg.header.frame_id = "serial";
            msg.game_time = packet.game_time;
            msg.center_state = packet.center_state;
            msg.if_in_hp = packet.if_in_hp;
            msg.if_in_center = packet.if_in_center;
            msg.sentry_hp = packet.sentry_hp;
            msg.aim_x = packet.aim_x;
            msg.aim_y = packet.aim_y;
            msg.aim_z = packet.aim_z;
            this->pub_serial_->publish(msg);

            if (!initial_set_param_ || packet.detect_color != previous_receive_color_)
            {
              setParam(rclcpp::Parameter("detect_color", packet.detect_color));
              previous_receive_color_ = packet.detect_color;
            }

            if (packet.reset_tracker)
            {
              resetTracker();
            }

            if (packet.change_target)
            {
              changeTarget();
            }

            // tbd
            std_msgs::msg::String task;
            std::string theory_task;
            task.data = "aim";
            task_pub_->publish(task);

            // RCLCPP_DEBUG(
            //     get_logger(), "Game time: %d, Task mode: %d, Theory task: %s", packet.game_time,
            //     packet.task_mode, theory_task.c_str());

            geometry_msgs::msg::TransformStamped t;
            timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
            t.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
            t.header.frame_id = "odom_v";
            t.child_frame_id = "gimbal_link";
            tf2::Quaternion q;
            q.setRPY(packet.eular[2], packet.eular[1], packet.eular[0]);
            t.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(t);

            // publish time
            core_global::msg::TimeInfo aim_time_info;
            aim_time_info.header = t.header;
            aim_time_info.time = this->now().seconds() / 1000.0; // ms
            aim_time_info_pub_->publish(aim_time_info);

            if (abs(packet.aim_x) > 0.01)
            {
              aiming_point_.header.stamp = this->now();
              aiming_point_.pose.position.x = packet.aim_x;
              aiming_point_.pose.position.y = packet.aim_y;
              aiming_point_.pose.position.z = packet.aim_z;
              marker_pub_->publish(aiming_point_);
            }
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "CRC error!");
          }
        }
        else
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
        }
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
        reopenSerialPort();
      }
    }
  }
  void CoreSerialNode::getSerialParams()
  {
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try
    {
      device_name_ = declare_parameter<std::string>("device_name", "");
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }

    try
    {
      baud_rate = declare_parameter<int>("baud_rate", 0);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }

    try
    {
      const auto fc_string = declare_parameter<std::string>("flow_control", "");

      if (fc_string == "none")
      {
        fc = FlowControl::NONE;
      }
      else if (fc_string == "hardware")
      {
        fc = FlowControl::HARDWARE;
      }
      else if (fc_string == "software")
      {
        fc = FlowControl::SOFTWARE;
      }
      else
      {
        throw std::invalid_argument{
            "The flow_control parameter must be one of: none, software, or "
            "hardware."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
      throw ex;
    }

    try
    {
      const auto pt_string = declare_parameter<std::string>("parity", "");

      if (pt_string == "none")
      {
        pt = Parity::NONE;
      }
      else if (pt_string == "odd")
      {
        pt = Parity::ODD;
      }
      else if (pt_string == "even")
      {
        pt = Parity::EVEN;
      }
      else
      {
        throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
      throw ex;
    }

    try
    {
      const auto sb_string = declare_parameter<std::string>("stop_bits", "");

      if (sb_string == "1" || sb_string == "1.0")
      {
        sb = StopBits::ONE;
      }
      else if (sb_string == "1.5")
      {
        sb = StopBits::ONE_POINT_FIVE;
      }
      else if (sb_string == "2" || sb_string == "2.0")
      {
        sb = StopBits::TWO;
      }
      else
      {
        throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
      throw ex;
    }

    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  }
  void CoreSerialNode::reopenSerialPort()
  {
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try
    {
      if (serial_driver_->port()->is_open())
      {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      if (rclcpp::ok())
      {
        rclcpp::sleep_for(std::chrono::seconds(1));
        this->reopenSerialPort();
      }
    }
  }
  void CoreSerialNode::setParam(const rclcpp::Parameter &param)
  {
    if (!detector_param_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
      return;
    }

    if (
        !set_param_future_.valid() ||
        set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
      set_param_future_ = detector_param_client_->set_parameters(
          {param}, [this, param](const ResultFuturePtr &results)
          {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true; });
    }
  }

  void CoreSerialNode::resetTracker()
  {
    if (!reset_tracker_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_tracker_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Reset tracker!");
  }

  void CoreSerialNode::changeTarget()
  {
    if (!change_target_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping target change");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    change_target_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Change target!");
  }
}
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(cs::CoreSerialNode)
