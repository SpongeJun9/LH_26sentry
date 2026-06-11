#include "core_global/msg/serial_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

class DecFakePublisher : public rclcpp::Node
{
public:
    DecFakePublisher() : Node("dec_fake_publisher_node")
    {
        init_parameters();
        publisher_ = create_publisher<core_global::msg::SerialMsg>("/serial_msg", 10);

        double interval = get_parameter("publish_interval").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(interval),
            std::bind(&DecFakePublisher::publish_message, this));

        RCLCPP_INFO(get_logger(), "DecFakePublisher 节点已启动");
    }

private:
    void init_parameters()
    {
        declare_parameter("enable_debug", true);
        declare_parameter("publish_interval", 0.5);
        declare_parameter("sentry_hp", 400);
        declare_parameter("game_time", 300);
        declare_parameter("if_in_hp", 0);
        declare_parameter("if_sentry_occupy", 0);
        declare_parameter("center_state", 0);
        declare_parameter("aim_x", 0.0);
        declare_parameter("aim_y", 0.0);
        declare_parameter("aim_z", 0.0);
    }

    void publish_message()
    {
        if (!get_parameter("enable_debug").as_bool())
            return;

        core_global::msg::SerialMsg msg;
        msg.sentry_hp = get_parameter("sentry_hp").as_int();
        msg.game_time = get_parameter("game_time").as_int();
        msg.if_in_hp = get_parameter("if_in_hp").as_int();
        msg.if_in_center = get_parameter("if_sentry_occupy").as_int();
        msg.center_state = get_parameter("center_state").as_int();
        msg.aim_x = get_parameter("aim_x").as_double();
        msg.aim_y = get_parameter("aim_y").as_double();
        msg.aim_z = get_parameter("aim_z").as_double();

        publisher_->publish(msg);

        RCLCPP_INFO(get_logger(),
            "发布数据 | 血量:%d 时间:%d 回血区:%d 占领:%d 中心:%d 目标:(%.1f,%.1f,%.1f)",
            msg.sentry_hp, msg.game_time, msg.if_in_hp, msg.if_in_center,
            msg.center_state, msg.aim_x, msg.aim_y, msg.aim_z);
    }

    rclcpp::Publisher<core_global::msg::SerialMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecFakePublisher>());
    rclcpp::shutdown();
    return 0;
}
