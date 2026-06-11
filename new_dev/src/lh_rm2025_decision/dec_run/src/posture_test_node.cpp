#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "core_global/msg/serial_msg.hpp"
#include "core_global/msg/decision_msg.hpp"
#include "core_global/msg/target.hpp"
#include "core_global/msg/armors.hpp"

class PostureTestNode : public rclcpp::Node
{
public:
    PostureTestNode()
        : Node("posture_test_node")
    {
        hp_threshold_ = this->declare_parameter<int>("hp_threshold", 250);

        pub_decision_ = this->create_publisher<core_global::msg::DecisionMsg>("/decision_msg", 10);
        sub_serial_ = this->create_subscription<core_global::msg::SerialMsg>(
            "/serial_msg",
            10,
            std::bind(&PostureTestNode::serialCallback, this, std::placeholders::_1));
        auto qos = rclcpp::QoS(10).best_effort();
        sub_target_ = this->create_subscription<core_global::msg::Target>(
            "/tracker/target",
            qos,
            std::bind(&PostureTestNode::targetCallback, this, std::placeholders::_1));
        sub_armors_ = this->create_subscription<core_global::msg::Armors>(
            "/detector/armors",
            qos,
            std::bind(&PostureTestNode::armorsCallback, this, std::placeholders::_1));

        current_posture_ = Posture::Move;
        RCLCPP_INFO(this->get_logger(), "姿态切换节点已启动，默认为移动姿态，当前血量=%u", latest_hp_);
        publishDecision();
    }

private:
    enum class Posture
    {
        Attack,
        Move,
        Defense
    };

    void serialCallback(const core_global::msg::SerialMsg::SharedPtr serial_msg)
    {
        if (serial_msg->sentry_hp > 600)
        {
            return;
        }
        latest_hp_ = serial_msg->sentry_hp;
        has_serial_ = true;
        const Posture target = decidePosture();
        if (target != current_posture_)
        {
            current_posture_ = target;
            RCLCPP_INFO(
                this->get_logger(),
                "触发条件[%s] -> 切换姿态[%s]，血量=%u，阈值=%d，看到敌方=%s",
                getTriggerReason(target),
                getPostureName(current_posture_),
                serial_msg->sentry_hp,
                hp_threshold_,
                has_enemy_ ? "是" : "否");
        }
        publishDecision();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "状态 心跳: 血量=%u, 敌方可见=%s, 姿态=%s, flag=%d",
            latest_hp_,
            has_enemy_ ? "是" : "否",
            getPostureName(current_posture_),
            getPostureFlag(current_posture_));
    }

    void targetCallback(const core_global::msg::Target::SharedPtr target_msg)
    {
        has_enemy_ = target_msg->tracking;
        if (!has_serial_)
        {
            return;
        }
        const Posture target = decidePosture();
        if (target != current_posture_)
        {
            current_posture_ = target;
            RCLCPP_INFO(
                this->get_logger(),
                "触发条件[%s] -> 切换姿态[%s]，血量=%u，阈值=%d，看到敌方=%s",
                getTriggerReason(target),
                getPostureName(current_posture_),
                latest_hp_,
                hp_threshold_,
                has_enemy_ ? "是" : "否");
            publishDecision();
        }
    }

    void armorsCallback(const core_global::msg::Armors::SharedPtr armors_msg)
    {
        has_enemy_ = !armors_msg->armors.empty();
        if (!has_serial_)
        {
            return;
        }
        const Posture target = decidePosture();
        if (target != current_posture_)
        {
            current_posture_ = target;
            RCLCPP_INFO(
                this->get_logger(),
                "触发条件[%s] -> 切换姿态[%s]，血量=%u，阈值=%d，看到敌方=%s",
                getTriggerReason(target),
                getPostureName(current_posture_),
                latest_hp_,
                hp_threshold_,
                has_enemy_ ? "是" : "否");
            publishDecision();
        }
    }

    void publishDecision()
    {
        core_global::msg::DecisionMsg msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "posture_test";
        msg.id = getPostureFlag(current_posture_);
        msg.mode = 0;
        msg.if_spin = false;
        msg.spin_speed = 0.0f;
        msg.if_super_cat = false;
        pub_decision_->publish(msg);
    }

    Posture decidePosture() const
    {
        if (latest_hp_ < hp_threshold_)
        {
            return Posture::Defense;
        }
        if (has_enemy_)
        {
            return Posture::Attack;
        }
        return Posture::Move;
    }

    const char *getPostureName(Posture posture) const
    {
        if (posture == Posture::Attack)
        {
            return "进攻";
        }
        if (posture == Posture::Defense)
        {
            return "防御";
        }
        return "移动";
    }

    int getPostureFlag(Posture posture) const
    {
        if (posture == Posture::Attack)
        {
            return 0;
        }
        if (posture == Posture::Defense)
        {
            return 1;
        }
        return 2;
    }

    const char *getTriggerReason(Posture target) const
    {
        if (target == Posture::Defense)
        {
            return "血量低于阈值";
        }
        if (target == Posture::Attack)
        {
            return "检测到敌方目标";
        }
        return "未检测到敌方且血量正常";
    }

    int hp_threshold_;
    bool has_enemy_ = false;
    bool has_serial_ = false;
    uint16_t latest_hp_ = 0;
    Posture current_posture_;
    rclcpp::Subscription<core_global::msg::SerialMsg>::SharedPtr sub_serial_;
    rclcpp::Subscription<core_global::msg::Target>::SharedPtr sub_target_;
    rclcpp::Subscription<core_global::msg::Armors>::SharedPtr sub_armors_;
    rclcpp::Publisher<core_global::msg::DecisionMsg>::SharedPtr pub_decision_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PostureTestNode>());
    rclcpp::shutdown();
    return 0;
}
