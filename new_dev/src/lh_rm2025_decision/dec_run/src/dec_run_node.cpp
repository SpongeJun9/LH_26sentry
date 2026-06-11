#include "dec_run_node.h"
#include "rclcpp/rclcpp.hpp"

namespace dr
{
    // -----function-----
    void DecRunNode::init()
    {
        RCLCPP_INFO(this->get_logger(), "dec_run_node start init!");
        this->getListParams();
        int init_ret = this->list_->init();
        if (init_ret < 0)
        {
            this->list_->init_waypoints_only();
        }
        RCLCPP_INFO(this->get_logger(), "way_point数量:%zu, policy数量:%zu, mode数量:%zu",
                    this->list_->way_point_arr_.size(), this->list_->policy_arr_.size(), this->list_->mode_arr_.size());
        this->sub_nav_feedback_ = this->create_subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>(
            "/navigate_through_poses/_action/feedback",
            10,
            std::bind(&DecRunNode::nav2FeedBackCallBack, this, _1));
        this->sub_nav_goal_status_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_through_poses/_action/status",
            10,
            std::bind(&DecRunNode::nav2GoalStatusCallBack, this, _1));
        // pub初始化
        this->pub_dec_ = this->create_publisher<core_global::msg::DecisionMsg>("/decision_msg", 10);
        // client
        this->client_nav_through_poses_action_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");

        if (!this->client_nav_through_poses_action_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->state_ = State::ERROR;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "\n" << "已连接至nav动作服务器");
            // this->client_nav_through_poses_action_->async_cancel_all_goals();
            RCLCPP_INFO(this->get_logger(), "dec_run_node init ok!");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        this->sub_serial_ = this->create_subscription<core_global::msg::SerialMsg>("/serial_msg", 10, std::bind(&DecRunNode::serialCallback, this, std::placeholders::_1));
    }
    // 串口回调函数
    void DecRunNode::serialCallback(const core_global::msg::SerialMsg::SharedPtr serial_msg)
    {
        this->processSerialMsg(serial_msg);
        this->checkNavState();
        if (this->state_ == State::ERROR)
        {
            RCLCPP_WARN(this->get_logger(), "state Error!");
            return;
        }
        this->checkGameState();
        if (this->state_ < 0)
        {
            RCLCPP_WARN(this->get_logger(), "state < 0 !");
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "准备执行 decisionOnce, 当前状态=%d", this->state_);
        if (this->decisionOnce())
        {
            if (this->state_ == State::STAY)
            {
                // this->client_nav_through_poses_action_->async_cancel_all_goals();
                if (this->nav_info_.timer)
                {
                    this->nav_info_.timer->cancel();
                    this->nav_info_.timer.reset();
                }
            }
            else if (this->state_ == State::CRUISE)
            {
                this->makeNavGoal();
                // 巡航模式下定义一个定时器，定时触发发布导航点函数
                this->nav_info_.timer = this->create_wall_timer(
                    std::chrono::milliseconds(this->policy_info_.now_policy->cruise_time * 1000), std::bind(&DecRunNode::sendNavGoal, this));
            }
            else if (this->state_ == State::NAV)
            {
                if (this->nav_info_.timer)
                {
                    this->nav_info_.timer->cancel();
                    this->nav_info_.timer.reset();
                }
                this->acummulated_poses_.clear();
                // 开始导航时重置位置信息，避免使用旧位置进行策略判断
                this->nav_info_.now_way_point_id = -1;
                this->makeNavGoal();
                this->sendNavGoal();
            }
        }
        this->pubMsg();
    }
    bool DecRunNode::decisionOnce()
    {
        auto temp = choose_policy();
        if (temp == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "未找到合适的策略");
            return false;
        }

        if (this->policy_info_.now_policy != nullptr && temp->id == this->policy_info_.now_policy->id)
        {
            RCLCPP_DEBUG(this->get_logger(), "策略未变化: %s", temp->name.c_str());
            return false; // 若决策结果与现决策相同返回
        }
        // if (this->policy_info_.now_policy != nullptr && this->policy_info_.now_policy->interrupted == 0 && this->state_ != State::STAY && temp->weight < this->policy_info_.now_policy->weight)
        // {
        //     RCLCPP_WARN_STREAM(this->get_logger(), "高权重决策[" << this->policy_info_.now_policy->name << "]运行中无法被打断");
        //     return false; // 若当前决策不可打断且正在运行返回
        // }
        this->policy_info_.now_policy = temp;
        RCLCPP_INFO_STREAM(this->get_logger(), "决策[" << this->policy_info_.now_policy->name << "]已更新");
        // this->client_nav_through_poses_action_->async_cancel_all_goals();
        //  更新决策取消当前目标
        if (this->policy_info_.now_policy->decide_wayPoint[0] < 0)
        {
            this->state_ = State::STAY;
        }
        else if (this->policy_info_.now_policy->cruise_time > 0)
        {
            this->state_ = State::CRUISE;
        }
        else
        {
            this->state_ = State::NAV;
        }

        // 打印状态并添加解释
        std::string state_name;
        switch(this->state_)
        {
            case State::WAITING: state_name = "等待中"; break;
            case State::DIE: state_name = "死亡"; break;
            case State::ERROR: state_name = "错误"; break;
            case State::STAY: state_name = "停留/驻守"; break;
            case State::NAV: state_name = "导航中"; break;
            case State::CRUISE: state_name = "巡航中"; break;
            default: state_name = "未知"; break;
        }
        RCLCPP_INFO(this->get_logger(), "状态已更新: [%d] %s", this->state_, state_name.c_str());
        return true;
    }
    std::shared_ptr<dr::Policy> DecRunNode::choose_policy()
    {
        std::shared_ptr<dr::Policy> temp = nullptr;
        RCLCPP_DEBUG(this->get_logger(), "开始选择策略: now_way_point_id=%d, if_fight=%d, sentry_hp=%d, if_enermy_occupy=%d, if_sentry_occupy=%d, if_in_hp=%d",
                    this->nav_info_.now_way_point_id, this->vision_info_.is_enermy, this->game_info_.sentry_hp,
                    this->game_info_.if_enermy_occupy, this->game_info_.if_sentry_occupy, this->game_info_.if_in_hp);
        for (auto policy : this->list_->policy_arr_)
        {
            PolicyCondition condition = policy.condition;
            // 根据条件筛选
            // -----nav condition-----
            auto it = std::find(condition.wayPointID.begin(), condition.wayPointID.end(), this->nav_info_.now_way_point_id);
            if (it == condition.wayPointID.end() && condition.wayPointID[0] != -1)
                continue; // 若当前所处路径点不符合条件
            // -----vision condition-----
            if (condition.if_fight != -1 && condition.if_fight != this->vision_info_.is_enermy)
                continue;
            // -----game condition-----
            if (condition.maxHP != -1 && condition.maxHP < this->game_info_.sentry_hp)
                continue;
            if (condition.if_enermy_occupy != -1 && condition.if_enermy_occupy != this->game_info_.if_enermy_occupy)
                continue;
            if (condition.if_sentry_occupy != -1 && condition.if_sentry_occupy != this->game_info_.if_sentry_occupy)
                continue;
            if (condition.if_in_hp != -1 && condition.if_in_hp != this->game_info_.if_in_hp)
                continue;
            RCLCPP_DEBUG(this->get_logger(), "策略[%s](id=%d, weight=%d)满足条件", policy.name.c_str(), policy.id, policy.weight);
            if (temp == nullptr || policy.weight > temp->weight)
                temp = std::make_shared<dr::Policy>(policy);
        }
        if (temp != nullptr)
        {
            RCLCPP_DEBUG(this->get_logger(), "最终选择策略[%s](id=%d, weight=%d)", temp->name.c_str(), temp->id, temp->weight);
        }
        return temp;
    }
    void DecRunNode::makeNavGoal()
    {
        auto pose = geometry_msgs::msg::PoseStamped();
        this->acummulated_poses_.clear();
        for (auto way_point_id : this->policy_info_.now_policy->decide_wayPoint)
        {

            auto temp = this->list_->way_point_arr_[way_point_id];
            pose.header.stamp = this->get_clock()->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = temp.x;
            pose.pose.position.y = temp.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.0;
            this->acummulated_poses_.push_back(pose);
            RCLCPP_INFO_STREAM(this->get_logger(), "[" << way_point_id << "]路径点已添加");
        }
    }
    void DecRunNode::sendNavGoal()
    {
        if (this->state_ == State::CRUISE)
        {
            // 巡航，仅访问缓冲区头元素
            this->nav_through_poses_goal_.poses.push_back(this->acummulated_poses_.front());
        }
        else if (this->state_ == State::NAV)
        {
            this->nav_through_poses_goal_.poses = this->acummulated_poses_;
        }
        if (this->client_nav_through_poses_action_->wait_for_action_server(std::chrono::microseconds(100)))
        {
            if (!this->acummulated_poses_.empty())
            {
                auto future_goal_handle = client_nav_through_poses_action_->async_send_goal(this->nav_through_poses_goal_, rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions());
                RCLCPP_INFO_STREAM(this->get_logger(), "nav目标已发布");
                if (this->state_ == State::CRUISE)
                {
                    // 将巡航头元素置于队尾
                    this->acummulated_poses_.push_back(this->acummulated_poses_.front());
                    this->acummulated_poses_.erase(this->acummulated_poses_.begin());
                }
                else if (this->state_ == State::NAV)
                {
                    this->acummulated_poses_.clear();
                }
            }
            else
            {
                this->acummulated_poses_.clear();
                RCLCPP_WARN_STREAM(this->get_logger(), "\n" << "nav目标缓冲区空");
            }
        }
        else
        {
            this->acummulated_poses_.clear();
            RCLCPP_WARN_STREAM(this->get_logger(), "\n" << "未连接至nav动作服务器");
        }
    }
    void DecRunNode::processSerialMsg(const core_global::msg::SerialMsg::SharedPtr serial_msg)
    {
        // 数据合理性检查
        bool data_valid = (serial_msg->sentry_hp >= 0 && serial_msg->sentry_hp <= 600) &&
                         (serial_msg->game_time >= 0 && serial_msg->game_time <= 420) &&
                         (serial_msg->if_in_hp >= 0 && serial_msg->if_in_hp <= 1) &&
                         (serial_msg->if_in_center >= 0 && serial_msg->if_in_center <= 1) &&
                         (serial_msg->center_state >= 0 && serial_msg->center_state <= 3);

        if (!data_valid)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "收到异常数据: 血量=%d, 时间=%d, 回血区=%d, 中心=%d, 状态=%d (已忽略)",
                                serial_msg->sentry_hp, serial_msg->game_time,
                                serial_msg->if_in_hp, serial_msg->if_in_center, serial_msg->center_state);
            return; // 忽略异常数据
        }

        // 保存旧值用于比较
        bool changed = false;
        std::string changes;

        if (this->game_info_.sentry_hp != serial_msg->sentry_hp)
        {
            changes += "血量: " + std::to_string(this->game_info_.sentry_hp) + " -> " + std::to_string(serial_msg->sentry_hp) + " ";
            this->game_info_.sentry_hp = serial_msg->sentry_hp;
            changed = true;
        }
        else
        {
            this->game_info_.sentry_hp = serial_msg->sentry_hp;
        }

        if (this->game_info_.game_time != serial_msg->game_time)
        {
            this->game_info_.game_time = serial_msg->game_time;
            // 游戏时间变化不打印，太频繁
        }
        else
        {
            this->game_info_.game_time = serial_msg->game_time;
        }

        if (this->game_info_.if_in_hp != serial_msg->if_in_hp)
        {
            changes += "回血区: " + std::to_string(this->game_info_.if_in_hp) + " -> " + std::to_string(serial_msg->if_in_hp) + " ";
            this->game_info_.if_in_hp = serial_msg->if_in_hp;
            changed = true;
        }
        else
        {
            this->game_info_.if_in_hp = serial_msg->if_in_hp;
        }

        if (this->game_info_.if_sentry_occupy != serial_msg->if_in_center)
        {
            changes += "中心占领: " + std::to_string(this->game_info_.if_sentry_occupy) + " -> " + std::to_string(serial_msg->if_in_center) + " ";
            this->game_info_.if_sentry_occupy = serial_msg->if_in_center;
            changed = true;
        }
        else
        {
            this->game_info_.if_sentry_occupy = serial_msg->if_in_center;
        }

        // 只在数据变化时打印
        if (changed)
        {
            RCLCPP_INFO(this->get_logger(), "裁判系统数据变化: %s", changes.c_str());
        }

        // 实时打印裁判系统数据（降低频率）
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "裁判系统数据 | 血量:%d | 比赛时间:%d | 在回血区:%d | 占领中心:%d | 中心状态:%d",
                    this->game_info_.sentry_hp,
                    this->game_info_.game_time,
                    this->game_info_.if_in_hp,
                    this->game_info_.if_sentry_occupy,
                    serial_msg->center_state);
        if (serial_msg->center_state == 2)
        {
            this->game_info_.if_enermy_occupy = 1;
        }
        else
        {
            this->game_info_.if_enermy_occupy = 0;
        }
        if (serial_msg->aim_x == 0. && serial_msg->aim_y == 0. && serial_msg->aim_z == 0.)
        {
            this->vision_info_.is_enermy = 0;
        }
        else
        {
            this->vision_info_.is_enermy = 1;
        }
    }
    void DecRunNode::checkGameState()
    {
        if (this->game_info_.game_time <= 0)
        {
            RCLCPP_INFO(this->get_logger(), "waiting game start");
            this->state_ = State::WAITING;
        }
        else if (this->game_info_.sentry_hp <= 0)
        {
            RCLCPP_INFO(this->get_logger(), "sentry die");
            this->state_ = State::DIE;
        }
        else
        {
            // 只有在 WAITING 或 DIE 状态时才自动切换到 STAY
            // 不要覆盖 NAV 和 CRUISE 状态
            if (this->state_ < State::STAY)
            {
                // 从 DIE 状态恢复时，重置位置信息
                // 避免死前在某个点，复活后系统仍认为在该点，导致选择错误的stay策略
                if (this->state_ == State::DIE)
                {
                    this->nav_info_.now_way_point_id = -1;
                    RCLCPP_INFO(this->get_logger(), "复活后重置位置信息: now_way_point_id = -1");
                }
                this->state_ = State::STAY;
            }
        }
    }
    void DecRunNode::checkNavState()
    {
        if (!this->client_nav_through_poses_action_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->state_ = State::ERROR;
        }
        switch (this->nav_info_.goal_status)
        {
        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
            RCLCPP_DEBUG(this->get_logger(), "导航状态: UNKNOWN");
            break;
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
            RCLCPP_DEBUG(this->get_logger(), "导航状态: ACCEPTED");
            break;
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
            RCLCPP_DEBUG(this->get_logger(), "导航状态: EXECUTING");
            break;
        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
            // 只在状态变化时打印
            if (this->nav_info_.last_goal_status != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "导航成功，当前状态=%d, 策略=%s",
                            this->state_,
                            this->policy_info_.now_policy ? this->policy_info_.now_policy->name.c_str() : "null");
            }
            this->nav_info_.last_goal_status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;

            // 只要导航成功且有策略，就处理到达逻辑（不限制必须是 NAV 状态）
            if (this->policy_info_.now_policy != nullptr &&
                !this->policy_info_.now_policy->decide_wayPoint.empty())
            {
                auto goal = this->policy_info_.now_policy->decide_wayPoint.back();

                

                if (goal == 1)
                {
                    // 目标是中心点
                    if (this->game_info_.if_sentry_occupy == 0)
                    {
                        RCLCPP_WARN(this->get_logger(), "未占领中心，重发导航目标");
                        this->makeNavGoal();
                        this->sendNavGoal();
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "已占领中心，切换到 STAY 状态");
                        this->nav_info_.now_way_point_id = goal;
                        this->state_ = State::STAY;
                    }
                }
                else if (goal == 0)
                {
                    // 目标是回血点
                    if (this->game_info_.if_in_hp == 0)
                    {
                        RCLCPP_WARN(this->get_logger(), "未到回血点，重发导航目标");
                        this->makeNavGoal();
                        this->sendNavGoal();
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "已到回血点，切换到 STAY 状态");
                        this->nav_info_.now_way_point_id = goal;
                        this->state_ = State::STAY;
                        this->nav_info_.hp_block_retry_count = 0;
                    }
                }
                else
                {
                    // 其他路点
                    if (goal >= 0)
                    {
                        // 正常路点，直接认为到达
                        RCLCPP_INFO(this->get_logger(), "到达路点 %d，切换到 STAY 状态", goal);
                        this->nav_info_.now_way_point_id = goal;
                        this->state_ = State::STAY;
                    }
                    // goal == -1 表示不需要导航，不做任何处理
                }
            }
            break;
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
            if (this->state_ == State::NAV)
            {
                // RCLCPP_INFO_STREAM(this->get_logger(), 4);
                if (this->nav_info_.now_way_point_id != this->policy_info_.now_policy->decide_wayPoint.back())
                {
                    this->makeNavGoal();
                    this->sendNavGoal();
                    RCLCPP_WARN(this->get_logger(), "失败重发");
                }
                else
                {
                    this->state_ = State::STAY;
                }
            }

            break;
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
            // if (this->state_ > State::STAY)
            // {
            //     RCLCPP_INFO_STREAM(this->get_logger(), 6);
            //     this->nav_info_.now_way_point_id = this->nav_info_.now_goal_id;
            //     this->state_ = State::STAY;
            //     this->nav_info_.now_goal_id = -1;
            // }
            // RCLCPP_INFO_STREAM(this->get_logger(), 6);
            break;
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
            if (this->state_ > State::STAY)
            {
                // RCLCPP_INFO_STREAM(this->get_logger(), 7);
                this->state_ = State::STAY;
            }
            // RCLCPP_INFO_STREAM(this->get_logger(), 7);
            break;
        default:
            break;
        }
    }
    void DecRunNode::getListParams()
    {
        try
        {
            const auto json_dir_string = declare_parameter<std::string>("json_dir", "/home/lionheart/new_dev/src/lh_rm2025_decision/dec_run/json/");
            if (json_dir_string == "")
            {
                throw std::invalid_argument{"The json_dir parameter is empty, please provide a valid path"};
            }
            this->list_ = std::make_unique<dr::DecRunList>(json_dir_string);
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The parameters provided were invalid");
            throw ex;
        }
    }
    void DecRunNode::pubMsg()
    {
        auto mode = this->list_->mode_arr_[this->policy_info_.now_policy->decide_mode];
        core_global::msg::DecisionMsg decision_msg;
        decision_msg.header.frame_id = "decision";
        decision_msg.header.stamp = rclcpp::Clock().now();
        decision_msg.id = this->policy_info_.now_policy->id;
        decision_msg.mode = mode.id;
        decision_msg.if_spin = mode.if_spin;
        decision_msg.spin_speed = mode.spin_speed;
        decision_msg.if_super_cat = mode.if_super_cat;

        this->pub_dec_->publish(decision_msg);
    }
    // -----function_callback-----
    void DecRunNode::nav2FeedBackCallBack(const nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr nav2_fb_msg)
    {
        this->nav_info_.current_NTP_FeedBack_msg = nav2_fb_msg;
    }
    // nav2 goalstatus回调
    void DecRunNode::nav2GoalStatusCallBack(const action_msgs::msg::GoalStatusArray::SharedPtr nav2_goal_msg)
    {
        if (this->nav_info_.goal_status != nav2_goal_msg->status_list.back().status)
        {
            this->nav_info_.goal_status = nav2_goal_msg->status_list.back().status;
        }
    }
    DecRunNode::DecRunNode(const rclcpp::NodeOptions &options)
        : Node("dec_run_node", options)
    {
        RCLCPP_INFO(get_logger(), "Start dec_run_node!");
        this->init();
    }
    DecRunNode::~DecRunNode() = default;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(dr::DecRunNode);
