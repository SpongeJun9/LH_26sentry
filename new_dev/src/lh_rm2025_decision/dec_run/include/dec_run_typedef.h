#ifndef DEC_STRUCTS_H
#define DEC_STRUCTS_H
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
namespace dr
{
    // 决策条件结构体
    typedef struct PolicyCondition
    {
        std::vector<int> wayPointID;
        int maxHP;    // 哨兵最大血量
        int if_fight; // 是否需要战斗
        int if_enermy_occupy;
        int if_sentry_occupy;
        int if_in_hp;
    } PolicyCondition;
    typedef struct Policy
    {
        // 信息
        int id;
        std::string name;
        int weight;                       // 权重
        int decide_mode;                  // 决策模式
        int decide_vision_region;         // 决策云台巡视区域
        int interrupted;                  // 是否可打断
        int cruise_time;                  // 巡逻间隔时间
        std::vector<int> decide_wayPoint; // 决策巡航位置
        // 条件
        PolicyCondition condition;
    } Policy;
    typedef struct WayPoint
    {
        int id;
        std::string name;
        double x;
        double y;
    } WayPoint;
    /*
    模式结构体
    */
    typedef struct Mode
    {
        int id;
        std::string name;
        float spin_speed;
        bool if_spin;
        bool if_super_cat;
    } Mode;
    /*
    决策信息
    */
    typedef struct PolicyInfo
    {
        std::shared_ptr<dr::Policy> now_policy = nullptr; // 当前决策
        std::string json_dir;
        std::string policy_path;
        std::string way_point_path;
        std::string mode_path;
    } PolicyInfo;
    /*
    导航相关反馈信息
    */
    typedef struct NavInfo
    {
        nav2_msgs::action::NavigateThroughPoses_FeedbackMessage::SharedPtr current_NTP_FeedBack_msg = nullptr;
        int8_t goal_status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        int8_t last_goal_status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
        int now_way_point_id = 0;
        rclcpp::TimerBase::SharedPtr timer;
        // 回血点被挡时的重试计数，用于超限后转移到安全点
        int hp_block_retry_count = 0;
    } NavInfo;
    /*
    裁判系统反馈信息
    */
    typedef struct GameInfo
    {
        int sentry_hp; // 哨兵最大血量
        int game_time;
        int if_in_hp;
        int if_enermy_occupy;
        int if_sentry_occupy;
    } GameInfoRMUL;
    /*
    视觉反馈信息
    */
    typedef struct VisionInfo
    {
        bool is_enermy = false;
    } VisionInfo;
    /*
    节点状态机
    */
    typedef enum State
    {
        WAITING = -3,
        DIE,
        ERROR,
        STAY,
        NAV,
        CRUISE
    } State;
    /*
视觉哨卫区域状态机
*/
    typedef enum VisionRegion
    {
        FIRST = 0,
        SECOND,
        THIRD,
        FORTH,
        FORWARD_SEMICIRCLE,
        BACKWARD_SEMICIRCLE,
        LEFT_SEMICIRCLE,
        RIGHT_SEMICIRCLE,
        LACK_FIRST,
        LACK_SECOND,
        LACK_THIRD,
        LACK_FORTH,
        CIRCLE
    } VisionRegion;
}
#endif