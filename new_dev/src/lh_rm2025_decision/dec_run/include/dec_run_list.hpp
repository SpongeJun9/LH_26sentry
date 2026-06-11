#ifndef DEC_RUN_LIST_HPP
#define DEC_RUN_LIST_HPP
#include "iostream"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "dec_run_typedef.h"

namespace dr
{
    class DecRunList
    {
    private:
        std::string json_dir_;
        std::string policy_path_;
        std::string way_point_path_;
        std::string mode_path_;
        std::string resolve_path(const std::string &base, const std::string &path) const
        {
            std::filesystem::path p(path);
            if (p.is_absolute())
            {
                return p.string();
            }
            std::filesystem::path b(base);
            return (b / p).string();
        }

    public:
        std::vector<dr::Policy> policy_arr_; // 策略集
        std::vector<dr::WayPoint> way_point_arr_;
        std::vector<dr::Mode> mode_arr_;
        int init()
        {
            RCLCPP_INFO(rclcpp::get_logger("json"), "json list start init!");
            Json::CharReaderBuilder builder;
            Json::Value policy_value;
            Json::Value way_point_value;
            Json::Value mode_value;
            std::string errs;
            std::string policy_full = resolve_path(json_dir_, policy_path_);
            std::string way_point_full = resolve_path(json_dir_, way_point_path_);
            std::string mode_full = resolve_path(json_dir_, mode_path_);
            std::ifstream policy_file(policy_full);
            std::ifstream way_point_file(way_point_full);
            std::ifstream mode_file(mode_full);
            if (!policy_file.is_open())
            {
                RCLCPP_ERROR(rclcpp::get_logger("json"), "policy json unable to open: %s", policy_full.c_str());
                return -1;
            }
            else if (!Json::parseFromStream(builder, policy_file, &policy_value, &errs))
            {
                RCLCPP_ERROR(rclcpp::get_logger("json"), "policy value parse error: %s", errs.c_str());
                return -2;
            }
            if (!way_point_file.is_open())
            {
                RCLCPP_ERROR(rclcpp::get_logger("json"), "way_point unable to open: %s", way_point_full.c_str());
                return -1;
            }
            else if (!Json::parseFromStream(builder, way_point_file, &way_point_value, &errs))
            {
                RCLCPP_ERROR(rclcpp::get_logger("json"), "way_point value parse error: %s", errs.c_str());
                return -2;
            }

            if (!mode_file.is_open())
            {
                RCLCPP_ERROR(rclcpp::get_logger("json"), "mode json unable to open: %s", mode_full.c_str());
                return -1;
            }
            else if (!Json::parseFromStream(builder, mode_file, &mode_value, &errs))
            {
                RCLCPP_ERROR(rclcpp::get_logger("json"), "mode value parse error: %s", errs.c_str());
                return -2;
            }
            for (unsigned int i = 0; i < way_point_value["data"].size(); i++)
            {
                dr::WayPoint g;
                Json::Value value = way_point_value["data"][i];
                g.id = value["id"].asInt();
                g.name = value["name"].asString();
                g.x = value["position"]["x"].asFloat();
                g.y = value["position"]["y"].asFloat();
                way_point_arr_.push_back(g);
            }
            for (unsigned int i = 0; i < mode_value["data"].size(); i++)
            {
                dr::Mode g;
                Json::Value value = mode_value["data"][i];
                g.id = value["id"].asInt();
                g.name = value["name"].asString();
                g.if_spin = value["if_spin"].asBool();
                g.if_super_cat = value["if_super_cat"].asBool();
                g.spin_speed = value["spin_speed"].asFloat();
                mode_arr_.push_back(g);
            }
            for (unsigned int i = 0; i < policy_value["stay"].size(); i++)
            {
                dr::Policy g;
                Json::Value value = policy_value["stay"];
                g.id = value[i]["id"].asInt();
                g.name = value[i]["name"].asString();
                g.weight = value[i]["weight"].asInt();
                g.cruise_time = value[i]["cruise_time"].asInt();
                g.interrupted = value[i]["interrupted"].asInt();
                g.condition.if_in_hp = value[i]["if_in_hp"].asInt();
                g.condition.if_enermy_occupy = value[i]["if_enermy_occupy"].asInt();
                g.condition.if_sentry_occupy = value[i]["if_sentry_occupy"].asInt();
                g.condition.maxHP = value[i]["maxHP"].asInt();
                g.condition.if_fight = value[i]["if_fight"].asInt();
                g.decide_mode = value[i]["decide_mode"].asInt();
                for (unsigned int j = 0; j < value[i]["wayPointID"].size(); j++)
                {
                    g.condition.wayPointID.push_back(value[i]["wayPointID"][j].asInt());
                }
                for (unsigned int j = 0; j < value[i]["decide_wayPoint"].size(); j++)
                {
                    g.decide_wayPoint.push_back(value[i]["decide_wayPoint"][j].asInt());
                }
                policy_arr_.push_back(g);
            }
            for (unsigned int i = 0; i < policy_value["nav"].size(); i++)
            {
                dr::Policy g;
                Json::Value value = policy_value["nav"];
                g.id = value[i]["id"].asInt();
                g.name = value[i]["name"].asString();
                g.weight = value[i]["weight"].asInt();
                g.cruise_time = value[i]["cruise_time"].asInt();
                g.interrupted = value[i]["interrupted"].asInt();
                g.condition.if_in_hp = value[i]["if_in_hp"].asInt();
                g.condition.if_enermy_occupy = value[i]["if_enermy_occupy"].asInt();
                g.condition.if_sentry_occupy = value[i]["if_sentry_occupy"].asInt();
                g.condition.maxHP = value[i]["maxHP"].asInt();
                g.condition.if_fight = value[i]["if_fight"].asInt();
                g.decide_mode = value[i]["decide_mode"].asInt();
                for (unsigned int j = 0; j < value[i]["wayPointID"].size(); j++)
                {
                    g.condition.wayPointID.push_back(value[i]["wayPointID"][j].asInt());
                }
                for (unsigned int j = 0; j < value[i]["decide_wayPoint"].size(); j++)
                {
                    g.decide_wayPoint.push_back(value[i]["decide_wayPoint"][j].asInt());
                }
                policy_arr_.push_back(g);
            }
            for (unsigned int i = 0; i < policy_value["patrol"].size(); i++)
            {
                dr::Policy g;
                Json::Value value = policy_value["patrol"];
                g.id = value[i]["id"].asInt();
                g.name = value[i]["name"].asString();
                g.weight = value[i]["weight"].asInt();
                g.cruise_time = value[i]["cruise_time"].asInt();
                g.interrupted = value[i]["interrupted"].asInt();
                g.condition.if_in_hp = value[i]["if_in_hp"].asInt();
                g.condition.if_enermy_occupy = value[i]["if_enermy_occupy"].asInt();
                g.condition.if_sentry_occupy = value[i]["if_sentry_occupy"].asInt();
                g.condition.maxHP = value[i]["maxHP"].asInt();
                g.condition.if_fight = value[i]["if_fight"].asInt();
                g.decide_mode = value[i]["decide_mode"].asInt();
                for (unsigned int j = 0; j < value[i]["wayPointID"].size(); j++)
                {
                    g.condition.wayPointID.push_back(value[i]["wayPointID"][j].asInt());
                }
                for (unsigned int j = 0; j < value[i]["decide_wayPoint"].size(); j++)
                {
                    g.decide_wayPoint.push_back(value[i]["decide_wayPoint"][j].asInt());
                }
                policy_arr_.push_back(g);
            }
            RCLCPP_INFO(rclcpp::get_logger("json"), "json list init ok!");
            return 1;
        }
        int init_waypoints_only()
        {
            RCLCPP_INFO(rclcpp::get_logger("json"), "json list start init (waypoints only)!");
            Json::CharReaderBuilder builder;
            Json::Value way_point_value;
            std::string errs;
            std::string way_point_full = resolve_path(json_dir_, way_point_path_);
            std::ifstream way_point_file(way_point_full);

            if (!way_point_file.is_open())
            {
                RCLCPP_ERROR(rclcpp::get_logger("json"), "way_point unable to open: %s", way_point_full.c_str());
                return -1;
            }
            else if (!Json::parseFromStream(builder, way_point_file, &way_point_value, &errs))
            {
                RCLCPP_ERROR(rclcpp::get_logger("json"), "way_point value parse error: %s", errs.c_str());
                return -2;
            }

            way_point_arr_.clear();
            for (unsigned int i = 0; i < way_point_value["data"].size(); i++)
            {
                dr::WayPoint g;
                Json::Value value = way_point_value["data"][i];
                g.id = value["id"].asInt();
                g.name = value["name"].asString();
                g.x = value["position"]["x"].asFloat();
                g.y = value["position"]["y"].asFloat();
                way_point_arr_.push_back(g);
            }
            RCLCPP_INFO(rclcpp::get_logger("json"), "json list init (waypoints only) ok!");
            return 1;
        }

        DecRunList(std::string json_dir, std::string policy_path, std::string way_point_path, std::string mode_path)
        {
            this->json_dir_ = json_dir;
            this->policy_path_ = policy_path;
            this->way_point_path_ = way_point_path;
            this->mode_path_ = mode_path;
        }
        DecRunList(std::string json_dir)
        {
            this->json_dir_ = json_dir;
            this->policy_path_ = "policy/policy_test.json";
            this->way_point_path_ = "wayPoint/way_point_rmul.json";
            this->mode_path_ = "mode/mode_test.json";
        }
        ~DecRunList() = default;
    };
}
#endif
