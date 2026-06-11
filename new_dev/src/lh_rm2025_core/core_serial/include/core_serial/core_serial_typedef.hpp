#ifndef CORE_SERIAL_TYPEDEF_H_
#define CORE_SERIAL_TYPEDEF_H_

#include <future>
#include <string>
#include <vector>
#include <iostream>

namespace cs
{
    // imu 协方差矩阵
    std::array<double, 9> covariance_li = {{-1, 0, 0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 9> covariance_or = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}};
    std::array<double, 9> covariance_an = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}};

    struct WatchDogSend
    {
        bool serial;
        bool nav;
        bool vision;
        bool decision;
    } __attribute__((packed));

    struct NavSend
    {
        uint8_t header = 0x02;
        float x;
        float y;
    } __attribute__((packed));

    struct DecSend
    {
        uint8_t header = 0x03;
        float spin_speed;
        uint8_t if_spin : 1;
        uint8_t if_super_cat : 1;

    } __attribute__((packed));

    // struct SerialReceive
    // {
    //     // 底盘
    //     float acc[3];
    //     float gyo[3];
    //     float eular[3];
    //     // 裁判系统
    //     int mode;
    //     int game_state;
    //     int plan;
    //     uint16_t game_time;
    //     uint16_t sentry_hp;
    //     uint16_t our_outpost_hp;
    //     uint16_t enermy_outpost_hp;
    //     uint16_t our_base_hp;
    //     uint16_t enermy_base_hp;
    //     uint32_t rfid_state;
    //     float defence_buff;
    //     int can_shoot;
    // } serial_receive;
    struct ReceivePacket
    {
        uint8_t header;
        // imu
        float eular[3];
        // 裁判系统
        uint16_t game_time;
        uint16_t sentry_hp;
        uint8_t if_in_hp;
        uint8_t if_in_center;
        uint8_t center_state; // 中心增益点情况
        // vision
        uint8_t detect_color : 1; // 0-red 1-blue
        bool reset_tracker : 1;
        uint8_t is_play : 3;
        bool change_target : 1;
        uint8_t reserved : 2;
        float aim_x;
        float aim_y;
        float aim_z;
        uint32_t timestamp; // (ms) board time
        uint16_t checksum;  // crc
    } __attribute__((packed));

    struct VisionSendPacket
    {
        uint8_t header = 0x01;
        uint8_t state : 2;      // 0-untracking 1-tracking-aim 2-tracking-buff
        uint8_t id : 3;         // aim: 0-outpost 6-guard 7-base
        uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
        float x;                // aim: robot-center || buff: rune-center
        float y;                // aim: robot-center || buff: rune-center
        float z;                // aim: robot-center || buff: rune-center
        float yaw;              // aim: robot-yaw || buff: rune-theta
        // spd = a*sin(w*t)+b || spd > 0 ==> clockwise
        float vx; // aim: robot-vx || buff: rune spin speed param - a
        float vy; // aim: robot-vy || buff: rune spin speed param - b
        float vz; // aim: robot-vz || buff: rune spin speed param - w
        float v_yaw;
        float r1;
        float r2;
        float dz;
        uint16_t t_offset; // (ms) speed t offset
        uint16_t checksum = 0;
    } __attribute__((packed));

    inline ReceivePacket fromVector(const std::vector<uint8_t> &data)
    {
        ReceivePacket packet;
        std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
        return packet;
    }

    template <typename T>
    inline std::vector<uint8_t> toVector(const T &data)
    {
        std::vector<uint8_t> packet(sizeof(T));
        std::copy(
            reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(T), packet.begin());
        return packet;
    }
}
#endif