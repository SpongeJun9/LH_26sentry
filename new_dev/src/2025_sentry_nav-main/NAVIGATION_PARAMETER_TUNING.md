# 导航系统参数调整指南（精简版）

本文档列出实际部署时必须根据机器人实际情况调整的关键参数。

## 1. 机器人尺寸参数（必须调整）

### 1.1 机器人半径

**位置**: `pb2025_nav_bringup/config/reality/nav2_params.yaml`

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.2  # ⚠️ 必须根据实际机器人尺寸调整

global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.2  # ⚠️ 必须与 local_costmap 保持一致
```

**调整方法**: 测量机器人底盘最大半径（从中心到最外边缘），建议值 = 实际半径 + 0.05~0.1m 安全余量

### 1.2 车辆高度

**位置**: `pb2025_nav_bringup/config/reality/nav2_params.yaml`

```yaml
terrain_analysis:
  ros__parameters:
    vehicleHeight: 0.5  # ⚠️ 根据实际车高调整（米）

terrain_analysis_ext:
  ros__parameters:
    vehicleHeight: 1.0  # ⚠️ 可以比实际车高稍大
```

**调整方法**: 测量机器人底盘到最高点的垂直距离

---

## 2. 传感器配置（必须调整）

### 2.1 LiDAR 外参（安装位置和姿态）

**位置**: `pb2025_nav_bringup/config/reality/mid360_user_config.json`

```json
{
  "lidar_configs": [
    {
      "ip": "192.168.1.189",  // ⚠️ 修改为实际 LiDAR IP
      "extrinsic_parameter": {
        "roll": 0.0,    // ⚠️ LiDAR 相对于机器人基座的姿态（弧度）
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,         // ⚠️ LiDAR 相对于机器人基座的位置（米）
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

**调整方法**: 
- 测量 LiDAR 中心相对于机器人基座（`gimbal_yaw`）的位置和姿态
- 如果 LiDAR 倾斜安装，需要正确设置 `pitch` 或 `roll` 角度

### 2.2 LiDAR 网络配置

**位置**: `pb2025_nav_bringup/config/reality/mid360_user_config.json`

```json
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip": "192.168.1.50",      // ⚠️ 修改为主机 IP
      "push_msg_ip": "192.168.1.50",
      "point_data_ip": "192.168.1.50",
      "imu_data_ip": "192.168.1.50"
    }
  }
}
```

**调整方法**: 根据实际网络配置修改主机 IP 地址，确保与 LiDAR 在同一网段

### 2.3 Point-LIO 外参（如果使用标定结果）

**位置**: `pb2025_nav_bringup/config/reality/nav2_params.yaml`

```yaml
point_lio:
  ros__parameters:
    mapping:
      extrinsic_T: [ -0.011, -0.02329, 0.04412 ]  # ⚠️ LiDAR 到 IMU 的平移
      extrinsic_R: [ 1.0, 0.0, 0.0,                # ⚠️ LiDAR 到 IMU 的旋转矩阵
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0 ]
      gravity: [0.0, 0.0, 9.8383715748786926]       # ⚠️ 重力向量（根据 IMU 安装方向）
```

**调整方法**: 如果进行了 LiDAR-IMU 标定（如使用 LI-Init），将标定结果填入

---

## 3. 速度限制（建议调整）

**位置**: `pb2025_nav_bringup/config/reality/nav2_params.yaml`

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      v_linear_min: -2.5   # ⚠️ 根据机器人实际最大速度调整
      v_linear_max: 2.5
      v_angular_min: -3.0
      v_angular_max: 3.0

velocity_smoother:
  ros__parameters:
    max_velocity: [2.5, 2.5, 3.0]      # ⚠️ 与 FollowPath 保持一致
    min_velocity: [-2.5, -2.5, -3.0]
    max_accel: [4.5, 4.5, 5.0]        # ⚠️ 根据机器人加速度能力调整
    max_decel: [-4.5, -4.5, -5.0]
```

**调整方法**: 
- 根据机器人实际最大速度设置，建议为实际最大速度的 80-90%
- 如果速度变化过快导致抖动，降低 `max_accel` 和 `max_decel`

---

## 4. 坐标系名称（如果与默认不同）

**位置**: `pb2025_nav_bringup/config/reality/nav2_params.yaml`

如果机器人系统的坐标系名称与默认值不同，需要修改以下配置：

```yaml
# 机器人基座坐标系（默认: gimbal_yaw）
sensor_scan_generation:
  ros__parameters:
    robot_base_frame: "gimbal_yaw"  # ⚠️ 如果不同则修改

fake_vel_transform:
  ros__parameters:
    robot_base_frame: "gimbal_yaw"  # ⚠️ 如果不同则修改

# LiDAR 坐标系（默认: front_mid360）
loam_interface:
  ros__parameters:
    lidar_frame: "front_mid360"  # ⚠️ 如果不同则修改

sensor_scan_generation:
  ros__parameters:
    lidar_frame: "front_mid360"  # ⚠️ 如果不同则修改
```

**调整方法**: 检查实际 TF 树中的坐标系名称，如果不同则统一修改

---

## 5. 其他重要参数（可选调整）

### 5.1 膨胀半径

**位置**: `pb2025_nav_bringup/config/reality/nav2_params.yaml`

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.7  # 建议: robot_radius + 0.2~0.5
```

**调整方法**: 如果机器人经常撞到障碍物，适当增大此值

### 5.2 障碍物检测范围

**位置**: `pb2025_nav_bringup/config/reality/nav2_params.yaml`

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      intensity_voxel_layer:
        terrain_map:
          obstacle_max_range: 5.0  # 根据 LiDAR 有效检测范围调整
```

---

## 快速检查清单

部署前必须检查的参数：

- [ ] **机器人尺寸**
  - [ ] `robot_radius` (local_costmap, global_costmap)
  - [ ] `vehicleHeight` (terrain_analysis, terrain_analysis_ext)

- [ ] **传感器配置**
  - [ ] LiDAR IP 地址 (`mid360_user_config.json`)
  - [ ] 主机 IP 地址 (`mid360_user_config.json`)
  - [ ] LiDAR 外参 (`extrinsic_parameter`)

- [ ] **速度限制**
  - [ ] `v_linear_max`, `v_angular_max` (controller_server)
  - [ ] `max_velocity`, `max_accel` (velocity_smoother)

- [ ] **坐标系名称**（如果与默认不同）
  - [ ] `robot_base_frame`
  - [ ] `lidar_frame`

---

## 常见问题

**Q: 机器人经常撞到障碍物**
- 检查 `robot_radius` 是否设置正确
- 检查 `inflation_radius` 是否足够大

**Q: 定位不准确**
- 检查 LiDAR 外参是否正确
- 检查 Point-LIO 外参（如果使用标定结果）

**Q: 速度过快导致控制不稳定**
- 降低 `v_linear_max` 和 `v_angular_max`
- 降低 `max_accel` 和 `max_decel`

---

## 参考

- 完整参数说明请查看配置文件中的注释
- 项目 Wiki: https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/wiki
