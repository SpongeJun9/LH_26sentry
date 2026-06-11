# lh_rm2025_decision 开源报告

本仓库由重庆三峡科技大学 LIonHeart 机器人战队算法组开源，主要借鉴了 2023 年沈航的开源框架。

## 范围
`lh_rm2025_decision` 不是单一 ROS 包，而是一个决策层工作区目录，当前包含 3 个子包：
- `dec_run`
- `dec_fake_publisher`
- `position_monitor`

## 模块职责
- `dec_run`: 决策主逻辑，订阅串口/导航状态/目标状态并输出决策消息。
- `dec_fake_publisher`: 用于调试的假数据发布器。
- `position_monitor`: 监听 TF 并发布当前位姿，便于调试和联调。

## 依赖概览
- ROS 2 基础依赖：`rclcpp`、`rclcpp_action`、`rclcpp_components`、`geometry_msgs`、`std_msgs`、`action_msgs`、`nav2_msgs`、`tf2_ros`、`tf2_geometry_msgs`
- 项目内消息包：`core_global`
- 第三方库：`jsoncpp`（`dec_run` 直接包含并链接）

## 与其他框架的关系
- `lh_rm2025_decision` 不是一个独立替代 `Nav2` 的导航框架，而是位于上层的决策层，最终仍通过 `NavigateThroughPoses` action 向导航层下发目标。
- 这个项目在整体思路上借鉴了 2023 年沈航的开源框架，但实现上更偏向“数据驱动决策”，把策略和参数尽量放到配置文件里维护。
- 和常见的 `BehaviorTree` 方案相比，它没有把全部逻辑组织成树节点，而是把“条件判断 + 权重选择 + 目标点/模式映射”拆成 JSON 配置和代码两层，便于比赛中快速改表调参。

## 项目特点
- 用 JSON 文件代替行为树来描述决策内容：`policy` 负责策略条件和目标点，`wayPoint` 负责地图路径点，`mode` 负责动作模式参数。
- 启动时读取配置文件，运行时根据血量、占点、敌情、当前路径点等条件筛选策略，再按权重选出当前策略。
- 决策输出比较轻量，最终只发布统一的 `DecisionMsg`，包含 `id`、`mode`、`spin_speed`、`if_spin`、`if_super_cat` 等字段，方便其他模块消费。
- `json_dir`、`policy_path`、`way_point_path`、`mode_path` 都可以通过参数配置，适合不同场景快速切换。
- 代码里保留了巡航、驻守、导航等不同状态的处理逻辑，整体更像一个“可配置策略表 + 执行器”的组合，而不是传统意义上的行为树引擎。

## 开源状态检查
- 3 个子包的 `package.xml` 目前仍有 `TODO: License declaration`
- `dec_run` 依赖 `jsoncpp`，需要在对外发布前确认第三方许可和分发方式
- `core_global` 属于项目内接口层，发布时应和决策层一起统一说明许可

## 结论
从代码结构看，`lh_rm2025_decision` 已经具备明确的模块划分，适合开源发布前整理成独立说明文档。当前最需要补齐的是许可证字段、第三方依赖说明，以及对外发布时的包名一致性。
