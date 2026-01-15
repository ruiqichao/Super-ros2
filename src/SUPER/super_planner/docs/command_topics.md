# 命令话题说明

## 概述

在SUPER项目中，存在两个重要的命令话题用于控制无人机：
- `cmd_topic`: 用于发布即时位置命令
- `mpc_cmd_topic`: 用于发布多项式轨迹命令

这两个话题体现了系统的分层控制架构，分别服务于不同的控制层级。

## cmd_topic: "/planning/pos_cmd"

### 消息类型
- `mars_quadrotor_msgs::msg::PositionCommand`

### 消息结构
- 位置 (geometry_msgs/Point)
- 速度 (geometry_msgs/Vector3)
- 加速度 (geometry_msgs/Vector3)
- 加加速度 (geometry_msgs/Vector3)
- 角速度 (geometry_msgs/Vector3)
- 姿态 (geometry_msgs/Vector3)
- 推力 (geometry_msgs/Vector3)
- 偏航角 (float64)
- 偏航角速度 (float64)
- PID控制器参数 (kx[3], kv[3])

### 功能特点
- **控制层级**: 低级控制指令
- **实时性**: 高频更新
- **应用场景**: PID控制器输入
- **数据内容**: 单时刻的状态信息

## mpc_cmd_topic: "/planning_cmd/poly_traj"

### 消息类型
- `mars_quadrotor_msgs::msg::PolynomialTrajectory`

### 消息结构
- 轨迹ID (uint32)
- 轨迹类型 (uint32 - 位置轨迹、偏航轨迹、心跳信号等)
- 轨迹多项式系数 (float64[])
- 时间分配 (float64[])
- 起始时间 (float64)

### 功能特点
- **控制层级**: 高级规划轨迹
- **时间维度**: 一段时间内的完整轨迹
- **应用场景**: 模型预测控制（MPC）输入
- **数据内容**: 多项式轨迹系数

## 系统架构关系

```
[规划器] 
    ↓ (完整轨迹)
[PolynomialTrajectory消息] → [MPC控制器]
    
[规划器]
    ↓ (即时命令) 
[PositionCommand消息] → [PID控制器]
```

## 话题配置

在配置文件中（如 `click_smooth_ros2.yaml`）：

```yaml
fsm:
  cmd_topic: "/planning/pos_cmd"               # 位置命令话题名称
  mpc_cmd_topic: "/planning_cmd/poly_traj"     # MPC命令话题名称
```

## 使用场景

- **cmd_topic**: 用于即时控制指令，适用于需要快速响应的场景
- **mpc_cmd_topic**: 用于长期轨迹跟踪，适用于需要高精度轨迹跟随的场景

这种双话题设计允许系统在保证轨迹跟踪精度的同时，具备应对突发情况的即时响应能力。