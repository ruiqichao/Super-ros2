# SUPER系统架构文档

## 1. 系统概述

SUPER（Safe Unmanned aerial vehicle Path Explorer and Replanner）是一个高性能的无人机轨迹规划系统，专为复杂环境下的高速导航设计。该系统结合了前端路径搜索和后端轨迹优化技术，提供了探索轨迹和备份轨迹的双重保障。

## 2. 架构概览

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Mission     │    │   SuperPlanner  │    │   FSM Module    │
│   Planner     │───▶│   (Core Logic)  │───▶│   (Control)     │
│               │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   Rog Map       │
                    │   (Mapping)     │
                    └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   ROS Interface │
                    │   (Communication)│
                    └─────────────────┘
```

## 3. 核心模块介绍

### 3.1 SuperPlanner (主规划器)
- **职责**: 负责轨迹规划的核心逻辑
- **功能**:
  - 探索轨迹生成 (ExpTraj)
  - 备份轨迹生成 (BackupTraj)
  - 动态重规划
  - 轨迹优化

### 3.2 Astar (路径搜索)
- **职责**: 执行A*算法进行路径搜索
- **功能**:
  - 3D网格路径规划
  - 多种启发式函数支持
  - 地图类型适配

### 3.3 TrajOpt (轨迹优化)
- **职责**: 轨迹平滑和优化
- **功能**:
  - MINCO轨迹优化算法
  - 约束处理
  - 时间和空间联合优化

### 3.4 RogMap (地图系统)
- **职责**: 环境建模和地图管理
- **功能**:
  - 3D占用栅格地图
  - 概率地图
  - 碰撞检测

## 4. 数据流

### 4.1 规划流程
```
起点状态 → A*路径搜索 → 引导路径 → 轨迹优化 → 探索轨迹 → 备份轨迹 → 控制命令
```

### 4.2 重规划流程
```
当前状态 → 环境感知 → 轨迹评估 → 重规划触发 → 轨迹更新 → 命令发布
```

## 5. 通信接口

### 5.1 ROS话题
- `/planning/pos_cmd` - 位置命令 (PositionCommand)
- `/planning_cmd/poly_traj` - 多项式轨迹 (PolynomialTrajectory)
- `/planning/px4_cmd` - PX4命令 (Command)
- `/goal_pose` - 目标点 (PoseStamped)

### 5.2 服务接口
- 轨迹重置服务
- 参数配置服务

## 6. 实时性能要求

- **规划频率**: 15Hz (默认)
- **重规划延迟**: < 100ms
- **轨迹平滑度**: 7次多项式
- **安全性**: 双轨迹保障机制

## 7. 模块间依赖关系

```
FSM Module
    ├── SuperPlanner
    │   ├── Astar (路径搜索)
    │   ├── TrajOpt (轨迹优化)
    │   │   ├── MINCO (数学优化)
    │   │   └── GeometryUtils (几何计算)
    │   └── CorridorGenerator (走廊生成)
    ├── RogMap (地图系统)
    │   ├── OccupancyMap (占用栅格)
    │   └── ProbabilisticMap (概率地图)
    └── ROS Interface
        ├── Publisher/Subscriber (通信)
        └── Visualization (可视化)
```

## 8. 扩展性设计

- **插件化架构**: 支持不同优化算法的替换
- **参数化配置**: 通过YAML文件灵活配置
- **模块化设计**: 各组件松耦合，易于维护和升级
- **接口标准化**: 清晰的API定义，便于第三方集成