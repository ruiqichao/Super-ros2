# SUPER Planner API 文档

## 目录
1. [概述](#概述)
2. [架构设计](#架构设计)
3. [核心类 API](#核心类-api)
4. [轨迹优化器](#轨迹优化器)
5. [规划流程](#规划流程)
6. [使用示例](#使用示例)
7. [返回码参考](#返回码参考)

---

## 概述

**SUPER** 是面向多旋翼无人机的**高速自主导航规划系统**，包含路径搜索、轨迹优化、备份轨迹等模块。

**核心能力**：
- ✅ 快速路径搜索（A* 算法）
- ✅ 双层轨迹优化（探索轨迹 + 备份轨迹）
- ✅ 动态参数配置（ROS2 参数服务器）
- ✅ 安全保证（膨胀碰撞检测）
- ✅ 目标跟踪和自主探索

---

## 架构设计

```
┌─────────────────────────────────────┐
│    SuperPlanner (核心规划器)         │
│  - 路径搜索                          │
│  - 轨迹优化协调                      │
└──────────────────┬──────────────────┘
         ┌─────────┼─────────┐
         ↓         ↓         ↓
    ┌────────┐ ┌──────────┐ ┌──────────┐
    │ Astar  │ │ExpTrajOpt│ │BackupTraj│
    │(路径)  │ │(探索轨迹)│ │Opt(备份) │
    └────────┘ └──────────┘ └──────────┘
         ↓         ↓         ↓
    ┌────────┐ ┌──────────┐ ┌──────────┐
    │ ROGMap │ │ MINCO_S4 │ │ MINCO_S4 │
    │(地图)  │ │(多项式)  │ │(多项式)  │
    └────────┘ └──────────┘ └──────────┘
         ↓         ↓         ↓
    ┌──────────────────────────────────┐
    │   CorridorGenerator (走廊生成)    │
    └──────────────────────────────────┘
         ↓
    ┌──────────────────────────────────┐
    │   FOVChecker (视场角检测)         │
    └──────────────────────────────────┘
```

---

## 核心类 API

### SuperPlanner 类

**位置**：`include/super_core/super_planner.h`

#### 初始化

```cpp
class SuperPlanner {
public:
    // 构造函数
    explicit SuperPlanner(const std::string &cfg_path,
                         const ros_interface::RosInterface::Ptr &ros_ptr,
                         const rog_map::ROGMapROS::Ptr &map_ptr);
    
    typedef std::shared_ptr<SuperPlanner> Ptr;
};
```

**参数**：
- `cfg_path`: YAML 配置文件路径
- `ros_ptr`: ROS2 接口指针
- `map_ptr`: ROG-Map 指针

**示例**：
```cpp
auto planner = std::make_shared<SuperPlanner>(
    config_path, ros_interface_ptr, map_ptr
);
```

---

#### 规划接口（公共 API）

##### 1. 从静止状态规划

```cpp
RET_CODE PlanFromRest(const Vec3f &goal_p,
                      const double &goal_yaw,
                      const bool &new_goal);
```

**参数**：
- `goal_p`: 目标位置（3D 世界坐标）
- `goal_yaw`: 目标偏航角（弧度）
- `new_goal`: 是否为新目标（`true` = 重新规划，`false` = 继续上一个目标）

**返回值**：`RET_CODE` 枚举（见[返回码参考](#返回码参考)）

**说明**：
- 用于无人机从停止状态开始规划
- 生成完整的位置轨迹和偏航轨迹
- 包含路径搜索 → 走廊生成 → 轨迹优化全流程

---

##### 2. 动态重规划

```cpp
RET_CODE ReplanOnce(const Vec3f &goal_p,
                    const double &goal_yaw,
                    const bool &new_goal);
```

**参数**：同 `PlanFromRest`

**返回值**：`RET_CODE` 枚举

**说明**：
- 在无人机飞行中进行增量重规划
- 复用已生成的轨迹（若仍有效）
- 仅重新规划受障碍物影响的部分

---

#### 轨迹获取接口

##### 3. 获取已提交的轨迹

```cpp
// 获取位置轨迹
Trajectory getCommittedPositionTrajectory();

// 获取偏航轨迹
Trajectory getCommittedYawTrajectory();
```

**返回值**：`Trajectory` 对象（多项式轨迹）

**说明**：
- 返回当前执行中的已确认轨迹
- 用于实时轨迹跟踪控制

---

##### 4. 获取单条命令

```cpp
void getOneCommandFromTraj(StatePVAJ &pvaj,        // 输出：位置/速度/加速度/加加速度
                          double &yaw,             // 输出：偏航角
                          double &yaw_dot,         // 输出：偏航角速度
                          bool &on_backup_traj,    // 输出：是否在备份轨迹上
                          bool &traj_finish);      // 输出：轨迹是否完成
```

**功能**：从轨迹中提取当前时刻的状态命令

**使用场景**：控制循环中的实时指令获取

---

##### 5. 心跳信息

```cpp
void getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish);
```

**功能**：获取轨迹的心跳检测信息

**参数**：
- `start_WT_pos`: 轨迹起始时刻
- `traj_finish`: 轨迹是否完成

---

#### 状态查询接口

##### 6. 获取机器人状态

```cpp
void getRobotState(rog_map::RobotState &out);
```

**输出**：
```cpp
struct RobotState {
    Vec3f p;           // 位置
    Quatf q;           // 四元数
    Vec3f v;           // 速度
    bool rcv;          // 是否接收到状态
    double rcv_time;   // 接收时间戳
};
```

---

##### 7. 检查目标是否可达

```cpp
bool isEasyGoal(const Vec3f &goal_position);
```

**功能**：判断目标是否直接可达（直线路径无碰撞）

**返回值**：`true` = 可直接到达，`false` = 需要规划

---

##### 8. 获取地图指针

```cpp
rog_map::ROGMapROS::Ptr &getMap();
```

**功能**：访问底层占用栅格地图

---

#### 动态参数接口

##### 9. 获取探索轨迹优化器的动态配置

```cpp
traj_opt::SuperTrajConfig& getExpTrajOptDynamicConfig();
```

**返回值**：动态配置对象引用

**说明**：
- 用于访问和修改探索轨迹的实时参数
- 包括速度、加速度、惩罚权重等

---

##### 10. 获取备份轨迹优化器的动态配置

```cpp
traj_opt::BackupTrajConfig& getBackupTrajOptDynamicConfig();
```

**返回值**：动态配置对象引用

---

##### 11. 更新运行时参数

```cpp
void updateRuntimeParams(rclcpp::Node::SharedPtr node);
```

**功能**：从 ROS2 参数服务器同步所有动态参数到规划器

**说明**：
- 在参数回调中调用
- 自动同步到所有子模块（优化器、地图等）
- 线程安全（使用互斥锁）

---

##### 12. 获取优化器指针

```cpp
traj_opt::ExpTrajOpt::Ptr getExpTrajOpt();
traj_opt::BackupTrajOpt::Ptr getBackupTrajOpt();
```

**功能**：直接访问优化器对象

---

#### 轨迹锁定接口

```cpp
void lockCommittedTraj();      // 锁定轨迹（线程安全）
void unlockCommittedTraj();    // 解锁轨迹
```

**说明**：多线程访问轨迹时的同步保护

---

#### 性能统计接口

##### 13. 获取时间消耗

```cpp
void getModuleTimeConsuming(vector<double> &time);

double getFrontendTime();   // 平均前端耗时 (ms)
double getBackendTime();    // 平均后端耗时 (ms)
```

**功能**：性能分析和监控

---

#### 地图更新接口

```cpp
void updateROGMap(const rog_map::PointCloud &cloud, 
                  const super_utils::Pose &pose) const;
```

**功能**：更新底层占用栅格地图

---

#### 日志接口

```cpp
LogOneReplan getLatestReplanLog();
```

**功能**：获取最新一次重规划的详细日志信息

---

### 轨迹优化器

#### ExpTrajOpt 类（探索轨迹）

**位置**：`include/traj_opt/exp_traj_optimizer_s4.h`

##### 初始化

```cpp
class ExpTrajOpt {
public:
    // 从配置文件初始化
    ExpTrajOpt(const traj_opt::Config &cfg,
               const ros_interface::RosInterface::Ptr &ros_ptr);
    
    typedef std::shared_ptr<ExpTrajOpt> Ptr;
};
```

---

##### 轨迹优化接口

```cpp
bool optimize(
    const PolytopeVec &sfcs,          // 输入：走廊
    const Vec3f &start_pt,            // 起点
    const Trajectory &guide_traj,     // 引导轨迹
    Trajectory &out_traj              // 输出：优化后的轨迹
);
```

**参数**：
- `sfcs`: 安全走廊（多个凸多面体）
- `guide_traj`: 初始引导轨迹（用于初值）
- `out_traj`: 输出的优化轨迹

**返回值**：`true` = 优化成功，`false` = 失败

---

##### 动态配置

```cpp
traj_opt::SuperTrajConfig& getDynamicConfig();
void updateOptimizerConfig();
```

**功能**：获取和更新动态参数

---

#### BackupTrajOpt 类（备份轨迹）

**位置**：`include/traj_opt/backup_traj_optimizer_s4.h`

##### 初始化

```cpp
class BackupTrajOpt {
public:
    BackupTrajOpt(const traj_opt::Config &cfg,
                  const ros_interface::RosInterface::Ptr &ros_ptr);
    
    typedef std::shared_ptr<BackupTrajOpt> Ptr;
};
```

---

##### 轨迹优化接口

```cpp
// 版本 1：基础优化
bool optimize(const Trajectory &exp_traj,
              const double &t_0,
              const double &t_e,
              const double &heu_ts,
              const VecDf &heu_end_pt,
              double &heu_dur,
              const Polytope &sfc,
              Trajectory &out_traj,
              double &out_ts,
              const bool &debug = false);

// 版本 2：使用初值优化
bool optimize(const Trajectory &exp_traj,
              const double &t_0,
              const double &t_e,
              const double &heu_ts,
              const Polytope &sfc,
              const VecDf &init_t_vec,
              const vec_Vec3f &init_ps,
              Trajectory &out_traj,
              double &out_ts);
```

**参数**：
- `exp_traj`: 参考探索轨迹
- `t_0`, `t_e`: 时间范围 [起始时刻, 结束时刻]
- `heu_ts`: 启发式启动时刻
- `sfc`: 单个安全走廊
- `out_ts`: 输出的启动时刻

---

##### 初值获取

```cpp
void getInitValue(double &ts, VecDf &times, vec_Vec3f &ps);
```

**功能**：获取上次优化的初值（用于下一次热启动）

---

### 其他核心组件

#### CorridorGenerator 类

**位置**：`include/super_core/corridor_generator.h`

```cpp
class CorridorGenerator {
public:
    // 从路径生成安全走廊
    bool GenerateCorridorFromPath(const vec_Vec3f &path, 
                                  PolytopeVec &out_sfcs);
    
    // 从直线生成走廊
    bool GeneratePolytopeFromLine(const Line &line, 
                                  Polytope &polytope);
    
    // 获取最新生成的点云（用于可视化）
    PointCloud getLatestCloud();
    
    typedef std::shared_ptr<CorridorGenerator> Ptr;
};
```

**功能**：
- 基于地图信息生成膨胀安全走廊
- 保证轨迹在走廊内不碰撞

---

#### FOVChecker 类

**位置**：`include/super_core/fov_checker.h`

```cpp
class FOVChecker {
public:
    // 根据视场角裁剪走廊
    bool cutPolyByFov(const Vec3f &camera_pos,
                      const Quatf &camera_q,
                      const Vec3f &target_pos,
                      Polytope &poly);
    
    // 根据传感器范围裁剪走廊
    bool cutPolyBySensingHorizon(const Vec3f &robot_pos,
                                 const Vec3f &target_pos,
                                 double horizon,
                                 Polytope &poly);
    
    typedef std::shared_ptr<FOVChecker> Ptr;
};
```

**功能**：视场角和传感范围的约束处理

---

#### CIRI 类

**位置**：`include/super_core/ciri.h`

```cpp
class CIRI {
public:
    // 检测是否需要紧急制动
    bool needEmergencyStop(const Vec3f &pos, 
                          const Vec3f &vel,
                          double safe_dist);
    
    typedef std::shared_ptr<CIRI> Ptr;
};
```

**功能**：碰撞检测和紧急回避

---

## 规划流程

### 完整规划周期

```
PlanFromRest / ReplanOnce
    ↓
┌─────────────────────────────────────┐
│ 1. 路径搜索 (Astar)                  │
│    输入：起点、目标点、地图           │
│    输出：离散路径点                   │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│ 2. 走廊生成 (CorridorGenerator)      │
│    输入：离散路径、地图               │
│    输出：安全走廊序列                 │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│ 3. 视场角裁剪 (FOVChecker)           │
│    输入：走廊、相机参数               │
│    输出：裁剪后走廊                   │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│ 4. 轨迹优化 (ExpTrajOpt)             │
│    输入：走廊、初值                   │
│    输出：平滑轨迹                     │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│ 5. 备份轨迹生成 (BackupTrajOpt)      │
│    输入：主轨迹、地图                 │
│    输出：紧急备份轨迹                 │
└─────────────────────────────────────┘
    ↓
输出：完整轨迹 (位置 + 偏航)
```

---

## 使用示例

### 示例 1：基础规划

```cpp
#include <super_core/super_planner.h>

int main() {
    // 初始化
    auto planner = std::make_shared<super_planner::SuperPlanner>(
        config_path, ros_ptr, map_ptr
    );
    
    // 设置目标
    Vec3f goal(10.0, 10.0, 2.0);
    double goal_yaw = 0.0;
    
    // 规划
    auto ret = planner->PlanFromRest(goal, goal_yaw, true);
    
    if (ret == RET_CODE::SUCCESS) {
        RCLCPP_INFO(logger, "Planning succeeded");
        
        // 获取轨迹
        auto pos_traj = planner->getCommittedPositionTrajectory();
        auto yaw_traj = planner->getCommittedYawTrajectory();
    } else {
        RCLCPP_ERROR(logger, "Planning failed with code: %d", ret);
    }
    
    return 0;
}
```

---

### 示例 2：实时控制循环

```cpp
void controlLoop() {
    rclcpp::Rate rate(100);  // 100 Hz
    
    while (rclcpp::ok()) {
        // 获取当前状态
        StatePVAJ pvaj;
        double yaw, yaw_dot;
        bool on_backup, traj_finish;
        
        planner->getOneCommandFromTraj(pvaj, yaw, yaw_dot, 
                                       on_backup, traj_finish);
        
        // 发送控制命令
        sendCommand(pvaj, yaw);
        
        // 轨迹完成则规划新轨迹
        if (traj_finish) {
            Vec3f new_goal = getNextGoal();
            planner->ReplanOnce(new_goal, 0.0, true);
        }
        
        rate.sleep();
    }
}
```

---

### 示例 3：动态参数调整

```cpp
void dynamicParameterCallback(const std::vector<rclcpp::Parameter> &params) {
    // 更新规划器参数
    planner->updateRuntimeParams(node);
    
    // 获取探索轨迹优化器的配置
    auto &exp_cfg = planner->getExpTrajOptDynamicConfig();
    {
        std::lock_guard<std::mutex> lock(exp_cfg.getConfigMutex());
        // 新参数已自动同步
        RCLCPP_INFO(logger, "Updated max_vel: %.2f", exp_cfg.max_vel);
    }
}
```

---

### 示例 4：目标跟踪

```cpp
void trackTarget(const Vec3f &target_pos) {
    static Vec3f last_target = Vec3f::Zero();
    
    // 目标变化时重规划
    if ((target_pos - last_target).norm() > 0.5) {
        auto ret = planner->ReplanOnce(target_pos, 0.0, true);
        
        if (ret == RET_CODE::SUCCESS) {
            last_target = target_pos;
        } else {
            RCLCPP_WARN(logger, "Replan failed");
        }
    }
    
    // 获取当前命令
    StatePVAJ cmd;
    double yaw, yaw_dot;
    bool on_backup, finish;
    planner->getOneCommandFromTraj(cmd, yaw, yaw_dot, on_backup, finish);
    
    publishCommand(cmd);
}
```

---

## 返回码参考

### RET_CODE 枚举

**位置**：`include/super_core/super_ret_code.hpp`

```cpp
enum class RET_CODE : int {
    SUCCESS                    = 1,   // 规划成功
    FINISH                    = 2,   // 轨迹完成
    
    FAILED                    = -1,  // 规划失败
    SUPER_NO_START_POINT      = -2,  // 无起始点
    SUPER_NO_GOAL_POINT       = -3,  // 无目标点
    SUPER_NO_PATH             = -4,  // 路径搜索失败
    SUPER_WRONG_PARAM         = -5,  // 参数错误
    OPT_FAILED                = -6,  // 轨迹优化失败
    NO_NEED                   = -7,  // 不需要重规划
    PATH_VISIBILITY_ERROR     = -8,  // 路径可见性错误
    CORRIDOR_GENERATION_ERROR = -9,  // 走廊生成错误
    FRONTIER_NO_UPDATE        = -10, // 边界未更新
};
```

---

## 数据结构

### StatePVAJ (位置/速度/加速度/加加速度)

```cpp
typedef Eigen::Matrix<double, 4, 3> StatePVAJ;
//  行 0: 位置 [x, y, z]
//  行 1: 速度 [vx, vy, vz]
//  行 2: 加速度 [ax, ay, az]
//  行 3: 加加速度 [jx, jy, jz]
```

---

### Trajectory (多项式轨迹)

```cpp
class Trajectory {
public:
    // 获取指定时刻的位置
    Vec3f getPos(double t) const;
    
    // 获取指定时刻的速度
    Vec3f getVel(double t) const;
    
    // 获取指定时刻的加速度
    Vec3f getAcc(double t) const;
    
    // 获取指定时刻的状态 [P, V, A, J]
    StatePVAJ getState(double t) const;
    
    // 获取总时长
    double getTotalDuration() const;
    
    // 获取分段数
    int getPieceNum() const;
    
    // 获取轨迹起始时刻（墙上时间）
    double getStartWallTime() const;
    
    // 检查轨迹是否为空
    bool empty() const;
};
```

---

## 配置参数

### 关键配置项

**位置**：`config/click.yaml`

```yaml
# 路径搜索
astar:
  resolution: 0.2        # A* 网格分辨率
  max_search_time: 0.05  # 最大搜索时间
  
# 轨迹优化
traj_opt:
  exp_traj:
    max_vel: 5.0         # 最大速度
    max_acc: 5.0         # 最大加速度
    max_jerk: 20.0       # 最大加加速度
    penna_t: 12000       # 时间惩罚权重
    penna_pos: 1e6       # 位置惩罚权重
    
  backup_traj:
    max_vel: 3.0
    piece_num: 2
    uniform_time_en: true
```

---

## 性能指标

| 模块 | 耗时 | 说明 |
|------|------|------|
| 路径搜索 | 50ms | A* 算法 |
| 走廊生成 | 20ms | 线性膨胀 |
| 轨迹优化 | 100-200ms | LBFGS 迭代 |
| 备份轨迹 | 50-100ms | 快速优化 |
| **总计** | **200-400ms** | 完整规划周期 |

---

## 常见问题

**Q1: 如何修改轨迹的最大速度？**

A: 通过 ROS2 参数服务器：
```bash
ros2 param set /super_planner_node traj_opt.exp_traj.max_vel 6.0
```

**Q2: 备份轨迹何时生成？**

A: 当探索轨迹被障碍物阻挡时，自动生成备份轨迹。

**Q3: 如何调试规划失败？**

A: 检查返回码并查看日志：
```cpp
auto log = planner->getLatestReplanLog();
```

