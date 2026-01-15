# SUPER Planner 快速开始指南

## 10 分钟快速上手

### 1. 编译

```bash
cd ~/ros2_ws/super_ws
colcon build --packages-select super_planner rog_map
```

---

### 2. 启动规划器

```bash
# 方式 1: 使用 launch 文件
ros2 launch super_planner real_drone.launch.py

# 方式 2: 直接运行节点
ros2 run super_planner fsm_node_ros2 --ros-args \
    --params-file config/click.yaml
```

---

### 3. 在 RViz 中可视化

```bash
# 启动 RViz
rviz2

# 订阅关键话题：
# - /planning/traj_pos (Trajectory)
# - /planning/sfc_cloud (PointCloud)
# - /planning/path (Path)
```

---

### 4. 设置目标并规划

```bash
# 发送目标点
ros2 topic pub -1 /planning/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 20, y: 20, z: 2}}}"

# 观察规划结果
ros2 topic echo /planning/traj_pos
```

---

## 核心概念速查

### 规划流程 (5 步)

```
目标 → 路径搜索 → 走廊生成 → 轨迹优化 → 备份轨迹 → 输出
```

| 步骤 | 输入 | 处理 | 输出 |
|------|------|------|------|
| 路径搜索 | 起点、目标、地图 | A* 算法 | 离散路径点 |
| 走廊生成 | 路径、地图 | 膨胀障碍物 | 安全走廊 |
| 主轨迹 | 走廊、初值 | LBFGS 优化 | 光滑轨迹 |
| 备份轨迹 | 主轨迹、地图 | 快速优化 | 应急轨迹 |

---

### 返回码含义

| 代码 | 含义 | 动作 |
|------|------|------|
| SUCCESS | 规划成功 | ✅ 发送轨迹 |
| NO_NEED | 不需要重规划 | ⏭️ 继续执行 |
| NO_PATH | 路径不存在 | ❌ 目标无法到达 |
| OPT_FAILED | 轨迹优化失败 | 🔄 重试或调整参数 |
| FAILED | 一般失败 | 📋 检查日志 |

---

## 常用代码片段

### 片段 1: 单次规划

```cpp
#include <super_core/super_planner.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("planner_test");
    
    // 初始化规划器（需要提供 map_ptr 和 ros_ptr）
    auto planner = std::make_shared<super_planner::SuperPlanner>(
        config_path, ros_interface, map_ptr
    );
    
    // 规划到目标
    Vec3f goal(50, 50, 5);
    auto ret = planner->PlanFromRest(goal, 0.0, true);
    
    if (ret == RET_CODE::SUCCESS) {
        auto traj = planner->getCommittedPositionTrajectory();
        RCLCPP_INFO(node->get_logger(), 
                   "Planned trajectory with duration: %.2f s", 
                   traj.getTotalDuration());
    }
    
    rclcpp::shutdown();
    return 0;
}
```

---

### 片段 2: 实时控制

```cpp
void controlLoop() {
    rclcpp::Rate rate(100);  // 100 Hz
    
    while (rclcpp::ok()) {
        // 1. 获取当前命令
        StatePVAJ pvaj;
        double yaw, yaw_dot;
        bool on_backup, finish;
        planner->getOneCommandFromTraj(pvaj, yaw, yaw_dot, 
                                       on_backup, finish);
        
        // 2. 发送到控制器
        PositionCommand cmd;
        cmd.position = Vector3d(pvaj(0, 0), pvaj(0, 1), pvaj(0, 2));
        cmd.velocity = Vector3d(pvaj(1, 0), pvaj(1, 1), pvaj(1, 2));
        cmd.acceleration = Vector3d(pvaj(2, 0), pvaj(2, 1), pvaj(2, 2));
        cmd_pub->publish(cmd);
        
        // 3. 检查是否完成
        if (finish) {
            RCLCPP_INFO(logger, "Trajectory completed");
            // 规划新目标
            Vec3f next_goal = getNextGoal();
            planner->ReplanOnce(next_goal, 0.0, true);
        }
        
        rate.sleep();
    }
}
```

---

### 片段 3: 动态参数调整

```cpp
void onParameterUpdate() {
    // 从参数服务器同步参数
    planner->updateRuntimeParams(node);
    
    // 验证新参数
    auto& exp_cfg = planner->getExpTrajOptDynamicConfig();
    {
        std::lock_guard<std::mutex> lock(exp_cfg.getConfigMutex());
        RCLCPP_INFO(logger, "Updated max_vel: %.2f m/s", exp_cfg.max_vel);
        RCLCPP_INFO(logger, "Updated max_acc: %.2f m/s²", exp_cfg.max_acc);
    }
}
```

---

### 片段 4: 性能监控

```cpp
// 计算平均规划时间
double avg_time_ms = planner->getFrontendTime() * 1000;
if (avg_time_ms > 0) {
    RCLCPP_WARN_THROTTLE(logger, *clock, 1000,
        "Planning time: %.1f ms", avg_time_ms);
}

// 获取详细日志
auto log = planner->getLatestReplanLog();
std::cout << "Success: " << (log.getRetCode() > 0 ? "Yes" : "No") << std::endl;
```

---

## 调试技巧

### 技巧 1: 启用日志输出

在配置文件中：
```yaml
super_planner:
  print_log: true
  save_log_en: true
```

运行时查看日志：
```bash
ros2 run super_planner fsm_node_ros2 --ros-args --log-level DEBUG
```

---

### 技巧 2: 在 RViz 中查看走廊

添加 PointCloud2 订阅：
- 话题：`/planning/sfc_cloud`
- 颜色：绿色表示有效走廊

---

### 技巧 3: 使用 rqt_reconfigure 调参

```bash
ros2 run rqt_reconfigure rqt_reconfigure &
```

实时观察参数变化的效果。

---

### 技巧 4: 记录轨迹数据

```bash
# 录制 bag 文件
ros2 bag record /planning/traj_pos /planning/traj_yaw

# 回放分析
ros2 bag play rosbag2_*.db3
```

---

## 常见场景

### 场景 1: 无人机卡住不动

**排查**：
1. 检查目标是否在已探索区域
   ```bash
   ros2 topic echo /rog_map/occ | head -20
   ```

2. 查看规划返回码
   ```cpp
   if (ret == RET_CODE::NO_PATH) {
       RCLCPP_ERROR(logger, "No path to goal");
   }
   ```

3. 尝试更近的目标
   ```bash
   ros2 param set ... traj_opt.boundary.max_vel 2.0
   ```

---

### 场景 2: 轨迹碰撞

**原因**：通常是安全边界不足

**解决**：
```bash
# 增加膨胀步数
ros2 param set /rog_map_node rog_map.inflation_step 3

# 增加位置约束
ros2 param set ... traj_opt.exp_traj.penna_pos 2.0e+6
```

---

### 场景 3: 规划太慢

**原因**：分辨率过高或网络延迟

**解决**：
```yaml
# config/click.yaml
astar:
  resolution: 0.3        # 从 0.2 改为 0.3
  max_search_time: 0.03  # 减少搜索时间

traj_opt:
  exp_traj:
    opt_accuracy: 1.0e-3 # 从 1.0e-4 改为 1.0e-3
```

---

### 场景 4: 轨迹不平滑

**原因**：加速度或加加速度惩罚太低

**解决**：
```bash
ros2 param set ... traj_opt.exp_traj.penna_acc 2.0e+5
ros2 param set ... traj_opt.exp_traj.penna_jerk 2.0e+5
```

---

## 与其他模块的交互

### 与地图的交互

```cpp
// 获取地图指针
auto map = planner->getMap();

// 检查点是否碰撞
if (map->isOccupiedInflate(test_point)) {
    RCLCPP_WARN(logger, "Point in collision");
}

// 获取最近障碍物
Vec3f nearest;
if (map->getNearestCellIs(GridType::OCCUPIED, 
                          robot_pos, nearest, 5.0)) {
    RCLCPP_WARN(logger, "Nearest obstacle: %.2f m away", 
               (nearest - robot_pos).norm());
}
```

---

### 与控制器的交互

```cpp
// 获取当前轨迹命令
StatePVAJ cmd;
planner->getOneCommandFromTraj(cmd, yaw, yaw_dot, on_backup, finish);

// 转换为控制指令
geometry_msgs::PointStamped pos_cmd;
pos_cmd.point.x = cmd(0, 0);  // x 位置
pos_cmd.point.y = cmd(0, 1);  // y 位置
pos_cmd.point.z = cmd(0, 2);  // z 位置
controller_pub->publish(pos_cmd);
```

---

### 与任务规划的交互

```cpp
// 目标跟踪
void trackTarget(const Vec3f& target) {
    auto ret = planner->ReplanOnce(target, target_yaw, true);
    
    if (ret == RET_CODE::SUCCESS) {
        // 发送下一个任务
    } else if (ret == RET_CODE::NO_PATH) {
        // 目标无法到达，选择替代目标
    }
}
```

---

## 性能基准

### 典型性能指标

| 指标 | 数值 | 备注 |
|------|------|------|
| 路径搜索耗时 | 30-50ms | 取决于距离和分辨率 |
| 走廊生成耗时 | 10-20ms | 线性复杂度 |
| 轨迹优化耗时 | 100-200ms | LBFGS 迭代 |
| 总规划耗时 | 150-400ms | 完整周期 |
| 轨迹跟踪频率 | 50-100Hz | 命令生成 |

---

## 下一步

1. **阅读完整 API 文档**：`API_REFERENCE.md`
2. **参数调优指南**：`CONFIGURATION_GUIDE.md`
3. **部署指南**：`DEPLOYMENT_GUIDE.md`
4. **源码注释**：`include/super_core/super_planner.h`

---

## 获取帮助

### 查看日志

```bash
# 实时日志
tail -f ~/.ros/log/$(ls -dt /root/.ros/log/*/ | head -1)/rosout.log

# 或使用 rclcpp 的日志聚合
ros2 run super_planner fsm_node_ros2 2>&1 | tee planner.log
```

### 常见错误码

- `SUPER_NO_PATH`: 路径搜索失败，检查地图和目标
- `OPT_FAILED`: 轨迹优化失败，尝试放松约束
- `CORRIDOR_GENERATION_ERROR`: 走廊生成失败，检查地图分辨率
- `PATH_VISIBILITY_ERROR`: 路径可见性检查失败

