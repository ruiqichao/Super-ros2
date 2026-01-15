# SUPER Planner 参数和配置指南

## 概览

SUPER Planner 支持通过 YAML 文件和 ROS2 动态参数进行灵活配置。

---

## 配置文件位置

```
super_planner/config/
├── click.yaml                 (主配置文件)
├── click_smooth_ros2.yaml     (平滑优化版)
├── static_dense.yaml          (静态高密度)
└── static_high_speed.yaml     (高速场景)
```

---

## 完整参数参考

### 1. 路径搜索 (A*)

```yaml
astar:
  resolution: 0.2              # 网格分辨率 (m)，越小越精细
  max_search_time: 0.05        # 最大搜索时间 (s)
  allow_diag: true             # 是否允许对角线移动
```

**说明**：
- `resolution`: 控制搜索精度和速度的权衡
- `max_search_time`: 防止路径搜索卡顿

---

### 2. 探索轨迹优化

```yaml
traj_opt:
  boundary:
    max_vel: 5.0               # 最大速度限制 (m/s)
    max_acc: 5.0               # 最大加速度 (m/s²)
    max_jerk: 120.0            # 最大加加速度 (m/s³)
    max_acc_thr: 10.0          # 最大推力加速度
    min_acc_thr: 1.0           # 最小推力加速度
  
  exp_traj:
    # 约束和精度
    pos_constraint_type: 2     # 1=路径点, 2=走廊约束
    block_energy_cost: false   # 是否禁用能量代价
    opt_accuracy: 1.0e-4       # 优化精度
    smooth_eps: 0.05           # 平滑参数
    integral_reso: 10          # 积分分辨率
    
    # 惩罚权重 (重要参数)
    penna_t: 12000.0           # 时间惩罚 (轨迹越短越好)
    penna_pos: 1.0e+6          # 位置惩罚 (保持在走廊内)
    penna_vel: 1.0e+5          # 速度惩罚
    penna_acc: 1.0e+5          # 加速度惩罚
    penna_jerk: 1.0e+5         # 加加速度惩罚
    penna_attract: 1.0e+2      # 吸引力惩罚 (靠近目标)
    penna_omg: 1.0e+5          # 角速度惩罚
    penna_thr: 1.0e+4          # 推力惩罚
```

**参数调优指南**：

| 场景 | 调整 | 说明 |
|------|------|------|
| 轨迹太长 | ↑ `penna_t` | 增加时间惩罚 |
| 轨迹不平滑 | ↑ `penna_acc/jerk` | 增加加速度惩罚 |
| 轨迹碰撞 | ↑ `penna_pos` | 增加位置约束 |
| 优化慢 | ↓ `opt_accuracy` | 降低精度要求 |
| 轨迹抖动 | ↑ `smooth_eps` | 增加平滑因子 |

---

### 3. 备份轨迹优化

```yaml
  backup_traj:
    # 基础参数
    uniform_time_en: true      # 是否使用均匀时间分配
    piece_num: 2               # 轨迹分段数
    pos_constraint_type: 1     # 1=点约束, 2=走廊约束
    block_energy_cost: true    # 禁用能量代价（快速响应）
    
    # 精度和平滑
    opt_accuracy: 1.0e-4
    smooth_eps: 0.05
    integral_reso: 10
    
    # 惩罚权重
    penna_t: 12000.0
    penna_ts: 1.0e+5           # 时间段惩罚（新参数）
    penna_pos: 1.0e+6
    penna_vel: 1.0e+5
    penna_acc: 1.0e+5
    penna_jerk: 1.0e+5
    penna_attract: 1.0e+2
    penna_omg: 1.0e+5
    penna_thr: 1.0e+4
    penna_max_acc_thr: 1.0e+4
    penna_min_acc_thr: 1.0e+4
```

**说明**：
- 备份轨迹通常用于紧急避障，参数更激进
- `uniform_time_en=true`: 适合高速动态场景
- `piece_num`: 越小越快，但可能轨迹质量下降

---

### 4. 平坦性模型

```yaml
  flatness:
    mass: 1.0                  # 无人机质量 (kg)
    dh: 0.7                    # 水平阻力系数
    dv: 0.8                    # 垂直阻力系数
    grav: 9.81                 # 重力加速度
    cp: 0.01                   # 推力系数
    v_eps: 0.0001              # 速度 epsilon
```

**说明**：
- 用于将轨迹转换为推力命令
- 参数需根据实际无人机调整

---

### 5. 走廊生成

```yaml
corridor:
  resolution: 0.2              # 走廊生成分辨率
  safety_margin: 0.2           # 安全边界膨胀
  erosion_steps: 3             # 膨胀步数
```

---

### 6. 视场角约束

```yaml
camera:
  enable: false                # 是否启用 FOV 约束
  fov_angle: 60.0              # 视场角 (度)
  sensor_range: 10.0           # 传感范围 (m)
```

---

### 7. 地图和规划

```yaml
super_planner:
  resolution: 0.2              # 规划分辨率
  sensing_horizon: 15.0        # 传感范围
  safe_corridor_line_max_length: 20.0
  robot_r: 0.5                 # 无人机半径 (m)
  sample_traj_dt: 0.1          # 轨迹采样间隔
  
  # 调试选项
  print_log: false             # 打印调试日志
  save_log_en: false           # 保存优化日志
  visualization_en: true       # 启用可视化
```

---

## 动态参数 (ROS2)

### 参数等级结构

```
traj_opt/
├── boundary/
│   ├── max_vel
│   ├── max_acc
│   ├── max_jerk
│   └── ...
├── exp_traj/
│   ├── penna_t
│   ├── penna_pos
│   ├── max_vel (deprecated, use boundary)
│   └── ...
└── backup_traj/
    ├── uniform_time_en
    ├── piece_num
    ├── penna_t
    └── ...
```

### 运行时修改示例

```bash
# 修改最大速度
ros2 param set /super_planner_node traj_opt.boundary.max_vel 6.0

# 修改时间惩罚
ros2 param set /super_planner_node traj_opt.exp_traj.penna_t 15000.0

# 查看所有参数
ros2 param list | grep traj_opt

# 查看参数值
ros2 param get /super_planner_node traj_opt.boundary.max_vel
```

### 动态参数回调

参数变化时自动回调（见 `fsm_ros2.hpp`）：

```cpp
void dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
    planner_ptr_->updateRuntimeParams(nh_);
    // 参数已自动同步到所有子模块
}
```

---

## 预配置场景

### 场景 1: 高速导航

```yaml
traj_opt:
  boundary:
    max_vel: 8.0
    max_acc: 8.0
    max_jerk: 150.0
  
  exp_traj:
    penna_t: 15000          # 优先速度
    penna_pos: 0.5e+6       # 放松位置约束
    smooth_eps: 0.1         # 降低平滑
```

---

### 场景 2: 精细避障

```yaml
traj_opt:
  boundary:
    max_vel: 2.0
    max_acc: 3.0
    max_jerk: 50.0
  
  exp_traj:
    penna_t: 5000           # 放松时间约束
    penna_pos: 2.0e+6       # 加强位置约束
    smooth_eps: 0.05        # 增加平滑
```

---

### 场景 3: 自主探索

```yaml
traj_opt:
  exp_traj:
    penna_attract: 1.0e+3   # 增加吸引力
    penna_pos: 1.5e+6       # 平衡约束
  
  backup_traj:
    piece_num: 3            # 更多段数
    uniform_time_en: false  # 非均匀时间分配
```

---

### 场景 4: 室内环境

```yaml
astar:
  resolution: 0.1           # 精细网格
  allow_diag: false         # 禁用对角线
  
  corridor:
    safety_margin: 0.5      # 大安全边界
    erosion_steps: 5
```

---

## 参数调试流程

### 1. 编译并启动节点

```bash
colcon build --packages-select super_planner
ros2 launch super_planner real_drone.launch.py
```

### 2. 使用 rqt_reconfigure 动态调整

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

在 GUI 中找到 `/super_planner_node` 并实时调整参数。

### 3. 观察效果

- **RViz 可视化**：查看规划的轨迹、走廊
- **日志输出**：检查优化迭代次数、时间消耗
- **实际飞行**：测试轨迹平滑度、响应速度

### 4. 固化最优参数

最优参数后保存到 `config/click.yaml`：

```bash
# 从参数服务器导出
ros2 param dump /super_planner_node > params.yaml
```

---

## 常见问题

### Q1: 轨迹优化总是失败

**检查清单**：
1. 走廊是否有效（不为空）
2. 惩罚权重是否过大
3. 初值是否合理

**解决**：
```yaml
traj_opt:
  exp_traj:
    opt_accuracy: 1.0e-3    # 降低精度要求
    penna_pos: 0.8e+6       # 放松约束
```

---

### Q2: 轨迹有跳跃或不连续

**原因**：加加速度惩罚过小或平滑参数不足

**解决**：
```yaml
traj_opt:
  exp_traj:
    penna_jerk: 5.0e+5      # 增加加加速度惩罚
    smooth_eps: 0.1         # 增加平滑因子
```

---

### Q3: 规划速度慢

**原因**：分辨率过高、惩罚权重过大

**解决**：
```yaml
astar:
  resolution: 0.3           # 降低分辨率
traj_opt:
  exp_traj:
    opt_accuracy: 1.0e-3    # 降低精度
    integral_reso: 5        # 降低积分分辨率
```

---

### Q4: 动态参数不生效

**检查**：
1. 参数名是否正确（区分大小写）
2. 节点是否收到参数更新回调
3. 是否需要重启节点

```bash
# 强制重新加载参数
ros2 service call /super_planner_node/describe_parameters rcl_interfaces/srv/ListParameters
```

---

## 最佳实践

1. **先用高精度、保守参数测试**
   - 大安全边界、低速
   - 确保不碰撞后再优化性能

2. **逐项调整单个参数**
   - 一次只改一个，观察效果
   - 不要一次改多个参数

3. **记录各场景的最优参数**
   - 建立参数库
   - 根据环境切换配置

4. **定期备份工作配置**
   - 保存有效的参数文件
   - 版本控制

5. **使用日志记录性能指标**
   ```bash
   ros2 run super_planner super_planner_node > planner.log 2>&1
   ```

