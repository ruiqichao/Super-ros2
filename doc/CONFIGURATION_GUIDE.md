# SUPER项目配置指南

## 1. 配置文件结构

### 1.1 主要配置文件
```
config/
├── click_smooth_ros2.yaml          # 主配置文件
├── static_dense.yaml              # 静态密集环境配置
├── static_high_speed.yaml         # 高速飞行配置
└── click.yaml                     # 点击控制配置
```

### 1.2 配置层次结构
```yaml
# 主要配置分区
fsm:                                # 有限状态机配置
  click_goal_en: true              # 是否启用点击目标功能
  click_goal_topic: "/goal_pose"   # 点击目标话题名称
  replan_rate: 15.0                # 重规划频率 (Hz)
  cmd_topic: "/planning/pos_cmd"   # 位置命令话题名称
  mpc_cmd_topic: "/planning_cmd/poly_traj"  # MPC命令话题名称
  px4_cmd_topic: "/planning/px4_cmd"        # PX4命令话题名称

super_planner:                     # 超级规划器配置
  backup_traj_en: true             # 是否启用备份轨迹
  visualization_en: true           # 是否启用可视化
  planning_horizon: 7.0            # 规划时间范围
  robot_r: 0.2                     # 机器人半径

traj_opt:                          # 轨迹优化配置
  boundary:                        # 边界约束配置
    max_vel: 5.0                   # 最大速度
    max_acc: 5.0                   # 最大加速度
    max_jerk: 120.0                # 最大加加速度

astar:                             # A*搜索配置
  map_voxel_num: [500, 500, 100]  # 地图体素数量
  heu_type: 2                      # 启发式函数类型

rog_map:                           # ROG地图配置
  resolution: 0.1                  # 分辨率
  map_size: [50, 50, 6]           # 地图大小
```

## 2. 关键参数说明

### 2.1 规划参数
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `planning_horizon` | double | 7.0 | 规划时域长度（米） |
| `sensing_horizon` | double | -1.0 | 感知范围（米），-1表示全局 |
| `replan_rate` | double | 15.0 | 重规划频率（Hz） |
| `replan_forward_dt` | double | 0.1 | 前瞻重规划时间间隔 |

### 2.2 动力学参数
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_vel` | double | 5.0 | 最大速度限制（m/s） |
| `max_acc` | double | 5.0 | 最大加速度限制（m/s²） |
| `max_jerk` | double | 120.0 | 最大加加速度限制（m/s³） |
| `robot_r` | double | 0.2 | 机器人半径（米） |

### 2.3 优化参数
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `penna_pos` | double | 1.0e+6 | 位置惩罚系数 |
| `penna_vel` | double | 1.0e+5 | 速度惩罚系数 |
| `penna_acc` | double | 1.0e+5 | 加速度惩罚系数 |
| `opt_accuracy` | double | 1.0e-4 | 优化精度 |

## 3. 动态参数配置

### 3.1 ROS2参数服务
SUPER支持通过ROS2参数服务动态调整参数：

```bash
# 查看可用参数
ros2 param list

# 获取参数值
ros2 param get /fsm_node traj_opt.boundary.max_vel

# 设置参数值
ros2 param set /fsm_node traj_opt.boundary.max_vel 3.0
```

### 3.2 动态参数类型
- **边界约束参数**: 速度、加速度限制
- **优化权重参数**: 各项惩罚系数
- **规划参数**: 时域长度、重规划频率
- **可视化参数**: 可视化开关、显示选项

## 4. 配置最佳实践

### 4.1 环境适配
根据不同环境调整配置：

**密集环境配置**:
```yaml
super_planner:
  planning_horizon: 5.0    # 缩短规划距离
  robot_r: 0.3            # 增加安全半径
  receding_dis: 2.0       # 减少回收距离

traj_opt:
  boundary:
    max_vel: 3.0          # 降低最大速度
    max_acc: 3.0          # 降低最大加速度
```

**高速飞行配置**:
```yaml
super_planner:
  planning_horizon: 10.0   # 增加规划距离
  replan_rate: 20.0       # 提高重规划频率

traj_opt:
  boundary:
    max_vel: 8.0          # 提高最大速度
    max_acc: 8.0          # 提高最大加速度
    penna_jerk: 1.0e+5    # 增加急动度惩罚
```

### 4.2 安全配置
```yaml
super_planner:
  robot_r: 0.25           # 保守的安全半径
  corridor_bound_dis: 1.5 # 增加走廊边界距离
  safe_corridor_line_max_length: 3.0  # 限制走廊长度

rog_map:
  resolution: 0.05        # 提高地图分辨率
  inflation_step: 3       # 增加膨胀步数
```

## 5. 配置验证

### 5.1 参数验证
- **范围检查**: 确保参数在合理范围内
- **一致性检查**: 验证参数间的逻辑一致性
- **实时性检查**: 确认动态参数能够正确更新

### 5.2 配置测试
- **仿真测试**: 在仿真环境中验证配置效果
- **渐进测试**: 从小规模环境逐步扩大测试范围
- **边界测试**: 测试极端参数组合的效果

## 6. 配置管理

### 6.1 版本控制
- 配置文件纳入版本控制系统
- 重要配置变更需要文档记录
- 生产环境配置需要额外备份

### 6.2 部署建议
- 开发、测试、生产环境使用不同配置
- 配置文件权限设置为只读（部署时）
- 配置变更需要经过充分测试