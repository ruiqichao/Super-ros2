# SUPER项目动态参数配置说明

## 概述

SUPER项目支持通过ROS2参数服务器进行动态参数调整，无需重启节点即可实时调整轨迹优化器的参数。这些参数可以通过`rqt_reconfigure`、命令行工具或编程方式进行动态修改。

## 支持动态调试的参数

### 1. 显式轨迹优化器 (ExpTrajOpt) 参数

#### 边界约束参数
- `traj_opt.boundary.max_vel` - 最大速度 (double)
- `traj_opt.boundary.max_acc` - 最大加速度 (double) 
- `traj_opt.boundary.max_jerk` - 最大加加速度 (double)

#### 惩罚系数参数
- `traj_opt.exp_traj.penna_t` - 时间惩罚系数 (double)
- `traj_opt.exp_traj.penna_pos` - 位置惩罚系数 (double)
- `traj_opt.exp_traj.penna_vel` - 速度惩罚系数 (double)
- `traj_opt.exp_traj.penna_acc` - 加速度惩罚系数 (double)
- `traj_opt.exp_traj.penna_jerk` - 加加速度惩罚系数 (double)
- `traj_opt.exp_traj.penna_attract` - 吸引子惩罚系数 (double)
- `traj_opt.exp_traj.penna_omg` - 角速度惩罚系数 (double)
- `traj_opt.exp_traj.penna_thr` - 推力惩罚系数 (double)

#### 优化参数
- `traj_opt.exp_traj.opt_accuracy` - 优化精度 (double)
- `traj_opt.exp_traj.smooth_eps` - 平滑参数 (double)
- `traj_opt.exp_traj.integral_reso` - 积分分辨率 (int)

#### 算法配置参数
- `traj_opt.exp_traj.pos_constraint_type` - 位置约束类型 (int)
- `traj_opt.exp_traj.block_energy_cost` - 是否禁用能量代价 (bool)

### 2. 备份轨迹优化器 (BackupTrajOpt) 参数

#### 边界约束参数
- `traj_opt.boundary.max_vel` - 最大速度 (double)
- `traj_opt.boundary.max_acc` - 最大加速度 (double)
- `traj_opt.boundary.max_jerk` - 最大加加速度 (double)

#### 惩罚系数参数
- `traj_opt.backup_traj.penna_t` - 时间惩罚系数 (double)
- `traj_opt.backup_traj.penna_ts` - 时间段惩罚系数 (double)
- `traj_opt.backup_traj.penna_pos` - 位置惩罚系数 (double)
- `traj_opt.backup_traj.penna_vel` - 速度惩罚系数 (double)
- `traj_opt.backup_traj.penna_acc` - 加速度惩罚系数 (double)
- `traj_opt.backup_traj.penna_jerk` - 加加速度惩罚系数 (double)
- `traj_opt.backup_traj.penna_attract` - 吸引子惩罚系数 (double)
- `traj_opt.backup_traj.penna_omg` - 角速度惩罚系数 (double)
- `traj_opt.backup_traj.penna_thr` - 推力惩罚系数 (double)
- `traj_opt.backup_traj.penna_max_acc_thr` - 最大推力加速度惩罚系数 (double)
- `traj_opt.backup_traj.penna_min_acc_thr` - 最小推力加速度惩罚系数 (double)

#### 优化参数
- `traj_opt.backup_traj.opt_accuracy` - 优化精度 (double)
- `traj_opt.backup_traj.smooth_eps` - 平滑参数 (double)
- `traj_opt.backup_traj.integral_reso` - 积分分辨率 (int)

#### 算法配置参数
- `traj_opt.backup_traj.uniform_time_en` - 是否启用均匀时间分配 (bool)
- `traj_opt.backup_traj.pos_constraint_type` - 位置约束类型 (int)
- `traj_opt.backup_traj.piece_num` - 轨迹分段数量 (int)
- `traj_opt.backup_traj.block_energy_cost` - 是否禁用能量代价 (bool)

## 使用方法

### 通过命令行动态修改参数

```bash
# 修改显式轨迹优化器参数
ros2 param set /fsm_node traj_opt.exp_traj.penna_pos 2.0e+6

# 修改备份轨迹优化器参数
ros2 param set /fsm_node traj_opt.backup_traj.penna_pos 2.0e+6

# 修改边界约束参数
ros2 param set /fsm_node traj_opt.boundary.max_vel 2.0
```

### 通过rqt_reconfigure界面

1. 启动rqt_reconfigure：
```bash
rqt_reconfigure
```

2. 选择fsm_node节点
3. 在界面中找到对应的参数并进行调整

### 配置文件中的参数

注意：配置文件中的参数（如`click_real_ros2.yaml`）只在节点启动时加载一次，不会动态更新。动态参数修改只影响运行时的参数值。

## 与备份轨迹优化失败的关系

根据您的日志显示备份轨迹优化失败（"Omg or thr or Pos violation"），可以重点关注以下参数：

- `traj_opt.backup_traj.penna_omg` - 角速度惩罚系数
- `traj_opt.backup_traj.penna_thr` - 推力惩罚系数  
- `traj_opt.backup_traj.penna_pos` - 位置惩罚系数
- `traj_opt.boundary.max_vel` - 最大速度
- `traj_opt.boundary.max_acc` - 最大加速度
- `traj_opt.backup_traj.block_energy_cost` - 是否禁用能量代价

适当调整这些参数可以改善备份轨迹优化的成功率。

## 注意事项

1. 动态参数修改是线程安全的，使用互斥锁保护
2. 参数修改后会立即生效，无需重启节点
3. 在优化过程中修改参数可能会影响当前优化结果，但不影响正在进行的优化计算
4. 优化器会在每次优化前使用最新的参数值