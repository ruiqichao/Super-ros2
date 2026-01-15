# SUPER项目开发与调试指南

## 1. 开发环境设置

### 1.1 依赖安装
```bash
# ROS2 Jazzy安装
sudo apt update
sudo apt install ros-jazzy-desktop

# 额外依赖
sudo apt install libeigen3-dev libsuitesparse-dev
sudo apt install libyaml-cpp-dev libpcl-all-dev
```

### 1.2 工作空间设置
```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 编译项目
colcon build --packages-select super_planner rog_map mars_quadrotor_msgs px4ctrl_msgs

# 源入环境
source install/setup.bash
```

## 2. 编译系统

### 2.1 CMakeLists.txt 结构
```
super_planner/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── super_planner/
├── src/
│   └── super_planner/
├── config/
│   └── *.yaml
└── launch/
    └── *.py
```

### 2.2 编译选项
- `RELEASE=ON`: 启用优化编译
- `CMAKE_BUILD_TYPE=Release`: Release模式编译
- 并行编译: `colcon build --parallel-workers 4`

## 3. 代码结构

### 3.1 核心模块
```
include/
├── fsm/              # 有限状态机
├── path_search/      # 路径搜索算法
├── traj_opt/         # 轨迹优化
├── super_core/       # 核心规划逻辑
├── ros_interface/    # ROS接口
└── utils/            # 工具函数
```

### 3.2 关键类说明
- `SuperPlanner`: 主规划器类
- `FsmRos2`: ROS2状态机接口
- `ExpTrajOpt`: 探索轨迹优化器
- `BackupTrajOpt`: 备份轨迹优化器
- `Astar`: A*路径搜索器

## 4. 调试方法

### 4.1 日志系统
```cpp
// 使用ROS2日志宏
RCLCPP_INFO(node->get_logger(), "Planning successful");
RCLCPP_WARN(node->get_logger(), "Replanning overdue");
RCLCPP_ERROR(node->get_logger(), "Planning failed");
```

### 4.2 调试开关
在配置文件中启用调试模式：
```yaml
super_planner:
  print_log: true           # 打印详细日志
  visual_process: true      # 启用过程可视化
  debug_visualization_en: true  # 启用调试可视化
```

### 4.3 性能分析
```cpp
// 使用计时器分析性能
TimeConsuming timer("FunctionName");
// ... 代码执行 ...
double elapsed = timer.stop();
```

## 5. 测试策略

### 5.1 单元测试
```cpp
// 测试轨迹优化
TEST(TrajOptTest, OptimizeTrajectory) {
    // 创建测试场景
    // 验证优化结果
    ASSERT_TRUE(result.success);
}
```

### 5.2 集成测试
- 仿真环境测试
- 边界条件测试
- 异常处理测试

### 5.3 回归测试
- 保存测试用例
- 自动化测试流程
- 性能基线比较

## 6. 常见问题调试

### 6.1 规划失败
**现象**: 规划器返回FAILED状态
**检查项**:
- 里程计数据是否正常
- 地图数据是否更新
- 起始点是否在自由空间
- 目标点是否可达

**调试命令**:
```bash
# 检查话题数据
ros2 topic echo /your_odom_topic
ros2 topic echo /your_map_topic
```

### 6.2 优化失败
**现象**: 轨迹优化不收敛
**检查项**:
- 约束条件是否过于严格
- 初始值是否合理
- 参数配置是否合适

### 6.3 性能问题
**现象**: 规划时间过长
**优化方向**:
- 减少搜索空间
- 调整优化参数
- 使用更高效的算法

## 7. 调试工具

### 7.1 ROS2工具
```bash
# 节点信息
ros2 node info /fsm_node

# 话题信息
ros2 topic info /planning/pos_cmd

# 服务调用
ros2 service call /reset_map rog_map_interfaces/srv/ResetMap
```

### 7.2 可视化工具
- RViz2: 实时轨迹和地图显示
- rqt_plot: 参数变化曲线
- rqt_graph: 节点图

### 7.3 性能分析工具
- `htop`: CPU/内存使用监控
- `ros2 bag`: 数据录制回放
- 自定义性能监控节点

## 8. 代码规范

### 8.1 命名规范
- 类名: `CamelCase` (如 `SuperPlanner`)
- 函数名: `camelCase` (如 `planFromRest`)
- 变量名: `snake_case` (如 `robot_state`)
- 常量名: `UPPER_SNAKE_CASE` (如 `MAX_ITERATIONS`)

### 8.2 代码组织
- 头文件保护: `#pragma once`
- 命名空间: 按模块组织
- 类设计: 单一职责原则
- 函数长度: 保持简短，一般不超过50行

### 8.3 注释规范
```cpp
/**
 * @brief 函数功能简要说明
 * @param[in] param1 输入参数说明
 * @param[out] param2 输出参数说明
 * @return 返回值说明
 * @note 注意事项
 */
ReturnType functionName(Type param1, Type& param2);
```

## 9. 版本控制

### 9.1 Git工作流
- 功能分支开发
- 代码审查流程
- 版本标签管理

### 9.2 提交规范
```
feat: 新功能添加
fix: 问题修复
docs: 文档更新
style: 代码格式调整
refactor: 代码重构
test: 测试代码
chore: 其他杂项
```

## 10. 部署指南

### 10.1 硬件要求
- CPU: 多核处理器，推荐8核以上
- 内存: 16GB RAM以上
- GPU: 可选，用于加速计算

### 10.2 部署配置
- 参数文件备份
- 日志路径配置
- 实时性参数调整

### 10.3 监控运维
- 系统健康检查
- 性能指标监控
- 异常自动恢复