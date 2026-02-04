#!/bin/bash

echo "=== backup_traj_en 参数调试脚本 ==="
echo ""

# 构建项目
echo "1. 重新构建项目..."
cd /home/ruiqichao/ros2_demo/planner/super_ws/super_ros
colcon build --packages-select super_planner

if [ $? -ne 0 ]; then
    echo "❌ 构建失败"
    exit 1
fi

echo "✅ 构建成功"
echo ""

echo "=== 调试步骤 ==="
echo ""
echo "1. 启动系统（使用详细日志级别）："
echo "   source install/setup.bash"
echo "   ros2 launch super_planner click_demo.launch.py --ros-args --log-level info"
echo ""
echo "2. 在另一个终端监控日志："
echo "   ros2 run rqt_console rqt_console"
echo ""
echo "3. 检查初始参数状态："
echo "   ros2 param list /fsm_node | grep backup_traj_en"
echo "   ros2 param get /fsm_node super_planner.backup_traj_en"
echo "   ros2 param get /fsm_node super_planner.print_log"
echo ""
echo "4. 触发一次规划（在RVIZ中点击目标点）"
echo "   观察日志中是否显示："
echo "   - [Config] updateRuntimeParams called"
echo "   - [Config] backup_traj_en updated: false -> false"
echo "   - [FSM] Calling updateRuntimeParams for super_planner parameters"
echo "   - [FSM] updateRuntimeParams completed"
echo "   - [SUPER] PlanFromRest - backup_traj_en=X, print_log=Y"
echo ""
echo "5. 修改参数并再次规划："
echo "   ros2 param set /fsm_node super_planner.backup_traj_en true"
echo "   再次触发规划，观察参数值变化和行为差异"
echo ""
echo "6. 测试print_log参数："
echo "   ros2 param set /fsm_node super_planner.print_log true"
echo "   观察是否能看到更多日志信息"
echo ""
echo "=== 关键检查点 ==="
echo "🔍 参数更新链路：参数声明 → 动态回调 → updateRuntimeParams → 规划使用"
echo "🔍 日志显示条件：print_log=true 且 ROS日志级别为INFO或更低"
echo "🔍 参数生效验证：backup_traj_en值变化时规划行为应该相应改变"
echo ""
echo "按 Ctrl+C 停止测试"

# 等待用户中断
sleep infinity