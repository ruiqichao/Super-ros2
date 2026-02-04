#!/bin/bash

echo "=== backup_traj_en 参数动态调整功能验证 ==="
echo ""

# 构建项目
echo "1. 构建项目..."
cd /home/ruiqichao/ros2_demo/planner/super_ws/super_ros
colcon build --packages-select super_planner

if [ $? -ne 0 ]; then
    echo "❌ 构建失败"
    exit 1
fi

echo "✅ 构建成功"
echo ""

echo "=== 测试步骤 ==="
echo ""
echo "1. 启动系统："
echo "   source install/setup.bash"
echo "   ros2 launch super_planner click_demo.launch.py"
echo ""
echo "2. 观察初始状态："
echo "   - 确认backup_traj_en默认值为false"
echo "   - 观察规划日志中是否显示跳过备份轨迹生成"
echo ""
echo "3. 启用备份轨迹："
echo "   ros2 param set /fsm_node super_planner.backup_traj_en true"
echo ""
echo "4. 触发规划："
echo "   在RVIZ中点击设置目标点"
echo "   或发布目标点：ros2 topic pub /goal geometry_msgs/msg/PoseStamped ..."
echo ""
echo "5. 观察日志变化："
echo "   - 应该看到备份轨迹生成的相关日志"
echo "   - 规划时间可能会增加"
echo ""
echo "6. 禁用备份轨迹："
echo "   ros2 param set /fsm_node super_planner.backup_traj_en false"
echo ""
echo "7. 再次触发规划："
echo "   - 应该看到跳过备份轨迹生成的日志"
echo "   - 规划时间应该减少"
echo ""
echo "=== 预期结果 ==="
echo "✅ backup_traj_en=false时：跳过备份轨迹生成，规划更快"
echo "✅ backup_traj_en=true时：生成备份轨迹，提供更多安全保障"
echo "✅ 参数动态调整应该立即生效"
echo "✅ 系统稳定性不受影响"
echo ""
echo "按 Ctrl+C 停止测试"

# 等待用户中断
sleep infinity