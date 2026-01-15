#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

def publish_circular_path():
    """
    发布一个圆环轨迹
    """
    rospy.init_node('circular_path_publisher', anonymous=True)
    
    # 创建发布器，发布到 /pct_path 话题
    pub = rospy.Publisher('/pct_path', Path, queue_size=10)
    
    # 等待订阅者连接
    rospy.sleep(0.5)
    
    # 创建 Path 消息
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()
    
    # 圆环轨迹参数设置
    num_points = 48  # 路径点数（360度，每7.5度一个点，更密集）
    radius = 15.0  # 圆环半径（米）
    center_x = 15.0  # 圆心X坐标 - 使得 (0,0,0) 在圆周上
    center_y = 0.0  # 圆心Y坐标 - 使得 (0,0,0) 在圆周上
    z_pos = 1.5  # Z 坐标固定（飞行高度）
    
    # 生成圆环路径点
    for i in range(num_points):
        # 参数化圆：x = center_x + radius * cos(theta), y = center_y + radius * sin(theta)
        theta = 2.0 * math.pi * i / num_points  # 从 0 到 2π
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        z = z_pos
        
        # 创建 PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # 设置四元数（不旋转）
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        path.poses.append(pose)
    
    # 发布路径
    rospy.loginfo("\n" + "="*50)
    rospy.loginfo("发布圆环轨迹")
    rospy.loginfo("="*50)
    rospy.loginfo("路径点数: %d", len(path.poses))
    rospy.loginfo("轨迹类型: 圆环")
    rospy.loginfo("圆环半径: %.2f 米", radius)
    rospy.loginfo("圆心位置: (%.2f, %.2f)", center_x, center_y)
    rospy.loginfo("飞行高度: %.2f 米", z_pos)
    rospy.loginfo("="*50)
    
    # 发布一次即可
    path.header.stamp = rospy.Time.now()
    pub.publish(path)
    rospy.loginfo("\n✓ 圆环轨迹已发布到 /pct_path 话题\n")
    
    # 持续发布轨迹
    rate = rospy.Rate(1)  # 1Hz 发布频率
    rospy.loginfo("正在持续发布轨迹...按 Ctrl+C 停止\n")
    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_circular_path()
    except rospy.ROSInterruptException:
        rospy.loginfo("圆环轨迹发布节点已停止")
