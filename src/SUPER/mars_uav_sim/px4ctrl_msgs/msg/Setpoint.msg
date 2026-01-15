
# 消息头，包含时间戳和坐标系信息
std_msgs/Header header         

# 状态设定点：[x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
# x, y, z: 位置 (米)
# vx, vy, vz: 速度 (米/秒)
# qw, qx, qy, qz: 姿态四元数
# wx, wy, wz: 角速度 (弧度/秒)
float64[13] x

# 命令设定点
px4ctrl_msgs/Command cmd
