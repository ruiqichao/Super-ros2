
# 消息头，包含时间戳和坐标系信息
std_msgs/Header header         

# 状态向量，包含位置、速度、姿态四元数和角速度
# [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
float64[13] x  

# 四个旋翼的转速或推力
# [rotor1, rotor2, rotor3, rotor4]
float64[4] rotors
