
std_msgs/Header header  # 消息头，包含时间戳和坐标系信息
float64[4] u  # 控制输入向量
float64[3] pos  # 位置 [x, y, z]
float64[3] vel  # 速度 [vx, vy, vz]
float64[3] acc  # 加速度 [ax, ay, az]
float64[3] jerk  # 加加速度 [jx, jy, jz]
float64 yaw  # 偏航角
float64 yawdot  # 偏航角速度
float64[4] quat  # 四元数 [w, x, y, z]

uint8 type  # 命令类型
uint8 THRUST_BODYRATE=0  # 集体推力和机体角速率
uint8 THRUST_TORQUE=1    # 集体推力和力矩
uint8 ROTORS_FORCE=2     # 各旋翼力
uint8 DESIRED_POS=3      # 期望状态
uint8 THRUST_QUAT=4      # 期望状态（推力和四元数）
