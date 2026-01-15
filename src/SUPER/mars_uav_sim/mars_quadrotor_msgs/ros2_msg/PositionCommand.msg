
std_msgs/Header header                          # 消息头，包含时间戳和坐标系信息
geometry_msgs/Point position                    # 目标位置（三维坐标）
geometry_msgs/Vector3 velocity                  # 目标速度（三维向量）
geometry_msgs/Vector3 acceleration              # 目标加速度（三维向量）
geometry_msgs/Vector3 jerk                      # 目标加加速度/急动度（三维向量）
geometry_msgs/Vector3 angular_velocity          # 目标角速度（三维向量）
geometry_msgs/Vector3 attitude                  # 目标姿态（三维向量，通常表示欧拉角）
geometry_msgs/Vector3 thrust                    # 推力向量（三维向量）
float64 yaw                                     # 偏航角（绕z轴的旋转角度）
float64 yaw_dot                                 # 偏航角速度（偏航角的变化率）
float64 vel_norm                                # 速度的模长/范数
float64 acc_norm                                # 加速度的模长/范数


float64[3] kx                                   # 位置控制增益（3个参数）
float64[3] kv                                   # 速度控制增益（3个参数）
uint32 trajectory_id                            # 轨迹ID，用于标识特定的轨迹
uint8 TRAJECTORY_STATUS_EMPTY = 0               # 轨迹为空
uint8 TRAJECTORY_STATUS_EMER = 2                # 紧急状态
uint8 TRAJECTORY_STATUS_READY = 1               # 轨迹就绪
uint8 TRAJECTORY_STATUS_COMPLETED = 3           # 轨迹完成
uint8 TRAJECTROY_STATUS_ABORT = 4               # 轨迹中止
uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5       # 非法起始点
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6       # 非法终止点
uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7          # 轨迹不可行
uint32 ACTION_STOP                 =   8        # 停止动作
# Its ID number will start from 1, allowing you comparing it with 0.
uint8 trajectory_flag                           # 轨迹标志位，ID从1开始，可与0进行比较
