
std_msgs/Header header                          # 消息头，包含时间戳和坐标系信息
# the trajectory id, starts from "1".
uint32 trajectory_id                            # 轨迹ID，从1开始

# The type of the trajectory
# use bit to represent the type of the trajectory
# c++ code example:
# To build a type flag: if this trajectory is a POSITION_TRAJ and YAW_TRAJ, then "type = POSITION_TRAJ | YAW_TRAJ"
# To decode this type flag: bool is_position_traj = type & POSITION_TRAJ, bool is_yaw_traj = type & YAW_TRAJ
uint32 type                                     # 轨迹类型标志位
uint32 HEART_BEAT                  =   1        # (1<<0) 心跳信号
uint32 POSITION_TRAJ               =   2        # (1<<1) 位置轨迹
uint32 YAW_TRAJ                    =   4        # (1<<2) 偏航角轨迹
uint32 YAW_COMMAND                 =   8        # (1<<3) 偏航角指令
uint32 EMER_STOP                   =   16       # (1<<4) 紧急停止

# the order of trajectory.
uint32 piece_num_yaw                            # 偏航角轨迹的段数
uint32 piece_num_pos                            # 位置轨迹的段数
uint32 order_pos                                # 位置多项式的阶数
uint32 order_yaw                                # 偏航角多项式的阶数

# the polynomial coecfficients of the trajectory.
# The start system wall time of the yaw trajectory
float64 start_wt_yaw                            # 偏航角轨迹的系统起始时间
float64 yaw                                     # 当前偏航角
float64 yaw_rate                                # 当前偏航角速率
float64[] coef_yaw                              # 偏航角多项式系数
float64[] time_yaw                              # 偏航角轨迹各段的时间长度
# The start system wall time of the pos trajectory
float64 start_wt_pos                            # 位置轨迹的系统起始时间
float64[] coef_pos_x                            # X轴位置多项式系数
float64[] coef_pos_y                            # Y轴位置多项式系数
float64[] coef_pos_z                            # Z轴位置多项式系数
float64[] time_pos                              # 位置轨迹各段的时间长度

string debug_info                               # 调试信息
