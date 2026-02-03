
/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SUPER_PLANNER_CONFIG_HPP
#define SUPER_PLANNER_CONFIG_HPP

#include <rog_map/rog_map_core/config.hpp>
#include <traj_opt/config.hpp>
#include <utils/header/yaml_loader.hpp>
#include <rclcpp/rclcpp.hpp>

namespace super_planner {
    using namespace traj_opt;
    using std::cout;
    using std::endl;

    /**
     * @class Config
     * @brief SUPER规划器配置类，封装所有规划相关的参数配置
     * @note 此类管理轨迹规划、传感器范围、安全走廊等关键参数
     */
    class Config {
    public:
        /**
         * @enum YawMode
         * @brief 偏航角控制模式枚举
         */
        enum YawMode{
            YAW_TO_VEL = 1,      // 偏航角朝向速度方向
            YAW_TO_GOAL = 2      // 偏航角朝向目标点
        };

        traj_opt::Config exp_traj_cfg, back_traj_cfg;    // 探索轨迹和备份轨迹配置

        // 布尔类型参数
        bool visualization_en{true};           // 可视化功能开关
        bool detailed_log_en{false};          // 详细日志输出开关
        bool backup_traj_en;                  // 备份轨迹功能开关
        bool use_fov_cut, print_log;          // FOV裁剪开关和日志打印开关
        bool goal_vel_en,goal_yaw_en;         // 目标速度和偏航角使能开关
        bool visual_process;                  // 视觉处理开关
        bool frontend_in_known_free;          // 前端在已知自由空间中运行开关
        std::string px4_cmd_topic{"/planning/px4_cmd"};  // PX4命令话题名称

        double resolution;                     // 地图分辨率
        double planning_horizon;               // 规划时域长度
        double receding_dis;                   // 后退距离
        double safe_corridor_line_max_length;  // 安全走廊线段最大长度
        double sensing_horizon;                // 传感器探测范围，用于FOV裁剪

        // Astar 路径搜索参数
        int astar_heu_type;                   // Astar 启发式类型
        bool astar_allow_diag;                // 是否允许对角线移动
        bool astar_debug_visualization_en;    // 调试可视化开关

        // 走廊生成参数
        int obs_skip_num;                      // 障碍物跳过数量
        double corridor_bound_dis, corridor_line_max_length;  // 走廊边界距离和线段最大长度
        double min_overlap_threshold;          // 最小重叠阈值
        double virtual_ground_height, virtual_ceil_height; // 虚拟地面和天花板高度
        double replan_forward_dt;              // 重规划前向时间步长
        double sample_traj_dt;                 // 轨迹采样时间步长
        double robot_r;                        // 机器人半径
        int iris_iter_num;                     // IRIS算法迭代次数

        int mpc_horizon{};                     // MPC预测时域

        double yaw_dot_max;                    // 最大偏航角速度，单位：rad/s
        int yaw_mode = YAW_TO_VEL;            // 偏航角模式：1-朝向速度方向，2-朝向目标点

        rog_map::vec_E<rog_map::Vec3i> seed_line_neighbour;  // 种子线邻域点集合


        Config() = default;
        
        /**
         * @brief 从配置文件构造配置对象
         * @param cfg_path 配置文件路径
         * @note 此构造函数会加载所有规划器相关参数并初始化邻域点集
         */
        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            
            // 加载轨迹优化配置
            exp_traj_cfg = traj_opt::Config(cfg_path, "exp_traj");
            back_traj_cfg = traj_opt::Config(cfg_path, "backup_traj");
            
            // 加载布尔类型参数
            loader.LoadParam("super_planner/print_log", print_log, false);
            loader.LoadParam("super_planner/detailed_log_en", detailed_log_en, false);
            loader.LoadParam("super_planner/visualization_en", visualization_en, false);
            loader.LoadParam("super_planner/backup_traj_en", backup_traj_en, false);
            loader.LoadParam("super_planner/goal_vel_en", goal_vel_en, false);
            loader.LoadParam("super_planner/goal_yaw_en", goal_yaw_en, false);
            loader.LoadParam("super_planner/visual_process", visual_process, false);
            loader.LoadParam("super_planner/use_fov_cut", use_fov_cut, false);
            loader.LoadParam("super_planner/frontend_in_known_free", frontend_in_known_free, false);
            
            // 加载 Astar 相关参数
            loader.LoadParam("astar/heu_type", astar_heu_type, 2);
            loader.LoadParam("astar/allow_diag", astar_allow_diag, true);
            loader.LoadParam("astar/debug_visualization_en", astar_debug_visualization_en, false);
            
            // 加载数值类型参数
            loader.LoadParam("super_planner/safe_corridor_line_max_length", safe_corridor_line_max_length, 3.0);
            loader.LoadParam("super_planner/sensing_horizon", sensing_horizon, 3.0);
            loader.LoadParam("super_planner/obs_skip_num", obs_skip_num, 1);
            loader.LoadParam("super_planner/replan_forward_dt", replan_forward_dt, 0.3);
            loader.LoadParam("super_planner/corridor_bound_dis", corridor_bound_dis, 3.0);
            loader.LoadParam("super_planner/corridor_line_max_length", corridor_line_max_length, 3.0);
            loader.LoadParam("super_planner/min_overlap_threshold", min_overlap_threshold, 0.1);
            loader.LoadParam("super_planner/planning_horizon", planning_horizon, 10.0);
            loader.LoadParam("super_planner/receding_dis", receding_dis, 5.0);
            loader.LoadParam("super_planner/robot_r", robot_r, 0.3);
            loader.LoadParam("super_planner/iris_iter_num", iris_iter_num, 1);
            loader.LoadParam("super_planner/yaw_mode", yaw_mode, 1);
            loader.LoadParam("super_planner/mpc_horizon", mpc_horizon, 1);
            loader.LoadParam("super_planner/yaw_dot_max", yaw_dot_max, 3.14);

            // 加载虚拟高度参数 (通常在 rog_map 中定义)
            loader.LoadParam("rog_map/virtual_ground_height", virtual_ground_height, -0.1);
            loader.LoadParam("rog_map/virtual_ceil_height", virtual_ceil_height, 3.5);

            loader.LoadParam("rog_map/resolution", resolution, 0.01, true);

            // 计算轨迹采样时间步长
            sample_traj_dt = resolution / exp_traj_cfg.max_vel;

            // 初始化种子线邻域点集，构建以机器人为中心的球形邻域
            int step = ceil(robot_r / resolution);
            for (int x = -step; x <= step; x++) {
                for (int y = -step; y <= step; y++) {
                    for (int z = -step; z <= step; z++) {
                        // 只添加球形范围内的点
                        if (x * x + y * y + z * z <= step * step) {
                            seed_line_neighbour.push_back({x, y, z});
                        }
                    }
                }
            }
            // 按距离排序邻域点，便于从近到远搜索
            std::sort(seed_line_neighbour.begin(), seed_line_neighbour.end(),
                      [](const auto& a, const auto& b) {
                          return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
                      });
        }

        /**
         * @brief 更新运行时参数
         * @param node ROS2节点指针，用于获取参数
         * @note 此函数用于在运行时动态更新参数值，支持参数热更新
         */
        void updateRuntimeParams(rclcpp::Node::SharedPtr node) {
            // 更新super_planner布尔类型参数
            node->get_parameter_or("super_planner.backup_traj_en", backup_traj_en, backup_traj_en);
            node->get_parameter_or("super_planner.use_fov_cut", use_fov_cut, use_fov_cut);
            node->get_parameter_or("super_planner.print_log", print_log, print_log);
            node->get_parameter_or("super_planner.goal_vel_en", goal_vel_en, goal_vel_en);
            node->get_parameter_or("super_planner.goal_yaw_en", goal_yaw_en, goal_yaw_en);
            node->get_parameter_or("super_planner.visual_process", visual_process, visual_process);
            node->get_parameter_or("super_planner.frontend_in_known_free", frontend_in_known_free, frontend_in_known_free);
            
            // 更新 Astar 参数
            node->get_parameter_or("astar.heu_type", astar_heu_type, astar_heu_type);
            node->get_parameter_or("astar.allow_diag", astar_allow_diag, astar_allow_diag);
            node->get_parameter_or("astar.debug_visualization_en", astar_debug_visualization_en, astar_debug_visualization_en);
            
            // 更新虚拟高度参数 (从 rog_map 命名空间获取)
            node->get_parameter_or("rog_map.virtual_ground_height", virtual_ground_height, virtual_ground_height);
            node->get_parameter_or("rog_map.virtual_ceil_height", virtual_ceil_height, virtual_ceil_height);
            
            // 更新 super_planner 数值类型参数
            node->get_parameter_or("super_planner.safe_corridor_line_max_length", safe_corridor_line_max_length, safe_corridor_line_max_length);
            node->get_parameter_or("super_planner.sensing_horizon", sensing_horizon, sensing_horizon);
            node->get_parameter_or("super_planner.obs_skip_num", obs_skip_num, obs_skip_num);
            node->get_parameter_or("super_planner.replan_forward_dt", replan_forward_dt, replan_forward_dt);
            node->get_parameter_or("super_planner.corridor_bound_dis", corridor_bound_dis, corridor_bound_dis);
            node->get_parameter_or("super_planner.corridor_line_max_length", corridor_line_max_length, corridor_line_max_length);
            node->get_parameter_or("super_planner.min_overlap_threshold", min_overlap_threshold, min_overlap_threshold);
            node->get_parameter_or("super_planner.planning_horizon", planning_horizon, planning_horizon);
            node->get_parameter_or("super_planner.receding_dis", receding_dis, receding_dis);
            node->get_parameter_or("super_planner.robot_r", robot_r, robot_r);
            node->get_parameter_or("super_planner.iris_iter_num", iris_iter_num, iris_iter_num);
            node->get_parameter_or("super_planner.yaw_mode", yaw_mode, yaw_mode);
            node->get_parameter_or("super_planner.mpc_horizon", mpc_horizon, mpc_horizon);
            node->get_parameter_or("super_planner.yaw_dot_max", yaw_dot_max, yaw_dot_max);
            
            // 更新轨迹优化边界约束参数
            node->get_parameter_or("traj_opt.boundary.max_vel", exp_traj_cfg.max_vel, exp_traj_cfg.max_vel);
            node->get_parameter_or("traj_opt.boundary.max_acc", exp_traj_cfg.max_acc, exp_traj_cfg.max_acc);
            node->get_parameter_or("traj_opt.boundary.max_jerk", exp_traj_cfg.max_jerk, exp_traj_cfg.max_jerk);
            
            // 更新探索轨迹优化算法配置参数
            node->get_parameter_or("traj_opt.exp_traj.pos_constraint_type", exp_traj_cfg.pos_constraint_type, exp_traj_cfg.pos_constraint_type);
            node->get_parameter_or("traj_opt.exp_traj.block_energy_cost", exp_traj_cfg.block_energy_cost, exp_traj_cfg.block_energy_cost);
            
            // 更新探索轨迹优化惩罚项参数
            node->get_parameter_or("traj_opt.exp_traj.penna_t", exp_traj_cfg.penna_t, exp_traj_cfg.penna_t);
            node->get_parameter_or("traj_opt.exp_traj.penna_pos", exp_traj_cfg.penna_pos, exp_traj_cfg.penna_pos);
            node->get_parameter_or("traj_opt.exp_traj.penna_vel", exp_traj_cfg.penna_vel, exp_traj_cfg.penna_vel);
            node->get_parameter_or("traj_opt.exp_traj.penna_acc", exp_traj_cfg.penna_acc, exp_traj_cfg.penna_acc);
            node->get_parameter_or("traj_opt.exp_traj.penna_jerk", exp_traj_cfg.penna_jerk, exp_traj_cfg.penna_jerk);
            node->get_parameter_or("traj_opt.exp_traj.penna_attract", exp_traj_cfg.penna_attract, exp_traj_cfg.penna_attract);
            node->get_parameter_or("traj_opt.exp_traj.penna_omg", exp_traj_cfg.penna_omg, exp_traj_cfg.penna_omg);
            node->get_parameter_or("traj_opt.exp_traj.penna_thr", exp_traj_cfg.penna_thr, exp_traj_cfg.penna_thr);
            node->get_parameter_or("traj_opt.exp_traj.opt_accuracy", exp_traj_cfg.opt_accuracy, exp_traj_cfg.opt_accuracy);
            node->get_parameter_or("traj_opt.exp_traj.smooth_eps", exp_traj_cfg.smooth_eps, exp_traj_cfg.smooth_eps);
            
            // 更新备份轨迹优化算法配置参数
            node->get_parameter_or("traj_opt.backup_traj.uniform_time_en", back_traj_cfg.uniform_time_en, back_traj_cfg.uniform_time_en);
            node->get_parameter_or("traj_opt.backup_traj.pos_constraint_type", back_traj_cfg.pos_constraint_type, back_traj_cfg.pos_constraint_type);
            node->get_parameter_or("traj_opt.backup_traj.piece_num", back_traj_cfg.piece_num, back_traj_cfg.piece_num);
            node->get_parameter_or("traj_opt.backup_traj.block_energy_cost", back_traj_cfg.block_energy_cost, back_traj_cfg.block_energy_cost);
            
            // 更新备份轨迹优化惩罚项参数
            node->get_parameter_or("traj_opt.backup_traj.penna_t", back_traj_cfg.penna_t, back_traj_cfg.penna_t);
            node->get_parameter_or("traj_opt.backup_traj.penna_ts", back_traj_cfg.penna_ts, back_traj_cfg.penna_ts);
            node->get_parameter_or("traj_opt.backup_traj.penna_pos", back_traj_cfg.penna_pos, back_traj_cfg.penna_pos);
            node->get_parameter_or("traj_opt.backup_traj.penna_vel", back_traj_cfg.penna_vel, back_traj_cfg.penna_vel);
            node->get_parameter_or("traj_opt.backup_traj.penna_acc", back_traj_cfg.penna_acc, back_traj_cfg.penna_acc);
            node->get_parameter_or("traj_opt.backup_traj.penna_jerk", back_traj_cfg.penna_jerk, back_traj_cfg.penna_jerk);
            node->get_parameter_or("traj_opt.backup_traj.penna_attract", back_traj_cfg.penna_attract, back_traj_cfg.penna_attract);
            node->get_parameter_or("traj_opt.backup_traj.penna_omg", back_traj_cfg.penna_omg, back_traj_cfg.penna_omg);
            node->get_parameter_or("traj_opt.backup_traj.penna_thr", back_traj_cfg.penna_thr, back_traj_cfg.penna_thr);
            node->get_parameter_or("traj_opt.backup_traj.penna_max_acc_thr", back_traj_cfg.max_acc_thr, back_traj_cfg.max_acc_thr);
            node->get_parameter_or("traj_opt.backup_traj.penna_min_acc_thr", back_traj_cfg.min_acc_thr, back_traj_cfg.min_acc_thr);
            node->get_parameter_or("traj_opt.backup_traj.opt_accuracy", back_traj_cfg.opt_accuracy, back_traj_cfg.opt_accuracy);
            node->get_parameter_or("traj_opt.backup_traj.smooth_eps", back_traj_cfg.smooth_eps, back_traj_cfg.smooth_eps);
            node->get_parameter_or("traj_opt.backup_traj.integral_reso", back_traj_cfg.integral_reso, back_traj_cfg.integral_reso);
            
            // 重新计算轨迹采样时间步长
            sample_traj_dt = resolution / exp_traj_cfg.max_vel;
        }


    };
}

#endif
