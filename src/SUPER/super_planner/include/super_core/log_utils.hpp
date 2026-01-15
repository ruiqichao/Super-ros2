
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

#ifndef LOG_UTILS_HPP
#define LOG_UTILS_HPP

#include "ros_interface/ros_interface.hpp"

namespace super_planner {
    // 日志时间类型枚举
    enum LogTime {
        EPX_TRAJ_FRONTEND = 0,      // 探索轨迹前端
        EXP_TRAJ_OPT,               // 探索轨迹优化
        GENERATE_EXP_TRAJ,          // 生成探索轨迹
        BACK_TRAJ_FRONTEND,         // 备份轨迹前端
        BACK_TRAJ_OPT,              // 备份轨迹优化
        GENERATE_BACK_TRAJ,         // 生成备份轨迹
        TOTAL_REPLAN,               // 总重规划时间
        VISUALIZATION               // 可视化
    };

    // 日志时间字符串数组
    static vector<string> log_time_str
            {
                    "EPX_TRAJ_FRONTEND",
                    " EXP_TRAJ_OPT",
                    " GENERATE_EXP_TRAJ",
                    " BACK_TRAJ_FRONTEND",
                    " BACK_TRAJ_OPT",
                    " GENERATE_BACK_TRAJ",
                    " TOTAL_REPLAN",
                    " VISUALIZATION"
            };

    /**
     * @brief 单次重规划日志类
     * 用于记录和管理单次重规划过程中的所有数据
     */
    class LogOneReplan {
        // 重规划目标信息
        Vec3f robot_p;              // 机器人位置
        Vec4f robot_q;              // 机器人四元数
        Vec3f goal_p;               // 目标位置
        double goal_yaw;            // 目标偏航角
        double replan_stamp;        // 重规划时间戳
        Vec3f local_start_p;        // 局部起始位置
        
        // 路径搜索相关
        vec_Vec3f reference_path;   // 参考路径
        vec_Vec3f pc_for_sfc;       // 用于安全飞行走廊的点云
        PolytopeVec exp_sfc;        // 探索轨迹的安全飞行走廊

        // 探索轨迹相关
        StatePVAJ exp_init_state, exp_fina_state;   // 初始和最终状态
        VecDf exp_init_t_vec;       // 初始时间向量
        vec_Vec3f exp_init_ps;      // 初始位置点
        Trajectory exp_traj;        // 探索轨迹
        Trajectory exp_yaw_traj;    // 探索偏航轨迹

        // 备份轨迹相关
        double backup_init_ts, ts_max, ts_min;      // 时间参数
        VecDf backup_init_t_vec;    // 初始时间向量
        vec_Vec3f backup_init_ps;   // 初始位置点（最后一点是初始最终位置）
        StatePVAJ backup_init_state, backup_fina_state;  // 初始和最终状态
        Polytope backup_sfc;        // 备份轨迹的安全飞行走廊
        Trajectory backup_traj;     // 备份轨迹
        Trajectory backup_yaw_traj; // 备份偏航轨迹

        // 返回代码
        int ret_code{SUPPER_UNDEFINED};

        // 计算时间统计
        double mapping_t{0.0}, astar_t{0.0}, exp_sfc_t{0.0}, exp_opt_t{0.0}, backup_sfc_t{0.0}, backup_opt_t{0.0},
                total_t{0.0}, viz_t{0.0};

    public:
        /**
         * @brief 打印日志信息
         */
        void print() {
            fmt::print("Goal: {}\n", goal_p.transpose());
            fmt::print("Goal yaw: {}\n", goal_yaw);
        }

        /**
         * @brief 获取返回代码
         */
        int getRetCode() const {
            return ret_code;
        }

        /**
         * @brief 获取机器人位置
         */
        Vec3f getRobotP() const {
            return robot_p;
        }

        /**
         * @brief 获取总计算时间
         */
        double getTotalCompT() const {
            return total_t;
        }

        /**
         * @brief 可视化重规划日志
         * @param viz_ptr 可视化接口指针
         */
        void visualize(ros_interface::RosInterface::Ptr &viz_ptr) {
            viz_ptr->vizReplanLog(exp_traj, backup_traj, exp_yaw_traj, backup_yaw_traj, exp_sfc, backup_sfc, pc_for_sfc,
                                  ret_code);
            viz_ptr->vizFrontendPath(exp_init_ps);
        }

    public:
        /**
         * @brief 序列化函数
         * @param archive 存档对象
         */
        template<class Archive>
        void serialize(Archive &archive) {
            archive(robot_p, robot_q,
                    goal_p, goal_yaw, replan_stamp, local_start_p, reference_path,
                    pc_for_sfc,
                    exp_sfc,
                    exp_init_state, exp_fina_state, exp_init_t_vec, exp_init_ps, exp_traj, exp_yaw_traj,
                    backup_init_ts, ts_max, ts_min, backup_init_t_vec, backup_init_ps, backup_init_state,
                    backup_fina_state,
                    backup_sfc,
                    backup_traj, backup_yaw_traj, ret_code,
                    mapping_t, astar_t, exp_sfc_t, exp_opt_t, backup_sfc_t, backup_opt_t, total_t, viz_t);
        }

        /**
         * @brief 重置日志数据
         */
        void reset() {
            mapping_t = 0.0;
            astar_t = 0.0;
            exp_sfc_t = 0.0;
            exp_opt_t = 0.0;
            backup_sfc_t = 0.0;
            backup_opt_t = 0.0;
            total_t = 0.0;
            viz_t = 0.0;
            ret_code = SUPER_RET_CODE::SUPPER_UNDEFINED;
            exp_traj.clear();
            exp_yaw_traj.clear();
            backup_traj.clear();
            backup_yaw_traj.clear();
            exp_sfc.clear();
            backup_sfc.Reset();
            exp_init_ps.clear();
            backup_init_ps.clear();
            reference_path.clear();
        }

        /**
         * @brief 设置目标信息
         * @param _goal_p 目标位置
         * @param _goal_yaw 目标偏航角
         * @param _robot 机器人状态
         */
        void setGoal(const Vec3f &_goal_p, const double &_goal_yaw,
                     const super_utils::RobotState &_robot) {
            robot_p = _robot.p;
            robot_q = _robot.q.coeffs();
            goal_p = _goal_p;
            goal_yaw = _goal_yaw;
        }

        /**
         * @brief 设置返回代码
         * @param _ret 返回代码
         */
        void setRetCode(const int &_ret) {
            ret_code = _ret;
        }

        /**
         * @brief 设置局部起始位置
         * @param _local_start_p 局部起始位置
         */
        void setLocalStartP(const Vec3f &_local_start_p) {
            local_start_p = _local_start_p;
        }

        /**
         * @brief 设置引导路径
         * @param _path 路径点集合
         */
        void setGuidePath(const vec_Vec3f &_path) {
            reference_path = _path;
        }

        /**
         * @brief 设置探索轨迹条件
         * @param _ts 时间向量
         * @param _ps 位置点集合
         * @param _init_state 初始状态
         * @param _fina_state 最终状态
         * @param _sfcs 安全飞行走廊集合
         */
        void setExpCondition(const VecDf &_ts, const vec_Vec3f &_ps,
                             const StatePVAJ &_init_state, const StatePVAJ &_fina_state,
                             const PolytopeVec &_sfcs) {
            exp_init_t_vec = _ts;
            exp_init_ps = _ps;
            exp_init_state = _init_state;
            exp_fina_state = _fina_state;
            exp_sfc = _sfcs;
        }

        /**
         * @brief 设置备份轨迹条件
         * @param _init_ts 初始时间戳
         * @param _init_t_vec 初始时间向量
         * @param _init_ps 初始位置点集合
         * @param _ts_min 最小时间
         * @param _ts_max 最大时间
         * @param _sfc 安全飞行走廊
         */
        void setBackupCondition(const double &_init_ts, const VecDf &_init_t_vec,
                                const vec_Vec3f &_init_ps,
                                const double &_ts_min,
                                const double &_ts_max,
                                const Polytope &_sfc) {
            ts_max = _ts_max;
            ts_min = _ts_min;
            backup_init_ts = _init_ts;
            backup_init_t_vec = _init_t_vec;
            backup_sfc = _sfc;
            backup_init_ps = _init_ps;
            backup_sfc = _sfc;
        }

        /**
         * @brief 设置探索轨迹
         * @param _traj 轨迹对象
         */
        void setExpTraj(const Trajectory &_traj) {
            exp_traj = _traj;
        }

        /**
         * @brief 设置探索偏航轨迹
         * @param _traj 轨迹对象
         */
        void setExpYawTraj(const Trajectory &_traj) {
            exp_yaw_traj = _traj;
        }

        /**
         * @brief 设置备份轨迹
         * @param _traj 轨迹对象
         */
        void setBackupTraj(const Trajectory &_traj) {
            backup_traj = _traj;
        }

        /**
         * @brief 设置备份偏航轨迹
         * @param _traj 轨迹对象
         */
        void setBackupYawTraj(const Trajectory &_traj) {
            backup_yaw_traj = _traj;
        }

        /**
         * @brief 设置计算时间
         * @param comp_times 计算时间数组
         */
        void setComptT(const vector<double> &comp_times) {
            astar_t = comp_times[LogTime::EPX_TRAJ_FRONTEND];
            exp_opt_t = comp_times[LogTime::EXP_TRAJ_OPT];
            backup_opt_t = comp_times[LogTime::BACK_TRAJ_OPT];
            viz_t = comp_times[LogTime::VISUALIZATION];
            total_t = astar_t + exp_opt_t + backup_opt_t + viz_t;
        }

        /**
         * @brief 设置安全飞行走廊点云
         * @param _pc 点云数据
         */
        void setSfcPc(const vec_Vec3f &_pc) {
            pc_for_sfc = _pc;
        }

        /**
         * @brief 重新规划探索轨迹
         * @param exp_traj_opt 探索轨迹优化器
         * @param traj 输出轨迹
         */
        void replanExpTrajectory(const ExpTrajOpt::Ptr &exp_traj_opt, Trajectory &traj) {
            // 检查必要条件
            if (exp_sfc.size() == 0 || exp_init_ps.empty() || exp_init_t_vec.size() == 0) {
                return;
            }
            fmt::print("Replan exp trajectory==========================\n");
            fmt::print("init_state: {}\n", exp_init_state);
            fmt::print("fina_state: {}\n", exp_fina_state);
            fmt::print("sfcs: {}\n", exp_sfc.size());
            fmt::print("init_ps: \n{}\n", exp_init_ps.size());
            fmt::print("init_ts: {}\n", exp_init_t_vec.transpose());

            traj.clear();
            // 调用优化器生成轨迹
            exp_traj_opt->optimize(exp_init_state,
                                   exp_fina_state,
                                   exp_sfc,
                                   exp_init_ps,
                                   exp_init_t_vec,
                                   traj
            );
        }

        /**
         * @brief 重新规划备份轨迹
         * @param backup_traj_opt 备份轨迹优化器
         * @param traj 输出轨迹
         */
        void replanBackupTrajectory(const BackupTrajOpt::Ptr &backup_traj_opt, Trajectory &traj) {
            // 检查必要条件
            if (exp_traj.empty() || backup_init_ps.size() == 0 || backup_init_t_vec.size() == 0
                || (backup_init_ps.size()!=backup_init_t_vec.size())
            ) {
                fmt::print(fg(fmt::color::indian_red),"Backup traj is empty\n");
                return;
            }
            fmt::print("Online backup trajectory==========================\n");
            fmt::print("ts_min: {}\n", ts_min);
            fmt::print("ts_max: {}\n", ts_max);
            fmt::print("backup_init_ts: {}\n", backup_init_ts);
            fmt::print("backup_init_ps: \n{}\n", backup_init_ps.size());
            fmt::print("backup_init_t_vec: {}\n", backup_init_t_vec.size());

            traj.clear();
            double out_ts;
            // 调用优化器生成备份轨迹
            backup_traj_opt->optimize(exp_traj,
                                      ts_min,
                                      ts_max,
                                      backup_init_ts,
                                      backup_sfc,
                                      backup_init_t_vec,
                                      backup_init_ps,
                                      traj,
                                      out_ts
            );
        }
    };

}

#endif //LOG_UTILS_HPP
