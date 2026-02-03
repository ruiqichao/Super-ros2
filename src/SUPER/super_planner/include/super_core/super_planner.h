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

#pragma once

#include <iostream>
#include <fstream>
#include "Eigen/Eigen"


#include <super_core/config.hpp>
#include <ros_interface/ros_interface.hpp>
#include <data_structure/base/trajectory.h>

#include <data_structure/base/polytope.h>


#include "traj_opt/exp_traj_optimizer_s4.h"
#include "traj_opt/backup_traj_optimizer_s4.h"
#include "path_search/astar.h"
#include "rog_map/rog_map.h"
#include "super_core/corridor_generator.h"
#include "super_core/fov_checker.h"

#include "traj_opt/yaw_traj_opt.h"
#include "super_core/super_ret_code.hpp"
#include "utils/header/fmt_eigen.hpp"
#include "path_search/astar_config.h"
#include "super_core/corridor_generator_config.h"

#include <super_core/log_utils.hpp>
#include <data_structure/exp_traj.h>
#include <data_structure/cmd_traj.h>
#include <data_structure/backup_traj.h>


namespace super_planner {
    using namespace color_text;
    using namespace geometry_utils;

    class SuperPlanner {
        LogOneReplan latest_replan;
        super_planner::Config cfg_;
        rog_map::ROGMapROS::Ptr map_ptr_;
        CorridorGenerator::Ptr cg_ptr_;
        path_search::Astar::Ptr astar_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;
        Vec3f shifted_sfc_start_pt_;

        traj_opt::ExpTrajOpt::Ptr exp_traj_opt_;
        traj_opt::BackupTrajOpt::Ptr back_traj_opt_;
        traj_opt::YawTrajOpt::Ptr yaw_traj_opt_;

        CIRI::Ptr ciri_;

        super_utils::RobotState robot_state_;

        std::mutex drone_state_mutex_;
        std::mutex replan_lock_;

        Vec3f local_start_p_;

        bool robot_on_backup_traj_{false};
        // use negative value to indicate the traj is not available
        double on_backup_start_WT{-1}, on_backup_end_WT{-1};

        double planner_process_start_WT_;

        struct GoalInfo {
            Vec3f goal_p{0, 0, 0};
            double goal_yaw{0};
            bool new_goal{true};
            bool goal_valid{true};
        } gi_;

        FOVChecker::Ptr fov_checker_;

        CmdTraj cmd_traj_info_;
        ExpTraj last_exp_traj_info_;

        vector<double> time_consuming_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit SuperPlanner(const std::string &cfg_path,
                              const ros_interface::RosInterface::Ptr &ros_ptr,
                              const rog_map::ROGMapROS::Ptr &map_ptr);

        ~SuperPlanner() = default;

        void lockCommittedTraj() {
            cmd_traj_info_.lock();
        }

        void unlockCommittedTraj() {
            cmd_traj_info_.unlock();
        }

        bool goalValid() const {
            return gi_.goal_valid;
        }

        typedef std::shared_ptr<SuperPlanner> Ptr;

        void getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish);

        Trajectory getCommittedPositionTrajectory();

        Trajectory getCommittedYawTrajectory();

        void getOneCommandFromTraj(StatePVAJ &pvaj,
                                   double &yaw,
                                   double &yaw_dot,
                                   bool &on_backup_traj,
                                   bool &traj_finish);

        void getModuleTimeConsuming(vector<double> &time);

        /* Tow type of replan strategy */
        RET_CODE PlanFromRest(const Vec3f &goal_p,
                              const double &goal_yaw,
                              const bool &new_goal);

        RET_CODE
        ReplanOnce(const Vec3f &goal_p,
                   const double &goal_yaw,
                   const bool &new_goal);

    private:
        RET_CODE generateExpTraj(ExpTraj &last_exp_traj_info,
                                 ExpTraj &out_exp_traj_info);

        /* For Backup traj generation */
        RET_CODE generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info);

        int getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt);

        bool PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                        const double &searching_horizon,
                        vec_Vec3f &path);


    public:
        void getRobotState(rog_map::RobotState &out);

        bool isEasyGoal(const Vec3f &goal_position);

        rog_map::ROGMapROS::Ptr &getMap() {
            return map_ptr_;
        }

        /**
         * @brief 获取规划器配置对象
         * @return Config对象的常量引用
         */
        const super_planner::Config& getConfig() const {
            return cfg_;
        }

        /**
         * @brief 获取轨迹优化器的动态参数配置
         * @return ExpTrajOpt的动态参数配置引用
         * @note 用于外部访问和修改动态参数
         */
        traj_opt::SuperTrajConfig& getExpTrajOptDynamicConfig() {
            return exp_traj_opt_->getDynamicConfig();
        }

        /**
         * @brief 获取备份轨迹优化器的动态参数配置
         * @return BackupTrajOpt的动态参数配置引用
         * @note 用于外部访问和修改备份轨迹优化器的动态参数
         */
        traj_opt::BackupTrajConfig& getBackupTrajOptDynamicConfig() {
            return back_traj_opt_->getDynamicConfig();
        }

        /**
         * @brief 获取偏航轨迹优化器的动态参数配置
         * @return YawTrajOpt的动态参数配置引用
         * @note 用于外部访问和修改偏航轨迹优化器的动态参数
         */
        traj_opt::YawTrajConfig& getYawTrajOptDynamicConfig() {
            return yaw_traj_opt_->getDynamicConfig();
        }

        /**
         * @brief 获取Astar路径搜索器的动态参数配置
         * @return Astar的动态参数配置引用
         * @note 用于外部访问和修改Astar的动态参数
         */
        path_search::AstarConfig& getAstarDynamicConfig() {
            return astar_ptr_->getDynamicConfig();
        }

        /**
         * @brief 获取走廊生成器的动态参数配置
         * @return CorridorGenerator的动态参数配置引用
         * @note 用于外部访问和修改走廊生成器的动态参数
         */
        super_planner::CorridorGenConfig& getCorridorGenDynamicConfig() {
            return cg_ptr_->getDynamicConfig();
        }

        /**
         * @brief 获取探索轨迹优化器指针
         * @return ExpTrajOpt的指针
         * @note 用于直接访问优化器对象
         */
        traj_opt::ExpTrajOpt::Ptr getExpTrajOpt() {
            return exp_traj_opt_;
        }

        /**
         * @brief 获取备份轨迹优化器指针
         * @return BackupTrajOpt的指针
         * @note 用于直接访问优化器对象
         */
        traj_opt::BackupTrajOpt::Ptr getBackupTrajOpt() {
            return back_traj_opt_;
        }

        /**
         * @brief 获取偏航轨迹优化器指针
         * @return YawTrajOpt的指针
         * @note 用于直接访问优化器对象
         */
        traj_opt::YawTrajOpt::Ptr getYawTrajOpt() {
            return yaw_traj_opt_;
        }

        /**
         * @brief 获取Astar路径搜索器指针
         * @return Astar的指针
         * @note 用于直接访问Astar对象
         */
        path_search::Astar::Ptr getAstar() {
            return astar_ptr_;
        }

        /**
         * @brief 获取走廊生成器指针
         * @return CorridorGenerator的指针
         * @note 用于直接访问走廊生成器对象
         */
        CorridorGenerator::Ptr getCorridorGenerator() {
            return cg_ptr_;
        }

        double ft{0}, bt{0};
        int ft_cnt{0}, bt_cnt{0};

        double getFrontendTime() {
            if (ft_cnt == 0) return -1;
            double ave_t = ft / ft_cnt;
            ft = 0;
            ft_cnt = 0;
            return ave_t;
        }

        double getBackendTime() {
            if (bt_cnt == 0) return -1;
            double ave_t = bt / bt_cnt;
            bt = 0;
            bt_cnt = 0;
            return ave_t;
        }

        void updateROGMap(const rog_map::PointCloud &cloud, const super_utils::Pose &pose) const {
            map_ptr_->updateMap(cloud, pose);
        }

        LogOneReplan getLatestReplanLog() {
            latest_replan.setSfcPc(cg_ptr_->getLatestCloud());
            latest_replan.setComptT(time_consuming_);
            return latest_replan;
        }

        /**
         * @brief 更新运行时参数
         * @param node ROS2节点指针，用于获取参数
         * @note 此函数用于在运行时动态更新参数值
         */
        void updateRuntimeParams(rclcpp::Node::SharedPtr node) {
            // 从ROS2参数服务器更新cfg_中的参数
            cfg_.updateRuntimeParams(node);
            
            // 同时更新ExpTrajOpt中的动态参数（从cfg_同步到优化器）
            auto& exp_dyn_cfg = exp_traj_opt_->getDynamicConfig();
            {
                std::lock_guard<std::mutex> l(exp_dyn_cfg.getConfigMutex());
                exp_dyn_cfg.max_vel = cfg_.exp_traj_cfg.max_vel;
                exp_dyn_cfg.max_acc = cfg_.exp_traj_cfg.max_acc;
                exp_dyn_cfg.max_jerk = cfg_.exp_traj_cfg.max_jerk;
                exp_dyn_cfg.penna_t = cfg_.exp_traj_cfg.penna_t;
                exp_dyn_cfg.penna_pos = cfg_.exp_traj_cfg.penna_pos;
                exp_dyn_cfg.penna_vel = cfg_.exp_traj_cfg.penna_vel;
                exp_dyn_cfg.penna_acc = cfg_.exp_traj_cfg.penna_acc;
                exp_dyn_cfg.penna_jerk = cfg_.exp_traj_cfg.penna_jerk;
                exp_dyn_cfg.penna_attract = cfg_.exp_traj_cfg.penna_attract;
                exp_dyn_cfg.penna_omg = cfg_.exp_traj_cfg.penna_omg;
                exp_dyn_cfg.penna_thr = cfg_.exp_traj_cfg.penna_thr;
                exp_dyn_cfg.opt_accuracy = cfg_.exp_traj_cfg.opt_accuracy;
                exp_dyn_cfg.smooth_eps = cfg_.exp_traj_cfg.smooth_eps;
                exp_dyn_cfg.integral_reso = cfg_.exp_traj_cfg.integral_reso;
                // 同步算法配置参数
                exp_dyn_cfg.pos_constraint_type = cfg_.exp_traj_cfg.pos_constraint_type;
                exp_dyn_cfg.block_energy_cost = cfg_.exp_traj_cfg.block_energy_cost;
            }
            
            // 同时更新BackupTrajOpt中的动态参数（从cfg_同步到优化器）
            auto& back_dyn_cfg = back_traj_opt_->getDynamicConfig();
            {
                std::lock_guard<std::mutex> l(back_dyn_cfg.getConfigMutex());
                back_dyn_cfg.max_vel = cfg_.back_traj_cfg.max_vel;
                back_dyn_cfg.max_acc = cfg_.back_traj_cfg.max_acc;
                back_dyn_cfg.max_jerk = cfg_.back_traj_cfg.max_jerk;
                back_dyn_cfg.penna_t = cfg_.back_traj_cfg.penna_t;
                back_dyn_cfg.penna_ts = cfg_.back_traj_cfg.penna_ts;
                back_dyn_cfg.penna_pos = cfg_.back_traj_cfg.penna_pos;
                back_dyn_cfg.penna_vel = cfg_.back_traj_cfg.penna_vel;
                back_dyn_cfg.penna_acc = cfg_.back_traj_cfg.penna_acc;
                back_dyn_cfg.penna_jerk = cfg_.back_traj_cfg.penna_jerk;
                back_dyn_cfg.penna_attract = cfg_.back_traj_cfg.penna_attract;
                back_dyn_cfg.penna_omg = cfg_.back_traj_cfg.penna_omg;
                back_dyn_cfg.penna_thr = cfg_.back_traj_cfg.penna_thr;
                back_dyn_cfg.penna_max_acc_thr = cfg_.back_traj_cfg.max_acc_thr;
                back_dyn_cfg.penna_min_acc_thr = cfg_.back_traj_cfg.min_acc_thr;
                back_dyn_cfg.opt_accuracy = cfg_.back_traj_cfg.opt_accuracy;
                back_dyn_cfg.smooth_eps = cfg_.back_traj_cfg.smooth_eps;
                back_dyn_cfg.integral_reso = cfg_.back_traj_cfg.integral_reso;
                // 同步算法配置参数
                back_dyn_cfg.uniform_time_en = cfg_.back_traj_cfg.uniform_time_en;
                back_dyn_cfg.pos_constraint_type = cfg_.back_traj_cfg.pos_constraint_type;
                back_dyn_cfg.piece_num = cfg_.back_traj_cfg.piece_num;
                back_dyn_cfg.block_energy_cost = cfg_.back_traj_cfg.block_energy_cost;
            }
            
            // 同时更新YawTrajOpt中的动态参数（从cfg_同步到优化器）
            if (yaw_traj_opt_) {
                auto& yaw_dyn_cfg = yaw_traj_opt_->getDynamicConfig();
                {
                    std::lock_guard<std::mutex> l(yaw_dyn_cfg.getConfigMutex());
                    yaw_dyn_cfg.yaw_dot_max = cfg_.yaw_dot_max;
                }
            }
            
            // 同时更新Astar中的动态参数（从cfg_同步到搜索器）
            if (astar_ptr_) {
                auto& astar_dyn_cfg = astar_ptr_->getDynamicConfig();
                {
                    std::lock_guard<std::mutex> l(astar_dyn_cfg.getConfigMutex());
                    astar_dyn_cfg.heu_type = cfg_.astar_heu_type;
                    astar_dyn_cfg.allow_diag = cfg_.astar_allow_diag;
                    astar_dyn_cfg.debug_visualization_en = cfg_.astar_debug_visualization_en;
                }
            }
            
            // 同时更新CorridorGenerator中的动态参数（从cfg_同步到生成器）
            if (cg_ptr_) {
                auto& cg_dyn_cfg = cg_ptr_->getDynamicConfig();
                {
                    std::lock_guard<std::mutex> l(cg_dyn_cfg.getConfigMutex());
                    cg_dyn_cfg.bound_dis = cfg_.corridor_bound_dis;
                    cg_dyn_cfg.seed_line_max_length = cfg_.corridor_line_max_length;
                    cg_dyn_cfg.min_overlap_threshold = cfg_.min_overlap_threshold;
                    cg_dyn_cfg.robot_r = cfg_.robot_r;
                    cg_dyn_cfg.box_search_skip_num = cfg_.obs_skip_num;
                    cg_dyn_cfg.iris_iter_num = cfg_.iris_iter_num;
                    cg_dyn_cfg.virtual_ground_height = cfg_.virtual_ground_height;
                    cg_dyn_cfg.virtual_ceil_height = cfg_.virtual_ceil_height;
                }
            }
        }
    };
}
