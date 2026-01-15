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

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <mutex>

namespace traj_opt {

    /**
     * @brief BackupTrajOpt备份轨迹优化器配置类，支持动态参数调整
     * @note 此类封装了备份轨迹优化器的可动态调整参数
     */
    class BackupTrajConfig {
    private:
        std::mutex config_mutex_;  // 用于线程安全的互斥锁

    public:
        // 边界约束参数
        double max_vel{1.0};              // 最大速度，单位：m/s
        double max_acc{5.0};              // 最大加速度，单位：m/s²
        double max_jerk{120.0};           // 最大加加速度，单位：m/s³

        // 惩罚系数参数
        double penna_t{12000.0};          // 时间惩罚系数
        double penna_ts{1.0e+5};          // 时间段惩罚系数
        double penna_pos{1.0e+6};         // 位置惩罚系数
        double penna_vel{1.0e+5};         // 速度惩罚系数
        double penna_acc{1.0e+5};         // 加速度惩罚系数
        double penna_jerk{-1.0e+5};       // 加加速度惩罚系数
        double penna_attract{1.0e+2};     // 吸引子惩罚系数
        double penna_omg{1.0e+5};         // 角速度惩罚系数
        double penna_thr{1.0e+4};         // 推力惩罚系数
        double penna_max_acc_thr{1.0e+4}; // 最大推力加速度惩罚系数（此参数在traj_opt::Config中没有对应字段，需要特别处理）
        double penna_min_acc_thr{1.0e+4}; // 最小推力加速度惩罚系数（此参数在traj_opt::Config中没有对应字段，需要特别处理）

        // 优化参数
        double opt_accuracy{1.0e-4};      // 优化精度
        double smooth_eps{0.05};          // 平滑参数
        int integral_reso{10};            // 积分分辨率
        
        // 算法配置参数
        bool uniform_time_en{true};       // 是否启用均匀时间分配
        int pos_constraint_type{1};        // 位置约束类型: 1-路径点约束, 2-走廊约束
        int piece_num{2};                  // 轨迹分段数量
        bool block_energy_cost{true};       // 是否禁用能量代价

        /**
         * @brief 动态参数回调函数
         * @param parameters 待更新的参数列表
         * @return 参数设置结果
         * @note 此函数在参数更新时被调用，处理动态参数变化
         */
        rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
            auto result = rcl_interfaces::msg::SetParametersResult();
            std::lock_guard<std::mutex> l(config_mutex_); // 确保线程安全

            for (auto parameter : parameters) {
                const auto & type = parameter.get_type();
                const auto & name = parameter.get_name();

                if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
                    // 边界约束参数
                    if (name == "traj_opt.boundary.max_vel") {
                        max_vel = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: max_vel = %f", max_vel);
                    } else if (name == "traj_opt.boundary.max_acc") {
                        max_acc = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: max_acc = %f", max_acc);
                    } else if (name == "traj_opt.boundary.max_jerk") {
                        max_jerk = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: max_jerk = %f", max_jerk);
                    }
                    // 惩罚系数参数
                    else if (name == "traj_opt.backup_traj.penna_t") {
                        penna_t = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_t = %f", penna_t);
                    } else if (name == "traj_opt.backup_traj.penna_ts") {
                        penna_ts = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_ts = %f", penna_ts);
                    } else if (name == "traj_opt.backup_traj.penna_pos") {
                        penna_pos = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_pos = %f", penna_pos);
                    } else if (name == "traj_opt.backup_traj.penna_vel") {
                        penna_vel = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_vel = %f", penna_vel);
                    } else if (name == "traj_opt.backup_traj.penna_acc") {
                        penna_acc = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_acc = %f", penna_acc);
                    } else if (name == "traj_opt.backup_traj.penna_jerk") {
                        penna_jerk = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_jerk = %f", penna_jerk);
                    } else if (name == "traj_opt.backup_traj.penna_attract") {
                        penna_attract = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_attract = %f", penna_attract);
                    } else if (name == "traj_opt.backup_traj.penna_omg") {
                        penna_omg = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_omg = %f", penna_omg);
                    } else if (name == "traj_opt.backup_traj.penna_thr") {
                        penna_thr = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_thr = %f", penna_thr);
                    } else if (name == "traj_opt.backup_traj.penna_max_acc_thr") {
                        penna_max_acc_thr = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_max_acc_thr = %f", penna_max_acc_thr);
                    } else if (name == "traj_opt.backup_traj.penna_min_acc_thr") {
                        penna_min_acc_thr = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: penna_min_acc_thr = %f", penna_min_acc_thr);
                    }
                    // 优化参数
                    else if (name == "traj_opt.backup_traj.opt_accuracy") {
                        opt_accuracy = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: opt_accuracy = %f", opt_accuracy);
                    } else if (name == "traj_opt.backup_traj.smooth_eps") {
                        smooth_eps = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: smooth_eps = %f", smooth_eps);
                    }
                }
                else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
                    // 积分分辨率参数
                    if (name == "traj_opt.backup_traj.integral_reso") {
                        integral_reso = parameter.as_int();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: integral_reso = %d", integral_reso);
                    }
                    // 算法配置参数
                    else if (name == "traj_opt.backup_traj.uniform_time_en") {
                        uniform_time_en = parameter.as_int() != 0;
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: uniform_time_en = %s", uniform_time_en ? "true" : "false");
                    }
                    else if (name == "traj_opt.backup_traj.pos_constraint_type") {
                        pos_constraint_type = parameter.as_int();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: pos_constraint_type = %d", pos_constraint_type);
                    }
                    else if (name == "traj_opt.backup_traj.piece_num") {
                        piece_num = parameter.as_int();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: piece_num = %d", piece_num);
                    }
                }
                else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
                    // 算法配置参数
                    if (name == "traj_opt.backup_traj.block_energy_cost") {
                        block_energy_cost = parameter.as_bool();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "BackupTraj parameter updated: block_energy_cost = %s", block_energy_cost ? "true" : "false");
                    }
                }
            }

            result.successful = true;
            return result;
        }

        /**
         * @brief 获取配置互斥锁
         * @return 配置互斥锁的引用
         * @note 在优化过程中使用此锁确保参数不被修改
         */
        std::mutex& getConfigMutex() {
            return config_mutex_;
        }
    };

} // namespace traj_opt