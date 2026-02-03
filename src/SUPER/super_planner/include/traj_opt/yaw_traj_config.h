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
     * @brief YawTrajOpt偏航轨迹优化器配置类，支持动态参数调整
     * @note 此类封装了偏航轨迹优化器的可动态调整参数
     */
    class YawTrajConfig {
    private:
        std::mutex config_mutex_;  // 用于线程安全的互斥锁

    public:
        // 偏航角相关参数
        double yaw_dot_max{3.14};              // 最大偏航角速度，单位：rad/s

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
                    // 偏航角参数
                    if (name == "traj_opt.yaw_traj.yaw_dot_max") {
                        yaw_dot_max = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "YawTraj parameter updated: yaw_dot_max = %f", yaw_dot_max);
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