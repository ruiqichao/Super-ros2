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

namespace super_planner {

    /**
     * @brief 走廊生成器配置类，支持动态参数调整
     * @note 此类封装了走廊生成器的可动态调整参数
     */
    class CorridorGenConfig {
    private:
        std::mutex config_mutex_;  // 用于线程安全的互斥锁

    public:
        // 走廊生成相关参数
        double bound_dis{3.0};              // 边界距离，单位：m
        double seed_line_max_length{3.0};   // 种子线段最大长度，单位：m
        double min_overlap_threshold{0.1};  // 最小重叠阈值，单位：m
        double robot_r{0.3};               // 机器人半径，单位：m
        int box_search_skip_num{1};         // 包围盒搜索跳过数量
        int iris_iter_num{1};               // IRIS算法迭代次数
        double virtual_ground_height{0.0};   // 虚拟地面高度，单位：m
        double virtual_ceil_height{0.0};    // 虚拟天花板高度，单位：m

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
                    // 走廊生成参数
                    if (name == "super_planner.corridor_bound_dis") {
                        bound_dis = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "CorridorGen parameter updated: bound_dis = %f", bound_dis);
                    } else if (name == "super_planner.corridor_line_max_length") {
                        seed_line_max_length = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "CorridorGen parameter updated: seed_line_max_length = %f", seed_line_max_length);
                    } else if (name == "super_planner.min_overlap_threshold") {
                        min_overlap_threshold = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "CorridorGen parameter updated: min_overlap_threshold = %f", min_overlap_threshold);
                    } else if (name == "super_planner.robot_r") {
                        robot_r = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "CorridorGen parameter updated: robot_r = %f", robot_r);
                    } else if (name == "rog_map.virtual_ground_height") {
                        virtual_ground_height = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "CorridorGen parameter updated: virtual_ground_height = %f", virtual_ground_height);
                    } else if (name == "rog_map.virtual_ceil_height") {
                        virtual_ceil_height = parameter.as_double();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "CorridorGen parameter updated: virtual_ceil_height = %f", virtual_ceil_height);
                    }
                } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
                    // 走廊生成参数
                    if (name == "super_planner.obs_skip_num") {
                        box_search_skip_num = parameter.as_int();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "CorridorGen parameter updated: box_search_skip_num = %d", box_search_skip_num);
                    } else if (name == "super_planner.iris_iter_num") {
                        iris_iter_num = parameter.as_int();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "CorridorGen parameter updated: iris_iter_num = %d", iris_iter_num);
                    }
                }
            }

            result.successful = true;
            return result;
        }

        /**
         * @brief 获取配置互斥锁
         * @return 配置互斥锁的引用
         * @note 在生成过程中使用此锁确保参数不被修改
         */
        std::mutex& getConfigMutex() {
            return config_mutex_;
        }
    };

} // namespace super_planner