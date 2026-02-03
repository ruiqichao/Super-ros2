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

namespace path_search {

    /**
     * @brief Astar路径搜索配置类，支持动态参数调整
     * @note 此类封装了Astar路径搜索器的可动态调整参数
     */
    class AstarConfig {
    private:
        std::mutex config_mutex_;  // 用于线程安全的互斥锁

    public:
        // Astar算法相关参数
        int heu_type{0};              // 启发式类型: 0-DIAG, 1-MANH, 2-EUCL
        bool allow_diag{false};       // 是否允许对角线移动
        bool debug_visualization_en{false};  // 调试可视化开关

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

                if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
                    // Astar算法参数
                    if (name == "astar.heu_type") {
                        heu_type = parameter.as_int();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "Astar parameter updated: heu_type = %d", heu_type);
                    }
                } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
                    // Astar算法参数
                    if (name == "astar.allow_diag") {
                        allow_diag = parameter.as_bool();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "Astar parameter updated: allow_diag = %s", allow_diag ? "true" : "false");
                    } else if (name == "astar.debug_visualization_en") {
                        debug_visualization_en = parameter.as_bool();
                        RCLCPP_INFO(rclcpp::get_logger("super_planner"), 
                                   "Astar parameter updated: debug_visualization_en = %s", debug_visualization_en ? "true" : "false");
                    }
                }
            }

            result.successful = true;
            return result;
        }

        /**
         * @brief 获取配置互斥锁
         * @return 配置互斥锁的引用
         * @note 在搜索过程中使用此锁确保参数不被修改
         */
        std::mutex& getConfigMutex() {
            return config_mutex_;
        }
    };

} // namespace path_search