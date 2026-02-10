/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ROG_MAP_ROS2_DYNAMIC_HPP
#define ROG_MAP_ROS2_DYNAMIC_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rog_map/rog_map.h>
#include <super_utils/color_msg_utils.hpp>
#include <rog_map/srv/reset_map.hpp>
#include "rog_map_ros2.hpp"

#include <mutex>

namespace rog_map {
    using namespace super_utils;

    /**
     * @brief 扩展ROGMapROS类以支持额外的动态参数回调机制
     */
    class ROGMapROSDynamic : public ROGMapROS {
    private:
        // 动态参数相关
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        mutable std::mutex param_mutex_;  // 保护参数访问的互斥锁
        mutable std::mutex config_mutex_; // 保护配置参数的互斥锁
        rclcpp::Node::SharedPtr nh_;

        // Logit函数
        double calculate_logit(double val) {
            if (val <= 0.0) val = 0.001;
            if (val >= 1.0) val = 0.999;
            return std::log(val / (1.0 - val));
        }

    public:
        typedef shared_ptr<ROGMapROSDynamic> Ptr;

        ROGMapROSDynamic(const rclcpp::Node::SharedPtr nh, const std::string& cfg_path): ROGMapROS(nh, cfg_path), nh_(nh) {
            // 注意：父类ROGMapROS已经注册了参数回调和声明了参数
            // 这里不需要重复声明参数，只需要提供extendedParametersCallback供外部调用
            std::cout << GREEN << " -- [ROGMapROSDynamic] Extended dynamic parameter support initialized" << RESET << std::endl;
        }

        // 扩展的参数回调函数，处理额外的参数
        rcl_interfaces::msg::SetParametersResult extendedParametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
            std::lock_guard<std::mutex> lock(param_mutex_);
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto &param : parameters) {
                RCLCPP_INFO(nh_->get_logger(), "Setting extended parameter: %s", param.get_name().c_str());

                // 更新可视化参数
                if (param.get_name() == "visualization_enable") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.visualization_en = param.as_bool();
                    RCLCPP_INFO(nh_->get_logger(), "Updated visualization_en to %s", 
                                cfg_.visualization_en ? "true" : "false");

                } else if (param.get_name() == "visualization_range_x") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.visualization_range[0] = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated visualization_range.x to %.2f", 
                                cfg_.visualization_range[0]);

                } else if (param.get_name() == "visualization_range_y") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.visualization_range[1] = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated visualization_range.y to %.2f", 
                                cfg_.visualization_range[1]);

                } else if (param.get_name() == "visualization_range_z") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.visualization_range[2] = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated visualization_range.z to %.2f", 
                                cfg_.visualization_range[2]);

                } else if (param.get_name() == "esdf_enable") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.esdf_en = param.as_bool();
                    RCLCPP_INFO(nh_->get_logger(), "Updated esdf_en to %s", 
                                cfg_.esdf_en ? "true" : "false");

                } else if (param.get_name() == "esdf_resolution") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.esdf_resolution = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated esdf_resolution to %.2f", 
                                cfg_.esdf_resolution);

                } else if (param.get_name() == "raycasting_enable") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.raycasting_en = param.as_bool();
                    RCLCPP_INFO(nh_->get_logger(), "Updated raycasting_en to %s", 
                                cfg_.raycasting_en ? "true" : "false");

                } else if (param.get_name() == "raycasting_p_hit") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.p_hit = param.as_double();
                    cfg_.l_hit = calculate_logit(cfg_.p_hit);
                    RCLCPP_INFO(nh_->get_logger(), "Updated p_hit to %.2f", cfg_.p_hit);

                } else if (param.get_name() == "raycasting_p_miss") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.p_miss = param.as_double();
                    cfg_.l_miss = calculate_logit(cfg_.p_miss);
                    RCLCPP_INFO(nh_->get_logger(), "Updated p_miss to %.2f", cfg_.p_miss);

                } else if (param.get_name() == "raycasting_p_occ") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.p_occ = param.as_double();
                    cfg_.l_occ = calculate_logit(cfg_.p_occ);
                    RCLCPP_INFO(nh_->get_logger(), "Updated p_occ to %.2f", cfg_.p_occ);

                } else if (param.get_name() == "raycasting_p_free") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.p_free = param.as_double();
                    cfg_.l_free = calculate_logit(cfg_.p_free);
                    RCLCPP_INFO(nh_->get_logger(), "Updated p_free to %.2f", cfg_.p_free);

                } else if (param.get_name() == "inflation_step") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.inflation_step = param.as_int();
                    RCLCPP_INFO(nh_->get_logger(), "Updated inflation_step to %d", cfg_.inflation_step);

                } else if (param.get_name() == "visualization_time_rate") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.viz_time_rate = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated viz_time_rate to %.2f", cfg_.viz_time_rate);

                } else if (param.get_name() == "visualization_pub_unknown_map_en") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.pub_unknown_map_en = param.as_bool();
                    RCLCPP_INFO(nh_->get_logger(), "Updated pub_unknown_map_en to %s", 
                                cfg_.pub_unknown_map_en ? "true" : "false");

                } else if (param.get_name() == "map_sliding_threshold") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.map_sliding_thresh = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated map_sliding_thresh to %.2f", cfg_.map_sliding_thresh);

                } else if (param.get_name() == "resolution") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.resolution = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated resolution to %.2f", cfg_.resolution);

                } else if (param.get_name() == "inflation_resolution") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.inflation_resolution = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated inflation_resolution to %.2f", cfg_.inflation_resolution);

                } else if (param.get_name() == "unk_inflation_en") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.unk_inflation_en = param.as_bool();
                    RCLCPP_INFO(nh_->get_logger(), "Updated unk_inflation_en to %s", 
                                cfg_.unk_inflation_en ? "true" : "false");

                } else if (param.get_name() == "unk_inflation_step") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.unk_inflation_step = param.as_int();
                    RCLCPP_INFO(nh_->get_logger(), "Updated unk_inflation_step to %d", cfg_.unk_inflation_step);

                } else if (param.get_name() == "load_pcd_en") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.load_pcd_en = param.as_bool();
                    RCLCPP_INFO(nh_->get_logger(), "Updated load_pcd_en to %s", 
                                cfg_.load_pcd_en ? "true" : "false");

                } else if (param.get_name() == "map_sliding_en") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.map_sliding_en = param.as_bool();
                    RCLCPP_INFO(nh_->get_logger(), "Updated map_sliding_en to %s", 
                                cfg_.map_sliding_en ? "true" : "false");

                } else if (param.get_name() == "ros_callback_en") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.ros_callback_en = param.as_bool();
                    RCLCPP_INFO(nh_->get_logger(), "Updated ros_callback_en to %s", 
                                cfg_.ros_callback_en ? "true" : "false");

                } else if (param.get_name() == "cloud_topic") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.cloud_topic = param.as_string();
                    RCLCPP_INFO(nh_->get_logger(), "Updated cloud_topic to %s", cfg_.cloud_topic.c_str());

                } else if (param.get_name() == "odom_topic") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.odom_topic = param.as_string();
                    RCLCPP_INFO(nh_->get_logger(), "Updated odom_topic to %s", cfg_.odom_topic.c_str());

                } else if (param.get_name() == "odom_timeout") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.odom_timeout = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated odom_timeout to %.2f", cfg_.odom_timeout);

                } else if (param.get_name() == "use_dynamic_reconfigure") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.use_dynamic_reconfigure = param.as_bool();
                    RCLCPP_INFO(nh_->get_logger(), "Updated use_dynamic_reconfigure to %s", 
                                cfg_.use_dynamic_reconfigure ? "true" : "false");

                } else if (param.get_name() == "viz_frame_rate") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.viz_frame_rate = param.as_int();
                    RCLCPP_INFO(nh_->get_logger(), "Updated viz_frame_rate to %d", cfg_.viz_frame_rate);

                } else if (param.get_name() == "frame_id") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.frame_id = param.as_string();
                    RCLCPP_INFO(nh_->get_logger(), "Updated frame_id to %s", cfg_.frame_id.c_str());

                } else if (param.get_name() == "intensity_thresh") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.intensity_thresh = param.as_int();
                    RCLCPP_INFO(nh_->get_logger(), "Updated intensity_thresh to %d", cfg_.intensity_thresh);

                } else if (param.get_name() == "point_filt_num") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.point_filt_num = param.as_int();
                    RCLCPP_INFO(nh_->get_logger(), "Updated point_filt_num to %d", cfg_.point_filt_num);

                } else if (param.get_name() == "batch_update_size") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.batch_update_size = param.as_int();
                    RCLCPP_INFO(nh_->get_logger(), "Updated batch_update_size to %d", cfg_.batch_update_size);

                } else if (param.get_name() == "unk_thresh") {
                    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
                    cfg_.unk_thresh = param.as_double();
                    RCLCPP_INFO(nh_->get_logger(), "Updated unk_thresh to %.2f", cfg_.unk_thresh);

                } else {
                    RCLCPP_WARN(nh_->get_logger(), "Unknown extended parameter: %s", param.get_name().c_str());
                    // 不中断处理，因为可能父类能处理这个参数
                    continue;
                }
            }

            return result;
        }

    private:
        // 从父类继承的可视化函数保持不变
    };
}

#endif // ROG_MAP_ROS2_DYNAMIC_HPP