
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

#include "memory"

#include <super_core/config.hpp>
#include <super_core/ciri.h>

#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <utils/header/fmt_eigen.hpp>

#include <ros_interface/ros_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "super_core/corridor_generator_config.h"  // 新增动态参数配置头文件



namespace super_planner {
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
    using namespace geometry_utils;
    using super_utils::Vec3i;
    using super_utils::vec_E;
    using super_utils::Line;
    using namespace color_text;
    using super_utils::OCCUPIED;

    /**
     * @brief 走廊生成器类
     * 
     * 该类负责在给定路径上生成安全飞行走廊(Safe Flight Corridor)
     * 使用CIRI算法从点云地图中提取凸多面体区域
     */
    class CorridorGenerator {
    private:
        ros_interface::RosInterface::Ptr ros_ptr_;  // ROS接口指针
        double bound_dis_;  // 边界距离
        double seed_line_max_length_;  // 种子线段最大长度
        double min_overlap_threshold_;  // 最小重叠阈值
        double robot_r_;  // 机器人半径
        int box_search_skip_num_;  // 包围盒搜索跳过数量
        int iris_iter_num_;  // IRIS迭代次数
        double virtual_ground_height_ = 0.0;  // 虚拟地面高度
        double virtual_ceil_height_ = 0.0;  // 虚拟天花板高度
        rog_map::ROGMapROS::Ptr map_ptr_;  // ROG地图指针
        vec_E<Vec3i> line_seed_neighbor_list;  // 线段种子邻居列表
        CIRI::Ptr ciri_;  // CIRI算法指针
        std::ofstream failed_traj_log;  // 失败轨迹日志文件流
        CorridorGenConfig dyn_config_;  // 动态参数配置
        mutable std::mutex corridor_gen_mutex_;  // 用于更新参数的互斥锁

        vec_Vec3f latest_pc;  // 最新点云数据

        double ciri_t{0};  // CIRI计算总时间
        int ciri_cnt{0};  // CIRI调用次数
    public:
        /**
         * @brief 获取最新的点云数据
         * @return 点云数据向量,获取后会清空内部缓存
         */
        vec_Vec3f getLatestCloud() {
            vec_Vec3f out = latest_pc;
            latest_pc.clear();
            return out;
        }

        /**
         * @brief 构造函数
         * @param ros_ptr ROS接口智能指针
         * @param map_ptr ROG地图智能指针
         * @param bound_dis 边界距离参数
         * @param seed_line_max_dis 种子线段最大距离
         * @param min_overlap_threshold 最小重叠阈值
         * @param virtual_ground_height 虚拟地面高度
         * @param virtual_ceil_height 虚拟天花板高度
         * @param robot_r 机器人半径
         * @param box_search_skip_num 包围盒搜索跳过数量
         * @param iris_iter_num IRIS算法迭代次数
         */
        CorridorGenerator(const ros_interface::RosInterface::Ptr &ros_ptr,
                          const rog_map::ROGMapROS::Ptr & map_ptr,
                          const double bound_dis,
                          const double seed_line_max_dis,
                          const double min_overlap_threshold,
                          const double virtual_ground_height,
                          const double virtual_ceil_height,
                          const double robot_r,
                          const int box_search_skip_num,
                          const int iris_iter_num);

        ~CorridorGenerator() = default;

        /**
         * @brief 设置线段邻居列表
         * @param line_seed_neighbor_list 线段种子邻居列表
         */
        void SetLineNeighborList(const vec_E<Vec3i> &line_seed_neighbor_list);

        typedef std::shared_ptr<CorridorGenerator> Ptr;

        /**
         * @brief 获取动态参数配置
         * @return CorridorGenConfig的引用
         * @note 用于外部访问和修改动态参数
         */
        CorridorGenConfig& getDynamicConfig() {
            return dyn_config_;
        }

        /**
         * @brief 更新优化器配置
         * @note 从动态配置中更新内部参数
         */
        void updateOptimizerConfig() {
            std::lock_guard<std::mutex> lock(corridor_gen_mutex_);
            bound_dis_ = dyn_config_.bound_dis;
            seed_line_max_length_ = dyn_config_.seed_line_max_length;
            min_overlap_threshold_ = dyn_config_.min_overlap_threshold;
            robot_r_ = dyn_config_.robot_r;
            box_search_skip_num_ = dyn_config_.box_search_skip_num;
            iris_iter_num_ = dyn_config_.iris_iter_num;
            virtual_ground_height_ = dyn_config_.virtual_ground_height;
            virtual_ceil_height_ = dyn_config_.virtual_ceil_height;
        }

        /**
         * @brief 在给定路径上搜索凸多面体
         * @param path 输入路径点序列
         * @param sfcs 输出的安全飞行走廊(凸多面体向量)
         * @param shifted_start_pt 输出的偏移后起始点
         * @param cut_first_poly 是否裁剪第一个多面体,默认为false
         * @return 搜索是否成功
         */
        bool SearchPolytopeOnPath(const vec_Vec3f &path, PolytopeVec &sfcs,
                                  Vec3f & shifted_start_pt,
                                  bool cut_first_poly = false);

        /**
         * @brief 获取种子包围盒
         * @param p1 线段起点
         * @param p2 线段终点
         * @param box_min 输出包围盒最小顶点
         * @param box_max 输出包围盒最大顶点
         */
        void getSeedBBox(const Vec3f &p1, const Vec3f &p2,
                         Vec3f &box_min, Vec3f &box_max);

        /**
         * @brief 从单个点生成凸多面体
         * @param pt 输入点
         * @param polytope 输出的凸多面体
         * @return 生成是否成功
         */
        bool GeneratePolytopeFromPoint(const Vec3f &pt, Polytope &polytope);

        /**
         * @brief 生成空的凸多面体
         * @param pt 中心点
         * @param dis 距离参数
         * @param polytope 输出的凸多面体
         * @return 生成是否成功
         */
        bool GenerateEmptyPolytope(const super_utils::Vec3f &pt,
                                   const double & dis,
                                   Polytope & polytope);

        /**
         * @brief 从线段生成凸多面体
         * @param line 输入线段
         * @param polytope 输出的凸多面体
         * @return 生成是否成功
         */
        bool GeneratePolytopeFromLine(Line &line, Polytope &polytope);

        /**
         * @brief 获取CIRI算法的平均计算时间
         * @return 平均计算时间(秒),如果没有调用过则返回-1
         * @note 调用后会重置内部计时器和计数器
         */
        double getCiriComputationTime() {
            if (ciri_cnt == 0) {
                return -1;
            }
            double aver_T = ciri_t / ciri_cnt;
            ciri_t = 0;
            ciri_cnt = 0;
            return aver_T;
        }

        /**
         * @brief 设置IRIS算法迭代次数
         * @param iter 迭代次数
         */
        void setIterNum(int iter){
            iris_iter_num_ = iter;
        }

    };
}
