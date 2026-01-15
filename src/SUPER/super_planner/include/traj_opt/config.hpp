
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

#include <string>
#include <utils/geometry/quadrotor_flatness.hpp>
#include <utils/header/yaml_loader.hpp>
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+(name)))  // 调试文件目录宏定义

namespace traj_opt {
    using std::string;
    using std::vector;

    /**
     * @enum PosConstrainType
     * @brief 位置约束类型枚举
     */
    enum PosConstrainType {
        WAYPOINT = 1,  // 路径点约束模式
        CORRIDOR = 2,  // 走廊约束模式
    };

    /**
     * @class Config
     * @brief 轨迹优化配置类，管理优化器的所有参数
     * @note 从YAML配置文件加载参数，支持命名空间配置
     */
    class Config {
    public:
        bool uniform_time_en{false};                      // 是否启用均匀时间分配

        flatness::FlatnessMap quadrotot_flatness;         // 四旋翼平坦性映射对象

        bool print_optimizer_log{false};                  // 是否打印优化器日志

        // 平坦性相关参数
        double mass;                                      // 四旋翼质量，单位：kg
        double dh;                                        // 水平阻力系数
        double dv;                                        // 垂直阻力系数
        double grav;                                      // 重力加速度，单位：m/s²
        double cp;                                        // 推力系数
        double v_eps;                                     // 速度epsilon参数

        bool save_log_en{false};                          // 是否保存优化问题日志

        int pos_constraint_type{CORRIDOR};                // 位置约束类型
        bool block_energy_cost{false};                    // 是否禁用能量代价（仅最小时间优化）

        // 边界条件限制
        double max_vel{0};                                // 最大速度限制，单位：m/s
        double max_acc{0};                                // 最大加速度限制，单位：m/s²
        double max_jerk{0};                               // 最大加加速度限制，单位：m/s³
        double max_omg{0};                                // 最大角速度限制，单位：rad/s
        double max_acc_thr{0};                            // 最大推力加速度限制，单位：m/s²
        double min_acc_thr{0};                            // 最小推力加速度限制，单位：m/s²

        // 惩罚代价权重
        double penna_scale{-1};                           // 惩罚项全局缩放系数
        double penna_vel{0};                              // 速度惩罚权重
        double penna_acc{0};                              // 加速度惩罚权重
        double penna_jerk{0};                             // 加加速度惩罚权重
        double penna_omg{0};                              // 角速度惩罚权重
        double penna_thr{0};                              // 推力惩罚权重
        double penna_t{0};                                // 时间惩罚权重（仅走廊约束模式）
        double penna_pos{0};                              // 位置惩罚权重（仅走廊约束模式）
        double penna_attract{0};                          // 吸引力惩罚权重
        double penna_ts{0};                               // 时间段惩罚权重（仅备份轨迹）

        int piece_num{0};                                 // 备份轨迹分段数量

        double penna_margin{0.05};                        // 安全边界惩罚权重

        double smooth_eps{0};                             // 平滑epsilon参数
        int integral_reso{0};                             // 积分分辨率
        double opt_accuracy{0};                           // 优化精度

        /**
         * @brief 默认构造函数
         */
        Config() = default;

        /**
         * @brief 参数化构造函数，从YAML文件加载配置
         * @param cfg_path YAML配置文件路径
         * @param ns 参数命名空间
         * @note 支持通过命名空间实现多配置加载
         */
        Config(const std::string & cfg_path, string ns) {
            // 创建YAML加载器
            yaml_loader::YamlLoader loader(cfg_path);
            
            // 处理命名空间
            if (ns.empty()) {
                ns = "/";
            }
            else {
                ns = "/" + ns + "/";
            }

            // 加载开关参数
            loader.LoadParam("traj_opt/switch/print_optimizer_log", print_optimizer_log, false);  // 优化器日志开关
            
            // 加载平坦性参数
            loader.LoadParam("traj_opt/flatness/mass", mass, 1.0);           // 四旋翼质量
            loader.LoadParam("traj_opt/flatness/dh", dh, 0.7);               // 水平阻力系数
            loader.LoadParam("traj_opt/flatness/dv", dv, 0.8);               // 垂直阻力系数
            loader.LoadParam("traj_opt/flatness/grav", grav, 1.0);           // 重力加速度
            loader.LoadParam("traj_opt/flatness/cp", cp, 0.01);              // 推力系数
            loader.LoadParam("traj_opt/flatness/v_eps", v_eps, 0.0001);      // 速度epsilon

            // 加载日志和约束类型参数
            loader.LoadParam("traj_opt/switch/save_log_en", save_log_en, false);                      // 日志保存开关
            loader.LoadParam("traj_opt" + ns + "pos_constraint_type", pos_constraint_type, 2);        // 位置约束类型
            loader.LoadParam("traj_opt" + ns + "piece_num", piece_num, 1);                            // 轨迹分段数量
            loader.LoadParam("traj_opt" + ns + "uniform_time_en", uniform_time_en, false);            // 均匀时间分配开关
            loader.LoadParam("traj_opt" + ns + "block_energy_cost", block_energy_cost, false);        // 能量代价禁用开关
            loader.LoadParam("traj_opt" + ns + "opt_accuracy", opt_accuracy, 1.0e-5);                 // 优化精度
            loader.LoadParam("traj_opt" + ns + "integral_reso", integral_reso, 10);                   // 积分分辨率
            loader.LoadParam("traj_opt" + ns + "smooth_eps", smooth_eps, 0.01);                       // 平滑epsilon

            // 加载边界约束参数
            loader.LoadParam("traj_opt/boundary/max_vel", max_vel, -5.0);                             // 最大速度
            loader.LoadParam("traj_opt/boundary/max_acc", max_acc, -5.0);                             // 最大加速度
            loader.LoadParam("traj_opt/boundary/max_jerk", max_jerk, -1.0);                           // 最大加加速度
            loader.LoadParam("traj_opt/boundary/max_omg", max_omg, -1.0);                             // 最大角速度
            loader.LoadParam("traj_opt/boundary/max_acc_thr", max_acc_thr, -1.0);                     // 最大推力加速度
            loader.LoadParam("traj_opt/boundary/min_acc_thr", min_acc_thr, -1.0);                     // 最小推力加速度
            loader.LoadParam("traj_opt/boundary/penna_margin", penna_margin, 0.05);                   // 安全边界

            // 加载惩罚项权重参数
            loader.LoadParam("traj_opt" + ns + "penna_scale", penna_scale, -1.0);                     // 全局缩放系数
            loader.LoadParam("traj_opt" + ns + "penna_t", penna_t, -1.0);                             // 时间惩罚权重
            loader.LoadParam("traj_opt" + ns + "penna_ts", penna_ts, -1.0);                           // 时间段惩罚权重
            loader.LoadParam("traj_opt" + ns + "penna_pos", penna_pos, -1.0);                         // 位置惩罚权重
            loader.LoadParam("traj_opt" + ns + "penna_vel", penna_vel, -1.0);                         // 速度惩罚权重
            loader.LoadParam("traj_opt" + ns + "penna_acc", penna_acc, -1.0);                         // 加速度惩罚权重
            loader.LoadParam("traj_opt" + ns + "penna_jerk", penna_jerk, -1.0);                       // 加加速度惩罚权重
            loader.LoadParam("traj_opt" + ns + "penna_attract", penna_attract, -1.0);                 // 吸引力惩罚权重
            loader.LoadParam("traj_opt" + ns + "penna_omg", penna_omg, -1.0);                         // 角速度惩罚权重
            loader.LoadParam("traj_opt" + ns + "penna_thr", penna_thr, -1.0);                         // 推力惩罚权重

            // 应用全局缩放系数
            if (penna_scale > 0) {
                penna_t = penna_t * penna_scale;                              // 缩放时间惩罚
                penna_ts = penna_ts * penna_scale;                            // 缩放时间段惩罚
                penna_pos = penna_pos * penna_scale;                          // 缩放位置惩罚
                penna_vel = penna_vel * penna_scale;                          // 缩放速度惩罚
                penna_acc = penna_acc * penna_scale;                          // 缩放加速度惩罚
                penna_jerk = penna_jerk * penna_scale;                        // 缩放加加速度惩罚
                penna_attract = penna_attract * penna_scale;                  // 缩放吸引力惩罚
                penna_omg = penna_omg * penna_scale;                          // 缩放角速度惩罚
                penna_thr = penna_thr * penna_scale;                          // 缩放推力惩罚
            }

            // 初始化四旋翼平坦性映射
            quadrotot_flatness.reset(mass, grav, dh, dv, cp, v_eps);
        }
    };
}
