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
#include <vector>

#include "utils/geometry/geometry_utils.h"
#include "traj_opt/config.hpp"
#include "traj_opt/yaw_traj_config.h"  // 新增动态参数配置头文件
#include <utils/optimization/minco.h>

#include <utils/header/type_utils.hpp>
#include <super_utils/scope_timer.hpp>
#include <mutex>  // 添加互斥锁支持

namespace traj_opt {
    using namespace geometry_utils;
    using namespace optimization_utils;
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class YawTrajOpt {
    private:
        bool free_goal_{false};
        double yaw_dot_max_{10};
        YawTrajConfig dyn_config_;  // 动态参数配置
        mutable std::mutex yaw_opt_mutex_;  // 用于更新yaw_dot_max的互斥锁

    public:

        explicit YawTrajOpt(const double &_yaw_dot_max);

        typedef std::shared_ptr<YawTrajOpt> Ptr;

        /**
         * @brief 获取动态参数配置
         * @return YawTrajConfig的引用
         * @note 用于外部访问和修改动态参数
         */
        YawTrajConfig& getDynamicConfig() {
            return dyn_config_;
        }

        /**
         * @brief 更新优化器配置
         * @note 从动态配置中更新内部参数
         */
        void updateOptimizerConfig() {
            std::lock_guard<std::mutex> lock(yaw_opt_mutex_);
            yaw_dot_max_ = dyn_config_.yaw_dot_max;
        }

        void getYawTimeAllocation(const double &duration, VecDf &times) const ;

        static void getYawWaypointAllocation(const Vec4f &init_state, Vec4f &goal_state, VecDf &way_pts, VecDf &times,
                                      const Trajectory &pos_traj) ;

        bool optimize(const Vec4f &istate_in,
                      const Vec4f &gstate_in,
                      const Trajectory &pos_traj,
                      Trajectory &out_traj,
                      const int & order = 3,
                      const bool &free_start = false,
                      const bool &free_goal = true);

    };


}