
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

#ifndef SUPER_RET_CODE_HPP
#define SUPER_RET_CODE_HPP

#include <cstring>
#include <vector>

namespace super_planner {
    // SUPER返回状态码枚举
    enum SUPER_RET_CODE {
        SUPER_SUCCESS_WITH_BACKUP = 3,  // 成功，且备份轨迹也成功
        SUPER_SUCCESS_NO_BACKUP = 2,     // 成功，无需备份
        SUPER_SUCCESS = 1,               // 成功
        SUPPER_UNDEFINED = -0,           // 未定义状态
        SUPER_NO_ODOM = -1,              // 无里程计数据
        SUPER_NO_START_POINT = -2,       // 无法找到起始点

    };

    // 将返回状态码转换为描述字符串
    static std::string SUPER_RET_CODE_STR(const int& ret) {
        switch (ret) {
        case SUPER_SUCCESS_WITH_BACKUP:
            return "Success, with backup trajectory also success";  // 成功，备份轨迹也成功
        case SUPER_SUCCESS_NO_BACKUP:
            return "Success, without need of backup";  // 成功，无需备份
        case SUPER_SUCCESS:
            return "Success";  // 成功
        case SUPPER_UNDEFINED:
            return "Undefined";  // 未定义
        case SUPER_NO_ODOM:
            return "No odom, return at the start of the replan";  // 无里程计，在重规划开始时返回
        case SUPER_NO_START_POINT:
            return "Cannot find a start point in the local map";  // 无法在局部地图中找到起始点
        }
    };
}
#endif
