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

#include <fsm/fsm.h>
#include <memory>
#include <cmath>

using namespace super_utils;

namespace fsm {
    /**
     * FSM析构函数，关闭时间写入文件
     */
    Fsm::~Fsm() {
        write_time_.close();
    }

    /**
     * 将模块执行时间写入日志文件
     * 记录当前仿真时间与系统启动时间的差值，以及各模块的执行时间
     */
    void Fsm::WriteTimeToLog() {
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    /**
     * 执行一次重规划
     * 在FOLLOW_TRAJ状态下，当需要重新规划轨迹时调用此函数
     * 根据规划结果更新状态机状态并发布轨迹
     */
    void Fsm::callReplanOnce() {
        // 检查是否停止标志
        if (stop) {
            return;
        }

        // 检查当前状态是否为跟随轨迹状态
        if (machine_state_ != FOLLOW_TRAJ) {
            return;
        }

        // 检查是否已完成规划
        if (finish_plan) {
            return;
        }

        // 检查是否刚从静止规划，如果是则返回
        if (plan_from_rest_) {
            plan_from_rest_ = false;
            return;
        }

        // 寻找最近的非占用栅格作为目标点
        planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, gi_.goal_p, gi_.goal_p, 3.0);

        // 记录重规划执行时间
        TimeConsuming replan_once_time("replan_once_time", false);

        // 执行一次重规划
        RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        if (ret_code == FAILED) {
//            cout << YELLOW << " -- [Fsm] ReplanOnce failed." << RESET << endl;
        } else { cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl; }

        // 根据规划结果更新状态机
        if (ret_code == EMER) {
            ChangeState("ReplanTimerCallback", EMER_STOP);
        } else if (ret_code == NEW_TRAJ) {
            ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
        } else if (ret_code == SUCCESS || ret_code == FINISH) {
            gi_.new_goal = false;
            publishPolyTraj();
        }

        // 获取模块时间消耗并记录重规划时间
        planner_ptr_->getModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        // 保存重规划日志
        replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
        WriteTimeToLog();
    }

    /**
     * 执行一次主状态机循环
     * 根据当前状态执行相应的规划或控制逻辑
     * 包括初始化、等待目标、生成轨迹、跟随轨迹和紧急停止等状态
     */
    void Fsm::callMainFsmOnce() {
        if (stop) {
            return;
        }
        static double fsm_start_time = ros_ptr_->getSimTime();
        double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);
        static double last_print_t = 0.0;
        planner_ptr_->getRobotState(robot_state_);


        // 每秒打印一次状态信息
        if (cur_t - last_print_t > 1.0) {
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                return;
            }
            if (!started_) {
                cout << YELLOW << " -- [Fsm] Wait for goal." << RESET << endl;
            }
            cout << std::fixed << std::setprecision(3);
            cout << GREEN << " -- [Fsm " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_]
                 << RESET << endl;
        }

        // 根据当前状态执行相应操作
        switch (machine_state_) {
            case INIT: {
                // 初始化状态：检查是否已启动，更新到等待目标状态
                if (!started_) {
                    return;
                }
                if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            case WAIT_GOAL: {
                // 等待目标状态：当有新目标时切换到生成轨迹状态
                if (!gi_.new_goal) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                resetVisualizedPath();
                break;
            }
            case GENERATE_TRAJ: {
                // 生成轨迹状态：检查是否接近目标，执行从静止规划
                if (closeToGoal(0.1)) {
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    finish_plan = true;
                    return;
                }
                RET_CODE retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
                if (!planner_ptr_->goalValid()) {
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    return;
                }
                if (retcode == SUCCESS || retcode == FINISH) {
                    gi_.new_goal = false;
                    plan_from_rest_ = true;
                    finish_plan = false;
                    if (retcode == FINISH) {
                        finish_plan = true;
                    }

                    publishPolyTraj();

                    ChangeState("MainFsmCallback", FOLLOW_TRAJ);
                } else {
                    cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan." << RESET << endl;
                    // ros::Duration(0.1).sleep();
                }
                replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
                break;
            }
            case FOLLOW_TRAJ: {
                // 跟随轨迹状态：发布当前位姿到路径
                publishCurPoseToPath();
                break;
            }
            case EMER_STOP: {
                // 紧急停止状态：切换回等待目标状态
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
        }
    }

    /**
     * 检查机器人是否接近目标点
     * @param thresh_dis 距离阈值
     * @return 如果机器人与目标点的距离小于阈值则返回true
     */
    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    /**
     * 设置目标位置和偏航角
     * 处理用户通过RViz点击设置的目标点，验证目标有效性并更新规划目标
     * @param p 目标位置向量
     * @param q 目标姿态四元数
     */
    void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q) {

        // 处理点击点的高度设置
        auto click_point = p;
        if (cfg_.click_height > -5) {
            // 验证参数使用
            std::cout << "[Fsm] Using click_height: " << cfg_.click_height << std::endl;
            click_point.z() = cfg_.click_height;
        }

        // 寻找最近的非占用栅格作为最终目标点
        if (planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, click_point, gi_.goal_p, 3.0)) {
            cout << GREEN << " -- [Fsm] Get goal at " << RESET << gi_.goal_p.transpose() << endl;
        } else {
            fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
            return;
        }

        // 检查目标点是否太接近当前位置
        if ((robot_state_.p - gi_.goal_p).norm() <
            0.1) {
            //                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            return;
        }

        // 处理目标偏航角设置
        if (cfg_.click_yaw_en) {
            if (std::isnan(q.w()) || std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z())) {
                gi_.goal_yaw = NAN;
                ros_ptr_->info(" -- [Fsm] Receive click goal at: [{}, {}, {}]; goal yaw disabled",
                               gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
            } else {
                gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
                cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw: "
                     << gi_.goal_yaw * 57.3 << " deg" << RESET << endl;
            }

        } else {
            gi_.goal_yaw = NAN;
            cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }

        // 更新启动标志和新目标标志
        started_ = true;
        gi_.new_goal = true;
    }

    /**
     * 状态切换函数
     * 将状态机从当前状态切换到新状态，并打印状态切换信息
     * @param call_func 调用此函数的函数名
     * @param new_state 新的状态
     */
    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
        machine_state_ = new_state;
    }
}
