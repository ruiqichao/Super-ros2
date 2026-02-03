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

#ifndef SRC_FSM_ROS2_HPP
#define SRC_FSM_ROS2_HPP

#include "fsm/fsm.h"

#include <rclcpp/rclcpp.hpp>
#include <ros_interface/ros2/ros2_interface.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mars_quadrotor_msgs/msg/position_command.hpp"
#include "mars_quadrotor_msgs/msg/polynomial_trajectory.hpp"
#include "px4ctrl_msgs/msg/command.hpp"
#include "traj_opt/super_traj_config.h"


namespace fsm {
    class FsmRos2 : public Fsm {

        rclcpp::Node::SharedPtr nh_;
        rclcpp::Publisher<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub_;
        rclcpp::Publisher<mars_quadrotor_msgs::msg::PolynomialTrajectory>::SharedPtr mpc_cmd_pub_;
        rclcpp::Publisher<px4ctrl_msgs::msg::Command>::SharedPtr px4_cmd_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

        rclcpp::TimerBase::SharedPtr execution_timer_, replan_timer_, cmd_timer_;
        rclcpp::CallbackGroup::SharedPtr exec_cbk_group_, replan_cbk_group_, cmd_cbk_group_, goal_cbk_group_;

        // 动态参数回调处理器
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

        mars_quadrotor_msgs::msg::PositionCommand pid_cmd_;
        rog_map::ROGMapROS::Ptr map_ptr_;
        mars_quadrotor_msgs::msg::PositionCommand latest_cmd;
        nav_msgs::msg::Path path;

        vector<mars_quadrotor_msgs::msg::PositionCommand> cmd_logs_;

        void resetVisualizedPath() override {
            path.poses.clear();
        }

        void publishCurPoseToPath() override {
            path.header.frame_id = "world";
            ros_ptr_->getSimTime(path.header.stamp.sec, path.header.stamp.nanosec);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = robot_state_.p(0);
            pose.pose.position.y = robot_state_.p(1);
            pose.pose.position.z = robot_state_.p(2);
            pose.pose.orientation.x = robot_state_.q.x();
            pose.pose.orientation.y = robot_state_.q.y();
            pose.pose.orientation.z = robot_state_.q.z();
            pose.pose.orientation.w = robot_state_.q.w();
            path.poses.push_back(pose);
            path_pub_->publish(path);
        }

        void publishPolyTraj() override {
            mars_quadrotor_msgs::msg::PolynomialTrajectory cmd_traj;
            getCommittedTrajectory(cmd_traj);
            mpc_cmd_pub_->publish(cmd_traj);
        }

        void getOneHeartBeatMsg(mars_quadrotor_msgs::msg::PolynomialTrajectory &heartbeat, bool &traj_finish) {
            heartbeat.type = mars_quadrotor_msgs::msg::PolynomialTrajectory::HEART_BEAT;
            ros_ptr_->getSimTime(heartbeat.header.stamp.sec, heartbeat.header.stamp.nanosec);
            heartbeat.header.frame_id = "world";
            double swt;
            planner_ptr_->getOneHeartbeatTime(swt, traj_finish);
            heartbeat.start_wt_pos = swt;
        }

        void getCommittedTrajectory(mars_quadrotor_msgs::msg::PolynomialTrajectory &cmd_traj) {
            ros_ptr_->getSimTime(cmd_traj.header.stamp.sec, cmd_traj.header.stamp.nanosec);
            cmd_traj.header.frame_id = "world";
            cmd_traj.type = mars_quadrotor_msgs::msg::PolynomialTrajectory::POSITION_TRAJ |
                            mars_quadrotor_msgs::msg::PolynomialTrajectory::HEART_BEAT;
            planner_ptr_->lockCommittedTraj();
            const Trajectory pos_traj = planner_ptr_->getCommittedPositionTrajectory();
            const Trajectory yaw_traj = planner_ptr_->getCommittedYawTrajectory();
            planner_ptr_->unlockCommittedTraj();

            cmd_traj.start_wt_pos = pos_traj.start_WT;

            cmd_traj.piece_num_pos = pos_traj.getPieceNum();
            cmd_traj.order_pos = 7;
            cmd_traj.time_pos.resize(pos_traj.getPieceNum());
            cmd_traj.coef_pos_x.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.coef_pos_y.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.coef_pos_z.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.start_wt_pos = pos_traj.start_WT;

            if (!yaw_traj.empty()) {
                cmd_traj.type = cmd_traj.type |
                                mars_quadrotor_msgs::msg::PolynomialTrajectory::YAW_TRAJ;
                cmd_traj.piece_num_yaw = yaw_traj.getPieceNum();
                cmd_traj.order_yaw = 7;
                double col_size = cmd_traj.order_yaw + 1;
                cmd_traj.coef_yaw.resize(cmd_traj.piece_num_yaw * col_size);
                cmd_traj.time_yaw.resize(cmd_traj.piece_num_yaw);
                for (int i = 0; i < cmd_traj.piece_num_yaw; i++) {
                    Eigen::VectorXd yaw_coef = yaw_traj[i].getCoeffMat().row(0);
                    Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_yaw[col_size * i], col_size) = yaw_coef;
                    cmd_traj.time_yaw[i] = yaw_traj[i].getDuration();
                }
                cmd_traj.start_wt_yaw = yaw_traj.start_WT;
            }

            for (int i = 0; i < cmd_traj.piece_num_pos; i++) {
                Eigen::Matrix<double, 3, 8> coef = pos_traj[i].getCoeffMat();
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_x[8 * i], 8) = coef.row(0);
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_y[8 * i], 8) = coef.row(1);
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_z[8 * i], 8) = coef.row(2);
                cmd_traj.time_pos[i] = pos_traj[i].getDuration();
            }
        }

        void getOnePositionCommand(mars_quadrotor_msgs::msg::PositionCommand &pos_cmd, bool &traj_finish) {
            pos_cmd.trajectory_flag = 0;
            StatePVAJ pvaj;
            double yaw, yaw_dot;
            bool on_backup_traj;
            planner_ptr_->getOneCommandFromTraj(pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
//            ros_ptr_->getSimTime(pos_cmd.header.stamp.sec, pos_cmd.header.stamp.nanosec);
            ros_ptr_->getSimTime(pos_cmd.header.stamp.sec, pos_cmd.header.stamp.nanosec);
            pos_cmd.header.frame_id = "world";
            pos_cmd.position.x = pvaj(0, 0);
            pos_cmd.position.y = pvaj(1, 0);
            pos_cmd.position.z = pvaj(2, 0);
            pos_cmd.velocity.x = pvaj(0, 1);
            pos_cmd.velocity.y = pvaj(1, 1);
            pos_cmd.velocity.z = pvaj(2, 1);
            pos_cmd.acceleration.x = pvaj(0, 2);
            pos_cmd.acceleration.y = pvaj(1, 2);
            pos_cmd.acceleration.z = pvaj(2, 2);
            pos_cmd.jerk.x = pvaj(0, 3);
            pos_cmd.jerk.y = pvaj(1, 3);
            pos_cmd.jerk.z = pvaj(2, 3);
            pos_cmd.yaw = yaw;
            pos_cmd.yaw_dot = yaw_dot;
            pos_cmd.trajectory_flag = on_backup_traj ? 2 : 1;
            Vec3f rpy, omg;
            double aT;
            geometry_utils::convertFlatOutputToAttAndOmg(pvaj.col(0), pvaj.col(1), pvaj.col(2), pvaj.col(3), yaw,
                                                         yaw_dot, rpy, omg, aT);
            pos_cmd.attitude.x = rpy(0);
            pos_cmd.attitude.y = rpy(1);
            pos_cmd.attitude.z = rpy(2);
            pos_cmd.angular_velocity.x = omg(0);
            pos_cmd.angular_velocity.y = omg(1);
            pos_cmd.angular_velocity.z = omg(2);
            pos_cmd.thrust.z = aT;
            latest_cmd = pos_cmd;
            cmd_logs_.push_back(latest_cmd);
        }

    public:
        FsmRos2() = default;

        ~FsmRos2() {
            saveReplanLogToFile();
        };

        typedef std::shared_ptr<FsmRos2> Ptr;

        void saveReplanLogToFile(const string &name = "") {
            // run statistic
            double total_length{0.0};
            int total_replan_num{0};
            double average_compt_t{0.0};
            Vec3f cur_p{0, 0, 0};
            for (auto rp: replan_logs_) {
                if (rp.getRetCode() > 0) {
                    if (cur_p.norm() < 1e-6) {
                        cur_p = rp.getRobotP();
                    } else {
                        total_length += (rp.getRobotP() - cur_p).norm();
                        cur_p = rp.getRobotP();
                    }
                    total_replan_num++;
                    average_compt_t += rp.getTotalCompT();
                }
            }


            fmt::print("Total replan num: {}, total length: {}, average computation time: {} ms\n",
                       total_replan_num, total_length,
                       average_compt_t / (total_replan_num == 0 ? 1 : total_replan_num) * 1000);


            const std::string save_path = name.empty()
                                          ? LOG_FILE_DIR(
                                                  "replan_logs/" + BinaryFileHandler<int>::getCurrentTimeStr() + ".bin")
                                          : LOG_FILE_DIR("replan_logs/" + name + ".bin");
            const std::string csv_path = name.empty()
                                         ? LOG_FILE_DIR(
                                                 "cmd_logs/" + BinaryFileHandler<int>::getCurrentTimeStr() + ".csv")
                                         : LOG_FILE_DIR("cmd_logs/" + name + ".csv");
            BinaryFileHandler<vector<LogOneReplan>>::save(save_path, replan_logs_);

            std::ofstream csv_writer;
            csv_writer.open(csv_path, std::ios::out | std::ios::trunc);
            csv_writer
                    << "time,posi_x,posi_y,posi_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,jerk_x,jerk_y,jerk_z,yaw,yaw_rate,backup"
                    << std::endl;
            csv_writer << std::fixed << std::setprecision(15);
            for (const auto &cmd: cmd_logs_) {
                const double timestamp = static_cast<double>(cmd.header.stamp.sec) +
                                         static_cast<double>(cmd.header.stamp.nanosec) * 1e-9;
                csv_writer << timestamp - system_start_time_
                           << "," << cmd.position.x << "," << cmd.position.y << ","
                           << cmd.position.z << ","
                           << cmd.velocity.x << "," << cmd.velocity.y << "," << cmd.velocity.z << ","
                           << cmd.acceleration.x << "," << cmd.acceleration.y << "," << cmd.acceleration.z << ","
                           << cmd.jerk.x << "," << cmd.jerk.y << "," << cmd.jerk.z << ","
                           << cmd.yaw << "," << cmd.yaw_dot << "," << static_cast<int>(cmd.trajectory_flag)
                           << std::endl;
            }
            csv_writer.close();
        }

        bool getPoseFromTraj(super_utils::Pose &pose) {
            if (machine_state_ != FOLLOW_TRAJ) {
                cout << YELLOW << "[Fsm] Not in FOLLOW_TRAJ state, can't get pose from traj." << RESET << endl;
                return false;
            }
            getOnePositionCommand(pid_cmd_, traj_finish_);
            if (traj_finish_) {
                cout << GREEN << " -- [Fsm] Traj finish." << RESET << endl;
                if (closeToGoal(0.1)) {
                    ChangeState("getPoseFromTraj", WAIT_GOAL);
                } else {
                    ChangeState("getPoseFromTraj", GENERATE_TRAJ);
                }
            }
            pose.first = Vec3f{pid_cmd_.position.x, pid_cmd_.position.y, pid_cmd_.position.z};
            pose.second = eulerToQuaternion(pid_cmd_.attitude.x, pid_cmd_.attitude.y, pid_cmd_.attitude.z);


            /// for checking the trajectory continuty
            static double max_delta_v{0.0};
            static double last_v = pid_cmd_.vel_norm;
            double delta_v = std::abs(pid_cmd_.vel_norm - last_v);
            last_v = pid_cmd_.vel_norm;
            if (delta_v > max_delta_v) {
                max_delta_v = delta_v;
            }
            fmt::print(" -- [Fsm] Cur vel: {}, delta_v: {}, max_delta_v: {}\n", pid_cmd_.vel_norm, delta_v,
                       max_delta_v);
            cmd_logs_.push_back(latest_cmd);
            return true;
        }

        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            super_utils::Vec3f goal_p = Vec3f{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
            super_utils::Quatf goal_q = super_utils::Quatf{msg->pose.orientation.w, msg->pose.orientation.x,
                                                           msg->pose.orientation.y, msg->pose.orientation.z};
            setGoalPosiAndYaw(goal_p, goal_q);
        }

        void init(const rclcpp::Node::SharedPtr nh, const std::string &cfg_path) {
            const rclcpp::QoS qos(rclcpp::QoS(10)
                                          .reliable()
                                          .keep_last(10)
                                          .durability_volatile());

            // 初始化参数读取
            nh_ = nh;
            cfg_ = Config(cfg_path);
            map_ptr_ = std::make_shared<rog_map::ROGMapROS>(nh_, cfg_path);
            // 初始化Planner
            ros_ptr_ = std::make_shared<ros_interface::Ros2Interface>(nh_);
            planner_ptr_ = std::make_shared<SuperPlanner>(cfg_path, ros_ptr_, map_ptr_);
            cmd_pub_ = nh_->create_publisher<mars_quadrotor_msgs::msg::PositionCommand>(cfg_.cmd_topic, qos);
            mpc_cmd_pub_ = nh_->create_publisher<mars_quadrotor_msgs::msg::PolynomialTrajectory>(cfg_.mpc_cmd_topic,
                                                                                                qos);
            px4_cmd_pub_ = nh_->create_publisher<px4ctrl_msgs::msg::Command>(cfg_.px4_cmd_topic, qos);
            path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("fsm/path", qos);

            int cmd_cnt = 0;

            if (cfg_.click_goal_en) {
                goal_cbk_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions so;
                so.callback_group = goal_cbk_group_;
                goal_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
                        cfg_.click_goal_topic,
                        qos,
                        std::bind(&FsmRos2::goalCallback, this, std::placeholders::_1),
                        so);
                cout << YELLOW << " -- [Fsm] CLICKGOAL ENABLE." << RESET << endl;
                cmd_cnt++;
            }

            if (cmd_cnt != 1) {
                cout << YELLOW << " -- [Fsm] CMD INPUT ERROR." << RESET << endl;
                exit(0);
            }

            if (cfg_.timer_en) {
                exec_cbk_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                execution_timer_ = nh_->create_wall_timer(
                        std::chrono::milliseconds(10),
                        std::bind(&FsmRos2::mainFsmTimerCallback, this),
                        exec_cbk_group_
                );

                cmd_cbk_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                cmd_timer_ = nh_->create_wall_timer(
                        std::chrono::milliseconds(10),
                        std::bind(&FsmRos2::pubCmdTimerCallback, this),
                        cmd_cbk_group_
                );
                const int replan_ratems = static_cast<int>(1.0 / cfg_.replan_rate * 1000);
                replan_cbk_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                replan_timer_ = nh_->create_wall_timer(
                        std::chrono::milliseconds(replan_ratems),
                        std::bind(&FsmRos2::replanTimerCallback, this),
                        exec_cbk_group_
                );
            }

            write_time_.open(DEBUG_FILE_DIR("time_consuming.csv"), std::ios::out | std::ios::trunc);
            log_module_time.resize(9);
            for (int i = 0; i < 9; i++) {
                write_time_ << log_time_str[i];
                if (i != 8) {
                    write_time_ << ",";
                }
            }
            write_time_ << endl;
            machine_state_ = INIT;
            system_start_time_ = ros_ptr_->getSimTime();

            pid_cmd_.kx[0] = 5.7;
            pid_cmd_.kx[1] = 5.7;
            pid_cmd_.kx[2] = 4.2;

            pid_cmd_.kv[0] = 3.4;
            pid_cmd_.kv[1] = 3.4;
            pid_cmd_.kv[2] = 4.0;
            
            // 注册动态参数回调
            registerDynamicParameters();
        }

        void pubCmdTimerCallback() {
            if (stop) {
                return;
            }
            if (machine_state_ != FOLLOW_TRAJ && machine_state_ != EMER_STOP) {
                return;
            }


            mars_quadrotor_msgs::msg::PolynomialTrajectory heartbeat;
            getOneHeartBeatMsg(heartbeat, traj_finish_);
            getOnePositionCommand(pid_cmd_, traj_finish_);
            
            // 创建并发布PX4 Command消息
            px4ctrl_msgs::msg::Command px4_cmd;
            px4_cmd.header = pid_cmd_.header;
            px4_cmd.pos[0] = pid_cmd_.position.x;
            px4_cmd.pos[1] = pid_cmd_.position.y;
            px4_cmd.pos[2] = pid_cmd_.position.z;
            px4_cmd.vel[0] = pid_cmd_.velocity.x;
            px4_cmd.vel[1] = pid_cmd_.velocity.y;
            px4_cmd.vel[2] = pid_cmd_.velocity.z;
            px4_cmd.acc[0] = pid_cmd_.acceleration.x;
            px4_cmd.acc[1] = pid_cmd_.acceleration.y;
            px4_cmd.acc[2] = pid_cmd_.acceleration.z;
            px4_cmd.jerk[0] = pid_cmd_.jerk.x;
            px4_cmd.jerk[1] = pid_cmd_.jerk.y;
            px4_cmd.jerk[2] = pid_cmd_.jerk.z;
            px4_cmd.yaw = pid_cmd_.yaw;
            px4_cmd.yawdot = pid_cmd_.yaw_dot;
            // 将欧拉角转换为四元数
            double roll = pid_cmd_.attitude.x;
            double pitch = pid_cmd_.attitude.y;
            double yaw = pid_cmd_.attitude.z;
            
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);
            double cp = cos(pitch * 0.5);
            double sp = sin(pitch * 0.5);
            double cr = cos(roll * 0.5);
            double sr = sin(roll * 0.5);
            
            px4_cmd.quat[0] = cr * cp * cy + sr * sp * sy;  // w
            px4_cmd.quat[1] = sr * cp * cy - cr * sp * sy;  // x
            px4_cmd.quat[2] = cr * sp * cy + sr * cp * sy;  // y
            px4_cmd.quat[3] = cr * cp * sy - sr * sp * cy;  // z
            px4_cmd.type = px4_cmd.THRUST_QUAT;  // 使用推力和四元数类型
            
            mpc_cmd_pub_->publish(heartbeat);
            cmd_pub_->publish(pid_cmd_);
            px4_cmd_pub_->publish(px4_cmd);
            if (traj_finish_) {
                cout << GREEN << " -- [Fsm] Traj finish." << RESET << endl;
                if (closeToGoal(0.1)) {
                    ChangeState("PubCmdCallback", WAIT_GOAL);
                } else {
                    ChangeState("PubCmdCallback", GENERATE_TRAJ);
                }
            }
        }

        void replanTimerCallback() {
            callReplanOnce();
        }

        void mainFsmTimerCallback() {
            callMainFsmOnce();
        }

        /**
         * @brief 动态参数回调函数
         * @param parameters 待更新的参数列表
         * @return 参数设置结果
         * @note 此函数在参数更新时被调用，处理动态参数变化
         */
        rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
                    
            // 更新SuperPlanner的运行时参数（从ROS2参数服务器获取最新参数值）
            if (planner_ptr_) {
                planner_ptr_->updateRuntimeParams(nh_);
            }
                    
            // 调用轨迹优化器的动态参数回调函数（同步到动态配置）
            if (planner_ptr_) {
                auto planner_params = parameters;
                auto exp_traj_result = planner_ptr_->getExpTrajOptDynamicConfig().dynamicParametersCallback(planner_params);
                if (!exp_traj_result.successful) {
                    result.successful = false;
                }
                        
                auto backup_traj_params = parameters;
                auto backup_traj_result = planner_ptr_->getBackupTrajOptDynamicConfig().dynamicParametersCallback(backup_traj_params);
                if (!backup_traj_result.successful) {
                    result.successful = false;
                }
                
                auto yaw_traj_params = parameters;
                auto yaw_traj_result = planner_ptr_->getYawTrajOptDynamicConfig().dynamicParametersCallback(yaw_traj_params);
                if (!yaw_traj_result.successful) {
                    result.successful = false;
                }
                
                // 添加Astar动态参数回调
                auto astar_params = parameters;
                auto astar_result = planner_ptr_->getAstarDynamicConfig().dynamicParametersCallback(astar_params);
                if (!astar_result.successful) {
                    result.successful = false;
                }
                
                // 添加CorridorGenerator动态参数回调
                auto corridor_params = parameters;
                auto corridor_result = planner_ptr_->getCorridorGenDynamicConfig().dynamicParametersCallback(corridor_params);
                if (!corridor_result.successful) {
                    result.successful = false;
                }
                        
                // 立即更新优化器配置以应用最新参数
                if (planner_ptr_) {
                    if (planner_ptr_->getExpTrajOpt()) {
                        planner_ptr_->getExpTrajOpt()->updateOptimizerConfig();
                    }
                    if (planner_ptr_->getBackupTrajOpt()) {
                        planner_ptr_->getBackupTrajOpt()->updateOptimizerConfig();
                    }
                    if (planner_ptr_->getYawTrajOpt()) {
                        planner_ptr_->getYawTrajOpt()->updateOptimizerConfig();
                    }
                    if (planner_ptr_->getAstar()) {
                        planner_ptr_->getAstar()->updateOptimizerConfig();
                    }
                    if (planner_ptr_->getCorridorGenerator()) {
                        planner_ptr_->getCorridorGenerator()->updateOptimizerConfig();
                    }
                }
            }
                    
            return result;
        }

        /**
         * @brief 注册动态参数回调
         * @note 在初始化时调用此函数注册参数回调
         */
        void registerDynamicParameters() {
            if (!planner_ptr_) return;
            const auto& planner_cfg = planner_ptr_->getConfig();

            // 声明并初始化参数，使用从YAML加载的配置值作为默认值
            nh_->declare_parameter("traj_opt.boundary.max_vel", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.max_vel));
            nh_->declare_parameter("traj_opt.boundary.max_acc", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.max_acc));
            nh_->declare_parameter("traj_opt.boundary.max_jerk", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.max_jerk));
            
            // 探索轨迹优化参数
            nh_->declare_parameter("traj_opt.exp_traj.pos_constraint_type", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.pos_constraint_type));
            nh_->declare_parameter("traj_opt.exp_traj.block_energy_cost", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.block_energy_cost));
            nh_->declare_parameter("traj_opt.exp_traj.opt_accuracy", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.opt_accuracy));
            nh_->declare_parameter("traj_opt.exp_traj.smooth_eps", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.smooth_eps));
            nh_->declare_parameter("traj_opt.exp_traj.integral_reso", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.integral_reso));
            nh_->declare_parameter("traj_opt.exp_traj.penna_t", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.penna_t));
            nh_->declare_parameter("traj_opt.exp_traj.penna_pos", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.penna_pos));
            nh_->declare_parameter("traj_opt.exp_traj.penna_vel", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.penna_vel));
            nh_->declare_parameter("traj_opt.exp_traj.penna_acc", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.penna_acc));
            nh_->declare_parameter("traj_opt.exp_traj.penna_jerk", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.penna_jerk));
            nh_->declare_parameter("traj_opt.exp_traj.penna_attract", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.penna_attract));
            nh_->declare_parameter("traj_opt.exp_traj.penna_omg", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.penna_omg));
            nh_->declare_parameter("traj_opt.exp_traj.penna_thr", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.penna_thr));
            
            // 备份轨迹优化参数
            nh_->declare_parameter("traj_opt.backup_traj.uniform_time_en", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.uniform_time_en));
            nh_->declare_parameter("traj_opt.backup_traj.pos_constraint_type", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.pos_constraint_type));
            nh_->declare_parameter("traj_opt.backup_traj.piece_num", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.piece_num));
            nh_->declare_parameter("traj_opt.backup_traj.block_energy_cost", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.block_energy_cost));
            nh_->declare_parameter("traj_opt.backup_traj.opt_accuracy", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.opt_accuracy));
            nh_->declare_parameter("traj_opt.backup_traj.smooth_eps", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.smooth_eps));
            nh_->declare_parameter("traj_opt.backup_traj.integral_reso", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.integral_reso));
            nh_->declare_parameter("traj_opt.backup_traj.penna_t", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.penna_t));
            nh_->declare_parameter("traj_opt.backup_traj.penna_ts", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.penna_ts));
            nh_->declare_parameter("traj_opt.backup_traj.penna_pos", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.penna_pos));
            nh_->declare_parameter("traj_opt.backup_traj.penna_vel", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.penna_vel));
            nh_->declare_parameter("traj_opt.backup_traj.penna_acc", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.penna_acc));
            nh_->declare_parameter("traj_opt.backup_traj.penna_jerk", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.penna_jerk));
            nh_->declare_parameter("traj_opt.backup_traj.penna_attract", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.penna_attract));
            nh_->declare_parameter("traj_opt.backup_traj.penna_omg", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.penna_omg));
            nh_->declare_parameter("traj_opt.backup_traj.penna_thr", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.penna_thr));
            nh_->declare_parameter("traj_opt.backup_traj.penna_max_acc_thr", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.max_acc_thr));
            nh_->declare_parameter("traj_opt.backup_traj.penna_min_acc_thr", rclcpp::ParameterValue(planner_cfg.back_traj_cfg.min_acc_thr));
            
            // 偏航轨迹优化参数
            nh_->declare_parameter("traj_opt.yaw_traj.yaw_dot_max", rclcpp::ParameterValue(planner_cfg.yaw_dot_max));
            
            // 其他轨迹优化边界参数
            nh_->declare_parameter("traj_opt.boundary.max_omg", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.max_omg));
            nh_->declare_parameter("traj_opt.boundary.max_acc_thr", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.max_acc_thr));
            nh_->declare_parameter("traj_opt.boundary.min_acc_thr", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.min_acc_thr));
            nh_->declare_parameter("traj_opt.boundary.penna_margin", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.penna_margin));
            
            // 轨迹优化开关 (路径与YAML保持一致)
            nh_->declare_parameter("traj_opt.switch.save_log_en", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.save_log_en));
            nh_->declare_parameter("traj_opt.switch.print_optimizer_log", rclcpp::ParameterValue(planner_cfg.exp_traj_cfg.print_optimizer_log));
            
            // 规划器策略开关与全局参数
            nh_->declare_parameter("super_planner.backup_traj_en", rclcpp::ParameterValue(planner_cfg.backup_traj_en));
            nh_->declare_parameter("super_planner.use_fov_cut", rclcpp::ParameterValue(planner_cfg.use_fov_cut));
            nh_->declare_parameter("super_planner.print_log", rclcpp::ParameterValue(planner_cfg.print_log));
            nh_->declare_parameter("super_planner.goal_vel_en", rclcpp::ParameterValue(planner_cfg.goal_vel_en));
            nh_->declare_parameter("super_planner.goal_yaw_en", rclcpp::ParameterValue(planner_cfg.goal_yaw_en));
            nh_->declare_parameter("super_planner.visual_process", rclcpp::ParameterValue(planner_cfg.visual_process));
            nh_->declare_parameter("super_planner.frontend_in_known_free", rclcpp::ParameterValue(planner_cfg.frontend_in_known_free));
            
            nh_->declare_parameter("super_planner.planning_horizon", rclcpp::ParameterValue(planner_cfg.planning_horizon));
            nh_->declare_parameter("super_planner.receding_dis", rclcpp::ParameterValue(planner_cfg.receding_dis));
            nh_->declare_parameter("super_planner.sensing_horizon", rclcpp::ParameterValue(planner_cfg.sensing_horizon));
            nh_->declare_parameter("super_planner.replan_forward_dt", rclcpp::ParameterValue(planner_cfg.replan_forward_dt));
            nh_->declare_parameter("super_planner.yaw_mode", rclcpp::ParameterValue(planner_cfg.yaw_mode));
            nh_->declare_parameter("super_planner.mpc_horizon", rclcpp::ParameterValue(planner_cfg.mpc_horizon));
            nh_->declare_parameter("super_planner.safe_corridor_line_max_length", rclcpp::ParameterValue(planner_cfg.safe_corridor_line_max_length));
            
            // 走廊生成相关参数 (路径与YAML保持一致)
            nh_->declare_parameter("super_planner.corridor_bound_dis", rclcpp::ParameterValue(planner_cfg.corridor_bound_dis));
            nh_->declare_parameter("super_planner.corridor_line_max_length", rclcpp::ParameterValue(planner_cfg.corridor_line_max_length));
            nh_->declare_parameter("super_planner.min_overlap_threshold", rclcpp::ParameterValue(planner_cfg.min_overlap_threshold));
            nh_->declare_parameter("super_planner.robot_r", rclcpp::ParameterValue(planner_cfg.robot_r));
            nh_->declare_parameter("super_planner.obs_skip_num", rclcpp::ParameterValue(planner_cfg.obs_skip_num));
            nh_->declare_parameter("super_planner.iris_iter_num", rclcpp::ParameterValue(planner_cfg.iris_iter_num));
            
            // 虚拟高度参数 (路径与YAML保持一致)
            nh_->declare_parameter("rog_map.virtual_ground_height", rclcpp::ParameterValue(planner_cfg.virtual_ground_height));
            nh_->declare_parameter("rog_map.virtual_ceil_height", rclcpp::ParameterValue(planner_cfg.virtual_ceil_height));
            
            // Astar相关参数
            nh_->declare_parameter("astar.heu_type", rclcpp::ParameterValue(planner_cfg.astar_heu_type));
            nh_->declare_parameter("astar.allow_diag", rclcpp::ParameterValue(planner_cfg.astar_allow_diag));
            nh_->declare_parameter("astar.debug_visualization_en", rclcpp::ParameterValue(planner_cfg.astar_debug_visualization_en));
            
            // 添加回调用于动态参数
            dyn_params_handler_ = nh_->add_on_set_parameters_callback(
                [this](std::vector<rclcpp::Parameter> parameters) {
                    return this->dynamicParametersCallback(parameters);
                });
        }

    };
}

#endif //SRC_FSM_ROS2_HPP
