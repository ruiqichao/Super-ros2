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

#include <path_search/astar.h>

using namespace color_text;
using namespace super_utils;

namespace path_search {
    using namespace rog_map;

    /**
     * @brief A*算法构造函数
     * @param cfg_path 配置文件路径
     * @param ros_ptr ROS接口指针
     * @param rm ROG地图指针
     */
    Astar::Astar(const std::string &cfg_path,
                 const ros_interface::RosInterface::Ptr &ros_ptr,
                 rog_map::ROGMapROS::Ptr rm) : ros_ptr_(ros_ptr), map_ptr_(rm) {
        cfg_ = PathSearchConfig(cfg_path);
        cout << rog_map::GREEN << " -- [RM] Init Astar-map." << rog_map::RESET << endl;

        // 初始化网格节点缓冲区
        int map_buffer_size = cfg_.map_voxel_num(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2);
        grid_node_buffer_.resize(map_buffer_size);
        for (auto &i: grid_node_buffer_) {
            i = new GridNode;
            i->rounds = 0;
        }

        cout << rog_map::BLUE << "\tmap index size: " << cfg_.map_size_i.transpose() << rog_map::RESET << endl;
        cout << rog_map::BLUE << "\tmap vox_num: " << cfg_.map_voxel_num.transpose() << rog_map::RESET << endl;

        // 预先生成排序的邻域点列表
        int test_num = 100;
        for (int i = -test_num; i <= test_num; i++) {
            for (int j = -test_num; j <= test_num; j++) {
                for (int k = -test_num; k <= test_num; k++) {
                    rog_map::Vec3i delta(i, j, k);
                    sorted_pts.push_back(delta);
                }
            }
        }
        // 按照距离排序邻域点
        sort(sorted_pts.begin(), sorted_pts.end(),
             [](const rog_map::Vec3i &pt1, const rog_map::Vec3i &pt2) {
                 double dist1 = pt1.x() * pt1.x() + pt1.y() * pt1.y() + pt1.z() * pt1.z();
                 double dist2 = pt2.x() * pt2.x() + pt2.y() * pt2.y() + pt2.z() * pt2.z();
                 return dist1 < dist2;
             });
    }

    /**
     * @brief 设置A*搜索参数
     * @param start_pt 起始点
     * @param goal_pt 目标点
     * @param flag 搜索标志
     * @param searching_horizon 搜索范围
     * @return 返回码
     */
    RET_CODE
    Astar::setup(const Vec3f &start_pt, const Vec3f &goal_pt, const int &flag, const double &searching_horizon) {
        md_.start_pt = start_pt;
        md_.goal_pt = goal_pt;
        md_.mission_rcv_WT = ros_ptr_->getSimTime();
        md_.searching_horizon = searching_horizon;

        // 解析搜索标志
        md_.use_inf_map = flag & ON_INF_MAP;
        md_.use_prob_map = flag & ON_PROB_MAP;
        md_.unknown_as_occ = flag & UNKNOWN_AS_OCCUPIED;
        md_.unknown_as_free = flag & UNKNOWN_AS_FREE;
        md_.use_inf_neighbor = flag & USE_INF_NEIGHBOR;
        if (flag & DONT_USE_INF_NEIGHBOR) {
            md_.use_inf_neighbor = false;
        }

        // 检查地图类型配置
        if ((md_.use_inf_map && md_.use_prob_map) ||
            (!md_.use_inf_map && !md_.use_prob_map)) {
            cout << YELLOW << " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                    << ": cannot use both inf map and prob map." << RESET << endl;
            return INIT_ERROR;
        }

        // 检查未知区域处理方式
        if (md_.unknown_as_occ && md_.unknown_as_free) {
            cout << YELLOW << " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                    << ": cannot use both unknown_as_occupied and unknown_as_free." << RESET << endl;
            return INIT_ERROR;
        }

        // 设置地图分辨率
        if (md_.use_prob_map) {
            md_.resolution = map_ptr_->getResolution();
        } else {
            md_.resolution = map_ptr_->getInfResolution();
        }

        // 设置局部地图中心
        if (searching_horizon > 0) {
            md_.local_map_center_d = start_pt;
        } else {
            md_.local_map_center_d = (start_pt + goal_pt) / 2;
        }

        // 计算局部地图边界
        posToGlobalIndex(md_.local_map_center_d, md_.local_map_center_id_g);
        md_.local_map_min_d = md_.local_map_center_d - md_.resolution * cfg_.map_size_i.cast<double>();
        md_.local_map_max_d = md_.local_map_center_d + md_.resolution * cfg_.map_size_i.cast<double>();;

        // 可视化边界框
        if (cfg_.visual_process || cfg_.debug_visualization_en) {
            ros_ptr_->vizAstarBoundingBox(md_.local_map_min_d, md_.local_map_max_d);
        }

        return SUCCESS;
    }

    /**
     * @brief 计算启发式函数值
     * @param node1 节点1
     * @param node2 节点2
     * @param type 启发式类型(DIAG/MANH/EUCL)
     * @return 启发式值
     */
    double Astar::getHeu(GridNodePtr node1, GridNodePtr node2, int type) const {
        switch (type) {
            case DIAG: {
                // 对角线距离
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                double h = 0.0;
                int diag = std::min(std::min(dx, dy), dz);
                dx -= diag;
                dy -= diag;
                dz -= diag;

                if (dx == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * std::abs(dy - dz);
                }
                if (dy == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * std::abs(dx - dz);
                }
                if (dz == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * std::abs(dx - dy);
                }
                return tie_breaker_ * h;
            }
            case MANH: {
                // 曼哈顿距离
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                return tie_breaker_ * (dx + dy + dz);
            }
            case EUCL: {
                // 欧几里得距离
                return tie_breaker_ * (node2->id_g - node1->id_g).norm();
            }
            default: {
                fmt::print(fg(fmt::color::indian_red), " -- [A*] Wrong hue type.\n");
                return 0;
            }
        }
    }

    /**
     * @brief 获取局部索引的哈希值
     * @param id_in 输入索引
     * @return 哈希值
     */
    int Astar::getLocalIndexHash(const Vec3i &id_in) const {
        rog_map::Vec3i id = id_in - md_.local_map_center_id_g + cfg_.map_size_i;
        return id(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2) +
               id(1) * cfg_.map_voxel_num(2) +
               id(2);
    }

    /**
     * @brief 位置转全局索引
     * @param pos 位置
     * @param id_g 全局索引(输出)
     */
    void Astar::posToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const {
        if (md_.use_inf_map) {
            map_ptr_->infMapPosToGlobalIndex(pos, id_g);
        } else if (md_.use_prob_map) {
            map_ptr_->probMapPosToGlobalIndex(pos, id_g);
        } else {
            throw std::runtime_error(" -- [A*] Map type not defined.");
        }
    }

    /**
     * @brief 全局索引转位置
     * @param id_g 全局索引
     * @param pos 位置(输出)
     */
    void Astar::globalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const {
        if (md_.use_inf_map) {
            map_ptr_->infMapGlobalIndexToPos(id_g, pos);
        } else if (md_.use_prob_map) {
            map_ptr_->probMapGlobalIndexToPos(id_g, pos);
        } else {
            throw std::runtime_error(" -- [A*] Map type not defined.");
        }
    }

    /**
     * @brief 判断位置是否在局部地图内
     * @param pos 位置
     * @return 是否在地图内
     */
    bool Astar::insideLocalMap(const rog_map::Vec3f &pos) const {
        rog_map::Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        return insideLocalMap(id_g);
    }

    /**
     * @brief 判断索引是否在局部地图内
     * @param id_g 全局索引
     * @return 是否在地图内
     */
    bool Astar::insideLocalMap(const rog_map::Vec3i &id_g) const {
        rog_map::Vec3i delta = id_g - md_.local_map_center_id_g;
        if (fabs(delta.x()) > cfg_.map_size_i.x() ||
            fabs(delta.y()) > cfg_.map_size_i.y() ||
            fabs(delta.z()) > cfg_.map_size_i.z()) {
            return false;
        }
        return true;
    }

    /**
     * @brief 设置可视化进程使能
     * @param en 使能标志
     */
    void Astar::setVisualProcessEn(const bool &en) {
        cfg_.visual_process = en;
    }

    /**
     * @brief 回溯路径
     * @param current 当前节点
     * @param path 路径(输出)
     */
    void Astar::retrievePath(GridNodePtr current, vector<GridNodePtr> &path) {
        path.push_back(current);
        while (current->father_ptr != NULL) {
            current = current->father_ptr;
            path.push_back(current);
        }
    }

    /**
     * @brief 节点路径转点路径
     * @param node_path 节点路径
     * @param point_path 点路径(输出)
     */
    void Astar::ConvertNodePathToPointPath(const vector<GridNodePtr> &node_path, rog_map::vec_Vec3f &point_path) {
        point_path.clear();
        for (auto ptr: node_path) {
            rog_map::Vec3f pos;
            globalIndexToPos(ptr->id_g, pos);
            point_path.push_back(pos);
        }
        reverse(point_path.begin(), point_path.end());
    }

    /**
     * @brief 设置精细的膨胀邻域
     * @param neighbor_step 邻域步长
     */
    void Astar::setFineInfNeighbors(const int &neighbor_step) {
        neighbor_list.clear();
        for (int i = -neighbor_step; i <= neighbor_step; i++) {
            for (int j = -neighbor_step; j <= neighbor_step; j++) {
                for (int k = -neighbor_step; k <= neighbor_step; k++) {
                    if (i == 0 && j == 0 && k == 0) {
                        continue;
                    }
                    if (i * i + j * j + k * k > neighbor_step * neighbor_step) {
                        continue;
                    }
                    neighbor_list.emplace_back(i, j, k);
                }
            }
        }
    }


    /**
     * @brief 点到点路径搜索
     * @param start_pt 起始点
     * @param end_pt 终止点
     * @param flag 搜索标志
     * @param searching_horizon 搜索范围
     * @param out_path 输出路径
     * @param time_out 超时时间
     * @return 返回码
     */
    RET_CODE Astar::pointToPointPathSearch(const rog_map::Vec3f &start_pt, const rog_map::Vec3f &end_pt,
                                           const int &flag, const double &searching_horizon,
                                           rog_map::vec_Vec3f &out_path, const double &time_out) {
        // 初始化设置
        RET_CODE setup_ret = setup(start_pt, end_pt, flag, searching_horizon);
        if (setup_ret != SUCCESS) {
            return setup_ret;
        }
        out_path.clear();
        double time_1 = ros_ptr_->getSimTime();
        ++rounds_;

        // 处理起点和终点在局部地图外的情况
        rog_map::Vec3f hit_pt;
        rog_map::Vec3f local_start_pt, local_end_pt;
        bool start_pt_out_local_map = false;

        local_start_pt = start_pt;
        local_end_pt = end_pt;

        // 检查起点是否在局部地图外
        if (!insideLocalMap(start_pt)) {
            ros_ptr_->warn(" -- [A*] Start point [{}] is out of local map, find a waypoint to the map edge.",
                           start_pt.transpose());
            if (rog_map::lineIntersectBox(start_pt, md_.local_map_center_d, md_.local_map_min_d,
                                          md_.local_map_max_d, hit_pt)) {
                rog_map::Vec3f dir = (hit_pt - start_pt).normalized();
                double dis = (hit_pt - start_pt).norm();
                local_start_pt = start_pt + dir * (dis + md_.resolution * 2);
                start_pt_out_local_map = true;
                if (!map_ptr_->getNearestInfCellNot(OCCUPIED, local_start_pt,
                                                    local_start_pt, 3.0)) {
                    if (cfg_.visual_process || cfg_.debug_visualization_en) {
                        ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(),
                                                 "local_start_pt",
                                                 0.3, 1);
                    }
                    cout << rog_map::RED <<
                            " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                            << " : start point deeply occupied, cannot find feasible path.\n" << rog_map::RESET << endl;
                    return INIT_ERROR;
                }
            }
        }

        // 检查终点是否在局部地图外
        if (!insideLocalMap(end_pt)) {
            rog_map::Vec3f seed_pt = start_pt_out_local_map ? md_.local_map_center_d : start_pt;
            if (rog_map::lineIntersectBox(end_pt, seed_pt, md_.local_map_min_d, md_.local_map_max_d,
                                          hit_pt)) {
                rog_map::Vec3f dir = (hit_pt - end_pt).normalized();
                double dis = (hit_pt - end_pt).norm();
                local_end_pt = end_pt + dir * (dis + 2.5);

                if (!map_ptr_->getNearestInfCellNot(OCCUPIED, local_end_pt, local_end_pt, 2.0)) {
                    ros_ptr_->error(
                        " -- [A*] Error with: {}, Goal point [{}] deeply occupied, cannot find feasible path.",
                        RET_CODE_STR[INIT_ERROR],
                        local_end_pt.transpose());
                    if (cfg_.visual_process || cfg_.debug_visualization_en) {
                        ros_ptr_->vizAstarPoints(local_end_pt, Color::Red(), "local_end_pt",
                                                 0.5,
                                                 1);
                    }
                    return INIT_ERROR;
                }
            }
        }

        // 可视化局部起点和终点
        if (cfg_.visual_process) {
            ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt",
                                     0.3,
                                     1);
            ros_ptr_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3,
                                     1);
        }

        // 转换为索引
        rog_map::Vec3i start_idx, end_idx;
        posToGlobalIndex(local_start_pt, start_idx);
        posToGlobalIndex(local_end_pt, end_idx);
        if (cfg_.visual_process) {
            ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt", 0.3, 1);
            ros_ptr_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3, 1);
        }

        // 检查索引是否在局部地图内
        if (!insideLocalMap(start_idx) || !insideLocalMap(end_idx)) {
            cout << rog_map::RED << " -- [RM] Start or end point is out of local map, which should not happen." <<
                    rog_map::RESET
                    << endl;
            ros_ptr_->error(" -- [RM] Start [{}] or end point [{}] is out of local map, which should not happen.",
                            local_start_pt.transpose(),
                            local_end_pt.transpose()
            );
            if (cfg_.visual_process || cfg_.debug_visualization_en) {
                ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt", 0.3, 1);
                ros_ptr_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3, 1);
            }
            return INIT_ERROR;
        }


        // 初始化起点和终点节点
        GridNodePtr startPtr = grid_node_buffer_[getLocalIndexHash(start_idx)];
        GridNodePtr endPtr = grid_node_buffer_[getLocalIndexHash(end_idx)];
        endPtr->id_g = end_idx;

        // 初始化优先队列
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set;
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, FrontierComparator> frontier_queue;


        GridNodePtr neighborPtr = NULL;
        GridNodePtr current = NULL;

        // 初始化起点
        startPtr->id_g = start_idx;
        startPtr->rounds = rounds_;
        startPtr->distance_score = 0;
        startPtr->total_score = getHeu(startPtr, endPtr, cfg_.heu_type);
        startPtr->state = GridNode::OPENSET;
        startPtr->father_ptr = NULL;
        open_set.push(startPtr);
        int num_iter = 0;
        vector<GridNodePtr> node_path;

        // 可视化起点和终点
        if (cfg_.visual_process) {
            ros_ptr_->vizAstarPoints(
                start_pt,
                Color::Green(),
                "start_pt",
                0.3, 1);
            ros_ptr_->vizAstarPoints(
                end_pt,
                Color::Blue(),
                "goal_pt",
                0.3, 1);
        }

        // A*主循环
        while (!open_set.empty()) {
            num_iter++;
            current = open_set.top();
            open_set.pop();

            // 可视化搜索过程
            if (cfg_.visual_process) {
                rog_map::Vec3f local_pt;
                globalIndexToPos(current->id_g, local_pt);
                ros_ptr_->vizAstarPoints(
                    local_pt,
                    Color(Color::Pink(), 0.5),
                    "astar_process",
                    0.1);
                usleep(1000);
            }

            // 检查是否到达目标
            if (current->id_g(0) == endPtr->id_g(0) &&
                current->id_g(1) == endPtr->id_g(1) &&
                current->id_g(2) == endPtr->id_g(2)) {
                retrievePath(current, node_path);
                if (start_pt_out_local_map) {
                    rog_map::Vec3i start_idx_g;
                    posToGlobalIndex(start_pt, start_idx_g);
                    GridNodePtr temp_ptr(new GridNode);
                    temp_ptr->id_g = start_idx_g;
                    node_path.push_back(temp_ptr);
                }
                ConvertNodePathToPointPath(node_path, out_path);
                return REACH_GOAL;
            }

            // 检查距离终止条件
            if (searching_horizon > 0 && current->distance_score > searching_horizon / md_.resolution) {
                GridNodePtr local_goal = current;
                if (md_.unknown_as_occ && !frontier_queue.empty()) {
                    local_goal = frontier_queue.top()->father_ptr;
                    if (local_goal->distance_to_goal > current->distance_to_goal) {
                        local_goal = current;
                    }
                }
                retrievePath(local_goal, node_path);
                if (start_pt_out_local_map) {
                    node_path.push_back(startPtr);
                }
                ConvertNodePathToPointPath(node_path, out_path);
                return REACH_HORIZON;
            }


            current->state = GridNode::CLOSEDSET;

            // 遍历邻居节点
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        // 不允许对角线移动
                        if (!cfg_.allow_diag &&
                            (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1)) {
                            continue;
                        }

                        rog_map::Vec3i neighborIdx;
                        rog_map::Vec3f neighborPos;
                        neighborIdx(0) = (current->id_g)(0) + dx;
                        neighborIdx(1) = (current->id_g)(1) + dy;
                        neighborIdx(2) = (current->id_g)(2) + dz;
                        globalIndexToPos(neighborIdx, neighborPos);

                        if (!insideLocalMap(neighborIdx)) {
                            continue;
                        }

                        // 获取邻居节点类型
                        rog_map::GridType neighbor_type;

                        if (md_.use_inf_map) {
                            neighbor_type = map_ptr_->getInfGridType(neighborPos);
                        } else {
                            if (!md_.use_inf_neighbor) {
                                neighbor_type = map_ptr_->getGridType(neighborPos);
                            } else {
                                // 使用概率地图，但查询当前节点的所有邻居
                                // 如果有一个邻居被占用，则该邻居被占用
                                neighbor_type = neighborHaveOne(OCCUPIED, neighborIdx) ? OCCUPIED : UNDEFINED;
                                // 如果有一个已知的自由邻居，则该邻居是已知自由的
                                if (md_.unknown_as_occ && neighbor_type != OCCUPIED) {
                                    neighbor_type = neighborHaveOne(KNOWN_FREE, neighborIdx) ? KNOWN_FREE : UNKNOWN;
                                }
                            }
                        }

                        if (neighbor_type == OCCUPIED || neighbor_type == OUT_OF_MAP) {
                            continue;
                        }

                        if (md_.unknown_as_occ && neighbor_type == UNKNOWN) {
                            continue;
                        }

                        neighborPtr = grid_node_buffer_[getLocalIndexHash(neighborIdx)];
                        if (neighborPtr == nullptr) {
                            cout << rog_map::RED << " -- [RM] neighborPtr is null, which should not happen." <<
                                    rog_map::RESET
                                    << endl;
                            continue;
                        }
                        neighborPtr->id_g = neighborIdx;

                        bool flag_explored = neighborPtr->rounds == rounds_;

                        if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                            continue;
                        }

                        // 处理前沿节点（未知区域）
                        if (md_.unknown_as_occ && neighbor_type == UNKNOWN && neighborPtr) {
                            // 前沿被记录但不扩展
                            neighborPtr->father_ptr = current;
                            rog_map::Vec3f pos;
                            globalIndexToPos(neighborIdx, pos);
                            neighborPtr->distance_to_goal = getHeu(neighborPtr, endPtr, cfg_.heu_type);
                            frontier_queue.push(neighborPtr);
                            continue;
                        }

                        // 更新节点信息
                        neighborPtr->rounds = rounds_;
                        double distance_score = sqrt(dx * dx + dy * dy + dz * dz);
                        distance_score = current->distance_score + distance_score;
                        rog_map::Vec3f pos;
                        globalIndexToPos(neighborIdx, pos);
                        double heu_score = getHeu(neighborPtr, endPtr, cfg_.heu_type);

                        if (!flag_explored) {
                            // 发现新节点
                            neighborPtr->state = GridNode::OPENSET;
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->distance_to_goal = heu_score;
                            neighborPtr->total_score = distance_score + heu_score;
                            open_set.push(neighborPtr);
                        } else if (distance_score < neighborPtr->distance_score) {
                            // 更新更优路径
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->distance_to_goal = heu_score;
                            neighborPtr->total_score = distance_score + heu_score;
                        }
                    }
            // 检查超时
            double time_2 = ros_ptr_->getSimTime();
            if (!cfg_.visual_process && (time_2 - time_1) > time_out) {
                fmt::print(fg(fmt::color::indian_red),
                           "Failed in A star path searching !!! {} seconds time limit exceeded.\n", time_out);
                return TIME_OUT;
            }
        }

        double time_2 = ros_ptr_->getSimTime();
        if ((time_2 - time_1) > time_out) {
            fmt::print(fg(fmt::color::indian_red), "Time consume in A star path finding is {} s, iter={}.\n",
                       (time_2 - time_1),
                       num_iter);
            return NO_PATH;
        }

        // 处理前沿队列
        if (md_.unknown_as_occ && !frontier_queue.empty()) {
            GridNodePtr local_goal;
            while (!frontier_queue.empty()) {
                local_goal = frontier_queue.top();
                frontier_queue.pop();
                rog_map::Vec3f pos;
                globalIndexToPos(local_goal->id_g, pos);
                if ((pos - start_pt).norm() < 1.0) {
                    continue;
                }
                break;
            }
            if (frontier_queue.empty()) {
                cout << rog_map::RED << " -- [A*] Frontier queue is empty, return." << rog_map::RESET << endl;
                return NO_PATH;
            }
            retrievePath(local_goal, node_path);
            if (start_pt_out_local_map) {
                node_path.push_back(startPtr);
            }
            ConvertNodePathToPointPath(node_path, out_path);
            cout << rog_map::BLUE << "Frontier queue: " << frontier_queue.size() << endl;
            return REACH_HORIZON;
        }
        ros_ptr_->error(" -- [A*] Point to point path cannot find path with iter num: {}, return.", num_iter);
        return NO_PATH;
    }

    /**
     * @brief 逃逸路径搜索（从障碍物区域逃逸）
     * @param start_pt 起始点
     * @param flag 搜索标志
     * @param out_path 输出路径
     * @return 返回码
     */
    RET_CODE Astar::escapePathSearch(const rog_map::Vec3f &start_pt, const int flag, rog_map::vec_Vec3f &out_path) {
        Vec3f tmp;
        RET_CODE setup_ret = setup(start_pt, tmp, flag, 999);
        if (setup_ret != SUCCESS) {
            return setup_ret;
        }

        double time_1 = ros_ptr_->getSimTime();
        ++rounds_;

        posToGlobalIndex(md_.local_map_center_d, md_.local_map_center_id_g);

        // 检查起点
        if (!insideLocalMap(start_pt) ||
            !map_ptr_->insideLocalMap(start_pt)) {
            fmt::print(fg(fmt::color::indian_red), " -- [A*] {}: escape start point is not inside local map.\n",
                       RET_CODE_STR[INIT_ERROR].c_str());
            return INIT_ERROR;
        }
        rog_map::Vec3f local_start_pt = start_pt;

        // 寻找最近的非占用单元格
        if (!map_ptr_->getNearestCellNot(OCCUPIED, start_pt, local_start_pt, 3.0)) {
            cout << rog_map::RED <<
                    " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                    << " : escape start point deeply occupied, cannot find feasible path.\n" << rog_map::RESET << endl;
            return INIT_ERROR;
        }

        rog_map::Vec3i start_idx;
        posToGlobalIndex(local_start_pt, start_idx);

        GridNodePtr startPtr = grid_node_buffer_[getLocalIndexHash(start_idx)];
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set;
        GridNodePtr neighborPtr = NULL;
        GridNodePtr current = NULL;

        // 初始化起点
        startPtr->id_g = start_idx;
        startPtr->rounds = rounds_;
        startPtr->distance_score = 0;
        startPtr->total_score = 0;
        startPtr->state = GridNode::OPENSET;
        startPtr->father_ptr = NULL;
        open_set.push(startPtr);
        int num_iter = 0;

        vector<GridNodePtr> node_path;

        if (cfg_.visual_process) {
            ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt",
                                     0.05,
                                     1);
        }

        // 逃逸搜索主循环
        while (!open_set.empty()) {
            num_iter++;
            current = open_set.top();
            open_set.pop();

            if (cfg_.visual_process) {
                rog_map::Vec3f local_pt;
                globalIndexToPos(current->id_g, local_pt);
                ros_ptr_->vizAstarPoints(local_pt,
                                         Color::Pink(),
                                         "astar_process",
                                         0.05);
                usleep(1000);
            }

            rog_map::Vec3f cur_pos;
            globalIndexToPos(current->id_g, cur_pos);
            rog_map::GridType cur_inf_type = map_ptr_->getInfGridType(cur_pos);

            // 检查是否到达自由空间（未知区域视为占用）
            if (md_.unknown_as_occ && cur_inf_type != OCCUPIED && cur_inf_type != UNKNOWN) {
                retrievePath(current, node_path);
                ConvertNodePathToPointPath(node_path, out_path);
                return REACH_HORIZON;
            }

            // 检查是否到达自由空间（未知区域视为自由）
            if (md_.unknown_as_free && cur_inf_type != OCCUPIED) {
                retrievePath(current, node_path);
                ConvertNodePathToPointPath(node_path, out_path);
                return REACH_HORIZON;
            }

            current->state = GridNode::CLOSEDSET;

            // 遍历26邻域
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        rog_map::Vec3i neighborIdx;
                        rog_map::Vec3f neighborPos;
                        neighborIdx(0) = (current->id_g)(0) + dx;
                        neighborIdx(1) = (current->id_g)(1) + dy;
                        neighborIdx(2) = (current->id_g)(2) + dz;
                        globalIndexToPos(neighborIdx, neighborPos);

                        if (!map_ptr_->insideLocalMap(neighborPos) ||
                            !insideLocalMap(neighborIdx)) {
                            continue;
                        }

                        // 获取邻居类型
                        rog_map::GridType neighbor_type;
                        if (md_.use_inf_map) {
                            neighbor_type = map_ptr_->getInfGridType(neighborPos);
                        } else {
                            neighbor_type = map_ptr_->getGridType(neighborPos);
                        }

                        if (neighbor_type == OCCUPIED || neighbor_type == OUT_OF_MAP) {
                            continue;
                        }

                        if (md_.unknown_as_occ && neighbor_type == UNKNOWN) {
                            continue;
                        }

                        neighborPtr = grid_node_buffer_[getLocalIndexHash(neighborIdx)];
                        if (neighborPtr == nullptr) {
                            cout << rog_map::RED << " -- [RM] neighborPtr is null, which should not happen" <<
                                    rog_map::RESET << endl;
                            continue;
                        }
                        neighborPtr->id_g = neighborIdx;

                        bool flag_explored = neighborPtr->rounds == rounds_;

                        if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                            continue;
                        }

                        // 更新节点信息
                        neighborPtr->rounds = rounds_;
                        double distance_score = sqrt(dx * dx + dy * dy + dz * dz);
                        distance_score = current->distance_score + distance_score;
                        rog_map::Vec3f pos;
                        globalIndexToPos(neighborIdx, pos);
                        double heu_score = 0; // 逃逸搜索不使用启发式

                        if (!flag_explored) {
                            // 发现新节点
                            neighborPtr->state = GridNode::OPENSET;
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->total_score = distance_score + heu_score;
                            open_set.push(neighborPtr);
                        } else if (distance_score < neighborPtr->distance_score) {
                            // 更新更优路径
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->total_score = distance_score + heu_score;
                        }
                    }
            // 检查超时
            double time_2 = ros_ptr_->getSimTime();
            if (!cfg_.visual_process && (time_2 - time_1) > 0.2) {
                fmt::print(fg(fmt::color::indian_red),
                           "Failed in A star path searching !!! 0.2 seconds time limit exceeded.\n");
                return TIME_OUT;
            }
        }

        double time_2 = ros_ptr_->getSimTime();
        if ((time_2 - time_1) > 0.1) {
            fmt::print(fg(fmt::color::indian_red), "Time consume in A star path finding is {} s, iter={}.\n",
                       (time_2 - time_1),
                       num_iter);
        }
        cout << rog_map::RED << " -- [A*] Escape path searcher, cannot find path, return." << rog_map::RESET << endl;
        return NO_PATH;
    }

    /**
     * @brief 判断邻域中是否有指定类型的单元格
     * @param type 单元格类型
     * @param src_id 源索引
     * @return 是否存在
     */
    bool Astar::neighborHaveOne(const rog_map::GridType &type, const rog_map::Vec3i &src_id) {
        for (const auto &nei: neighbor_list) {
            rog_map::Vec3i nei_id = src_id + nei;
            if (!insideLocalMap(nei_id)) {
                continue;
            }
            rog_map::Vec3f nei_pos;
            globalIndexToPos(nei_id, nei_pos);
            rog_map::GridType nei_type;
            if (md_.use_inf_map) {
                nei_type = map_ptr_->getInfGridType(nei_pos);
            } else {
                nei_type = map_ptr_->getGridType(nei_pos);
            }
            if (nei_type == type) {
                return true;
            }
        }
        return false;
    }
}
