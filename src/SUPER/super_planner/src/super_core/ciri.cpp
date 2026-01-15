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

#include <super_core/ciri.h>
using namespace color_text;
using namespace optimization_utils;
using namespace super_utils;


namespace super_planner {

    /**
     * @brief 执行凸分解算法，计算包含给定线段的最大椭球体
     * 
     * 该函数实现CIRI算法的核心逻辑，通过迭代优化找到一个包含线段ab的最大体积椭球体，
     * 同时确保椭球体不与障碍物相交。
     * 
     * @param bd 边界约束矩阵，每行表示一个半空间约束 (ax + by + cz + d <= 0)
     * @param pc 障碍物点云矩阵，每列是一个3D点坐标
     * @param a 线段起点
     * @param b 线段终点
     * @return RET_CODE 算法执行结果状态码
     */
    RET_CODE CIRI::comvexDecomposition(const Eigen::MatrixX4d& bd, const Eigen::Matrix3Xd& pc, const Eigen::Vector3d& a,
                                       const Eigen::Vector3d& b) {
        const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
        const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);

        /// 如果种子点不在边界内部，则强制返回错误
        if ((bd * ah).maxCoeff() > epsilon_ ||
            (bd * bh).maxCoeff() > epsilon_) {
            return INIT_ERROR;
        }

        /// 获取边界约束数量M和点约束数量N
        const int M = bd.rows();
        const int N = pc.cols();

        // 初始化椭球体，以线段中点为中心
        Ellipsoid E(Mat3f::Identity(), (a + b) / 2);
        if ((a - b).norm() > 0.1) {
            /// 使用线段种子初始化椭球体
            findEllipsoid(pc, a, b, E);
        }

        vector<Eigen::Vector4d> planes;  // 存储支撑平面
        MatD4f hPoly;                   // 存储半空间多面体表示

        Vec3f infeasible_pt_w;          // 不可行点（用于调试）

        // 主迭代循环：逐步构建包含线段的凸多面体
        for (int loop = 0; loop < iter_num_; ++loop) {
            // 将几何元素变换到椭球体框架中
            const Eigen::Vector3d fwd_a = E.toEllipsoidFrame(a);      // 点a在椭球框架中的坐标
            const Eigen::Vector3d fwd_b = E.toEllipsoidFrame(b);      // 点b在椭球框架中的坐标
            const Eigen::MatrixX4d bd_e = E.toEllipsoidFrame(bd);     // 边界约束在椭球框架中的表示
            // 计算边界约束到原点的距离
            const Eigen::VectorXd distDs = bd_e.rightCols<1>().cwiseAbs().cwiseQuotient(
                    bd_e.leftCols<3>().rowwise().norm());
            const Eigen::Matrix3Xd pc_e = E.toEllipsoidFrame(pc);     // 点云在椭球框架中的表示
            Eigen::VectorXd distRs = pc_e.colwise().norm();           // 点云中各点到椭球中心的距离

            // 标志数组，标记边界约束和点约束是否已处理
            Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
            Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

            bool completed = false;       // 标记是否完成当前迭代
            int bdMinId, pcMinId;         // 距离最近的边界约束和点约束索引
            double minSqrD = distDs.minCoeff(&bdMinId);  // 最近边界约束距离
            double minSqrR = distRs.minCoeff(&pcMinId);  // 最近点约束距离

            Eigen::Vector4d temp_tangent, temp_plane_w;  // 临时变量，用于计算切平面
            const Mat3f C_inv = E.C().inverse();         // 椭球体形状矩阵的逆

            planes.clear();
            planes.reserve(30);
            Vec3f tmp_nn_pt;

            // 内循环：逐步添加约束平面，直到所有约束都被满足
            for (int i = 0; !completed && i < (M + N); ++i) {
                if (minSqrD < minSqrR) {
                    /// 情况1：边界约束更近，添加边界约束平面
                    Vec4f p_e = bd_e.row(bdMinId);
                    temp_plane_w = E.toWorldFrame(p_e);  // 将椭球框架中的平面转换回世界坐标系
                    bdFlags(bdMinId) = 0;                // 标记该边界约束已处理
                }
                else {
                    /// 情况2：障碍物点更近，添加障碍物点约束
                    const auto & pt_w = pc.col(pcMinId);  // 当前最近的障碍物点
                    const auto dis = distancePointToSegment(pt_w, a, b);  // 计算点到线段的距离
                    
                    // 如果障碍物点距离线段太近，说明问题不可行
                    if(dis < robot_r_ - 1e-2) {
                        infeasible_pt_w = pt_w;
                        cout<<YELLOW<<" -- [CIRI] WARNING! The problem is not feasible, the min dis to obstacle is only: "<<dis<<RESET<<endl;
                        return FAILED;
                    }

                    if (robot_r_ < epsilon_) {
                        // 机器人半径为0的情况：使用点约束
                        const Vec3f& pt_e = pc_e.col(pcMinId);
                        temp_tangent(3) = -distRs(pcMinId);
                        temp_tangent.head(3) = pt_e.transpose() / distRs(pcMinId);

                        // 确保切平面不会将线段起点a排除在外
                        if (temp_tangent.head(3).dot(fwd_a) + temp_tangent(3) > epsilon_) {
                            const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_a;
                            temp_tangent.head(3) = fwd_a - (delta.dot(fwd_a) / delta.squaredNorm()) * delta;
                            distRs(pcMinId) = temp_tangent.head(3).norm();
                            temp_tangent(3) = -distRs(pcMinId);
                            temp_tangent.head(3) /= distRs(pcMinId);
                        }
                        // 确保切平面不会将线段终点b排除在外
                        if (temp_tangent.head(3).dot(fwd_b) + temp_tangent(3) > epsilon_) {
                            const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_b;
                            temp_tangent.head(3) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
                            distRs(pcMinId) = temp_tangent.head(3).norm();
                            temp_tangent(3) = -distRs(pcMinId);
                            temp_tangent.head(3) /= distRs(pcMinId);
                        }
                        if (temp_tangent.head(3).dot(fwd_b) + temp_tangent(3) > epsilon_) {
                            const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_b;
                            temp_tangent.head(3) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
                            distRs(pcMinId) = temp_tangent.head(3).norm();
                            temp_tangent(3) = -distRs(pcMinId);
                            temp_tangent.head(3) /= distRs(pcMinId);
                        }
                        temp_plane_w = E.toWorldFrame(temp_tangent);
                    }
                    else {
                        /// 机器人半径大于0的情况：考虑机器人形状
                        const Vec3f &pt_e = pc_e.col(pcMinId);  // 障碍物点在椭球框架中
                        const Vec3f &pt_w = pc.col(pcMinId);    // 障碍物点在世界坐标系中
                        // 创建以障碍物点为中心的球体椭球
                        Ellipsoid E_pe(C_inv * sphere_template_.C(), pt_e);
                        Vec3f close_pt_e;
                        // 计算球体椭球到原点的最近点
                        E_pe.pointDistaceToEllipsoid(Vec3f(0, 0, 0), close_pt_e);
                        Vec3f c_pt_w = E.toWorldFrame(close_pt_e);  // 最近点转换到世界坐标系
                        
                        // 计算法向量和距离
                        temp_plane_w.head(3) = (pt_w - c_pt_w).normalized();
                        temp_plane_w(3) = -temp_plane_w.head(3).dot(c_pt_w);

                        /// 处理与线段端点的相交情况
                        if (temp_plane_w.head(3).dot(a) + temp_plane_w(3) > -epsilon_) {
                            // 平面会将点a排除，需要调整切平面
                            findTangentPlaneOfSphere(pt_w, robot_r_, a, E.d(), temp_plane_w);
                        } else if (temp_plane_w.head(3).dot(b) + temp_plane_w(3) > -epsilon_) {
                            // 平面会将点b排除，需要调整切平面
                            findTangentPlaneOfSphere(pt_w, robot_r_, b, E.d(), temp_plane_w);
                        }
                    }
                    pcFlags(pcMinId) = 0;  // 标记该点约束已处理
                    tmp_nn_pt = pc.col(pcMinId);
                }
                
                // 更新下一个要处理的约束
                completed = true;
                minSqrD = INFINITY;
                for (int j = 0; j < M; ++j) {
                    if (bdFlags(j)) {
                        completed = false;
                        if (minSqrD > distDs(j)) {
                            bdMinId = j;
                            minSqrD = distDs(j);
                        }
                    }
                }
                minSqrR = INFINITY;
                for (int j = 0; j < N; ++j) {
                    if (pcFlags(j)) {
                        // 检查该点是否被当前平面有效约束
                        if ((temp_plane_w.head(3).dot(pc.col(j)) + temp_plane_w(3)) > robot_r_ - epsilon_) {
                            pcFlags(j) = 0;
                        }
                        else {
                            completed = false;
                            if (minSqrR > distRs(j)) {
                                pcMinId = j;
                                minSqrR = distRs(j);
                            }
                        }
                    }
                }
                planes.push_back(temp_plane_w);
            }

            // 构建半空间多面体表示
            hPoly.resize(planes.size(), 4);
            for (size_t i = 0; i < planes.size(); ++i) {
                hPoly.row(i) = planes[i];
            }

            if (loop == iter_num_ - 1) {
                break;  // 达到最大迭代次数
            }

            // 检查是否有NaN值，这表明算法失败
            if(hPoly.array().isNaN().any()) {
                cout << YELLOW << " -- [CIRI] ERROR! maxVolInsEllipsoid failed." << RESET << endl;
                return FAILED;
            }

            // 使用最大体积内切椭球算法更新椭球体
            if (!MVIE::maxVolInsEllipsoid(hPoly, E)) {
                return FAILED;
            }
        }

        // 验证最终结果
        if (std::isnan(hPoly.sum())) {
            cout << YELLOW << " -- [CIRI] ERROR! There is nan in generated planes." << RESET << endl;
            cout << a.transpose() << endl;
            cout << b.transpose() << endl;
            ros_ptr_->vizCiriSeedLine(a, b,robot_r_);
            ros_ptr_->vizCiriEllipsoid(E);
            return FAILED;
        }
        
        // 检查多面体是否有内部点
        Vec3f inner;
        if (!geometry_utils::findInterior(hPoly, inner)) {
            cout<<RED<<" -- [CIRI] The polytope is empty."<<RESET<<endl;
            ros_ptr_->vizCiriSeedLine(a, b,robot_r_);
            ros_ptr_->vizCiriEllipsoid(E);
            return FAILED;
        }
        
        // 保存优化结果
        optimized_polytope_.Reset();
        optimized_polytope_.SetPlanes(hPoly);
        optimized_polytope_.SetSeedLine(std::make_pair(a, b));
        optimized_polytope_.SetEllipsoid(E);

        return SUCCESS;
    }

    /**
     * @brief 获取优化后的多面体
     * @param optimized_poly 输出参数，存储优化后的多面体
     */
    void CIRI::getPolytope(Polytope& optimized_poly) {
        optimized_poly = optimized_polytope_;
    }

    /**
     * @brief 设置算法参数
     * @param robot_r 机器人半径
     * @param iter_num 最大迭代次数
     */
    void CIRI::setupParams(double robot_r, int iter_num) {
        robot_r_ = robot_r;
        iter_num_ = iter_num;
        // 创建球体模板椭球体
        sphere_template_ = Ellipsoid(Mat3f::Identity(), robot_r_ * Vec3f(1, 1, 1), Vec3f(0, 0, 0));
    }

    /**
     * @brief 计算通过指定点的球体切平面
     * 
     * 该函数计算一个切于球体的平面，该平面通过给定的pass_point点，
     * 并确保seed_p点位于平面的正确一侧。
     * 
     * @param center 球体中心
     * @param r 球体半径
     * @param pass_point 平面必须通过的点
     * @param seed_p 种子点（用于确定平面方向）
     * @param outter_plane 输出参数，存储计算得到的切平面
     */
    void CIRI::findTangentPlaneOfSphere(const Eigen::Vector3d& center, const double& r,
                                        const Eigen::Vector3d& pass_point,
                                        const Eigen::Vector3d& seed_p,
                                        Eigen::Vector4d& outter_plane) {

        Vec3f seed = seed_p;
        Vec3f dif = pass_point - pass_point;  // 这里可能是个错误，应该是 pass_point - seed_p
        if (dif.norm() < 1e-3) {
            // 如果pass_point和seed_p太接近，调整seed点
            if ((pass_point - center).head(2).norm() > 1e-3) {
                Vec3f v1 = (pass_point - center).normalized();
                v1(2) = 0;
                seed = seed_p + 0.01 * v1.cross(Vec3f(0, 0, 1)).normalized();
            }
            else {
                seed = seed_p + 0.01 * (pass_point - center).cross(Vec3f(1, 0, 0)).normalized();
            }
        }
        
        // 使用几何方法计算切平面
        Eigen::Vector3d P = pass_point - center;  // pass_point相对于球心的向量
        Eigen::Vector3d norm_ = (pass_point - center).cross(seed - center).normalized();  // 计算法向量
        Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(norm_, Vec3f(0, 0, 1)).matrix();  // 旋转矩阵
        P = R * P;  // 将P旋转到Z轴方向
        Eigen::Vector3d C = R * (seed - center);  // 将seed相对于球心的向量也旋转
        
        Eigen::Vector3d Q;  // 切点在旋转坐标系中的位置
        double r2 = r * r;
        double p1p2n = P.head(2).squaredNorm();  // P在XY平面上的投影长度平方
        double d = sqrt(p1p2n - r2);  // 切线长度
        double rp1p2n = r / p1p2n;
        
        // 计算两个可能的切点
        double q11 = rp1p2n * (P(0) * r - P(1) * d);
        double q21 = rp1p2n * (P(1) * r + P(0) * d);

        double q12 = rp1p2n * (P(0) * r + P(1) * d);
        double q22 = rp1p2n * (P(1) * r - P(0) * d);
        
        // 选择正确的切点，确保seed点在平面的正确一侧
        if (q11 * C(0) + q21 * C(1) < 0) {
            Q(0) = q12;
            Q(1) = q22;
        }
        else {
            Q(0) = q11;
            Q(1) = q21;
        }
        Q(2) = 0;
        
        // 将切点和法向量转换回原坐标系
        outter_plane.head(3) = R.transpose() * Q;  // 法向量
        Q = outter_plane.head(3) + center;         // 切点
        outter_plane(3) = -Q.dot(outter_plane.head(3));  // 距离项
        
        // 确保平面法向量指向正确方向
        if (outter_plane.head(3).dot(seed) + outter_plane(3) > epsilon_) {
            outter_plane = -outter_plane;
        }
    }

    /**
     * @brief 根据给定的点云和线段，初始化椭球体
     * 
     * 该函数根据线段ab和点云pc，初始化一个合适的椭球体作为算法的起点。
     * 椭球体需要包含线段，但不能与点云中的任何点相交。
     * 
     * @param pc 障碍物点云
     * @param a 线段起点
     * @param b 线段终点
     * @param out_ell 输出参数，存储初始化后的椭球体
     */
    void CIRI::findEllipsoid(const Eigen::Matrix3Xd& pc,
                             const Eigen::Vector3d& a,
                             const Eigen::Vector3d& b,
                             Ellipsoid& out_ell) {
        double f = (a - b).norm() / 2;  // 线段长度的一半
        Mat3f C = f * Mat3f::Identity();  // 初始椭球体形状矩阵
        Vec3f r = Vec3f::Constant(f);     // 初始半轴长度
        Vec3f center = (a + b) / 2;       // 椭球体中心（线段中点）
        
        // 考虑机器人半径
        C(0, 0) += robot_r_;
        r(0) += robot_r_;
        if (r(0) > 0) {
            double ratio = r(1) / r(0);
            r *= ratio;
            C *= ratio;
        }

        // 计算椭球体的旋转矩阵，使其长轴与线段ab对齐
        Mat3f Ri = Eigen::Quaterniond::FromTwoVectors(Vec3f::UnitX(), (b - a)).toRotationMatrix();
        Ellipsoid E(Ri, r, center);
        Mat3f Rf = Ri;  // 最终旋转矩阵
        Mat3Df obs;      // 障碍物点
        int min_dis_id;   // 最近障碍物点索引
        Vec3f pw;         // 最近障碍物点
        
        // 检查初始椭球体是否包含所有点
        if (E.pointsInside(pc, obs, min_dis_id)) {
            pw = obs.col(min_dis_id);
        }
        else {
            out_ell = E;
            return;  // 如果初始椭球体已经满足条件，直接返回
        }
        
        Mat3Df obs_inside = obs;
        int max_iter = 100;
        
        // 第一阶段：调整椭球体的y轴半径
        while (max_iter--) {
            Vec3f p_e = Ri.transpose() * (pw - E.d());  // 将最近点转换到椭球体局部坐标系
            const double roll = atan2(p_e(2), p_e(1));  // 计算旋转角度
            Rf = Ri * Eigen::Quaterniond(cos(roll / 2), sin(roll / 2), 0, 0);  // 更新旋转矩阵
            p_e = Rf.transpose() * (pw - E.d());
            if (p_e(0) < r(0)) {
                // 根据椭球体方程调整y轴半径
                r(1) = std::abs(p_e(1)) / std::sqrt(1 - std::pow(p_e(0) / r(0), 2));
            }
            E = Ellipsoid(Rf, r, center);
            if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
                pw = obs_inside.col(min_dis_id);
            }
            else {
                break;
            }
        }
        if (max_iter == 0) {
            cout << YELLOW << " -- [CIRI] Find Ellipsoid reach max iteration, may cause error." << endl;
        }
        max_iter = 100;

        // 检查第一阶段后的状态
        if (E.pointsInside(obs, obs_inside, min_dis_id)) {
            pw = obs_inside.col(min_dis_id);
        }
        else {
            out_ell = E;
            return;
        }

        // 第二阶段：调整椭球体的z轴半径
        while (max_iter--) {
            Vec3f p = Rf.transpose() * (pw - E.d());
            double dd = 1 - std::pow(p(0) / r(0), 2) - std::pow(p(1) / r(1), 2);
            if (dd > epsilon_) {
                // 根据椭球体方程调整z轴半径
                r(2) = std::abs(p(2)) / std::sqrt(dd);
            }
            E = Ellipsoid(Rf, r, center);
            if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
                pw = obs_inside.col(min_dis_id);
            }
            else {
                out_ell = E;
                break;
            }
        }

        if (max_iter == 0) {
            cout << YELLOW << " -- [CIRI] Find Ellipsoid reach max iteration, may cause error." << endl;
        }
        E = Ellipsoid(Rf, r, center);
        out_ell = E;
    }
}
