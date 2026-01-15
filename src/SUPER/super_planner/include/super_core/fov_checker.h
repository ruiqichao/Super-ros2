
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

#include <utils/geometry/geometry_utils.h>

namespace super_planner {
    using namespace geometry_utils;

    // FOV（视场）类型枚举
    enum FOVType {
        UNDEFINED,
        CONE = 1,  // 锥形视场
        OMNI = 2   // 全向视场
    };


    // FOV检查器类，用于处理视场相关的几何计算
    class FOVChecker {
        Mat3Df four_pts_;      // 存储4个关键点（用于上下两个切面）
        Mat3Df sixteen_pts_;   // 存储16个点（用于生成完整的FOV平面）

    public:
        // FOV配置结构体
        struct FOVConfig {
            FOVType type;           // FOV类型
            // for OMNI（全向视场参数）
            double upper_angle;     // 上仰角
            double lower_angle;     // 下俯角
            // for CONE（锥形视场参数）
            double angle;           // 视场角
        } cfg_;

        typedef std::shared_ptr<FOVChecker> Ptr;

        /**
         * @brief 根据感知范围裁剪多面体
         * @param robot_p 机器人位置
         * @param guide_p 引导点位置
         * @param sensing_horizon 感知距离
         * @param poly 待裁剪的多面体
         * @return 是否成功裁剪（返回false表示裁剪后多面体为空）
         */
        bool cutPolyBySensingHorizon( const Vec3f & robot_p, const Vec3f & guide_p,
                                      const double & sensing_horizon, Polytope & poly) const {
            // 计算从机器人到引导点的方向向量
            Vec3f dir = (guide_p - robot_p).normalized();
            // 计算感知范围边界点
            Vec3f p = robot_p + dir * sensing_horizon;
            // 构建裁剪平面 (法向量为dir，过点p)
            Vec4f new_plane;
            new_plane.head(3) = dir;
            new_plane(3) = -dir.dot(p);
            cout<<new_plane.transpose()<<endl;
            // 获取原有平面并添加新平面
            auto planes = poly.GetPlanes();
            MatD4f new_planes(planes.rows() + 1, 4);
            new_planes.topRows(planes.rows()) = planes;
            new_planes.bottomRows(1) = new_plane.transpose();
            cout<<"123123124512======================"<<endl;
            cout<<new_planes<<endl;
            poly.SetPlanes(new_planes);
            Vec3f it;
            // 平面约束方程: h0*x + h1*y + h2*z + h3 <= 0
            // 检查裁剪后多面体是否有内部点
            if (!geometry_utils::findInterior(new_planes,it)) {
                return false;
            }
            return true;
        }

        /**
         * @brief 根据FOV约束裁剪多面体
         * @param robot_p 机器人位置
         * @param robot_q 机器人姿态（四元数）
         * @param guide_p 引导点位置
         * @param poly 待裁剪的多面体
         * @return 是否成功裁剪
         */
        bool cutPolyByFov(const Vec3f & robot_p, const super_utils::Quatf & robot_q,
            const Vec3f & guide_p, Polytope & poly) const {
            const double small_x = 0.1;  // 小偏移量，避免数值问题
            // 将引导点转换到机器人坐标系
            Vec3f guide_p_B = robot_q.matrix().transpose() * (guide_p - robot_p);
            // 计算在机器人坐标系中的偏航角
            double yaw_in_body_frame = atan2(guide_p_B.y(), guide_p_B.x());
            // 构建绕z轴的旋转矩阵
            Mat3f rotation_matrix3;
            rotation_matrix3 = Eigen::AngleAxisd(yaw_in_body_frame, Eigen::Vector3d::UnitZ());
            // 组合旋转矩阵
            Mat3f R = robot_q.matrix() * rotation_matrix3;
            MatD4f fov_plane, temp_plane;
            vec_E<Mat3f> fov_plane_pt;
            // 计算FOV检查点（略微向后偏移）
            Vec3f dir = (robot_p - guide_p).normalized();
            getFovCheckPlane(R, robot_p + dir * small_x * 2, fov_plane,
                                           fov_plane_pt);
            // 合并原有平面和FOV平面
            temp_plane.resize(poly.SurfNum() + fov_plane.rows(), 4);
            temp_plane << poly.GetPlanes(), fov_plane;
            Vec3f interior;
            // 检查多面体内部点是否满足最小距离要求
            if (geometry_utils::findInteriorDist(poly.GetPlanes(), interior) < small_x) {
                return false;
            }
            poly.SetPlanes(temp_plane);
            return true;
        }

        /**
         * @brief FOVChecker构造函数
         * @param fov_type FOV类型
         * @param cone_fov_angle 锥形FOV角度（度）
         * @param lower_fov_angle 下俯角（度）
         * @param upper_fov_angle 上仰角（度）
         */
        FOVChecker(const FOVType &fov_type,
                   const double &cone_fov_angle,
                   const double &lower_fov_angle,
                   const double &upper_fov_angle) {
            cfg_.type = fov_type;
            // 角度转弧度
            cfg_.angle = cone_fov_angle / 180.0 * M_PI;
            cfg_.lower_angle = lower_fov_angle / 180.0 * M_PI;
            cfg_.upper_angle = upper_fov_angle / 180.0 * M_PI;
            switch (cfg_.type) {
                case OMNI: {
                    // 全向FOV模式：构建上下圆环的关键点
                    static const double sqrt2 = sqrt(2);
                    static double zup = sin(upper_fov_angle) * 5;       // 上圆环z坐标
                    static double rup = cos(upper_fov_angle) * 5;       // 上圆环半径
                    static double zdown = sin(lower_fov_angle) * 5;     // 下圆环z坐标
                    static double rdown = cos(lower_fov_angle) * 5;     // 下圆环半径
                    
                    // 构建4个点用于上下两个切面
                    four_pts_.resize(3, 4);
                    four_pts_.col(0) = Eigen::Vector3d(rup, -3, zup);
                    four_pts_.col(1) = Eigen::Vector3d(rup, 3, zup);
                    four_pts_.col(2) = Eigen::Vector3d(rdown, -3, zdown);
                    four_pts_.col(3) = Eigen::Vector3d(rdown, 3, zdown);

                    // 构建16个点用于完整FOV平面（上下圆环各8个点）
                    sixteen_pts_.resize(3, 16);
                    // 上圆环8个点
                    sixteen_pts_.col(0) = Eigen::Vector3d(0, rup, zup);
                    sixteen_pts_.col(1) = Eigen::Vector3d(rup / sqrt2, rup / sqrt2, zup);
                    sixteen_pts_.col(2) = Eigen::Vector3d(rup, 0, zup);
                    sixteen_pts_.col(3) = Eigen::Vector3d(rup / sqrt2, -rup / sqrt2, zup);
                    sixteen_pts_.col(4) = Eigen::Vector3d(0, -rup, zup);
                    sixteen_pts_.col(5) = Eigen::Vector3d(-rup / sqrt2, -rup / sqrt2, zup);
                    sixteen_pts_.col(6) = Eigen::Vector3d(-rup, 0, zup);
                    sixteen_pts_.col(7) = Eigen::Vector3d(-rup / sqrt2, rup / sqrt2, zup);

                    // 下圆环8个点
                    sixteen_pts_.col(8) = Eigen::Vector3d(0, rdown, zdown);
                    sixteen_pts_.col(9) = Eigen::Vector3d(rdown / sqrt2, rdown / sqrt2, zdown);
                    sixteen_pts_.col(10) = Eigen::Vector3d(rdown, 0, zdown);
                    sixteen_pts_.col(11) = Eigen::Vector3d(rdown / sqrt2, -rdown / sqrt2, zdown);
                    sixteen_pts_.col(12) = Eigen::Vector3d(0, -rdown, zdown);
                    sixteen_pts_.col(13) = Eigen::Vector3d(-rdown / sqrt2, -rdown / sqrt2, zdown);
                    sixteen_pts_.col(14) = Eigen::Vector3d(-rdown, 0, zdown);
                    sixteen_pts_.col(15) = Eigen::Vector3d(-rdown / sqrt2, rdown / sqrt2, zdown);

                    break;
                }
                default: {
                    throw std::runtime_error("Unsupported FOV type");
                }
            }
        }

        /**
         * @brief 获取所有FOV平面（16个侧面）
         * @param R 旋转矩阵
         * @param t 平移向量
         * @param fov_pts 输出：FOV平面的顶点集合
         */
        void getAllFovPlane(
                const Mat3f R,
                const Vec3f t,
                vec_E<Mat3f> &fov_pts) const {
            fov_pts.clear();
            // 将16个点变换到世界坐标系
            Eigen::Matrix3Xd sixteen_pts = (R * sixteen_pts_).colwise() + t;
            Mat3f temp_mat;
            // 生成上圆环的8个侧面
            for (int i = 0; i < 7; i++) {
                temp_mat << sixteen_pts.col(i), sixteen_pts.col(i + 1), t;
                fov_pts.push_back(temp_mat);
            }
            // 闭合上圆环
            temp_mat << sixteen_pts.col(7), sixteen_pts.col(0), t;
            fov_pts.push_back(temp_mat);
            // 生成下圆环的8个侧面
            for (int i = 8; i < 15; i++) {
                temp_mat << sixteen_pts.col(i), sixteen_pts.col(i + 1), t;
                fov_pts.push_back(temp_mat);
            }
            // 闭合下圆环
            temp_mat << sixteen_pts.col(15), sixteen_pts.col(8), t;
            fov_pts.push_back(temp_mat);
        }

        /**
         * @brief 获取FOV检查平面（仅上下两个切面）
         * @param R 旋转矩阵
         * @param t 平移向量
         * @param fov_planes 输出：FOV平面方程（每行为[nx, ny, nz, d]）
         * @param fov_pts 输出：FOV平面的顶点
         */
        void getFovCheckPlane(
                const Eigen::Matrix3d R,
                const Eigen::Vector3d t,
                Eigen::MatrixX4d &fov_planes,
                vec_E<Eigen::Matrix3d> &fov_pts) const {
            // 只使用上下两个切面来约束
            fov_planes.resize(2, 4);
            fov_pts.clear();

            // FOV内部参考点（用于确定平面法向量方向）
            static Eigen::Vector3d inner_pt(1, 0, 0);

            // 将4个关键点变换到世界坐标系
            Eigen::Matrix3Xd four_pts = (R * four_pts_).colwise() + t;
            Eigen::Vector3d fov_inner_pt = R * inner_pt + t;
            Eigen::Vector4d temp;
            
            // 计算上切面
            geometry_utils::FromPointsToPlane(four_pts.col(0), four_pts.col(1), t, temp);
            // 确保法向量指向FOV内部
            if (temp.head(3).dot(fov_inner_pt) + temp(3) > 0) {
                temp = -temp;
            }

            // 保存上切面的顶点
            Eigen::Matrix3d temp_p;
            temp_p << four_pts.col(0), four_pts.col(1), t;
            fov_pts.push_back(temp_p);
            // 保存下切面的顶点
            temp_p << four_pts.col(2), four_pts.col(3), t;
            fov_pts.push_back(temp_p);
            fov_planes.row(0) = temp;
            
            // 计算下切面
            geometry_utils::FromPointsToPlane(four_pts.col(2), four_pts.col(3), t, temp);
            // 确保法向量指向FOV内部
            if (temp.head(3).dot(fov_inner_pt) + temp(3) > 0) {
                temp = -temp;
            }
            fov_planes.row(1) = temp;
        }

        /**
         * @brief 获取FOV检查平面（重载版本，不返回顶点）
         * @param R 旋转矩阵
         * @param t 平移向量
         * @param fov_planes 输出：FOV平面方程
         */
        void getFovCheckPlane(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d &fov_planes) {
            vec_E<Mat3f> temp;
            getFovCheckPlane(R, t, fov_planes, temp);
        }


    };

}
