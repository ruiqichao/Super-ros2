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

#include <rog_map/rog_map_core/sliding_map.h>

using namespace rog_map;
using namespace color_text;
using namespace super_utils;

/**
 * @brief 滑动地图构造函数
 * @param half_map_size_i 地图半尺寸（以网格为单位）
 * @param resolution 地图分辨率
 * @param map_sliding_en 是否启用地图滑动
 * @param sliding_thresh 滑动阈值
 * @param fix_map_origin 固定地图原点
 */
SlidingMap::SlidingMap(const Vec3i &half_map_size_i, const double &resolution, const bool &map_sliding_en,
                       const double &sliding_thresh, const Vec3f &fix_map_origin) {
    std::cout<<"half_map_size_i: "<<half_map_size_i.transpose()<<std::endl;
    std::cout<<"resolution: "<<resolution<<std::endl;
    std::cout<<"map_sliding_en: "<<map_sliding_en<<std::endl;
    std::cout<<"sliding_thresh: "<<sliding_thresh<<std::endl;
    std::cout<<"fix_map_origin: "<<fix_map_origin.transpose()<<std::endl;
    initSlidingMap(half_map_size_i, resolution, map_sliding_en, sliding_thresh, fix_map_origin);
}

/**
 * @brief 初始化滑动地图
 * @param half_map_size_i 地图半尺寸（以网格为单位）
 * @param resolution 地图分辨率
 * @param map_sliding_en 是否启用地图滑动
 * @param sliding_thresh 滑动阈值
 * @param fix_map_origin 固定地图原点
 */
void
SlidingMap::initSlidingMap(const rog_map::Vec3i &half_map_size_i, const double &resolution, const bool &map_sliding_en,
                           const double &sliding_thresh, const rog_map::Vec3f &fix_map_origin) {
    // 检查是否已经初始化过
    if (had_been_initialized) {
        throw std::runtime_error(" -- [SlidingMap]: init can only be called once!");
    }
    // 检查坐标系定义宏是否冲突
#ifdef ORIGIN_AT_CORNER
#ifdef ORIGIN_AT_CENTER
    throw std::runtime_error(" -- [SlidingMap]: ORIGIN_AT_CORNER and ORIGIN_AT_CENTER cannot be both true!");
#endif
#endif
    // 设置配置参数
    sc_.resolution = resolution;              // 地图分辨率
    sc_.resolution_inv = 1.0 / resolution;    // 分辨率倒数
    sc_.map_sliding_en = map_sliding_en;      // 是否启用滑动
    sc_.sliding_thresh = sliding_thresh;      // 滑动阈值
    sc_.fix_map_origin = fix_map_origin;      // 固定地图原点
    sc_.half_map_size_i = half_map_size_i;    // 半地图尺寸
    sc_.map_size_i = 2 * sc_.half_map_size_i + Vec3i::Constant(1);  // 完整地图尺寸
    sc_.map_vox_num = sc_.map_size_i.prod();  // 地图体素总数
    // 如果不启用滑动，则使用固定原点
    if (!map_sliding_en) {
        local_map_origin_d_ = fix_map_origin;  // 设置本地地图原点
        posToGlobalIndex(local_map_origin_d_, local_map_origin_i_);  // 转换为全局索引
    }
    had_been_initialized = true;  // 标记已初始化
}

/**
 * @brief 打印地图信息
 */
void SlidingMap::printMapInformation() {
    std::cout << GREEN << "\tresolution: " << sc_.resolution << RESET << std::endl;      // 输出分辨率
    std::cout << GREEN << "\tmap_sliding_en: " << sc_.map_sliding_en << RESET << std::endl;  // 输出是否启用滑动
    std::cout << GREEN << "\tlocal_map_size_i: " << sc_.map_size_i.transpose() << RESET << std::endl;  // 输出本地地图尺寸（网格）
    std::cout << GREEN << "\tlocal_map_size_d: " << sc_.map_size_i.cast<double>().transpose() * sc_.resolution << RESET  // 输出本地地图尺寸（实际距离）
              << std::endl;
}

/**
 * @brief 判断位置是否在本地地图范围内
 * @param pos 3D位置
 * @return true 如果在本地地图范围内，false 否则
 */
bool SlidingMap::insideLocalMap(const Vec3f &pos) const {
    Vec3i id_g;
    posToGlobalIndex(pos, id_g);  // 将位置转换为全局索引
    return insideLocalMap(id_g);  // 判断索引是否在本地地图范围内
}

/**
 * @brief 判断网格索引是否在本地地图范围内
 * @param id_g 全局网格索引
 * @return true 如果在本地地图范围内，false 否则
 */
bool SlidingMap::insideLocalMap(const Vec3i &id_g) const {
    // 检查网格索引是否超出本地地图边界
    if (((id_g - local_map_origin_i_).cwiseAbs() - sc_.half_map_size_i).maxCoeff() > 0) {
        return false;
    }
    return true;
}

/**
 * @brief 更新本地地图原点和边界
 * @param new_origin_d 新的原点位置（双精度）
 * @param new_origin_i 新的原点索引
 */
void SlidingMap::updateLocalMapOriginAndBound(const rog_map::Vec3f &new_origin_d, const rog_map::Vec3i &new_origin_i) {
    // 更新本地地图原点
    local_map_origin_i_ = new_origin_i;  // 更新原点索引
    local_map_origin_d_ = new_origin_d;  // 更新原点位置

    // 计算新的边界
    local_map_bound_max_i_ = local_map_origin_i_ + sc_.half_map_size_i;  // 最大边界索引
    local_map_bound_min_i_ = local_map_origin_i_ - sc_.half_map_size_i;  // 最小边界索引

    // 将边界索引转换为实际位置
    globalIndexToPos(local_map_bound_min_i_, local_map_bound_min_d_);  // 最小边界位置
    globalIndexToPos(local_map_bound_max_i_, local_map_bound_max_d_);  // 最大边界位置
}

/**
 * @brief 清除地图边界外的内存
 * @param clear_id 需要清除的索引列表
 * @param i 当前轴方向
 */
void SlidingMap::clearMemoryOutOfMap(const vector<int> &clear_id, const int &i) {
    // 定义三个轴的方向（当前轴、另外两轴）
    vector<int> ids{i, (i + 1) % 3, (i + 2) % 3};
    // 遍历需要清除的索引
    for (const auto &idd: clear_id) {
        // 遍历第二个轴的范围
        for (int x = -sc_.half_map_size_i(ids[1]); x <= sc_.half_map_size_i(ids[1]); x++) {
            // 遍历第三个轴的范围
            for (int y = -sc_.half_map_size_i(ids[2]); y <= sc_.half_map_size_i(ids[2]); y++) {
                // 创建临时清除索引
                Vec3i temp_clear_id;
                temp_clear_id(ids[0]) = idd;  // 当前轴设置为待清除的索引
                temp_clear_id(ids[1]) = x;   // 第二轴设置为x
                temp_clear_id(ids[2]) = y;   // 第三轴设置为y
                // 重置对应哈希索引的单元格
                resetCell(getLocalIndexHash(temp_clear_id));
            }
        }
    }
}

/**
 * @brief 执行地图滑动操作
 * @param odom 里程计位置
 */
void SlidingMap::mapSliding(const Vec3f &odom) {
    TimeConsuming update_local_shift_timer("updateLocalShift", false);  // 计时
    /// 计算新的滑动索引
    Vec3i new_origin_i;
    posToGlobalIndex(odom, new_origin_i);  // 将里程计位置转换为全局索引
    Vec3f new_origin_d = new_origin_i.cast<double>() * sc_.resolution;  // 转换为实际位置
    /// 计算偏移量
    Vec3i shift_num = new_origin_i - local_map_origin_i_;  // 计算滑动距离（以网格为单位）
    // 检查是否移动距离过大，需要重置整个地图
    for (long unsigned int i = 0; i < 3; i++) {
        if (fabs(shift_num[i]) > sc_.map_size_i[i]) {
            // 重置整个地图
            resetLocalMap();
            updateLocalMapOriginAndBound(new_origin_d, new_origin_i);
            return;
        }
    }
    // 归一化函数：将值限制在指定范围内
    static auto normalize = [](int x, int a, int b) -> int {
        int range = b - a + 1;  // 范围大小
        int y = (x - a) % range;  // 计算模
        return (y < 0 ? y + range : y) + a;  // 处理负数情况并返回归一化值
    };

    /// 清除地图边界外的内存
    for (int i = 0; i < 3; i++) {
        if (shift_num[i] == 0) {
            continue;  // 如果没有移动，则跳过
        }
        // 计算最小全局ID
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sc_.map_size_i(i);  // 计算局部ID
        vector<int> clear_id;  // 待清除的ID列表
        if (shift_num(i) > 0) {
            /// 正向滑动，需要裁剪最小ID
            for (int k = 0; k < shift_num(i); k++) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sc_.half_map_size_i(i), sc_.half_map_size_i(i));  // 归一化
                clear_id.push_back(temp_id);  // 添加到待清除列表
            }
        } else {
            /// 反向滑动，需要裁剪最大ID
            for (int k = -1; k >= shift_num(i); k--) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sc_.half_map_size_i(i), sc_.half_map_size_i(i));  // 归一化
                clear_id.push_back(temp_id);  // 添加到待清除列表
            }
        }

        if (clear_id.empty()) {
            continue;  // 如果没有需要清除的ID，则跳过
        }
        clearMemoryOutOfMap(clear_id, i);  // 清除超出边界的内存
    }

    updateLocalMapOriginAndBound(new_origin_d, new_origin_i);  // 更新地图原点和边界
}

/**
 * @brief 获取局部索引的哈希值
 * @param id_in 输入的局部索引
 * @return 哈希值
 */
int SlidingMap::getLocalIndexHash(const Vec3i &id_in) const {
    Vec3i id = id_in + sc_.half_map_size_i;  // 将输入索引转换为非负索引
    // 计算三维索引的线性哈希值
    return id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) +  // X轴权重
           id(1) * sc_.map_size_i(2) +                    // Y轴权重
           id(2);                                         // Z轴权重
}

/**
 * @brief 将位置转换为全局索引
 * @param pos 输入的位置
 * @param id 输出的全局索引
 */
void SlidingMap::posToGlobalIndex(const Vec3f &pos, Vec3i &id) const {

#ifdef ORIGIN_AT_CENTER
    // 以中心为原点的计算方式
    id = (sc_.resolution_inv * pos + pos.cwiseSign() * 0.5).cast<int>();
#endif

#ifdef ORIGIN_AT_CORNER
    // 以角落为原点的计算方式
    id = (pos.array() * sc_.resolution_inv).floor().cast<int>();
#endif
}

/**
 * @brief 将单个坐标值转换为全局索引
 * @param pos 输入的坐标值
 * @param id 输出的全局索引
 */
void SlidingMap::posToGlobalIndex(const double &pos, int &id) const {
#ifdef ORIGIN_AT_CENTER
    // 以中心为原点的计算方式
    id = static_cast<int>((sc_.resolution_inv * pos + SIGN(pos) * 0.5));
#endif
#ifdef ORIGIN_AT_CORNER
    // 以角落为原点的计算方式
    id = floor(pos * sc_.resolution_inv);
#endif
}

/**
 * @brief 将全局索引转换为位置
 * @param id_g 输入的全局索引
 * @param pos 输出的位置
 */
void SlidingMap::globalIndexToPos(const Vec3i &id_g, Vec3f &pos) const {

#ifdef ORIGIN_AT_CENTER
    // 以中心为原点的转换方式
    pos = id_g.cast<double>() * sc_.resolution;
#endif

#ifdef ORIGIN_AT_CORNER
    // 以角落为原点的转换方式
    pos = (id_g.cast<double>() + Vec3f(0.5, 0.5, 0.5)) * sc_.resolution;
#endif
}

/**
 * @brief 将全局索引转换为局部索引
 * @param id_g 输入的全局索引
 * @param id_l 输出的局部索引
 */
void SlidingMap::globalIndexToLocalIndex(const Vec3i &id_g, Vec3i &id_l) const {
    for (int i = 0; i < 3; ++i) {
        /// [eq. (7) in paper] 计算 i_k
        id_l(i) = id_g(i) % sc_.map_size_i(i);  // 计算模运算得到局部索引
        /// [eq. (8) in paper] 归一化局部索引
        if (id_l(i) > sc_.half_map_size_i(i)) {
            id_l(i) -= sc_.map_size_i(i);  // 如果超出正边界，减去总尺寸
        } else if (id_l(i) < -sc_.half_map_size_i(i)) {
            id_l(i) += sc_.map_size_i(i);  // 如果超出负边界，加上总尺寸
        }
//        id_l(i) += id_l(i) > sc_.half_map_size_i(i) ? -sc_.map_size_i(i) : 0;
//        id_l(i) += id_l(i) < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;
    }
}

/**
 * @brief 将局部索引转换为全局索引
 * @param id_l 输入的局部索引
 * @param id_g 输出的全局索引
 */
void SlidingMap::localIndexToGlobalIndex(const Vec3i &id_l, Vec3i &id_g) const {
    for (int i = 0; i < 3; ++i) {
        // 计算最小全局ID
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sc_.map_size_i(i);  // 计算模
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;  // 归一化
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;  // 归一化
        int cur_dis_to_min_id = id_l(i) - min_id_l;  // 计算到最小ID的距离
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;  // 处理负数情况
        int cur_id = cur_dis_to_min_id + min_id_g;  // 计算当前全局ID
        id_g(i) = cur_id;  // 设置输出
    }
}

/**
 * @brief 将局部索引转换为位置
 * @param id_l 输入的局部索引
 * @param pos 输出的位置
 */
void SlidingMap::localIndexToPos(const Vec3i &id_l, Vec3f &pos) const {
#ifdef ORIGIN_AT_CENTER
    for (int i = 0; i < 3; ++i) {
        // 计算最小全局ID
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);

        int min_id_l = min_id_g % sc_.map_size_i(i);  // 计算模
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;  // 归一化
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;  // 归一化

        int cur_dis_to_min_id = id_l(i) - min_id_l;  // 计算到最小ID的距离
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;  // 处理负数情况
        int cur_id = cur_dis_to_min_id + min_id_g;  // 计算当前全局ID
        pos(i) = cur_id * sc_.resolution;  // 计算位置（中心模式）
    }
#endif

#ifdef ORIGIN_AT_CORNER
    for (int i = 0; i < 3; ++i) {
        // 计算最小全局ID
        int min_id_g = -sc_.half_map_size_i(i) + local_map_origin_i_(i);

        int min_id_l = min_id_g % sc_.map_size_i(i);  // 计算模
        min_id_l -= min_id_l > sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;  // 归一化
        min_id_l += min_id_l < -sc_.half_map_size_i(i) ? sc_.map_size_i(i) : 0;  // 归一化

        int cur_dis_to_min_id = id_l(i) - min_id_l;  // 计算到最小ID的距离
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sc_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;  // 处理负数情况
        int cur_id = cur_dis_to_min_id + min_id_g;  // 计算当前全局ID
        pos(i) = (cur_id + 0.5) * sc_.resolution;  // 计算位置（角落模式）
    }
#endif

}

/**
 * @brief 将哈希ID转换为局部索引
 * @param hash_id 输入的哈希ID
 * @param id 输出的局部索引
 */
void SlidingMap::hashIdToLocalIndex(const int &hash_id, Vec3i &id) const {
    id(0) = hash_id / (sc_.map_size_i(1) * sc_.map_size_i(2));  // 计算X轴索引
    id(1) = (hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2)) / sc_.map_size_i(2);  // 计算Y轴索引
    id(2) = hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) - id(1) * sc_.map_size_i(2);  // 计算Z轴索引
    id -= sc_.half_map_size_i;  // 转换为相对于中心的索引
}

/**
 * @brief 将哈希ID转换为全局索引
 * @param hash_id 输入的哈希ID
 * @param id_g 输出的全局索引
 */
void SlidingMap::hashIdToGlobalIndex(const int &hash_id, rog_map::Vec3i &id_g) const {
    Vec3i id;
    id(0) = hash_id / (sc_.map_size_i(1) * sc_.map_size_i(2));  // 计算X轴索引
    id(1) = (hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2)) / sc_.map_size_i(2);  // 计算Y轴索引
    id(2) = hash_id - id(0) * sc_.map_size_i(1) * sc_.map_size_i(2) - id(1) * sc_.map_size_i(2);  // 计算Z轴索引
    id -= sc_.half_map_size_i;  // 转换为相对于中心的索引
    localIndexToGlobalIndex(id, id_g);  // 将局部索引转换为全局索引
}

/**
 * @brief 将哈希ID转换为位置
 * @param hash_id 输入的哈希ID
 * @param pos 输出的位置
 */
void SlidingMap::hashIdToPos(const int &hash_id, Vec3f &pos) const {
    Vec3i id;
    hashIdToLocalIndex(hash_id, id);  // 将哈希ID转换为局部索引
    localIndexToPos(id, pos);  // 将局部索引转换为位置
}

/**
 * @brief 从位置获取哈希索引
 * @param pos 输入的位置
 * @return 哈希索引
 */
int SlidingMap::getHashIndexFromPos(const Vec3f &pos) const {
    Vec3i id_g, id_l;
    posToGlobalIndex(pos, id_g);  // 将位置转换为全局索引
    globalIndexToLocalIndex(id_g, id_l);  // 将全局索引转换为局部索引
    return getLocalIndexHash(id_l);  // 获取局部索引的哈希值
}

/**
 * @brief 从全局索引获取哈希索引
 * @param id_g 输入的全局索引
 * @return 哈希索引
 */
int SlidingMap::getHashIndexFromGlobalIndex(const Vec3i &id_g) const {
    Vec3i id_l;
    globalIndexToLocalIndex(id_g, id_l);  // 将全局索引转换为局部索引
    return getLocalIndexHash(id_l);  // 获取局部索引的哈希值
}