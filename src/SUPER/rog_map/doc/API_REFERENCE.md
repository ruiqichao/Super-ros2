# ROG-Map API 文档

## 目录
1. [概述](#概述)
2. [架构设计](#架构设计)
3. [核心类 API](#核心类-api)
4. [查询接口](#查询接口)
5. [地图维护](#地图维护)
6. [ROS2 接口](#ros2-接口)
7. [使用示例](#使用示例)
8. [性能特性](#性能特性)

---

## 概述

**ROG-Map** 是一个用于多旋翼无人机高速自主导航的**实时占用栅格地图库**。

**核心特性**：
- ✅ 实时 3D 占用栅格更新（无时间维度）
- ✅ 多层膨胀处理（碰撞检测）
- ✅ 滑动窗口地图（内存高效）
- ✅ ESDF 距离场计算
- ✅ 边界提取（自主探索）
- ✅ ROS2 原生支持

---

## 架构设计

```
┌─────────────────────────────────────┐
│      ROGMapROS (ROS2 接口层)        │
│   - 话题订阅/发布                   │
│   - 服务回调                        │
│   - 可视化                          │
└──────────────────┬──────────────────┘
                   │
┌──────────────────▼──────────────────┐
│      ROGMap (核心基类)              │
│   - 线段自由检测                    │
│   - 最近邻搜索                      │
└──────────────────┬──────────────────┘
                   │
┌──────────────────▼──────────────────┐
│      ProbMap (概率栅格)             │
│   - 单点占有查询                    │
│   - 盒子搜索                        │
│   - 膨胀查询                        │
└──────────────────┬──────────────────┘
                   │
┌──────────────────▼──────────────────┐
│      InfMap (膨胀层)                │
│   - 膨胀计数维护                    │
│   - 膨胀盒子搜索                    │
└──────────────────┬──────────────────┘
                   │
┌──────────────────▼──────────────────┐
│      CounterMap (计数层)            │
│   - 占据/未知计数                   │
│   - 网格类型判断                    │
└──────────────────┬──────────────────┘
                   │
┌──────────────────▼──────────────────┐
│      SlidingMap (滑动地图)          │
│   - 3D 网格索引                     │
│   - 坐标转换                        │
│   - 内存管理                        │
└─────────────────────────────────────┘
```

---

## 核心类 API

### 1. ProbMap 类（概率占用栅格）

**位置**：`include/rog_map/prob_map.h`

#### 单点查询

```cpp
class ProbMap {
public:
    // 单点查询 - 基础状态
    bool isOccupied(const Vec3f &pos) const;      // 是否被占据
    bool isUnknown(const Vec3f &pos) const;       // 是否未知
    bool isKnownFree(const Vec3f &pos) const;     // 是否已知自由
    
    // 单点查询 - 膨胀状态
    bool isOccupiedInflate(const Vec3f &pos) const;
    bool isUnknownInflate(const Vec3f &pos) const;
    bool isKnownFreeInflate(const Vec3f &pos) const;
    
    // 获取网格类型
    GridType getGridType(const Vec3f &pos) const;
    GridType getInfGridType(const Vec3f &pos) const;
    
    // 获取占有度值 [0, 1]
    double getMapValue(const Vec3f &pos) const;
    
    // 边界查询
    bool isFrontier(const Vec3f &pos) const;
    bool isFrontier(const Vec3i &id_g) const;
};
```

**参数说明**：
- `pos` (Vec3f): 世界坐标系中的 3D 位置
- 返回值: `true` = 满足条件，`false` = 不满足

**性能**：O(1) 查询

---

#### 盒子搜索

```cpp
// 在指定盒子范围内搜索所有特定类型的体素
void boxSearch(const Vec3f &_box_min, const Vec3f &_box_max,
               const GridType &gt, vec_E<Vec3f> &out_points) const;

// 膨胀版本
void boxSearchInflate(const Vec3f &box_min, const Vec3f &box_max,
                      const GridType &gt, vec_E<Vec3f> &out_points) const;
```

**参数**：
- `_box_min`, `_box_max`: 搜索盒子的最小/最大角点（世界坐标）
- `gt`: 网格类型过滤器（`OCCUPIED`, `UNKNOWN`, `FRONTIER`）
- `out_points`: 输出的点云列表（自动清空并填充）

**示例**：
```cpp
Vec3f robot_pos(0, 0, 1);
Vec3f box_min = robot_pos - Vec3f(5, 5, 3);
Vec3f box_max = robot_pos + Vec3f(5, 5, 3);
vec_E<Vec3f> obstacles;

map_ptr->boxSearch(box_min, box_max, GridType::OCCUPIED, obstacles);
std::cout << "Found " << obstacles.size() << " occupied cells" << std::endl;
```

---

### 2. ROGMap 类（核心查询扩展）

**位置**：`include/rog_map/rog_map.h`

#### 线段自由检测

```cpp
class ROGMap : public ProbMap {
public:
    // 基础版：检测两点连线是否自由
    bool isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                    const double& max_dis = 999999,
                    const vec_Vec3i& neighbor_list = vec_Vec3i{}) const;

    // 返回自由终点版
    bool isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                    Vec3f& free_local_goal,
                    const double& max_dis = 999999,
                    const vec_Vec3i& neighbor_list = vec_Vec3i{}) const;

    // 膨胀版
    bool isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                    const bool& use_inf_map = false,
                    const bool& use_unk_as_occ = false) const;
};
```

**参数**：
- `start_pt`, `end_pt`: 线段起止点
- `max_dis`: 最大查询距离（优化参数）
- `neighbor_list`: 邻域掩码（通常留空）
- `use_inf_map`: 是否使用膨胀地图（更严格的碰撞检测）
- `use_unk_as_occ`: 是否将未知视为占据

**返回**：`true` = 线段自由，`false` = 线段与障碍物相交

---

#### 最近邻搜索

```cpp
// 查询类型为 target_type 的最近体素
bool getNearestCellIs(const GridType& target_type,
                      const Vec3f& start_pos,
                      Vec3f& nearest_pt,
                      const double& max_dis) const;

// 查询**不是** target_type 的最近体素
bool getNearestCellNot(const GridType& target_type,
                       const Vec3f& start_pos,
                       Vec3f& nearest_pt,
                       const double& max_dis) const;

// 膨胀版本
bool getNearestInfCellIs(const GridType& target_type,
                         const Vec3f& start_pos,
                         Vec3f& nearest_pt,
                         const double& max_dis) const;

bool getNearestInfCellNot(const GridType& target_type,
                          const Vec3f& start_pos,
                          Vec3f& nearest_pt,
                          const double& max_dis) const;
```

**参数**：
- `target_type`: 搜索的网格类型
- `start_pos`: 搜索起始位置
- `nearest_pt`: 输出的最近点
- `max_dis`: 搜索半径

**返回**：`true` = 找到目标，`false` = 在范围内未找到

---

### 3. InfMap 类（膨胀层）

**位置**：`include/rog_map/inf_map.h`

```cpp
class InfMap : public CounterMap {
public:
    // 膨胀层查询
    bool isOccupiedInflate(const Vec3f &pos) const;
    bool isUnknownInflate(const Vec3f &pos) const;
    bool isKnownFreeInflate(const Vec3f &pos) const;
    
    // 盒子搜索
    void boxSearch(const Vec3f &box_min, const Vec3f &box_max,
                   const GridType &gt, vec_E<Vec3f> &out_points) const;
    
    // 获取膨胀分辨率
    double getResolution() const;
    
    // 获取统计信息
    void getInflationNumAndTime(double &inf_n, double &inf_t);
};
```

---

## 查询接口

### GridType 枚举

```cpp
enum GridType {
    OCCUPIED,       // 被障碍物占据
    UNKNOWN,        // 未知区域
    KNOWN_FREE,     // 已知自由空间
    FRONTIER,       // 边界（未知-自由交界）
    OUT_OF_MAP      // 超出地图范围
};
```

### 坐标转换

```cpp
// 世界坐标 → 网格索引
void probMapPosToGlobalIndex(const Vec3f &pos, Vec3i &id_g) const;

// 网格索引 → 世界坐标
void probMapGlobalIndexToPos(const Vec3i &id_g, Vec3f &pos) const;

// 膨胀地图版本
void infMapPosToGlobalIndex(const Vec3f &pos, Vec3i &id) const;
void infMapGlobalIndexToPos(const Vec3i &id_g, Vec3f &pos) const;
```

### 地图信息查询

```cpp
// 获取本地地图原点
Vec3f getLocalMapOrigin() const;

// 获取本地地图尺寸（立方体边长）
Vec3f getLocalMapSize() const;

// 获取分辨率
double getResolution() const;
double getInfResolution() const;

// 获取地图配置
rog_map::Config getMapConfig() const;

// 获取机器人状态
RobotState getRobotState() const;

// 限制盒子在地图范围内
void boundBoxByLocalMap(Vec3f &box_min, Vec3f &box_max) const;
```

---

## 地图维护

### 更新接口

```cpp
// 主接口：更新点云和机器人姿态
void updateMap(const PointCloud& cloud, const Pose& pose);
```

**参数**：
- `cloud`: PCL 点云（PointXYZI 格式）
- `pose`: 机器人姿态（位置 + 四元数）

**处理流程**：
1. 检查是否需要地图滑动
2. 执行 raycasting（光线投射）
3. 更新占有概率
4. 触发膨胀传播
5. 更新 ESDF（如启用）

---

### 重置接口

```cpp
// 清空整个本地地图
void resetLocalMap();
```

---

### 日志输出

```cpp
// 写时间消耗到文件
void writeTimeConsumingToLog(std::ofstream &log_file);

// 写地图信息到文件
void writeMapInfoToLog(std::ofstream &log_file);
```

---

## ROS2 接口

### ROGMapROS 类

**位置**：`include/rog_map_ros/rog_map_ros2.hpp`

#### 初始化

```cpp
ROGMapROS(const rclcpp::Node::SharedPtr nh, 
          const std::string& cfg_path);
```

**参数**：
- `nh`: ROS2 节点句柄
- `cfg_path`: 配置文件路径（YAML）

**自动订阅/发布**（如启用 `ros_callback_en`）：
- 订阅：`/lidar_slam/odom`, `/cloud_registered`
- 发布：`rog_map/occ`, `rog_map/unk` 等

---

### 话题接口

#### 订阅

| 话题名 | 消息类型 | 说明 |
|-------|---------|------|
| `/lidar_slam/odom` | `nav_msgs/Odometry` | 机器人位姿 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | LiDAR 点云 |

#### 发布

| 话题名 | 消息类型 | 说明 |
|-------|---------|------|
| `rog_map/occ` | `sensor_msgs/PointCloud2` | 占据体素 |
| `rog_map/unk` | `sensor_msgs/PointCloud2` | 未知体素 |
| `rog_map/inf_occ` | `sensor_msgs/PointCloud2` | 膨胀占据 |
| `rog_map/inf_unk` | `sensor_msgs/PointCloud2` | 膨胀未知 |
| `rog_map/frontier` | `sensor_msgs/PointCloud2` | 边界体素 |
| `rog_map/esdf` | `sensor_msgs/PointCloud2` | ESDF 距离场 |
| `rog_map/map_bound` | `visualization_msgs/MarkerArray` | 地图边界可视化 |

---

### 服务接口

```cpp
Service: /rog_map/reset_map
Type: rog_map/srv/ResetMap
Request:
  (empty)
Response:
  bool success
  string message
```

**使用**：
```bash
ros2 service call /rog_map/reset_map rog_map/srv/ResetMap
```

---

## 使用示例

### 示例 1：基础单点查询

```cpp
#include <rog_map/rog_map_ros/rog_map_ros2.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("query_demo");
    
    // 创建地图
    auto map = std::make_shared<rog_map::ROGMapROS>(
        node, "/path/to/config.yaml"
    );
    
    // 查询单点
    Vec3f query_pos(1.0, 2.0, 1.5);
    
    if (map->isOccupied(query_pos)) {
        RCLCPP_INFO(node->get_logger(), "Position is occupied");
    } else if (map->isUnknown(query_pos)) {
        RCLCPP_INFO(node->get_logger(), "Position is unknown");
    } else {
        RCLCPP_INFO(node->get_logger(), "Position is free");
    }
    
    rclcpp::shutdown();
    return 0;
}
```

---

### 示例 2：线段碰撞检测

```cpp
Vec3f robot_pos(0, 0, 1);
Vec3f target_pos(10, 10, 1);

if (map->isLineFree(robot_pos, target_pos, 100.0)) {
    RCLCPP_INFO(node->get_logger(), "Path is free");
} else {
    RCLCPP_INFO(node->get_logger(), "Path collision detected");
}
```

---

### 示例 3：盒子搜索获取障碍物

```cpp
Vec3f robot_pos = map->getRobotState().p;
Vec3f box_min = robot_pos - Vec3f(10, 10, 5);
Vec3f box_max = robot_pos + Vec3f(10, 10, 5);

vec_E<Vec3f> obstacles;
map->boxSearch(box_min, box_max, GridType::OCCUPIED, obstacles);

RCLCPP_INFO(node->get_logger(), "Found %lu obstacles", obstacles.size());
for (const auto& obs : obstacles) {
    RCLCPP_DEBUG(node->get_logger(), "Obstacle at: (%.2f, %.2f, %.2f)",
                 obs.x(), obs.y(), obs.z());
}
```

---

### 示例 4：最近邻搜索

```cpp
Vec3f start_pos(0, 0, 1);
Vec3f nearest_obstacle;
double search_radius = 5.0;

if (map->getNearestCellIs(GridType::OCCUPIED, start_pos, 
                          nearest_obstacle, search_radius)) {
    double distance = (nearest_obstacle - start_pos).norm();
    RCLCPP_WARN(node->get_logger(), 
                "Nearest obstacle at distance: %.2f m", distance);
} else {
    RCLCPP_INFO(node->get_logger(), 
                "No obstacles within %.1f m", search_radius);
}
```

---

### 示例 5：安全距离检测（使用膨胀地图）

```cpp
Vec3f drone_pos(5, 5, 2);
double safe_radius = 0.5;  // 无人机半径

if (map->isOccupiedInflate(drone_pos)) {
    RCLCPP_ERROR(node->get_logger(), "COLLISION RISK!");
    // 执行紧急避障
} else if (map->isUnknownInflate(drone_pos)) {
    RCLCPP_WARN(node->get_logger(), "Entering unknown region");
}
```

---

## 性能特性

### 时间复杂度

| 操作 | 复杂度 | 说明 |
|------|------|------|
| 单点查询 | O(1) | 哈希表直接访问 |
| 线段碰撞检测 | O(N) | N = 沿线段的体素数 |
| 盒子搜索 | O(M) | M = 盒子内总体素数 |
| 最近邻搜索 | O(K³) | K = 搜索球的网格边长 |
| 地图更新 | O(P + O) | P = 点云大小，O = 膨胀计算 |

### 空间复杂度

- **单地图**：O(N³)，N = `half_map_size_i` × 2
- **多层**：占有图 + 膨胀层 + ESDF（可选）

### 配置优化建议

```yaml
rog_map:
  resolution: 0.1              # 更小 = 更精细但更慢
  map_size: [60, 60, 40]       # 根据计算能力调整
  raycasting/enable: true      # 启用光线投射更新自由空间
  frontier_extraction_en: true # 需要时启用边界提取
  esdf_en: false               # ESDF 计算开销大
  point_filt_num: 2            # 点云下采样因子
```

---

### 注意事项

⚠️ **重要限制**：
1. **无动态参数**：所有参数在节点启动时加载，修改需重启
2. **3D 纯空间**：不包含时间维度，地图通过滑动窗口管理时效性
3. **本地地图**：采用滑动窗口，保存有限大小的本地地图

---

## 相关文件

```
rog_map/
├── include/
│   ├── rog_map/
│   │   ├── rog_map.h          (核心 ROGMap 类)
│   │   ├── prob_map.h         (概率地图 ProbMap)
│   │   ├── inf_map.h          (膨胀层 InfMap)
│   │   └── esdf_map.h         (ESDF 距离场)
│   └── rog_map_ros/
│       └── rog_map_ros2.hpp   (ROS2 接口 ROGMapROS)
├── src/rog_map/
│   ├── prob_map.cpp
│   ├── inf_map.cpp
│   └── esdf_map.cpp
└── doc/
    ├── API_REFERENCE.md       (本文档)
    └── ...
```

