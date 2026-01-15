# ROG-Map 快速开始指南

## 5 分钟快速上手

### 1. 编译

```bash
cd ~/ros2_ws/super_ws
colcon build --packages-select rog_map
```

---

### 2. 最小化 ROS2 节点示例

创建文件：`test_rog_map.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rog_map/rog_map_ros/rog_map_ros2.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rog_map_test");
    
    // 创建地图实例
    std::string config_path = "/path/to/click.yaml";
    auto map = std::make_shared<rog_map::ROGMapROS>(node, config_path);
    
    RCLCPP_INFO(node->get_logger(), "ROG-Map initialized successfully");
    
    // 等待点云和里程计消息
    rclcpp::spin(node);
    
    return 0;
}
```

编译：
```bash
g++ -std=c++17 test_rog_map.cpp -o test_rog_map $(pkg-config --cflags --libs rog_map)
```

运行：
```bash
./test_rog_map
```

---

### 3. 常用查询代码片段

#### 单点查询

```cpp
Vec3f pos(1.0, 2.0, 1.5);

if (map->isOccupied(pos)) {
    RCLCPP_WARN(node->get_logger(), "Occupied!");
}
```

#### 线段碰撞检测

```cpp
Vec3f start(0, 0, 1), goal(10, 10, 1);

if (map->isLineFree(start, goal)) {
    RCLCPP_INFO(node->get_logger(), "Direct path is clear");
} else {
    RCLCPP_INFO(node->get_logger(), "Path blocked");
}
```

#### 获取周围障碍物

```cpp
Vec3f robot = map->getRobotState().p;
Vec3f box_min = robot - Vec3f(5, 5, 3);
Vec3f box_max = robot + Vec3f(5, 5, 3);

vec_E<Vec3f> obstacles;
map->boxSearch(box_min, box_max, GridType::OCCUPIED, obstacles);

RCLCPP_INFO(node->get_logger(), "Found %lu obstacles", obstacles.size());
```

#### 安全距离检测

```cpp
if (map->isOccupiedInflate(drone_pos)) {
    RCLCPP_ERROR(node->get_logger(), "COLLISION!");
}
```

---

### 4. 在 RViz 中可视化

```bash
# 启动 RViz
rviz2

# 添加订阅话题：
# - rog_map/occ (PointCloud2, Red = 占据)
# - rog_map/unk (PointCloud2, Gray = 未知)
# - rog_map/frontier (PointCloud2, Yellow = 边界)
# - rog_map/map_bound (MarkerArray = 地图边界)
```

---

### 5. 配置参数

编辑 `click.yaml`：

```yaml
rog_map:
  resolution: 0.1              # 调整精度
  map_size: [60, 60, 40]       # 调整范围
  raycasting:
    enable: true
  inflation_step: 2            # 调整安全距离
```

---

## 常用场景

### 场景 1: 避障检测

**需求**：检测前方是否有障碍物

```cpp
Vec3f drone_pos = map->getRobotState().p;
Vec3f forward = drone_pos + Vec3f(5, 0, 0);  // 前方 5m

bool is_safe = map->isLineFree(drone_pos, forward, true);  // 使用膨胀地图
```

---

### 场景 2: 最近障碍物距离

**需求**：获取最近障碍物的距离

```cpp
Vec3f nearest_obs;
const double search_radius = 10.0;

if (map->getNearestCellIs(GridType::OCCUPIED, drone_pos, 
                          nearest_obs, search_radius)) {
    double dist = (nearest_obs - drone_pos).norm();
    RCLCPP_WARN(node->get_logger(), "Nearest obstacle: %.2f m", dist);
}
```

---

### 场景 3: 自主探索目标点选择

**需求**：获取边界点作为探索目标

```cpp
Vec3f robot = map->getRobotState().p;
Vec3f box_min = robot - Vec3f(20, 20, 10);
Vec3f box_max = robot + Vec3f(20, 20, 10);

vec_E<Vec3f> frontier_points;
map->boxSearch(box_min, box_max, GridType::FRONTIER, frontier_points);

if (!frontier_points.empty()) {
    Vec3f goal = frontier_points[0];  // 选择第一个边界点
    RCLCPP_INFO(node->get_logger(), "Frontier target: (%.1f, %.1f, %.1f)",
                goal.x(), goal.y(), goal.z());
}
```

---

## 故障排除

### 问题 1: 节点启动失败

**错误消息**：`Error loading config file`

**解决**：
```bash
# 检查配置文件路径
ls -l /path/to/click.yaml

# 或用完整路径
rosrun rog_map rog_map_node __params:=/path/to/click.yaml
```

---

### 问题 2: 点云不更新

**症状**：RViz 中无地图显示

**排查步骤**：
1. 检查话题是否发送：
   ```bash
   ros2 topic hz /cloud_registered
   ros2 topic hz /lidar_slam/odom
   ```

2. 检查配置：
   ```yaml
   ros_callback:
     enable: true
     cloud_topic: "/cloud_registered"
     odom_topic: "/lidar_slam/odom"
   ```

3. 查看日志：
   ```bash
   ros2 run rog_map rog_map_node --ros-args --log-level DEBUG
   ```

---

### 问题 3: 查询速度慢

**症状**：盒子搜索或线段碰撞检测耗时长

**优化**：
```yaml
# 方案 1: 降低分辨率
resolution: 0.2  # 原 0.1

# 方案 2: 减小地图
map_size: [40, 40, 30]  # 原 [60, 60, 40]

# 方案 3: 关闭不需要的功能
esdf_en: false
frontier_extraction_en: false
unk_inflation_en: false
```

---

## 关键概念

### GridType（网格类型）

```
┌─────────────────────────────────┐
│  OCCUPIED (占据)                 │  危险，无法通过
├─────────────────────────────────┤
│  FRONTIER (边界)                 │  未知-自由交界，自主探索目标
├─────────────────────────────────┤
│  UNKNOWN (未知)                  │  未探索，不安全
├─────────────────────────────────┤
│  KNOWN_FREE (自由)               │  安全通过
└─────────────────────────────────┘
```

---

### 膨胀层 (Inflate)

膨胀层为无人机体积提供安全边界：

```
原始占据:  ■
膨胀后:   ■■■
          ■■■
          ■■■
```

查询时：
- 普通查询：基于原始体素
- 膨胀查询：基于扩展体素（更保守）

**建议**：碰撞检测时总是使用 `isOccupiedInflate()` 或 `isLineFree(..., true)`

---

### 坐标系统

- **世界坐标系**：固定的全局坐标 (X, Y, Z)
- **网格索引**：整数网格坐标 (ix, iy, iz)
- **转换**：
  ```cpp
  Vec3f world_pos(1.5, 2.3, 0.8);
  Vec3i grid_idx;
  map->probMapPosToGlobalIndex(world_pos, grid_idx);
  
  // 反向转换
  Vec3f pos_back;
  map->probMapGlobalIndexToPos(grid_idx, pos_back);
  ```

---

## API 速查表

| 需求 | 函数 | 复杂度 |
|------|------|--------|
| 是否碰撞 | `isOccupied(pos)` | O(1) |
| 安全距离检测 | `isOccupiedInflate(pos)` | O(1) |
| 路径检查 | `isLineFree(start, end)` | O(N) |
| 周围障碍物 | `boxSearch(...)` | O(M) |
| 最近障碍物 | `getNearestCellIs(...)` | O(K³) |
| 获取类型 | `getGridType(pos)` | O(1) |
| 坐标转换 | `probMapPosToGlobalIndex(...)` | O(1) |

---

## 更多资源

- **完整 API 文档**: `API_REFERENCE.md`
- **配置参考**: `CONFIGURATION.md`
- **源码注释**: 参考 `include/rog_map/prob_map.h`
- **示例代码**: 参考 `test/test_dynamic_params.cpp`

