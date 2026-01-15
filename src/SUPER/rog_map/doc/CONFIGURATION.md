# ROG-Map 配置文档

## 概览

ROG-Map 通过 YAML 配置文件进行参数化，所有参数在节点初始化时加载，**不支持运行时动态修改**。

---

## 配置文件位置

```
rog_map/config/
├── visualization.cfg    # 可视化配置
└── (其他配置文件)
```

---

## 完整配置参数

### 1. 地图分辨率

```yaml
rog_map:
  resolution: 0.1              # 基础分辨率 (m)
  inflation_resolution: 0.1    # 膨胀层分辨率 (m)
                               # 通常 ≥ resolution
```

**说明**：
- `resolution`: 概率地图的体素大小
- `inflation_resolution`: 膨胀处理的分辨率
- 较小值 = 更精细但计算量大

---

### 2. 地图尺寸

```yaml
  map_size: [60, 60, 40]      # 地图大小 [X, Y, Z] (m)
```

**说明**：
- 定义本地地图的立方体范围
- 地图中心与机器人位置同步（滑动窗口）
- 较大的地图内存占用↑，计算量↑

---

### 3. 地图滑动

```yaml
  map_sliding:
    enable: true              # 启用滑动窗口
    threshold: 1.0            # 触发滑动的距离阈值 (m)
  
  fix_map_origin: [0, 0, 0]   # 固定地图原点（禁用滑动时）
```

**说明**：
- `enable=true`: 地图跟随机器人移动
- `enable=false`: 地图固定在 `fix_map_origin`
- `threshold`: 机器人移动此距离后触发地图重新中心化

---

### 4. 光线投射 (Raycasting)

```yaml
  raycasting:
    enable: true              # 启用光线投射更新自由空间
    ray_range: [0.3, 10]      # 光线范围 [最小, 最大] (m)
    p_hit: 0.70               # 击中概率增益
    p_miss: 0.30              # 未击中概率衰减
    p_min: 0.12               # 最小概率值
    p_max: 0.97               # 最大概率值
    p_occ: 0.80               # 占据阈值
    p_free: 0.30              # 自由阈值
    batch_update_size: 1      # 批处理大小
    unk_thresh: 0.70          # 未知判断阈值
```

**概率论理**：
- 点云中被击中的射线末尾：占有度 += `p_hit`
- 被遍历但未击中的射线：占有度 -= `p_miss`
- 占有度范围 [0, 1]，其中：
  - `[0, p_free)`: 已知自由
  - `[p_free, p_occ)`: 未知
  - `[p_occ, 1]`: 占据

---

### 5. 膨胀配置

```yaml
  inflation_step: 1           # 基础膨胀步数
  unk_inflation_en: false     # 启用未知膨胀
  unk_inflation_step: 1       # 未知膨胀步数
```

**说明**：
- `inflation_step`: 从占据体素向外膨胀的网格层数
- `unk_inflation_en=true`: 对未知区域也进行膨胀处理
- 膨胀用于无人机体积的安全边界

---

### 6. 可视化

```yaml
  visualization:
    enable: true              # 启用可视化
    frame_id: "world"         # TF 坐标系名
    range: [30, 30, 20]       # 可视化范围 (m)
    time_rate: 10.0           # 可视化频率 (Hz)
    frame_rate: 0             # 帧率（0 = 无限制）
    pub_unknown_map_en: true  # 发布未知地图
    use_dynamic_reconfigure: false  # 动态重配置（未实现）
```

---

### 7. ESDF (Euclidean Signed Distance Field)

```yaml
  esdf:
    enable: false             # 启用 ESDF 距离场
    resolution: 0.2           # ESDF 分辨率
    local_update_box: [10, 10, 5]  # 本地更新范围 (m)
```

**说明**：
- ESDF 计算每个网格到障碍物的精确欧几里得距离
- 开销大（~30% 计算时间），按需启用
- 适用于梯度下降优化的轨迹规划

---

### 8. 边界提取 (Frontier)

```yaml
  frontier_extraction_en: true  # 启用边界提取
```

**说明**：
- 识别已知-未知的交界边界
- 用于自主探索中的目标点选择

---

### 9. ROS2 回调

```yaml
  ros_callback:
    enable: true              # 启用 ROS2 话题回调
    cloud_topic: "/cloud_registered"     # 点云话题
    odom_topic: "/lidar_slam/odom"       # 里程计话题
    odom_timeout: 0.05        # 里程计超时 (s)
```

**说明**：
- `enable=false`: 手动调用 `updateMap()`，不使用 ROS2 回调
- `odom_timeout`: 点云与里程计的最大延迟容限

---

### 10. 点云处理

```yaml
  point_filt_num: 2           # 点云下采样因子
  intensity_thresh: -1        # 光学强度过滤 (-1 = 禁用)
  load_pcd_en: false          # 从 PCD 文件加载初始地图
  pcd_name: "map.pcd"
```

**说明**：
- `point_filt_num=2`: 每 2 个点处理 1 个（降低计算量）
- `intensity_thresh`: LiDAR 反射强度阈值（滤除噪声）

---

## 配置示例

### 低延迟配置（实时导航）

```yaml
rog_map:
  resolution: 0.2             # 粗分辨率，快速查询
  map_size: [40, 40, 30]      # 较小地图
  raycasting:
    enable: true
    batch_update_size: 5
  inflation_step: 2
  unk_inflation_en: false
  esdf_en: false              # 禁用 ESDF
  frontier_extraction_en: false
  point_filt_num: 3
```

---

### 高精度配置（探索模式）

```yaml
rog_map:
  resolution: 0.05            # 精细分辨率
  map_size: [80, 80, 50]      # 大地图范围
  raycasting:
    enable: true
    batch_update_size: 1
  inflation_step: 2
  unk_inflation_en: true
  unk_inflation_step: 1
  esdf_en: true               # 启用距离场
  frontier_extraction_en: true # 启用边界提取
  point_filt_num: 1           # 无下采样
```

---

### 仿真配置

```yaml
rog_map:
  resolution: 0.1
  map_size: [50, 50, 40]
  map_sliding:
    enable: true
    threshold: 1.0
  raycasting:
    enable: true
    ray_range: [0.5, 50]
  visualization:
    enable: true
    time_rate: 20
  ros_callback:
    enable: true
    cloud_topic: "/unified_cloud"
    odom_topic: "/unifiednav/odometry"
```

---

## 性能指标

### 根据配置的典型性能

| 配置 | 分辨率 | 地图尺寸 | 更新频率 | 查询耗时 |
|------|--------|---------|---------|---------|
| 低延迟 | 0.2m | 40×40×30 | ~50Hz | <1ms |
| 平衡 | 0.1m | 60×60×40 | ~30Hz | ~2ms |
| 高精度 | 0.05m | 80×80×50 | ~10Hz | ~5ms |

---

## 常见问题

### Q1: 如何调整安全距离？

**A**: 修改膨胀参数：

```yaml
inflation_step: 3           # 增大此值扩大安全边界
```

膨胀距离 ≈ `inflation_step × inflation_resolution`

---

### Q2: 如何禁用滑动地图？

**A**:

```yaml
map_sliding:
  enable: false
fix_map_origin: [0, 0, 0]   # 固定在此点
```

---

### Q3: 点云更新太慢怎么办？

**A**: 优化配置：

1. 增加 `point_filt_num`（跳过更多点）
2. 降低 `resolution`（减少网格总数）
3. 减小 `map_size`（减少计算范围）
4. 关闭 `esdf_en` 和 `frontier_extraction_en`
5. 禁用 `unk_inflation_en`

---

### Q4: 为什么关闭点云订阅后仍有地图更新？

**A**: 检查是否手动调用了 `updateMap()`：

```cpp
if (map_ptr_) {
    map_ptr_->updateMap(cloud, pose);
}
```

如需完全禁用自动更新：

```yaml
ros_callback:
  enable: false
```

---

## 调试提示

### 打印地图信息

启用时间日志记录查看性能：

```cpp
std::ofstream log_file("map_info.log");
map_ptr->writeMapInfoToLog(log_file);
map_ptr->writeTimeConsumingToLog(log_file);
```

### 可视化调试

在 RViz 中订阅以下话题观察地图状态：

- `rog_map/occ` - 占据体素
- `rog_map/unk` - 未知体素  
- `rog_map/inf_occ` - 膨胀占据
- `rog_map/map_bound` - 地图边界

