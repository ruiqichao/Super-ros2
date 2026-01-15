# A* 启发式类型说明

## 概述

在SUPER项目的A*路径搜索算法中，提供了三种不同的启发式距离计算方法，通过 `heu_type` 参数进行配置。这些启发式类型影响路径搜索的效率和结果。

## 配置参数

在配置文件中（如 `click_smooth_ros2.yaml`）：

```yaml
astar:
  heu_type: 2                                  # 启发式类型: 0 对角线, 1 曼哈顿, 2 欧几里得
```

## 启发式类型详解

### 1. DIAG (对角线距离, type=0)

#### 代码实现
```cpp
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
```

#### 特点
- 适用于允许对角线移动的3D网格
- 计算考虑了三种类型的移动：对角线移动（3D对角线，长度为√3）、面对角线移动（长度为√2）和轴向移动（长度为1）
- 在3D空间中，考虑了同时在x、y、z三个方向上移动的情况
- 更精确地反映了在允许对角线移动的网格中的实际最短路径距离

### 2. MANH (曼哈顿距离, type=1)

#### 代码实现
```cpp
case MANH: {
    // 曼哈顿距离
    double dx = std::abs(node1->id_g(0) - node2->id_g(0));
    double dy = std::abs(node1->id_g(1) - node2->id_g(1));
    double dz = std::abs(node1->id_g(2) - node2->id_g(2));

    return tie_breaker_ * (dx + dy + dz);
}
```

#### 特点
- 又称"城市街区距离"或"L1距离"
- 计算方式是各坐标轴距离的总和：|x₁-x₂| + |y₁-y₂| + |z₁-z₂|
- 适用于只允许轴向移动（前后左右上下）而不允许对角线移动的网格
- 保证是可接受的启发式（admissible），即不会高估实际距离
- 计算简单，速度快
- 在不允许对角线移动的情况下是最精确的启发式

### 3. EUCL (欧几里得距离, type=2)

#### 代码实现
```cpp
case EUCL: {
    // 欧几里得距离
    return tie_breaker_ * (node2->id_g - node1->id_g).norm();
}
```

#### 特点
- 计算两点之间的直线距离（L2距离）
- 公式为：√[(x₁-x₂)² + (y₁-y₂)² + (z₁-z₂)²]
- 最符合实际空间的几何距离
- 在允许任意方向移动的连续空间中是最精确的启发式
- 对于网格地图，它是实际路径距离的良好近似
- 由于网格限制，实际路径通常比直线距离更长，但仍然是可接受的启发式

## 比较与应用场景

### 启发式精确度
- EUCL ≤ 实际距离 ≤ DIAG ≤ MANH（在允许对角线移动的网格中）
- 欧几里得距离通常提供最精确的估计

### 计算复杂度
- MANH：最低（只需加法和绝对值）
- DIAG：中等（涉及乘法、开方和条件判断）
- EUCL：较高（涉及平方、加法和开方）

### 搜索效率
- EUCL：通常搜索效率最高（扩展节点最少）
- DIAG：中等
- MANH：可能扩展更多节点（估计值偏小）

### 在当前项目中的应用
- `heu_type: 2`（欧几里得距离）是默认设置，因为对于无人机路径规划，欧几里得距离提供了最接近实际飞行距离的估计
- 使用 `tie_breaker_ = 1.0 + 1e-5` 来打破距离相等节点的平局，优先选择距离目标更近的节点

## tie_breaker的作用
代码中使用了 `tie_breaker_ = 1.0 + 1e-5`，这是一个略大于1的系数，用于在A*算法中处理f值相等的节点时，优先选择h值（到目标的启发式距离）更小的节点，从而使得搜索方向更直接地朝向目标点。