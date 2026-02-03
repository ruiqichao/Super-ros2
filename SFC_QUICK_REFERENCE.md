# 安全走廊参数快速参考卡

## 核心参数

| 参数名 | 含义 | 默认值 | 调节方向 |
|--------|------|--------|----------|
| `corridor_bound_dis` | 通道边界距离 | 1.0米 | ↑ 更安全 ↓ 更灵活 |
| `corridor_line_max_length` | 通道线最大长度 | 1.2米 | ↑ 更快 ↓ 更精 |
| `safe_corridor_line_max_length` | 安全通道最大长度 | 5.0米 | ↑ 更快 ↓ 更精 |

## 快速定位

**配置文件位置**: `src/SUPER/super_planner/config/click_real_ros2.yaml`

**参数行号**:
- `corridor_bound_dis`: 第27行
- `corridor_line_max_length`: 第28行
- `safe_corridor_line_max_length`: 第29行

## 环境推荐配置

### 狭窄环境 (障碍物间距 < 2米)
```yaml
corridor_bound_dis: 0.8        # 适应狭窄空间
corridor_line_max_length: 1.0  # 提高精度
safe_corridor_line_max_length: 6.0  # 平衡安全与精度
```

### 标准环境 (障碍物间距 2-5米)
```yaml
corridor_bound_dis: 1.0        # 标准安全边界
corridor_line_max_length: 1.2  # 平衡精度效率
safe_corridor_line_max_length: 5.0  # 适中设置
```

### 开阔环境 (障碍物间距 > 5米)
```yaml
corridor_bound_dis: 3.0        # 大安全边界
corridor_line_max_length: 3.0  # 提高效率
safe_corridor_line_max_length: 999  # 最大化效率
```

## 故障排除

### 规划失败太多？
→ 减小 `corridor_bound_dis` (如 ×0.8)

### 路径太保守？
→ 增加 `corridor_bound_dis` (如 ×1.2)

### 计算太慢？
→ 增加 `corridor_line_max_length` 和 `safe_corridor_line_max_length`

### 有碰撞风险？
→ 增加 `corridor_bound_dis` (如 ×1.5)

## 关键约束

- `corridor_bound_dis` ≥ [robot_r](file:///home/ruiqichao/ros2_demo/planner/super_ws/super_ros/src/SUPER/super_planner/config/click_real_ros2.yaml#L36-L36) (机器人半径) + 0.1米
- 高速飞行时应使用更大的 `corridor_bound_dis`