# SUPER 项目文档

## 概述

此目录包含SUPER项目的相关技术文档，涵盖系统架构、算法说明和配置参数详解。

## 文档列表

### [command_topics.md](command_topics.md)
- 说明系统的两个主要命令话题
- `cmd_topic` (即时位置命令) 与 `mpc_cmd_topic` (MPC轨迹命令) 的区别
- 消息类型和系统架构关系

### [astar_heuristic_types.md](astar_heuristic_types.md)
- A*路径搜索算法的三种启发式类型详解
- DIAG (对角线距离)、MANH (曼哈顿距离)、EUCL (欧几里得距离) 的实现和比较
- 各类型在无人机路径规划中的应用场景

### [trajectory_types.md](trajectory_types.md)
- 预期轨迹与备用轨迹的概念说明
- 两组轨迹优化参数的区别和应用场景
- 双轨迹策略的设计理念

## 贡献

如需添加或更新文档，请确保：
1. 保持内容准确并与代码实现一致
2. 提供清晰的技术说明和实际应用场景
3. 使用标准Markdown格式