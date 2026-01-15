# SUPER项目错误处理机制分析与改进方案

## 1. 当前错误处理机制分析

### 1.1 RET_CODE枚举定义
当前项目使用整数枚举作为返回码系统：

```cpp
enum RET_CODE {
    /// FOR Planner
    FAILED = 0,      // 失败
    NO_NEED = 1,     // 无需执行
    SUCCESS = 2,     // 成功
    FINISH = 3,      // 完成
    NEW_TRAJ = 4,    // 新轨迹
    EMER = 5,        // 紧急情况
    OPT_FAILED = 6,  // 优化失败
    INIT_ERROR = 7,  // 初始化错误

    /// FOR path search
    REACH_HORIZON,   // 到达边界
    REACH_GOAL,      // 到达目标
    NO_PATH,         // 无路径
    TIME_OUT         // 超时
};
```

### 1.2 错误处理特点

#### 1.2.1 返回码系统
- **整数返回码**: 使用整数枚举而非布尔值，提供更丰富的状态信息
- **混合语义**: 返回码既包含错误状态（FAILED、OPT_FAILED），也包含正常状态（SUCCESS、FINISH、REACH_GOAL）
- **状态传播**: 通过返回码在函数调用链中传播状态信息

#### 1.2.2 实际应用示例
在`SuperPlanner::PlanFromRest`函数中的错误处理：
```cpp
// 检查里程计数据
if (robot_state_.rcv == false) {
    ros_ptr_->warn(" -- [SUPER] in [PlanFromRest]: No odom, force return.");
    latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_ODOM);
    return FAILED;  // 返回失败状态
}

// 检查起始点有效性
if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_star_pt, 3.0)) {
    ros_ptr_->error(" -- [SUPER] in [PlanFromRest] Local start point is deeply occupied...");
    latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
    return FAILED;  // 返回失败状态
}
```

#### 1.2.3 错误处理模式
- **早期返回**: 检测到错误立即返回
- **日志记录**: 使用ROS2日志系统记录错误信息
- **状态设置**: 更新内部状态以反映错误情况

## 2. 当前机制的优势

### 2.1 实时系统兼容性
- **确定性**: 整数返回码避免了异常处理的不确定性
- **性能**: 无异常抛出和捕获开销
- **可预测**: 执行路径可预测，适合实时系统

### 2.2 丰富的状态信息
- **多种状态**: 不仅是成功/失败，还包括完成、紧急、新轨迹等多种状态
- **细粒度控制**: 调用方可以根据不同返回码采取不同措施

### 2.3 简单直观
- **易于理解**: 整数返回码概念简单
- **易于调试**: 返回值可以直接打印和检查
- **兼容性强**: 与C/C++传统编程风格一致

## 3. 当前机制的局限性

### 3.1 语义混淆
- **错误与状态混合**: REACH_GOAL等返回码表示正常状态而非错误
- **缺乏清晰分类**: 无法直观区分错误、警告和正常状态

### 3.2 错误信息贫乏
- **上下文缺失**: 返回码本身不携带错误原因或上下文信息
- **调试困难**: 仅凭返回码难以定位具体问题

### 3.3 错误处理繁琐
- **手动检查**: 每一层都需要手动检查返回码
- **重复代码**: 错误处理代码重复出现

### 3.4 资源管理风险
- **RAII不完整**: 未在所有地方强制使用RAII原则
- **清理不完整**: 在某些错误路径上可能存在资源泄漏

## 4. 预期改进方案

### 4.1 现代化错误处理

#### 4.1.1 使用std::expected (C++23)
```cpp
#include <expected>

enum class PlanningError {
    NoOdom,
    NoStartPoint,
    OptimizationFailed,
    Timeout
};

using PlanningResult = std::expected<Trajectory, PlanningError>;

PlanningResult planFromRest(const Vec3f& goal_p, const double& goal_yaw, const bool& new_goal) {
    if (robot_state_.rcv == false) {
        return std::unexpected(PlanningError::NoOdom);
    }
    
    // ... 其他逻辑
    return trajectory;  // 成功返回
}
```

#### 4.1.2 强类型错误枚举
```cpp
enum class PlannerError : int {
    Success = 0,
    Failed = -1,
    NoPath = -2,
    // ... 其他错误码
};

enum class PlannerStatus : int {
    ReachGoal = 1,
    Finish = 2,
    NewTraj = 3,
    // ... 其他状态码
};
```

### 4.2 增强错误上下文
```cpp
struct ErrorContext {
    std::string function_name;
    std::string error_message;
    std::string file;
    int line;
    double timestamp;
    // 可能还有其他上下文信息
};

class EnhancedResult {
private:
    RET_CODE code_;
    std::optional<ErrorContext> context_;
    
public:
    RET_CODE getCode() const { return code_; }
    bool isSuccess() const { return code_ >= SUCCESS; }
    const std::optional<ErrorContext>& getContext() const { return context_; }
    
    static EnhancedResult success() { 
        return EnhancedResult{SUCCESS, std::nullopt}; 
    }
    
    static EnhancedResult failure(RET_CODE code, const std::string& msg, 
                                 const std::string& func, const std::string& file, int line) {
        ErrorContext ctx{func, msg, file, line, getCurrentTimestamp()};
        return EnhancedResult{code, ctx};
    }
};
```

### 4.3 改进的错误处理模式

#### 4.3.1 错误处理宏
```cpp
#define CHECK_AND_RETURN(condition, error_code, message) \
    if (!(condition)) { \
        if constexpr (cfg_.detailed_log_en) { \
            ros_ptr_->error(" -- [SUPER] {}: {} in {} at {}:{}", #condition, message, __FUNCTION__, __FILE__, __LINE__); \
        } \
        return error_code; \
    }

// 使用示例
CHECK_AND_RETURN(robot_state_.rcv, FAILED, "No odom data received");
CHECK_AND_RETURN(map_ptr_->isValid(), INIT_ERROR, "Map pointer is invalid");
```

#### 4.3.2 RAII增强
```cpp
class ScopedResourceGuard {
public:
    template<typename Resource, typename CleanupFunc>
    ScopedResourceGuard(Resource& resource, CleanupFunc cleanup_func) 
        : resource_(resource), cleanup_func_(cleanup_func), active_(true) {}
    
    ~ScopedResourceGuard() {
        if (active_) {
            cleanup_func_(resource_);
        }
    }
    
    void release() { active_ = false; }
    
private:
    decltype(auto) resource_;
    std::function<void(decltype(auto))> cleanup_func_;
    bool active_;
};
```

## 5. 改进实施策略

### 5.1 渐进式改进
1. **保持兼容性**: 在不破坏现有接口的情况下引入新机制
2. **逐步替换**: 从关键模块开始逐步改进错误处理
3. **保留旧接口**: 为向后兼容保留原有返回码接口

### 5.2 具体改进步骤

#### 5.2.1 第一阶段：错误分类
- 区分错误码和状态码
- 创建专门的错误枚举和状态枚举

#### 5.2.2 第二阶段：上下文增强
- 为关键函数添加错误上下文
- 增强日志记录能力

#### 5.2.3 第三阶段：现代化改造
- 引入现代C++错误处理机制
- 改进资源管理

## 6. 当前与预期的对比总结

| 方面 | 当前机制 | 预期改进 |
|------|----------|----------|
| 类型系统 | 整数枚举 | 强类型枚举/expected |
| 语义清晰度 | 混合错误和状态 | 分离错误和状态 |
| 错误信息 | 仅返回码 | 包含上下文信息 |
| 资源管理 | 部分RAII | 完整RAII |
| 性能 | 优秀 | 保持性能的同时增强功能 |
| 可维护性 | 一般 | 显著提升 |
| 调试友好性 | 有限 | 大幅改进 |

## 7. 推荐的改进优先级

### 高优先级
1. 分离错误码和状态码，使语义更清晰
2. 增强错误日志，包含更多上下文信息
3. 完善资源管理，确保所有路径都正确清理资源

### 中优先级
1. 引入错误处理辅助宏，减少重复代码
2. 改进配置加载的错误处理
3. 增强地图访问的错误检查

### 低优先级
1. 逐步引入现代C++错误处理机制
2. 重构返回码系统为更现代的形式

这样的改进既能保持当前机制的优点（性能、实时性），又能弥补其不足（语义不清、上下文缺失、维护困难），使错误处理更加健壮和易维护。