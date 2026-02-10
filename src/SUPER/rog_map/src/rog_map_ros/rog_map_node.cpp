#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "../include/rog_map_ros/rog_map_ros2_dynamic.hpp"

namespace rog_map {

class ROGMapNode : public rclcpp::Node {
public:
    explicit ROGMapNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("rog_map_node", options) {
        
        // 获取配置文件路径参数
        this->declare_parameter("config_file", "");
        config_file_ = this->get_parameter("config_file").as_string();
        
        if (config_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Config file path is not provided!");
            throw std::runtime_error("Config file path is required");
        }

        // 创建ROGMapROSDynamic实例，该类已包含完整的动态参数回调功能
        try {
            map_ = std::make_shared<ROGMapROSDynamic>(this->shared_from_this(), config_file_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ROGMap: %s", e.what());
            throw;
        }
        
        RCLCPP_INFO(this->get_logger(), "ROGMapNode initialized with dynamic parameter callbacks");
    }

private:
    std::shared_ptr<ROGMapROSDynamic> map_;
    std::string config_file_;
};

} // namespace rog_map

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rog_map::ROGMapNode)