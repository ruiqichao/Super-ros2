#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <iostream>

/**
 * @brief 测试SUPER规划器动态参数功能
 * @note 该测试脚本用于验证动态参数更新功能是否正常工作
 */
class DynamicParamTester : public rclcpp::Node
{
public:
    DynamicParamTester() : Node("dynamic_param_tester")
    {
        // 创建参数客户端，连接到fsm_node
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "fsm_node");
        
        // 等待参数服务可用
        while (!param_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the fsm_node parameter service...");
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected to fsm_node parameter service.");
        
        // 设置定时器来测试动态参数更新
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&DynamicParamTester::testDynamicParams, this)
        );
    }

private:
    void testDynamicParams()
    {
        static int test_count = 0;
        
        if (test_count == 0) {
            RCLCPP_INFO(this->get_logger(), "=== 开始测试动态参数更新功能 ===");
            
            // 测试更新max_vel参数
            std::vector<rclcpp::Parameter> parameters;
            parameters.push_back(rclcpp::Parameter("traj_opt.boundary.max_vel", 2.0));
            parameters.push_back(rclcpp::Parameter("traj_opt.exp_traj.penna_vel", 2.0e+5));
            
            RCLCPP_INFO(this->get_logger(), "正在设置动态参数: max_vel=2.0, penna_vel=2.0e+5");
            
            auto future = param_client_->set_parameters(parameters);
            future.wait();
            
            auto results = future.get();
            for (auto & result : results) {
                if (result.successful) {
                    RCLCPP_INFO(this->get_logger(), "参数设置成功!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "参数设置失败: %s", result.reason.c_str());
                }
            }
        } else if (test_count == 1) {
            // 测试更新其他参数
            std::vector<rclcpp::Parameter> parameters;
            parameters.push_back(rclcpp::Parameter("traj_opt.boundary.max_acc", 8.0));
            parameters.push_back(rclcpp::Parameter("traj_opt.exp_traj.penna_acc", 2.0e+5));
            
            RCLCPP_INFO(this->get_logger(), "正在设置动态参数: max_acc=8.0, penna_acc=2.0e+5");
            
            auto future = param_client_->set_parameters(parameters);
            future.wait();
            
            auto results = future.get();
            for (auto & result : results) {
                if (result.successful) {
                    RCLCPP_INFO(this->get_logger(), "参数设置成功!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "参数设置失败: %s", result.reason.c_str());
                }
            }
        } else if (test_count == 2) {
            // 测试获取参数值
            RCLCPP_INFO(this->get_logger(), "正在获取参数值...");
            
            auto future = param_client_->get_parameters({
                "traj_opt.boundary.max_vel",
                "traj_opt.boundary.max_acc"
            });
            future.wait();
            
            auto values = future.get();
            for (auto & value : values) {
                RCLCPP_INFO(this->get_logger(), "参数 %s = %f", 
                           value.get_name().c_str(), 
                           value.as_double());
            }
            
            RCLCPP_INFO(this->get_logger(), "=== 动态参数测试完成 ===");
            timer_->cancel();
        }
        
        test_count++;
    }

    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DynamicParamTester>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}