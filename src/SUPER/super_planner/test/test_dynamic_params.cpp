#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "super_core/super_planner.h"
#include "ros_interface/ros2/fsm_ros2.hpp"
#include "traj_opt/yaw_traj_config.h"
#include "path_search/astar_config.h"
#include "super_core/corridor_generator_config.h"

class DynamicParamTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 初始化ROS2节点用于测试
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        // 创建一个简单的节点用于参数测试
        test_node_ = std::make_shared<rclcpp::Node>("test_dynamic_param_node");
    }

    void TearDown() override {
        test_node_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    rclcpp::Node::SharedPtr test_node_;
};

TEST_F(DynamicParamTest, YawTrajConfigInitialization) {
    traj_opt::YawTrajConfig config;

    // 测试默认值
    EXPECT_NEAR(config.yaw_dot_max, 3.14, 1e-6);

    // 测试参数更新
    std::vector<rclcpp::Parameter> params = {
        rclcpp::Parameter("traj_opt.yaw_traj.yaw_dot_max", 2.0)
    };

    auto result = config.dynamicParametersCallback(params);
    EXPECT_TRUE(result.successful);

    EXPECT_NEAR(config.yaw_dot_max, 2.0, 1e-6);
}

TEST_F(DynamicParamTest, AstarConfigInitialization) {
    path_search::AstarConfig config;

    // 测试默认值
    EXPECT_EQ(config.heu_type, 0);
    EXPECT_FALSE(config.allow_diag);
    EXPECT_FALSE(config.debug_visualization_en);

    // 测试参数更新
    std::vector<rclcpp::Parameter> params = {
        rclcpp::Parameter("astar.heu_type", 2),
        rclcpp::Parameter("astar.allow_diag", true),
        rclcpp::Parameter("astar.debug_visualization_en", true)
    };

    auto result = config.dynamicParametersCallback(params);
    EXPECT_TRUE(result.successful);

    EXPECT_EQ(config.heu_type, 2);
    EXPECT_TRUE(config.allow_diag);
    EXPECT_TRUE(config.debug_visualization_en);
}

TEST_F(DynamicParamTest, CorridorGenConfigInitialization) {
    super_planner::CorridorGenConfig config;

    // 测试默认值
    EXPECT_NEAR(config.bound_dis, 3.0, 1e-6);
    EXPECT_NEAR(config.robot_r, 0.3, 1e-6);
    EXPECT_EQ(config.iris_iter_num, 1);

    // 测试参数更新
    std::vector<rclcpp::Parameter> params = {
        rclcpp::Parameter("super_planner.corridor_bound_dis", 5.0),
        rclcpp::Parameter("super_planner.robot_r", 0.5),
        rclcpp::Parameter("super_planner.iris_iter_num", 3)
    };

    auto result = config.dynamicParametersCallback(params);
    EXPECT_TRUE(result.successful);

    EXPECT_NEAR(config.bound_dis, 5.0, 1e-6);
    EXPECT_NEAR(config.robot_r, 0.5, 1e-6);
    EXPECT_EQ(config.iris_iter_num, 3);
}

TEST_F(DynamicParamTest, ConfigMutexAccess) {
    traj_opt::YawTrajConfig yaw_config;
    path_search::AstarConfig astar_config;
    super_planner::CorridorGenConfig corridor_config;

    // 测试互斥锁访问
    std::mutex &yaw_mutex = yaw_config.getConfigMutex();
    std::mutex &astar_mutex = astar_config.getConfigMutex();
    std::mutex &corridor_mutex = corridor_config.getConfigMutex();

    // 尝试锁定互斥锁
    EXPECT_NO_THROW({
        std::lock_guard<std::mutex> lock1(yaw_mutex);
        std::lock_guard<std::mutex> lock2(astar_mutex);
        std::lock_guard<std::mutex> lock3(corridor_mutex);
        });
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
