import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    default_config_name = 'click_real_ros2.yaml'

    declare_config_name_cmd = DeclareLaunchArgument(
        'config_name', default_value=default_config_name,
        description='Yaml config file name'
    )

    config_name = LaunchConfiguration('config_name')

    ld = LaunchDescription()
    ld.add_action(declare_config_name_cmd)

    # FSM节点 - 轨迹规划系统的主入口
    fsm_node = Node(
        package='super_planner',
        executable='fsm_node',
        output='screen',
        parameters=[{
            'config_name': config_name,
        }]
    )
    ld.add_action(fsm_node)

    return ld