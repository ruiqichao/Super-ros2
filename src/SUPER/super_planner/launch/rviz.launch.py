from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('super_planner')
    default_rviz_config_path = package_path + '/rviz/super_planner.rviz'

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_config_path', default_value=default_rviz_config_path,
        description='Full path to the RViz config file to use'
    )

    rviz_config_path = LaunchConfiguration('rviz_config_path')

    ld = LaunchDescription()
    ld.add_action(declare_rviz_config_path_cmd)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    ld.add_action(rviz_node)

    return ld