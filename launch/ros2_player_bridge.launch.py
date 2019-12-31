import launch_ros.actions
import launch.actions

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('player_ip',
                                             default_value='127.0.0.1',
                                             description="Player server IP address"),
        launch_ros.actions.Node(package='ros2_player_bridge',
                                node_executable='mobile_base_node',
                                output='screen',
                                arguments=[LaunchConfiguration('player_ip')]),
        launch_ros.actions.Node(package='ros2_player_bridge',
                                node_executable='sensors_node',
                                output='screen',
                                arguments=[LaunchConfiguration('player_ip')]),
    ])