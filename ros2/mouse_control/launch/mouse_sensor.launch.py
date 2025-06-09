from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('mouse_control'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='mouse_control',
            executable='mouse_sensor',
            name='mouse_sensor',
            parameters = [config_path],
            arguments=['--ros-args', '--log-level', 'debug']
        )
    ])