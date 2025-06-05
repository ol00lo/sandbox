from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mouse_sensor_node = Node(
        package='mouse_control',
        executable='mouse_sensor',
        name='mouse_sensor',
    )

    return LaunchDescription([
        mouse_sensor_node
    ])