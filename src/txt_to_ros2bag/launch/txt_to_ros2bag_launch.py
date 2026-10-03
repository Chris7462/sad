from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param = join(
        get_package_share_directory('txt_to_ros2bag'), 'param', 'txt_to_ros2bag.yaml'
    )

    txt_to_ros2bag_node = Node(
        package='txt_to_ros2bag',
        executable='txt_to_ros2bag_node',
        name='txt_to_ros2bag_node',
        output='screen',
        parameters=[param]
    )

    return LaunchDescription([
        txt_to_ros2bag_node
    ])
