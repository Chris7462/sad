from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motion_node = Node(
        package='ch2_ros',
        executable='ch2_ros_node',
        name='ch2_ros_node'
    )

    trajectory1_node = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        name='trajectory_server_node',
        namespace='se3_trajectory',
        parameters=[{
            'target_frame_name': 'map',
            'source_frame_name': 'se3_link',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 10.0
        }]
    )

    trajectory2_node = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        name='trajectory_server_node',
        namespace='rpt_trajectory',
        parameters=[{
            'target_frame_name': 'map',
            'source_frame_name': 'rpt_link',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 10.0
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(get_package_share_directory('ch2_ros'), 'rviz', 'motion.rviz')]
    )

    return LaunchDescription([
        motion_node,
        trajectory1_node,
        trajectory2_node,
        rviz_node
    ])
