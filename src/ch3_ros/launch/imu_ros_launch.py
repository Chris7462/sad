from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_ros_node = Node(
        package='ch3_ros',
        executable='imu_ros_node',
        name='imu_ros_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'imu_topic': '/sad/imu'
        }]
    )

    trajectory_node = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        name='trajectory_server_node',
        namespace='imu_trajectory',
        parameters=[{
            'target_frame_name': 'map',
            'source_frame_name': 'imu_link',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 10.0,
            'use_sim_time': True,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(get_package_share_directory('ch3_ros'), 'rviz', 'imu_ros.rviz')],
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        imu_ros_node,
        trajectory_node,
        rviz_node
    ])
