from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value='/home/yi-chen/Research/sad/sad_ch3_10_bag',
        description='Path to the bag created by txt_to_ros2bag'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Bag playback rate'
    )

    gnss_ros_node = Node(
        package='ch3_ros',
        executable='gnss_ros_node',
        name='gnss_ros_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'gnss_topic': '/sad/gnss',
            'antenna_angle': 12.06,
            'antenna_pos_x': -0.17,
            'antenna_pos_y': -0.20
        }]
    )

    trajectory_node = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        name='trajectory_server_node',
        namespace='gnss_trajectory',
        parameters=[{
            'use_sim_time': True,
            'target_frame_name': 'map',
            'source_frame_name': 'gnss_link',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 10.0
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(get_package_share_directory('ch3_ros'), 'rviz', 'gnss_ros.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    bag_exec = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
             '--clock', '-r', LaunchConfiguration('rate')],
        output='screen'
    )

    return LaunchDescription([
        bag_arg,
        rate_arg,
        gnss_ros_node,
        trajectory_node,
        rviz_node,
        TimerAction(
            period=3.0,  # give the nodes time to subscribe before playback starts
            actions=[
                bag_exec
            ]
        )
    ])
