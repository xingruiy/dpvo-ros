import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from os.path import join
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [get_package_share_directory('ros_orb_slam3'), 'configs', 'default.rviz'])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(
            get_package_share_directory('realsense2_camera'),
            'launch',
            'rs_launch.py')),
        launch_arguments=[('output', 'log')])

    ld = LaunchDescription()
    ld.add_action(camera)
    ld.add_action(rviz_node)

    return ld
