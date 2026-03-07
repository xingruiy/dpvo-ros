import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from os.path import join
from launch.substitutions import PathJoinSubstitution
import numpy as np

sensor_types = {
    "MONOCULAR": 0,
    "STEREO": 1, 
    "RGBD": 2,
    "IMU_MONOCULAR": 3,
    "IMU_STEREO": 4,
    "IMU_RGBD": 5
}

def generate_launch_description():
    config_name = PathJoinSubstitution(
        [get_package_share_directory('ros_orb_slam3'), 'configs', 'd455.yaml'])
    vocab_file = PathJoinSubstitution(
        [get_package_share_directory('ros_orb_slam3'), 'Vocabulary', 'ORBvoc.bin'])
    sensor = 'RGBD'#LaunchConfiguration('sensor_type')

    orb_slam_node = Node(
        package='ros_orb_slam3',
        executable='orb_slam_node',
        name='orb_slam_node',
        output='screen',
        # prefix=['gdbserver localhost:3000'],
        parameters=[
            {"vocabulary_file_path": vocab_file},
            {"settings_file_path": config_name},
            {"sensor_type": sensor_types[sensor]}
        ],
        remappings=[
            ("/depth", '/camera/camera/aligned_depth_to_color/image_raw'),
            ("/image", '/camera/camera/color/image_raw'),
            ("/imu", '/camera/camera/imu'),
        ]
    )

    # camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(join(
    #         get_package_share_directory('realsense2_camera'),
    #         'launch',
    #         'rs_launch.py')),
    #     launch_arguments=[('output', 'log')])

    ld = LaunchDescription()
    ld.add_action(orb_slam_node)
    ld.add_action(DeclareLaunchArgument(
        "sensor_type", default_value="RGBD", choices=list(sensor_types.keys()),
        description="Sensor type: MONOCULAR, STEREO, RGBD, IMU_MONOCULAR, IMU_STEREO, IMU_RGBD"))
    # ld.add_action(camera)

    return ld
