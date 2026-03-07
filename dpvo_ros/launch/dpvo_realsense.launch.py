import os.path as osp
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dpvo_path = get_package_share_directory('dpvo_ros')
    dpvo_node = Node(
        package='dpvo_ros',
        executable='dpvo_node',
        output='screen',
        namespace='dpvo_node',
        remappings=[
            ('/camera/rgb/image_color', '/camera/rgb/image_color'),
        ],
        parameters=[
            {'config': osp.join(dpvo_path, "config", "default.yaml")},
            {'calib': osp.join(dpvo_path, "calib", "tum3.txt")},
            {'network': osp.join(dpvo_path, "network", "dpvo.pth")}
        ],
    )

    ld = LaunchDescription()
    ld.add_action(dpvo_node)
    return ld
