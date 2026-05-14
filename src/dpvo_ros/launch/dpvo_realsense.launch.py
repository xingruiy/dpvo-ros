import os.path as osp
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dpvo_path = get_package_share_directory('dpvo_ros')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    config = LaunchConfiguration('config')
    calib = LaunchConfiguration('calib')
    network = LaunchConfiguration('network')
    loop_closure = LaunchConfiguration('loop_closure')
    classic_loop_closure = LaunchConfiguration('classic_loop_closure')

    dpvo_node = Node(
        package='dpvo_ros',
        executable='dpvo_node',
        output='screen',
        namespace='dpvo_node',
        parameters=[
            {'image_topic': image_topic},
            {'camera_info_topic': camera_info_topic},
            {'config': config},
            {'calib': calib},
            {'network': network},
            {'loop_closure': loop_closure},
            {'classic_loop_closure': classic_loop_closure},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/rgb/image_color',
        description='Color image topic consumed by DPVO. For realsense2_camera this is often /camera/camera/color/image_raw.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/rgb/camera_info',
        description='Camera info topic used to initialize DPVO intrinsics.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'config',
        default_value=osp.join(dpvo_path, "config", "default.yaml"),
        description='DPVO config yaml file.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'calib',
        default_value=osp.join(dpvo_path, "calib", "tum3.txt"),
        description='DPVO camera calibration txt file.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'network',
        default_value=osp.join(dpvo_path, "network", "dpvo.pth"),
        description='DPVO checkpoint file.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'loop_closure',
        default_value='false',
        description='Enable DPVO loop-closure factors.',
    ))
    ld.add_action(DeclareLaunchArgument(
        'classic_loop_closure',
        default_value='false',
        description='Enable classic long-term loop closure.',
    ))
    ld.add_action(dpvo_node)
    return ld
