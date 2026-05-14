import torch
import numpy as np
import rclpy
from dpvo_ros.dpvo.config import cfg
# from dpvo_ros.dpvo.plot_utils import plot_trajectory, save_output_for_COLMAP, save_ply
# from dpvo_ros.dpvo.stream import image_stream, video_stream
from dpvo_ros.dpvo.utils import Timer
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header


class DpVOInterfaceNode(Node):
    def __init__(self):
        super().__init__('dpvo_node')
        self.declare_parameter('message_queue_size', 100)
        self.declare_parameter('network', rclpy.Parameter.Type.STRING)
        self.declare_parameter('config', rclpy.Parameter.Type.STRING)
        self.declare_parameter('image_topic', '/camera/rgb/image_color')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        cfg.merge_from_file(self.get_parameter(
            'config').get_parameter_value().string_value)
        # self.get_logger().info(f"dpvo_ROS node config: {cfg}")

        self.network = self.get_parameter(
            'network').get_parameter_value().string_value

        self.intr_mat = None

        image_topic = self.get_parameter(
            'image_topic').get_parameter_value().string_value
        self.image_sub_ = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        camera_info_topic = self.get_parameter(
            'camera_info_topic').get_parameter_value().string_value
        self.camera_info_sub_ = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.pub_pcd = self.create_timer(2, self.point_cloud_publish_callback)
        self.pose_pub_ = self.create_publisher(PoseStamped, "/camera/estimate_pose", 10)
        self.render_pub_ = self.create_publisher(Image, '/camera/render', 10)
        self.map_pub_ = self.create_publisher(PointCloud2, '/map/point_cloud', 10)

        self.impl_ = None
        self.get_logger().info(f"DPVO_ROS node initialized! Subscribed to {image_topic}")

    def camera_info_callback(self, msg: CameraInfo):
        if self.impl_ is not None:
            return
        # K is row-major 3x3: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.intr_mat = np.array([msg.k[0], msg.k[4], msg.k[2], msg.k[5]])
        from dpvo_ros.dpvo.dpvo import DPVO
        self.impl_ = DPVO(cfg, self.network, ht=msg.height, wd=msg.width, viz=False)
        self.get_logger().info(f"DPVO initialized: {msg.width}x{msg.height}")

    def image_msg_to_numpy(self, image):
        if image.encoding not in ('rgb8', 'bgr8', '8UC3'):
            raise ValueError(f"Unsupported image encoding: {image.encoding}")

        channels = 3
        row_stride = image.step
        expected_stride = image.width * channels
        image_data = np.frombuffer(image.data, dtype=np.uint8)
        image_data = image_data.reshape(image.height, row_stride)
        image_data = image_data[:, :expected_stride]
        return np.ascontiguousarray(image_data.reshape(image.height, image.width, channels))

    @torch.no_grad()
    def image_callback(self, image):
        if self.impl_ is None:
            self.get_logger().warn("Waiting for camera info...")
            return
        self.get_logger().info("Received new images!")
        try:
            cv_image = self.image_msg_to_numpy(image)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        image = torch.from_numpy(cv_image).permute(2, 0, 1).cuda()
        intrinsics = torch.from_numpy(self.intr_mat).cuda()
        t = self.get_clock().now().nanoseconds * 1e-9

        with Timer("SLAM", enabled=False):
            self.impl_(t, image, intrinsics)

        self.publish_pose()

    def publish_pose(self):
        pose = self.impl_.poses[0, self.impl_.n-1].cpu().numpy().tolist()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = pose[2]
        pose_msg.pose.orientation.x = pose[3]
        pose_msg.pose.orientation.y = pose[4]
        pose_msg.pose.orientation.z = pose[5]
        pose_msg.pose.orientation.w = pose[6]
        self.pose_pub_.publish(pose_msg)

    def point_cloud_publish_callback(self):
        if self.impl_ is not None and self.impl_.m > 0:
            points = self.impl_.pg.points_.cpu().numpy()[:self.impl_.m]
            colors = self.impl_.pg.colors_.view(-1, 3).cpu().numpy()[:self.impl_.m]
            header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
            pcd = PointCloud2(
                header=header,
                height=1,
                width=points.shape[0],
                is_dense=False,
                is_bigendian=False,
                fields=[
                    PointField(name='x', offset=0, datatype=7, count=1),
                    PointField(name='y', offset=4, datatype=7, count=1),
                    PointField(name='z', offset=8, datatype=7, count=1),
                    PointField(name='r', offset=12, datatype=7, count=1),
                    PointField(name='g', offset=16, datatype=7, count=1),
                    PointField(name='b', offset=20, datatype=7, count=1)],
                point_step=24,
                row_step=points.shape[0] * 24,
                data=np.concatenate((points, colors), axis=1).tobytes())
            self.map_pub_.publish(pcd)


def main(args=None):
    rclpy.init(args=args)
    ros2_node = DpVOInterfaceNode()
    rclpy.spin(ros2_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()