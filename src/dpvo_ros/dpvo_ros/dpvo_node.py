import torch
import numpy as np
import rclpy
from lietorch import SE3
from dpvo_ros.dpvo.config import cfg
# from dpvo_ros.dpvo.plot_utils import plot_trajectory, save_output_for_COLMAP, save_ply
# from dpvo_ros.dpvo.stream import image_stream, video_stream
from dpvo_ros.dpvo.utils import Timer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
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
        self.declare_parameter('loop_closure', False)
        self.declare_parameter('classic_loop_closure', False)

        cfg.merge_from_file(self.get_parameter(
            'config').get_parameter_value().string_value)
        cfg.LOOP_CLOSURE = self.get_parameter(
            'loop_closure').get_parameter_value().bool_value
        cfg.CLASSIC_LOOP_CLOSURE = self.get_parameter(
            'classic_loop_closure').get_parameter_value().bool_value
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
        self.trajectory_pub_ = self.create_publisher(Path, '/camera/trajectory', 10)

        self.impl_ = None
        self.last_image_np_ = None
        self.last_image_enc_ = None
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
        try:
            cv_image = self.image_msg_to_numpy(image)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        self.last_image_np_ = cv_image
        self.last_image_enc_ = image.encoding

        image = torch.from_numpy(cv_image).permute(2, 0, 1).cuda()
        intrinsics = torch.from_numpy(self.intr_mat).cuda()
        t = self.get_clock().now().nanoseconds * 1e-9

        with Timer("SLAM", enabled=False):
            self.impl_(t, image, intrinsics)

        self.publish_pose()
        self.publish_render()
        self.publish_trajectory()

    def publish_pose(self):
        # pg.poses_ stores world-to-camera (w2c); invert to get camera-in-world (c2w)
        c2w = SE3(self.impl_.poses[0, self.impl_.n - 1]).inv()
        pose = c2w.data.cpu().numpy()  # [tx, ty, tz, qx, qy, qz, qw] in DPVO world frame

        # Convert from DPVO world frame (X=right, Y=down, Z=forward)
        # to ROS REP-103 (X=forward, Y=left, Z=up):
        #   ros_x = dpvo_z,  ros_y = -dpvo_x,  ros_z = -dpvo_y
        tx, ty, tz = pose[0], pose[1], pose[2]
        ros_tx, ros_ty, ros_tz = tz, -tx, -ty

        # Rotate orientation: q_ros = q_d2r ⊗ q_c2w
        # q_d2r = (x=-0.5, y=0.5, z=-0.5, w=0.5) encodes the DPVO→ROS frame rotation
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        dx, dy, dz, dw = -0.5, 0.5, -0.5, 0.5
        ros_qw = dw*qw - dx*qx - dy*qy - dz*qz
        ros_qx = dw*qx + dx*qw + dy*qz - dz*qy
        ros_qy = dw*qy - dx*qz + dy*qw + dz*qx
        ros_qz = dw*qz + dx*qy - dy*qx + dz*qw

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(ros_tx)
        pose_msg.pose.position.y = float(ros_ty)
        pose_msg.pose.position.z = float(ros_tz)
        pose_msg.pose.orientation.x = float(ros_qx)
        pose_msg.pose.orientation.y = float(ros_qy)
        pose_msg.pose.orientation.z = float(ros_qz)
        pose_msg.pose.orientation.w = float(ros_qw)
        self.pose_pub_.publish(pose_msg)

    def publish_render(self):
        if self.last_image_np_ is None:
            return
        img = self.last_image_np_
        enc = self.last_image_enc_ if self.last_image_enc_ != '8UC3' else 'rgb8'
        h, w = img.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height = h
        msg.width = w
        msg.encoding = enc
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = img.tobytes()
        self.render_pub_.publish(msg)

    def publish_trajectory(self):
        if self.impl_ is None or self.impl_.n == 0:
            return
        n = self.impl_.n
        poses_c2w = SE3(self.impl_.pg.poses_[:n]).inv().data.cpu().numpy()

        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "world"

        for i in range(n):
            pose = poses_c2w[i]
            tx, ty, tz = pose[0], pose[1], pose[2]
            ros_tx, ros_ty, ros_tz = tz, -tx, -ty
            qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
            dx, dy, dz, dw = -0.5, 0.5, -0.5, 0.5
            ros_qw = dw*qw - dx*qx - dy*qy - dz*qz
            ros_qx = dw*qx + dx*qw + dy*qz - dz*qy
            ros_qy = dw*qy - dx*qz + dy*qw + dz*qx
            ros_qz = dw*qz + dx*qy - dy*qx + dz*qw
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = "world"
            ps.pose.position.x = float(ros_tx)
            ps.pose.position.y = float(ros_ty)
            ps.pose.position.z = float(ros_tz)
            ps.pose.orientation.x = float(ros_qx)
            ps.pose.orientation.y = float(ros_qy)
            ps.pose.orientation.z = float(ros_qz)
            ps.pose.orientation.w = float(ros_qw)
            path_msg.poses.append(ps)

        self.trajectory_pub_.publish(path_msg)

    def point_cloud_publish_callback(self):
        if self.impl_ is None or self.impl_.m == 0:
            return

        m = self.impl_.m
        pts = self.impl_.pg.points_[:m].cpu().numpy()
        colors = self.impl_.pg.colors_.reshape(-1, 3)[:m].cpu().numpy()

        # Convert DPVO frame (X=right, Y=down, Z=forward) → ROS REP-103 (X=forward, Y=left, Z=up)
        ros_pts = np.stack([pts[:, 2], -pts[:, 0], -pts[:, 1]], axis=1).astype(np.float32)

        # Filter out zero-initialized / degenerate points
        valid = np.linalg.norm(ros_pts, axis=1) > 1e-6
        ros_pts = ros_pts[valid]
        colors = colors[valid]
        if ros_pts.shape[0] == 0:
            return

        # Pack RGB into single float32 field (standard RViz packed encoding: 0x00RRGGBB)
        r = colors[:, 0].astype(np.uint32)
        g = colors[:, 1].astype(np.uint32)
        b = colors[:, 2].astype(np.uint32)
        rgb = ((r << 16) | (g << 8) | b).view(np.float32).reshape(-1, 1)

        data = np.concatenate([ros_pts, rgb], axis=1)  # (n_valid, 4) float32

        n_pts = ros_pts.shape[0]
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        pcd = PointCloud2(
            header=header,
            height=1,
            width=n_pts,
            is_dense=False,
            is_bigendian=False,
            fields=[
                PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            point_step=16,
            row_step=n_pts * 16,
            data=data.tobytes())
        self.map_pub_.publish(pcd)


def main(args=None):
    rclpy.init(args=args)
    ros2_node = DpVOInterfaceNode()
    rclpy.spin(ros2_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
