import os
from multiprocessing import Process, Queue
from pathlib import Path

import cv2
import torch
import numpy as np
import rclpy
from cv_bridge import CvBridge
from dpvo_ros.dpvo.config import cfg
from dpvo_ros.dpvo.dpvo import DPVO
# from dpvo_ros.dpvo.plot_utils import plot_trajectory, save_output_for_COLMAP, save_ply
# from dpvo_ros.dpvo.stream import image_stream, video_stream
from dpvo_ros.dpvo.utils import Timer
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header, String


class DpVOInterfaceNode(Node):
    def __init__(self):
        super().__init__('dpvo_node')
        self.declare_parameter('message_queue_size', 100)
        self.declare_parameter('network', rclpy.Parameter.Type.STRING)
        self.declare_parameter('config', rclpy.Parameter.Type.STRING)
        self.declare_parameter('calib', rclpy.Parameter.Type.STRING)

        cfg.merge_from_file(self.get_parameter(
            'config').get_parameter_value().string_value)
        # self.get_logger().info(f"dpvo_ROS node config: {cfg}")

        self.network = self.get_parameter(
            'network').get_parameter_value().string_value

        calib = np.loadtxt(self.get_parameter(
            "calib").get_parameter_value().string_value, delimiter=" ")
        fx, fy, cx, cy = calib[:4]

        self.intr_mat = np.array([fx, fy, cx, cy])

        self.image_sub_ = self.create_subscription(
            Image, "/camera/rgb/image_color", self.image_callback, 10)

        self.pub_pcd = self.create_timer(2, self.point_cloud_publish_callback)
        self.pose_pub_ = self.create_publisher(PoseStamped, "/camera/estimate_pose", 10)
        self.render_pub_ = self.create_publisher(Image, '/camera/render', 10)
        self.map_pub_ = self.create_publisher(PointCloud2, '/map/point_cloud', 10)

        self.img_converter = CvBridge()
        self.impl_ = None
        self.get_logger().info("DPVO_ROS node initialized!")

    @torch.no_grad()
    def image_callback(self, image):
        self.get_logger().info("Received new images!")
        cv_image = self.img_converter.imgmsg_to_cv2(image, "8UC3")
        image = torch.from_numpy(cv_image).permute(2, 0, 1).cuda()
        intrinsics = torch.from_numpy(self.intr_mat).cuda()
        t = self.get_clock().now().nanoseconds * 1e-9

        # _, h, w = image.shape
        # # self.get_logger().info(f"h: {h}, w: {w}")
        # image = image[:, :h-h%16, :w-w%16]

        if self.impl_ is None:
            _, H, W = image.shape
            self.impl_ = DPVO(cfg, self.network, ht=H, wd=W, viz=False)

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


# SKIP = 0

# def show_image(image, t=0):
#     image = image.permute(1, 2, 0).cpu().numpy()
#     cv2.imshow('image', image / 255.0)
#     cv2.waitKey(t)

# @torch.no_grad()
# def run(cfg, network, imagedir, calib, stride=1, skip=0, viz=False, timeit=False):

#     slam = None
#     queue = Queue(maxsize=8)

#     if os.path.isdir(imagedir):
#         reader = Process(target=image_stream, args=(queue, imagedir, calib, stride, skip))
#     else:
#         reader = Process(target=video_stream, args=(queue, imagedir, calib, stride, skip))

#     reader.start()

#     while 1:
#         (t, image, intrinsics) = queue.get()
#         if t < 0: break

#         image = torch.from_numpy(image).permute(2,0,1).cuda()
#         intrinsics = torch.from_numpy(intrinsics).cuda()

#         if slam is None:
#             _, H, W = image.shape
#             slam = DPVO(cfg, network, ht=H, wd=W, viz=viz)

#         with Timer("SLAM", enabled=timeit):
#             slam(t, image, intrinsics)

#     reader.join()

#     points = slam.pg.points_.cpu().numpy()[:slam.m]
#     colors = slam.pg.colors_.view(-1, 3).cpu().numpy()[:slam.m]

#     return slam.terminate(), (points, colors, (*intrinsics, H, W))


# if __name__ == '__main__':
#     import argparse
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--network', type=str, default='dpvo.pth')
#     parser.add_argument('--imagedir', type=str)
#     parser.add_argument('--calib', type=str)
#     parser.add_argument('--name', type=str, help='name your run', default='result')
#     parser.add_argument('--stride', type=int, default=2)
#     parser.add_argument('--skip', type=int, default=0)
#     parser.add_argument('--config', default="config/default.yaml")
#     parser.add_argument('--timeit', action='store_true')
#     parser.add_argument('--viz', action="store_true")
#     parser.add_argument('--plot', action="store_true")
#     parser.add_argument('--opts', nargs='+', default=[])
#     parser.add_argument('--save_ply', action="store_true")
#     parser.add_argument('--save_colmap', action="store_true")
#     parser.add_argument('--save_trajectory', action="store_true")
#     args = parser.parse_args()

#     cfg.merge_from_file(args.config)
#     cfg.merge_from_list(args.opts)

#     print("Running with config...")
#     print(cfg)

#     (poses, tstamps), (points, colors, calib) = run(cfg, args.network, args.imagedir, args.calib, args.stride, args.skip, args.viz, args.timeit)
#     trajectory = PoseTrajectory3D(positions_xyz=poses[:,:3], orientations_quat_wxyz=poses[:, [6, 3, 4, 5]], timestamps=tstamps)

#     if args.save_ply:
#         save_ply(args.name, points, colors)

#     if args.save_colmap:
#         save_output_for_COLMAP(args.name, trajectory, points, colors, *calib)

#     if args.save_trajectory:
#         Path("saved_trajectories").mkdir(exist_ok=True)
#         file_interface.write_tum_trajectory_file(f"saved_trajectories/{args.name}.txt", trajectory)

#     if args.plot:
#         Path("trajectory_plots").mkdir(exist_ok=True)
#         plot_trajectory(trajectory, title=f"DPVO Trajectory Prediction for {args.name}", filename=f"trajectory_plots/{args.name}.pdf")

# def main():
#     print('Hi from DPVO_ROS.')


# if __name__ == '__main__':
#     main()
