# ROS_DPVO

deep patch visual odometry for ROS2
## Build

```bash
colcon build --symlink-install
```

## Usage

```bash
source ./install/setup.bash
ros2 launch dpvo_ros dpvo_realsense.launch.py \
  image_topic:=/camera/rgb/image_color \
  camera_info_topic:=/camera/rgb/camera_info
```

In another terminal:

```bash
source ./install/setup.bash
ros2 bag play <bag_file>
```

## Tips

you can convert bag1 file to bag2 file using `dpvo_ros/scripts/bag_converter.sh`