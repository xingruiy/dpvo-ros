# ROS_DPVO

deep patch visual odometry for ROS2

## Install

```bash
pip install torch-scatter -f https://data.pyg.org/whl/torch-2.5.0+${CUDA}.html
```

## Build

```bash
colcon build --symlink-install
```

## Usage

```bash
source ./install/setup.bash
ros2 launch dpvo_ros dpvo.launch.py \
  image_topic:=/camera/rgb/image_color \
  camera_info_topic:=/camera/rgb/camera_info
```

In another terminal:

```bash
source ./install/setup.bash
ros2 bag play <bag_file>
```

if you need loop closure

```bash
 ros2 launch dpvo_ros dpvo_realsense.launch.py loop_closure:=true
```

For classic long-term loop closure:

```bash
ros2 launch dpvo_ros dpvo_realsense.launch.py classic_loop_closure:=true
```

## Visualization (RViz2)

The RealSense launch file starts RViz2 automatically with a pre-configured layout (`config/dpvo.rviz`):

```bash
ros2 launch dpvo_ros realsense.launch.py
```

To disable RViz2:

```bash
ros2 launch dpvo_ros realsense.launch.py use_rviz:=false
```

To use a custom RViz config:

```bash
ros2 launch dpvo_ros realsense.launch.py rviz_config:=/path/to/your.rviz
```

The default config displays:

| Topic | Type | Description |
|---|---|---|
| `/camera/estimate_pose` | PoseStamped | Current camera pose (red arrow) |
| `/camera/trajectory` | Path | Full camera trajectory (green) |
| `/map/point_cloud` | PointCloud2 | Colored 3D map points |

## Tips

you can convert bag1 file to bag2 file using `dpvo_ros/scripts/bag_converter.sh`

## Issues

Jetson users:

```bash
pip install https://developer.nvidia.com/w/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
```

also install https://developer.nvidia.com/cusparselt-downloads