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

## Tips

you can convert bag1 file to bag2 file using `dpvo_ros/scripts/bag_converter.sh`

## Issues

Jetson users:

```bash
pip install https://developer.nvidia.com/w/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
```

also install https://developer.nvidia.com/cusparselt-downloads