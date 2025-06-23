# Synchronized RGB-D + IMU from OAK for RTAB-Map

This repository provides a synchronized stereo vision pipeline using the **Luxonis OAK ** camera. It generates **disparity-based RGB-D images** using
a custom stereo matching network along with **IMU data**, making it fully compatible with **RTAB-Map** for real-time 3D SLAM and map reconstruction.

---

##  Features

-  **Synchronized** stereo RGB and IMU data streams
-  Real-time **disparity estimation** with TensorRT acceleration
-  Generates **depth images** from stereo pairs using a deep learning model
-  Publishes calibrated and synchronized **IMU** data
-  ROS 2 compatible node
-  Fully compatible with **RTAB-Map** for 3D mapping

---

##  Applications

- Visual-Inertial SLAM
- Real-time 3D Reconstruction
- Robot Perception & Navigation
- Autonomous Mapping Systems

---

##  Requirements

- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) 
- [DepthAI SDK](https://docs.luxonis.com/software/) 
- [TensorRT](https://developer.nvidia.com/tensorrt) + CUDA 
- [RTAB-Map](https://github.com/introlab/rtabmap)

---

##  Quick Start

```bash
mkdir -p depthai_rgbd_oak/src
cd depthai_rgbd_oak/src
git clone <this-repository-url>
cd ..
colcon build
source install/setup.bash
```

### Stereo Matching Network
Use ```onnx_transformed.py``` to transfer the stereo matching model to .onnx and 
use the below command to generate .plan file:

```bash
/usr/local/TensorRT-10.11.0.33/bin/trtexec --onnx=StereoModel.onnx --noTF32 --saveEngine=StereoModel.plan
```

and 

```bash
cp StereoModel.plan /tmp
```
### Terminal 1: Oak camera publilsher

```bash
ros2 launch depthai_oakdpro depthai_oakdpro_cuda_node.launch.py
```

### Terminal 2: RTAB-map 

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  args:="--delete_db_on_start" \
  rgb_topic:=/left/image_rect \
  depth_topic:=/disparity/image_raw \
  camera_info_topic:=/left/camera_info \
  imu_topic:=/oak/imu \
  frame_id:=oak_stereo_frame \
  approx_sync:=true \
  approx_sync_max_interval:=0.001 \
  wait_imu_to_init:=true
```
### Settings

The following parameters can be adjusted in the launch file

```bash
depthai_node = Node(
    package='depthai_oakdpro',
    executable='depthai_oakdpro_cuda_node',
    name='depthai_oakdpro_cuda_node',
    output='screen',
    parameters=[{
        'fx': 379.0, # focal legnth
        'baseline': 0.15, # stereo baseline
        'width': 640, # camera width
        'height': 400, # camera height
        'net_input_width': 640, # network input width
        'net_input_height': 384, # network input height
        'Imux': 0.0, # Imu x offset from the left camera
        'Imuy': -0.02, # Imu y offset from the left camera
        'Imuz': 0.0 # Imu z offset from the left camera
    }]
)
