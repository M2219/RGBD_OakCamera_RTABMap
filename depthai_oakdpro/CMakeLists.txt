cmake_minimum_required(VERSION 3.10)
project(depthai_oakdpro LANGUAGES CXX)

# Use C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# oak pro gpu
find_package(CUDA REQUIRED)

# TensorRT paths
set(TENSORRT_INCLUDE_DIR /usr/local/TensorRT-10.11.0.33/include)
set(TENSORRT_LIBRARY_DIR /usr/local/TensorRT-10.11.0.33/lib)

# Link directories
link_directories(
  ${TENSORRT_LIBRARY_DIR}
)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  /usr/include/x86_64-linux-gnu

)


# Define executable
add_executable(depthai_oakdpro_cuda_node src/depthai_oakdpro_cuda_node.cpp)

target_link_libraries(depthai_oakdpro_cuda_node
  depthai::core
  depthai_bridge::depthai_bridge  # Ensure depthai_bridge is linked
  depthai::opencv
  nvinfer
  nvonnxparser
  nvinfer_plugin
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  /usr/lib/x86_64-linux-gnu

)

ament_target_dependencies(depthai_oakdpro_cuda_node
  rclcpp
  sensor_msgs
  depthai_ros_msgs
  depthai_bridge
  OpenCV
)



# Install the node
install(TARGETS depthai_oakdpro_cuda_node  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)
# Ament package
ament_package()
