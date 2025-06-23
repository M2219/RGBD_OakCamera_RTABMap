import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_desc'), 'launch')

    # Launch Configuration Parameters
    base_frame   = LaunchConfiguration('base_frame',    default='oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default='oak-d-base-frame')

    cam_pos_x    = LaunchConfiguration('cam_pos_x',     default='0.0')
    cam_pos_y    = LaunchConfiguration('cam_pos_y',     default='0.0')
    cam_pos_z    = LaunchConfiguration('cam_pos_z',     default='0.0')
    cam_roll     = LaunchConfiguration('cam_roll',      default='0.0')
    cam_pitch    = LaunchConfiguration('cam_pitch',     default='0.0')
    cam_yaw      = LaunchConfiguration('cam_yaw',       default='0.0')

    # Declare the arguments
    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link in the TF Tree.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from another robot TF that can be connected to the base of the OAK device.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    # Include the URDF launch file
    urdf_oak_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(urdf_launch_dir, 'urdf_oak_launch.py')),
        launch_arguments={
            'base_frame'  : base_frame,
            'parent_frame': parent_frame,
            'cam_pos_x'   : cam_pos_x,
            'cam_pos_y'   : cam_pos_y,
            'cam_pos_z'   : cam_pos_z,
            'cam_roll'    : cam_roll,
            'cam_pitch'   : cam_pitch,
            'cam_yaw'     : cam_yaw
        }.items())

    # Define the main node (depthai_oakdpro_node)
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

    return LaunchDescription([
        declare_base_frame_cmd,
        declare_parent_frame_cmd,
        declare_pos_x_cmd,
        declare_pos_y_cmd,
        declare_pos_z_cmd,
        declare_roll_cmd,
        declare_pitch_cmd,
        declare_yaw_cmd,
        urdf_oak_launch,
        depthai_node,
    ])
