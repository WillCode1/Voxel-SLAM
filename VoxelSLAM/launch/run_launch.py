import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('voxel_slam'), 'config', 'ros2_param.yaml')
    rviz_config = os.path.join(get_package_share_directory('voxel_slam'), 'rviz_cfg', 'mapping_ros2.rviz')

    voxel_slam = Node(package="voxel_slam", executable="voxelslam", prefix=['stdbuf -o L'], output='screen', parameters=[config])
    # voxel_slam = Node(package="voxel_slam", executable="voxelslam", prefix=['gdb -ex run --args'], output='screen', parameters=[config])
    rviz2 = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config])

    return LaunchDescription([voxel_slam, rviz2])
