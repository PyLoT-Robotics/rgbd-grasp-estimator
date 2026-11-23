from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default=Path(get_package_share_directory('rgbd_grasp'))/'config'/'params.yaml'
    return LaunchDescription([DeclareLaunchArgument('params_file',default_value=str(default)),
        Node(package='rgbd_grasp',executable='rgbd_grasp_node',name='rgbd_grasp',
             parameters=[LaunchConfiguration('params_file')],output='screen')])
