# erp42_racing_serial_bridge.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # ERP42 Racing serial bridge parameter file
    erp42_racing_serial_bridge_parameter_file = DeclareLaunchArgument('erp42_racing_serial_bridge_parameter_file', 
        default_value=PathJoinSubstitution([
            FindPackageShare('erp42_racing_serial'), 'config', 'erp42_racing_serial_bridge.param.yaml'
        ])
    )

    # ERP42 Racing serial bridge
    erp42_racing_serial_bridge = Node(
        package    = 'erp42_racing_serial', 
        executable = 'serial_bridge', 
        name       = 'erp42_racing_serial_bridge', 
        output     = 'screen',
        parameters = [{LaunchConfiguration('erp42_racing_serial_bridge_parameter_file')}]
    )

    return LaunchDescription([
        erp42_racing_serial_bridge_parameter_file,
        erp42_racing_serial_bridge
    ])