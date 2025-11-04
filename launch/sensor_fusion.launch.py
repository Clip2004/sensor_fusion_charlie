# library to move between files and folders in the O.S.
import os
from ament_index_python.packages import get_package_share_directory
# libraries to define the Launch file and Function
from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='sensor_fusion_charlie' #<--- CHANGE ME

    # Define Launch configuration variables specific to simulation
    bicycle_model_node_charlie = Node(package=package_name,
                             executable='bicycle_model_node_charlie',
                             name='bicycle_model_node_charlie',
    )
    velocity_model_node_charlie = Node(package=package_name,
                          executable='velocity_model_node_charlie',
                          name='velocity_model_node_charlie',
    )
    yaw_estimation_node_charlie = Node(package=package_name,
                          executable='yaw_estimation_node_charlie',
                          name='yaw_gyro_filtered_node',
    )

# Launch them all!
    return LaunchDescription([
        bicycle_model_node_charlie,
        velocity_model_node_charlie,
        yaw_estimation_node_charlie,
    ])