import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
def generate_launch_description():

    package_name = 'sensor_fusion_charlie'

    # Nodes
    ackermann_kinematics_node_charlie = Node(
        package=package_name,
        executable='ackermann_kinematics_node_charlie',
        name='ackermann_kinematics_node_charlie',
    )

    linear_velocity_node_charlie = Node(
        package=package_name,
        executable='linear_velocity_node_charlie',
        name='linear_velocity_node_charlie',
    )

    tf_broadcaster_node_charlie = Node(
        package=package_name,
        executable='tf_broadcaster_node_charlie',
        name='tf_broadcaster_node_charlie',
    )

    # RViz2 config path
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'rviz_sim_config.rviz'
    )

    rviz2_node_mapping = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    slam_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_charlie'),
                'launch',
                'online_async_launch.py'
            )
        )
    )


    # RETURN inside the function (important!)
    return LaunchDescription([
        ackermann_kinematics_node_charlie,
        linear_velocity_node_charlie,
        tf_broadcaster_node_charlie,
        rviz2_node_mapping,
        slam_mapping_launch,
    ])
