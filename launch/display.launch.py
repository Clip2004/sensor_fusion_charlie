from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('sensor_fusion_charlie').find('sensor_fusion_charlie')

    robot_desc_path = PathJoinSubstitution([
        pkg_share,
        'description',
        'robot.urdf.xacro'   # Cambia el nombre si tu archivo .xacro se llama distinto
    ])

    # ✅ Generar correctamente la descripción del robot
    robot_description = ParameterValue(
        Command(['xacro ', robot_desc_path]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        )
    ])
