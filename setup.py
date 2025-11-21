from glob import glob
import os
from setuptools import setup, find_packages

package_name = 'sensor_fusion_charlie'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        ('share/' + package_name + '/launch', ['launch/sensor_fusion.launch.py','launch/display.launch.py','launch/mapping.launch.py','launch/localization.launch.py']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Sensor fusion package for Charlie robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yaw_estimation_node_charlie = sensor_fusion_charlie.yaw_estimation_node_charlie:main',
            'bicycle_model_node_charlie = sensor_fusion_charlie.bicycle_model_node_charlie:main',
            'velocity_model_node_charlie = sensor_fusion_charlie.velocity_model_node_charlie:main',
            'madgwick_filter_node_charlie = sensor_fusion_charlie.madgwick_filter_node_charlie:main',
            'pose_broadcaster_node_charlie = sensor_fusion_charlie.pose_broadcaster_node_charlie:main',
            'ackermann_kinematics_node_charlie = sensor_fusion_charlie.ackermann_kinematics_node_charlie:main',
            'linear_velocity_node_charlie = sensor_fusion_charlie.linear_velocity_node_charlie:main',
            'tf_broadcaster_node_charlie = sensor_fusion_charlie.tf_broadcaster_node_charlie:main',
            
        ],
    },
)
