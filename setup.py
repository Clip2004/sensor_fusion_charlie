from setuptools import find_packages, setup

package_name = 'sensor_fusion_charlie'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clip2004',
    maintainer_email='felipe.mercado59@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yaw_estimation_node = sensor_fusion_charlie.yaw_estimation_node_charlie:main',
            'velocity_model_node_charlie = sensor_fusion_charlie.velocity_model_node_charlie:main',
            'bicycle_model_node_charlie = sensor_fusion_charlie.bicycle_model_node_charlie:main',
        ],
    },
)
