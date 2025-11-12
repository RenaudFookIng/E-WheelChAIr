from setuptools import setup

package_name = 'lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource/', ['resource/config.yaml']),
        ('share/' + package_name + '/launch/', ['launch/lidar.launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Package ROS2 pour le LIDAR du Rosmaster X3',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'lidar_node = lidar.lidar_node:main',
        ],
    },
)
