from setuptools import setup
import os

package_name = 'master_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/master_config.yaml', 'config/servo_config.yaml']),
    ],
    install_requires=['setuptools', 'pyserial', 'PyYAML'],
    zip_safe=True,
    maintainer='Renaud Janet',
    maintainer_email='renaud.janet@etu.sorbonne-universite.fr',
    description='Central ROS2 node for E-WheelChAIr to merge all sensor data and control Sabertooth',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'master_node = master_node.master_node:main',
        ],
    },
)
