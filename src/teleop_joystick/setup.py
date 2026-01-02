from setuptools import setup
import os

package_name = 'teleop_joystick'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), 
            ['config/joystick_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@etu.sorbonne-universite.fr',
    description='Teleop Joystick Node for E-WheelChAIr',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_joystick_node = teleop_joystick.teleop_joystick_node:main'
        ],
    },
)