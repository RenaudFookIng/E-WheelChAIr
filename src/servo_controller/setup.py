from setuptools import setup
import os

package_name = 'servo_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), 
            ['config/servo_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@etu.sorbonne-universite.fr',
    description='Servo Controller Node for E-WheelChAIr',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_controller_node = servo_controller.servo_controller_node:main'
        ],
    },
)