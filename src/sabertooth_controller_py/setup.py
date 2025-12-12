from setuptools import setup

package_name = 'sabertooth_controller_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Renaud Janet',
    maintainer_email='renaud.janet@proton.me',
    description='Python ROS2 node for controlling Sabertooth motor driver',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'sabertooth_controller_node = sabertooth_controller_py.sabertooth_controller:main',
        ],
    },
)