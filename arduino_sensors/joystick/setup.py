from setuptools import setup, find_packages

package_name = 'joystick'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource/', ['resource/config.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Driver ROS2 pour le joystick proportionnel',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node = joystick.joystick.joystick_node:main',
        ],
    },
)
