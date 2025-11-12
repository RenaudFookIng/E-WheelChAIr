from setuptools import setup

package_name = 'motor_speed_calculator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Calculates motor speed from voltage for ROS2',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'motor_speed_node = motor_speed_calculator.motor_speed_calculator:main',
        ],
    },
)
