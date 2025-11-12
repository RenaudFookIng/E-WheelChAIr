from setuptools import setup, find_packages
import os

package_name = 'ultrasonic'

data_files = [
    ('share/' + package_name, ['package.xml']),
]



setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Driver ROS2 pour le capteur ultrason',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'ultrasonic_node = ultrasonic.ultrasonic.ultrasonic_node:main',
        ],
    },
)
