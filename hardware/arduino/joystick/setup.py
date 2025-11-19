from setuptools import setup, find_packages
import os

package_name = 'joystick'

# Préparer la liste des fichiers à inclure
data_files = [
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
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
