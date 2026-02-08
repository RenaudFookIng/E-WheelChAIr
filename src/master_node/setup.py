from setuptools import setup
import os
from glob import glob

package_name = 'master_node'

# Assure-toi que le marker existe
os.makedirs('resource', exist_ok=True)
marker_file = os.path.join('resource', package_name)
if not os.path.exists(marker_file):
    open(marker_file, 'w').close()

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Marker pour ament_index
        ('share/ament_index/resource_index/packages', [marker_file]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # fichiers de configuration
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'PyYAML',
    ],
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
