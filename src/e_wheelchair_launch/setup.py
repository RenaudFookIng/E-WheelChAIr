from setuptools import setup
import os
from glob import glob

package_name = 'e_wheelchair_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # index ament
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # install launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join(package_name, 'launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renaud Janet',
    maintainer_email='renaud.janet@etu.sorbonne-universite.fr',
    description='Launch files for E-WheelChAIr project',
    license='Apache License 2.0',
    tests_require=['pytest'],
)
