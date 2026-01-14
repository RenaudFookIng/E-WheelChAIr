from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wyes_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Package pour envoyer les intentions de direction depuis les lunettes Wyes',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wyes_teleop = wyes_teleop.wyes_teleop_node:main',
        ],
    },
)
