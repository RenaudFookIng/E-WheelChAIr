from setuptools import setup

package_name = 'master_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='renaud janet',
    maintainer_email='renaud.janet@etu.sorbonne-universite.fr',
    description='Central ROS2 node for E-WheelChAIr to merge all data and control Sabertooth',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'master_node = master_node.master_node:main',
        ],
    },
)
