from setuptools import setup

package_name = 'depth_camera_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Driver pour la depth camera',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'depth_camera_node = depth_camera_driver.depth_camera_node:main',
        ],
    },
)