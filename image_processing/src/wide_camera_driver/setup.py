from setuptools import setup

package_name = 'wide_camera_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Driver pour les caméras Fit0892 grand angle',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'wide_camera_node = wide_camera_driver.wide_camera_node:main',
        ],
    },
)