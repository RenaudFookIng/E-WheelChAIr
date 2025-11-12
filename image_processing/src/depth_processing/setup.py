from setuptools import setup

package_name = 'depth_processing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy', 'std_msgs', 'numpy'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Traitement des images de la depth camera',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'depth_processing_node = depth_processing.depth_processing_node:main',
        ],
    },
)