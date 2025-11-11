from setuptools import setup

package_name = 'wide_processing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource/', ['resource/.gitkeep']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Traitement des images des caméras grand angle (YOLO + Depth Anything)',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'wide_processing_node = wide_processing.wide_processing_node:main',
        ],
    },
)