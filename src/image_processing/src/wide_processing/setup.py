from setuptools import setup, find_packages
import os

package_name = 'wide_processing'

# Inclure tous les fichiers dans resource/
resource_files = []
for root, dirs, files in os.walk('resource'):
    for f in files:
        resource_files.append(os.path.join(root, f))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + [('share/' + package_name + '/' + os.path.dirname(f), [f]) for f in resource_files],
    install_requires=['setuptools', 'torch', 'onnxruntime', 'opencv-python'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Traitement d\'images large FOV avec YOLO et Depth Anything',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'wide_processing_node = wide_processing.wide_processing_node:main',
        ],
    },
)
