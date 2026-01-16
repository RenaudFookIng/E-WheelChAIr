from setuptools import setup

package_name = 'arduino_data_receiver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial'
    ],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Central Arduino USB data receiver for E-WheelChAIr',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_data_receiver_node = arduino_data_receiver.arduino_data_receiver_node:main',
        ],
    },
)
