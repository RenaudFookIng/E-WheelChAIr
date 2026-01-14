from setuptools import setup

package_name = 'wide_processing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renaud Janet',
    maintainer_email='renaud.janet@proton.me',
    description='Wide camera processing node for E-WheelChAIr',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wide_processing_node = wide_processing.wide_processing_node:main',
        ],
    },
)
