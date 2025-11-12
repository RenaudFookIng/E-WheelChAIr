from setuptools import setup

package_name = 'visualization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'matplotlib', 'numpy'],
    zip_safe=True,
    maintainer='Renaud JANET',
    maintainer_email='renaud.janet@proton.me',
    description='Real-time plotting for E-WheelChAIr',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'real_time_plot = visualization.real_time_plot_node:main',
        ],
    },
)
