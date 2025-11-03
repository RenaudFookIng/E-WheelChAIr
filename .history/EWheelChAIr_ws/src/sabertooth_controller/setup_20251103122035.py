from setuptools import find_packages, setup

package_name = 'sabertooth_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='renaud',
    maintainer_email='renaud@todo.todo',
    description='Package for controlling the Sabertooth motor driver',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sabertooth_controller = sabertooth_controller.sabertooth_node:main',
        ],


        package_name = 'sabertooth_controller'

        setup(
            name=package_name,
            version='0.0.0',
            packages=[package_name],
            data_files=[
                ('share/ament_index/resource_index/packages',
                    ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
            ],
            install_requires=['setuptools'],
            zip_safe=True,
            maintainer='votre_nom',
            maintainer_email='votre@email.com',
            description='Package pour contrôler un Sabertooth et visualiser les données en temps réel',
            license='Apache License 2.0',
            tests_require=['pytest'],
            entry_points={
                'console_scripts': [
                    'motor_speed_calculator = sabertooth_controller.motor_speed_calculator:main',
                    'real_time_plot = sabertooth_controller.real_time_plot:main',
                    # Ajoutez ici d'autres nœuds si nécessaire
                ],
            },
        )
    },
)
