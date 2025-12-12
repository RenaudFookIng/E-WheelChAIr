from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Fichier de lancement Python-only pour E-WheelChAIr
    Compatible avec Raspberry Pi 3 (pas de dépendances C++)
    """
    
    # Arguments de lancement configurables
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS0',
        description='Port série pour le contrôleur Sabertooth'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Vitesse de communication série'
    )
    
    # Configuration des nœuds
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    
    return LaunchDescription([
        # Arguments configurables
        serial_port_arg,
        baud_rate_arg,
        
        # Nœud principal - Master Node
        Node(
            package='master_node',
            executable='master_node',
            name='master_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        
        # Calculateur de vitesse moteur
        Node(
            package='motor_speed_calculator',
            executable='motor_speed_calculator_node',
            name='motor_speed_calculator',
            output='screen'
        ),
        
        # Nouveau contrôleur Sabertooth en Python (remplace la version C++)
        Node(
            package='sabertooth_controller_py',
            executable='sabertooth_controller_node',
            name='sabertooth_controller',
            output='screen',
            parameters=[
                {'serial_port': serial_port},
                {'baud_rate': baud_rate}
            ]
        ),
        
        # Visualisation (si disponible)
        Node(
            package='visualization',
            executable='visualization_node',
            name='visualization',
            output='screen'
        ),
    ])