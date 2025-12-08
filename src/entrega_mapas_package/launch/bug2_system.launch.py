
#!/usr/bin/env python3
"""
Launch file para el sistema completo Bug2
Lanza todos los nodos necesarios
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Genera la descripción del launch
    
    Lanza 3 nodos:
    1. coppelia_interface_node - Interfaz con CoppeliaSim
    2. bug2_controller_node - Controlador Bug2
    3. goal_manager_node - Gestor de metas
    """
    
    # Argumentos del launch (pueden pasarse al ejecutar)
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='Pioneer_p3dx',
        description='Nombre del robot en CoppeliaSim'
    )
    
    declare_control_rate = DeclareLaunchArgument(
        'control_rate',
        default_value='20.0',
        description='Frecuencia de control en Hz'
    )
    
    declare_auto_goals = DeclareLaunchArgument(
        'auto_generate_goals',
        default_value='true',
        description='Generar metas automáticamente'
    )
    
    # Configuraciones (valores de los argumentos)
    robot_name = LaunchConfiguration('robot_name')
    control_rate = LaunchConfiguration('control_rate')
    auto_generate_goals = LaunchConfiguration('auto_generate_goals')
    
    # Nodo 1: Interfaz con CoppeliaSim
    coppelia_interface = Node(
        package='entrega_mapas_package',  # Cambia por el nombre de tu paquete
        executable='coppelia_interface_node',
        name='coppelia_interface',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'update_rate': control_rate,
            'max_speed': 2.0
        }],
        emulate_tty=True  # Para ver logs en color
    )
    
    # Nodo 2: Controlador Bug2
    bug2_controller = Node(
        package='entrega_mapas_package',
        executable='bug2_controller_node',
        name='bug2_controller',
        output='screen',
        parameters=[{
            'control_rate': control_rate,
            'max_linear_speed': 2.0,
            'max_angular_speed': 1.5,
            'goal_tolerance': 0.3,
            'm_line_tolerance': 0.2,
            'obstacle_threshold': 0.5,
            'front_obstacle_threshold': 0.55,
            'wall_distance': 0.6
        }],
        emulate_tty=True
    )
    
    # Nodo 3: Gestor de metas
    goal_manager = Node(
        package='entrega_mapas_package',
        executable='goal_manager_node',
        name='goal_manager',
        output='screen',
        parameters=[{
            'goal_tolerance': 0.8,
            'min_goal_distance': 2.0,
            'map_min_x': -4.0,
            'map_max_x': 4.0,
            'map_min_y': -4.0,
            'map_max_y': 4.0,
            'map_buffer': 1.5,
            'auto_generate': auto_generate_goals
        }],
        emulate_tty=True
    )
    
    # Nodo 4: Occupancy Mapper
    occupancy_mapper = Node(
        package='entrega_mapas_package',
        executable='occupancy_mapper_node',
        name='occupancy_mapper',
        output='screen',
        parameters=[{
            'map_width': 10.0,
            'map_height': 10.0,
            'resolution': 0.05,
            'origin_x': -5.0,
            'origin_y': -5.0,
            'save_interval': 30.0,
            'output_dir': '/ros2_ws/maps'
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        # Declarar argumentos
        declare_robot_name,
        declare_control_rate,
        declare_auto_goals,
        
        # Lanzar nodos
        coppelia_interface,
        bug2_controller,
        goal_manager,
        occupancy_mapper
    ])