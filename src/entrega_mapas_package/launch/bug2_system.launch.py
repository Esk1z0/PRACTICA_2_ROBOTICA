#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='Pioneer_p3dx'
    )

    declare_control_rate = DeclareLaunchArgument(
        'control_rate',
        default_value='20.0'
    )

    declare_auto_goals = DeclareLaunchArgument(
        'auto_generate_goals',
        default_value='true'
    )

    declare_laser_x = DeclareLaunchArgument('laser_x', default_value='0.20')
    declare_laser_y = DeclareLaunchArgument('laser_y', default_value='0.00')
    declare_laser_z = DeclareLaunchArgument('laser_z', default_value='0.20')
    declare_laser_yaw = DeclareLaunchArgument('laser_yaw', default_value='0.0')

    robot_name = LaunchConfiguration('robot_name')
    control_rate = LaunchConfiguration('control_rate')
    auto_generate_goals = LaunchConfiguration('auto_generate_goals')

    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')

    coppelia_interface = Node(
        package='entrega_mapas_package',
        executable='coppelia_interface_node',
        name='coppelia_interface',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'update_rate': control_rate,
            'max_speed': 2.0,
            'scan_frame': 'laser'
        }],
        emulate_tty=True
    )

    bug2_controller = Node(
        package='entrega_mapas_package',
        executable='bug2_controller_node',
        name='bug2_controller',
        output='screen',
        parameters=[{
            'control_rate': control_rate,
            'max_linear_speed': 2.0,
            'max_angular_speed': 1.0,
            'goal_tolerance': 0.3,
            'm_line_tolerance': 0.2,
            'obstacle_threshold': 0.2,
            'front_obstacle_threshold': 0.2,
            'wall_distance': 0.2
        }],
        emulate_tty=True
    )

    goal_manager = Node(
        package='entrega_mapas_package',
        executable='goal_manager_node',
        name='goal_manager',
        output='screen',
        parameters=[{
            'goal_tolerance': 0.8,
            'min_goal_distance': 2.0,
            'map_min_x': -2.45,
            'map_max_x': 2.45,
            'map_min_y': -2.45,
            'map_max_y': 2.45,
            'map_buffer': 0.2,
            'auto_generate': auto_generate_goals,
            'goal_frame': 'world',
            'check_rate_hz': 10.0,
            'max_attempts': 200
        }],
        emulate_tty=True
    )

    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        output='screen',
        arguments=[
            laser_x, laser_y, laser_z,
            '0.0', '0.0', laser_yaw,
            'base_link', 'laser'
        ]
    )

    # Si ya tienes un nodo que publique /map (OccupancyGrid), esto te lo guarda a disco:
    # map_saver = Node(
    #     package='nav2_map_server',
    #     executable='map_saver_cli',
    #     name='map_saver',
    #     output='screen',
    #     arguments=['-f', '/ros2_ws/maps/map']
    # )

    return LaunchDescription([
        declare_robot_name,
        declare_control_rate,
        declare_auto_goals,
        declare_laser_x,
        declare_laser_y,
        declare_laser_z,
        declare_laser_yaw,

        coppelia_interface,
        laser_static_tf,
        bug2_controller,
        goal_manager,
        # map_saver,
    ])
