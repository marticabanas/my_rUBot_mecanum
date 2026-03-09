import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Arguments de Llançament ---
    
    # Arguments per especificar NOMÉS el nom dels fitxers
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='map_square3m_walls.yaml',
        description="Name of the map file in the 'map' directory of the package")

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='rubot_sw.yaml',
        description="Name of the param file in the 'param' directory of the package")

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # --- Obtenim els valors dels arguments ---
    
    # Capturem els noms dels fitxers
    map_filename = LaunchConfiguration('map_file')
    params_filename = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Construïm les rutes completes usant substitucions de ROS 2
    map_path = PathJoinSubstitution([
        FindPackageShare('my_robot_navigation2'),
        'map',
        map_filename
    ])
    
    params_path = PathJoinSubstitution([
        FindPackageShare('my_robot_navigation2'),
        'param',
        params_filename
    ])

    # --- Configuració dels Nodes ---

    # Node per llançar RViz2 (la ruta del fitxer .rviz és estàtica)
    rviz_config_path = os.path.join(
        get_package_share_directory('my_robot_navigation2'),
        'rviz',
        'my_robot_navigation2.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Inclusió del fitxer de llançament principal de Nav2
    nav2_bringup_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': map_path, # Passem la ruta completa que hem construït
            'use_sim_time': use_sim_time,
            'params_file': params_path, # Passem la ruta completa que hem construït
            'autostart': 'true'
        }.items(),
    )

    # --- Creació de la Descripció de Llançament ---
    
    return LaunchDescription([
        # Declaració de tots els arguments
        declare_map_file_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,

        # Llançament de Nav2
        nav2_bringup_launch,

        # Llançament de RViz2
        rviz_node,
    ])