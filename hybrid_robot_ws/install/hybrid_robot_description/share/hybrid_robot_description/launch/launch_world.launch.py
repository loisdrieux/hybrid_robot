import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. CONFIGURATION DE BASE ---
    pkg_desc = get_package_share_directory('hybrid_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Paramètres globaux
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_path = os.path.join(pkg_desc, 'worlds', 'storage_world.world')
    map_yaml_path = os.path.join(pkg_desc, 'map', 'storage_map.yaml') # Chemin du fichier YAML
    
    # Charger le XACRO
    robot_description = {'robot_description': Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', PathJoinSubstitution([pkg_desc, 'urdf', 'hybrid_terrestrial.xacro'])
    ])}
    
    # --- 2. LANCEMENT DES COMPOSANTS BASIQUES ---
    
    # a) Lance Gazebo Server et Client
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'gzclient': 'true', 'use_sim_time': use_sim_time}.items()
    )

    # b) Lance Robot State Publisher (TF odom -> base_link)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # c) Spawn du robot dans Gazebo (avec Z=0.01 pour la friction)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'hybrid_robot_terrestrial',
                                   '-x', '0.5', 
                                   '-y', '5.0',
                                   '-z', '0.01', 
                                   '--ros-args', '--param', 'use_sim_time:=true'],
                        output='screen')

    # --- 3. LANCEMENT ET CONFIGURATION DE LA CARTE (MAP) ---

    # d) Lance le Map Server
    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_path}]
    )

    # e) Publie les TF statiques (World -> Map et Map -> Odom)
    # TF World -> Map
    static_tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map', '--ros-args', '-p', 'use_sim_time:=true']
    )
    # TF Map -> Odom (déjà publié par le contrôleur)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom', '--ros-args', '-p', 'use_sim_time:=true']
    )

    # f) Activation du Map Server (Déclenché après 5 secondes pour laisser le temps au nœud de démarrer)
    activate_map_server = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                output='screen'
            )
        ]
    )
    
    # --- 4. LANCEMENT DE RVIZ ---
    
    # g) Lance Rviz2 (après 7 secondes pour s'assurer que la carte est publiée)
    rviz_config_file = os.path.join(pkg_desc, 'rviz', 'default.rviz') # Assurez-vous d'avoir ce fichier
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    launch_rviz = TimerAction(
        period=7.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([
        # Définition des actions dans l'ordre
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity,
        
        # Composants de la carte/TF
        node_map_server,
        static_tf_world_map,
        static_tf_map_odom,
        
        # Activation du cycle de vie et lancement de Rviz
        activate_map_server,
        launch_rviz,
    ])