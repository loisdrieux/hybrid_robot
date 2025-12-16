import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. CONFIGURACIÓN DE BASE ---
    pkg_desc = get_package_share_directory('hybrid_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Parámetros globales
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_path = os.path.join(pkg_desc, 'worlds', 'hybrid_map.world')
    map_yaml_path = os.path.join(pkg_desc, 'map', 'storage_map.yaml') # Ruta del archivo YAML de la mapa
    
    # Cargar el archivo XACRO del robot
    robot_description = {'robot_description': Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', PathJoinSubstitution([pkg_desc, 'urdf', 'hybrid_terrestrial.xacro'])
    ])}
    
    # --- 2. LANZAMIENTO DE COMPONENTES BÁSICOS ---
    
    # a) Lanzar el Servidor y Cliente de Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'gzclient': 'true', 'use_sim_time': use_sim_time}.items()
    )

    # b) Lanzar Robot State Publisher (Publica las transformaciones TF odom -> base_link)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # c) Generar (spawn) la entidad del robot en Gazebo (con Z=0.01 para asegurar fricción inicial)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'hybrid_robot_terrestrial',
                                   '-x', '0.5', 
                                   '-y', '5.0',
                                   '-z', '0.01', 
                                   '--ros-args', '--param', 'use_sim_time:=true'],
                        output='screen')

    # --- 3. LANZAMIENTO Y CONFIGURACIÓN DEL MAPA (MAP) ---

    # d) Lanzar el Servidor de Mapas (Map Server)
    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_path}]
    )

    # e) Publicar transformaciones estáticas (World -> Map y Map -> Odom)
    # TF World -> Map
    static_tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map', '--ros-args', '-p', 'use_sim_time:=true']
    )
    # TF Map -> Odom (Nota: esto suele ser gestionado por el controlador, pero se incluye como estático aquí)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom', '--ros-args', '-p', 'use_sim_time:=true']
    )

    # f) Activación del Map Server (Se activa tras 5 segundos para permitir que el nodo inicie correctamente)
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
    
    # --- 4. LANZAMIENTO DE RVIZ ---
    
    # g) Lanzar Rviz2 (después de 7 segundos para asegurar que el mapa ya esté publicado)
    rviz_config_file = os.path.join(pkg_desc, 'rviz', 'default.rviz') 
    
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
        # Ejecución de acciones en orden lógico
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity,
        
        # Componentes de mapa y transformaciones TF
        node_map_server,
        static_tf_world_map,
        static_tf_map_odom,
        
        # Activación del ciclo de vida del mapa y lanzamiento de Rviz
        activate_map_server,
        launch_rviz,
    ])