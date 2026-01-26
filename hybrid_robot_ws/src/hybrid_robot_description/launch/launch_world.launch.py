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
    map_yaml_path = os.path.join(pkg_desc, 'map', 'storage_map.yaml') 
    
    # Cargar el archivo XACRO del robot
    robot_description = {'robot_description': Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', PathJoinSubstitution([pkg_desc, 'urdf', 'hybrid_terrestrial.xacro'])
    ])}
    
    # --- 2. LANZAMIENTO DE COMPONENTES BÁSICOS ---
    
    # a) Lanzar Gazebo (Servidor y Cliente)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'gzclient': 'true', 'use_sim_time': use_sim_time}.items()
    )

    # b) Robot State Publisher (TF estáticas del robot)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # c) Spawn del robot en Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'hybrid_robot_terrestrial',
                                   '-x', '0.5', '-y', '5.0', '-z', '0.05', # Ajustado a 0.05 para evitar enterrarse
                                   '--ros-args', '--param', 'use_sim_time:=true'],
                        output='screen')

    # --- 3. GESTIÓN DE CONTROLADORES (NUEVO) ---

    # d) Carga del Joint State Broadcaster (Publica estados de los joints)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # e) Carga del Diff Drive Controller (Control de movimiento terrestre)
    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    #Launch del calculo del path
    rrt_planner_node = Node(
        package='hybrid_robot_description',
        executable='rrt_planner_node.py',
        name='rrt_planner_node',
        output='screen',
        emulate_tty=True, 
        parameters=[{'use_sim_time': True}]
    )

    # --- 4. LANZAMIENTO Y CONFIGURACIÓN DEL MAPA ---

    # f) Map Server
    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server', 
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml_path
        }]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'] # <--- DOIT CORRESPONDRE AU NOM CI-DESSUS
        }]
    )

    # g) TF estáticas globales
    static_tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map', '--ros-args', '-p', 'use_sim_time:=true']
    )

    # Liaison entre la carte et l'odométrie
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0.5', '5.0', '0', '0', '0', '0', 'map', 'odom'] #Mismo que el spawn en Gazebo
    )


    
    # --- 5. LANZAMIENTO DE RVIZ ---
    
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
        period=15.0,
        actions=[rviz_node]
    )

    launch_map_logic = TimerAction(
        period=5.0,
        actions=[
            node_map_server,
            lifecycle_manager_node,
            static_tf_world_map,
            static_tf_map_odom
        ]
    )
    

    return LaunchDescription([
        # Orden lógico de ejecución
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity,
        
        # Carga automática de controladores
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        
        # Mapa y TFs
        rrt_planner_node,
        launch_map_logic,

        # Visualización
        launch_rviz,
    ])