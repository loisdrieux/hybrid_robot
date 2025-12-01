import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 1. Chemin des Modèles Gazebo
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_desc = get_package_share_directory('hybrid_robot_description')

    gazebo_models_path = os.path.join(pkg_desc, 'models')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', gazebo_models_path]
    )

    # --- 2. Définition du Modèle URDF (à partir du XACRO) ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_desc, 'urdf', 'hybrid_terrestrial.xacro']),
    ])
    robot_description = {'robot_description': robot_description_content}

    # --- 3. Chemin vers le Monde Gazebo ---
    world_file_name = 'hybrid_map.world'
    world_path = os.path.join(pkg_desc, 'worlds', world_file_name)

    # --- 4. Définition du Chemin de Configuration du Contrôleur ---
    # Le fichier terrestrial_control.yaml définit le diff_drive_controller
    terrestrial_control_config = PathJoinSubstitution(
        [pkg_desc, "config", "terrestrial_control.yaml"]
    )

    #  Lancer Gazebo avec le monde personnalisé
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # Argument pour spécifier le monde à charger
        launch_arguments={'world': world_path}.items()
    )

   # B. Publication de l'État du Robot (RSP)
    # Publie le modèle URDF pour RViz et l'insertion dans Gazebo
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # C. Insertion du Modèle dans Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'hybrid_robot_terrestrial',
                                   '-x', '0.5', 
                                   '-y', '5.0',
                                   '-z', '0.0'],
                        output='screen')
    
    # D. Charger et Démarrer le Controller Manager
    # Le Controller Manager (CM) gère tous les contrôleurs du robot
    load_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, terrestrial_control_config],
        output='screen',
    )
    
    # E. Charger et Démarrer le Contrôleur Différentiel
    # Le spawner demande au CM de démarrer le diff_drive_controller
    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    # F. Attacher la Commande I/O du Contrôleur (Joint State Broadcaster)
    # Nécessaire pour publier les états des roues (vitesse/position)
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    return LaunchDescription([
        set_model_path,
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity,
        load_diff_drive_controller,
           ])
