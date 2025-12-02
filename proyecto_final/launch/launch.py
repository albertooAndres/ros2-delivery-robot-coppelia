# launch/launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    # ───── rutas y argumentos ─────────────────────────────────────────────
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('proyecto_final'),
            'param', 'burger.yaml'))

    scene_dir = LaunchConfiguration(
        'scene_dir',
        default=os.path.join(
            get_package_share_directory('proyecto_final'),
            'scenes', 'trabajo_final.ttt'))

    rviz_dir = LaunchConfiguration(
        'rviz_dir',
        default=os.path.join(
            get_package_share_directory('proyecto_final'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(
        get_package_share_directory('proyecto_final'),
        'urdf', 'turtlebot3_burger.urdf')

    # ───── NODOS ROS 2 ────────────────────────────────────────────────────
    # 1) Freno de emergencia LIDAR
    lidar_bridge_node = Node(
        package   ='proyecto_final',
        executable='lidar_bridge_node',
        name      ='lidar_bridge',
        output    ='screen',
        parameters=[{
            'safe_distance': 0.17,     # m
            'use_sim_time' : use_sim_time
        }]
    )

    # 2) Seguidor de línea IR  
    ir_line_follower_node = Node(
        package   ='proyecto_final',
        executable='ir_line_follower_node',
        name      ='ir_line_follower',
        output    ='screen',
        parameters=[{
            'forward_speed'      : 0.12,  # m/s
            'search_turn_speed'  : 0.4,   # rad/s
            'turn_direction'     : 1,     # 1 = izquierda, -1 = derecha
            'lost_line_timeout'  : 1.0,   # s
            'pub_rate'           : 20.0,  # Hz
            'use_sim_time'       : use_sim_time
        }]
    )
    
    # 3) Semaforo paso de zebra  
    traffic_light_node = Node(
        package   ='proyecto_final',
        executable='traffic_light_node',
        name      ='traffic_light',
        output    ='screen',
        parameters =[{
            'green': True
        }]
    )

    # ───── DESCRIPCIÓN DEL LANZAMIENTO ────────────────────────────────────
    return LaunchDescription([
        LogInfo(msg=['Lanzando proyecto final ROM!!']),

        DeclareLaunchArgument('param_dir',  default_value=param_dir,
                              description='Specifying parameter file'),
        DeclareLaunchArgument('scene_dir',  default_value=scene_dir,
                              description='Specifying Coppelia scene path'),

        # Ejecuta CoppeliaSim con la escena
        ExecuteProcess(
            cmd=[['./coppeliaSim -f ', scene_dir, ' -s 0']],
            cwd='/home/alberto_andres/ROM/Coppelia',
            shell=True,
            output='screen'),

        # RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_dir, '/rviz2.launch.py'])
        ),

        # Robot State Publisher
        Node(
            package   ='robot_state_publisher',
            executable='robot_state_publisher',
            name      ='robot_state_publisher',
            output    ='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments =[urdf]
        ),

        # ─── mis nodos ───────────────────────────────────────────
        lidar_bridge_node,
        ir_line_follower_node,     
        traffic_light_node       
    ])

