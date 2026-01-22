import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    openneato_nav_dir = get_package_share_directory('openneato_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    # Paths
    default_map_path = os.path.join(openneato_nav_dir, 'maps', 'map.yaml')
    default_params_path = os.path.join(openneato_nav_dir, 'params', 'nav2_params.yaml')
    urdf_file = os.path.join(openneato_nav_dir, 'urdf', 'neato.urdf')

    # Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM'
    )

    # --- Nodes ---

    # 1. Neato Driver (Hardware Interface)
    neato_driver_node = Node(
        package='openneato_driver',
        executable='neato_driver',
        name='neato_driver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. Robot State Publisher (TF)
    # Legge l'URDF e lo pubblica su /robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['cat ', urdf_file])
        }]
    )

    # 3. Rosbridge Server (WebSocket for Web Interface)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # SLAM Toolbox
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('slam')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Nav2 Bringup (AMCL, Planner, Controller, Costmaps)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_slam_cmd,
        neato_driver_node,
        robot_state_publisher_node,
        rosbridge_node,
        slam_toolbox_node,
        nav2_launch
    ])
