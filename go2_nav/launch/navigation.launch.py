import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('go2_nav')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    # Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'rviz', 
        default_value='True',
        description='Whether to start RViz'
    )
    declare_map_cmd = DeclareLaunchArgument(
        'map', 
        default_value='/ros2_ws/may4.yaml',
        description='Full path to map yaml file to load'
    )
    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=os.path.join(package_dir, 'params', 'nav2_humble_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Nav2 Bringup (Nav2 stack + AMCL + Map Server)
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'namespace': namespace,
            'use_rviz': 'false',  # Handled separately below
            'autostart': 'true'
        }.items()
    )

    # RViz Bringup
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'namespace': namespace
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_nav_params_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)

    return ld