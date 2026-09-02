import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('go2_nav')
    nav2_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='False',
        description='Use simulation clock if true'
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=os.path.join(package_dir, 'params', 'nav2_explore_params.yaml'),
        description='Path to Nav2 params file'
    )

    # Use navigation_launch.py (Omits AMCL and Map Server)
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'namespace': namespace,
            'autostart': 'true'
        }.items()
    )

    # # Launch RViz
    # rviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_dir, 'launch', 'rviz_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'namespace': namespace
    #     }.items()
    # )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_nav_params_cmd)
    ld.add_action(navigation_cmd)
    # ld.add_action(rviz_cmd)

    return ld