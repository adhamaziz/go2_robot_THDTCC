# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros.descriptions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []
    nodes = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='go2_description',
            description='Description package with robot URDF/xacro files made by manufacturer.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='go2_description.urdf',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix to be added to the robot description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation/Gazebo clock if true',
        )
    )

    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare('go2_description'), 'urdf', description_file]),
            ' ',
            'prefix:=', prefix
        ]
    )

    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

    ekf_config = PathJoinSubstitution([
        FindPackageShare('go2_driver'),
        'config',
        'ekf_odom.yaml'
    ])

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_param,
            'publish_frequency': 100.0,
            'frame_prefix': '',
            }],
        )
    )

    '''nodes.append(Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
                ('cloud_in', '/best_points'),
                ('scan', '/best_scan'),
        ],
        parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 0.1,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
            }]

    ))'''

    '''nodes.append(Node(
        package='robot_localization',
        name='base_link_to_odom_ekf',
        executable='ekf_node',
        output ='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        )
    )'''
    
    
    # Go2 URDF connection (base_footprint -> base_link)  
    # nodes.append(Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_footprint_to_base_link_tf_node',
    #     arguments=[
    #         '--x', '0', '--y', '0', '--z', '0',
    #         '--roll', '0', '--pitch', '0', '--yaw', '0',
    #         '--frame-id', 'base_footprint',
    #         '--child-frame-id', 'base_link' ],
    # ))

    return LaunchDescription(declared_arguments + nodes)
