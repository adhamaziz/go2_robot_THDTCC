from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # autostart variant: skips the manual `ros2 lifecycle set configure/activate`
    # step (same lifecycle-node pattern as slam_toolbox, just self-starting).
    aruco_tracker_node = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "cam_base_topic": "/d435i/color/image_raw",
            "marker_size": 0.15,
            "image_is_rectified": True,   # simulated cameras have no real lens distortion
            "marker_dict": "6X6_250",
        }],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            aruco_tracker_node,
        ]
    )
