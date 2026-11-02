"""Camera (terrain profile: depth + IMU, emitter on, filters) plus the terrain node.

    ros2 launch robot_bringup terrain.launch.py
    ros2 launch robot_bringup terrain.launch.py debug_image:=true step_up:=0.10
    ros2 topic echo /terrain/features

In rviz2 set Fixed Frame to terrain_ground and add a Map display on /terrain/grid.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("robot_bringup"), "launch", "camera.launch.py")),
        launch_arguments={"profile": LaunchConfiguration("profile")}.items(),
    )
    terrain = Node(
        package="rover_perception",
        executable="terrain_node",
        name="terrain_node",
        output="screen",
        parameters=[{
            "camera_ns": "/camera/camera",
            "debug_image": ParameterValue(LaunchConfiguration("debug_image"), value_type=bool),
            "step_up": ParameterValue(LaunchConfiguration("step_up"), value_type=float),
            "step_down": ParameterValue(LaunchConfiguration("step_down"), value_type=float),
            "max_range": ParameterValue(LaunchConfiguration("max_range"), value_type=float),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="terrain", description="terrain | usb2 | detect"),
        DeclareLaunchArgument("debug_image", default_value="false"),
        DeclareLaunchArgument("step_up", default_value="0.08", description="m above ground = bump"),
        DeclareLaunchArgument("step_down", default_value="0.08", description="m below ground = dip"),
        DeclareLaunchArgument("max_range", default_value="3.0"),
        camera,
        terrain,
    ])
