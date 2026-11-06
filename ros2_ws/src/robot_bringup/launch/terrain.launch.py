"""Camera (terrain profile: depth + IMU, emitter on, filters) plus the terrain node.

    ros2 launch robot_bringup terrain.launch.py
    ros2 launch robot_bringup terrain.launch.py debug_image:=true step_up:=0.10
    ros2 topic echo /terrain/features

    ros2 launch robot_bringup terrain.launch.py fake_odom:=true   # bench test: camera held still, map in odom

With odometry running (TF odom -> camera), the node keeps a rolling 6 x 6 m map in odom. Without it,
each frame is a fresh single-frame map. In rviz2 set Fixed Frame to odom (or terrain_ground without
odometry) and add a Map display on /terrain/grid.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
            "use_odom": ParameterValue(LaunchConfiguration("use_odom"), value_type=bool),
        }],
    )
    # Bench test only: pretend the camera is fixed in odom so the rolling map can be checked
    # before cuVSLAM / OpenVINS exist. Never run this together with real odometry.
    fake_odom = Node(
        package="tf2_ros", executable="static_transform_publisher", name="fake_odom",
        arguments=["--frame-id", "odom", "--child-frame-id", "camera_link"],
        condition=IfCondition(LaunchConfiguration("fake_odom")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="terrain", description="terrain | usb2 | detect"),
        DeclareLaunchArgument("debug_image", default_value="false"),
        DeclareLaunchArgument("step_up", default_value="0.08", description="m above ground = bump"),
        DeclareLaunchArgument("step_down", default_value="0.08", description="m below ground = dip"),
        DeclareLaunchArgument("max_range", default_value="3.0"),
        DeclareLaunchArgument("use_odom", default_value="true", description="rolling map in odom when TF exists"),
        DeclareLaunchArgument("fake_odom", default_value="false", description="static odom -> camera_link for bench tests"),
        camera,
        terrain,
        fake_odom,
    ])
