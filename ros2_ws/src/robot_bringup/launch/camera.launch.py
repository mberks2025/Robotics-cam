"""Start the single realsense2_camera node with one of this repo's profiles.

    ros2 launch robot_bringup camera.launch.py                  # default: vio profile (d435i_orin.yaml)
    ros2 launch robot_bringup camera.launch.py profile:=detect  # adds aligned depth for YOLO
    ros2 launch robot_bringup camera.launch.py profile:=terrain
    ros2 launch robot_bringup camera.launch.py profile:=usb2    # while still on USB 2.1

Only one process can open the D435i, so every other node subscribes to this one.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PROFILES = {
    "vio": "d435i_orin.yaml",
    "detect": "d435i_detect.yaml",
    "terrain": "d435i_terrain.yaml",
    "usb2": "d435i_usb2.yaml",
}


def _include_camera(context):
    profile = LaunchConfiguration("profile").perform(context)
    if profile not in PROFILES:
        raise RuntimeError(f"unknown profile '{profile}', pick one of {sorted(PROFILES)}")
    config = os.path.join(get_package_share_directory("robot_bringup"), "config", PROFILES[profile])
    rs_launch = os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                "config_file": config,
                "camera_namespace": LaunchConfiguration("camera_namespace").perform(context),
                "camera_name": LaunchConfiguration("camera_name").perform(context),
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="vio", description="vio | detect | terrain | usb2"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        OpaqueFunction(function=_include_camera),
    ])
