"""Camera (detect profile, aligned depth on) plus the YOLO detector node.

    ros2 launch robot_bringup detector.launch.py engine_path:=$HOME/Robotics-cam/models/yolo26n.engine

The detector needs Ultralytics from the venv made by scripts/05_setup_yolo_venv.sh. This launch
file puts that venv's site-packages on PYTHONPATH (venv:=... to change it), so it works whether or
not the venv is activated.
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _venv_pythonpath(context):
    venv = os.path.expanduser(LaunchConfiguration("venv").perform(context))
    site = sorted(glob.glob(os.path.join(venv, "lib", "python3*", "site-packages")))
    if not site:
        raise RuntimeError(f"no venv at {venv}; run scripts/05_setup_yolo_venv.sh or pass venv:=<path>")
    paths = [site[-1]] + [p for p in os.environ.get("PYTHONPATH", "").split(os.pathsep) if p]
    return [SetEnvironmentVariable("PYTHONPATH", os.pathsep.join(paths))]


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("robot_bringup"), "launch", "camera.launch.py")),
        launch_arguments={"profile": LaunchConfiguration("profile")}.items(),
    )
    detector = Node(
        package="rover_perception",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        parameters=[{
            "engine_path": LaunchConfiguration("engine_path"),
            "conf": 0.4,
            "imgsz": 640,
            "camera_ns": "/camera/camera",
            "publish_debug_image": LaunchConfiguration("debug_image"),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="detect", description="detect | usb2"),
        DeclareLaunchArgument("engine_path", default_value=os.path.expanduser("~/Robotics-cam/models/yolo26n.engine")),
        DeclareLaunchArgument("debug_image", default_value="false"),
        DeclareLaunchArgument("venv", default_value="~/yolo-venv"),
        OpaqueFunction(function=_venv_pythonpath),
        camera,
        detector,
    ])
