from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    view_ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(([
            get_package_share_directory("ur_description"),
            "/launch/view_ur.launch.py"
        ])),
        launch_arguments={
            "ur_type": "ur5e",
        }.items(),
    )

    return LaunchDescription([
        view_ur_launch,

    ])