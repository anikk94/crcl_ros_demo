from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    actions = []
    arguments = []

    # launch file substitutions
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # actual cli launch arguments for the launch file
    # check arg list with `$ ros2 launch <package> <launch_file> -s`
    arguments.append(DeclareLaunchArgument("ur_type", choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],))
    arguments.append(DeclareLaunchArgument("launch_rviz", default_value="false"))
    arguments.append(DeclareLaunchArgument("use_sim_time", default_value="true"))

    # executables to be run, using launch configuration substitutions and declared launch arguments
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ur_moveit_config"),
            "/launch/ur_moveit.launch.py"
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz,
        }.items(),
    ))

    demo_node = Node(
        package="crcl_ros_demo",
        executable="demo",
        output="screen",
        arguments=[ur_type],
        parameters=[{
            "ur_type": ur_type,
            "use_sim_time": use_sim_time,
        }],
    )

    actions.append(demo_node)

    # specifying runnables
    return LaunchDescription(
        arguments + actions
    )