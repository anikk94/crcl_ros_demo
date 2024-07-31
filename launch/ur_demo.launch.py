from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
    TextSubstitution,
    PythonExpression
)
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    LogInfo
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    actions = []
    arguments = []

    # =========================================================================
    # launch file substitutions
    # =========================================================================
    # basic
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ros_gz_bridge
    config_file = LaunchConfiguration('config_file')
    container_name = LaunchConfiguration('container_name')
    namespace = LaunchConfiguration('namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')


    # =========================================================================
    # launch arguments
    # =========================================================================
    # actual cli launch arguments for the launch file
    # check arg list with `$ ros2 launch <package> <launch_file> -s`




    # =========================================================================
    # executables
    # =========================================================================
    # executables to be run, using launch configuration substitutions and declared launch arguments

    # UR Robot Description
    ur_type                  = LaunchConfiguration("ur_type")
    safety_limits            = LaunchConfiguration("safety_limits")

    runtime_config_package   = LaunchConfiguration("runtime_config_package")
    controllers_file         = LaunchConfiguration("controllers_file")
    moveit_config_package    = LaunchConfiguration("moveit_config_package")
    moveit_config_file       = LaunchConfiguration("moveit_config_file")
    
    safety_pos_margin        = LaunchConfiguration("safety_pos_margin")
    safety_k_position        = LaunchConfiguration("safety_k_position")
    description_package      = LaunchConfiguration("description_package")
    description_file         = LaunchConfiguration("description_file")
    
    prefix                   = LaunchConfiguration("prefix")
    launch_rviz              = LaunchConfiguration("launch_rviz")
    

    # ur specific arguments
    arguments.append(
        DeclareLaunchArgument(
            "ur_type", 
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"], 
            default_value="ur5e")
    )
    arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    # general arguments
    arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_simulation_gazebo",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false")
    )
    # simulation arguments
    arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true")
    )
    actions.append(
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value='/root/ur_ws/install/ur_description/share:/root/ur_ws/install/gzdatabase/share:/root/ur_ws/install/gzdatabase/share/gzdatabase/models'
        )
    )
    actions.append(
        LogInfo(
            msg=["GZ_SIM_RESOURCE_PATH: ", EnvironmentVariable('GZ_SIM_RESOURCE_PATH')]
        )
    )
    # ros_gz_bridge
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='', description='YAML config file'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='ros_gz_container',
        description='Name of container that nodes will load in if use composition',
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False', description='Use composed bringup if True'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )
    arguments.append(declare_config_file_cmd)
    arguments.append(declare_container_name_cmd)
    arguments.append(declare_namespace_cmd)
    arguments.append(declare_use_composition_cmd)
    arguments.append(declare_use_respawn_cmd)
    arguments.append(declare_log_level_cmd)
    
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    actions.append(robot_state_publisher)

    # moveit
    # rviz
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ur_moveit_config"),
            "/launch/ur_moveit.launch.py"
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    actions.append(moveit)

    # gazebo simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ros_gz_sim"),
            "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={
            # "gz_args": "empty.sdf",
            "gz_args": "/root/ur_ws/install/crcl_ros_demo/share/crcl_ros_demo/worlds/lab_world.sdf",
        }.items(),
    )

    actions.append(gz_sim)

    # # ros_gz_bridge

    # this doesn't exist for humble

    # ros_gz_bridge = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [PathJoinSubstitution([FindPackageShare('ros_gz_bridge'),
    #                                'launch',
    #                                'ros_gz_bridge.launch.py'])]),
    #     launch_arguments={
    #         'config_file': config_file,
    #         'container_name': container_name,
    #         'namespace': namespace,
    #         'use_composition': use_composition,
    #         'use_respawn': use_respawn,
    #         'log_level': log_level,
    #     }.items(),
    # )

    # actions.append(ros_gz_bridge)

    # spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable='create',
        arguments=[
            '-name', ur_type,
            '-topic', 'robot_description', 
            '-x', '2.2587',
            '-y', '-0.5',
            '-z', '0.8382',
        ],
        output='screen',
    )

    actions.append(spawn_robot)

    # user program
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