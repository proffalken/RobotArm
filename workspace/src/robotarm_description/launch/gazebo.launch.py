import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory

def generate_launch_description():
    print("Getting the description dir")
    robotarm_description_dir = get_package_share_directory("robotarm_description")
    print("Setting the XACRO path")
    xacro_path = os.path.join(
            robotarm_description_dir,
            "urdf", 
            "robotarm.urdf.xacro"),

    print(f"XACRO PATH: {xacro_path}")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=xacro_path,
        description="The absolute path to the robot definition"
        )

    gazebo_resource_path = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH", 
            value = [
                str(Path(robotarm_description_dir).parent.resolve())
                ]
            )
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humber" else "False"
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"



    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
            ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]        
    )
    gz_launch_py = os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
                )

    print(gz_launch_py)

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_launch_py]),
                    launch_arguments = [
                        ("gz_args", [" -v 4 -r empty.sdf ", physics_engine ])
                    ]
            )

    gz_spawn_entity = Node(
            package = "ros_gz_sim",
            executable = "create",
            output="screen",
            arguments=["-topic", "robot_description",
                       "-name", "robotarm"
                       ]
            )

#    gz_ros2_bridge = Node(
#            package = "ros_gz_bridge",
#            executable = "parameter_bridge",
#            arguments = [
#                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock", 
#                ]
#            )

 # ros gz bridge
    bridge_params = os.path.join(
        get_package_share_directory('robotarm_description'),
        'params',
        'params.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '--ros-args',
        '-p',
        f'config_file:={bridge_params}',
    ],
    output='screen',
    )


    # ros gz bridge for image
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/rgb_camera/image_raw'],
        output='screen',
    )

    return LaunchDescription([
            DeclareLaunchArgument(
            'world',
            default_value='my_world',
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file',
                               value=[LaunchConfiguration('world'),
                                      TextSubstitution(text='.sdf')]),
            
                model_arg,
                gazebo_resource_path,
                robot_state_publisher,
                gazebo,
                gz_spawn_entity,
                #gz_ros2_bridge
                start_gazebo_ros_bridge_cmd,
                start_gazebo_ros_image_bridge_cmd
            ]
            )
