from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
            "is_sim",
            default_value="True"
            )

    is_sim = LaunchConfiguration("is_sim")

    moveit_config = ( MoveItConfigsBuilder("robotarm", package_name="robotarm_moveit")
    .robot_description(file_path=os.path.join(get_package_share_directory("robotarm_description"), "urdf", "robotarm.urdf.xacro"))
    .robot_description_semantic(file_path="config/robotarm.srdf")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .moveit_cpp(file_path="config/planning_python_api.yaml")
    .to_moveit_configs()
    )

    task_server_node_py = Node(
            package="robotarm_remote",
            executable="task_server.py",
            parameters=[moveit_config.to_dict(), {"use_sim_time": is_sim}]
            )

    alexa_interface_node = Node(
            package="robotarm_remote",
            executable="alexa_interface.py",
            parameters=[{"use_sim_time": is_sim}]
            )


    return LaunchDescription(
            [
                is_sim_arg,
                task_server_node_py,
                alexa_interface_node
            ]
            )
