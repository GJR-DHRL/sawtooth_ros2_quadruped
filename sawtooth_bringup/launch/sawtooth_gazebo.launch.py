from launch import LaunchDescription
import os 
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() :

    # filepaths
    urdf_path = os.path.join(get_package_share_path ("sawtooth_description"),"urdf","sawtooth_robot.xacro")

    robot_description = ParameterValue(Command(['xacro ',urdf_path]), value_type=str)

    # Nodes 
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_path("ros_gz_sim"),"launch","gz_sim.launch.py")
        ]),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
        
    )

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"]
    )


    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_node,
        spawn_node,
    ])




