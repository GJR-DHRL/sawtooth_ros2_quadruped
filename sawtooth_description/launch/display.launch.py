from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
# ParameterValue(...): This is a class from launch_ros.parameter_descriptions used to define a parameter's value within a ROS 2 launch file. It allows for dynamic generation of parameter values.
from launch.substitutions import Command
from launch_ros.actions import Node
# import files are inorder as they are used 

def generate_launch_description() :

    urdf_path = os.path.join(get_package_share_path("sawtooth_description"),"urdf","sawtooth_robot.xacro")

    rviz_config_path = os.path.join(get_package_share_path ("sawtooth_description"),"rviz","sawtooth_config.rviz")

    # create variable in order to pass it as param
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # robot_description = xacro.process_file(urdf_path).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # pass params as dictionary
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        # pass args as array
        arguments=['-d', rviz_config_path],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])


""" 
robot_description = ParameterValue(Command(['xacro ',urdf_path]), value_type=str)

robot_description: This is a parameter name, typically used by ROS 2 nodes like robot_state_publisher to receive the robot's URDF (Unified Robot Description Format) or Xacro model.

ParameterValue(...): This is a class from launch_ros.parameter_descriptions used to define a parameter's value within a ROS 2 launch file. It allows for dynamic generation of parameter values.

Command(['xacro ',urdf_path]): This is a substitution from launch.substitutions that allows you to execute an external command as part of your launch file.

'xacro ': This specifies the xacro command-line tool, which is used to process Xacro files and expand them into standard URDF XML.

urdf_path: This is a variable or substitution representing the path to your Xacro file (e.g., my_robot.urdf.xacro). The xacro command will take this file as input.

value_type=str: This indicates that the output of the Command (which is the expanded URDF XML from the Xacro file) should be treated as a string when assigned to the robot_description parameter.

In essence, this line dynamically generates the robot's URDF description by executing the xacro command on the specified Xacro file (urdf_path) and then assigns the resulting URDF XML as a string to the robot_description parameter. This allows you to define your robot model using the more flexible and modular Xacro format while still providing the necessary URDF to ROS 2 components that require it.

"""