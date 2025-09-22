# to launch whole simulation for sawtooth

import os   # for modularity and to perform file/path operations
import launch_ros   # mandetory to connect with ros/ to get excess of ros
from launch_ros.actions import Node # to make nodes 

from launch.actions import (
    DeclareLaunchArgument   # to declare launch-time variables so user can change them through CLI
    
)

from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution     # To get excess of substitutions module

"""
-> Outside = fixed at parse-time (no substitution, no CLI override, just a python string).
-> Inside = launch-time substitution (dynamic, user-configurable, user can pass arguments from CLI ).
-> That's why you always see DeclareLaunchArgument + LaunchConfiguration inside generate_launch_description() → it gives flexibility, supports CLI overrides, and works with ROS 2's launch system.
"""


def generate_launch_description():
    
    sawtooth_sim_path = launch_ros.substitutions.FindPackageShare(package="sawtooth_sim").find("sawtooth_sim")
    """
    1. Why is launch_ros used here?
    -> launch_ros is the ROS 2 launch extension for Python.
    -> It adds ROS-specific launch features on top of the generic launch framework (which is not ROS-only).
    -> Since we want to deal with ROS 2 packages (sawtooth_sim in this case), we use launch_ros.

    2. What is substitutions?
    -> In ROS 2 launch files, substitutions are placeholders that get resolved at runtime.
    -> Instead of hardcoding paths or values, we use substitutions so they are flexible and work across different systems.
    -> Example: FindPackageShare("my_pkg") → resolves to 
    /home/user/ros2_ws/install/my_pkg/share/my_pkg (but this is resolved at runtime).
    -> This makes launch files portable — you don't need to hardcode /home/....

    3. Why do we use FindPackageShare?
    -> FindPackageShare(package="sawtooth_sim") searches your ROS 2 environment for the installed path of that package, and specifically its share/ directory bcz it contains launch files, URDFs, meshes, configs
    -> FindPackageShare ensures you always get the correct share/ path for the package, no matter where the workspace is installed.
    -> Example:
        -> On your PC → 
        /home/you/ros2_ws/install/sawtooth_sim/share/sawtooth_sim
        -> On a robot → 
        /opt/ros/humble/install/sawtooth_sim/share/sawtooth_sim
        …it will still work.

    4. What is the use of .find("sawtooth_sim") if we already found the package path?
    -> FindPackageShare("sawtooth_sim") returns a path to the package's share/ directory, e.g.:
        /home/you/ros2_ws/install/sawtooth_sim/share
    -> .find("sawtooth_sim") is then appending/locating a folder inside that share directory bcz there might be many other directories. The .find("sawtooth_sim") step helps get you to: /home/you/ros2_ws/install/sawtooth_sim/share/sawtooth_sim
    """
    sawtooth_description_path = launch_ros.substitutions.FindPackageShare(package="sawtooth_description").find("sawtooth_description")

    # absolute file paths 
    """
    os.path.join(pathA, path2) gives 'pathA/path2' (in a single string)
    """
    default_model_path = os.path.join(sawtooth_description_path,"urdf/sawtooth_robot.xacro")    # change accordingly
    default_world_path = os.path.join(sawtooth_sim_path, "worlds/default.sdf")  # change accordingly

    # declaring launch-time variables so user can change them through CLI

    # Robot name
    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="Sawtooth",
        description="Name of the robot" )
    
    # to use_sim_time or not
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true", # why value is passed as string instead of bool ? bcz launch arguments are processed in strings only for flexibility.
        description="Use simulation (Gazebo) clock if true" )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # simulator world file-path 
    declare_gazebo_world_path = DeclareLaunchArgument(
        "gazebo_world_path",
        default_value=default_world_path,
        description="Gazebo file path" )
    
    # robot model file-path
    declare_model_description_path = DeclareLaunchArgument(
        "sawtooth_description_path",
        default_value=default_model_path,
        description="Path to robot model description xacro file" )

    # spawning position of robot in gazebo world
    declare_init_spawn_x = DeclareLaunchArgument(
        "init_spawn_x",
        default_value="0.0",
        description="x coordinate to spawn robot in world" )
    
    declare_init_spawn_y = DeclareLaunchArgument(
        "init_spawn_y",
        default_value="0.0",
        description="y coordinate to spawn robot in world" )
    
    declare_init_spawn_z = DeclareLaunchArgument(
        "init_spawn_z",
        default_value="0.375",
        description="z coordinate to spawn robot in world" )
    
    # Description of parameters  
    robot_description_path = {"robot_description": Command(["xacro ", LaunchConfiguration("sawtooth_description_path")])}
     
    # Description of nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description_path,
            {"use_sim_time": use_sim_time}
        ]
    )
