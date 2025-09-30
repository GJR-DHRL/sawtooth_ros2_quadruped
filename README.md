# Sawtooth ROS2 Quadruped Robot

**Sawtooth** is a quadruped robot project built to be fully compatible with ROS2. It integrates perception, control, navigation, simulation, and teleoperation into a modular architecture. The goal is to provide a flexible platform for research, experimentation, and deployment of quadruped locomotion and autonomy.

(“Inspired by the Sawtooth from Horizon Zero Dawn” — fan project, not affiliated with the game.)

---

Work-In progress

Work Completed till now :
* CAD design
* structured repo
* URDF description
* Visualized in Rviz through -
    ```
    ros2 launch urdf_tutorial display.launch.py model:=path/to/your/file
    ```

Future Tasks :
* make launch file for rivz
* drop robot in gazebo
* make launch file for robot only in gazebo
* sensor integration - 
    - imu
    - camera
    - lidar
    