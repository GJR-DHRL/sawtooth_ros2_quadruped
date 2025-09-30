# Sawtooth ROS2 Quadruped Robot

**Sawtooth** is a **quadruped robot** project built to be fully compatible with ROS2. It integrates perception, control, navigation, simulation, and teleoperation into a modular architecture. The goal is to provide a flexible platform for research, experimentation, and deployment of quadruped locomotion and autonomy.

(“Inspired by the Sawtooth from Horizon Zero Dawn” — fan project, not affiliated with the game.)

---

## Work-In progress ...

### Work Completed till now :
* CAD design
(images)
* Structured repo
* URDF description
* Visualized in Rviz through -    
```ros2 launch urdf_tutorial display.launch.py model:=path/to/your/file```
* made launch file for rivz -   
```ros2 launch sawtooth_description display.launch.xml```
![alt text](<assets/rviz_img1.png>)
![alt text](<assets/rviz_img2.png>)
* dropped robot in gazebo
* made launch file to spawn robot only with rviz in gazebo -    
```ros2 launch sawtooth_bringup sawtooth_gazebo.launch.xml```
![alt text](<assets/gz_img1.png>)

### Future Tasks :

* implement custom controller using `ros2_control`
* sensor integration - 
    - IMU
    - Camera
    - LiDAR
    