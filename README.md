# ROS2 based control of FANUC M10iA Robot:

Libraries needed:
ROS 2 Humble
ROS Industrial support files of FANUC M10iA
OPW_Kinematics

```
pkill -9 -f rviz2
pkill -9 -f robot_state_publisher
pkill -9 -f joint_state_publisher_gui
sleep 1
pgrep -af "rviz2\|robot_state_publisher\|joint_state_publisher" || echo "all dead"

cd ~/Vaibhav-GitHub/FANUC-Robot-Control
colcon build --packages-select fanuc_description
source install/setup.bash
ros2 launch fanuc_description view_robot.launch.py
```