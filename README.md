# ur5_demo

# Version
Kinetic

# Required library
```
sudo apt-get install ros-kinetic-universal-robot

sudo apt-get install \
  ros-kinetic-ur-gazebo \
  ros-kinetic-ur5-moveit-config \
  ros-kinetic-ur-kinematics
```

# Build

```
cd ~/catkin_ws/src
git clone https://github.com/EdXian/ur5_control.git
cd..
catkin_make
```

Check out the path of package
```
rospack list

--------result----------
.
.
.
ur10_moveit_config /home/catkin_ws/src/ur5_control/universal_robot/ur10_moveit_config
ur3_moveit_config /home/catkin_ws/src/ur5_control/universal_robot/ur3_moveit_config
ur5_moveit_config /home/catkin_ws/src/ur5_control/universal_robot/ur5_moveit_config
ur_bringup /home/catkin_ws/src/ur5_control/universal_robot/ur_bringup
ur_description /home/catkin_ws/src/ur5_control/universal_robot/ur_description
ur_driver /home/catkin_ws/src/ur5_control/universal_robot/ur_driver
ur_gazebo /home/catkin_ws/src/ur5_control/universal_robot/ur_gazebo
ur_kinematics /home/catkin_ws/src/ur5_control/universal_robot/ur_kinematics
ur_msgs /home/catkin_ws/src/ur5_control/universal_robot/ur_msgs
.
.
```

# How to run simulation?

run gazebo simulation `roslaunch ur_gazebo ur5.launch`

run moveit and rviz simulatiom.

```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

# How to Control?

C++ example   `rosrun urtest ur5test`

Python example `rosrun urtest ur5test.py `
