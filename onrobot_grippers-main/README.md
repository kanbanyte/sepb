# onrobot
ROS drivers for OnRobot Grippers.\
This repository was inspired by [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq).

## Features
* ROS Noetic (Python3)
* Controller for OnRobot RG2

## Dependency
* pymodbus==2.5.3
* [roboticsgroup/roboticsgroup_upatras_gazebo_plugins](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git)

## Installation
```bash
cd catkin_ws/src
git clone https://github.com/takuya-ki/onrobot.git --depth 1
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git --depth 1
cd ../
sudo rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y --os=ubuntu:jammy -y
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Usage
1. Connect the cable between Compute Box and Tool Changer.
2. Connect an ethernet cable between Compute Box and your computer.
3. Execute programs (Please refer to [onrobot/Tutorials](http://wiki.ros.org/onrobot/Tutorials)).

### RG2
#### Send motion commands
##### Interactive mode
```bash
roslaunch onrobot_rg_control bringup.launch gripper:=[rg2/rg6] ip:=XXX.XXX.XXX.XXX
rosrun onrobot_rg_control OnRobotRGSimpleController.py
```

##### ROS service call
```bash
roslaunch onrobot_rg_control bringup.launch gripper:=[rg2/rg6] ip:=XXX.XXX.XXX.XXX
rosrun onrobot_rg_control OnRobotRGSimpleControllerServer.py
rosservice call /onrobot_rg/set_command c
rosservice call /onrobot_rg/set_command o
rosservice call /onrobot_rg/set_command '!!str 300'
rosservice call /onrobot_rg/restart_power
```

#### Simulation
##### Display models
```bash
roslaunch onrobot_rg_description disp_rg2_model.launch
```

##### Gazebo simulation
```bash
roslaunch onrobot_rg_gazebo bringup_rg2_gazebo.launch
rostopic pub -1 /onrobot_rg2/joint_position_controller/command std_msgs/Float64 "data: 0.5"
```

### VG10 / VGC10
#### Send motion commands
##### Interactive mode
```bash
roslaunch onrobot_vg_control bringup.launch ip:=YYY.YYY.YYY.YYY
rosrun onrobot_vg_control OnRobotVGSimpleController.py
```

##### ROS service call
```bash
roslaunch onrobot_vg_control bringup.launch ip:=YYY.YYY.YYY.YYY
rosrun onrobot_vg_control OnRobotVGSimpleControllerServer.py
rosservice call /onrobot_vg/set_command g
rosservice call /onrobot_vg/set_command r
rosservice call /onrobot_vg/set_command '!!str 128'
```

#### Simulation
##### Display models
```bash
roslaunch onrobot_vg_description disp_vgc10_1cup_model.launch
roslaunch onrobot_vg_description disp_vgc10_4cups_model.launch
roslaunch onrobot_vg_description disp_vg10_model.launch
```

## Author
[Takuya Kiyokawa](https://takuya-ki.github.io/)
