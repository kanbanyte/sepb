# onrobot
ROS drivers for OnRobot Grippers.\
This repository was inspired by [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq).

## Features
* ROS Noetic (Python3)
* Controller for OnRobot RG2 via Modbus/TCP

## Dependency
* pymodbus==2.5.3
* [roboticsgroup/roboticsgroup_upatras_gazebo_plugins](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins)

## Installation
```bash
cd ws/src
git clone https://github.com/takuya-ki/onrobot.git --depth 1
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git --depth 1
cd ..
sudo rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro humble -y --os=ubuntu:jammy -y
sudo apt install ros-humble-ros-control ros-humble-ros-controllers
colcon build --symlink-install
```

## Usage
1. Connect the cable between Compute Box and Tool Changer
2. Connect an ethernet cable between Compute Box and your computer
3. Execute programs (Please refer to [onrobot/Tutorials](http://wiki.ros.org/onrobot/Tutorials))

### RG2
#### Send motion commands
##### Interactive mode
```bash
roslaunch onrobot_rg_control bringup.launch gripper:=[rg2] ip:=XXX.XXX.XXX.XXX
rosrun onrobot_rg_control OnRobotRGSimpleController.py
```

##### ROS service call
```bash
roslaunch onrobot_rg_control bringup.launch gripper:=[rg2] ip:=XXX.XXX.XXX.XXX
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

## Author
[Takuya Kiyokawa](https://takuya-ki.github.io/)

## Contributors
[Roberto Mendivil C](https://github.com/Robertomendivil97)
