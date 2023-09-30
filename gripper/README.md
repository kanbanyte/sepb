# onrobot
ROS drivers for OnRobot Grippers.

## Features
* ROS2 Humble (Python3)
* Controller for OnRobot RG2 via Modbus/TCP

## Dependency
* pymodbus==2.5.3

## Installation
```bash
cd ~/Documents/repos/sepb/gripper/src
```

```bash
git clone https://github.com/takuya-ki/onrobot.git --depth 1
```

```bash
cd ..
```

```bash
sudo apt install ros-humble-ros-control ros-humble-ros-controllers
```

```bash
init_ros
```

```bash
rosdep install -i --from-paths src --ignore-packages-from-source --rosdistro humble -y
```

```bash
colcon build --symlink-install
```

```bash
colcon build --packages-select onrobot_rg_control
```

## Usage
1. Connect the cable between Compute Box and Tool Changer.
2. Connect an ethernet cable between Compute Box and your computer.
3. Execute programs (Please refer to [onrobot/Tutorials](http://wiki.ros.org/onrobot/Tutorials)).

### RG2
#### Send motion commands
##### Interactive mode
```bash
ros2 launch onrobot_rg_control bringup.launch gripper:=rg2 ip:=172.21.0.121
```

```bash
ros2 run onrobot_rg_control OnRobotRGSimpleController.py
```

##### ROS2 service call
```bash
ros2 launch onrobot_rg_control bringup.launch gripper:=rg2 ip:=172.21.0.121
```

```bash
ros2 run onrobot_rg_control OnRobotRGSimpleControllerServer.py
```

```bash
ros2 service call /onrobot_rg/set_command c
```

```bash
ros2 service call /onrobot_rg/set_command o
```

```bash
ros2 service call /onrobot_rg/set_command '!!str 300'
```

```bash
ros2 service call /onrobot_rg/restart_power
```
