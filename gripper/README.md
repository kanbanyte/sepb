# onrobot
ROS drivers for OnRobot Grippers.

## Features
* ROS2 Humble (Python3)
* Controller for OnRobot RG2 via Modbus/TCP

## Dependency
* pymodbus==2.5.3

## Installation
```bash
cd ~/Documents/repos/sepb/gripper
```

```bash
# cannot be found?
# sudo apt install ros-humble-ros-control ros-humble-ros-controllers
```

```bash
# exports ROS_DOMAIN_ID=10 and sources /opt/ros/humble/setup.bash
init_ros
```

```bash
# might be fine without "--ignore-packages-from-source" included?
rosdep install -i --from-paths src --ignore-packages-from-source --rosdistro humble -y
```

```bash
colcon build --symlink-install
```

```bash
# also works...
colcon build
```

```bash
colcon build --packages-select onrobot_rg_control
```

```bash
# colcon build --packages-select onrobot_rg_modbus_tcp
```

## Usage
1. Connect the cable between Compute Box and Tool Changer.
2. Connect an ethernet cable between Compute Box and your computer.
3. Execute programs (Please refer to [onrobot/Tutorials](http://wiki.ros.org/onrobot/Tutorials)).

### RG2
#### Send motion commands
##### Interactive mode
```bash
# might be able to execute line without "gripper:=rg2 ip:=172.21.0.121" appended to the end?
# ros2 launch onrobot_rg_control rg2_launch.py gripper:=rg2 ip:=172.21.0.121
ros2 launch onrobot_rg_control rg2_launch.py
```

```bash
ros2 run onrobot_rg_control rg2_controller.py
```

##### ROS2 service call
```bash
# might be server_launch.py instead?
# ros2 launch onrobot_rg_control rg2_launch.py gripper:=rg2 ip:=172.21.0.121
ros2 launch onrobot_rg_control server_launch.py
```

```bash
ros2 run onrobot_rg_control rg2_server.py
```

```bash
# close grip?
ros2 service call /onrobot_rg/set_command c
```

```bash
# open grip?
ros2 service call /onrobot_rg/set_command o
```

```bash
# currently unknown...
ros2 service call /onrobot_rg/set_command '!!str 300'
```

```bash
# restart grip but not entire cobot?
ros2 service call /onrobot_rg/restart_power
```
