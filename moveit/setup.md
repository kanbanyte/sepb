# UR5e Setup
Source ROS setup file in every new terminal.
```bash
source /opt/ros/humble/setup.bash
```

## Real Hardware
Setup UR robot driver; source robot type, and IP.
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=172.21.0.121 launch_rviz:=false
```

<!-- Find out what this does -->
```bash
ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load filename:\ \'ROS.urp\'
```

Power on robot, release brakes, and play to run programs.
<!-- Find out what play does exactly -->
```bash
ros2 service call /dashboard_client/power_on std_srvs/srv/Trigger
ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger
ros2 service call /dashboard_client/play std_srvs/srv/Trigger
```

Run robot using UR drivers.
```bash
ros2 control switch_controllers --activate scaled_joint_trajectory_controller
```

Run robot using moveit.
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

Power off after use.
```bash
ros2 service call /dashboard_client/power_off std_srvs/srv/Trigger
```

## Fake Hardware
When using fake hardware, change controllers.yaml file.
```bash
cd /opt/ros/humble/share/ur_moveit_config/config
code controller.yaml
```

Change `default` field to `true` for the `joint_trajectory_controller` and `false` for the `scaled_joint_trajectory_controller` then save the file.\
### Default
```yaml
controller_names:
  - scaled_joint_trajectory_controller
  - joint_trajectory_controller


scaled_joint_trajectory_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: false
  joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint


joint_trajectory_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
```

### Change
```diff
scaled_joint_trajectory_controller:
-  default: true
+  default: false

joint_trajectory_controller:
-  default: false
+  default: true
```

Run the following two commands in separate terminals after sourcing the ROS setup file.
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
```

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

## Creating a Package
### C++ package creation
```bash
ros2 pkg create \
 --build-type ament_cmake \
 --dependencies moveit_ros_planning_interface rclcpp \
 --node-name <file_name> <package_name>
```

### Python package creation
```bash
ros2 pkg create \
 --build-type ament_python \
 --dependencies moveit_ros_planning_interface rclpy \
 --node-name <file_name> <package_name>
```

## VSCode Include Path Settings
VSCode will complain about include errors unless the following settings is applied.
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**",
                "/usr/include/eigen3"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

The `.json` file should be created in the hidden directory (.vscode) with `c_cpp_properties` as the file name.
```gherkin
/sepb
  |-- .vscode
      |-- c_cpp_properties.json
```
