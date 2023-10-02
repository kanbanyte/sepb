# Steps Produced
```bash
cobot@cobot:~/Documents/repos/sepb/ws/src$ init_ros
cobot@cobot:~/Documents/repos/sepb/ws/src$ ros2 pkg create --build-type ament_python --node-name bot_node bot_package
# ...
cobot@cobot:~/Documents/repos/sepb/ws/src$ cd ..
cobot@cobot:~/Documents/repos/sepb/ws$ colcon build --packages-select bot_package
Starting >>> bot_package
# ...
Finished <<< bot_package [0.80s]

Summary: 1 package finished [0.95s]
  1 package had stderr output: bot_package
cobot@cobot:~/Documents/repos/sepb/ws$ source install/local_setup.bash
cobot@cobot:~/Documents/repos/sepb/ws$ ros2 run bot_package bot_node
Hi from bot_package.
cobot@cobot:~/Documents/repos/sepb/ws$ rosdep install -i --from-path src --rosdistro humble -y
#All required rosdeps installed successfully
cobot@cobot:~/Documents/repos/sepb/ws$ colcon build --packages-select bot_package
Starting >>> bot_package
# ...
Finished <<< bot_package [0.81s]

Summary: 1 package finished [0.97s]
  1 package had stderr output: bot_package
cobot@cobot:~/Documents/repos/sepb/ws$ source install/setup.bash
cobot@cobot:~/Documents/repos/sepb/ws$ ros2 launch bot_package bot_launch.py
# ...
cobot@cobot:~/Documents/repos/sepb/ws$
```

## Copy and Paste
```bash
cd ~/Documents/repos/sepb/ws/src
```

```bash
init_ros
```

```bash
cd ..
```

```bash
colcon build --packages-select bot_package
```

```bash
source install/local_setup.bash
```

```bash
ros2 run bot_package bot_node
```

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

```bash
colcon build --packages-select bot_package
```

```bash
source install/setup.bash
```

```bash
ros2 launch bot_package bot_launch.py
```
