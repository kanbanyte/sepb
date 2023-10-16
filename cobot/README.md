# Steps Produced
```bash
cobot@cobot:~/Documents/repos/sepb/cobot/src$ init_ros
cobot@cobot:~/Documents/repos/sepb/cobot/src$ ros2 pkg create --build-type ament_python --node-name cobot_node pick_place_package
# ...
cobot@cobot:~/Documents/repos/sepb/cobot/src$ cd ..
cobot@cobot:~/Documents/repos/sepb/cobot$ colcon build --packages-select pick_place_package
Starting >>> pick_place_package
# ...
Finished <<< pick_place_package [0.80s]

Summary: 1 package finished [0.95s]
  1 package had stderr output: pick_place_package
cobot@cobot:~/Documents/repos/sepb/cobot$ source install/local_setup.bash
cobot@cobot:~/Documents/repos/sepb/cobot$ ros2 run pick_place_package cobot_node
Hi from pick_place_package.
cobot@cobot:~/Documents/repos/sepb/cobot$ rosdep install -i --from-path src --rosdistro humble -y
#All required rosdeps installed successfully
cobot@cobot:~/Documents/repos/sepb/cobot$ colcon build --packages-select pick_place_package
Starting >>> pick_place_package
# ...
Finished <<< pick_place_package [0.81s]

Summary: 1 package finished [0.97s]
  1 package had stderr output: pick_place_package
cobot@cobot:~/Documents/repos/sepb/cobot$ source install/setup.bash
cobot@cobot:~/Documents/repos/sepb/cobot$ ros2 launch pick_place_package pick_place_launch.py
# ...
cobot@cobot:~/Documents/repos/sepb/cobot$
```

## Copy and Paste
```bash
cd ~/Documents/repos/sepb/cobot/src
```

```bash
init_ros
```

```bash
cd ..
```

```bash
colcon build --packages-select pick_place_package
```

```bash
source install/local_setup.bash
```

```bash
ros2 run pick_place_package cobot_node
```

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

```bash
colcon build --packages-select pick_place_package
```

```bash
source install/setup.bash
```

```bash
ros2 launch pick_place_package pick_place_launch.py
```
