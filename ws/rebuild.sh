#!/bin/bash
rm -r /home/cobot/Documents/repos/sepb/ws/build
rm -r /home/cobot/Documents/repos/sepb/ws/install
rm -r /home/cobot/Documents/repos/sepb/ws/log
cd /home/cobot/Documents/repos/sepb/ws
colcon build
source install/local_setup.bash
rosdep install -i --from-path src --rosdistro humble -y
