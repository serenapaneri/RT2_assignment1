#!/bin/bash

gnome-terminal --tab --title="roscore" -- bash -c "source ros.sh; roscore &"
gnome-terminal --tab --title="ros1" -- bash -c "sleep 5; source ros.sh; roslaunch rt2_assignment1 sim.launch"
gnome-terminal --tab --title="ros12_bridge" -- bash -c "sleep 5, source ros12.sh; ros2 run ros1_bridge dynamic_bridge"
gnome-terminal --tab --title="ros2" -- bash -c "sleep 5; source ros2.sh; ros2 launch rt2_assignment1 ros2.py"
