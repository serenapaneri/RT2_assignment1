# First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)

### General information
This ROS package contains two nodes written in ROS2 and the simulation environment for controlling a mobile robot in a Gazebo simulation environment.
The general beaviour will be: you have a user interface with which you can directly interact to the robot making it moves pressing 1 and makes it stops pressing any other number.
In particular, when you press 1, and so starting the simulation, the robot starts moving since it is trying to reaching a target position and a target orientation in the space.
When it stops, a new random position is sent to it and it will again try to reach it. 
But please, pay attention to the fact that when you try to stop the simulation pressing whatever number (except 1) the robot would not stop immediately to move, instead it will reach the last target position and orientation, given to it, before stopping.

### Nodes
We have four nodes in total:
1. [**position_service**](https://github.com/serenapaneri/rt2_assignment1/blob/ros2/src/position_service.cpp): this node is contained in the src folder and it implements a service, it waits for a request from a client and replies with a position x and y and the orientation theta. This is a c++ node.
2. [**state_machine**](https://github.com/serenapaneri/rt2_assignment1/blob/ros2/src/state_machine.cpp): this node is contained in the src folder and it represents a finite state machine which has only two states that are when the robot starts and when the robot stops. This node may call the position server and after called the position server, so after it receives the target x y and theta position, it will call the navigation service. It also implement a service for interacting with the user interface. This is a c++ node.

### Custom services in the srv folder
1. [**RandomPosition**](https://github.com/serenapaneri/rt2_assignment1/blob/main/srv/RandomPosition.srv): it expects a x_max, x_min, y_max and y_min and replies with an x, y and theta. This is the service message used by the random position service. 
2. [**Position**](https://github.com/serenapaneri/rt2_assignment1/blob/main/srv/Position.srv): it is the message sent to the go_to_point service. 
3. [**Command**](https://github.com/serenapaneri/rt2_assignment1/blob/main/srv/Command.srv): it is just a string command which will be start or stop. 

### Launch file
There is a launch file contained in the folder launch which is [**ros2.py**](https://github.com/serenapaneri/rt2_assignment1/blob/ros2/launch/ros2.py) that will just spawn the robot in the Gazebo simulator and then it launches the two nodes, or better the two executables inside a container used for composable nodes. 

### mapping_rule.yaml
This file is used to make possible the communication between ros1 and ros2 nodes through the package ros1_bridge.

## Required packages
You will need the ros1_bridge package, that you can dowload from [**here**](https://github.com/ros2/ros1_bridge.git) and 3 source files in order to set the environments of ros1 and ros2 and also the two environments together.
For ros1 environment you should create a file ros.sh:
```
#!/bin/bash
source /opt/ros/noetic/setup.bash
source /root/your_ros1_ws/devel/setup.bash
```
For ros2 environment you should create a file ros2.sh:
```
#!bin/bash
source /opt/ros/foxy/setup.bash
source /root/your_ros2_ws/install/local_setup.bash
```
For the communication between ros1 and ros2 with the bridge you should create a file ros12.sh:
```
#! /bin/bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
```

### Warnings
Before running the code correctly you should download also the package in the branch main and put it in your ros1 worksapce. Moreover you need to do a little modification that consists on opening the launch file [**sim.launch**](https://github.com/serenapaneri/rt2_assignment1/blob/main/launch/sim.launch) and delete the lines related to the nodes **user_interface.py** and **go_to_point.py".

## How to run the code
In order to run the Gazebo simulation:
```
./sim_start.sh
```
