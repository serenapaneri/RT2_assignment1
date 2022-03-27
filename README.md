# First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)

### General information
This ROS package contains four nodes and the simulation environment for controlling a mobile robot in a Gazebo simulation environment.
The general beaviour will be: you have a user interface with which you can directly interact to the robot making it moves pressing 1 and makes it stops pressing any other number.
In particular, when you press 1, and so starting the simulation, the robot starts moving since it is trying to reaching a target position and a target orientation in the space.
When it stops, a new random position is sent to it and it will again try to reach it. 
But please, pay attention to the fact that when you try to stop the simulation pressing whatever number (except 1) the robot now would stop immediately to move, differently from what happens in the other two branches.

### Nodes
We have four nodes in total:
1. [**position_service**](https://github.com/serenapaneri/rt2_assignment1/blob/action/src/position_service.cpp): this node is contained in the src folder and it implements a service, it waits for a request from a client and replies with a position x and y and the orientation theta. This is a c++ node.
2. [**state_machine**](https://github.com/serenapaneri/rt2_assignment1/blob/action/src/state_machine.cpp): this node is contained in the src folder and it represents a finite state machine which has only two states that are when the robot starts and when the robot stops. This node may call the position server and after called the position server, so after it receives the target x y and theta position, it will call the navigation service. It also implement a service for interacting with the user interface. In this node an action client has been implemented. This is a c++ node.
3. [**go_to_point**](https://github.com/serenapaneri/rt2_assignment1/blob/action/scripts/go_to_point.py): this node is contained in the script folder and it implements a service to drive a robot toward a point in the environment. It also waits for a client to call the service, the client will send the x, the y and the theta and it will drive the robot towards the required x, y and theta. This node directly interacts with the simulation, indeed it subscribes with the odometry of the robot and it publishes on the cmd_vel of the robot. In this node an action server has been implemented. In this way, since actions are not atomic, we can for instance solve the problem related to the fact that when we ask to the robot to stop, this can be done immediately, indeed with actions we are able to cancel goals. This is a python node.
4. [**user_interface**](https://github.com/serenapaneri/rt2_assignment1/blob/action/scripts/user_interface.py): this node is contained in the script folder and it asks to the user to start and stop the robot, and it calls a service implemented in the robot finite state machine. Also in this node an action client has been implememented, in order to achieve the stopping beavior of the robot, when requested. This is a python node.

### Custom services in the srv folder
1. [**RandomPosition**](https://github.com/serenapaneri/rt2_assignment1/blob/action/srv/RandomPosition.srv): it expects a x_max, x_min, y_max and y_min and replies with an x, y and theta. This is the service message used by the random position service. 
2. [**Position**](https://github.com/serenapaneri/rt2_assignment1/blob/action/srv/Position.srv): it is the message sent to the go_to_point service. 
3. [**Command**](https://github.com/serenapaneri/rt2_assignment1/blob/action/srv/Command.srv): it is just a string command which will be start or stop. 


### Action
In the action folder we can find the [**Target**](https://github.com/serenapaneri/rt2_assignment1/blob/action/action/Target.action) action. In this file we have the goal definition which corresponds to the x, y and theta of the target to reach, then we have the result which is a simple boolean value and finally we have the feedback that consists of a string about the status of that action.

### Urdf folder
This folder contains a file in which you find the description of the robot that will be spawned in the Gazebo simulation environemnt.

### Launch file
There is only one launch file whcih is the [**sim.launch**](https://github.com/serenapaneri/rt2_assignment1/blob/action/launch/sim.launch) that will just spawn the robot in the Gazebo simulator and then it launches all the four nodes. 

## Required packages
For this simulation you need the **Actionlib** package.

## How to run the code
In order to run the gazebo simulation you have first to open the terminal and source it as a ROS environment, then you need to run the roscore, and then run the launch file:
```
roslaunch rt2_assignment1 sim.launch
```
