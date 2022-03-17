# First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)

### General information
This ROS package contains four nodes and the simulation environment for controlling a mobile robot in a Gazebo simulation environment and in a CoppeliaSim environment.
The general beaviour will be: you have a user interface with which you can directly interact to the robot making it moves pressing 1 and makes it stops pressing any other number.
In particular, when you press 1, and so starting the simulation, the robot starts moving since it is trying to reaching a target position and a target orientation in the space.
When it stops, a new random position is sent to it and it will again try to reach it. 
But please, pay attention to the fact that when you try to stop the simulation pressing whatever number (except 1) the robot would not stop immediately to move, instead it will reach the last target position and orientation, given to it, before stopping.

### Nodes
We have four nodes in total:
1.[**position_service**](https://github.com/serenapaneri/rt2_assignment1/blob/main/src/position_service.cpp): this node is contained in the src folder and it implements a service, it waits for a request from a client and replies with a position x and y and the orientation theta. This is a c++ node.
2.[**state_machine**](https://github.com/serenapaneri/rt2_assignment1/blob/main/src/state_machine.cpp): this node is contained in the src folder and it represents a finite state machine which has only two states that are when the robot starts and when the robot stops. This node may call the position server and after called the position server, so after it receives the target x y and theta position, it will call the navigation service. It also implement a service for interacting with the user interface. This is a c++ node.
3.[**go_to_point**](https://github.com/serenapaneri/rt2_assignment1/blob/main/scripts/go_to_point.py): this node is contained in the script folder and it implements a service to drive a robot toward a point in the environment. It also waits for a client to call the service, the client will send the x, the y and the theta and it will drive the robot towards the required x, y and theta. This node directly interacts with the simulation, indeed it subscribes with the odometry of the robot and it publishes on the cmd_vel of the robot. This is a python node.
4.[**user_interface**](https://github.com/serenapaneri/rt2_assignment1/blob/main/scripts/user_interface.py): this node is contained in the script folder and it asks to the user to start and stop the robot, and it calls a service implemented in the robot finite state machine.This is a python node.

### Custom services in the srv folder
1.[**RandomPosition**](https://github.com/serenapaneri/rt2_assignment1/blob/main/srv/RandomPosition.srv): it expects a x_max, x_min, y_max and y_min and replies with an x, y and theta. This is the service message used by the random position service. 
2.[**Position**](https://github.com/serenapaneri/rt2_assignment1/blob/main/srv/Position.srv): it is the message sent to the go_to_point service. 
3.[**Command**](https://github.com/serenapaneri/rt2_assignment1/blob/main/srv/Command.srv): it is just a string command which will be start or stop. 

### Urdf folder
This folder contains a file in which you find the description of the robot that will be spawned in the Gazebo simulation environemnt.

### Launch file
There are two launch files contained in the folder launch: 
- One of them is [**sim.launch**](https://github.com/serenapaneri/rt2_assignment1/blob/main/launch/sim.launch) that will just spawn the robot in the Gazebo simulator and then it launches all the four nodes. 
- The second one which is the [**coppelia.launch**]() will launch the four nodes together that will be used to communicate with the CoppeliaSim simulation.

## CoppeliaSim
In the CoppeliaSim simulation [**Pioneer_CoppeliaSim.ttt**]() the robot used is the Pioneer_p3dx. This simulation is supposed to interract with the ROS nodes, in particular within the coppeliasim simulation we publish the robot position on the topic /odom and we receive the velocity through the topic /cmd_vel.

## Required packages
For the coppeliasim simulation you need the **Coppeliasim** and the **SimExtROS** packages.

## How to run the code
In order to run the coppeliasim simulation (required by the assignment) you have first to open a terminal and source it as a ROS environment, and then you need to run the roscore.
Then you need to open another terminal in which you will open the coppeliasim environment, and after done that, you need to load the coppeliasim scene and then start the simulation by pressing the play button.
Finally you need to go back in the first terminal and run:
```
roslaunch rt2_assignment1 coppeliasim.launch
```
