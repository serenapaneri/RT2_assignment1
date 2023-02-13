# Second Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)

### General information
This ROS package contains three nodes, the simulation environment and the jupyter notebbok for controlling a mobile robot in a Gazebo simulation environment.
The general beaviour will be: you have a user interface, created with a Jupyter notebook with which you can directly interact to the robot making it moves and stops by pressing the start and the stop button, you can control the robot linear and angular velocity trough the sliders, you can directly teleoperate the robot using four buttons with the different direnction and in the meanwhile you can check the behavior of the robot trought graphs about the given and the real velocity, about how many targets have been reached or cancelled, about the time required to reach a target and about the position and orientation in the space.


### Nodes
We have three nodes in total and a Jupyter notebook:
1. [**position_service**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/src/position_service.cpp): this node is contained in the src folder and it implements a service, it waits for a request from a client and replies with a position x and y and the orientation theta. This is a c++ node.
2. [**state_machine**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/src/state_machine.cpp): this node is contained in the src folder and it represents a finite state machine which has only two states that are when the robot starts and when the robot stops. This node may call the position server and after called the position server, so after it receives the target x y and theta position, it will call the navigation service. It also implement a service for interacting with the user interface. In this node an action client has been implemented. This is a c++ node.
3. [**go_to_point**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/scripts/go_to_point.py): this node is contained in the script folder and it implements a service to drive a robot toward a point in the environment. It also waits for a client to call the service, the client will send the x, the y and the theta and it will drive the robot towards the required x, y and theta. This node directly interacts with the simulation, indeed it subscribes with the odometry of the robot and it publishes on the cmd_vel of the robot. In this node an action server has been implemented. In this way, since actions are not atomic, we can for instance solve the problem related to the fact that when we ask to the robot to stop, this can be done immediately, indeed with actions we are able to cancel goals. This is a python node.
4. [**user_interface**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/notebook/user_interface.ipybn): this node implementedy via Jupyter notebook allows you to directly interact with the robot and also displaying graphs of its behavior.

### Custom services in the srv folder
1. [**RandomPosition**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/srv/RandomPosition.srv): it expects a x_max, x_min, y_max and y_min and replies with an x, y and theta. This is the service message used by the random position service. 
2. [**Position**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/srv/Position.srv): it is the message sent to the go_to_point service. 
3. [**Command**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/srv/Command.srv): it is just a string command which will be start or stop. 


### Action
In the action folder we can find the [**Target**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/action/Target.action) action. In this file we have the goal definition which corresponds to the x, y and theta of the target to reach, then we have the result which is a simple boolean value and finally we have the feedback that consists of a string about the status of that action.

### Urdf folder
This folder contains a file in which you find the description of the robot that will be spawned in the Gazebo simulation environemnt.

### Launch file
There is only one launch file whcih is the [**notebook.launch**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/launch/sim.launch) that will just spawn the robot in the Gazebo simulator and then it launches all the four nodes. 

## Required packages
For this simulation you need the **Actionlib** package.

## How to run the code
In order to run the gazebo simulation you have first to open the terminal and source it as a ROS environment, then you need to run the roscore, and then run the launch file:
```
roslaunch rt2_assignment1 notebook.launch
```
Then to control the robot, in another terminal you have to type:
```
jupyter notebook --allow-root --ip 0.0.0.0
```
Open the **user_interface.ipybn** in the [**notebook**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinix/notebook) folder and start to interact with the robot. 
All the explanation are inside the **user_interface.ipybn** file.

## Sphinx documentation
The sphinx documentation interface can be seen by open with your browser the file [**index.html**](https://github.com/serenapaneri/rt2_assignment1/blob/sphinx/_build/html/index.html) in the **_build** folder.
