# Part 1 - AGV Movement Simulation
This is an assignment for my Industrial Automation course. The detials to recreate this project is at below.

## Prerequisite
This project requires
* [ROS](http://wiki.ros.org/) (I have tested on [ROS Noetic](http://wiki.ros.org/noetic/Installation) and [ROS Kinetic](http://wiki.ros.org/kinetic/Installation))
* [Gazebo](http://www.gazebosim.org/tutorials?tut=install_ubuntu) (The combination of ROS/Gazebo version is shown in [here](http://gazebosim.org/tutorials/?tut=ros_wrapper_versions)
* [gazebo-ros-pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros) (install according to ROS version)
* [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) package

The links for the detail of [setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) and [simulation with Gazebo](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

## Pyhton Script for ROS
The [move_to_coordinate.py](https://github.com/ccxuan123/agv_assignment/blob/main/nodes/move_to_coordinate.py) is the main Python script for the turtlebot to move.
[move_to_coordinate.py](https://github.com/ccxuan123/agv_assignment/blob/main/nodes/move_to_coordinate.py) originally is write for python3, if using python2 edit the 1st line to 
```
#!/usr/bin/env python
```
## Import the project package
Clone this main branch repo to ~/catkin_ws/src/ by:
```
$ cd ~/catkin_ws/src/
$ git clone -b main https://github.com/ccxuan123/agv_assignment.git
$ cd ~/catkin_ws && catkin_make
```

## Import model files
The model files is located in model branch or you can find it [here](https://github.com/ccxuan123/agv_assignment/tree/model).
Import the 'square_wall_5x5' folder under this directory:
```
home/user/.gazebo/models/
```
You can download the model branch as zip file and extract to the directory or manually create and copy the model config files. Once imported, your model files location should be like this
```
home/user/.gazebo/models/square_wall_5x5/model.config
home/user/.gazebo/models/square_wall_5x5/model.sdf
```

## Start the simulation
To initiate the simulation on Gazebo by run the command below in terminal
```
$ roslaunch agv_assignment agv_world.launch
```
Then open another terminal and run the command below to start movement simulation
```
$ roslaunch agv_assignment move_to_coordinate.launch
```

## About the simulation program
A matrix of the map will shown in the terminal. The program only allow the user to select 2 point fo the AGV to travel before returning to the starting position which represent its charging station.
