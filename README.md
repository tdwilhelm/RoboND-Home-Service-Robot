# RoboND-Home-Service-Robot
Project 7 of Udacity Robotics Software Engineer Nanodegree Program

## Overview 
The goal of this project is for a Robot to autonomously go navigate to a destination and pickup an object. Once the object is picked up it will take it to a drop-off location.
* The project contains scripts to build the map using gmapping SLAM, to localize itself with AMCL, navigate to predetermined goal locations and pick-up and deliver visual markers in Rviz.

## Installation
This simulation have been created and tested in:

* Ubuntu 16.04 (supports Ubuntu 16.04)
* ROS Kinetic (supports ROS kinetic, with melodic has some issues)
* Gazebo 7.0 (supports Gazebo 7.0 or superior)

## Structure  
 
![Demo](catkin.png) 

## Install Packages
* mkdir -p ~/catkin_ws/src
* cd ~/catkin_ws/src
* catkin_init_workspace
* cd ..
* catkin_make
* sudo apt-get update
* cd ~/catkin_ws/src
* git clone https://github.com/ros-perception/slam_gmapping
* git clone https://github.com/turtlebot/turtlebot
* git clone https://github.com/turtlebot/turtlebot_interactions
* git clone https://github.com/turtlebot/turtlebot_simulator
* cd ~/catkin_ws/
* source devel/setup.bash
* rosdep -i install gmapping
* #All required rosdeps installed successfully
* rosdep -i install turtlebot_teleop
* #All required rosdeps installed successfully
* rosdep -i install turtlebot_rviz_launchersAll required rosdeps installed successfully
* rosdep -i install turtlebot_gazebo
* All required rosdeps installed successfully
* catkin_make
* source devel/setup.bash

## Project Steps:
* Design a simple environment with the Building Editor in Gazebo.

* Teleoperate your robot and manually test SLAM.

* Use the ROS navigation stack and manually commands your robot using the 2D Nav Goal arrow in rviz to move to 2 different waypoints for desired positions and orientations.

* Write a pick_objects node that commands your robot to move to the desired pickup and drop off zones.

* Write an add_markers node that subscribes to your robot odometry, keeps track of your robot pose, and publishes markers to rviz.

## Create catkin workspace:
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws catkin_make

## Run the project
* Clone the following repository to catkin_ws/src:
https://github.com/tdwilhelm/RoboND-Home-Service-Robot.git

* cd ~/catkin_ws catkin_make

* Run ./home_service.sh in Scripts directory to deploy the home service robot.
```  
