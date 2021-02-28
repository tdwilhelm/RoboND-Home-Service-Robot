#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=`rospack find home_serivce_robot`/worlds/corridor.world " &
sleep 3
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=`rospack find home_serivce_robot`/map/myMapcorridor.yaml " &
sleep 3
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 3