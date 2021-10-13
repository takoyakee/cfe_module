#!/bin/bash

#publishes to move_base_goal to test if robots can move
gnome-terminal --tab -- bash -c "rostopic pub -1 /robot_1/move_base_simple/goal geometry_msgs/PoseStamped '{header : {stamp: now, frame_id: \"robot_1/map\"}, pose: {position: {x: 8, y: 2, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}'" 
gnome-terminal --tab -- bash -c "rostopic pub -1 /robot_2/move_base_simple/goal geometry_msgs/PoseStamped '{header : {stamp: now, frame_id: \"robot_2/map\"}, pose: {position: {x: 5, y: 5, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}'" 

