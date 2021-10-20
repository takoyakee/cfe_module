#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <map>

float norm(geometry_msgs::PoseStamped ps1, geometry_msgs::PoseStamped ps2);
float norm(std::vector<int> v1, std::vector<int> v2);
std::string replace(std::string& str,std::string from, std::string to);
std::string eraseSubStr(std::string mainStr, const std::string toErase);
