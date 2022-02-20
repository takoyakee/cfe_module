#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <map>
#include <queue>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base/move_base.h>
#include <navfn/navfn_ros.h>
#include <../../../devel/include/fyp_api/centroidArray.h>
#include <../../../devel/include/fyp_api/centroid.h>
#include <costmap_2d/costmap_2d_ros.h>

float norm(geometry_msgs::PoseStamped ps1, geometry_msgs::PoseStamped ps2);
float norm(std::vector<int> v1, std::vector<int> v2);
std::string replace(std::string& str,std::string from, std::string to);
std::string eraseSubStr(std::string mainStr, const std::string toErase);
void bresenham2D(std::vector<int> start, std::vector<int> end, std::vector<std::vector<int>>& visitedCells);
