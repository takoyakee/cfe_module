/*
 * functions.cpp
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */

#include <fyp/functions.h>

float norm(geometry_msgs::PoseStamped ps1, geometry_msgs::PoseStamped ps2){
	return sqrt(pow((ps1.pose.position.x - ps2.pose.position.x ),2) +
			pow((ps1.pose.position.y - ps2.pose.position.y),2));
}

