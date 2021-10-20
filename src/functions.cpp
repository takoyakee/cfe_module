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

float norm(std::vector<int> v1, std::vector<int> v2){
	return sqrt(pow((v1[0] -v2[0]),2) + pow((v1[1] -v2[1]),2));
}

std::string replace(std::string& str,std::string from, std::string to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos){
    	return str;
    }
    str.replace(start_pos, from.length(), to);
    return str;
}

std::string eraseSubStr(std::string mainStr, const std::string toErase)
{
    // Search for the substring in string
    size_t pos = mainStr.find(toErase);
    if (pos != std::string::npos)
    {
        // If found then erase it from string
        mainStr.erase(pos, toErase.length());
    }

    return mainStr;
}
