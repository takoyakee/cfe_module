/*
 * functions.cpp
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */

#include <fyp_api/functions.h>

float norm(geometry_msgs::PoseStamped ps1, geometry_msgs::PoseStamped ps2){
	return sqrt(pow((ps1.pose.position.x - ps2.pose.position.x ),2) +
			pow((ps1.pose.position.y - ps2.pose.position.y),2));
}

float norm(std::vector<float> v1, std::vector<float> v2){
	return sqrt(pow((v1[0] -v2[0]),2) + pow((v1[1] -v2[1]),2));
}

float norm(std::vector<float> v1, geometry_msgs::PoseStamped ps2){
	return sqrt(pow((v1[0] -ps2.pose.position.x),2)+pow((v1[1] -ps2.pose.position.y),2));
}


float norm(std::vector<int> v1, std::vector<int> v2){
	return sqrt(pow((v1[0] -v2[0]),2) + pow((v1[1] -v2[1]),2));
}

float norm(geometry_msgs::Point pt1, geometry_msgs::Point pt2){
	return sqrt(pow((pt1.x - pt2.x ),2) + pow((pt1.y - pt2.y),2));
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

void bresenham2D(std::vector<int> start, std::vector<int> end, std::vector<std::vector<int>>& visitedCells)
{
	if (start == end){
		return;
	}

    int x1,x2, y1,y2;
    x1 = std::min(start[0],end[0]);
    x2 = std::max(start[0],end[0]);
	y1 = std::min(start[1],end[1]);
	y2 = std::max(start[1],end[1]);

    if (start[0] == end [0]){
    	//vertical
    	for (int j = y1; j < y2+1; j++){
    		visitedCells.push_back({start[0],j});
    	}

    }
    else if (start[1] == end[1]){
    	//horizontal
    	for (int i = x1; i < x2+1; i++){
    		visitedCells.push_back({i, start[1]});
    	}
    }
    else {
    	//First and third quadrant
    	if ((start[0] == x1 && start[1] == y1) || (start[0] == x2 && start[1] == y2)){
			//calculating range for line between start and end point
			int dx = x2 - x1;
			int dy = y2 - y1;
			int x = x1;
			int y = y1;

			// gradient < 1
			if(dx > dy)
			{
				visitedCells.push_back({x,y});
				int pk = (2 * dy) - dx;
				for(int i = 0; i < dx ; i++)
				{
					x = x + 1;
					if (pk < 0)
					{
						pk = pk + (2 * dy);
					} else
					{
						y = y + 1;
						pk = pk + (2 * dy) - (2 * dx);
					}
					visitedCells.push_back({x,y});
				}
			}
			// gradient > 1
			else
			{
				visitedCells.push_back({x,y});
				int pk = (2 * dx) - dy;
				for(int i = 0; i < dy ; i++)
				{
					y = y + 1;
					if(pk < 0) {
						pk = pk + (2 * dx);

					} else
					{
						x = x + 1;
						pk = pk + (2 * dx) - (2 *dy);
					}
					visitedCells.push_back({x,y});
				}
			 }

    	}
    	else {
    		int swp = y2;
    		y2 = y1;
    		y1 = swp;

			int dx = x2 - x1;
			int dy = y1 - y2;
			int x = x1;
			int y = y1;

			// gradient < 1
			if(dx > dy)
			{
				visitedCells.push_back({x,y});
				int pk = (2 * dy) - dx;
				for(int i = 0; i < dx ; i++)
				{
					x = x + 1;
					if (pk < 0)
					{
						pk = pk + (2 * dy);
					} else
					{
						y = y - 1;
						pk = pk + (2 * dy) - (2 * dx);
					}
					visitedCells.push_back({x,y});

				}
			} else {

				visitedCells.push_back({x,y});
				int pk = (2 * dx) - dy;
				for(int i = 0; i < dy ; i++)
				{
					y = y - 1;
					if(pk < 0) {
						pk = pk + (2 * dx);

					} else
					{
						x = x + 1;
						pk = pk + (2 * dx) - (2 *dy);
					}
					visitedCells.push_back({x,y});

				}
			}
    	}
    }


}


