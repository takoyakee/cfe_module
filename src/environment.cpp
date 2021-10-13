/*
 * environment.cpp
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */
#include "fyp/environment.h"


struct Frontier {
	int x;
	int y;
	unsigned char utility;
	Frontier(int x_,int y_,unsigned char utility_): x(x_),y(y_),utility(utility_){};
};

Environment::Environment(){ // @suppress("Class members should be properly initialized")
	envWidth = 0;
	envHeight = 0;
	maxDist = 0;
	searchRadius = 30; //defined in cells;
	discountMult = 5;
	costMult = 5;
	dweight = 0.5;
	cweight = 0.5;
	expirationTime = 100;
	reserveSize = 10;
	neighbourCells = {{1,1},{1,-1},{-1,1},{-1,-1}};
	bUpdateRobotPose = false;
	bUpdateRobotTeamPoses = false;
	discountGrid = NULL;
	costGrid = NULL;
	frontierGrid = NULL;
	occupancy2D = NULL;
	globalFrame = "/robot_1/map";
}

Environment::~Environment(){
	for(int i = 0; i < envWidth; i++) {
	    delete[] discountGrid[i];
		delete[] costGrid[i];
		//delete[] frontierGrid[i];
	}
	delete[] discountGrid;
	delete[] costGrid;
	//delete[] frontierGrid;

	discountGrid = NULL;
	costGrid = NULL;
	//frontierGrid = NULL;
}

//Vector is sorted accordingly to utility
std::vector<geometry_msgs::PoseStamped> Environment::returnFrontierChoice(){
	getFrontierCells();
	updateDiscountCells();
	updateCostCells();

	std::vector<geometry_msgs::PoseStamped> frontierPose;
    auto cmp = [](Frontier left, Frontier right) { return (left.utility) < (right.utility); };
    std::priority_queue<Frontier, std::vector<Frontier>, decltype(cmp)> pq(cmp);
	for (auto idx : prevFrontierCells){
		unsigned char utility_ = (unsigned char)
				(255 - cweight*costGrid[idx[0]][idx[1]] - dweight*discountGrid[idx[0]][idx[1]]);
		Frontier fcell(idx[0],idx[1],utility_);
		pq.push(fcell);
	}

	ros::Time timenow = ros::Time::now();
	int i = 0;
	//todo: shouldn't discard the value
	while (!pq.empty() &&  i < reserveSize){
		Frontier temp = pq.top();
		pq.pop();
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = temp.x;
		pose.pose.position.y = temp.y;
		pose.header.frame_id = globalFrame;
		pose.header.stamp = timenow;
		frontierPose.push_back(pose);
		i++;
	}

	resetDiscountGrid();
	resetCostGrid();

	return frontierPose;
}


void Environment::Environment::updateOccupancyGrid(nav_msgs::OccupancyGrid occupancyGrid_)
{
	if (occupancyGrid.data.size() == 0){
		initialiseGrids(occupancyGrid_);
		envWidth = occupancyGrid.info.width;
		envHeight = occupancyGrid.info.height;
		maxDist = (int)(sqrt(pow(envWidth,2)+pow(envHeight,2)));
	} else if (occupancyGrid_.data.size() > 0){
		if (occupancyGrid_.info.width != envWidth || occupancyGrid_.info.height != envHeight){
			envWidth = occupancyGrid.info.width;
			envHeight = occupancyGrid.info.height;
			maxDist = (int)(sqrt(pow(envWidth,2)+pow(envHeight,2)));
		}
		occupancyGrid = occupancyGrid_;
	}
}

void Environment::Environment::initialiseGrids(nav_msgs::OccupancyGrid occupancyGrid_){
	occupancyGrid = occupancyGrid_;
	discountGrid = new unsigned char* [envWidth];
	costGrid = new unsigned char* [envWidth];
	//frontierGrid = new char* [envWidth];
	for(int i = 0; i < envWidth; i++) {
	    discountGrid[i] = new unsigned char[envHeight];
		costGrid[i] = new unsigned char[envHeight];
		//frontierGrid[i] = new int[envHeight];
		for (int j=0; j < envHeight; j++){
			discountGrid[i][j] = 0;
			costGrid[i][j] = 0;
			//frontierGrid[i][j] = 0;
		}
	}
	//considering to have a converted occupancyGrid to cut down number of repeated conversion
	occupancy2D[envWidth][envHeight];

}


bool Environment::Environment::updateCostCells(){
	ros::Time poseTime = currentPose.header.stamp;
	ros::Duration timePassed (ros::Time::now() - poseTime);
	if (timePassed.toSec() > expirationTime){
		waitForRobotPose();
		return false;
	}
	//FrontierGrid contains value of different cells (distance, and proximity of other robots)
	//currently based on sensor's occupancyGrid, consider costmap
	std::vector<int> pos = pointToCoord(currentPose.pose.position.x, currentPose.pose.position.y);
	for (auto idx: prevFrontierCells){
		costGrid[idx[0]][idx[1]] = costOfCell(idx[0],idx[1],pos);
	}
	return true;
}

void Environment::Environment::updateDiscountCells(){
	prevDCCells.clear();
	// neighbors pose is in meters
	for (auto robotPose : robotTeamPoses){
		ros::Time poseTime = robotPose.header.stamp;
		ros::Duration timePassed (ros::Time::now() - poseTime);
		if (timePassed.toSec() > expirationTime){
			waitForRobotTeamPoses();
			continue;
		}
		std::vector<int> robotCoord = pointToCoord(robotPose.pose.position.x, robotPose.pose.position.y);
		int xc = robotCoord[0];
		int yc = robotCoord[1];
		//consider first quadrant, can translate after
		for (int x = 0; x <= searchRadius; x++){
			for (int y = 0; y <= x; y++){
				if (x*x + y*y <= searchRadius * searchRadius){
					int dist = x + y; //taking Manhattan distance
					for (std::vector<int> cell : neighbourCells) {
						int x_ = xc+cell[0]*x, y_ = yc+cell[1]*y;
						if (inMap(x_,y_) && occupancyGrid.data[coordToIndex(x_,y_)] == -1){
							discountGrid[x_][y_] = distToDiscount(dist);
							prevDCCells.push_back({x_,y_});
						}
						x_ = xc+cell[0]*y, y_ = yc+cell[1]*x;
						if (inMap(x_,y_) && occupancyGrid.data[coordToIndex(x_,y_)] == -1){
							discountGrid[x_][y_] = distToDiscount(dist);
							prevDCCells.push_back({x_,y_});
						}
					}
				}
			}
		}
	}
}



std::vector<std::vector<int>> Environment::getFrontierCells(){
	prevFrontierCells.clear();
	for (int i; i < envWidth; i++){
		for (int j; j < envHeight; j++){
			if (occupancyGrid.data[coordToIndex(i, j)] == -1){
				prevFrontierCells.push_back({i,j});
			}
		}
	}
	return prevFrontierCells;
}

void Environment::updateRobotPose(geometry_msgs::PoseStamped currentPose_){
	currentPose = currentPose_;
}

void Environment::waitForRobotPose(){
	bUpdateRobotPose = true;
}

void Environment::updateRobotTeamPoses(std::vector<geometry_msgs::PoseStamped> robotTeamPoses_){
	robotTeamPoses = std::move(robotTeamPoses_);
}

void Environment::waitForRobotTeamPoses(){
	bUpdateRobotTeamPoses = true;
}



void Environment::resetCostGrid(){
	for (auto coord : prevFrontierCells){
		costGrid[coord[0]][coord[1]] = 0;
	}
}

void Environment::resetDiscountGrid(){
	for (auto coord : prevDCCells){
		discountGrid[coord[0]][coord[1]] = 0;
	}
}

char Environment::Environment::costOfCell(int cx_, int cy_, std::vector<int> pos_){
	// only interested in unknown cells
	// instead of using euclidean distance, use ros navigation global planner as a plugin
	return 254*(sqrt((pos_[0]-cx_)^2 + (pos_[1]-cy_)^2))/maxDist;
}

//The greater the value, the closer the cell is to neighbouring cells
unsigned char Environment::Environment::distToDiscount(int dist){
	double factor = exp(-1*discountMult * dist);
	return (unsigned char)(255-1)*factor;
}

/*
 * Converts coord (discountGrid/valueGrid) to index (OccupancyGrid)
 */
int Environment::Environment::coordToIndex(int cx_, int cy_){
	return cy_*envWidth + cx_;
}

/*
 * Converts index (Occupancy Grid) to coord (discountGrid/valueGrid)
 */
std::vector<int> Environment::Environment::indexToCoord(int index_){
	return {(int)(index_%envWidth), (int)(floor(index_/envWidth))};
}

/*
 * mx_ : actual x coordinate of point in map frame
 * my_ : actual y coordinate of point in map frame
 * need to be calculated wrt origin
 */
std::vector<int> Environment::Environment::pointToCoord(float mx_, float my_){
	//get index then convert to Coord?
	float originx_ = occupancyGrid.info.origin.position.x;
	float originy_ = occupancyGrid.info.origin.position.y;
	return {(int)(floor((my_-originy_)/occupancyGrid.info.resolution)), (int)(floor((mx_-originx_)/occupancyGrid.info.resolution))};

}

bool Environment::inMap(int cx_, int cy_){
	return (cx_ >= 0 && cx_ <= envWidth && cy_ >= 0 && cy_ <= envHeight);
}

/*

void Environment::hardResetCostGrid(){
	costGrid[envWidth][envHeight];
	int i,j;
	for (i=0; i<envWidth; i++){
		for (j=0; j<envHeight; j++){
			costGrid[i][j] = 0;
		}
	}
}

void Environment::hardResetDiscountGrid(){
	int i,j;
	for (i=0; i<envWidth; i++){
		for (j=0; j<envHeight; j++){
			discountGrid[i][j] = 0;
		}
	}
} */

