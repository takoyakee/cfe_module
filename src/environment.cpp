/*
 * environment.cpp
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */
#include "fyp/environment.h"



Environment::Environment(){ // @suppress("Class members should be properly initialized")
	envWidth = 0;
	envHeight = 0;
	maxDist = 0;
	searchRadius = 30; //defined in cells, should be references to actual laser range
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
Environment::Frontier Environment::returnFrontierChoice(){
	getFrontierCells();
	updateDiscountCells();
	updateCostCells();

    auto cmp = [](Frontier left, Frontier right) { return (left.utility) < (right.utility); };
    std::priority_queue<Frontier, std::vector<Frontier>, decltype(cmp)> pq(cmp);
	for (auto idx : prevFrontierCells){
		unsigned char utility_ = evaluateUtility(idx[0],idx[1]);
		Frontier fcell(idx[0],idx[1],utility_);
		pq.push(fcell);
	}

	ros::Time timenow = ros::Time::now();
	//todo: shouldn't discard the value
	Frontier bestFrontier = pq.top();
	pq.pop();
	bestFrontier.pose.pose.position.x = bestFrontier.x;
	bestFrontier.pose.pose.position.y = bestFrontier.y;
	bestFrontier.pose.header.frame_id = globalFrame;
	bestFrontier.pose.header.stamp = timenow;

	resetDiscountGrid();
	resetCostGrid();

	return bestFrontier;
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
	discountGrid = new unsigned double* [envWidth];
	costGrid = new unsigned char* [envWidth];
	//frontierGrid = new char* [envWidth];
	for(int i = 0; i < envWidth; i++) {
	    discountGrid[i] = new unsigned double[envHeight];
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
							if (discountGrid[x_][y_] == 0){
								discountGrid[x_][y_] = distToDiscount(dist);
							} else {
								//inspired by alpha blending kekek
								discountGrid[x_][y_] = discountGrid[x_][y_]  + (1-discountGrid[x_][y_])*distToDiscount(dist);
							}
							prevDCCells.push_back({x_,y_});
						}
						x_ = xc+cell[0]*y, y_ = yc+cell[1]*x;
						if (inMap(x_,y_) && occupancyGrid.data[coordToIndex(x_,y_)] == -1){
							if (discountGrid[x_][y_] == 0){
								discountGrid[x_][y_] = distToDiscount(dist);
							} else {
								discountGrid[x_][y_] = discountGrid[x_][y_]  + (1-discountGrid[x_][y_])*distToDiscount(dist);
							}
							prevDCCells.push_back({x_,y_});
						}
					}
				}
			}
		}
	}

}

unsigned char Environment::evaluateUtility(int cx_, int cy_){
	return (unsigned char) MAX(0,(255 - cweight*costGrid[cx_][cy_] - dweight*254*discountGrid[cx_][cy_]));

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

void Environment::updateRobotTeamPoses(std::map<std::string, geometry_msgs::PoseStamped> robotTeamPoses_){
	std::vector<geometry_msgs::PoseStamped> poseVector;
	for(auto& pose: robotTeamPoses_) {
		poseVector.push_back(pose.second);
	}
	robotTeamPoses = std::move(poseVector);
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
double Environment::Environment::distToDiscount(int dist){
	//return exp(-1*discountMult * dist);
	return dist/searchRadius ;

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

Environment::Frontier::Frontier(int x_,int y_,unsigned char utility_){
	x = x_;
	y = x_;
	utility = utility_;
}

Environment::Frontier& Environment::Frontier::operator = (const Frontier& f1){
	x = f1.x;
	y = f1.y;
	utility = f1.utility;
	pose = f1.pose;
	return *this;
};


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

