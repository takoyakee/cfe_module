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
	expirationTime = 10000;
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
	resetGrids();
}

//Vector is sorted accordingly to utility
Environment::Frontier Environment::returnFrontierChoice(){
	ROS_INFO("Returning frontier choice...");
	getFrontierCells();
	ROS_INFO("Frontier Cells received");
	bool discounted = updateDiscountCells();
	ROS_INFO("Discounted cells");
	bool costed = updateCostCells();
	ROS_INFO("Costed cells");

	if (!discounted || !costed){
		ROS_INFO("Discount: %d, Cost: %d", discounted, costed);
		return Frontier(0,0,0);
	}

    auto cmp = [](Frontier left, Frontier right) { return (left.utility) < (right.utility); };
    std::priority_queue<Frontier, std::vector<Frontier>, decltype(cmp)> pq(cmp);
	for (auto idx : prevFrontierCells){
		//ROS_INFO("Frontier at (%d,%d)", idx[0],idx[1]);
		unsigned char utility_ = evaluateUtility(idx[0],idx[1]);
		Frontier fcell(idx[0],idx[1],utility_);
		pq.push(fcell);
	}

	ROS_INFO("Finished finding candidate");

	ros::Time timenow = ros::Time::now();
	//todo: shouldn't discard the value
	Frontier bestFrontier = pq.top();
	ROS_INFO("Candidate is at (%d, %d) with utility (%d)", bestFrontier.x, bestFrontier.y, bestFrontier.utility);
	pq.pop();
	std::vector<float> frontierPosition = coordToPoint(bestFrontier.x, bestFrontier.y);
	ROS_INFO("In world frame: %f, %f", frontierPosition[0], frontierPosition[1]);

	bestFrontier.pose.pose.position.x = frontierPosition[0];
	bestFrontier.pose.pose.position.y = frontierPosition[1];
	bestFrontier.pose.pose.orientation.w = 1;
	bestFrontier.pose.header.frame_id = globalFrame;
	bestFrontier.pose.header.stamp = timenow;

	resetDiscountGrid();
	resetCostGrid();

	return bestFrontier;
}


void Environment::Environment::updateOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_)
{
	//ROS_INFO("Original Occupancy grid size: %d", occupancyGrid.data.size());
	if (occupancyGrid.data.size() == 0){
		occupancyGrid = *occupancyGrid_;
		envWidth = occupancyGrid.info.width;
		envHeight = occupancyGrid.info.height;
		maxDist = (int)(sqrt(pow(envWidth,2)+pow(envHeight,2)));
		initialiseGrids();
	} else if (occupancyGrid_->data.size() != occupancyGrid.data.size()){
		ROS_INFO("Grids resetted!");
		occupancyGrid = *occupancyGrid_;
		envWidth = occupancyGrid.info.width;
		envHeight = occupancyGrid.info.height;
		maxDist = (int)(sqrt(pow(envWidth,2)+pow(envHeight,2)));
		resetGrids();
		initialiseGrids();
	}
	ROS_INFO("max dist: %d", maxDist);
}

void Environment::Environment::initialiseGrids(){
	ROS_INFO("Initialising grids of (%d x %d)", envWidth, envHeight);
	discountGrid = new double* [envWidth];
	costGrid = new unsigned char* [envWidth];
	//frontierGrid = new char* [envWidth];
	for(int i = 0; i < envWidth; i++) {
	    discountGrid[i] = new double[envHeight];
		costGrid[i] = new unsigned char[envHeight];
		//frontierGrid[i] = new int[envHeight];
		for (int j=0; j < envHeight; j++){
			discountGrid[i][j] = 0;
			costGrid[i][j] = 0;
			//frontierGrid[i][j] = 0;
		}
	}
}

void Environment::resetGrids(){
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


bool Environment::Environment::updateCostCells(){
	ros::Time poseTime = currentPose.header.stamp;
	//sensitive to simtime
	ros::Duration timePassed = ros::Time::now() - poseTime ;
	//ROS_INFO("Current time: %f, PoseTime: %f, timePassed: %f", ros::Time::now().toSec(), poseTime.toSec(), timePassed.toSec());
	if (timePassed.toSec() > expirationTime){
		waitForRobotPose();
		return false;
	}
	//FrontierGrid contains value of different cells (distance, and proximity of other robots)
	//currently based on sensor's occupancyGrid, consider costmap
	std::vector<int> pos = pointToCoord(currentPose.pose.position.x, currentPose.pose.position.y);
	ROS_INFO("Current position (%d,%d)", pos[0],pos[1]);
	for (auto idx: prevFrontierCells){
		costGrid[idx[0]][idx[1]] = costOfCell(idx[0],idx[1],pos);
	}
	return true;
}

bool Environment::Environment::updateDiscountCells(){
	prevDCCells.clear();
	if (robotTeamPoses.size() == 0){
		ROS_INFO("No Team Received.");
		return true;
	} else {
		// neighbors pose is in meters
		for (auto robotPose : robotTeamPoses){
			ros::Time poseTime = robotPose.header.stamp;
			ros::Duration timePassed = ros::Time::now() - poseTime;
			if (timePassed.toSec() > expirationTime){
				waitForRobotTeamPoses();
				return false;
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
			return true;
		}
	}

}

unsigned char Environment::evaluateUtility(int cx_, int cy_){
	//ERROR! Recalculate utility cells!!
	ROS_INFO("(%d,%d), cost: %d, dc: %d, utility: %d", cx_, cy_, costGrid[cx_][cy_],254*discountGrid[cx_][cy_],
			MAX(0,(255 - (int) cweight*costGrid[cx_][cy_] - dweight*254*discountGrid[cx_][cy_])));

	return (unsigned char) MAX(0,(255 - cweight*costGrid[cx_][cy_] - dweight*254*discountGrid[cx_][cy_]));

}

bool Environment::isEnvironmentInitialised(){
	return !bUpdateRobotPose && !bUpdateRobotTeamPoses && (occupancyGrid.data.size()>0);
}

std::vector<std::vector<int>> Environment::getFrontierCells(){
	prevFrontierCells.clear();
	for (int i = 0; i < envWidth; i++){
		for (int j = 0; j < envHeight; j++){
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

unsigned char Environment::Environment::costOfCell(int cx_, int cy_, std::vector<int> pos_){
	// only interested in unknown cells, need to convert sqrt to int
	// instead of using euclidean distance, use ros navigation global planner as a plugin
	//ROS_INFO("cx: %d, cy:%d, cost: %d", cx_, cy_, (unsigned char) 254* (int) sqrt(((pos_[0]-cx_)*(pos_[0]-cx_) + (pos_[1]-cy_)*(pos_[1]-cy_)))/maxDist);
	return (unsigned char) 254* (int) sqrt(((pos_[0]-cx_)*(pos_[0]-cx_) + (pos_[1]-cy_)*(pos_[1]-cy_)))/maxDist;
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

// resolution 0.1m/cell
std::vector<float> Environment::coordToPoint(int cx_, int cy_){
	float originx_ = occupancyGrid.info.origin.position.x;
	float originy_ = occupancyGrid.info.origin.position.y;
	return {originx_+cx_*occupancyGrid.info.resolution, originy_+cy_*occupancyGrid.info.resolution};
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
	return {(int)(floor((mx_-originx_)/occupancyGrid.info.resolution)),(int)(floor((my_-originy_)/occupancyGrid.info.resolution))};

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

