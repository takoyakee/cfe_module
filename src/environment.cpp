/*
 * environment.cpp
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */
#include "fyp/environment.h"

// should create a function that listens to rosparam!
Environment::Environment(){ // @suppress("Class members should be properly initialized")
	envWidth = 0;
	envHeight = 0;
	maxDist = 0;
	searchRadius = 50; //defined in cells, should be references to actual laser range
	discountMult = 5;
	costMult = 5;
	dweight = 0.7;
	cweight = 0.5;
	frontierThreshold = 4;
	expirationTime = 10000;
	reserveSize = 10;
	circleCorners = {{1,1},{1,-1},{-1,1},{-1,-1}};
	kernel = { 	{1,{-1,1}}, {2,{0,1}}, {3,{1,1}},
				{4,{-1,0}}, 		   {6,{1,0}},
				{7,{-1,-1}}, {8,{0,-1}}, {9,{1,-1}} };
	bUpdateRobotPose = true;
	bUpdateTeamGoalPose = true;
	bUpdate = true;
	discountGrid = NULL;
	costGrid = NULL;
	frontierGrid = NULL;
	occupancy2D = NULL;
	offx = 0.0f;
	offy = 0.0f;
}

Environment::~Environment(){
	deleteGrids();
}

/*
 *  main function that computes costs and discount and returns optimal frontier
 */
std::priority_queue<Environment::Frontier> Environment::returnFrontiers(){
	frontierCells= getFrontierCells();
	fCentroids = processFrontierCells(frontierCells);
	bool discounted = updateDiscountCells();
	bool costed = updateCostCells();

	//Evaluating Utility...
    std::priority_queue<Environment::Frontier> pq;

	if (discounted && costed){
		for (auto idx : fCentroids){
			unsigned char utility_ = evaluateUtility(idx[0],idx[1]);
			Frontier fcell(idx[0],idx[1],utility_);
			fcell.pose = coordToPS(idx[0], idx[1]);
			pq.push(fcell);
			//ROS_INFO("Frontier (%d,%d) with utility: %d",idx[0],idx[1],utility_);
		}
	}

	resetGrids();
	frontierPQ = pq;
	return pq;

}


void Environment::Environment::updateOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_)
{
	if (occupancyGrid.data.size() == 0){
		ROS_INFO("INITIALISING OCCUPANCY GRID...");
		frontiers_pub = nh_.advertise<visualization_msgs::Marker>(robotName + "_frontier", 500);
		centroid_pub = nh_.advertise<visualization_msgs::Marker>( robotName + "_centroids", 500);
		occupancyGrid = *occupancyGrid_;
		envWidth = occupancyGrid.info.width;
		envHeight = occupancyGrid.info.height;
		maxDist = (int)(sqrt(pow(envWidth,2)+pow(envHeight,2)));
		initialiseGrids();
	} else if (occupancyGrid_->data.size() != occupancyGrid.data.size()){
		ROS_INFO("**************OCCUPANCY GRID RESET**********");
		deleteGrids();
		occupancyGrid = *occupancyGrid_;
		envWidth = occupancyGrid.info.width;
		envHeight = occupancyGrid.info.height;
		maxDist = (int)(sqrt(pow(envWidth,2)+pow(envHeight,2)));
		initialiseGrids();
	} else {
		ROS_INFO("**********REPLACED INFO***********");
		occupancyGrid = *occupancyGrid_;
	}
}


//Due to offset between merge map and global frame
void Environment::updateMapOffSet(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_){
	offx = occupancyGrid_->info.origin.position.x - occupancyGrid.info.origin.position.x ;
	offy = occupancyGrid_->info.origin.position.y - occupancyGrid.info.origin.position.y ;
}

void Environment::Environment::initialiseGrids(){
	ROS_INFO("Initialising grids of (%d x %d)", envWidth, envHeight);
	discountGrid = new double* [envWidth];
	informationGrid = new double* [envWidth];
	costGrid = new unsigned char* [envWidth];
	frontierGrid = new int* [envWidth];
	for(int i = 0; i < envWidth; i++) {
	    discountGrid[i] = new double[envHeight];
		informationGrid[i] = new double[envHeight];
		costGrid[i] = new unsigned char[envHeight];
		frontierGrid[i] = new int[envHeight];
		for (int j=0; j < envHeight; j++){
			informationGrid[i][j] = 0;
			discountGrid[i][j] = 0;
			costGrid[i][j] = 0;
			frontierGrid[i][j] = -1;
		}
	}
}

void Environment::deleteGrids(){
	for(int i = 0; i < envWidth; i++) {
	    delete[] discountGrid[i];
	    delete[] informationGrid[i];
		delete[] costGrid[i];
		delete[] frontierGrid[i];
	}
	delete[] discountGrid;
	delete[] informationGrid;
	delete[] costGrid;
	delete[] frontierGrid;

	discountGrid = NULL;
	informationGrid = NULL;
	costGrid = NULL;
	frontierGrid = NULL;
}


bool Environment::Environment::updateCostCells(){
	ros::Time poseTime = currentPose.header.stamp;
	ros::Duration timePassed = ros::Time::now() - poseTime ;
	ros::Time rosTime = ros::Time(0);
	if (timePassed.toSec() > expirationTime){
		waitForRobotPose();
		return false;
	}
	std::vector<int> currPos = pointToCoord(currentPose.pose.position.x, currentPose.pose.position.y);

	for (auto idx: fCentroids){
		costGrid[idx[0]][idx[1]] = (unsigned char) (costOfCell(idx,currPos));
/*
		unsigned int costs = 0;
		//obtain all costs to cell
		for (auto rPose : teamPose){
			std::vector<int> pos = pointToCoord(rPose.pose.position.x, rPose.pose.position.y);
			costs += costOfCell(idx,pos);
		}
		if (costs>0){
			costs = costs/teamPose.size();
		}
		//costGrid[idx[0]][idx[1]] = (unsigned char) MAX(0,(2*costOfCell(idx,currPos) - costs));
		ROS_INFO("Cost: %d", costGrid[idx[0]][idx[1]]);
		ROS_INFO("Modified: %d", (unsigned char) MAX(0,(2*costOfCell(idx,currPos) - costs)));*/
	}
	return true;
}


bool Environment::Environment::updateDiscountCells(){
	if (teamGoalPose.size() == 0 && teamPose.size() == 0){
		ROS_INFO("No info on other robots");
		return true;

	}
	ROS_INFO("TEAM GOAL SIZE: %d", teamGoalPose.size());
	ROS_INFO("TEAM POSE SIZE: %d", teamPose.size());

	//Discount frontiers that are in other robots range
	if (teamGoalPose.size() > 0){
		for (auto rGoal : teamGoalPose){
			ros::Time poseTime = rGoal.header.stamp;
			ros::Duration timePassed = ros::Time::now() - poseTime;
			if (timePassed.toSec() > expirationTime){
				waitForTeamGoalPose();
				return false;
			}
			std::vector<int> pos = pointToCoord(rGoal.pose.position.x, rGoal.pose.position.y);
			for (auto fC: fCentroids){
				float dist = (pos[0]-fC[0])*(pos[0]-fC[0]) + (pos[1]-fC[1])*(pos[1]-fC[1]);
				if( dist<= searchRadius * searchRadius){
					dist = sqrt(dist);
					if (discountGrid[fC[0]][fC[1]] == 0){
						discountGrid[fC[0]][fC[1]] = distToDiscount(dist);
					} else {
						discountGrid[fC[0]][fC[1]] = discountGrid[fC[0]][fC[1]]  + (1-discountGrid[fC[0]][fC[1]])*distToDiscount(dist);
					}
				}
				//ROS_INFO("GOAL DC: %f", discountGrid[fC[0]][fC[1]]);
			}
		}
	}
	// Discount frontiers near other robots current pose
	if (teamPose.size() > 0) {
		for (auto rPose : teamPose){
			ros::Time poseTime = rPose.header.stamp;
			ros::Duration timePassed = ros::Time::now() - poseTime;
			if (timePassed.toSec() > expirationTime){
				waitForTeamGoalPose();
				return false;
			}
			std::vector<int> pos = pointToCoord(rPose.pose.position.x, rPose.pose.position.y);
			for (auto fC: fCentroids){
				float dist = (pos[0]-fC[0])*(pos[0]-fC[0]) + (pos[1]-fC[1])*(pos[1]-fC[1]);
				if( dist<= searchRadius * searchRadius){
					dist = sqrt(dist);
					if (discountGrid[fC[0]][fC[1]] == 0){
						discountGrid[fC[0]][fC[1]] = distToDiscount(dist);
					} else {
						discountGrid[fC[0]][fC[1]] = discountGrid[fC[0]][fC[1]]  + (1-discountGrid[fC[0]][fC[1]])*distToDiscount(dist);
					}
				}
				//ROS_INFO("POSE DC: %f", discountGrid[fC[0]][fC[1]]);
			}
		}
	}
	return true;
}


bool Environment::updateIGCells(){
	return true;
}


unsigned char Environment::evaluateUtility(int cx_, int cy_){
	//ROS_INFO("Discount: %f",dweight*254*discountGrid[cx_][cy_]);
	return (unsigned char) (MAX(0,(255 - cweight*costGrid[cx_][cy_] - dweight*254*discountGrid[cx_][cy_])));

}

std::vector<std::vector<int>> Environment::getFrontierCells(){
	std::vector<std::vector<int>> prevFrontierCells;
	int frontierCount = 0;	//gives position of coord in prevFrontierCells
	for (int i = 0; i < envWidth; i++){
		for (int j = 0; j < envHeight; j++){
			if (occupancyGrid.data[coordToIndex(i, j)] == -1 && isFrontier(i,j)){
				frontierGrid[i][j] = frontierCount;
				prevFrontierCells.push_back({i,j});
				frontierCount ++;
			}
		}
	}
	visualizeCoords(prevFrontierCells, 0.0f, 1.0f, 0.0f, frontiers_pub);
	return prevFrontierCells;

}

/*
 * Checks for connected frontiers and returns centroid of each region
 * More accurately: Voronoi Partition
 */
std::vector<std::vector<int>> Environment::processFrontierCells(std::vector<std::vector<int>> frontierCells){
	std::vector<std::vector<int>> centroids;
	std::map<int,std::vector<int>> frontierMap;
	for (int i = 0; i < frontierCells.size(); i ++){
		frontierMap[i] = frontierCells[i];
	}

	int** visitedGrid_;
	visitedGrid_ = new int*[envWidth];
	for (int i = 0; i < envWidth; i++){
		visitedGrid_[i] = new int[envHeight];
		for (int j = 0; j < envHeight; j++){
			visitedGrid_[i][j] = 0;
		}
	}
	while (!frontierMap.empty()){
		int totalx = 0 , totaly = 0;

		std::vector<std::vector<int>> toVisitList;
		std::vector<std::vector<int>> cluster;

		toVisitList.push_back(frontierMap.begin()->second);
		frontierMap.erase(frontierMap.begin()->first);

		while (!toVisitList.empty()){
			std::vector<int> v = toVisitList.back();

			//"Visit" v's neighbors and add to toVisitList if they are frontiers
			int rad = 10; 	//hyper parameter
			int lowerx = MAX(1,v[0]-rad); int upperx=MIN(envWidth-2, v[0]+rad);
			int lowery = MAX(1,v[1]-rad); int uppery=MIN(envHeight-2, v[1]+rad);
			for (int i = lowerx; i < upperx; i++){
				for (int j = lowery; j < uppery; j++){
					int nx_ = i; int ny_ = j;
					if (visitedGrid_[nx_][ny_] == 0){
						//Connected!
						if (frontierGrid[nx_][ny_] != -1){
							totalx += nx_;
							totaly += ny_;
							cluster.push_back({nx_,ny_});
							toVisitList.push_back({nx_,ny_});
							frontierMap.erase(frontierGrid[nx_][ny_]);
						}
						visitedGrid_[nx_][ny_] = 1;
					}
				}

			}
			toVisitList.pop_back();
		}


		//Process cluster to output centroid; (Cluster is a simple mean) ,, ignore if cluster size too small/ combine different clusters
		if (cluster.size() > 5){
			int centroidx = (int)(totalx/cluster.size());
			int centroidy = (int)(totaly/cluster.size());
			//processing centroid:
			while (occupancyGrid.data[coordToIndex(centroidx, centroidy)] > 0){
				ROS_INFO("Unfeasible CENTROID (%d,%d)", centroidx, centroidy);
				std::vector<int> pt = cluster.back();
				totalx -= pt[0];
				totaly -= pt[1];
				cluster.pop_back();
				centroidx = (int)(totalx/cluster.size());
				centroidy = (int)(totaly/cluster.size());
			}
			if (cluster.size() > 5 ){
				centroids.push_back({centroidx,centroidy});
			}
		}
	}
	for (int i = 0; i < envWidth; i ++){
		delete[] visitedGrid_[i];
	}
	delete[] visitedGrid_;

	visualizeCoords(centroids, 1.0f, 0.0f, 0.0f, centroid_pub);
	return centroids;
}

bool Environment::isFrontier(int cx_, int cy_){
	// only perform convolution on non edge cells
	if (isEdge(cx_, cy_)){
		return false;
	}

	double grad = edgeGrad(cx_, cy_);
	if (grad > frontierThreshold){
		return true;
	}
	return false;
}


void Environment::updateRobotPose(geometry_msgs::PoseStamped currentPose_){
	currentPose = currentPose_;
}

void Environment::waitForRobotPose(){
	bUpdateRobotPose = true;
}

void Environment::updateTeamGoalPose(std::map<std::string, geometry_msgs::PoseStamped> teamGoalPose_){
	std::vector<geometry_msgs::PoseStamped> poseVector;
	for(auto& pose: teamGoalPose_) {
		poseVector.push_back(pose.second);
	}
	teamGoalPose = poseVector;
	ROS_INFO("TGP SIZE: %d", teamGoalPose.size());
}

void Environment::waitForTeamGoalPose(){
	bUpdateTeamGoalPose = true;
}

void Environment::updateRobotTeamPose(std::map<std::string, geometry_msgs::PoseStamped> teamPose_){
	std::vector<geometry_msgs::PoseStamped> poseVector;
	for(auto& pose: teamPose_) {
		poseVector.push_back(pose.second);
	}
	teamPose = std::move(poseVector);
}


void Environment::updateFailedFrontiers(std::vector<geometry_msgs::PoseStamped> failedFrontiers_){
	for (auto ff: failedFrontiers_){
		std::vector<int> coord = pointToCoord(ff.pose.position.x, ff.pose.position.y);
		failedCells.push_back(coord);
	}
}


void Environment::resetGrids(){
	for (auto coord : frontierCells){
		frontierGrid[coord[0]][coord[1]] = -1;
		informationGrid[coord[0]][coord[1]] = 0;
	}

	for (auto coord : fCentroids){
		costGrid[coord[0]][coord[1]] = 0;
		discountGrid[coord[0]][coord[1]] = 0;
	}

	frontierCells.clear();
	prevDCCells.clear();
	fCentroids.clear();

}

unsigned char Environment::Environment::costOfCell(std::vector<int> idx, std::vector<int> pos_){
	// only interested in unknown cells, need to convert sqrt to int
	// instead of using euclidean distance, use ros navigation global planner as a plugin
	//ROS_INFO("cx: %d, cy:%d, cost: %d", cx_, cy_, (unsigned char) (254 * sqrt(((pos_[0]-cx_)*(pos_[0]-cx_) + (pos_[1]-cy_)*(pos_[1]-cy_)))/maxDist));
	return (unsigned char) (254 * norm(idx,pos_)/maxDist);
}

//Larger discount for nearing cells
double Environment::Environment::distToDiscount(int dist){
	//return exp(-1*discountMult * dist);
	return 1-(dist/searchRadius);

}

double Environment::infoOfCell(int cx_, int cy_){
	return 1.0;
}

geometry_msgs::PoseStamped Environment::coordToPS(int cx_, int cy_){
	geometry_msgs::PoseStamped poseStamped;
	std::vector<float> pt = coordToPoint(cx_,cy_);
	poseStamped.pose.position.x = pt[0];
	poseStamped.pose.position.y = pt[1];
	poseStamped.pose.orientation.w = 1;
	poseStamped.header.frame_id = globalFrame;
	poseStamped.header.stamp = ros::Time(0);
	return poseStamped;
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
	float originx_ = occupancyGrid.info.origin.position.x + offx;
	float originy_ = occupancyGrid.info.origin.position.y + offy;
	return {originx_+cx_*occupancyGrid.info.resolution, originy_+cy_*occupancyGrid.info.resolution};
}

/*
 * mx_ : actual x coordinate of point in map frame
 * my_ : actual y coordinate of point in map frame
 * need to be calculated wrt origin
 */
std::vector<int> Environment::Environment::pointToCoord(float mx_, float my_){
	//get index then convert to Coord?
	float originx_ = occupancyGrid.info.origin.position.x + offx;
	float originy_ = occupancyGrid.info.origin.position.y + offy;
	return {(int)(floor((mx_-originx_)/occupancyGrid.info.resolution)),(int)(floor((my_-originy_)/occupancyGrid.info.resolution))};

}

bool Environment::isEnvUpdated(){
	return !bUpdateRobotPose && !bUpdateTeamGoalPose && !bUpdate;
}

bool Environment::isEnvInitialised(){
	return occupancyGrid.data.size() > 0 && offx != 0;
}

bool Environment::inMap(int cx_, int cy_){
	return (cx_ >= 0 && cx_ <= envWidth && cy_ >= 0 && cy_ <= envHeight);
}

bool Environment::isEdge(int cx_, int cy_){
	return (cx_ <= 0 || cx_ >= envWidth-1 || cy_ <= 0 || cy_ >= envHeight-1);
}

bool Environment::inFailed(int cx_, int cy_){
	for (auto fc: failedCells){
		if (fc[0] == cx_ && fc[1] == cy_){
			ROS_INFO("Failed cell: (%d,%d)", cx_,cy_);
			return true;
		}
	}
	return false;
}


Environment::Frontier::Frontier(int x_,int y_,unsigned char utility_){
	x = x_;
	y = y_;
	utility = utility_;
}

Environment::Frontier& Environment::Frontier::operator = (const Frontier& f1){
	x = f1.x;
	y = f1.y;
	utility = f1.utility;
	pose = f1.pose;
	return *this;
};


visualization_msgs::Marker Environment::visualizeCoords(std::vector<std::vector<int>> coordCells,
		float r, float g, float b, ros::Publisher pub){
  //uint32_t shape = visualization_msgs::Marker::CUBE;
	ROS_INFO("Visualising...");

	visualization_msgs::Marker marker;
	marker.header.frame_id = "robot_1/map";
	marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;
    //marker.type = shape;
    marker.type = visualization_msgs::Marker::POINTS;

    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(coordCells.size());
    float res = occupancyGrid.info.resolution;

    for (int i = 0; i < coordCells.size(); i++){
    	std::vector<float> fPoint = coordToPoint(coordCells[i][0],coordCells[i][1]);
    	marker.points[i].x = fPoint[0];
    	marker.points[i].y = fPoint[1];
    }

    marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
// %EndTag(SCALE)%


    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    pub.publish(marker);

    return marker;
}

double Environment::edgeGrad(int cx_, int cy_){
	kcells i;
	float gradx, grady;
	int idxTL = coordToIndex(kernel[i=TL][0]+cx_, kernel[i=TL][1]+cy_);
	int idxT = coordToIndex(kernel[i=T][0]+cx_, kernel[i=T][1]+cy_);
	int idxTR = coordToIndex(kernel[i=TR][0]+cx_, kernel[i=TR][1]+cy_);
	int idxL = coordToIndex(kernel[i=L][0]+cx_, kernel[i=L][1]+cy_);
	int idxR = coordToIndex(kernel[i=R][0]+cx_, kernel[i=R][1]+cy_);
	int idxBL = coordToIndex(kernel[i=BL][0]+cx_, kernel[i=BL][1]+cy_);
	int idxB = coordToIndex(kernel[i=B][0]+cx_, kernel[i=B][1]+cy_);
	int idxBR = coordToIndex(kernel[i=BR][0]+cx_, kernel[i=BR][1]+cy_);

	gradx = -1 * cap(occupancyGrid.data[idxTL]) + cap(occupancyGrid.data[idxTR]);
	gradx += -1 * cap(occupancyGrid.data[idxBL]) + cap(occupancyGrid.data[idxBR]);
	gradx += -2 * cap(occupancyGrid.data[idxL]) + 2*cap(occupancyGrid.data[idxR]);

	grady = -1 * cap(occupancyGrid.data[idxBL]) + cap(occupancyGrid.data[idxTL]);
	grady += -1 * cap(occupancyGrid.data[idxBR]) + cap(occupancyGrid.data[idxTR]);
	grady += -2 * cap(occupancyGrid.data[idxB]) + 2*cap(occupancyGrid.data[idxT]);

	return abs(gradx)+abs(grady);

}

int Environment::cap(int value_){
	return -MIN(0,value_);

}

/*bool Environment::Environment::updateDiscountCells(){
	prevDCCells.clear();
	if (teamGoalPose.size() == 0 && teamPose.size() == 0){
		ROS_INFO("No info on other robots");
		return true;
	} else if (teamGoalPose.size() > 0){
		// neighbors pose is in meters
		for (auto rGoal : teamGoalPose){
			ros::Time poseTime = rGoal.header.stamp;
			ros::Duration timePassed = ros::Time::now() - poseTime;
			if (timePassed.toSec() > expirationTime){
				waitForTeamGoalPose();
				return false;
			}
			std::vector<int> robotCoord = pointToCoord(rGoal.pose.position.x, rGoal.pose.position.y);
			int xc = robotCoord[0];
			int yc = robotCoord[1];
			//consider first quadrant, can translate after
			for (int x = 0; x <= searchRadius; x++){
				for (int y = 0; y <= x; y++){
					if (x*x + y*y <= searchRadius * searchRadius){
						int dist = x + y; //taking Manhattan distance
						for (std::vector<int> cell : circleCorners) {
							int x_ = xc+cell[0]*x, y_ = yc+cell[1]*y;
							if (inMap(x_,y_) && frontierGrid[x_][y_] != -1){
								if (discountGrid[x_][y_] == 0){
									discountGrid[x_][y_] = distToDiscount(dist);
								} else {
									//inspired by alpha blending kekek
									discountGrid[x_][y_] = discountGrid[x_][y_]  + (1-discountGrid[x_][y_])*distToDiscount(dist);
								}
								prevDCCells.push_back({x_,y_});
							}
							x_ = xc+cell[0]*y, y_ = yc+cell[1]*x;
							if (inMap(x_,y_) && frontierGrid[x_][y_] != -1){
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
		return true;
	} else if (teamPose.size() > 0) {
		// neighbors pose is in meters
		for (auto robotPose : teamPose){
			ros::Time poseTime = robotPose.header.stamp;
			ros::Duration timePassed = ros::Time::now() - poseTime;
			if (timePassed.toSec() > expirationTime){
				waitForTeamGoalPose();
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
						for (std::vector<int> cell : circleCorners) {
							int x_ = xc+cell[0]*x, y_ = yc+cell[1]*y;
							if (inMap(x_,y_) && frontierGrid[x_][y_] != 1){
								if (discountGrid[x_][y_] == 0){
									discountGrid[x_][y_] = distToDiscount(dist);
								} else {
									//inspired by alpha blending kekek
									discountGrid[x_][y_] = discountGrid[x_][y_]  + (1-discountGrid[x_][y_])*distToDiscount(dist);
								}
								prevDCCells.push_back({x_,y_});
							}
							x_ = xc+cell[0]*y, y_ = yc+cell[1]*x;
							if (inMap(x_,y_) && frontierGrid[x_][y_] != 1){
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
		return true;
	}
	return true;

}*/
