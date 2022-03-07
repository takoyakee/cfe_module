/*
 * performance_evaluation.cpp
 *
 *  Created on: 8 Dec 2021
 *      Author: yanling
 */
#include <fyp_api/functions.h>
#include <fyp_api/Visualisation.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <../../../devel/include/fyp_api/distMetrics.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#define UPPERTHRESHOLD 8
#define FREE 0
#define OCCUPIED 100
#define UNKNOWN -1

geometry_msgs::TransformStamped computeg2l(std::string globalFrame, std::string robotMapFrame){
	//Convert received posestamped to global frame if not already in global frame
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener tfl(buffer);
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::Pose empty;
	try {
		transformStamped = buffer.lookupTransform(robotMapFrame, globalFrame, ros::Time(0), ros::Duration(3.0));
	} catch (tf2::TransformException &e) {
		ROS_WARN("Error with transform");
	}
	return transformStamped;

	//ROS_INFO("QUATERNION (%.3f,%.3f)", transformStamped.transform.rotation.w,  transformStamped.transform.rotation.x);
}
geometry_msgs::TransformStamped computel2g(std::string globalFrame, std::string robotMapFrame){
	//Convert received posestamped to global frame if not already in global frame
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener tfl(buffer);
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::Pose empty;
	try {
		transformStamped = buffer.lookupTransform(globalFrame, robotMapFrame, ros::Time(0), ros::Duration(3.0));
	} catch (tf2::TransformException &e) {
		ROS_WARN("Error with transform");
	}
	return transformStamped;

	//ROS_INFO("QUATERNION (%.3f,%.3f)", transformStamped.transform.rotation.w,  transformStamped.transform.rotation.x);
}

bool inMap(int cx_, int cy_, int envWidth_, int envHeight_){
	return (cx_ >= 0 && cx_ <=envWidth_-1 && cy_ >= 0 && cy_ <= envHeight_-1);
}

int coordToIndex(int cx_, int cy_, int envWidth_){
	return cy_*envWidth_ + cx_;
}


struct Metrics{
	float mapFree, mapOccupied, mapTotal;
	float exploredFree, exploredOccupied;
	float exploreRate;

	//to track robot performance overtime
	std::vector<float> robotExploredArea, robotDist;

	void setAreas(float free, float occupied){
		exploredFree = free;
		exploredOccupied = occupied;
	}

	void setRate(float prevExplored, float dt){
		exploreRate = (exploredTotal()-prevExplored)/dt;
	}

	float exploredTotal(){
		return exploredFree+exploredOccupied;
	}
	float exploredRatio(){
		return exploredTotal()/mapTotal;
	}

	Metrics():exploredFree(0), exploredOccupied(0), exploreRate(0)
	,mapFree(0), mapOccupied(0), mapTotal(0)
	{};

	void Initialise(float mapFree_, float mapOccupied_, int teamSize)
	{
		mapFree = mapFree_;
		mapOccupied = mapOccupied_;
		mapTotal = mapFree_ + mapOccupied_;
		robotExploredArea.reserve(teamSize);
		for (int i = 0; i < teamSize; i++){
			robotDist.push_back(0);
		}
	}

};

//tentatively, combined listens to map_merge node
std::vector<std::unique_ptr<nav_msgs::OccupancyGrid>> maps;
std::unique_ptr<nav_msgs::OccupancyGrid> combined;
geometry_msgs::Point combinedMapOrigin;
std::map<int,geometry_msgs::PoseStamped> robotPoses;
std::map<int,nav_msgs::Path> robotTrajs;
std::vector<geometry_msgs::Point> scanOriginGlobal, ScanOrigin;
std::vector<geometry_msgs::TransformStamped> global2Local, local2Global;
std::string ogFrame, odomFrame;

int envWidth = 0, envHeight = 0, teamSize = 0;
double res;
Metrics metrics;
std::vector<std::string> mapUninitialised, distUninitialised, odomUninitialised;
bool mapsReceived = false, distReceived = false, odomReceived = false;

void resetMap(){
	//ROS_INFO("Resetting map");
	for (int i = 0; i < combined->info.width * combined->info.height; i++){
		combined->data[i] = -1;
	}
}
/*
 * local2global actually contains a transformation matrix as below
 * [global x] 		[R11 R12 R13  P1][local x]
 * [global y]	= 	[R21 R22 R23  P2][local y]
 * [global z] 		[R31 R32 R33  P3][local z]
 * [	1	] 		[0 	  0	   0  1][	1	]
 * However, since we assume that combined map is same frame as global map (same orientation) but with different origins,
 * we need not use transformstamped
 */
std::vector<float> coordToPoint(int cx_, int cy_, geometry_msgs::Point scanOrigin_, geometry_msgs::TransformStamped local2global){
	float originx_ = scanOrigin_.x;
	float originy_ = scanOrigin_.y;
	return {originx_+(cx_*res), originy_+(cy_*res)};
}

/*
 * global2local actually contains a transformation matrix as below
 * [local x] 		[R11 R12 R13  P1][global x]
 * [local y]	= 	[R21 R22 R23  P2][global y]
 * [local z] 		[R31 R32 R33  P3][global z]
 * [   1   ] 		[0 	  0	   0  1][	1	]
 *
 */
std::vector<int> pointToCoord(float mx_, float my_, geometry_msgs::Point scanOrigin, geometry_msgs::TransformStamped global2local ){
	//convert to ownv
/*	tf2::Vector3 ownv, globalv;
	globalv.setX(mx_);
	globalv.setY(my_);
	globalv.setZ(0);
	ownv = m.inverse()*globalv;*/

	geometry_msgs::Pose globalPose, localPose;
	globalPose.position.x = mx_;
	globalPose.position.y = my_;
	globalPose.position.z = 0;
	globalPose.orientation.w = 0;
	globalPose.orientation.x = 0;
	globalPose.orientation.y = 0;
	globalPose.orientation.z = 1;

	tf2::doTransform(globalPose, localPose, global2local);

	//get index then convert to Coord?
	float originx = scanOrigin.x;
	float originy = scanOrigin.y;
	return {(int)(floor((localPose.position.x-originx)/res)),(int)(floor((localPose.position.y-originy)/res))};

}

void updateMergedMap(){
	resetMap();
	combined->info.origin.position = combinedMapOrigin;
	for (int i = 0; i < combined->info.width; i ++){
		for (int j = 0; j < combined->info.height; j++){
			int cellValue = -1;
			std::vector<float> gp = coordToPoint(i, j, combinedMapOrigin, local2Global[0]);

			for (int t = 0; t < teamSize; t++){
				std::vector<int> own = pointToCoord(gp[0],gp[1], ScanOrigin[t],global2Local[t]);
				int ownmapx = own[0];
				int ownmapy = own[1];

				if (inMap(ownmapx, ownmapy, maps[t]->info.width, maps[t]->info.height)){
					int ownidx = coordToIndex(ownmapx, ownmapy, maps[t]->info.width);
					if (maps[t]->data[ownidx] > cellValue){
						cellValue = maps[t]->data[ownidx];
					}
				}

			}
			int idx = coordToIndex(i, j,  combined->info.width);
			combined->data[idx] = cellValue;

		}
	}
}
void updateAreaExplored(double dt){

	int size = combined->data.size();
	int free = 0, occupied = 0, unknown = 0;

	for (int i = 0; i < size; i++){
		if (combined->data[i] == FREE){
			free++;

		} else if (combined->data[i] == OCCUPIED){
			occupied++;

		} else if (combined->data[i] == UNKNOWN){
			continue;

		} else {
			ROS_WARN("Check occupancy grid values");
		}

	}

	float cell2tom2 = res*res;
	float prevExplored = metrics.exploredTotal();
	metrics.setAreas(free * cell2tom2, occupied * cell2tom2);
	metrics.setRate(prevExplored, dt);

	ROS_INFO("Time: %.3f \t Explored: %.3f", ros::Time::now().toSec(),metrics.exploredRatio());
}

void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg){
	std::string frame = msg->header.frame_id;
	int rn = atoi(&eraseSubStr(frame, "/"+odomFrame).back());
	geometry_msgs::PoseStamped currentPose, prevPose;
	currentPose.pose = msg->pose.pose;


	if (!odomReceived){
		robotPoses[rn-1] = currentPose;
		auto it = std::find(odomUninitialised.begin(), odomUninitialised.end(), frame);
		if ( it != odomUninitialised.end()){
			//register dist as received;
			odomUninitialised.erase(it);
		}
		odomReceived = (odomUninitialised.size()==0);
		ROS_INFO("odom from: %s received, remaining: %d",frame.c_str(), odomUninitialised.size());

	} else {
		if ( robotPoses.find(rn-1) == robotPoses.end() ) {
		  ROS_WARN("ROBOT NUMBER NOT FOUND");
		} else {
			prevPose = robotPoses[rn-1];
			float dist = norm(currentPose,prevPose);
			metrics.robotDist[rn-1] += dist;
			robotPoses[rn-1] = currentPose;

		}
	}
	if (robotPoses.find(rn-1) != robotPoses.end()){
		if (robotTrajs.find(rn-1)!= robotTrajs.end()){
			currentPose.header = msg->header;
			robotTrajs[rn-1].poses.push_back(currentPose);
		}
	}


}

void distCallBack(const fyp_api::distMetrics msg){
	std::string name = msg.robotName;
	int rn = atoi(&name.back());
	metrics.robotDist[rn-1] = msg.dist;

	if (!distReceived){
		auto it = std::find(distUninitialised.begin(), distUninitialised.end(),name);
		if ( it != distUninitialised.end()){
			//register dist as received;
			distUninitialised.erase(it);
		}
		distReceived = (distUninitialised.size()==0);
		ROS_INFO("dist from: %s received, remaining: %d",name.c_str(), distUninitialised.size());

	}

}


void ogCallBack(const nav_msgs::OccupancyGrid::ConstPtr msg){
	std::string frame = msg->header.frame_id;
	int rn = atoi(&eraseSubStr(frame, "/"+ogFrame).back());
	*(maps[rn-1]) = *msg;

	geometry_msgs::Pose localPose, globalPose;
	localPose.position =  msg->info.origin.position;

	tf2::doTransform(localPose, globalPose, local2Global[rn-1]);

	scanOriginGlobal[rn-1] = globalPose.position;
	ScanOrigin[rn-1] = msg->info.origin.position;

	combinedMapOrigin.x = std::min(combinedMapOrigin.x,  scanOriginGlobal[rn-1].x);
	combinedMapOrigin.y = std::min(combinedMapOrigin.y,  scanOriginGlobal[rn-1].y);

	//determining if the current map reach is okay

	float max_x, min_x, max_y, min_y;
	max_x =  scanOriginGlobal[rn-1].x + (msg->info.width*res);
	max_y = scanOriginGlobal[rn-1].y + (msg->info.height*res);


	if (max_x > combinedMapOrigin.x + envWidth*res){
		envWidth = (int)((max_x - combinedMapOrigin.x)/res);
	}

	if (max_y > combinedMapOrigin.y + envHeight*res){
		envHeight = (int)((max_y - combinedMapOrigin.y)/res);
	}


/*	envWidth = std::max((int)(msg->info.width),envWidth);
	envHeight = std::max((int)(msg->info.height),envHeight);*/


	//filling in mapinformation
	if (combined->info.height != envHeight || combined->info.width != envWidth){
		combined->info.height = envHeight;
		combined->info.width = envWidth;
		combined->data.resize(envWidth * envHeight);
		for (int i = 0; i < envWidth * envHeight; i++){
			combined->data[i] = -1;
		}

	}

	if (!mapsReceived){
		res = msg->info.resolution;

		auto it = std::find(mapUninitialised.begin(), mapUninitialised.end(),frame);
		if ( it != mapUninitialised.end()){
			//register OG as received;
			mapUninitialised.erase(it);
		}
		mapsReceived = (mapUninitialised.size()==0);
		ROS_INFO("OG from: %s received, remaining: %d",frame.c_str(), mapUninitialised.size());

	}


}

void printFile(std::ofstream& file_){
	file_ << ros::Time::now().toSec() << "\t"
			<< metrics.exploredFree << "\t"
			<< metrics.exploredOccupied << "\t"
			<< metrics.exploredRatio() << "\t"
			<< metrics.exploreRate << "\t";
	for (int i = 0; i < teamSize; i ++){
		file_ << metrics.robotDist[i] << "\t";
	}
	file_ << std::endl;

}

//To subscribe to merged map and compare with environment
//To subscribe to distance travelled by each robot
// 
// (FAR) To subscribe to all maps received and compute percentage overlap -> openCV

int main(int argc, char** argv){
	ros::init(argc, argv, "evaluator");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");


	double iterFreq;
	std::string globalFrame, sharedStatusTopic;
	std::string nameSpace, occupancyGridTopic, distanceTopic, odomTopic;
	std::string fileName, envUsed, mergedMapTopic, trajectoryTopic;
	bool measureByOdom = false;

	nh_private.getParam("iter_freq", iterFreq);
	nh_private.getParam("team_size", teamSize);
	nh_private.getParam("global_frame", globalFrame);
	nh_private.getParam("shared_status_topic", sharedStatusTopic);

	nh_private.getParam("namespace", nameSpace);
	nh_private.getParam("og_frame",ogFrame);
	nh_private.getParam("occupancy_grid_topic", occupancyGridTopic);
	nh_private.getParam("distance_topic", distanceTopic);
	nh_private.getParam("odom_topic", odomTopic);
	nh_private.getParam("odom_frame", odomFrame);
	nh_private.getParam("measure_by_odom",measureByOdom);

	// Measurement of exploration volume
	float freeAreas,occupiedAreas, totalAreas,exploreThreshold;
	nh_private.getParam("Free_Area",freeAreas);
	nh_private.getParam("Occupied_Area",occupiedAreas);
	nh_private.getParam("Total_Area",totalAreas);
	nh_private.getParam("env_used", envUsed);
	nh_private.getParam("explore_threshold", exploreThreshold);

	nh_private.getParam("file_name",fileName);
	nh_private.getParam("merged_map_topic",mergedMapTopic);
	nh_private.getParam("trajectory_topic",trajectoryTopic);


	ROS_INFO("Loading Metrics for %s Environment", envUsed.c_str());

	metrics.Initialise(freeAreas, occupiedAreas, teamSize);
	//Broadcast merged map as well as exploration status
	combined = std::unique_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);
	ros::Publisher mapPub = nh.advertise<nav_msgs::OccupancyGrid>(mergedMapTopic,1);
	ros::Publisher explorationStatus = nh.advertise<std_msgs::Bool>(sharedStatusTopic,1);
	std::vector<ros::Publisher> trajPubs;
	std::vector<ros::Subscriber> mapSubs, distSubs, odomSubs;

	mapSubs.reserve(teamSize+1);
	distSubs.reserve(teamSize+1);

	scanOriginGlobal.reserve(teamSize+1);
	ScanOrigin.reserve(teamSize+1);


	for (int i = 0; i < teamSize; i++){
		std::string robotName = nameSpace+std::to_string(i+1);
		std::string robotOGTopic = robotName+ "/" + occupancyGridTopic;
		std::string robotOGFrame = robotName.substr(1)+"/"+ogFrame;
		std::string robotOdomTopic = robotName+ "/" + odomTopic;
		std::string robotOdomFrame = robotName.substr(1)+"/"+odomFrame;
		std::string robotDistTopic = robotName+ "/" + distanceTopic;
		std::string robotTrajTopic = robotName+"/" + trajectoryTopic;

		//subscribing to grid and dist
		mapSubs[i] = nh.subscribe<nav_msgs::OccupancyGrid>(robotOGTopic, 1, &ogCallBack);

		if (measureByOdom){
			//not sure why odomsub and trajpub cannot be initialised the same way as mapsubs/distsubs
			ros::Subscriber odomsub = nh.subscribe(robotOdomTopic, 1,&odomCallBack);
			odomSubs.push_back(std::move(odomsub));
			distReceived = true;

			ros::Publisher trajpub = nh.advertise<nav_msgs::Path>(robotTrajTopic,1);
			trajPubs.push_back(std::move(trajpub));

			nav_msgs::Path path;
			path.header.frame_id = robotOdomFrame;
			robotTrajs[i] = path;



			odomUninitialised.push_back(robotOdomFrame);
		} else {
			distSubs[i] = nh.subscribe<fyp_api::distMetrics>(robotDistTopic,1,&distCallBack);
			odomReceived = true;

			distUninitialised.push_back(robotName);
		}

		mapUninitialised.push_back(robotOGFrame);

		std::unique_ptr<nav_msgs::OccupancyGrid> tmpmap(new nav_msgs::OccupancyGrid);
		maps.push_back(std::move(tmpmap));
		global2Local.push_back(computeg2l(globalFrame, robotOGFrame));
		local2Global.push_back(computel2g(globalFrame, robotOGFrame));

	}

	std::ofstream file, distfile;
	file.open(fileName);
	file << "Time" << "\t" << "Free (m^2)" << "\t" << "Occupied(m^2)" << "\t" << "Explored Ratio" << "\t" << "Explore Rate (m^2/s)" << "\t"
			<< "r1 dist (m)" << "\t" << "r2 dist (m)" << "\t" << "r3 dist (m)" << "\t" << std::endl;

	ros::Rate rate(iterFreq);
	double time = ros::Time::now().toSec(), dt = 0;
	while(ros::ok() && (!mapsReceived || !odomReceived || !distReceived)){
		//ROS_INFO("Waiting for topics...");
		ros::spinOnce();
	}

	//filling in map information
	combined->header.frame_id = globalFrame;
	combined->info.resolution = res;
	bool stop = false;
	while (ros::ok()){
		if (stop){
			ROS_INFO("Save map now..., else shut down node");
			ros::spinOnce();
			updateMergedMap();
			mapPub.publish(*combined);
			rate.sleep();
			continue;
		}
		ros::spinOnce();

		dt = ros::Time::now().toSec() - time;
		if (ros::Time::now().toSec() - time == 0){
			continue;
		}
		time = ros::Time::now().toSec();
		//For visualisation
		updateMergedMap();
		mapPub.publish(*combined);

		for (int i = 0; i < teamSize; i ++){
			trajPubs[i].publish(robotTrajs[i]);
		}

		//For file output
		updateAreaExplored(dt);
		printFile(file);


		std_msgs::Bool finished;
		if (metrics.exploredRatio() >= exploreThreshold){
			finished.data = true;
			explorationStatus.publish(finished);

			stop = true;
		} else {
			finished.data = false;
			explorationStatus.publish(finished);

		}
		rate.sleep();

	}
	file << "Free:" << "\t"  << freeAreas << "\t" << "occupied:" << "\t" << occupiedAreas << "\t" << "total:" << "\t" <<totalAreas << std::endl;
	file.close();
	//ros::shutdown();
	return 0;


}
