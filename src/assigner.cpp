/*
 * assigner.cpp
 *
 *  Created on: 8 Dec 2021
 *      Author: yanling
 */
#include <fyp_api/functions.h>
#include <fyp_api/Visualisation.h>
#define UPPERTHRESHOLD 8


//TODO: Subscribes to merge map topic, robot pose topic, frontiers as detected by robots
//TODO:  Mark out regions with frontiers as detected, assign robots to frontiers
//TODO: Publish individual robot's destination

enum kcells{TL=1, T=2, TR = 3, L=4, R=6, BL=7, B=8, BR=9} ;
std::map<int,std::vector<int>> kernel = {{1,{-1,1}}, {2,{0,1}}, {3,{1,1}},
											{4,{-1,0}}, 		   {6,{1,0}},
											{7,{-1,-1}}, {8,{0,-1}}, {9,{1,-1}} };
std::vector<std::vector<int>> tosee;
double clusterRad;
int centroidRad = 2; //same as clusterRad used during clustering
struct envlite{
	geometry_msgs::Point scanOrigin, offSet;
	int envWidth, envHeight;
	double res;
	nav_msgs::OccupancyGrid occupancyGrid;
	std::string robotName, robotMapFrame, globalFrame;
	geometry_msgs::TransformStamped own2Global, global2Own;

	envlite(std::string robotName_, std::string robotMapFrame_, std::string globalFrame_):
	envWidth(0),envHeight(0),res(0)
	{
		robotName = robotName_;
		robotMapFrame = robotMapFrame_;
		globalFrame = globalFrame_;
		computeOffSet();
	}
	~envlite(){};

	void updateOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_){
		if (occupancyGrid_->data.size() > 0){
			occupancyGrid = *occupancyGrid_;
			envWidth = occupancyGrid_->info.width;
			envHeight = occupancyGrid_->info.height;
			res = occupancyGrid_->info.resolution;
			scanOrigin = occupancyGrid_->info.origin.position;
		}
	}

	float calGrad(int cx_, int cy_){
		kcells i;
		int idxTL = coordToIndex(kernel[i=TL][0]+cx_, kernel[i=TL][1]+cy_);
		int idxT = coordToIndex(kernel[i=T][0]+cx_, kernel[i=T][1]+cy_);
		int idxTR = coordToIndex(kernel[i=TR][0]+cx_, kernel[i=TR][1]+cy_);
		int idxL = coordToIndex(kernel[i=L][0]+cx_, kernel[i=L][1]+cy_);
		int idxR = coordToIndex(kernel[i=R][0]+cx_, kernel[i=R][1]+cy_);
		int idxBL = coordToIndex(kernel[i=BL][0]+cx_, kernel[i=BL][1]+cy_);
		int idxB = coordToIndex(kernel[i=B][0]+cx_, kernel[i=B][1]+cy_);
		int idxBR = coordToIndex(kernel[i=BR][0]+cx_, kernel[i=BR][1]+cy_);

		float gradx, grady;
		gradx = -1 * cap(occupancyGrid.data[idxTL]) + cap(occupancyGrid.data[idxTR]);
		gradx += -1 * cap(occupancyGrid.data[idxBL]) + cap(occupancyGrid.data[idxBR]);
		gradx += -2*cap(occupancyGrid.data[idxL]) + 2* cap(occupancyGrid.data[idxR]);

		grady = -1 * cap(occupancyGrid.data[idxBL]) + cap(occupancyGrid.data[idxTL]);
		grady += -1 * cap(occupancyGrid.data[idxBR]) + cap(occupancyGrid.data[idxTR]);
		grady += - 2* cap(occupancyGrid.data[idxB]) + 2* cap(occupancyGrid.data[idxT]);

		return abs(gradx)+abs(grady);

	}
	bool exclude(int cx_, int cy_)
	{
		if(isEdge(cx_,cy_)) return false;
		kcells i;
		int idxTL = coordToIndex(kernel[i=TL][0]+cx_, kernel[i=TL][1]+cy_);
		int idxT = coordToIndex(kernel[i=T][0]+cx_, kernel[i=T][1]+cy_);
		int idxTR = coordToIndex(kernel[i=TR][0]+cx_, kernel[i=TR][1]+cy_);
		int idxL = coordToIndex(kernel[i=L][0]+cx_, kernel[i=L][1]+cy_);
		int idxR = coordToIndex(kernel[i=R][0]+cx_, kernel[i=R][1]+cy_);
		int idxBL = coordToIndex(kernel[i=BL][0]+cx_, kernel[i=BL][1]+cy_);
		int idxB = coordToIndex(kernel[i=B][0]+cx_, kernel[i=B][1]+cy_);
		int idxBR = coordToIndex(kernel[i=BR][0]+cx_, kernel[i=BR][1]+cy_);
		//checking if its occupied
		float gradx, grady;
		gradx = -1 * cap(occupancyGrid.data[idxTL]) + cap(occupancyGrid.data[idxTR]);
		gradx += -1 * cap(occupancyGrid.data[idxBL]) + cap(occupancyGrid.data[idxBR]);
		gradx += -2*cap(occupancyGrid.data[idxL]) + 2* cap(occupancyGrid.data[idxR]);

		grady = -1 * cap(occupancyGrid.data[idxBL]) + cap(occupancyGrid.data[idxTL]);
		grady += -1 * cap(occupancyGrid.data[idxBR]) + cap(occupancyGrid.data[idxTR]);
		grady += - 2* cap(occupancyGrid.data[idxB]) + 2* cap(occupancyGrid.data[idxT]);

		if ((abs(gradx) +abs(grady)) > UPPERTHRESHOLD){
			return true;
		}

		if ((abs(gradx) +abs(grady))>= 4){
			//ROS_INFO("IT IS ACTUALLY A FRONTIER");
			return false;
		}
		int sum = 0;

		sum += (occupancyGrid.data[idxTL] == -1);
		sum += (occupancyGrid.data[idxT] == -1);
		sum += (occupancyGrid.data[idxTR] == -1);
		sum += (occupancyGrid.data[idxL] == -1);
		sum += (occupancyGrid.data[idxR] == -1);
		sum += (occupancyGrid.data[idxBL] == -1);
		sum += (occupancyGrid.data[idxB] == -1);
		sum += (occupancyGrid.data[idxBR] == -1);
		if (sum > 2 ) {
			//ROS_INFO("IT IS UNKNOWN");
			return false;
		}

		//if it is known by other cell,  do not exclude if it is a frontier, else exclude
		return true;
	}


	int cap(int value_){
		if (value_ < 0){
			return -value_;
		}
		return value_;
	}

	bool isEdge(int cx_, int cy_){
		return (cx_ <= 0 || cx_ >= envWidth-1 || cy_ <= 0 || cy_ >= envHeight-1);
	}



	void computeOffSet(){
		//Convert received posestamped to global frame if not already in global frame
		tf2_ros::Buffer buffer;
		tf2_ros::TransformListener tfl(buffer);
		geometry_msgs::TransformStamped transformStamped;
		try {
			transformStamped = buffer.lookupTransform(globalFrame, robotMapFrame, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException &e) {
			ROS_WARN("Error with transform");
		}
		offSet.x = transformStamped.transform.translation.x;
		offSet.y = transformStamped.transform.translation.y;
		own2Global = transformStamped;

		try {
			transformStamped = buffer.lookupTransform(robotMapFrame, globalFrame,ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException &e) {
			ROS_WARN("Error with transform");
		}
		global2Own = transformStamped;

		//ROS_INFO("%s to %s (%.3f,%.3f)", robotMapFrame.c_str(),globalFrame.c_str(), offSet.x, offSet.y);
	}

	std::vector<int> ptToCoord(geometry_msgs::Point pt){
		return pointToCoord(pt.x, pt.y);
	}

	int coordToIndex(int cx_, int cy_){
		return cy_*envWidth + cx_;
	}

	std::vector<int> pointToCoord(float mx_, float my_){
		geometry_msgs::Pose globalPose, ownPose;
		globalPose.position.x = mx_;
		globalPose.position.y = my_;
		globalPose.position.z = 0;
		globalPose.orientation.w = 0;
		globalPose.orientation.x = 0;
		globalPose.orientation.y = 0;
		globalPose.orientation.z = 1;

		tf2::doTransform(globalPose, ownPose, global2Own);

		//get index then convert to Coord?
		float originx = scanOrigin.x;
		float originy = scanOrigin.y;


		return {(int)(floor((ownPose.position.x-originx)/res)),(int)(floor((ownPose.position.y-originy)/res))};

	}

	bool inMap(int cx_, int cy_){
		return (cx_ >= 0 && cx_ <=envWidth-1 && cy_ >= 0 && cy_ <= envHeight-1);
	}

	bool isFrontier(int& cx_, int& cy_, float angle){
		float rawx, rawy,prevgrad = 0;
		rawx = cx_;
		rawy = cy_;
		int cx = cx_; int cy= cy_;
		int count = 0,maxcount = 3,bestStep = 0, sign = 1;
		bool flip = false;
		while (inMap(cx,cy)){
			if ( calGrad(cx,cy) > UPPERTHRESHOLD){
				//ROS_INFO("STEPS TAKEN: %d", count);
				return false;
			}
			if (flip && count == maxcount){
				cx_ += (int)(sign*bestStep*(cos(angle)));
				cy_ += (int)(sign*bestStep*(sin(angle)));
				return true;
			}
			if (count == maxcount){
				flip = true;
				count = 0; sign = -1;
				rawx = cx_;
				rawy = cy_;
			}

			rawx += sign*(cos(angle));
			rawy += sign*(sin(angle));
			cx = (int) rawx;
			cy = (int) rawy;
			count ++;
			if (calGrad(cx,cy) > prevgrad){
				bestStep = count;
			}
			prevgrad = calGrad(cx,cy);
			//ROS_INFO("count %d: %d,%d",count, cx,cy);
		}

		return true;
	}

};

std::vector<fyp_api::centroid> clusterFrontier(std::vector<fyp_api::centroid> centroidPts){
		std::map<std::vector<double>, fyp_api::centroid> frontierMap;
		for (int i = 0; i < centroidPts.size(); i ++){
			std::vector<double> key = {centroidPts[i].point.x,centroidPts[i].point.y};
			frontierMap[key] = centroidPts[i];
		}

		std::vector<fyp_api::centroid> newCentroids;

		while (!frontierMap.empty()){
			float totalx = 0 , totaly = 0, totalig = 0, totalgrad = 0;
			int size = 0;

			std::vector<fyp_api::centroid> toVisitList;

			toVisitList.push_back(frontierMap.begin()->second);
			totalx += frontierMap.begin()->second.point.x; totaly += frontierMap.begin()->second.point.y;
			totalig +=frontierMap.begin()->second.ig; totalgrad +=frontierMap.begin()->second.grad;
			++size;
			frontierMap.erase(frontierMap.begin()->first);

			while (!toVisitList.empty()){
				fyp_api::centroid curr = toVisitList.back();
				std::vector<std::vector<double>> toErase;
				for (auto key: frontierMap){
					float dist = norm(curr.point, key.second.point);
					if (dist <= clusterRad && abs(curr.grad-key.second.grad)<=0.5){
						toVisitList.push_back(key.second);
						totalx += key.second.point.x; totaly += key.second.point.y; totalig += key.second.ig;
						totalgrad += key.second.grad;
						size++;

						std::vector<double> akey = {key.second.point.x,key.second.point.y};
						toErase.push_back(akey);
					}
				}
				for (auto it: toErase){
					frontierMap.erase(it);
				}
				toVisitList.pop_back();
			}

			fyp_api::centroid c;
			c.point.x = totalx/size;
			c.point.y = totaly/size;
			c.ig = totalig/size;
			c.grad = totalgrad/size;
			newCentroids.push_back(c);
		}
		return newCentroids;
	}

std::vector<envlite> Envs;
//tentatively, combined listens to map_merge node
std::unique_ptr<nav_msgs::OccupancyGrid> maps = std::unique_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);
std::map<std::string, fyp_api::centroidArray> teamCentroids;
std::vector<std::string> mapUninitialised;
bool mapsReceived = false;


void ogCallBack(const nav_msgs::OccupancyGrid::ConstPtr msg){
	std::string frame = msg->header.frame_id;
	int rn = atoi(&eraseSubStr(frame, "/map").back());
	Envs[rn-1].updateOccupancyGrid(msg);

	if (!mapsReceived){
		auto it = std::find(mapUninitialised.begin(), mapUninitialised.end(),frame);
		if ( it != mapUninitialised.end()){
			//register OG as received;
			mapUninitialised.erase(it);
		}
		if (mapUninitialised.size() == 0) mapsReceived = true;
		ROS_INFO("OG from: %s received, remaining: %d",frame.c_str(), mapUninitialised.size());

	}


}

void centroidCB(fyp_api::centroidArray c){
	if (c.robotName.compare("assigner")!=0){
		teamCentroids[c.robotName] = c;
		//ROS_INFO("Centroid CB: %s REGISTERED, size: %d", c.robotName.c_str(), teamCentroids.size());
	}

}

int main(int argc, char** argv){
	ros::init(argc, argv, "assigner");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	int teamSize = 0;

	std::string globalFrame;
	std::string param_ns = "/frontier_common/", sharedFrontierTopic, mapMergedTopic; // /frontier/common/
	nh_private.getParam(param_ns+"team_size", teamSize);
	nh_private.getParam(param_ns+"global_frame", globalFrame);
	nh_private.getParam(param_ns+"shared_frontier_topic", sharedFrontierTopic);
	nh_private.getParam(param_ns+"cluster_rad_m", clusterRad);
	nh_private.getParam(param_ns+"map_merged_topic", mapMergedTopic);

	float freeAreas,occupiedAreas, totalAreas;
	nh_private.getParam(param_ns+"Free_Area",freeAreas);
	nh_private.getParam(param_ns+"Occupied_Area",occupiedAreas);
	nh_private.getParam(param_ns+"Total_Area",totalAreas);


	std::vector<ros::Subscriber> mapSubs;
	ros::Subscriber centroid_sub = nh.subscribe<fyp_api::centroidArray>(sharedFrontierTopic, 10, centroidCB);
	ros::Publisher centroid_pub = nh.advertise<fyp_api::centroidArray>(sharedFrontierTopic, 1);

	std::vector<float> color = {1,0,0};
	Vis_ns::Vis visualiser(globalFrame, nh_private,color);


	//Envs.reserve(teamSize+1);
	mapSubs.reserve(teamSize+1);

	for (int i = 0; i < teamSize; i++){
		std::string robotName = "robot_"+std::to_string(i+1);
		std::string robotMapFrame = robotName+ "/map";
		envlite env(robotName, robotMapFrame, globalFrame);
		Envs.push_back(env);
		mapSubs[i] = nh.subscribe<nav_msgs::OccupancyGrid>("/"+robotMapFrame, 1, &ogCallBack);
		mapUninitialised.push_back(robotMapFrame);

	}

	ros::Rate rate(1);
	std::vector<fyp_api::centroid> allCentroids;
	std::vector<geometry_msgs::Point> centroidPoints;
	while(ros::ok() && !mapsReceived){
		ros::spinOnce();
	}
	geometry_msgs::Point os;
	os.x = Envs[0].scanOrigin.x + Envs[0].offSet.x;
	os.y = Envs[0].scanOrigin.y + Envs[0].offSet.y;
	visualiser.updateParam(Envs[0].res, os, Envs[0].own2Global);
	while(ros::ok()){

		//Collecting detected centroids and publishing them
		allCentroids.clear();
		centroidPoints.clear();
		for (auto key: teamCentroids){
			for (int i = 0; i < key.second.centroids.size(); i++){
				allCentroids.push_back(key.second.centroids[i]);
			}
		}

		for (int i = 0; i < teamSize; i++){
			auto it = allCentroids.begin();
			while (it != allCentroids.end())
			{

				 std::vector<int> coord = Envs[i].ptToCoord((*it).point);
				 if (Envs[i].inMap(coord[0],coord[1])){
					 if(Envs[i].isFrontier(coord[0], coord[1],it->grad)){
						 if (Envs[i].exclude(coord[0],coord[1]))
						 {
							it = allCentroids.erase(it);
						 }
						 else {
							 ++it;
						 }
					 }
					 else {
						 tosee.push_back(coord);
						it = allCentroids.erase(it);
					 }

				 } else {
					 ++it;
				 }

			 }

		}
		std::vector<fyp_api::centroid> clustered = clusterFrontier(allCentroids);

		/*for (int i = 0; i < teamSize; i++){
			auto it = clustered.begin();
			std::vector<int> coord = Envs[i].ptToCoord((*it).point);

		}*/



		for (auto c: clustered){
			 centroidPoints.push_back(c.point);
		}
		visualiser.visShape(centroidPoints, 1, 0, 0);
		//visualiser.visShape(tosee,1,0,0,0.2);
		visualiser.visPub();
		visualiser.resetVis();
		if (!clustered.empty()){
			fyp_api::centroidArray cArray;
			cArray.robotName = "assigner";
			cArray.centroids = clustered;
			centroid_pub.publish(cArray);
		}


		//Updating on Exploration progress


		//rate.sleep();
		ros::spinOnce();
	}





}
