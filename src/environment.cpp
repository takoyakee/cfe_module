/*
 * environment.cpp
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */
#include <fyp_api/environment.h>
#define UPPERTHRESHOLD 8

namespace Environment_ns {

	Frontier::Frontier(int x_,int y_){
		x = x_;
		y = y_; grad = 0.0f;
		cost = 0.0f, discount = 0.0f, ig = 0.0f, validate = 0.0f, penalty = 0.0f,utility = 0.0f;
	}

	Frontier& Frontier::operator = (const Frontier& f1){
		x = f1.x;
		y = f1.y;
		utility = f1.utility;
		cost = f1.cost;
		discount = f1.discount;
		ig = f1.ig;
		pose = f1.pose;
		validate = f1.validate;
		grad = f1.grad;
		return *this;
	};

	bool Frontier::empty(){
		if (pose.header.frame_id.empty()){
			return true;
		}return false;
	}

	void Frontier::evaluate(float c,float d, float i){
		//previous: weighted sum
		utility = -(c*cost)+(i*ig)-(d*discount);

	}

	std::vector<int> Frontier::key(){
		return {x,y};
	}



	std::vector<std::vector<int>> circleCorners = {{1,1},{1,-1},{-1,1},{-1,-1}};
	std::map<int,std::vector<int>> kernel = {{1,{-1,1}}, {2,{0,1}}, {3,{1,1}},
											{4,{-1,0}}, 		   {6,{1,0}},
											{7,{-1,-1}}, {8,{0,-1}}, {9,{1,-1}} };
	std::map<std::string,std::vector<float>> colorMap = {{"robot_1",{0,1.0,0}},{"robot_2",{0.1,0,1}},{"robot_3",{1,0.6,0}}};

	Environment::Environment(ros::NodeHandle nh_, ros::NodeHandle nh_private):
			planner(NULL),
			poseReceived(false),
			computePotential(false),
			plannerInitialised(false)
	{ // @suppress("Class members should be properly initialized")
		Initialise(nh_, nh_private);
		frontierGrid = NULL;

	}

	Environment::~Environment(){
		deleteGrids();

		if (planner != NULL){
			delete planner;
		}
	}

	void Environment::Initialise(ros::NodeHandle nh_, ros::NodeHandle nh_private){
		if (!EP.loadParameters(nh_private)){
			return;
		}

		if (EP.planType.compare("2D") == 0){
			globalOGreceived = false;
		} else {
			globalOGreceived = true;
		}

		if (!EP.robotMapFrame.empty() && !EP.globalFrame.empty()){
			computeoffSet();
			cpub.robotName = EP.robotName;
			bPath.header.frame_id=EP.globalFrame;
			navPath.header.frame_id = EP.globalFrame;
			color = colorMap[EP.robotName];
			vis = std::unique_ptr<Vis_ns::Vis>(new Vis_ns::Vis(EP.globalFrame, nh_, color));

		}

		poseCloud = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
		frontierCloud = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
		posKDTree = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZI>>();
		frontierKDTree = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZI>>();

		occupancyGrid = std::unique_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);

/*		frontiers_pub = nh_.advertise<visualization_msgs::Marker>("frontier", 50);
		brensenham_pub = nh_.advertise<visualization_msgs::Marker>("bresenham", 50);
		path_pub = nh_.advertise<visualization_msgs::Marker>("navfn_path", 50);
		centroidmarker_pub = nh_.advertise<visualization_msgs::Marker>("centroid",50);*/
		path_pub = nh_.advertise<nav_msgs::Path>("navfn_path", 50);
		brensenham_pub = nh_.advertise<nav_msgs::Path>("bresenham", 50);
		centroid_pub = nh_.advertise<fyp_api::centroidArray>(EP.sharedFrontierTopic, 1);
		centroid_sub = nh_.subscribe<fyp_api::centroidArray>(EP.sharedFrontierTopic, 10, &Environment::centroidCB, this);

		ROS_INFO("FINISHED INITIALISATION");
	}

	bool EnvironmentParameters::loadParameters(ros::NodeHandle nh_private){
		////// 2D //////
		nh_private.getParam("robot_name", robotName);
		nh_private.getParam("robot_map_frame", robotMapFrame);

		std::string param_ns = "/frontier_common/";
		nh_private.getParam(param_ns+"plan_type", planType);
		nh_private.getParam(param_ns+"global_frame", globalFrame);
		nh_private.getParam(param_ns+"shared_frontier_topic",sharedFrontierTopic);
		nh_private.getParam(param_ns+"base_name", baseName);
		nh_private.getParam(param_ns+"search_radius", searchRadius); //to square
		nh_private.getParam(param_ns+"obs_mult", obsMult);
		nh_private.getParam(param_ns+"oob_mult", oobMult);
		nh_private.getParam(param_ns+"c_weight", cweight);
		nh_private.getParam(param_ns+"d_weight", dweight);
		nh_private.getParam(param_ns+"i_weight", iweight);
		nh_private.getParam(param_ns+"frontier_threshold", frontierThreshold);
		nh_private.getParam(param_ns+"cluster_rad", clusterRad);
		nh_private.getParam(param_ns+"cluster_size", clusterSize);
		nh_private.getParam(param_ns+"expiration_time", expirationTime);
		nh_private.getParam(param_ns+"local_range", localRange); //localrange is the xy size of occupancyGrid
		nh_private.getParam(param_ns+"og_resolution", resolution);

		ROS_INFO("SHARED TOPIC: %s", sharedFrontierTopic.c_str());

		localCellRange = localRange/resolution;
		return true;
	}

	/*
	 *  main function that computes costs and discount and returns optimal frontier
	 */
	std::priority_queue<Frontier> Environment::returnFrontiers(){

		frontierCells= getFrontierCells();
		clusterFrontier(frontierCells);
		publishCentroid();
		compileCentroid();
		std::priority_queue<Frontier> pq = evaluateFrontiers();


		vis->visShape(visCentroid, color[0],color[1],color[2],0.3);
		//vis->visShape(navPS, 0.5f, 1.0f, 0,0.1);
		vis->visPub();
		bPath.header.stamp = ros::Time::now();
		navPath.header.stamp = ros::Time::now();
		brensenham_pub.publish(bPath);
		path_pub.publish(navPath);
		resetGrids();
		frontierPQ = pq;
		return pq;

	}

	void Environment::Environment::updateOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_)
	{
		//ROS_INFO("og frame_id: %s vs %s",occupancyGrid->header.frame_id.c_str(), EP.robotMapFrame.c_str());
		if (occupancyGrid->data.size() == 0){
			//ROS_INFO("**** INITIALISING OCCUPANCY GRID ****");
			*occupancyGrid = *occupancyGrid_;
			EP.envWidth = occupancyGrid_->info.width;
			EP.envHeight = occupancyGrid_->info.height;
			EP.resolution = occupancyGrid_->info.resolution;
			EP.maxDist = sqrt(pow(EP.envWidth,2)+pow(EP.envHeight,2));

			initialiseGrids();
		} else {
			//ROS_INFO("**** RESET OCCUPANCY GRlooIDS ****");v
			deleteGrids();
			*occupancyGrid = *occupancyGrid_;
			EP.envWidth = occupancyGrid_->info.width;
			EP.envHeight = occupancyGrid_->info.height;
			EP.resolution = occupancyGrid_->info.resolution;
			EP.maxDist = sqrt(pow(EP.envWidth,2)+pow(EP.envHeight,2));
			initialiseGrids();
		}

		if (vis != NULL){
			scanOrigin = occupancyGrid->info.origin.position;
			geometry_msgs::Point visPt;
			visPt.x = scanOrigin.x;
			visPt.y = scanOrigin.y;
			vis->updateParam(EP.resolution, visPt, own2Global);
		}
	}


	std::vector<std::vector<int>> Environment::getFrontierCells(){
		std::vector<std::vector<int>> prevFrontierCells;
		int frontierCount = 0;	//gives position of coord in prevFrontierCells
		for (int i = 0; i < EP.envWidth; i++){
			for (int j = 0; j < EP.envHeight; j++){

				if (occupancyGrid->data[coordToIndex(i, j)] == -1 ){
					edge cell = isFrontier(i,j);
					if (cell.isEdge){
						frontierGrid[i][j] = frontierCount;
						grads[{i,j}] = cell.gradient;
						prevFrontierCells.push_back({i,j});
						frontierCount ++;
					}
				}
				if (occupancyGrid->data[coordToIndex(i, j)] > 0) {
				}
			}
		}

		//vis->visShape(prevFrontierCells, color[0], color[1], color[2],0.2);
		return prevFrontierCells;

	}

	/*
	 * Checks for connected frontiers and returns centroid of each region
	 * More accurately: Voronoi Partition
	 */
	void Environment::clusterFrontier(std::vector<std::vector<int>> frontierCells){

		std::map<int,std::vector<int>> frontierMap;
		for (int i = 0; i < frontierCells.size(); i ++){
			frontierMap[i] = frontierCells[i];
		}

		int** visitedGrid_;
		visitedGrid_ = new int*[EP.envWidth];
		for (int i = 0; i < EP.envWidth; i++){
			visitedGrid_[i] = new int[EP.envHeight];
			for (int j = 0; j < EP.envHeight; j++){
				visitedGrid_[i][j] = 0;
			}
		}

		int maxClusterSize = 0;
		std::map<std::vector<int>, fyp_api::centroid> clusterSizeMap;

		geometry_msgs::PoseArray poses;
		tf2::Quaternion q;


		while (!frontierMap.empty()){
			int totalx = 0 , totaly = 0;
			float totalgrad = 0;

			std::vector<std::vector<int>> toVisitList;
			std::vector<std::vector<int>> cluster;

			toVisitList.push_back(frontierMap.begin()->second);
			frontierMap.erase(frontierMap.begin()->first);

			while (!toVisitList.empty()){
				std::vector<int> v = toVisitList.back();

				//"Visit" v's neighbors and add to toVisitList if they are frontiers 	//hyper parameter
				int lowerx = std::max(1,v[0]-EP.clusterRad); int upperx=std::min(EP.envWidth-2, v[0]+EP.clusterRad);
				int lowery = std::max(1,v[1]-EP.clusterRad); int uppery=std::min(EP.envHeight-2, v[1]+EP.clusterRad);
				for (int i = lowerx; i < upperx; i++){
					for (int j = lowery; j < uppery; j++){
						int nx_ = i; int ny_ = j;
						if (visitedGrid_[nx_][ny_] == 0){
							//Connected!
							if (frontierGrid[nx_][ny_] != -1){
								totalx += nx_;
								totaly += ny_;
								totalgrad += grads[{nx_,ny_}];
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

			if (cluster.size() > EP.clusterSize){
				int x = (int)(totalx/cluster.size());
				int y = (int)(totaly/cluster.size());
				findNearestUnknown(x, y, totalgrad/cluster.size());

				fyp_api::centroid c;
				c.ig = cluster.size();
				c.grad = totalgrad/cluster.size();
				clusterSizeMap[{x,y}] = c;
				std::vector<float> pt = coordToPoint(x,y);


/*				geometry_msgs::Pose p;
				p.position.x = pt[0];
				p.position.y = pt[1];
				q.setEuler(0, 0, totalgrad/cluster.size());
				p.orientation.w = q.getW();
				p.orientation.x = q.getX();
				p.orientation.y = q.getY();
				p.orientation.z = q.getZ();

				poses.poses.push_back(p);*/

				//ROS_INFO("AVG GRADIENTS: %.3f", totalgrad/cluster.size());
			}

			maxClusterSize = std::fmax(maxClusterSize, cluster.size());
		}

		EP.numCluster = ownCentroid.size();

		for (auto it: clusterSizeMap){
			Frontier centroid{it.first[0],it.first[1]};
			centroid.ig = (float)(it.second.ig)/maxClusterSize;
			centroid.grad = it.second.grad;
			insert(ownCentroid,centroid);
		}
		std::vector<std::vector<int>> visOwnCentroid;
		for (auto k : ownCentroid){
			visOwnCentroid.push_back(k.first);
		}
		vis->visShape(visOwnCentroid, color[0], color[1], color[2],0.2);

		for (int i = 0; i < EP.envWidth; i ++){
			delete[] visitedGrid_[i];
		}
		delete[] visitedGrid_;

		//vis->visPose(poses);
	}

	std::priority_queue<Frontier> Environment::evaluateFrontiers(){
		std::vector<std::vector<int>>tmpcoord;
		std::priority_queue<Frontier> pq;
		for (auto key : finalCentroid){
			//for visualisation
			visCentroid.push_back(key.first);
			Frontier centroid = key.second;

			std::map<std::vector<int>,double> tmap = cellValidate(key.first[0],  key.first[1]);
			centroid.discount = cellDiscount(key.first[0],  key.first[1]);
			if (localFrontier.size() > 0){
				centroid.validate = tmap.begin()->second ;
				std::vector<int> newCoord;
				if (centroid.validate == 0){
					newCoord = interpolateCoord(key.first);
				} else {
					newCoord = {tmap.begin()->first[0],tmap.begin()->first[1]};
				}
				centroid.pose = coordToPS(newCoord[0], newCoord[1]);
				tmpcoord.push_back({newCoord[0],newCoord[1]});
			}

			centroid.evaluate(EP.cweight, EP.dweight, EP.iweight);
			pq.push(centroid);
		}


		//visualiseCells(tmpcoord, 1.0, 0.0f, 1.0, obs_pub);
		return pq;
	}


	void Environment::updateRobotPose(geometry_msgs::PoseStamped currentPose_){
		currentPose = currentPose_;
		geometry_msgs::Point pt;
		pt.x = currentPose.pose.position.x;
		pt.y = currentPose.pose.position.y;
		currPt = pt;
		currPos = {currentPose.pose.position.x, currentPose.pose.position.y};
		currCoord = pointToCoord(currPos[0], currPos[1]);
		if (!poseReceived){
			poseReceived = true;
		}
		//ROS_INFO("curr pos: %f,%f, coord: %d,%d", currPos[0],currPos[1], currCoord[0], currCoord[1]);
	}


	void Environment::updateTeamGoalPose(std::map<std::string, geometry_msgs::PoseStamped> teamGoalPose_){
		std::vector<geometry_msgs::PoseStamped> poseVector;
		for(auto& pose: teamGoalPose_) {
			poseVector.push_back(pose.second);
		}
		teamGoalPose = poseVector;
		//vis->visShape(teamGoalPose, color[0],color[1],color[2]);
		vis->visPub();
		//ROS_INFO("TGP SIZE: %d", teamGoalPose.size());
	}



	void Environment::updateRobotTeamPose(std::map<std::string, geometry_msgs::PoseStamped> teamPose_){
		std::vector<geometry_msgs::PoseStamped> poseVector;
		teamPose.clear();
		pose_weight.clear();
		for(auto& pose: teamPose_) {
			teamPose.push_back(pose.second);
			double poseweight = std::fmin(1.0,(norm(pose.second,currentPose))/(EP.searchRadius/4));
			pose_weight.push_back(poseweight);
			//ROS_INFO("pose weight: %.3f",poseweight);
		}
	}

	void Environment::updateLocalFrontierCloud(std::vector<Eigen::Vector3d> localFrontier_){
		//ROS_INFO("Received local frontier!");
		std::vector<std::vector<int>> temp;
		for (auto lf: localFrontier_){
			std::vector<int> coord = pointToCoord(lf.x(),lf.y());
			temp.push_back(coord);
		}
		localFrontier = temp;

	}

	void Environment::addEnvPosVertex(Eigen::Vector3d pos){
		pcl::PointXYZI pt;
		pt.x = pos.x();
		pt.y = pos.y();
		pt.z = pos.z();
		pt.intensity = 0;
		poseCloud->points.push_back(pt);
		posKDTree->setInputCloud(poseCloud);

	}

	void Environment::addEnvFrontierVertex(Eigen::Vector3d pos){
		pcl::PointXYZI pt;
		pt.x = pos.x();
		pt.y = pos.y();
		pt.z = pos.z();
		pt.intensity = 0;
		if (!frontierCloud->points.empty()){
			//obtain closest selected frontier point;
			int K = 2;
			std::vector<int> pointIdx(K);
			std::vector<float> pointDist(K);
			if ( frontierKDTree->nearestKSearch(pt, K, pointIdx, pointDist) > 0 )
			{
				for (std::size_t i = 0; i < pointIdx.size (); ++i){
					float oldIntensity = (*frontierCloud)[pointIdx[i]].intensity;
					(*frontierCloud)[pointIdx[i]].intensity = oldIntensity + (1-oldIntensity)*(pointDist[i]/EP.localRange);
				}
			}

		}
		frontierCloud->points.push_back(pt);
		//updatefrontiercloud values
		frontierKDTree->setInputCloud(frontierCloud);

		for (auto pt: frontierCloud->points){
			ROS_INFO("pt (%f,%f,%f) has intensity %f", pt.x,pt.y,pt.z,pt.intensity);
		}


		sensor_msgs::PointCloud2 scan_data;
		pcl::toROSMsg(*frontierCloud, scan_data);
		scan_data.header.stamp = ros::Time::now();
		scan_data.header.frame_id = EP.globalFrame;
		clusteredpub.publish(scan_data);

	}

	std::vector<int> Environment::interpolateCoord(std::vector<int> targetCell){
		int diffx = targetCell[0]-currCoord[0];
		int diffy = targetCell[1]-currCoord[1];
		double angle = atan2(diffy,diffx);
		int newx = (int) (cos(angle)*EP.localCellRange);
		int newy = (int) (sin(angle)*EP.localCellRange);
		return {currCoord[0]+newx,currCoord[1]+newy};
	}


	double Environment::cellCost(int cx_, int cy_){
		int navObs = 0, navOob = 0; float totalCost = 0.0f;
		geometry_msgs::PoseStamped endPS = coordToPS(cx_,cy_);
		std::vector<geometry_msgs::PoseStamped> plan;
		if (planner->makePlan(currentPose, endPS, plan)){
			for (auto pose : plan){
				std::vector<int> coord = pointToCoord(pose.pose.position.x, pose.pose.position.y);
				if (inMap(coord[0],coord[1])){
					int idx = coordToIndex(coord[0], coord[1]);
					if (occupancyGrid->data[idx] != 0){
						navObs++;
					}
				} else {
					navOob++;
				}
			}
			totalCost = plan.size() + EP.obsMult*navObs;
		} else {

			std::vector<std::vector<int>> visitedCell;
			std::vector<int> goalCoord = {cx_,cy_};
			bresenham2D(currCoord,goalCoord,visitedCell);
			for (auto coord: visitedCell){
				if (inMap(coord[0],coord[1])){
					int idx = coordToIndex(coord[0], coord[1]);
					if (occupancyGrid->data[idx] != 0){
						navObs++;
						if (occupancyGrid->data[idx] > 0){
							navObs ++;
						}
					}
				} else {
					navOob++;
				}
			}
			//ROS_INFO("OOB/TOTAL: (%d/%d)", navOob, visitedCell.size());
			totalCost = visitedCell.size() + EP.obsMult*navObs + EP.oobMult*navOob;
		}
		return std::fmin(1.0, (totalCost/EP.maxDist));
	}

	double Environment::cellDiscount (int cx_, int cy_){
		//Discount frontiers that are in other robots range
		std::vector<float> cellPt = coordToPoint(cx_, cy_);
		float goalDC = 0.0f, poseDC = 0.0f;
		for (auto rGoal : teamGoalPose){
			float dist = norm(cellPt,rGoal);
			if (dist < EP.searchRadius){
				float oldgoalDC = goalDC;
				goalDC = goalDC + (1-goalDC)*(1-(dist/EP.searchRadius));
			}
		}
		for (int i = 0; i < teamPose.size(); i++){
			float dist = norm(cellPt,teamPose[i]);
			if (dist < EP.searchRadius){
				poseDC = poseDC + (1-poseDC)*(1-(dist/EP.searchRadius));
				poseDC *= pose_weight[i];
			}
		}
		/*
		if (!EP.robotName.compare("robot_1")){
			ROS_INFO("(%.3f,%.3f): GoalDC: %.3f, PoseDC: %.3f, FINAL: %.3f", cellPt[0], cellPt[1],
					goalDC, poseDC,std::max(goalDC,poseDC)+ (1-std::max(goalDC,poseDC))*std::min(goalDC,poseDC));
		}*/


		return std::max(goalDC,poseDC)+ (1-std::max(goalDC,poseDC))*std::min(goalDC,poseDC);
	}

	std::map<std::vector<int>, double> Environment::cellValidate(int cx_, int cy_){
		std::vector<int> frontierCoord = {cx_,cy_};
		float dist = norm(currCoord,frontierCoord);
		//ROS_INFO("(%d,%d) dist from robot: %f", cx_,cy_, dist);
		std::map<std::vector<int>, double> returnmap;
		bool far = false;
		if (dist > EP.localCellRange || localFrontier.size() == 0) far = true;
		float disparity = DBL_MAX;
		std::vector<int> closest;
		for (auto lf: localFrontier){
			if (norm(lf,frontierCoord) < disparity){
				disparity = norm(lf,frontierCoord);
				closest = lf;
			}
		}
		if (!far) ROS_INFO("validate: %f", 1 - (disparity/EP.localCellRange));
		returnmap[closest] = far ? 0 : 1 - (disparity/EP.localCellRange);
		return returnmap;
	}

	double Environment::cellPenalty(int cx_, int cy_){
		std::vector<float> pos = coordToPoint(cx_, cy_);
		pcl::PointXYZI pt;
		pt.x = pos[0];
		pt.y = pos[1];
		pt.z = 0;
		pt.intensity = 0;

		int K = 1;
		std::vector<int> pointIdx(K);
		std::vector<float> pointDist(K);
		if (!frontierCloud->points.empty()){
			if (frontierKDTree->nearestKSearch(pt, K, pointIdx, pointDist) > 0 )
			{
				for (std::size_t i = 0; i < pointIdx.size (); ++i){
				float oldIntensity = (*frontierCloud)[pointIdx[i]].intensity;
				return oldIntensity*std::fmin(1.0,pointDist[i]/EP.localRange);
				}
			}
		}

		return 0;
	}


	void Environment::Environment::initialiseGrids(){
		//ROS_INFO("Initialising grids of (%d x %d)", EP.envWidth, EP.envHeight);
		frontierGrid = new int* [EP.envWidth];
		for(int i = 0; i < EP.envWidth; i++) {
			frontierGrid[i] = new int[EP.envHeight];
			for (int j=0; j < EP.envHeight; j++){
				frontierGrid[i][j] = -1;
			}
		}

	}

	void Environment::deleteGrids(){
		for(int i = 0; i < EP.envWidth; i++) {
			delete[] frontierGrid[i];
		}
		delete[] frontierGrid;

		frontierGrid = NULL;
	}

	void Environment::resetGrids(){
		for(int i = 0; i < EP.envWidth; i++) {
			for(int j = 0; j< EP.envHeight; j++){
				frontierGrid[i][j] = -1;
			}
		}
		while(!frontierPQ.empty()){
			frontierPQ.pop();
		}


		frontierCells.clear();
		ownCentroid.clear();
		finalCentroid.clear();
		visCentroid.clear();
		grads.clear();

		navPath.poses.clear();
		bPath.poses.clear();
		badCoord.clear();

		vis->resetVis();

	}

	/*
	 * Finds cost of centroid, if centroid is out of map, return nearest point
	 */
	int record = 1;

	bool Environment::findFeasibleCentroid(Frontier& frontier){
		float navObs = 0, navOob = 0, free = 0; float totalCost = 0.0f;
		if (!inMap(frontier.x, frontier.y)){
			findInMap(frontier);
		}

		//TODO:I need to find a free cell
		geometry_msgs::PoseStamped endPS = coordToPS(frontier.x,frontier.y);
		std::vector<geometry_msgs::PoseStamped> plan;
		if (planner->makePlan(currentPose, endPS, plan)){
			std::vector<int> prev = pointToCoord(plan[0].pose.position.x, plan[0].pose.position.y);
			int p = 0;
			for (auto pose : plan){
				std::vector<int> coord = pointToCoord(pose.pose.position.x, pose.pose.position.y);
				if (inMap(coord[0],coord[1])){
					int idx = coordToIndex(coord[0], coord[1]);
					if (occupancyGrid->data[idx] != 0){
						navObs += norm(prev,coord);
					} else {
						free += norm(prev,coord);
					}
				} else {
					navOob += norm(prev,coord);
				}

				prev = coord;

			}
			frontier.cost = std::fmin(1.0,(free + EP.obsMult*navObs + EP.oobMult*navOob)/EP.maxDist);
			frontier.pose = coordToPS(frontier.x,frontier.y);
			return true;


			//if (!EP.robotName.compare("robot_1")) ROS_INFO("COSTS: %.0f obs, %.0f oob, %.0f free vs total: %d", navObs, navOob,free, plan.size());

		} else {
/*			std::vector<std::vector<int>> visitedCell;
			std::vector<int> goalCoord = frontier.key();
			inaccessible.push_back(goalCoord);
			bresenham2D(currCoord,goalCoord,visitedCell);
			for (auto coord: visitedCell){
				bPath.poses.push_back(coordToPS(coord[0], coord[1]));

				if (inMap(coord[0],coord[1])){
					int idx = coordToIndex(coord[0], coord[1]);
					if (occupancyGrid->data[idx] != 0){
						navObs++;
						if (occupancyGrid->data[idx] > 0){
							navObs ++;
						}
					}
				} else {
					navOob++;
				}
			}
			ROS_INFO("OOB/TOTAL: (%d/%d)", navOob, visitedCell.size());
			frontier.cost = std::m(1,(visitedCell.size() + EP.obsMult*navObs + EP.oobMult*navOob)/EP.maxDist);
			frontier.pose = coordToPS(frontier.x, frontier.y);
			ROS_INFO("OH no");*/
			return false;

		}

	}
	/*
	 * Finding the closest side of the map
	 */
	void Environment::findInMap(Frontier& frontier){

		double dist1, dist2, dist3, dist4;
		dist1 = frontier.y;
		dist2 = frontier.x;
		dist3 = EP.envWidth -1 - frontier.x;
		dist4 = EP.envHeight -1  - frontier.y;
		double minDist = std::min(std::min(std::min(dist1,dist2),dist3),dist4);
		int diffx = frontier.x-currCoord[0];
		int diffy = frontier.y-currCoord[1];
		double m,c;
		if (diffx==0 && diffy == 0) return;
		//x = constant;
		if (diffx == 0){
			frontier.y = 0 ? (dist1<dist4): EP.envHeight-1;
		} else if (diffy == 0){
			frontier.x = 0 ? (dist2<dist3): EP.envWidth-1;
		} else {
			m = (float)(diffy)/diffx;
			c = (currCoord[1] - m*(currCoord[0]));
			//y = mx +c
			// y = 0
			if (dist1 == minDist){
				frontier.y = 0;
				frontier.x = (int)(-c/m);
			} else if (dist2 == minDist){
				frontier.x = 0;
				frontier.y = (int)c;
			} else if (dist3 == minDist){
				frontier.x = EP.envWidth-1;
				frontier.y = (int)((m*(EP.envWidth-1))+c);
			} else {
				frontier.y = EP.envHeight-1;
				frontier.x = (int)(((EP.envHeight-1)-c)/m);
			}
		}
		if (!inMap(frontier.x,frontier.y)){
			ROS_ERROR("SMTH WENT WRONG");
		}

	}


	/*
	 * Converts coord (discountGrid/valueGrid) to index (OccupancyGrid)
	 */
	int Environment::Environment::coordToIndex(int cx_, int cy_){
		return cy_*EP.envWidth + cx_;
	}


	//Point should be in global frame
	std::vector<float> Environment::coordToPoint(int cx_, int cy_){
		float originx_ = scanOrigin.x;
		float originy_ = scanOrigin.y;
		geometry_msgs::Pose own, global;
		own.position.x = originx_+(cx_*EP.resolution);
		own.position.y = originy_+(cy_*EP.resolution);
		own.position.z = 0;
		//own.orientation.w = 0;
		//own.orientation.x = 0;
		//own.orientation.y = 0;
		//own.orientation.z = 1;
		tf2::doTransform(own, global, own2Global);
		return {global.position.x, global.position.y};
	}


	geometry_msgs::PoseStamped Environment::coordToPS(int cx_, int cy_){
		geometry_msgs::PoseStamped poseStamped;
		std::vector<float> pt = coordToPoint(cx_,cy_);
		poseStamped.pose.position.x = pt[0];
		poseStamped.pose.position.y = pt[1];
		poseStamped.pose.orientation.w = 1;
		poseStamped.header.frame_id = EP.globalFrame;
		poseStamped.header.stamp = ros::Time(0);

		return poseStamped;
	}

	/*
	 * mx_ : actual x coordinate of point in map frame
	 * my_ : actual y coordinate of point in map frame
	 * need to be calculated wrt origin
	 */
	std::vector<int> Environment::Environment::pointToCoord(float mx_, float my_){
		geometry_msgs::Pose globalPose, ownPose;
		globalPose.position.x = mx_;
		globalPose.position.y = my_;
		globalPose.position.z = 0;
		//globalPose.orientation.w = 0;
		//globalPose.orientation.x = 0;
		//globalPose.orientation.y = 0;
		//globalPose.orientation.z = 1;

		tf2::doTransform(globalPose, ownPose, global2Own);

		//get index then convert to Coord?
		float originx = scanOrigin.x;
		float originy = scanOrigin.y;


		return {(int)(floor((ownPose.position.x-originx)/EP.resolution)),(int)(floor((ownPose.position.y-originy)/EP.resolution))};

		//get index then convert to Coord?
		//float originx = scanOrigin.x+offSet.x;
		//float originy = scanOrigin.y+offSet.y;
		//return {(int)(floor((mx_-originx)/EP.resolution)),(int)(floor((my_-originy)/EP.resolution))};

	}

	/*
	 * Converts index (Occupancy Grid) to coord (discountGrid/valueGrid)
	 */
	std::vector<int> Environment::Environment::indexToCoord(int index_){
		return {(int)(index_%EP.envWidth), (int)(floor(index_/EP.envWidth))};
	}

	bool Environment::isEnvInitialised(){
		return occupancyGrid->data.size() > 0 &&  poseReceived && plannerInitialised;
	}

	edge Environment::isFrontier(int cx_, int cy_){
		// only perform convolution on non edge cells
		edge cell;
		if (isEdge(cx_, cy_)){
			return cell;
		}

		cell = edgeGrad(cx_, cy_);
		return cell;
	}

	bool Environment::inMap(int cx_, int cy_){
		return (cx_ >= 0 && cx_ <= EP.envWidth-1 && cy_ >= 0 && cy_ <= EP.envHeight-1);
	}

	bool Environment::isEdge(int cx_, int cy_){
		return (cx_ <= 0 || cx_ >= EP.envWidth-1 || cy_ <= 0 || cy_ >= EP.envHeight-1);
	}

	bool Environment::isFree(int cx_, int cy_){
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

		int sum = occupancyGrid->data[idxTL] + occupancyGrid->data[idxT] + occupancyGrid->data[idxTR]
				+ occupancyGrid->data[idxL] + occupancyGrid->data[idxR] + occupancyGrid->data[idxBL]
				+ occupancyGrid->data[idxB] + occupancyGrid->data[idxBR] ;

		if (sum > 100 ) {
			return false;
		}
		return true;
	}



	edge Environment::edgeGrad(int cx_, int cy_){
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

		gradx = -1 * cap(occupancyGrid->data[idxTL]) + cap(occupancyGrid->data[idxTR]);
		gradx += -1 * cap(occupancyGrid->data[idxBL]) + cap(occupancyGrid->data[idxBR]);
		gradx += -2*cap(occupancyGrid->data[idxL]) + 2* cap(occupancyGrid->data[idxR]);

		grady = -1 * cap(occupancyGrid->data[idxBL]) + cap(occupancyGrid->data[idxTL]);
		grady += -1 * cap(occupancyGrid->data[idxBR]) + cap(occupancyGrid->data[idxTR]);
		grady += - 2* cap(occupancyGrid->data[idxB]) + 2* cap(occupancyGrid->data[idxT]);

		edge newEdge;
		newEdge.isEdge = true ? ((abs(gradx)+abs(grady)) >= EP.frontierThreshold) && (abs(gradx)+abs(grady) <= UPPERTHRESHOLD) : false;
		newEdge.gradient = atan2(grady,gradx);
		return newEdge;

	}

	int Environment::cap(int value_){
		if (value_ < 0){
			return -value_;
		}
		return value_;
	}



	////2D PLANNING////
	void Environment::updateGlobalOrigin(const nav_msgs::OccupancyGrid::ConstPtr& globalOG_){
		if (!occupancyGrid->data.empty()){
			if (!globalOGreceived){
				globalOGreceived = true;
			}
		}
	}

	void Environment::computeoffSet(){
		//Convert received posestamped to global frame if not already in global frame
		tf2_ros::Buffer buffer;
		tf2_ros::TransformListener tfl(buffer);
		geometry_msgs::TransformStamped transformStamped;
		try {
			//			transformStamped = buffer.lookupTransform(EP.globalFrame, EP.robotMapFrame, ros::Time(0), ros::Duration(3.0));

			transformStamped = buffer.lookupTransform(EP.globalFrame, EP.globalFrame, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException &e) {
			ROS_WARN("Error with transform");
		}
		offSet.x = transformStamped.transform.translation.x;
		offSet.y = transformStamped.transform.translation.y;
		own2Global = transformStamped;
		//ROS_INFO("own2global: %.3f,%.3f", transformStamped.transform.translation.x,transformStamped.transform.translation.y);

		try {
			//			transformStamped = buffer.lookupTransform(EP.robotMapFrame, EP.globalFrame,ros::Time(0), ros::Duration(3.0));

			transformStamped = buffer.lookupTransform( EP.globalFrame,EP.globalFrame,ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException &e) {
			ROS_WARN("Error with transform");
		}
		global2Own = transformStamped;

		//ROS_INFO("global2own: %.3f,%.3f", transformStamped.transform.translation.x,transformStamped.transform.translation.y);
	}

	void Environment::centroidCB(fyp_api::centroidArray c){
		if (c.robotName.compare("assigner") == 0){
			assigned = c;
		}

	}

	void Environment::publishCentroid(){
		fyp_api::centroid c;
		for (auto it: ownCentroid){
			c.point = coordToPt(it.first[0],it.first[1]);
			c.ig = it.second.ig;
			c.grad = it.second.grad;
			cpub.centroids.push_back(c);
		}

		centroid_pub.publish(cpub);
		cpub.centroids.clear();

	}

	geometry_msgs::Point Environment::coordToPt(int cx_, int cy_){
		geometry_msgs::Point pt;
		std::vector<float> point = coordToPoint(cx_,cy_);
		pt.x = point[0];
		pt.y = point[1];
		return pt;
	}




	void Environment::compileCentroid(){
		int arraySize = assigned.centroids.size();
		for (int i = 0; i < arraySize; i++){
			std::vector<int> coord = pointToCoord(assigned.centroids[i].point.x, assigned.centroids[i].point.y);
			Frontier centroid{coord[0],coord[1]};
			//need better frontier verification
			if (!inMap(coord[0],coord[1])) {
				if (findFeasibleCentroid(centroid)){
					centroid.ig = assigned.centroids[i].ig;
					insert(finalCentroid,centroid);
				}
			}
			else {
				if (isFrontier(coord[0],coord[1]).isEdge || !isKnown(coord[0], coord[1]) ){
					if (findFeasibleCentroid(centroid)){
						centroid.ig = assigned.centroids[i].ig;
						insert(finalCentroid,centroid);
					}
				}
			}
		}

		if (arraySize == 0){
			for (auto key: ownCentroid){
				Frontier centroid = key.second;
				if (findFeasibleCentroid(centroid)){
					insert(finalCentroid,centroid);
				}
			}
		}


	}

	bool Environment::isKnown(int cx_, int cy_){
		kcells i;
		int idxTL = coordToIndex(kernel[i=TL][0]+cx_, kernel[i=TL][1]+cy_);
		int idxT = coordToIndex(kernel[i=T][0]+cx_, kernel[i=T][1]+cy_);
		int idxTR = coordToIndex(kernel[i=TR][0]+cx_, kernel[i=TR][1]+cy_);
		int idxL = coordToIndex(kernel[i=L][0]+cx_, kernel[i=L][1]+cy_);
		int idxR = coordToIndex(kernel[i=R][0]+cx_, kernel[i=R][1]+cy_);
		int idxBL = coordToIndex(kernel[i=BL][0]+cx_, kernel[i=BL][1]+cy_);
		int idxB = coordToIndex(kernel[i=B][0]+cx_, kernel[i=B][1]+cy_);
		int idxBR = coordToIndex(kernel[i=BR][0]+cx_, kernel[i=BR][1]+cy_);

		int sum = 0;
		sum += (occupancyGrid->data[idxTL] == -1);
		sum += (occupancyGrid->data[idxT] == -1);
		sum += (occupancyGrid->data[idxTR] == -1);
		sum += (occupancyGrid->data[idxL] == -1);
		sum += (occupancyGrid->data[idxR] == -1);
		sum += (occupancyGrid->data[idxBL] == -1);
		sum += (occupancyGrid->data[idxB] == -1);
		sum += (occupancyGrid->data[idxBR] == -1);

		if (sum > 5 ) {
			// had to be increased due to conversion errors (cell to point, point to cell)
			return false;
		}
		return true;
	}


	void Environment::setPlanner(navfn::NavfnROS* planner_){
		planner = planner_;
		plannerInitialised = true;
	}

	void Environment::findNearestUnknown(int& cx_, int& cy_, float angle){
		float rawx, rawy;
		rawx = cx_;
		rawy = cy_;
		int count = 0;
		while (inMap(cx_,cy_)){
			if (!isKnown(cx_,cy_)){
				//ROS_INFO("STEPS TAKEN: %d", count);
				return;
			}
			rawx += (cos(angle));
			rawy += (sin(angle));
			cx_ = (int) rawx;
			cy_ = (int) rawy;
			count ++;
		}
		return ;

	}

	void Environment::insert(std::map<std::vector<int>, Frontier>& dict, Frontier elem){
		if (dict.find(elem.key()) == dict.end()){
			dict.insert(std::pair<std::vector<int>, Frontier>(elem.key(),elem));
		} else {
			dict.at(elem.key()) = elem;
		}

	}


}

