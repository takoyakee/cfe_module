/*
 * environment.cpp
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */
#include <fyp_api/environment.h>
#define UPPERTHRESHOLD 8
namespace Environment_ns {

	std::vector<std::vector<int>> circleCorners = {{1,1},{1,-1},{-1,1},{-1,-1}};
	std::map<int,std::vector<int>> kernel = {{1,{-1,1}}, {2,{0,1}}, {3,{1,1}},
											{4,{-1,0}}, 		   {6,{1,0}},
											{7,{-1,-1}}, {8,{0,-1}}, {9,{1,-1}} };

	Frontier::Frontier(int x_,int y_, double utility_){
		x = x_;
		y = y_;
		utility = utility_;
		cost = 0.0f, discount = 0.0f, ig = 0.0f, validate = 0.0f, penalty = 0.0f;
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
		return *this;
	};

	bool Frontier::empty(){
		if (pose.header.frame_id.empty()){
			return true;
		}return false;
	}

	void Frontier::evaluate(float c,float d, float i){
		utility = (i*ig)-(c*cost)-(d*discount) + d*validate;
	}


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


		computeoffSet();
		cpub.robotName = EP.robotName;


		poseCloud = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
		frontierCloud = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
		posKDTree = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZI>>();
		frontierKDTree = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZI>>();

		occupancyGrid = std::unique_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);
		vis = std::unique_ptr<Vis_ns::Vis>(new Vis_ns::Vis(EP.globalFrame));

/*		if (!EP.costmap_topic.empty()){
			ROS_INFO("before costmap");
			costMap = new costmap_2d::Costmap2DROS(std::string("fcostmap"), TF_buffer);
			ROS_INFO("after costmap");
			//planner = new navfn::NavfnROS("explore_planner", costMap);
		}*/

		frontiers_pub = nh_.advertise<visualization_msgs::Marker>("frontier", 50);
		centroid_pub = nh_.advertise<fyp_api::centroidArray>(EP.sharedFrontierTopic, 1);
		centroid_sub = nh_.subscribe<fyp_api::centroidArray>(EP.sharedFrontierTopic, 10, &Environment::centroidCB, this);
		brensenham_pub = nh_.advertise<visualization_msgs::Marker>("bresenham", 50);
		path_pub = nh_.advertise<visualization_msgs::Marker>("navfn_path", 50);
		centroidmarker_pub = nh_.advertise<visualization_msgs::Marker>("centroid",50);

	}

	bool EnvironmentParameters::loadParameters(ros::NodeHandle nh_private){
		////// 2D //////
		nh_private.getParam("plan_type", planType);
		nh_private.getParam("costmap_topic",costmap_topic);
		nh_private.getParam("robot_map_frame", robotMapFrame);
		nh_private.getParam("global_frame", globalFrame);
		nh_private.getParam("robot_name", robotName);
		nh_private.getParam("shared_frontier_topic",sharedFrontierTopic);
		nh_private.getParam("base_name", baseName);
		nh_private.getParam("search_radius", searchRadius); //to square
		nh_private.getParam("obs_mult", obsMult);
		nh_private.getParam("c_weight", cweight);
		nh_private.getParam("d_weight", dweight);
		nh_private.getParam("i_weight", iweight);
		nh_private.getParam("frontier_threshold", frontierThreshold);
		nh_private.getParam("cluster_rad", clusterRad);
		nh_private.getParam("cluster_size", clusterSize);
		nh_private.getParam("expiration_time", expirationTime);
		nh_private.getParam("local_range", localRange); //localrange is the xy size of occupancyGrid
		nh_private.getParam("og_resolution", resolution);

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
		std::vector<std::vector<int>> tmp;
		tmp.push_back(pointToCoord(0, -0.8));
		centroidmarker_pub.publish(vis->visMarker(centroidVector, 0, 0, 1));
		brensenham_pub.publish(vis->visMarker(pathCoord,0.5f,0.5f,0,0.1));
		path_pub.publish(vis->visMarker(navPS, 1.0f, 1.0f, 1.0f,0.1));
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
			EP.maxDist = sqrt(pow(EP.envWidth,2)+ pow(EP.envHeight,2));

			initialiseGrids();
		} else {
			//ROS_INFO("**** RESET OCCUPANCY GRlooIDS ****");v
			deleteGrids();
			*occupancyGrid = *occupancyGrid_;
			EP.envWidth = occupancyGrid_->info.width;
			EP.envHeight = occupancyGrid_->info.height;
			EP.resolution = occupancyGrid_->info.resolution;
			EP.maxDist = sqrt(pow(EP.envWidth,2)+ pow(EP.envHeight,2));
			ROS_INFO("%d + %d = %d",EP.envWidth, EP.envHeight,EP.maxDist);
			initialiseGrids();
		}
		scanOrigin = occupancyGrid->info.origin.position;
		geometry_msgs::Point visPt;
		visPt.x = scanOrigin.x + offSet.x;
		visPt.y = scanOrigin.y + offSet.y;
		vis->updateParam(EP.resolution, visPt);
	}


	std::vector<std::vector<int>> Environment::getFrontierCells(){
		std::vector<std::vector<int>> prevFrontierCells;
		int frontierCount = 0;	//gives position of coord in prevFrontierCells
		for (int i = 0; i < EP.envWidth; i++){
			for (int j = 0; j < EP.envHeight; j++){
				if (occupancyGrid->data[coordToIndex(i, j)] == -1 ){
					if (isFrontier(i,j)){
						frontierGrid[i][j] = frontierCount;
						prevFrontierCells.push_back({i,j});
						frontierCount ++;
					}
				}
				if (occupancyGrid->data[coordToIndex(i, j)] > 0) {
				}
			}
		}

		frontiers_pub.publish(vis->visMarker(prevFrontierCells, 0.0f, 1.0f, 0.0f, 0.2));
		return prevFrontierCells;

	}

	/*
	 * Checks for connected frontiers and returns centroid of each region
	 * More accurately: Voronoi Partition
	 */
	std::map<std::vector<int>,float> Environment::clusterFrontier(std::vector<std::vector<int>> frontierCells){

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
		while (!frontierMap.empty()){
			int totalx = 0 , totaly = 0;

			std::vector<std::vector<int>> toVisitList;
			std::vector<std::vector<int>> cluster;

			toVisitList.push_back(frontierMap.begin()->second);
			frontierMap.erase(frontierMap.begin()->first);

			while (!toVisitList.empty()){
				std::vector<int> v = toVisitList.back();

				//"Visit" v's neighbors and add to toVisitList if they are frontiers 	//hyper parameter
				int lowerx = MAX(1,v[0]-EP.clusterRad); int upperx=MIN(EP.envWidth-2, v[0]+EP.clusterRad);
				int lowery = MAX(1,v[1]-EP.clusterRad); int uppery=MIN(EP.envHeight-2, v[1]+EP.clusterRad);
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

			//ROS_INFO("Cluster size: %d", cluster.size());
			if (cluster.size() > EP.clusterSize){
				int centroidx = (int)(totalx/cluster.size());
				int centroidy = (int)(totaly/cluster.size());
				centroidMap[{centroidx,centroidy}] = cluster.size();
				//ROS_INFO("Added (%d,%d)", centroidx, centroidy);

			}

			maxClusterSize = MAX(maxClusterSize, cluster.size());
		}

		EP.numCluster = centroidMap.size();

		for (auto it: centroidMap){
			centroidMap[it.first] = it.second/maxClusterSize;
		}

		for (int i = 0; i < EP.envWidth; i ++){
			delete[] visitedGrid_[i];
		}
		delete[] visitedGrid_;

		return centroidMap;
	}

	std::priority_queue<Frontier> Environment::evaluateFrontiers(){
		std::vector<std::vector<int>>tmpcoord;
		std::priority_queue<Frontier> pq;
		for (auto fCell : centroidMap){
			centroidVector.push_back(fCell.first);
			Frontier frontier(0,0,0);
			frontier.x = fCell.first[0];
			frontier.y = fCell.first[1];
			frontier.ig = fCell.second;
			std::map<std::vector<int>,double> tmap = cellValidate(fCell.first[0],  fCell.first[1]);
			frontier.cost = cellCost(fCell.first[0], fCell.first[1]);
			frontier.discount = cellDiscount(fCell.first[0], fCell.first[1]);
			if (localFrontier.size() > 0){
				frontier.validate = tmap.begin()->second ;
				std::vector<int> newCoord;
				if (frontier.validate == 0){
					newCoord = interpolateCoord(fCell.first);
				} else {
					newCoord = {tmap.begin()->first[0],tmap.begin()->first[1]};
				}
				frontier.pose = coordToPS(newCoord[0], newCoord[1]);
				tmpcoord.push_back({newCoord[0],newCoord[1]});
			}
			else {
				frontier.pose = coordToPS(fCell.first[0],fCell.first[1]);
				tmpcoord.push_back({fCell.first[0],fCell.first[1]});
			}


			frontier.evaluate(EP.cweight, EP.dweight, EP.iweight);
			//ROS_INFO("(%d,%d) has penalty of : %f", frontier.x, frontier.y, cellPenalty(frontier.x, frontier.y));
			pq.push(frontier);
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
		//ROS_INFO("TGP SIZE: %d", teamGoalPose.size());
	}



	void Environment::updateRobotTeamPose(std::map<std::string, geometry_msgs::PoseStamped> teamPose_){
		std::vector<geometry_msgs::PoseStamped> poseVector;
		for(auto& pose: teamPose_) {
			poseVector.push_back(pose.second);
		}
		teamPose = std::move(poseVector);
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
		//Brensenham approach
		std::vector<std::vector<int>> visitedCell;
		std::vector<int> goalCoord = {cx_,cy_};
		bresenham2D(currCoord,goalCoord,visitedCell);
		double cost = 0.0f;
		int obsCell = 0;
		for (auto coord: visitedCell){
			if (inMap(coord[0],coord[1])){
				int idx = coordToIndex(coord[0], coord[1]);
				if (occupancyGrid->data[idx] != 0){
					obsCell++;
				}
			} else {
				obsCell++;
			}
		}
		float totalCell = visitedCell.size() + EP.obsMult*obsCell;

		//navfnROS
		int navObs = 0; float navTotal = 0.0f;
		geometry_msgs::PoseStamped endPS = coordToPS(cx_,cy_);
		std::vector<geometry_msgs::PoseStamped> plan;
		if (planner->makePlan(currentPose, endPS, plan)){
			for (auto pose : plan){
				std::vector<int> coord = pointToCoord(pose.pose.position.x, pose.pose.position.y);
				if (inMap(coord[0],coord[1])){
					int idx = coordToIndex(coord[0], coord[1]);
					if (occupancyGrid->data[idx] != 0){
						//ROS_INFO("hiii");
						navObs++;
					}
				} else {
					ROS_INFO("OUT OF BOUNDS");
					navObs++;
				}
			}
			navTotal = plan.size() + EP.obsMult*navObs;
			ROS_INFO("avg cost: %.3f (vs %.3f)",MIN(1, (navTotal/EP.maxDist)), MIN(1, (totalCell/EP.maxDist)));
		} else {
			ROS_INFO("failed: %.3f (bresenham)", MIN(1, (totalCell/EP.maxDist)));
		}
		if (navTotal > totalCell){
			for (auto coord: visitedCell){
				pathCoord.push_back(coord);
			}
			for (auto pose : plan){
				navPS.push_back(pose);
			}
		}
		return MIN(1, (totalCell/EP.maxDist));
	}

	double Environment::cellDiscount (int cx_, int cy_){
		//Discount frontiers that are in other robots range
		std::vector<int> cellCoord{cx_,cy_};
		std::vector<float> cellPt = coordToPoint(cx_, cy_);
		float goalDC = 0.0f, poseDC = 0.0f;
		std::vector<geometry_msgs::PoseStamped> close;
		for (auto rGoal : teamGoalPose){
			std::vector<int> goalCoord = pointToCoord(rGoal.pose.position.x, rGoal.pose.position.y);
			float dist = norm(cellCoord,goalCoord);
			if (dist < EP.searchRadius){
				goalDC = goalDC + (1-goalDC)*(1-dist/EP.searchRadius);
				close.push_back(rGoal);
				/*ROS_INFO("%s's (%.3f,%.3f) is close to (%.3f,%.3f), goalDC: %.3f",
						EP.robotName.c_str(), cellPt[0], cellPt[1], rGoal.pose.position.x, rGoal.pose.position.y, goalDC);
*/
			}
		}
		for (auto rPose : teamPose){
			std::vector<int> poseCoord = pointToCoord(rPose.pose.position.x, rPose.pose.position.y);
			float dist = norm(cellCoord,poseCoord);
			//ROS_INFO("dist %f vs searchRadius %d", dist,EP.searchRadius);
			if (dist < EP.searchRadius){
				poseDC = poseDC + (1-poseDC)*(1-dist/EP.searchRadius);

			}
		}
		return MAX(goalDC,poseDC)+ (1-MAX(goalDC,poseDC))*MIN(goalDC,poseDC);
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
				return oldIntensity*MIN(1,pointDist[i]/EP.localRange);
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
		centroidMap.clear();
		centroidVector.clear();

		pathCoord.clear();
		navPS.clear();

	}

	/*
	 * Converts coord (discountGrid/valueGrid) to index (OccupancyGrid)
	 */
	int Environment::Environment::coordToIndex(int cx_, int cy_){
		return cy_*EP.envWidth + cx_;
	}


	//Point should be in global frame
	std::vector<float> Environment::coordToPoint(int cx_, int cy_){
		float originx_ = scanOrigin.x+offSet.x;
		float originy_ = scanOrigin.y+offSet.y;
		return {originx_+(cx_*EP.resolution), originy_+(cy_*EP.resolution)};
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
		//get index then convert to Coord?
		float originx = scanOrigin.x+offSet.x;
		float originy = scanOrigin.y+offSet.y;
		return {(int)(floor((mx_-originx)/EP.resolution)),(int)(floor((my_-originy)/EP.resolution))};

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

	bool Environment::isFrontier(int cx_, int cy_){
		// only perform convolution on non edge cells
		if (isEdge(cx_, cy_)){
			return false;
		}

		double grad = edgeGrad(cx_, cy_);
		if (grad >= EP.frontierThreshold && grad<=UPPERTHRESHOLD){
			return true;
		}
		return false;
	}

	bool Environment::inMap(int cx_, int cy_){
		return (cx_ >= 0 && cx_ <= EP.envWidth && cy_ >= 0 && cy_ <= EP.envHeight);
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

		gradx = -1 * cap(occupancyGrid->data[idxTL]) + cap(occupancyGrid->data[idxTR]);
		gradx += -1 * cap(occupancyGrid->data[idxBL]) + cap(occupancyGrid->data[idxBR]);
		gradx += -2*cap(occupancyGrid->data[idxL]) + 2* cap(occupancyGrid->data[idxR]);

		grady = -1 * cap(occupancyGrid->data[idxBL]) + cap(occupancyGrid->data[idxTL]);
		grady += -1 * cap(occupancyGrid->data[idxBR]) + cap(occupancyGrid->data[idxTR]);
		grady += - 2* cap(occupancyGrid->data[idxB]) + 2* cap(occupancyGrid->data[idxT]);

		return abs(gradx)+abs(grady);

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
			transformStamped = buffer.lookupTransform(EP.globalFrame, EP.robotMapFrame, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException &e) {
			ROS_WARN("Error with transform");
		}
		offSet.x = transformStamped.transform.translation.x;
		offSet.y = transformStamped.transform.translation.y;
		ROS_INFO("%s to %s (%.3f,%.3f)",  EP.robotMapFrame.c_str(),EP.globalFrame.c_str(), offSet.x, offSet.y);
	}

	void Environment::centroidCB(fyp_api::centroidArray c){
		if(c.robotName.compare(EP.robotName) != 0){
			teamCentroids[c.robotName] = c;
		}
	}

	void Environment::publishCentroid(){
		ROS_INFO("%s publishing centroid...", EP.robotName.c_str());
		fyp_api::centroid c;
		for (auto it: centroidMap){
			c.point = coordToPt(it.first[0],it.first[1]);
			c.csize = it.second;
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
		for (auto it: teamCentroids){
			int arraySize = it.second.centroids.size();
			for (int i = 0; i < arraySize; i++){
				std::vector<int> coord = pointToCoord(it.second.centroids[i].point.x, it.second.centroids[i].point.y);
				//need better frontier verification
				if (isFrontier(coord[0],coord[1])){
					centroidMap[coord] = it.second.centroids[i].csize;
					//ROS_INFO("It is a frontier!");
				} else {
					if (!isKnown(coord[0], coord[1])){
						centroidMap[coord] = it.second.centroids[i].csize;
						//ROS_INFO("It is unknown!");
					} else {
						//ROS_INFO("%s's (%d,%d) rejected by %s", it.first.c_str(), coord[0], coord[1], EP.robotName.c_str());
					}
				}
				std::vector<float> point = coordToPoint(coord[0], coord[1]);
				/*ROS_INFO("%s: (%.3f,%.3f) -> (%d, %d) -> (%.3f, %.3f)",
						EP.robotName.c_str(), it.second.centroids[i].point.x, it.second.centroids[i].point.y,
						coord[0], coord[1], point[0], point[1]);*/
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

		if (sum > 2 ) {
			return false;
		}
		return true;
	}

	void Environment::setPlanner(navfn::NavfnROS* planner_){
		planner = planner_;
		plannerInitialised = true;
	}

}

