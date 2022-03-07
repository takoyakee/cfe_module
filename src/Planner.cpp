#include <fyp_api/Planner.h>

//This is the middleman between TARE ENV sim to my own package
namespace Planner_ns
{

	SensorManager::SensorManager(ros::NodeHandle& nh_, ros::NodeHandle& nh_private,
			navfn::NavfnROS* planner_): moveBaseClient("move_base",true){
		planner = planner_;
		Initialise(nh_,nh_private);
	}

	bool SensorParameters::loadParameters(ros::NodeHandle& nh_private){
		std::string param_ns = "/frontier_common/";
		nh_private.getParam("robot_name", robotName);
		nh_private.getParam("state_topic", stateTopic);

		nh_private.getParam(param_ns+"plan_type", planType);
		nh_private.getParam(param_ns+"shared_status_topic",sharedStatusTopic);
		nh_private.getParam(param_ns+"shared_goal_topic", sharedGoalTopic);
		nh_private.getParam(param_ns+"globalmap_topic", globalmapTopic);
		nh_private.getParam(param_ns+"global_frame", globalFrame);
		nh_private.getParam(param_ns+"waypoint_frame", waypointFrame);
		nh_private.getParam(param_ns+"scan_topic", scanTopic);
		nh_private.getParam(param_ns+"velodyne_topic",velodyneTopic);
		nh_private.getParam(param_ns+"waypoint_topic", waypointTopic);
		nh_private.getParam(param_ns+"mergemap_topic", mergemapTopic);
		return true;
	}

	bool PCParameters::loadParameters(ros::NodeHandle& nh_private){
		nh_private.getParam("leaf_size", leafSize);
		nh_private.getParam("k_radius_threshold", kRadiusThreshold);
		nh_private.getParam("k_neighbour_threshold", kNeighborThreshold);
		nh_private.getParam("k_frontier_cluster_min_size", kFrontierClusterMinSize);

		kZDiffMax = kRadiusThreshold*5;
		kZDiffMin = kRadiusThreshold;

		return true;
	}

	void SensorManager::statusCallBack(const std_msgs::Bool msg){
		finishedExploration = msg.data;

	}


	void SensorManager::Initialise(ros::NodeHandle& nh_, ros::NodeHandle& nh_private){
		if(!SP.loadParameters(nh_private) || !(pcP.loadParameters(nh_private))){
			ROS_ERROR("Parameters not fully loaded");
			return;
		}

		ROS_INFO("Received: %s", SP.planType.c_str());

		//////2D PLANNING/////
		if (SP.planType.compare("2D") == 0){

			//Here is where you sub to goals
			goalPub 	=	 nh_.advertise<geometry_msgs::PoseStamped>(SP.sharedGoalTopic,1);
			goalSub 	= 	nh_.subscribe(SP.sharedGoalTopic, 10, &SensorManager::goalCallBack, this);
			statusSub = nh_.subscribe(SP.sharedStatusTopic,1,&SensorManager::statusCallBack,this);
		} else if (SP.planType.compare("3D") == 0){
			velodyneSub 	=	nh_.subscribe(SP.velodyneTopic, 1, &SensorManager::velodyneCallBack, this);
			stateSub 		= 	nh_.subscribe<nav_msgs::Odometry>(SP.stateTopic, 3, &SensorManager::stateCallBack, this);
			waypointPub = 	nh_.advertise<geometry_msgs::PointStamped>(SP.waypointTopic,5);
			frontierpub = nh_.advertise<sensor_msgs::PointCloud2>("frontier_cloud",5);
			filteredpub = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud",5);
			clusteredpub = nh_.advertise<sensor_msgs::PointCloud2>("clustered_cloud",5);
			occludedpub = nh_.advertise<sensor_msgs::PointCloud2>("occluded_cloud",5);
			freepub = nh_.advertise<sensor_msgs::PointCloud2>("free_cloud",5);
			localOGPub = nh_.advertise<nav_msgs::OccupancyGrid>("local_grid",5);
			globalOGPub =  nh_.advertise<nav_msgs::OccupancyGrid>("global_grid",5);
			worldGrid = std::make_unique<GridManager_ns::GridManager>(nh_,nh_private);
			localGrid = std::make_unique<GridManager_ns::GridManager>(nh_,nh_private);
			frontier_plane = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
			local_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
			global_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
			frontier_centroids = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
		}
		//costMapSub 	=	nh_.subscribe(SP.globalmapTopic, 1, &SensorManager::globalMapCallBack, this);
		scanSub 		= 	nh_.subscribe(SP.scanTopic, 1, &SensorManager::scanCallBack, this);

		distanceTravelled = 0.0f;
		desiredRate = 1;
		finishedExploration = false;

		robotController = std::make_unique<Robot_ns::Robot>(nh_,nh_private);
		robotController->setEnvPlanner(planner);

		ROS_INFO("FINISHED INITIALISATION");

	}

	bool SensorManager::execute(){
		//ROS_INFO("Execute called!");
		if (SP.planType.compare("3D") == 0){
			ROS_INFO("3D planning");
			sendLocalFrontier();
			geometry_msgs::PointStamped wayPoint;
			if (robotController->explore()){
				wayPoint = robotController->getWayPoint();
				//robotController->addFrontierVertex(wayPoint);
				//waypointPub.publish(wayPoint);
			}
		}
		else if (SP.planType.compare("2D") == 0){
			//ROS_INFO("2D planning");
			robotController->status = moveBaseClient.getState();
			//print out state if not active:
			if (moveBaseClient.getState() != actionlib::SimpleClientGoalState::ACTIVE){
				//ROS_INFO("State: %s",moveBaseClient.getState().toString().c_str());
			}
			robotController->updatePoses();
			robotController->addTrajectory();
			if (robotController->explore()){
	    	    if (robotController->hasGoal){
	    	    	if (finishedExploration){
	    	    		moveBaseClient.cancelGoal();
	    	    	} else {
		    			moveBaseClient.sendGoal(robotController->getMBG());
		    			goalPub.publish((robotController->getMBG()).target_pose);
	    	    	}

	    		}
	    	}
		}
		else {
			ROS_INFO("Not supported planning!");
			return false;
		}
		return finishedExploration;


	}

	bool SensorManager::isInitialised(){
		return robotController->isInitialised();
	}

	void SensorManager::sendLocalFrontier(){
		pcl::PointCloud<pcl::PointXYZI>::Ptr frontier_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
		localGrid->getFrontierCloud(frontier_cloud);
		sensor_msgs::PointCloud2 scan_data;
		pcl::toROSMsg(*frontier_cloud, scan_data);
		scan_data.header.stamp = ros::Time::now();
		scan_data.header.frame_id = SP.waypointFrame;
		frontierpub.publish(scan_data);


		extractFrontierPlane(frontier_cloud, frontier_plane);
		clusterFrontier(frontier_plane);
		//robotController->updateEnvLocalFrontier(*frontier_centroids);
	}

	void SensorManager::stateCallBack(const nav_msgs::Odometry::ConstPtr& odomMsg){
		//conversion to appropriate typeUpdateRobotPosition
		ROS_INFO("state callback!");
		Eigen::Vector3d prevPose;
		prevPose = currPose;
		updateCurrPose(*odomMsg);

		robotController->updateRobotPose(odomToPoseStamped(*odomMsg));
		distanceTravelled += distBtwPose(prevPose, currPose);
		if (distanceTravelled > 5){
			robotController->addPosVertex(currPose);
			distanceTravelled = 0.0f;
		}

		localGrid->updatePoseOrigin((*odomMsg).pose.pose);
/*		localmap = localGrid->visualizeGrid(3);
		localOGPub.publish(localmap);*/

/*		worldGrid->updateRobotPose((*odomMsg).pose.pose);
		worldGrid->checkExpansion();
		globalmap = worldGrid->visualizeGrid(2);
		globalOGPub.publish(globalmap);*/

	}


	//TODO: have two maps, global (fixed frame) and local (moves with robot)
	// Specifically, to merge world grid and local grid!!! (using offset!!)
	// ScanCallBack: Merged map, identical for all robots
	void SensorManager::scanCallBack(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_){
		robotController->updateEnvGrid(occupancyGrid_);
	}


	/////////2D PLANNING ONLY ////////
	// for offset calculations - scan data received in merge map frame, but calculation is done in GLOBAL FRAME

/*	void SensorManager::selfMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_){

	}*/
	/////////////////////////////////////////////


	void SensorManager::velodyneCallBack(const sensor_msgs::PointCloud2ConstPtr& scanMsg){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr dwz_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr frontier_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

		ROS_INFO("velodyne callback!!");
		pcl::fromROSMsg(*scanMsg, *pcl_cloud);
		local_cloud->clear();
		//*global_cloud += *pcl_cloud;
		*local_cloud = *pcl_cloud;

		nav_msgs::OccupancyGrid localmap, globalmap;
		localGrid->resetGrid();

		downsizeCloud(pcl_cloud, dwz_cloud);
		localGrid->updateOccupancyGrid(dwz_cloud);
/*		localmap = localGrid->visualizeGrid(3);
		localOGPub.publish(localmap);*/


/*		localGrid->merge(*worldGrid);
		globalmap = worldGrid->visualizeGrid(2);
		globalOGPub.publish(globalmap);*/

	}

	void SensorManager::downsizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out){
		pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		voxel_filter.setInputCloud(cloud_in);
		voxel_filter.setLeafSize (pcP.leafSize,pcP.leafSize,pcP.leafSize);
		voxel_filter.filter(*cloud_out);
		//ROS_INFO("size reduction from %d to %d", cloud_in->points.size(), cloud_out->points.size());
	}

	void SensorManager::updateFrontierCentroids(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in){
		pcl::PointCloud<pcl::PointXYZI>::Ptr frontier_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

		//update local map with pcl cloud -> update global map
		localGrid->updateOccupancyGrid(cloud_in);
		localGrid->getFrontierCloud(frontier_cloud);

		extractFrontierPlane(frontier_cloud, frontier_plane);
		clusterFrontier(frontier_plane);
	}


	void SensorManager::extractFrontierPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out,
			double z_max, double z_min)
	{
		//ROS_INFO("Extracting Frontier...");
		pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr extractKdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr extractor_cloud_ (new pcl::PointCloud<pcl::PointXYZI>());
		if (cloud_in->points.empty())
		{
		  return;
		}
		pcl::copyPointCloud(*cloud_in, *extractor_cloud_);
		for (auto& point : extractor_cloud_->points)
		{
		  point.intensity = point.z;
		  point.z = 0.0;
		}

		extractKdtree->setInputCloud(extractor_cloud_);
		if (!cloud_out->points.empty()){
			cloud_out->clear();
			//ROS_INFO("Cleared!");
		}
		std::vector<int> neighbor_indices;
		std::vector<float> neighbor_sqdist;
		for (int i = 0; i < extractor_cloud_->points.size(); i++)
		{
		  pcl::PointXYZI point = extractor_cloud_->points[i];
		  if (point.intensity > z_max || point.intensity < z_min)
			continue;
		  extractKdtree->radiusSearch(point, pcP.kRadiusThreshold, neighbor_indices, neighbor_sqdist);
		  bool is_vertical = false;
		  int neighbor_count = 0;
		  for (const auto& idx : neighbor_indices)
		  {
			double z_diff = std::abs(point.intensity - extractor_cloud_->points[idx].intensity);
			if (z_diff > pcP.kZDiffMin && z_diff < pcP.kZDiffMax)
			{
			  neighbor_count++;
			  if (neighbor_count >= pcP.kNeighborThreshold)
			  {
				is_vertical = true;
				break;
			  }
			}
		  }
		  if (is_vertical)
		  {
			pcl::PointXYZI point_out;
			point_out.x = cloud_in->points[i].x;
			point_out.y = cloud_in->points[i].y;
			point_out.z = cloud_in->points[i].z;
			cloud_out->points.push_back(point_out);
		  }
		}

		  //visualize frontier cloud
		  //ROS_INFO("Extracted Frontier Planes!");
		  sensor_msgs::PointCloud2 scan_data;
		  pcl::toROSMsg(*cloud_out, scan_data);
		  scan_data.header.stamp = ros::Time::now();
		  scan_data.header.frame_id = SP.waypointFrame;
		  filteredpub.publish(scan_data);

	}

	void SensorManager::clusterFrontier(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){
		pcl::search::KdTree<pcl::PointXYZI>::Ptr  kdtree_frontier_cloud(new pcl::search::KdTree<pcl::PointXYZI>);
		pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	    // Cluster frontiers
	    if (!cloud->points.empty())
	    {
			//ROS_INFO("clustering...");

	    	if (!frontier_centroids->points.empty()){
		    	frontier_centroids->clear();
		    	//ROS_INFO("Cleared!");
	    	}

	    	kdtree_frontier_cloud->setInputCloud(cloud);
	    	std::vector<pcl::PointIndices> cluster_indices;
	    	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	    	ec.setClusterTolerance(1.0f);
	    	ec.setMinClusterSize(pcP.kFrontierClusterMinSize);
	    	ec.setMaxClusterSize(10000);
	    	ec.setSearchMethod(kdtree_frontier_cloud);
	    	ec.setInputCloud(cloud);
	    	ec.extract(cluster_indices);

	    	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	    	int cluster_count = 0;
	    	//ROS_INFO("no. of cluster: %d", cluster_indices.size());
	    	for (int i = 0; i < cluster_indices.size(); i++)
	    	{

		        if (cluster_indices[i].indices.size() < pcP.kFrontierClusterMinSize)
		        {
		          continue;
		        }

	    	 	Eigen::Vector4d centroidPos {0,0,0,0};
	    		pcl::compute3DCentroid(*cloud, cluster_indices[i], centroidPos);
				pcl::PointXYZI centroidpt;
				centroidpt.x = centroidPos.x();
				centroidpt.y = centroidPos.y();
				centroidpt.z = centroidPos.z();
				//ROS_INFO("Centroid: %f,%f,%f",centroidpt.x,centroidpt.y,centroidpt.z);
		    	frontier_centroids->points.push_back(centroidpt);
	    		for (int j = 0; j < cluster_indices[i].indices.size(); j++)
	    		{
	    			int point_ind = cluster_indices[i].indices[j];
	    			cloud->points[point_ind].intensity = cluster_count;
	    			inliers->indices.push_back(point_ind);
	    		}
	    		cluster_count++;
	    	}
	    	//ROS_INFO("Number of recorded clusters: %d", cluster_count);
	    	pcl::ExtractIndices<pcl::PointXYZI> extract;
	    	extract.setInputCloud(cloud);
	    	extract.setIndices(inliers);
	    	extract.setNegative(false);
	    	extract.filter(*cloud);
	    }


	    //TODO: visualization:
		//ROS_INFO("Clustering Frontier Cloud");
		//visualize frontier cloud
	 	sensor_msgs::PointCloud2 scan_data;
	 	pcl::toROSMsg(*frontier_centroids, scan_data);
	 	scan_data.header.stamp = ros::Time::now();
	 	scan_data.header.frame_id = SP.waypointFrame;
	 	clusteredpub.publish(scan_data);



	}

	double SensorManager::distBtwPose(Eigen::Vector3d p1, Eigen::Vector3d p2){
		if (p1.size() == 0 || p2.size() == 0){
			return 0.0f;
		}
		return (p1-p2).norm();
	}


	void SensorManager::updateCurrPose(const nav_msgs::Odometry odom){
		currPose.x() = odom.pose.pose.position.x;
		currPose.y() = odom.pose.pose.position.y;
		currPose.z() = odom.pose.pose.position.z;

	}


	geometry_msgs::PoseStamped SensorManager::odomToPoseStamped(const nav_msgs::Odometry odom){
		geometry_msgs::PoseStamped PS;
		PS.pose = odom.pose.pose;
		PS.header = odom.header;
		return PS;
	}

	///////// 2DD /////////////
    void SensorManager::goalCallBack(const geometry_msgs::PoseStamped& msg){
    	robotController->teamGoalCallBack(msg);
    }



}



