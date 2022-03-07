#include <fyp_api/robot.h>


namespace Robot_ns{
	bool RobotParameters::loadParameters(ros::NodeHandle& nh_private){
		nh_private.getParam("robot_name", robotName);
		nh_private.getParam("robot_map_frame", robotMapFrame);

		std::string param_ns = "/frontier_common/";
		nh_private.getParam(param_ns+"global_frame", globalFrame);
		nh_private.getParam(param_ns+"waypoint_frame", waypointFrame);
		nh_private.getParam(param_ns+"base_name", baseName); //baseName = vehicle
		nh_private.getParam(param_ns+"comm_radius", commRad);
		nh_private.getParam(param_ns+"team_size", teamSize);
		nh_private.getParam(param_ns+"file_path",filePath);
		nh_private.getParam(param_ns+"goal_horizon",goalHorizon);
		return true;

	}
	Robot::Robot(ros::NodeHandle& nh_, ros::NodeHandle& nh_private)
	{
		Initialise(nh_,nh_private);
		robotEnvironment = std::make_unique<Environment_ns::Environment>(nh_,nh_private);
		ROS_INFO("ROBOT initialised!");
	}

	void Robot::Initialise(ros::NodeHandle& nh_, ros::NodeHandle& nh_private){
		if(!RP.loadParameters(nh_private)){
			ROS_ERROR("Parameters not fully loaded");
			return;
		}
		envInitialised = false;
		poseInitialised = false;
		hasGoal = false;
		processingGoal = false;
		goalReached = false;
		recoveryState = false;
		searchEnded = false;
		rate = RP.goalHorizon; //previously 4
		counter = 0;
		distTravelledSince = 1000;
		distcounter = 4;
		distrate = 5;
		//file.open(RP.filePath+RP.robotName+".txt");
		//file << "Time" << "\t"  << "rbt x" << "\t" << "rbt y" << "t" << "dist" << std::endl;
/*
		posefile.open(RP.filePath+RP.robotName+"_poses.txt");
		file << "Time" << "\t" << "rbt x" << "\t" << "rbt y" << "\t" << "x" << "\t" << "y" << "\t"
			<< "cost" << "\t" << "discount" << "\t" << "ig" << "\t" << "utility" << std::endl;
		posefile << "Time" << "\t" << "first x" << "\t" << "first y"  << "second x " << "\t" << "second y "  << "\t"
				 << "first goal x" << "\t" << "first goal y"  << "second goal x " << "\t" << "second goal y "  << "\t"
				 << "x" << "\t" << "y" << "\t" << "discount" << std::endl;
*/


		trajPub = nh_.advertise<nav_msgs::Path>("traj",1);
		posePub = nh_.advertise<geometry_msgs::PoseStamped>("pose",1);
		distPub = nh_.advertise<fyp_api::distMetrics>("dist",1);
		odomSub = nh_.subscribe("/"+RP.robotName+"/odom", 1, &Robot::odomCallBack,this);
		trajectory.header.frame_id = RP.globalFrame;
	}

	bool Robot::explore(){
		if (!isInitialised()){
			ROS_INFO("Waiting to be initialised");
			return false;
		}
		counter ++;
		distcounter ++;
		if (distcounter == distrate){
			if (distTravelledSince > 0.1){
				getLatestFrontier();
			} else {
				recoveryState = true;
				ROS_INFO("RECOVERY");
			}

			//ROS_INFO("Dist since: %.3f", distTravelledSince);
			distcounter = 0;
			distTravelledSince = 0;
		} else {
			getLatestFrontier();
		}

		setNewGoal();
		//writeFile(file,frontierCandidate);
		return hasGoal;
	}

	bool Robot::getLatestFrontier(){
		//ROS_INFO("Getting latest frontier");
		//2d planning:
		if (getEnvFrontier()){
			rankEnvFrontier(frontierCandidates);
			return true;
		}
		ROS_INFO("NO FRONTIERS!");
		return false;
	}

	void Robot::setNewGoal(){
		Environment_ns::Frontier chosen = frontierCandidate;
		//consider distcounter
		if (!hasGoal || counter >= rate || recoveryState){
			recoveryState = false;
			chosen = chooseFrontier();
			hasGoal = !chosen.empty();
		}
		if (hasGoal){
			updateGoal(chosen.pose);
			updateMBG();
		}
		frontierCandidate = chosen;
		if (counter >= rate){
			counter = 0;

			ROS_INFO("(%.1f,%.1f) with C:%.2f D: %.2f I: %.2f",chosen.pose.pose.position.x,chosen.pose.pose.position.y,
		chosen.cost,chosen.discount,chosen.ig);
		}

	}

	bool Robot::getEnvFrontier(){
		//To return a number, enum?  0:FINISHED 1:EXPLORING 2:RECOVERY
		frontierCandidates = robotEnvironment->returnFrontiers();
		return !frontierCandidates.empty();
	}

	void Robot::rankEnvFrontier(std::priority_queue<Environment_ns::Frontier> candidates){
		while (!closeFrontier.empty()){
			closeFrontier.pop();
		}
		while (!rejectedFrontier.empty()){
			rejectedFrontier.pop();
		}

		bool writetofile = true;
		while (!candidates.empty()){
			Environment_ns::Frontier f = candidates.top();
			//writePoseFile(posefile, f);

			bool closest = true;

			if (closest){
				closeFrontier.push(f);
			} else {
				rejectedFrontier.push(f);

			}
			candidates.pop();
		}
		//file << std::endl;
	}

	Environment_ns::Frontier Robot::chooseFrontier(){
		Environment_ns::Frontier f{0,0};
		if (!closeFrontier.empty()){
			f = closeFrontier.top();
			closeFrontier.pop();
			//ROS_INFO("CLOSE: (%d,%d)", f.x, f.y);
		} else if (!rejectedFrontier.empty()){
			f = rejectedFrontier.top();
			rejectedFrontier.pop();
			//ROS_INFO("rejected: (%d,%d)", f.x, f.y);
		}
		return f;
	}


	void Robot::addPosVertex(Eigen::Vector3d pos){
		geometry_msgs::PoseStamped pose, globalPose;
		pose.pose.position.x = pos.x();
		pose.pose.position.y = pos.y();
		pose.pose.position.z = pos.z();
		pose.header.frame_id = RP.waypointFrame;
		pose.header.stamp = ros::Time::now();
		globalPose = transformToGF(pose);
		Eigen::Vector3d newPos(globalPose.pose.position.x,globalPose.pose.position.y,globalPose.pose.position.z);
		robotEnvironment->addEnvPosVertex(newPos);
		//ROS_INFO("Added (%f,%f,%f)", newPos.x(), newPos.y(), newPos.z());

	}

	void Robot::addFrontierVertex(geometry_msgs::PointStamped point){
		geometry_msgs::PoseStamped pose, globalPose;
		pose.pose.position.x = point.point.x;
		pose.pose.position.y = point.point.y;
		pose.pose.position.z = point.point.z;
		pose.header.frame_id = RP.waypointFrame;
		pose.header.stamp = ros::Time::now();
		globalPose = transformToGF(pose);
		Eigen::Vector3d newPos(globalPose.pose.position.x,globalPose.pose.position.y,globalPose.pose.position.z);
		robotEnvironment->addEnvFrontierVertex(newPos);
		ROS_INFO("Frontier Added (%f,%f,%f)", newPos.x(), newPos.y(), newPos.z());
	}


	void Robot::updateRobotPose(geometry_msgs::PoseStamped poseStamped){
		geometry_msgs::PoseStamped tempPose = transformToGF(poseStamped);
		currentPose = tempPose;

	}

	void Robot::updateEnvGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_){
		//ROS_INFO("update env grid");
		robotEnvironment->updateOccupancyGrid(occupancyGrid_);
		if (!envInitialised){
			envInitialised = robotEnvironment->isEnvInitialised();
		}
	}

	void Robot::updateEnvLocalFrontier(pcl::PointCloud<pcl::PointXYZI> frontierCloud){
		std::vector<Eigen::Vector3d> localFrontier;
		for (auto pt: frontierCloud.points){
			geometry_msgs::PoseStamped pose, globalPose;
			pose.pose.position.x = pt.x;
			pose.pose.position.y = pt.y;
			pose.pose.position.z = pt.z;
			pose.header.frame_id = RP.waypointFrame;
			pose.header.stamp = ros::Time::now();
			globalPose = transformToGF(pose);
			localFrontier.push_back(Eigen::Vector3d(globalPose.pose.position.x,globalPose.pose.position.y,globalPose.pose.position.z));
		}
		robotEnvironment->updateLocalFrontierCloud(localFrontier);
	}

	void Robot::updateEnvPoses(){
		robotEnvironment->updateRobotPose(currentPose);
		robotEnvironment->updateRobotTeamPose(teamPose);
		robotEnvironment->updateTeamGoalPose(teamGoalPose);
	}

	void Robot::getTeamPoses(){
		for (int i = 1; i < RP.teamSize+1; i++){
			geometry_msgs::PoseStamped otherPose;
			std::string otherRobotName = "robot_" + std::to_string(i);
			if (otherRobotName.compare(RP.robotName) == 0){
				continue;
			}
			//ROS_INFO("robotname: %s", otherRobotName.c_str());
			std::string targetFrame = otherRobotName + "/" + RP.baseName;
			otherPose = inCommunicationRange(targetFrame);
			// e.g. {robot_2: PoseStamped}
			teamPose[otherRobotName] = otherPose;
		}
	}

	geometry_msgs::PointStamped Robot::getWayPoint(){
		geometry_msgs::PoseStamped wpPose = transformToTF(goalPose, RP.waypointFrame);
		geometry_msgs::PointStamped goalPoint;
		goalPoint.header = wpPose.header;
		goalPoint.point.x = wpPose.pose.position.x;
		goalPoint.point.y = wpPose.pose.position.y;
		goalPoint.point.z = wpPose.pose.position.z;
		return goalPoint;
	}


	bool Robot::updateGoal(geometry_msgs::PoseStamped goalPose_){
		goalPose = goalPose_;
		return true;
	}

	/*
	 * robotFrameName: robot name + base name (e.g. robot_1/base_link
	 * checks through TF: converts to global frame gives position of robot in global frame
	 * Then, checks if within communication range and return pose if within
	 */
	geometry_msgs::PoseStamped Robot::inCommunicationRange(std::string robotFrameName){
		geometry_msgs::PoseStamped tempPose, targetPose, emptyPose;
		tempPose.header.frame_id = robotFrameName;
		targetPose = transformToGF(tempPose);
		if (norm(targetPose ,currentPose) <= RP.commRad){return targetPose;}
		return emptyPose;
	}

	geometry_msgs::PoseStamped Robot::transformToGF(geometry_msgs::PoseStamped pose){
		//Convert received posestamped to global frame if not already in global frame
		tf2_ros::Buffer buffer;
		tf2_ros::TransformListener tfl(buffer);
		geometry_msgs::TransformStamped transformStamped;
		geometry_msgs::PoseStamped globalPose;
		try {
			transformStamped = buffer.lookupTransform(RP.globalFrame, pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException &e) {
			ROS_WARN("Error with transform");
		}
		tf2::doTransform(pose, globalPose, transformStamped);
		return globalPose;
	}

	geometry_msgs::PoseStamped Robot::transformToTF(geometry_msgs::PoseStamped pose, std::string targetFrame){
		//Convert received posestamped to global frame if not already in global frame
		tf2_ros::Buffer buffer;
		tf2_ros::TransformListener tfl(buffer);
		geometry_msgs::TransformStamped transformStamped;
		geometry_msgs::PoseStamped targetPose;
		try {
			transformStamped = buffer.lookupTransform(targetFrame, pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException &e) {
			ROS_WARN("Error with transform");
		}
		tf2::doTransform(pose, targetPose, transformStamped);
		return targetPose;
	}

//////////////////////////////////////////////////////////////////////////////////////
	bool Robot::isInitialised(){
		int goalNumber = atoi(&RP.robotName.back())-1;
		//goalReceived = true;

		if (!goalReceived){
			if((int)(teamGoalPose.size()) >= goalNumber){
				goalReceived = true;
			}
		}
		return envInitialised && poseInitialised && goalReceived;

	}
	void Robot::updatePoses(){
		getCurrentPose();
		getTeamPoses();
		updateEnvPoses();
	}
	//THIS IS FOR 2D EXPLORATION
	geometry_msgs::PoseStamped Robot::getCurrentPose(){
		//ROS_INFO("Getting current pose");
	    tf2_ros::Buffer buffer;
	    tf2_ros::TransformListener tfl(buffer);
	    geometry_msgs::TransformStamped transformStamped;
	    geometry_msgs::PoseStamped tempPose;
	    try {
	        transformStamped = buffer.lookupTransform(RP.globalFrame, RP.robotName+"/"+RP.baseName, ros::Time(0), ros::Duration(3.0));
	    } catch (tf2::TransformException &e) {
	    	ROS_WARN("Error with getting Curr Pose");
	    	poseInitialised = false;
	    }

	    tf2::doTransform(tempPose, currentPose, transformStamped);
	    currentPose.header.stamp = ros::Time::now();
	    currentPose.header.frame_id = RP.globalFrame;
	    //ROS_INFO("POSITION: %.3f,%.3f", currentPose.pose.position.x,currentPose.pose.position.y);
	    poseInitialised = true;

	    return currentPose;

	}

	move_base_msgs::MoveBaseGoal Robot::getMBG(){
		//ROS_INFO("MBG: %f,%f,%f", moveBaseGoal.target_pose.pose.position.x,moveBaseGoal.target_pose.pose.position.y,moveBaseGoal.target_pose.pose.position.z);
		return moveBaseGoal;
	}

	void Robot::updateMBG(){
		geometry_msgs::PoseStamped rfPose = transformToTF(goalPose, RP.robotMapFrame);
		//to include orientation
		rfPose.pose.orientation.w = 1;
		rfPose.pose.orientation.x = 0;
		rfPose.pose.orientation.y = 0;
		rfPose.pose.orientation.z = 0;

		moveBaseGoal.target_pose = rfPose;
	}

	void Robot::teamGoalCallBack(const geometry_msgs::PoseStamped& msg){
		if (!msg.header.frame_id.empty()){
			std::string poseMapFrame = msg.header.frame_id;
			std::string robotFrameName =  eraseSubStr(poseMapFrame, "/map") + "/" + RP.baseName; // robot_1/base_link -> to review and avoid hardcode
			//ROS_INFO("received robotFrameName: %s", robotFrameName.c_str());
			if (poseMapFrame.compare(RP.robotMapFrame) != 0){
				geometry_msgs::PoseStamped inGF;
				if (!inCommunicationRange(robotFrameName).header.frame_id.empty()){
					inGF = transformToGF(msg);
					teamGoalPose[poseMapFrame] = inGF;
				} else {
					//ROS_INFO("OUT OF COMM RANGE");
				}
			}
		}

	}

	void Robot::addTrajectory(){
		getCurrentPose();
		geometry_msgs::PoseStamped temp = currentPose;
		float distTravelled  = 0;
		if (!trajectory.poses.empty()) distTravelled = norm(trajectory.poses[trajectory.poses.size()-1], temp);
		temp.pose.orientation.w = 0;
		temp.pose.orientation.x = 0;
		temp.pose.orientation.y = 0;
		temp.pose.orientation.z = 1;
		trajectory.header.stamp = ros::Time::now();
		trajectory.poses.push_back(temp);
		if (!trajectory.poses.empty()){
			trajPub.publish(trajectory);
			RP.dist += distTravelled;
			fyp_api::distMetrics dm;
			dm.robotName = RP.robotName;
			dm.dist = RP.dist;
			distPub.publish(dm);
		}

		if (distcounter != distrate){
			distTravelledSince += distTravelled;
		}
	}

	//2D EXPLORATON

	void Robot::setEnvPlanner(navfn::NavfnROS* planner){
		robotEnvironment->setPlanner(planner);
	}

	void Robot::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg){
		geometry_msgs::PoseStamped temp;
		temp.pose = msg->pose.pose;
		temp.header = msg->header;

		posePub.publish(temp);

	}

	void Robot::writeFile(std::ofstream& file_, Environment_ns::Frontier f){
		file_ << ros::Time::now().toSec() << "\t" << currentPose.pose.position.x << "\t" << currentPose.pose.position.y << "\t" << f.pose.pose.position.x << "\t" << f.pose.pose.position.y << "\t"
											<< f.cost << "\t" << f.discount << "\t" << f.ig << "\t" << f.utility << std::endl;

	}

	void Robot::writePoseFile(std::ofstream& file_, Environment_ns::Frontier f){
		file_ << ros::Time::now().toSec() << "\t"
				<< teamPose.begin()->second.pose.position.x << "\t" << teamPose.begin()->second.pose.position.y << "\t"
				<< teamPose.end()->second.pose.position.x << "\t" << teamPose.end()->second.pose.position.y << "\t"
				<< teamGoalPose.begin()->second.pose.position.x << "\t" << teamGoalPose.begin()->second.pose.position.y << "\t"
				<< teamGoalPose.end()->second.pose.position.x << "\t" << teamGoalPose.end()->second.pose.position.y << "\t"
				<<f.x << "\t" << f.y << "\t" << f.discount << std::endl;


	}



}






