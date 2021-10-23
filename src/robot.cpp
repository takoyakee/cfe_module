#include <fyp/robot.h>

Robot::Robot(std::string robotName_, std::string robotMapFrame_, std::string globalFrame_,
        		std::string robotFrame_, std::string mergeMapTopic_, std::string mapTopic_, std::string frontierGoal_)
{
    robotName = robotName_;
    robotMapFrame = robotMapFrame_;
    globalFrame = globalFrame_;
    //e.g. robot_1/baselink
    robotFrame = robotFrame_;
    mergeMapTopic = mergeMapTopic_;
    mapTopic = mapTopic_;
    frontierGoal = frontierGoal_;
    currentPose = getCurrentPose();
    hasGoal = false;
    processingGoal = false;
    goalReached = false;
    recoveryState = false;
    commRadius = 10000;
    failedRun = 0;
    frontiersExplored = 0;
    robotEnvironment.globalFrame = globalFrame;
    robotEnvironment.robotName = robotName;
    teamSize = 2;

}

void Robot::explore(){
	ROS_INFO("EXPLORING...");
	if(!robotEnvironment.isEnvInitialised()){
		ROS_INFO("Waiting for Envt to Initialise");
		return ;
	}
	if (status.state_ == actionlib::SimpleClientGoalState::LOST ||!status.isDone()){
		processingGoal = status.state_ == actionlib::SimpleClientGoalState::ACTIVE;
		ROS_INFO("Recovery: %d", recoveryState);
		ROS_INFO("Processing goal: %d", processingGoal);
		//recoveryState is a flag that blocks updating
		if (!recoveryState){
			robotEnvironment.bUpdate = true;
			while (!robotEnvironment.isEnvUpdated()){
				updateEnv();
			}
			if (getFrontierCandidates()){
				ROS_INFO("Candidates obtained");
				rankFrontiers(frontierCandidates);
				Environment::Frontier chosen = chooseFrontier();
				ROS_INFO("GOAL: (%d,%d)", chosen.x, chosen.y);
				updateGoal(chosen.pose);
				updateMBG();
				hasGoal = true;
			} else {
				// to code some recovery behaviours (e.g. decrease required clustered size?)
				ROS_INFO("NO FRONTIERS!");
				hasGoal = false;
			}
		}
	} else {
		if (status.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
			processingGoal = false;
			recoveryState = false;
		} else {
			recoveryState = true;
			processingGoal = false;
			Environment::Frontier chosen = chooseFrontier();
			if (chosen.empty()){
				ROS_INFO("NO FRONTIERS!");
				hasGoal = false;
				recoveryState = false;
				// to code some recovery behaviours
			} else {
				ROS_INFO("recovery GOAL: (%d,%d)", chosen.x, chosen.y);
				updateGoal(chosen.pose);
				updateMBG();
				hasGoal = true;
			}
		}
	}
}

Environment::Frontier Robot::chooseFrontier(){
	Environment::Frontier f{0,0,0};
	if (!closeFrontier.empty()){
		f = closeFrontier.top();
		closeFrontier.pop();
		ROS_INFO("CLOSE: (%d,%d)", f.x, f.y);
	} else if (!rejectedFrontier.empty()){
		f = rejectedFrontier.top();
		rejectedFrontier.pop();
		ROS_INFO("rejected: (%d,%d)", f.x, f.y);
	}
	return f;
}

bool Robot::getFrontierCandidates(){
	ROS_INFO("Getting Frontier Candidates...");
	frontierCandidates = robotEnvironment.returnFrontiers();
	return !frontierCandidates.empty();
}
void Robot::rankFrontiers(std::priority_queue<Environment::Frontier> candidates){
	while (!candidates.empty()){
		Environment::Frontier f = candidates.top();
		bool closest = true;
		for (auto rPose: teamPose){
			if (norm(currentPose, f.pose) > norm(rPose.second,f.pose)){
				closest = false;
			}
		}
		if (closest){
			closeFrontier.push(f);
		} else {
			rejectedFrontier.push(f);
			//ROS_INFO("%s: FAR (%d,%d) with utility: %d", robotName.c_str(),f.x, f.y, f.utility);

		}

		candidates.pop();
	}

}

void Robot::teamGoalCallBack(const geometry_msgs::PoseStamped& msg){
	//only consider if pose published is not own (e.g. robot_1/map)
	if (!msg.header.frame_id.empty()){
		std::string poseMapFrame = msg.header.frame_id;
		std::string robotFrameName =  eraseSubStr(poseMapFrame, "/map") + "/base_link"; // robot_1/base_link -> to review and avoid hardcode
		ROS_INFO("%s vs %s", poseMapFrame.c_str(), robotMapFrame.c_str());
		if (poseMapFrame.compare(robotMapFrame) != 0){

			if (!inCommunicationRange(robotFrameName).header.frame_id.empty()){
				ROS_INFO("GOAL %s is REGISTERED BY %s", poseMapFrame.c_str(), robotMapFrame.c_str());
				//e.g. {robot_2/map, PoseStamped}
				teamGoalPose[poseMapFrame] = msg;
			} else {
				ROS_INFO("OUT OF COMM RANGE");
			}
		}
	}
}

void Robot::mergeMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	if (msg->data.size()> 0){
		robotEnvironment.updateOccupancyGrid(msg);
	}
}

void Robot::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	if (msg->data.size()> 0 && robotEnvironment.occupancyGrid.data.size()> 0){
		robotEnvironment.updateMapOffSet(msg);
	}
}



geometry_msgs::PoseStamped Robot::Robot::getCurrentPose(){
	ROS_INFO("Getting current pose");
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfl(buffer);
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped tempPose;
    try {
        transformStamped = buffer.lookupTransform(globalFrame, robotFrame, ros::Time(0), ros::Duration(3.0));
    } catch (tf2::TransformException &e) {
    	ROS_WARN("Error with getting Curr Pose");
    }

    tf2::doTransform(tempPose, currentPose, transformStamped);
    return currentPose;

}

/*
 * Returns PoseStamped of robot with robotFrameName if within communication range
 */
geometry_msgs::PoseStamped Robot::inCommunicationRange(std::string robotFrameName){
	//todo: Obtain other robots poses via tf, evaluate distance before appending to vector
	// two ways: calculate norm (TRY FIRST) OR listen twice
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener tfl(buffer);
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::PoseStamped tempPose, otherPose;
	geometry_msgs::PoseStamped emptyPose;
	try {
		transformStamped = buffer.lookupTransform(globalFrame, robotFrameName, ros::Time(0), ros::Duration(3.0));
	} catch (tf2::TransformException &e) {
		ROS_WARN("Error with transform");
	}
	tf2::doTransform(tempPose, otherPose, transformStamped);
	getCurrentPose();
	if (norm(otherPose,currentPose) <= commRadius){return otherPose;}
	return emptyPose;
}


void Robot::updateProcessingGoal(bool processingGoal_){
	processingGoal = processingGoal_;
}

void Robot::updateExplorationResults(int failed, int succeeded){
	failedRun += failed;
	frontiersExplored += succeeded;
}

void Robot::addFailedFrontiers(geometry_msgs::PoseStamped pose){
	failedFrontiers.push_back(pose);
	updateEnvFailed();

}

void Robot::updateEnv(){
	updateEnvCurrentPose();
	updateEnvTeamGoal();
	updateEnvRobotTeamPose();
	robotEnvironment.bUpdate = false;
	robotEnvironment.bUpdateRobotPose = false;
	robotEnvironment.bUpdateTeamGoalPose = false;
}

void Robot::updateEnvFailed(){
	robotEnvironment.updateFailedFrontiers(failedFrontiers);
}

bool Robot::updateGoal(geometry_msgs::PoseStamped goalPose_){
	goalPose = goalPose_;
	return true;
}


void Robot::updateEnvCurrentPose(){
	getCurrentPose();
	//ROS_INFO("Updated Curr Pose");
	robotEnvironment.updateRobotPose(currentPose);
}

void Robot::updateEnvTeamGoal(){
	ROS_INFO("UPDATING ENV GOAL");
	robotEnvironment.updateTeamGoalPose(teamGoalPose);
}

void Robot::updateEnvRobotTeamPose(){
	getTeamPoses();
	robotEnvironment.updateRobotTeamPose(teamPose);
}


void Robot::updateMBG(){
	//Convert pose from global frame to robot map frame for move_base
	geometry_msgs::PoseStamped rfPose = convertToRobotFrame(goalPose);
	moveBaseGoal.target_pose = rfPose;
}

geometry_msgs::PoseStamped Robot::convertToRobotFrame(geometry_msgs::PoseStamped pose_){
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfl(buffer);
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped rfPose;
    try {
        transformStamped = buffer.lookupTransform(robotMapFrame, globalFrame, ros::Time(0), ros::Duration(3.0));
    } catch (tf2::TransformException &e) {
    	ROS_WARN("Error with transform");
    }
    //transform input pose_ to rfPose
    tf2::doTransform(pose_, rfPose, transformStamped);
    return rfPose;
}


//Tentatively dont use waitforresult!
/*bool Robot::waitForResult(){
	moveBaseClient.waitForResult();
	if (moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		goalReached = true;
		return true;
	}
	goalReached = false;
	return false;
}*/

void Robot::getTeamPoses(){
	for (int i = 1; i < teamSize+1; i++){
		geometry_msgs::PoseStamped otherPose;
		std::string otherRobotName = "robot_" + std::to_string(i);
		if (otherRobotName.compare(robotName) == 0){
			continue;
		}
		// todo: avoid hard code
		std::string targetFrame = otherRobotName + "/base_link";
		otherPose = inCommunicationRange(targetFrame);

		ROS_INFO("Obtained %s 's pose: %f, %f", otherRobotName.c_str(), otherPose.pose.position.x, otherPose.pose.position.y);

		// e.g. {robot_2: PoseStamped}
		teamPose[otherRobotName] = otherPose;
	}
}







