#include <fyp/robot.h>

Robot::Robot(std::string robotName_, std::string robotMapFrame_, std::string globalFrame_, std::string robotFrame_, std::string frontierGoal_)
{
    robotName = robotName_;
    robotMapFrame = robotMapFrame_;
    globalFrame = globalFrame_;
    //e.g. robot_1/baselink
    robotFrame = robotFrame_;
    frontierGoal = frontierGoal_;
    currentPose = getCurrentPose();
    hasGoal = false;
    processingGoal = false;
    goalReached = false;
    commRadius = 30;
    failedFrontiers = 0;
    frontiersExplored = 0;
}

void Robot::explore(){
	ROS_INFO("Robot.explore() called!");
	if (robotEnvironment.isEnvironmentInitialised()){
		bool successfulCandidate = getFrontierCandidates();
		ROS_INFO("Success: %d", successfulCandidate);
		if (!processingGoal && successfulCandidate){
			updateGoal(frontierCandidate.pose);
			updateMBG();
			hasGoal = true;
		}
	} else {
		updateEnvCurrentPose();
		updateEnvTeamPose();
		robotEnvironment.bUpdateRobotPose = false;
		robotEnvironment.bUpdateRobotTeamPoses = false;
	}
}

void Robot::teamGoalCallBack(const geometry_msgs::PoseStamped& msg){
	ROS_INFO("TEAM GOAL CALL BACK!");
	//only consider if pose published is not own (e.g. robot_1/map)
	if (!msg.header.frame_id.empty()){
		std::string poseMapFrame = msg.header.frame_id.substr(1, poseMapFrame.length()-2);
		std::string robotFrameName =  eraseSubStr(poseMapFrame, "/map") + "/base_link"; // robot_1/base_link -> to review and avoid hardcode
		//ROS_INFO("robotMapFrame: %s vs received goal: %s", robotMapFrame.c_str(), poseMapFrame.c_str());
		if (poseMapFrame.compare(robotMapFrame) != 1){
			if (inCommunicationRange(robotFrameName)){
				ROS_INFO("Added to teamGoalPose!");
				teamGoalPose[poseMapFrame] = msg;
			}
		}
	}
}

void Robot::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	//ROS_INFO("CURRENT map size: %d of dimension (%d x %d) ",robotEnvironment.occupancyGrid.data.size(),robotEnvironment.envWidth, robotEnvironment.envHeight);
	//ROS_INFO("RECEIVED map size: %d of dimension (%d x %d) ",msg->data.size(),msg->info.width, msg->info.height);
	if (msg->data.size()> 0){
		robotEnvironment.updateOccupancyGrid(msg);
	}
	ROS_INFO("ALTERED map size: %d of dimension (%d x %d) ",robotEnvironment.occupancyGrid.data.size(),robotEnvironment.envWidth, robotEnvironment.envHeight);

}

bool Robot::getFrontierCandidates(){
	ROS_INFO("Getting Frontier Candidates...");
	//Feed latest currentPose and TeamPose
	updateEnvTeamPose();
	updateEnvCurrentPose();
	frontierCandidate = robotEnvironment.returnFrontierChoice();
	if (frontierCandidate.pose.header.frame_id.empty()){
		return false;
	} return true;
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
    	ROS_WARN("Error with transform");
    }

    tf2::doTransform(tempPose, currentPose, transformStamped);
    return currentPose;

}

bool Robot::inCommunicationRange(std::string robotFrameName){
	//todo: Obtain other robots poses via tf, evaluate distance before appending to vector
	// two ways: calculate norm (TRY FIRST) OR listen twice
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener tfl(buffer);
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::PoseStamped tempPose, otherPose;
	try {
		transformStamped = buffer.lookupTransform(globalFrame, robotFrameName, ros::Time(0), ros::Duration(3.0));
	} catch (tf2::TransformException &e) {
		ROS_WARN("Error with transform");
	}
	tf2::doTransform(tempPose, otherPose, transformStamped);
	getCurrentPose();
	if (norm(otherPose,currentPose) <= commRadius){return true;}
	return false;
}


void Robot::updateProcessingGoal(bool processingGoal_){
	processingGoal = processingGoal_;
}

void Robot::updateExplorationResults(int failed, int succeeded){
	failedFrontiers += failed;
	frontiersExplored += succeeded;
}

bool Robot::updateGoal(geometry_msgs::PoseStamped goalPose_){
	goalPose = goalPose_;
	return true;
}


void Robot::updateEnvCurrentPose(){
	getCurrentPose();
	robotEnvironment.updateRobotPose(currentPose);
}

void Robot::updateEnvTeamPose(){
	robotEnvironment.updateRobotTeamPoses(teamGoalPose);
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

/*std::vector<geometry_msgs::PoseStamped> Robot::getTeamPoses(){
	teamPose.clear();
	for (int i = 1; i < teamSize+1; i++){
		std::string tempName = "robot_" + std::to_string(i);
		if (tempName.compare(robotName) == 0){
			continue;
		}
		// todo: avoid hard code
		std::string targetFrame = tempName + "/base_link";

		//todo: Obtain other robots poses via tf, evaluate distance before appending to vector
		// two ways: calculate norm (TRY FIRST) OR listen twice
	    tf2_ros::Buffer buffer;
	    tf2_ros::TransformListener tfl(buffer);
	    geometry_msgs::TransformStamped transformStamped;
	    geometry_msgs::PoseStamped tempPose;
	    geometry_msgs::PoseStamped otherPose;
	    try {
	        transformStamped = buffer.lookupTransform(globalFrame, targetFrame, ros::Time(0), ros::Duration(3.0));
	    } catch (tf2::TransformException &e) {
	    	ROS_WARN("Error with transform");
	    }
	    tf2::doTransform(tempPose, otherPose, transformStamped);
	    getCurrentPose();
	    if (norm(otherPose,currentPose) <= commRadius) {teamPose.push_back(otherPose);}
	}
	return teamPose;

}*/







