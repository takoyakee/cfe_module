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
    failedRun = 0;
    frontiersExplored = 0;
    robotEnvironment.globalFrame = globalFrame;
    teamSize = 2;
}

void Robot::explore(){
	ROS_INFO("Robot.explore() called!");
	if (robotEnvironment.occupancyGrid.data.size() > 0) {
		while (!robotEnvironment.isEnvUpdated()){
			updateEnvCurrentPose();
			updateEnvTeamGoal();
			updateEnvRobotTeamPose();
			robotEnvironment.bUpdateRobotPose = false;
			robotEnvironment.bUpdateTeamGoalPose = false;
		}
		bool successfulCandidate = getFrontierCandidates();
		if (successfulCandidate){
			updateGoal(frontierCandidate.pose);
			updateMBG();
			hasGoal = true;
		}
	}

}

bool Robot::getFrontierCandidates(){
	ROS_INFO("Getting Frontier Candidates...");
	//Feed latest currentPose and TeamPose
	frontierCandidate = robotEnvironment.returnFrontierChoice();
	if (frontierCandidate.pose.header.frame_id.empty()){
		return false;
	} return true;
}

void Robot::teamGoalCallBack(const geometry_msgs::PoseStamped& msg){
	//only consider if pose published is not own (e.g. robot_1/map)
	if (!msg.header.frame_id.empty()){
		std::string poseMapFrame = msg.header.frame_id.substr(1, poseMapFrame.length()-2);
		std::string robotFrameName =  eraseSubStr(poseMapFrame, "/map") + "/base_link"; // robot_1/base_link -> to review and avoid hardcode
		if (poseMapFrame.compare(robotMapFrame) != 0){
			if (!inCommunicationRange(robotFrameName).header.frame_id.empty()){
				ROS_INFO("Goal from %s is registered", poseMapFrame);

				//e.g. {robot_2/map, PoseStamped}
				teamGoalPose[poseMapFrame] = msg;
			}
		}
	}
}

void Robot::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	//ROS_INFO("CURRENT map size: %d of dimension (%d x %d) ",robotEnvironment.occupancyGrid.data.size(),robotEnvironment.envWidth, robotEnvironment.envHeight);
	ROS_INFO("RECEIVED map size: %d of dimension (%d x %d) ",msg->data.size(),msg->info.width, msg->info.height);
	if (msg->data.size()> 0){
		robotEnvironment.updateOccupancyGrid(msg);
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
    	ROS_WARN("Error with transform");
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

void Robot::updateEnvFailed(){
	robotEnvironment.updateFailedFrontiers(failedFrontiers);
}

bool Robot::updateGoal(geometry_msgs::PoseStamped goalPose_){
	goalPose = goalPose_;
	return true;
}


void Robot::updateEnvCurrentPose(){
	getCurrentPose();
	ROS_INFO("Updated Curr Pose");
	robotEnvironment.updateRobotPose(currentPose);
}

void Robot::updateEnvTeamGoal(){
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
	teamPose.clear();
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







