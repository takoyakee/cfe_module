#include <fyp/robot.h>

Robot::Robot(std::string robotName_, std::string globalFrame_, std::string robotFrame_, int teamSize_)
{
    robotName = robotName_;
    globalFrame = globalFrame_;
    //e.g. robot_1/baselink
    robotFrame = robotFrame_;
    currentPose = getCurrentPose();
    teamSize = teamSize_;
    hasGoal = false;
    processingGoal = false;
    goalReached = false;
    commRadius = 30;
    failedAttempts = 0;
    frontiersExplored = 0;

}


geometry_msgs::PoseStamped Robot::Robot::getCurrentPose(){
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

std::vector<geometry_msgs::PoseStamped> Robot::getTeamPoses(){
	teamPoses.clear();
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
	    if (norm(otherPose,currentPose) <= commRadius) {teamPoses.push_back(otherPose);}
	}
	return teamPoses;

}

bool Robot::updateGoal(geometry_msgs::PoseStamped goalPose_){
	goalPose = goalPose_;
	return true;
}


void Robot::mapCallBack(const nav_msgs::OccupancyGrid& msg){
	if (msg.data.size()> 0){
		robotEnvironment.updateOccupancyGrid(msg);
	}
	//ROS_INFO("Callback! Map Size: %d",robotEnvironment.occupancyGrid.data.size());
}

void Robot::updateEnvCurrentPose(){
	getCurrentPose();
	robotEnvironment.updateRobotPose(currentPose);
}

void Robot::updateEnvTeamPose(){
	getTeamPoses();
	robotEnvironment.updateRobotTeamPoses(teamPoses);
}


void Robot::getFrontierCandidates(){
	//Feed latest currentPose and TeamPose
	updateEnvTeamPose();
	updateEnvCurrentPose();
	frontierCandidates = std::move(robotEnvironment.returnFrontierChoice());
}

//todo: If robot cannot find suitable position to move to, let him move randomly.
geometry_msgs::PoseStamped Robot::getFrontier(){
	if (frontierCandidates.empty()){
		if (!processingGoal){
			getFrontierCandidates();
		} else {
			throw "Something went wrong!";
		}
	}
	geometry_msgs::PoseStamped pose = frontierCandidates.front();
	frontierCandidates.erase(frontierCandidates.begin());
	return pose;
}

void Robot::explore(){
	if (!processingGoal){
		geometry_msgs::PoseStamped pose = getFrontier();
		updateGoal(pose);
		updateMBG();
		processingGoal = sendGoal();
	}

}


void Robot::updateMBG(){
	//Convert pose from global frame to robot map frame for move_base
	geometry_msgs::PoseStamped rfPose = convertToRobotFrame(goalPose);
	moveBaseGoal.target_pose = rfPose;
}


bool Robot::sendGoal(){
	while(!moveBaseClient.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for server to be connected");
	}

	ROS_INFO("Sending goal...");
	//consider sendGoalAndWait
	moveBaseClient.sendGoal(moveBaseGoal);
	ROS_INFO("Goal sent...");
	return true;

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






