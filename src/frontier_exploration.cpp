#include <ros/ros.h>
#include <fyp/robot.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char **argv)
{
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    ros::init(argc,argv,"exploration_module");
    ros::NodeHandle nh;
    //require nodeName to access param values
    std::string nodeName = ros::this_node::getName();

    //Loading launch file param values (default values e.g. "/robot_1/map")
    std::string robotName_, robotMapFrame_, globalFrame_, robotFrame_, mapTopic_, frontierGoal_;
    ros::param::param<std::string>(nodeName+"/robot_name", robotName_, "robot_1");
    ros::param::param<std::string>(nodeName+"/robot_map_frame", robotMapFrame_, "robot_1/map");
    ros::param::param<std::string>(nodeName+"/global_frame", globalFrame_, "robot_1/map"); 
    ros::param::param<std::string>(nodeName+"/robot_frame", robotFrame_, "robot_1/base_link"); 
    ros::param::param<std::string>(nodeName+"/map_topic", mapTopic_, "/robot_1/map");
    ros::param::param<std::string>(nodeName+"/frontier_goal", frontierGoal_, "/frontier/goal");

    //initialise instance of robot
    Robot robot(robotName_, robotMapFrame_, globalFrame_, robotFrame_, frontierGoal_);

    MoveBaseClient moveBaseClient{robotName_+"/move_base",true};
	while(!moveBaseClient.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for server to be connected");
	}

    //Ros Subscribers and Publishers
    ros::Subscriber mapSub = nh.subscribe(mapTopic_,1, &Robot::mapCallBack, &robot);
    ros::Subscriber teamGoalSub = nh.subscribe(frontierGoal_ , 10, &Robot::teamGoalCallBack, &robot);
    ros::Publisher goalPub = nh.advertise<geometry_msgs::PoseStamped>(frontierGoal_,10);

    ros::Rate rate(10);

    while (ros::ok()){
    	robot.explore();
    	actionlib::SimpleClientGoalState rStatus = moveBaseClient.getState();
    	if (!robot.processingGoal && robot.hasGoal){
    		moveBaseClient.sendGoal(robot.moveBaseGoal);
    		goalPub.publish(robot.goalPose);
    	} else if (rStatus == actionlib::SimpleClientGoalState::ACTIVE){
    		robot.updateProcessingGoal(true);
    		robot.hasGoal = false;

    	} else if (rStatus == actionlib::SimpleClientGoalState::SUCCEEDED){
    		robot.updateProcessingGoal(false);
    		robot.updateExplorationResults(0, 1);
       		robot.hasGoal = false;

    	} else if (rStatus == actionlib::SimpleClientGoalState::ABORTED){
    		robot.updateProcessingGoal(false);
    		robot.updateExplorationResults(1, 0);
       		robot.hasGoal = false;
    	}

    	ros::spinOnce();
    	rate.sleep();

    }

}


