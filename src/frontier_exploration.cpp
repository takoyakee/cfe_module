#include <ros/ros.h>
#include <fyp/robot.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"exploration_module");
    ros::NodeHandle nh;
    //require nodeName to access param values
    std::string nodeName = ros::this_node::getName();

    //Loading launch file param values (default values e.g. "/robot_1/map")
    std::string robotName_, globalFrame_, robotFrame_, mapTopic_;
    int teamSize_;
    ros::param::param<std::string>(nodeName+"/robot_name", robotName_, "robot_1");
    ros::param::param<std::string>(nodeName+"/global_frame", globalFrame_, "robot_1/map"); 
    ros::param::param<std::string>(nodeName+"/robot_frame", robotFrame_, "robot_1/base_link"); 
    ros::param::param<std::string>(nodeName+"/map_topic", mapTopic_, "/robot_1/map");
    ros::param::param<int>(nodeName+"/team_size", teamSize_, 1);

    //initialise instance of robot
    Robot robot(robotName_, globalFrame_, robotFrame_, teamSize_);

    //Subscribe to map
    ros::Subscriber sub = nh.subscribe(mapTopic_,1, &Robot::mapCallBack, &robot);

    ros::Rate rate(10);

    //todo: Better construct this!
    while (ros::ok()){
    	robot.explore();
    	//blocks all activity until goal is reached.. but is that what we really want?
    	if (!robot.goalReached){
    		robot.failedAttempts++;
    	} else {
    		robot.frontiersExplored++;
    	}
    	robot.goalReached = false;

    	ros::spinOnce();
    	rate.sleep();

    }

}


