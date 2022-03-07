/*
 * exploration_module.cpp
 *
 *  Created on: 2 Nov 2021
 *      Author: yanling
 */
#include <fyp_api/Planner.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "frontier_exploration");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfl(buffer);
    costmap_2d::Costmap2DROS* costMap(NULL);
    costMap = new costmap_2d::Costmap2DROS("navfn",buffer);
    navfn::NavfnROS* planner(NULL);
    planner = new navfn::NavfnROS(std::string("fplanner"), costMap);
	Planner_ns::SensorManager sensorManager(nh,nh_private,planner);

	double iterFreq;
	std::string param_ns = "/frontier_common/";
	nh.getParam(param_ns+"iter_freq", iterFreq);
	ros::Rate rate(iterFreq);

/*
	std::string robotName;
	double bufferRate = 1;
	nh_private.getParam("robot_name", robotName);
	int bufferNum = atoi(&robotName.back());
	ROS_INFO("%d", bufferNum);
	ros::Duration bufferTime(bufferRate*bufferNum);
	bool delay = true;
*/
	bool stop = false;

	while (ros::ok()){
	    ros::spinOnce();
/*		if (sensorManager.isInitialised() && delay){
			ROS_INFO("DELAYING..");
			bufferTime.sleep();
			delay=false;
		}*/
		//ROS_INFO("is Initialised: %d", sensorManager.isInitialised());

		stop = sensorManager.execute();

		rate.sleep();
		if (stop){
			break;
		}


	}
/*    if (costMap != NULL){
    	delete costMap;
	}
	if (planner != NULL){
		delete planner;
	}*/
	ROS_INFO("Save maps now...");
	ros::shutdown();
	return 0;
}


