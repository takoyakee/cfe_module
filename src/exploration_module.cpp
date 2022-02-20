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
	std::string robotName;
	nh_private.getParam("robot_name", robotName);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfl(buffer);
    costmap_2d::Costmap2DROS* costMap(NULL);
    costMap = new costmap_2d::Costmap2DROS("fcostmap",buffer);
    navfn::NavfnROS* planner(NULL);
    planner = new navfn::NavfnROS(std::string("fplanner"), costMap);

	Planner_ns::SensorManager sensorManager(nh,nh_private,planner);

	ros::Rate rate(1);
/*	ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("frontiermap", 1, true);

	  nav_msgs::OccupancyGrid map;
	  map.header.stamp = ros::Time::now();
	  map.header.frame_id = costMap.getGlobalFrameID();

	  costmap_2d::Costmap2D explore_costmap = *costMap.getCostmap();

	    map.info.width = explore_costmap.getSizeInCellsX();
	    map.info.height = explore_costmap.getSizeInCellsY();
	    map.info.resolution = explore_costmap.getResolution();
	    map.info.origin.position.x = explore_costmap.getOriginX();
	    map.info.origin.position.y = explore_costmap.getOriginY();
	    map.info.origin.position.z = 0;
	    map.info.origin.orientation.x = 0;
	    map.info.origin.orientation.y = 0;
	    map.info.origin.orientation.z = 0;
	    map.info.origin.orientation.w = 1;

	    int size = map.info.width * map.info.height;*/

	while (ros::ok()){

		sensorManager.execute();

/*	    const unsigned char* char_map = explore_costmap.getCharMap();

	    map.data.resize((size_t)size);
	    for (int i=0; i<size; i++) {
	      if (char_map[i] == costmap_2d::NO_INFORMATION)
	      	map.data[i] = -1;
	      else if (char_map[i] == costmap_2d::LETHAL_OBSTACLE)
	      	map.data[i] = 100;
	      else
	      	map.data[i] = 0;
	    }

	    map_pub.publish(map);*/
	    ros::spinOnce();
	}

    if (costMap != NULL){
    	delete costMap;
	}
	if (planner != NULL){
		delete planner;
	}
	return 1;
}


