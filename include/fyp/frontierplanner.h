/*
 * frontierplanner.h
 *
 *  Created on: 11 Oct 2021
 *      Author: yanling
 */

#ifndef FRONTIERPLANNER_H_
#define FRONTIERPLANNER_H_

#include <fyp/functions.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/inflation_layer.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace frontier_planner{

class FrontierPlanner : public nav_core::BaseGlobalPlanner{
public:
	FrontierPlanner();
	FrontierPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	/** Overriden classes from interface nav_core::BaseGlobalPlanner**/
	void initialise(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	bool makePlan(const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal,
			std::vector<geometry_msgs::PoseStamped>& plan);
};
};

#endif /* FRONTIERPLANNER_H_ */
