/*
 * environment.h
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */
#include <fyp/functions.h>
#include <queue>

#ifndef FYP_INCLUDE_FYP_ENVIRONMENT_H_
#define FYP_INCLUDE_FYP_ENVIRONMENT_H_

class Environment {


	public:
		//constructor
		Environment();
		~Environment();

		//Current resolution: 0.1 (0.1m/cell)
		nav_msgs::OccupancyGrid occupancyGrid;
		//costmap_2d::Costmap2DROS costMap;
		//grid_map::GridMap gridMap;

		//Poses to help in cost/discount calculations
		//todo: Consider keeping track of robot names
		geometry_msgs::PoseStamped currentPose;
		std::vector<geometry_msgs::PoseStamped> robotTeamPoses;

		//Grids for frontier allocations
		//todo: Possibly convert OccupancyGrid to occupancy2D to reeduce calling CoordToIndex
		//Allocate memory?
		char ** occupancy2D;
		unsigned char** discountGrid;
		unsigned char** costGrid;
		//frontier Grids
		unsigned char** frontierGrid;

		bool bUpdateRobotPose;
		bool bUpdateRobotTeamPoses;

		void updateOccupancyGrid(nav_msgs::OccupancyGrid occupancyGrid_);

		void updateRobotPose(geometry_msgs::PoseStamped currentPose_);
		void waitForRobotPose();
		void updateRobotTeamPoses(std::vector<geometry_msgs::PoseStamped> robotTeamPoses_);
		void waitForRobotTeamPoses();
		//todo: combine cost and discount grid, TO have updateInformationGrids ( have discount and i.g.)
		//discount to consider other robots' allocated goals

		//Updates selected cells (not entire map)
		bool updateCostCells();
		void updateDiscountCells();
		std::vector<std::vector<int>> getFrontierCells();
		std::vector<geometry_msgs::PoseStamped> returnFrontierChoice();

		char costOfCell(int cellRow, int cellCol, std::vector<int> pos_);

		//Resetting costGrid and discountGrid values
		void resetCostGrid();
		void resetDiscountGrid();

		//To keep track of other robot's location

	private:
		//initialising every Grid
		void initialiseGrids(nav_msgs::OccupancyGrid occupancyGrid_);
		void hardResetCostGrid();
		void hardResetDiscountGrid();

		// Storing previously changed cells
		std::vector<std::vector<int>> prevFrontierCells;
		std::vector<std::vector<int>> prevDCCells;

		int envWidth;
		int envHeight;
		int maxDist;
		std::string globalFrame;
		std::vector<std::vector<int>> neighbourCells;

		//Hyper parameters
		int searchRadius;
		int reserveSize;
		float discountMult;
		float costMult;
		float cweight;
		float dweight;
		float expirationTime;

		//Internal conversion functions
		bool inMap(int cx_, int cy_);

		int coordToIndex(int cx_, int cy_);
		std::vector<int> pointToCoord(float mx_, float my_);
		std::vector<int> indexToCoord(int index_);
		unsigned char distToDiscount(int dist);
};



#endif /* FYP_INCLUDE_FYP_ENVIRONMENT_H_ */
