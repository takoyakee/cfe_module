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

		struct Frontier {
			int x;
			int y;
			unsigned char utility;
			geometry_msgs::PoseStamped pose;
			Frontier& operator =(const Frontier& f1);
			Frontier(int x_,int y_,unsigned char utility_);
		};

		//constructor
		Environment();
		~Environment();

		int envWidth;
		int envHeight;

		//Current resolution: 0.1 (0.1m/cell)
		nav_msgs::OccupancyGrid occupancyGrid;


		//Poses to help in cost/discount calculations
		//todo: Consider keeping track of robot names
		geometry_msgs::PoseStamped goalPose;
		geometry_msgs::PoseStamped currentPose;
		//robotTeamPose should be robotTeamGoalPose
		std::vector<geometry_msgs::PoseStamped> robotTeamPoses;

		bool bUpdateRobotPose;
		bool bUpdateRobotTeamPoses;

		bool isEnvironmentInitialised();
		void updateOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_);
		void updateRobotPose(geometry_msgs::PoseStamped currentPose_);
		void waitForRobotPose();
		void updateRobotTeamPoses(std::map<std::string, geometry_msgs::PoseStamped> robotTeamPoses_);
		void waitForRobotTeamPoses();
		//todo: combine cost and discount grid, TO have updateInformationGrids ( have discount and i.g.)
		//discount to consider other robots' allocated goals

		//Updates selected cells (not entire map)
		bool updateCostCells();
		bool updateDiscountCells();
		std::vector<std::vector<int>> getFrontierCells();
		Frontier returnFrontierChoice();
		unsigned char evaluateUtility(int cx_, int cy_);


		unsigned char costOfCell(int cellRow, int cellCol, std::vector<int> pos_);

		//Resetting costGrid and discountGrid values
		void resetCostGrid();
		void resetDiscountGrid();

		//To keep track of other robot's location

	private:
		//initialising every Grid
		void initialiseGrids();
		void resetGrids();

		// Storing previously changed cells
		std::vector<std::vector<int>> prevFrontierCells;
		std::vector<std::vector<int>> prevDCCells;

		//Grids for frontier allocations
		char ** occupancy2D;
		double** discountGrid;
		unsigned char** costGrid;
		//frontier Grids
		unsigned char** frontierGrid;

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
		std::vector<float> coordToPoint(int cx_, int cy_);
		std::vector<int> pointToCoord(float mx_, float my_);
		std::vector<int> indexToCoord(int index_);
		double distToDiscount(int dist);
};



#endif /* FYP_INCLUDE_FYP_ENVIRONMENT_H_ */
