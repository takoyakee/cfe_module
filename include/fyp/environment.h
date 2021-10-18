/*
 * environment.h
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */
#include <fyp/functions.h>
#include <queue>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>

#ifndef FYP_INCLUDE_FYP_ENVIRONMENT_H_
#define FYP_INCLUDE_FYP_ENVIRONMENT_H_

class Environment {

	public:
		enum kcells{TL=1, T=2, TR = 3, L=4, R=6, BL=7, B=8, BR=9} ;

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

		std::string globalFrame;
		int envWidth;
		int envHeight;
		bool bUpdateRobotPose;
		bool bUpdateRobotTeamPoses;

		//Current resolution: 0.1 (0.1m/cell)
		nav_msgs::OccupancyGrid occupancyGrid;

		geometry_msgs::PoseStamped goalPose;
		geometry_msgs::PoseStamped currentPose;
		std::vector<geometry_msgs::PoseStamped> robotTeamPoses;
		std::vector<std::vector<int>> failedCells;

		Frontier returnFrontierChoice();
		bool isEnvUpdated();
		void updateOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_);
		void updateRobotPose(geometry_msgs::PoseStamped currentPose_);
		void updateRobotTeamPoses(std::map<std::string, geometry_msgs::PoseStamped> robotTeamPoses_);
		void updateFailedFrontiers(std::vector<geometry_msgs::PoseStamped> failedFrontiers_);

		//todo: combine cost and discount grid, TO have updateInformationGrids ( have discount and i.g.)
		//discount to consider other robots' allocated goals
		//To keep track of other robot's location

	private:
		ros::NodeHandle nh_;
		ros::Publisher marker_pub;

		//Grids for frontier allocations
		char ** occupancy2D;
		double** discountGrid;
		double** informationGrid;
		unsigned char** costGrid;
		unsigned char** frontierGrid;

		// Storing previously changed cells
		std::vector<std::vector<int>> prevFrontierCells;
		std::vector<std::vector<int>> prevDCCells;
		std::vector<std::vector<int>> circleCorners;
		std::map<int,std::vector<int>> kernel;


		int maxDist;


		//Hyper parameters
		int searchRadius;
		int reserveSize;
		float discountMult;
		float costMult;
		float cweight;
		float dweight;
		float expirationTime;
		float frontierThreshold;

		void initialiseGrids();
		void deleteGrids();
		void resetGrids();
		visualization_msgs::Marker visualiseFrontier();

		void waitForRobotPose();
		void waitForRobotTeamPoses();

		//Internal conversion functions
		bool inMap(int cx_, int cy_);
		bool isEdge(int cx_, int cy_);
		bool inFailed(int cx_, int cy_);


		//**************IMPORTANT*********************//
		//Utility evaluation
		std::vector<std::vector<int>> getFrontierCells();
		bool updateCostCells();
		bool updateDiscountCells();
		bool updateIGCells();
		bool isFrontier(int cx_, int cy_);


		unsigned char costOfCell(int cx_, int cy_, std::vector<int> pos_);
		double distToDiscount(int dist);
		double infoOfCell(int cx_, int cy_);
		int cap(int value_);

		unsigned char evaluateUtility(int cx_, int cy_);
		double edgeGrad(int cx_, int cy_);
		int coordToIndex(int cx_, int cy_);
		std::vector<float> coordToPoint(int cx_, int cy_);
		std::vector<int> pointToCoord(float mx_, float my_);
		std::vector<int> indexToCoord(int index_);
		geometry_msgs::PoseStamped coordToPS (int cx_, int cy_);


};



#endif /* FYP_INCLUDE_FYP_ENVIRONMENT_H_ */
