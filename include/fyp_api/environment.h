/*
 * environment.h
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */
#include <fyp_api/functions.h>
#include <fyp_api/Visualisation.h>
#include <iostream>
#include <fstream>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <navfn/navfn.h>

#ifndef FYP_INCLUDE_FYP_ENVIRONMENT_H_
#define FYP_INCLUDE_FYP_ENVIRONMENT_H_

namespace Environment_ns{
	//consider using BST or kdtree
	struct Node
	{
		Eigen::Vector2i cell;
		Eigen::Vector3d pos;
		Node* parent;
	};

	struct Frontier {
		int x;
		int y;
		double utility;
		double cost, discount, ig, validate, penalty;
		geometry_msgs::PoseStamped pose;
		Frontier& operator =(const Frontier& f1);
		bool empty();
		bool operator==(const Frontier& rhs) const
		{
				return x == rhs.x && y == rhs.y;
		 }

		bool operator<(const Frontier& rhs) const
		{
				return utility < rhs.utility;
		 }

		void evaluate(float cweight,float dweight, float iweight);

		Frontier(int x_,int y_, double utility_);
	};

	struct EnvironmentParameters{
		//To load:
		std::string globalFrame, robotName, robotMapFrame, sharedFrontierTopic, baseName;
		float discountMult, costMult, obsMult;
		float cweight, dweight, iweight;
		float frontierThreshold;
		int searchRadius, clusterRad, clusterSize;
		float expirationTime;
		float localRange, localCellRange, resolution;

		// 2D ///
		std::string planType, costmap_topic;
		//To calculate and update:
		int envWidth, envHeight, maxDist, numCluster;
		float offSet_x, offSet_y;

		bool loadParameters(ros::NodeHandle nh_private);
	};


	class Environment {
		public:
			enum kcells{TL=1, T=2, TR = 3, L=4, R=6, BL=7, B=8, BR=9} ;

			//constructor
			Environment(ros::NodeHandle nh_, ros::NodeHandle nh_private);
			~Environment();

			std::priority_queue<Frontier> returnFrontiers();
			void updateOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_);
			bool isEnvInitialised();
			void updateRobotPose(geometry_msgs::PoseStamped currentPose_);
			void updateTeamGoalPose(std::map<std::string, geometry_msgs::PoseStamped> teamGoalPose_);
			void updateRobotTeamPose(std::map<std::string, geometry_msgs::PoseStamped> teamPose_);
			void updateLocalFrontierCloud(std::vector<Eigen::Vector3d> localFrontier_);
			void addEnvPosVertex(Eigen::Vector3d pos);
			void addEnvFrontierVertex(Eigen::Vector3d pos);


			////////// 2D PLANNING ///////
			std::unique_ptr<Vis_ns::Vis> vis;
			bool globalOGreceived, poseReceived, computePotential, plannerInitialised;
			fyp_api::centroidArray cpub;
			std::map<std::string, fyp_api::centroidArray> teamCentroids;
			geometry_msgs::Point scanOrigin, offSet, currPt; //can consider using a point if thats the type
			navfn::NavfnROS* planner;
			std::vector<std::vector<int>> pathCoord;
			std::vector<geometry_msgs::PoseStamped> navPS;

			void setPlanner(navfn::NavfnROS* planner_);
			void updateGlobalOrigin(const nav_msgs::OccupancyGrid::ConstPtr& globalOG_);
			void computeoffSet();
			void centroidCB(fyp_api::centroidArray c);
			void publishCentroid();
			void compileCentroid();
			geometry_msgs::Point coordToPt(int cx_, int cy_);
			bool isKnown(int cx_, int cy_);



		private:
			EnvironmentParameters EP;

			ros::Publisher frontiers_pub, centroid_pub, brensenham_pub,centroidmarker_pub, path_pub;
			ros::Publisher clusteredpub;
			ros::Publisher map_pub;
			ros::Subscriber centroid_sub, costmap_sub;

			//Grids for frontier allocations
			int** frontierGrid;
			pcl::PointCloud<pcl::PointXYZI>::Ptr poseCloud, frontierCloud;
			pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr posKDTree, frontierKDTree;

			//Current resolution: 0.1 (0.1m/cell)
			std::unique_ptr<nav_msgs::OccupancyGrid> occupancyGrid;
			std::vector<std::vector<int>> localFrontier;

			geometry_msgs::PoseStamped goalPose;
			geometry_msgs::PoseStamped currentPose;
			std::vector<float> currPos;
			std::vector<int> currCoord;
			std::vector<geometry_msgs::PoseStamped> teamGoalPose;
			std::vector<geometry_msgs::PoseStamped> teamPose;

			std::vector<std::vector<int>> frontierCells, centroidVector, tempCells;
			std::map<std::vector<int>, float> centroidMap;
			std::priority_queue<Frontier> frontierPQ;


			void Initialise(ros::NodeHandle nh_, ros::NodeHandle nh_private);

			//**************IMPORTANT*********************//
			//Utility evaluation
			std::vector<std::vector<int>> getFrontierCells();
			std::map<std::vector<int>,float> clusterFrontier(std::vector<std::vector<int>> frontierCells);

			std::priority_queue<Frontier> evaluateFrontiers();


			std::vector<int> interpolateCoord(std::vector<int> targetCell);
			double cellCost(int cx_, int cy_);
			double cellDiscount(int cx_, int cy_);
			std::map<std::vector<int>, double> cellValidate(int cx_, int cy_);
			double cellPenalty(int cx_, int cy_);
			void initialiseGrids();
			void deleteGrids();
			void resetGrids();

			//------ Helper functions ------
			double edgeGrad(int cx_, int cy_);

			int coordToIndex(int cx_, int cy_);
			std::vector<float> coordToPoint(int cx_, int cy_);
			geometry_msgs::PoseStamped coordToPS (int cx_, int cy_);
			std::vector<int> pointToCoord(float mx_, float my_);
			std::vector<int> indexToCoord(int index_);

			bool inMap(int cx_, int cy_);
			bool isFrontier(int cx_, int cy_);
			bool isEdge(int cx_, int cy_);
			bool isFree(int cx_, int cy_);

			int cap(int value_);




	};
}


#endif /* FYP_INCLUDE_FYP_ENVIRONMENT_H_ */
