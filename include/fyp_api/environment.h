/*
 * environment.h
 *
 *  Created on: 8 Oct 2021
 *      Author: yanling
 */
#include <fyp_api/functions.h>
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
#include <fyp_api/Visualisation.h>
#include <geometry_msgs/PoseArray.h>


namespace Environment_ns{
	//consider using BST or kdtree
	struct Node
	{
		Eigen::Vector2i cell;
		Eigen::Vector3d pos;
		Node* parent;
	};

	struct edge{
		bool isEdge;
		float gradient;
		edge(){
			isEdge = false;
			gradient = 0;
		}

	};
	struct Frontier {
		int x;
		int y;
		double utility,grad;
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
		std::vector<int> key();

		Frontier(int x_,int y_);
	};

	struct EnvironmentParameters{
		//To load:
		std::string globalFrame, robotName, robotMapFrame, sharedFrontierTopic, baseName;
		float discountMult, costMult, obsMult, oobMult;
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
			fyp_api::centroidArray assigned;
			geometry_msgs::Point scanOrigin, offSet, currPt; //can consider using a point if thats the type
			navfn::NavfnROS* planner;
			nav_msgs::Path navPath, bPath;
			std::vector<std::vector<int>> badCoord, inaccessible;
			std::vector<float> color, pose_weight;
			geometry_msgs::TransformStamped global2Own, own2Global;


			void setPlanner(navfn::NavfnROS* planner_);
			void updateGlobalOrigin(const nav_msgs::OccupancyGrid::ConstPtr& globalOG_);
			void computeoffSet();
			void centroidCB(fyp_api::centroidArray c);
			void publishCentroid();
			void compileCentroid();
			bool findFeasibleCentroid(Frontier& centroid);
			void findInMap(Frontier& frontier);
			void findNearestUnknown(int& cx_, int& cy_, float angle);
			void insert(std::map<std::vector<int>, Frontier>& dict, Frontier elem);

			geometry_msgs::Point coordToPt(int cx_, int cy_);
			std::vector<int> ptToCoord(geometry_msgs::Point pt);
			bool isKnown(int cx_, int cy_);
			//so that conversion such as coordtopoint can be used
			void setParamsForAssigner(std::string RobotName, std::string robotMapFrame, std::string GlobalFrame);



		private:
			EnvironmentParameters EP;

			ros::Publisher centroid_pub, brensenham_pub, path_pub, clusteredpub;
			ros::Subscriber centroid_sub;

			//Grids for frontier allocations
			int** frontierGrid;
			std::map<std::vector<int>,float>  grads;
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

			std::vector<std::vector<int>> frontierCells, visCentroid, tempCells;
			std::map<std::vector<int>, Frontier> ownCentroid, finalCentroid;
			std::priority_queue<Frontier> frontierPQ;


			void Initialise(ros::NodeHandle nh_, ros::NodeHandle nh_private);

			//**************IMPORTANT*********************//
			//Utility evaluation
			std::vector<std::vector<int>> getFrontierCells();
			void clusterFrontier(std::vector<std::vector<int>> frontierCells);

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
			edge edgeGrad(int cx_, int cy_);

			int coordToIndex(int cx_, int cy_);
			std::vector<float> coordToPoint(int cx_, int cy_);
			geometry_msgs::PoseStamped coordToPS (int cx_, int cy_);
			std::vector<int> pointToCoord(float mx_, float my_);
			std::vector<int> indexToCoord(int index_);

			bool inMap(int cx_, int cy_);
			edge isFrontier(int cx_, int cy_);
			bool isEdge(int cx_, int cy_);
			bool isFree(int cx_, int cy_);

			int cap(int value_);




	};
}


#endif /* FYP_INCLUDE_FYP_ENVIRONMENT_H_ */
