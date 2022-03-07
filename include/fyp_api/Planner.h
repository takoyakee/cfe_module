/*
 * frontierplanner.h
 *
 *  Created on: 11 Oct 2021
 *      Author: yanling
 */

#include <fyp_api/robot.h>
#include <fyp_api/GridManager.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace Planner_ns{

	struct SensorParameters{
		std::string planType, robotName, sharedGoalTopic, globalmapTopic, costmapTopic;
		std::string globalFrame;
		std::string waypointFrame;
		std::string stateTopic;
		std::string scanTopic;
		std::string velodyneTopic;
		std::string waypointTopic;
		std::string mergemapTopic;
		std::string sharedStatusTopic;
		bool loadParameters(ros::NodeHandle& nh_private);

	};

	struct PCParameters{
		float leafSize;
		  double kRadiusThreshold;
		  double kZDiffMax;
		  double kZDiffMin;
		  int kNeighborThreshold;
		  int kFrontierClusterMinSize;
		bool loadParameters(ros::NodeHandle& nh_private);
	};


	class SensorManager{
		public:
			SensorManager(ros::NodeHandle& nh_, ros::NodeHandle& nh_private,
					navfn::NavfnROS* planner_);
			bool execute();
			bool isInitialised();

			//2D PLANNING
			navfn::NavfnROS* planner;
		private:
		    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
		    bool finishedExploration;
			 enum CellState : char
			  {
			    UNKNOWN = -1,
			    OCCUPIED = 127,
			    FREE = 0,
			    NOT_FRONTIER = 3
			  };

			SensorParameters SP;
			PCParameters pcP;

			ros::Subscriber stateSub;
			ros::Subscriber scanSub;
			ros::Subscriber velodyneSub;
			ros::Subscriber statusSub;

			ros::Publisher waypointPub;
			ros::Publisher localOGPub;
			ros::Publisher globalOGPub;
			ros::Publisher frontierpub;
			ros::Publisher filteredpub;
			ros::Publisher clusteredpub;
			ros::Publisher occludedpub;
			ros::Publisher freepub;

			pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud;
			pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud;
			pcl::PointCloud<pcl::PointXYZI>::Ptr frontier_plane, frontier_centroids;

			Eigen::Vector3d currPose;
			std::vector<int> updatedScanIdx;
			float distanceTravelled;
			int desiredRate, counter;

			std::unique_ptr<Robot_ns::Robot> robotController;
			std::unique_ptr<GridManager_ns::GridManager> worldGrid;
			std::unique_ptr<GridManager_ns::GridManager> localGrid;

			void Initialise(ros::NodeHandle& nh_, ros::NodeHandle& nh_private);
			void stateCallBack(const nav_msgs::Odometry::ConstPtr& odomMsg);
			void scanCallBack(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_);
			void velodyneCallBack(const sensor_msgs::PointCloud2ConstPtr& scanMsg);
			void mergemapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_);
			void statusCallBack(const std_msgs::Bool msg);

			void sendLocalFrontier();
			void downsizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
			void updateFrontierCentroids(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in);
			void extractFrontierPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out,
					double z_max = DBL_MAX, double z_min= - DBL_MAX);
			void clusterFrontier(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

			double distBtwPose(Eigen::Vector3d p1, Eigen::Vector3d p2);
			void updateCurrPose(const nav_msgs::Odometry odom);
			geometry_msgs::PoseStamped odomToPoseStamped(const nav_msgs::Odometry odom);

			////////////////////////// 2D ///////////////////
			MoveBaseClient moveBaseClient;
			ros::Subscriber globalMapSub, goalSub;
		    ros::Publisher goalPub;
		    void globalMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& globalOG);
		    void goalCallBack(const geometry_msgs::PoseStamped& msg);
		    //void selfMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_);


	};

}
