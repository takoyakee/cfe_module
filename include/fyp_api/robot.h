#include "ros/ros.h"
#include <fyp_api/functions.h>
#include <fyp_api/environment.h>

namespace Robot_ns{
	struct RobotParameters{
		//To Load:
		std::string robotName,globalFrame,baseName, waypointFrame;
		std::string robotMapFrame;
		double commRad;
		int teamSize;

		bool loadParameters(ros::NodeHandle& nh_private);
	};

	class Robot{

		public:
			//Constructor
			Robot(ros::NodeHandle& nh_, ros::NodeHandle& nh_private);
			std::unique_ptr<Environment_ns::Environment> robotEnvironment;

			//Most important functionality
			bool explore();
			void updateRobotPose(geometry_msgs::PoseStamped poseStamped);
			void updateEnvGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid_);
			void updateEnvLocalFrontier(pcl::PointCloud<pcl::PointXYZI> frontierCloud);
			geometry_msgs::PointStamped getWayPoint();
			void addPosVertex(Eigen::Vector3d pos);
			void addFrontierVertex(geometry_msgs::PointStamped point);
			bool hasGoal;

			//////////////////// 2D EXPLORATION //////////////////////
			void setEnvPlanner(navfn::NavfnROS* planner);
			void updatePoses();
	        move_base_msgs::MoveBaseGoal moveBaseGoal;
	        move_base_msgs::MoveBaseGoal getMBG();
	        actionlib::SimpleClientGoalState status{actionlib::SimpleClientGoalState::PENDING};
	        void updateMBG();
	        void teamGoalCallBack(const geometry_msgs::PoseStamped& msg);
	        nav_msgs::Path trajectory;
	        void addPoseToPath();
	        ros::Publisher trajPub;

		private:
			RobotParameters RP;

			void Initialise(ros::NodeHandle& nh_, ros::NodeHandle& nh_private);
			bool getLatestFrontier();
			void setNewGoal();
			bool getEnvFrontier();
			void rankEnvFrontier(std::priority_queue<Environment_ns::Frontier> candidates);
			Environment_ns::Frontier chooseFrontier();


			//Status indicators
			bool envInitialised, poseInitialised;
			bool searchEnded;

			bool processingGoal; //represents that Robot is in motion
			bool goalReached;
			bool recoveryState;

			geometry_msgs::PoseStamped currentPose, goalPose;

			std::map<std::string, geometry_msgs::PoseStamped> teamPose,teamGoalPose;
			std::priority_queue<Environment_ns::Frontier> frontierCandidates, closeFrontier,rejectedFrontier;
			Environment_ns::Frontier frontierCandidate{0,0,0};

			//get goal position, get other robots position if within communication range, exchange map info

			bool updateGoal(geometry_msgs::PoseStamped goalPose_);
			geometry_msgs::PoseStamped inCommunicationRange(std::string robotFrameName);
			geometry_msgs::PoseStamped transformToTF(geometry_msgs::PoseStamped pose, std::string targetFrame);
			geometry_msgs::PoseStamped transformToGF(geometry_msgs::PoseStamped pose);

			//2D Explorations
			bool isInitialised();
			geometry_msgs::PoseStamped getCurrentPose();
			void getTeamPoses();
			void updateEnvPoses();

	};
}
