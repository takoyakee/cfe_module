#include "ros/ros.h"
#include <fyp_api/functions.h>
#include <fyp_api/environment.h>
#include <../../../devel/include/fyp_api/distMetrics.h>


namespace Robot_ns{
	struct RobotParameters{
		//To Load:
		std::string robotName,globalFrame,baseName, waypointFrame;
		std::string robotMapFrame,filePath;
		double commRad;
		float dist;
		int teamSize, goalHorizon;

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
			void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
			ros::Subscriber odomSub;
			std::ofstream file, posefile;
			int rate, counter, distcounter, distrate;
			float distTravelledSince;
			bool isInitialised(), firstexploration;
			void writeFile(std::ofstream& file, Environment_ns::Frontier f);
			void writePoseFile(std::ofstream& file, Environment_ns::Frontier f);


			geometry_msgs::PoseStamped currentPose, goalPose;
			void updatePoses();
	        move_base_msgs::MoveBaseGoal moveBaseGoal;
	        move_base_msgs::MoveBaseGoal getMBG();
	        actionlib::SimpleClientGoalState status{actionlib::SimpleClientGoalState::PENDING};
	        void updateMBG();
	        void teamGoalCallBack(const geometry_msgs::PoseStamped& msg);
	        nav_msgs::Path trajectory;
	        void addTrajectory();
	        ros::Publisher trajPub, posePub, distPub;

		private:
			RobotParameters RP;

			void Initialise(ros::NodeHandle& nh_, ros::NodeHandle& nh_private);
			bool getLatestFrontier();
			void setNewGoal();
			bool getEnvFrontier();
			void rankEnvFrontier(std::priority_queue<Environment_ns::Frontier> candidates);
			Environment_ns::Frontier chooseFrontier();


			//Status indicators
			bool envInitialised, poseInitialised ,goalReceived;
			bool searchEnded;

			bool processingGoal; //represents that Robot is in motion
			bool goalReached;
			bool recoveryState;



			std::map<std::string, geometry_msgs::PoseStamped> teamPose,teamGoalPose;
			std::priority_queue<Environment_ns::Frontier> frontierCandidates, closeFrontier,rejectedFrontier;
			Environment_ns::Frontier frontierCandidate{0,0};

			//get goal position, get other robots position if within communication range, exchange map info

			bool updateGoal(geometry_msgs::PoseStamped goalPose_);
			geometry_msgs::PoseStamped inCommunicationRange(std::string robotFrameName);
			geometry_msgs::PoseStamped transformToTF(geometry_msgs::PoseStamped pose, std::string targetFrame);
			geometry_msgs::PoseStamped transformToGF(geometry_msgs::PoseStamped pose);

			//2D Explorations
			geometry_msgs::PoseStamped getCurrentPose();
			void getTeamPoses();
			void updateEnvPoses();

	};
}
