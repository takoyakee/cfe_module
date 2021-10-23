#include "ros/ros.h"
#include <fyp/functions.h>
#include <fyp/environment.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>

class Robot{

    public: 
        //Constructor 
        Robot(std::string robotName_, std::string robotMapFrame_, std::string globalFrame_,
        		std::string robotFrame_, std::string mergeMapTopic_, std::string mapTopic_, std::string frontierGoal_);


        //NOTE: MBG is in robotmapframe (e.g. robot_2/map) whereas goalPose is in global frame
        move_base_msgs::MoveBaseGoal moveBaseGoal;
        geometry_msgs::PoseStamped goalPose;
        std::vector<geometry_msgs::PoseStamped> failedFrontiers;

        //Status indicators
        bool hasGoal;
        bool processingGoal; //represents that Robot is in motion
        bool goalReached;
		bool recoveryState;
        Environment robotEnvironment;
        actionlib::SimpleClientGoalState status{actionlib::SimpleClientGoalState::PENDING};

        void mergeMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void teamGoalCallBack(const geometry_msgs::PoseStamped& msg);

        //Most important functionality
        void explore();
        void updateProcessingGoal(bool processingGoal_);
        void updateExplorationResults(int failed, int succeeded);
        void addFailedFrontiers(geometry_msgs::PoseStamped pose);

    private:
        //info from main
        std::string robotName;
        std::string globalFrame;
        std::string robotFrame;
        std::string robotMapFrame;
        std::string mergeMapTopic;
        std::string mapTopic;
        std::string frontierGoal;

        int teamSize;
        int failedRun;
        int frontiersExplored;

        //hyper parameter

        float commRadius;

        geometry_msgs::PoseStamped currentPose;
        std::map<std::string, geometry_msgs::PoseStamped> teamPose;
        std::map<std::string, geometry_msgs::PoseStamped> teamGoalPose;
        std::priority_queue<Environment::Frontier> frontierCandidates;
        std::priority_queue<Environment::Frontier> closeFrontier;
        std::priority_queue<Environment::Frontier> rejectedFrontier;
        Environment::Frontier frontierCandidate{0,0,0};

        void rankFrontiers(std::priority_queue<Environment::Frontier> candidates);
        Environment::Frontier chooseFrontier();

        //get goal position, get other robots position if within communication range, exchange map info
        bool updateGoal(geometry_msgs::PoseStamped goalPose_);
        void updateMBG();
        geometry_msgs::PoseStamped convertToRobotFrame(geometry_msgs::PoseStamped pose_);

        geometry_msgs::PoseStamped getCurrentPose();
        geometry_msgs::PoseStamped inCommunicationRange(std::string robotFrameName);
        void getTeamPoses();
        void updateEnv();
        void updateEnvCurrentPose();
        void updateEnvTeamGoal();
        void updateEnvRobotTeamPose();
        void updateEnvFailed();
        bool getFrontierCandidates();


};
