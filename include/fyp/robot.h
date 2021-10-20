#include "ros/ros.h"
#include <fyp/functions.h>
#include <fyp/environment.h>
#include <move_base_msgs/MoveBaseGoal.h>


class Robot{

    public: 
        //Constructor 
        Robot(std::string robotName_, std::string robotMapFrame_, std::string globalFrame_,
        		std::string robotFrame_, std::string mergeMapTopic_, std::string mapTopic_, std::string frontierGoal_);

        move_base_msgs::MoveBaseGoal moveBaseGoal;
        geometry_msgs::PoseStamped goalPose;
        std::vector<geometry_msgs::PoseStamped> failedFrontiers;

        //Status indicators
        bool hasGoal;
        bool processingGoal; //represents that Robot is in motion
        bool goalReached;
        Environment robotEnvironment;

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
        Environment::Frontier frontierCandidate{0,0,0};


        //get goal position, get other robots position if within communication range, exchange map info
        bool updateGoal(geometry_msgs::PoseStamped goalPose_);
        void updateMBG();
        geometry_msgs::PoseStamped convertToRobotFrame(geometry_msgs::PoseStamped pose_);

        geometry_msgs::PoseStamped getCurrentPose();
        geometry_msgs::PoseStamped inCommunicationRange(std::string robotFrameName);
        void getTeamPoses();
        void updateEnvCurrentPose();
        void updateEnvTeamGoal();
        void updateEnvRobotTeamPose();
        void updateEnvFailed();
        bool getFrontierCandidates();


};
