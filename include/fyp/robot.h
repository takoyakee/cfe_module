#include "ros/ros.h"
#include <fyp/functions.h>
#include <fyp/environment.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class Robot{
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    public: 
        //Constructor 
        Robot(std::string robotName_, std::string globalFrame_, std::string robotFrame_, int teamSize_);

        //info from main
        std::string robotName;
        std::string globalFrame;
        std::string robotFrame;
        std::string robotMapFrame;
        int teamSize;

        //to be compatible with move_base goal,
        //goalPose and currentPose both in global frame
        // Require a converter to send goal (in robot map frame)
        geometry_msgs::PoseStamped goalPose;
        geometry_msgs::PoseStamped currentPose;
        std::vector<geometry_msgs::PoseStamped> teamPoses;
        std::vector<geometry_msgs::PoseStamped> frontierCandidates;
        move_base_msgs::MoveBaseGoal moveBaseGoal;
        MoveBaseClient moveBaseClient{"robot_1",true};
        Environment robotEnvironment;

        //Status indicators
        bool hasGoal;
        bool processingGoal; //represents that Robot is in motion
        bool goalReached;
        int failedAttempts;
        int frontiersExplored;

        //hyper parameter
        float commRadius;



        //get goal position, get other robots position if within communication range, exchange map info
        bool updateGoal(geometry_msgs::PoseStamped goalPose_);
        void updateMBG();
        void mapCallBack(const nav_msgs::OccupancyGrid& msg);
        void teamPoseCallBack();
        geometry_msgs::PoseStamped getCurrentPose();
        std::vector<geometry_msgs::PoseStamped> getTeamPoses();
        void updateEnvCurrentPose();
        void updateEnvTeamPose();
        void getFrontierCandidates();
        geometry_msgs::PoseStamped getFrontier();

        //Most important functionality
        void explore();
        bool sendGoal();
        bool waitForResult();

    private:
        
        geometry_msgs::PoseStamped convertToRobotFrame(geometry_msgs::PoseStamped pose_);



};
