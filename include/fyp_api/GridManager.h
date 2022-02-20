/*
 * GridManager.h
 *
 *  Created on: 1 Dec 2021
 *      Author: yanling
 */

#include <fyp_api/functions.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>

#define MY_ASSERT(val)                                                                                                 \
  if (!(val))                                                                                                          \
  {                                                                                                                    \
    std::cout << "\033[31m Error at [File: " << __FILE__ << "][Line: " << __LINE__ << "]"                              \
              << "[Function: " << __FUNCTION__ << "] \033[0m\n"                                                        \
              << std::endl;                                                                                            \
    exit(1);                                                                                                           \
  }

namespace GridManager_ns{
	struct GridParameters{
		float leafSize;
		int totalCellNo, neighbourNum;
		float cellSize,cellHeight, resolutionX, resolutionY, resolutionZ, expandSize;
		std::string globalFrame;

		Eigen::Vector3d range, origin;
		Eigen::Vector3i gridSize;
		Eigen::Vector3d resolution;

		bool loadParameters(ros::NodeHandle& nh_private);

	};

	class GridManager{
	public:
		GridManager(ros::NodeHandle& nh_, ros::NodeHandle& nh_private);
		void Initialise(ros::NodeHandle& nh_, ros::NodeHandle& nh_private);
		void updateOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
		void getFrontierCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out);
		void checkExpansion();
		void merge(GridManager& merge_out);
		void resetGrid();


		void updatePoseOrigin(geometry_msgs::Pose robotPose);
		void updateRobotPose(geometry_msgs::Pose robotPose);
		void setOrigin(Eigen::Vector3d newOrigin);
		void setValue(int idx, char value);

		nav_msgs::OccupancyGrid grid();
		Eigen::Vector3d origin();
		Eigen::Vector3d resolution();
		int cellnumber();
		Eigen::Vector3i gridsize();
		char getValue(int idx);

		nav_msgs::OccupancyGrid visualizeGrid(int addlayer);

	private:
		 enum CellState : char
		  {
		    UNKNOWN = -1,
		    OCCUPIED = 127,
		    FREE = 0,
		    NOT_FRONTIER = 3
		  };

		GridParameters GP;
		ros::Publisher frontierpub;
		nav_msgs::OccupancyGrid voxelGrid;
		Eigen::Vector3d currPose, lastPose;
		Eigen::Vector3i currSub, lastSub;
		bool isInitialised;

		void expandGrid();
		void translateGrid(GridParameters oldParam, GridParameters newParam,
				nav_msgs::OccupancyGrid oldGrid, nav_msgs::OccupancyGrid& newGrid);
		int compute2DNeighbour(Eigen::Vector3i sub, char desiredState);
		void InitialiseRayTrace(pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ>& rayTracer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
		void raytraceHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                std::vector<Eigen::Vector3i>& cells);

		void initialiseOG(nav_msgs::OccupancyGrid& og);
		void initialiseOG(nav_msgs::OccupancyGrid& og, GridParameters gp);
		void fillOGInfo(nav_msgs::OccupancyGrid& og);
		void fillOGInfo(nav_msgs::OccupancyGrid& og, GridParameters gp);

		Eigen::Vector3i Pos2Sub(Eigen::Vector3d pos);
		Eigen::Vector3i Idx2Sub(int idx);
		Eigen::Vector3i Idx2Sub(int idx, Eigen::Vector3i gridSize);
		Eigen::Vector3d Sub2Pos(Eigen::Vector3i sub);
		int Sub2Idx(Eigen::Vector3i sub);
		bool inRange(Eigen::Vector3i sub);
		bool inRange(Eigen::Vector3i sub,Eigen::Vector3i minsub, Eigen::Vector3i maxsub);
		void setRange(Eigen::Vector3d newRange);
		void setGridSize(Eigen::Vector3i newSize);
		int signum(int x);
		double intbound(double s, double ds);
		double mod(double value, double modulus);

	};

}

