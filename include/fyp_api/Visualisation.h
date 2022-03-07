/*
 * Visualization.h
 *
 *  Created on: 19 Feb 2022
 *      Author: yanling
 */
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifndef FYP_API_INCLUDE_FYP_API_VISUALIZATION_H_
#define FYP_API_INCLUDE_FYP_API_VISUALIZATION_H_


namespace Vis_ns{
	class Vis{

	public:
		Vis(std::string globalFrame, ros::NodeHandle nh_, std::vector<float> color_);
		~Vis();
		void Initialise(ros::NodeHandle nh_);
		void resetVis();
		void visPub();
		ros::Publisher markersPub, pathPub, posesPub;

		visualization_msgs::MarkerArray markerArray;
		geometry_msgs::PoseArray poseArray;
		geometry_msgs::TransformStamped own2Global;
		std::vector<float> color;
		int queueSize;
		ros::NodeHandle nh;
		void updateParam(float res_, geometry_msgs::Point scanOrigin_,geometry_msgs::TransformStamped own2Global_);
		visualization_msgs::Marker visShape(std::vector<std::vector<int>> coordCells,float r, float g, float b, float size=0.4);
		visualization_msgs::Marker visShape(std::vector<geometry_msgs::PoseStamped> poses,float  r, float g, float b,float size=0.4);
		visualization_msgs::Marker visShape(std::vector<std::vector<float>> ptCells ,float  r, float g, float b,float size=0.4);
		visualization_msgs::Marker visShape(std::vector<geometry_msgs::Point> points,float  r, float g, float b,float size=0.4);

		visualization_msgs::Marker visLine(std::vector<std::vector<int>> coordCells,float  r, float g, float b, float size=0.4);
		visualization_msgs::Marker visLine(std::vector<geometry_msgs::PoseStamped> poses,float  r, float g, float b,float size=0.4);
		visualization_msgs::Marker visLine(std::vector<std::vector<float>> ptCells ,float  r, float g, float b,float size=0.4);

		void visPose(std::vector<geometry_msgs::Pose> poses);
		void visPose(geometry_msgs::PoseArray poses);



	private:
		geometry_msgs::Point scanOrigin;
		float res;
		std::string gf;
		std::vector<float> coordToPoint(std::vector<int> coord);


	};

}
#endif /* FYP_API_INCLUDE_FYP_API_VISUALIZATION_H_ */
