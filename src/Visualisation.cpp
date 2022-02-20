#include <fyp_api/Visualisation.h>

namespace Vis_ns{
	Vis::Vis(std::string globalFrame):
		res(0.0f){
		gf = globalFrame;
	}

	Vis::~Vis(){
	}

	void Vis::updateParam(float res_, geometry_msgs::Point offSet_){
		res = res_;
		offSet = offSet_;
	}

	std::vector<float> Vis::coordToPoint(std::vector<int> coord){
		return {offSet.x+(coord[0]*res), offSet.y+(coord[1]*res)};
	}

	visualization_msgs::Marker Vis::visMarker(std::vector<std::vector<int>> coordCells,float r, float g, float b, float size){
		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = 0;
		marker.header.frame_id = gf;
		marker.header.stamp = ros::Time::now();
		//marker.type = shape;
		marker.type = visualization_msgs::Marker::POINTS;

		marker.action = visualization_msgs::Marker::ADD;
		marker.points.resize(coordCells.size());

		for (int i = 0; i < coordCells.size(); i++){
			std::vector<float> point = coordToPoint(coordCells[i]);
			marker.points[i].x = point[0];
			marker.points[i].y = point[1];
		}

		marker.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = size;
		marker.scale.y = size;
		marker.scale.z = size;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		return marker;

	}

	visualization_msgs::Marker Vis::visMarker(std::vector<geometry_msgs::PoseStamped> poses,float r, float g, float b, float size){
		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = 0;
		marker.header.frame_id = gf;
		marker.header.stamp = ros::Time::now();
		//marker.type = shape;
		marker.type = visualization_msgs::Marker::POINTS;

		marker.action = visualization_msgs::Marker::ADD;
		marker.points.resize(poses.size());

		for (int i = 0; i < poses.size(); i++){
			marker.points[i].x = poses[i].pose.position.x;
			marker.points[i].y = poses[i].pose.position.y;
		}

		marker.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = size;
		marker.scale.y = size;
		marker.scale.z = size;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		return marker;

	}

	visualization_msgs::Marker Vis::visMarker(std::vector<std::vector<float>> ptCells,float r, float g, float b, float size){
		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = 0;
		marker.header.frame_id = gf;
		marker.header.stamp = ros::Time::now();
		//marker.type = shape;
		marker.type = visualization_msgs::Marker::POINTS;

		marker.action = visualization_msgs::Marker::ADD;
		marker.points.resize(ptCells.size());

		for (int i = 0; i < ptCells.size(); i++){
			marker.points[i].x = ptCells[i][0];
			marker.points[i].y = ptCells[i][1];
		}

		marker.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = size;
		marker.scale.y = size;
		marker.scale.z = size;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		return marker;

	}


}
