#include <fyp_api/Visualisation.h>

namespace Vis_ns{
	Vis::Vis(std::string globalFrame, ros::NodeHandle nh_, std::vector<float> color_):
		res(0.0f), queueSize(0){
		gf = globalFrame;
		poseArray.header.frame_id = gf;
		color = color_;
		Initialise(nh_);
	}

	Vis::~Vis(){
		markerArray.markers.clear();
		poseArray.poses.clear();
	}

	void Vis::Initialise(ros::NodeHandle nh_){
		markersPub = nh_.advertise<visualization_msgs::MarkerArray>("markers", 50, true);
		pathPub = nh_.advertise<nav_msgs::Path>("pos_paths",50,true);
		posesPub = nh_.advertise<geometry_msgs::PoseArray>("poses",50,true);

	}

	void Vis::resetVis(){
		markerArray.markers.clear();
		poseArray.poses.clear();
		queueSize = 0;
	}

	void Vis::visPub(){
		if (!markerArray.markers.empty()) markersPub.publish(markerArray);
		if (!poseArray.poses.empty()) posesPub.publish(poseArray);
	}

	void Vis::updateParam(float res_, geometry_msgs::Point scanOrigin_,geometry_msgs::TransformStamped own2Global_){
		res = res_;
		scanOrigin = scanOrigin_;
		own2Global = own2Global_;
	}

	std::vector<float> Vis::coordToPoint(std::vector<int> coord){
		float originx_ = scanOrigin.x;
		float originy_ = scanOrigin.y;
		geometry_msgs::Pose own, global;
		own.position.x = originx_+(coord[0]*res);
		own.position.y = originy_+(coord[1]*res);
		own.position.z = 0;
		own.orientation.w = own2Global.transform.rotation.w;
		own.orientation.x = own2Global.transform.rotation.x;
		own.orientation.y = own2Global.transform.rotation.y;
		own.orientation.z = own2Global.transform.rotation.z;
		tf2::doTransform(own, global, own2Global);
		return {global.position.x, global.position.y};
	}

	visualization_msgs::Marker Vis::visShape(std::vector<std::vector<int>> coordCells,float r, float g, float b, float size){
		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = queueSize;
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
		queueSize++;
		markerArray.markers.push_back(marker);

		return marker;

	}

	visualization_msgs::Marker Vis::visShape(std::vector<geometry_msgs::PoseStamped> poses,float r, float g, float b, float size){
		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = queueSize;
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
		queueSize++;
		markerArray.markers.push_back(marker);

		return marker;

	}

	visualization_msgs::Marker Vis::visShape(std::vector<std::vector<float>> ptCells,float r, float g, float b, float size){

		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = queueSize;
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

		queueSize++;
		markerArray.markers.push_back(marker);


		return marker;

	}

	visualization_msgs::Marker Vis::visShape(std::vector<geometry_msgs::Point> points,float r, float g, float b, float size){
		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = queueSize;
		marker.header.frame_id = gf;
		marker.header.stamp = ros::Time::now();
		//marker.type = shape;
		marker.type = visualization_msgs::Marker::POINTS;

		marker.action = visualization_msgs::Marker::ADD;
		marker.points.resize(points.size());

		for (int i = 0; i < points.size(); i++){
			marker.points[i].x = points[i].x;
			marker.points[i].y = points[i].y;
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
		queueSize++;
		markerArray.markers.push_back(marker);

		return marker;

	}

	visualization_msgs::Marker Vis::visLine(std::vector<std::vector<int>> coordCells,float r, float g, float b, float size){
		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = queueSize;
		marker.header.frame_id = gf;
		marker.header.stamp = ros::Time::now();
		//marker.type = shape;
		marker.type = visualization_msgs::Marker::LINE_STRIP;

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
		queueSize++;
		markerArray.markers.push_back(marker);


		return marker;

	}

	visualization_msgs::Marker Vis::visLine(std::vector<geometry_msgs::PoseStamped> poses,float r, float g, float b, float size){
		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = queueSize;
		marker.header.frame_id = gf;
		marker.header.stamp = ros::Time::now();
		//marker.type = shape;
		marker.type = visualization_msgs::Marker::LINE_STRIP;

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
		queueSize++;
		markerArray.markers.push_back(marker);

		return marker;

	}

	visualization_msgs::Marker Vis::visLine(std::vector<std::vector<float>> ptCells,float r, float g, float b, float size){

		visualization_msgs::Marker marker;

		marker.ns = "basic_shapes";
		marker.id = queueSize;
		marker.header.frame_id = gf;
		marker.header.stamp = ros::Time::now();
		//marker.type = shape;
		marker.type = visualization_msgs::Marker::LINE_STRIP;

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

		markerArray.markers.push_back(marker);
		queueSize++;
		markerArray.markers.push_back(marker);

		return marker;
	}

	void Vis::visPose(std::vector<geometry_msgs::Pose> poses){
		for (auto p: poses){
			poseArray.poses.push_back(p);
		}

	}

	void Vis::visPose(geometry_msgs::PoseArray poses){
		for (auto p: poses.poses){
			poseArray.poses.push_back(p);
		}

	}



}

