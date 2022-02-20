/*
 * Visualization.h
 *
 *  Created on: 19 Feb 2022
 *      Author: yanling
 */
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#ifndef FYP_API_INCLUDE_FYP_API_VISUALIZATION_H_
#define FYP_API_INCLUDE_FYP_API_VISUALIZATION_H_

namespace Vis_ns{
	class Vis{

	public:
		Vis(std::string globalFrame);
		~Vis();
		void updateParam(float res_, geometry_msgs::Point offSet_);
		visualization_msgs::Marker visMarker(std::vector<std::vector<int>> coordCells,float r, float g, float b, float size=0.4);
		visualization_msgs::Marker visMarker(std::vector<geometry_msgs::PoseStamped> poses,float r, float g, float b,float size=0.4);
		visualization_msgs::Marker visMarker(std::vector<std::vector<float>> ptCells ,float r, float g, float b,float size=0.4);





	private:
		geometry_msgs::Point offSet;
		float res;
		std::string gf;
		std::vector<float> coordToPoint(std::vector<int> coord);
	};

}
#endif /* FYP_API_INCLUDE_FYP_API_VISUALIZATION_H_ */
