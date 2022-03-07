/*
 * GridManager.cpp
 *
 *  Created on: 1 Dec 2021
 *      Author: yanling
 */

#include <fyp_api/GridManager.h>

namespace GridManager_ns{
	bool GridParameters::loadParameters(ros::NodeHandle& nh_private){
		std::string param_ns = "/frontier_common/";
		nh_private.getParam(param_ns+"leaf_size", leafSize);
		nh_private.getParam(param_ns+"cell_size", cellSize);
		nh_private.getParam(param_ns+"cell_height", cellHeight);
		nh_private.getParam(param_ns+"neighbour_num", neighbourNum);
		nh_private.getParam(param_ns+"resolution_x", resolutionX);
		nh_private.getParam(param_ns+"resolution_y", resolutionY);
		nh_private.getParam(param_ns+"resolution_z", resolutionZ);
		nh_private.getParam(param_ns+"expand_size", expandSize);
		nh_private.getParam(param_ns+"waypoint_frame", globalFrame);

		range = Eigen::Vector3d(cellSize * neighbourNum,cellSize * neighbourNum, cellHeight*neighbourNum);
		//rolloverRange = Eigen::Vector3d(cellSize,cellSize, cellHeight);
		resolution = Eigen::Vector3d(resolutionX, resolutionX, resolutionY);
		for (int i = 0; i < 3; i++)

		{
			gridSize(i) = static_cast<int>(range(i) / resolution(i));
			//rolloverSize(i) = static_cast<int>(rolloverRange(i) / resolution(i));
			origin(i) = -range(i) / 2;
		}
		totalCellNo = gridSize.x() * gridSize.y() * gridSize.z();

		return true;
	}

	GridManager::GridManager(ros::NodeHandle& nh_, ros::NodeHandle& nh_private){
		Initialise(nh_,nh_private);
	}


	void GridManager::Initialise(ros::NodeHandle& nh_, ros::NodeHandle& nh_private){
		if (!GP.loadParameters(nh_private)){
			return;
		}

		initialiseOG(voxelGrid);

		frontierpub = nh_.advertise<sensor_msgs::PointCloud2>("frontier_cloud",5);

	}

	void GridManager::updateOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
		pcl::PointCloud<pcl::PointXYZ>::Ptr occluded_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		//pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ> rayTracer;
		//InitialiseRayTrace(rayTracer,cloud);

		int occludedCount = 0;
		int freeCount = 0;
		int oobCount = 0;

		for (auto & pt: cloud->points){
			Eigen::Vector3i vgSub = Pos2Sub(Eigen::Vector3d(pt.x,pt.y,pt.z));
			if (inRange(vgSub)){
				int vgIdx = Sub2Idx(vgSub);
				voxelGrid.data[vgIdx] = OCCUPIED;
			} else {
				oobCount++;
			}
		}
		//ROS_INFO("iterated through all points");
		if (!inRange(currSub))return;
		for (auto& pt : cloud->points){
			Eigen::Vector3i end_sub = Pos2Sub(Eigen::Vector3d(pt.x,pt.y,pt.z));
			if (!inRange(end_sub)){
				continue;
			}
			std::vector<Eigen::Vector3i> raycastCells;
			Eigen::Vector3i start_sub = currSub;
			/*ROS_INFO("start (%d,%d,%d) to end (%d,%d,%d)", start_sub.x(),start_sub.y(),start_sub.z(),
					end_sub.x(),end_sub.y(),end_sub.z());*/
			raytraceHelper(start_sub, end_sub, raycastCells);
			for (auto sub : raycastCells){
				int idx = Sub2Idx(sub);
				if (getValue(idx) == OCCUPIED){
					continue;
				} else {
					voxelGrid.data[idx] = FREE;
					Eigen::Vector3d pos = Sub2Pos(sub);
					pcl::PointXYZ point;
					point.x = pos.x();
					point.y = pos.y();
					point.z = pos.z();
					free_cloud->points.push_back(point);
				}

			}

		}


		   /*std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > outRay;
		   Eigen::Vector3i gridCoord = rayTracer.getGridCoordinates(pt.x, pt.y, pt.z);

		   int gridState;
		   int ret=rayTracer.occlusionEstimation(gridState, outRay, gridCoord);
		   if (outRay.size() > 1){
			   outRay.pop_back();
			   //ROS_INFO("size of outRay: %d", outRay.size());
		   }

		   for (auto &it : outRay){
			   Eigen::Vector3f pos3f = rayTracer.getCentroidCoordinate(it).head<3>();
			   Eigen::Vector3d pos = pos3f.cast<double>();
			   Eigen::Vector3i sub = Pos2Sub(pos);
			   if (!inRange(sub)){
				  continue;
			   }
			   int idx = Sub2Idx(sub);
			   //ROS_INFO("(%d,%d,%d) ---> idx: %d", it.x(), it.y(), it.z(),idx);
			   voxelGrid.data[idx] = FREE;
		   }*/




	}


	//TODO: spped up process of getting neighbours as this is too slow!
	void GridManager::getFrontierCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out){
		//ROS_INFO("Getting Frontier Cloud");
		Eigen::Vector3i sub_max = Pos2Sub(GP.origin + GP.range);
		Eigen::Vector3i sub_min = Pos2Sub(GP.origin - GP.range);
		Eigen::Vector3i origin_sub = Pos2Sub(GP.origin);

		//ROS_INFO("origin: %d,%d,%d", origin_sub.x(),origin_sub.y(),origin_sub.z());
		if (!inRange(origin_sub))
		{
			ROS_WARN("RollingOccupancyGrid::GetFrontierInRange(), robot not in range");
			return;
		}
		int ray_trace_count = 0;

		int cell_num = GP.totalCellNo;
		for (int ind = 0; ind < cell_num; ind++)
		{
			Eigen::Vector3i cur_sub = Idx2Sub(ind);
			if (!inRange(cur_sub, sub_min, sub_max))
			{
				continue;
			}
			if (voxelGrid.data[ind] == UNKNOWN)
			{
				bool z_free = false;
				bool xy_free = false;
				// If the unknown cell has neighboring free cells in xy but not z direction
				cur_sub(2)--;
				int array_ind;
				if (inRange(cur_sub))
				{
					array_ind = Sub2Idx(cur_sub);
					if (voxelGrid.data[array_ind] == FREE )
					{
						z_free = true;
						continue;
					}
				}
				cur_sub(2) += 2;
				if (inRange(cur_sub))
				{
					array_ind = Sub2Idx(cur_sub);
					if (voxelGrid.data[array_ind] == FREE )
					{
						z_free = true;
						continue;
					}
				}
				cur_sub(2)--;

				for (int i = 0; i < 2; i++)
			  {
				cur_sub(i)--;
				if (inRange(cur_sub))
				{
				  array_ind = Sub2Idx(cur_sub);
				  if (getValue(array_ind) == FREE)
				  {
					xy_free = true;
					cur_sub(i)++;
					break;
				  }
				}
				cur_sub(i) += 2;
				if (inRange(cur_sub))
				{
					array_ind = Sub2Idx(cur_sub);
				  if (getValue(array_ind) == FREE)
				  {
					xy_free = true;
					cur_sub(i)--;
					break;
				  }
				}
				cur_sub(i)--;
			  }

/*				int xy_count = compute2DNeighbour(cur_sub, FREE);
				if (xy_count < 4){
					continue;
				}*/

				if (xy_free && !z_free){
					//ROS_INFO("xy_count: %d", xy_count);
					Eigen::Vector3d pos = Sub2Pos(cur_sub);
					pcl::PointXYZI point;
					point.x = pos.x();
					point.y = pos.y();
					point.z = pos.z();
					point.intensity = 0;
					cloud_out->points.push_back(point);
				}

			}
		}
		//ROS_INFO("Obtained frontier clouds with size: %d", cloud_out->points.size());
	}

	//TODO: create a function, translate grids -> can be used for merge, expand, update of position,
	void GridManager::merge(GridManager& merge_out){
		nav_msgs::OccupancyGrid newGrid = merge_out.voxelGrid;
		translateGrid(GP, merge_out.GP, voxelGrid, newGrid);
		merge_out.voxelGrid = newGrid;

	}

	void GridManager::checkExpansion(){
		Eigen::Vector3i currSub = Pos2Sub(currPose);
		//ROS_INFO("currsub: %d,%d,%d", currSub.x(),currSub.y(),currSub.z());
		//ROS_INFO("position percentage: %f,%f", (float)(currSub.x())/GP.gridSize.x(),(float)(currSub.y())/GP.gridSize.y());
		if (abs(1- (float)(currSub.x())/GP.gridSize.x()) <= 0.2 || abs(1- (float)(currSub.y())/GP.gridSize.y()) <= 0.2){
			ROS_INFO("gridsize: %d,%d,%d", GP.gridSize.x(), GP.gridSize.y(),GP.gridSize.z());
			expandGrid();
			ROS_INFO("new gridsize: %d,%d,%d", GP.gridSize.x(), GP.gridSize.y(),GP.gridSize.z());		}
	}


	//TODO: Translate Grid or reinitialise grid
	void GridManager::updatePoseOrigin(geometry_msgs::Pose robotPose){
		Eigen::Vector3d newPose(robotPose.position.x,robotPose.position.y,robotPose.position.z);
		if (newPose == currPose){
			return;
		} else {
			currPose = newPose;
			currSub = Pos2Sub(currPose);
		}

		GridParameters newParam = GP;
		newParam.origin = currPose - GP.range/2;
		nav_msgs::OccupancyGrid newGrid;
		initialiseOG(newGrid, newParam);
		translateGrid(GP, newParam, voxelGrid, newGrid);
		GP = newParam;
		voxelGrid = newGrid;

	}

	void GridManager::updateRobotPose(geometry_msgs::Pose robotPose){
		currPose.x() = robotPose.position.x;
		currPose.y() = robotPose.position.y;
		currPose.z() = robotPose.position.z;

	}

	void GridManager::expandGrid(){
		Eigen::Vector3d newRange(GP.range.x()*GP.expandSize,GP.range.y()*GP.expandSize,GP.range.z());
		Eigen::Vector3d newOrigin(0,0,0);
		Eigen::Vector3i newSize;
		for (int i = 0; i < 3; i++){
			newSize(i)= newRange(i)/GP.resolution(i);
			newOrigin(i) = -newRange(i)/2;
		}

		int newTotalCells = newSize.x()*newSize.y()*newSize.z();

		GridParameters newParam = GP;
		newParam.range = newRange;
		newParam.origin = newOrigin;
		newParam.gridSize = newSize;
		newParam.totalCellNo = newTotalCells;

		nav_msgs::OccupancyGrid newGrid;
		for (int i = 0; i < newTotalCells; i++){
			newGrid.data.push_back(UNKNOWN);
		}

		translateGrid(GP, newParam, voxelGrid, newGrid);

		GP = newParam;
		voxelGrid = newGrid;

	}

	//Translate contents of oldGrid into newGrid according to the newParameters
	void GridManager::translateGrid(GridParameters oldParam, GridParameters newParam,
			nav_msgs::OccupancyGrid oldGrid, nav_msgs::OccupancyGrid& newGrid){

		Eigen::Vector3d oldOrigin = oldParam.origin;
		int totalCell = oldParam.totalCellNo;
		Eigen::Vector3i oldSize = oldParam.gridSize;

		Eigen::Vector3d newOrigin = newParam.origin;
		Eigen::Vector3i newSize = newParam.gridSize;

		if (oldParam.resolution != newParam.resolution){
			return ;
		}
		Eigen::Vector3d resolution = newParam.resolution;

		Eigen::Vector3i offset (0,0,0);
		for (int i = 0; i < 3; i++){
			offset(i) = static_cast<int>((oldOrigin(i) - newOrigin(i))/resolution(i));
		}

		Eigen::Vector3i minSub(0,0,0);
		Eigen::Vector3i maxSub = newSize - Eigen::Vector3i(1,1,1);

		for (int idx = 0; idx < totalCell; idx++){
			Eigen::Vector3i oldSub = Idx2Sub(idx, oldSize);
			Eigen::Vector3i newSub = oldSub + offset;
			if (inRange(newSub, minSub, maxSub)){
				int newIdx = newSub.x() + newSub.y()*newSize.x() + newSub.z()*newSize.x()*newSize.y();
				newGrid.data[newIdx] = oldGrid.data[idx];
			}
		}

		fillOGInfo(newGrid, newParam);
	}

	int GridManager::compute2DNeighbour(Eigen::Vector3i sub, char desiredState){
		Eigen::Vector3i botleft(sub.x()-1,sub.y()-1,sub.z()), topright(sub.x()+1,sub.y()+1,sub.z());
		if (!inRange(botleft) || !inRange(topright)){
			return -1;
		}
		int sub_idx = Sub2Idx(sub);
		int TL = sub_idx+GP.gridSize(0)-1, T = sub_idx+GP.gridSize(0), TR = sub_idx+GP.gridSize(0)+1,
				L = sub_idx-1, R = sub_idx+1, BL = sub_idx-GP.gridSize(0)-1, B = sub_idx-GP.gridSize(0), BR = sub_idx-GP.gridSize(0)+1;

		int freeCount = 0;
	/*	freeCount += (voxelGrid.data[TL]==desiredState);
		freeCount += (voxelGrid.data[T]==desiredState);
		freeCount += (voxelGrid.data[TR]==desiredState);
		freeCount += (voxelGrid.data[L]==desiredState);
		freeCount += (voxelGrid.data[R]==desiredState);
		freeCount += (voxelGrid.data[BL]==desiredState);
		freeCount += (voxelGrid.data[B]==desiredState);
		freeCount += (voxelGrid.data[BR]==desiredState);*/

		freeCount += (abs(voxelGrid.data[TL]- voxelGrid.data[TR])==1);
		freeCount += 2*(abs(voxelGrid.data[L]- voxelGrid.data[R])==1);
		freeCount += (abs(voxelGrid.data[BL]- voxelGrid.data[BR])==1);
		freeCount += (abs(voxelGrid.data[TL]- voxelGrid.data[BL])==1);
		freeCount += 2*(abs(voxelGrid.data[T]- voxelGrid.data[B])==1);
		freeCount += (abs(voxelGrid.data[TR]- voxelGrid.data[BR])==1);

		return freeCount;

	}

	void GridManager::InitialiseRayTrace(pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ>& rayTracer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
		if (cloud->points.empty()){
			return;
		}
		rayTracer.setInputCloud(cloud);
		rayTracer.setLeafSize(GP.leafSize,GP.leafSize,GP.leafSize);
		rayTracer.initializeVoxelGrid();
	}

	void GridManager::raytraceHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
            std::vector<Eigen::Vector3i>& cells){
		  cells.clear();
		  MY_ASSERT(inRange(start_sub));
		  MY_ASSERT(inRange(end_sub));
		  if (start_sub == end_sub)
		  {
		    cells.push_back(start_sub);
		    return;
		  }
		  Eigen::Vector3i diff_sub = end_sub - start_sub;
		  double max_dist = diff_sub.squaredNorm();
		  int step_x = signum(diff_sub.x());
		  int step_y = signum(diff_sub.y());
		  int step_z = signum(diff_sub.z());
		  double t_max_x = step_x == 0 ? DBL_MAX : intbound(start_sub.x(), diff_sub.x());
		  double t_max_y = step_y == 0 ? DBL_MAX : intbound(start_sub.y(), diff_sub.y());
		  double t_max_z = step_z == 0 ? DBL_MAX : intbound(start_sub.z(), diff_sub.z());
		  double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
		  double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
		  double t_delta_z = step_z == 0 ? DBL_MAX : (double)step_z / (double)diff_sub.z();
		  double dist = 0;
		  Eigen::Vector3i cur_sub = start_sub;

		  while (inRange(cur_sub))
		  {
		    cells.push_back(cur_sub);
		    dist = (cur_sub - start_sub).squaredNorm();
		    int array_ind = Sub2Idx(cur_sub);
		    if (cur_sub == end_sub || dist > max_dist || getValue(array_ind) == OCCUPIED)
		    {
		      return;
		    }
		    if (t_max_x < t_max_y)
		    {
		      if (t_max_x < t_max_z)
		      {
		        cur_sub.x() += step_x;
		        t_max_x += t_delta_x;
		      }
		      else
		      {
		        cur_sub.z() += step_z;
		        t_max_z += t_delta_z;
		      }
		    }
		    else
		    {
		      if (t_max_y < t_max_z)
		      {
		        cur_sub.y() += step_y;
		        t_max_y += t_delta_y;
		      }
		      else
		      {
		        cur_sub.z() += step_z;
		        t_max_z += t_delta_z;
		      }
		    }
		  }
	}

	void GridManager::resetGrid(){
		initialiseOG(voxelGrid);
	}


	char GridManager::getValue(int idx){
		return voxelGrid.data[idx];
	}

	void GridManager::setValue(int idx, char value){
		if (value < -126 || value > 127){
			return;
		}
		voxelGrid.data[idx] = value;
	}

	void GridManager::setOrigin(Eigen::Vector3d newOrigin){
		GP.origin = newOrigin;

	}

	void GridManager::setRange(Eigen::Vector3d newRange){
		GP.range = newRange;
	}

	void GridManager::setGridSize(Eigen::Vector3i newSize){
		GP.gridSize = newSize;
		GP.totalCellNo = newSize.x()*newSize.y()*newSize.z();
	}

	void GridManager::initialiseOG(nav_msgs::OccupancyGrid& og){
		fillOGInfo(og);
		og.data.clear();

		for (int i = 0; i < GP.totalCellNo; i++){
			og.data.push_back(UNKNOWN);
		}
	}

	void GridManager::initialiseOG(nav_msgs::OccupancyGrid& og, GridParameters gp){
		fillOGInfo(og, gp);
		og.data.clear();

		for (int i = 0; i < gp.totalCellNo; i++){
			og.data.push_back(UNKNOWN);
		}
	}


	nav_msgs::OccupancyGrid GridManager::visualizeGrid(int addlayer){
		//Single layer visualization
		nav_msgs::OccupancyGrid outputmap;
		fillOGInfo(outputmap);
		outputmap.header.stamp = ros::Time::now();

		int layer = (addlayer+GP.range.z()/2)/GP.resolution.z();


		for (int i = 0; i < GP.gridSize.x()*GP.gridSize.y(); i++){
			outputmap.data.push_back(UNKNOWN);
		}

		for (int i = 0; i < GP.gridSize.x(); i++){
			for (int j = 0; j < GP.gridSize.y(); j++){
				Eigen::Vector3i sub(i,j,layer);
				int vxidx = Sub2Idx(sub);
				outputmap.data[i+j*GP.gridSize.x()] = voxelGrid.data[vxidx];
			}
		}

		return outputmap;
	}


	Eigen::Vector3i GridManager::Pos2Sub(Eigen::Vector3d pos){
		Eigen::Vector3i sub(0, 0, 0);
		for (int i = 0; i < 3; i++)
		{
			sub(i) =  static_cast<int>((pos(i) - GP.origin(i)) / GP.resolution(i));
		}
		return sub;
	}

	Eigen::Vector3i GridManager::Idx2Sub(int idx){
		Eigen::Vector3i sub;
		sub.z() = floor((float)(idx)/(GP.gridSize.x()*GP.gridSize.y()));
		int remainder = idx%((int)(GP.gridSize.x()*GP.gridSize.y()));
		sub.y() = floor((float)(remainder)/GP.gridSize.x());
		sub.x() = remainder%GP.gridSize.x();
		return sub;
	}

	Eigen::Vector3i GridManager::Idx2Sub(int idx, Eigen::Vector3i gridSize){
		Eigen::Vector3i sub;
		sub.z() = floor((float)(idx)/(gridSize.x()*gridSize.y()));
		int remainder = idx%((int)(gridSize.x()*gridSize.y()));
		sub.y() = floor((float)(remainder)/gridSize.x());
		sub.x() = remainder%gridSize.x();
		return sub;
	}


	Eigen::Vector3d GridManager::Sub2Pos(Eigen::Vector3i sub){
		Eigen::Vector3d pos(0.f, 0.f, 0.f);
		for (int i = 0; i < 3; i++)
		{
		  pos(i) = sub(i) >= 0 ? (sub(i) * GP.resolution(i)) + GP.origin(i) : GP.range(i)+1;
		}
		return pos;
	}

	int GridManager::Sub2Idx(Eigen::Vector3i sub){
		return	sub.x() + (sub.y() * GP.gridSize.x()) + (sub.z() * GP.gridSize.x() * GP.gridSize.y());
	}

	bool GridManager::inRange(Eigen::Vector3i sub){
		bool in_range = true;
		for (int i = 0; i < 3; i++)
		{
		  in_range &= sub(i) >= 0 && sub(i) < GP.gridSize(i);
		}
		return in_range;

	}

	bool GridManager::inRange(Eigen::Vector3i sub, Eigen::Vector3i minsub, Eigen::Vector3i maxsub){
		bool in_range = true;
		for (int i = 0; i < 3; i++)
		{
		  in_range &= sub(i) >=  minsub(i) && sub(i) <= maxsub(i);
		}
		return in_range;

	}

	void GridManager::fillOGInfo(nav_msgs::OccupancyGrid& og){
		og.info.origin.position.x = GP.origin.x();
		og.info.origin.position.y = GP.origin.y();
		og.info.origin.position.z = GP.origin.z();
		og.info.width = GP.gridSize.x();
		og.info.height = GP.gridSize.y();
		og.info.resolution = GP.resolution.x();
		og.header.frame_id = GP.globalFrame;
		og.header.stamp = ros::Time::now();
	}

	void GridManager::fillOGInfo(nav_msgs::OccupancyGrid& og, GridParameters gp){
		og.info.origin.position.x = gp.origin.x();
		og.info.origin.position.y = gp.origin.y();
		og.info.origin.position.z = gp.origin.z();
		og.info.width = gp.gridSize.x();
		og.info.height = gp.gridSize.y();
		og.info.resolution = gp.resolution.x();
		og.header.frame_id = gp.globalFrame;
		og.header.stamp = ros::Time::now();
	}

	nav_msgs::OccupancyGrid GridManager::grid(){
		return voxelGrid;
	}

	Eigen::Vector3d GridManager::origin(){
		return GP.origin;
	}

	Eigen::Vector3d GridManager::resolution(){
		return GP.resolution;
	}

	int GridManager::cellnumber(){
		return GP.totalCellNo;
	}

	Eigen::Vector3i GridManager::gridsize(){
		return GP.gridSize;
	}

	int GridManager::signum(int x)
	{
	  return x == 0 ? 0 : x < 0 ? -1 : 1;
	}

	double GridManager::intbound(double s, double ds)
	{
	  // Find the smallest positive t such that s+t*ds is an integer.
	  if (ds < 0)
	  {
	    return intbound(-s, -ds);
	  }
	  else
	  {
	    s = mod(s, 1);
	    // problem is now s+t*ds = 1
	    return (1 - s) / ds;
	  }
	}

	double GridManager::mod(double value, double modulus){
		return fmod(fmod(value, modulus) + modulus, modulus);
	}

}


