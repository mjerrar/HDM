/**
* @file voxel.h
* @author Jerrar Bukhari
* @date 16 Oct 2018
* @copyright 2018 Jerrar Bukhari
* @brief <Header file for voxel grid filter based on PCL>
*/

#ifndef VOXEL_
#define VOXEL_

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


class VoxelGridFilter{
	protected:
	float x,y,z;
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
	public:
	VoxelGridFilter(float x, float y, float z);
	void set_leafsize(float x, float y, float z);
	void get_leafsize();
	bool filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif //VOXEL_
