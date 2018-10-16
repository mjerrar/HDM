/**
 * @file voxel.cpp
 * @author Jerrar Bukhari
 * @date 15 Oct 2018
 * @copyright 2018 Jerrar Bukhari
 * @brief <Implementation file for voxel grid filter based on PCL>
 */

#include <voxel.h>

VoxelGridFilter::VoxelGridFilter(float x, float y, float z)
    : voxel_filter() {
  this->voxel_filter.setLeafSize(x, y, z);
}

void VoxelGridFilter::set_leafsize(float x, float y, float z) {
  this->voxel_filter.setLeafSize(x, y, z);
}

void VoxelGridFilter::get_leafsize() {
  std::cout << "x:" << this->x << std::endl << "y:" << this->y << std::endl
      << "z:" << this->z << std::endl;
}

/**
 * @brief <compute the voxel filter on cloud data>
 * @param [in] <cloud> <cloud data to be filtered>
 * @return <filtered cloud data>
 */

bool VoxelGridFilter::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  this->voxel_filter.setInputCloud(cloud);
  this->voxel_filter.filter(*cloud);
  if (1) {
    return true;
  }
}
