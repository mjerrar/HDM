#include <voxel.h>

VoxelGridFilter::VoxelGridFilter(float x, float y, float z):voxel_filter(){
	this->voxel_filter.setLeafSize(x, y, z);
}

void VoxelGridFilter::set_leafsize(float x, float y, float z){
	this->voxel_filter.setLeafSize(x, y, z);
}

void VoxelGridFilter::get_leafsize(){
	std::cout << "x:" << this->x << std::endl <<
	"y:" << this->y << std::endl <<
	"z:" << this->z << std::endl;
}

bool VoxelGridFilter::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	this->voxel_filter.filter(*cloud);
	if (1){
	return true;
	}
}
