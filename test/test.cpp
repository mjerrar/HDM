#include <gtest/gtest.h>
#include <../include/main.h>
//#include <voxel.h>
#include <voxel.cpp>

TEST(negative, should_pass) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string false_file="";
  VoxelGridFilter voxfilter(0.1,0.1,0.1);
  EXPECT_EQ(voxfilter.filter(cloud), 1);
  EXPECT_EQ(1, 1);
}
