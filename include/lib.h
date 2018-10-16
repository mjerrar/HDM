#ifndef LIB_
#define LIB_

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <voxel.h>
#include <passthrough.h>


void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
 {
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }
 }

void visualize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
 {
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }
 }

int load_pcd(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
 {
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
    std::cout << "Loaded " << std::endl
              << cloud->width << std::endl
              << cloud->height << std::endl;
  return (1);
 }



#endif //LIB_
