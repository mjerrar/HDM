// Copyright [2018] [Jerrar Bukhari]
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file main.cpp
 * @author Jerrar Bukhari
 * @date 15 Oct 2018
 * @brief <3D LIdar Based Human Detection and localization module>
 */

#include <main.h>

#include <pcl/common/centroid.h>

#include <iostream>
#include <vector>



int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  std::string filename =
      "./test_data/vel_3m970398000.pcd";

  load_pcd(filename, cloud);

  visualize(cloud);

  VoxelGridFilter voxfilter(0.1, 0.1, 0.1);
  voxfilter.filter(cloud);

  visualize(cloud);

  PassThroughFilter passfilter(-5.0, 5.0, -5.0, 5.0, -2.0, 1.0);
  passfilter.filter(cloud);

  visualize(cloud);

  PlaneExtraction planeExtractor(100, 0.02);
  planeExtractor.extractGroundPlane(cloud);

  ClusterExtractor clusteror(0.1, 10, 25000);
  clusteror.extract(cloud);

  visualize(cloud);

  Eigen::Vector4f human_pos;
  pcl::compute3DCentroid(*cloud, human_pos);

  std::cout << "Coordinates of Human are" << std::endl;
  std::cout << "x : " << human_pos[0] << std::endl;
  std::cout << "y : " << human_pos[1] << std::endl;
  std::cout << "z : " << human_pos[2] << std::endl;

  return (0);
}

/**
 * @brief <visualize point cloud data>
 * @param [in] <cloud> <point cloud data>
 */
void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {
  }
}

/**
 * @brief <visualize point cloud data>
 * @param [in] <cloud> <point cloud data>
 */
void visualize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {
  }
}

/**
 * @brief <load point cloud data>
 * @param [in] <cloud> <point cloud data holder>
 * @param [in] <filename> <path to cloud data>
 */
int load_pcd(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  if (pcl::io::loadPCDFile < pcl::PointXYZ > (filename, *cloud) == -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << std::endl << cloud->width << std::endl
      << cloud->height << std::endl;
  return (1);
}
