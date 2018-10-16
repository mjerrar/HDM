//Copyright [2018] [Jerrar Bukhari]
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.

#include <iostream>

#include <lib.h>

int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  std::string filename = "/home/vendetta/Documents/3dLidar_TestData/vel_2m2l1230400000.pcd";

  load_pcd(filename, cloud);

  VoxelGridFilter voxfilter(0.1,0.1,0.1);
  voxfilter.filter(cloud);

  PassThroughFilter passfilter(-5.0,5.0,-5.0,5.0,-2.0,1.0);
  passfilter.filter(cloud);

  PlaneExtraction planeExtractor(100,0.02);
  planeExtractor.extractGroundPlane(cloud);

  ClusterExtractor clusteror(0.1,10,25000);
  clusteror.extract(cloud);

  visualize(cloud);

  return(0);

}
