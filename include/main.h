// Copyright [2018] [Jerrar Bukhari]
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file lib.h
 * @author Jerrar Bukhari
 * @date 15 Oct 2018
 * @copyright 2018 Jerrar Bukhari
 * @brief <Header file for main.cpp>
 */

#ifndef INCLUDE_MAIN_H_
#define INCLUDE_MAIN_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <voxel.h>
#include <passthrough.h>
#include <planeExtraction.h>
#include <cluster.h>

#include <iostream>
#include <string>

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void visualize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
int load_pcd(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

#endif  // INCLUDE_MAIN_H_
