/**
* @file cluster.h
* @author Jerrar Bukhari
* @date 15 Oct 2018
* @copyright 2018 Jerrar Bukhari
* @brief <Header file for clustering based on PCL>
*/

#ifndef CLUSTER_
#define CLUSTER_

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>


class ClusterExtractor{
	protected:
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	public:
		ClusterExtractor(float tolerance, int minClusterSize, int maxClusterSize);
	void initEuclideanClustractor(float tolerance, int minClusterSize, int maxClusterSize);
	bool extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif //CLUSTER_
