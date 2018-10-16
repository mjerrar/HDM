/**
* @file cluster.cpp
* @author Jerrar Bukhari
* @date 15 Oct 2018
* @copyright 2018 Jerrar Bukhari
* @brief <Implementation file for clustering based on PCL>
*/
#include <cluster.h>

ClusterExtractor::ClusterExtractor(float tolerance, int minClusterSize, int maxClusterSize){
  this->initEuclideanClustractor(tolerance, minClusterSize, maxClusterSize);
}

/**
* @brief <Initialize Euclidean space cluster extractor>
* @param [in] <tolerance> <max accepetable tolerance between cloud points>
* @param [in] <minClusterSize> <minimum cluster size>
* @param [in] <maxClusterSize> <maximum cluster size>
*/
void ClusterExtractor::initEuclideanClustractor(float tolerance, int minClusterSize, int maxClusterSize){
  this->ec.setClusterTolerance (tolerance);
  this->ec.setMinClusterSize (minClusterSize);
  this->ec.setMaxClusterSize (maxClusterSize);
}

/**
* @brief <extracts euclidean clusters in cloud data>
* @param [in] <cloud> <point cloud data>
* @return <true for succesful run>
* @details <computes clusters of data with respect to their 3d vicinity>
*/
bool ClusterExtractor::extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  this->ec.setInputCloud (cloud);
  this->ec.extract (cluster_indices);
  this->ec.setSearchMethod (tree);
  tree->setInputCloud (cloud);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = this->cluster_indices.begin (); it != this->cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud = cloud_cluster;
  }

  return (true);
}
