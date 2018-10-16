/**
 * @file planeExtraction.h
 * @author Jerrar Bukhari
 * @date 15 Oct 2018
 * @copyright 2018 Jerrar Bukhari
 * @brief <Header file for passthrough filter based on PCL>
 */

#include <planeExtraction.h>

PlaneExtraction::PlaneExtraction(int iter, float distThresh) {
  this->initSegmentator(iter, distThresh);
}

/**
 * @brief <Initializer for SACSegmentor>
 * @param [in] <iter> <number of iterations to repeat>
 * @param [in] <distThresh> <maximum distance to percieve as planar>
 */
void PlaneExtraction::initSegmentator(int iter, float distThresh) {
  this->seg.setOptimizeCoefficients(false);
  this->seg.setModelType(pcl::SACMODEL_PLANE);
  this->seg.setMethodType(pcl::SAC_RANSAC);
  this->seg.setMaxIterations(100);
  this->seg.setDistanceThreshold(0.02);
}

/**
 * @brief <Extract the ground plance from Cloud data>
 * @param [in] <cloud> <point cloud data>
 * @details <Segment the largest planar component from the cloud and isolate rest>
 */
void PlaneExtraction::extractGroundPlane(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZ>());
  int i = 0, nr_points = int(cloud->points.size());
  while (cloud->points.size() > 0.3 * nr_points) {
    ///< Segment the largest planar component from the remaining cloud>
    this->seg.setInputCloud(cloud);
    this->seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      std::cout << "Could not estimate a planar model for the given dataset."
          << std::endl;
      break;
    }

    ///< Extract the planar inliers from the input cloud>
    this->extract.setInputCloud(cloud);
    this->extract.setIndices(inliers);
    this->extract.setNegative(false);

    ///< Get the points associated with the planar surface>
    this->extract.filter(*cloud_plane);
    std::cout << "PointCloud representing the planar component: "
        << cloud_plane->points.size() << " data points." << std::endl;

    ///< Remove the planar inliers, extract the rest>
    this->extract.setNegative(true);
    this->extract.filter(*cloud);
  }
}
