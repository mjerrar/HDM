/**
 * @file planeExtraction.h
 * @author Jerrar Bukhari
 * @date 15 Oct 2018
 * @copyright 2018 Jerrar Bukhari
 * @brief <Header file for passthrough filter based on PCL>
 */

#ifndef INCLUDE_PLANEEXTRACTION_H_
#define INCLUDE_PLANEEXTRACTION_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class PlaneExtraction {
 private:
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;

 public:
  PlaneExtraction(int iter, float distThresh);
  void initSegmentator(int iter, float distThresh);
  void extractGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif  // INCLUDE_PLANEEXTRACTION_H_
