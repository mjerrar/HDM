/**
 * @file passthrough.h
 * @author Jerrar Bukhari
 * @date 15 Oct 2018
 * @copyright 2018 Jerrar Bukhari
 * @brief <Header file for passthrough filter based on PCL>
 */

#ifndef INCLUDE_PASSTHROUGH_H_
#define INCLUDE_PASSTHROUGH_H_

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <iostream>

class PassThroughFilter {
 protected:
  float x_min, x_max, y_min, y_max, z_min, z_max;
  pcl::PassThrough<pcl::PointXYZ> pass_filter;
 public:
  PassThroughFilter(float x_min_, float x_max_, float y_min_, float y_max_,
                    float z_min_, float z_max_);
  void set_cutoffs(float x_min_, float x_max_, float y_min_, float y_max_,
                   float z_min_, float z_max_);
  void get_cutoffs();
  bool filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif  //  INCLUDE_PASSTHROUGH_H_
