/**
 * @file passthorugh.cpp
 * @author Jerrar Bukhari
 * @date 15 Oct 2018
 * @copyright 2018 Jerrar Bukhari
 * @brief <Implementation file for passthrough filter based on PCL>
 */

#include <passthrough.h>

PassThroughFilter::PassThroughFilter(float x_min_, float x_max_, float y_min_,
                                     float y_max_, float z_min_, float z_max_)
    : pass_filter() {
  this->set_cutoffs(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
}

/**
 * @brief <set min and max cut_off regions for passthrough filter>
 * @param [in] <x_min, x_max> < cut_off region for x-axis>
 * @param [in] <y_min, y_max> < cut_off region for y-axis>
 * @param [in] <z_min, z_max> < cut_off region for z-axis>
 */
void PassThroughFilter::set_cutoffs(float x_min_, float x_max_, float y_min_,
                                    float y_max_, float z_min_, float z_max_) {
  this->x_min = x_min_;
  this->x_max = x_max_;
  this->y_min = y_min_;
  this->y_max = y_max_;
  this->z_min = z_min_;
  this->z_max = z_max_;
}

void PassThroughFilter::get_cutoffs() {
  std::cout << "x:" << this->x_min << std::endl << "y:" << this->y_min
      << std::endl << "z:" << this->z_min << std::endl;
}

/**
 * @brief <compute the passthorugh filter on cloud data at each dimension>
 * @param [in] <cloud> <cloud data to be filtered>
 * @return <filtered cloud data and boolean success>
 */
bool PassThroughFilter::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  this->pass_filter.setInputCloud(cloud);
  this->pass_filter.setFilterFieldName("x");
  this->pass_filter.setFilterLimits(this->x_min, this->x_max);
  this->pass_filter.filter(*cloud);

  this->pass_filter.setInputCloud(cloud);
  this->pass_filter.setFilterFieldName("y");
  this->pass_filter.setFilterLimits(this->y_min, this->y_max);
  this->pass_filter.filter(*cloud);

  this->pass_filter.setInputCloud(cloud);
  this->pass_filter.setFilterFieldName("z");
  this->pass_filter.setFilterLimits(this->z_min, this->z_max);
  this->pass_filter.filter(*cloud);

  if (1) {
    return true;
  }
}
