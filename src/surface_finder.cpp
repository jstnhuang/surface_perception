#include "surface_perception/surface_finder.h"
#include "surface_perception/surface_history_recorder.h"

#include <utility>
#include <vector>
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <limits.h>
#include <map>
#include <functional>
#include <cmath>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/common/angles.h"
#include "ros/ros.h"
#include "pcl/filters/extract_indices.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace {
/**
 * This is a helper function for calculating angle between given plane and z axis
 */
double calculateAngle(const double &a, const double &b, const double &c) {
  double denominator = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
  double nominator = fabs(c);
  double tmpRes = nominator / denominator; 
  return acos(tmpRes);
}

/**
 * This function finds the coefficients of a plane equation, given three points
 */
void planeEquation(const std::vector<PointC>& pts, double* a, double* b, double* c, double* d) {
  *a = (pts[1].y - pts[0].y) * (pts[2].z - pts[0].z)
       - (pts[2].y - pts[0].y) * (pts[1].z - pts[0].z);
  *b = (pts[1].z - pts[0].z) * (pts[2].x - pts[0].x)
       - (pts[2].z - pts[0].z) * (pts[1].x - pts[0].x);
  *c = (pts[1].x - pts[0].x) * (pts[2].y - pts[0].y)
       - (pts[2].x - pts[0].x) * (pts[1].y - pts[0].y);
  *d = -1 * (*a) * pts[0].x - (*b) * pts[0].y - (*c) * pts[0].z;
}

/**
 * This function gives a vector of points within certain distance to a plane specified by
 * the z_val
 */
void filterIndices(const double& dist_limit,
                   const std::map<double, std::vector<int> >& sortedIndices,
                   const double& z_val,
                   std::vector<int>* output_pts) {

  std::map<double, std::vector<int> >::const_iterator iter = sortedIndices.find(z_val);
  if (iter == sortedIndices.end()) {
    ROS_INFO("Error: sampled point with height %f not on the plane", z_val);
    return;
  }

  std::map<double, std::vector<int> >::const_iterator curr_iter = iter;
  iter++;

  // Find the indices below z_val
  while (curr_iter != sortedIndices.begin()
         && (z_val - curr_iter->first) <= dist_limit) {
    for (size_t i = 0; i < curr_iter->second.size(); i++) {
      output_pts->push_back(curr_iter->second[i]);
    }
    curr_iter--;
  }
  if (curr_iter == sortedIndices.begin()
      && (z_val - curr_iter->first) <= dist_limit) {
    for (size_t i = 0; i < curr_iter->second.size(); i++) {
      output_pts->push_back(curr_iter->second[i]);
    }
  }

  // Find the indices above z_val
  curr_iter = iter;
  while (curr_iter != sortedIndices.end()
         && (curr_iter->first - z_val) <= dist_limit) {
    for (size_t i = 0; i < curr_iter->second.size(); i++) {
      output_pts->push_back(curr_iter->second[i]);
    }
    curr_iter++;
  }
}

/**
 * This function determines if given two planes are similar
 */
bool isSimilar(const double& dist, const pcl::ModelCoefficients& plane1, const pcl::ModelCoefficients& plane2) {
  double z1 = -1.0 * plane1.values[3] / plane1.values[2];
  double z2 = -1.0 * plane2.values[3] / plane2.values[2];
  return fabs(z1 - z2) < 10 * dist;
}

/**
 * This function stores three random numbers into rand_nums
 */
void sampleThreeNums(size_t rand_range, size_t nums_size, size_t rand_nums[]) {
  for (size_t i = 0; i < nums_size; i++) {
    rand_nums[i] = std::rand() % rand_range;
    for (size_t j = 0; j < i; j++) {
      while (rand_nums[j] == rand_nums[i]) {
        rand_nums[i] = std::rand() % rand_range;
      }
    }
  }
}
}  // Anonymous namespace

namespace surface_perception {
SurfaceFinder::SurfaceFinder()
    : cloud_(new PointCloudC),
      cloud_indices_(new pcl::PointIndices),
      rad_(0.0),
      dist_(0.01),
      max_iter_(100),
      sortedIndices_() {}

void SurfaceFinder::setCloud(const PointCloudC::Ptr& cloud) {
  cloud_ = cloud;

  // Fill up indices, if the indices is not specified yet
  if (cloud_indices_->header.frame_id == "" && cloud_indices_->indices.size() == 0) {
    for (size_t i = 0; i < cloud_->points.size(); i++) {
      cloud_indices_->indices.push_back(i);
    }
    cloud_indices_->header.frame_id = cloud_->header.frame_id;
  }

  sortIndices();
}

void SurfaceFinder::setCloudIndices(const pcl::PointIndices::Ptr indices) {
  cloud_indices_->header.frame_id = indices->header.frame_id;
  cloud_indices_->indices = indices->indices;

  sortIndices();
}

void SurfaceFinder::setToleranceAngle(const double& degrees) {
  rad_ = pcl::deg2rad(degrees);
}

void SurfaceFinder::setMaxPointDistance(const double& dist) {
  dist_ = dist;
}

void SurfaceFinder::setMaxIteration(const size_t& max_iter) {
  max_iter_ = max_iter;
}

void SurfaceFinder::setSurfacePointThreshold(const size_t& min_point) {
  min_point_ = min_point;
}

void SurfaceFinder::exploreSurfaces(const size_t& min_surface_amount,
                                    const size_t& max_surface_amount,
                                    std::vector<pcl::PointIndices::Ptr>* indices_internals,
                                    std::vector<pcl::ModelCoefficients>* coeffs,
                                    std::vector<PointCloudC::Ptr>* history) {

  // Algorithm overview:
  // 1. Get a point randomly from cloud_->points, which is a vector<PointCloudC> 
  // 2. Calculate the horizontal plane
  // 3. Store the plane and rank it by number of points the plane covers

  ROS_INFO("Start exploring surfaces in %ld indices of %s",
           cloud_indices_->indices.size(),
           cloud_indices_->header.frame_id.c_str());

  size_t num_surface = 0; 
  size_t max_inlier_count = std::numeric_limits<size_t>::min();
  std::srand(unsigned(std::time(0)));

  std::map<size_t,
           std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr>,
           std::greater<size_t> > ranking;

  SurfaceHistoryRecorder recorder;

  // Timer start point
  time_t start = std::time(0);

  // Sample max_iter_ of horizontal surfaces
  while (num_surface < max_iter_ || ranking.size() < min_surface_amount) {
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    coeff->values.resize(4);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    // Sample 1 point randomly to establish a plane, by assuming it's horizontal
    size_t rand_index = std::rand() % cloud_indices_->indices.size();
    const PointC& pt = cloud_->points[cloud_indices_->indices[rand_index]];

    // Count points within given distance to the plane
    std::vector<int> inlier_indices;
    filterIndices(dist_, sortedIndices_, pt.z, &inlier_indices);

    // Establish coefficients
    coeff->values[0] = 0;
    coeff->values[1] = 0;
    coeff->values[2] = 1;
    coeff->values[3] = -1 * pt.z;
    indices->indices = inlier_indices;

    // Check surface point threshold
    if (indices->indices.size() > min_point_) {
      // Update plane choices
      bool qualify = true;
      bool pre_exist = false;
      size_t old_indices_size;
      for (std::map<size_t,
                    std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr>,
                    std::greater<size_t> >::iterator it = ranking.begin();
           it != ranking.end(); it++) {
        pcl::ModelCoefficients& old_coeff = *(it->second.first);
  
        old_indices_size = it->first;
  
        // If newly found surface is better, replace
        if (isSimilar(dist_, old_coeff, *coeff)) {
          if (old_indices_size < indices->indices.size()) {
            ranking.erase(it);
            pre_exist = true;
            break;
          } else {
            qualify = false;
          }
        }
      }
      if (qualify) {
        std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> pr(coeff, indices);
        ranking[indices->indices.size()] = pr;
        if (pre_exist) {
          recorder.update(old_indices_size, indices->indices.size(), cloud_, indices, num_surface);
        } else {
          recorder.record(indices->indices.size(), cloud_, indices, num_surface);
        }
      }
    }

    num_surface++;
  }

  //  Report surfaces
  if (ranking.size() > 0) {
    size_t amount = max_surface_amount;
    for (std::map<size_t,
                  std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr>,
                  std::greater<size_t> >::iterator it = ranking.begin();
         it != ranking.end(); it++) {
      if (amount == 0) {
        break;
      }
      pcl::ModelCoefficients& coeff = *(it->second.first);
      pcl::PointIndices::Ptr indices = it->second.second;
      indices_internals->push_back(indices);
      coeffs->push_back(coeff);

      time_t elapsed_time;
      size_t iter_amount;
      PointCloudC::Ptr past_cloud(new PointCloudC);
      recorder.getCloud(indices->indices.size(), past_cloud);
      recorder.getTime(indices->indices.size(), &elapsed_time);
      recorder.getIteration(indices->indices.size(), &iter_amount);

      history->push_back(past_cloud);

      ROS_INFO("%f seconds spent at %ldth iteration for  %ldth surface with size %ld",
               std::difftime(elapsed_time, start),
               iter_amount,
               max_surface_amount - amount + 1,
               indices->indices.size());

      amount--;
    }
  } else {
    ROS_INFO("Warning: no surface found.");
  }
}

void SurfaceFinder::sortIndices() {
  sortedIndices_.clear();

  // Record the indices and sort by height
  for (size_t i = 0; i < cloud_indices_->indices.size(); i++) {
    const PointC& pt = cloud_->points[cloud_indices_->indices[i]];
    std::map<double, std::vector<int> >::iterator iter = sortedIndices_.find(pt.z);
    if (iter != sortedIndices_.end()) {
      sortedIndices_[pt.z].push_back(cloud_indices_->indices[i]);
    } else {
      std::vector<int> indices_vec;
      indices_vec.push_back(cloud_indices_->indices[i]);
      sortedIndices_[pt.z] = indices_vec;
    }
  }
}
}
