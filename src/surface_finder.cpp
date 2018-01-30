#include "surface_perception/surface_finder.h"
#include "surface_perception/surface_history_recorder.h"

#include <limits.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"
#include "pcl/common/angles.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace {
/**
 * This is a helper function for calculating angle between given plane and z
 * axis
 */
double calculateAngle(const double& a, const double& b, const double& c) {
  double denominator = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
  double nominator = fabs(c);
  double tmpRes = nominator / denominator;
  return acos(tmpRes);
}

/**
 * This function finds the coefficients of a plane equation, given three points
 */
void planeEquation(const std::vector<PointC>& pts, double* a, double* b,
                   double* c, double* d) {
  *a = (pts[1].y - pts[0].y) * (pts[2].z - pts[0].z) -
       (pts[2].y - pts[0].y) * (pts[1].z - pts[0].z);
  *b = (pts[1].z - pts[0].z) * (pts[2].x - pts[0].x) -
       (pts[2].z - pts[0].z) * (pts[1].x - pts[0].x);
  *c = (pts[1].x - pts[0].x) * (pts[2].y - pts[0].y) -
       (pts[2].x - pts[0].x) * (pts[1].y - pts[0].y);
  *d = -1 * (*a) * pts[0].x - (*b) * pts[0].y - (*c) * pts[0].z;
}

/**
 * This function gives a vector of points within certain distance to a plane
 * specified by the z_val
 */
void filterIndices(const double& dist_limit,
                   const std::map<double, std::vector<int> >& sortedIndices,
                   const double& z_val, std::vector<int>* output_pts) {
  std::map<double, std::vector<int> >::const_iterator iter =
      sortedIndices.find(z_val);
  if (iter == sortedIndices.end()) {
    ROS_INFO("Error: sampled point with height %f not on the plane", z_val);
    return;
  }

  std::map<double, std::vector<int> >::const_iterator curr_iter = iter;
  iter++;

  // Find the indices below z_val
  while (curr_iter != sortedIndices.begin() &&
         (z_val - curr_iter->first) <= dist_limit) {
    for (size_t i = 0; i < curr_iter->second.size(); i++) {
      output_pts->push_back(curr_iter->second[i]);
    }
    curr_iter--;
  }
  if (curr_iter == sortedIndices.begin() &&
      (z_val - curr_iter->first) <= dist_limit) {
    for (size_t i = 0; i < curr_iter->second.size(); i++) {
      output_pts->push_back(curr_iter->second[i]);
    }
  }

  // Find the indices above z_val
  curr_iter = iter;
  while (curr_iter != sortedIndices.end() &&
         (curr_iter->first - z_val) <= dist_limit) {
    for (size_t i = 0; i < curr_iter->second.size(); i++) {
      output_pts->push_back(curr_iter->second[i]);
    }
    curr_iter++;
  }
}

/**
 * This function determines if given two planes are similar
 */
bool isSimilar(const double& dist, const pcl::ModelCoefficients& plane1,
               const pcl::ModelCoefficients& plane2) {
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
      angle_tolerance_degree_(0.0),
      max_point_distance_(0.01),
      min_iteration_(100),
      surface_point_threshold_(1000),
      sorted_indices_() {}

void SurfaceFinder::set_cloud(const PointCloudC::Ptr& cloud) {
  cloud_ = cloud;

  // Fill up indices, if the indices is not specified yet
  if (cloud_indices_->header.frame_id == "" &&
      cloud_indices_->indices.size() == 0) {
    for (size_t i = 0; i < cloud_->points.size(); i++) {
      cloud_indices_->indices.push_back(i);
    }
    cloud_indices_->header.frame_id = cloud_->header.frame_id;
  }

  SortIndices();
}

void SurfaceFinder::set_cloud_indices(
    const pcl::PointIndices::Ptr cloud_indices) {
  cloud_indices_->header.frame_id = cloud_indices->header.frame_id;
  cloud_indices_->indices = cloud_indices->indices;

  SortIndices();
}

void SurfaceFinder::set_angle_tolerance_degree(
    const double& angle_tolerance_degree) {
  angle_tolerance_degree_ = pcl::deg2rad(angle_tolerance_degree);
}

void SurfaceFinder::set_max_point_distance(const double& max_point_distance) {
  max_point_distance_ = max_point_distance;
}

void SurfaceFinder::set_min_iteration(const int& min_iteration) {
  min_iteration_ = min_iteration;
}

void SurfaceFinder::set_surface_point_threshold(
    const int& surface_point_threshold) {
  surface_point_threshold_ = surface_point_threshold;
}

void SurfaceFinder::ExploreSurfaces(
    const size_t& min_surface_amount, const size_t& max_surface_amount,
    std::vector<pcl::PointIndices::Ptr>* indices_internals,
    std::vector<pcl::ModelCoefficients>* coeffs) {

  bool debug = false;

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
           std::greater<size_t> >
      ranking;

  SurfaceHistoryRecorder recorder;

  // Timer start point
  clock_t start = std::clock();

  // Sample min_iteration_ of horizontal surfaces
  while (num_surface < min_iteration_ || ranking.size() < min_surface_amount) {
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    coeff->values.resize(4);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    // Sample 1 point randomly to establish a plane, by assuming it's horizontal
    size_t rand_index = std::rand() % cloud_indices_->indices.size();
    const PointC& pt = cloud_->points[cloud_indices_->indices[rand_index]];

    // Count points within given distance to the plane
    std::vector<int> inlier_indices;
    filterIndices(max_point_distance_, sorted_indices_, pt.z, &inlier_indices);

    // Establish coefficients
    coeff->values[0] = 0;
    coeff->values[1] = 0;
    coeff->values[2] = 1;
    coeff->values[3] = -1 * pt.z;
    indices->indices = inlier_indices;

    // Check surface point threshold
    if (indices->indices.size() > surface_point_threshold_) {
      // Update plane choices
      bool qualify = true;
      bool pre_exist = false;
      size_t old_indices_size;
      for (std::map<
               size_t,
               std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr>,
               std::greater<size_t> >::iterator it = ranking.begin();
           it != ranking.end(); it++) {
        pcl::ModelCoefficients& old_coeff = *(it->second.first);

        old_indices_size = it->first;

        // If newly found surface is better, replace
        if (isSimilar(max_point_distance_, old_coeff, *coeff)) {
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
        std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> pr(
            coeff, indices);
        ranking[indices->indices.size()] = pr;
        if (debug) {
          if (pre_exist) {
            recorder.Update(old_indices_size, indices->indices.size(), cloud_,
                            indices, num_surface);
          } else {
            recorder.Record(indices->indices.size(), cloud_, indices,
                            num_surface);
          }
        }
      }
    }

    num_surface++;
  }

  //  Report surfaces
  if (ranking.size() > 0) {
    size_t amount = max_surface_amount;
    for (std::map<
             size_t,
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

      if (debug) {
        clock_t elapsed_clock;
        size_t iter_amount;
        PointCloudC::Ptr past_cloud(new PointCloudC);
        recorder.GetCloudHistory(indices->indices.size(), past_cloud);
        recorder.GetClock(indices->indices.size(), &elapsed_clock);
        recorder.GetIteration(indices->indices.size(), &iter_amount);

        ROS_INFO(
            "%f seconds spent at %ldth iteration for  %ldth surface with size "
            "%ld",
            ((float)elapsed_clock - start) / CLOCKS_PER_SEC, iter_amount,
            max_surface_amount - amount + 1, indices->indices.size());
      }
      amount--;
    }
  } else {
    ROS_INFO("Warning: no surface found.");
  }
}

void SurfaceFinder::SortIndices() {
  sorted_indices_.clear();

  // Record the indices and sort by height
  for (size_t i = 0; i < cloud_indices_->indices.size(); i++) {
    const PointC& pt = cloud_->points[cloud_indices_->indices[i]];
    std::map<double, std::vector<int> >::iterator iter =
        sorted_indices_.find(pt.z);
    if (iter != sorted_indices_.end()) {
      sorted_indices_[pt.z].push_back(cloud_indices_->indices[i]);
    } else {
      std::vector<int> indices_vec;
      indices_vec.push_back(cloud_indices_->indices[i]);
      sorted_indices_[pt.z] = indices_vec;
    }
  }
}
}  // namespace surface_perception
