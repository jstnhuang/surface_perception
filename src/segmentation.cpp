#include "surface_perception/segmentation.h"

#include <vector>
#include <limits>

#include "Eigen/Eigen"
#include "pcl/common/angles.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "ros/ros.h"

#include "surface_perception/object.h"
#include "surface_perception/shape_extraction.h"
#include "surface_perception/surface.h"
#include "surface_perception/surface_objects.h"
#include "surface_perception/typedefs.h"

#include "surface_ransac/surface_finder.h"

namespace surface_perception {
Segmentation::Segmentation()
    : cloud_(),
      indices_(new pcl::PointIndices),
      horizontal_tolerance_degrees_(10),
      margin_above_surface_(0.005),
      cluster_distance_(0.01),
      min_cluster_size_(10),
      max_cluster_size_(10000) {}

void Segmentation::set_input_cloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  cloud_ = cloud;
}

void Segmentation::set_indices(pcl::PointIndices::Ptr indices) {
  indices_ = indices;
}

void Segmentation::set_horizontal_tolerance_degrees(double degrees) {
  horizontal_tolerance_degrees_ = degrees;
}
void Segmentation::set_margin_above_surface(double margin) {
  margin_above_surface_ = margin;
}
void Segmentation::set_cluster_distance(double cluster_distance) {
  cluster_distance_ = cluster_distance;
}
void Segmentation::set_min_cluster_size(int min_cluster_size) {
  min_cluster_size_ = min_cluster_size;
}
void Segmentation::set_max_cluster_size(int max_cluster_size) {
  max_cluster_size_ = max_cluster_size;
}

bool Segmentation::Segment(std::vector<SurfaceObjects>* surfaces) const {
  Surface surface;
  bool success =
      FindSurface(cloud_, indices_, horizontal_tolerance_degrees_, &surface);
  if (!success) {
    ROS_ERROR("Failed to find surface.");
    return false;
  }

  SurfaceObjects surface_objects;
  success = FindObjectsOnSurface(
      cloud_, indices_, surface, margin_above_surface_, cluster_distance_,
      min_cluster_size_, max_cluster_size_, &surface_objects);
  surfaces->push_back(surface_objects);
  if (!success) {
    ROS_ERROR("Failed to cluster objects.");
    return false;
  }
  return true;
}

bool FindSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices,
                 double horizontal_tolerance_degrees, Surface* surface) {
  pcl::PointIndices::Ptr indices_internal(new pcl::PointIndices);
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);
  if (indices && indices->indices.size() > 0) {
    seg.setIndices(indices);
  }
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(horizontal_tolerance_degrees));

  surface->coefficients.reset(new pcl::ModelCoefficients);
  seg.segment(*indices_internal, *surface->coefficients);
  if (surface->coefficients->values.size() < 4) {
    return false;
  }

  surface->pose_stamped.header.frame_id = cloud->header.frame_id;
  FitBox(cloud, indices_internal, surface->coefficients,
         &surface->pose_stamped.pose, &surface->dimensions);
  return true;
}

bool FindSurfaces(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                  pcl::PointIndicesPtr indices,
                  double horizontal_tolerance_degrees, std::vector<Surface>* surfaces) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(indices);
  extract_indices.filter(*cropped_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_nan_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> transform_index;
  pcl::removeNaNFromPointCloud(*cropped_cloud, *no_nan_cloud, transform_index);

  surface_ransac::SurfaceFinder surfaceFinder;
  std::vector<pcl::PointIndices::Ptr> indices_vec;
  std::vector<pcl::ModelCoefficients> coeffs_vec;
  surfaceFinder.setCloud(no_nan_cloud);
  surfaceFinder.setMaxIteration(1000);
  surfaceFinder.setSurfacePointThreshold(1000);
  surfaceFinder.setToleranceAngle(0.0);
  surfaceFinder.setMaxPointDistance(0.01);
  surfaceFinder.exploreSurfaces(0, 10, &indices_vec, &coeffs_vec);

  if (indices_vec.size() == 0 || coeffs_vec.size() == 0) {
    ROS_INFO("Warning: no surface found.");
    return false;
  }

  for (size_t i = 0; i < indices_vec.size() && i < coeffs_vec.size(); i++) {
    Surface surface;
    surface.coefficients.reset(new pcl::ModelCoefficients);
    surface.coefficients->values = coeffs_vec[i].values;
    surface.pose_stamped.header.frame_id = no_nan_cloud->header.frame_id;
    FitBox(no_nan_cloud, indices_vec[i], surface.coefficients, &surface.pose_stamped.pose, &surface.dimensions);
    surfaces->push_back(surface);
  }

  return true;
}

void FindHeightInterval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                  pcl::PointIndicesPtr indices,
                  double* low, double* hi) {
  double tmp_low = std::numeric_limits<double>::max();
  double tmp_hi = std::numeric_limits<double>::min();

  for (size_t i = 0; i < indices->indices.size(); i++) {
    const pcl::PointXYZRGB& pt = cloud->points[indices->indices[i]];
    if (pt.z < tmp_low) {
      tmp_low = pt.z;
    }
    if (pt.z > tmp_hi) {
      tmp_hi = pt.z;
    }
  }

  *low = tmp_low;
  *hi = tmp_hi;
}

bool GetSceneAboveSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          pcl::PointIndices::Ptr indices,
                          const pcl::ModelCoefficients& coefficients,
                          double margin_above_surface,
                          pcl::PointIndices::Ptr above_surface_indices) {
  if (coefficients.values.size() < 4) {
    return false;
  }

  // Build custom indices based on margin_above_surface.
  double a = coefficients.values[0];
  double b = coefficients.values[1];
  double c = coefficients.values[2];
  double d = coefficients.values[3];

  if (indices && indices->indices.size() > 0) {
    for (size_t i = 0; i < indices->indices.size(); ++i) {
      size_t index = indices->indices[i];
      const PointC& pt = cloud->points[index];
      float val = a * pt.x + b * pt.y + c * pt.z + d;
      if (val >= margin_above_surface) {
        above_surface_indices->indices.push_back(index);
      }
    }
  } else {
    for (size_t i = 0; i < cloud->size(); ++i) {
      const PointC& pt = cloud->points[i];
      float val = a * pt.x + b * pt.y + c * pt.z + d;
      if (val >= margin_above_surface) {
        above_surface_indices->indices.push_back(i);
      }
    }
  }

  if (above_surface_indices->indices.size() == 0) {
    return false;
  }
  return true;
}

bool FindObjectsOnSurface(PointCloudC::Ptr cloud, pcl::PointIndicesPtr indices,
                          const Surface& surface, double margin_above_surface,
                          double cluster_distance, int min_cluster_size,
                          int max_cluster_size,
                          SurfaceObjects* surface_objects) {
  pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
  bool success =
      GetSceneAboveSurface(cloud, indices, *surface.coefficients,
                           margin_above_surface, above_surface_indices);
  if (!success) {
    return false;
  }

  std::vector<pcl::PointIndices> object_indices;
  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setIndices(above_surface_indices);
  euclid.setClusterTolerance(cluster_distance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(object_indices);

  surface_objects->surface = surface;
  for (size_t i = 0; i < object_indices.size(); ++i) {
    Object object;
    object.cloud = cloud;
    object.indices.reset(new pcl::PointIndices(object_indices[i]));
    object.pose_stamped.header.frame_id = cloud->header.frame_id;
    FitBox(cloud, object.indices, surface.coefficients,
           &object.pose_stamped.pose, &object.dimensions);
    surface_objects->objects.push_back(object);
  }

  return true;
}
}  // namespace surface_perception
