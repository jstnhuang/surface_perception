#include "surface_perception/segmentation.h"

#include <vector>

#include "Eigen/Eigen"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "ros/ros.h"

#include "surface_perception/object.h"
#include "surface_perception/shape_extraction.h"
#include "surface_perception/surface.h"
#include "surface_perception/surface_objects.h"
#include "surface_perception/typedefs.h"

#include "surface_perception/surface_finder.h"

namespace {
bool SurfaceComparator(const surface_perception::Surface& s1,
                       const surface_perception::Surface& s2) {
  return s1.pose_stamped.pose.position.z < s2.pose_stamped.pose.position.z;
}

}  // Anonymous namespace

namespace surface_perception {
Segmentation::Segmentation()
    : cloud_(),
      indices_(new pcl::PointIndices),
      horizontal_tolerance_degrees_(10),
      margin_above_surface_(0.005),
      cluster_distance_(0.01),
      min_cluster_size_(10),
      max_cluster_size_(10000),
      min_surface_size_(5000) {}

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
void Segmentation::set_min_surface_size(int min_surface_size) {
  min_surface_size_ = min_surface_size;
}
bool Segmentation::Segment(std::vector<SurfaceObjects>* surfaces) const {
  std::vector<Surface> surface_vec;
  bool success = FindSurfaces(cloud_, indices_, margin_above_surface_,
                              horizontal_tolerance_degrees_, min_surface_size_,
                              &surface_vec);
  if (!success) {
    ROS_ERROR("Failed to find any surface.");
    return false;
  }

  success = FindObjectsOnSurfaces(
      cloud_, indices_, surface_vec, margin_above_surface_, cluster_distance_,
      min_cluster_size_, max_cluster_size_, surfaces);

  if (!success) {
    ROS_ERROR("Failed to cluster objects.");
    return false;
  }
  return true;
}

bool FindSurfaces(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices,
                  double margin_above_surface,
                  double horizontal_tolerance_degrees, int min_surface_size,
                  std::vector<Surface>* surfaces) {
  SurfaceFinder surfaceFinder;
  std::vector<pcl::PointIndices::Ptr> indices_vec;
  std::vector<pcl::ModelCoefficients> coeffs_vec;
  surfaceFinder.set_cloud(cloud);
  surfaceFinder.set_cloud_indices(indices);
  surfaceFinder.set_min_iteration(1000);
  surfaceFinder.set_surface_point_threshold(min_surface_size);
  surfaceFinder.set_angle_tolerance_degree(horizontal_tolerance_degrees);
  surfaceFinder.set_max_point_distance(margin_above_surface);
  surfaceFinder.ExploreSurfaces(&indices_vec, &coeffs_vec);

  if (indices_vec.size() == 0 || coeffs_vec.size() == 0) {
    ROS_INFO("Warning: no surface found.");
    return false;
  }

  for (size_t i = 0; i < indices_vec.size() && i < coeffs_vec.size(); i++) {
    Surface surface;
    surface.coefficients.reset(new pcl::ModelCoefficients);
    surface.coefficients->values = coeffs_vec[i].values;
    surface.pose_stamped.header.frame_id = cloud->header.frame_id;
    FitBox(cloud, indices_vec[i], surface.coefficients,
           &surface.pose_stamped.pose, &surface.dimensions);

    // Adjust the center of surface
    double offset =
        surface.coefficients->values[0] * surface.pose_stamped.pose.position.x +
        surface.coefficients->values[1] * surface.pose_stamped.pose.position.y +
        surface.coefficients->values[2] * surface.pose_stamped.pose.position.z +
        surface.coefficients->values[3];
    surface.pose_stamped.pose.position.z -= offset;
    surfaces->push_back(surface);
  }

  return true;
}

bool GetSceneAboveSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          pcl::PointIndices::Ptr indices,
                          const pcl::ModelCoefficients& coefficients,
                          double margin_above_surface, float height_limit,
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
      if (val >= margin_above_surface &&
          height_limit - pt.z > margin_above_surface) {
        above_surface_indices->indices.push_back(index);
      }
    }
  } else {
    for (size_t i = 0; i < cloud->size(); ++i) {
      const PointC& pt = cloud->points[i];
      float val = a * pt.x + b * pt.y + c * pt.z + d;
      if (val >= margin_above_surface &&
          height_limit - pt.z > margin_above_surface) {
        above_surface_indices->indices.push_back(i);
      }
    }
  }

  if (above_surface_indices->indices.size() == 0) {
    return false;
  }
  return true;
}

bool FindObjectsOnSurfaces(PointCloudC::Ptr cloud, pcl::PointIndicesPtr indices,
                           const std::vector<Surface>& surface_vec,
                           double margin_above_surface, double cluster_distance,
                           int min_cluster_size, int max_cluster_size,
                           std::vector<SurfaceObjects>* surfaces_objects_vec) {
  // Copy the vector and sort by height
  std::vector<Surface> surfaces = surface_vec;
  std::sort(surfaces.begin(), surfaces.end(), SurfaceComparator);
  std::vector<pcl::PointIndices::Ptr> above_surface_indices_vec;

  // Obtain indices between each horizontal surfaces
  for (size_t i = 0; i < surfaces.size(); i++) {
    float height_limit = std::numeric_limits<float>::max();
    pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices);
    if (i != (surfaces.size() - 1)) {
      // Estimate the height using information computed through FitBox
      height_limit = surfaces[i + 1].pose_stamped.pose.position.z -
                     surfaces[i + 1].dimensions.z / 2.0;
    }

    bool success = GetSceneAboveSurface(
        cloud, indices, *surfaces[i].coefficients, margin_above_surface,
        height_limit, above_surface_indices);
    if (!success) {
      ROS_ERROR(
          "Error: extraction of indices above a horizontal surface failed");
      for (size_t j = 0; j < surfaces.size(); j++) {
        ROS_ERROR("Surface %ld at (%f, %f, %f) with dimesions (%f, %f, %f)", j,
                  surfaces[j].pose_stamped.pose.position.x,
                  surfaces[j].pose_stamped.pose.position.y,
                  surfaces[j].pose_stamped.pose.position.z,
                  surfaces[j].dimensions.x, surfaces[j].dimensions.y,
                  surfaces[j].dimensions.z);
      }
      return false;
    }

    above_surface_indices_vec.push_back(above_surface_indices);
  }

  for (size_t i = 0; i < above_surface_indices_vec.size(); i++) {
    std::vector<pcl::PointIndices> object_indices;
    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    euclid.setIndices(above_surface_indices_vec[i]);
    euclid.setClusterTolerance(cluster_distance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(object_indices);

    SurfaceObjects surface_objects;
    surface_objects.surface = surfaces[i];
    for (size_t j = 0; j < object_indices.size(); j++) {
      Object object;
      object.cloud = cloud;
      object.indices.reset(new pcl::PointIndices(object_indices[j]));
      object.pose_stamped.header.frame_id = cloud->header.frame_id;
      FitBox(cloud, object.indices, surfaces[i].coefficients,
             &object.pose_stamped.pose, &object.dimensions);
      surface_objects.objects.push_back(object);
    }
    surfaces_objects_vec->push_back(surface_objects);
  }

  return true;
}
}  // namespace surface_perception
