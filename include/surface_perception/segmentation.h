#ifndef _SURFACE_PERCEPTION_SEGMENTATION_H_
#define _SURFACE_PERCEPTION_SEGMENTATION_H_

#include <vector>

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "surface_perception/surface.h"
#include "surface_perception/surface_objects.h"

namespace surface_perception {
class Segmentation {
 public:
  Segmentation();
  void set_input_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  void set_indices(pcl::PointIndicesPtr indices);

  void set_horizontal_tolerance_degrees(double degrees);
  void set_margin_above_surface(double margin);
  void set_cluster_distance(double cluster_distance);
  void set_min_cluster_size(int min_cluster_size);
  void set_max_cluster_size(int max_cluster_size);

  bool Segment(std::vector<SurfaceObjects>* surfaces) const;

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  pcl::PointIndicesPtr indices_;

  double horizontal_tolerance_degrees_;
  double margin_above_surface_;
  double cluster_distance_;
  int min_cluster_size_;
  int max_cluster_size_;
};

bool FindSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 pcl::PointIndicesPtr indices,
                 double horizontal_tolerance_degrees, Surface* coefficients);

bool GetSceneAboveSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          pcl::PointIndicesPtr indices,
                          const pcl::ModelCoefficients& coefficients,
                          double margin_above_surface,
                          pcl::PointIndices::Ptr above_surface_indices);

bool FindObjectsOnSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          pcl::PointIndicesPtr indices, const Surface& surface,
                          double margin_above_surface, double cluster_distance,
                          int min_cluster_size, int max_cluster_size,
                          SurfaceObjects* surface_objects);
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SEGMENTATION_H_
