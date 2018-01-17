#ifndef _SURFACE_PERCEPTION_SEGMENTATION_H_
#define _SURFACE_PERCEPTION_SEGMENTATION_H_

#include <vector>

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "surface_perception/surface.h"
#include "surface_perception/surface_objects.h"

namespace surface_perception {
/// \brief Segmentation is the algorithm for tabletop segmentation.
///
/// This algorithm takes in a tabletop scene and segments it into a tabletop
/// surface with some number of objects above it. The objects are segmented
/// using a Euclidean clustering algorithm. The algorithm fits oriented bounding
/// boxes around the surface and the objects. For each object, the z direction
/// points "up," and the x direction points toward the shorter side of the
/// oriented bounding box.
///
/// The algorithm assumes that the input scene is provided such that the
/// positive z direction points "up."
///
/// \b Example usage:
/// \code
///   surface_perception::Segmentation seg;
///   seg.set_input_cloud(pcl_cloud);
///   seg.set_indices(point_indices);
///   seg.set_horizontal_tolerance_degrees(10);
///   seg.set_margin_above_surface(0.01);
///   seg.set_cluster_distance(0.01);
///   seg.set_min_cluster_size(10);
///   seg.set_max_cluster_size(10000);
///
///   std::vector<SurfaceObjects> surface_objects;
///   bool success = seg.Segment(&surface_objects);
/// \endcode
///
/// In the future, we hope to extend this algorithm to automatically segment
/// shelf scenes as well as tabletop scenes.
class Segmentation {
 public:
  /// \brief Default constructor.
  Segmentation();

  /// \brief Sets the input cloud to use.
  ///
  /// The cloud should be provided such that the positive z direction points up.
  void set_input_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  /// \brief Sets the indices in the point cloud to use as input.
  void set_indices(pcl::PointIndicesPtr indices);

  /// \brief Sets the tolerance for a surface to be considered horizontal.
  ///
  /// \param[in] degrees The tolerance, in degrees, for a surface to be
  ///   considered horizontal.
  void set_horizontal_tolerance_degrees(double degrees);

  /// \brief Sets a margin above the surface to start segmenting objects from.
  ///
  /// The algorithm fits a plane to the surface. Some points above that plane
  /// will correspond to the surface, so the margin is used to ignore all points
  /// within a certain distance above the plane.
  ///
  /// \param[in] margin The margin, in the same units as the point cloud
  ///   (usually meters), above the fitted plane to consider as part of the
  ///   surface.
  void set_margin_above_surface(double margin);

  /// \brief Sets the minimum distance between object clusters.
  ///
  /// The algorithm clusters nearby points and considers each cluster an object.
  /// This parameter is the threshold used to determine if two points are part
  /// of the same object or if they are part of different objects.
  ///
  /// \param[in] cluster_distance The cluster distance, in the same units as the
  ///   point cloud (usually meters).
  void set_cluster_distance(double cluster_distance);

  /// \brief Sets the minimum cluster size.
  ///
  /// The algorithm clusters nearby points and considers each cluster an object.
  /// This parameter is used as a filter to ignore noise. This parameter is
  /// sensitive to the density of the input point cloud.
  ///
  /// \param[in] min_cluster_size The number of points that must be in a cluster
  ///   for that cluster to be considered an object.
  void set_min_cluster_size(int min_cluster_size);

  /// \brief Sets the minimum cluster size.
  ///
  /// The algorithm clusters nearby points and considers each cluster an object.
  /// This parameter is used to filter out large parts of the scene that are
  /// above the surface but are not objects (e.g., a back wall). This parameter
  /// is sensitive to the density of the input point cloud.
  ///
  /// \param[in] max_cluster_size The maximum number of points that can be in a
  ///   cluster for the cluster to be considered an object.
  void set_max_cluster_size(int max_cluster_size);

  /// \brief Segments the scene.
  ///
  /// \param[out] surfaces The vector of SurfaceObjects to append to. This
  ///   algorithm only supports tabletop segmentation at this time, so it will
  ///   append one SurfaceObjects instance if successful.
  ///
  /// \returns true if the segmentation was successful, false otherwise. Reasons
  ///   why segmentation might fail include the surface not being found and no
  ///   points were found above the surface.
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

/// \brief Finds a surface in the given point cloud.
///
/// \param[in] cloud The point cloud to find a surface in, where positive z
///   points up.
/// \param[in] indices The indices in the point cloud to find a surface in.
/// \param[in] horizontal_tolerance_degrees The tolerance, in degrees, for a
///   surface to be considered horizontal.
/// \param[out] surface The detected surface (may be changed even if no surface
///   is found).
///
/// \returns true if a surface was found, false otherwise.
bool FindSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 pcl::PointIndicesPtr indices,
                 double horizontal_tolerance_degrees, Surface* surface);

/// \brief Extracts the part of the point cloud above a given surface.
///
/// \param[in] cloud The point cloud to find a surface in, where positive z
///   points up.
/// \param[in] indices The indices in the point cloud to extract from.
/// \param[in] coefficients The coefficients of a plane representing a
///   horizontal surface: ax + by + cz + d = 0, where a, b, c, and d are the
///   first four coefficients.
/// \param[in] margin_above_surface The margin to extend above the surface
///   plane, in meters.
/// \param[out] above_surface_indices The indices in the given point cloud
///   representing the points above the plane (and the margin above the plane).
///
/// \returns true if the plane model coefficients are valid and a non-zero
///   number of points were found above the plane, false otherwise.
bool GetSceneAboveSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          pcl::PointIndicesPtr indices,
                          const pcl::ModelCoefficients& coefficients,
                          double margin_above_surface,
                          pcl::PointIndices::Ptr above_surface_indices);

/// \brief The algorithm that segments objects above a given surface.
///
/// \param[in] cloud The point cloud to find a surface in, where positive z
///   points up.
/// \param[in] indices The indices in the point cloud to segment from.
/// \param[in] surface The surface to segment objects from.
/// \param[in] margin_above_surface The margin to extend above the surface
///   plane, in meters.
/// \param[in] cluster_distance The distance between points (in meters) for
///   those points to be considered part of separate objects.
/// \param[in] min_cluster_size The minimum number of points that must be in a
///   cluster for that cluster to be considered an object.
/// \param[in] max_cluster_size The maximum number of points that can be in a
///   cluster for that cluster to be considered an object.
/// \param[out] surface_objects The surface and the objects above it that were
///   found.
///
/// \returns true if the segmentation was successful, false otherwise.
bool FindObjectsOnSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          pcl::PointIndicesPtr indices, const Surface& surface,
                          double margin_above_surface, double cluster_distance,
                          int min_cluster_size, int max_cluster_size,
                          SurfaceObjects* surface_objects);
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SEGMENTATION_H_
