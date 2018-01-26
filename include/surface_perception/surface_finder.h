#ifndef _SURFACE_PERCEPTION_SURFACE_FINDER_H_
#define _SURFACE_PERCEPTION_SURFACE_FINDER_H_

#include <map>
#include <vector>

#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace surface_perception {
/// \brief SurfaceFinder attempt to find multiple horizontal surfaces given a
/// shelf scene.
///
/// This class is designed to find surfaces in a shelf scene such as a
/// bookshelf. In particular, this class finds the horizontal surfaces if the
/// following conditions are met:
/// 1. No NaN points in the input cloud
/// 2. Each target surface has different height
///
/// If the input cloud meets the requirement. Mutation functions of this class
/// can be used to adjust the parameters based on the scenario of the point
/// cloud scene.
///
/// \b Example usage:
/// \code
///   SurfaceFinder finder;
///   finder.setCloud(pcl_cloud);
///   finder.setCloudIndices(point_indices);
///   finder.setToleranceAngle(10);
///   finder.setMaxPointDistance(0.01);
///   finder.setMaxIteration(1000);
///   finder.setSurfacePointThreshold(10000);
///
///   std::vector<pcl::PointIndices::Ptr> indices;
///   std::vector<pcl::ModelCoefficients> coeffs;
///   finder.exploreSurfaces(0, 10, &indices, &coeffs);
/// \endcode
class SurfaceFinder {
 public:
  /// \brief Default constructor
  SurfaceFinder();

  /// \brief Set the input point cloud
  ///
  /// \param NaN values in the input cloud should be removed before being passed
  /// to this function.
  void setCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  void setCloudIndices(const pcl::PointIndices::Ptr indices);
  void setToleranceAngle(const double& degrees);
  void setMaxPointDistance(const double& dist);
  void setMaxIteration(const size_t& max_iter);
  void setSurfacePointThreshold(const size_t& min_point);
  void exploreSurfaces(
      const size_t& min_surface_amount, const size_t& max_surface_amount,
      std::vector<pcl::PointIndices::Ptr>* indices_internals,
      std::vector<pcl::ModelCoefficients>* coeffs,
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* history =
          new std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>());

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  pcl::PointIndices::Ptr cloud_indices_;
  double rad_;
  double dist_;
  size_t max_iter_;
  size_t min_point_;
  std::map<double, std::vector<int> > sortedIndices_;
  void sortIndices();
};
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SURFACE_FINDER_H_
