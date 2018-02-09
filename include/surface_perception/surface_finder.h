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
///   finder.set_cloud(pcl_cloud);
///   finder.set_cloud_indices(point_indices);
///   finder.set_angle_tolerance_degree(10);
///   finder.set_max_point_distance(0.01);
///   finder.set_min_iteration(1000);
///   finder.set_surface_point_threshold(10000);
///
///   std::vector<pcl::PointIndices::Ptr> indices;
///   std::vector<pcl::ModelCoefficients> coeffs;
///   finder.ExploreSurfaces(&indices, &coeffs);
/// \endcode
class SurfaceFinder {
 public:
  /// \brief Default constructor
  SurfaceFinder();

  /// \brief Set the input point cloud.
  ///
  /// \param[in] cloud 	The input cloud for surface detection. NaN values in
  ///   the input cloud should be removed before being passed to this function.
  void set_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

  /// \brief Set the indices of the input point cloud.
  ///
  /// \param[in] cloud_indices The indices of the input point cloud.
  void set_cloud_indices(const pcl::PointIndices::Ptr cloud_indices);

  /// \brief Set the angle of the surface compare to horizontal surface
  ///
  /// Because the current state algorithm only search for horizontal surfaces,
  /// the angle set through this function is not used in surface detection.
  ///
  /// \param[in] angle_tolerance_degree The maximum angle difference between a
  ///   surface candidate against a horizontal surface.
  void set_angle_tolerance_degree(double angle_tolerance_degree);

  /// \brief Set the maximum distance for a point to be considered part of
  /// surfaces.
  ///
  /// \param[in] max_point_distance The maximum distance between a point and
  ///   a plane that represents the surface.
  void set_max_point_distance(double max_point_distance);

  /// \brief Set the minimum number of iterations for the algorithm to find
  /// surfaces
  ///
  /// The algorithm is designed to run at least the given number of iteration or
  /// finding the specified minimum number of surfaces in a given scene.
  ///
  /// \param[in] min_iteration The minimum number of iteration required before
  ///   the algorithm can stop search for surfaces.
  void set_min_iteration(size_t min_iteration);

  /// \brief Set the minimum amount of points contained by a surface candidate.
  ///
  /// The amount of points for a surface is indicated by the number of points
  /// reside within the maximum distance of a plane. As the algorithm explores
  /// surfaces, the surface that have points less that required amount will be
  /// considered a candidate.
  ///
  /// \param[in] surface_point_threshold The minimum number of points a surface
  ///   candidate must have.
  void set_surface_point_threshold(size_t surface_point_threshold);

  /// \brief Set the minimum number of surfaces in the output of
  ///   ExploreSurfaces.
  ///
  /// \param[in] min_surface_amount The specified number of surfaces that must
  ///   be in the output of ExploreSurfaces.
  void set_min_surface_amount(size_t min_surface_amount);

  /// \brief Set the maximum number of the surfaces in the output of
  ///   ExploreSurfaces.
  ///
  /// \param[in] max_surface_amount The upper bound of the surface amount that
  ///   can not be exceeded in the output of ExploreSurfaces.
  void set_max_surface_amount(size_t max_surface_amount);

  /// \brief Find the horizontal surfaces in a point cloud scene
  ///
  /// The algorithm attempts to surfaces in a point cloud scene and terminate if
  /// all of the following conditions are met:
  ///  1. The algorithm finds the minimum number of surface required.
  ///  2. The algorithm finishes the specified number of iteration.
  ///
  /// Once the surfaces are found, the surface i has:
  ///  1. indices as indices_vec[i]
  ///  2. coefficients as coeffs_vec[i]
  ///
  /// \param[out] indices_vec The indices for of each output surface.
  /// \param[out] coeffs_vec The coefficients of planes that represent each
  ///   surface.
  void ExploreSurfaces(std::vector<pcl::PointIndices::Ptr>* indices_vec,
                       std::vector<pcl::ModelCoefficients>* coeffs_vec);

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  pcl::PointIndices::Ptr cloud_indices_;
  double angle_tolerance_degree_;
  double max_point_distance_;
  size_t min_iteration_;
  size_t surface_point_threshold_;
  size_t min_surface_amount_;
  size_t max_surface_amount_;
  std::map<double, std::vector<int> > sorted_indices_;
  void SortIndices();
  void FitSurface(const pcl::PointIndices::Ptr old_indices_ptr,
                  const pcl::ModelCoefficients::Ptr old_coeff_ptr,
                  pcl::PointIndices::Ptr new_indices_ptr,
                  pcl::ModelCoefficients::Ptr new_coeff_ptr);
};
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SURFACE_FINDER_H_
