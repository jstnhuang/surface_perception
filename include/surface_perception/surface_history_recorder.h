#ifndef _SURFACE_PERCEPTION_SURFACE_HISTORY_RECORDER_H_
#define _SURFACE_PERCEPTION_SURFACE_HISTORY_RECORDER_H_

#include <ctime>
#include <map>

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace surface_perception {
/// \brief SurfaceHistoryRecorder records information of time, iteration and
/// history of each surface.
///
/// This class designed for SurfaceFinder in order to record the performance of
/// the algorithm. In particular, SurfaceHistoryRecorder tracks clock info, the
/// latest iteration and the concatenated point cloud that represents the
/// evolution of a surface.
///
/// \b Example usage:
/// \code
///  SurfaceHistoryRecorder recorder;
///
///  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
///  ...Find some surface and fill indices...
///  recorder.Record(indices->indices.size(), input_cloud, indices,
///                  iteration_number)
///
///  pcl::PointIndices::Ptr new_indices(new pcl::PointIndices);
///  ...Find another surface and fill new_indices...
///  if (surface is already seen) {
///    recorder.Update(indices->indices.size(), new_indices->indices.size(),
///                    input_cloud, new_indices,
///                    iteration_number);
///  } else {
///    recorder.Record(...)
///  }
///
///  ...
///
///  // Get the recorded result for surface i
///  pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_history(
///      new pcl::PointCloud<pcl::PointXYZRGB>);
///  clock_t current_clock;
///  size_t latest_iteration;
///  recorder.GetCloudHistory(indices_of_surface_i->indices.size(),
///                           surface_history);
///  recorder.GetClock(indices_of_surface_i->indices.size(), &current_clock);
///  recorder.GetIteration(indices_of_surface_i->indices.size(),
///                        &last_iteration);
/// \endcode
class SurfaceHistoryRecorder {
 public:
  /// \brief Record the surface data when the surface is seen in the first time.
  ///
  /// This function records the data for the surface not stored in this
  /// instance. If the surface is already seen, please use Update(...).
  ///
  /// \param[in] id The identification number of data. In SurfaceFinder, the id
  /// is the number of points
  ///  in the surface.
  /// \param[in] cloud The input cloud of SurfaceFinder.
  /// \param[in] indices The indices of the surface found.
  /// \param[in] iteration The iteration number when this surface is found.
  void Record(const size_t& id,
              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
              const pcl::PointIndices::Ptr& indices, const size_t& iteration);

  /// \brief Update the existing surface data stored in this instance.
  ///
  /// This function updates the surface data for a surface that the algorithm
  /// has already seen. If the surface found is not seen before, please use
  /// Record(...), because the id of a surface may change over time.
  ///
  /// \param[in] old_id The old identification number of the surface.
  /// \param[in] new_id The new identification number of the surface.
  /// \param[in] cloud The input point cloud of SurfaceFinder.
  /// \param[in] indices The indices of the surface found.
  /// \param[in] iteration The iteration number when the surface is found.
  void Update(const size_t& old_id, const size_t& new_id,
              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
              const pcl::PointIndices::Ptr& indices, const size_t& iteration);

  /// \brief Output the point cloud history of a surface
  ///
  /// The point cloud history, or the evolution of surface, is defined as the
  /// concatenation of past and resent points for a surface.
  ///
  /// \param[in] id The identification number of the surface.
  /// \param[out] output_cloud The concatenated point cloud history as output.
  void GetCloudHistory(
      const size_t& id,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud) const;

  /// \brief Output the latest clock when the surface is found.
  ///
  /// The clock indicates the system clock when the algorithm finds the surface.
  /// The resultant clock info is expected to be used to calculate time spent
  /// on finding a surface.
  ///
  /// \param[in] id The identification number of the surface.
  /// \param[out] clock_ptr The pointer to the recorded clock_t value.
  void GetClock(const size_t& id, clock_t* clock_ptr) const;

  /// \brief Output the latest iteration number when the surface is found.
  ///
  /// \param[in] id The identification number of the surface.
  /// \param[out] iteration_ptr The pointer to the recorded iteration number.
  void GetIteration(const size_t& id, size_t* iteration_ptr) const;

 private:
  std::map<size_t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_history_;
  std::map<size_t, clock_t> time_history_;
  std::map<size_t, size_t> iteration_history_;
};
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SURFACE_HISTORY_RECORDER_H_
