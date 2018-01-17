#ifndef _SURFACE_RANSAC_SURFACE_HISTORY_RECORDER_H_
#define _SURFACE_RANSAC_SURFACE_HISTORY_RECORDER_H_

#include <ctime>
#include <map>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"

namespace surface_ransac {
class SurfaceHistoryRecorder {
 public:
  void record(const size_t& id,
              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& full_cloud,
              const pcl::PointIndices::Ptr& indices,
              const size_t& iteration);
  void update(const size_t& old_id,
              const size_t& new_id,
              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& full_cloud,
              const pcl::PointIndices::Ptr& indices,
              const size_t& iteration);
  void getCloud(const size_t& id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud) const;
  void getTime(const size_t& id, time_t* time_ptr) const;
  void getIteration(const size_t& id, size_t* iter_ptr) const;

 private:
  std::map<size_t,
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_history_;
  std::map<size_t, time_t> time_history_;
  std::map<size_t, size_t> iter_history_;
};
}  // namespace surface_ransac

#endif  // _SURFACE_RANSAC_SURFACE_HISTORY_RECORDER_H_
