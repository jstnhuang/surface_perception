#include "surface_perception/surface_history_recorder.h"

#include <ctime>
#include <map>

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace surface_ransac {
void SurfaceHistoryRecorder::record(const size_t& id,
                                    const PointCloudC::Ptr& full_cloud,
                                    const pcl::PointIndices::Ptr& indices,
                                    const size_t& iteration) {
  update(0, id, full_cloud, indices, iteration);
}

void SurfaceHistoryRecorder::update(const size_t& old_id,
                                    const size_t& new_id,
                                    const PointCloudC::Ptr& full_cloud,
                                    const pcl::PointIndices::Ptr& indices,
                                    const size_t& iteration) {
  if (new_id == 0) {
    ROS_INFO("Warning: SurfaceHistoryRecorder doesn't record indices of size 0");
    return;
  }

  PointCloudC::Ptr new_cloud(new PointCloudC);
  pcl::ExtractIndices<PointC> extract_indices;
  extract_indices.setInputCloud(full_cloud);
  extract_indices.setIndices(indices);
  extract_indices.filter(*new_cloud);

  std::map<size_t, PointCloudC::Ptr>::iterator iter = cloud_history_.find(old_id);
  if (iter != cloud_history_.end()) {
    *new_cloud += *(iter->second);
  }

  cloud_history_[new_id] = new_cloud;
  time_history_[new_id] = std::time(0);
  iter_history_[new_id] = iteration;
}

void SurfaceHistoryRecorder::getCloud(const size_t& id, const PointCloudC::Ptr output_cloud) const {
  std::map<size_t, PointCloudC::Ptr>::const_iterator iter = cloud_history_.find(id);
  if (iter != cloud_history_.end()) {
    *output_cloud = *(iter->second);
  }
}

void SurfaceHistoryRecorder::getTime(const size_t& id, time_t* time_ptr) const {
  std::map<size_t, time_t>::const_iterator iter = time_history_.find(id);
  if (iter != time_history_.end()) {
    *time_ptr = iter->second;
  }
}

void SurfaceHistoryRecorder::getIteration(const size_t& id, size_t* iter_ptr) const {
  std::map<size_t, size_t>::const_iterator iter = iter_history_.find(id);
  if (iter != iter_history_.end()) {
    *iter_ptr = iter->second;
  }
}
}  // namespace surface_ransac
