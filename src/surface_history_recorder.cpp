#include "surface_perception/surface_history_recorder.h"

#include <ctime>
#include <map>

#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace surface_perception {
void SurfaceHistoryRecorder::Record(const size_t& id,
                                    const PointCloudC::Ptr& cloud,
                                    const pcl::PointIndices::Ptr& indices,
                                    const size_t& iteration) {
  Update(0, id, cloud, indices, iteration);
}

void SurfaceHistoryRecorder::Update(const size_t& old_id, const size_t& new_id,
                                    const PointCloudC::Ptr& cloud,
                                    const pcl::PointIndices::Ptr& indices,
                                    const size_t& iteration) {
  if (new_id == 0) {
    ROS_INFO(
        "Warning: Update(...) doesn't update id to 0.");
    return;
  }

  PointCloudC::Ptr new_cloud(new PointCloudC);
  pcl::ExtractIndices<PointC> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(indices);
  extract_indices.filter(*new_cloud);

  std::map<size_t, PointCloudC::Ptr>::iterator iter =
      cloud_history_.find(old_id);
  if (iter != cloud_history_.end()) {
    *new_cloud += *(iter->second);
  }

  cloud_history_[new_id] = new_cloud;
  time_history_[new_id] = std::time(0);
  iteration_history_[new_id] = iteration;
}

void SurfaceHistoryRecorder::GetCloudHistory(
    const size_t& id, const PointCloudC::Ptr output_cloud) const {
  std::map<size_t, PointCloudC::Ptr>::const_iterator iter =
      cloud_history_.find(id);
  if (iter != cloud_history_.end()) {
    *output_cloud = *(iter->second);
  }
}

void SurfaceHistoryRecorder::GetTimeSpent(const size_t& id,
                                          time_t* time_ptr) const {
  std::map<size_t, time_t>::const_iterator iter = time_history_.find(id);
  if (iter != time_history_.end()) {
    *time_ptr = iter->second;
  }
}

void SurfaceHistoryRecorder::GetIteration(const size_t& id,
                                          size_t* iteration_ptr) const {
  std::map<size_t, size_t>::const_iterator iter = iteration_history_.find(id);
  if (iter != iteration_history_.end()) {
    *iteration_ptr = iter->second;
  }
}
}  // namespace surface_perception
