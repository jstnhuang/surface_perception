#ifndef _SURFACE_PERCEPTION_OBJECT_H_
#define _SURFACE_PERCEPTION_OBJECT_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace surface_perception {
struct Object {
 public:
  geometry_msgs::PoseStamped pose_stamped;
  geometry_msgs::Vector3 dimensions;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointIndices::Ptr indices;
};
}  // namespace surface_perception
#endif  // _SURFACE_PERCEPTION_OBJECT_H_
