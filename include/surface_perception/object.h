#ifndef _SURFACE_PERCEPTION_OBJECT_H_
#define _SURFACE_PERCEPTION_OBJECT_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace surface_perception {
/// \brief Object represents a segmented object on a surface.
struct Object {
 public:
  /// \brief The pose of the object
  ///
  /// The origin is at the center of the oriented bounding box around the
  /// object. The z direction points "up" relative to the surface, while the x
  /// and y directions are parallel to the sides of the box. There are no other
  /// constraints for which direction is x or y.
  geometry_msgs::PoseStamped pose_stamped;

  /// \brief The dimensions of the oriented bounding box around the object.
  ///
  /// The dimensions correspond to the x/y/z axis directions of the pose.
  geometry_msgs::Vector3 dimensions;

  /// \brief A pointer to the point cloud from which this object was segmented.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  /// \brief The indices of the point cloud that this object comprises.
  pcl::PointIndices::Ptr indices;
};
}  // namespace surface_perception
#endif  // _SURFACE_PERCEPTION_OBJECT_H_
