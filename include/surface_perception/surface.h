#ifndef _SURFACE_PERCEPTION_SURFACE_H_
#define _SURFACE_PERCEPTION_SURFACE_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/ModelCoefficients.h"

namespace surface_perception {
struct Surface {
 public:
  geometry_msgs::PoseStamped pose_stamped;
  geometry_msgs::Vector3 dimensions;
  pcl::ModelCoefficients::Ptr coefficients;
};
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SURFACE_H_
