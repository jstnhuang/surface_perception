#ifndef _SURFACE_PERCEPTION_SURFACE_H_
#define _SURFACE_PERCEPTION_SURFACE_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/ModelCoefficients.h"

namespace surface_perception {
/// \brief Represents a surface.
struct Surface {
 public:
  /// \brief The pose of the surface.
  ///
  /// The pose is located at the center of an oriented bounding box. The z
  /// direction points up and the x and y directions are aligned with the box.
  geometry_msgs::PoseStamped pose_stamped;

  /// \brief The dimensions of the oriented bounding box around the surface.
  geometry_msgs::Vector3 dimensions;

  /// \brief The coefficients representing the plane that models this surface.
  ///
  /// The first four coefficients, a, b, c, and d are such that ax+by+cz+d = 0.
  pcl::ModelCoefficients::Ptr coefficients;
};
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SURFACE_H_
