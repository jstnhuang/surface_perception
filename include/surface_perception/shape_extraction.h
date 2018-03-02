#ifndef _SURFACE_PERCEPTION_SHAPE_EXTRACTION_H_
#define _SURFACE_PERCEPTION_SHAPE_EXTRACTION_H_

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace surface_perception {
/// \brief Fits an oriented bounding box around a given point cloud representing
///   an object or a surface.
///
/// Note: this algorithm is adapted from the <a
/// href="http://wiki.ros.org/simple_grasping">simple_grasping</a> package.
///
/// \param[in] input The input point cloud to fit the box around.
/// \param[in] indices The indices in the input point cloud to use.
/// \param[in] model The model coefficients for the plane that the object is
///   resting on. If fitting a box around a surface, use the surface's own
///   coefficients.
/// \param[out] pose The pose representing the center of the box. The z
///   direction points "up" relative to the surface. The x and y directions are
///   aligned with the fitted box, with the x direction pointing toward the
///   shorter side of the box.
/// \param[out] dimensions The dimensions of the oriented bounding box. x, y,
///   and z correspond to the directions of the pose.
///
/// \returns reports true when a bounding box can be constructed for the object,
///   or false if the construction fails.
bool FitBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input,
            const pcl::PointIndicesPtr& indices,
            const pcl::ModelCoefficients::Ptr& model, geometry_msgs::Pose* pose,
            geometry_msgs::Vector3* dimensions);
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SHAPE_EXTRACTION_
