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

/// \brief Modify the given rotation matrix to have the desired orientation.
///
/// The given rotation matrix should represents the orientation of the box:
///  --         --
///  |  |  |  |  |
///  |  |  |  |  |
///  |  x  y  z  |
///  |  |  |  |  |
///  |  |  |  |  |
///  --         --
/// Note: x, y, z in the figure above are basis vectors.
///
/// The good orientation is defined as the following:
///  1. If x dimension of the box must be smaller than or equal to y dimension.
///   If not, x basis vector and y basis vector should be swapped.
///  2. The angle between x-axis and the x basis vector of the rotation matrix
///   is less than or equal to 90 degrees.
///  3. The z basis vector is the same as the normal vector of the given plane
///   coefficients.
/// 
/// After checking the conditions above with corrections, The y basis vector is
/// then computed as the cross product of the x basis vector and z basis
/// vector.
///
/// \param[in] plane_coeff The coefficient of the plane where the box lies on.
/// \param[in] x_dimension The current x dimension of the box.
/// \param[in] y_dimension The current y dimension of the box.
/// \param[in] rotaton_matrix The given rotation matrix of the box.
/// \param[out] output_matrix The corrected rotation matrix of the box.
Eigen::Matrix3f StandardizeBoxOrientation(double x_dimension,
		double y_dimension,
		const Eigen::Matrix3f& rotation_matrix);
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SHAPE_EXTRACTION_
