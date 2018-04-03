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

/// \brief Returns a standardized orientation for a box.
///
/// The standardized box orientation is defined as the following:
///  1. If x dimension of the box must be smaller than or equal to y dimension.
///   If not, x basis vector and y basis vector should be swapped.
///  2. The angle between x-axis and the x basis vector of the rotation matrix
///   is less than or equal to 90 degrees.
///  3. The z basis vector is the same as the normal vector of the plane where
///   the box will be constructed.
///
/// \param[in] rotaton_matrix The given rotation matrix of the box on a plane.
///  The z basis vector is assumed to be the same as the plane normal vector.
/// \param[in] x_dimension The current x dimension of the box.
/// \param[in] y_dimension The current y dimension of the box.
/// \param[out] updated_x_dim If x basis vector and y basis vector are swapped,
///  updated_x_dim will points to the value of y_dim. Otherwise, it points to
///  value of x_dim.
/// \param[out] updated_y_dim If y basis vector and x basis vector are swapped,
///  updated_y_dim will points to the value of x_dim. Otherwise, it points to
///  the value of y_dim.
///
/// \return A rotation matrix with the standardized box orientation is returned,
///  based on the given rotation matrix and dimensions.
Eigen::Matrix3f StandardizeBoxOrientation(
		const Eigen::Matrix3f& rotation_matrix,
		double x_dim,
		double y_dim,
		double* updated_x_dim,
		double* updated_y_dim);
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SHAPE_EXTRACTION_
