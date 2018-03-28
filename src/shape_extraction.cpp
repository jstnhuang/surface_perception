#include "surface_perception/shape_extraction.h"

#include <limits.h>

#include "Eigen/Eigen"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/surface/convex_hull.h"
#include "ros/ros.h"

#include "pcl/features/normal_3d.h"
#include "surface_perception/typedefs.h"

namespace {
/**
 * This helper function checks if the input object can be projected onto the xy
 * plane in the ConvexHull algorithm.
 */
bool hasHullOnXYPlane(const pcl::ModelCoefficients::Ptr& model,
                      const PointCloudC::Ptr& flat_projected) {
  Eigen::Vector3d x_axis_(1.0, 0.0, 0.0);
  Eigen::Vector3d y_axis_(0.0, 1.0, 0.0);
  Eigen::Vector3d z_axis_(0.0, 0.0, 1.0);

  std::vector<int>* indices_ = new std::vector<int>();
  for (size_t i = 0; i < flat_projected->points.size(); i++) {
    indices_->push_back(i);
  }
  double projection_angle_thresh_ = cos(0.174532925);
  int dimension = 2;
  bool xy_proj_safe = true;
  bool yz_proj_safe = true;
  bool xz_proj_safe = true;

  // Check the input's normal to see which projection to use
  PointC p0 = flat_projected->points[(*indices_)[0]];
  PointC p1 = flat_projected->points[(*indices_)[indices_->size() - 1]];
  PointC p2 = flat_projected->points[(*indices_)[indices_->size() / 2]];

  Eigen::Array4f dy1dy2 = (p1.getArray4fMap() - p0.getArray4fMap()) /
                          (p2.getArray4fMap() - p0.getArray4fMap());
  while (!((dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]))) {
    p0 = flat_projected->points[(*indices_)[rand() % indices_->size()]];
    p1 = flat_projected->points[(*indices_)[rand() % indices_->size()]];
    p2 = flat_projected->points[(*indices_)[rand() % indices_->size()]];
    dy1dy2 = (p1.getArray4fMap() - p0.getArray4fMap()) /
             (p2.getArray4fMap() - p0.getArray4fMap());
  }

  pcl::PointCloud<PointC> normal_calc_cloud;
  normal_calc_cloud.points.resize(3);
  normal_calc_cloud.points[0] = p0;
  normal_calc_cloud.points[1] = p1;
  normal_calc_cloud.points[2] = p2;

  Eigen::Vector4d normal_calc_centroid;
  Eigen::Matrix3d normal_calc_covariance;
  pcl::computeMeanAndCovarianceMatrix(normal_calc_cloud, normal_calc_covariance,
                                      normal_calc_centroid);
  // Need to set -1 here. See eigen33 for explanations.
  Eigen::Vector3d::Scalar eigen_value;
  Eigen::Vector3d plane_params;
  pcl::eigen33(normal_calc_covariance, eigen_value, plane_params);
  float theta_x = fabsf(static_cast<float>(plane_params.dot(x_axis_)));
  float theta_y = fabsf(static_cast<float>(plane_params.dot(y_axis_)));
  float theta_z = fabsf(static_cast<float>(plane_params.dot(z_axis_)));

  std::vector<PointC> points;
  points.push_back(p0);
  points.push_back(p1);
  points.push_back(p2);

  // Check for degenerate cases of each projection
  // We must avoid projections in which the plane projects as a line
  if (theta_z > projection_angle_thresh_) {
    xz_proj_safe = false;
    yz_proj_safe = false;
  }
  if (theta_x > projection_angle_thresh_) {
    xz_proj_safe = false;
    xy_proj_safe = false;
  }
  if (theta_y > projection_angle_thresh_) {
    xy_proj_safe = false;
    yz_proj_safe = false;
  }

  if (!xy_proj_safe) {
    ROS_WARN(
        "Warning: could not use the following three points to calculate convex "
        "hull!");
    ROS_WARN("P0 (%f, %f, %f)", p0.x, p0.y, p0.z);
    ROS_WARN("P1 (%f, %f, %f)", p1.x, p1.y, p1.z);
    ROS_WARN("P2 (%f, %f, %f)", p2.x, p2.y, p2.z);

    return false;
  }

  return true;
}
}  // namespace

namespace surface_perception {
bool FitBox(const PointCloudC::Ptr& input,
            const pcl::PointIndices::Ptr& indices,
            const pcl::ModelCoefficients::Ptr& model, geometry_msgs::Pose* pose,
            geometry_msgs::Vector3* dimensions) {
  double min_volume = std::numeric_limits<double>::max();  // the minimum volume
                                                           // shape found thus
                                                           // far.
  Eigen::Matrix3f transformation;  // the transformation for the best-fit shape

  // Compute z height as maximum distance from planes
  double height = 0.0;
  for (size_t i = 0; i < indices->indices.size(); ++i) {
    int index = indices->indices[i];
    const PointC& pt = input->at(index);
    const Eigen::Vector4f pp(pt.x, pt.y, pt.z, 1);
    Eigen::Vector4f m(model->values[0], model->values[1], model->values[2],
                      model->values[3]);
    double distance_to_plane = fabs(pp.dot(m));
    if (distance_to_plane > height) {
      height = distance_to_plane;
    }
  }

  // Project object into 2d, using plane model coefficients
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr flat(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> projection;
  projection.setModelType(pcl::SACMODEL_PLANE);
  projection.setInputCloud(input);
  if (indices && indices->indices.size() > 0) {
    projection.setIndices(indices);
  }
  projection.setModelCoefficients(model);
  projection.filter(*flat);

  // Rotate plane so that Z=0
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr flat_projected(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Vector3f normal(model->values[0], model->values[1], model->values[2]);
  Eigen::Quaternionf qz;
  qz.setFromTwoVectors(normal, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f plane_rotation = qz.toRotationMatrix();
  Eigen::Matrix3f inv_plane_rotation = plane_rotation.inverse();

  for (size_t i = 0; i < flat->size(); ++i) {
    pcl::PointXYZRGB p;
    p.getVector3fMap() = plane_rotation * (*flat)[i].getVector3fMap();
    flat_projected->push_back(p);
  }

  // Find the convex hull
  pcl::PointCloud<pcl::PointXYZRGB> hull;
  pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;

  // Check if ConvexHull works for the given object
  if (hasHullOnXYPlane(model, flat_projected)) {
    convex_hull.setInputCloud(flat_projected);
    convex_hull.setDimension(2);
    convex_hull.reconstruct(hull);
  } else {
    return false;
  }

  double last_x_min, last_x_max, last_y_min, last_y_max;
  // Try fitting a rectangle
  for (size_t i = 0; i < hull.size() - 1; ++i) {
    // For each pair of hull points, determine the angle
    double rise = hull[i + 1].y - hull[i].y;
    double run = hull[i + 1].x - hull[i].x;
    // and normalize..
    {
      double l = sqrt((rise * rise) + (run * run));
      rise = rise / l;
      run = run / l;
    }

    // Build rotation matrix from change of basis
    Eigen::Matrix3f rotation;
    rotation(0, 0) = run;
    rotation(0, 1) = rise;
    rotation(0, 2) = 0.0;
    rotation(1, 0) = -rise;
    rotation(1, 1) = run;
    rotation(1, 2) = 0.0;
    rotation(2, 0) = 0.0;
    rotation(2, 1) = 0.0;
    rotation(2, 2) = 1.0;
    Eigen::Matrix3f inv_rotation = rotation.inverse();

    // Project hull to new coordinate system
    pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
    for (size_t j = 0; j < hull.size(); ++j) {
      pcl::PointXYZRGB p;
      p.getVector3fMap() = rotation * hull[j].getVector3fMap();
      projected_cloud.push_back(p);
    }

    // Compute min/max
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    for (size_t j = 0; j < projected_cloud.size(); ++j) {
      if (projected_cloud[j].x < x_min) x_min = projected_cloud[j].x;
      if (projected_cloud[j].x > x_max) x_max = projected_cloud[j].x;

      if (projected_cloud[j].y < y_min) y_min = projected_cloud[j].y;
      if (projected_cloud[j].y > y_max) y_max = projected_cloud[j].y;
    }

    // Is this the best estimate?
    double area = (x_max - x_min) * (y_max - y_min);
    if (area * height < min_volume) {
      transformation = inv_plane_rotation * inv_rotation;

      Eigen::Vector3f pose3f((x_max + x_min) / 2.0, (y_max + y_min) / 2.0,
                             projected_cloud[0].z + height / 2.0);
      pose3f = transformation * pose3f;
      pose->position.x = pose3f(0);
      pose->position.y = pose3f(1);
      pose->position.z = pose3f(2);

      last_x_max = x_max;
      last_x_min = x_min;
      last_y_max = y_max;
      last_y_min = y_min;

      min_volume = area * height;
    }
  }

  // Flip orientation if necessary to force x dimension < y dimension
  double x_dim = last_x_max - last_x_min;
  double y_dim = last_y_max - last_y_min;
  if (x_dim > y_dim) {
    Eigen::Vector3f y_axis = transformation.col(1);
    // There are two choices for the new x axis. This chooses the one that
    // is closer to the positive x direction of the data.
    if (y_axis.x() < 0) {
      y_axis = -1 * transformation.col(1);
    }
    transformation.col(0) = y_axis;
    transformation.col(1) =
        transformation.col(2).cross(transformation.col(0));
  }

  if (x_dim > y_dim) {
    dimensions->x = (last_y_max - last_y_min);
    dimensions->y = (last_x_max - last_x_min);
  } else {
    dimensions->x = (last_x_max - last_x_min);
    dimensions->y = (last_y_max - last_y_min);
  }
  dimensions->z = height;

  std::stringstream ss;
  ss << transformation;
  ROS_INFO("Getting transformation of %s", ss.str().c_str());

  Eigen::Quaternionf q(transformation);
  pose->orientation.x = q.x();
  pose->orientation.y = q.y();
  pose->orientation.z = q.z();
  pose->orientation.w = q.w();

  return true;
}

void MakeGoodBoxOrientation(const pcl::ModelCoefficients::Ptr& plane_coeff,
		const Eigen::Matrix3f& rotation_matrix,
		Eigen::Matrix3f* output_matrix) {
  // Compute the expected vectors of box
  Eigen::Vector3f x_axis(1.0, 0.0, 0.0);
  Eigen::Vector3f z_axis(plane_coeff->values[0], plane_coeff->values[1],
		  plane_coeff->values[2]);
  Eigen::Vector3f y_axis = z_axis.cross(x_axis);

  // Check if the object face towards the positive x-axis
  if (rotation_matrix.col(0).dot(x_axis) <= 0.0) {
    ROS_WARN("The box doesn't face towards the positive x-axis. It has x-basis (%f, %f, %f) with dot product result of %f",
		    rotation_matrix.col(0)(0),
		    rotation_matrix.col(0)(1),
		    rotation_matrix.col(0)(2),
		    rotation_matrix.col(0).dot(x_axis));
    output_matrix->col(0) = rotation_matrix.col(0) * -1.0;
  } 
  // Check if the object has the same z-axis as the normal vector of the plane.
  if ((rotation_matrix.col(2) - z_axis).array().abs().matrix().sum() > 0.0001) {
    ROS_WARN("The box has the wrong z-axis (%f, %f, %f) compared to expected (%f, %f, %f)",
		    rotation_matrix.col(2)(0),
		    rotation_matrix.col(2)(1),
		    rotation_matrix.col(2)(2),
		    z_axis(0),
		    z_axis(1),
		    z_axis(2));
    output_matrix->col(2) = z_axis;
  }

  // Compute the y-basis based on the
  return res;
}
}  // namespace surface_perception
