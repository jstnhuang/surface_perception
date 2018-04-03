#include "surface_perception/axes_marker.h"

#include "Eigen/Eigen"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace surface_perception {
visualization_msgs::Marker GetAxesMarker(const std::string& name_space,
		geometry_msgs::Pose pose, double scale) {
  visualization_msgs::Marker output_marker;
  output_marker.type = visualization_msgs::Marker::LINE_LIST;
  output_marker.ns = name_space;
  output_marker.scale.x = scale * 0.1;
  // The minimum line width is 1 centimeter
  if (output_marker.scale.x < 0.01) {
    output_marker.scale.x = 0.01;
  }
  output_marker.pose = pose;

  // Set the origin
  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;

  // Points one three axes with unit distances
  geometry_msgs::Point x_point;
  x_point.x = scale;
  x_point.y = 0.0;
  x_point.z = 0.0;

  geometry_msgs::Point y_point;
  y_point.x = 0.0;
  y_point.y = scale;
  y_point.z = 0.0;

  geometry_msgs::Point z_point;
  z_point.x = 0.0;
  z_point.y = 0.0;
  z_point.z = scale;

  // Axes color: red, green and blue
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0.0;
  red.b = 0.0;
  red.a = 1.0;

  std_msgs::ColorRGBA green;
  green.r = 0.0;
  green.g = 1.0;
  green.b = 0.0;
  green.a = 1.0;

  std_msgs::ColorRGBA blue;
  blue.r = 0.0;
  blue.g = 0.0;
  blue.b = 1.0;
  blue.a = 1.0;

  // Blue x-axis
  output_marker.points.push_back(origin);
  output_marker.points.push_back(x_point);
  output_marker.colors.push_back(red);
  output_marker.colors.push_back(red);

  // Green y-axis
  output_marker.points.push_back(origin);
  output_marker.points.push_back(y_point);
  output_marker.colors.push_back(green);
  output_marker.colors.push_back(green);

  // Blue z-axis
  output_marker.points.push_back(origin);
  output_marker.points.push_back(z_point);
  output_marker.colors.push_back(blue);
  output_marker.colors.push_back(blue);

  return output_marker;
}

visualization_msgs::MarkerArray GetAxesMarkerArray(const std::string& name_space,
		const std::string& frame_id,
		geometry_msgs::Pose pose, double scale) {

  visualization_msgs::MarkerArray res;
  Eigen::Matrix3f rotation_matrix = Eigen::Quaternionf(pose.orientation.w,
		  pose.orientation.x,
		  pose.orientation.y,
		  pose.orientation.z).toRotationMatrix();

  // X axis setup
  visualization_msgs::Marker x_axis;
  x_axis.ns = name_space;
  x_axis.header.frame_id = frame_id;
  x_axis.id = 0;
  x_axis.type = visualization_msgs::Marker::CYLINDER;
  x_axis.scale.x = x_axis.scale.y = std::max(scale * 0.1, 0.01);
  x_axis.scale.z = scale;

  // X axis orientation
  Eigen::Matrix3f x_matrix;
  x_matrix.col(0) = rotation_matrix.col(2) * -1.0;
  x_matrix.col(2) = rotation_matrix.col(0);
  x_matrix.col(1) = rotation_matrix.col(1);
  Eigen::Quaternionf x_quaternion(x_matrix);
  x_axis.pose.orientation.x = x_quaternion.x();
  x_axis.pose.orientation.y = x_quaternion.y();
  x_axis.pose.orientation.z = x_quaternion.z();
  x_axis.pose.orientation.w = x_quaternion.w();

  // Shift x axis position a bit to relocate the cylinder
  Eigen::Vector3f x_position = Eigen::Vector3f(pose.position.x,
		  pose.position.y, pose.position.z)
	  + x_matrix.col(2) * scale / 2.0;
  x_axis.pose.position.x = x_position(0);
  x_axis.pose.position.y = x_position(1);
  x_axis.pose.position.z = x_position(2);

  // X axis color
  x_axis.color.r = 1.0;
  x_axis.color.g = 0.0;
  x_axis.color.b = 0.0;
  x_axis.color.a = 1.0;

  // Y axis setup
  visualization_msgs::Marker y_axis;
  y_axis.ns = name_space;
  y_axis.header.frame_id = frame_id;
  y_axis.id = 1;
  y_axis.type = visualization_msgs::Marker::CYLINDER;
  y_axis.scale.x = y_axis.scale.y = std::max(scale * 0.1, 0.01);
  y_axis.scale.z = scale;

  // Y axis orientation
  Eigen::Matrix3f y_matrix;
  y_matrix.col(0) = rotation_matrix.col(0);
  y_matrix.col(2) = rotation_matrix.col(1);
  y_matrix.col(1) = rotation_matrix.col(2) * -1.0;
  Eigen::Quaternionf y_quaternion(y_matrix);
  y_axis.pose.orientation.x = y_quaternion.x();
  y_axis.pose.orientation.y = y_quaternion.y();
  y_axis.pose.orientation.z = y_quaternion.z();
  y_axis.pose.orientation.w = y_quaternion.w();

  // Y axis position shifting
  Eigen::Vector3f y_position = Eigen::Vector3f(pose.position.x,
		  pose.position.y, pose.position.z)
	  + y_matrix.col(2) * scale / 2.0;
  y_axis.pose.position.x = y_position(0);
  y_axis.pose.position.y = y_position(1);
  y_axis.pose.position.z = y_position(2);

  // Y axis color
  y_axis.color.r = 0.0;
  y_axis.color.g = 1.0;
  y_axis.color.b = 0.0;
  y_axis.color.a = 1.0;

  // Z axis setup
  visualization_msgs::Marker z_axis;
  z_axis.ns = name_space;
  z_axis.header.frame_id = frame_id;
  z_axis.id = 2;
  z_axis.type = visualization_msgs::Marker::CYLINDER;
  z_axis.scale.x = z_axis.scale.y = std::max(scale * 0.1, 0.01);
  z_axis.scale.z = scale;

  // Z axis orientation
  z_axis.pose.position = pose.position;
  Eigen::Vector3f z_position = Eigen::Vector3f(pose.position.x,
		  pose.position.y, pose.position.z)
	  + rotation_matrix.col(2) * scale / 2.0;
  z_axis.pose.position.x = z_position(0);
  z_axis.pose.position.y = z_position(1);
  z_axis.pose.position.z = z_position(2);

  // Z axis color
  z_axis.color.r = 0.0;
  z_axis.color.g = 0.0;
  z_axis.color.b = 1.0;
  z_axis.color.a = 1.0;

  res.markers.push_back(x_axis);
  res.markers.push_back(y_axis);
  res.markers.push_back(z_axis);

  return res;
}
}  // namespace surface_perception
