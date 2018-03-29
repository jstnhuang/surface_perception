#include "Eigen/Eigen"
#include "ros/ros.h"

#include "surface_perception/shape_extraction.h"

enum matrix_option {
  standard_axises,
  inverted_axises,
  random_rotation
};

bool GetMatrix(const matrix_option& option, bool dimension_inverted,
		Eigen::Matrix3f test_matrix,
		Eigen::Matrix3f expected_matrix) {
  if (option == matrix_option::standard_axises) {
    
  } else if (option == matrix_option::inverted_axises) {
  } else if (option == matrix_option::random_rotation) {
  } else {
    ROS_ERROR("Invalid option for GetMatrix function in rotation_matrix_test");
    return false;
  }

  if (dimension_inverted) {
    Eigen::Vector3f y_axis = expected_matrix.col(1);
    expected_matrix.col(1) = expected_matrix.col(0);
    expected_matrix.col(0) = y_axis;
    expected_matrix.col(2) = expected_matrix.col(0).cross(expected_matrix.col(1));
  }

  return true;
}

// This test the equality of two matrix through approximation.
bool AssertMatrix(const Eigen::Matrix3f& actual_matrix,
		const Eigen::Matrix3f& expected_matrix) {
  // Check matrix dimensions before testing content
  if (actual_matrix.cols() != expected_matrix.cols()
		  || actual_matrix.rows() != expected_matrix.rows()) {
    return false;
  }

  return (actual_matrix - expected_matrix).array().abs().matrix().sum() < 0.0001;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotation_matrix_test");

  return 0;
}
