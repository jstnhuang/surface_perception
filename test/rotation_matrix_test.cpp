#include "surface_perception/shape_extraction.h"
#include "Eigen/Eigen"

#include "pcl/ModelCoefficients.h"

#include <gtest/gtest.h>
namespace {
// This test the equality of two matrix through approximation.
bool isSameMatrix(const Eigen::Matrix3f& actual_matrix,
		const Eigen::Matrix3f& expected_matrix) {
  // Check matrix dimensions before testing content
  if (actual_matrix.cols() != expected_matrix.cols()
		  || actual_matrix.rows() != expected_matrix.rows()) {
    return false;
  }

  return (actual_matrix - expected_matrix).array().abs().matrix().sum() < 0.0001;
}
}  // Anonymous namespace

namespace surface_perception {

// Plane structure for constant declaration
struct plane_model {
  float x;
  float y;
  float z;
  float d;
};

// Constant flat plane
const plane_model flat_plane = {
  0.0,
  0.0,
  1.0,
  0.0
};

// Constant tilted plane
const plane_model tilted_plane = {
  0.01,
  0.5,
  0.86597,
  0.1
};

// Consant dimensions
const double long_side = 2.0;
const double short_side = 1.0;

TEST(TestMakeGoodBoxOrientation, standard_axises) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  model->values.resize(4);
  model->values[0] = flat_plane.x;
  model->values[1] = flat_plane.y;
  model->values[2] = flat_plane.z;
  model->values[3] = flat_plane.d;

  Eigen::Matrix3f expected_matrix;
  expected_matrix << 1.0, 0.0, 0.0,
		  0.0, 1.0, 0.0,
		  0.0, 0.0, 1.0;

  Eigen::Matrix3f input_matrix;
  input_matrix << 1.0, 0.0, 0.0,
	      0.0, 1.0, 0.0,
	      0.0, 0.0, 1.0;

  Eigen::Matrix3f actual_matrix;
  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);

  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

TEST(TestMakeGoodBoxOrientation, swapping_x_y_basis_vectors) {
}

TEST(TestMakeGoodBoxOrientation, inverted_axises) {
}

TEST(TestMakeGoodBoxOrientation, tilted_45degrees_xy_plane) {
}

TEST(TestMakeGoodBoxOrientation, tilted_135degrees_xy_plane) {
}

}  // surface_perception namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
