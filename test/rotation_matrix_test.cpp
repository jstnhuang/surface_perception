#include "Eigen/Eigen"
#include "surface_perception/shape_extraction.h"

#include "pcl/ModelCoefficients.h"

#include <gtest/gtest.h>

namespace surface_perception {
// Consant dimensions
const double kLongSide = 2.0;
const double kShortSide = 1.0;

TEST(TestStandardizeBoxOrientation, IdentityMatrix) {
  Eigen::Matrix3f expected_matrix = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f input_matrix = Eigen::Matrix3f::Identity();

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(
      input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.0001));
}

TEST(TestStandardizeBoxOrientation, IdentityMatrixRotate180DegreesAroundYAxis) {
  // clang-format off
  Eigen::Matrix3f expected_matrix;
  expected_matrix << 1.0, 0.0, 0.0,
		  0.0, -1.0, 0.0,
		  0.0, 0.0, -1.0;

  Eigen::Matrix3f input_matrix;
  input_matrix << -1.0, 0.0, 0.0,
	       0.0, 1.0, 0.0,
	       0.0, 0.0, -1.0;
  // clang-format on

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(
      input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.0001));
}

TEST(TestStandardizeBoxOrientation, IdentityMatrixRotate45DegreesAroundZAxis) {
  // clang-format off
  Eigen::Matrix3f expected_matrix;
  expected_matrix << 0.70711, -0.70711, 0.0,
		  0.70711, 0.70711, 0.0,
		  0.0, 0.0, 1.0;

  Eigen::Matrix3f input_matrix;
  input_matrix << 0.70711, -0.70711, 0.0,
	       0.70711, 0.70711, 0.0,
	       0.0, 0.0, 1.0;
  // clang-format on

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(
      input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.0001));
}

TEST(TestStandardizeBoxOrientation, IdentityMatrixRotate135DegreesAroundZAxis) {
  // clang-format off
  Eigen::Matrix3f expected_matrix;
  expected_matrix << 0.70711, -0.70711, 0.0,
		  0.70711, 0.70711, 0.0,
		  0.0, 0.0, 1.0;

  Eigen::Matrix3f input_matrix;
  input_matrix << -0.70711, 0.70711, 0.0,
	       -0.70711, -0.70711, 0.0,
	       0.0, 0.0, 1.0;
  // clang-format on

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(
      input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.0001));
}

TEST(TestStandardizeBoxOrientation, TiltedMatrix) {
  // clang-format off
  Eigen::Matrix3f expected_matrix;
  expected_matrix << 0.99980, 0.01732, 0.01,
		  -0.02, 0.86580, 0.5,
		  0.0, -0.5001, 0.86597;

  Eigen::Matrix3f input_matrix;
  input_matrix << 0.99980, 0.01732, 0.01,
	       -0.02, 0.86580, 0.5,
	       0.0, -0.5001, 0.86597;
  // clang-format on

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(
      input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.0001));
}

TEST(TestStandardizeBoxOrientation, TiltedMatrixRotate180DegreesAroundYAxis) {
  // clang-format off
  Eigen::Matrix3f expected_matrix;
  expected_matrix << 0.99980, -0.01732, -0.01,
		  -0.02, -0.86580, -0.5,
		  0.0, 0.5001, -0.86597;

  Eigen::Matrix3f input_matrix;
  input_matrix << -0.99980, 0.01732, -0.01,
	       0.02, 0.86580, -0.5,
	       0.0, -0.5001, -0.86597;
  // clang-format on

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(
      input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.0001));
}

TEST(TestStandardizeBoxOrientation, IdentityMatrixWithXLongSideYShortSdie) {
  // clang-format off
  Eigen::Matrix3f expected_matrix;
  expected_matrix << 0.0, -1.0, 0.0,
		  1.0, 0.0, 0.0,
		  0.0, 0.0, 1.0;

  Eigen::Matrix3f input_matrix;
  input_matrix << 1.0, 0.0, 0.0,
	       0.0, 1.0, 0.0,
	       0.0, 0.0, 1.0;
  // clang-format on

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(
      input_matrix, kLongSide, kShortSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.0001));
}

TEST(TestStandardizeBoxOrientation, TiltedMatrixWithXLongSideYShortSide) {
  // clang-format off
  Eigen::Matrix3f expected_matrix;
  expected_matrix << 0.01732, -0.99980, 0.01,
		  0.86580, 0.02, 0.5,
		  -0.5001, 0.0, 0.86597;

  Eigen::Matrix3f input_matrix;
  input_matrix << 0.99980, 0.01732, 0.01,
	       -0.02, 0.86580, 0.5,
	       0.0, -0.5001, 0.86597;
  // clang-format on

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(
      input_matrix, kLongSide, kShortSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.0001));
}
}  // namespace surface_perception

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
