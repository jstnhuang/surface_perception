#include "surface_perception/shape_extraction.h"
#include "Eigen/Eigen"

#include "pcl/ModelCoefficients.h"

#include <gtest/gtest.h>
namespace {
void getHorizontalPlane(pcl::ModelCoefficients::Ptr model) {
  model->values.resize(4);
  model->values[0] = 0.0;
  model->values[1] = 0.0;
  model->values[2] = 1.0;
  model->values[3] = 2.0;
}

void getTiltedPlane(pcl::ModelCoefficients::Ptr model) {
  model->values.resize(4);
  model->values[0] = 0.01;
  model->values[1] = 0.5;
  model->values[2] = 0.86597;
  model->values[3] = 0.1;
}
}  // Anonymous namespace

namespace surface_perception {
// Consant dimensions
const double kLongSide = 2.0;
const double kShortSide = 1.0;

TEST(TestStandardizeBoxOrientation, standard_axises_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(1.0, 0.0, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix = Eigen::Matrix3f::Identity();

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.000001));
}

TEST(TestStandardizeBoxOrientation, standard_axises_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(1.0, 0.0, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix = Eigen::Matrix3f::Identity();

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.00001));
}


TEST(TestStandardizeBoxOrientation, inverted_axises_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(1.0, 0.0, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix = Eigen::Matrix3f::Identity() * -1.0;

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.00001));
}

TEST(TestStandardizeBoxOrientation, inverted_axises_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(1.0, 0.0, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix = Eigen::Matrix3f::Identity() * -1.0;

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.00001));
}

TEST(TestStandardizeBoxOrientation, swapping_x_y_basis_vectors_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(0.0, 1.0, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix = Eigen::Matrix3f::Identity();

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kLongSide, kShortSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.00001));
}

TEST(TestStandardizeBoxOrientation, swapping_x_y_basis_vectors_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(0.0, 1.0, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix = Eigen::Matrix3f::Identity();

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kLongSide, kShortSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.00001));
}

TEST(TestStandardizeBoxOrientation, tilted_45degrees_xy_direction_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(0.70711, 0.70711, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix;
  input_matrix << 0.70711, -0.70711, 0.0,
	      0.70711, 0.70711, 0.0,
	      0.0, 0.0, 1.0;

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.00001));
}

TEST(TestStandardizeBoxOrientation, tilted_45degrees_xy_direction_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(0.70711, 0.70711, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix;
  input_matrix << 0.70711, -0.70711, 0.0,
	      0.70711, 0.70711, 0.0,
	      0.0, 0.0, 1.0;

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.00001));
}

TEST(TestStandardizeBoxOrientation, tilted_135degrees_xy_direction_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(0.70711, 0.70711, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix;
  input_matrix << -0.70711, 0.70711, 0.0,
	      -0.70711, 0.-70711, 0.0,
	      0.0, 0.0, 1.0;

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.00001));
}

TEST(TestStandardizeBoxOrientation, tilted_135degrees_xy_direction_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(0.70711, 0.70711, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix;
  input_matrix << -0.70711, 0.70711, 0.0,
	      -0.70711, 0.-70711, 0.0,
	      0.0, 0.0, 1.0;

  double x_dim, y_dim;
  Eigen::Matrix3f actual_matrix = StandardizeBoxOrientation(input_matrix, kShortSide, kLongSide, &x_dim, &y_dim);

  ASSERT_TRUE(expected_matrix.isApprox(actual_matrix, 0.00001));
}
}  // surface_perception namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
