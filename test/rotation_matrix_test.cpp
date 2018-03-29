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
const double long_side = 2.0;
const double short_side = 1.0;

TEST(TestMakeGoodBoxOrientation, standard_axises_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(1.0, 0.0, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix;
  input_matrix << 1.0, 0.0, 0.0,
	      0.0, 1.0, 0.0,
	      0.0, 0.0, 1.0;

  Eigen::Matrix3f actual_matrix;
  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);

  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

TEST(TestMakeGoodBoxOrientation, standard_axises_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(1.0, 0.0, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix;
  input_matrix << 1.0, 0.0, 0.0,
	      0.0, 1.0, 0.0,
	      0.0, 0.0, 1.0;

  Eigen::Matrix3f actual_matrix;
  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);

  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}


TEST(TestMakeGoodBoxOrientation, inverted_axises_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  Eigen::Matrix3f expected_matrix;
  expected_matrix.col(0) = Eigen::Vector3f(1.0, 0.0, 0.0);
  expected_matrix.col(2) = Eigen::Vector3f(model->values[0],
		  model->values[1],
		  model->values[2]);
  expected_matrix.col(1) = expected_matrix.col(2).cross(expected_matrix.col(0));

  Eigen::Matrix3f input_matrix;
  input_matrix << -1.0, 0.0, 0.0,
	      0.0, -1.0, 0.0,
	      0.0, 0.0, -1.0;
  Eigen::Matrix3f actual_matrix;
  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);

  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

TEST(TestMakeGoodBoxOrientation, inverted_axises_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  // TODO:: fill input/expexted

//  Eigen::Matrix3f actual_matrix;
//  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);
//
//  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

TEST(TestMakeGoodBoxOrientation, swapping_x_y_basis_vectors_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  // TODO:: fill input/expexted

//  Eigen::Matrix3f actual_matrix;
//  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);
//
//  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

TEST(TestMakeGoodBoxOrientation, swapping_x_y_basis_vectors_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  // TODO:: fill input/expexted

//  Eigen::Matrix3f actual_matrix;
//  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);
//
//  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

TEST(TestMakeGoodBoxOrientation, tilted_45degrees_xy_direction_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  // TODO:: fill input/expexted

//  Eigen::Matrix3f actual_matrix;
//  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);
//
//  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

TEST(TestMakeGoodBoxOrientation, tilted_45degrees_xy_direction_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  // TODO:: fill input/expexted

//  Eigen::Matrix3f actual_matrix;
//  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);
//
//  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

TEST(TestMakeGoodBoxOrientation, tilted_135degrees_xy_direction_horizontal_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getHorizontalPlane(model);

  // TODO:: fill input/expexted

//  Eigen::Matrix3f actual_matrix;
//  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);
//
//  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

TEST(TestMakeGoodBoxOrientation, tilted_135degrees_xy_direction_tilted_plane) {
  pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
  getTiltedPlane(model);

  // TODO:: fill input/expexted

//  Eigen::Matrix3f actual_matrix;
//  MakeGoodBoxOrientation(model, short_side, long_side, input_matrix, &actual_matrix);
//
//  ASSERT_TRUE(isSameMatrix(actual_matrix, expected_matrix));
}

}  // surface_perception namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
