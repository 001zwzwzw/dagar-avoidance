#include <gtest/gtest.h>

#include "../src/nodes/planner_functions.h"

using namespace avoidance;

TEST(PlannerFunctions, generateNewHistogramEmpty) {
  // GIVEN: an empty pointcloud
  pcl::PointCloud<pcl::PointXYZ> empty_cloud;
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, empty_cloud, location);

  // THEN: the histogram should be all zeros
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      EXPECT_LE(histogram_output.get_dist(e, z), 0.001);
    }
  }
}

TEST(PlannerFunctions, generateNewHistogramSpecificCells) {
  // GIVEN: a pointcloud with an object of one cell size
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;
  double distance = 1.0;

  std::vector<double> e_angle_filled = {-90, -30, 0, 20, 40, 90};
  std::vector<double> z_angle_filled = {-180, -50, 0, 59, 100, 175};
  std::vector<Eigen::Vector3f> middle_of_cell;

  for (int i = 0; i < e_angle_filled.size(); i++) {
    for (int j = 0; j < z_angle_filled.size(); j++) {
      middle_of_cell.push_back(fromPolarToCartesian(e_angle_filled[i],
                                                    z_angle_filled[j], distance,
                                                    location.pose.position));
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < middle_of_cell.size(); i++) {
    for (int j = 0; j < 1; j++) {
      cloud.push_back(toXYZ(middle_of_cell[i]));
    }
  }

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, cloud, location);

  // THEN: the filled cells in the histogram should be one and the others be
  // zeros

  std::vector<int> e_index;
  std::vector<int> z_index;
  for (int i = 0; i < e_angle_filled.size(); i++) {
    e_index.push_back(elevationAngletoIndex((int)e_angle_filled[i], ALPHA_RES));
    z_index.push_back(azimuthAngletoIndex((int)z_angle_filled[i], ALPHA_RES));
  }

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      bool e_found =
          std::find(e_index.begin(), e_index.end(), e) != e_index.end();
      bool z_found =
          std::find(z_index.begin(), z_index.end(), z) != z_index.end();
      if (e_found && z_found) {
        EXPECT_GE(histogram_output.get_dist(e, z), 0.0);
      } else {
        EXPECT_LE(histogram_output.get_dist(e, z), 0.001);
      }
    }
  }
}

TEST(PlannerFunctions, padPolarMatrixAzimuthWrapping) {
  // GIVEN: a matrix with known data. Where every cell has the value of its column index.
  // And by how many lines should be padded
  int n_lines_padding = 3;
  Eigen::MatrixXd matrix;
  matrix.resize(30, 60);
  matrix.fill(1.0);
  for(int c = 0; c < matrix.cols(); c++){
	  matrix.col(c) = c * matrix.col(c);
  }

  // WHEN: we pad the matrix
  Eigen::MatrixXd matrix_padded;
  padPolarMatrix(matrix, n_lines_padding, matrix_padded);

  // THEN: the output matrix should have the right size,
  // the middle part should be equal to the original matrix,
  // and the wrapping around the azimuth angle should be correct.

  ASSERT_EQ(30 + 2 * n_lines_padding, matrix_padded.rows());
  ASSERT_EQ(60 + 2 * n_lines_padding, matrix_padded.cols());

  bool middle_part_correct = matrix_padded.block(n_lines_padding, n_lines_padding, matrix.rows(), matrix.cols()) == matrix;
  bool col_0_correct = matrix_padded.col(0) == matrix_padded.col(60);
  bool col_1_correct = matrix_padded.col(1) == matrix_padded.col(61);
  bool col_2_correct = matrix_padded.col(2) == matrix_padded.col(62);
  bool col_63_correct = matrix_padded.col(63) == matrix_padded.col(3);
  bool col_64_correct = matrix_padded.col(64) == matrix_padded.col(4);
  bool col_65_correct = matrix_padded.col(65) == matrix_padded.col(5);

  EXPECT_TRUE(middle_part_correct);
  EXPECT_TRUE(col_0_correct);
  EXPECT_TRUE(col_1_correct);
  EXPECT_TRUE(col_2_correct);
  EXPECT_TRUE(col_63_correct);
  EXPECT_TRUE(col_64_correct);
  EXPECT_TRUE(col_65_correct);
}

TEST(PlannerFunctions, padPolarMatrixElevationWrapping) {
  // GIVEN: a matrix with known data. Where every cell has the value of its column index.
  // And by how many lines should be padded
  int n_lines_padding = 2;
  Eigen::MatrixXd matrix;
  matrix.resize(6, 10);
  matrix.fill(0.0);

  matrix(0, 0) = 1;
  matrix(0, 1) = 2;
  matrix(1, 0) = 3;
  matrix(0, 5) = 4;
  matrix(1, 6) = 5;
  matrix(0, 9) = 6;
  matrix(5, 0) = 7;
  matrix(4, 4) = 8;
  matrix(5, 9) = 9;

  // WHEN: we pad the matrix
  Eigen::MatrixXd matrix_padded;
  padPolarMatrix(matrix, n_lines_padding, matrix_padded);

  // THEN: the output matrix should have the right size,
  // the middle part should be equal to the original matrix,
  // and the wrapping around the elevation angle should be correct.

  ASSERT_EQ(6 + 2 * n_lines_padding, matrix_padded.rows());
  ASSERT_EQ(10 + 2 * n_lines_padding, matrix_padded.cols());

  bool middle_part_correct = matrix_padded.block(n_lines_padding, n_lines_padding, matrix.rows(), matrix.cols()) == matrix;
  bool val_1 = matrix_padded(1, 7) == 1;
  bool val_2 = matrix_padded(1, 8) == 2;
  bool val_3 = matrix_padded(0, 7) == 3;
  bool val_4 = matrix_padded(1, 2) == 4;
  bool val_5 = matrix_padded(0, 3) == 5;
  bool val_6 = matrix_padded(1, 6) == 6;
  bool val_7 = matrix_padded(7, 12) == 7;
  bool val_8 = matrix_padded(9, 11) == 8;
  bool val_9 = matrix_padded(8, 6) == 9;

  EXPECT_TRUE(middle_part_correct);
  EXPECT_TRUE(val_1);
  EXPECT_TRUE(val_2);
  EXPECT_TRUE(val_3);
  EXPECT_TRUE(val_4);
  EXPECT_TRUE(val_5);
  EXPECT_TRUE(val_6);
  EXPECT_TRUE(val_7);
  EXPECT_TRUE(val_8);
  EXPECT_TRUE(val_9);
}
