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
  matrix.resize(GRID_LENGTH_E, GRID_LENGTH_Z);
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

  ASSERT_EQ(GRID_LENGTH_E + 2 * n_lines_padding, matrix_padded.rows());
  ASSERT_EQ(GRID_LENGTH_Z + 2 * n_lines_padding, matrix_padded.cols());

  bool middle_part_correct = matrix_padded.block(n_lines_padding, n_lines_padding, matrix.rows(), matrix.cols()) == matrix;
  bool col_0_correct = matrix_padded.col(0) == matrix_padded.col(GRID_LENGTH_Z);
  bool col_1_correct = matrix_padded.col(1) == matrix_padded.col(GRID_LENGTH_Z + 1);
  bool col_2_correct = matrix_padded.col(2) == matrix_padded.col(GRID_LENGTH_Z + 2);
  bool col_63_correct = matrix_padded.col(GRID_LENGTH_Z + 3) == matrix_padded.col(3);
  bool col_64_correct = matrix_padded.col(GRID_LENGTH_Z + 4) == matrix_padded.col(4);
  bool col_65_correct = matrix_padded.col(GRID_LENGTH_Z + 5) == matrix_padded.col(5);

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
  matrix.resize(GRID_LENGTH_E, GRID_LENGTH_Z);
  matrix.fill(0.0);
  int half_z = GRID_LENGTH_Z/2;
  int last_z = GRID_LENGTH_Z - 1;
  int last_e = GRID_LENGTH_E - 1;

  matrix(0, 0) = 1;
  matrix(0, 1) = 2;
  matrix(1, 0) = 3;
  matrix(0, half_z) = 4;
  matrix(1, half_z + 1) = 5;
  matrix(0, last_z) = 6;
  matrix(last_e, 0) = 7;
  matrix(last_e - 1, half_z - 1) = 8;
  matrix(last_e, last_z) = 9;

  // WHEN: we pad the matrix
  Eigen::MatrixXd matrix_padded;
  padPolarMatrix(matrix, n_lines_padding, matrix_padded);

  // THEN: the output matrix should have the right size,
  // the middle part should be equal to the original matrix,
  // and the wrapping around the elevation angle should be correct.

  ASSERT_EQ(GRID_LENGTH_E + 2 * n_lines_padding, matrix_padded.rows());
  ASSERT_EQ(GRID_LENGTH_Z + 2 * n_lines_padding, matrix_padded.cols());


  bool middle_part_correct = matrix_padded.block(n_lines_padding, n_lines_padding, matrix.rows(), matrix.cols()) == matrix;
  bool val_1 = matrix_padded(1, half_z + n_lines_padding) == 1;
  bool val_2 = matrix_padded(1, half_z + n_lines_padding + 1) == 2;
  bool val_3 = matrix_padded(0, half_z + n_lines_padding) == 3;
  bool val_4 = matrix_padded(1, 2) == 4;
  bool val_5 = matrix_padded(0, 3) == 5;
  bool val_6 = matrix_padded(1, half_z - 1 + n_lines_padding) == 6;
  bool val_7 = matrix_padded(last_e + 1 + n_lines_padding, half_z + n_lines_padding) == 7;
  bool val_8 = matrix_padded(last_e + 2 + n_lines_padding, GRID_LENGTH_Z + n_lines_padding - 1) == 8;
  bool val_9 = matrix_padded(last_e + 1 + n_lines_padding, half_z - 1 + n_lines_padding) == 9;

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

TEST(PlannerFunctions, getBestCandidatesFromCostMatrix) {
  // GIVEN: a known cost matrix and the number of needed candidates
  int n_candidates = 4;
  std::vector<candidateDirection> candidate_vector;
  Eigen::MatrixXd matrix;
  matrix.resize(GRID_LENGTH_E, GRID_LENGTH_Z);
  matrix.fill(10);

  matrix(0, 2) = 1.1;
  matrix(0, 1) = 2.5;
  matrix(1, 2) = 3.8;
  matrix(1, 0) = 4.7;
  matrix(2, 2) = 4.9;

  // WHEN: calculate the candidates from the matrix
  getBestCandidatesFromCostMatrix(matrix, n_candidates, candidate_vector);

  // THEN: the output vector should have the right candidates in the right order

  ASSERT_EQ(n_candidates, candidate_vector.size());

  EXPECT_FLOAT_EQ(1.1, candidate_vector[0].cost);
  EXPECT_FLOAT_EQ(2.5, candidate_vector[1].cost);
  EXPECT_FLOAT_EQ(3.8, candidate_vector[2].cost);
  EXPECT_FLOAT_EQ(4.7, candidate_vector[3].cost);
}

TEST(PlannerFunctions, smoothPolarMatrix) {
  // GIVEN: a smoothing radius and a known cost matrix with one costly cell, otherwise all zeros.
  unsigned int smooth_radius = 2;
  Eigen::MatrixXd matrix;
  Eigen::MatrixXd matrix_old;
  matrix.resize(GRID_LENGTH_E, GRID_LENGTH_Z);
  matrix.fill(0);

  int r_object = GRID_LENGTH_E/2;
  int c_object = GRID_LENGTH_Z/2;
  matrix(r_object, c_object) = 100;

  // WHEN: we calculate the smoothed matrix
  matrix_old = matrix;
  smoothPolarMatrix(matrix, smooth_radius);

  // THEN: The smoothed matrix should be element-wise greater-equal than the input matrix
  // and the elements inside the smoothing radius around the costly cell should be greater than before
  for(int r = r_object - smooth_radius; r < r_object + smooth_radius; r ++){
	  for(int c = c_object - smooth_radius; c < c_object + smooth_radius; c ++){
		  if(!(r == r_object && c == c_object)){
			  EXPECT_GT(matrix(r, c), matrix_old(r, c));
		  }
	  }
  }
  bool greater_equal = (matrix.array() >= matrix_old.array()).all();
  EXPECT_TRUE(greater_equal);
}

TEST(PlannerFunctions, getCostMatrixNoObstacles) {
  // GIVEN: a position, goal and an empty histogram
  Eigen::Vector3f position(0, 0, 0);
  Eigen::Vector3f goal(0, 5, 0);
  Eigen::Vector3f last_sent_waypoint(0, 1, 0);
  costParameters cost_params;
  cost_params.goal_cost_param = 2.f;
  cost_params.smooth_cost_param = 1.5f;
  cost_params.height_change_cost_param = 4.f;
  cost_params.height_change_cost_param_adapted = 4.f;
  Eigen::MatrixXd cost_matrix;
  Histogram histogram = Histogram(ALPHA_RES);

  // WHEN: we calculate the cost matrix from the input data
  getCostMatrix(histogram, goal, position, last_sent_waypoint, cost_params,
                          false, cost_matrix);

  // THEN: The minimum cost should be in the direction of the goal
  float best_angle_e = elevationAnglefromCartesian(goal, position);
  float best_angle_z = azimuthAnglefromCartesian(goal, position);
  int best_index_e = elevationAngletoIndex(best_angle_e, ALPHA_RES);
  int best_index_z = azimuthAngletoIndex(best_angle_z, ALPHA_RES);

  Eigen::MatrixXd::Index minRow, minCol;
  float min = cost_matrix.minCoeff(&minRow, &minCol);

  EXPECT_NEAR((int)minRow, best_index_e, 1);
  EXPECT_NEAR((int)minCol, best_index_z, 1);

  // And the cost should grow as we go away from the goal index
  int check_radius = 3;
  Eigen::MatrixXd matrix_padded;
  //extend the matrix, as the goal direction might be at the border
  padPolarMatrix(cost_matrix, check_radius, matrix_padded);

  int best_index_padded_e = (int)minRow + check_radius;
  int best_index_padded_z = (int)minCol + check_radius;

  //check that rows farther away have bigger cost. Leave out direct neighbor rows as the center might be split over cells
  bool row1 = (matrix_padded.row(best_index_padded_e + 2).array() > matrix_padded.row(best_index_padded_e + 1).array()).all();
  bool row2 = (matrix_padded.row(best_index_padded_e + 3).array() > matrix_padded.row(best_index_padded_e + 2).array()).all();
  bool row3 = (matrix_padded.row(best_index_padded_e - 2).array() > matrix_padded.row(best_index_padded_e).array() - 1).all();
  bool row4 = (matrix_padded.row(best_index_padded_e - 3).array() > matrix_padded.row(best_index_padded_e).array() - 2).all();

  //check that columns farther away have bigger cost. Leave out direct neighbor rows as the center might be split over cells
  Eigen::MatrixXd matrix_padded2 = matrix_padded.block(3 , 0, cost_matrix.rows(), matrix_padded.cols());  //cut off padded top part
  bool col1 = (matrix_padded2.col(best_index_padded_z + 2).array() > matrix_padded2.col(best_index_padded_z + 1).array()).all();
  bool col2 = (matrix_padded2.col(best_index_padded_z + 3).array() > matrix_padded2.col(best_index_padded_z + 2).array()).all();
  bool col3 = (matrix_padded2.col(best_index_padded_z - 2).array() > matrix_padded2.col(best_index_padded_z).array() - 1).all();
  bool col4 = (matrix_padded2.col(best_index_padded_z - 3).array() > matrix_padded2.col(best_index_padded_z).array() - 2).all();

  EXPECT_TRUE(col1);
  EXPECT_TRUE(col2);
  EXPECT_TRUE(col3);
  EXPECT_TRUE(col4);
  EXPECT_TRUE(row1);
  EXPECT_TRUE(row2);
  EXPECT_TRUE(row3);
  EXPECT_TRUE(row4);
}
