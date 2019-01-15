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
      EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z));
    }
  }
}

TEST(PlannerFunctions, generateNewHistogramSpecificCells) {
  // GIVEN: a which would fill specific histogram cells
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;
  double distance = 1.0;

  std::vector<double> e_angle_filled = {-90, -10, 0, 20, 40, 90};
  std::vector<double> z_angle_filled = {-180, -50, 0, 70, 100, 175};
  std::vector<geometry_msgs::Point> middle_of_cell;

  for (int i = 0; i < e_angle_filled.size(); i++) {
    for (int j = 0; j < z_angle_filled.size(); j++) {
      middle_of_cell.push_back(
          fromPolarToCartesian((int)e_angle_filled[i], (int)z_angle_filled[j],
                               distance, location.pose.position));
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < middle_of_cell.size(); i++) {
    for (int j = 0; j < 1; j++) {
      cloud.push_back(pcl::PointXYZ(middle_of_cell[i].x, middle_of_cell[i].y,
                                    middle_of_cell[i].z));
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
        EXPECT_DOUBLE_EQ(1.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      } else {
        EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      }
    }
  }
}

TEST(PlannerFunctions, propagateHistogramNoReprojectedPoints) {
  // GIVEN: we have no reprojected points (empty_cloud)
  Histogram propagated_histogram = Histogram(2 * ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;

  pcl::PointCloud<pcl::PointXYZ> empty_cloud;
  std::vector<double> reprojected_points_age;
  std::vector<double> reprojected_points_dist;

  // WHEN: we build a histogram from the reprojected points
  propagateHistogram(propagated_histogram, empty_cloud, reprojected_points_age,
                     reprojected_points_dist, location);

  // THEN: the histogram should be all zeros
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      EXPECT_DOUBLE_EQ(0.0, propagated_histogram.get_bin(e, z));
      EXPECT_DOUBLE_EQ(0.0, propagated_histogram.get_dist(e, z));
      EXPECT_DOUBLE_EQ(0.0, propagated_histogram.get_age(e, z));
    }
  }
}

TEST(PlannerFunctions, DISABLED_propagateHistogramEverywhereReprojectedPoints) {
  // GIVEN: we have a lot of reprojected points all at the same distance
  Histogram propagated_histogram = Histogram(2 * ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<double> reprojected_points_age;
  std::vector<double> reprojected_points_dist;

  double distance = 3.0;
  double age = 5.0;
  double old_dist = 2.0;

  for (int e = 0; e < 180; e++) {
    for (int z = 0; z < 360; z++) {
      for (int n = 0; n < 6; n++) {
        geometry_msgs::Point p =
            fromPolarToCartesian(e, z, distance, location.pose.position);
        cloud.push_back(pcl::PointXYZ(p.x, p.y, p.z));
        reprojected_points_age.push_back(age);
        reprojected_points_dist.push_back(old_dist);
      }
    }
  }

  // WHEN: we build a histogram from the reprojected points
  propagateHistogram(propagated_histogram, cloud, reprojected_points_age,
                     reprojected_points_dist, location);

  // THEN: the histogram should be all zeros
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      EXPECT_DOUBLE_EQ(1.0, propagated_histogram.get_bin(
                                e, z));  // we expect all cells to be filled
      EXPECT_DOUBLE_EQ(5.0, propagated_histogram.get_age(
                                e, z));  // we expect the age of each cell to be
                                         // the one of the reprojected points
      // EXPECT_DOUBLE_EQ(0.0, propagated_histogram.get_dist(e, z)); TODO: add a
      // test that the distance is set correctly
    }
  }
}
