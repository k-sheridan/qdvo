/*
 * test_general.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: kevin
 */


#include <ros/ros.h>

#include "Params.h"
#include <Feature.h>
#include <ParseParams.h>

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(Feature, feature_cov_test)
{
	parseROSParams();

	DEFAULT_POINT_HOMOGENOUS_VARIANCE = 1; // force


	Eigen::Matrix<ScalarType, 3, 3> K;
	K.setIdentity();
	Sophus::SE3<ScalarType> unit_pose;
	Feature ft = Feature(cv::Point2f(0, 0), 2, 1, K, unit_pose);

	Eigen::Matrix<ScalarType, 3, 3> cov;
	cov << 4, 0, 0, 0, 4, 0, 0, 0, 1;

	ASSERT_LE((ft.getWorldUncertainty()-cov).norm(), 1e-8) << " uncertainty: " << ft.getWorldUncertainty();
}

// Declare another test
TEST(Feature, not_used)
{

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

