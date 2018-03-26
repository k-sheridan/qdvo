/*
 * test_general.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: kevin
 */


#include <ros/ros.h>

#include "Params.h"
#include "../include/invio/VIO.h"
#include <StateEstimator.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "general_test"); // initializes ros

	ros::NodeHandle nh;


	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);
	ros::param::param<double>("~default_point_depth_variance", DEFAULT_POINT_DEPTH_VARIANCE, D_DEFAULT_POINT_DEPTH_VARIANCE);
		ros::param::param<double>("~default_point_homogenous_variance", DEFAULT_POINT_HOMOGENOUS_VARIANCE, D_DEFAULT_POINT_HOMOGENOUS_VARIANCE);
	//parseROSParams();

	Eigen::MatrixXf A;

	A = Eigen::MatrixXf::Identity(2, 2);
	A(0, 1) = 2;
	A(1, 0) = 3;

	Eigen::MatrixXf B = A;

	B.conservativeResize(4, 4);

	ROS_ASSERT(A == (B.block<2, 2>(0, 0)));
	ROS_INFO_STREAM("A: \n" << A << "\n vs B: \n" << B);

	// test imu measurement function
	StateEstimator se;
	
	StateEstimator::State mu;
	mu.mean.setZero();
	mu.setLambda(1);
	mu.setOmega(Eigen::Matrix<ScalarType, 3, 1>(1, 2, 3));

	Eigen::Matrix<float, 3, 3> R;
	R.setIdentity();

	Eigen::Matrix<float, 3, 1> r;
	r << 0.1, 0, 0;

	Eigen::Matrix<float, 6, BASE_STATE_SIZE> H;
	Eigen::Matrix<float, 6, 1> z;

	se.imuMeasurementFromState(mu, H, z, R, r);

	ROS_INFO_STREAM("z: " << z.transpose());
	ROS_INFO_STREAM("H: " << H.transpose());


	return 0;
}

