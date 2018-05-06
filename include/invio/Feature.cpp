/*
 * Feature.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "Feature.h"

Feature::Feature() {
}

Feature::Feature(cv::Point2f pt, ScalarType depth, ScalarType depth_variance, Eigen::Matrix<ScalarType, 3, 3> K, Sophus::SE3<ScalarType> observation_pose){

	this->px = pt;
	Eigen::Matrix<ScalarType, 2, 1> bearing = Feature::pixel2Metric(K, pt);

	ROS_ASSERT(depth > 0);

	//set the feature position
	this->mu(0) = depth*bearing(0);
	this->mu(1) = depth*bearing(1);
	this->mu(2) = depth;

	ROS_ASSERT(depth_variance >= 0);
	//set the uncertainty of the feature position in homogenous coordinates
	this->Sigma << DEFAULT_POINT_HOMOGENOUS_VARIANCE, 0, 0,
								0, DEFAULT_POINT_HOMOGENOUS_VARIANCE, 0,
								0, 0, depth_variance;


	//transform uncertainty to euclidean coordinates
	Eigen::Matrix<ScalarType, 3, 3> J;
	J << depth, 0, bearing(0),
			0, depth, bearing(1),
			0, 0, 1; // linearized map from homo+z to euclidean coord

	this->Sigma = J*this->Sigma*J.transpose();

	//transform the uncertainty to world coordinates
	Eigen::Matrix<ScalarType, 3, 3> R = observation_pose.rotationMatrix();
	this->Sigma = R*this->Sigma*R.transpose();

	ROS_DEBUG_STREAM("feature pos cov: " << this->Sigma);

	//transform this feature position to world coordinates
	this->mu = observation_pose * this->mu;


	R_inv << 1, 0, 0, 1; // just in case

}

Feature::~Feature() {
	// TODO Auto-generated destructor stub
}


Eigen::Matrix<ScalarType, 3, 1> Feature::projectFeature(Sophus::SE3<ScalarType> into_frame){
	// to project the feature in we need to compute the pose -> observation_pose transform

	// transform point into next frame
	Eigen::Matrix<ScalarType, 3, 1> new_point = into_frame.inverse() * this->mu;

	return new_point;
}


ScalarType Feature::occlusionSSD(cv::Mat test_patch){
	ROS_ASSERT(false);
	/*ROS_ASSERT(test_patch.rows == this->patches.front().rows && test_patch.cols == this->patches.front().cols);

	ROS_ASSERT(test_patch.rows != 0 && test_patch.cols != 0);

	ScalarType SSD = 0;

	//TODO add weights

	for(int i = 0; i < test_patch.rows; i++){
		for(int j = 0; j < test_patch.cols; j++){
			ScalarType error = test_patch.at<uchar>(i, j) - this->patches.front().at<uchar>(i, j);
			SSD += error*error;
		}
	}

	return SSD/(ScalarType)(test_patch.rows*test_patch.cols);*/
}
