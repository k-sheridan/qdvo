/*
 * Feature.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "Feature.h"

Feature::Feature() {
}

Feature::Feature(cv::Point2f pt, ScalarType depth, ScalarType depth_variance, Eigen::Matrix<ScalarType, 3, 3> K, Sophus::SE3<ScalarType> observation_pose, cv::Mat patch){

	this->px = pt;
	Eigen::Matrix<ScalarType, 2, 1> bearing = Feature::pixel2Metric(K, pt);

	ROS_ASSERT(depth > 0);

	//set the feature position
	this->mu(0) = depth*bearing(0);
	this->mu(1) = depth*bearing(1);
	this->mu(2) = depth;

	ROS_ASSERT(depth_variance >= 0);
	//set the uncertainty of the feature position
	this->Sigma << DEFAULT_POINT_HOMOGENOUS_VARIANCE, 0, 0,
								0, DEFAULT_POINT_HOMOGENOUS_VARIANCE, 0,
								0, 0, depth_variance;

	//ROS_DEBUG_STREAM("feature mu initially: " << this->mu.transpose());
	this->observation_pose = observation_pose;
	this->patches.push_back(patch);

	R_inv << 1000, 0, 0, 1000; // just in case

}

Feature::~Feature() {
	// TODO Auto-generated destructor stub
}


Eigen::Matrix<ScalarType, 3, 1> Feature::projectFeature(Sophus::SE3<ScalarType> into_frame){
	// to project the feature in we need to compute the pose -> observation_pose transform


	Sophus::SE3<ScalarType> p2o = into_frame.inverse() * this->observation_pose;

	// transform point into next frame
	Eigen::Matrix<ScalarType, 3, 1> new_point = p2o * this->mu;

	return new_point;
}

/*
 * add patch, delete old patch
 */
void Feature::addPatch(cv::Mat patch){
	this->patches.push_back(patch);

	while(this->patches.size() > REFERENCE_PATCH_DEPTH){
		this->patches.pop_front();
	}
}

ScalarType occlusionSSD(cv::Mat test_patch){
	ROS_ASSERT(test_patch.rows == this->patches.front().rows && test_patch.cols == this->patches.front().cols);

	ROS_ASSERT(test_patch.rows != 0 && test_patch.cols != 0);

	ScalarType SSD = 0;

	//TODO add weights

	for(int i = 0; i < test_patch.rows; i++){
		for(int j = 0; j < test_patch.cols; j++){
			ScalarType error = test_patch[i+j] - this->patches.front()[i+j];
			SSD += error*error;
		}
	}

	return SSD/(ScalarType)(test_patch.rows*test_patch.cols);
}
