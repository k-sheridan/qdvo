/*
 * Feature.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "Feature.h"

Feature::Feature() {

}

Feature::Feature(cv::Point2f pt, float depth, float depth_variance, Eigen::Matrix<ScalarType, 3, 3> K){
	this->px = pt;
	Eigen::Matrix<ScalarType, 2, 1> bearing = Feature::pixel2Metric(K, pt);

	//make sure that the pixel is in the frame
	ROS_ASSERT(f.isPixelInBox(pt));

	ROS_ASSERT(depth > 0);

	//set the feature position
	this->mu(0) = bearing(0);
	this->mu(1) = bearing(1);
	this->mu(2) = 1/depth;

	ROS_ASSERT(depth_variance >= 0);
	//set the uncertainty of the feature position
	this->Sigma << DEFAULT_POINT_HOMOGENOUS_VARIANCE, 0, 0,
								0, DEFAULT_POINT_HOMOGENOUS_VARIANCE, 0,
								0, 0, depth_variance

}

Feature::~Feature() {
	// TODO Auto-generated destructor stub
}
