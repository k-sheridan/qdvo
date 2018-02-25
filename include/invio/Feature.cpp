/*
 * Feature.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#include "Feature.h"

Feature::Feature() {

}

Feature::Feature(Eigen::Vector2f homogenous, float depth, Frame& f){
	this->last_result_from_klt_tracker = homogenous;
	this->bearing = homogenous;
	this->depth_inv = 1.0/depth;
	this->delete_flag = false;

	// set variance
	//this->feature_covariance.setZero() // no correlations initially
	this->depth_inv_sigma = DEFAULT_POINT_DEPTH_VARIANCE;
}

Feature::~Feature() {
	// TODO Auto-generated destructor stub
}

Eigen::Vector2f Feature::getBearing(){
	return this->bearing;
}

float Feature::getDepth(){
	return 1.0/this->depth_inv;
}

cv::Point2f Feature::getPixel(const Frame& f){
	return cv::Point2f(f.K(0)*this->bearing(0) + f.K(2), f.K(4)*this->bearing(1) + f.K(5));
}
