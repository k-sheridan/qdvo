/*
 * DepthSolver.h
 *
 *  Created on: Sep 17, 2017
 *      Author: kevin
 */

#ifndef INVIO_INCLUDE_INVIO_DEPTHSOLVER_H_
#define INVIO_INCLUDE_INVIO_DEPTHSOLVER_H_

#include <sophus/se3.hpp>

#include "opencv2/core/core.hpp"


#include <Params.h>
#include <Frame.h>
#include <Feature.h>

class DepthSolver {
public:

	// the state of the feature at the last observation
	Sophus::SE3<ScalarType> first_observation_pose_inv;
	Eigen::Matrix<ScalarType, 2, 1> first_bearing;
	ScalarType last_depth_inv;

	cv::Mat reference_patch; // a template of the feature

	bool solved; // is the inverse depth accurate enough for integration into BA

	DepthSolver();
	virtual ~DepthSolver();

	//bool solveAndUpdatePointDepthLinear(Sophus::SE3f current_pose, Eigen::Vector2f current_bearing);

};

#endif /* INVIO_INCLUDE_INVIO_DEPTHSOLVER_H_ */
