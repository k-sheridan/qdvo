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
	Sophus::SE3f first_observation_pose_inv;
	Eigen::Vector2f first_bearing;
	float last_depth_inv;

	bool solved; // is the inverse depth accurate enough for integration into BA

	Feature* ft; // a pointer to the feature corresponding to this solver

	DepthSolver();
	DepthSolver(Feature* parent);
	virtual ~DepthSolver();

	bool solveAndUpdatePointDepthLinear(Sophus::SE3f current_pose, Eigen::Vector2f current_bearing);

};

#endif /* INVIO_INCLUDE_INVIO_DEPTHSOLVER_H_ */
