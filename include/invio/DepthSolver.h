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
#include <Feature.h>

class DepthSolver {
public:

	bool solved; // is the inverse depth accurate enough for integration into BA

	DepthSolver();
	virtual ~DepthSolver();

	//bool solveAndUpdatePointDepthLinear(Sophus::SE3f current_pose, Eigen::Vector2f current_bearing);

};

#endif /* INVIO_INCLUDE_INVIO_DEPTHSOLVER_H_ */
