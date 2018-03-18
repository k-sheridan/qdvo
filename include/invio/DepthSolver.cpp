/*
 * DepthSolver.cpp
 *
 *  Created on: Sep 17, 2017
 *      Author: kevin
 */

#include <DepthSolver.h>

DepthSolver::DepthSolver() {
	this->solved = false;
}


DepthSolver::~DepthSolver() {

}

/*
bool DepthSolver::solveAndUpdatePointDepthLinear(Sophus::SE3f current_pose, Eigen::Vector2f current_bearing)
{

	// stereo depth estimate of feature in current frame
	Sophus::SE3f rf_2_cf = this->last_observation_pose_inv * current_pose;
	Sophus::SE3f cf_2_rf = rf_2_cf.inverse();

	Eigen::Vector3f epiline = rf_2_cf.rotationMatrix() * Eigen::Vector3f(current_bearing(0), current_bearing(1), 1.0);

	//solve for the depth
	Eigen::Matrix<float,3,2> A; A << epiline, Eigen::Vector3f(this->last_bearing(0), this->last_bearing(1), 1.0);

	const Eigen::Matrix2f AtA = A.transpose()*A;

	//double AtA_det = AtA.determinant();

	if(AtA.determinant() < MINIMUM_DEPTH_DETERMINANT)
	{
		ROS_DEBUG("determinant too low");
	    return false;
	}

	const Eigen::Vector2f depth2 = - AtA.inverse()*A.transpose()*rf_2_cf.translation();

	double depth = fabs(depth2[0]);

	if(depth < MIN_POINT_Z || depth > MAX_POINT_Z)
	{
		ROS_DEBUG_STREAM("point at extreme depth");
		return false;
	}

	//evaluate the reprojection error
	//Eigen::Vector3d projected_ref_ft = (cf_2_rf * (depth * pt->getInitialHomogenousCoordinate()));

	//double chi2 = pow(projected_ref_ft(0)/projected_ref_ft(2) - curr_ft(0), 2) + pow(projected_ref_ft(1)/projected_ref_ft(2) - curr_ft(1), 2);

	//Angle variance
	Eigen::Vector3f t = cf_2_rf.translation();
	Eigen::Vector3f d = depth*Eigen::Vector3f(current_bearing(0), current_bearing(1), 1.0);
	Eigen::Vector3f t2d = d - t;

	float d_norm = d.norm();
	float t2d_norm = t2d.norm();
	//double t_norm = t.norm();

	float sine_theta_d_t = d.cross(t2d).norm() / (d_norm*t2d_norm);

	float variance = 1 / pow(sine_theta_d_t + DBL_MIN, 2);

	ROS_DEBUG_STREAM("updating point with depth: " << depth << " and depth variance: " << variance << "where the sine is: " << sine_theta_d_t);


	//TODO find good homogeneous variance
	Eigen::Vector3d point_in_rf = rf_2_cf * (depth * Eigen::Vector3f(current_bearing(0), current_bearing(1), 1.0)); // transform the measured point into the reference frame

	Eigen::Vector3d sigma = Eigen::Vector3d(variance, variance, variance);

	Eigen::Vector3d z = Eigen::Vector3d(point_in_rf(0) / point_in_rf(2), point_in_rf(1) / point_in_rf(2), point_in_rf(2));

	pt->update(z, sigma);
	
	//check if the feature has converged enough to become a candidate
	if(pt->getDepthVariance() <= MOBA_CANDIDATE_VARIANCE)
	{
		pt->moba_candidate = true; // flag for final step before moba integration or deletion
	}

	return true;

}
*/

