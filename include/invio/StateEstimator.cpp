/*
 * StateEstimator.cpp
 *
 *  Created on: Nov 25, 2017
 *      Author: kevin
 */

#include <StateEstimator.h>

StateEstimator::StateEstimator() {

	this->Sigma.resize(BASE_STATE_SIZE, BASE_STATE_SIZE);

	//set the time to unknown, it is based of the stamp of the first message recieved
	this->t = ros::Time(0);

	//setup initial variances and values of the base state
	this->initializeState();

}


void StateEstimator::initializeState()
{
	this->mu.mean.setZero();

	//set the scale factor to 1
	this->mu.setLambda(1);

	//this->mu.setOmega(Eigen::Matrix<ScalarType, 3, 1>(0, 0, VIO_PI));
	this->mu.setGyroBiases(Eigen::Matrix<ScalarType, 3, 1>(0, 0.02, 0.07));

	//this->Sigma.block<BASE_STATE_SIZE, BASE_STATE_SIZE>(0, 0).setZero(); //wipe the base state sigmas
	this->Sigma.setZero(); // this should sufficiently reserve enough indices

	//derivative variances
#define ZEROTH_VAR 0
#define FIRST_VAR 3*3
#define SECOND_VAR 3*3

#define PHI_VAR VIO_PI*VIO_PI
#define BIAS_VAR 0.1*0.1
#define GYRO_BIAS_VAR 0.1*0.1
#define LAMBDA_VAR 2*2

	this->Sigma(POSX_INDEX, POSX_INDEX) = ZEROTH_VAR;
	this->Sigma(POSX_INDEX+1, POSX_INDEX+1) = ZEROTH_VAR;
	this->Sigma(POSX_INDEX+2, POSX_INDEX+2) = ZEROTH_VAR;

	this->Sigma(THETAX_INDEX, THETAX_INDEX) = ZEROTH_VAR;
	this->Sigma(THETAX_INDEX+1, THETAX_INDEX+1) = ZEROTH_VAR;
	this->Sigma(THETAX_INDEX+2, THETAX_INDEX+2) = ZEROTH_VAR;

	this->Sigma(VELX_INDEX, VELX_INDEX) = FIRST_VAR;
	this->Sigma(VELX_INDEX+1, VELX_INDEX+1) = FIRST_VAR;
	this->Sigma(VELX_INDEX+2, VELX_INDEX+2) = FIRST_VAR;

	this->Sigma(OMEGAX_INDEX, OMEGAX_INDEX) = FIRST_VAR;
	this->Sigma(OMEGAX_INDEX+1, OMEGAX_INDEX+1) = FIRST_VAR;
	this->Sigma(OMEGAX_INDEX+2, OMEGAX_INDEX+2) = FIRST_VAR;

	this->Sigma(ACCELX_INDEX, ACCELX_INDEX) = SECOND_VAR;
	this->Sigma(ACCELX_INDEX+1, ACCELX_INDEX+1) = SECOND_VAR;
	this->Sigma(ACCELX_INDEX+2, ACCELX_INDEX+2) = SECOND_VAR;

	this->Sigma(PHIX_INDEX, PHIX_INDEX) = PHI_VAR;
	this->Sigma(PHIX_INDEX+1, PHIX_INDEX+1) = PHI_VAR;
	this->Sigma(PHIX_INDEX+2, PHIX_INDEX+2) = PHI_VAR;

	this->Sigma(ACCELBIASX_INDEX, ACCELBIASX_INDEX) = BIAS_VAR;
	this->Sigma(ACCELBIASX_INDEX+1, ACCELBIASX_INDEX+1) = BIAS_VAR;
	this->Sigma(ACCELBIASX_INDEX+2, ACCELBIASX_INDEX+2) = BIAS_VAR;

	this->Sigma(GYROBIASX_INDEX, GYROBIASX_INDEX) = GYRO_BIAS_VAR;
	this->Sigma(GYROBIASX_INDEX+1, GYROBIASX_INDEX+1) = GYRO_BIAS_VAR;
	this->Sigma(GYROBIASX_INDEX+2, GYROBIASX_INDEX+2) = GYRO_BIAS_VAR;

	this->Sigma(LAMBDA_INDEX, LAMBDA_INDEX) = LAMBDA_VAR;


}

/*
 * transform the tangent space representation of the pose into the tangent space around the current pose estimate
 */
void StateEstimator::transformToTangentSpace(){
	Sophus::SE3<ScalarType> twist = Sophus::SE3<ScalarType>::exp(this->mu.getTwist());

	// apply twist to the true pose
	this->mu.true_pose = this->mu.true_pose * twist;

	//compute the inverse adjoint map
	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> A;
	A.setIdentity();
	A.block(0, 0, 6, 6) = twist.inverse().Adj();

	this->Sigma = A * this->Sigma * A.transpose(); // transform uncertainty into the tangent space around the current pose estimate

	// zero the tangent space again
	this->mu.setLinearTwist(Eigen::Matrix<ScalarType, 3, 1>(0,0,0));
	this->mu.setAngularTwist(Eigen::Matrix<ScalarType, 3, 1>(0,0,0));
}

void StateEstimator::process(ScalarType dt){
	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> F = this->linearizeProcess(dt); // compute the jacobian of the process numerically

	// process the base mu
	this->mu = this->convolveState(this->mu, dt);

	ROS_DEBUG_STREAM("twist: " << this->mu.getTwist());

	// update the Sigma
	this->Sigma = F * this->Sigma * F.transpose(); // transform the uncertainty into the future in the last pose's tangent space
	this->Sigma += this->generateProcessNoise(dt); // add process noise

	// transform the tangent space again
	this->transformToTangentSpace();

	// increment the time
	this->t += ros::Duration(dt);

	ROS_DEBUG_STREAM("ran processwith dt: " << dt << " t: " << this->t);
}

Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> StateEstimator::generateProcessNoise(ScalarType dt){

	ScalarType low_noise = 0.00001 * dt;
	ScalarType pos_noise = 0.5 * dt;
	ScalarType angle_noise = 0.01 * dt;
	ScalarType velocity_noise = 1*dt;
	ScalarType omega_noise = 5*dt;
	ScalarType accel_noise = 5*dt;
	ScalarType bias_noise = 0.0001*dt;

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> Q;
	Q.setZero();

	// set up base part
	Q(0, 0) = pos_noise;
	Q(1, 1) = pos_noise;
	Q(2, 2) = pos_noise;
	Q(3, 3) = angle_noise;
	Q(4, 4) = angle_noise;
	Q(5, 5) = angle_noise;

	Q(6, 6) = velocity_noise;
	Q(7, 7) = velocity_noise;
	Q(8, 8) = velocity_noise;
	Q(9, 9) = omega_noise;
	Q(10, 10) = omega_noise;
	Q(11, 11) = omega_noise;

	Q(12, 12) = accel_noise;
	Q(13, 13) = accel_noise;
	Q(14, 14) = accel_noise;

	Q(15, 15) = low_noise;
	Q(16, 16) = low_noise;
	Q(17, 17) = low_noise;

	Q(18, 18) = bias_noise;
	Q(19, 19) = bias_noise;
	Q(20, 20) = bias_noise;
	Q(21, 21) = bias_noise;
	Q(22, 22) = bias_noise;
	Q(23, 23) = bias_noise;

	Q(24, 24) = low_noise;

	return Q;
}

/*
 * only applies to the process around the tangent space
 */
Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> StateEstimator::linearizeProcess(ScalarType dt){

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> F;
	F.setIdentity();

	//pos by vel
	F(POSX_INDEX, VELX_INDEX) = dt;
	F(POSX_INDEX+1, VELX_INDEX+1) = dt;
	F(POSX_INDEX+2, VELX_INDEX+2) = dt;

	//pos by accel
	ScalarType half_dt_2 = 0.5*dt*dt;
	F(POSX_INDEX, ACCELX_INDEX) = half_dt_2;
	F(POSX_INDEX+1, ACCELX_INDEX+1) = half_dt_2;
	F(POSX_INDEX+2, ACCELX_INDEX+2) = half_dt_2;

	//theta by omega
	F(THETAX_INDEX, OMEGAX_INDEX) = dt;
	F(THETAX_INDEX+1, OMEGAX_INDEX+1) = dt;
	F(THETAX_INDEX+2, OMEGAX_INDEX+2) = dt;

	//vel by accel
	F(VELX_INDEX, ACCELX_INDEX) = dt;
	F(VELX_INDEX+1, ACCELX_INDEX+1) = dt;
	F(VELX_INDEX+2, ACCELX_INDEX+2) = dt;

	return F;
}


/*
 * the state is assumed to represent the tangent space around the current pose
 * This simplifies the process significantly
 */
StateEstimator::State StateEstimator::convolveState(State& last, ScalarType dt){

	State new_mu = last;

	// the position is represented by a twist
	ROS_ASSERT(last.getAngularTwist() == (Eigen::Matrix<ScalarType, 3, 1>(0,0,0)));
	ROS_ASSERT(last.getLinearTwist() == (Eigen::Matrix<ScalarType, 3, 1>(0,0,0)));

	new_mu.setLinearTwist(dt*last.getVelocity() + dt*dt*0.5*last.getAcceleration());
	new_mu.setAngularTwist(dt*last.getOmega());

	// higher derivatives

	new_mu.setVelocity(last.getVelocity() + dt*last.getAcceleration());

	new_mu.setOmega(last.getOmega());

	new_mu.setAcceleration(last.getAcceleration());

	new_mu.setPhi(last.getPhi());

	new_mu.setAccelBiases(last.getAccelBiases());

	new_mu.setGyroBiases(last.getGyroBiases());

	new_mu.setLambda(last.getLambda());

	return new_mu;
}

/*
 * uses tracked features and their position estimates to update the pose iteratively
 */
void StateEstimator::updateWithTrackedFeatures(Frame& cf){

	// invert the covariance matrix to be used during update
	//Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> Sigma_inv = Sigma.llt().solve(Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE>::Identity());
	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> Sigma_inv = Sigma.inverse();

	Eigen::Matrix<ScalarType, 6, 6> A; //LHS
	Eigen::Matrix<ScalarType, 6, 1> b; //RHS

	ScalarType chi2_sum_last, chi2_sum_curr; // store the current error and last error to determine whether to stop the optimization


	// perform iterative pose update
	for(size_t i = 0; i < (size_t)MOBA_MAX_ITERATIONS; i++){
		b.setZero();
		A.setZero();

		chi2_sum_last = chi2_sum_curr;
		chi2_sum_curr = 0;

		Sophus::SE3<ScalarType> pose_inv = this->mu.true_pose.inverse();

		for(auto& e : cf.features){
			Eigen::Matrix<ScalarType, 2, 6> H;
			Eigen::Matrix<ScalarType, 3, 1> xyz_f(pose_inv * e.getWorldCoordinate()); // the point in the current estimated frame's pose
			StateEstimator::jacobian_xyz2uv(xyz_f, H); // compute the linear map for a small twist to a bearing

			Eigen::Matrix<ScalarType, 2, 1> residual = Feature::pixel2Metric(cf.K, e.getPx()) - Eigen::Matrix<ScalarType, 2, 1>(xyz_f(0)/xyz_f(2), xyz_f(1)/xyz_f(2));

			ScalarType chi2 = residual.squaredNorm();
			chi2_sum_curr += chi2;

			// compute this edges weight
			ScalarType huber = this->getHuberWeight(sqrt(chi2));

			A.noalias() += H.transpose() * H * huber;
			b.noalias() += H.transpose() * residual * huber;
		}


		// check if the error has increased
		if(i != 0){
			if(chi2_sum_curr > chi2_sum_last){
				ROS_DEBUG_STREAM("ERROR INCREASED at iteration: " << i+1 << " with chi2_avg: " << chi2_sum_curr/cf.features.size());
				ROS_DEBUG_STREAM("BREAKING");
				break;
			}
			else{
				ROS_DEBUG_STREAM("SUCCESSFUL iteration: " << i+1 << " with chi2_avg: " << chi2_sum_curr/cf.features.size());
			}
		}

		//solve the system
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> LHS = Sigma_inv;
		LHS.block<6, 6>(0, 0) += A;

		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> RHS;
		RHS.setZero();
		RHS.block<6, 1>(0, 0) = b;

		// compute the dx
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> dx = LHS.ldlt().solve(RHS);

		// apply the dx onto the mean
		this->mu.mean.noalias() += dx;
		Sophus::SE3<ScalarType> twist = Sophus::SE3<ScalarType>::exp(this->mu.getTwist());

		this->mu.true_pose = this->mu.true_pose * twist; // apply the small twist to the pose

		// zero the tangent space again
		this->mu.setLinearTwist(Eigen::Matrix<ScalarType, 3, 1>(0,0,0));
		this->mu.setAngularTwist(Eigen::Matrix<ScalarType, 3, 1>(0,0,0));

		//transform state into new tangent space
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> Adj;
		Adj.setIdentity();
		Adj.block<6, 6>(0, 0) = twist.Adj(); // typically it is the inverse transform

		// P' = Ainv * P * AinvT => P'inv = AinvTinv * Pinv * Ainvinv = AT * Pinv * A
		// P'^{-1} = (Jt)^{-1} * P^{-1} * (J)^{-1}

		Sigma_inv = Adj.transpose() * Sigma_inv * Adj; // transform the uncertainty into the new optimized tangent space

		//last_A = A; // save the previous A (information) mat

		ROS_DEBUG_STREAM("iteration " << i+1 << ", dx= " << dx.transpose());

		if(dx.block<6, 1>(0, 0).norm() <= EPS_MOBA){
			ROS_DEBUG_STREAM("EPSILON REACHED... STOPPING");
			break;
		}

	}

	// apply modified josephs uncertainty update

	Eigen::Matrix<ScalarType, 25, 25> A_full;
	A_full.setZero();
	A_full.block<6, 6>(0, 0) = A;

	Eigen::Matrix<ScalarType, 25, 25> T = Sigma_inv + A_full;
	//T = T.ldlt().solve(Eigen::Matrix<ScalarType, 25, 25>::Identity()); // invert
	T = T.inverse(); // invert

	Eigen::Matrix<ScalarType, 25, 25> I_KH = (Eigen::Matrix<ScalarType, 25, 25>::Identity() - T*A_full);



	//ROS_DEBUG_STREAM("A_full: " << A_full);

	//ROS_DEBUG_STREAM("sigma inv: " << Sigma_inv);

	//ROS_DEBUG_STREAM("I_KH: " << I_KH);

	//ROS_DEBUG_STREAM("KRKt: " << T*A_full*T.transpose());

	//invert sigma back
	//this->Sigma = Sigma_inv.llt().solve(Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE>::Identity());
	this->Sigma = Sigma_inv.inverse();

	//propagate uncertainty
	this->Sigma = I_KH * this->Sigma * I_KH.transpose();
	this->Sigma.noalias() += T*A_full*T.transpose();

}


Eigen::Matrix<ScalarType, 2, 2> StateEstimator::getMetric2PixelMap(Eigen::Matrix3f& K){
	Eigen::Matrix<ScalarType, 2, 2> J;
	J.setIdentity();
	J(0, 0) = K(0, 0);
	J(1, 1) = K(1, 1);
	return J;
}

Eigen::Matrix<ScalarType, 2, 2> StateEstimator::getPixel2MetricMap(Eigen::Matrix3f& K){
	Eigen::Matrix<ScalarType, 2, 2> J;
	J.setIdentity();
	J(0, 0) = 1.0f/K(0, 0);
	J(1, 1) = 1.0f/K(1, 1);
	return J;
}

void StateEstimator::checkSigma(){
#define SYM_EPS 0.001
	// first check the diagonal to make sure all members are positive
	for(int i = 0; i < this->Sigma.rows(); i++){
		ROS_FATAL_STREAM_COND(this->Sigma(i, i) < 0, "variance is negative for index: " << i);

		//ROS_ASSERT(this->Sigma(i, i) >= 0);

		// check for symmetry
		for(int j = i+1; j < this->Sigma.rows(); j++){
			ROS_FATAL_STREAM_COND(fabs(this->Sigma(i, j) - this->Sigma(j, i)) > SYM_EPS, "correlation is not symmetric: " << fabs(this->Sigma(i, j) - this->Sigma(j, i)) << " - " <<i<<", "<<j);
			//ROS_ASSERT(fabs(this->Sigma(i, j) - this->Sigma(j, i)) <= SYM_EPS);
		}
	}

}

void StateEstimator::fixSigma(){
	//this->Sigma = (this->Sigma + this->Sigma.transpose()) / 2.0;
}

/*
 * sqrt(chi2)
 */
ScalarType StateEstimator::getHuberWeight(ScalarType chi_abs)
{
	if(chi_abs <= HUBER_WIDTH)
	{
		return 1.0;
	}
	else
	{
		return HUBER_WIDTH / chi_abs;
	}
}
