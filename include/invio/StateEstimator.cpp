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


void StateEstimator::process(ScalarType dt){
	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> F = this->linearizeProcess(dt); // compute the jacobian of the process numerically

	// process the base mu
	this->mu = this->convolveState(this->mu, dt);

	// update the Sigma
	this->Sigma = F * this->Sigma * F.transpose(); // transform the uncertainty into the future in the last pose's tangent space
	this->Sigma += this->generateProcessNoise(dt); // add process noise

	// increment the time
	this->t += ros::Duration(dt);

	ROS_DEBUG_STREAM("ran processwith dt: " << dt << " t: " << this->t);
}

Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> StateEstimator::generateProcessNoise(ScalarType dt){

	ScalarType low_noise = 0.00001 * dt;
	ScalarType pos_noise = 0.0025 * dt;
	ScalarType angle_noise = 0.001 * dt;
	ScalarType velocity_noise = 4*dt;
	ScalarType omega_noise = 16*dt;
	ScalarType accel_noise = 16*dt;
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
	//ROS_ASSERT(last.getAngularTwist() == (Eigen::Matrix<ScalarType, 3, 1>(0,0,0)));
	//ROS_ASSERT(last.getLinearTwist() == (Eigen::Matrix<ScalarType, 3, 1>(0,0,0)));

	new_mu.setLinearTwist(dt*last.getVelocity() + dt*dt*0.5*last.getAcceleration());
	new_mu.setAngularTwist(dt*last.getOmega());

	ROS_DEBUG_STREAM("twist: " << new_mu.getTwist());

	// higher derivatives

	new_mu.setVelocity(last.getVelocity() + dt*last.getAcceleration());

	new_mu.setOmega(last.getOmega());

	new_mu.setAcceleration(last.getAcceleration());

	new_mu.setPhi(last.getPhi());

	new_mu.setAccelBiases(last.getAccelBiases());

	new_mu.setGyroBiases(last.getGyroBiases());

	new_mu.setLambda(last.getLambda());

	//apply odometry on the pose and zero the tangent space
	new_mu.true_pose = new_mu.true_pose * Sophus::SE3<ScalarType>::exp(new_mu.getTwist());

	new_mu.setLinearTwist(Eigen::Matrix<ScalarType, 3, 1>(0,0,0));
	new_mu.setAngularTwist(Eigen::Matrix<ScalarType, 3, 1>(0,0,0));


	return new_mu;
}

/*
 * uses tracked features and their position estimates to update the pose iteratively
 */
void StateEstimator::motionOptimization(Frame& cf){
	ROS_DEBUG_STREAM("update with : " << cf.features.size(););

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> P_inv = this->Sigma.inverse();

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> last_full_A, last_T_inv;

	bool final_iteration = false;

	Eigen::Matrix<ScalarType, 6, 6> A;
	Eigen::Matrix<ScalarType, 6, 1> b;
	ScalarType chi2_last = 0;

	// these two defines allow us to either dampen or strengthen the effect this update has on both motion and structure
#define INV_BEARING_VARIANCE_FOR_MOBA 100000

	/*
	 * iteratatively update the motion of the camera and the structure of the scene
	 */
	for (int i = 0; i < MOBA_MAX_ITERATIONS; i++){

		//MOTION ONLY BUNDLE ADJUSTMENT ITERATION
		A.setZero();
		b.setZero();

		ScalarType chi2_curr = 0;
		int num_feature = 0;

		Sophus::SE3<ScalarType> pose_inv = this->mu.true_pose.inverse();
		Eigen::Matrix<ScalarType, 3, 3> Rt = pose_inv.rotationMatrix();

		//compute the A and b matrices.
		for(std::list<Feature>::iterator it = cf.features.begin(); it != cf.features.end(); it++){
			//project the feature into the current pose estimate
			Eigen::Matrix<ScalarType, 3, 1> xyz_proj = pose_inv * it->getWorldCoordinate();

			// make sure that this feature is ahead of us.
			if(xyz_proj.z() <= 0){
				ROS_WARN("feature behind camera!");
				continue;
			}

			// compute the linearized relationship between a small twist and a small metric pixel movement
			Eigen::Matrix<ScalarType, 2, 6> H;
			this->twist2uv(xyz_proj, H);

			// compute the error we are trying to minimize
			Eigen::Matrix<ScalarType, 2, 1> e = Feature::pixel2Metric(cf.K, it->getPx()) - Eigen::Matrix<ScalarType, 2, 1>(xyz_proj(0)/xyz_proj(2), xyz_proj(1)/xyz_proj(2));

			ScalarType chi2 = e.squaredNorm();

			chi2_curr += chi2;

			// compute the huber weight for this feature
			ScalarType huber = this->getHuberWeight(sqrt(chi2));

			//ScalarType inv_z_cov2 = 1 / (Rt * it->getWorldUncertainty() * Rt.transpose())(2, 2);

			ScalarType weight = huber * INV_BEARING_VARIANCE_FOR_MOBA;

			// set up system
			A.noalias() += H.transpose() * H * weight;
			b.noalias() += H.transpose() * e * weight;

			num_feature++;

		}

		ROS_INFO_STREAM("avg error: " << chi2_curr/num_feature);
		//TODO check for stopping condition

		// perform modified iterative Kalman update on the state using A and b
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> A_full;
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> b_full;

		A_full.setZero();
		b_full.setZero();

		A_full.block<6, 6>(0,0) = A;
		b_full.block<6, 1>(0,0) = b;

		last_full_A = A_full; // set this for later use

		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> T_inv = (P_inv + A_full);

		last_T_inv = T_inv;

		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> dx = T_inv.ldlt().solve(b_full);

		this->mu.mean.noalias() += dx;

		this->mu.true_pose = this->mu.true_pose * Sophus::SE3<ScalarType>::exp(dx.block<6, 1>(0,0));


	}


	//apply modified uncertainty update to the state uncertainty
	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> T = last_T_inv.inverse();

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> i_kh;
	i_kh.setIdentity();
	i_kh.noalias() -= T * last_full_A;

	this->Sigma = i_kh * this->Sigma * i_kh.transpose();
	this->Sigma.noalias() += T*last_full_A*T.transpose();

}



void StateEstimator::structureOptimization(Frame& cf){
	//SPARSE STRUCTURE OPTIMIZATION

	Sophus::SE3<ScalarType> pose_inv = this->mu.true_pose.inverse(); // precomputed for efficiency
	Eigen::Matrix<ScalarType, 3, 3> Rt = pose_inv.rotationMatrix();

#define BEARING_VARIANCE_FOR_STRUCTURE 1

	// perform one iteration
	for(std::list<Feature>::iterator it = cf.features.begin(); it != cf.features.end(); it++){
		//project the feature into the current pose estimate
		Eigen::Matrix<ScalarType, 3, 1> xyz_proj = pose_inv * it->getWorldCoordinate();

		// make sure that this feature is ahead of us.
		if(xyz_proj.z() <= 0){
			ROS_WARN("feature behind camera!");
			continue;
		}

		// compute the linearized map between pixel error and change in feature position
		Eigen::Matrix<ScalarType, 2, 3> H;
		this->featurePosition2uv(xyz_proj, H);

		H = H*Rt; // this is a map from world coordinate position change to bearing error

		// compute the bearing error
		Eigen::Matrix<ScalarType, 2, 1> e = Feature::pixel2Metric(cf.K, it->getPx()) - Eigen::Matrix<ScalarType, 2, 1>(xyz_proj(0)/xyz_proj(2), xyz_proj(1)/xyz_proj(2));

		//ROS_DEBUG_STREAM("prefit: " e.norm();)

		// apply a standard kalman update

		Eigen::Matrix<ScalarType, 3, 2> K = it->getWorldUncertainty() * H.transpose();

		Eigen::Matrix<ScalarType, 2, 2> S = H*it->getWorldUncertainty()*H.transpose();
		S(0,0) += BEARING_VARIANCE_FOR_STRUCTURE;
		S(1, 1) += BEARING_VARIANCE_FOR_STRUCTURE;

		K = K * S.inverse();

		// if this is not the last iteration just update the mean other wise just update the covariance

		Eigen::Matrix<ScalarType, 3, 3> i_kh;
		i_kh.setIdentity();
		i_kh.noalias() -= K*H;
		it->setWorldUncertainty(i_kh * it->getWorldUncertainty() * i_kh.transpose() + K*K.transpose() * BEARING_VARIANCE_FOR_STRUCTURE);

		it->setWorldCoordinate(it->getWorldCoordinate() + K*e);



	}
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

/*
 * removes any potential outliers from the state
 */
void StateEstimator::removeOutliers(Frame& cf, ScalarType threshold){


	std::vector<ScalarType> chi_vec;
	chi_vec.resize(cf.features.size(), -1);

	std::vector<bool> remove;
	remove.resize(cf.features.size(), false);

	int i = 0;

	Sophus::SE3<ScalarType> pose_inv = this->mu.true_pose.inverse();

	for(std::list<Feature>::iterator it = cf.features.begin(); it != cf.features.end(); it++){
		//project the feature into the current pose estimate
		Eigen::Matrix<ScalarType, 3, 1> xyz_proj = pose_inv * it->getWorldCoordinate();

		// make sure that this feature is ahead of us.
		if(xyz_proj.z() <= 0){
			ROS_WARN("feature behind camera!");
			remove[i] = true;
			i++;
			continue;
		}

		// compute the bearing error
		Eigen::Matrix<ScalarType, 2, 1> e = Feature::pixel2Metric(cf.K, it->getPx()) - Eigen::Matrix<ScalarType, 2, 1>(xyz_proj(0)/xyz_proj(2), xyz_proj(1)/xyz_proj(2));

		chi_vec[i] = e.norm();
		i++;
	}


	// REMOVE features with too high of an error
	i = 0;

	for(std::list<Feature>::iterator it = cf.features.begin(); it != cf.features.end();){

		if(remove[i] || chi_vec[i] > threshold){
			it = cf.features.erase(it);
		}
		else{
			it++;
		}

		i++;

	}
}
