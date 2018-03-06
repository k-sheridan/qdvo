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

	//this->Sigma.block<BASE_STATE_SIZE, BASE_STATE_SIZE>(0, 0).setZero(); //wipe the base state sigmas
	this->Sigma.setZero(); // this should sufficiently reserve enough indices

	this->Sigma(0, 0) = 0;
	this->Sigma(1, 1) = 0;
	this->Sigma(2, 2) = 0;
	this->Sigma(3, 3) = 0;
	this->Sigma(4, 4) = 0;
	this->Sigma(5, 5) = 0;
	this->Sigma(6, 6) = 0;

	this->Sigma(7, 7) = 30;
	this->Sigma(8, 8) = 30;
	this->Sigma(9, 9) = 30;
	this->Sigma(10, 10) = 30;
	this->Sigma(11, 11) = 30;
	this->Sigma(12, 12) = 30;
	this->Sigma(13, 13) = 30;
	this->Sigma(14, 14) = 30;
	this->Sigma(15, 15) = 30;

	this->Sigma(16, 16) = 0.5; // somewhat certain about the biases
	this->Sigma(17, 17) = 0.5;
	this->Sigma(18, 18) = 0.5;
	this->Sigma(19, 19) = 0.5;
	this->Sigma(20, 20) = 0.5;
	this->Sigma(21, 21) = 0.5;

}

void StateEstimator::addNewFeatures(std::vector<Eigen::Vector2f> new_homogenous_features, Frame& f){
	if(!new_homogenous_features.size()){return;}

	//TODO compute the average depth in the scene
	ScalarType average_scene_depth = DEFAULT_POINT_DEPTH;

	// add all new features to the state
	for(auto e : new_homogenous_features){
		this->features.push_back(Feature(e, average_scene_depth, f)); // add a point with an estimated depth
	}
}

void StateEstimator::process(ScalarType dt){
	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> F = this->linearizeProcess(dt); // compute the jacobian of the process numerically


	// process and update the feature vector because it depends on the base mu
	/*for(auto& e : this->features){
		e.setMu(this->convolveFeature(this->mu, e.getMu(), dt));
	}*/

	// process the base mu
	this->mu = this->convolveState(this->mu, dt);


	ROS_DEBUG("start process");
	// update the Sigma
	this->Sigma = F * this->Sigma * F.transpose();
	this->Sigma += this->generateProcessNoise(dt);

	ROS_DEBUG("finish process");
}

Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> StateEstimator::generateProcessNoise(ScalarType dt){

	ScalarType low_noise = 0.00001 * dt;
	ScalarType pos_noise = 0.0001 * dt;
	ScalarType velocity_noise = 0.01*dt;
	ScalarType omega_noise = 5*dt;
	ScalarType accel_noise = 5*dt;
	ScalarType bias_noise = 0.001*dt;

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> Q;
	Q.setZero();

	// set up base part
	Q(0, 0) = pos_noise;
	Q(1, 1) = pos_noise;
	Q(2, 2) = pos_noise;
	Q(3, 3) = pos_noise;
	Q(4, 4) = pos_noise;
	Q(5, 5) = pos_noise;

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

Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> StateEstimator::linearizeProcess(ScalarType dt){

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> F;
	F.setIdentity();

	//pos by vel
	F(0, 3) = dt;
	F(1, 4) = dt;
	F(2, 5) = dt;
	ROS_ASSERT(false);

	//pos by accel
	float half_dt_2 = 0.5*dt*dt;
	F(0, 9) = half_dt_2;
	F(1, 10) = half_dt_2;
	F(2, 11) = half_dt_2;

	//theta by omega
	F(3, 6) = dt;
	F(4, 7) = dt;
	F(5, 8) = dt;

	//vel by accel
	F(6, 9) = dt;
	F(7, 10) = dt;
	F(8, 11) = dt;

	return F;
}



StateEstimator::State StateEstimator::convolveState(State& last, ScalarType dt){

	State new_mu;

	new_mu.setPosition(last.getPosition() + dt*last.getVelocity() + 0.5*dt*dt*last.getAcceleration());

	// rotate the quat
	ROS_ASSERT(false);

	new_mu.setVelocity(last.getVelocity() + dt*last.getAcceleration());

	new_mu.setOmega(last.getOmega());

	new_mu.setAcceleration(last.getAcceleration());

	new_mu.setPhi(last.getPhi());

	new_mu.setAccelBiases(last.getAccelBiases());

	new_mu.setGyroBiases(last.getGyroBiases());

	new_mu.setLambda(last.getLambda());

	return new_mu;
}

void StateEstimator::updateWithFeaturePositions(std::vector<Eigen::Vector2f> measured_positions, std::vector<Eigen::Matrix<ScalarType, 2, 2> > estimated_covariance, std::vector<bool> pass, ros::Time t){
	
	int i = 0;
	for(std::list<Feature>::iterator it = this->features.begin(); it != this->features.end(); it++){

		if(pass.at(i)){

		}
		else{
			this->deleteFeature(it);
			it++; // increment because we just deleted this feature
		}

		i++; //increment
	}

	ROS_ASSERT(i == measured_positions.size()-1); // ensure that we went through each element


	//find the most recent IMU updated state to start from

}

/*
 * update the state with an imu measurement
 */
void StateEstimator::updateWithIMU(Eigen::Matrix<ScalarType, 3, 1> accel, Eigen::Matrix<ScalarType, 3, 1> gryo, Eigen::Matrix<ScalarType, 3, 3>& accel_cov, Eigen::Matrix<ScalarType, 3, 3>& gyro_cov, tf::Transform& c2imu){

	//measurement function must use angular velocity, phi, theta, accel.

}


void StateEstimator::imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 6, BASE_STATE_SIZE>& H, Eigen::Matrix<ScalarType, 6, 1>& z_est, tf::Transform& c2imu){

}

// simply evaluates the nonlinear measurement function. used for numerical linearization
void StateEstimator::imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 6, 1>& z_est, tf::Transform& c2imu){

}

std::vector<Eigen::Vector2f> StateEstimator::previousFeaturePositionVector(){
	std::vector<Eigen::Vector2f> output;
	//output.reserve(this->features.size());
	for(auto e : this->features){
		output.push_back(e.getLastResultFromKLTTracker());
	}

	return output;
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

void StateEstimator::deleteFeature(std::list<Feature>::iterator it){
	this->features.erase(it);
}

/*
 * sqrt(chi2)
 */
double StateEstimator::getHuberWeight(double chi)
{
	if(chi <= HUBER_WIDTH)
	{
		return 1.0;
	}
	else
	{
		return HUBER_WIDTH / chi;
	}
}
