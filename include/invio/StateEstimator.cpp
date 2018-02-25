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
	this->mu.setZero();
	//this->Sigma.block<BASE_STATE_SIZE, BASE_STATE_SIZE>(0, 0).setZero(); //wipe the base state sigmas
	this->Sigma.setZero(); // this should sufficiently reserve enough indices

	this->Sigma(0, 0) = 0;
	this->Sigma(1, 1) = 0;
	this->Sigma(2, 2) = 0;
	this->Sigma(3, 3) = 0;
	this->Sigma(4, 4) = 0;
	this->Sigma(5, 5) = 0;
	this->Sigma(6, 6) = 0;

	this->mu(3) = 1.0; // no rotation

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
	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> F = this->linearizeProcess(this->mu, dt); // compute the jacobian of the process numerically


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
	int dim = BASE_STATE_SIZE + this->features.size()*3;

	ScalarType low_noise = 0.0001 * dt;
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
	Q(6, 6) = pos_noise;

	Q(7, 7) = velocity_noise;
	Q(8, 8) = velocity_noise;
	Q(9, 9) = velocity_noise;
	Q(10, 10) = omega_noise;
	Q(11, 11) = omega_noise;
	Q(12, 12) = omega_noise;

	Q(13, 13) = accel_noise;
	Q(14, 14) = accel_noise;
	Q(15, 15) = accel_noise;

	Q(16, 16) = bias_noise;
	Q(17, 17) = bias_noise;
	Q(18, 18) = bias_noise;
	Q(19, 19) = bias_noise;
	Q(20, 20) = bias_noise;
	Q(21, 21) = bias_noise;

	// add feature noises
	for(int index = BASE_STATE_SIZE; index < dim;){
		Q(index, index) = low_noise;
		index++;
		Q(index, index) = low_noise;
		index++;
		Q(index, index) = low_noise;
		index++;
	}

	return Q;
}

Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> StateEstimator::linearizeProcess(Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1>& mu, ScalarType dt){


}



Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> StateEstimator::convolveState(Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1>& last, ScalarType dt){


}

Eigen::Vector3f StateEstimator::convolveFeature(Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1>& base_state, Eigen::Vector3f& feature_state, ScalarType dt)
{

	static ScalarType last_omegax = 0;
	static ScalarType last_omegay = 0;
	static ScalarType last_omegaz = 0;
	static Eigen::Quaternionf dq_inv = Eigen::Quaternionf::Identity();

	Eigen::Vector3f vel, accel;

	//pos = Eigen::Vector3f(base_state(0), base_state(1), base_state(2));
	vel = Eigen::Vector3f(base_state(7), base_state(8), base_state(9));
	accel = Eigen::Vector3f(base_state(13), base_state(14), base_state(15));

	//convert feature to position in camera's coordinate frame
	Eigen::Vector3f feature_pos = feature_state;

	//ROS_DEBUG_STREAM("feature pos to convolve: " << feature_pos);

	feature_pos(0) = feature_pos(0)*feature_pos(2);
	feature_pos(1) = feature_pos(1)*feature_pos(2);


	Eigen::Vector3f translation = dt*vel + 0.5*dt*dt*accel;

	if(last_omegax != base_state(10) || last_omegay != base_state(11) || last_omegaz != base_state(12))
	{
		//ROS_DEBUG("omega has changed");
		Eigen::Vector3f omega = Eigen::Vector3f(base_state(10), base_state(11), base_state(12));

		ScalarType omega_norm = omega.norm();

		if(omega_norm < 1e-10){
			//small angle approximation
			dq_inv = Eigen::Quaternionf(1.0, -omega.x()*dt, -omega.y()*dt, -omega.z()*dt);
			dq_inv.normalize();
		}
		else{
			ScalarType theta = dt*omega_norm;
			Eigen::Vector3f omega_hat = omega / omega_norm;
			ScalarType st2 = sin(theta/2);

			dq_inv = Eigen::Quaternionf(cos(theta/2), -omega_hat.x() * st2, -omega_hat.y() * st2, -omega_hat.z() * st2);
		}

		last_omegax = base_state(10);
		last_omegay = base_state(11);
		last_omegaz = base_state(12);

	}

	// this transforms the point into the next camera frame
	feature_pos = dq_inv*feature_pos;
	feature_pos.noalias() += -(dq_inv*translation);

	//bring the point back to homogenous coordinates
	feature_pos(0) /= feature_pos(2);
	feature_pos(1) /= feature_pos(2);

	//ROS_DEBUG_STREAM("convolved feature: " << feature_pos);

	return feature_pos;
}

std::vector<Eigen::Vector2f> StateEstimator::previousFeaturePositionVector(){
	std::vector<Eigen::Vector2f> output;
	//output.reserve(this->features.size());
	for(auto e : this->features){
		output.push_back(e.getLastResultFromKLTTracker());
	}

	return output;
}


Eigen::Matrix2f StateEstimator::getFeatureHomogenousCovariance(int index){
	int start = BASE_STATE_SIZE + index * 3;
	return this->Sigma.block(start, start, 2, 2);
}

void StateEstimator::setFeatureHomogenousCovariance(int index, Eigen::Matrix2f cov)
{
	int start = BASE_STATE_SIZE + index * 3;
	ROS_ERROR("tried to set to sparse matrix");
	this->Sigma(start, start) = cov(0, 0);
	this->Sigma(start+1, start) = cov(1, 0);
	this->Sigma(start, start+1) = cov(0, 1);
	this->Sigma(start+1, start+1) = cov(1, 1);
}

ScalarType StateEstimator::getFeatureDepthVariance(int index){
	int start = BASE_STATE_SIZE + index * 3 + 2;
	return this->Sigma(start, start);
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
