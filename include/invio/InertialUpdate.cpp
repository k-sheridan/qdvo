/*
 * InertialUpdate.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: kevin
 */

#include <StateEstimator.h>

void StateEstimator::gyroUpdate(Eigen::Matrix<ScalarType, 3, 1> gyro, Eigen::Matrix<ScalarType, 3, 3>& gyro_cov, tf::Transform& c2imu){
	tf::Quaternion q = c2imu.getRotation().inverse();
	Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam = Eigen::Quaternion<ScalarType>(q.w(), q.x(), q.y(), q.z()).matrix();

	// perform EKF update
	Eigen::Matrix<ScalarType, 3, 1> z_est;
	Eigen::Matrix<ScalarType, 3, BASE_STATE_SIZE> H = this->gyroMeasurementMap(R_imu_2_cam);

	z_est = this->gyroMeasurementFromState(this->mu, R_imu_2_cam);

	Eigen::Matrix<ScalarType, 3, 1> residual = gyro;
	residual.noalias() -= z_est;

	Eigen::Matrix<ScalarType, 3, 3> S = H*this->Sigma*H.transpose();
	S.noalias() += gyro_cov;

	Eigen::Matrix<ScalarType, 3, 3> S_inv = S.ldlt().solve(Eigen::Matrix<ScalarType, 3, 3>::Identity());

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 3> K = this->Sigma * H.transpose() * S_inv; // kalman gain

	this->mu.mean += K * residual; // update the estimate

	ROS_DEBUG_STREAM("residual: "<< residual.transpose());
	//ROS_DEBUG_STREAM("Kt: " << K);

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> i_kh;
	i_kh.setIdentity();
	i_kh.noalias() -= (K * H);

	// update the uncertainty
	this->Sigma = i_kh * this->Sigma * i_kh.transpose();
	this->Sigma.noalias() += K * gyro_cov * K.transpose();

	// project uncertainty back into the tangent space
	this->transformToTangentSpace();

}

/*
 * linear mapping from state to gyroscope measurement
 */
Eigen::Matrix<ScalarType, 3, BASE_STATE_SIZE> StateEstimator::gyroMeasurementMap(Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam){
	// no correlations
	Eigen::Matrix<ScalarType, 3, BASE_STATE_SIZE> H;
	H.setZero();

	// simply a rotation with the accel relation being more complex
	H.block<3, 3>(0, OMEGAX_INDEX) = R_imu_2_cam;

	H.block<3, 3>(0, GYROBIASX_INDEX).setIdentity();

	return H;
}

/*
 * update the state with an full imu measurement
 * the state must be at the current measurement time
 */
void StateEstimator::fullImuUpdate(Eigen::Matrix<ScalarType, 3, 1> accel, Eigen::Matrix<ScalarType, 3, 1> gyro, Eigen::Matrix<ScalarType, 3, 3>& accel_cov, Eigen::Matrix<ScalarType, 3, 3>& gyro_cov, tf::Transform& c2imu){
	//measurement function must use angle ,angular velocity, phi, theta, accel, lambda, biases. in total: 22 dof!!!

	ROS_DEBUG_STREAM("performing imu update");

	Eigen::Matrix<ScalarType, 3, 1> r_cam_2_imu = Eigen::Matrix<ScalarType, 3, 1>(c2imu.getOrigin().x(), c2imu.getOrigin().y(), c2imu.getOrigin().z());

	tf::Quaternion q = c2imu.getRotation().inverse();
	Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam = Eigen::Quaternion<ScalarType>(q.w(), q.x(), q.y(), q.z()).matrix();

	// perform EKF update
	Eigen::Matrix<ScalarType, 6, 1> z_est;
	Eigen::Matrix<ScalarType, 6, BASE_STATE_SIZE> H;

	this->imuMeasurementFromState(this->mu, H, z_est, R_imu_2_cam, r_cam_2_imu);

	Eigen::Matrix<ScalarType, 6, 1> z;
	z << accel, gyro;

	Eigen::Matrix<ScalarType, 6, 6> R;
	R.block<3, 3>(0, 0) = accel_cov;
	R.block<3, 3>(0, 3).setZero();
	R.block<3, 3>(3, 3) = gyro_cov;
	R.block<3, 3>(3, 0).setZero();

	Eigen::Matrix<ScalarType, 6, 1> residual = z;
	residual.noalias() -= z_est;

	Eigen::Matrix<ScalarType, 6, 6> S = H*this->Sigma*H.transpose();
	S.noalias() += R;

	Eigen::Matrix<ScalarType, 6, 6> S_inv = S.ldlt().solve(Eigen::Matrix<ScalarType, 6, 6>::Identity());

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 6> K = this->Sigma * H.transpose() * S_inv; // kalman gain

	this->mu.mean += K * residual; // update the estimate

	ROS_DEBUG_STREAM("residual: "<< residual.transpose());
	ROS_DEBUG_STREAM("Kt: " << K);

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> i_kh;
	i_kh.setIdentity();
	i_kh.noalias() -= (K * H);

	// update the uncertainty
	this->Sigma = i_kh * this->Sigma * i_kh.transpose();
	this->Sigma.noalias() += K * R * K.transpose();

	// project uncertainty back into the tangent space
	this->transformToTangentSpace();

	ROS_DEBUG_STREAM("performed imu update");
}

Eigen::Matrix<ScalarType, 3, 1> StateEstimator::numDiffAccel(StateEstimator::State& mean, int index,  Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam, Eigen::Matrix<ScalarType, 3, 1> r_cam_2_imu, Eigen::Matrix<ScalarType, 3, 1> G_transformed){

	mu.mean(index) += DELTA;
	Eigen::Matrix<ScalarType, 3, 1> result = this->accelMeasurementFromState(mu, R_imu_2_cam, r_cam_2_imu, G_transformed); // high
	mu.mean(index) -= 2*DELTA;
	result.noalias() -= this->accelMeasurementFromState(mu, R_imu_2_cam, r_cam_2_imu, G_transformed); // minus low
	result = (1.0/(2*DELTA)) * result; // divided by 2 delta
	mu.mean(index) += DELTA; // bring mu back

	return result;
}

Eigen::Matrix<ScalarType, 3, 1> StateEstimator::numDiffGyro(StateEstimator::State& mean, int index,  Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam){
	mu.mean(index) += DELTA;
	Eigen::Matrix<ScalarType, 3, 1> result = this->gyroMeasurementFromState(mu, R_imu_2_cam); // high
	mu.mean(index) -= 2*DELTA;
	result.noalias() -= this->gyroMeasurementFromState(mu, R_imu_2_cam); // minus low
	result = (1.0/(2*DELTA)) * result; // divided by 2 delta
	mu.mean(index) += DELTA; // bring mu back

	return result;
}

/*
 * numerically differentiates the imu measurement function and returns the mean result
 */
void StateEstimator::imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 6, BASE_STATE_SIZE>& H, Eigen::Matrix<ScalarType, 6, 1>& z_est, Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam, Eigen::Matrix<ScalarType, 3, 1> r_cam_2_imu){

	// compute the gravity vector in the camera frame
	Eigen::Matrix<ScalarType, 3, 1> gravity;
	gravity << 0, 0, VIO_G;
	gravity = mu.true_pose.unit_quaternion().inverse()*gravity;

	z_est = this->imuMeasurementFromState(mu, R_imu_2_cam, r_cam_2_imu, gravity); // set the mean estimate

	/*for(int j = 0; j < BASE_STATE_SIZE; j++){
		H.block<3, 1>(0, j) = numDiffAccel(mu, j, R_imu_2_cam, r_cam_2_imu, gravity);
		H.block<3, 1>(3, j) = numDiffGyro(mu, j, R_imu_2_cam);
	}*/

	// no correlations
	H.block<6, 3>(0, POSX_INDEX).setZero();
	H.block<6, 3>(0, VELX_INDEX).setZero();

	// simply a rotation with the accel relation being more complex
	H.block<3, 3>(3, OMEGAX_INDEX) = R_imu_2_cam;

	H.block<3, 1>(0, OMEGAX_INDEX) = numDiffAccel(mu, OMEGAX_INDEX, R_imu_2_cam, r_cam_2_imu, gravity);
	H.block<3, 1>(0, OMEGAX_INDEX+1) = numDiffAccel(mu, OMEGAX_INDEX+1, R_imu_2_cam, r_cam_2_imu, gravity);
	H.block<3, 1>(0, OMEGAX_INDEX+2) = numDiffAccel(mu, OMEGAX_INDEX+2, R_imu_2_cam, r_cam_2_imu, gravity);

	// biases are identity
	H.block<3, 3>(0, ACCELBIASX_INDEX).setIdentity();
	H.block<3, 3>(3, GYROBIASX_INDEX).setIdentity();
	H.block<3, 3>(3, ACCELBIASX_INDEX).setZero();
	H.block<3, 3>(0, GYROBIASX_INDEX).setZero();

	H.block<3, 1>(0, THETAX_INDEX) = numDiffAccel(mu, THETAX_INDEX, R_imu_2_cam, r_cam_2_imu, gravity);
	H.block<3, 1>(0, THETAX_INDEX+1) = numDiffAccel(mu, THETAX_INDEX+1, R_imu_2_cam, r_cam_2_imu, gravity);
	H.block<3, 1>(0, THETAX_INDEX+2) = numDiffAccel(mu, THETAX_INDEX+2, R_imu_2_cam, r_cam_2_imu, gravity);
	H.block<3, 3>(3, THETAX_INDEX).setZero();

	// the rest of the params are numerically differentiated
	for(int j = ACCELX_INDEX; j < ACCELBIASX_INDEX; j++){
		H.block<3, 1>(0, j) = numDiffAccel(mu, j, R_imu_2_cam, r_cam_2_imu, gravity);
		H.block<3, 1>(3, j).setZero();
	}

	H.block<3, 1>(0, LAMBDA_INDEX) = numDiffAccel(mu, LAMBDA_INDEX, R_imu_2_cam, r_cam_2_imu, gravity);
	H.block<3, 1>(3, LAMBDA_INDEX).setZero();

}

// simply evaluates the nonlinear measurement function. used for numerical linearization
// order of z: [accel, gyro]
// this is a very nonlinear and complex measurement function
// it may need even more. It may need to estimate the imu_2_cam rotation
// the gravity vector is precomputed
Eigen::Matrix<ScalarType, 6, 1> StateEstimator::imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam, Eigen::Matrix<ScalarType, 3, 1> r_cam_2_imu, Eigen::Matrix<ScalarType, 3, 1> gravity){

	Eigen::Matrix<ScalarType, 6, 1> z_est;

	z_est.block<3, 1>(0, 0) = this->accelMeasurementFromState(mu, R_imu_2_cam, r_cam_2_imu, gravity);
	z_est.block<3, 1>(3, 0) = this->gyroMeasurementFromState(mu, R_imu_2_cam);

	//ROS_DEBUG_STREAM("z_est: " << z_est.transpose());

	return z_est;
}

Eigen::Matrix<ScalarType, 3, 1> StateEstimator::StateEstimator::accelMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam, Eigen::Matrix<ScalarType, 3, 1> r_cam_2_imu, Eigen::Matrix<ScalarType, 3, 1> gravity){
	Eigen::Matrix<ScalarType, 3, 1> z_est;

	z_est.noalias() = mu.getOmega().cross(mu.getOmega().cross(r_cam_2_imu)); // centripetal acceleration in cam frame
	z_est = R_imu_2_cam * z_est; // centripetal acceleration rotated into imu

	gravity = Sophus::SO3<ScalarType>::exp(mu.getPhi()) * gravity; // rotate the gravity vector into the camera frame
	//gravity = Sophus::SO3<ScalarType>::exp(mu.getAngularTwist()).inverse() * gravity; // assuming that the current state is in the tangent space around the current pose

	//temp = Sophus::SO3<ScalarType>::exp(mu.getPhi())*temp; // rotate the gravity vector into the camera frame

	gravity.noalias() += mu.getLambda() * mu.getAcceleration(); // add on the body frame acceleration scaled

	//ROS_DEBUG_STREAM("gravity here: " << gravity);

	gravity = R_imu_2_cam * gravity; // rotate this new acceleration vector into the imu frame

	z_est.noalias() += gravity; // add on the accel

	z_est.noalias() += mu.getAccelBiases(); // tack on the biases

	return z_est;
}
Eigen::Matrix<ScalarType, 3, 1> StateEstimator::StateEstimator::gyroMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam){
	Eigen::Matrix<ScalarType, 3, 1> z_est;

	// compute the angular velocity

	z_est.noalias() = R_imu_2_cam * mu.getOmega();

	z_est.noalias() += mu.getGyroBiases(); // tack on the biases

	return z_est;
}


