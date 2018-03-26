#include "../invio/VIO.h"


void VIO::imu_callback(const sensor_msgs::ImuConstPtr& msg){
	ROS_DEBUG_STREAM("got imu message: " << msg->header.stamp);

	// set the time if this is the first message
	if(this->state_estimator.t == ros::Time(0)){
		this->state_estimator.t = msg->header.stamp;
	}

	if(msg->header.stamp >= this->state_estimator.t){

		IMUUpdate us;

		us.msg = *msg; // store this message for potential later use
		us.t = msg->header.stamp;

		this->imu_update_buffer.push_back(us); // add the latest measurement

	}
	else{
		ROS_WARN("imu message is too old to use... ignoring");
	}

	// apply any unused imu messages
	this->applyAllNewIMUMeasurements();

	this->publishOdometry();
}

/*
 * ensures that all measurements have been applied at this point
 */
void VIO::applyAllNewIMUMeasurements(){
	// update state with all "new" measurements
	for(auto& e : this->imu_update_buffer){
		if(!e.applied){ // if this message was not applied
			if(e.t >= this->state_estimator.t){ // is this message measurement
				this->applyIMUUpdate(e);
			}
			else{
				ROS_WARN("imu message is too old to be applied, and was not already applied");
			}
		}
	}
}

/*
 * updates state to current measurement and stores the update state within the ImuUpdate
 */
void VIO::applyIMUUpdate(IMUUpdate& measurement){
	//update the state with this imu message
	//check that this imu message is not from the past
	ScalarType dt = (measurement.msg.header.stamp - this->state_estimator.t).toSec();
	ROS_ASSERT(dt >= 0); // ensure that we don't update wiht an old measurement

	// run process if we need to
	if(dt > 0){
		this->state_estimator.process(dt); // propagate state into the future
	}

	//update the state using this measurment
	Eigen::Matrix<ScalarType, 3, 3> accel_cov, gyro_cov;
	Eigen::Matrix<ScalarType, 3, 1> acc, gyr;

	this->fixImuMessage(measurement.msg, acc, gyr, accel_cov, gyro_cov);

	this->state_estimator.imuUpdate(acc, gyr, accel_cov, gyro_cov, c2imu);

	//set the state and sigma for this update
	measurement.Sigma = this->state_estimator.Sigma;
	measurement.mu = this->state_estimator.mu;
	measurement.applied = true;

	ROS_DEBUG("updated state with imu measurement");
}

/*
 * checks and corrects an IMU message and outputs the corrected evaluates
 */
void VIO::fixImuMessage(sensor_msgs::Imu& msg, Eigen::Matrix<ScalarType, 3, 1>& acc,
		Eigen::Matrix<ScalarType, 3, 1>& gyr, Eigen::Matrix<ScalarType, 3, 3>& accel_cov,
		Eigen::Matrix<ScalarType, 3, 3>&  gyro_cov){
	// fill the gyro covariance units: [rad/s]
	if(USE_CUSTOM_IMU_UNCERTAINTIES){
		gyro_cov.setZero();
		gyro_cov(0, 0) = GYRO_VARIANCE;
		gyro_cov(1, 1) = GYRO_VARIANCE;
		gyro_cov(2, 2) = GYRO_VARIANCE;
	}
	else{

		if(msg.angular_velocity_covariance.at(0) <= 0 || msg.angular_velocity_covariance.at(4) <= 0 || msg.angular_velocity_covariance.at(8) <= 0){
			ROS_ERROR("gyro covariance broken ignoring gyro");
			gyro_cov.setZero();
			gyro_cov(0, 0) = 1e10;
			gyro_cov(1, 1) = 1e10;
			gyro_cov(2, 2) = 1e10;
		}
		else{
			for(int i = 0; i < 9; i++){
				gyro_cov(i) = msg.angular_velocity_covariance.at(i);
			}
		}
	}

	// fill the accel covariance units: [m/s^2]
	if(USE_CUSTOM_IMU_UNCERTAINTIES){
		accel_cov.setZero();
		accel_cov(0, 0) = ACCEL_VARIANCE;
		accel_cov(1, 1) = ACCEL_VARIANCE;
		accel_cov(2, 2) = ACCEL_VARIANCE;
	}
	else{
		if(msg.linear_acceleration_covariance.at(0) <= 0 || msg.linear_acceleration_covariance.at(4) <= 0 || msg.linear_acceleration_covariance.at(8) <= 0){
			ROS_ERROR("gyro covariance broken ignoring accelerometer");
			accel_cov.setZero();
			accel_cov(0, 0) = 1e10;
			accel_cov(1, 1) = 1e10;
			accel_cov(2, 2) = 1e10;
		}
		else{
			for(int i = 0; i < 9; i++){
				accel_cov(i) = msg.linear_acceleration_covariance.at(i);
			}
		}
	}

	acc << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
	gyr << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
}

/*
* finds the closest state behind this time, set it to the current state estimate, delete old the older messages
* , and flag all of the following messages to not applied
*/
void VIO::revertStateBackToClosestIMUUpdate(ros::Time t_next){
	// if there were no imu messages
	if(!this->imu_update_buffer.size()){
		return;
	}

	//apply all un applied imu messages
	this->applyAllNewIMUMeasurements();

	std::deque<IMUUpdate>::iterator chosen_state = this->imu_update_buffer.end()-1; // by default the chose state

	for(std::deque<IMUUpdate>::iterator it = this->imu_update_buffer.begin(); it != this->imu_update_buffer.end(); it++){
		if((t_next - it->t).toSec() < 0){
			ROS_ASSERT(it != this->imu_update_buffer.begin()); // this can't be the first updated state in the buffer

			// the last iterator is the chosen state
			chosen_state = it-1;
			break;
		}
	}

	ROS_DEBUG_STREAM("reverting to state with time: " << chosen_state->t);

	// set the new state estimate
	this->state_estimator.mu = chosen_state->mu;
	this->state_estimator.t = chosen_state->t;
	this->state_estimator.Sigma = chosen_state->Sigma;

	//erase old imu_messages
	this->imu_update_buffer.erase(this->imu_update_buffer.begin(), chosen_state);

	//set all next messages as applied
	for(auto& e : this->imu_update_buffer){
		e.applied = false;
	}

}
