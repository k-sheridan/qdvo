/*
 * TightlyCoupledEKF.h
 *
 *  Created on: Nov 25, 2017
 *      Author: kevin
 */

#ifndef INVIO_INCLUDE_INVIO_STATEESTIMATOR_H_
#define INVIO_INCLUDE_INVIO_STATEESTIMATOR_H_


#include <Feature.h>
#include <Frame.h>
#include <Params.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/LU>
#include <Eigen/SparseCholesky>
#include <Eigen/Cholesky>

#include <sophus/se3.hpp>
#include <sophus/common.hpp>
#include <sophus/types.hpp>

/*
 * this class provides a hybrid method of bundle adjustment and an ekf to estimate the state of the camera
 * it also allows an imu to be fused into the estimates
 */

#define POSX_INDEX 0
#define THETAX_INDEX 3
#define VELX_INDEX 6
#define OMEGAX_INDEX 9
#define ACCELX_INDEX 12
#define PHIX_INDEX 15
#define ACCELBIASX_INDEX 18
#define GYROBIASX_INDEX 21
#define LAMBDA_INDEX 24

#define DELTA (ScalarType)1e-3

class StateEstimator {
public:
	StateEstimator();

	struct State{
		//note: lambda is the scale that the body frame acceleration undergoes to model the accelerometer measurement

		//State: 6d twist, bdx, bdy, bdz, bwx, bwy, bwz, bax, bay, baz, phix, phiy, phiz, baccx, baccy, baccz, bgyrx, bgyry, bgyrz, lambda
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> mean;

		Sophus::SE3<ScalarType> true_pose; // this is the true pose wrt the starting point

		// this is the tangent space pose which coincides with the Sigma
		// this is regularly transformed into the tangent space of the current pose
		Eigen::Matrix<ScalarType, 3, 1> getLinearTwist(){return Eigen::Matrix<ScalarType, 3, 1>(mean(POSX_INDEX), mean(POSX_INDEX+1), mean(POSX_INDEX+2));}
		Eigen::Matrix<ScalarType, 3, 1> getAngularTwist(){return Eigen::Matrix<ScalarType, 3, 1>(mean(THETAX_INDEX), mean(THETAX_INDEX+1), mean(THETAX_INDEX+2));}

		Sophus::SE3<ScalarType>::Tangent getTwist(){Eigen::Matrix<ScalarType, 6, 1> temp; temp << getLinearTwist(), getAngularTwist(); return temp;}

		Eigen::Matrix<ScalarType, 3, 1> getVelocity(){return Eigen::Matrix<ScalarType, 3, 1>(mean(VELX_INDEX), mean(VELX_INDEX+1), mean(VELX_INDEX+2));}
		Eigen::Matrix<ScalarType, 3, 1> getOmega(){return Eigen::Matrix<ScalarType, 3, 1>(mean(OMEGAX_INDEX), mean(OMEGAX_INDEX+1), mean(OMEGAX_INDEX+2));}
		Eigen::Matrix<ScalarType, 3, 1> getAcceleration(){return Eigen::Matrix<ScalarType, 3, 1>(mean(ACCELX_INDEX), mean(ACCELX_INDEX+1), mean(ACCELX_INDEX+2));}
		Eigen::Matrix<ScalarType, 3, 1> getPhi(){return Eigen::Matrix<ScalarType, 3, 1>(mean(PHIX_INDEX), mean(PHIX_INDEX+1), mean(PHIX_INDEX+2));}
		Eigen::Matrix<ScalarType, 3, 1> getAccelBiases(){return Eigen::Matrix<ScalarType, 3, 1>(mean(ACCELBIASX_INDEX), mean(ACCELBIASX_INDEX+1), mean(ACCELBIASX_INDEX+2));}
		Eigen::Matrix<ScalarType, 3, 1> getGyroBiases(){return Eigen::Matrix<ScalarType, 3, 1>(mean(GYROBIASX_INDEX), mean(GYROBIASX_INDEX+1), mean(GYROBIASX_INDEX+2));}
		double getLambda(){return mean(LAMBDA_INDEX);}

		void setLinearTwist(Eigen::Matrix<ScalarType, 3, 1> in){const int i = POSX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setAngularTwist(Eigen::Matrix<ScalarType, 3, 1> in){const int i = THETAX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setVelocity(Eigen::Matrix<ScalarType, 3, 1> in){const int i = VELX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setOmega(Eigen::Matrix<ScalarType, 3, 1> in){const int i = OMEGAX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setAcceleration(Eigen::Matrix<ScalarType, 3, 1> in){const int i = ACCELX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setPhi(Eigen::Matrix<ScalarType, 3, 1> in){const int i = PHIX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setAccelBiases(Eigen::Matrix<ScalarType, 3, 1> in){const int i = ACCELBIASX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setGyroBiases(Eigen::Matrix<ScalarType, 3, 1> in){const int i = GYROBIASX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setLambda(double in){mean(LAMBDA_INDEX) = in;}

	} mu;

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> Sigma; // stores the current uncertainty and correlations for the state
	ros::Time t; // store the current time of the state

	void initializeState();

	void process(ScalarType dt);

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> generateProcessNoise(ScalarType dt);

	State convolveState(State& last, ScalarType dt);

	Eigen::Matrix<ScalarType, 3, 1> convolveFeatures(State& current_state, ScalarType dt);

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> linearizeProcess(ScalarType dt);

	void motionOptimization(Frame& cf);

	void structureOptimization(Frame& cf);

	void removeOutliers(Frame& cf, ScalarType threshold);

	// gyro update functions
	Eigen::Matrix<ScalarType, 3, BASE_STATE_SIZE> gyroMeasurementMap(Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam);

	Eigen::Matrix<ScalarType, 3, 1> gyroMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam);

	void gyroUpdate(Eigen::Matrix<ScalarType, 3, 1> gryo, Eigen::Matrix<ScalarType, 3, 3>& gyro_cov, tf::Transform& c2imu);


	// full imu update
	void fullImuUpdate(Eigen::Matrix<ScalarType, 3, 1> accel, Eigen::Matrix<ScalarType, 3, 1> gryo, Eigen::Matrix<ScalarType, 3, 3>& accel_cov, Eigen::Matrix<ScalarType, 3, 3>& gyro_cov, tf::Transform& c2imu);

	void imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 6, BASE_STATE_SIZE>& H, Eigen::Matrix<ScalarType, 6, 1>& z_est, Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam, Eigen::Matrix<ScalarType, 3, 1> r_cam_2_imu);

	Eigen::Matrix<ScalarType, 6, 1> imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam, Eigen::Matrix<ScalarType, 3, 1> r_cam_2_imu, Eigen::Matrix<ScalarType, 3, 1> G_transformed);

	Eigen::Matrix<ScalarType, 3, 1> accelMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam, Eigen::Matrix<ScalarType, 3, 1> r_cam_2_imu, Eigen::Matrix<ScalarType, 3, 1> gravity);

	Eigen::Matrix<ScalarType, 3, 1> numDiffAccel(StateEstimator::State& mean, int index,  Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam, Eigen::Matrix<ScalarType, 3, 1> r_cam_2_imu, Eigen::Matrix<ScalarType, 3, 1> G_transformed);
	Eigen::Matrix<ScalarType, 3, 1> numDiffGyro(StateEstimator::State& mean, int index,  Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam);

	/*
	Eigen::Matrix<ScalarType, 6, 1> StateEstimator::numDiffAccel(StateEstimator::State& mean, int index,  Eigen::Matrix<ScalarType, 3, 3> R_imu_2_cam);
	 * jacobian which takes a small twist and gives a pixel position change
	 */
	inline static void twist2uv(const Eigen::Matrix<ScalarType, 3, 1>& xyz_in_f, Eigen::Matrix<ScalarType, 2, 6>& J)
	{
		const double x = xyz_in_f[0];
		const double y = xyz_in_f[1];
		const double z_inv = 1./xyz_in_f[2];
		const double z_inv_2 = z_inv*z_inv;

		J(0,0) = -z_inv;              // -1/z
		J(0,1) = 0.0;                 // 0
		J(0,2) = x*z_inv_2;           // x/z^2
		J(0,3) = y*J(0,2);            // x*y/z^2
		J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
		J(0,5) = y*z_inv;             // y/z

		J(1,0) = 0.0;                 // 0
		J(1,1) = -z_inv;              // -1/z
		J(1,2) = y*z_inv_2;           // y/z^2
		J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
		J(1,4) = -J(0,3);             // -x*y/z^2
		J(1,5) = -x*z_inv;            // x/z
	}

	/*
	 * relates a small change in feature position to a small change in bearing
	 */
	inline static void featurePosition2uv(const Eigen::Matrix<ScalarType, 3, 1>& xyz_in_f, Eigen::Matrix<ScalarType, 2, 3>& H){
		const double x = xyz_in_f[0];
		const double y = xyz_in_f[1];
		const double z_inv = 1./xyz_in_f[2];
		const double z_inv_2 = z_inv*z_inv;

		H(0, 0) = z_inv;
		H(0, 1) = 0;
		H(0, 2) = -z_inv_2 * x;
		H(1, 0) = 0;
		H(1, 1) = z_inv;
		H(1, 2) = -z_inv_2 * y;
	}


	ScalarType getHuberWeight(ScalarType chi_abs);


};

#endif /* INVIO_INCLUDE_INVIO_STATEESTIMATOR_H_ */
