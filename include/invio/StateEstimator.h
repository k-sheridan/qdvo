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
#define QUATW_INDEX 0
#define VELX_INDEX 0
#define OMEGAX_INDEX 0
#define ACCELX_INDEX 0
#define PHIX_INDEX 0
#define ACCELBIASX_INDEX 0
#define GYROBIASX_INDEX 0
#define LAMBDA_INDEX 0

class StateEstimator {
public:
	StateEstimator();

	struct State{
		//State: x, y, z, qw, qx, qy, qz, dx, dy, dz, wx, wy, wz, ax, ay, az, phix, phiy, phiz, baccx, baccy, baccz, bgyrx, bgyry, bgyrz, lambda
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> mean;

		Eigen::Vector3d getPosition(){return Eigen::Vector3d(mean(POSX_INDEX), mean(POSX_INDEX+1), mean(POSX_INDEX+2));}
		Eigen::Quaterniond getQuat(){return Eigen::Quaterniond(mean(QUATW_INDEX), mean(QUATW_INDEX+1), mean(QUATW_INDEX+2), mean(QUATW_INDEX+3));}
		Eigen::Vector3d getVelocity(){return Eigen::Vector3d(mean(VELX_INDEX), mean(VELX_INDEX+1), mean(VELX_INDEX+2));}
		Eigen::Vector3d getOmega(){return Eigen::Vector3d(mean(OMEGAX_INDEX), mean(OMEGAX_INDEX+1), mean(OMEGAX_INDEX+2));}
		Eigen::Vector3d getAcceleration(){return Eigen::Vector3d(mean(ACCELX_INDEX), mean(ACCELX_INDEX+1), mean(ACCELX_INDEX+2));}
		Eigen::Vector3d getPhi(){return Eigen::Vector3d(mean(PHIX_INDEX), mean(PHIX_INDEX+1), mean(PHIX_INDEX+2));}
		Eigen::Vector3d getAccelBiases(){return Eigen::Vector3d(mean(ACCELBIASX_INDEX), mean(ACCELBIASX_INDEX+1), mean(ACCELBIASX_INDEX+2));}
		Eigen::Vector3d getGyroBiases(){return Eigen::Vector3d(mean(GYROBIASX_INDEX), mean(GYROBIASX_INDEX+1), mean(GYROBIASX_INDEX+2));}
		double getLambda(){return mean(LAMBDA_INDEX);}

		void setPosition(Eigen::Vector3d in){const int i = POSX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setQuat(Eigen::Quaterniond in){const int i = QUATW_INDEX; mean(i)=in.w(); mean(i+1)=in.x(); mean(i+2)=in.y(); mean(i+3)=in.z();}
		void setVelocity(Eigen::Vector3d in){const int i = VELX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setOmega(Eigen::Vector3d in){const int i = OMEGAX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setAcceleration(Eigen::Vector3d in){const int i = ACCELX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setPhi(Eigen::Vector3d in){const int i = PHIX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setAccelBiases(Eigen::Vector3d in){const int i = ACCELBIASX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setGyroBiases(Eigen::Vector3d in){const int i = GYROBIASX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setLambda(double in){mean(LAMBDA_INDEX) = in;}

	} mu;

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> Sigma; // stores the current uncertainty and correlations for the state

	std::list<Feature> features; // store the features in this frame

	ros::Time t; // store the current time of the state

	void initializeState();

	void addNewFeatures(std::vector<Eigen::Vector2f> new_homogenous_features, Frame& f);

	std::vector<Eigen::Vector2f> previousFeaturePositionVector();

	void process(ScalarType dt);

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> generateProcessNoise(ScalarType dt);

	State convolveState(State& last, ScalarType dt);

	Eigen::Vector3f convolveFeatures(State& current_state, ScalarType dt);

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> linearizeProcess(ScalarType dt);

	void updateWithFeaturePositions(std::vector<Eigen::Vector2f> measured_positions, std::vector<Eigen::Matrix<ScalarType, 2, 2> > estimated_covariance, std::vector<bool> pass, ros::Time t);

	void updateWithIMU(Eigen::Matrix<ScalarType, 3, 1> accel, Eigen::Matrix<ScalarType, 3, 1> gryo, Eigen::Matrix<ScalarType, 3, 3>& accel_cov, Eigen::Matrix<ScalarType, 3, 3>& gyro_cov, tf::Transform& c2imu);

	void imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 6, BASE_STATE_SIZE>& H, Eigen::Matrix<ScalarType, 6, 1>& z_est, tf::Transform& c2imu);
	void imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 6, 1>& z_est, tf::Transform& c2imu);

	Eigen::Matrix<ScalarType, 6, 7> linearizeExponentialMap(Sophus::SE3<ScalarType> pose, float delta);
	Eigen::Matrix<ScalarType, 7, 6> linearizeLogarithmMap(Eigen::Matrix<ScalarType, 6, 1> tangent, float delta);

	void checkSigma();
	void fixSigma();

	void deleteFeature(std::list<Feature>::iterator it);

	double getHuberWeight(double chi);


	Eigen::Matrix<ScalarType, 2, 2> getMetric2PixelMap(Eigen::Matrix3f& K);
	Eigen::Matrix<ScalarType, 2, 2> getPixel2MetricMap(Eigen::Matrix3f& K);

};

#endif /* INVIO_INCLUDE_INVIO_STATEESTIMATOR_H_ */
