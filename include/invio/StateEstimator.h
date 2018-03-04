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

/*
 * this class provides a hybrid method of bundle adjustment and an ekf to estimate the state of the camera
 * it also allows an imu to be fused into the estimates
 */



class StateEstimator {
public:
	StateEstimator();

	struct State{
		//State: x, y, z, thetax, thetay, thetaz, dx, dy, dz, wx, wy, wz, ax, ay, az, phix, phiy, phiz, baccx, baccy, baccz, bgyrx, bgyry, bgyrz, lambda
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> mean;

		Eigen::Vector3d getPosition(){return Eigen::Vector3d(mean(0), mean(1), mean(2));}
		Eigen::Vector3d getTheta(){return Eigen::Vector3d(mean(3), mean(4), mean(5));}
		Eigen::Vector3d getVelocity(){return Eigen::Vector3d(mean(6), mean(7), mean(8));}
		Eigen::Vector3d getOmega(){return Eigen::Vector3d(mean(9), mean(10), mean(11));}
		Eigen::Vector3d getAcceleration(){return Eigen::Vector3d(mean(12), mean(13), mean(14));}
		Eigen::Vector3d getPhi(){return Eigen::Vector3d(mean(15), mean(16), mean(17));}
		Eigen::Vector3d getAccelBiases(){return Eigen::Vector3d(mean(18), mean(19), mean(20));}
		Eigen::Vector3d getGyroBiases(){return Eigen::Vector3d(mean(21), mean(22), mean(23));}
		double getLambda(){return mean(24);}

		void setPosition(Eigen::Vector3d in){mean(0)=in.x(); mean(1)=in.y(); mean(2)=in.z();}
		void setTheta(Eigen::Vector3d in){mean(3)=in.x(); mean(4)=in.y(); mean(5)=in.z();}
		void setVelocity(Eigen::Vector3d in){mean(6)=in.x(); mean(7)=in.y(); mean(8)=in.z();}
		void setOmega(Eigen::Vector3d in){mean(9)=in.x(); mean(10)=in.y(); mean(11)=in.z();}
		void setAcceleration(Eigen::Vector3d in){mean(12)=in.x(); mean(13)=in.y(); mean(14)=in.z();}
		void setPhi(Eigen::Vector3d in){mean(15)=in.x(); mean(16)=in.y(); mean(17)=in.z();}
		void setAccelBiases(Eigen::Vector3d in){mean(18)=in.x(); mean(19)=in.y(); mean(20)=in.z();}
		void setGyroBiases(Eigen::Vector3d in){mean(21)=in.x(); mean(22)=in.y(); mean(23)=in.z();}
		void setLambda(double in){mean(24) = in;}

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

	void updateWithFeaturePositions(std::vector<Eigen::Vector2f> measured_positions, std::vector<Eigen::Matrix<ScalarType, 2, 2> > estimated_covariance, std::vector<bool> pass);

	void updateWithIMU(Eigen::Matrix<ScalarType, 3, 1> accel, Eigen::Matrix<ScalarType, 3, 1> gryo, Eigen::Matrix<ScalarType, 3, 3>& accel_cov, Eigen::Matrix<ScalarType, 3, 3>& gyro_cov, tf::Transform& c2imu);

	void imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 6, BASE_STATE_SIZE>& H, Eigen::Matrix<ScalarType, 6, 1>& z_est, tf::Transform& c2imu);
	void imuMeasurementFromState(StateEstimator::State& mu, Eigen::Matrix<ScalarType, 6, 1>& z_est, tf::Transform& c2imu);

	void checkSigma();

	void fixSigma();

	double getHuberWeight(double chi);


	Eigen::Matrix<ScalarType, 2, 2> getMetric2PixelMap(Eigen::Matrix3f& K);
	Eigen::Matrix<ScalarType, 2, 2> getPixel2MetricMap(Eigen::Matrix3f& K);

};

#endif /* INVIO_INCLUDE_INVIO_STATEESTIMATOR_H_ */
