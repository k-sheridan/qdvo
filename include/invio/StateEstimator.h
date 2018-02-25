/*
 * TightlyCoupledEKF.h
 *
 *  Created on: Nov 25, 2017
 *      Author: kevin
 */

#ifndef INVIO_INCLUDE_INVIO_STATEESTIMATOR_H_
#define INVIO_INCLUDE_INVIO_STATEESTIMATOR_H_

//State: x, y, z, qw, qx, qy, qz, b_dx, b_dy, b_dz, b_wx, b_wy, b_wz, b_ax, b_ay, b_az, baccx, baccy, baccz, bgyrx, bgyry, bgyrz

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

typedef float ScalarType;

class StateEstimator {
public:
	StateEstimator();

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> mu; // [x y z thetax thetay thetaz dx dy dz wx wy wz ax ay az gx ]
	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> Sigma; // stores the current uncertainty and correlations for the state

	std::list<Feature> features; // store the features in this frame

	ros::Time t; // store the current time of the state

	void initializeState();

	void addNewFeatures(std::vector<Eigen::Vector2f> new_homogenous_features, Frame& f);

	std::vector<Eigen::Vector2f> previousFeaturePositionVector();

	void process(ScalarType dt);

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> generateProcessNoise(ScalarType dt);

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> convolveState(Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1>& last, ScalarType dt);

	Eigen::Vector3f convolveFeature(Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1>& base_state, Eigen::Vector3f& feature_state, ScalarType dt);

	Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> linearizeProcess(Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1>& base_mu, ScalarType dt);

	Eigen::Matrix2f getFeatureHomogenousCovariance(int index);
	ScalarType getFeatureDepthVariance(int index);

	void setFeatureHomogenousCovariance(int index, Eigen::Matrix2f cov);

	void checkSigma();

	void fixSigma();

	double getHuberWeight(double chi);


	Eigen::Matrix<ScalarType, 2, 2> getMetric2PixelMap(Eigen::Matrix3f& K);
	Eigen::Matrix<ScalarType, 2, 2> getPixel2MetricMap(Eigen::Matrix3f& K);



	Eigen::Vector3d getPosition(){return Eigen::Vector3d(mu(0), mu(1), mu(2));}
	Eigen::Vector3d getTheta(){return Eigen::Vector3d(mu(3), mu(4), mu(5));}
	Eigen::Vector3d getVelocity(){return Eigen::Vector3d(mu(6), mu(7), mu(8));}
	Eigen::Vector3d getOmega(){return Eigen::Vector3d(mu(9), mu(10), mu(11));}
	Eigen::Vector3d getAcceleration(){return Eigen::Vector3d(mu(12), mu(13), mu(14));}
	Eigen::Vector3d getPhi(){return Eigen::Vector3d(mu(15), mu(16), mu(17));}
	Eigen::Vector3d getAccelBiases(){return Eigen::Vector3d(mu(18), mu(19), mu(20));}
	Eigen::Vector3d getGyroBiases(){return Eigen::Vector3d(mu(21), mu(22), mu(23));}

	void setPosition(Eigen::Vector3d in){mu(0)=in.mu(); mu(1)=in.y(); mu(2)=in.z();}
	void setTheta(Eigen::Vector3d in){mu(3)=in.mu(); mu(4)=in.y(); mu(5)=in.z();}
	void setVelocity(Eigen::Vector3d in){mu(6)=in.mu(); mu(7)=in.y(); mu(8)=in.z();}
	void setOmega(Eigen::Vector3d in){mu(9)=in.mu(); mu(10)=in.y(); mu(11)=in.z();}
	void setAcceleration(Eigen::Vector3d in){mu(12)=in.mu(); mu(13)=in.y(); mu(14)=in.z();}
	void setPhi(Eigen::Vector3d in){mu(15)=in.mu(); mu(16)=in.y(); mu(17)=in.z();}
	void setAccelBiases(Eigen::Vector3d in){mu(18)=in.mu(); mu(19)=in.y(); mu(20)=in.z();}
	void setGyroBiases(Eigen::Vector3d in){mu(21)=in.mu(); mu(22)=in.y(); mu(23)=in.z();}

};

#endif /* INVIO_INCLUDE_INVIO_STATEESTIMATOR_H_ */
