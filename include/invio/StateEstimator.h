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

class StateEstimator {
public:
	StateEstimator();

	struct State{
		//State: 6d twist, bdx, bdy, bdz, bwx, bwy, bwz, bax, bay, baz, phix, phiy, phiz, baccx, baccy, baccz, bgyrx, bgyry, bgyrz, lambda
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, 1> mean;

		Sophus::SE3<ScalarType> true_pose; // this is the true pose wrt the starting point

		// this is the tangent space pose which coincides with the Sigma
		// this is regularly transformed into the tangent space of the current pose
		Eigen::Vector3f getLinearTwist(){return Eigen::Vector3f(mean(POSX_INDEX), mean(POSX_INDEX+1), mean(POSX_INDEX+2));}
		Eigen::Vector3f getAngularTwist(){return Eigen::Vector3f(mean(THETAX_INDEX), mean(THETAX_INDEX+1), mean(THETAX_INDEX+2));}

		Sophus::SE3<ScalarType>::Tangent getTwist(){Eigen::Matrix<ScalarType, 6, 1> temp; temp << getLinearTwist(), getAngularTwist(); return temp;}

		Eigen::Vector3f getVelocity(){return Eigen::Vector3f(mean(VELX_INDEX), mean(VELX_INDEX+1), mean(VELX_INDEX+2));}
		Eigen::Vector3f getOmega(){return Eigen::Vector3f(mean(OMEGAX_INDEX), mean(OMEGAX_INDEX+1), mean(OMEGAX_INDEX+2));}
		Eigen::Vector3f getAcceleration(){return Eigen::Vector3f(mean(ACCELX_INDEX), mean(ACCELX_INDEX+1), mean(ACCELX_INDEX+2));}
		Eigen::Vector3f getPhi(){return Eigen::Vector3f(mean(PHIX_INDEX), mean(PHIX_INDEX+1), mean(PHIX_INDEX+2));}
		Eigen::Vector3f getAccelBiases(){return Eigen::Vector3f(mean(ACCELBIASX_INDEX), mean(ACCELBIASX_INDEX+1), mean(ACCELBIASX_INDEX+2));}
		Eigen::Vector3f getGyroBiases(){return Eigen::Vector3f(mean(GYROBIASX_INDEX), mean(GYROBIASX_INDEX+1), mean(GYROBIASX_INDEX+2));}
		double getLambda(){return mean(LAMBDA_INDEX);}

		void setLinearTwist(Eigen::Vector3f in){const int i = POSX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setAngularTwist(Eigen::Vector3f in){const int i = THETAX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setVelocity(Eigen::Vector3f in){const int i = VELX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setOmega(Eigen::Vector3f in){const int i = OMEGAX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setAcceleration(Eigen::Vector3f in){const int i = ACCELX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setPhi(Eigen::Vector3f in){const int i = PHIX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setAccelBiases(Eigen::Vector3f in){const int i = ACCELBIASX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
		void setGyroBiases(Eigen::Vector3f in){const int i = GYROBIASX_INDEX; mean(i)=in.x(); mean(i+1)=in.y(); mean(i+2)=in.z();}
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

	/*
	 * jacobian which takes a small twist and gives a pixel position change
	 */
	inline static void jacobian_xyz2uv(
			const Eigen::Vector3d& xyz_in_f,
			Eigen::Matrix<double,2,6>& J)
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

	void checkSigma();
	void fixSigma();

	void deleteFeature(std::list<Feature>::iterator it);

	double getHuberWeight(double chi);


	Eigen::Matrix<ScalarType, 2, 2> getMetric2PixelMap(Eigen::Matrix3f& K);
	Eigen::Matrix<ScalarType, 2, 2> getPixel2MetricMap(Eigen::Matrix3f& K);

};

#endif /* INVIO_INCLUDE_INVIO_STATEESTIMATOR_H_ */
