/*
 * FeatureTracker.h
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_
#define PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_

#include <ros/ros.h>

#include <cmath>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <vector>
#include <string>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <sensor_msgs/PointCloud2.h>

#include <sophus/se3.hpp>

#include <Feature.h>
#include <Frame.h>
#include <Params.h>
#include <ParseParams.h>
#include <StateEstimator.h>
#include <KLTTracker.h>

#include <Eigen/Eigenvalues>

#include <stereo_msgs/DisparityImage.h>

class VIO {
public:


	bool initialized; // has the program been initialized
	bool tracking_lost; // do we still have tracking

	std::deque<Frame> frame_buffer; // stores all frames and pose estimates at this frames

	// used to store a state updated between images
	struct IMUUpdate{
		// mu and Sigma may or may not be up to date
		StateEstimator::State mu;
		Eigen::Matrix<ScalarType, BASE_STATE_SIZE, BASE_STATE_SIZE> Sigma;
		ros::Time t;
		sensor_msgs::Imu msg;
		bool applied;

		IMUUpdate(){
			applied = false;
		}
	};

	std::list<IMUUpdate> imu_update_buffer; // store states updated with IMU readings to reduce latency

	std::list<stereo_msgs::DisparityImage> disparity_buffer; // stores disparity messages to be linked with an image. this is only used when the disparity map is computed externally from INVIO

	tf::TransformListener tf_listener;

	StateEstimator state_estimator;

	KLTTracker tracker;

	tf::Transform b2c, c2b, c2imu;

	ros::Subscriber imu_sub, disparity_sub;

	ros::Publisher insight_pub, insight_cinfo_pub, odom_pub, points_pub;

	VIO();

	void imu_callback(const sensor_msgs::ImuConstPtr& msg);

	void findClosestIMUUpdate(ros::Time t);

	void fixImuMessage(sensor_msgs::Imu& msg, Eigen::Matrix<ScalarType, 3, 1>& acc, Eigen::Matrix<ScalarType, 3, 1>& gyr, Eigen::Matrix<ScalarType, 3, 3>& accel_cov, Eigen::Matrix<ScalarType, 3, 3>&  gyro_cov);

	void applyAllNewIMUMeasurements();

	void applyIMUUpdate(IMUUpdate& measurement);

	void camera_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam);

	void disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg);

	void linkFrameAndReplenishFeaturesWithDisparityBuffer();

	void applyDisparityUpdate(stereo_msgs::DisparityImage& d, Frame& frame);

	void addFrame(Frame f);

	void removeExcessFrames(std::deque<Frame>& buffer);

	void replenishFeatures(Frame& f);

	void replenishFeatures(Frame& f, stereo_msgs::DisparityImage& d);

	std::vector<cv::Point2f> extractNewFeatures(Frame& f);

	void applyImageUpdate(Frame& lf, Frame& cf);

	void publishInsight(Frame& f);

	void publishPoints(Frame& f);

	void publishOdometry();

	cv::RotatedRect getErrorEllipse(double chisquare_val, cv::Point2f mean, Eigen::Matrix2f covmat);




};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_VIO_H_ */
