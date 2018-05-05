/*
 * Feature.h
 *
 *  Created on: Aug 1, 2017
 *      Author: kevin
 */

#ifndef INVIO_INCLUDE_INVIO_FEATURE_H_
#define INVIO_INCLUDE_INVIO_FEATURE_H_

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
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
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Params.h>

#include <sophus/se3.hpp>


class Frame; // need to tell the feature that there is something called frame

class Feature {

private:
	// The feature position is represented in world coordinates
	// mean [x, y, z]
	Eigen::Matrix<ScalarType, 3, 1> mu;
	// covariance of the position estimate
	Eigen::Matrix<ScalarType, 3, 3> Sigma;

	cv::Point2f px; // the current pixel position of this feature

	Eigen::Matrix<ScalarType, 2, 2> R_inv; // METERS! pixel position measurement uncertainty (used to "inform" update about edgy features)

public:


	Feature();
	Feature(cv::Point2f pt, ScalarType depth, ScalarType depth_variance, Eigen::Matrix<ScalarType, 3, 3> K, Sophus::SE3<ScalarType> observation_pose);
	virtual ~Feature();

	Eigen::Matrix<ScalarType, 3, 1> getWorldCoordinate(){return this->mu;}
	void setWorldCoordinate(Eigen::Matrix<ScalarType, 3, 1> point){this->mu = point;}

	Eigen::Matrix<ScalarType, 3, 3> getWorldUncertainty(){return this->Sigma;}
	void setWorldUncertainty(Eigen::Matrix<ScalarType, 3, 3> cov){this->Sigma = cov;}

	cv::Point2f getPx(){return this->px;}
	void setPx(cv::Point2f p){this->px = p;}


	Eigen::Matrix<ScalarType, 3, 1> projectFeature(Sophus::SE3<ScalarType> into_frame);

	void addPatch(cv::Mat patch);

	ScalarType occlusionSSD(cv::Mat test_patch);

	/*
	* extracts the feature patch from the image around the feature position
	*/
	static inline cv::Mat extractPatch(cv::Point2f px, cv::Mat img){
		cv::Rect roi;
		roi.x = px.x - (WINDOW_SIZE/2.0);
		roi.y = px.y - (WINDOW_SIZE/2.0);
		roi.width = WINDOW_SIZE;
		roi.height = WINDOW_SIZE;
		return cv::Mat(img, roi);
	}


	static inline Eigen::Matrix<ScalarType, 2, 1> pixel2Metric(Eigen::Matrix<ScalarType, 3, 3> K, const cv::Point2f px){
		Eigen::Matrix<ScalarType, 2, 1> temp;
		temp << (px.x - K(2)) / K(0), (px.y - K(5)) / K(4);
		return temp;
	}

	static inline cv::Point2f metric2Pixel(Eigen::Matrix<ScalarType, 3, 3> K, Eigen::Matrix<ScalarType, 2, 1> bearing){
		return cv::Point2f(bearing.x()*K(0) + K(2), bearing.y()*K(4) + K(5));
	}

	/*
	 * convert [u,v,zinv] <--> [x,y,z]
	 */
	static inline Eigen::Matrix<ScalarType, 3, 1> bearingAndZinv2Point(Eigen::Matrix<ScalarType, 3, 1> mu){
		mu(2) = 1.0/mu(2);
		mu(0) = mu(2)*mu(0);
		mu(1) = mu(2)*mu(1);
		return mu;
	}
	/*
	 * from
	 */
	static inline Eigen::Matrix<ScalarType, 3, 1> point2bearingAndzinv(Eigen::Matrix<ScalarType, 3, 1> point){
		return Feature::bearingAndZinv2Point(point); // this function works both ways
	}

};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_ */
