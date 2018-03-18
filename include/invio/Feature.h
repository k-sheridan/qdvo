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

#include <Frame.h>
#include <Params.h>

#include <sophus/se3.hpp>


class Frame; // need to tell the feature that there is something called frame

class Feature {
private:

	Eigen::Vector2f bearing; // [u, v] (u and v are in homogenous coord)
	float depth_inv; // [I use an inverse depth paramaterization]
	float depth_inv_sigma; // the variance of this features depth


	//KLT
	Eigen::Vector2f last_result_from_klt_tracker; // used to store the previous feature position in the last frame as local reference for how it looks

public:

	Feature();
	Feature(Eigen::Vector2f homogenous, float depth, Frame& f);
	virtual ~Feature();

	Eigen::Vector2f getBearing();

	float getDepth();

	cv::Point2f getPixel(const Frame& f);

	Eigen::Vector2f getLastResultFromKLTTracker(){
		return this->last_result_from_klt_tracker;
	}

	/*
	 * this should be given in meters
	 */
	void setLastResultFromKLTTracker(Eigen::Vector2f in){
		this->last_result_from_klt_tracker = in;
	}

	void setBearing(Eigen::Vector2f in){
		this->bearing(0) = in(0);
		this->bearing(1) = in(1);
	}

	void setDepth(float depth){
		this->depth_inv = 1.0/depth;
	}

	void setDepthInv(float inv_depth){
		this->depth_inv = inv_depth;
	}

	void setMu(Eigen::Vector3f in){this->bearing(0) = in(0); this->bearing(1) = in(1); this->depth_inv = in(2);}

	Eigen::Vector3f getMu(){return Eigen::Vector3f(this->bearing(0), this->bearing(1), this->depth_inv);}

	Eigen::Vector3f getPoint(){return Eigen::Vector3f(this->bearing(0), this->bearing(1), 1.0/this->depth_inv);}

	float& depth_inv_ref(){return this->depth_inv;}

	float& depth_inv_sigma_ref(){return this->depth_inv_sigma;}


	static inline Eigen::Vector2f pixel2Metric(const Frame& f, const cv::Point2f px){
		return Eigen::Vector2f((px.x - f.K(2)) / f.K(0), (px.y - f.K(5)) / f.K(4));
	}

	static inline cv::Point2f metric2Pixel(const Frame& f, const Eigen::Vector2f pos){
		return cv::Point2f(pos.x()*f.K(0) + f.K(2), pos.y()*f.K(4) + f.K(5));
	}

};

#endif /* PAUVSI_VIO_INCLUDE_PAUVSI_VIO_FEATURE_H_ */
