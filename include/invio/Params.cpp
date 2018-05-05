/*
 * Params.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: kevin
 */


#include "Params.h"

bool PUBLISH_INSIGHT;
std::string INSIGHT_TOPIC;
std::string INSIGHT_CINFO_TOPIC;
int MAX_VARIANCE_SIZE, MIN_VARIANCE_SIZE;
int FAST_THRESHOLD;
double FAST_BLUR_SIGMA;
double INVERSE_IMAGE_SCALE;
int KILL_PAD;
double KLT_MIN_EIGEN;
double MIN_NEW_FEATURE_DIST;
int NUM_FEATURES;
int START_FEATURE_COUNT;
int MINIMUM_TRACKABLE_FEATURES;
int FRAME_BUFFER_SIZE;
double MAXIMUM_REPROJECTION_ERROR;
double MAXIMUM_CANDIDATE_REPROJECTION_ERROR;
double MOBA_CANDIDATE_VARIANCE;
double DEFAULT_POINT_DEPTH;
double DEFAULT_POINT_DEPTH_VARIANCE;
double DEFAULT_POINT_HOMOGENOUS_VARIANCE;
double EPS_MOBA;
double HUBER_WIDTH;
int MOBA_MAX_ITERATIONS;
double MAX_POINT_Z;
double MIN_POINT_Z;
//double MAX_RANGE_PER_DEPTH;
int WINDOW_SIZE;
int MAX_PYRAMID_LEVEL;
std::string ODOM_TOPIC;
std::string POINTS_PUB_TOPIC;
std::string ODOM_FRAME;
std::string CAMERA_FRAME;
std::string CAMERA_TOPIC;
std::string BASE_FRAME;
std::string WORLD_FRAME;
bool USE_IMU;
std::string IMU_TOPIC;
std::string IMU_FRAME;
bool USE_EXTERNAL_DISPARITY;
std::string EXTERNAL_DISPARITY_TOPIC;
double GYRO_VARIANCE;
double ACCEL_VARIANCE;
bool USE_CUSTOM_IMU_UNCERTAINTIES;
int REFERENCE_PATCH_DEPTH;
double OCCLUSION_THRESHOLD;


void parseROSParams(){
	//parseROSParams();
	ros::param::param<bool>("~publish_insight", PUBLISH_INSIGHT, D_PUBLISH_INSIGHT);
	ros::param::param<std::string>("~insight_topic", INSIGHT_TOPIC, D_INSIGHT_TOPIC);
	ros::param::param<std::string>("~insight_camera_info_topic", INSIGHT_CINFO_TOPIC, D_INSIGHT_CINFO_TOPIC);
	ros::param::param<int>("~fast_threshold", FAST_THRESHOLD, D_FAST_THRESHOLD);
	ros::param::param<double>("~fast_blur_sigma", FAST_BLUR_SIGMA, D_FAST_BLUR_SIGMA);
	ros::param::param<double>("~inverse_image_scale", INVERSE_IMAGE_SCALE, D_INVERSE_IMAGE_SCALE);
	ros::param::param<int>("~kill_pad", KILL_PAD, D_KILL_PAD);
	ros::param::param<double>("~min_klt_eigen_val", KLT_MIN_EIGEN, D_KLT_MIN_EIGEN);
	ros::param::param<double>("~min_new_feature_dist", MIN_NEW_FEATURE_DIST, D_MIN_NEW_FEATURE_DIST);
	ros::param::param<int>("~num_features", NUM_FEATURES, D_NUM_FEATURES);
	ros::param::param<int>("~start_feature_count", START_FEATURE_COUNT, D_START_FEATURE_COUNT);
	ros::param::param<int>("~minimum_trackable_features", MINIMUM_TRACKABLE_FEATURES, D_MINIMUM_TRACKABLE_FEATURES);
	ros::param::param<int>("~frame_buffer_size", FRAME_BUFFER_SIZE, D_FRAME_BUFFER_SIZE);
	ros::param::param<double>("~maximum_reprojection_error", MAXIMUM_REPROJECTION_ERROR, D_MAXIMUM_REPROJECTION_ERROR);
	ros::param::param<double>("~moba_candidate_variance", MOBA_CANDIDATE_VARIANCE, D_MOBA_CANDIDATE_VARIANCE);
	ros::param::param<double>("~maximum_candidate_reprojection_error", MAXIMUM_CANDIDATE_REPROJECTION_ERROR, D_MAXIMUM_CANDIDATE_REPROJECTION_ERROR);
	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);
	ros::param::param<double>("~default_point_depth_variance", DEFAULT_POINT_DEPTH_VARIANCE, D_DEFAULT_POINT_DEPTH_VARIANCE);
	ros::param::param<double>("~default_point_homogenous_variance", DEFAULT_POINT_HOMOGENOUS_VARIANCE, D_DEFAULT_POINT_HOMOGENOUS_VARIANCE);
	ros::param::param<double>("~eps_moba", EPS_MOBA, D_EPS_MOBA);
	ros::param::param<double>("~huber_width", HUBER_WIDTH, D_HUBER_WIDTH);
	ros::param::param<int>("~moba_max_iterations", MOBA_MAX_ITERATIONS, D_MOBA_MAX_ITERATIONS);
	ros::param::param<double>("~max_point_z", MAX_POINT_Z, D_MAX_POINT_Z);
	ros::param::param<double>("~min_point_z", MIN_POINT_Z, D_MIN_POINT_Z);
	ros::param::param<std::string>("~odom_topic", ODOM_TOPIC, D_ODOM_TOPIC);
	ros::param::param<std::string>("~odom_frame", ODOM_FRAME, D_ODOM_FRAME);
	ros::param::param<std::string>("~point_pub_topic", POINTS_PUB_TOPIC, D_POINTS_PUB_TOPIC);
	ros::param::param<std::string>("~camera_topic", CAMERA_TOPIC, D_CAMERA_TOPIC);
	ros::param::param<std::string>("~base_frame", BASE_FRAME, D_BASE_FRAME);
	ros::param::param<std::string>("~world_frame", WORLD_FRAME, D_WORLD_FRAME);
	ros::param::param<std::string>("~camera_frame", CAMERA_FRAME, D_CAMERA_FRAME);
	ros::param::param<std::string>("~imu_topic", IMU_TOPIC, D_IMU_TOPIC);
	ros::param::param<std::string>("~imu_frame", IMU_FRAME, D_IMU_FRAME);
	ros::param::param<bool>("~use_imu", USE_IMU, D_USE_IMU);
	ros::param::param<bool>("~use_custom_imu_variances", USE_CUSTOM_IMU_UNCERTAINTIES, D_USE_CUSTOM_IMU_UNCERTAINTIES);

	ros::param::param<bool>("~use_external_disparity", USE_EXTERNAL_DISPARITY, D_USE_EXTERNAL_DISPARITY);
	ros::param::param<std::string>("~disparity_topic", EXTERNAL_DISPARITY_TOPIC, D_EXTERNAL_DISPARITY_TOPIC);

	ros::param::param<double>("~gyro_variance", GYRO_VARIANCE, D_GYRO_VARIANCE);
	ros::param::param<double>("~accel_variance", ACCEL_VARIANCE, D_ACCEL_VARIANCE);
	ros::param::param<int>("~min_variance_box_size", MIN_VARIANCE_SIZE, D_MIN_VARIANCE_SIZE);
	ros::param::param<int>("~max_variance_box_size", MAX_VARIANCE_SIZE, D_MAX_VARIANCE_SIZE);
	ros::param::param<int>("~max_pyramids", MAX_PYRAMID_LEVEL, D_MAX_PYRAMID_LEVEL);
	ros::param::param<int>("~klt_window_size", WINDOW_SIZE, D_WINDOW_SIZE);
	ros::param::param<int>("~reference_patch_depth", REFERENCE_PATCH_DEPTH, D_REFERENCE_PATCH_DEPTH);
	ros::param::param<double>("~occlusion_threshold", OCCLUSION_THRESHOLD, D_OCCLUSION_THRESHOLD);
}
