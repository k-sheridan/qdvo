/*
 * VIO.cpp
 *
 *  Created on: Jul 8, 2017
 *      Author: kevin
 */

#include "../invio/VIO.h"

VIO::VIO() {

	//set uninitialized
	this->initialized = false;
	//set tracking lost to false initially
	this->tracking_lost = false;

	ros::NodeHandle nh; // we all know what this is

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
	ros::param::param<double>("~gyro_variance", GYRO_VARIANCE, D_GYRO_VARIANCE);
	ros::param::param<double>("~accel_variance", ACCEL_VARIANCE, D_ACCEL_VARIANCE);
	ros::param::param<int>("~min_variance_box_size", MIN_VARIANCE_SIZE, D_MIN_VARIANCE_SIZE);
	ros::param::param<int>("~max_variance_box_size", MAX_VARIANCE_SIZE, D_MAX_VARIANCE_SIZE);
	ros::param::param<int>("~max_pyramids", MAX_PYRAMID_LEVEL, D_MAX_PYRAMID_LEVEL);
	ros::param::param<int>("~klt_window_size", WINDOW_SIZE, D_WINDOW_SIZE);
	ros::param::param<int>("~depth_solver_patch_size", DEPTH_SOLVER_PATCH_SIZE, D_DEPTH_SOLVER_PATCH_SIZE);

	image_transport::ImageTransport it(nh);
	image_transport::CameraSubscriber bottom_cam_sub = it.subscribeCamera(
			CAMERA_TOPIC, 10, &VIO::camera_callback, this);

	if(PUBLISH_INSIGHT){
		this->insight_pub = nh.advertise<sensor_msgs::Image>(INSIGHT_TOPIC, 1);
		this->insight_cinfo_pub = nh.advertise<sensor_msgs::CameraInfo>(INSIGHT_CINFO_TOPIC, 1);
	}

	//set up IMU sub
	if(USE_IMU){
		this->imu_sub = nh.subscribe(IMU_TOPIC, 1000, &VIO::imu_callback, this);

		// c2imu
		tf::StampedTransform c2i_st;
		ROS_INFO_STREAM("WAITING FOR TANSFORM FROM " << CAMERA_FRAME << " TO " << IMU_FRAME);
		if(this->tf_listener.waitForTransform(CAMERA_FRAME, IMU_FRAME, ros::Time(0), ros::Duration(10))){
			try {
				this->tf_listener.lookupTransform(CAMERA_FRAME, IMU_FRAME,
						ros::Time(0), c2i_st);
			} catch (tf::TransformException& e) {
				ROS_WARN_STREAM(e.what());
			}

			this->c2imu = tf::Transform(c2i_st);
		}
		else
		{
			ROS_FATAL("COULD NOT GET TRANSFORM");
			ros::shutdown();
			return;
		}
		ROS_INFO("got transform");

	}

	this->odom_pub = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 1);

	this->points_pub = nh.advertise<sensor_msgs::PointCloud>(POINTS_PUB_TOPIC, 1);



	//get the b2c transform
	tf::StampedTransform b2c_st;
	ROS_INFO_STREAM("WAITING FOR TANSFORM FROM " << BASE_FRAME << " TO " << CAMERA_FRAME);
	if(this->tf_listener.waitForTransform(BASE_FRAME, CAMERA_FRAME, ros::Time(0), ros::Duration(10))){
		try {
			this->tf_listener.lookupTransform(BASE_FRAME, CAMERA_FRAME,
					ros::Time(0), b2c_st);
		} catch (tf::TransformException& e) {
			ROS_WARN_STREAM(e.what());
		}

		this->b2c = tf::Transform(b2c_st);
		this->c2b = this->b2c.inverse();
	}
	else
	{
		ROS_FATAL("COULD NOT GET TRANSFORM");
		ros::shutdown();
		return;
	}
	ROS_INFO("got transform");


	// start the callbacks
	ros::spin();
}

void VIO::addFrame(Frame f) {

	//TODO revert the state to the most recent IMU message

	if (this->frame_buffer.size() == 0) // if this is the first frame that we are receiving
	{
		ROS_DEBUG("adding the first frame");
		//f.setPose(Frame::tf2sophus(b2c)); // set the initial position to 0 (this is world to camera)

		this->frame_buffer.push_front(f); // add the frame to the front of the buffer

		// set the time if this is the first message
		if(this->state_estimator.t == ros::Time(0)){
			this->state_estimator.t = f.t;
		}

		this->replenishFeatures((this->frame_buffer.front()));
	}

	else // we have atleast 1 frame in the buffer
	{

		this->frame_buffer.push_front(f); // add the frame to the front of the buffer

		//set the predicted pose of the current frame
		float dt = (f.t - this->state_estimator.t).toSec();

		ROS_ASSERT(dt >= 0);
		this->state_estimator.process(dt);
		this->state_estimator.t = f.t;

		//update the frame's position estimate with the predicted
		this->frame_buffer.front().pose = this->state_estimator.mu.true_pose;

		if(this->state_estimator.features.size()) // run update if we have enough features
		{
			// attempt to flow features into the next frame if there are features
			// then perform iterative pose update
			// then apply depth update
			this->applyImageUpdate(this->frame_buffer.at(1), this->frame_buffer.front());
		}

		this->replenishFeatures((this->frame_buffer.front())); // try to get more features if needed
	}


	// publish visualization info

	if( PUBLISH_INSIGHT)
	{
		if(this->frame_buffer.size() > 0)
		{
			this->publishInsight(this->frame_buffer.front());
		}
	}

	// publish odometry
	this->publishOdometry(this->frame_buffer.front());

	//publish the mature 3d points
	this->publishPoints(this->frame_buffer.front());

	ROS_ERROR_COND(this->tracking_lost, "lost tracking!");

	//finally remove excess frames from the buffer
	this->removeExcessFrames(this->frame_buffer);
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

	std::deque<IMUUpdate>::iterator chosen_state = this->imu_update_buffer.back(); // by default the chose state

	for(std::deque<IMUUpdate>::iterator it = this->imu_update_buffer.begin(); it != this->imu_update_buffer.end(); it++){
		if((t_next - *it.t).toSec() < 0){
			ROS_ASSERT(it != this->imu_update_buffer.begin()); // this can't be the first updated state in the buffer
			ROS_ASSERT();

			// the last iterator is the chosen state
			chosen_state = it;
		}
	}

	// set the new state estimate
	this->state_estimator.mu = *chosen_state.mu;
	this->state_estimator.t = *chosen_state.t;
	this->state_estimator.Sigma = *chosen_state.Sigma;

	//TODO remove old messages

	//TODO set all next messages as applied

}

void VIO::removeExcessFrames(std::deque<Frame>& buffer)
{
	// remove the last element if the buffer is larger than the desired size
	if(buffer.size() > (size_t)FRAME_BUFFER_SIZE)
	{
		buffer.pop_back();
	}
}

/*
 * covariance and mean must be in pixels
 */
cv::RotatedRect VIO::getErrorEllipse(double chisquare_val, cv::Point2f mean, Eigen::Matrix2f eig_covmat){

	//Get the eigenvalues and eigenvectors
	Eigen::EigenSolver<Eigen::Matrix2f> es;
	es.compute(eig_covmat, true);

	Eigen::EigenSolver<Eigen::Matrix2f>::EigenvalueType eig_vals = es.eigenvalues();

	if(es.info() != Eigen::ComputationInfo::Success)
	{
		ROS_DEBUG_STREAM("eigen vals and or vecs not computed: " << eig_covmat);
		return cv::RotatedRect(mean, cv::Size2f(10, 10), 0);
	}

	Eigen::EigenSolver<Eigen::Matrix2f>::EigenvectorsType eig_vecs = es.eigenvectors();

	double angle;
	double halfmajoraxissize;
	double halfminoraxissize;

	if(eig_vals(0).real() > eig_vals(1).real())
	{
		//Calculate the angle between the largest eigenvector and the x-axis
		angle = atan2(eig_vecs(1,0).real(), eig_vecs(0,0).real());

		//Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
		if(angle < 0)
			angle += 6.28318530718;

		//Conver to degrees instead of radians
		angle = 180*angle/3.14159265359;

		//Calculate the size of the minor and major axes
		halfmajoraxissize=chisquare_val*sqrt(eig_vals(0).real());
		halfminoraxissize=chisquare_val*sqrt(eig_vals(1).real());
	}
	else
	{
		//Calculate the angle between the largest eigenvector and the x-axis
		angle = atan2(eig_vecs(1,1).real(), eig_vecs(0,1).real());

		//Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
		if(angle < 0)
			angle += 6.28318530718;

		//Conver to degrees instead of radians
		angle = 180*angle/3.14159265359;

		//Calculate the size of the minor and major axes
		halfmajoraxissize=chisquare_val*sqrt(eig_vals(1).real());
		halfminoraxissize=chisquare_val*sqrt(eig_vals(0).real());
	}


	halfmajoraxissize = std::max(halfmajoraxissize, 0.1);
	halfminoraxissize = std::max(halfminoraxissize, 0.1);

	//Return the oriented ellipse
	//The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
	return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);

}

void VIO::publishInsight(Frame& f)
{
	cv::Mat img;

	cv::cvtColor(f.img, img, CV_GRAY2BGR);

	int i = 0; // track the feature count
	for(auto& e : f.features)
	{

		//ROS_DEBUG_STREAM(e.getPixel(f));
		cv::drawMarker(img, e.getPixel(f), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 22, 1);

		//ROS_DEBUG_STREAM("plotting covariance in pixels: " << this->state_estimator.getMetric2PixelMap(f.K)*this->state_estimator.getFeatureHomogenousCovariance(i)*this->state_estimator.getMetric2PixelMap(f.K).transpose());
		//Eigen::SparseMatrix<float> J = this->state_estimator.getMetric2PixelMap(f.K);

		//cv::RotatedRect rr = this->getErrorEllipse(0.99, e.getPixel(f), J*this->state_estimator.getFeatureHomogenousCovariance(i)*J);
		//ROS_DEBUG_STREAM(rr.size);
		//cv::ellipse(img, rr, cv::Scalar(255, 255, 0), 1);

		// next feature
		i++;
	}

	sensor_msgs::CameraInfo cinfo;

	cinfo.header.frame_id = ODOM_FRAME;
	cinfo.header.stamp = f.t;

	cinfo.height = img.rows;
	cinfo.width = img.cols;

	cinfo.K.at(0) = f.K(0);
	cinfo.K.at(1) = f.K(1);
	cinfo.K.at(2) = f.K(2);
	cinfo.K.at(3) = f.K(3);
	cinfo.K.at(4) = f.K(4);
	cinfo.K.at(5) = f.K(5);
	cinfo.K.at(6) = f.K(6);
	cinfo.K.at(7) = f.K(7);
	cinfo.K.at(8) = f.K(8);

	//TODO make it the actual projection mat
	cinfo.P.at(0) = f.K(0);
	cinfo.P.at(2) = f.K(2);
	cinfo.P.at(5) = f.K(4);
	cinfo.P.at(6) = f.K(5);
	cinfo.P.at(10) = 1.0;

	//TODO add distortion coeffs

	this->insight_cinfo_pub.publish(cinfo);

	cv_bridge::CvImage cv_img;

	cv_img.image = img;
	cv_img.header.frame_id = ODOM_FRAME;
	cv_img.header.stamp = f.t;
	cv_img.encoding = sensor_msgs::image_encodings::BGR8;

	this->insight_pub.publish(cv_img.toImageMsg());
	ROS_DEBUG("end publish");
}

void VIO::publishOdometry(Frame& cf)
{
	nav_msgs::Odometry msg;
	static tf::TransformBroadcaster br;


	msg.child_frame_id = CAMERA_FRAME;
	msg.header.frame_id = WORLD_FRAME;

	Eigen::Vector3f temp = this->state_estimator.mu.getOmega();
	msg.twist.twist.angular.x = temp.x();
	msg.twist.twist.angular.y = temp.y();
	msg.twist.twist.angular.z = temp.z();

	Eigen::Quaternionf quat = this->state_estimator.mu.true_pose.unit_quaternion();

	temp = quat.inverse() * this->state_estimator.mu.getVelocity(); // transform the velocity into the body frame

	msg.twist.twist.linear.x = temp.x();
	msg.twist.twist.linear.y = temp.y();
	msg.twist.twist.linear.z = temp.z();

	msg.pose.pose.orientation.w = quat.w();
	msg.pose.pose.orientation.x = quat.x();
	msg.pose.pose.orientation.y = quat.y();
	msg.pose.pose.orientation.z = quat.z();

	temp = this->state_estimator.mu.true_pose.translation();
	msg.pose.pose.position.x = temp.x();
	msg.pose.pose.position.y = temp.y();
	msg.pose.pose.position.z = temp.z();

	//TODO add convariance computation

	this->odom_pub.publish(msg); // publish


	tf::Transform currentPose = tf::Transform(tf::Quaternion(quat.w(), quat.x(), quat.y(), quat.z()), tf::Vector3(temp.x(), temp.y(), temp.z()));

	br.sendTransform(tf::StampedTransform(currentPose, cf.t, WORLD_FRAME, ODOM_FRAME));

}

void VIO::publishPoints(Frame& f)
{

	sensor_msgs::PointCloud msg;

	sensor_msgs::ChannelFloat32 ch;

	ch.name = "intensity";

	msg.header.stamp = f.t;
	msg.header.frame_id = ODOM_FRAME;


	for(auto e : f.features)
	{

		Eigen::Vector3f p_in_f = e.getPoint();

		p_in_f(0) *= p_in_f(2);
		p_in_f(1) *= p_in_f(2);

		geometry_msgs::Point32 pt;

		pt.x = p_in_f.x();
		pt.y = p_in_f.y();
		pt.z = p_in_f.z();

		ch.values.push_back(f.img.at<uchar>(e.getPixel(f)));

		msg.points.push_back(pt);

	}

	msg.channels.push_back(ch);

	this->points_pub.publish(msg);

}
