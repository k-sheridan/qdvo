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

	parseROSParams();

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

	// set up the stereo subs
	if(USE_EXTERNAL_DISPARITY){
		this->disparity_sub = nh.subscribe(EXTERNAL_DISPARITY_TOPIC, 2, &VIO::disparityCallback, this);
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

	/*
	 * this section allows the visual odometry algorithm to initialize with external information
	 * in this case the world to camera transform
	 */

	tf::StampedTransform w2c_st;
	ROS_INFO_STREAM("WAITING FOR TANSFORM FROM " << WORLD_FRAME << " TO " << CAMERA_FRAME);
	if(this->tf_listener.waitForTransform(WORLD_FRAME, CAMERA_FRAME, ros::Time(0), ros::Duration(10))){
		try {
			this->tf_listener.lookupTransform(WORLD_FRAME, CAMERA_FRAME,
					ros::Time(0), w2c_st);
		} catch (tf::TransformException& e) {
			ROS_WARN_STREAM(e.what());
		}
	}
	else
	{
		ROS_FATAL("COULD NOT GET TRANSFORM - USING IDENTITY TRANSFORM");
		w2c_st.setOrigin(tf::Vector3(0,0,0));
		w2c_st.setRotation(tf::Quaternion(0,0,0,1));
		return;
	}
	ROS_INFO("got transform");

	//set the state estimate to the default camera position
	this->state_estimator.mu.true_pose = Sophus::SE3<ScalarType>(Eigen::Quaternion<ScalarType>(w2c_st.getRotation().w(), w2c_st.getRotation().x(), w2c_st.getRotation().y(), w2c_st.getRotation().z()), Eigen::Matrix<ScalarType, 3, 1>(w2c_st.getOrigin().x(), w2c_st.getOrigin().y(), w2c_st.getOrigin().z()));

	// start the callbacks
	ros::spin();
}

void VIO::addFrame(Frame f) {

	//find closest imu update
	this->findClosestIMUUpdate(f.t);

	if (this->frame_buffer.size() == 0) // if this is the first frame that we are receiving
	{
		ROS_DEBUG("adding the first frame");
		f.pose = this->state_estimator.mu.true_pose; // set the initial pose (this is world to camera)

		this->frame_buffer.push_front(f); // add the frame to the front of the buffer

		// set the time if this is the first message
		if(this->state_estimator.t == ros::Time(0)){
			this->state_estimator.t = f.t;
		}
	}

	else // we have atleast 1 frame in the buffer
	{
		// copy over the features and pose from the last frame
		f.features = this->frame_buffer.front().features;
		f.pose = this->frame_buffer.front().pose;

		this->frame_buffer.push_front(f); // add the frame to the front of the buffer

		//set the predicted pose of the current frame
		float dt = (f.t - this->state_estimator.t).toSec();

		ROS_DEBUG_STREAM("processing with dt: " << dt);

		ROS_ASSERT(dt >= 0);
		this->state_estimator.process(dt);
		this->state_estimator.t = f.t;

		//update the frame's position estimate with the predicted
		this->frame_buffer.front().pose = this->state_estimator.mu.true_pose;

		if(this->frame_buffer.front().features.size() > MINIMUM_TRACKABLE_FEATURES) // run update if we have enough features
		{
			// attempt to flow features into the next frame if there are features
			// then perform iterative pose update
			// then apply depth update
			this->applyImageUpdate(this->frame_buffer.at(1), this->frame_buffer.front());
		}
		else{
			ROS_WARN("not enough features to estimate motion visually");
		}

	}

	if(!USE_EXTERNAL_DISPARITY){
		this->replenishFeatures((this->frame_buffer.front()));
	}
	else{
		this->linkFrameAndReplenishFeaturesWithDisparityBuffer(this->frame_buffer.front());
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
	this->publishOdometry();

	//publish the mature 3d points
	this->publishPoints(this->frame_buffer.front());


	//finally remove excess frames from the buffer
	this->removeExcessFrames(this->frame_buffer);
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
		cv::drawMarker(img, e.getPx(), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 22, 1);

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

void VIO::publishOdometry()
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

	temp = this->state_estimator.mu.getVelocity(); // transform the velocity into the body frame

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
	boost::array<double, 36> arr;
	Eigen::Matrix<double, 6, 6> casted = this->state_estimator.Sigma.block<6, 6>(0, 0).cast<double>();
	Eigen::Map<Eigen::Matrix<double, 6, 6>>( arr.data(), casted.rows(), casted.cols() ) =   casted;
	msg.pose.covariance = arr;

	boost::array<double, 36> arr2;
	Eigen::Matrix<double, 6, 6> casted2 = this->state_estimator.Sigma.block<6, 6>(6, 6).cast<double>();
	Eigen::Map<Eigen::Matrix<double, 6, 6>>( arr2.data(), casted2.rows(), casted2.cols() ) =   casted2;
	msg.twist.covariance = arr2;

	this->odom_pub.publish(msg); // publish


	tf::Transform currentPose = tf::Transform(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()), tf::Vector3(temp.x(), temp.y(), temp.z()));

	br.sendTransform(tf::StampedTransform(currentPose, this->state_estimator.t, WORLD_FRAME, ODOM_FRAME));

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

		//ROS_DEBUG_STREAM("feature mu at point pub: " << e.mu.transpose());

		//Eigen::Vector3f p_in_f = (e.projectFeature(f.pose));
		Eigen::Vector3f p_in_f = (e.projectFeature(this->state_estimator.mu.true_pose));

		geometry_msgs::Point32 pt;

		pt.x = p_in_f.x();
		pt.y = p_in_f.y();
		pt.z = p_in_f.z();

		ch.values.push_back(f.img.at<uchar>(e.getPx()));

		msg.points.push_back(pt);

	}

	msg.channels.push_back(ch);

	this->points_pub.publish(msg);

}
