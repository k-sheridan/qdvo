#include "../invio/VIO.h"

void VIO::camera_callback(const sensor_msgs::ImageConstPtr& img,
		const sensor_msgs::CameraInfoConstPtr& cam) {
	static int dt_count = 1;
	static double dt_sum = 0;

	ROS_DEBUG_STREAM("got image: " << img->header.stamp);

	ros::Time start = ros::Time::now();

	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	Frame f = Frame(INVERSE_IMAGE_SCALE, temp.clone(), cam->K, cam->D, img->header.stamp);

	ROS_DEBUG_STREAM("start");

	this->addFrame(f);

	double current_dt = (ros::Time::now() - start).toSec() * 1000.0;
	dt_sum += current_dt;

	ROS_INFO_STREAM("average dt: " << dt_sum/dt_count << " this dt: " << current_dt);
	dt_count++;
}



void VIO::applyImageUpdate(Frame& lf, Frame& cf){

	// track old features
	tracker.findNewFeaturePositions(lf, cf);

	// run iterative update
	this->state_estimator.updateWithTrackedFeatures(cf);

	// note the frame's position must be current with the new state estimate for depth update
	//TODO run feature depth/position update

}


/*
 * get more features after updating the pose
 */
void VIO::replenishFeatures(Frame& f) {

	//add more features if needed
	ROS_DEBUG_STREAM("current 2d feature count: " << f.features.size() + f.candidates.size());

	if (f.features.size() + f.candidates.size() < (size_t)NUM_FEATURES) {
		//add the new features to the current state
		f.addFeatures(this->extractNewFeatures(f));
	}

}


std::vector<cv::Point2f> VIO::extractNewFeatures(Frame& f){
	std::vector<cv::Point2f> new_features;

	cv::Mat img;
	if (FAST_BLUR_SIGMA != 0.0) {
		cv::GaussianBlur(f.img, img, cv::Size(5, 5), FAST_BLUR_SIGMA);
	} else {
		img = f.img;
	}

	std::vector<cv::KeyPoint> fast_kp;

	cv::FAST(img, fast_kp, FAST_THRESHOLD, true);

	int needed = NUM_FEATURES - f.features.size();

	ROS_DEBUG_STREAM("need " << needed << "more features");

	/*cv::flann::Index tree;

			 if (this->state.features.size() > 0) {
			 std::vector<cv::Point2f> prev = this->state.getPixels2fInOrder();
			 tree = cv::flann::Index(cv::Mat(prev).reshape(1),
			 cv::flann::KDTreeIndexParams());
			 }*/

	//image which is used to check if a close feature already exists
	cv::Mat checkImg = cv::Mat::zeros(img.size(), CV_8U);
	for (auto& e : f.features) {
		cv::circle(checkImg, e.px, MIN_NEW_FEATURE_DIST, cv::Scalar(255), -1);
	}

	for (int i = 0; i < needed && (size_t)i < fast_kp.size(); i++) {
		/*if (this->state.features.size() > 0) {
				 //make sure that this corner is not too close to any old corners
				 std::vector<float> query;
				 query.push_back(fast_kp.at(i).pt.x);
				 query.push_back(fast_kp.at(i).pt.y);

				 std::vector<int> indexes;
				 std::vector<float> dists;

				 tree.knnSearch(query, indexes, dists, 1);

				 if (dists.front() < MIN_NEW_FEATURE_DIST) // if this featrue is too close to a already tracked feature skip it
				 {
				 continue;
				 }
				 }*/

		//check if there is already a close feature
		if (checkImg.at<unsigned char>(fast_kp.at(i).pt)) {
			//ROS_DEBUG("feature too close to previous feature, not adding");
			needed++; // we need one more now
			continue;
		}

		// remove features at too high of a radius
		if(!f.isPixelInBox(fast_kp.at(i).pt))
		{
			ROS_DEBUG("feature is out of keep box");
			needed++; // we need one more now
			continue;
		}

		// add this new ft to the check img
		cv::circle(checkImg, fast_kp.at(i).pt, MIN_NEW_FEATURE_DIST, cv::Scalar(255), -1);

		//ROS_DEBUG_STREAM("adding feature " << fast_kp.at(i).pt);

		// this is a new feature
		new_features.push_back(fast_kp.at(i).pt);
	}

	return new_features;
}

