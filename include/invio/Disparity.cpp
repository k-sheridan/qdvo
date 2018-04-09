#include "../invio/VIO.h"


void VIO::disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg){
	// push this point cloud message onto the buffer
  this->disparity_buffer.push_back(*msg);

  ROS_INFO_STREAM("POINT CLOUD: got pc at: " << msg->header.stamp);

  this->updateDepthsUsingDisparity();
}

/*
* Updates the depths of candidates if applicable and cleans up the the point cloud buffer
*/
void VIO::updateDepthsUsingDisparity(){
  if(this->disparity_buffer.empty() || this->frame_buffer.empty()){
    ROS_DEBUG_STREAM("no frames and/or point clouds to be used for depth update");
    return;
  }

  #define STAMP_EPS 0.001

  //TODO make this function update older features

  // while there are poijnt cloud messages that are older than the current frame, delete them
  while(!this->disparity_buffer.empty() && (this->disparity_buffer.front().header.stamp - this->frame_buffer.front().t).toSec() < -STAMP_EPS){
    this->disparity_buffer.pop_front();
  }

  if(this->disparity_buffer.empty()){
    ROS_DEBUG_STREAM("point cloud message too old to be used during update");
    return;
  }

  // check if the next point cloud is from the same time as the current frame
  if(std::fabs((this->disparity_buffer.front().header.stamp - this->frame_buffer.front().t).toSec()) < STAMP_EPS){
    //update the depths of features and candidates using the point cloud
	ROS_DEBUG("point cloud associated with frame");
	this->applyDisparityUpdate(this->disparity_buffer.front(), this->frame_buffer.front());

    //remove this point cloud after use
    this->disparity_buffer.pop_front();
  }
}

/*
 * uses a point cloud to update the depths of features and candidates
 */
void VIO::applyDisparityUpdate(stereo_msgs::DisparityImage& d, Frame& frame){
	//TODO implement kalman update
	//TODO update candidate features too

	ROS_ASSERT(d.image.width == frame.img.cols && d.image.height == frame.img.rows);

	const cv::Mat_<float> dmat(d.image.height, d.image.width,
	                             (float*)&d.image.data[0], d.image.step);

	for(auto& e : frame.features){
		//Z = fT/d where d is disparity

		float disp = dmat.at<float>((e.px));

		if(disp > d.min_disparity && disp < d.max_disparity){

			ScalarType z = (ScalarType)(d.f * d.T / disp);

			ROS_DEBUG_STREAM("updating with depth: " << z);

			//set the feature pos with a new world coordinate
			Eigen::Matrix<ScalarType, 3, 1> point;
			point << z*e.pixel2Metric(frame.K, e.px), z;

			point = frame.pose * point;

			e.mu = point;

			e.transformFromWorldFrame();


		}
		else{
			ROS_DEBUG_STREAM("invalid disparity");
		}
	}
}
