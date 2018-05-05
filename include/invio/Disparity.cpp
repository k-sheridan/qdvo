#include "../invio/VIO.h"


void VIO::disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg){
	// push this point cloud message onto the buffer
  this->disparity_buffer.push_back(*msg);

  ROS_INFO_STREAM("POINT CLOUD: got pc at: " << msg->header.stamp);

  this->linkFrameAndReplenishFeaturesWithDisparityBuffer();
}

/*
 * finds a corresponding depth map and links it.
* Updates the depths of candidates if applicable and cleans up the the point cloud buffer
*/
void VIO::linkFrameAndReplenishFeaturesWithDisparityBuffer(){
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

	this->replenishFeatures(this->frame_buffer.front(), this->disparity_buffer.front());

    //remove this point cloud after use
    this->disparity_buffer.pop_front();
  }
}

/*
 * extracts new features and initializes their depth with a disparity image
 */
void VIO::replenishFeatures(Frame& frame, stereo_msgs::DisparityImage& d){
	ROS_ASSERT(d.image.width == frame.img.cols && d.image.height == frame.img.rows);

		//get potential new features
		std::vector<cv::Point2f> new_feature_positions = this->extractNewFeatures(frame);

		ROS_DEBUG_STREAM("extracted " << new_feature_positions.size() << " potential new features.");

		const cv::Mat_<float> dmat(d.image.height, d.image.width,
		                             (float*)&d.image.data[0], d.image.step);

		int needed = NUM_FEATURES - frame.features.size();

		for(auto& e : new_feature_positions){
			//Z = fT/d where d is disparity

			if(needed <= 0){
				ROS_DEBUG_STREAM("have enough features.");
				break;
			}

			float disp = dmat.at<float>((e));

			if(disp > d.min_disparity && disp < d.max_disparity){

				ScalarType z = (ScalarType)(d.f * d.T / disp);

				ROS_DEBUG_STREAM("initializing feature with depth: " << z);

				ScalarType std_dev = z*z/(d.f*d.T) * d.delta_d; // from disparity image documentation


				Feature f = Feature(e, z, std_dev*std_dev, frame.K, frame.pose);

			}
			else{
				ROS_DEBUG_STREAM("invalid disparity");
			}
		}
}
