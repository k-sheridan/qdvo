#include "../invio/VIO.h"


void VIO::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	// push this point cloud message onto the buffer
  this->point_cloud_buffer.push_back(*msg);
}

/*
* Updates the depths of candidates if applicable and cleans up the the point cloud buffer
*/
void VIO::updateDepthsUsingPointCloud(){
  if(this->point_cloud_buffer.empty() || this->frame_buffer.empty()){
    ROS_DEBUG_STREAM("no frames and/or point clouds to be used for depth update");
    return;
  }

  #define STAMP_EPS 0.001

  // while there are poijnt cloud messages that are older than the current frame, delete them
  while(!this->point_cloud_buffer.empty() && (this->point_cloud_buffer.front().header.stamp - this->frame_buffer.front().t).toSec() < -STAMP_EPS){
    this->point_cloud_buffer.pop_front();
  }

  if(this->point_cloud_buffer.empty()){
    ROS_DEBUG_STREAM("point cloud message too old to be used during update");
    return;
  }

  // check if the next point cloud is from the same time as the current frame
  if(std::fabs((this->point_cloud_buffer.front().header.stamp - this->frame_buffer.front().t).toSec()) < STAMP_EPS){
    //TODO update the depths of features and candidates using the point cloud

    //remove this point cloud after use
    this->point_cloud_buffer.pop_front();
  }
}
