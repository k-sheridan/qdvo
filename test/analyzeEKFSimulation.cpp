#include <ros/ros.h>

#include "Params.h"
#include "../include/invio/VIO.h"



int main(int argc, char **argv)
{
	ros::init(argc, argv, "general_test"); // initializes ros

	ros::NodeHandle nh;


	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);
	ros::param::param<double>("~default_point_depth_variance", DEFAULT_POINT_DEPTH_VARIANCE, D_DEFAULT_POINT_DEPTH_VARIANCE);
	ros::param::param<double>("~default_point_homogenous_variance", DEFAULT_POINT_HOMOGENOUS_VARIANCE, D_DEFAULT_POINT_HOMOGENOUS_VARIANCE);
	//parseROSParams();



	return 0;
}
