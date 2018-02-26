/*
 * jacobian_test.cpp
 *
 *  Created on: Jan 1, 2018
 *      Author: kevin
 */


#include <ros/ros.h>

#include "../include/invio/VIO.h"
#include "Params.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jacobian_test"); // initializes ros


	ros::NodeHandle nh;


	ros::param::param<double>("~default_point_depth", DEFAULT_POINT_DEPTH, D_DEFAULT_POINT_DEPTH);



	return 0;
}

