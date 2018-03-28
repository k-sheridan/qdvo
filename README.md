# invio
## INdirect VIsual Inertial Odometry.
Developed by Kevin Sheridan, Purdue University.

## Purpose and Intended Use

InVIO was developed for ROS and uses a few ROS tools like tf. In addition to standard ROS libraries, this algorithm extensively uses OpenCV 3.0/2.0, Sophus and Eigen.




## Performance



Runs using 1 thread.

## Quick ROS installation guide

(developed using ROS Kinetic)

1. >cd ~/catkin_ws/src
2. >git clone https://github.com/pauvsi/invio
3. >sudo apt-get install ros-{your distribution}-sophus
4. >cd ..
5. >catkin_make

this should compile the entire package. If there is an issue please create an issue on this repo!

## Usage

You must provide a rectified mono image with a corresponding camera info
