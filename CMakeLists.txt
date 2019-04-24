cmake_minimum_required(VERSION 2.8.3)
project (qdvo)


find_package(Ceres REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Sophus REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${SOPHUS_INCLUDE_DIRS})
