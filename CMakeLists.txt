cmake_minimum_required(VERSION 2.8.3)
project (qdvo)


find_package(Ceres REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Sophus REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${SOPHUS_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/CameraModels/)

set(QDVO_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CameraModels/CameraModel.cpp
  ${PROJECT_SOURCE_DIR}/src/CameraModels/FOVCameraModel.cpp
)


# add the library
add_library(qdvo ${QDVO_SOURCE_FILES})
