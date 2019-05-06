cmake_minimum_required(VERSION 2.8.3)
project (qdvo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Sophus REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${SOPHUS_INCLUDE_DIRS})

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/CameraModels/
    ${PROJECT_SOURCE_DIR}/include/ErrorTerms/
    ${PROJECT_SOURCE_DIR}/include/FeatureDetection/
    ${PROJECT_SOURCE_DIR}/include/DataStructures/
    )

set(QDVO_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CameraModels/CameraModel.cpp
  ${PROJECT_SOURCE_DIR}/src/CameraModels/FOVCameraModel.cpp
  ${PROJECT_SOURCE_DIR}/src/ErrorTerms/QuasiDirectErrorTerm_obsFrame.cpp
  ${PROJECT_SOURCE_DIR}/src/FeatureDetection/FeatureDetector.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Landmark.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Feature.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Graph.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/IMUState.cpp
  ${PROJECT_SOURCE_DIR}/src/VO.cpp
)


# add the library
add_library(qdvo ${QDVO_SOURCE_FILES})
target_link_libraries(qdvo ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} ${Ceres_LIBRARIES})
