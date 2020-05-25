#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "BasicPipeline.h"
#include "DataStructures/TrackingLog.h"
#include "EquidistantCameraModel.h"

#define MAX_VISUALIZATION_DEPTH 10

/**
 * This visualizer is meant to both visualize and test the QDVO algorithm.
 * This specific visualizer is for a monocular use case.
 */
class QDVOVisualizer {
 public:
  QDVOVisualizer();

  QDVO::BasicPipeline algorithm;

  /// flag used to tell the visualization thread
  /// when the algorithm has been updated.
  std::atomic_bool newFrameAdded;

  /// Flag telling the visualizer that the dataset is done.
  std::atomic_bool datasetFinished = false;

  std::mutex algorithmMutex;

  QDVO::CameraModelMap::key_type cameraModelKey;
  QDVO::ExtrinsicMap::key_type extrinsicKey;

  // Visualizer Variables
  std::string window_name;

  struct VisualizationData {
    int imageWidth = 256, imageHeight = 256;
    pangolin::GlTexture currentFrameImage;
    std::vector<pangolin::GlTexture> keyframeImages;
    std::vector<Eigen::Vector3d> activePoints;
    std::vector<Eigen::Vector3d> inactivePoints;
    std::vector<Eigen::Vector3d> marginalizedPoints;
    std::vector<Sophus::SE3d> keyframePoses;
    Sophus::SE3d currentFramePose;

  } visualizationData;

  /*
   * Sets up the algorithm by adding a camera model and
   * preallocating/precomputing.
   */
  void initialize(std::unique_ptr<QDVO::CameraModel> cameraModel,
                  const QDVO::SE3& T_imu_camera);

  void runQDVO(cv::Mat& image, double time, bool notifyVisualizer,
               QDVO::TrackingLog* trackingLog);

  void runVisualization();

 private:
  void transferVisualizationData();

  void draw3DPointCloud();
};

class ColorMap {
 public:
  ColorMap() { this->init(); }

  void init() {
    static const float r[] = {0,
                              0.03968253968253968f,
                              0.07936507936507936f,
                              0.119047619047619f,
                              0.1587301587301587f,
                              0.1984126984126984f,
                              0.2380952380952381f,
                              0.2777777777777778f,
                              0.3174603174603174f,
                              0.3571428571428571f,
                              0.3968253968253968f,
                              0.4365079365079365f,
                              0.4761904761904762f,
                              0.5158730158730158f,
                              0.5555555555555556f,
                              0.5952380952380952f,
                              0.6349206349206349f,
                              0.6746031746031745f,
                              0.7142857142857142f,
                              0.753968253968254f,
                              0.7936507936507936f,
                              0.8333333333333333f,
                              0.873015873015873f,
                              0.9126984126984127f,
                              0.9523809523809523f,
                              0.992063492063492f,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1};
    static const float g[] = {0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0.03174603174603163f,
                              0.0714285714285714f,
                              0.1111111111111112f,
                              0.1507936507936507f,
                              0.1904761904761905f,
                              0.23015873015873f,
                              0.2698412698412698f,
                              0.3095238095238093f,
                              0.3492063492063491f,
                              0.3888888888888888f,
                              0.4285714285714284f,
                              0.4682539682539679f,
                              0.5079365079365079f,
                              0.5476190476190477f,
                              0.5873015873015872f,
                              0.6269841269841268f,
                              0.6666666666666665f,
                              0.7063492063492065f,
                              0.746031746031746f,
                              0.7857142857142856f,
                              0.8253968253968254f,
                              0.8650793650793651f,
                              0.9047619047619047f,
                              0.9444444444444442f,
                              0.984126984126984f,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1,
                              1};
    static const float b[] = {0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              0.04761904761904745f,
                              0.1269841269841265f,
                              0.2063492063492056f,
                              0.2857142857142856f,
                              0.3650793650793656f,
                              0.4444444444444446f,
                              0.5238095238095237f,
                              0.6031746031746028f,
                              0.6825396825396828f,
                              0.7619047619047619f,
                              0.8412698412698409f,
                              0.92063492063492f,
                              1};

    this->rLUT = cv::Mat(64, 1, CV_32FC1, (void*)r).clone();  // red
    this->gLUT = cv::Mat(64, 1, CV_32FC1, (void*)g).clone();  // green
    this->bLUT = cv::Mat(64, 1, CV_32FC1, (void*)b).clone();  // blue

    this->n = 64;
  }

  float lerp(float low, float high, float interp) {
    assert(interp <= 1 && interp >= 0);
    if (high < low) {
      return low;
    }
    return low + interp * (high - low);
  }

  cv::Scalar getColor(float normalizedValue, bool invert = true) {
    if (invert) {
      normalizedValue = std::abs(normalizedValue - 1);
    }
    float fIndex = std::clamp(normalizedValue * this->n, 0.0f, 63.0f);

    int lIdx = std::floor(fIndex);
    int hIdx = std::ceil(fIndex);
    float dIdx = fIndex - lIdx;

    return 255 * cv::Scalar(this->lerp(this->rLUT.at<float>(lIdx),
                                       this->rLUT.at<float>(hIdx), dIdx),
                            this->lerp(this->gLUT.at<float>(lIdx),
                                       this->gLUT.at<float>(hIdx), dIdx),
                            this->lerp(this->bLUT.at<float>(lIdx),
                                       this->bLUT.at<float>(hIdx), dIdx));
  }

  int n;
  cv::Mat rLUT, bLUT, gLUT;
};
