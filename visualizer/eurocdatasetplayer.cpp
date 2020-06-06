#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include "Config.h"
#include "DataStructures/TrackingLog.h"
#include "Logging.h"
#include "QDVOVisualizer.h"

/*
 * This executable will run QDVO through a euroc format dataset
 * as a monocular vision only algorithm.
 *
 * By default the dataset will be visualized.
 *
 * Optionally, a log file can be saved to a desired location for later analysis.
 */

DEFINE_string(datasetPath, "~/Desktop/datasets", "dataset path");
DEFINE_int32(frames, 100000, "number of frames to run");
DEFINE_bool(headless, false, "Run the without visualization");
DEFINE_bool(logTrackingData, false, "Log tracking information for later use.");
DEFINE_string(trackingLogPath, "trackingLog.json",
              "The path to the tracking log file output.");

QDVOVisualizer visualizer;

void runDataset() {
  // Create a tracking log.
  std::ofstream os;
  std::unique_ptr<QDVO::TrackingLog> trackingLog;
  if (FLAGS_logTrackingData) {
    os = std::ofstream(FLAGS_trackingLogPath);
    trackingLog = std::make_unique<QDVO::TrackingLog>(
        os, boost::filesystem::absolute(FLAGS_datasetPath).string());
  }

  // start to parse the euroc dataset.
  std::string cam0CsvPath = (FLAGS_datasetPath + "mav0/cam0/data.csv");
  std::ifstream cam0CSV;
  cam0CSV.open(cam0CsvPath, std::ifstream::in);
  if (!cam0CSV.is_open()) {
    LOG_ERROR("failed to open {}", cam0CsvPath);
  }

  int frameCount = 0;

  std::string csvLine;
  std::getline(cam0CSV, csvLine);
  std::getline(cam0CSV, csvLine);

  while (csvLine.size()) {
    // find the time and file of the next image.
    std::stringstream ss(csvLine);
    std::string timeStr, fileStr;
    std::getline(ss, timeStr, ',');
    std::getline(ss, fileStr, ',');
    int64_t t = std::stoull(timeStr);  // nanoseconds

    // run qdvo
    cv::Mat img = cv::imread(FLAGS_datasetPath + "mav0/cam0/data/" + fileStr,
                             cv::IMREAD_GRAYSCALE);

    visualizer.runQDVO(img, t, !FLAGS_headless, trackingLog.get());

    // increment csv
    std::getline(cam0CSV, csvLine);

    ++frameCount;

    if (frameCount >= FLAGS_frames) {
      break;
    }
  }

  // Force the archive destructor to be called.
  trackingLog.reset();
}

void initialize() {
  auto camCalibPath = FLAGS_datasetPath + "mav0/cam0/sensor.yaml";
  auto imuCalibPath = FLAGS_datasetPath + "mav0/imu0/sensor.yaml";

  std::ifstream cam, imu;
  cam.open(camCalibPath, std::ifstream::in);
  imu.open(imuCalibPath, std::ifstream::in);

  if (!cam.is_open()) {
    LOG_ERROR("Could not open {}", camCalibPath);
  }

  if (!imu.is_open()) {
    LOG_ERROR("Could not open {}", imuCalibPath);
  }
  std::stringstream ss;
  ss << cam.rdbuf();
  YAML::Node camCalib = YAML::Load(ss.str());

  ss.str("");
  ss << imu.rdbuf();
  YAML::Node imuCalib = YAML::Load(ss.str());

  auto se3FromYamlNode = [](auto yamlNode) {
    CHECK(yamlNode.size() == 16, "Matrix size wrong.");
    std::vector<QDVO::Scalar> data;
    for (auto coeff : yamlNode) {
      data.push_back(coeff.template as<QDVO::Scalar>());
    }
    Eigen::Matrix<QDVO::Scalar, 4, 4> T(data.data());
    T.transposeInPlace();
    QDVO::SE3 T_body_cam0(T);
    return T_body_cam0;
  };

  QDVO::SE3 T_body_cam0 = se3FromYamlNode(camCalib["T_BS"]["data"]);
  QDVO::SE3 T_body_imu0 = se3FromYamlNode(imuCalib["T_BS"]["data"]);

  auto T_imu_cam0 = T_body_imu0.inverse() * T_body_cam0;

  CHECK(camCalib["distortion_model"].as<std::string>() == "equidistant",
        "Only equidistant camera models are supported.");

  // Default max fov.
  double maxFOV;
  try {
    maxFOV = camCalib["fov"].as<double>();
  } catch (std::exception e) {
    LOG_WARN("fov did not exist in camera calibration. Using default.");
    maxFOV = 1.44 * 2;
  }

  auto cameraModel = std::make_unique<QDVO::EquidistantCameraModel>(
      camCalib["intrinsics"][0].as<double>(),
      camCalib["intrinsics"][1].as<double>(),
      camCalib["intrinsics"][2].as<double>(),
      camCalib["intrinsics"][3].as<double>(), maxFOV,
      camCalib["resolution"][0].as<double>(),
      camCalib["resolution"][1].as<double>(),
      Eigen::Vector4d(camCalib["distortion_coefficients"][0].as<double>(),
                      camCalib["distortion_coefficients"][1].as<double>(),
                      camCalib["distortion_coefficients"][2].as<double>(),
                      camCalib["distortion_coefficients"][3].as<double>()));

  // Initialize.
  visualizer.initialize(std::move(cameraModel), T_imu_cam0);
}

int main(int argc, char** argv) {
  gflags::SetUsageMessage("some usage message");
  gflags::SetVersionString("1.0.0");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  config.setParameters(QDVO::Config::Parameters());

  initialize();

  std::thread qdvoThread(runDataset);

  if (!FLAGS_headless) {
    visualizer.runVisualization();
  }

  qdvoThread.join();

  gflags::ShutDownCommandLineFlags();
  return 0;
}
