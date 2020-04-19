#include <gflags/gflags.h>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include "Config.h"
#include "DataStructures/TrackingLog.h"
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
  std::ofstream os(FLAGS_trackingLogPath);
  std::unique_ptr<QDVO::TrackingLog> trackingLog;
  if (FLAGS_logTrackingData) {
    trackingLog = std::make_unique<QDVO::TrackingLog>(os, FLAGS_datasetPath);
  }

  // start to parse the euroc dataset.
  std::string cam0CsvPath = (FLAGS_datasetPath + "mav0/cam0/data.csv");
  std::ifstream cam0CSV;
  cam0CSV.open(cam0CsvPath, std::ifstream::in);
  if (!cam0CSV.is_open()) {
    std::cerr << "failed to open " << cam0CsvPath << std::endl;
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
    uint64_t t = std::stoull(timeStr);  // nanoseconds

    // run qdvo
    cv::Mat img = cv::imread(FLAGS_datasetPath + "mav0/cam0/data/" + fileStr,
                             cv::IMREAD_GRAYSCALE);

    visualizer.runQDVO(img, t / 1e-9, !FLAGS_headless, trackingLog.get());

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

int main(int argc, char** argv) {
  gflags::SetUsageMessage("some usage message");
  gflags::SetVersionString("1.0.0");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  config.setParameters(QDVO::Config::Parameters());

  visualizer.initialize();

  std::thread qdvoThread(runDataset);

  if (!FLAGS_headless) {
    visualizer.runVisualization();
  }

  qdvoThread.join();

  gflags::ShutDownCommandLineFlags();
  return 0;
}
