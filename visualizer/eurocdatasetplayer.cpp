#include <gflags/gflags.h>

#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include "QDVOVisualizer.h"

DEFINE_string(datasetPath, "~/Desktop/datasets", "dataset path");
DEFINE_int32(frames, 100000, "number of frames to run");

QDVOVisualizer visualizer;

void runDataset() {
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

    visualizer.runQDVO(img, t / 1e-9);

    // increment csv
    std::getline(cam0CSV, csvLine);

    ++frameCount;

    if (frameCount >= FLAGS_frames) {
      break;
    }
  }
}

int main(int argc, char** argv) {
  gflags::SetUsageMessage("some usage message");
  gflags::SetVersionString("1.0.0");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  visualizer.initialize();

  std::thread qdvoThread(runDataset);

  visualizer.runVisualization();

  qdvoThread.join();

  gflags::ShutDownCommandLineFlags();
  return 0;
}
