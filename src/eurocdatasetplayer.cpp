#include <VO.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <gflags/gflags.h>

DEFINE_string(datasetPath, "~/Desktop/datasets", "dataset path");

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("some usage message");
    gflags::SetVersionString("1.0.0");
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    QDVO::VO vo;

    // start to parse the euroc dataset.
    std::string cam0CsvPath = (FLAGS_datasetPath + "mav0/cam0/data.csv");
    std::ifstream cam0CSV;
    cam0CSV.open (cam0CsvPath, std::ifstream::in);
    if (!cam0CSV.is_open())
    {
        std::cerr << "failed to open " << cam0CsvPath << std::endl;
    }


    std::string csvLine;
    std::getline(cam0CSV, csvLine);
    std::getline(cam0CSV, csvLine);

    while(csvLine.size()) {
        // find the time and file of the next image.
        std::stringstream ss(csvLine);
        std::string timeStr, fileStr;
        std::getline(ss, timeStr, ',');
        std::getline(ss, fileStr, ',');
        uint64_t t = std::stoull(timeStr);

        // run qdvo
        cv::Mat img = cv::imread(FLAGS_datasetPath + "mav0/cam0/data/" + fileStr, cv::IMREAD_GRAYSCALE);

        cv::imshow("doodoo", img);
        cv::waitKey(1);

        // increment csv
        std::getline(cam0CSV, csvLine);
    }



    gflags::ShutDownCommandLineFlags();
    return 0;
}
