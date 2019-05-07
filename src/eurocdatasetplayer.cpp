#include <VO.h>
#include <opencv4/opencv2/core.hpp>
#include <iostream>
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




    gflags::ShutDownCommandLineFlags();
    return 0;
}
