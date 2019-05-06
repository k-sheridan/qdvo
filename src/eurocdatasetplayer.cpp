#include <VO.h>
#include <opencv4/opencv2/core.hpp>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    if(argc < 2){
        std::cerr << "no dataset path\n";
        return 0;
    }
    std::string datasetPath = std::string(argv[1]);

    QDVO::VO vo;

    // start to parse the euroc dataset.

    return 0;
}
