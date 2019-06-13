#include "Frame.h"

QDVO::Frame::Frame()
{

}

void QDVO::Frame::updateImage(cv::Mat& baseImage)
{
    this->imagePyr.generate(baseImage);
    this->imageStatistics.setupTables(baseImage);
}
