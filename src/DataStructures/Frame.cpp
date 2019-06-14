#include "Frame.h"

QDVO::Frame::Frame()
{
    this->status = FrameStatus::INACTIVE;
}

void QDVO::Frame::updateImage(cv::Mat& baseImage)
{
    this->imagePyr.generate(baseImage);
}

int QDVO::Frame::maxIntensity() {
    switch (imagePyr.getImage().type())
    {
    case CV_8U: return 255;

    //case CV_16U: return 65535;

    default: throw std::runtime_error("image type not supported.");

    }
}
