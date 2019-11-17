#pragma once

namespace QDVO {

class EpipolarDepthEstimator
{
public:
    EpipolarDepthEstimator();

    bool initialized = false; // flag which means that the landmark depth has been succesfully initialized.
};

} // namespace QDVO
