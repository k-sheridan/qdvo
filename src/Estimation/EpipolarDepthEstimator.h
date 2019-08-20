#ifndef EPIPOLARDEPTHESTIMATOR_H
#define EPIPOLARDEPTHESTIMATOR_H


class EpipolarDepthEstimator
{
public:
    EpipolarDepthEstimator();

    bool initialized = false; // flag which means that the landmark depth has been succesfully initialized.
};

#endif // EPIPOLARDEPTHESTIMATOR_H
