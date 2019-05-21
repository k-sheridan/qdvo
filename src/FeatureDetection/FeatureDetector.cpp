#include "FeatureDetector.h"

QDVO::FeatureDetector::FeatureDetector()
{

}


std::vector<QDVO::Feature> QDVO::FeatureDetector::detectFeatures(const Frame& frame)
{
    // First, compute the image gradients.
    cv::Sobel( frame.image, this->dx, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( frame.image, this->dy, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    // split up the detection into a grid process.
    const int dim = std::floor(sqrt(N_SECTIONS));
    const int maxCandidatesPerSection = ((frame.image.rows * frame.image.cols) / N_SECTIONS) + 1;

    const double rowsPerSection = double(frame.image.rows) / dim;
    const double colsPerSection = double(frame.image.cols) / dim;

    std::vector<int> rowBounds, colBounds;
    for(int i = 0; i < dim; ++i)
    {
        rowBounds.push_back(std::round(i * rowsPerSection));
        colBounds.push_back(std::round(i * colsPerSection));
    }

    rowBounds.push_back(frame.image.rows);
    colBounds.push_back(frame.image.cols);


    // reset/create the feature candidate vectors.
    std::vector<FeatureCandidate> localCandidates;
    localCandidates.reserve(maxCandidatesPerSection);

    const SCALAR_TYPE maxSqGradientMag = std::pow(MINIMUM_NORMALIZED_GRADIENT_MAG * frame.maxIntensity(), 2);


    // look for the pixels which have a sufficiently high gradient magnitude
    for (std::vector<int>::iterator upperRowBoundIt = rowBounds.begin() + 1; upperRowBoundIt != rowBounds.end(); ++upperRowBoundIt)
    {
        for (std::vector<int>::iterator upperColBoundIt = colBounds.begin() + 1; upperColBoundIt != colBounds.end(); ++upperColBoundIt)
        {
            // empty the local candidate vector
            localCandidates.clear();

            const int rl = *(upperRowBoundIt-1);
            const int cl = *(upperColBoundIt-1);
            const int ru = *(upperColBoundIt);
            const int cu = *(upperColBoundIt);


            // find all local feature candidates.
            for (int row = rl; row < ru; ++row)
            {
                const int16_t *rowPtr_dx = this->dx.ptr<short>(row);
                const int16_t *rowPtr_dy = this->dy.ptr<short>(row);

                for (int col = cl; col < cu; ++col)
                {
                    FeatureCandidate fc;
                    fc.dxdx = rowPtr_dx[col]*rowPtr_dx[col];
                    fc.dydy = rowPtr_dy[col]*rowPtr_dy[col];
                    fc.dxdy = rowPtr_dx[col]*rowPtr_dy[col];

                    fc.trace = fc.dxdx + fc.dydy;
                    fc.det = fc.dxdx*fc.dydy + fc.dxdy*fc.dxdy;

                    fc.harris = fc.det - HARRIS_K * fc.trace;

                    if (fc.harris < 0) {fc.score = -EDGE_WEIGHT * fc.harris;}
                    else {fc.score = fc.harris;}

                    fc.gradientNorm = sqrt(fc.trace);

                    fc.x = col;
                    fc.y = row;

                    localCandidates.push_back(fc); // push onto the list

                }
            }

            if (localCandidates.empty()){throw std::runtime_error("grid too fine.");}

            // compute the mean gradient norm
            double sum = 0;
            for (auto e : localCandidates)
            {
                sum += e.gradientNorm;
            }

            SCALAR_TYPE meanGradientNorm = sum / localCandidates.size();

            // filter candidates
            for (auto& e : localCandidates)
            {

            }

        }
    }

    std::vector<QDVO::Feature> features;

    return features;
}
