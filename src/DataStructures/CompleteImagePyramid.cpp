#include "CompleteImagePyramid.h"

QDVO::CompleteImagePyramid::CompleteImagePyramid(const int levels, const int standardDeviationTableResolution) : image(levels)
{

    this->standardDeviationTable = cv::Mat(standardDeviationTableResolution, standardDeviationTableResolution, CV_32F);

}

void QDVO::CompleteImagePyramid::generate(const cv::Mat& baseImage)
{
    this->image.generate(baseImage);

    // compute the image gradients
    /*cv::Sobel(baseImage, this->dx.getImage(), CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    this->dx.generate(this->dx.getImage());

    cv::Sobel(baseImage, this->dy.getImage(), CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
    this->dy.generate(this->dy.getImage());*/

    //// compute the standard deviation table

    // compute the boundaries of the sections.
    const int dim = this->standardDeviationTable.rows;
    const double rowsPerSection = double(baseImage.rows) / dim;
    const double colsPerSection = double(baseImage.cols) / dim;

    std::vector<int> rowBounds, colBounds;
    for(int i = 0; i < dim; ++i)
    {
        rowBounds.push_back(std::max(std::round(i * rowsPerSection), 0.0));
        colBounds.push_back(std::max(std::round(i * colsPerSection), 0.0));
    }
    rowBounds.push_back(baseImage.rows - 2);
    colBounds.push_back(baseImage.cols - 2);

    assert(rowBounds.size() == standardDeviationTable.rows + 1);

    // for each std dev compute it using opencv arithmetic functions.

    for (int i = 0; i < standardDeviationTable.rows; ++i)
    {
        for (int j = 0; j < standardDeviationTable.cols; ++j)
        {
            cv::Mat roi = baseImage(cv::Rect(cv::Point2i(colBounds.at(j), rowBounds.at(i)), cv::Point2i(colBounds.at(j+1), rowBounds.at(i+1))));
        }
    }

}
