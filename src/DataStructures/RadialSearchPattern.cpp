#include "RadialSearchPattern.h"

QDVO::RadialSearchPattern::RadialSearchPattern(const unsigned maxRadius)
{
    unsigned dim = 2*maxRadius;
    if (maxRadius % 2 == 0){dim++;} // ensure that the center of the array is an integer.
    dim += 2;

    // initialize a mask
    std::vector<std::vector<bool> > mask(dim, std::vector<bool>(dim, false));


    int centerIdx = (dim-1)/2;
    Eigen::Vector2i centerPoint(centerIdx, centerIdx);

    // generate the search pattern

    // add the zero radius delta
    std::vector<Eigen::Vector2i> set;
    set.push_back(Eigen::Vector2i(0, 0));
    this->searchPattern.push_back(set);

    const double twoPi = 2 * PI;

    for (int radius = 1; radius <= maxRadius; ++radius)
    {
        std::vector<Eigen::Vector2i> set; // set of points to search.

        // s = dTheta * r = 1 => 1/r = dTheta
        double dTheta = 1.0/radius;

        for (double theta = 0; theta < twoPi; theta += dTheta)
        {
            double dx = std::cos(theta) * radius;
            double dy = std::sin(theta) * radius;

            std::vector<Eigen::Vector2i> options{(Eigen::Vector2i(std::floor(dx), std::floor(dy))),
            (Eigen::Vector2i(std::round(dx), std::floor(dy))),
            (Eigen::Vector2i(std::round(dx), std::round(dy))),
            (Eigen::Vector2i(std::floor(dx), std::round(dy)))};

            for (auto& e : options)
            {
                if (!mask.at(centerIdx + e.y()).at(centerIdx + e.x()))
                {
                    mask.at(centerIdx + e.y()).at(centerIdx + e.x()) = true;
                    set.push_back(e);
                }

            }
        }

        this->searchPattern.push_back(set);
    }

}
