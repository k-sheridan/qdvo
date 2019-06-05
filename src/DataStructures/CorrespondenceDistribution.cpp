#include "CorrespondenceDistribution.h"

QDVO::CorrespondenceDistribution::CorrespondenceDistribution(unsigned width, unsigned height)
{
    this->correspondenceMap = QDVO::GenericQuadTree<PotentialCorrespondence*>(width, height);
}

void QDVO::CorrespondenceDistribution::addPotentialCorrespondence(const PotentialCorrespondence& pc)
{
    this->potentialCorrespondences.push_back(pc);
    PotentialCorrespondence* pcPtr = &(this->potentialCorrespondences.back());
    this->correspondenceMap.insert(pc.pixel, pcPtr);
}
