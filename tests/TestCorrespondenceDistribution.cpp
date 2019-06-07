#include "gtest/gtest.h"
#include <CorrespondenceDistribution.h>

TEST(CorrespondenceDistribution, Basic)
{
    QDVO::CorrespondenceDistribution dist(512, 512);

    QDVO::CorrespondenceDistribution::PotentialCorrespondence pc;
    pc.pixel = Eigen::Vector2i(400, 200);

    dist.addPotentialCorrespondence(pc);

    //QDVO::GenericQuadTree<QDVO::CorrespondenceDistribution::PotentialCorrespondence*>::Quad* quad = dist.correspondenceMap.root->getQuad(pc.pixel);

    //ASSERT_EQ(quad->data.size(), 1);
}
