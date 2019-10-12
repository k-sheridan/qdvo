#include "gtest/gtest.h"
#include "CorrespondenceDistribution.h"
#include "GlobalDefinitions.h"
#include "PatchComparer.h"

TEST(CorrespondenceDistribution, Basic)
{
    std::shared_ptr<QDVO::RadialSearchPattern> rsp(new QDVO::RadialSearchPattern(MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS));
    std::shared_ptr<QDVO::PatchComparer> patchComp(new QDVO::PatchComparer());
    QDVO::CorrespondenceDistribution dist(512, 512, rsp, nullptr);


}
