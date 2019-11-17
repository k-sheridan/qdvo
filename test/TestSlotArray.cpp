#include "gtest/gtest.h"
#include "Optimizer/SlotMap.h"
#include "Optimizer/SlotArray.h"
#include <numeric>

using namespace ArgMin;

template <typename T>
struct TypedKey : public SlotMapKeyBase {

};

TEST(SlotArray, Simple) {

    using SA = SlotArray<int, TypedKey<int>>;

    SA map;

    TypedKey<int> key1, key2, key3;
    key1.index = 1;
    key2.index = 5;
    key3.index = 3;    


    EXPECT_EQ(map.insert(key1, 10), SA::SUCCESS_NO_OVERWRITE);
    EXPECT_EQ(map.insert(key3, 30), SA::SUCCESS_NO_OVERWRITE);
    EXPECT_EQ(map.insert(key2, 20), SA::SUCCESS_NO_OVERWRITE);

    EXPECT_TRUE(*(map.at(key1)) == 10);
    EXPECT_TRUE(*(map.at(key2)) == 20);
    EXPECT_TRUE(*(map.at(key3)) == 30);

    EXPECT_TRUE(map.size() == 3);

    map.erase(key2);

    EXPECT_TRUE(*(map.at(key3)) == 30);
    EXPECT_TRUE(*(map.at(key1)) == 10);
    EXPECT_TRUE(map.size() == 2);

    EXPECT_EQ(map.at(key2), map.end());

    TypedKey<int> key4, key5;
    key4.index = 5;
    key5.index = 3;

    EXPECT_EQ(map.insert(key4, 20), SA::SUCCESS_NO_OVERWRITE);

    EXPECT_EQ(map.insert(key5, 50), SA::SUCCESS_OVERWRITE);

    EXPECT_TRUE(map.size() == 3);

    int i = 0;
    for (auto& e : map)
    {
        EXPECT_TRUE(e > 0);
        ++i;
    }
    EXPECT_EQ(i, 3);

    auto generatedKey = map.getKeyFromDataIndex(map.at(key5) - map.begin());

    EXPECT_EQ(generatedKey.index, key5.index);
}

